# -*- coding: utf-8 -*-
"""
RPLIDAR C1 实时点云显示（PyQt5 版，串口下拉选择）

用法：
  1. 装依赖：  pip install pyserial numpy matplotlib PyQt5
  2. 雷达（USB 转 TTL 或官方转接板）插电脑
  3. 运行：    python lidar_viewer.py
  4. 界面里点「扫描」→ 选雷达的 COM 口 → 点「连接」

功能：
  - 实时扫描可用串口，下拉选择
  - 后台线程读串口、解析 RPLIDAR 协议
  - matplotlib 实时点云，按距离上色（红=近障碍，蓝=远空旷）
"""

import sys
import math
import time
import struct
import threading

import numpy as np
import serial
import serial.tools.list_ports

from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout,
                             QHBoxLayout, QComboBox, QPushButton, QLabel, QStatusBar,
                             QDockWidget, QSpinBox, QTextEdit)
from PyQt5.QtCore import QTimer, Qt

from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
from matplotlib.patches import Rectangle
import matplotlib

# matplotlib 默认字体不含中文，指定系统字体，否则中文显示成方块
matplotlib.rcParams['axes.unicode_minus'] = False   # 负号正常显示

# 中文乱码修复：运行时从系统已装字体里挑一个能用的（不写死字体名）
from matplotlib import font_manager as _fm

def _setup_cjk_font():
    installed = {f.name for f in _fm.fontManager.ttflist}
    for name in ["Microsoft YaHei", "SimHei", "SimSun", "Noto Sans CJK SC",
                 "Noto Sans SC", "Source Han Sans SC", "WenQuanYi Zen Hei",
                 "WenQuanYi Micro Hei", "PingFang SC", "Arial Unicode MS"]:
        if name in installed:
            matplotlib.rcParams["font.sans-serif"] = [name]
            break

_setup_cjk_font()

# ================== 配置 ==================
BAUD = 460800
RANGE_MM = 12000        # 显示范围（±12m）
OBSTACLE_DIST = 4000        # 疑似障碍物检测距离 (mm)
OBSTACLE_FRONT_HALF = 70    # 前方半角 ±70°（共 140°；0°=正前方，顺时针）
SMOOTH_FRAMES = 3           # 多圈平均圈数（平滑点云飘动）
SMOOTH_MIN_COUNT = 2        # 一个角度 bin 至少出现在几圈才保留（滤随机噪声）
MIN_OBSTACLE_POINTS = 2     # 一个障碍至少几个点，否则算噪声（φ50圆柱远处只有2-3点）
OBSTACLE_ANGLE_GAP = 3.0    # 相邻点角度差 >此值(°) → 不同障碍
# ---- 障碍判别（φ50 圆柱专用，替代纯点数置信度）----
CLUSTER_MAX_EXTENT = 150.0  # 簇最大外接尺寸 (mm)：圆柱≈50mm，墙碎片几百mm → 直接扔
EXPECTED_PTS_1M = 3.0       # φ50 圆柱在 1m 处的期望点数（实测校准：1m 处约 2-3 点）
PERSIST_FRAMES = 5          # 位置持续性窗口：最近 N 帧里同位置反复出现 → 稳定障碍
MATCH_RADIUS_MM = 120.0     # 跨帧配同一障碍的位置容差 (mm)
MIN_CONFIDENCE = 60         # 置信度低于此值不显示（但照样计入持续性窗口攒分）
DEFAULT_RPM = 600           # 电机转速：600RPM=10Hz 标准；300RPM≈5Hz 每圈点数×2
# ==========================================


class LidarParser:
    """RPLIDAR SCAN 协议解析器：喂字节，产出完整一圈的点 [(angle_deg, dist_mm)]"""

    def __init__(self):
        self.reset()

    def reset(self):
        self.state = 0          # 0=找A5 1=找5A 2=跳应答头 3=节点
        self.hdr = 0
        self._is_scan = False   # 应答类型：只有 0x81(扫描) 才进节点解析
        self.node = bytearray()
        self.rev = []           # 当前一圈的点
        self.completed = None   # 完整一圈（等主线程取走）

    def feed(self, data):
        """喂入字节流，内部解析"""
        for b in data:
            if self.state == 0:
                if b == 0xA5:
                    self.state = 1
            elif self.state == 1:
                self.state = 2 if b == 0x5A else 0
                self.hdr = 0
            elif self.state == 2:
                if self.hdr == 4:
                    self._is_scan = (b == 0x81)  # 第5字节=类型：0x81 才是扫描应答
                self.hdr += 1
                if self.hdr >= 5:            # 跳过 len(4)+type(1)
                    # 调速等控制命令的应答直接丢弃，否则会带偏节点解析 → 点云全无
                    self.state = 3 if self._is_scan else 0
                    self.node = bytearray()
            elif self.state == 3:
                self.node.append(b)
                if len(self.node) >= 5:
                    self._parse_node()
                    self.node = bytearray()

    def _parse_node(self):
        n = self.node
        # n[0]: bit0=S(新一圈首点), bit1=~S, bit2-7=quality
        s = n[0] & 0x01
        s_inv = (n[0] >> 1) & 0x01
        if s == s_inv:                       # 校验失败，帧错位，丢一字节重对齐
            self.node = self.node[1:]
            return
        ang = ((n[2] << 8) | n[1]) >> 1      # 去 check 位，15bit
        dist = (n[4] << 8) | n[3]
        pt = (ang / 64.0, dist / 4.0)        # 角度°，距离 mm

        if s == 1 and self.rev:              # 新一圈开始 → 上一圈完成
            self.completed = self.rev
            self.rev = []
        if dist > 0:
            self.rev.append(pt)

    def take_completed(self):
        r = self.completed
        self.completed = None
        return r


class SerialReader:
    """后台串口线程：读串口 → 解析 → 保存最新一圈点"""

    def __init__(self, port, baud, parser, rpm=DEFAULT_RPM):
        self.port = port
        self.baud = baud
        self.parser = parser
        self.rpm = rpm
        self.ser = None
        self.thr = None
        self.running = False
        self.latest = None
        self.lost = False          # 非 stop() 导致的意外退出（如拔线）
        self.lost_reason = ""
        self._lock = threading.Lock()

    def start(self):
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.2)
        except Exception:
            return False
        # 启动电机（USB 转接板用 DTR 控制 MOTOCTL，DTR=False → 转）
        try:
            self.ser.dtr = False
        except Exception:
            pass
        # 设置电机转速（协议 MOTOR_SPEED_CTRL：A5 A8 02 + RPM 小端16位）
        # C1 闭环电机，掉速自动补偿；转速越低每圈点数越多（5000点/s 固定）
        # rpm=0 → 不发命令（行为与旧版完全一致，兜底用）
        if self.rpm:
            try:
                self.ser.write(b"\xA5\xA8\x02" + struct.pack("<H", int(self.rpm)))
                # 等应答描述符(7字节)真正到达再清缓冲，清早了残留字节会破坏解析
                deadline = time.time() + 0.5
                while time.time() < deadline and self.ser.in_waiting < 7:
                    time.sleep(0.01)
                self.ser.reset_input_buffer()
            except Exception:
                pass
        # 发 SCAN 命令（A5 20，无校验）
        try:
            self.ser.write(b"\xA5\x20")
        except Exception:
            pass
        self.parser.reset()
        self.running = True
        self.thr = threading.Thread(target=self._loop, daemon=True)
        self.thr.start()
        return True

    def set_rpm(self, rpm):
        """在线调速（已连接时调用）。应答描述符可能被读线程收走，解析器可自行重对齐"""
        self.rpm = rpm
        if self.ser and self.ser.is_open:
            try:
                self.ser.write(b"\xA5\xA8\x02" + struct.pack("<H", int(rpm)))
                return True
            except Exception:
                return False
        return False

    def stop(self):
        self.running = False
        if self.thr and self.thr.is_alive():
            self.thr.join(timeout=2)
        if self.ser:
            try:
                if self.ser.is_open:
                    # 1) 发 STOP 命令（A5 25）停扫描数据流
                    self.ser.write(b"\xA5\x25")
                    self.ser.flush()
                    # 2) DTR 拉高关电机（与 start 时 dtr=False 开电机对应），
                    #    否则断开后雷达一直空转
                    self.ser.dtr = True
                    self.ser.close()
            except Exception:
                pass
        self.ser = None
        self.thr = None

    def _loop(self):
        while self.running:
            try:
                if not self.ser or not self.ser.is_open:
                    self.lost_reason = "串口句柄失效"
                    break
                d = self.ser.read(256)
                if d:
                    self.parser.feed(d)
                    pts = self.parser.take_completed()
                    if pts is not None:
                        with self._lock:
                            self.latest = pts
            except Exception as e:
                self.lost_reason = str(e) or "串口读异常"
                break
        # running 仍为 True 说明不是用户主动断开 → 判定为意外掉线
        if self.running:
            self.lost = True
        self.running = False

    def get_latest(self):
        with self._lock:
            return self.latest


def kmeans_cluster(points, k, iters=30):
    """K-means：把 2D 点 (x,y) 分成 k 簇，返回 [(cx, cy, n_points), ...]"""
    pts = np.array(points)
    n = len(pts)
    if k <= 0 or n == 0:
        return []
    if k >= n:
        return [(float(x), float(y), 1) for x, y in pts]
    idx = np.linspace(0, n - 1, k).astype(int)
    centers = pts[idx].copy()
    labels = np.zeros(n, dtype=int)
    for _ in range(iters):
        d = np.linalg.norm(pts[:, None, :] - centers[None, :], axis=2)
        labels = np.argmin(d, axis=1)
        new = np.array([pts[labels == j].mean(axis=0) if (labels == j).any() else centers[j]
                        for j in range(k)])
        if np.allclose(new, centers):
            break
        centers = new
    return [(float(centers[j][0]), float(centers[j][1]), int((labels == j).sum()))
            for j in range(k)]


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("RPLIDAR C1 点云显示")
        self.resize(900, 900)
        self.reader = None
        self.parser = LidarParser()
        self.recent = []       # 最近几圈点云（平滑用）
        self._obs_hist = []    # 最近 PERSIST_FRAMES 帧的障碍位置（持续性判定用）
        self._last_rev = None  # 上一处理的圈（同一圈不重复计入持续性窗口）
        self._last_obstacles = []  # 最近一次检测结果（离线测试用）
        self._build()

        # 定时刷新点云（50ms）
        self._dt = QTimer(self)
        self._dt.timeout.connect(self._update_plot)
        self._dt.start(50)

    def _build(self):
        c = QWidget()
        self.setCentralWidget(c)
        root = QVBoxLayout(c)
        root.setContentsMargins(10, 8, 10, 8)
        root.setSpacing(6)

        # ---- 工具栏 ----
        bar = QHBoxLayout()
        bar.addWidget(QLabel("串口:"))
        self.port_cb = QComboBox()
        self.port_cb.setMinimumWidth(120)
        self._refresh_ports()
        bar.addWidget(self.port_cb)

        sc = QPushButton("🔄 扫描")
        sc.clicked.connect(self._refresh_ports)
        bar.addWidget(sc)

        bar.addSpacing(12)
        self.cn = QPushButton("🔌 连接")
        self.cn.clicked.connect(self._toggle_connect)
        bar.addWidget(self.cn)

        bar.addSpacing(6)
        bar.addWidget(QLabel("转速RPM:"))
        self.rpm_spin = QSpinBox()
        self.rpm_spin.setRange(0, 900)
        self.rpm_spin.setSingleStep(50)
        self.rpm_spin.setValue(DEFAULT_RPM)
        self.rpm_spin.setSpecialValueText("不调速")
        self.rpm_spin.setToolTip("600=10Hz标准；300≈5Hz，每圈点数×2（静止采集推荐）。\n"
                                 "0=不发调速命令（旧行为，调速异常时选这个）。\n"
                                 "连接时自动下发；已连接可点「应用转速」在线调速")
        self.rpm_spin.setMinimumWidth(55)
        bar.addWidget(self.rpm_spin)
        self.rpm_btn = QPushButton("应用转速")
        self.rpm_btn.clicked.connect(self._apply_rpm)
        bar.addWidget(self.rpm_btn)

        self.st = QLabel("● 未连接")
        self.st.setStyleSheet("color:#c0392b;font-weight:bold;padding:2px 8px")
        bar.addWidget(self.st)

        self.cnt = QLabel("点: 0")
        bar.addWidget(self.cnt)
        bar.addSpacing(12)
        self.meas = QLabel("测量: 点击图选点")
        self.meas.setStyleSheet("color:#16a085;font-weight:bold;padding:2px 8px")
        bar.addWidget(self.meas)
        bar.addSpacing(12)
        self.roi_btn = QPushButton("框选区域")
        self.roi_btn.setCheckable(True)
        self.roi_btn.setToolTip("按下后在图上左键拖拽画矩形，只识别矩形内的障碍")
        bar.addWidget(self.roi_btn)
        self.roi_clear = QPushButton("清除框选")
        self.roi_clear.clicked.connect(self._clear_roi)
        bar.addWidget(self.roi_clear)
        bar.addStretch()
        root.addLayout(bar)

        # ---- 点云图（matplotlib 嵌入 Qt）----
        self.figure = Figure(figsize=(8, 8), dpi=100)
        self.canvas = FigureCanvas(self.figure)
        self.ax = self.figure.add_subplot(111)
        self.ax.set_aspect("equal")
        self.ax.set_xlim(-RANGE_MM, RANGE_MM)
        self.ax.set_ylim(-RANGE_MM, RANGE_MM)
        self.ax.set_title("RPLIDAR C1 点云图")
        self.ax.grid(True, alpha=0.3)
        self.sc = self.ax.scatter([], [], s=2, cmap="turbo")
        self.sc.set_clim(0, RANGE_MM)
        self.cbar = self.figure.colorbar(self.sc, ax=self.ax, label="距离 (mm)")

        # ---- 点击测量：从雷达原点连线到点击点 ----
        self.ax.plot([0], [0], 'r+', ms=11, mew=2)                  # 雷达原点
        self.ax.text(0, RANGE_MM * 0.92, "↑ 前方 0°", ha='center', va='center',
                     color='gray', fontsize=9)                       # 方向提示

        # ---- 障碍检测区（前方 180°，半径 OBSTACLE_DIST 半圆）----
        zang = list(range(360 - OBSTACLE_FRONT_HALF, 361)) + list(range(0, OBSTACLE_FRONT_HALF + 1))
        zx = [OBSTACLE_DIST * np.sin(np.radians(a)) for a in zang]
        zy = [OBSTACLE_DIST * np.cos(np.radians(a)) for a in zang]
        self.ax.plot(zx, zy, 'r-', lw=1.5, alpha=0.5)     # 半圆弧
        for a in (360 - OBSTACLE_FRONT_HALF, OBSTACLE_FRONT_HALF):   # 两条半径边
            self.ax.plot([0, OBSTACLE_DIST * np.sin(np.radians(a))],
                         [0, OBSTACLE_DIST * np.cos(np.radians(a))],
                         'r-', lw=1.5, alpha=0.5)
        self.obs_sc = self.ax.scatter([], [], s=30, c='red', marker='x', lw=1.5)
        self.cluster_sc = self.ax.scatter([], [], s=120, c='orange', marker='o',
                                          edgecolors='black', lw=1.5)
        self.cluster_texts = []
        self.click_line, = self.ax.plot([], [], 'g--', lw=1.3)      # 连线
        self.click_marker, = self.ax.plot([], [], 'go', ms=8, mfc='none')  # 点击点
        self.click_text = self.ax.text(0, 0, '', color='green',
                                       fontsize=9, ha='left', va='bottom')
        self._panning = False
        self._drawing_roi = False
        self.roi = None                       # [xmin, xmax, ymin, ymax] 或 None（无框选）
        self.roi_rect = Rectangle((0, 0), 0, 0, fill=False, edgecolor='blue', lw=2, linestyle='--')
        self.roi_rect.set_visible(False)
        self.ax.add_patch(self.roi_rect)
        self.canvas.mpl_connect('button_press_event', self._on_press)
        self.canvas.mpl_connect('motion_notify_event', self._on_motion)
        self.canvas.mpl_connect('button_release_event', self._on_release)
        self.canvas.mpl_connect('scroll_event', self._on_scroll)

        root.addWidget(self.canvas, stretch=1)

        self.setStatusBar(QStatusBar())
        self.statusBar().showMessage("就绪 · 插雷达后点「扫描」→ 选 COM 口 → 连接")

        # ---- 疑似障碍物独立显示窗口（右侧停靠，可拖动/悬浮）----
        self.dock = QDockWidget("疑似障碍物", self)
        self.dock.setFeatures(QDockWidget.DockWidgetMovable | QDockWidget.DockWidgetFloatable)
        dw = QWidget()
        dl = QVBoxLayout(dw)
        dl.setContentsMargins(8, 8, 8, 8)
        mb = QHBoxLayout()
        mb.addWidget(QLabel("模式:"))
        self.mode_cb = QComboBox()
        self.mode_cb.addItems(["自动搜索", "固定个数"])
        mb.addWidget(self.mode_cb)
        mb.addSpacing(6)
        self.k_spin = QSpinBox()
        self.k_spin.setRange(1, 10)
        self.k_spin.setValue(2)
        self.k_spin.setMinimumWidth(45)
        self.k_spin.setToolTip("固定个数模式下的障碍物数量")
        mb.addWidget(self.k_spin)
        mb.addStretch()
        dl.addLayout(mb)
        self.obs_view = QTextEdit()
        self.obs_view.setReadOnly(True)
        self.obs_view.setHtml('<div style="color:#94A3B8;padding:10px">✓ 前方安全</div>')
        dl.addWidget(self.obs_view)
        self.dock.setWidget(dw)
        self.addDockWidget(Qt.RightDockWidgetArea, self.dock)

    # ========== 串口管理 ==========
    def _scan_ports(self):
        """扫描可用串口（能打开才算可用）"""
        raw = [p.device for p in serial.tools.list_ports.comports()]
        if sys.platform == "win32":
            try:
                import winreg
                with winreg.OpenKey(winreg.HKEY_LOCAL_MACHINE,
                                    r"HARDWARE\DEVICEMAP\SERIALCOMM") as k:
                    i = 0
                    while True:
                        try:
                            _, v, _ = winreg.EnumValue(k, i)
                            raw.append(v)
                            i += 1
                        except OSError:
                            break
            except Exception:
                pass
        raw = sorted(set(raw), key=lambda s: (not s.startswith("COM"), s))
        ok = []
        for p in raw:
            try:
                t = serial.Serial(p, BAUD, timeout=0.05)
                ok.append(p)
                t.close()
            except Exception:
                pass
        return ok

    def _refresh_ports(self):
        cur = self.port_cb.currentText()
        avail = self._scan_ports()
        self.port_cb.blockSignals(True)
        self.port_cb.clear()
        self.port_cb.addItems(avail if avail else ["(无)"])
        if cur in avail:
            self.port_cb.setCurrentText(cur)
        self.port_cb.blockSignals(False)
        if not avail:
            self.statusBar().showMessage("没扫到串口，检查雷达是否插入", 3000)

    def _toggle_connect(self):
        if self.reader and self.reader.running:
            self.reader.stop()
            self.reader = None
            self._set_ui(False)
            return
        port = self.port_cb.currentText()
        if not port or port == "(无)":
            return
        self.reader = SerialReader(port, BAUD, self.parser, rpm=self.rpm_spin.value())
        if self.reader.start():
            self._set_ui(True, port)
        else:
            self.reader = None
            self.statusBar().showMessage(f"⚠ 打开 {port} 失败（可能被占用）", 4000)

    def _apply_rpm(self):
        if self.reader and self.reader.running:
            ok = self.reader.set_rpm(self.rpm_spin.value())
            msg = f"转速已设为 {self.rpm_spin.value()} RPM" if ok else "调速失败"
            self.statusBar().showMessage(msg, 3000)

    def _set_ui(self, on, port=""):
        if on:
            self.st.setText(f"● 已连接 {port}")
            self.st.setStyleSheet("color:#27ae60;font-weight:bold;padding:2px 8px")
            self.cn.setText("🔌 断开")
        else:
            self.st.setText("● 未连接")
            self.st.setStyleSheet("color:#c0392b;font-weight:bold;padding:2px 8px")
            self.cn.setText("🔌 连接")

    # ========== 点云平滑 + 角度聚类 ==========
    def _smooth(self, raw):
        """多圈平均 + 一致性过滤：点需在最近几圈里多次出现才保留，滤掉随机噪声"""
        cur = {}
        for a, d in raw:
            b = int(round(a)) % 360
            cur[b] = (cur[b] + d) / 2 if b in cur else d
        self.recent.append(cur)
        if len(self.recent) > SMOOTH_FRAMES:
            self.recent.pop(0)
        merged = {}
        for frame in self.recent:
            for b, d in frame.items():
                merged.setdefault(b, []).append(d)
        # 只保留在 >= SMOOTH_MIN_COUNT 圈里都出现的 bin（真障碍每圈都在，噪声只偶尔出现）
        return [(float(b), sum(v) / len(v))
                for b, v in merged.items() if len(v) >= SMOOTH_MIN_COUNT]

    def _segment_obstacles(self, zone_pts):
        """轻量分段：只按角度间隔分段（纯比较，无 sqrt/cos，适合单片机）。
        近处障碍角度连续、不会被距离差误切；有空洞（缺角度）则断开。"""
        if not zone_pts:
            return []
        # 展开角度处理 0° 跨越
        unwrapped = sorted((a + 360 if a < 180 else a, d) for a, d in zone_pts)
        segments, cur = [], [unwrapped[0]]
        for i in range(1, len(unwrapped)):
            if (unwrapped[i][0] - unwrapped[i - 1][0]) > OBSTACLE_ANGLE_GAP:
                segments.append(cur)
                cur = [unwrapped[i]]
            else:
                cur.append(unwrapped[i])
        segments.append(cur)
        return [[(a % 360, d) for a, d in seg] for seg in segments]

    # ========== 点云刷新 ==========
    def _update_plot(self):
        if not self.reader:
            return
        # 意外掉线检测（拔线等）：由 50ms 定时器在主线程安全处理
        if self.reader.lost:
            reason = self.reader.lost_reason
            self.reader = None
            self._set_ui(False)
            self.statusBar().showMessage(f"⚠ 串口掉线: {reason}", 6000)
            return
        raw = self.reader.get_latest()
        if raw:
            if raw is self._last_rev:
                # 同一圈数据：50ms tick 只重画，不重复跑检测/持续性窗口
                self.canvas.draw_idle()
                return
            self._last_rev = raw
            # 多圈平均平滑
            pts = self._smooth(raw)

            # 机器人坐标系：0°=正前方(上)，顺时针增大
            xs = [d * np.sin(np.radians(a)) for a, d in pts]
            ys = [d * np.cos(np.radians(a)) for a, d in pts]
            ds = [d for a, d in pts]
            self.sc.set_offsets(np.c_[xs, ys])
            self.sc.set_array(np.array(ds))
            self.cnt.setText(f"点: {len(pts)}")

            # 障碍检测：前方 ±70°、距离 ≤ OBSTACLE_DIST、且在框选区域内
            zone = [(a, d) for a, d in pts
                    if d <= OBSTACLE_DIST
                    and (a >= 360 - OBSTACLE_FRONT_HALF or a <= OBSTACLE_FRONT_HALF)
                    and self._in_roi(d * np.sin(np.radians(a)), d * np.cos(np.radians(a)))]

            # 两种模式：自动搜索（ABD）/ 固定个数（K-means）
            obstacles = []      # [(angle, dist, conf, n_points)]
            obs_points = []     # 用于红叉显示的点
            if self.mode_cb.currentText() == "自动搜索":
                segments = self._segment_obstacles(zone)
                cands = []          # [(a, d, geom_conf, n, xy_points)]
                for s in segments:
                    if len(s) < MIN_OBSTACLE_POINTS:
                        continue
                    xy = [(d * math.sin(math.radians(a)),
                           d * math.cos(math.radians(a))) for a, d in s]
                    # 簇外接尺寸：圆柱≈50mm；墙/设施碎片几百mm → 直接丢弃
                    ext = max(math.dist(p, q) for p in xy for q in xy)
                    if ext > CLUSTER_MAX_EXTENT:
                        continue
                    # 用最近点（距离最小）作为障碍位置，避免平均值落空白
                    ca, cd = min(s, key=lambda p: p[1])
                    # 几何置信 = 点数是否达到该距离下的期望值 + 簇是否紧凑
                    exp_pts = max(2.0, EXPECTED_PTS_1M * 1000.0 / max(cd, 1.0))
                    s_pts = min(1.0, len(s) / exp_pts)
                    s_ext = min(1.0, 60.0 / max(ext, 1.0))   # 越接近φ50越像圆柱
                    cands.append((ca, cd, 0.5 * s_pts + 0.5 * s_ext, len(s), xy))
                # 持续性：同一位置连续多帧出现才算稳（闪烁毛刺被压掉）
                hits = []
                for ca, cd, geom, n, xy in cands:
                    p = (cd * math.sin(math.radians(ca)),
                         cd * math.cos(math.radians(ca)))
                    k = sum(1 for frame in self._obs_hist
                            if any(math.dist(p, q) < MATCH_RADIUS_MM for q in frame))
                    denom = max(len(self._obs_hist), 1)
                    persist = k / PERSIST_FRAMES if len(self._obs_hist) >= PERSIST_FRAMES else k / denom
                    conf = int(round(100 * (0.4 * geom + 0.6 * persist)))
                    hits.append(p)          # 先攒持续性，低分帧也算出现
                    obs_points.extend(xy)
                    if conf >= MIN_CONFIDENCE:
                        obstacles.append((ca, cd, conf, n))
                self._obs_hist.append(hits)
                if len(self._obs_hist) > PERSIST_FRAMES:
                    self._obs_hist.pop(0)
            else:
                k = self.k_spin.value()
                if k > 0 and zone:
                    xy = [(d * np.sin(np.radians(a)), d * np.cos(np.radians(a))) for a, d in zone]
                    for cx, cy, n in kmeans_cluster(xy, k):
                        ang = math.degrees(math.atan2(cx, cy)) % 360
                        dist = math.hypot(cx, cy)
                        # 与自动模式同源的点数置信：n 达到该距离期望值即 100%
                        exp_pts = max(2.0, EXPECTED_PTS_1M * 1000.0 / max(dist, 1.0))
                        conf = int(round(100 * min(1.0, n / exp_pts)))
                        obstacles.append((ang, dist, conf, n))
                    obs_points = zone
            self._last_obstacles = obstacles

            # 红叉：障碍点
            if obs_points:
                oxs = [d * np.sin(np.radians(a)) for a, d in obs_points]
                oys = [d * np.cos(np.radians(a)) for a, d in obs_points]
                self.obs_sc.set_offsets(np.c_[oxs, oys])
            else:
                self.obs_sc.set_offsets(np.empty((0, 2)))

            # 橙色圆圈 + 编号：障碍中心
            for t in self.cluster_texts:
                t.remove()
            self.cluster_texts = []
            if obstacles:
                cxs = [d * np.sin(np.radians(a)) for a, d, _, _ in obstacles]
                cys = [d * np.cos(np.radians(a)) for a, d, _, _ in obstacles]
                self.cluster_sc.set_offsets(np.c_[cxs, cys])
                for i, (a, d, conf, n) in enumerate(obstacles):
                    t = self.ax.text(d * np.sin(np.radians(a)), d * np.cos(np.radians(a)),
                                     str(i + 1), color='black', fontsize=10,
                                     ha='center', va='center', fontweight='bold')
                    self.cluster_texts.append(t)
            else:
                self.cluster_sc.set_offsets(np.empty((0, 2)))

            # 更新「疑似障碍物」窗口（HTML 分块卡片）
            if obstacles:
                html = ""
                for i, (a, d, conf, n) in enumerate(obstacles):
                    html += (f'<div style="background:#FFFFFF;border:1px solid #E2E8F0;'
                             f'border-radius:10px;padding:8px 12px;margin-bottom:6px;">'
                             f'<span style="font-weight:bold;color:#3B82F6;">障碍 {i+1}</span><br>'
                             f'<span style="font-size:15pt;font-weight:bold;color:#1E293B;">'
                             f'{d:.0f} mm @ {a:.0f}°</span><br>'
                             f'<span style="color:#64748B;font-size:9pt;">置信 {conf}% · {n} 点</span>'
                             f'</div>')
                self.obs_view.setHtml(html)
            else:
                self.obs_view.setHtml('<div style="color:#94A3B8;padding:10px">✓ 前方安全</div>')

            self.canvas.draw_idle()

    def _measure(self, x, y):
        """测距：从雷达原点连线到 (x,y)，显示角度/距离/方位"""
        dist = math.hypot(x, y)
        ang = math.degrees(math.atan2(x, y)) % 360

        # 方位（象限，0°=正前方，顺时针 90°=右侧）
        if 315 <= ang or ang < 45:
            d = "前方"
        elif 45 <= ang < 135:
            d = "右侧"
        elif 135 <= ang < 225:
            d = "后方"
        else:
            d = "左侧"

        self.click_line.set_data([0, x], [0, y])
        self.click_marker.set_data([x], [y])
        self.click_text.set_position((x, y))
        self.click_text.set_text(f"{ang:.1f}° {dist:.0f}mm")
        self.meas.setText(f"角度 {ang:.1f}° · 距离 {dist:.0f} mm · 方位 {d}")
        self.canvas.draw_idle()

    def _in_roi(self, x, y):
        """判断点 (x,y) 是否在框选区域内（无框选则全部通过）"""
        if self.roi is None:
            return True
        xmin, xmax, ymin, ymax = self.roi
        return xmin <= x <= xmax and ymin <= y <= ymax

    def _clear_roi(self):
        self.roi = None
        self.roi_rect.set_visible(False)
        self.canvas.draw_idle()

    def _on_press(self, event):
        """左键=测距（或框选区域），右键=平移"""
        if event.inaxes != self.ax:
            return
        if self.roi_btn.isChecked():
            # 框选模式：左键拖拽画矩形
            if event.button == 1 and event.xdata is not None and event.ydata is not None:
                self._drawing_roi = True
                self._roi_start = (event.xdata, event.ydata)
        else:
            if event.button == 1:
                if event.xdata is not None and event.ydata is not None:
                    self._measure(event.xdata, event.ydata)
            elif event.button == 3:
                self._panning = True
                self._pan_x = event.xdata
                self._pan_y = event.ydata

    def _on_motion(self, event):
        """框选拖拽 / 右键平移"""
        if getattr(self, '_drawing_roi', False):
            if event.inaxes == self.ax and event.xdata is not None and event.ydata is not None:
                x0, y0 = self._roi_start
                x1, y1 = event.xdata, event.ydata
                self.roi_rect.set_xy((min(x0, x1), min(y0, y1)))
                self.roi_rect.set_width(abs(x1 - x0))
                self.roi_rect.set_height(abs(y1 - y0))
                self.roi_rect.set_visible(True)
                self.canvas.draw_idle()
            return
        if not getattr(self, '_panning', False):
            return
        if event.inaxes != self.ax or event.xdata is None or event.ydata is None:
            return
        dx = event.xdata - self._pan_x
        dy = event.ydata - self._pan_y
        self._pan_x = event.xdata
        self._pan_y = event.ydata
        xlim = self.ax.get_xlim()
        ylim = self.ax.get_ylim()
        self.ax.set_xlim([xlim[0] - dx, xlim[1] - dx])
        self.ax.set_ylim([ylim[0] - dy, ylim[1] - dy])
        self.canvas.draw_idle()

    def _on_release(self, event):
        if getattr(self, '_drawing_roi', False):
            self._drawing_roi = False
            if event.inaxes == self.ax and event.xdata is not None and event.ydata is not None:
                x0, y0 = self._roi_start
                x1, y1 = event.xdata, event.ydata
                self.roi = (min(x0, x1), max(x0, x1), min(y0, y1), max(y0, y1))
        self._panning = False

    def _on_scroll(self, event):
        """滚轮缩放（以鼠标位置为中心）"""
        if event.inaxes != self.ax:
            return
        x, y = event.xdata, event.ydata
        if x is None or y is None:
            return
        scale = 0.8 if event.button == 'up' else 1.25   # 上滚=放大，下滚=缩小
        xlim = self.ax.get_xlim()
        ylim = self.ax.get_ylim()
        self.ax.set_xlim([x - (x - xlim[0]) * scale, x + (xlim[1] - x) * scale])
        self.ax.set_ylim([y - (y - ylim[0]) * scale, y + (ylim[1] - y) * scale])
        self.canvas.draw_idle()

    def closeEvent(self, e):
        if self.reader:
            self.reader.stop()
        super().closeEvent(e)


def main():
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    win = MainWindow()
    win.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()

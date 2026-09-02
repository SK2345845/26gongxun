# -*- coding: utf-8 -*-
"""
激光雷达双视角扫描合并建图 + 障碍检测 + 路径规划

用法：
  1. 依赖同 lidar_viewer.py / field_map.py：pyserial numpy matplotlib PyQt5
  2. 雷达（USB 转 TTL 或官方转接板）插电脑
  3. 运行：python lidar_scan_merge.py
  4. 流程（两种打法都支持）：
       原地转 90°：  选「启停区一」→ 采集第1帧 → 车头角改+90° → 采集第2帧
       两点位各采一帧：选「启停区一」采第1帧 → 车开到启停区二摆好朝向，
                      起点位改「启停区二」、车头角改实际值 → 采集第2帧
       → 合并分析（只识别显示障碍）→ 人工确认 → 「确认障碍 · 规划路线」

关键机制：
  - 每帧采集时自动记录当时的基准（起点位+车头角+雷达偏移），
    合并时各帧用各自的基准转世界坐标——忘切基准采完立刻能从状态栏看出来
  - 前向视场角可调（160°/180° 不确定）；15cm 近端门限滤车体自回波
  - 底部三面板云图：实时 | 第1帧 | 第2帧（采到的就是裁剪后的）
  - 转速 RPM 可调（0=不调速；降转速可提高每圈点数，需供电充足）

坐标约定（与 field_map.py 一致，单位 mm）：
  - 世界系：原点在场地左下角，+x 向右，+y 向上
  - 车头朝向 heading：0°=+x 向右，逆时针为正；180°=朝「仿真脚本左边」(-x)
  - 雷达角 a：0°=车正前，顺时针增大（与 lidar_viewer 一致）
  - 点世界方位角 = heading - a
"""

import sys
import math
import time
import os
import json
from datetime import datetime

import numpy as np
import serial
import serial.tools.list_ports

from PyQt5.QtWidgets import (QApplication, QWidget, QVBoxLayout, QHBoxLayout,
                             QComboBox, QPushButton, QLabel, QDockWidget,
                             QSpinBox, QFileDialog)
from PyQt5.QtCore import Qt, QTimer

import matplotlib

matplotlib.rcParams['axes.unicode_minus'] = False

# 复用 field_map 的场地几何 / 连通图 / A* / 地图窗口
import field_map as fm
# 复用 lidar_viewer 的雷达解析 / 串口线程
import lidar_viewer as lv

# 中文乱码修复：运行时从系统已装字体里挑一个能用的（不写死字体名）
from matplotlib import font_manager as _font_manager

def _setup_cjk_font():
    installed = {f.name for f in _font_manager.fontManager.ttflist}
    for name in ["Microsoft YaHei", "SimHei", "SimSun", "Noto Sans CJK SC",
                 "Noto Sans SC", "Source Han Sans SC", "WenQuanYi Zen Hei",
                 "WenQuanYi Micro Hei", "PingFang SC", "Arial Unicode MS"]:
        if name in installed:
            matplotlib.rcParams["font.sans-serif"] = [name]
            break

_setup_cjk_font()      # 必须在 import field_map 之后（它会覆盖 rcParams）


# ==================== 合并 / 过滤 / 聚类参数 ====================
MIN_RANGE = 150.0         # 近端门限 15cm：滤车体自回波（车身边缘最近约150mm）
EXCLUDE_MARGIN = 40.0     # 过滤固定设施时的外扩余量 (mm)
DBSCAN_EPS = 80.0         # 聚类邻域半径 (mm)，小于两个障碍的最小间距即可
DBSCAN_MIN_PTS = 1        # 成簇最少点数，低于此算噪声
                          # 远处 φ50 障碍只回 1~2 个点，设 3 会漏检
ANG_BIN = 0.2            # 角度聚合 bin 宽 (°)：越细越能保留远处障碍的多个采样点
ANG_BINS = int(360.0 / ANG_BIN)   # 1440
# 前向视场角在 GUI 上调（「前向视场°」，默认 180）
# ============================================================


def front_points(points, half_deg=90.0):
    """只保留前向扇区：雷达角 0°=正前、顺时针增，取 0° 两侧 ±half_deg；
    同时滤掉 15cm 内的近端点（车体自回波）"""
    return [(a, d) for a, d in points
            if d >= MIN_RANGE
            and (a <= half_deg or a >= 360.0 - half_deg)]


def radar_to_world(points, xc, yc, heading_deg, radar_offset_mm=0.0):
    """雷达极坐标点 [(angle_deg, dist_mm)] → 世界笛卡尔 [(x, y)]。

    heading_deg : 车头世界朝向（0°=+x，逆时针正）
    radar_offset_mm : 雷达相对车中心的纵向偏移（安装在车头 → 正值，约半车长）
    """
    rx = xc + radar_offset_mm * math.cos(math.radians(heading_deg))
    ry = yc + radar_offset_mm * math.sin(math.radians(heading_deg))
    out = []
    for a, d in points:
        wa = math.radians(heading_deg - a)          # 雷达角顺时针，故取负
        out.append((rx + d * math.cos(wa), ry + d * math.sin(wa)))
    return out


def world_to_polar(wx, wy, base_xy, heading_deg, radar_offset_mm=0.0):
    """世界坐标 → 雷达极坐标（radar_to_world 的逆，自测用）"""
    rx = base_xy[0] + radar_offset_mm * math.cos(math.radians(heading_deg))
    ry = base_xy[1] + radar_offset_mm * math.sin(math.radians(heading_deg))
    d = math.hypot(wx - rx, wy - ry)
    phi = math.degrees(math.atan2(wy - ry, wx - rx))
    return ((heading_deg - phi) % 360.0, d)


def is_excluded(wx, wy):
    """返回 True 表示该世界坐标点不应算障碍（固定设施 / 场外）。"""
    m = EXCLUDE_MARGIN
    # 1) 场外（含右侧二维码板，它在地图 x>2400 之外）
    if not (-m <= wx <= fm.FIELD_SIZE + m and -m <= wy <= fm.FIELD_SIZE + m):
        return True
    # 2) 原料区圆盘（圆心在地图外上边界上方，圆盘部分伸入场地）
    if math.hypot(wx - fm.TURNTABLE_C[0], wy - fm.TURNTABLE_C[1]) < fm.TURNTABLE_R + m:
        return True
    # 3) 暂存区 / 粗加工区矩形（内部物料不算障碍）
    for (x0, y0, x1, y1) in (fm.ZC_RECT, fm.CG_RECT):
        if (x0 - m) <= wx <= (x1 + m) and (y0 - m) <= wy <= (y1 + m):
            return True
    return False


def dbscan(points, eps, min_pts):
    """极简 DBSCAN，返回 [(cx, cy, n), ...]（簇中心 + 点数）"""
    pts = np.asarray(points, dtype=float)
    n = len(pts)
    if n == 0:
        return []
    labels = np.full(n, -1, dtype=int)   # -1 未访问，-2 噪声
    cid = 0
    for i in range(n):
        if labels[i] != -1:
            continue
        neigh = np.where(np.linalg.norm(pts - pts[i], axis=1) <= eps)[0]
        if len(neigh) < min_pts:
            labels[i] = -2
            continue
        labels[i] = cid
        queue = list(neigh)
        qi = 0
        while qi < len(queue):
            j = queue[qi]
            qi += 1
            if labels[j] == -2:
                labels[j] = cid
            if labels[j] != -1:
                continue
            labels[j] = cid
            n2 = np.where(np.linalg.norm(pts - pts[j], axis=1) <= eps)[0]
            if len(n2) >= min_pts:
                queue.extend(n2)
        cid += 1
    out = []
    for c in range(cid):
        idx = np.where(labels == c)[0]
        out.append((float(pts[idx, 0].mean()),
                    float(pts[idx, 1].mean()),
                    int(len(idx))))
    return out


def detect_obstacles(world_points):
    """世界坐标点列表 → 障碍中心列表 [(x, y), ...]"""
    keep = [(x, y) for x, y in world_points if not is_excluded(x, y)]
    return [(cx, cy) for cx, cy, _ in dbscan(keep, DBSCAN_EPS, DBSCAN_MIN_PTS)]


def scan_ports():
    """扫描可用串口（能打开才算），返回 [port, ...]"""
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
            t = serial.Serial(p, lv.BAUD, timeout=0.05)
            ok.append(p)
            t.close()
        except Exception:
            pass
    return ok


class MergeWindow(fm.MainWindow):
    """在 field_map 地图窗口基础上，叠加「雷达双视角采集 + 合并 + 障碍检测」"""

    def __init__(self):
        # 先初始化本类新增属性，再调 super().__init__()：
        # field_map.MainWindow.__init__ 内部会调 self._draw_map()（已被本类重写），
        # 若 merged_cloud 尚未建立会抛 AttributeError。
        self.reader = None
        self.parser = lv.LidarParser()
        self.frame1 = None          # 第 1 帧点 [(a, d)]
        self.frame2 = None          # 第 2 帧点 [(a, d)]
        self.frame1_pose = None     # 第1帧采集时的基准 {start, hdg, off}
        self.frame2_pose = None     # 第2帧同上
        self.merged_cloud = []      # 合并后的世界坐标点（画图用）
        self.obstacles = []
        super().__init__()
        self.setWindowTitle("雷达双视角合并建图 + 障碍检测 + 路径规划")
        self._build_radar_dock()
        self._build_cloud_dock()
        # 实时云图刷新（连上雷达后每 200ms 重画左面板）
        self._live_timer = QTimer(self)
        self._live_timer.timeout.connect(self._update_live)
        self._live_timer.start(200)

    # ---------------- 雷达控制停靠栏 ----------------
    def _build_radar_dock(self):
        dock = QDockWidget("雷达双视角采集", self)
        dock.setFeatures(QDockWidget.DockWidgetMovable | QDockWidget.DockWidgetFloatable)
        w = QWidget()
        v = QVBoxLayout(w)
        v.setContentsMargins(8, 8, 8, 8)
        v.setSpacing(6)

        # 串口行
        h1 = QHBoxLayout()
        h1.addWidget(QLabel("串口:"))
        self.port_cb = QComboBox()
        self.port_cb.setMinimumWidth(100)
        h1.addWidget(self.port_cb)
        b = QPushButton("扫描")
        b.clicked.connect(self._refresh_ports)
        h1.addWidget(b)
        self.cn = QPushButton("连接")
        self.cn.clicked.connect(self._toggle_connect)
        h1.addWidget(self.cn)
        v.addLayout(h1)

        # 车头朝向 / 转角 / 雷达偏移
        h2 = QHBoxLayout()
        h2.addWidget(QLabel("车头朝向°"))
        self.hdg_spin = QSpinBox()
        self.hdg_spin.setRange(0, 360)
        self.hdg_spin.setValue(180)
        h2.addWidget(self.hdg_spin)
        h2.addWidget(QLabel("转角°"))
        self.rot_spin = QSpinBox()
        self.rot_spin.setRange(-180, 180)
        self.rot_spin.setValue(90)
        h2.addWidget(self.rot_spin)
        h2.addWidget(QLabel("雷达偏移mm"))
        self.off_spin = QSpinBox()
        self.off_spin.setRange(0, 400)
        self.off_spin.setValue(150)
        h2.addWidget(self.off_spin)
        v.addLayout(h2)

        # 前向视场角 / 电机转速
        h2b = QHBoxLayout()
        h2b.addWidget(QLabel("前向视场°"))
        self.fov_spin = QSpinBox()
        self.fov_spin.setRange(60, 360)
        self.fov_spin.setSingleStep(10)
        self.fov_spin.setValue(180)
        self.fov_spin.setToolTip("不确定雷达实际视场时在此调整，采集即生效")
        h2b.addWidget(self.fov_spin)
        h2b.addWidget(QLabel("转速RPM"))
        self.rpm_spin = QSpinBox()
        self.rpm_spin.setRange(0, 900)
        self.rpm_spin.setSingleStep(50)
        self.rpm_spin.setValue(0)
        self.rpm_spin.setSpecialValueText("不调速")
        self.rpm_spin.setToolTip("600=10Hz标准；300≈5Hz 每圈点数×2（需供电充足）。\n"
                                 "0=不发调速命令（最稳）")
        h2b.addWidget(self.rpm_spin)
        v.addLayout(h2b)

        # 采集 / 合并
        h3 = QHBoxLayout()
        self.f1b = QPushButton("采集第1帧")
        self.f1b.clicked.connect(self._capture_frame1)
        h3.addWidget(self.f1b)
        self.f2b = QPushButton("采集第2帧")
        self.f2b.clicked.connect(self._capture_frame2)
        h3.addWidget(self.f2b)
        v.addLayout(h3)

        self.mg = QPushButton("合并分析（仅识别障碍）")
        self.mg.clicked.connect(self._merge_analyze)
        v.addWidget(self.mg)

        self.plan_btn = QPushButton("确认障碍 · 规划路线")
        self.plan_btn.clicked.connect(self._default_task)
        self.plan_btn.setStyleSheet("font-weight:bold")
        v.addWidget(self.plan_btn)

        # 保存 / 加载帧（离线重放调参用）
        h4 = QHBoxLayout()
        self.svb = QPushButton("保存帧")
        self.svb.clicked.connect(self._save_frames)
        h4.addWidget(self.svb)
        self.ldbtn = QPushButton("加载帧")
        self.ldbtn.clicked.connect(self._load_frames)
        h4.addWidget(self.ldbtn)
        v.addLayout(h4)

        self.st = QLabel("未连接")
        self.st.setStyleSheet("color:#16a085;font-weight:bold")
        v.addWidget(self.st)
        v.addStretch()

        dock.setWidget(w)
        self.addDockWidget(Qt.RightDockWidgetArea, dock)
        self._refresh_ports()

    # ---------------- 三面板云图停靠栏 ----------------
    def _build_cloud_dock(self):
        from matplotlib.figure import Figure
        from matplotlib.backends.backend_qt5agg import FigureCanvas
        dock = QDockWidget("雷达云图：实时 | 第1帧 | 第2帧", self)
        dock.setFeatures(QDockWidget.DockWidgetMovable | QDockWidget.DockWidgetFloatable)
        self.cloud_fig = Figure(figsize=(11, 3.4), dpi=100)
        self.cloud_canvas = FigureCanvas(self.cloud_fig)
        self.ax_live, self.ax_f1, self.ax_f2 = self.cloud_fig.subplots(1, 3)
        self._draw_local(self.ax_live, [], "实时", "#0ea5e9")
        self._draw_local(self.ax_f1, [], "第1帧", "#16a34a")
        self._draw_local(self.ax_f2, [], "第2帧", "#d97706")
        dock.setWidget(self.cloud_canvas)
        self.addDockWidget(Qt.BottomDockWidgetArea, dock)
        self.resizeDocks([dock], [320], Qt.Vertical)

    def _draw_local(self, ax, pts, title, color):
        """雷达本地坐标画点云：0°朝上（车正前），右侧角度增"""
        ax.clear()
        if pts:
            xs = [d * math.sin(math.radians(a)) for a, d in pts]
            ys = [d * math.cos(math.radians(a)) for a, d in pts]
            ax.scatter(xs, ys, s=2, c=color)
        r = 4000
        ax.plot(0, 0, marker="v", color="red", ms=8)    # 红三角=车头
        ax.set_title(title, fontsize=9)
        ax.set_aspect("equal")
        ax.set_xlim(-r, r)
        ax.set_ylim(-0.15 * r, 1.05 * r)
        ax.grid(alpha=0.3)

    def _update_live(self):
        """定时刷新实时面板（未连接时不动）"""
        if not (self.reader and self.reader.running):
            return
        latest = self.reader.get_latest()
        if not latest:
            return
        self._draw_local(self.ax_live, latest,
                         f"实时 ({len(latest)}点)", "#0ea5e9")
        self.cloud_canvas.draw_idle()

    # ---------------- 串口管理 ----------------
    def _refresh_ports(self):
        avail = scan_ports()
        cur = self.port_cb.currentText()
        self.port_cb.blockSignals(True)
        self.port_cb.clear()
        self.port_cb.addItems(avail if avail else ["(无)"])
        if cur in avail:
            self.port_cb.setCurrentText(cur)
        self.port_cb.blockSignals(False)

    def _toggle_connect(self):
        if self.reader and self.reader.running:
            self.reader.stop()
            self.reader = None
            self.cn.setText("连接")
            self.st.setText("未连接")
            return
        port = self.port_cb.currentText()
        if not port or port == "(无)":
            return
        self.reader = lv.SerialReader(port, lv.BAUD, self.parser,
                                      rpm=self.rpm_spin.value())
        if self.reader.start():
            self.cn.setText("断开")
            if self.reader.rpm and self.reader.speed_applied:
                self.st.setText(f"已连接 {port} · {self.reader.rpm}RPM")
            elif self.reader.rpm:
                self.st.setText("已连接（调速未生效，已按默认转速恢复）")
            else:
                self.st.setText(f"已连接 {port}")
        else:
            self.reader = None
            self.st.setText("打开串口失败")

    def _capture_revolution(self):
        """短时间收集多圈并按角度 bin 平均，降噪；返回 [(a, d)] 或 None"""
        if not self.reader:
            return None
        frames = []
        last = None
        deadline = time.time() + 0.4
        while time.time() < deadline and len(frames) < 4:
            cur = self.reader.get_latest()
            if cur is not None and cur is not last:
                frames.append(cur)
                last = cur
            time.sleep(0.02)
        if not frames:
            return None
        acc = {}
        for f in frames:
            for a, d in f:
                b = int(round(a / ANG_BIN)) % ANG_BINS
                acc[b] = (acc[b] + d) / 2 if b in acc else d
        return [(float(b) * ANG_BIN, d) for b, d in acc.items()]

    def _cur_pose(self):
        """当前 GUI 参数作为一帧的采集基准"""
        return {"start": self.start_cb.currentText(),
                "hdg": self.hdg_spin.value(),
                "off": self.off_spin.value()}

    @staticmethod
    def _pose_base(pose):
        """基准点位文字（'启停区一'/'启停区二'）→ 场地坐标"""
        node = "S1" if pose == "启停区一" else "S2"
        return fm.NODES[node]

    def _capture_frame1(self):
        raw = self._capture_revolution()
        if raw:
            half = self.fov_spin.value() / 2.0
            self.frame1 = front_points(raw, half)
            self.frame1_pose = self.start_cb.currentText()   # 记住基准点位
            self._draw_local(self.ax_f1, self.frame1,
                             f"第1帧 ({len(self.frame1)}点)", "#16a34a")
            self.cloud_canvas.draw_idle()
            self.st.setText(f"第1帧：{len(self.frame1)} 点 · 基准 {self.frame1_pose}")
        else:
            self.st.setText("没采到数据，先连接雷达")

    def _capture_frame2(self):
        raw = self._capture_revolution()
        if raw:
            half = self.fov_spin.value() / 2.0
            self.frame2 = front_points(raw, half)
            self.frame2_pose = self.start_cb.currentText()
            self._draw_local(self.ax_f2, self.frame2,
                             f"第2帧 ({len(self.frame2)}点)", "#d97706")
            self.cloud_canvas.draw_idle()
            self.st.setText(f"第2帧：{len(self.frame2)} 点 · 基准 {self.frame2_pose}")
        else:
            self.st.setText("没采到数据，先连接雷达")

    # ---------------- 帧存取（JSON，含采集参数，可离线重放调参） ----------------
    def _write_frames(self, path):
        data = {
            "type": "lidar_merge_frames",
            "saved_at": datetime.now().isoformat(timespec="seconds"),
            "start1": self.frame1_pose,      # 第1帧基准点位（'启停区一'/'启停区二'）
            "start2": self.frame2_pose,      # 第2帧基准点位
            "hdg_deg": self.hdg_spin.value(),
            "rot_deg": self.rot_spin.value(),
            "radar_off_mm": self.off_spin.value(),
            "fov_deg": self.fov_spin.value(),
            "rpm": self.rpm_spin.value(),
            "frame1": self.frame1,           # [[angle_deg, dist_mm], ...] 已裁剪帧
            "frame2": self.frame2,
        }
        with open(path, "w", encoding="utf-8") as f:
            json.dump(data, f, ensure_ascii=False)

    def _read_frames(self, path):
        with open(path, "r", encoding="utf-8") as f:
            data = json.load(f)
        if data.get("type") != "lidar_merge_frames":
            raise ValueError("不是本工具保存的雷达帧文件")
        if not data.get("frame1") or not data.get("frame2"):
            raise ValueError("文件里没有完整的两帧数据")
        self.frame1 = [tuple(p) for p in data["frame1"]]
        self.frame2 = [tuple(p) for p in data["frame2"]]
        # 恢复两帧基准点位（同时把起点位下拉框切到第1帧，保持 UI 一致）
        if data.get("start1"):
            self.frame1_pose = data["start1"]
            idx = self.start_cb.findText(data["start1"])
            if idx >= 0:
                self.start_cb.setCurrentIndex(idx)
        if data.get("start2"):
            self.frame2_pose = data["start2"]
        # 恢复采集时的参数（字段缺省则保留当前值）
        for key, spin in (("hdg_deg", self.hdg_spin), ("rot_deg", self.rot_spin),
                          ("radar_off_mm", self.off_spin), ("fov_deg", self.fov_spin),
                          ("rpm", self.rpm_spin)):
            if key in data:
                spin.setValue(max(spin.minimum(), min(spin.maximum(), int(data[key]))))

    def _save_frames(self):
        if not self.frame1 or not self.frame2:
            self.st.setText("没有可保存的帧，先采集两帧")
            return
        default = os.path.abspath(f"lidar_frames_{datetime.now():%Y%m%d_%H%M%S}.json")
        path, _ = QFileDialog.getSaveFileName(self, "保存雷达帧", default,
                                              "JSON (*.json)")
        if not path:
            return
        try:
            self._write_frames(path)
            self.st.setText(f"已保存: {os.path.basename(path)}")
        except Exception as e:
            self.st.setText(f"保存失败: {e}")

    def _load_frames(self):
        path, _ = QFileDialog.getOpenFileName(self, "加载雷达帧", "",
                                              "JSON (*.json)")
        if not path:
            return
        try:
            self._read_frames(path)
        except Exception as e:
            self.st.setText(f"加载失败: {e}")
            return
        self._draw_local(self.ax_f1, self.frame1,
                         f"第1帧 ({len(self.frame1)}点)", "#16a34a")
        self._draw_local(self.ax_f2, self.frame2,
                         f"第2帧 ({len(self.frame2)}点)", "#d97706")
        self.cloud_canvas.draw_idle()
        self.st.setText(f"已加载: {os.path.basename(path)} · 参数一并恢复，可直接合并")

    # ---------------- 合并 + 检测 + 规划 ----------------
    def _merge_analyze(self):
        if not self.frame1 or not self.frame2:
            self.st.setText("请先采集两帧")
            return

        # 车头角/转角：与实测可靠版完全一致，合并时读取（h2 = 车头角 + 转角），
        # 采集时不动、也不用记——操作习惯不变
        # 每帧位姿只记录"基准点位"：修 S2 平移误检（两点位各采一帧时
        # 第二帧基准忘切会让障碍整体平移），采第2帧前只需切起点位
        p1 = self.frame1_pose or self.start_cb.currentText()
        p2 = self.frame2_pose or p1
        base1 = self._pose_base(p1)
        base2 = self._pose_base(p2)

        self._go()                       # 车体标记落在当前选择的起点位
        h1 = float(self.hdg_spin.value())
        rot = float(self.rot_spin.value())
        off = float(self.off_spin.value())
        h2 = h1 + rot

        w1 = radar_to_world(self.frame1, base1[0], base1[1], h1, off)
        w2 = radar_to_world(self.frame2, base2[0], base2[1], h2, off)
        self.merged_cloud = w1 + w2

        self.obstacles = detect_obstacles(self.merged_cloud)

        print(f"[合并] 第1帧基准 {p1} | 第2帧基准 {p2} | "
              f"车头 {h1:.0f}° / 转角 {rot:.0f}°")
        print(f"[合并] 第1帧 {len(self.frame1)} 点, 第2帧 {len(self.frame2)} 点, "
              f"合并 {len(self.merged_cloud)} 点, 检测到 {len(self.obstacles)} 个障碍")
        for i, (ox, oy) in enumerate(self.obstacles, 1):
            print(f"   障碍{i}: ({ox:.0f}, {oy:.0f}) mm")

        self.st.setText(f"障碍 {len(self.obstacles)} 个 · 基准 {p1}/{p2}"
                        f" · 车头 {h1:.0f}° + 转角 {rot:.0f}°"
                        f" · 确认后点「规划路线」")
        self.robot_st.setText(f"状态: 检测到 {len(self.obstacles)} 个障碍，"
                              f"确认无误后点「确认障碍 · 规划路线」")
        self._draw_map()

    # ---------------- 叠加合并点云显示 ----------------
    def _draw_map(self):
        super()._draw_map()
        if self.merged_cloud:
            xs = [p[0] for p in self.merged_cloud]
            ys = [p[1] for p in self.merged_cloud]
            self.ax.scatter(xs, ys, s=1, c="cyan", alpha=0.25, label="合并点云")
            for i, (ox, oy) in enumerate(self.obstacles, 1):
                self.ax.annotate(f"障碍{i}", (ox, oy),
                                 textcoords="offset points", xytext=(8, 8),
                                 fontsize=8, color="red", fontweight="bold")
            self.canvas.draw_idle()


def _selftest():
    """离线自测：①基准位姿往返正确 ②S2 基准忘切的旧 bug 不再复现
    ③视场角可调生效。退出码 0=通过。用法：python lidar_scan_merge.py --selftest
    """
    app = QApplication(sys.argv)
    win = MergeWindow()
    s1 = fm.NODES["S1"]
    s2 = fm.NODES["S2"]
    off = 150.0
    ok = True

    # ① 世界点 → 极坐标 → 世界点 往返（同一位姿）
    T = (1000, 1200)
    pol = world_to_polar(T[0], T[1], s1, 180, off)
    back = radar_to_world([pol], s1[0], s1[1], 180, off)[0]
    good = math.hypot(back[0] - T[0], back[1] - T[1]) < 1e-6
    print(f"[自测] 往返变换: {'OK' if good else 'X 失败'}")
    ok &= good

    # ② S2 基准回归：帧在 S2 采（基准记 S2），合并必须落回真值位置
    #   （旧 bug：基准还是 S1 时，点会被平移约 (S1-S2) 的距离）
    pts = [T, (T[0] + 30, T[1]), (T[0], T[1] + 30)]   # 3点成簇
    pol_s2 = [world_to_polar(x, y, s2, 180, off) for x, y in pts]
    win.frame1 = list(pol_s2)
    win.frame2 = list(pol_s2)
    win.frame1_pose = "启停区二"
    win.frame2_pose = "启停区二"
    win.fov_spin.setValue(180)
    win.rot_spin.setValue(0)          # h2 = 180 + 0
    win._merge_analyze()
    hit = any(math.hypot(ox - T[0], oy - T[1]) < 120
              for ox, oy in win.obstacles)
    print(f"[自测] S2 逐帧基准: {'OK' if hit else 'X 失败'}（障碍应落在 {T} 附近）")
    ok &= hit

    # ③ 视场角可调：160° 时 100° 方向的点应被裁掉、70° 保留
    keep = front_points([(70.0, 1000), (100.0, 1000)], 160 / 2)
    good = len(keep) == 1 and abs(keep[0][0] - 70.0) < 1
    print(f"[自测] 视场角裁剪: {'OK' if good else 'X 失败'}")
    ok &= good

    # ④ 15cm 近端门限
    keep = front_points([(45.0, 120), (45.0, 300)])
    good = len(keep) == 1 and abs(keep[0][1] - 300) < 1
    print(f"[自测] 15cm 近端门限: {'OK' if good else 'X 失败'}")
    ok &= good

    print("[自测] 全部通过" if ok else "[自测] 存在失败项")
    return 0 if ok else 1


def main():
    if "--selftest" in sys.argv:
        sys.exit(_selftest())
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    win = MergeWindow()
    win.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()

# -*- coding: utf-8 -*-
"""
激光雷达双视角扫描合并建图 + 障碍检测 + 路径规划

用法：
  1. 依赖同 lidar_viewer.py / field_map.py：pyserial numpy matplotlib PyQt5
  2. 雷达（USB 转 TTL 或官方转接板）插电脑
  3. 运行：python lidar_scan_merge.py
  4. 流程：
       选串口 → 连接 → 左侧选「启停区一(S1)/启停区二(S2)」
       → 采集第1帧 → 车原地转 90° → 采集第2帧
       → 合并分析 + 路径规划

原理：
  - 雷达前装，视场角不确定（160°/180°）→ GUI「前向视场°」可调；「最大距离mm」可调
  - 车在原地转 90° 采两帧，合并 = 各自用不同车头朝向转到世界坐标取并集
  - 过滤：场外(含二维码板)、原料区圆盘、暂存区/粗加工区矩形内的点都不算障碍
  - 剩余点 DBSCAN 聚类 → 簇尺寸>300mm 判墙丢弃 → 障碍中心 → 写入 field_map 的
    obstacles → 复用其 A* 规划
  - 底部三面板云图：实时 | 第1帧截取 | 第2帧截取

坐标约定（与 field_map.py 一致，单位 mm）：
  - 世界系：原点在场地左下角，+x 向右，+y 向上
  - 车头朝向 heading：0°=+x 向右，逆时针为正；180°=朝「仿真脚本左边」(-x)
  - 雷达角 a：0°=车正前，顺时针增大（与 lidar_viewer 一致）
  - 点世界方位角 = heading - a
"""

import sys
import math
import time
import json
import os
from datetime import datetime

import numpy as np
import serial
import serial.tools.list_ports

from PyQt5.QtWidgets import (QApplication, QWidget, QVBoxLayout, QHBoxLayout,
                             QComboBox, QPushButton, QLabel, QDockWidget,
                             QSpinBox, QFileDialog, QCheckBox)
from PyQt5.QtCore import Qt, QTimer

import matplotlib

matplotlib.rcParams['axes.unicode_minus'] = False
from matplotlib.patches import Circle
from matplotlib.figure import Figure
from matplotlib.backends.backend_qt5agg import FigureCanvas

# 复用 field_map 的场地几何 / 连通图 / A* / 地图窗口
import field_map as fm
# 复用 lidar_viewer 的雷达解析 / 串口线程
import lidar_viewer as lv


# ---- 中文乱码修复：不写死字体名，运行时从系统已装字体里挑一个能用的 ----
from matplotlib import font_manager

def _setup_cjk_font():
    """按优先级挑系统里真实存在的 CJK 字体；一个都没有就扫名字带汉字特征的。"""
    prefer = ["Microsoft YaHei", "SimHei", "SimSun", "Noto Sans CJK SC",
              "Noto Sans SC", "Source Han Sans SC", "WenQuanYi Zen Hei",
              "WenQuanYi Micro Hei", "PingFang SC", "Arial Unicode MS"]
    installed = {f.name for f in font_manager.fontManager.ttflist}
    for name in prefer:
        if name in installed:
            matplotlib.rcParams["font.sans-serif"] = [name]
            break
    else:
        keys = ("CJK", "YaHei", "SimHei", "SimSun", "Hei", "Song", "Kai",
                "WenQuanYi", "Han")
        for name in sorted(installed):
            if any(k.lower() in name.lower() for k in keys):
                matplotlib.rcParams["font.sans-serif"] = [name]
                break

_setup_cjk_font()      # 必须在 import field_map 之后（它会覆盖 rcParams）


# ==================== 合并 / 过滤 / 聚类参数 ====================
MIN_RANGE = 260.0         # 近端门限 (mm)：车体最大回转半径212mm，必须滤掉自回波
EXCLUDE_MARGIN = 40.0     # 过滤固定设施时的外扩余量 (mm)
DBSCAN_EPS = 80.0         # 聚类邻域半径 (mm)，小于两个障碍的最小间距即可
DBSCAN_MIN_PTS = 2        # 成簇最少点数：场内即障碍（宁多勿漏），单点毛刺扔掉
MIN_VIEW_PTS = 2          # 双视角确认：每个障碍在两个视角各至少几个支撑点
DBSCAN_SPLIT_EPS = 45.0   # 大簇二次拆分邻域 (mm)：障碍挨着别的回波被链式吞掉时，
                          # 拆出紧凑子簇保留；真墙点距密，拆完仍是大条 → 照样扔
MAX_CLUSTER_EXTENT = 300.0  # 簇最大外接尺寸 (mm)：φ50 障碍 + 噪声余量；
                            # 超过判为墙/长条设施，丢弃（否则墙心会出假障碍）
# 视场角（160°/180° 不确定）与最大扫描距离在 GUI 上调，见 _build_radar_dock
# ============================================================


def crop_points(points, half_deg, max_range):
    """按视场半角 + 距离窗裁剪：雷达角 0°=正前、顺时针增，前向 = 0° 两侧 ±half_deg"""
    return [(a, d) for a, d in points
            if MIN_RANGE <= d <= max_range
            and (a <= half_deg or a >= 360.0 - half_deg)]


def aggregate_revs(revs, min_frac=0.6):
    """多圈聚合滤波：按 1° 分箱，只保留出现在 ≥min_frac 圈数里的 bin，
    距离取中值。毛刺只在个别圈出现 → 被出现率门槛干掉；真障碍每圈都在，
    中值压抖动。静止采集时这是零成本的免费滤波。"""
    n = len(revs)
    if n <= 1:
        return list(revs[0]) if n else []
    need = max(2, math.ceil(n * min_frac))
    bins = {}
    for rev in revs:
        for a, d in rev:
            b = int(round(a)) % 360
            bins.setdefault(b, []).append(d)
    out = []
    for b, ds in sorted(bins.items()):
        if len(ds) >= need:
            ds = sorted(ds)
            m = len(ds) // 2
            med = ds[m] if len(ds) % 2 else (ds[m - 1] + ds[m]) / 2
            out.append((float(b), med))
    return out


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
    """极简 DBSCAN，返回 [(cx, cy, n, extent, members), ...]

    members = [(x, y), ...] 簇成员点（供大簇二次拆分用）
    """
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
        sub = pts[idx]
        extent = float(max(sub[:, 0].max() - sub[:, 0].min(),
                           sub[:, 1].max() - sub[:, 1].min()))
        out.append((float(sub[:, 0].mean()),
                    float(sub[:, 1].mean()),
                    int(len(idx)),
                    extent,
                    [(float(x), float(y)) for x, y in sub]))
    return out


def detect_obstacles(world_points):
    """世界坐标点列表 → 障碍中心列表 [(x, y), ...]

    过滤链：固定设施/场外掩膜 → DBSCAN 聚类 → 簇尺寸判墙（大簇二次拆分）。
    场外不用距离门限（对角线可达 3.4m，门限会误杀场内点），
    统一转世界坐标后按场地矩形做几何裁剪。
    """
    keep = [(x, y) for x, y in world_points if not is_excluded(x, y)]
    clusters = dbscan(keep, DBSCAN_EPS, DBSCAN_MIN_PTS)
    out = []
    for cx, cy, _, ext, members in clusters:
        if ext <= MAX_CLUSTER_EXTENT:
            out.append((cx, cy))
            continue
        # 大簇不整团扔：障碍挨着别的回波会被链式聚类吞掉（实测 25 点真障碍簇
        # 被连坐误扔）。用更小邻域二次拆分，紧凑子簇照样算障碍；
        # 真墙点距密、拆完仍是大条 → 照样扔
        for sx, sy, sn, sext, _ in dbscan(members, DBSCAN_SPLIT_EPS,
                                          DBSCAN_MIN_PTS):
            if sext <= MAX_CLUSTER_EXTENT:
                out.append((sx, sy))
    return out


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
        super().__init__()
        self.setWindowTitle("雷达双视角合并建图 + 障碍检测 + 路径规划")
        self.reader = None
        self.parser = lv.LidarParser()
        self.frame1 = None          # 第 1 帧原始点 [(a, d)]（未裁剪）
        self.frame2 = None          # 第 2 帧原始点 [(a, d)]（未裁剪）
        self.frame1_pose = None     # 第1帧采集时的基准位姿 {start,hdg,off}
        self.frame2_pose = None     # 第2帧同上——支持两个点位各采一帧
        self.merged_cloud = []      # 合并后的世界坐标点（画图用）
        self.obstacle_info = []     # [(cx, cy, 视角1点数, 视角2点数)]
        self._build_radar_dock()
        self._build_cloud_dock()
        # 实时云图刷新（连上雷达后每 200ms 重画左 panel）
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

        # 视场角 / 最大扫描距离（160° 还是 180° 不确定 → 都可调）
        h2b = QHBoxLayout()
        h2b.addWidget(QLabel("前向视场°"))
        self.fov_spin = QSpinBox()
        self.fov_spin.setRange(60, 360)
        self.fov_spin.setSingleStep(10)
        self.fov_spin.setValue(180)
        h2b.addWidget(self.fov_spin)
        h2b.addWidget(QLabel("最大距离mm"))
        self.rng_spin = QSpinBox()
        self.rng_spin.setRange(300, 10000)
        self.rng_spin.setSingleStep(100)
        self.rng_spin.setValue(4000)
        h2b.addWidget(self.rng_spin)
        h2b.addWidget(QLabel("转速RPM"))
        self.rpm_spin = QSpinBox()
        self.rpm_spin.setRange(0, 900)
        self.rpm_spin.setSingleStep(50)
        self.rpm_spin.setValue(0)          # 默认不调速（供电不稳会掉数据流）
        self.rpm_spin.setSpecialValueText("不调速")
        self.rpm_spin.setToolTip("600=10Hz标准；300≈5Hz 每圈点数×2。\n"
                                 "静止采集建议 300，圆柱点更多、检测更稳。\n"
                                 "0=不发调速命令（旧行为，调速异常时选这个）")
        h2b.addWidget(self.rpm_spin)
        h2b.addWidget(QLabel("采集圈数"))
        self.cnt_spin = QSpinBox()
        self.cnt_spin.setRange(1, 20)
        self.cnt_spin.setValue(5)
        self.cnt_spin.setToolTip("每帧采 N 圈做多圈聚合滤波：毛刺被出现率门槛干掉，\n"
                                 "真障碍取中值压抖动。静止采集建议 5 圈")
        h2b.addWidget(self.cnt_spin)
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

        # 双视角过滤（默认关）：勾选后单视角障碍剔除
        h4 = QHBoxLayout()
        self.both_chk = QCheckBox("剔除单视角障碍")
        self.both_chk.setChecked(False)
        self.both_chk.setToolTip("勾选后：两个视角各至少2个支撑点的障碍才保留。\n"
                                 "不勾选：全部显示，单视角障碍画虚线圈并在控制台标注")
        h4.addWidget(self.both_chk)
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
        dock = QDockWidget("雷达云图：实时 | 第1帧截取 | 第2帧截取", self)
        dock.setFeatures(QDockWidget.DockWidgetMovable | QDockWidget.DockWidgetFloatable)
        self.cloud_fig = Figure(figsize=(11, 3.4), dpi=100)
        self.cloud_canvas = FigureCanvas(self.cloud_fig)
        self.ax_live, self.ax_f1, self.ax_f2 = self.cloud_fig.subplots(1, 3)
        self._draw_local(self.ax_live, [], "实时", "#0ea5e9")
        self._draw_local(self.ax_f1, [], "第1帧截取", "#16a34a")
        self._draw_local(self.ax_f2, [], "第2帧截取", "#d97706")
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
        r = float(self.rng_spin.value())
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
                b = int(round(a)) % 360
                acc[b] = (acc[b] + d) / 2 if b in acc else d
        return [(float(b), d) for b, d in acc.items()]

    def _crop_cur(self, raw):
        """按当前 GUI 的视场角/最大距离裁剪原始帧"""
        return crop_points(raw, self.fov_spin.value(), self.rng_spin.value())

    def _cur_pose(self):
        """当前 GUI 参数作为一帧的采集位姿"""
        return {"start": self.start_cb.currentText(),
                "hdg": self.hdg_spin.value(),
                "off": self.off_spin.value()}

    @staticmethod
    def _pose_base(pose):
        """位姿里的起点文字 → 场地坐标"""
        node = "S1" if pose.get("start") == "启停区一" else "S2"
        return fm.NODES[node]

    def _capture_multi(self):
        """连采 N 圈做聚合滤波（毛刺剔除+中值），返回 (帧, 实际圈数)"""
        n = self.cnt_spin.value()
        revs = []
        for _ in range(n):
            QApplication.processEvents()         # 长采集保持界面响应
            rev = self._capture_revolution()
            if rev:
                revs.append(rev)
        if not revs:
            return None
        return aggregate_revs(revs), len(revs)

    def _capture_frame1(self):
        res = self._capture_multi()
        if res:
            raw, nrev = res
            self.frame1 = raw           # 存聚合后的帧，改裁剪参数后合并时重新截取
            self.frame1_pose = self._cur_pose()   # 记住这是在哪、朝哪采的
            crop = self._crop_cur(raw)
            self._draw_local(self.ax_f1, crop,
                             f"第1帧截取 ({len(crop)}点)", "#16a34a")
            self.cloud_canvas.draw_idle()
            self.st.setText(f"第1帧：{nrev} 圈聚合 {len(raw)} 点，截取 {len(crop)} 点"
                            f" · 基准 {self.frame1_pose['start']} 车头{self.frame1_pose['hdg']}°")
        else:
            self.st.setText("没采到数据，先连接雷达")

    def _capture_frame2(self):
        res = self._capture_multi()
        if res:
            raw, nrev = res
            self.frame2 = raw
            self.frame2_pose = self._cur_pose()
            crop = self._crop_cur(raw)
            self._draw_local(self.ax_f2, crop,
                             f"第2帧截取 ({len(crop)}点)", "#d97706")
            self.cloud_canvas.draw_idle()
            self.st.setText(f"第2帧：{nrev} 圈聚合 {len(raw)} 点，截取 {len(crop)} 点"
                            f" · 基准 {self.frame2_pose['start']} 车头{self.frame2_pose['hdg']}°")
        else:
            self.st.setText("没采到数据，先连接雷达")

    # ---------------- 帧存取（JSON，含全部采集参数，可离线重放调阈值） ----------------
    def _write_frames(self, path):
        data = {
            "type": "lidar_merge_frames",
            "saved_at": datetime.now().isoformat(timespec="seconds"),
            "start": self.start_cb.currentText(),
            "hdg_deg": self.hdg_spin.value(),
            "rot_deg": self.rot_spin.value(),
            "radar_off_mm": self.off_spin.value(),
            "fov_deg": self.fov_spin.value(),
            "max_range_mm": self.rng_spin.value(),
            "rpm": self.rpm_spin.value(),
            "frame1": self.frame1,      # [[angle_deg, dist_mm], ...] 原始帧
            "frame2": self.frame2,
            "frame1_pose": self.frame1_pose,    # 各帧采集时的基准位姿
            "frame2_pose": self.frame2_pose,
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
        self.frame1_pose = data.get("frame1_pose") or self._cur_pose()
        self.frame2_pose = data.get("frame2_pose") or self._cur_pose()
        # 恢复采集时的参数（字段缺省则保留当前值）
        if data.get("start"):
            idx = self.start_cb.findText(data["start"])
            if idx >= 0:
                self.start_cb.setCurrentIndex(idx)
        for key, spin in (("hdg_deg", self.hdg_spin), ("rot_deg", self.rot_spin),
                          ("radar_off_mm", self.off_spin), ("fov_deg", self.fov_spin),
                          ("max_range_mm", self.rng_spin), ("rpm", self.rpm_spin)):
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
        c1 = self._crop_cur(self.frame1)
        c2 = self._crop_cur(self.frame2)
        self._draw_local(self.ax_f1, c1, f"第1帧截取 ({len(c1)}点)", "#16a34a")
        self._draw_local(self.ax_f2, c2, f"第2帧截取 ({len(c2)}点)", "#d97706")
        self.cloud_canvas.draw_idle()
        self.st.setText(f"已加载: {os.path.basename(path)} · "
                        f"参数一并恢复，可直接改视场/距离后合并")

    # ---------------- 合并 + 检测 + 规划 ----------------
    def _merge_analyze(self):
        if not self.frame1 or not self.frame2:
            self.st.setText("请先采集两帧")
            return

        # 每帧用各自采集时的基准位姿：既支持原地转90°，也支持两点位各采一帧
        # （开过去采的第二帧如果还用同一基准，障碍会被平移出误检——实测踩过的坑）
        p1 = self.frame1_pose or self._cur_pose()
        p2 = self.frame2_pose or self._cur_pose()
        base1 = self._pose_base(p1)
        base2 = self._pose_base(p2)
        xc, yc = base2                      # 车体标记画在第2帧基准位（最后一次发车）
        h1 = float(p1["hdg"])
        h2 = float(p2["hdg"])

        f1 = self._crop_cur(self.frame1)  # 用当前视场角/最大距离重新截取
        f2 = self._crop_cur(self.frame2)
        w1 = radar_to_world(f1, base1[0], base1[1], h1, float(p1["off"]))
        w2 = radar_to_world(f2, base2[0], base2[1], h2, float(p2["off"]))
        self.merged_cloud = w1 + w2

        print(f"[合并] 第1帧基准 {p1['start']}·车头{h1:.0f}° | "
              f"第2帧基准 {p2['start']}·车头{h2:.0f}°")

        self.obstacles = detect_obstacles(self.merged_cloud)

        # 逐障碍统计两个视角的支撑点数（诊断信息，默认不剔除）：
        # 真障碍转 90° 后常能从另一方向再看一次；单视角簇可能是车体残留/多径/反光，
        # 但视场有限，真障碍也可能只落在一个视角 → 是否剔除由用户勾选决定
        self.obstacle_info = []                  # [(cx, cy, n1, n2)]
        for cx, cy in self.obstacles:
            n1 = sum(1 for x, y in w1 if math.hypot(x - cx, y - cy) < DBSCAN_EPS * 1.5)
            n2 = sum(1 for x, y in w2 if math.hypot(x - cx, y - cy) < DBSCAN_EPS * 1.5)
            self.obstacle_info.append((cx, cy, n1, n2))

        if self.both_chk.isChecked():
            dual = [o[:2] for o in self.obstacle_info
                    if o[2] >= MIN_VIEW_PTS and o[3] >= MIN_VIEW_PTS]
            for cx, cy, n1, n2 in self.obstacle_info:
                if (cx, cy) not in dual:
                    print(f"   剔除单视角障碍 ({cx:.0f},{cy:.0f})：视角1 {n1} 点 / 视角2 {n2} 点")
            self.obstacles = dual

        for i, (cx, cy, n1, n2) in enumerate(self.obstacle_info, 1):
            if (cx, cy) in self.obstacles:
                tag = "双视角" if (n1 >= MIN_VIEW_PTS and n2 >= MIN_VIEW_PTS) else "仅单视角"
                print(f"   障碍{i}: ({cx:.0f}, {cy:.0f}) mm [{tag}: 视角1 {n1} 点 / 视角2 {n2} 点]")

        print(f"[合并] 视场 {self.fov_spin.value()}°, 最大距离 {self.rng_spin.value()}mm, "
              f"第1帧 {len(f1)} 点, 第2帧 {len(f2)} 点, "
              f"合并 {len(self.merged_cloud)} 点, 最终 {len(self.obstacles)} 个障碍")

        self.st.setText(f"障碍 {len(self.obstacles)} 个 · 视场 {self.fov_spin.value()}°"
                        f" · {self.rng_spin.value()}mm · 基准 {p1['start']}{h1:.0f}°/"
                        f"{p2['start']}{h2:.0f}°")
        self.robot_st.setText(f"状态: 检测到 {len(self.obstacles)} 个障碍，确认无误后点「规划路线」")
        # 不立刻规划：先看障碍对不对，人工确认后再开始

    # ---------------- 叠加合并点云 / 障碍显示 ----------------
    def _draw_map(self):
        # fm.MainWindow.__init__ 会先调 _draw_map，此时子类属性还没建，需兜底
        if not hasattr(self, "merged_cloud"):
            return super()._draw_map()
        super()._draw_map()
        if self.merged_cloud:
            xs = [p[0] for p in self.merged_cloud]
            ys = [p[1] for p in self.merged_cloud]
            self.ax.scatter(xs, ys, s=1, c="cyan", alpha=0.25, label="合并点云")
        info = {((round(ox), round(oy))): (n1, n2)
                for ox, oy, n1, n2 in getattr(self, "obstacle_info", [])}
        for i, (ox, oy) in enumerate(getattr(self, "obstacles", []), 1):
            n1, n2 = info.get((round(ox), round(oy)), (0, 0))
            single = (n1 < MIN_VIEW_PTS or n2 < MIN_VIEW_PTS)
            if single:   # 单视角：虚线圈提示可能是误检（车体残留/多径/反光）
                self.ax.add_patch(Circle((ox, oy), fm.OBSTACLE_R + 10,
                                         facecolor="none", edgecolor="orange",
                                         lw=2, linestyle="--"))
                self.ax.annotate(f"障碍{i}?单视角", (ox, oy),
                                 textcoords="offset points", xytext=(8, 8),
                                 fontsize=8, color="darkorange", fontweight="bold")
            else:        # 双视角支撑：实心红圈
                self.ax.add_patch(Circle((ox, oy), fm.OBSTACLE_R + 10,
                                         facecolor="none", edgecolor="red", lw=2))
                self.ax.annotate(f"障碍{i}", (ox, oy), textcoords="offset points",
                                 xytext=(8, 8), fontsize=8, color="red", fontweight="bold")
        if self.merged_cloud or getattr(self, "obstacles", []):
            self.canvas.draw_idle()


def _selftest():
    """无雷达仿真自测：合成两帧点云（3 个真值障碍 + 左墙/下墙）走完整流水线。

    验证点：① 3 个障碍全部检出 ② 无误检（墙被尺寸过滤掉）
    ③ 合并点云与 A* 路径正确画图。退出码 0=通过。
    用法：python lidar_scan_merge.py --selftest
    """
    app = QApplication(sys.argv)
    win = MergeWindow()
    win.start_cb.setCurrentText("启停区一")     # S1 = (2050, 2050)
    win.hdg_spin.setValue(180)                  # 车头朝仿真脚本的左
    win.rot_spin.setValue(90)                   # 原地转 90° 朝下（二维码板一侧）
    win.off_spin.setValue(150)
    win.fov_spin.setValue(160)                  # 故意用 160° 验证视场可调
    win.rng_spin.setValue(4000)

    xc, yc = fm.NODES["S1"]
    off = 150.0

    # 真值障碍（避开 ZC/CG 矩形、原料区轮盘、禁行黄区）
    true_obs = [(700, 1750), (1150, 1050), (1600, 1950)]

    # 世界坐标合成点云：障碍轮廓 + 两面墙（墙点密集，验证簇尺寸过滤）
    world = []
    R = fm.OBSTACLE_R
    for ox, oy in true_obs:
        for k in range(12):
            th = 2 * math.pi * k / 12
            world.append((ox + R * math.cos(th), oy + R * math.sin(th)))
    for k in range(80):     # 左墙 x=0
        world.append((0.0, 2400.0 * k / 79))
    for k in range(80):     # 下墙 y=0
        world.append((2400.0 * k / 79, 0.0))

    # 逆变换成雷达极坐标 → 前向裁剪，模拟两帧
    def to_polar(pts, heading):
        rx = xc + off * math.cos(math.radians(heading))
        ry = yc + off * math.sin(math.radians(heading))
        pol = []
        for wx, wy in pts:
            phi = math.degrees(math.atan2(wy - ry, wx - rx))
            pol.append(((heading - phi) % 360.0, math.hypot(wx - rx, wy - ry)))
        return pol

    win.frame1 = to_polar(world, 180.0)         # 存原始帧（GUI 用当前参数截取）
    win.frame2 = to_polar(world, 270.0)
    win.frame1_pose = {"start": "启停区一", "hdg": 180, "off": 150}
    win.frame2_pose = {"start": "启停区一", "hdg": 270, "off": 150}  # 原地转90°
    win._merge_analyze()

    # ---- 核对 ----
    det = win.obstacles
    print(f"[自测] 视场 {win.fov_spin.value()}°，真值障碍 {len(true_obs)} 个，检出 {len(det)} 个")
    matched = 0
    for tx, ty in true_obs:
        hit = any(math.hypot(dx - tx, dy - ty) < 120 for dx, dy in det)
        matched += 1 if hit else 0
        print(f"  真值({tx:>4},{ty:>4}) -> {'OK 检出' if hit else 'X 漏检'}")
    false_pos = [(round(dx), round(dy)) for dx, dy in det
                 if not any(math.hypot(dx - tx, dy - ty) < 120 for tx, ty in true_obs)]
    print(f"[自测] 匹配 {matched}/{len(true_obs)}，误检 {len(false_pos)} {false_pos}")

    win._draw_map()
    out = "selftest_merge.png"
    win.figure.savefig(out, dpi=120)
    print(f"[自测] 地图已保存: {out}")
    win._draw_local(win.ax_f1, win._crop_cur(win.frame1),
                    f"第1帧截取 ({len(win._crop_cur(win.frame1))}点)", "#16a34a")
    win._draw_local(win.ax_f2, win._crop_cur(win.frame2),
                    f"第2帧截取 ({len(win._crop_cur(win.frame2))}点)", "#d97706")
    win._draw_local(win.ax_live, win.frame1, "实时(用第1帧演示)", "#0ea5e9")
    clouds = "selftest_clouds.png"
    win.cloud_fig.savefig(clouds, dpi=120)
    print(f"[自测] 三面板云图已保存: {clouds}")

    # ---- 帧存取往返测试 ----
    n1, n2 = len(win.frame1), len(win.frame2)
    tmp = "selftest_frames.json"
    win._write_frames(tmp)
    win.frame1, win.frame2 = None, None
    win.fov_spin.setValue(180)          # 故意改乱，验证加载时参数恢复
    win.hdg_spin.setValue(0)
    win._read_frames(tmp)
    ok_io = (len(win.frame1) == n1 and len(win.frame2) == n2
             and win.fov_spin.value() == 160 and win.hdg_spin.value() == 180)
    os.remove(tmp)
    print(f"[自测] 帧存取往返: {'OK' if ok_io else 'X 失败'}"
          f"（{n1}/{n2} 点, 参数恢复 视场{win.fov_spin.value()}°/车头{win.hdg_spin.value()}°）")

    # ---- 多圈聚合滤波测试：圆柱每圈都在，毛刺只出现1/5圈 ----
    revs = []
    for k in range(5):
        r = [(10.2, 1000 + (k % 3) * 8), (10.8, 1004 - (k % 2) * 6)]
        if k == 2:
            r.append((200.0, 1500))          # 毛刺：只在第3圈出现
        revs.append(r)
    agg = aggregate_revs(revs)
    ok_agg = (len(agg) == 2
              and not any(abs(a - 200.0) < 1 for a, _ in agg)
              and all(990 <= d <= 1015 for _, d in agg))
    print(f"[自测] 多圈聚合: {'OK' if ok_agg else 'X 失败'}（保留 {len(agg)}/3 个bin，毛刺被剔除）")
    return 0 if (matched == len(true_obs) and not false_pos and ok_io and ok_agg) else 1


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

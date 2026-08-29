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
  - 雷达前装，只有前向约 180° 视野；车在原地转 90° 采两帧，拼成约 270° 视野
  - 两帧在同一位置（S1/S2 原地旋转），合并 = 各自用不同车头朝向转到世界坐标取并集
  - 过滤：场外(含二维码板)、原料区圆盘、暂存区/粗加工区矩形内的点都不算障碍
  - 剩余点 DBSCAN 聚类 → 障碍中心 → 写入 field_map 的 obstacles → 复用其 A* 规划

坐标约定（与 field_map.py 一致，单位 mm）：
  - 世界系：原点在场地左下角，+x 向右，+y 向上
  - 车头朝向 heading：0°=+x 向右，逆时针为正；180°=朝「仿真脚本左边」(-x)
  - 雷达角 a：0°=车正前，顺时针增大（与 lidar_viewer 一致）
  - 点世界方位角 = heading - a
"""

import sys
import math
import time

import numpy as np
import serial
import serial.tools.list_ports

from PyQt5.QtWidgets import (QApplication, QWidget, QVBoxLayout, QHBoxLayout,
                             QComboBox, QPushButton, QLabel, QDockWidget,
                             QSpinBox)
from PyQt5.QtCore import Qt

import matplotlib

matplotlib.rcParams['font.sans-serif'] = ['Microsoft YaHei', 'SimHei', 'SimSun', 'Arial Unicode MS']
matplotlib.rcParams['axes.unicode_minus'] = False
from matplotlib.patches import Circle

# 复用 field_map 的场地几何 / 连通图 / A* / 地图窗口
import field_map as fm
# 复用 lidar_viewer 的雷达解析 / 串口线程
import lidar_viewer as lv


# ==================== 合并 / 过滤 / 聚类参数 ====================
FRONT_HALF_DEG = 90.0     # 前向半角：只取前向 180°（背向被车体挡住）。可调
EXCLUDE_MARGIN = 40.0     # 过滤固定设施时的外扩余量 (mm)
DBSCAN_EPS = 80.0         # 聚类邻域半径 (mm)，小于两个障碍的最小间距即可
DBSCAN_MIN_PTS = 3        # 成簇最少点数，低于此算噪声
MAX_CLUSTER_EXTENT = 300.0  # 簇最大外接尺寸 (mm)：φ50 障碍 + 噪声余量；
                            # 超过判为墙/长条设施，丢弃（否则墙心会出假障碍）
# ============================================================


def front_points(points):
    """只保留前向 180°：雷达角 0°=正前、顺时针增，前向 = 0° 两侧 ±FRONT_HALF_DEG"""
    return [(a, d) for a, d in points
            if a <= FRONT_HALF_DEG or a >= 360.0 - FRONT_HALF_DEG]


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
    """极简 DBSCAN，返回 [(cx, cy, n, extent), ...]（簇中心 + 点数 + 外接尺寸 mm）"""
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
                    extent))
    return out


def detect_obstacles(world_points):
    """世界坐标点列表 → 障碍中心列表 [(x, y), ...]

    三层过滤：固定设施/场外掩膜 → DBSCAN 聚类 → 簇尺寸判墙。
    场外不用距离门限（对角线可达 3.4m，门限会误杀场内点），
    统一转世界坐标后按场地矩形做几何裁剪。
    """
    keep = [(x, y) for x, y in world_points if not is_excluded(x, y)]
    clusters = dbscan(keep, DBSCAN_EPS, DBSCAN_MIN_PTS)
    return [(cx, cy) for cx, cy, _, ext in clusters if ext <= MAX_CLUSTER_EXTENT]


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
        self.frame1 = None          # 第 1 帧点 [(a, d)]
        self.frame2 = None          # 第 2 帧点 [(a, d)]
        self.merged_cloud = []      # 合并后的世界坐标点（画图用）
        self._build_radar_dock()

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

        # 采集 / 合并
        h3 = QHBoxLayout()
        self.f1b = QPushButton("采集第1帧")
        self.f1b.clicked.connect(self._capture_frame1)
        h3.addWidget(self.f1b)
        self.f2b = QPushButton("采集第2帧")
        self.f2b.clicked.connect(self._capture_frame2)
        h3.addWidget(self.f2b)
        v.addLayout(h3)

        self.mg = QPushButton("合并分析 + 路径规划")
        self.mg.clicked.connect(self._merge_analyze)
        v.addWidget(self.mg)

        self.st = QLabel("未连接")
        self.st.setStyleSheet("color:#16a085;font-weight:bold")
        v.addWidget(self.st)
        v.addStretch()

        dock.setWidget(w)
        self.addDockWidget(Qt.RightDockWidgetArea, dock)
        self._refresh_ports()

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
        self.reader = lv.SerialReader(port, lv.BAUD, self.parser)
        if self.reader.start():
            self.cn.setText("断开")
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

    def _capture_frame1(self):
        raw = self._capture_revolution()
        if raw:
            self.frame1 = front_points(raw)
            self.st.setText(f"第1帧：{len(self.frame1)} 点（前向）")
        else:
            self.st.setText("没采到数据，先连接雷达")

    def _capture_frame2(self):
        raw = self._capture_revolution()
        if raw:
            self.frame2 = front_points(raw)
            self.st.setText(f"第2帧：{len(self.frame2)} 点（前向）")
        else:
            self.st.setText("没采到数据，先连接雷达")

    # ---------------- 合并 + 检测 + 规划 ----------------
    def _merge_analyze(self):
        if not self.frame1 or not self.frame2:
            self.st.setText("请先采集两帧")
            return

        self._go()                       # 按左侧选择 S1/S2 落位
        xc, yc = self.robot
        h1 = float(self.hdg_spin.value())
        rot = float(self.rot_spin.value())
        off = float(self.off_spin.value())
        h2 = h1 + rot

        w1 = radar_to_world(self.frame1, xc, yc, h1, off)
        w2 = radar_to_world(self.frame2, xc, yc, h2, off)
        self.merged_cloud = w1 + w2

        self.obstacles = detect_obstacles(self.merged_cloud)

        print(f"[合并] 第1帧 {len(self.frame1)} 点, 第2帧 {len(self.frame2)} 点, "
              f"合并 {len(self.merged_cloud)} 点, 检测到 {len(self.obstacles)} 个障碍")
        for i, (ox, oy) in enumerate(self.obstacles, 1):
            print(f"   障碍{i}: ({ox:.0f}, {oy:.0f}) mm")

        self.st.setText(f"障碍 {len(self.obstacles)} 个 · 车头 {h1}° / 转角 {rot}°")
        self.robot_st.setText(f"状态: 检测到 {len(self.obstacles)} 个障碍，规划默认任务")
        self._default_task()             # 用检测到的障碍跑 A*，画出路径

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
        for i, (ox, oy) in enumerate(getattr(self, "obstacles", []), 1):
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

    win.frame1 = front_points(to_polar(world, 180.0))
    win.frame2 = front_points(to_polar(world, 270.0))
    win._merge_analyze()

    # ---- 核对 ----
    det = win.obstacles
    print(f"[自测] 真值障碍 {len(true_obs)} 个，检出 {len(det)} 个")
    matched = 0
    for tx, ty in true_obs:
        hit = any(math.hypot(dx - tx, dy - ty) < 120 for dx, dy in det)
        matched += 1 if hit else 0
        print(f"  真值({tx:>4},{ty:>4}) → {'✅ 检出' if hit else '❌ 漏检'}")
    false_pos = [(round(dx), round(dy)) for dx, dy in det
                 if not any(math.hypot(dx - tx, dy - ty) < 120 for tx, ty in true_obs)]
    print(f"[自测] 匹配 {matched}/{len(true_obs)}，误检 {len(false_pos)} {false_pos}")

    win._draw_map()
    out = "selftest_merge.png"
    win.figure.savefig(out, dpi=120)
    print(f"[自测] 地图已保存: {out}")
    return 0 if (matched == len(true_obs) and not false_pos) else 1


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

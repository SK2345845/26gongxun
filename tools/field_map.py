# -*- coding: utf-8 -*-
"""
搬运机器人比赛场地地图 + 连通图 A* 导航（初版）

地图规则：
  - 灰色 = 可踏入（整个场地）
  - 黄色 450×450 = 禁止踏入
  - 暂存区 / 粗加工区 长方形 = 禁止踏入（机器人在其朝向十字中心一侧的「接近点」停下）
  - 原料区轮盘 = 不撞上，停在其下方的接近点

导航（体积感知版，连通图 + A*）：
  - 节点 = 九宫格接近点；边 = 九宫格相邻节点之间的直线路段
  - 机器人有实际体积（300×300）：障碍贴边时路段不封锁（机器人擦边过），
    但代价升高 → A* 优先走宽敞路，实在没路才贴边挤
  - 路段真被堵死 → 在障碍两侧自动生成绕行途经点（绿点），贴着障碍局部绕行
  - 完全无路 → 该路段封锁（红），A* 换路或报告无路径
  - 路径点任务队列：启停区→二维码板→原料区→粗加工区→暂存区→原料区→粗加工区→暂存区→启停区

交互：
  - 左键放障碍 / 右键删障碍 / 随机放 / 清空 / 保存 / 加载
  - 发车(选启停区) → 目标区去 / 返回 / 默认任务
"""

import json
import math
import sys

import numpy as np
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout,
                             QHBoxLayout, QPushButton, QLabel, QStatusBar,
                             QComboBox)
from PyQt5.QtCore import Qt, QTimer

from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
from matplotlib.patches import Rectangle, Circle, Wedge
import matplotlib

matplotlib.rcParams['font.sans-serif'] = ['Microsoft YaHei', 'SimHei', 'SimSun']
matplotlib.rcParams['axes.unicode_minus'] = False


# ==================== 场地参数 ====================
FIELD_SIZE = 2400
ROAD_WIDTH = 400
YELLOW_SIZE = 450
OBSTACLE_DIAMETER = 50          # 障碍物 φ50
ROBOT_SIZE = 300                # 机器人 300×300
ROBOT_HALF = ROBOT_SIZE / 2     # 150
OBSTACLE_R = OBSTACLE_DIAMETER / 2  # 25
CLEAR_RECT = ROBOT_HALF                     # 150：距禁区边的安全距离
# ---- 体积感知导航参数 ----
SAFETY_MARGIN = 10            # 贴边通过时保留的最后余量
COMFORT_CLEAR = ROBOT_HALF + OBSTACLE_R + 90   # 265：净空低于此视为"拥挤"，加软代价
SOFT_PENALTY = 0.9            # 拥挤路段的代价放大上限
VIA_STEP = 20                 # 绕障途经点的搜索步长
VIA_EXTRA_MAX = 140           # 绕障点在紧贴距离基础上的最大外扩
TURNTABLE_R = 150

CX = CY = FIELD_SIZE / 2        # 1200
HALF = ROAD_WIDTH / 2           # 200

# 4 个黄色禁止区
YELLOW = [
    (CX - HALF - YELLOW_SIZE, CY + HALF),                  # 左上
    (CX + HALF, CY + HALF),                                # 右上
    (CX - HALF - YELLOW_SIZE, CY - HALF - YELLOW_SIZE),     # 左下
    (CX + HALF, CY - HALF - YELLOW_SIZE),                   # 右下
]

# 暂存区 / 粗加工区 矩形（禁止踏入）
ZC_RECT = (0, 910, 150, 1490)          # 左T，贴左边界
CG_RECT = (910, 0, 1490, 150)          # 下T，贴下边界

# 原料区轮盘（圆心在地图外，半径150）
TURNTABLE_C = (CX, FIELD_SIZE + 50)    # (1200, 2450)

# 功能区（画图用）
AREAS = [
    ("启停区一", 2100, 2100, 300, 300, "#3B82F6"),
    ("启停区二", 2100, 0, 300, 300, "#3B82F6"),
    ("暂存区", 0, 910, 150, 580, "#10B981"),
    ("粗加工区", 910, 0, 580, 150, "#EF4444"),
]

# ==================== 导航图 ====================
# 节点：九宫格接近点
NODE_NAME = {
    "LT": "左上通道", "YL": "原料区", "S1": "启停区一",
    "ZC": "暂存区", "C": "十字中心", "QR": "二维码板",
    "LB": "左下通道", "CG": "粗加工区", "S2": "启停区二",
}
NODES = {
    "LT": (350, 2050),
    "YL": (1200, 2050),
    "S1": (2050, 2050),
    "ZC": (350, 1200),
    "C":  (1200, 1200),
    "QR": (2050, 1200),
    "LB": (350, 350),
    "CG": (1200, 350),
    "S2": (2050, 350),
}
# 边（九宫格相邻节点）
EDGES = [
    ("LT", "YL"), ("YL", "S1"),
    ("ZC", "C"), ("C", "QR"),
    ("LB", "CG"), ("CG", "S2"),
    ("LT", "ZC"), ("ZC", "LB"),
    ("YL", "C"), ("C", "CG"),
    ("S1", "QR"), ("QR", "S2"),
]

# 默认任务路径点（启停区→二维码板→原料→粗加工→暂存→原料→粗加工→暂存→启停）
def default_task(start_node):
    return ["QR", "YL", "CG", "ZC", "YL", "CG", "ZC", start_node]


# ==================== 几何工具 ====================
def pt_seg_dist(px, py, ax, ay, bx, by):
    """点到线段的最短距离"""
    vx, vy = bx - ax, by - ay
    wx, wy = px - ax, py - ay
    L2 = vx * vx + vy * vy
    if L2 == 0:
        return math.hypot(px - ax, py - ay)
    t = max(0.0, min(1.0, (wx * vx + wy * vy) / L2))
    return math.hypot(px - (ax + t * vx), py - (ay + t * vy))


def pt_rect_dist(px, py, x0, y0, x1, y1):
    """点到矩形的最短距离（在矩形内则 0）"""
    dx = max(x0 - px, 0, px - x1)
    dy = max(y0 - py, 0, py - y1)
    return math.hypot(dx, dy)


def zone_point_clear(px, py):
    """单点是否满足对静态禁区（黄区/暂存/粗加工/轮盘）的安全距离"""
    for (rx, ry) in YELLOW:
        if pt_rect_dist(px, py, rx, ry, rx + YELLOW_SIZE, ry + YELLOW_SIZE) < CLEAR_RECT:
            return False
    if pt_rect_dist(px, py, *ZC_RECT) < CLEAR_RECT:
        return False
    if pt_rect_dist(px, py, *CG_RECT) < CLEAR_RECT:
        return False
    if math.hypot(px - TURNTABLE_C[0], py - TURNTABLE_C[1]) < CLEAR_RECT + TURNTABLE_R:
        return False
    return True


def obstacle_spot_ok(x, y):
    """障碍摆放点合法：不出界，且障碍圆不压在静态禁区/轮盘上"""
    if not (OBSTACLE_R <= x <= FIELD_SIZE - OBSTACLE_R and
            OBSTACLE_R <= y <= FIELD_SIZE - OBSTACLE_R):
        return False
    for (rx, ry) in YELLOW:
        if pt_rect_dist(x, y, rx, ry, rx + YELLOW_SIZE, ry + YELLOW_SIZE) < OBSTACLE_R:
            return False
    if pt_rect_dist(x, y, *ZC_RECT) < OBSTACLE_R:
        return False
    if pt_rect_dist(x, y, *CG_RECT) < OBSTACLE_R:
        return False
    if math.hypot(x - TURNTABLE_C[0], y - TURNTABLE_C[1]) < TURNTABLE_R + OBSTACLE_R:
        return False
    return True


def obstacle_fit_clear(ax, ay, bx, by, obstacles):
    """机器人中心沿路段 (a,b) 行驶时，与最近障碍表面之间的最小净空。
    用点到线段的精确距离，不采样——障碍贴边还是挡路一目了然。"""
    worst = float("inf")
    for (ox, oy) in obstacles:
        d = pt_seg_dist(ox, oy, ax, ay, bx, by) - OBSTACLE_R
        if d < worst:
            worst = d
    return worst


def segment_clear(ax, ay, bx, by, obstacles):
    """路段可通行判定（考虑机器人实际体积）：
    - 静态禁区是"墙"：沿线采样，任意点距禁区边缘 < CLEAR_RECT → 不可通行
    - 障碍物看净空：路段线到障碍中心距离 ≥ ROBOT_HALF+OBSTACLE_R+SAFETY_MARGIN
      （即障碍表面距机器人行驶线 ≥ 半车宽+余量）
      → 障碍贴边时不封锁（机器人擦边通过），只有真正挡住去路才封锁"""
    length = math.hypot(bx - ax, by - ay)
    steps = max(2, int(length / 30))
    for i in range(steps + 1):
        t = i / steps
        if not zone_point_clear(ax + (bx - ax) * t, ay + (by - ay) * t):
            return False
    return obstacle_fit_clear(ax, ay, bx, by, obstacles) >= ROBOT_HALF + SAFETY_MARGIN


def edge_cost(ax, ay, bx, by, obstacles):
    """边代价 = 几何长度 × 拥挤系数。
    净空充裕 → 1.0；越贴边越贵（上限 1+SOFT_PENALTY）。
    A* 自然优先走宽敞路，实在没路才贴边挤。不可通行返回 None。"""
    clear = obstacle_fit_clear(ax, ay, bx, by, obstacles)
    if clear < ROBOT_HALF + SAFETY_MARGIN:
        return None
    if not segment_clear(ax, ay, bx, by, obstacles):
        return None
    d = math.hypot(bx - ax, by - ay)
    if obstacles and clear < COMFORT_CLEAR:
        d *= 1.0 + SOFT_PENALTY * (COMFORT_CLEAR - clear) / COMFORT_CLEAR
    return d


def _make_via_nodes(a, b, obstacles, coords, adj, via_edges):
    """基础边被障碍挡死时，尝试在障碍两侧生成绕行途经点：
    途经点距障碍中心 ≥ 紧贴通过距离(need)，且与两端的子路段均可通行。
    每侧取总代价最小的偏移。成功则插入临时节点（名字以 '~' 开头），
    A* 就能贴着障碍绕过去，而不是整段封锁被迫绕九宫格。
    注意：偏移从【障碍中心】起算而非路段线——障碍不在正中线时，
    擦边通道往往就在原线路附近偏移一点点。"""
    xa, ya = NODES[a]; xb, yb = NODES[b]
    need = ROBOT_HALF + OBSTACLE_R + SAFETY_MARGIN
    # 找出挡得最狠的障碍（距路段线最近者），并记下它的带符号偏移
    blocker, best, d_signed = None, float("inf"), 0.0
    vx, vy = xb - xa, yb - ya
    L2 = vx * vx + vy * vy
    if L2 == 0:
        return False
    nx, ny = -vy / math.sqrt(L2), vx / math.sqrt(L2)   # 单位法向
    for (ox, oy) in obstacles:
        dseg = pt_seg_dist(ox, oy, xa, ya, xb, yb)
        if dseg < best:
            best = dseg
            blocker = (ox, oy)
            d_signed = (ox - xa) * nx + (oy - ya) * ny
    if blocker is None or best >= need:
        return False
    ox, oy = blocker
    t = max(0.08, min(0.92, ((ox - xa) * vx + (oy - ya) * vy) / L2))
    px, py = xa + t * vx, ya + t * vy          # 障碍在路段上的投影点
    ok_sides = 0
    for side in (+1, -1):
        best_pick = None
        # 候选点：从障碍中心沿法向量 need + k*VIA_STEP 处
        for k in range(VIA_EXTRA_MAX // VIA_STEP + 1):
            off = d_signed + side * (need + k * VIA_STEP)
            cx, cy = px + off * nx, py + off * ny
            if not (ROBOT_HALF < cx < FIELD_SIZE - ROBOT_HALF and
                    ROBOT_HALF < cy < FIELD_SIZE - ROBOT_HALF):
                break                           # 该侧已出界，更远只会更差
            if not zone_point_clear(cx, cy):
                continue                        # 该偏移撞静态禁区，试着再远一点
            ca = edge_cost(xa, ya, cx, cy, obstacles)
            cb = edge_cost(cx, cy, xb, yb, obstacles)
            if ca is None or cb is None:
                continue
            if best_pick is None or ca + cb < best_pick[0]:
                best_pick = (ca + cb, cx, cy, ca, cb)
        if best_pick:
            _, cx, cy, ca, cb = best_pick
            name = f"~{a}~{b}~{side}"
            coords[name] = (cx, cy)
            adj[name] = [(a, ca), (b, cb)]
            adj[a].append((name, ca))
            adj[b].append((name, cb))
            via_edges.append(((xa, ya), (cx, cy)))
            via_edges.append(((cx, cy), (xb, yb)))
            ok_sides += 1
    return ok_sides > 0


def build_graph(obstacles):
    """构建连通图（体积感知 + 绕障途经点）。
    返回 (adj, coords, edge_status, via_edges)：
      adj         邻接表，可能含 '~' 开头的临时绕障节点
      coords      节点名 → 坐标（含临时节点）
      edge_status 基础边状态：open 畅通 / tight 紧贴可过 / via 局部绕障 / blocked 封锁
      via_edges   绕障子路段列表（画图用）"""
    adj = {n: [] for n in NODES}
    coords = dict(NODES)
    edge_status = {}
    via_edges = []
    for (a, b) in EDGES:
        xa, ya = NODES[a]; xb, yb = NODES[b]
        cost = edge_cost(xa, ya, xb, yb, obstacles)
        if cost is not None:
            adj[a].append((b, cost))
            adj[b].append((a, cost))
            clear = obstacle_fit_clear(xa, ya, xb, yb, obstacles)
            edge_status[(a, b)] = "tight" if clear < COMFORT_CLEAR else "open"
        elif _make_via_nodes(a, b, obstacles, coords, adj, via_edges):
            edge_status[(a, b)] = "via"
        else:
            edge_status[(a, b)] = "blocked"
    return adj, coords, edge_status, via_edges


def astar(start, goal, adj, coords):
    """A* 最短路径（代价含拥挤系数），返回节点名列表；无路径返回 []"""
    if start not in adj or goal not in adj:
        return []
    gx, gy = coords[goal]
    h = {n: math.hypot(gx - coords[n][0], gy - coords[n][1]) for n in adj}
    came = {}
    g = {start: 0.0}
    open_set = {start}
    while open_set:
        cur = min(open_set, key=lambda n: g[n] + h[n])
        if cur == goal:
            path = [cur]
            while cur in came:
                cur = came[cur]
                path.append(cur)
            return path[::-1]
        open_set.remove(cur)
        for (nb, d) in adj.get(cur, []):
            ng = g[cur] + d
            if ng < g.get(nb, float("inf")):
                came[nb] = cur
                g[nb] = ng
                open_set.add(nb)
    return []


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("比赛场地地图 + 体积感知A*导航")
        self.resize(900, 1050)
        self.obstacles = []
        self.start_node = "S1"
        self.robot = NODES["S1"]
        self.cur_node = "S1"
        self.path = []            # 当前规划路径（节点名，可能含绕障临时节点）
        self.task_queue = []      # 路径点任务队列
        self.node_xy = dict(NODES)   # 节点名→坐标（含绕障临时节点）
        self.edge_status = {}        # 基础边状态
        self.via_edges = []          # 绕障子路段（画图用）
        self._build()
        self._draw_map()
        self._anim = QTimer(self)
        self._anim.timeout.connect(self._move_step)
        self._anim.start(50)

    def _build(self):
        c = QWidget()
        self.setCentralWidget(c)
        root = QVBoxLayout(c)
        root.setContentsMargins(8, 8, 8, 8)
        root.setSpacing(6)

        # ---- 障碍工具栏 ----
        bar = QHBoxLayout()
        bar.addWidget(QLabel("灰色可踏入/黄色禁止"))
        bar.addStretch()
        b1 = QPushButton("随机放10个")
        b1.clicked.connect(lambda: self._random_obstacles(10))
        bar.addWidget(b1)
        b2 = QPushButton("清空障碍")
        b2.clicked.connect(self._clear_obstacles)
        bar.addWidget(b2)
        b3 = QPushButton("保存")
        b3.clicked.connect(self._save)
        bar.addWidget(b3)
        b4 = QPushButton("加载")
        b4.clicked.connect(self._load)
        bar.addWidget(b4)
        root.addLayout(bar)

        # ---- 机器人任务栏 ----
        rbar = QHBoxLayout()
        rbar.addWidget(QLabel("机器人:"))
        self.start_cb = QComboBox()
        self.start_cb.addItems(["启停区一", "启停区二"])
        rbar.addWidget(self.start_cb)
        go = QPushButton("发车")
        go.clicked.connect(self._go)
        rbar.addWidget(go)
        rbar.addSpacing(10)
        rbar.addWidget(QLabel("目标:"))
        self.goal_cb = QComboBox()
        self.goal_cb.addItems(["左上通道", "原料区", "启停区一", "暂存区", "十字中心", "二维码板", "左下通道", "粗加工区", "启停区二"])
        rbar.addWidget(self.goal_cb)
        tk = QPushButton("去")
        tk.clicked.connect(self._set_goal)
        rbar.addWidget(tk)
        bk = QPushButton("返回")
        bk.clicked.connect(self._go_back)
        rbar.addWidget(bk)
        dt = QPushButton("默认任务")
        dt.clicked.connect(self._default_task)
        rbar.addWidget(dt)
        self.robot_st = QLabel("状态: 就绪")
        rbar.addWidget(self.robot_st)
        rbar.addStretch()
        root.addLayout(rbar)

        self.figure = Figure(figsize=(8, 8), dpi=100)
        self.canvas = FigureCanvas(self.figure)
        self.ax = self.figure.add_subplot(111)
        root.addWidget(self.canvas, stretch=1)

        self.setStatusBar(QStatusBar())
        self.statusBar().showMessage("就绪 · 左键放障碍，右键点障碍删除")

        self.canvas.mpl_connect('button_press_event', self._on_click)

    # ============ 绘制 ============
    def _draw_map(self):
        self.ax.clear()
        self.ax.set_xlim(-150, FIELD_SIZE + 300)
        self.ax.set_ylim(-150, FIELD_SIZE + 300)
        self.ax.set_aspect("equal")
        self.ax.set_title("比赛场地 · 体积感知A*导航"
                          "（绿=路径 蓝=宽敞 橙=紧贴可过 灰虚=局部绕障 红=堵死）")
        self._refresh_graph()

        # 场地（灰色可踏入）
        self.ax.add_patch(Rectangle((0, 0), FIELD_SIZE, FIELD_SIZE,
                                    facecolor="#9CA3AF", edgecolor="black", lw=2))
        # 十字路（稍深灰）
        self.ax.add_patch(Rectangle((CX - HALF, 0), ROAD_WIDTH, FIELD_SIZE,
                                    facecolor="#6B7280", edgecolor="none"))
        self.ax.add_patch(Rectangle((0, CY - HALF), FIELD_SIZE, ROAD_WIDTH,
                                    facecolor="#6B7280", edgecolor="none"))

        # 黄色禁止区
        for (x, y) in YELLOW:
            self.ax.add_patch(Rectangle((x, y), YELLOW_SIZE, YELLOW_SIZE,
                                        facecolor="#FDE047", edgecolor="#D97706", lw=2))
            self.ax.text(x + YELLOW_SIZE / 2, y + YELLOW_SIZE / 2, "禁止",
                         ha="center", va="center", fontsize=10, color="#B45309", fontweight="bold")

        # 暂存/粗加工矩形（禁止踏入）
        for (x0, y0, x1, y1), name in [(ZC_RECT, "暂存区"), (CG_RECT, "粗加工区")]:
            self.ax.add_patch(Rectangle((x0, y0), x1 - x0, y1 - y0,
                                        facecolor="#F87171", edgecolor="#B91C1C", lw=2))
            self.ax.text((x0 + x1) / 2, (y0 + y1) / 2, name, ha="center", va="center",
                         fontsize=9, color="white", fontweight="bold")

        # 原料区轮盘
        tc = TURNTABLE_C
        self.ax.add_patch(Circle(tc, TURNTABLE_R, facecolor="#F59E0B",
                                 edgecolor="black", lw=2, alpha=0.7))
        self.ax.text(tc[0], tc[1], "原料区", ha="center", va="center",
                     fontsize=9, color="white", fontweight="bold")

        # 二维码板
        self.ax.add_patch(Rectangle((2460, 1160), 60, 80, facecolor="#111827",
                                    edgecolor="black", lw=1))
        self.ax.text(2490, 1260, "二维码板", ha="center", fontsize=8)

        # 功能区
        for name, x, y, w, h, color in AREAS:
            self.ax.add_patch(Rectangle((x, y), w, h, facecolor=color,
                                        edgecolor="black", lw=1.5, alpha=0.6))
            self.ax.text(x + w / 2, y + h / 2, name, ha="center", va="center",
                         fontsize=8, color="white", fontweight="bold")

        # 连通图：按体积感知判定结果着色
        for (a, b) in EDGES:
            xa, ya = NODES[a]; xb, yb = NODES[b]
            st = self.edge_status.get((a, b), "open")
            color = {"open": "#1D4ED8", "tight": "#F59E0B",
                     "via": "#9CA3AF", "blocked": "#EF4444"}.get(st, "#1D4ED8")
            style = "--" if st in ("via", "blocked") else "-"
            self.ax.plot([xa, xb], [ya, yb], color=color, lw=1, alpha=0.7, ls=style)

        # 绕障子路段与途经点（绿虚线/绿点）
        for (p1, p2) in self.via_edges:
            self.ax.plot([p1[0], p2[0]], [p1[1], p2[1]],
                         color="#16A34A", lw=1.2, ls="--", alpha=0.8)
        for name, (x, y) in self.node_xy.items():
            if name.startswith("~"):
                self.ax.plot(x, y, "o", ms=4, color="#16A34A")

        # 规划路径（绿）
        if len(self.path) >= 2:
            xs = [self.node_xy[n][0] for n in self.path]
            ys = [self.node_xy[n][1] for n in self.path]
            self.ax.plot(xs, ys, 'g-', lw=3)

        # 障碍物
        for (x, y) in self.obstacles:
            self.ax.add_patch(Circle((x, y), OBSTACLE_R,
                                     facecolor="#111827", edgecolor="black", lw=1))

        # 节点
        for n, (x, y) in NODES.items():
            self.ax.plot(x, y, 'k+', ms=7)
            self.ax.text(x + 18, y + 18, n, fontsize=7, color="#111827",
                         ha="left", va="bottom", fontweight="bold")

        # 机器人
        rx, ry = self.robot
        self.ax.add_patch(Rectangle((rx - ROBOT_HALF, ry - ROBOT_HALF),
                                    ROBOT_SIZE, ROBOT_SIZE,
                                    facecolor="#1D4ED8", edgecolor="black", lw=2))
        self.ax.text(rx, ry, "R", ha="center", va="center",
                     fontsize=16, color="white", fontweight="bold")

        self.canvas.draw_idle()

    # ============ 路径规划 ============
    def _refresh_graph(self):
        """按当前障碍重算连通图与边状态（体积感知 + 绕障途经点）"""
        adj, coords, status, via = build_graph(self.obstacles)
        self.node_xy = coords
        self.edge_status = status
        self.via_edges = via
        return adj

    def _plan(self, goal_node):
        """从当前节点规划到目标节点，返回是否成功"""
        adj = self._refresh_graph()
        self.path = astar(self.cur_node, goal_node, adj, self.node_xy)
        if not self.path:
            self.robot_st.setText(f"状态: 到{NODE_NAME[goal_node]} 无路径(去路被堵死)")
            return False
        # 去掉起点自身
        if len(self.path) >= 2 and self.path[0] == self.cur_node:
            self.path = self.path[1:]
        self.robot_st.setText(f"状态: 去 {NODE_NAME[goal_node]}")
        self._draw_map()
        return True

    def _next_task(self):
        """取下一个任务点，规划路径"""
        if not self.task_queue:
            self.robot_st.setText("状态: 任务完成")
            return
        goal = self.task_queue.pop(0)
        if not self._plan(goal):
            self.task_queue = []   # 路径阻断，停下
            return

    # ============ 机器人 ============
    def _go(self):
        start = self.start_cb.currentText()
        self.start_node = "S1" if start == "启停区一" else "S2"
        self.robot = NODES[self.start_node]
        self.cur_node = self.start_node
        self.path = []
        self.task_queue = []
        self.robot_st.setText(f"状态: 已发车 {start}")
        self._draw_map()

    def _set_goal(self):
        goal = self.goal_cb.currentText()
        gn = {"左上通道": "LT", "原料区": "YL", "启停区一": "S1",
              "暂存区": "ZC", "十字中心": "C", "二维码板": "QR",
              "左下通道": "LB", "粗加工区": "CG", "启停区二": "S2"}[goal]
        self.task_queue = [gn]
        self._next_task()

    def _go_back(self):
        self.task_queue = [self.start_node]
        self._next_task()

    def _default_task(self):
        self.task_queue = default_task(self.start_node)
        self.robot_st.setText("状态: 默认任务开始")
        self._next_task()

    def _move_step(self):
        """沿规划路径移动（节点/绕障点之间直线）"""
        if not self.path:
            return
        nx, ny = self.node_xy[self.path[0]]
        rx, ry = self.robot
        dx, dy = nx - rx, ny - ry
        dist = math.hypot(dx, dy)
        step = 40
        if dist <= step:
            # 最后一段直接吸附到节点，避免跨过目标点后反复抖动
            self.robot = (nx, ny)
            self.cur_node = self.path.pop(0)
            if not self.path:
                self._next_task()
            self._draw_map()
            return
        self.robot = (rx + dx / dist * step, ry + dy / dist * step)
        self._draw_map()

    # ============ 障碍交互 ============
    def _on_click(self, event):
        if event.inaxes != self.ax or event.xdata is None or event.ydata is None:
            return
        x, y = event.xdata, event.ydata
        if not (0 <= x <= FIELD_SIZE and 0 <= y <= FIELD_SIZE):
            return
        if event.button == 1:
            if not obstacle_spot_ok(x, y):
                self.statusBar().showMessage("⚠ 障碍不能压在禁区/轮盘上或出界", 2500)
            elif not any(math.hypot(x - ox, y - oy) < OBSTACLE_DIAMETER for ox, oy in self.obstacles):
                self.obstacles.append((x, y))
                self.statusBar().showMessage(f"放置障碍 @ ({x:.0f}, {y:.0f})", 2000)
                # 障碍变化 → 重新规划当前任务
                if self.path:
                    self._next_task()
        elif event.button == 3:
            for i, (ox, oy) in enumerate(self.obstacles):
                if math.hypot(x - ox, y - oy) < OBSTACLE_R:
                    self.obstacles.pop(i)
                    self.statusBar().showMessage("删除障碍", 2000)
                    if self.path:
                        self._next_task()
                    break
        self._draw_map()

    def _random_obstacles(self, n):
        self.obstacles = []
        tries = 0
        while len(self.obstacles) < n and tries < 500:
            tries += 1
            x = np.random.uniform(50, FIELD_SIZE - 50)
            y = np.random.uniform(50, FIELD_SIZE - 50)
            if (obstacle_spot_ok(x, y) and
                    not any(math.hypot(x - ox, y - oy) < OBSTACLE_DIAMETER * 2
                            for ox, oy in self.obstacles)):
                self.obstacles.append((x, y))
        if self.path:
            self._next_task()
        self._draw_map()
        self.statusBar().showMessage(f"随机放置 {n} 个障碍", 2000)

    def _clear_obstacles(self):
        self.obstacles = []
        if self.path:
            self._next_task()
        self._draw_map()
        self.statusBar().showMessage("已清空障碍", 2000)

    def _save(self):
        with open("field_map.json", "w", encoding="utf-8") as f:
            json.dump({"obstacles": self.obstacles}, f, ensure_ascii=False, indent=2)
        self.statusBar().showMessage("已保存到 field_map.json", 3000)

    def _load(self):
        try:
            with open("field_map.json", "r", encoding="utf-8") as f:
                data = json.load(f)
            self.obstacles = data.get("obstacles", [])
            if self.path:
                self._next_task()
            self._draw_map()
            self.statusBar().showMessage(f"已加载 {len(self.obstacles)} 个障碍", 2000)
        except Exception as e:
            self.statusBar().showMessage(f"加载失败: {e}", 3000)


def main():
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    win = MainWindow()
    win.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()

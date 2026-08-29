# -*- coding: utf-8 -*-
"""field_map 体积感知导航冒烟测试（离屏运行）
用法：QT_QPA_PLATFORM=offscreen python3 test_field_map.py
"""
import os, sys, math
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import field_map as fm

PASS, FAIL = [], []
def check(name, cond, info=""):
    (PASS if cond else FAIL).append(name)
    print(("✅" if cond else "❌"), name, info)

# ---------- 纯函数级测试 ----------
need = fm.ROBOT_HALF + fm.OBSTACLE_R + fm.SAFETY_MARGIN
check("紧贴通过距离 = 185", need == 185, f"need={need}")

# 水平边 ZC-C
ax, ay = fm.NODES["ZC"]; bx, by = fm.NODES["C"]
print("ZC =", (ax, ay), " C =", (bx, by))
L = math.hypot(bx - ax, by - ay)
ux, uy = (bx - ax) / L, (by - ay) / L
mx, my = (ax + bx) / 2, (ay + by) / 2

# 贴边障碍：路段中点法向偏移 175（贴着黄区边缘能放下的极限位置，
# 距路段线 < 紧贴通过距离185 → 直线被挡，但车擦边绕一下就能过）
n1 = (-uy, ux)
cand1 = (mx + 175 * n1[0], my + 175 * n1[1])
cand2 = (mx - 175 * n1[0], my - 175 * n1[1])
def towards_center(c):
    return math.hypot(c[0] - fm.CX, c[1] - fm.CY) < math.hypot(mx - fm.CX, my - fm.CY)
ox, oy = cand1 if towards_center(cand1) else cand2
print("贴边障碍位置:", (ox, oy))
check("贴边障碍摆放合法", fm.obstacle_spot_ok(ox, oy))

adj, coords, status, via = fm.build_graph([(ox, oy)])
st = status.get(("ZC", "C"))
check("贴边障碍→边可经绕行点通过", st == "via", f"status={st}")
check("生成绕障途经点", any(n.startswith("~") for n in coords))
for n, (x, y) in coords.items():
    if n.startswith("~"):
        # 新语义：车体方形放在绕障点上，扫掠面距障碍表面 ≥ SAFETY_MARGIN
        d = fm.swept_point_dist(x, y, x, y, ox, oy) - fm.OBSTACLE_R
        check(f"绕障点{n}扫掠面净空≥{fm.SAFETY_MARGIN:.0f}",
              d >= fm.SAFETY_MARGIN - 1e-9, f"clear={d:.0f}")

# 场景B：障碍正挡在路段中点 → 边必须 via 或 blocked
adj2, coords2, status2, via2 = fm.build_graph([(mx, my)])
st2 = status2.get(("ZC", "C"))
check("正中障碍使边 via 或 blocked", st2 in ("via", "blocked"), f"status={st2}")
if st2 == "via":
    check("生成绕障途经点", any(n.startswith("~") for n in coords2))
    for n, (x, y) in coords2.items():
        if n.startswith("~"):
            d = math.hypot(x - mx, y - my) - fm.OBSTACLE_R
            check(f"绕障点{n}净空≥机器人半径", d >= fm.ROBOT_HALF - 1e-9, f"clear={d:.0f}")

# 场景E：斜向路段撞角——圆盘模型的盲区（用户实车反馈）
# 45° 路段，障碍距行驶线垂直距离 200mm > 旧圆盘阈值 185：
# 旧模型判定"能过"，但方形车的角实际扫进障碍 → 新模型必须识别
dax, day, dbx, dby = 600, 1000, 900, 1300
corner_ob = (1000, 1117)
old_disc_clear = abs((corner_ob[0]-dax)*(dby-day) - (corner_ob[1]-day)*(dbx-dax)) / \
                 math.hypot(dbx-dax, dby-day) - fm.OBSTACLE_R
check("撞角场景：旧圆盘模型确实判'能过'(证明盲区存在)",
      old_disc_clear >= fm.ROBOT_HALF + fm.SAFETY_MARGIN, f"圆盘净空={old_disc_clear:.0f}")
corner_clear = fm.obstacle_fit_clear(dax, day, dbx, dby, [corner_ob])
check("撞角场景：扫掠模型正确判'撞'", corner_clear < fm.SAFETY_MARGIN,
      f"扫掠净空={corner_clear:.0f}")
# 对照：障碍离得足够远时扫掠模型不误伤
far_ob = (1250, 1150)
far_clear = fm.obstacle_fit_clear(dax, day, dbx, dby, [far_ob])
check("对照：远处障碍扫掠净空充足", far_clear >= fm.SAFETY_MARGIN, f"净空={far_clear:.0f}")

# 场景A：无障碍 → 全部 open，S1→QR 有路径
adj0, coords0, status0, via0 = fm.build_graph([])
check("无障碍时全部畅通", all(v == "open" for v in status0.values()))
p = fm.astar("S1", "QR", adj0, coords0)
check("无障碍时 S1→QR 有路径", len(p) >= 2, f"path={p}")

def path_valid(path, coords, obstacles):
    for i in range(len(path) - 1):
        a, b = coords[path[i]], coords[path[i + 1]]
        if not fm.segment_clear(a[0], a[1], b[0], b[1], obstacles):
            return False, (path[i], path[i + 1])
    return True, None

# 场景D：随机障碍下，所有成功规划的路径必须逐段可通行
import random
random.seed(42)
bad = 0
for trial in range(30):
    obs = []
    for _ in range(12):
        for _t in range(50):
            x = random.uniform(60, fm.FIELD_SIZE - 60)
            y = random.uniform(60, fm.FIELD_SIZE - 60)
            if fm.obstacle_spot_ok(x, y) and all(math.hypot(x-a, y-b) >= 100 for a, b in obs):
                obs.append((x, y)); break
    adjd, cd, _, _ = fm.build_graph(obs)
    for s in ("S1", "S2"):
        for g in ("YL", "ZC", "CG", "QR", "C"):
            path = fm.astar(s, g, adjd, cd)
            if path:
                ok, seg = path_valid(path, cd, obs)
                if not ok:
                    bad += 1
                    print("  非法路径段:", s, "->", g, seg)
check("随机规划路径全部合法", bad == 0, f"非法数={bad}")

# ---------- GUI 级冒烟（离屏实例化 + 规划 + 动画推进）----------
from PyQt5.QtWidgets import QApplication
app = QApplication.instance() or QApplication(sys.argv)
win = fm.MainWindow()
win.obstacles = [(mx, my)]
win.task_queue = ["YL"]
win._next_task()
check("GUI: 被堵时能重新规划出路径", len(win.path) >= 1, f"path={win.path}")
for i in range(600):
    win._move_step()
    if not win.path:
        break
check("GUI: 动画走完整段路径", not win.path, f"steps={i}")
check("GUI: 机器人未穿轮盘禁区",
      math.hypot(win.robot[0] - fm.TURNTABLE_C[0], win.robot[1] - fm.TURNTABLE_C[1])
      >= fm.TURNTABLE_R + fm.ROBOT_HALF - 5)

print(f"\n结果: {len(PASS)} 通过 / {len(FAIL)} 失败")
sys.exit(1 if FAIL else 0)

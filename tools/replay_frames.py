# -*- coding: utf-8 -*-
"""离线重放用户保存的帧文件，对比不同位姿假设下的障碍检出"""
import json, math, os, sys
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import lidar_scan_merge as lm

def analyze(path, p1, p2):
    d = json.load(open(path, encoding='utf-8'))
    f1 = [tuple(p) for p in d['frame1']]
    f2 = [tuple(p) for p in d['frame2']]
    fov, rng = d['fov_deg'], d['max_range_mm']
    c1 = lm.crop_points(f1, fov/2, rng)
    c2 = lm.crop_points(f2, fov/2, rng)
    b1, b2 = lm.MergeWindow._pose_base(p1), lm.MergeWindow._pose_base(p2)
    w1 = lm.radar_to_world(c1, b1[0], b1[1], p1['hdg'], p1['off'])
    w2 = lm.radar_to_world(c2, b2[0], b2[1], p2['hdg'], p2['off'])
    obs = lm.detect_obstacles(w1 + w2)
    print(f"== {os.path.basename(path)}  p1={p1['start']}/{p1['hdg']}°  p2={p2['start']}/{p2['hdg']}°")
    for cx, cy in obs:
        n1 = sum(1 for x,y in w1 if math.hypot(x-cx,y-cy) < 120)
        n2 = sum(1 for x,y in w2 if math.hypot(x-cx,y-cy) < 120)
        tag = "双视角" if n1>=2 and n2>=2 else "仅单视角"
        print(f"   ({cx:6.0f},{cy:6.0f})  [{tag}: 视角1 {n1} / 视角2 {n2}]")
    print()

BASE = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '雷达帧')

# 15:53 文件：在启停区一采的。frame1 车头180；frame2 按当时设计=原地转90° → 270
analyze(os.path.join(BASE, 'lidar_frames_20260830_155344.json'),
        {'start':'启停区一','hdg':180,'off':150},
        {'start':'启停区一','hdg':270,'off':150})

# 16:26 文件：pose 记录=两帧都是 启停区二/180°，按原样重放
analyze(os.path.join(BASE, 'lidar_frames_20260830_162622.json'),
        {'start':'启停区二','hdg':180,'off':150},
        {'start':'启停区二','hdg':180,'off':150})

# 16:26 文件：假设第二帧车头实际是 270°（用户可能转了车但没改GUI）
analyze(os.path.join(BASE, 'lidar_frames_20260830_162622.json'),
        {'start':'启停区二','hdg':180,'off':150},
        {'start':'启停区二','hdg':270,'off':150})

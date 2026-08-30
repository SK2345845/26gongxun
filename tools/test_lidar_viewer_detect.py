# -*- coding: utf-8 -*-
"""lidar_viewer 自动搜索判别的离线回归测试（无需雷达）。

合成 8 圈点云喂假 reader：
  - 两个 φ50 真圆柱（复刻实测截图：915mm@328°、1255mm@5°）
  - 一段 60°~75°、3100mm 的横向墙碎片（旧算法会误报成障碍）
  - 远处 3357mm@348° 的偶发毛刺（只在第 2/5 圈出现）
验证：墙碎片和毛刺被滤掉，两个圆柱高分检出。退出码 0=通过。
用法：QT_QPA_PLATFORM=offscreen python3 test_lidar_viewer_detect.py
"""
import math
import os
import sys

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import lidar_viewer as lv
from PyQt5.QtWidgets import QApplication


def cyl(a_deg, d_mm, n=3, spread_deg=0.8, jitter=8):
    """φ50 圆柱：n 个点挤在零点几度里"""
    pts = []
    for i in range(n):
        a = a_deg + (i - (n - 1) / 2.0) * (spread_deg / max(n - 1, 1))
        pts.append((a % 360.0, d_mm + (i % 3 - 1) * jitter))
    return pts


def wall(a0, a1, d_mm, step_deg=0.5):
    """横向墙面：一长串近等距点（外接尺寸几百 mm）"""
    angs = [a0 + i * step_deg for i in range(int((a1 - a0) / step_deg) + 1)]
    return [(a % 360.0, d_mm + 6.0 * math.sin(a * 0.7)) for a in angs]


class FakeReader:
    running = True
    lost = False
    lost_reason = ""

    def __init__(self, frames):
        self.frames = frames
        self.i = 0

    def get_latest(self):
        f = self.frames[min(self.i, len(self.frames) - 1)]
        self.i += 1
        return f


def test_parser():
    """调速应答描述符不能破坏扫描解析（点云全无 bug 的回归测试）"""
    p = lv.LidarParser()
    # 伪造字节流：调速应答(7字节,类型0xA8) + 扫描应答头(类型0x81) + 3个测距节点
    # 节点格式: [S|~S|quality<<2, angle_l, angle_h, dist_l, dist_h]
    # 角度16位 = (度*64)<<1（最低位是校验位S）；距离16位 = mm*4
    stream = bytes([
        0xA5, 0x5A, 0x00, 0x00, 0x00, 0xA8, 0x85,   # 调速应答——必须被丢弃
        0xA5, 0x5A, 0x05, 0x00, 0x00, 0x40, 0x81,   # 扫描应答头
        0xF1, 0x80, 0x16, 0xA0, 0x0F,               # S=1 新圈: 45°, 1000mm
        0x02, 0x00, 0x2D, 0xD0, 0x07,               # S=0:      90°, 500mm
        0xF1, 0x80, 0x43, 0xE8, 0x03,               # S=1: 上一圈完成, 135°, 250mm
    ])
    p.feed(stream)
    rev = p.take_completed()
    ok = rev == [(45.0, 1000.0), (90.0, 500.0)]
    print(f"[测试] 解析器: {'OK ' if ok else 'X '}{rev}")
    return 0 if ok else 1


def main():
    rc = test_parser()
    app = QApplication(sys.argv)
    w = lv.MainWindow()

    frames = []
    for k in range(8):
        rev = []
        # 真实角宽：φ50 圆柱在 915mm 处约 3°、1255mm 处约 2.3°（不能给太窄，会被 1° 分箱塌成单点）
        rev += cyl(5, 1255, n=3, spread_deg=2.2)    # 真障碍（截图#4：1255mm@5°）
        rev += cyl(328, 915, n=3, spread_deg=3.0)   # 真障碍（截图#2：915mm@328°）
        rev += wall(60, 75, 3100)                   # 墙碎片（旧算法误报源）
        if k in (1, 4):
            rev += cyl(348, 3357, n=2, spread_deg=1.6)  # 远处偶发毛刺（旧算法误报源）
        frames.append(rev)

    w.reader = FakeReader(frames)
    for _ in range(8):
        w._update_plot()

    obs = w._last_obstacles
    print(f"[测试] 检出 {len(obs)} 个障碍（期望 2 个）")
    for a, d, conf, n in obs:
        print(f"   {d:.0f} mm @ {a:.0f}°  置信 {conf}%  {n} 点")

    def near(d, a):
        return any(math.hypot(d - dd, 0) < 200 and min(abs(a - aa), 360 - abs(a - aa)) < 15
                   for aa, dd, _, _ in obs)

    ok = True
    if len(obs) != 2:
        ok = False
        print("[测试] X 数量不对")
    if not near(915, 328):
        ok = False
        print("[测试] X 漏检 915mm@328° 圆柱")
    if not near(1255, 5):
        ok = False
        print("[测试] X 漏检 1255mm@5° 圆柱")
    if near(3100, 67) or near(3357, 348):
        ok = False
        print("[测试] X 墙碎片或毛刺未被滤除")
    if all(conf < lv.MIN_CONFIDENCE for _, _, conf, _ in obs):
        ok = False
        print("[测试] X 置信度没起来")
    print("[测试] OK 通过" if ok else "[测试] X 失败")
    return 0 if (ok and rc == 0) else 1


if __name__ == "__main__":
    sys.exit(main())

"""장애물 회피 구간만 평가: 이륙 ~ 목표점 (map 5, 0) 0.3 m 이내 첫 도달
실제 형상 사용: 기둥 = 반지름 0.2 m 원기둥, 벽 = 상자. 드론 반폭 0.21 m. 충돌 판정: 표면 여유 < 0 또는 기울기 > 15°
"""
import os
import sys

import numpy as np
from pyulog import ULog

GOAL = (5.0, 0.0)
HALF = 0.21
PILLARS = [(1.0, 2.05), (1.0, -1.95), (2.0, 0.05), (4.0, -1.95), (4.0, 1.05)]  # map 좌표, r = 0.2
BOXES = {'holewall': (5.95, 6.05, -1.90, 2.00)}
if os.environ.get('PX4_SITL_WORLD') == 'smartfactory_slalom_wide':
    BOXES['slalom_wall_1'] = (1.60, 1.80, -2.30, 0.30)
    BOXES['slalom_wall_2'] = (3.50, 3.70, -0.30, 2.30)
if os.environ.get('PX4_SITL_WORLD') == 'smartfactory_slalom':
    BOXES['slalom_wall_1'] = (0.90, 1.10, -1.75, 0.55)
    BOXES['slalom_wall_2'] = (3.20, 3.40, -0.25, 2.65)

u = ULog(sys.argv[1], ['vehicle_local_position', 'vehicle_attitude'])
lp = u.get_dataset('vehicle_local_position').data
t, mx, my, z = lp['timestamp'] / 1e6, lp['y'], lp['x'], lp['z']
air = np.where(z < -0.5)[0]
if len(air) == 0:
    print('avoid reached=NO reason=no_takeoff')
    sys.exit()
i0 = air[0]
near = np.where(np.hypot(mx - GOAL[0], my - GOAL[1]) < 0.3)[0]
near = near[near > i0]
reached = len(near) > 0
i1 = near[0] if reached else len(t) - 1
w = slice(i0, i1 + 1)

clear = [np.hypot(mx[w] - px, my[w] - py) - 0.2 - HALF for px, py in PILLARS]
for b in BOXES.values():
    dx = np.maximum(np.maximum(b[0] - mx[w], 0), mx[w] - b[1])
    dy = np.maximum(np.maximum(b[2] - my[w], 0), my[w] - b[3])
    clear.append(np.hypot(dx, dy) - HALF)
min_clear = float(np.min([c.min() for c in clear]))

att = u.get_dataset('vehicle_attitude').data
ta = att['timestamp'] / 1e6
q0, q1, q2, q3 = (att[f'q[{k}]'] for k in range(4))
tilt = np.degrees(np.maximum(np.abs(np.arctan2(2 * (q0 * q1 + q2 * q3), 1 - 2 * (q1 ** 2 + q2 ** 2))),
                             np.abs(np.arcsin(np.clip(2 * (q0 * q2 - q3 * q1), -1, 1)))))
aw = (ta >= t[i0]) & (ta <= t[i1])
max_tilt = float(tilt[aw].max())

g = np.arange(t[i0], t[i1], 0.5)
dist = float(np.sum(np.hypot(np.diff(np.interp(g, t, mx)), np.diff(np.interp(g, t, my)))))
collision = min_clear < 0 or max_tilt > 15
print(f"avoid reached={'YES' if reached else 'NO'} avoid_s={t[i1] - t[i0]:.1f} avoid_dist_m={dist:.2f} "
      f"avoid_clear_m={min_clear:.3f} avoid_tilt_deg={max_tilt:.1f} collision={'YES' if collision else 'NO'}")

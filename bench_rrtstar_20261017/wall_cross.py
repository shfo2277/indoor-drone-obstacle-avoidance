"""ulog에서 드론이 벽 평면(NED y=6.0)을 지난 순간의 위치와 기울기를 출력. 기울기 > 15° 면 창틀 접촉으로 판정"""
import sys
import numpy as np
from pyulog import ULog
WALL_Y = 6.0
CONTACT_TILT_DEG = 15.0
for f in sys.argv[1:]:
    u = ULog(f, ['vehicle_local_position', 'vehicle_attitude'])
    lp = u.get_dataset('vehicle_local_position').data
    t, x, y, z = lp['timestamp'] / 1e6, lp['x'], lp['y'], lp['z']
    att = u.get_dataset('vehicle_attitude').data
    ta = att['timestamp'] / 1e6
    q0, q1, q2, q3 = (att[f'q[{i}]'] for i in range(4))
    roll = np.degrees(np.arctan2(2 * (q0 * q1 + q2 * q3), 1 - 2 * (q1 ** 2 + q2 ** 2)))
    pitch = np.degrees(np.arcsin(np.clip(2 * (q0 * q2 - q3 * q1), -1, 1)))
    tilt = np.maximum(np.abs(roll), np.abs(pitch))
    idx = np.where((y[:-1] < WALL_Y) & (y[1:] >= WALL_Y))[0]
    if len(idx) == 0:
        print(f"wall_cross=none max_y={y.max():.2f}")
        continue
    i = idx[0]
    near = (ta > t[i] - 2) & (ta < t[i] + 2)
    mt = tilt[near].max()
    print(f"wall_cross x={x[i]:.3f} z={z[i]:.3f} crossings={len(idx)} tilt_deg={mt:.1f} contact={'YES' if mt > CONTACT_TILT_DEG or len(idx) > 1 else 'NO'}")

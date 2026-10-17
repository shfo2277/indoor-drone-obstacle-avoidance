"""PX4 ulog에서 미션 지표 추출 (회피 후 착륙 미션)
- 미션 시간: 시동(arm) ~ 해제(disarm)
- 착륙 오차: 착지 위치와 목표점 (map 5, 0) 사이 수평 거리
- 비행 거리: 수평 이동 거리 (위치 흔들림 영향을 줄이려고 2 Hz로 다시 샘플링)
- 최소 장애물 여유: 비행 궤적(드론 중심)과 장애물 상자 사이 최소 거리 - 드론 반폭 0.21 m
- 최대 기울기: 비행 중 roll/pitch 최대값
좌표: planning 노드와 같이 map x = NED y, map y = NED x 로 변환
"""
import os
import sys

import numpy as np
from pyulog import ULog

GOAL = (5.0, 0.0)
DRONE_HALF_WIDTH = 0.21
# 녹화한 /planning_map에서 뽑은 장애물 (팽창 0.4 m 제외, map 좌표 x_min, x_max, y_min, y_max)
OBSTACLES = {
    'pillar_A': (1.70, 2.00, -0.15, 0.30),
    'pillar_B': (3.70, 4.00, -2.10, -1.70),
    'pillar_C': (3.70, 4.05, 0.85, 1.30),
    'wall': (5.80, 6.00, -1.90, 2.05),
}
if os.environ.get('PX4_SITL_WORLD') == 'smartfactory_slalom_wide':
    OBSTACLES['slalom_wall_1'] = (1.60, 1.80, -2.30, 0.30)
    OBSTACLES['slalom_wall_2'] = (3.50, 3.70, -0.30, 2.30)
if os.environ.get('PX4_SITL_WORLD') == 'smartfactory_slalom':
    # smartfactory_slalom.world에 추가한 벽 (실제 형상, map 좌표)
    OBSTACLES['slalom_wall_1'] = (0.90, 1.10, -1.75, 0.55)
    OBSTACLES['slalom_wall_2'] = (3.20, 3.40, -0.25, 2.65)


def box_dist(px, py, b):
    dx = np.maximum(np.maximum(b[0] - px, 0), px - b[1])
    dy = np.maximum(np.maximum(b[2] - py, 0), py - b[3])
    return np.hypot(dx, dy)


def main(f):
    u = ULog(f, ['vehicle_status', 'vehicle_local_position', 'vehicle_attitude'])
    st = u.get_dataset('vehicle_status').data
    ts = st['timestamp'] / 1e6
    armed = np.where(st['arming_state'] == 2)[0]
    if len(armed) == 0:
        print('flight=not_armed')
        return
    t_arm, t_dis = ts[armed[0]], ts[armed[-1]]
    disarmed_after = np.any((ts > t_dis) & (st['arming_state'] != 2))

    lp = u.get_dataset('vehicle_local_position').data
    t = lp['timestamp'] / 1e6
    m = (t >= t_arm) & (t <= t_dis)
    mx, my, z = lp['y'][m], lp['x'][m], lp['z'][m]  # map 좌표로 변환
    tt = t[m]

    # 2 Hz 재샘플링 후 수평 이동 거리
    grid = np.arange(tt[0], tt[-1], 0.5)
    rx, ry = np.interp(grid, tt, mx), np.interp(grid, tt, my)
    dist = float(np.sum(np.hypot(np.diff(rx), np.diff(ry))))

    land_err = float(np.hypot(mx[-1] - GOAL[0], my[-1] - GOAL[1]))

    air = z < -0.3
    clear = {k: float((box_dist(mx[air], my[air], b) - DRONE_HALF_WIDTH).min()) for k, b in OBSTACLES.items()}
    near_name = min(clear, key=clear.get)

    att = u.get_dataset('vehicle_attitude').data
    ta = att['timestamp'] / 1e6
    q0, q1, q2, q3 = (att[f'q[{i}]'] for i in range(4))
    roll = np.degrees(np.arctan2(2 * (q0 * q1 + q2 * q3), 1 - 2 * (q1 ** 2 + q2 ** 2)))
    pitch = np.degrees(np.arcsin(np.clip(2 * (q0 * q2 - q3 * q1), -1, 1)))
    fm = (ta >= t_arm) & (ta <= t_dis)
    tilt = float(np.maximum(np.abs(roll[fm]), np.abs(pitch[fm])).max())

    print(f"flight mission_s={t_dis - t_arm:.1f} disarmed={'YES' if disarmed_after else 'NO'} "
          f"dist_m={dist:.2f} land_err_m={land_err:.3f} land_x={mx[-1]:.2f} land_y={my[-1]:.2f} "
          f"min_clear_m={clear[near_name]:.3f} min_clear_obj={near_name} max_tilt_deg={tilt:.1f}")


if __name__ == '__main__':
    main(sys.argv[1])

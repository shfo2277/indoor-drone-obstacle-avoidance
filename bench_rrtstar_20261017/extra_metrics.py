"""시험 폴더별 추가 지표: ulog 수평 속도(최대/회피 구간 평균), 비행 중 RTAB-Map 시각 오도메트리 상실(quality=0) 횟수"""
import re
import sys

import numpy as np
from pyulog import ULog

for d in sys.argv[1:]:
    res = open(f'{d}/result.txt', encoding='utf-8').read()
    ulg = re.search(r'ulog=(\S+)', res).group(1)
    t_alt = float(re.search(r't_alt=([0-9.]+)', res).group(1))
    t_av = float(re.search(r't_avoid=([0-9.]+)', res).group(1))
    t_ld = float(re.search(r't_land=([0-9.]+)', res).group(1))
    sim = open(f'{d}/sim.log', errors='ignore').read()
    ev = [float(x) for x in re.findall(r'\[(\d{10}\.\d+)\] \[rgbd_odometry\]: Odom: quality=0, std dev=0\.0', sim)]
    lost_avoid = sum(t_alt <= e < t_av for e in ev)
    lost_flight = sum(t_alt <= e < t_ld + 15 for e in ev)
    u = ULog(ulg, ['vehicle_local_position', 'vehicle_status'])
    lp = u.get_dataset('vehicle_local_position').data
    v = np.hypot(lp['vx'], lp['vy'])
    air = lp['z'] < -0.5
    vmax = float(v[air].max())
    # 회피 구간 평균 속도: 순항 구간(속도 > 0.05 m/s)만
    cruise = air & (v > 0.05)
    vmean = float(v[cruise].mean())
    with open(f'{d}/extra.txt', 'w') as f:
        f.write(f'vmax_mps={vmax:.3f} vmean_mps={vmean:.3f} odom_lost_avoid={lost_avoid} odom_lost_flight={lost_flight}\n')
    print(d.split('/')[-1], open(f'{d}/extra.txt').read().strip())

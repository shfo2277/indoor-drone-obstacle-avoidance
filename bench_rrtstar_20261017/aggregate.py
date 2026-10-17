"""trials/<label>_*/result.txt 를 모아 평가지표 통계 출력: python3 aggregate.py <label>"""
import glob
import math
import re
import sys

import numpy as np


def parse(path):
    txt = open(path, encoding='utf-8').read()
    kv = dict(re.findall(r'(\w+)=([^\s]+)', txt))
    kv['status'] = re.search(r'RESULT \S+ (\S+)', txt).group(1)
    kv['plan_ms_list'] = [float(x) for x in re.search(r'plan_ms=([^\n]*)', txt).group(1).split()] if 'plan_ms=' in txt else []
    return kv


def wilson(k, n, z=1.96):
    if n == 0:
        return (0, 0)
    p = k / n
    d = 1 + z * z / n
    c = p + z * z / (2 * n)
    r = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n))
    return ((c - r) / d, (c + r) / d)


def stat(name, vals, unit, fmt='{:.2f}'):
    v = np.array([x for x in vals if x is not None], dtype=float)
    if len(v) == 0:
        print(f'{name:28s} 데이터 없음')
        return
    print(f'{name:28s} 평균 {fmt.format(v.mean())} ± {fmt.format(v.std(ddof=1) if len(v) > 1 else 0)} {unit} | '
          f'중앙값 {fmt.format(np.median(v))} | 최소 {fmt.format(v.min())} | 최대 {fmt.format(v.max())} | '
          f'p95 {fmt.format(np.percentile(v, 95))} (n={len(v)})')


label = sys.argv[1]
runs = [parse(p) for p in sorted(glob.glob(f'trials/{label}_*/result.txt'))]
n = len(runs)
f = lambda r, k: float(r[k]) if k in r and r[k] not in ('', 'none') else None

ok = [r for r in runs if r['status'] == 'MISSION_DONE' and r.get('landing_detected') == '1'
      and r.get('disarmed') == 'YES' and r.get('failsafe_lines') == '0']
k = len(ok)
lo, hi = wilson(k, n)
print(f'== {label}: {n}회')
print(f'{"미션 성공률":28s} {k}/{n} = {100 * k / n:.0f}% (95% 신뢰구간 {100 * lo:.0f}~{100 * hi:.0f}%)')
print(f'{"failsafe 발생":28s} {sum(1 for r in runs if r.get("failsafe_lines") not in ("0", None))}회')
stat('미션 시간 (arm~disarm)', [f(r, 'mission_s') for r in runs], 's', '{:.1f}')
stat('회피 시간', [f(r, 'avoid_time_s') for r in runs], 's', '{:.1f}')
stat('수평 비행 거리 (2 Hz)', [f(r, 'dist_m') for r in runs], 'm')
stat('첫 경로 길이', [f(r, 'first_path_m') for r in runs], 'm')
stat('착륙 위치 오차', [f(r, 'land_err_m') for r in runs], 'm', '{:.3f}')
stat('장애물 최소 여유 (표면)', [f(r, 'min_clear_m') for r in runs], 'm', '{:.3f}')
stat('최대 기울기', [f(r, 'max_tilt_deg') for r in runs], '°', '{:.1f}')
allplan = [x for r in runs for x in r['plan_ms_list']]
stat('경로 계산 시간 (전체 계획)', allplan, 'ms', '{:.0f}')
stat('재계획 중 경로 교체', [f(r, 'adopted') for r in runs], '회', '{:.1f}')

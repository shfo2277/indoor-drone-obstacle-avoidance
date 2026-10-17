"""장애물 배치 검증: planning 노드와 같은 격자(5cm, 400x400, 원점 -10)·팽창(0.4m 정사각)으로 맵을 만들고
격자 A* 최단 경로 길이, 직선 대비 비율, 경로 위 가장 좁은 여유를 계산. bench 입력용 grid도 저장.
usage: python3 layout_check.py <layout_name>
좌표는 planning map 좌표 (= Gazebo world + (0, +2.05))
"""
import heapq
import json
import math
import sys

import numpy as np

RES, W, H, OX, OY = 0.05, 400, 400, -10.0, -10.0
INFL = 8  # (드론 반경 0.3 + 여유 0.1) / 0.05
START, GOAL = (0.0, 0.0), (5.0, 0.0)
SAMPLE = (-1.0, 5.5, -4.0, 5.5)  # RRT* 샘플링 범위 (x_min, x_max, y_min, y_max)

BASE = {  # smartfactory.world 기존 장애물 (map 좌표)
    'pillars': [(1.0, 2.05), (1.0, -1.95), (2.0, 0.05), (4.0, -1.95), (4.0, 1.05)],  # 반지름 0.2
    'boxes': [(6.0, 0.05, 0.1, 3.9)],  # 창문 벽 (cx, cy, sx, sy)
}
LAYOUTS = {
    'base': [],
    # 슬라럼: 벽 W1이 아래를 막아 위로 돌게 하고, 벽 W2가 위를 막아 아래로 돌게 함
    'slalom': [(1.0, -0.6, 0.2, 2.3), (3.3, 1.2, 0.2, 2.9)],
}


def build(extra):
    occ_raw = np.zeros((H, W), bool)
    ys, xs = np.mgrid[0:H, 0:W]
    cx = OX + (xs + 0.5) * RES
    cy = OY + (ys + 0.5) * RES
    for px, py in BASE['pillars']:
        occ_raw |= np.hypot(cx - px, cy - py) <= 0.2
    for bx, by, sx, sy in BASE['boxes'] + extra:
        occ_raw |= (np.abs(cx - bx) <= sx / 2) & (np.abs(cy - by) <= sy / 2)
    # planning.cpp cloud_callback과 같은 정사각 팽창
    occ = occ_raw.copy()
    for dy in range(-INFL, INFL + 1):
        for dx in range(-INFL, INFL + 1):
            occ |= np.roll(np.roll(occ_raw, dy, 0), dx, 1)
    return occ_raw, occ


def to_cell(p):
    return int((p[1] - OY) / RES), int((p[0] - OX) / RES)


def astar(occ):
    s, g = to_cell(START), to_cell(GOAL)
    x0, x1, y0, y1 = [int((v - o) / RES) for v, o in zip(SAMPLE, (OX, OX, OY, OY))]
    dist = {s: 0.0}
    prev = {}
    pq = [(0.0, s)]
    moves = [(dy, dx, math.hypot(dy, dx) * RES) for dy in (-1, 0, 1) for dx in (-1, 0, 1) if dy or dx]
    while pq:
        _, c = heapq.heappop(pq)
        if c == g:
            break
        for dy, dx, w in moves:
            n = (c[0] + dy, c[1] + dx)
            if not (y0 <= n[0] < y1 and x0 <= n[1] < x1) or occ[n]:
                continue
            nd = dist[c] + w
            if nd < dist.get(n, 1e9):
                dist[n] = nd
                prev[n] = c
                heapq.heappush(pq, (nd + math.hypot(n[0] - g[0], n[1] - g[1]) * RES, n))
    if g not in dist:
        return None, None
    path = [g]
    while path[-1] != s:
        path.append(prev[path[-1]])
    return dist[g], path[::-1]


def main(name):
    raw, occ = build(LAYOUTS[name])
    length, path = astar(occ)
    if length is None:
        print(f'{name}: ❌ 경로 없음')
        return
    # 경로 위 각 점에서 팽창 영역까지의 여유 (통로가 얼마나 좁은지)
    occ_idx = np.argwhere(occ)
    margins = []
    for c in path[::4]:
        d = np.min(np.hypot(occ_idx[:, 0] - c[0], occ_idx[:, 1] - c[1])) * RES
        margins.append(d)
    straight = math.dist(START, GOAL)
    print(f'{name}: A* 최단 {length:.2f} m (직선 {straight:.1f} m의 {length / straight:.2f}배), '
          f'경로 위 팽창 경계까지 최소 여유 {min(margins):.2f} m')
    data = np.where(occ, 100, 0).astype(np.int8)
    data.tofile(f'grid_{name}.bin')
    json.dump({'width': W, 'height': H, 'resolution': RES, 'origin_x': OX, 'origin_y': OY,
               'occupied_cells': int(occ.sum())}, open(f'grid_{name}.json', 'w'))


if __name__ == '__main__':
    for n in sys.argv[1:]:
        main(n)

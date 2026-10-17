# indoor-drone-obstacle-avoidance

GPS가 없는 실내 물류창고를 가정한 **자율비행 드론 장애물 회피 시스템**입니다.
RGB-D 카메라로 RTAB-Map 지도를 만들고, 포인트클라우드 기반 2D 점유 격자 위에서 **RRT\*** 로 경로를 계획해 PX4 Offboard 모드로 비행합니다.
(PX4 SITL + Gazebo Classic + ROS 2 Humble)

## 시스템 구성

```
RGB-D 카메라 ─▶ RTAB-Map (SLAM) ─/cloud_map─▶ filtering_map_node ─/filtering_map─▶ planning_node ─▶ PX4
                     └─ TF (map↔odom) ───────────────────────────────────────────▶      ▲
PX4 SITL ─/fmu/out/vehicle_odometry──────────────────────────────────────────────▶      │
canny_edge_detector ─/hole_info (창문 탐지)────────────────────────────────────────▶  ───┘
```

| 패키지 | 역할 |
|---|---|
| `src/planning` | 경로계획 + 비행 상태머신 (`planning_node`) |
| `src/planning/include/planning/rrt_star.hpp` | ROS 의존성 없는 RRT\* 코어 (노드와 오프라인 벤치마크가 같은 코드 사용) |
| `src/cannyedge_test1` | Canny 기반 사각형(창문) 탐지, 창틀 depth로 거리 실측 |
| `src/cannyedge_test1_interfaces` | `HoleInfo.msg` |
| `src/bridge_imu_ws` | PX4 IMU → `sensor_msgs/Imu` 변환 |
| `src/px4_msgs` | PX4 메시지 정의 (PX4 프로젝트, BSD-3) |
| `bench_rrtstar_20261017` | 실험 도구(자동 반복 실험, ulog 분석, 오프라인 벤치마크)와 결과 요약 |

RTAB-Map(`ws_rtabmap`), 포인트클라우드 필터(`voxel_mapper`), 실행용 launch(`launch_pkg`)는 별도 워크스페이스입니다.

## 경로계획 (`planning_node`)

1. `/filtering_map` 포인트클라우드에서 비행 고도대(0.3\~1.5 m) 점만 남겨 5 cm 2D 격자 생성, 0.4 m 팽창
2. **RRT\***: choose parent + rewire + 하위 트리 cost 전파, 첫 해를 찾은 뒤 200 ms 동안 경로 개선
3. **Path shortcut** 으로 잔꺾임 제거 → 10 cm 간격 보간
4. **Lookahead 추종** (0.5 m 앞 점을 목표로) 으로 PX4에 위치 setpoint 전송
5. 경로 계산은 백그라운드 스레드에서 수행 (100 ms 제어 루프·offboard 하트비트 유지)
6. 새로 보인 장애물이 남은 경로를 막으면 즉시 정지 + 재계획, 길이와 상관없이 새 경로 채택.
   재계획에 실패하면 지나온 길로 0.6 m 후진해 지도를 새로 받은 뒤 다시 계획

주요 ROS 파라미터: `use_shortcut`, `use_lookahead`, `lookahead_dist`, `pass_window`(창문 통과 미션 사용 여부), `land_settle_tol` 등

## 실행

```bash
# 빌드
cd ~/drone_1team && colcon build --packages-ignore my-test-package
source install/setup.bash

# 시뮬레이션 (PX4 SITL, Gazebo, XRCE Agent, RTAB-Map, 비전, 필터, RViz)
ros2 launch launch_pkg all_launch.py
#   장애물 맵 변경: PX4_SITL_WORLD=smartfactory_slalom_wide ros2 launch launch_pkg all_launch.py

# 경로계획 노드 (별도 터미널)
ros2 run planning planning_node
```

## 결과 요약 (PX4 SITL, `smartfactory.world`)

| 지표 | 기존 RRT (팀 원본) | 개선 RRT\* (1단계) | + shortcut + lookahead (2단계) |
|---|---|---|---|
| 계획 경로 길이 (오프라인 100회) | 6.66 ± 0.82 m | 5.17 ± 0.08 m (-22.4%) | 5.22 ± 0.04 m |
| 경로 꺾임 (오프라인 100회) | 31.6 rad | 2.8 rad | 0.7 rad |
| 회피 시간 (실비행) | 40.0 s (5회) | 33.5 s (10회) | **12.7 s (10회, -68%)** |
| 장애물 최소 여유 | — | 0.225 m | 0.315 m |
| 미션 성공 | — | 10/10 | 9/10 (+3/3) |

- 지표는 PX4 비행 기록(ulog)에서 자동 추출합니다 (`bench_rrtstar_20261017/analyze_flight.py`, `aggregate.py`).
- 넓힌 슬라럼 맵(`smartfactory_slalom_wide.world`, 최단 경로가 직선의 1.33배)에서 이륙 → 회피 → 창문 통과 → 착륙 전체 미션 성공 (반복 검증 진행 중).

## 알려진 한계

- 시뮬레이션에서는 PX4가 가상 GPS로 위치를 추정합니다. 실제 GPS 없는 환경에서는 RTAB-Map 시각 오도메트리를 PX4 EKF에 넣어야 합니다.
- 벽에 가깝거나 빠르게 비행할 때 RTAB-Map 시각 오도메트리가 끊길 수 있고(RGB·depth 프레임 시간차), 이 동안 지도가 갱신되지 않습니다.
- 시뮬레이션 창문 개구부(약 0.46 m)가 드론 폭(0.42 m)에 비해 좁아 창문 통과는 여유가 작습니다.

## Credits

졸업 프로젝트 드론 1팀 코드([MusiMusi-Shark/drone_1team](https://github.com/MusiMusi-Shark/drone_1team))를 바탕으로 경로계획·실험 체계를 개선했습니다.

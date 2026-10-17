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

## 프로젝트 구조

```
├── src/
│   ├── planning/                         # 경로계획 + 비행 제어 (planning_node)
│   │   ├── src/planning.cpp              #   ROS 노드: 점유 격자 생성, 재계획, 경로 추종, 미션 상태머신
│   │   └── include/planning/rrt_star.hpp #   RRT* 코어 + path shortcut (ROS 의존성 없음)
│   ├── cannyedge_test1/                  # 비전: 창문(사각형) 탐지
│   │   └── cannyedge_test1/canny_edge_detector.py  # Canny 에지 → 사각형 검출 → /hole_info 발행
│   ├── cannyedge_test1_interfaces/       # 메시지 정의
│   │   └── msg/HoleInfo.msg              #   창문 중심, 크기, 통과 가능 여부
│   ├── bridge_imu_ws/src/bridge_imu/     # PX4 IMU → sensor_msgs/Imu 변환 (RTAB-Map 입력용)
│   │   └── bridge_imu/imu_bridge.py
│   └── px4_msgs/                         # PX4 ROS 2 메시지 정의 (PX4 프로젝트, BSD-3)
└── bench_rrtstar_20261017/               # 실험 도구와 결과
    ├── run_trial.sh                      #   시뮬레이션 1회 자동 실행 (기동 → 미션 → 정리 → 지표 저장)
    ├── analyze_flight.py                 #   PX4 비행 기록(ulog)에서 미션 시간·착륙 오차·장애물 여유·기울기 추출
    ├── avoid_phase.py                    #   장애물 회피 구간만 평가 (충돌 판정 포함)
    ├── aggregate.py                      #   여러 회차 통계 (평균, p95, 성공률 신뢰구간)
    ├── extra_metrics.py                  #   비행 속도, 시각 오도메트리 상실 횟수
    ├── wall_cross.py                     #   창문 벽 통과 위치·기울기
    ├── record_grid.py                    #   비행 중 장애물 격자(/planning_map) 녹화
    ├── bench.cpp                         #   녹화한 격자에서 RRT 알고리즘 오프라인 비교 (시드 고정)
    ├── layout_check.py                   #   장애물 배치 검증 (A* 최단 경로, 통로 폭)
    ├── smartfactory_slalom*.world        #   추가 장애물 맵 (Gazebo)
    ├── summary_*.txt, res_*.csv          #   실험 결과 요약
    └── trials/                           #   회차별 결과 (result.txt, 장애물 격자)
```

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

| 지표 | 기존 RRT | 개선 RRT\* (1단계) | + shortcut + lookahead (2단계) |
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

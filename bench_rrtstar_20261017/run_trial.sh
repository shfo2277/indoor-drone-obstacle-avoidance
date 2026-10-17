#!/bin/bash
# 시뮬레이션 1회 자동 실행: ./run_trial.sh <label> <번호> [record]
# 결과: trials/<label>_<번호>/{sim.log,planning.log,result.txt}
B=/tmp/claude-1000/-home-rth/7c4197cc-4a0b-4c08-bebf-5ad66e7358be/scratchpad/bench
OUT=$B/trials/$1_$2
mkdir -p "$OUT"
source /opt/ros/humble/setup.bash
source ~/launch_pkg/install/setup.bash

cleanup() {
  [ -n "$REC" ] && kill -INT "$REC" 2>/dev/null
  kill -INT -- -"$PLAN" 2>/dev/null
  kill -INT -- -"$SIM" 2>/dev/null
  sleep 8
  pkill -f 'gzserver|gzclient|bin/px4|MicroXRCEAgent|QGroundControl|rviz2|rtabmap|rgbd_odometry|rgbd_sync|canny_edge_detector|filtering_map_node|imu_bridge|static_transform_publisher|planning_node|record_grid.py' 2>/dev/null
  sleep 3
  pkill -9 -f 'gzserver|gzclient|bin/px4' 2>/dev/null
  sleep 1
}

cd ~
setsid ${LAUNCH_CMD:-ros2 launch launch_pkg all_launch.py} > "$OUT/sim.log" 2>&1 &
SIM=$!
for i in $(seq 1 180); do grep -aq 'Ready for takeoff' "$OUT/sim.log" && break; sleep 1; done
if ! grep -aq 'Ready for takeoff' "$OUT/sim.log"; then echo "RESULT $1_$2 SIM_NOT_READY" | tee "$OUT/result.txt"; cleanup; exit 1; fi
sleep 3

source ~/drone_1team/install/setup.bash
if true; then  # 장애물 맵 항상 녹화 (막혔을 때 원인 확인용)
  python3 $B/record_grid.py "$OUT" > "$OUT/recorder.log" 2>&1 &
  REC=$!
fi
# PLAN_EXE: 다른 planning 실행 파일로 비교할 때 (예: 처음 코드)
if [ -n "$PLAN_EXE" ]; then
  RCUTILS_LOGGING_BUFFERED_STREAM=0 setsid stdbuf -oL $PLAN_EXE $PLAN_ARGS > "$OUT/planning.log" 2>&1 &
else
  RCUTILS_LOGGING_BUFFERED_STREAM=0 setsid stdbuf -oL ros2 run planning planning_node $PLAN_ARGS > "$OUT/planning.log" 2>&1 &
fi
PLAN=$!

STATUS=TIMEOUT
DEADLINE=$((SECONDS + ${TRIAL_DEADLINE:-600}))
while [ $SECONDS -lt $DEADLINE ]; do
  if grep -aq '장애물 회피 완료' "$OUT/planning.log"; then
    STATUS=AVOID_DONE
    # STOP_AFTER_AVOID: 회피 구간만 비교할 때 회피 완료 5초 뒤 종료
    if [ -n "$STOP_AFTER_AVOID" ]; then sleep 5; break; fi
  fi
  if grep -aq 'Landing command sent' "$OUT/planning.log"; then STATUS=MISSION_DONE; break; fi
  if grep -aqE 'Segmentation|core dumped|terminate called|Aborted' "$OUT/planning.log"; then STATUS=CRASH; break; fi
  sleep 1
done
# 착륙 명령 후 실제 착지(Landing detected)까지 최대 40초 대기
if [ "$STATUS" = MISSION_DONE ]; then
  for i in $(seq 1 40); do grep -aq 'Landing detected' "$OUT/sim.log" && break; sleep 1; done
fi
sleep 2

L="$OUT/planning.log"
ts() { grep -a "$1" "$L" | head -1 | grep -aoE '\[[0-9]{10}\.[0-9]+\]' | head -1 | tr -d '[]'; }
FS=$(sed 's/\x1b\[[0-9;]*m//g' "$OUT/sim.log" | grep -aciE 'failsafe activated|offboard.*(lost|loss)|Failsafe:')
{
  echo "RESULT $1_$2 $STATUS"
  echo "first_path_m=$(grep -aoP '첫 경로 생성 \(\K[0-9.]+' "$L" | head -1)"
  echo "avoid_dist_m=$(grep -aoP '최종 이동 거리: \K[0-9.]+' "$L" | head -1)"
  echo "avoid_time_s=$(grep -aoP '장애물 회피 시간: \K[0-9.]+' "$L" | head -1)"
  echo "plans=$(grep -ac '생성된 경로 길이' "$L") adopted=$(grep -ac '짧은 새 경로 채택' "$L") ignored=$(grep -ac '새 경로 무시' "$L") failed=$(grep -ac '경로 생성 실패' "$L")"
  echo "plan_ms=$(grep -aoP '경로 생성 시간: \K[0-9]+' "$L" | tr '\n' ' ')"
  echo "path_lengths=$(grep -aoP '생성된 경로 길이: \K[0-9.]+' "$L" | tr '\n' ' ')"
  echo "failsafe_lines=$FS"
  echo "landing_detected=$(sed 's/\x1b\[[0-9;]*m//g' "$OUT/sim.log" | grep -ac 'Landing detected')"
  echo "t_arm=$(ts 'Arm command') t_alt=$(ts 'Setpoint 위치') t_avoid=$(ts '장애물 회피 완료') t_detect=$(ts 'Detection Process End') t_align=$(ts '정렬 완료') t_pass=$(ts '모든 이동 완료') t_land=$(ts 'Landing command sent')"
  echo "detect=$(grep -aoE 'Detection Process End.*' "$L" | head -1)"
  echo "align=$(grep -aoE '정렬 완료.*' "$L" | head -1)"
} | tee "$OUT/result.txt"
cleanup
# PX4 비행 기록에서 벽 통과 위치/기울기 확인
ULG=$(ls -t ~/PX4-Autopilot/build/px4_sitl_default/rootfs/log/*/*.ulg | head -1)
(cd $B && python3 analyze_flight.py "$ULG" 2>/dev/null | grep -v Warn) | tee -a "$OUT/result.txt"
(cd $B && python3 avoid_phase.py "$ULG" 2>/dev/null | grep -v Warn) | tee -a "$OUT/result.txt"
echo "ulog=$ULG" >> "$OUT/result.txt"

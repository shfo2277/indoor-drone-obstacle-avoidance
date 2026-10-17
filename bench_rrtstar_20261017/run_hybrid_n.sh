#!/bin/bash
# 혼합 버전 1회: 회피 = 개선 RRT*, 창문 탐색·정렬·통과·목적지 = 원본, 비전·launch = 원본, 맵 = 넓힌 슬라럼
N=$1
B=/tmp/claude-1000/-home-rth/7c4197cc-4a0b-4c08-bebf-5ad66e7358be/scratchpad/bench
export PLAN_EXE=/tmp/claude-1000/-home-rth/7c4197cc-4a0b-4c08-bebf-5ad66e7358be/scratchpad/hybrid_ws/install/planning/lib/planning/planning_node
export PLAN_ARGS="--ros-args -p pass_window:=true"
export LAUNCH_CMD="ros2 launch $B/all_launch_orig.py"
export PX4_SITL_WORLD=smartfactory_slalom_wide
$B/run_trial.sh hybrid_wide $N > /dev/null 2>&1
R=$B/trials/hybrid_wide_$N
grep -vE '^detect=|^plan_ms|^path_lengths' $R/result.txt
U=$(grep -oE 'ulog=\S+' $R/result.txt | cut -d= -f2)
cd $B && python3 wall_cross.py "$U" 2>/dev/null | grep -v Warn
echo "맵갱신=$(grep -ac 'Filtering 후 포인트 수' $R/sim.log) 오도메트리상실=$(grep -ac 'quality=0, std dev=0.0' $R/sim.log)"
grep -aE '⛔|🔄|장애물 회피 완료|Detection Process End|정렬 완료|모든 이동 완료|Landing command sent' $R/planning.log | head -8 | cut -c1-100
grep -aE '↩️|후진 완료' $R/planning.log | head -6 | cut -c1-100
U=$(grep -oE 'ulog=\S+' $R/result.txt | cut -d= -f2)
cd $B && python3 wall_cross.py "$U" 2>/dev/null | grep -v Warn

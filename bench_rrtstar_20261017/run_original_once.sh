#!/bin/bash
# 맨 처음 코드 그대로 1회 실행 (원본 planning + 원본 launch/비전 + smartfactory.world)
B=/tmp/claude-1000/-home-rth/7c4197cc-4a0b-4c08-bebf-5ad66e7358be/scratchpad/bench
export PLAN_EXE=/tmp/claude-1000/-home-rth/7c4197cc-4a0b-4c08-bebf-5ad66e7358be/scratchpad/orig_ws/install/planning/lib/planning/planning_node
export LAUNCH_CMD="ros2 launch $B/all_launch_orig.py"
$B/run_trial.sh original 1 > /dev/null 2>&1
R=$B/trials/original_1
grep -vE '^detect=|^plan_ms|^path_lengths' $R/result.txt
U=$(grep -oE 'ulog=\S+' $R/result.txt | cut -d= -f2)
cd $B && python3 wall_cross.py "$U" 2>/dev/null | grep -v Warn
grep -aE '장애물 회피 완료|Detection Process End|정렬 완료|모든 이동 완료|Landing command sent' $R/planning.log | head -5 | cut -c1-100

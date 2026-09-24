#!/bin/bash
# GRAB阶段集成测试
# 用法: source install/setup.bash && bash src/manipulator/scripts/test_grab_phases.sh

echo "=== GRAB阶段集成测试 ==="
echo ""

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

PASS=0
FAIL=0

check_result() {
    local test_name="$1"
    local expected="$2"
    local actual="$3"
    if [ "$expected" = "$actual" ]; then
        echo -e "  ${GREEN}✓ $test_name${NC}"
        PASS=$((PASS + 1))
    else
        echo -e "  ${RED}✗ $test_name (期望: $expected, 实际: $actual)${NC}"
        FAIL=$((FAIL + 1))
    fi
}

# helper: 读一次话题（文本方式，非二进制）
read_once() {
    timeout 2 bash -c "ros2 topic echo --once '$1' '$2' --no-daemon 2>/dev/null | grep '^data:' | head -1 | sed 's/^data: //'" 2>/dev/null || echo ""
}

# ========== 0. 前置检查 ==========
echo "0. 前置检查"
echo "----------------------------------------"
NODES=$(ros2 node list 2>/dev/null)
echo "$NODES" | grep -q "manipulator_manager" && echo -e "  ${GREEN}✓ manipulator_manager_node${NC}" || echo -e "  ${YELLOW}⚠ manipulator_manager_node${NC}"
echo "$NODES" | grep -q "arm_controller" && echo -e "  ${GREEN}✓ arm_controller${NC}" || echo -e "  ${YELLOW}⚠ arm_controller${NC}"

# ========== 1. 完整GRAB流程 ==========
echo ""
echo "1. 完整GRAB流程测试"
echo "----------------------------------------"

echo "  [步骤1] 重置信号..."
ros2 topic pub --once /quad/arm/arrived std_msgs/msg/Bool "data: false" 2>/dev/null
ros2 topic pub --once /quad/logistics_pickup_reached std_msgs/msg/Bool "data: false" 2>/dev/null
sleep 1

echo "  [步骤2] 发送GRAB命令..."
ros2 topic pub --once /manipulator/cmd std_msgs/msg/UInt8 "data: 1" 2>/dev/null
sleep 2

STATE=$(read_once /manipulator/state std_msgs/msg/String)
echo "  状态: ${STATE:-空}"
check_result "GRAB启动" "RUNNING" "${STATE:-空}"

echo "  [步骤3] 模拟arm到达..."
ros2 topic pub --once /quad/arm/arrived std_msgs/msg/Bool "data: true" 2>/dev/null
sleep 3

RESULT=$(read_once /manipulator/result std_msgs/msg/Bool)
echo "  result: ${RESULT:-空}"
check_result "arm到达" "true" "${RESULT:-空}"

echo "  [步骤4] 模拟approach_ready..."
ros2 topic pub --once /quad/logistics_pickup_reached std_msgs/msg/Bool "data: true" 2>/dev/null
sleep 5

RESULT2=$(read_once /manipulator/result std_msgs/msg/Bool)
echo "  result: ${RESULT2:-空}"
if [ "$RESULT2" = "true" ]; then
    check_result "任务完成" "true" "true"
else
    STATE2=$(read_once /manipulator/state std_msgs/msg/String)
    echo "  state: ${STATE2:-空}"
    if [ "$STATE2" = "DONE" ]; then
        check_result "任务完成" "true" "DONE"
    else
        check_result "任务完成" "true" "state=$STATE2"
    fi
fi

# ========== 2. 超时场景 ==========
echo ""
echo "2. 超时场景测试"
echo "----------------------------------------"

echo "  [步骤1] 重置..."
ros2 topic pub --once /quad/arm/arrived std_msgs/msg/Bool "data: false" 2>/dev/null
ros2 topic pub --once /quad/logistics_pickup_reached std_msgs/msg/Bool "data: false" 2>/dev/null
sleep 1

echo "  [步骤2] 发送GRAB (不发approach_ready)..."
ros2 topic pub --once /manipulator/cmd std_msgs/msg/UInt8 "data: 1" 2>/dev/null
sleep 1

echo "  [步骤3] 发送arm_arrived..."
ros2 topic pub --once /quad/arm/arrived std_msgs/msg/Bool "data: true" 2>/dev/null
sleep 2

echo "  [步骤4] 等待超时 (10秒)..."
sleep 10

STATE3=$(read_once /manipulator/state std_msgs/msg/String)
echo "  超时后状态: ${STATE3:-空}"
if [ "$STATE3" = "DONE" ]; then
    check_result "超时后完成" "DONE" "DONE"
elif [ "$STATE3" = "RUNNING" ]; then
    check_result "超时后完成" "DONE" "RUNNING"
else
    check_result "超时后完成" "DONE" "${STATE3:-空}"
fi

# ========== 3. 节点日志检查 ==========
echo ""
echo "3. 节点状态"
echo "----------------------------------------"
echo "  可用话题:"
ros2 topic list 2>/dev/null | grep -E "manipulator|arm" | sed 's/^/    /'

# ========== 总结 ==========
echo ""
echo "=== 测试总结 ==="
echo -e "通过: ${GREEN}$PASS${NC}"
echo -e "失败: ${RED}$FAIL${NC}"
echo ""

if [ $FAIL -eq 0 ]; then
    echo -e "${GREEN}所有测试通过!${NC}"
else
    echo -e "${YELLOW}部分测试失败${NC}"
fi

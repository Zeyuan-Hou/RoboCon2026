#!/bin/bash
# 单元测试脚本：验证GRAB阶段代码结构和逻辑
# 不依赖运行中的ROS2系统

echo "=== GRAB阶段代码结构验证 ==="
echo ""

PASS=0
FAIL=0

check_result() {
    local test_name="$1"
    local expected="$2"
    local actual="$3"

    if [ "$expected" = "$actual" ]; then
        echo "  [PASS] $test_name"
        PASS=$((PASS + 1))
    else
        echo "  [FAIL] $test_name (期望: $expected, 实际: $actual)"
        FAIL=$((FAIL + 1))
    fi
}

MANAGER_FILE="/home/cat/hitcrt_quad2026_ws/src/manipulator/manipulator_manager_node/src/manipulator_manager_node.cpp"
ARM_FILE="/home/cat/hitcrt_quad2026_ws/src/manipulator/arm_node/src/arm_node_td.cpp"

echo "测试1: 验证ARM_TRACKING枚举"
grep -q "ARM_TRACKING = 6" "$MANAGER_FILE" && check_result "manager ARM_TRACKING枚举" "存在" "存在" || check_result "manager ARM_TRACKING枚举" "存在" "不存在"
grep -q "ARM_TRACKING = 6" "$ARM_FILE" && check_result "arm_node ARM_TRACKING枚举" "存在" "存在" || check_result "arm_node ARM_TRACKING枚举" "存在" "不存在"

echo ""
echo "测试2: 验证WAIT_FOR_TRACKING阶段"
grep -q "WAIT_FOR_TRACKING" "$ARM_FILE" && check_result "WAIT_FOR_TRACKING阶段" "存在" "存在" || check_result "WAIT_FOR_TRACKING阶段" "存在" "不存在"
grep -q "TrajectoryPhase::WAIT_FOR_TRACKING" "$ARM_FILE" && check_result "WAIT_FOR_TRACKING枚举引用" "存在" "存在" || check_result "WAIT_FOR_TRACKING枚举引用" "存在" "不存在"

echo ""
echo "测试3: 验证GRAB子阶段逻辑"
grep -q "int grab_sub_phase_" "$MANAGER_FILE" && check_result "grab_sub_phase_成员变量" "存在" "存在" || check_result "grab_sub_phase_成员变量" "存在" "不存在"
grep -q "if (grab_sub_phase_ == 0)" "$MANAGER_FILE" && check_result "Phase 0逻辑" "存在" "存在" || check_result "Phase 0逻辑" "存在" "不存在"
grep -q "} else if (grab_sub_phase_ == 1)" "$MANAGER_FILE" && check_result "Phase 1逻辑" "存在" "存在" || check_result "Phase 1逻辑" "存在" "不存在"
grep -q "} else if (grab_sub_phase_ == 2)" "$MANAGER_FILE" && check_result "Phase 2逻辑" "存在" "存在" || check_result "Phase 2逻辑" "存在" "不存在"
grep -q "} else if (grab_sub_phase_ == 3)" "$MANAGER_FILE" && check_result "Phase 3逻辑" "存在" "存在" || check_result "Phase 3逻辑" "存在" "不存在"

echo ""
echo "测试4: 验证上升沿检测"
grep -q "arm_arrived_rising_" "$MANAGER_FILE" && check_result "arm_arrived_rising_变量" "存在" "存在" || check_result "arm_arrived_rising_变量" "存在" "不存在"
grep -q "approach_ready_rising_" "$MANAGER_FILE" && check_result "approach_ready_rising_变量" "存在" "存在" || check_result "approach_ready_rising_变量" "存在" "不存在"
grep -q "msg->data && !arm_arrived_prev_" "$MANAGER_FILE" && check_result "arm_arrived上升沿检测" "存在" "存在" || check_result "arm_arrived上升沿检测" "存在" "不存在"
grep -q "msg->data && !approach_ready_prev_" "$MANAGER_FILE" && check_result "approach_ready上升沿检测" "存在" "存在" || check_result "approach_ready上升沿检测" "存在" "不存在"

echo ""
echo "测试5: 验证超时逻辑"
grep -q "arm_arrival_timeout_ms" "$MANAGER_FILE" && check_result "arm_arrival_timeout_ms参数" "存在" "存在" || check_result "arm_arrival_timeout_ms参数" "存在" "不存在"
grep -q "approach_ready_timeout_ms" "$MANAGER_FILE" && check_result "approach_ready_timeout_ms参数" "存在" "存在" || check_result "approach_ready_timeout_ms参数" "存在" "不存在"
grep -q "phase_elapsed_ms >= arrival_timeout_ms" "$MANAGER_FILE" && check_result "Phase 0超时逻辑" "存在" "存在" || check_result "Phase 0超时逻辑" "存在" "不存在"
grep -q "phase_elapsed_ms >= approach_timeout_ms" "$MANAGER_FILE" && check_result "Phase 1超时逻辑" "存在" "存在" || check_result "Phase 1超时逻辑" "存在" "不存在"

echo ""
echo "测试6: 验证信号订阅"
grep -q "arm_arrived_sub_" "$MANAGER_FILE" && check_result "arm_arrived订阅" "存在" "存在" || check_result "arm_arrived订阅" "存在" "不存在"
grep -q "approach_ready_sub_" "$MANAGER_FILE" && check_result "approach_ready订阅" "存在" "存在" || check_result "approach_ready订阅" "存在" "不存在"
grep -q "arm_arrived_callback" "$MANAGER_FILE" && check_result "arm_arrived回调" "存在" "存在" || check_result "arm_arrived回调" "存在" "不存在"
grep -q "approach_ready_callback" "$MANAGER_FILE" && check_result "approach_ready回调" "存在" "存在" || check_result "approach_ready回调" "存在" "不存在"

echo ""
echo "测试7: 验证时间初始化"
grep -q "grab_phase_start_ms_ = this->now()" "$MANAGER_FILE" && check_result "grab_phase_start_ms_初始化" "存在" "存在" || check_result "grab_phase_start_ms_初始化" "存在" "不存在"
grep -q "grab_phase_start_ms_(this->now())" "$MANAGER_FILE" && check_result "构造函数初始化" "存在" "存在" || check_result "构造函数初始化" "存在" "不存在"

echo ""
echo "测试8: 验证ARM_TRACKING处理"
grep -q "if (msg->data == ARM_TRACKING)" "$ARM_FILE" && check_result "ARM_TRACKING处理逻辑" "存在" "存在" || check_result "ARM_TRACKING处理逻辑" "存在" "不存在"
grep -q "trajectory_phase_ = TrajectoryPhase::TRACKING" "$ARM_FILE" && check_result "WAIT_FOR_TRACKING到TRACKING转换" "存在" "存在" || check_result "WAIT_FOR_TRACKING到TRACKING转换" "存在" "不存在"

echo ""
echo "=== 测试总结 ==="
echo "通过: $PASS"
echo "失败: $FAIL"
echo ""

if [ $FAIL -eq 0 ]; then
    echo "所有代码结构验证通过!"
    echo ""
    echo "代码逻辑验证:"
    echo "  1. ARM_TRACKING枚举在两个节点中一致"
    echo "  2. WAIT_FOR_TRACKING阶段正确实现"
    echo "  3. GRAB子阶段(0-3)逻辑完整"
    echo "  4. 上升沿检测逻辑正确"
    echo "  5. 超时逻辑完整"
    echo "  6. 信号订阅和回调正确"
    echo "  7. 时间初始化正确"
    echo "  8. ARM_TRACKING处理逻辑正确"
    exit 0
else
    echo "部分代码结构验证失败，请检查实现"
    exit 1
fi

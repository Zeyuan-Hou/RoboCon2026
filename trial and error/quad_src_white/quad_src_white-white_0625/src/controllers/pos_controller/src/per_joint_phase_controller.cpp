#include "per_joint_phase_controller.hpp"
#include <cmath>
#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "InterPolate.hpp"

/* ========== 辅助函数：生成统一 PD profile 数组 ========== */
static std::array<PDProfile, 12> make_uniform_pd(PDProfile p) {
    std::array<PDProfile, 12> arr{};
    arr.fill(p);
    return arr;
}

/* ========== 构造函数 ========== */
PerJointPhaseController::PerJointPhaseController() {
    init_stride_walking_phases();
    init_jump_add_phases();
    init_small_jump_phases();
}

/* ========== 初始化 StrideWalking 相位表 ========== */
void PerJointPhaseController::init_stride_walking_phases() {
    stride_data_.phases.resize(14);
    stride_data_.phases[0]  = {static_cast<int>(StrideWalkingState::Init),    stride_target_init,    0.8,  make_uniform_pd(PD_NORMAL)};
    stride_data_.phases[1]  = {static_cast<int>(StrideWalkingState::Down1),   stride_target_down1,   0.1,  make_uniform_pd(PD_NORMAL)};
    stride_data_.phases[2]  = {static_cast<int>(StrideWalkingState::Up1),     stride_target_up1,     0.01, make_uniform_pd(PD_JUMP)};
    stride_data_.phases[3]  = {static_cast<int>(StrideWalkingState::Up12),    stride_target_up12,    0.2,  make_uniform_pd(PD_NORMAL)};
    stride_data_.phases[4]  = {static_cast<int>(StrideWalkingState::Middle),  stride_target_middle,  0.18, make_uniform_pd(PD_NORMAL)};
    stride_data_.phases[5]  = {static_cast<int>(StrideWalkingState::Reach),   stride_target_reach,   0.08, make_uniform_pd(PD_NORMAL)};
    stride_data_.phases[6]  = {static_cast<int>(StrideWalkingState::Buffer1), stride_target_buffer1, 0.12, make_uniform_pd(PD_SOFT)};
    stride_data_.phases[7]  = {static_cast<int>(StrideWalkingState::Wait),    stride_target_wait,    0.8,  make_uniform_pd(PD_NORMAL)};
    stride_data_.phases[8]  = {static_cast<int>(StrideWalkingState::Down2),   stride_target_down2,   0.1,  make_uniform_pd(PD_NORMAL)};
    stride_data_.phases[9]  = {static_cast<int>(StrideWalkingState::Up2),     stride_target_up2,     0.01, make_uniform_pd(PD_JUMP)};
    stride_data_.phases[10] = {static_cast<int>(StrideWalkingState::Up22),    stride_target_up22,    0.15, make_uniform_pd(PD_NORMAL)};
    stride_data_.phases[11] = {static_cast<int>(StrideWalkingState::Still),   stride_target_still,   0.18, make_uniform_pd(PD_NORMAL)};
    stride_data_.phases[12] = {static_cast<int>(StrideWalkingState::Done),    stride_target_done,    0.08, make_uniform_pd(PD_SOFT)};
    stride_data_.phases[13] = {static_cast<int>(StrideWalkingState::Stand),   stride_target_stand,   0.4,  make_uniform_pd(PD_NORMAL)};
}

/* ========== 初始化 JumpAdd 相位表（10相位） ========== */
void PerJointPhaseController::init_jump_add_phases() {
    // 预备阶段：左右后腿小腿 PD_SOFT，其余 PD_NORMAL
    std::array<PDProfile, 12> init_pd = {
        PD_NORMAL, PD_NORMAL, PD_NORMAL,  // leg0 (前右) hip/thigh/calf
        PD_NORMAL, PD_NORMAL, PD_NORMAL,  // leg1 (前左) hip/thigh/calf
        PD_NORMAL, PD_NORMAL, PD_SOFT,    // leg2 (后右) hip/thigh/calf
        PD_NORMAL, PD_NORMAL, PD_SOFT,    // leg3 (后左) hip/thigh/calf
    };

    // 起跳：前腿 PD_JUMP，后腿 PD_SOFT
    std::array<PDProfile, 12> jump_pd = {
        PD_JUMP, PD_JUMP, PD_JUMP,  // leg0 (前右)
        PD_JUMP, PD_JUMP, PD_JUMP,  // leg1 (前左)
        PD_SOFT, PD_SOFT, PD_SOFT,  // leg2 (后右)
        PD_SOFT, PD_SOFT, PD_SOFT,  // leg3 (后左)
    };

    jump_data_.phases.resize(13);
    // TODO 起跳
    jump_data_.phases[0] = {static_cast<int>(JumpState::Init),            jump_target_init,              0.1,  init_pd};   // 预备
    jump_data_.phases[1] = {static_cast<int>(JumpState::Jump),            jump_target_jump,              0.15,  jump_pd};   // 起跳
    jump_data_.phases[2] = {static_cast<int>(JumpState::Done),            jump_target_stand,             0.1,  jump_pd};   // 收腿
    jump_data_.phases[3] = {static_cast<int>(JumpState::Done),            jump_target_stand,             0.2,  make_uniform_pd(PD_JUMP)};   // 收腿
    // jump_data_.phases[4] = {static_cast<int>(JumpState::Done),            jump_target_stand,             1.0,  make_uniform_pd(PD_SOFT)};   // 临时暂停
    // TODO 爬墙
    jump_data_.phases[4] = {static_cast<int>(JumpState::RetractRear),     jump_target_retract_rear,      1.0,  make_uniform_pd(PD_NORMAL)};  // 收后腿
    jump_data_.phases[5] = {static_cast<int>(JumpState::SweepFront),      jump_target_sweep_front,       0.6,  make_uniform_pd(PD_NORMAL)};  // 划前腿
    jump_data_.phases[6] = {static_cast<int>(JumpState::ExtendFront),     jump_target_extend_front,      0.2,  jump_pd};  // 伸前腿
    jump_data_.phases[7] = {static_cast<int>(JumpState::ExtendHip),       jump_target_extend_hip,        1.0,  make_uniform_pd(PD_NORMAL)};  // 展髋
    jump_data_.phases[8] = {static_cast<int>(JumpState::RetractRearCalf), jump_target_retract_rear_calf, 1.0,  make_uniform_pd(PD_NORMAL)};  // 收后腿小腿
    // 伸小腿及后续：前腿 PD_SOFT，后腿 PD_NORMAL
    std::array<PDProfile, 12> rear_normal_pd = {
        PD_SOFT, PD_SOFT, PD_SOFT,        // leg0 (前右)
        PD_SOFT, PD_SOFT, PD_SOFT,        // leg1 (前左)
        PD_NORMAL, PD_NORMAL, PD_NORMAL,  // leg2 (后右)
        PD_NORMAL, PD_NORMAL, PD_NORMAL,  // leg3 (后左)
    };

    // 划后腿：前腿 SOFT，后腿小腿 STIFF
    std::array<PDProfile, 12> sweep_pd = {
        PD_NORMAL, PD_NORMAL, PD_NORMAL,        // leg0 (前右)
        PD_NORMAL, PD_NORMAL, PD_NORMAL,        // leg1 (前左)
        PD_NORMAL, PD_NORMAL, PD_STIFF,   // leg2 (后右) 小腿 STIFF
        PD_NORMAL, PD_NORMAL, PD_STIFF,   // leg3 (后左) 小腿 STIFF
    };

    jump_data_.phases[9] = {static_cast<int>(JumpState::ExtendCalf),      jump_target_extend_calf,       0.5,  rear_normal_pd};   // 伸小腿
    jump_data_.phases[10] = {static_cast<int>(JumpState::ExtendRear),      jump_target_extend_rear,       0.5,  rear_normal_pd};   // 伸后腿
    jump_data_.phases[11] = {static_cast<int>(JumpState::SweepRear),      jump_target_sweep_rear,        1.0,  sweep_pd};         // 划后腿
    jump_data_.phases[12] = {static_cast<int>(JumpState::SweepRear),      jump_recovery_stand,        1.0,  make_uniform_pd(PD_NORMAL)};         // 恢复站立
}

/* ========== 初始化 SmallJump 相位表 ========== */
void PerJointPhaseController::init_small_jump_phases() {
    small_jump_data_.phases.resize(12);
    small_jump_data_.phases[0]  = {static_cast<int>(SmallJumpState::Init),    small_jump_target_init,    0.6,  make_uniform_pd(PD_NORMAL)};
    small_jump_data_.phases[1]  = {static_cast<int>(SmallJumpState::Prepare), small_jump_target_prepare, 0.5,  make_uniform_pd(PD_NORMAL)};
    small_jump_data_.phases[2]  = {static_cast<int>(SmallJumpState::Crouch),  small_jump_target_crouch,  0.25, make_uniform_pd(PD_NORMAL)};
    small_jump_data_.phases[3]  = {static_cast<int>(SmallJumpState::Jump),    small_jump_target_jump,    0.3,  make_uniform_pd(PD_JUMP)};
    small_jump_data_.phases[4]  = {static_cast<int>(SmallJumpState::Air),     small_jump_target_air,     0.3,  make_uniform_pd(PD_NORMAL)};
    small_jump_data_.phases[5]  = {static_cast<int>(SmallJumpState::Land),    small_jump_target_land,    0.2,  make_uniform_pd(PD_SOFT)};
    small_jump_data_.phases[6]  = {static_cast<int>(SmallJumpState::Recover), small_jump_target_recover, 0.2,  make_uniform_pd(PD_SOFT)};
    small_jump_data_.phases[7]  = {static_cast<int>(SmallJumpState::Still),   small_jump_target_still,   0.2,  make_uniform_pd(PD_NORMAL)};
    small_jump_data_.phases[8]  = {static_cast<int>(SmallJumpState::Stand),   small_jump_target_stand,   0.2,  make_uniform_pd(PD_NORMAL)};
    small_jump_data_.phases[9]  = {static_cast<int>(SmallJumpState::Push),    small_jump_target_push,    0.7,  make_uniform_pd(PD_NORMAL)};
    small_jump_data_.phases[10] = {static_cast<int>(SmallJumpState::Push1),   small_jump_target_push1,   0.7,  make_uniform_pd(PD_NORMAL)};
    small_jump_data_.phases[11] = {static_cast<int>(SmallJumpState::Done),    small_jump_target_push1,   0.3,  make_uniform_pd(PD_NORMAL)};
}

/* ========== 通用相位推进逻辑 ========== */
std::array<double, 12> PerJointPhaseController::compute_cmd_from_data(
    PerJointActionData& data,
    const std::array<double, 12>& current,
    double t,
    size_t last_phase_idx) {

    if (!data.initialized) {
        data.phase_start_pos = current;
        data.phase_start_time = t;
        data.current_phase = 0;
        data.initialized = true;
    }

    std::array<double, 12> pos_des_cur{};
    auto logger = rclcpp::get_logger("PerJointPhaseController");

    const auto& cur_phase = data.phases[data.current_phase];
    double tau = t - data.phase_start_time;

    // 相位切换
    if (tau >= cur_phase.duration && cur_phase.duration > 1e-6) {
        const size_t prev_phase = data.current_phase;

        data.phase_start_time += cur_phase.duration;
        data.phase_start_pos = data.phases[prev_phase].target;

        if (data.current_phase < last_phase_idx) {
            data.current_phase += 1;
        } else {
            data.current_phase = last_phase_idx;
        }

        tau = t - data.phase_start_time;

        RCLCPP_INFO(logger, "phase switch: prev=%zu, new=%zu", prev_phase, data.current_phase);
    }

    // 插值（用当前 phase 的 target）
    const auto& new_phase = data.phases[data.current_phase];
    for (int i = 0; i < 12; ++i) {
        pos_des_cur[i] = cosineInterpolate(
            data.phase_start_pos[i],
            new_phase.target[i],
            new_phase.duration,
            tau);
    }

    return pos_des_cur;
}

/* ========== 统一接口：计算动作命令 ========== */
std::array<double, 12> PerJointPhaseController::compute_action_cmd(
    ActionType action_type,
    const std::array<double, 12>& current,
    double t) {

    switch (action_type) {
        case ActionType::STRIDE_WALKING:
            return compute_cmd_from_data(stride_data_, current, t,
                static_cast<size_t>(StrideWalkingState::Stand));
        case ActionType::JUMP_ADD:
            return compute_cmd_from_data(jump_data_, current, t,
                jump_data_.phases.size() - 1);
        case ActionType::SMALL_JUMP:
            return compute_cmd_from_data(small_jump_data_, current, t,
                static_cast<size_t>(SmallJumpState::Done));
        default:
            return current;
    }
}

/* ========== 获取当前 per-joint PD ========== */
void PerJointPhaseController::get_current_pd(
    ActionType action_type,
    std::array<double, 12>& kp_out,
    std::array<double, 12>& kd_out) const {

    const PerJointActionData& data = get_action_data(action_type);

    if (data.phases.empty() || data.current_phase >= data.phases.size()) {
        return;
    }

    const auto& phase = data.phases[data.current_phase];

    for (int i = 0; i < 12; ++i) {
        const auto& pd = PD_TABLE[phase.pd_profiles[i]][i % 3];
        kp_out[i] = pd.kp;
        kd_out[i] = pd.kd;
    }

    // 应用覆盖值（>= 0 时覆盖）
    if (override_enabled_) {
        for (int i = 0; i < 12; ++i) {
            if (override_kp_[i] >= 0.0) kp_out[i] = override_kp_[i];
            if (override_kd_[i] >= 0.0) kd_out[i] = override_kd_[i];
        }
    }
}

/* ========== 获取当前状态ID ========== */
int PerJointPhaseController::get_current_state_id(ActionType action_type) const {
    const PerJointActionData& data = get_action_data(action_type);
    return data.phases.empty() ? -1 : static_cast<int>(data.current_phase);
}

/* ========== 重置动作 ========== */
void PerJointPhaseController::reset_action(ActionType action_type) {
    PerJointActionData& data = get_action_data(action_type);
    data.initialized = false;
    data.current_phase = 0;
    data.phase_start_time = 0.0;
    for (int i = 0; i < 12; ++i) {
        data.phase_start_pos[i] = 0.0;
    }
}

/* ========== 检查是否已初始化 ========== */
bool PerJointPhaseController::is_initialized(ActionType action_type) const {
    const PerJointActionData& data = get_action_data(action_type);
    return data.initialized;
}

/* ========== 设置关节级覆盖 ========== */
void PerJointPhaseController::set_joint_overrides(
    const std::array<double, 12>& kp,
    const std::array<double, 12>& kd) {

    override_kp_ = kp;
    override_kd_ = kd;
    override_enabled_ = true;

    // 检查是否所有值都 < 0（不需要覆盖）
    bool any = false;
    for (int i = 0; i < 12; ++i) {
        if (kp[i] >= 0.0 || kd[i] >= 0.0) {
            any = true;
            break;
        }
    }
    override_enabled_ = any;
}

/* ========== 获取动作数据 ========== */
PerJointPhaseController::PerJointActionData&
PerJointPhaseController::get_action_data(ActionType action_type) {
    switch (action_type) {
        case ActionType::STRIDE_WALKING: return stride_data_;
        case ActionType::JUMP_ADD:       return jump_data_;
        case ActionType::SMALL_JUMP:     return small_jump_data_;
        default:                         return stride_data_;
    }
}

const PerJointPhaseController::PerJointActionData&
PerJointPhaseController::get_action_data(ActionType action_type) const {
    switch (action_type) {
        case ActionType::STRIDE_WALKING: return stride_data_;
        case ActionType::JUMP_ADD:       return jump_data_;
        case ActionType::SMALL_JUMP:     return small_jump_data_;
        default:                         return stride_data_;
    }
}

/* ============================================================ */
/* ========== 目标位置常量（移植自 UnifiedActionController） ========== */
/* ============================================================ */

/* ========== StrideWalking 目标位置定义 ========== */
const std::array<double, 12> PerJointPhaseController::stride_target_init = {
    0.0,  -0.17,  0.33,
    0.0,  -0.17,  0.33,
    0.0,  -0.17,  0.33,
    0.0,  -0.17,  0.33
};

const std::array<double, 12> PerJointPhaseController::stride_target_down1 = {
    0.0,  0.0,  0.1,
    0.0,  0.0,  0.1,
    0.0,  0.0,  0.1,
    0.0,  0.0,  0.1
};

const std::array<double, 12> PerJointPhaseController::stride_target_up1 = {
    0.0,  -0.0,  1.1,
    0.0,  -0.0,  1.1,
    0.0,  0.1,  1.32,
    0.0,  0.1,  1.32
};

const std::array<double, 12> PerJointPhaseController::stride_target_up12 = {
    0.0,  -0.0,  1.1,
    0.0,  -0.0,  1.1,
    0.0,  0.1,  1.32,
    0.0,  0.1,  1.32
};

const std::array<double, 12> PerJointPhaseController::stride_target_middle = {
    0.0,  -0.6,  0.7,
    0.0,  -0.6,  0.7,
    0.0,  -0.6,  0.7,
    0.0,  -0.6,  0.7
};

const std::array<double, 12> PerJointPhaseController::stride_target_reach = {
    0.0,  -1.3,  1.2,
    0.0,  -1.3,  1.2,
    0.0,  -0.9,  1.1,
    0.0,  -0.9,  1.1
};

const std::array<double, 12> PerJointPhaseController::stride_target_buffer1 = {
    0.0,  -0.6,  1.0,
    0.0,  -0.6,  1.0,
    0.0,  -0.6,  1.0,
    0.0,  -0.6,  1.0
};

const std::array<double, 12> PerJointPhaseController::stride_target_wait = {
    0.0,  -0.6,  0.7,
    0.0,  -0.6,  0.7,
    0.0,  -0.6,  0.8,
    0.0,  -0.6,  0.8
};

const std::array<double, 12> PerJointPhaseController::stride_target_down2 = {
    0.0,  -0.55,  0.65,
    0.0,  -0.55,  0.65,
    0.0,  -0.55,  0.65,
    0.0,  -0.55,  0.65
};

const std::array<double, 12> PerJointPhaseController::stride_target_up2 = {
    0.0,   -0.0,  1.0,
    0.0,   -0.0,  1.0,
    0.0,   -0.1,  1.8,
    0.0,   -0.1,  1.8
};

const std::array<double, 12> PerJointPhaseController::stride_target_up22 = {
    0.0,   -0.0,  1.0,
    0.0,   -0.0,  1.0,
    0.0,   -0.1,  1.8,
    0.0,   -0.1,  1.8
};

const std::array<double, 12> PerJointPhaseController::stride_target_still = {
    0.0,  -1.5,  0.9,
    0.0,  -1.5,  0.9,
    0.0,  -0.6,  0.7,
    0.0,  -0.6,  0.7
};

const std::array<double, 12> PerJointPhaseController::stride_target_done = {
    0.0,  -1.1,  0.7,
    0.0,  -1.1,  0.7,
    0.0,  -0.8,  1.1,
    0.0,  -0.8,  1.1
};

const std::array<double, 12> PerJointPhaseController::stride_target_stand = {
    0.0,  -0.5, 0.6,
    0.0,  -0.5, 0.6,
    0.0,  -0.5, 0.6,
    0.0,  -0.5, 0.6
};

/* ========== JumpAdd 目标位置定义 ========== */
const std::array<double, 12> PerJointPhaseController::jump_target_init = {
    0.0,  0.0,  0.0,
    0.0,  0.0,  0.0,
    0.0,  0.0,  1.4,
    0.0,  0.0,  1.4
};

const std::array<double, 12> PerJointPhaseController::jump_target_jump = {
    0.0,  0.3,  1.4,
    0.0,  0.3,  1.4,
    0.0,  0.0,  1.4,
    0.0,  0.0,  1.4
};

const std::array<double, 12> PerJointPhaseController::jump_target_stand = {
    0.0,  -0.0,  0.0,
    0.0,  -0.0,  0.0,
    0.0,  0.0,  1.4,
    0.0,  0.0,  1.4
};

const std::array<double, 12> PerJointPhaseController::jump_target_retract_rear = {
    0.0,  0.0,  0.0,
    0.0,  0.0,  0.0,
    0.0,  -0.0,  0.7,
    0.0,  -0.0,  0.7
};

const std::array<double, 12> PerJointPhaseController::jump_target_sweep_front = {
    0.0,  0.0,  1.7,
    0.0,  0.0,  1.7,
    0.0,  -0.0,  1.6,
    0.0,  -0.0,  1.6
};

const std::array<double, 12> PerJointPhaseController::jump_target_extend_front = {
    0.0,  1.9,  0.7,   // leg0 前右 伸
    0.0,  1.9,  0.7,   // leg1 前左 伸
    0.0,  -0.0,  1.6,   // leg2 后右 伸
    0.0,  -0.0,  1.6    // leg3 后左 伸
};

const std::array<double, 12> PerJointPhaseController::jump_target_extend_hip = {
    0.0,  1.9,  0.7,   // leg0 前右 伸
    0.0,  1.9,  0.7,
    0.5,  -0.0,  1.6,   // leg2 
    0.5,  -0.0,  1.6    // leg3 
};

const std::array<double, 12> PerJointPhaseController::jump_target_retract_rear_calf = {
    0.0,  1.9,  0.7,   // leg0 前右 伸
    0.0,  1.9,  0.7,
    0.5,  0.0,  0.0,   // leg2
    0.5,  0.0,  0.0    // leg
};

const std::array<double, 12> PerJointPhaseController::jump_target_extend_calf = {
    0.0,  1.9,  0.7,   // leg0 前右 伸
    0.0,  1.9,  0.7,
    0.5,  1.5,  0.0,   // leg2 后右 伸小腿
    0.5,  1.5,  0.0    // leg3 后左 伸小腿
};

const std::array<double, 12> PerJointPhaseController::jump_target_extend_rear = {
    0.0,  1.9,  0.7,   // leg0 前右 伸
    0.0,  1.9,  0.7,
    0.5,  1.5,  1.2,   // leg2 
    0.5,  1.5,  1.2    // leg
};

const std::array<double, 12> PerJointPhaseController::jump_target_sweep_rear = {
    0.0,  1.0,  0.7,   // leg0 前右 伸
    0.0,  1.0,  0.7,
    0.0,  -0.1,  1.2,   // leg2 
    0.0,  -0.1,  1.2    // leg
};

const std::array<double, 12> PerJointPhaseController::jump_recovery_stand = {
    0.1,  0.8,  1.2,   // leg0 前右 伸
    0.1,  0.8,  1.2,
    0.1,  0.8,  1.2,   // leg2 
    0.1,  0.8,  1.2    // leg
};

/* ========== SmallJump 目标位置定义 ========== */
const std::array<double, 12> PerJointPhaseController::small_jump_target_init = {
    0.0,  0.2,  0.15,
    0.0,  0.2,  0.15,
    0.0,  0.2,  0.15,
    0.0,  0.2,  0.15
};

const std::array<double, 12> PerJointPhaseController::small_jump_target_prepare = {
    0.0,  0.4,  0.2,
    0.0,  0.4,  0.2,
    0.0,  0.4,  0.2,
    0.0,  0.4,  0.2
};

const std::array<double, 12> PerJointPhaseController::small_jump_target_crouch = {
    0.0,  -0.57,  1.5,
    0.0,  -0.57,  1.5,
    0.0,  -0.17,  0.685,
    0.0,  -0.17,  0.685
};

const std::array<double, 12> PerJointPhaseController::small_jump_target_jump = {
    0.0,  -0.0,  0.0,
    0.0,  -0.0,  0.0,
    0.0,  -0.15,  1.7,
    0.0,  -0.15,  1.7
};

const std::array<double, 12> PerJointPhaseController::small_jump_target_air = {
    0.0,  -1.4,  1.0,
    0.0,  -1.4,  1.0,
    0.0,  -0.11,  1.5,
    0.0,  -0.11,  1.5
};

const std::array<double, 12> PerJointPhaseController::small_jump_target_land = {
    0.0,  -1.0,  0.8,
    0.0,  -1.0,  0.8,
    0.0,  -0.7,  0.6,
    0.0,  -0.7,  0.6
};

const std::array<double, 12> PerJointPhaseController::small_jump_target_recover = {
    0.0,  -0.9,  0.6,
    0.0,  -0.9,  0.6,
    0.0,  -0.3,  0.6,
    0.0,  -0.3,  0.6
};

const std::array<double, 12> PerJointPhaseController::small_jump_target_still = {
    0.0,  -0.8,  0.2,
    0.0,  -0.8,  0.2,
    0.7,   0.5,  0.2,
    0.7,   0.5,  0.2
};

const std::array<double, 12> PerJointPhaseController::small_jump_target_stand = {
    0.0,  -0.6,  0.6,
    0.0,  -0.6,  0.6,
    0.7,  -1.2,  0.0,
    0.7,  -1.2,  0.0
};

const std::array<double, 12> PerJointPhaseController::small_jump_target_push = {
    0.0,  -0.6,  0.6,
    0.0,  -0.6,  0.6,
    0.0,  -1.2,  0.0,
    0.0,  -1.2,  0.0
};

const std::array<double, 12> PerJointPhaseController::small_jump_target_push1 = {
    0.0,  -0.6,  0.6,
    0.0,  -0.6,  0.6,
    0.0,  -0.6,  0.6,
    0.0,  -0.6,  0.6
};

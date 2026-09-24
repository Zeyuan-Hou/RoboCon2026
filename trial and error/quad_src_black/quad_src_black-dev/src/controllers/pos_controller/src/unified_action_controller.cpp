#include "unified_action_controller.hpp"
#include <cmath>
#include <array>
#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "InterPolate.hpp"

/* ========== 构造函数：初始化所有动作的相位表 ========== */
UnifiedActionController::UnifiedActionController() {
    init_stride_walking_phases();
    init_jump_add_phases();
    init_small_jump_phases();
}

/* ========== 初始化 StrideWalking 相位表 ========== */
void UnifiedActionController::init_stride_walking_phases() {
    stride_walking_data_.phases.resize(14);
    stride_walking_data_.phases[0] = {static_cast<int>(StrideWalkingState::Init),    stride_target_init,    0.8,  PD_NORMAL};
    stride_walking_data_.phases[1] = {static_cast<int>(StrideWalkingState::Down1),  stride_target_down1,   0.1,  PD_NORMAL};
    stride_walking_data_.phases[2] = {static_cast<int>(StrideWalkingState::Up1),    stride_target_up1,     0.01, PD_JUMP};
    stride_walking_data_.phases[3] = {static_cast<int>(StrideWalkingState::Up12),   stride_target_up12,    0.2,  PD_NORMAL};
    stride_walking_data_.phases[4] = {static_cast<int>(StrideWalkingState::Middle), stride_target_middle,  0.18, PD_NORMAL};
    stride_walking_data_.phases[5] = {static_cast<int>(StrideWalkingState::Reach),   stride_target_reach,   0.08, PD_NORMAL};
    stride_walking_data_.phases[6] = {static_cast<int>(StrideWalkingState::Buffer1), stride_target_buffer1, 0.12, PD_SOFT};
    stride_walking_data_.phases[7] = {static_cast<int>(StrideWalkingState::Wait),    stride_target_wait,    0.8,  PD_NORMAL};
    stride_walking_data_.phases[8] = {static_cast<int>(StrideWalkingState::Down2),  stride_target_down2,   0.1,  PD_NORMAL};
    stride_walking_data_.phases[9] = {static_cast<int>(StrideWalkingState::Up2),    stride_target_up2,     0.01, PD_JUMP};
    stride_walking_data_.phases[10] = {static_cast<int>(StrideWalkingState::Up22),   stride_target_up22,    0.15, PD_NORMAL};
    stride_walking_data_.phases[11] = {static_cast<int>(StrideWalkingState::Still),  stride_target_still,    0.18, PD_NORMAL};
    stride_walking_data_.phases[12] = {static_cast<int>(StrideWalkingState::Done),   stride_target_done,     0.08, PD_SOFT};
    stride_walking_data_.phases[13] = {static_cast<int>(StrideWalkingState::Stand),  stride_target_stand,    0.4,  PD_NORMAL};
}
/* ========== 初始化 JumpAdd 相位表 ========== */
void UnifiedActionController::init_jump_add_phases() {
    jump_add_data_.phases.resize(15);
    jump_add_data_.phases[0] = {static_cast<int>(JumpState::Init),     jump_target_init,     0.5,  PD_NORMAL};
    jump_add_data_.phases[1] = {static_cast<int>(JumpState::Down),     jump_target_down,     0.5,  PD_NORMAL};
    jump_add_data_.phases[2] = {static_cast<int>(JumpState::Jump),     jump_target_jump,     0.25, PD_NORMAL};
    jump_add_data_.phases[3] = {static_cast<int>(JumpState::Up),       jump_target_up,       0.3,  PD_JUMP};
    jump_add_data_.phases[4] = {static_cast<int>(JumpState::Wait1),   jump_target_wait1,    0.5,  PD_NORMAL};
    jump_add_data_.phases[5] = {static_cast<int>(JumpState::Bala),     jump_target_bala,     0.2,  PD_NORMAL};
    jump_add_data_.phases[6] = {static_cast<int>(JumpState::Down2),    jump_target_down2,    0.2,  PD_SOFT};
    jump_add_data_.phases[7] = {static_cast<int>(JumpState::Wait2),   jump_target_wait2,    0.5,  PD_NORMAL};
    jump_add_data_.phases[8] = {static_cast<int>(JumpState::Pre_Frog), jump_target_pre_frog, 0.5, PD_NORMAL};
    jump_add_data_.phases[9] = {static_cast<int>(JumpState::Frog),     jump_target_frog,     0.5, PD_NORMAL};
    jump_add_data_.phases[10] = {static_cast<int>(JumpState::Push),   jump_target_push,     0.8, PD_JUMP};
    jump_add_data_.phases[11] = {static_cast<int>(JumpState::Stend),  jump_target_stend,    0.3,  PD_NORMAL};
    jump_add_data_.phases[12] = {static_cast<int>(JumpState::Stend1),  jump_target_stend1,   0.2,  PD_SOFT};
    jump_add_data_.phases[13] = {static_cast<int>(JumpState::Stand),   jump_target_stand,    0.3,  PD_SOFT};
    jump_add_data_.phases[14] = {static_cast<int>(JumpState::Done),   jump_target_stand,    0.2,  PD_NORMAL};
}
// /* ========== 初始化 JumpAdd 相位表 ========== */
// void UnifiedActionController::init_jump_add_phases() {
//     jump_add_data_.phases.resize(15);
//     jump_add_data_.phases[0] = {static_cast<int>(JumpState::Init),     jump_target_init,     0.6,  PD_NORMAL};
//     jump_add_data_.phases[1] = {static_cast<int>(JumpState::Down),     jump_target_down,     0.5,  PD_NORMAL};
//     jump_add_data_.phases[2] = {static_cast<int>(JumpState::Jump),     jump_target_jump,     0.25, PD_NORMAL};
//     jump_add_data_.phases[3] = {static_cast<int>(JumpState::Up),       jump_target_up,       0.3,  PD_JUMP};
//     jump_add_data_.phases[4] = {static_cast<int>(JumpState::Wait1),   jump_target_wait1,    0.3,  PD_NORMAL};
//     jump_add_data_.phases[5] = {static_cast<int>(JumpState::Bala),     jump_target_bala,     0.2,  PD_NORMAL};
//     jump_add_data_.phases[6] = {static_cast<int>(JumpState::Down2),    jump_target_down2,    0.2,  PD_SOFT};
//     jump_add_data_.phases[7] = {static_cast<int>(JumpState::Wait2),   jump_target_wait2,    0.2,  PD_NORMAL};
//     jump_add_data_.phases[8] = {static_cast<int>(JumpState::Pre_Frog), jump_target_pre_frog, 0.12, PD_NORMAL};
//     jump_add_data_.phases[9] = {static_cast<int>(JumpState::Frog),     jump_target_frog,     0.12, PD_NORMAL};
//     jump_add_data_.phases[10] = {static_cast<int>(JumpState::Push),   jump_target_push,     0.5, PD_JUMP};
//     jump_add_data_.phases[11] = {static_cast<int>(JumpState::Stend),  jump_target_stend,    0.3,  PD_NORMAL};
//     jump_add_data_.phases[12] = {static_cast<int>(JumpState::Stend1),  jump_target_stend1,   0.2,  PD_SOFT};
//     jump_add_data_.phases[13] = {static_cast<int>(JumpState::Stand),   jump_target_stand,    0.3,  PD_SOFT};
//     jump_add_data_.phases[14] = {static_cast<int>(JumpState::Done),   jump_target_stand,    0.5,  PD_NORMAL};
// }

// /* ========== 初始化 SmallJump 相位表（8步骤） ========== */
void UnifiedActionController::init_small_jump_phases() {
    small_jump_data_.phases.resize(12);  // 0-8: Init, Prepare, Crouch, Jump, Air, Land, Recover, Stand, Done
    small_jump_data_.phases[0] = {static_cast<int>(SmallJumpState::Init),    small_jump_target_init,    0.8,  PD_NORMAL};
    small_jump_data_.phases[1] = {static_cast<int>(SmallJumpState::Prepare),  small_jump_target_prepare, 0.4,  PD_NORMAL};
    small_jump_data_.phases[2] = {static_cast<int>(SmallJumpState::Crouch),  small_jump_target_crouch,  0.1, PD_JUMP};
    small_jump_data_.phases[3] = {static_cast<int>(SmallJumpState::Jump),    small_jump_target_jump,    0.15,  PD_NORMAL};
    small_jump_data_.phases[4] = {static_cast<int>(SmallJumpState::Air),     small_jump_target_air,     0.1, PD_NORMAL};
    small_jump_data_.phases[5] = {static_cast<int>(SmallJumpState::Land),    small_jump_target_land,    0.08, PD_NORMAL};
    small_jump_data_.phases[6] = {static_cast<int>(SmallJumpState::Recover), small_jump_target_recover, 0.08, PD_NORMAL};
    small_jump_data_.phases[7] = {static_cast<int>(SmallJumpState::Still),   small_jump_target_still,    0.8,  PD_SOFT};
    small_jump_data_.phases[8] = {static_cast<int>(SmallJumpState::Stand),   small_jump_target_stand,    0.8,  PD_SOFT};
    small_jump_data_.phases[9] = {static_cast<int>(SmallJumpState::Push),   small_jump_target_push,    0.2,  PD_SOFT};
    small_jump_data_.phases[10] = {static_cast<int>(SmallJumpState::Push1),   small_jump_target_push1,    0.2,  PD_NORMAL};

    small_jump_data_.phases[11] = {static_cast<int>(SmallJumpState::Done),   small_jump_target_push1,    0.3,   PD_NORMAL};
}
// /* ========== 初始化 SmallJump 相位表（8步骤） ========== */
// void UnifiedActionController::init_small_jump_phases() {
//     small_jump_data_.phases.resize(12);  // 0-8: Init, Prepare, Crouch, Jump, Air, Land, Recover, Stand, Done
//     small_jump_data_.phases[0] = {static_cast<int>(SmallJumpState::Init),    small_jump_target_init,    0.6,  PD_NORMAL};
//     small_jump_data_.phases[1] = {static_cast<int>(SmallJumpState::Prepare),  small_jump_target_prepare, 0.5,  PD_NORMAL};
//     small_jump_data_.phases[2] = {static_cast<int>(SmallJumpState::Crouch),  small_jump_target_crouch,  0.25, PD_NORMAL};
//     small_jump_data_.phases[3] = {static_cast<int>(SmallJumpState::Jump),    small_jump_target_jump,    0.3,  PD_JUMP};
//     small_jump_data_.phases[4] = {static_cast<int>(SmallJumpState::Air),     small_jump_target_air,     0.3,  PD_NORMAL};
//     small_jump_data_.phases[5] = {static_cast<int>(SmallJumpState::Land),    small_jump_target_land,    0.2,  PD_SOFT};
//     small_jump_data_.phases[6] = {static_cast<int>(SmallJumpState::Recover), small_jump_target_recover, 0.2,  PD_SOFT};
//     small_jump_data_.phases[7] = {static_cast<int>(SmallJumpState::Still),   small_jump_target_still,    0.2,  PD_NORMAL};
//     small_jump_data_.phases[8] = {static_cast<int>(SmallJumpState::Stand),   small_jump_target_stand,    0.2,  PD_NORMAL};
//     small_jump_data_.phases[9] = {static_cast<int>(SmallJumpState::Push),   small_jump_target_push,    0.7,  PD_NORMAL};
//     small_jump_data_.phases[10] = {static_cast<int>(SmallJumpState::Push1),   small_jump_target_push1,    0.7,  PD_NORMAL};

//     small_jump_data_.phases[11] = {static_cast<int>(SmallJumpState::Done),   small_jump_target_push1,    0.3,   PD_NORMAL};
// }

/* ========== 统一接口：计算动作命令 ========== */
std::array<double, 12> UnifiedActionController::compute_action_cmd(
    ActionType action_type,
    const std::array<double, 12>& current,
    double t) {
    
    switch (action_type) {
        case ActionType::STRIDE_WALKING:
            return compute_stride_walking_cmd(current, t);
        case ActionType::JUMP_ADD:
            return compute_jump_add_cmd(current, t);
        case ActionType::SMALL_JUMP:
            return compute_small_jump_cmd(current, t);
        default:
            return current;
    }
}

/* ========== 计算 StrideWalking 命令 ========== */
std::array<double, 12> UnifiedActionController::compute_stride_walking_cmd(
    const std::array<double, 12>& current,
    double t) {
    
    ActionData& data = stride_walking_data_;
    
    if (!data.initialized) {
        data.phase_start_pos = current;
        data.phase_start_time = t;
        data.current_phase = 0;
        data.initialized = true;
    }
    
    std::array<double, 12> pos_des_cur{};
    auto logger = rclcpp::get_logger("UnifiedActionController");
    
    const auto& phase = data.phases[data.current_phase];
    double tau = t - data.phase_start_time;
    
    RCLCPP_INFO_THROTTLE(
        logger,
        *rclcpp::Clock::make_shared(),
        1000,
        "StrideWalking: phase=%zu, tau=%.3f, duration=%.3f",
        data.current_phase,
        tau,
        phase.duration
    );
    
    if (tau >= phase.duration && phase.duration > 1e-6) {
        const size_t prev_phase = data.current_phase;
        const size_t last_phase = static_cast<size_t>(StrideWalkingState::Stand);

        data.phase_start_time += phase.duration;
        data.phase_start_pos = data.phases[prev_phase].target;

        if (data.current_phase < last_phase) {
            data.current_phase += 1;
        } else {
            data.current_phase = last_phase;
        }

        RCLCPP_INFO(
            logger,
            "StrideWalking phase switch: prev=%zu, new=%zu",
            prev_phase,
            data.current_phase
        );
    }

    for (int i = 0; i < 12; ++i) {
        pos_des_cur[i] = cosineInterpolate(
            data.phase_start_pos[i],
            phase.target[i],
            phase.duration,
            tau
        );
    }

    return pos_des_cur;
}

/* ========== 计算 JumpAdd 命令 ========== */
std::array<double, 12> UnifiedActionController::compute_jump_add_cmd(
    const std::array<double, 12>& current,
    double t) {
    
    ActionData& data = jump_add_data_;
    
    if (!data.initialized) {
        data.phase_start_pos = current;
        data.phase_start_time = t;
        data.current_phase = 0;
        data.initialized = true;
    }
    
    std::array<double, 12> pos_des_cur{};
    auto logger = rclcpp::get_logger("UnifiedActionController");
    
    const auto& phase = data.phases[data.current_phase];
    double tau = t - data.phase_start_time;
    
    RCLCPP_INFO_THROTTLE(
        logger,
        *rclcpp::Clock::make_shared(),
        1000,
        "JumpAdd: phase=%zu, tau=%.3f, duration=%.3f",
        data.current_phase,
        tau,
        phase.duration
    );
    
    if (tau >= phase.duration && phase.duration > 1e-6) {
        const size_t prev_phase = data.current_phase;
        
        data.phase_start_time += phase.duration;
        data.phase_start_pos = data.phases[prev_phase].target;
        
        ++data.current_phase;
        if (data.current_phase > static_cast<size_t>(JumpState::Done)) {
            data.current_phase = static_cast<size_t>(JumpState::Done);
        }
        
        RCLCPP_INFO(
            logger,
            "JumpAdd phase switch: prev=%zu, new=%zu",
            prev_phase,
            data.current_phase
        );
    }
    
    for (int i = 0; i < 12; ++i) {
        pos_des_cur[i] = cosineInterpolate(
            data.phase_start_pos[i],
            phase.target[i],
            phase.duration,
            tau
        );
    }
    
    return pos_des_cur;
}

/* ========== 计算 SmallJump 命令 ========== */
std::array<double, 12> UnifiedActionController::compute_small_jump_cmd(
    const std::array<double, 12>& current,
    double t) {
    
    ActionData& data = small_jump_data_;
    
    if (!data.initialized) {
        data.phase_start_pos = current;
        data.phase_start_time = t;
        data.current_phase = 0;
        data.initialized = true;
    }
    
    std::array<double, 12> pos_des_cur{};
    auto logger = rclcpp::get_logger("UnifiedActionController");
    
    const auto& phase = data.phases[data.current_phase];
    double tau = t - data.phase_start_time;
    
    RCLCPP_INFO_THROTTLE(
        logger,
        *rclcpp::Clock::make_shared(),
        1000,
        "SmallJump: phase=%zu, tau=%.3f, duration=%.3f",
        data.current_phase,
        tau,
        phase.duration
    );
    
    if (tau >= phase.duration && phase.duration > 1e-6) {
        const size_t prev_phase = data.current_phase;
        const size_t last_phase = static_cast<size_t>(SmallJumpState::Done);
        
        data.phase_start_time += phase.duration;
        data.phase_start_pos = data.phases[prev_phase].target;
        
        if (data.current_phase < last_phase) {
            // 正常 phase 前进
            data.current_phase += 1;
        } 
        else {
            data.current_phase = last_phase;
        }
        // else {
        //     // 已到 Done
        //     data.loop_count += 1;

        //     if (data.loop_count < data.max_loop) {
        //         // 重新开始一轮 SmallJump
        //         data.current_phase = 0;
        //         data.phase_start_pos = current;   // 防止位置突变
        //     } else {
        //         // 真正结束，停在 Done
        //         data.current_phase = last_phase;
        //     }
        // }
        
        RCLCPP_INFO(
            logger,
            "SmallJump phase switch: prev=%zu, new=%zu",
            prev_phase,
            data.current_phase
        );
    }
    
    for (int i = 0; i < 12; ++i) {
        pos_des_cur[i] = cosineInterpolate(
            data.phase_start_pos[i],
            phase.target[i],
            phase.duration,
            tau
        );
    }
    
    return pos_des_cur;
}

/* ========== 获取当前状态ID ========== */
int UnifiedActionController::get_current_state_id(ActionType action_type) const {
    const ActionData& data = get_action_data(action_type);
    return data.phases.empty() ? -1 : static_cast<int>(data.current_phase);
}

/* ========== 获取当前PD参数 ========== */
void UnifiedActionController::get_current_pd(ActionType action_type, double pd_out[6]) const {
    const ActionData& data = get_action_data(action_type);
    
    if (data.phases.empty() || data.current_phase >= data.phases.size()) {
        return;
    }
    
    const auto& phase = data.phases[data.current_phase];
    const auto& pd = PD_TABLE[phase.pd_id];
    
    pd_out[0] = pd[HIP].kp;
    pd_out[1] = pd[THIGH].kp;
    pd_out[2] = pd[CALF].kp;
    pd_out[3] = pd[HIP].kd;
    pd_out[4] = pd[THIGH].kd;
    pd_out[5] = pd[CALF].kd;
}

/* ========== 重置动作 ========== */
void UnifiedActionController::reset_action(ActionType action_type) {
    ActionData& data = get_action_data(action_type);
    data.initialized = false;
    data.current_phase = 0;
    data.phase_start_time = 0.0;
    for (int i = 0; i < 12; ++i) {
        data.phase_start_pos[i] = 0.0;
    }
}

/* ========== 检查是否已初始化 ========== */
bool UnifiedActionController::is_initialized(ActionType action_type) const {
    const ActionData& data = get_action_data(action_type);
    return data.initialized;
}

/* ========== 获取动作数据（非const版本） ========== */
UnifiedActionController::ActionData& UnifiedActionController::get_action_data(ActionType action_type) {
    switch (action_type) {
        case ActionType::STRIDE_WALKING:
            return stride_walking_data_;
        case ActionType::JUMP_ADD:
            return jump_add_data_;
        case ActionType::SMALL_JUMP:
            return small_jump_data_;
        default:
            return stride_walking_data_;
    }
}

/* ========== 获取动作数据（const版本） ========== */
const UnifiedActionController::ActionData& UnifiedActionController::get_action_data(ActionType action_type) const {
    switch (action_type) {
        case ActionType::STRIDE_WALKING:
            return stride_walking_data_;
        case ActionType::JUMP_ADD:
            return jump_add_data_;
        case ActionType::SMALL_JUMP:
            return small_jump_data_;
        default:
            return stride_walking_data_;
    }
}

/* ========== StrideWalking 目标位置定义 ========== */
const std::array<double, 12> UnifiedActionController::stride_target_init = {
    0.0,  -0.17,  0.33,  
    0.0,  -0.17,  0.33,  
    0.0,  -0.17,  0.33,  
    0.0,  -0.17,  0.33
};

const std::array<double, 12> UnifiedActionController::stride_target_down1 = {
    0.0,  0.0,  0.1,  
    0.0,  0.0,  0.1,   
    0.0,  0.0,  0.1,    
    0.0,  0.0,  0.1
};

const std::array<double, 12> UnifiedActionController::stride_target_up1 = {
    0.0,  -0.0,  1.1,  
    0.0,  -0.0,  1.1,   
    0.0,  0.1,  1.32,      
    0.0,  0.1,  1.32
};

const std::array<double, 12> UnifiedActionController::stride_target_up12 = {
    0.0,  -0.0,  1.1,  
    0.0,  -0.0,  1.1,   
    0.0,  0.1,  1.32,      
    0.0,  0.1,  1.32
};

const std::array<double, 12> UnifiedActionController::stride_target_middle = {
    0.0,  -0.6,  0.7,  
    0.0,  -0.6,  0.7,   
    0.0,  -0.6,  0.7,   
    0.0,  -0.6,  0.7
};

const std::array<double, 12> UnifiedActionController::stride_target_reach = {
    0.0,  -1.3,  1.2,  
    0.0,  -1.3,  1.2,     
    0.0,  -0.9,  1.1,   
    0.0,  -0.9,  1.1
};

const std::array<double, 12> UnifiedActionController::stride_target_buffer1 = {
    0.0,  -0.6,  1.0,  
    0.0,  -0.6,  1.0,   
    0.0,  -0.6,  1.0,   
    0.0,  -0.6,  1.0
};

const std::array<double, 12> UnifiedActionController::stride_target_wait = {
    0.0,  -0.6,  0.7,  
    0.0,  -0.6,  0.7,   
    0.0,  -0.6,  0.8,   
    0.0,  -0.6,  0.8
};

const std::array<double, 12> UnifiedActionController::stride_target_down2 = {
    0.0,  -0.55,  0.65,  
    0.0,  -0.55,  0.65,   
    0.0,  -0.55,  0.65,   
    0.0,  -0.55,  0.65
};

const std::array<double, 12> UnifiedActionController::stride_target_up2 = {
    0.0,   -0.0,  1.0,  
    0.0,   -0.0,  1.0,   
    0.0,   -0.1,  1.8,   
    0.0,   -0.1,  1.8
};

const std::array<double, 12> UnifiedActionController::stride_target_up22 = {
    0.0,   -0.0,  1.0,  
    0.0,   -0.0,  1.0,   
    0.0,   -0.1,  1.8,   
    0.0,   -0.1,  1.8
};

const std::array<double, 12> UnifiedActionController::stride_target_still = {
    0.0,  -1.5,  0.9,  
    0.0,  -1.5,  0.9,   
    0.0,  -0.6,  0.7,   
    0.0,  -0.6,  0.7
};

const std::array<double, 12> UnifiedActionController::stride_target_done = {
    0.0,  -1.1,  0.7,  
    0.0,  -1.1,  0.7,   
    0.0,  -0.8,  1.1,   
    0.0,  -0.8,  1.1
};

const std::array<double, 12> UnifiedActionController::stride_target_stand = {
    0.0,  -0.5, 0.6,  
    0.0,  -0.5, 0.6,  
    0.0,  -0.5, 0.6,    
    0.0,  -0.5, 0.6 
};

/* ========== JumpAdd 目标位置定义 ========== */
const std::array<double, 12> UnifiedActionController::jump_target_init = {
    0.0,  0.2,  0.15,
    0.0,  0.2,  0.15,
    0.0,  0.2,  0.15,
    0.0,  0.2,  0.15
};

const std::array<double, 12> UnifiedActionController::jump_target_down = {
    0.0,  0.4,  0.2,
    0.0,  0.4,  0.2,
    0.0,  0.4,  0.2,
    0.0,  0.4,  0.2
};

const std::array<double, 12> UnifiedActionController::jump_target_jump = {
    0.0,  -0.57,  1.55,
    0.0,  -0.57,  1.55,
    0.0,  -0.17,  0.685,
    0.0,  -0.17,  0.685
};

const std::array<double, 12> UnifiedActionController::jump_target_up = {
    0.0,  -0.0,  0.0,
    0.0,  -0.0,  0.0,
    0.0,  -0.15,  1.3,
    0.0,  -0.15,  1.3
};

const std::array<double, 12> UnifiedActionController::jump_target_wait1 = {
    0.0,  -0.5,  0.0,
    0.0,  -0.5,  0.0,
    0.0,  -0.11,  1.3,
    0.0,  -0.11,  1.3
};

const std::array<double, 12> UnifiedActionController::jump_target_bala = {
    0.0,  -0.0,  1.4,
    0.0,  -0.0,  1.4,
    0.0,  -0.11,  1.8,
    0.0,  -0.11,  1.8
};

const std::array<double, 12> UnifiedActionController::jump_target_down2 = {
    0.0,  -2.3,   1.5,
    0.0,  -2.3,   1.5,
    0.0,  -0.11,  1.6,
    0.0,  -0.11,  1.6
};

const std::array<double, 12> UnifiedActionController::jump_target_wait2 = {
    0.0,  -2.3,   1.5,
    0.0,  -2.3,   1.5,
    0.7,  -0.11,  1.5,
    0.7,  -0.11,  1.5
};

const std::array<double, 12> UnifiedActionController::jump_target_pre_frog = {
    0.0,  -2.3,   1.5,
    0.0,  -2.3,   1.5,
    0.7,  -0.3,   0,
    0.7,  -0.3,   0
};

const std::array<double, 12> UnifiedActionController::jump_target_frog = {
    0.0,  -2.3,   1.6,
    0.0,  -2.3,   1.6,
    0.7,  -1.6,   0,
    0.7,  -1.6,   0
};

const std::array<double, 12> UnifiedActionController::jump_target_push = {
    0.2,  -1.9,   1.7,
    0.2,  -1.9,   1.7, 
    0.0,  -1.2,   1.3,
    0.0,  -1.2,   1.3//1.8
};
const std::array<double, 12> UnifiedActionController::jump_target_stend = {
    0.2,  -1.8, 1.3,  
    0.2,  -1.8, 1.3,  
    0.0,  -1.2,   1.3,
    0.0,  -1.2,   1.3//1.8
};

const std::array<double, 12> UnifiedActionController::jump_target_stend1 = {
    0.2,  -1.7, 1.3,  
    0.2,  -1.7, 1.3,  
    0.0,  -0.9, 0.9,  
    0.0,  -0.9, 0.9  
};

const std::array<double, 12> UnifiedActionController::jump_target_stand = {
    0.1, -1.0, 1.4,
    0.1, -1.0, 1.4,
    0.1, -0.8, 1.45,
    0.1, -0.8, 1.45
};


// const std::array<double, 12> UnifiedActionController::jump_target_stend = {
//     0.0,  -1.8, 1.3,  
//     0.0,  -1.8, 1.3,  
//     0.0,  -0.9, 0.9,  
//     0.0,  -0.9, 0.9  
// };

// const std::array<double, 12> UnifiedActionController::jump_target_stend1 = {
//     0.0,  -1.7, 1.3,  
//     0.0,  -1.7, 1.3,  
//     0.0,  -0.9, 0.9,  
//     0.0,  -0.9, 0.9  
// };

// const std::array<double, 12> UnifiedActionController::jump_target_stand = {
//     0.0,  -0.8, 0.7,  
//     0.0,  -0.8, 0.7,  
//     0.0,  -0.3, 0.7,  
//     0.0,  -0.3, 0.7  
// };

/* ========== SmallJump 目标位置定义（最远距离） ========== */
// TODO: 用户需要填写这些参数
const std::array<double, 12> UnifiedActionController::small_jump_target_init = {
    0.0,  -0.17,  0.33,  
    0.0,  -0.17,  0.33,  
    0.0,  -0.17,  0.33,  
    0.0,  -0.17,  0.33
};

const std::array<double, 12> UnifiedActionController::small_jump_target_prepare = {
    0.0,  -0.35,  0.0,  
    0.0,  -0.35,  0.0,   
    0.0,  -0.35,  0.0,    
    0.0,  -0.35,  0.0
};

const std::array<double, 12> UnifiedActionController::small_jump_target_crouch = {
    0.0,  -0.35,  0.45,  
    0.0,  -0.35,  0.45,   
    0.0,  -0.35,  2.0,    
    0.0,  -0.35,  2.0
};

const std::array<double, 12> UnifiedActionController::small_jump_target_jump = {
    0.0,  -0.35,  0.45,  
    0.0,  -0.35,  0.45,   
    0.0,  -0.35,  2.0,      
    0.0,  -0.35,  2.0
};

// const std::array<double, 12> UnifiedActionController::small_jump_target_prepare = {
//     0.0,  -0.0,  0.1,  
//     0.0,  -0.0,  0.1,   
//     0.0,  -0.0,  0.1,    
//     0.0,  -0.0,  0.1
// };

// const std::array<double, 12> UnifiedActionController::small_jump_target_crouch = {
//     0.0,  -0.0,  1.0,  
//     0.0,  -0.0,  1.0,   
//     0.0,  -0.0,  2.0,    
//     0.0,  -0.0,  2.0
// };

// const std::array<double, 12> UnifiedActionController::small_jump_target_jump = {
//     0.0,  -0.0,  1.0,  
//     0.0,  -0.0,  1.0,   
//     0.0,  -0.0,  2.0,      
//     0.0,  -0.0,  2.0
// };

const std::array<double, 12> UnifiedActionController::small_jump_target_air = {
    0.0,  -0.7,  0.8,  
    0.0,  -0.7,  0.8,   
    0.0,  -0.7,  0.8,   
    0.0,  -0.7,  0.8
};

const std::array<double, 12> UnifiedActionController::small_jump_target_land = {
    0.0,  -0.7,  0.8,  
    0.0,  -0.7,  0.8,   
    0.0,  -0.7,  0.8,   
    0.0,  -0.7,  0.8
};

const std::array<double, 12> UnifiedActionController::small_jump_target_recover = {
    0.0,  -0.7,  0.8,  
    0.0,  -0.7,  0.8,   
    0.0,  -0.7,  0.8,   
    0.0,  -0.7,  0.8
};

const std::array<double, 12> UnifiedActionController::small_jump_target_still = {
    0.0,  -0.4,  0.6,  
    0.0,  -0.4,  0.6,   
    0.0,  -0.3,  0.6,   
    0.0,  -0.3,  0.6
};

const std::array<double, 12> UnifiedActionController::small_jump_target_stand = {
    0.0,  -0.4,  0.6,  
    0.0,  -0.4,  0.6,   
    0.0,  -0.3,  0.6,   
    0.0,  -0.3,  0.6
};

const std::array<double, 12> UnifiedActionController::small_jump_target_push = {
    0.0,  -0.4,  0.6,  
    0.0,  -0.4,  0.6,   
    0.0,  -0.3,  0.6,   
    0.0,  -0.3,  0.6
};

const std::array<double, 12> UnifiedActionController::small_jump_target_push1 = {
    0.0,  -0.4,  0.6,  
    0.0,  -0.4,  0.6,   
    0.0,  -0.3,  0.6,   
    0.0,  -0.3,  0.6
};
/* ========== SmallJump 目标位置定义（8步骤，参数待用户填写） 可以跳过15cm========== */
// // TODO: 用户需要填写这些参数
// const std::array<double, 12> UnifiedActionController::small_jump_target_init = {
//     0.0,  -0.17,  0.33,  
//     0.0,  -0.17,  0.33,  
//     0.0,  -0.17,  0.33,  
//     0.0,  -0.17,  0.33
// };

// const std::array<double, 12> UnifiedActionController::small_jump_target_prepare = {
//     0.0,  -0.2,  0.1,  
//     0.0,  -0.2,  0.1,   
//     0.0,  -0.2,  0.1,    
//     0.0,  -0.2,  0.1
// };

// const std::array<double, 12> UnifiedActionController::small_jump_target_crouch = {
//     0.0,  -0.4,  1.0,  
//     0.0,  -0.4,  1.0,   
//     0.0,  -0.2,  2.0,    
//     0.0,  -0.2,  2.0
// };

// const std::array<double, 12> UnifiedActionController::small_jump_target_jump = {
//     0.0,  -0.4,  1.0,  
//     0.0,  -0.4,  1.0,   
//     0.0,  -0.2,  2.0,      
//     0.0,  -0.2,  2.0
// };

// const std::array<double, 12> UnifiedActionController::small_jump_target_air = {
//     0.0,  -1.6,  1.7,  
//     0.0,  -1.6,  1.7,   
//     0.0,  -0.2,  2.0,      
//     0.0,  -0.2,  2.0
// };

// const std::array<double, 12> UnifiedActionController::small_jump_target_land = {
//     0.0,  -1.5,  1.3,  
//     0.0,  -1.5,  1.3,  
//     0.0,  -0.2,  2.0,      
//     0.0,  -0.2,  2.0
// };

// const std::array<double, 12> UnifiedActionController::small_jump_target_recover = {
//     0.0,  -1.0,  0.8,  
//     0.0,  -1.0,  0.8,    
//     0.0,  -0.7,  0.6,   
//     0.0,  -0.7,  0.6
// };

// const std::array<double, 12> UnifiedActionController::small_jump_target_still = {
//     0.0,  -0.4,  0.6,  
//     0.0,  -0.4,  0.6,   
//     0.0,  -0.3,  0.6,   
//     0.0,  -0.3,  0.6
// };

// const std::array<double, 12> UnifiedActionController::small_jump_target_stand = {
//     0.0,  -0.4,  0.6,  
//     0.0,  -0.4,  0.6,   
//     0.0,  -0.3,  0.6,   
//     0.0,  -0.3,  0.6
// };
// /* ========== SmallJump 目标位置定义（15cm全过程可行） ========== */
// // TODO: 用户需要填写这些参数
// const std::array<double, 12> UnifiedActionController::small_jump_target_init = {
//     0.0,  0.2,  0.15,
//     0.0,  0.2,  0.15,
//     0.0,  0.2,  0.15,
//     0.0,  0.2,  0.15
// };

// const std::array<double, 12> UnifiedActionController::small_jump_target_prepare = {
//     0.0,  0.4,  0.2,
//     0.0,  0.4,  0.2,
//     0.0,  0.4,  0.2,
//     0.0,  0.4,  0.2
// };

// const std::array<double, 12> UnifiedActionController::small_jump_target_crouch = {
//     0.0,  -0.57,  1.5,
//     0.0,  -0.57,  1.5,
//     0.0,  -0.17,  0.685,
//     0.0,  -0.17,  0.685
// };

// const std::array<double, 12> UnifiedActionController::small_jump_target_jump = {
//     0.0,  -0.0,  0.0,
//     0.0,  -0.0,  0.0,
//     0.0,  -0.15,  1.7,
//     0.0,  -0.15,  1.7
// };

// const std::array<double, 12> UnifiedActionController::small_jump_target_air = {
//     0.0,  -1.4,  1.0,
//     0.0,  -1.4,  1.0,
//     0.0,  -0.11,  1.5,
//     0.0,  -0.11,  1.5
// };

// const std::array<double, 12> UnifiedActionController::small_jump_target_land = {
//     0.0,  -1.0,  0.8,  
//     0.0,  -1.0,  0.8,    
//     0.0,  -0.7,  0.6,   
//     0.0,  -0.7,  0.6
// };

// const std::array<double, 12> UnifiedActionController::small_jump_target_recover = {
//     0.0,  -0.9,  0.6,  
//     0.0,  -0.9,  0.6,   
//     0.0,  -0.3,  0.6,   
//     0.0,  -0.3,  0.6
// };

// const std::array<double, 12> UnifiedActionController::small_jump_target_still = {
//     0.0,  -0.8,  0.2,  
//     0.0,  -0.8,  0.2,   
//     0.7,   0.5,  0.2,   
//     0.7,   0.5,  0.2
// };

// const std::array<double, 12> UnifiedActionController::small_jump_target_stand = {
//     0.0,  -0.6,  0.6,  
//     0.0,  -0.6,  0.6,   
//     0.7,  -1.2,  0.0,   
//     0.7,  -1.2,  0.0
// };

// const std::array<double, 12> UnifiedActionController::small_jump_target_push = {
//     0.0,  -0.6,  0.6,  
//     0.0,  -0.6,  0.6,   
//     0.0,  -1.2,  0.0,   
//     0.0,  -1.2,  0.0
// };

// const std::array<double, 12> UnifiedActionController::small_jump_target_push1 = {
//     0.0,  -0.6,  0.6,  
//     0.0,  -0.6,  0.6,   
//     0.0,  -0.6,  0.6,   
//     0.0,  -0.6,  0.6
// };


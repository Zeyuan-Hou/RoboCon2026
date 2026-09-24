#ifndef PER_JOINT_PHASE_CONTROLLER_HPP
#define PER_JOINT_PHASE_CONTROLLER_HPP

#include <array>
#include <vector>
#include <cstddef>
#include "pd_type.hpp"
#include "unified_action_controller.hpp"

// 每个 phase 的每个关节都有独立 PD 配置
struct PerJointPhaseConfig {
    int state_id;
    std::array<double, 12> target;
    double duration;
    std::array<PDProfile, 12> pd_profiles;  // 12 个关节各自的 PD profile
};

class PerJointPhaseController {
public:
    // 运行期数据（需在 private 前声明以便 compute_cmd_from_data 使用）
    struct PerJointActionData {
        std::vector<PerJointPhaseConfig> phases;
        size_t current_phase = 0;
        size_t loop_count = 0;
        size_t max_loop = 7;
        double phase_start_time = 0.0;
        std::array<double, 12> phase_start_pos{};
        bool initialized = false;
    };

    PerJointPhaseController();

    // 计算动作命令
    std::array<double, 12> compute_action_cmd(
        ActionType action_type,
        const std::array<double, 12>& current,
        double t);

    // 获取当前状态ID
    int get_current_state_id(ActionType action_type) const;

    // 获取当前 per-joint PD（12 kp + 12 kd）
    void get_current_pd(ActionType action_type,
                        std::array<double, 12>& kp_out,
                        std::array<double, 12>& kd_out) const;

    // 重置动作
    void reset_action(ActionType action_type);

    // 检查是否已初始化
    bool is_initialized(ActionType action_type) const;

    // 设置关节级 PD 覆盖（值 < 0 表示不覆盖）
    void set_joint_overrides(const std::array<double, 12>& kp,
                             const std::array<double, 12>& kd);

private:
    void init_stride_walking_phases();
    void init_jump_add_phases();
    void init_small_jump_phases();

    std::array<double, 12> compute_cmd_from_data(
        PerJointActionData& data,
        const std::array<double, 12>& current,
        double t,
        size_t last_phase_idx);

    // 目标位置常量（移植自 UnifiedActionController）
    static const std::array<double, 12> stride_target_init;
    static const std::array<double, 12> stride_target_down1;
    static const std::array<double, 12> stride_target_up1;
    static const std::array<double, 12> stride_target_up12;
    static const std::array<double, 12> stride_target_middle;
    static const std::array<double, 12> stride_target_reach;
    static const std::array<double, 12> stride_target_buffer1;
    static const std::array<double, 12> stride_target_wait;
    static const std::array<double, 12> stride_target_down2;
    static const std::array<double, 12> stride_target_up2;
    static const std::array<double, 12> stride_target_up22;
    static const std::array<double, 12> stride_target_still;
    static const std::array<double, 12> stride_target_done;
    static const std::array<double, 12> stride_target_stand;

    static const std::array<double, 12> jump_target_init;
    static const std::array<double, 12> jump_target_down;
    static const std::array<double, 12> jump_target_jump;
    static const std::array<double, 12> jump_target_up;
    static const std::array<double, 12> jump_target_wait1;
    static const std::array<double, 12> jump_target_bala;
    static const std::array<double, 12> jump_target_down2;
    static const std::array<double, 12> jump_target_wait2;
    static const std::array<double, 12> jump_target_pre_frog;
    static const std::array<double, 12> jump_target_frog;
    static const std::array<double, 12> jump_target_push;
    static const std::array<double, 12> jump_target_stend;
    static const std::array<double, 12> jump_target_stend1;
    static const std::array<double, 12> jump_target_stand;

    static const std::array<double, 12> jump_target_retract_rear;
    static const std::array<double, 12> jump_target_sweep_front;
    static const std::array<double, 12> jump_target_extend_front;

    static const std::array<double, 12> jump_target_extend_hip;
    static const std::array<double, 12> jump_target_retract_rear_calf;
    static const std::array<double, 12> jump_target_extend_rear;
    static const std::array<double, 12> jump_target_sweep_rear;
    static const std::array<double, 12> jump_target_extend_calf;
    static const std::array<double, 12> jump_recovery_stand;

    static const std::array<double, 12> small_jump_target_init;
    static const std::array<double, 12> small_jump_target_prepare;
    static const std::array<double, 12> small_jump_target_crouch;
    static const std::array<double, 12> small_jump_target_jump;
    static const std::array<double, 12> small_jump_target_air;
    static const std::array<double, 12> small_jump_target_land;
    static const std::array<double, 12> small_jump_target_recover;
    static const std::array<double, 12> small_jump_target_still;
    static const std::array<double, 12> small_jump_target_stand;
    static const std::array<double, 12> small_jump_target_push;
    static const std::array<double, 12> small_jump_target_push1;

    PerJointActionData stride_data_;
    PerJointActionData jump_data_;
    PerJointActionData small_jump_data_;

    // 关节级覆盖
    std::array<double, 12> override_kp_{};
    std::array<double, 12> override_kd_{};
    bool override_enabled_ = false;

    PerJointActionData& get_action_data(ActionType action_type);
    const PerJointActionData& get_action_data(ActionType action_type) const;
};

#endif // PER_JOINT_PHASE_CONTROLLER_HPP

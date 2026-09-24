#ifndef UNIFIED_ACTION_CONTROLLER_HPP
#define UNIFIED_ACTION_CONTROLLER_HPP

#include <array>
#include <vector>
#include <cstddef>
#include "pd_type.hpp"

// 动作类型枚举
enum class ActionType {
    STRIDE_WALKING = 0,
    JUMP_ADD = 1,
    SMALL_JUMP = 2
};

// StrideWalking 状态枚举
enum class StrideWalkingState {
    Init,      // 0. 初始第一步上抬腿位置特殊，右脚先抬
    Down1,     // 1
    Up1,       // 2
    Up12,      // 3
    Middle,    // 4
    Reach,     // 5
    Buffer1,   // 6
    Wait,      // 7
    Down2,     // 8
    Up2,       // 9
    Up22,      // 10
    Still,     // 11
    Done,      // 12
    Stand      // 13
};

// JumpAdd 状态枚举
enum class JumpState {
    Init,      // 0. 初始化
    Down,      // 1. 所有腿下压
    Jump,      // 2. 所有腿伸长（前>后）
    Up,        // 3. 收前继续伸后
    Wait1,     // 4. 等待
    Bala,      // 5. 扒拉过墙
    Down2,     // 6. 前腿到地
    Wait2,     // 7. 等待
    Pre_Frog,  // 8. 准备青蛙翻
    Frog,      // 9. 后腿青蛙翻
    Push,      // 10. 后腿推
    Stend,     // 11. 后腿收一点
    Stend1,    // 12. 后腿收一点
    Stand,     // 13. 完成
    Done,      // 14. 完成
    RetractRear,  // 15. 收后腿
    SweepFront,   // 16. 划前腿
    ExtendFront,   // 17. 伸前腿
    ExtendHip,     // 18. 展髋
    RetractRearCalf, // 19. 收后腿小腿
    ExtendRear,    // 20. 伸后腿
    SweepRear,     // 21. 划后腿
    ExtendCalf     // 22. 伸小腿
};

// SmallJump 状态枚举（8步骤）
enum class SmallJumpState {
    Init,      // 0. 初始化
    Prepare,   // 1. 准备
    Crouch,    // 2. 下蹲
    Jump,      // 3. 起跳
    Air,       // 4. 空中
    Land,      // 5. 落地
    Recover,   // 6. 恢复
    Stand,     // 7. 站立
    Still,     // 7. 站立
    Push,
    Push1,
    Done       // 8. 完成
};

// 通用相位结构
struct ActionPhase {
    int state_id;                    // 状态ID（使用int以支持不同枚举类型）
    std::array<double, 12> target;     // 目标位置
    double duration;                  // 持续时间
    PDProfile pd_id;                  // PD参数配置
};

// 统一动作控制器类
class UnifiedActionController {
public:
    UnifiedActionController();
    
    // 计算动作命令
    std::array<double, 12> compute_action_cmd(
        ActionType action_type,
        const std::array<double, 12>& current,
        double t);
    
    // 获取当前状态ID
    int get_current_state_id(ActionType action_type) const;
    
    // 获取当前PD参数
    void get_current_pd(ActionType action_type, double pd_out[6]) const;
    
    // 重置动作（用于切换动作时）
    void reset_action(ActionType action_type);
    
    // 检查是否已初始化
    bool is_initialized(ActionType action_type) const;

private:
    // 初始化不同动作的相位表
    void init_stride_walking_phases();
    void init_jump_add_phases();
    void init_small_jump_phases();
    
    // 计算特定动作的命令（内部使用）
    std::array<double, 12> compute_stride_walking_cmd(
        const std::array<double, 12>& current,
        double t);
    std::array<double, 12> compute_jump_add_cmd(
        const std::array<double, 12>& current,
        double t);
    std::array<double, 12> compute_small_jump_cmd(
        const std::array<double, 12>& current,
        double t);
    
    // StrideWalking 目标位置
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
    
    // JumpAdd 目标位置
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
    
    // SmallJump 目标位置（8步骤，参数待用户填写）
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
    
    // 运行期数据（为每个动作类型分别存储）
    struct ActionData {
        std::vector<ActionPhase> phases;
        size_t current_phase = 0;
        size_t loop_count = 0;        // 已执行次数
        size_t max_loop = 7;          // 想循环的次数

        double phase_start_time = 0.0;
        std::array<double, 12> phase_start_pos{};
        bool initialized = false;
    };
    
    ActionData stride_walking_data_;
    ActionData jump_add_data_;
    ActionData small_jump_data_;
    
    // 获取动作数据的辅助函数
    ActionData& get_action_data(ActionType action_type);
    const ActionData& get_action_data(ActionType action_type) const;
};

#endif // UNIFIED_ACTION_CONTROLLER_HPP
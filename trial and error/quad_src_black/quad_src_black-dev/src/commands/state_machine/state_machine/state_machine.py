import rclpy
from rclpy.node import Node
from quad.msg import HighCommands
from quad.msg import StateCommand
from enum import Enum
from std_msgs.msg import UInt8, Int8, Int8MultiArray
from geometry_msgs.msg import Twist

# ================= 枚举定义 =================

class ControlMode(Enum):
    MANUAL = 0
    AUTO = 1

class StateID(Enum):
    STOP = 0
    PASSIVE = 1
    FIXED_DOWN = 2
    FIXED_STAND = 3
    FREE_STAND = 4
    RL_MOVE = 5
    JUMP = 6
    STRIDE = 7
    SMALL_JUMP = 8
    KNEEL_CRAWL = 9

class PolicyID(Enum):
    TROT = 0
    CREEP = 1
    UPSTAIR = 2
    KNEEL_CRAWL_POLICY = 3

class EventID(Enum):
    ENABLE = 0
    DISABLE = 1
    UP_DOWN = 2
    DAMPING = 3
    ENTER_RL = 4
    STRIDE = 5
    SMALL_JUMP = 6
    JUMP = 7
    KNEEL_CRAWL = 8
    POLICY_TROT = 9
    POLICY_CREEP = 10
    POLICY_UPSTAIR = 11
    ENTER_AUTO = 12
    ENTER_MANUAL = 13
    QR_RECOGNITION = 14
    PICK_UP = 15
    PLACE_LOW = 16
    PLACE_HIGH = 17
    LAY_ARM = 18
    SAVE = 19
    DELETE = 20
    POLICY_KNEEL_CRAWL = 21

class TaskStateID(Enum):
    CMD_IDLE = 0
    CMD_AUTO_NAV = 1
    CMD_MANUAL_NAV = 2
    CMD_QR_RECOGNITION = 3
    TASK_WALL_CROSS = 4

# ================= 状态转换表 =================
STATE_TRANSITIONS = {
    StateID.STOP: [
        (EventID.ENABLE, StateID.PASSIVE),
    ],
    StateID.PASSIVE: [
        (EventID.UP_DOWN, StateID.FIXED_DOWN),
        (EventID.DISABLE, StateID.STOP),
    ],
    StateID.FIXED_DOWN: [
        (EventID.DAMPING, StateID.PASSIVE),
        (EventID.UP_DOWN, StateID.FIXED_STAND),
        (EventID.STRIDE, StateID.STRIDE),
        (EventID.SMALL_JUMP, StateID.SMALL_JUMP),
        (EventID.JUMP, StateID.JUMP),
        (EventID.KNEEL_CRAWL, StateID.KNEEL_CRAWL),
    ],
    StateID.FIXED_STAND: [
        (EventID.DAMPING, StateID.PASSIVE),
        (EventID.UP_DOWN, StateID.FIXED_DOWN),
        (EventID.ENTER_RL, StateID.RL_MOVE),
    ],
    StateID.RL_MOVE: [
        (EventID.DAMPING, StateID.PASSIVE),
        (EventID.UP_DOWN, StateID.FIXED_STAND),
        (EventID.JUMP, StateID.JUMP),
    ],
    StateID.STRIDE:[
        (EventID.DAMPING, StateID.PASSIVE),
        (EventID.UP_DOWN, StateID.FIXED_DOWN),
    ],
    StateID.SMALL_JUMP:[
        (EventID.DAMPING, StateID.PASSIVE),
        (EventID.UP_DOWN, StateID.FIXED_DOWN),
    ],
    StateID.JUMP:[
        (EventID.DAMPING, StateID.PASSIVE),
        (EventID.UP_DOWN, StateID.FIXED_DOWN),
        (EventID.ENTER_RL, StateID.RL_MOVE),
    ],
    StateID.KNEEL_CRAWL:[
        (EventID.DAMPING, StateID.PASSIVE),
        (EventID.UP_DOWN, StateID.FIXED_DOWN),
    ],
}

POLICY_TRANSITIONS = [
    (EventID.POLICY_TROT, PolicyID.TROT),
    (EventID.POLICY_CREEP, PolicyID.CREEP),
    (EventID.POLICY_UPSTAIR, PolicyID.UPSTAIR),
    (EventID.POLICY_KNEEL_CRAWL, PolicyID.KNEEL_CRAWL_POLICY),
]

class StateMachineNode(Node):
    def __init__(self):
        super().__init__('state_machine')
        
        # 1. 统一事件订阅 (来自 Joystick 或 TaskManager)
        self.event_sub_ = self.create_subscription(Int8, 'cmd_evt', 
            self.control_event_callback, 10)
        
        # 2. 统一速度订阅 (来自 Joystick 或 TaskManager)
        # 互斥逻辑由发送端保证，状态机只管接收
        self.target_vel_sub_ = self.create_subscription(Twist, 'cmd_vel',
            self.target_vel_callback, 10)
        
        # 输出发布者
        self.task_cmd_pub_ = self.create_publisher(StateCommand, 'task_state_command', 10)
        self.high_cmd_pub_ = self.create_publisher(HighCommands, 'high_command', 10)
        self.state_array_pub_ = self.create_publisher(Int8MultiArray, 'state_array', 10)
        self.manip_pub_ = self.create_publisher(UInt8, '/manipulator/cmd', 10)
        
        # 内部状态
        self.current_mode_ = ControlMode.MANUAL
        self.current_state_ = StateID.STOP
        self.current_policy_ = PolicyID.TROT

        # 时间控制
        self.last_trigger_time_ = self.get_clock().now()
        self.debounce_time_ = 0.2*1e9 # 0.2s
        
        # 数据缓存
        self.target_velocity_ = Twist()
        
        # 定时器：分别用于发布状态反馈(低频)和控制指令(高频)
        self.status_timer = self.create_timer(0.05, self._publish_current_state) # 20Hz
        self.cmd_timer = self.create_timer(0.02, self._publish_high_command)     # 50Hz

    def target_vel_callback(self, msg: Twist):
        """
        接收目标速度
        无论当前是自动还是手动，都直接更新这个缓存。
        互斥逻辑保证了同一时间只有一个节点(手柄或自动程序)在向该话题发布数据。
        """
        self.target_velocity_ = msg

    def control_event_callback(self, msg: Int8):
        """
        接收控制事件
        """
        try:
            event = EventID(msg.data)
            self._process_event(event)
        except ValueError:
            self.get_logger().warn(f'Received invalid event ID: {msg.data}')

    def _process_event(self, event_id: EventID):
        """
        核心状态机处理逻辑
        """
        now = self.get_clock().now()
        
        # 1. 消抖处理
        if (now - self.last_trigger_time_).nanoseconds < self.debounce_time_:
            return

        triggered_events = {event_id}
        
        # 2. 模式切换检测 (Mode Transition)
        mode_changed = self._attempt_mode_transition(triggered_events, now)
        if mode_changed:
            return # 如果模式改变了，这一帧先不处理其他状态转换，等待下一帧

        # 3. 状态切换检测 (State Transition)
        if self._attempt_state_transition(triggered_events, now):
            return

        # 4. 策略切换检测 (Policy Transition)
        if self._attempt_policy_transition(triggered_events, now):
            return
        
        # 5. 其他特殊任务事件处理
        if event_id == EventID.QR_RECOGNITION and self.current_mode_ == ControlMode.AUTO:
            self._publish_task_command(TaskStateID.CMD_QR_RECOGNITION)
            self.last_trigger_time_ = now
            return
        
        if event_id == EventID.PICK_UP:
            msg = UInt8()
            msg.data = 1
            self.manip_pub_.publish(msg)
            self.last_trigger_time_ = now
            return
        elif event_id == EventID.PLACE_LOW:
            msg = UInt8()
            msg.data = 2
            self.manip_pub_.publish(msg)
            self.last_trigger_time_ = now
            return
        elif event_id == EventID.PLACE_HIGH:
            msg = UInt8()
            msg.data = 3
            self.manip_pub_.publish(msg)
            self.last_trigger_time_ = now
            return
        elif event_id == EventID.LAY_ARM:
            msg = UInt8()
            msg.data = 0
            self.manip_pub_.publish(msg)
            self.last_trigger_time_ = now
            return

        if event_id in (EventID.SAVE, EventID.DELETE):
            # Consumed by record_anchor_points.py; no state transition.
            return

    def _attempt_mode_transition(self, triggered_events, now):
        """处理 Manual <-> Auto 切换"""
        # 优先级最高：任何模式下按 DAMPING 触发急停并切回手动
        if EventID.DAMPING in triggered_events:
            if self.current_mode_ == ControlMode.AUTO:
                self.current_mode_ = ControlMode.MANUAL
                self.current_state_ = StateID.PASSIVE
                self.get_logger().warn('Emergency Switch: AUTO -> MANUAL (Triggered by DAMPING)')
                self.last_trigger_time_ = now
                self._publish_task_command(TaskStateID.CMD_MANUAL_NAV)
                return True

        # 进入自动模式
        allowed_states_for_auto = [StateID.RL_MOVE]
        if EventID.ENTER_AUTO in triggered_events:
            if self.current_mode_ == ControlMode.MANUAL:
                if self.current_state_ in allowed_states_for_auto:
                    self.current_mode_ = ControlMode.AUTO
                    self.get_logger().info('Mode Switch: MANUAL -> AUTO')
                    self.last_trigger_time_ = now
                    self._publish_task_command(TaskStateID.CMD_AUTO_NAV)
                    # 清空一下速度，防止残留
                    self.target_velocity_ = Twist()
                    return True
                else:
                    self.get_logger().warn(f'Denied: Cannot enter AUTO in state {self.current_state_.name}')
        
        # 进入手动模式
        elif EventID.ENTER_MANUAL in triggered_events:
            if self.current_mode_ == ControlMode.AUTO:
                self.current_mode_ = ControlMode.MANUAL
                self.get_logger().info('Mode Switch: AUTO -> MANUAL')
                self.last_trigger_time_ = now
                self._publish_task_command(TaskStateID.CMD_MANUAL_NAV)
                self.target_velocity_ = Twist()
                return True
        return False

    def _attempt_state_transition(self, triggered_events, now):
        transitions = STATE_TRANSITIONS.get(self.current_state_, [])
        for event_id, next_state in transitions:
            if event_id in triggered_events:
                self.get_logger().info(f'Transition: {self.current_state_.name} -> {next_state.name}')
                self.current_state_ = next_state
                self.last_trigger_time_ = now
                return True
        self.get_logger().info(f'No state transition triggered by {triggered_events}')
        return False 

    def _attempt_policy_transition(self, triggered_events, now):
        allowed_states = [StateID.FIXED_DOWN,StateID.FIXED_STAND,StateID.RL_MOVE]
        if self.current_state_ not in allowed_states:
            self.get_logger().info(f'Denied: Cannot switch policy in state {self.current_state_.name}')
            return False
        for event_id, target_policy in POLICY_TRANSITIONS:
            if event_id in triggered_events:
                self.current_policy_ = target_policy
                self.get_logger().info(f'Policy: {target_policy.name}')
                self.last_trigger_time_ = now
                return True
        return False

    def _publish_task_command(self, task_state):
        task_msg = StateCommand()
        task_msg.command_id = task_state.value
        self.task_cmd_pub_.publish(task_msg)
        self.get_logger().info(f'Published task command: {task_state.name}')
    
    def _publish_current_state(self):
        # 广播状态，供 JoystickHandler 和 TaskManager 读取以决定是否发布速度
        state_array = Int8MultiArray()
        state_array.data = [
            self.current_mode_.value,
            self.current_state_.value,
            self.current_policy_.value,
        ]
        self.state_array_pub_.publish(state_array)
    

    def _publish_high_command(self):
        pub_msg = HighCommands()
        pub_msg.command = self.current_state_.value
        pub_msg.policy_switch = self.current_policy_.value
        pub_msg.lin_x = self.target_velocity_.linear.x*1.5
        pub_msg.lin_y = min(max(self.target_velocity_.linear.y*0.8,-0.8),0.8)
        pub_msg.ang_yaw = self.target_velocity_.angular.z*2.0
        self.high_cmd_pub_.publish(pub_msg)

        # self.get_logger().info(
        # f"[HIGH_CMD] state: {pub_msg.command}, policy: {pub_msg.policy_switch}, "
        # f"vx: {pub_msg.lin_x:.3f}, vy: {pub_msg.lin_y:.3f}, wz: {pub_msg.ang_yaw:.3f}"
        # )

def main(args=None):
    rclpy.init(args=args)
    state_machine = StateMachineNode()
    rclpy.spin(state_machine)
    state_machine.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
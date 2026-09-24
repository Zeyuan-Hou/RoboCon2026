import rclpy
from rclpy.node import Node
from quad.msg import Joystick
from geometry_msgs.msg import Twist
from std_msgs.msg import Int8, Int8MultiArray
from enum import Enum

# ================= 配置与枚举定义 =================

class ControlMode(Enum):
    MANUAL = 0
    AUTO = 1

class EventID(Enum):
    ENABLE = 0
    DISABLE = 1
    STAND = 2
    DAMPING = 3
    ENTER_RL = 4
    STRIDE = 5
    SMALL_JUMP = 6
    JUMP = 7
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
    ALLOW_AUTO = 22
    START_LIDAR_INIT = 23
    DISABLE_MANIPULATOR = 24
    START_VISUAL_RECOGNITION = 25

# 手柄映射 (保持与原代码一致)
button_map = {
    'A': 0, 'B': 1, 'X': 3, 'Y': 4,
    'LB':6, 'RB':7, 'LT':8, 'RT':9,
    'START':11, 'HAT_UP':22, 'HAT_DOWN':21,
    'HAT_LEFT':19, 'HAT_RIGHT':20
}
axes_map = {
    'LX': 0, 'LY': 1, 'RX': 2, 'RY': 3,
    'LT':4, 'RT':5,
}
am = axes_map
bm = button_map

# 事件触发绑定
EVENT_BUTTONS = {
    EventID.ENABLE: ('LT', 'START'),
    EventID.DISABLE: ('RT', 'START'),
    EventID.STAND: ('LB', 'A'),
    EventID.DAMPING: ('LB', 'B'),
    EventID.ENTER_RL: ('LB', 'X'),
    EventID.STRIDE: ('RB', 'A'),
    EventID.SMALL_JUMP: ('RB', 'X'),
    EventID.JUMP: ('RB', 'Y'),
    EventID.POLICY_TROT: ('RB', 'HAT_UP'),
    EventID.POLICY_CREEP: ('RB', 'HAT_DOWN'),
    EventID.POLICY_UPSTAIR: ("RB", 'HAT_LEFT'),
    EventID.POLICY_KNEEL_CRAWL: ('RB', 'HAT_RIGHT'),
    EventID.ENTER_AUTO: ('LT', 'A'),
    EventID.ALLOW_AUTO: ('LT', 'B'),
    EventID.START_LIDAR_INIT: ('LT', 'Y'),
    EventID.START_VISUAL_RECOGNITION: ('LT', 'X'),
    EventID.ENTER_MANUAL: ('RT', 'A'),
    # EventID.QR_RECOGNITION: ('LT', 'Y'),
    EventID.PICK_UP: ('LT', 'HAT_UP'),
    EventID.PLACE_LOW: ('LT', 'HAT_DOWN'),
    EventID.PLACE_HIGH: ('LT', 'HAT_RIGHT'),
    EventID.LAY_ARM: ('LT', 'HAT_LEFT'),
    EventID.SAVE: ('LB', 'RB'),
    EventID.DELETE: ('LT', 'RT'),
    EventID.DISABLE_MANIPULATOR: ('RT', 'B'),
}

class JoystickHandler(Node):
    def __init__(self):
        super().__init__('joystick_handler')
        
        # 1. 订阅原始手柄数据
        self.js_sub_ = self.create_subscription(Joystick, 'js_pub', self.joy_callback, 10)
        
        # 2. 订阅系统状态以获取当前模式 (实现互斥的关键)
        self.state_sub_ = self.create_subscription(Int8MultiArray, 'state_array', self.state_callback, 10)
        
        # 3. 发布统一的控制话题
        # 这一话题与自动规划节点共用，通过逻辑互斥防止冲突
        self.event_pub_ = self.create_publisher(Int8, 'cmd_evt', 10)
        self.vel_pub_ = self.create_publisher(Twist, 'cmd_vel', 10)
        
        self.current_mode_ = ControlMode.MANUAL
        
        # 简单的按键防抖/边沿检测逻辑
        self.last_triggered_events = set()

        # 摇杆低通滤波
        self.filter_alpha = 0.1
        self.last_lx = 0.0
        self.last_ly = 0.0
        self.last_rx = 0.0  

    def state_callback(self, msg: Int8MultiArray):
        """更新当前控制模式"""
        # 约定 state_array[0] 为 ControlMode
        if len(msg.data) >= 1:
            try:
                self.current_mode_ = ControlMode(msg.data[0])
            except ValueError:
                pass

    def _check_buttons_state(self, msg, buttons):
        return all(msg.buttons[bm[btn]] for btn in buttons)

    def _get_active_events(self, msg):
        triggered = set()
        for event_id, buttons in EVENT_BUTTONS.items():
            if self._check_buttons_state(msg, buttons):
                triggered.add(event_id)
        return triggered

    def should_publish_event(self, event_id):
        """核心逻辑：根据模式过滤事件"""
        if self.current_mode_ == ControlMode.MANUAL:
            return True  # 手动模式：允许一切操作
        elif self.current_mode_ == ControlMode.AUTO:
            # 自动模式：只允许 切换回手动、急停、二维码识别、任务赛预扫描后确认继续
            return event_id in [
                EventID.ENTER_MANUAL,
                EventID.QR_RECOGNITION,
                EventID.DAMPING,
                EventID.ALLOW_AUTO
            ]
        return False

    def joy_callback(self, msg):
        # --- 1. 处理事件发布 ---
        current_events = self._get_active_events(msg)
        
        # 简单的上升沿检测，防止按住不放一直发事件
        # (State Machine 端也有消抖，但这里做一次预处理更好)
        new_events = current_events - self.last_triggered_events
        
        for event in new_events:
            if self.should_publish_event(event):
                event_msg = Int8()
                event_msg.data = event.value
                self.event_pub_.publish(event_msg)
                self.get_logger().debug(f"Event Pub: {event.name}")
        
        self.last_triggered_events = current_events

        # --- 2. 处理速度发布 (互斥逻辑) ---
        # 只有在 MANUAL 模式下，手柄才拥有发布速度的权限
        # 在 AUTO 模式下，我们保持静默，让自动规划节点去发布
        if self.current_mode_ == ControlMode.MANUAL:
            vel_msg = Twist()
            
            # 保持原有的映射系数
            # LY (前后) -> Twist.linear.y (Quad习惯)
            # LX (左右) -> Twist.linear.x
            # RX (旋转) -> Twist.angular.z
            
            ly = msg.axes[am['LY']]
            lx = msg.axes[am['LX']]
            rx = msg.axes[am['RX']]

            # 低通滤波
            ly = self.filter_alpha * ly + (1 - self.filter_alpha) * self.last_ly
            lx = self.filter_alpha * lx + (1 - self.filter_alpha) * self.last_lx
            rx = self.filter_alpha * rx + (1 - self.filter_alpha) * self.last_rx

            self.last_ly = ly
            self.last_lx = lx
            self.last_rx = rx

            # 死区处理与坐标转换
            if abs(ly) > 0.01:
                vel_msg.linear.x = -ly
            if abs(lx) > 0.01:
                vel_msg.linear.y = -lx
            if abs(rx) > 0.01:
                vel_msg.angular.z = -rx
            
            self.vel_pub_.publish(vel_msg)

def main(args=None):
    rclpy.init(args=args)
    node = JoystickHandler()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

import rclpy
from rclpy.node import Node
from quad.msg import Joystick
import pygame
import pygame.joystick as pyjs
import os

class JoystickPublisher(Node):

    def __init__(self):
        super().__init__('joystick_publisher')
        self.publisher_ = self.create_publisher(Joystick, 'js_pub', 10)
        timer_period = 0.01# seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        os.environ["SDL_VIDEODRIVER"] = "dummy"
        pygame.init()
        pygame.event.clear()

        if pyjs.get_count() >=1:
            print("joystick detected.")
        else:
            print("No joystick found")
            exit()
        
        self.js = pyjs.Joystick(0)
        self.js.init()

        # 如果是Xbox模式,需要长按北通键修改为北通模式
        # 1. 获取设备名称 (通常能反映出是 XInput 还是 DInput 模式)
        js_name = self.js.get_name()
        print(f"手柄名称 (Name): {js_name}")

        # 2. 获取 GUID (SDL 库分配的唯一 ID，非常适合用来区分不同硬件)
        js_guid = self.js.get_guid()
        print(f"手柄 GUID: {js_guid}")
        
        # 3. 打印轴和按钮的数量，这也能辅助判断模式
        print(f"轴数量: {self.js.get_numaxes()}")
        print(f"按钮数量: {self.js.get_numbuttons()}")

    def timer_callback(self):
        msg = Joystick()

        for i in range(self.js.get_numaxes()):
            msg.axes[i] = self.js.get_axis(i)
        for i in range(self.js.get_numbuttons()):
            msg.buttons[i] = self.js.get_button(i)
        hat = self.js.get_hat(0)
        # msg.hat[0] = hat[0]
        # msg.hat[1] = hat[1]
        
        # 将hat作为按键识别
        msg.buttons[19]  = 1 if hat[0] == -1 else 0
        msg.buttons[20]  = 1 if hat[0] == 1  else 0
        msg.buttons[21]  = 1 if hat[1] == -1 else 0
        msg.buttons[22]  = 1 if hat[1] == 1  else 0
        pygame.event.pump()
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    joystick_publisher = JoystickPublisher()

    rclpy.spin(joystick_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    joystick_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

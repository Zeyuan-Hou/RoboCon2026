#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from tf2_msgs.msg import TFMessage
from std_msgs.msg import Float32MultiArray
import math


class LidarNavControl(Node):
    def __init__(self):
        super().__init__('lidar_nav_control')
        
        self.declare_parameter('target_frame', 'camera_init')
        self.declare_parameter('child_frame', 'aft_mapped')
        self.declare_parameter('target_x', 2.0)
        self.declare_parameter('target_y', 1.0)
        self.declare_parameter('target_yaw', 0.0)
        self.declare_parameter('kp_x', 0.5)
        self.declare_parameter('kp_y', 0.5)
        self.declare_parameter('kp_yaw', 1.0)
        self.declare_parameter('max_vx', 0.5)
        self.declare_parameter('max_vy', 0.5)
        self.declare_parameter('max_dyaw', 0.5)
        self.declare_parameter('control_rate', 50.0)
        self.declare_parameter('enable_control', True)
        self.declare_parameter('tf_topic', '/tf')
        self.declare_parameter('control_topic', '/lidar_nav_control')
        
        self.target_frame = self.get_parameter('target_frame').value
        self.child_frame = self.get_parameter('child_frame').value
        self.target_x = self.get_parameter('target_x').value
        self.target_y = self.get_parameter('target_y').value
        self.target_yaw = self.get_parameter('target_yaw').value
        self.kp_x = self.get_parameter('kp_x').value
        self.kp_y = self.get_parameter('kp_y').value
        self.kp_yaw = self.get_parameter('kp_yaw').value
        self.max_vx = self.get_parameter('max_vx').value
        self.max_vy = self.get_parameter('max_vy').value
        self.max_dyaw = self.get_parameter('max_dyaw').value
        self.control_rate = self.get_parameter('control_rate').value
        self.enable_control = self.get_parameter('enable_control').value
        self.tf_topic = self.get_parameter('tf_topic').value
        self.control_topic = self.get_parameter('control_topic').value
        
        self.tf_sub = self.create_subscription(TFMessage, self.tf_topic, self.tf_callback, 100)
        self.cmd_vel_pub = self.create_publisher(Twist, '/quad/cmd_vel', 10)
        self.control_sub = self.create_subscription(Float32MultiArray, self.control_topic, self.control_callback, 10)
        
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.got_tf = False
        self.tf_count = 0
        self.printed_frames = False
        self.print_count = 0
        
        period = 1.0 / self.control_rate
        self.timer = self.create_timer(period, self.control_loop)
        
        self.get_logger().info('Started. TF: ' + self.tf_topic + ', Control: ' + self.control_topic)

    def tf_callback(self, msg: TFMessage):
        self.tf_count += 1
        if not self.printed_frames and self.tf_count <= 5:
            frames = [(t.header.frame_id, t.child_frame_id) for t in msg.transforms]
            self.get_logger().info('TF frames: ' + str(frames))
            if self.tf_count == 5:
                self.printed_frames = True
        
        for transform in msg.transforms:
            if transform.header.frame_id == self.target_frame and transform.child_frame_id == self.child_frame:
                self.x = transform.transform.translation.x
                self.y = transform.transform.translation.y
                qx = transform.transform.rotation.x
                qy = transform.transform.rotation.y
                qz = transform.transform.rotation.z
                qw = transform.transform.rotation.w
                self.yaw = math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
                self.got_tf = True
                break

    def control_callback(self, msg: Float32MultiArray):
        if len(msg.data) < 4:
            self.get_logger().warn('control array too short, need 4 elements')
            return
        
        self.enable_control = bool(msg.data[0])
        self.target_x = msg.data[1]
        self.target_y = msg.data[2]
        self.target_yaw = msg.data[3]
        
        if self.print_count % 100 == 0:
            self.get_logger().info('Control updated: enable=%s, target=(%.2f, %.2f, %.2f)' %
                (self.enable_control, self.target_x, self.target_y, self.target_yaw))

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def control_loop(self):
        if not self.got_tf:
            if self.tf_count == 0:
                self.get_logger().warn('No TF received yet', throttle_duration_sec=2.0)
            return
        
        err_x = self.target_x - self.x
        err_y = self.target_y - self.y
        err_yaw = self.normalize_angle(self.target_yaw - self.yaw)
        
        dist = math.sqrt(err_x * err_x + err_y * err_y)
        if dist < 0.1 and abs(err_yaw) < 0.1:
            self.cmd_vel_pub.publish(Twist())
            self.get_logger().info('Reached target!')
            return
        
        cmd = Twist()

        if self.enable_control:
            cmd.linear.x = max(-self.max_vx, min(self.kp_x * err_x, self.max_vx))
            cmd.linear.y = max(-self.max_vy, min(self.kp_y* err_y, self.max_vy))
            cmd.angular.z = max(-self.max_dyaw, min(self.kp_yaw * err_yaw, self.max_dyaw))

            self.get_logger().info(
                f"CMD vx: {cmd.linear.x:.3f}, vy: {cmd.linear.y:.3f}, wz: {cmd.angular.z:.3f}"
            )
        
        self.cmd_vel_pub.publish(cmd)
        
        if self.print_count % 25 == 0:
            self.get_logger().info('pos(%.2f,%.2f,%.2f) err(%.2f,%.2f,%.2f)' % 
                (self.x, self.y, self.yaw, err_x, err_y, err_yaw))
        self.print_count += 1


def main(args=None):
    rclpy.init(args=args)
    node = LidarNavControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

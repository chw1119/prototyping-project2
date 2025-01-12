import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class MotorController(Node):
    def __init__(self):
        super().__init__('motor_controller')
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.get_logger().info("Motor Controller Node started")

    def cmd_vel_callback(self, msg):
        # 메시지의 선형 및 각속도를 출력하거나 로봇에 전달
        self.get_logger().info(f"Linear: {msg.linear.x}, Angular: {msg.angular.z}")
        # 모터 제어 코드를 추가하세요 (예: PWM 출력)

def main(args=None):
    rclpy.init(args=args)
    motor_controller = MotorController()
    rclpy.spin(motor_controller)
    motor_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

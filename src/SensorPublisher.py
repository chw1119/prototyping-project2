import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Imu
import random  # 실제 데이터 대신 랜덤 값을 사용

class SensorPublisher(Node):
    def __init__(self):
        super().__init__('sensor_publisher')
        self.lidar_publisher = self.create_publisher(LaserScan, '/scan', 10)
        self.imu_publisher = self.create_publisher(Imu, '/imu', 10)
        self.timer = self.create_timer(0.1, self.publish_sensor_data)

    def publish_sensor_data(self):
        # LiDAR 데이터
        lidar_msg = LaserScan()
        lidar_msg.ranges = [random.uniform(0.1, 10.0) for _ in range(360)]  # 예제 데이터
        self.lidar_publisher.publish(lidar_msg)

        # IMU 데이터
        imu_msg = Imu()
        imu_msg.orientation.w = 1.0
        imu_msg.angular_velocity.z = random.uniform(-1.0, 1.0)  # 예제 데이터
        self.imu_publisher.publish(imu_msg)

def main(args=None):
    rclpy.init(args=args)
    sensor_publisher = SensorPublisher()
    rclpy.spin(sensor_publisher)
    sensor_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

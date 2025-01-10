import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import numpy as np
import matplotlib.pyplot as plt

class PotentialFieldVisualizer(Node):
    def __init__(self):
        super().__init__('potential_field_visualizer')
        # 구독: 비용 맵 데이터
        self.subscription = self.create_subscription(
            OccupancyGrid,
            '/global_costmap/costmap',  # Nav2의 글로벌 비용 맵 토픽
            self.costmap_callback,
            10
        )
        self.costmap_data = None

    def costmap_callback(self, msg):
        # OccupancyGrid 데이터를 numpy 배열로 변환
        width = msg.info.width
        height = msg.info.height
        data = np.array(msg.data, dtype=np.int8).reshape((height, width))
        self.costmap_data = data
        self.visualize_potential_field()

    def visualize_potential_field(self):
        if self.costmap_data is None:
            return

        # 잠재장 계산 (비용 맵 데이터를 직접 사용)
        potential_field = np.exp(-self.costmap_data / 10.0)  # 간단한 예: 지수 감소
        plt.imshow(potential_field, cmap='hot', origin='lower')
        plt.colorbar(label='Potential Field Intensity')
        plt.title('Potential Field')
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.show()

def main(args=None):
    rclpy.init(args=args)
    node = PotentialFieldVisualizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

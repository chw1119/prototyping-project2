#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity, SetEntityState
from gazebo_msgs.msg import EntityState
import time
import random
from BoxObject import BoxObject


class SpawnAndMoveBoxNode(Node):
    def __init__(self):
        super().__init__('spawn_and_move_box_node')

        # Gazebo의 /spawn_entity 서비스를 사용하기 위한 클라이언트 생성
        self.box_list = []

        self.spawn_client = self.create_client(SpawnEntity, '/spawn_entity')
        self.move_client = self.create_client(SetEntityState, '/set_entity_state')

        while not self.spawn_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /spawn_entity service...')

        while not self.move_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /gazebo/set_entity_state service...')

        self.get_logger().info('Services are available. Spawning box...')
        self.spawn_box()

    def spawn_box(self):
        # SpawnEntity 서비스 요청 생성
        request = SpawnEntity.Request()
        

        box_temp = BoxObject(
            name=f"human${random.uniform(1000, 9999)}", x_pose = 0.0, y_pose = 0.0, z_pose = 0.0,
            xml_string = self.generate_box_sdf()
        )

        self.box_list.append(box_temp)

        request.name = box_temp.name  # 생성할 물체 이름
        request.xml = box_temp.xml_string  # SDF 내용
        request.initial_pose.position.x = box_temp.x_pose  # 초기 위치 설정 (x)
        request.initial_pose.position.y = box_temp.y_pose  # 초기 위치 설정 (y)
        request.initial_pose.position.z = box_temp.z_pose  # 초기 위치 설정 (z)

        request.initial_pose.orientation.w = 1.0  # 방향 설정 (쿼터니언)

        # 서비스 호출
        future = self.spawn_client.call_async(request)
        future.add_done_callback(self.callback_spawn_response)

    def generate_box_sdf(self):
        # 긴 직육면체를 정의하는 SDF 문자열 생성
        return """
        <?xml version="1.0" ?>
        <sdf version="1.6">
          <model name="long_box">
            <static>false</static>
            <link name="link">
              <inertial>
                <mass>1.0</mass>
                <inertia>
                  <ixx>0.1</ixx>
                  <iyy>0.1</iyy>
                  <izz>0.1</izz>
                </inertia>
              </inertial>
              <visual name="visual">
                <geometry>
                  <box>
                    <size>4.0 0.5 0.5</size>
                  </box>
                </geometry>
                <material>
                  <ambient>1 0 0 1</ambient>
                  <diffuse>1 0 0 1</diffuse>
                </material>
              </visual>
              <collision name="collision">
                <geometry>
                  <box>
                    <size>4.0 0.5 0.5</size>
                  </box>
                </geometry>
              </collision>
            </link>
          </model>
        </sdf>
        """

    def callback_spawn_response(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f"Successfully spawned box: {response.status_message}")
                #time.sleep(2)  # 물체 생성 대기
                self.move_box()
            else:
                self.get_logger().error(f"Failed to spawn box: {response.status_message}")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {str(e)}")

    def move_box(self):
        # SetEntityState 서비스 요청 생성
        request = SetEntityState.Request()
        
        for box in self.box_list:
          state = EntityState()
          box.x_vector = 0.01
          box.animate()

          state.name = box.name  # 이동할 물체의 이름
          state.pose.position.x = box.x_pose  # 새로운 위치 x
          state.pose.position.y = box.y_pose  # 새로운 위치 y
          state.pose.position.z = box.z_pose  # 높이 유지
          state.pose.orientation.w = 1.0  # 방향 유지

          # 선속도와 각속도 설정

          request.state = state

          # 서비스 호출
          future = self.move_client.call_async(request)
          future.add_done_callback(self.callback_move_response)

    def callback_move_response(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f"Successfully moved box: {response}")
            else:
                self.get_logger().error(f"Failed to move box: {response}")

            #self.spawn_box()
        except Exception as e:
            self.get_logger().error(f"Service call failed: {str(e)}")


def main(args=None):
    rclpy.init(args=args)
    node = SpawnAndMoveBoxNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()

#!/usr/bin/env python3

import os
from rclpy.node import Node
from std_msgs.msg import String
from gazebo_msgs.srv import SpawnEntity, SetEntityState
from BoxObject import BoxObject
import random

class MakeEnvNode(Node):
    def __init__(self):
        super().__init__('make_env_node')
        self.get_logger().info('MakeEnvNode initialized')

        # Load STL file and setup parameters
        self.package_name = 'prototyping-project2'  # Replace with your package name
        self.stl_file_name = 'map.stl'       # Replace with your STL file name

        # Get package share directory
        self.package_share_path = self.get_package_share_directory()
        self.map_stl_file_path = os.path.join(self.package_share_path, 'map', self.stl_file_name)

        # Generate URDF content with STL mesh
        self.map_content = self.generate_sdf(self.stl_file_path)

        self.chiar_stl_file_path = os.path.join(self.package_share_path, 'map', self.stl_file_name)
        self.chair_content = self.generate_sdf(self.stl_file_path)

        self.spawn_client = self.create_client(SpawnEntity, '/spawn_entity')

        # Publish or use the URDF as needed (e.g., load into Gazebo)
        self.publish_urdf()

    def get_package_share_directory(self):
        from ament_index_python.packages import get_package_share_directory
        try:
            return get_package_share_directory(self.package_name)
        except Exception as e:
            self.get_logger().error(f"Failed to find package: {self.package_name}. Error: {e}")
            raise

    def generate_sdf(self, stl_path):
        return f'''
        <sdf version="1.6">
            <model name="MapObject">
                <static>true</static>
                <link name="base_link">
                    <visual name="visual">
                        <geometry>
                            <mesh>
                                <uri>{stl_path}</uri>
                                <scale>0.005 0.005 0.005</scale>
                            </mesh>
                        </geometry>
                    </visual>
                    <collision name="collision">
                        <geometry>
                            <mesh>
                                <uri>{stl_path}</uri>
                                <scale>0.005 0.005 0.005</scale>
                            </mesh>
                        </geometry>
                    </collision>
                </link>
            </model>
        </sdf>
        '''


    def publish_urdf(self):
        # Example of logging the URDF or extending for specific ROS usage
        self.get_logger().info("URDF generated:")
        self.get_logger().info(self.urdf_content)

        # Additional functionality to publish or load URDF can be added here
        # For example, publish it as a ROS topic or save to a file
        
        self.spawn_object()

    def spawn_object(self):
        # SpawnEntity 서비스 요청 생성
        request = SpawnEntity.Request()
        

        box_temp = BoxObject(
            name=f"map${random.uniform(1000, 9999)}", x_pose = 0.0, y_pose = 0.0, z_pose = 0.0,
            xml_string = self.urdf_content
        )


        request.name = box_temp.name  # 생성할 물체 이름
        request.xml = box_temp.xml_string  # SDF 내용
        request.initial_pose.position.x = box_temp.x_pose  # 초기 위치 설정 (x)
        request.initial_pose.position.y = box_temp.y_pose  # 초기 위치 설정 (y)
        request.initial_pose.position.z = box_temp.z_pose  # 초기 위치 설정 (z)

        request.initial_pose.orientation.w = 1.0  # 방향 설정 (쿼터니언)

        # 서비스 호출
        future = self.spawn_client.call_async(request)
        future.add_done_callback(self.callback_spawn_response)

    def callback_spawn_response(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f"Successfully spawned box: {response.status_message}")
                #time.sleep(2)  # 물체 생성 대기

            else:
                self.get_logger().error(f"Failed to spawn box: {response.status_message}")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {str(e)}")


def main(args=None):
    import rclpy
    rclpy.init(args=args)
    node = MakeEnvNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

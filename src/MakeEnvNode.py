#!/usr/bin/env python3

import os
from rclpy.node import Node
from std_msgs.msg import String
from gazebo_msgs.srv import SpawnEntity, SetEntityState
from BoxObject import BoxObject
from scipy.spatial.transform import Rotation as R
from time import sleep

# 오일러 각도 (roll, pitch, yaw) [rad]
euler_angles = [0.1, 0.2, 0.3]  # radians

# 오일러 각을 쿼터니언으로 변환
r = R.from_euler('xyz', euler_angles)  # 'xyz'는 회전 순서
quaternion = r.as_quat()  # [x, y, z, w]
print("Quaternion:", quaternion)

import random

class MakeEnvNode(Node):
    def __init__(self):
        super().__init__('make_env_node')
        self.get_logger().info('MakeEnvNode initialized')

        # Load STL file and setup parameters
        self.package_name = 'prototyping-project2'  # Replace with your package name
        self.map_stl_file_name = 'Prototype2_InternetCafe_Map.dae'       # Replace with your STL file name
        self.chair_sdf_file_name = "chair.sdf"
        self.robot_obj_file_name = "robot.obj"

        # Get package share directory
        self.package_share_path = self.get_package_share_directory()
        self.map_stl_file_path = os.path.join(self.package_share_path, 'map', self.map_stl_file_name)
        self.robot_obj_file_name = os.path.join(self.package_share_path, 'description', 'meshes', self.robot_obj_file_name)
        self.chiar_sdf_file_path = os.path.join(self.package_share_path, 'description',  'object' ,self.chair_sdf_file_name)

        with open(self.chiar_sdf_file_path, 'r') as file:
            chair_sdf = file.read()
            self.chair_sdf_contents = chair_sdf

            pass

        # Generate URDF content with STL mesh

        self.map_content = self.generate_sdf(self.map_stl_file_path, "0.01", "true", model_name="map_content")
        self.robot_content = self.generate_sdf(self.robot_obj_file_name, "0.1", "true", model_name= "robot_content")



        self.spawn_client = self.create_client(SpawnEntity, '/spawn_entity')

        sleep(10)
        # Publish or use the URDF as needed (e.g., load into Gazebo)
        self.publish_urdf()

    def get_package_share_directory(self):
        from ament_index_python.packages import get_package_share_directory
        try:
            return get_package_share_directory(self.package_name)
        except Exception as e:
            self.get_logger().error(f"Failed to find package: {self.package_name}. Error: {e}")
            raise

    def generate_sdf(self, stl_path = "", ratio = "1.0", static = "true", model_name = "default"):
        return f'''
        <sdf version="1.6">
            <model name="{model_name}">
                <static>{static}</static>
                <self_collide>true</self_collide>
                <link name="base_link">
                    <visual name="visual">
                        <geometry>
                            <mesh>
                                <uri>{stl_path}</uri>
                                <scale>{ratio} {ratio} {ratio}</scale>
                            </mesh>
                        </geometry>
                    </visual>
                    <collision name="collision">
                        <geometry>
                            <mesh>
                                <uri>{stl_path}</uri>
                                <scale>{ratio} {ratio} {ratio}</scale>
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
        self.get_logger().info(self.map_content)

        # Additional functionality to publish or load URDF can be added here
        # For example, publish it as a ROS topic or save to a file
        self.spawn_robot()
        self.spawn_map()
        self.spawn_chairs()

    def spawn_robot(self):
        request = SpawnEntity.Request()
        

        box_temp = BoxObject(
            name=f"robot${random.uniform(1000, 9999)}", x_pose = 8.0, y_pose = -5.0, z_pose = 20.0,
            xml_string = self.robot_content
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

    def spawn_map(self):
        # SpawnEntity 서비스 요청 생성
        request = SpawnEntity.Request()
        

        box_temp = BoxObject(
            name=f"map${random.uniform(1000, 9999)}", x_pose = 8.0, y_pose = -5.0, z_pose = 0.0,
            xml_string = self.map_content
        )


        request.name = box_temp.name  # 생성할 물체 이름
        request.xml = box_temp.xml_string  # SDF 내용

        request.initial_pose.position.x = box_temp.x_pose  # 초기 위치 설정 (x)
        request.initial_pose.position.y = box_temp.y_pose  # 초기 위치 설정 (y)
        request.initial_pose.position.z = box_temp.z_pose  # 초기 위치 설정 (z)
        
        euler_angles = [0.0, 0.0, -1.564232]  # radians

            # 오일러 각을 쿼터니언으로 변환
        r = R.from_euler('xyz', euler_angles)  # 'xyz'는 회전 순서
        quaternion = r.as_quat()  # [x, y, z, w]


        request.initial_pose.orientation.x = quaternion[0]  # 방향 설정 (쿼터니언)
        request.initial_pose.orientation.y = quaternion[1]  # 방향 설정 (쿼터니언)
        request.initial_pose.orientation.z = quaternion[2]  # 방향 설정 (쿼터니언)
        request.initial_pose.orientation.w = quaternion[3]  # 방향 설정 (쿼터니언)

        # 서비스 호출
        future = self.spawn_client.call_async(request)
        future.add_done_callback(self.callback_spawn_response)

    def spawn_chairs(self):
        # SpawnEntity 서비스 요청 생성
        for i in range(5):
            request = SpawnEntity.Request()
        

            box_temp = BoxObject(
                name=f"chair${random.uniform(1000, 9999)}", x_pose = 7.1  + (random.random() / 5 - (1 / 5)), y_pose = -3.0 + (0.9 * i) + (random.random() / 5 - (1 / 5)), z_pose = 1.1,
                xml_string = self.chair_sdf_contents
            )


            request.name = box_temp.name  # 생성할 물체 이름
            request.xml = box_temp.xml_string  # SDF 내용
            request.initial_pose.position.x = box_temp.x_pose  # 초기 위치 설정 (x)
            request.initial_pose.position.y = box_temp.y_pose  # 초기 위치 설정 (y)
            request.initial_pose.position.z = box_temp.z_pose  # 초기 위치 설정 (z)


            euler_angles = [0.0, 0.0, 1.6]  # radians

            # 오일러 각을 쿼터니언으로 변환
            r = R.from_euler('xyz', euler_angles)  # 'xyz'는 회전 순서
            quaternion = r.as_quat()  # [x, y, z, w]


            request.initial_pose.orientation.x = quaternion[0]  # 방향 설정 (쿼터니언)
            request.initial_pose.orientation.y = quaternion[1]  # 방향 설정 (쿼터니언)
            request.initial_pose.orientation.z = quaternion[2]  # 방향 설정 (쿼터니언)
            request.initial_pose.orientation.w = quaternion[3]  # 방향 설정 (쿼터니언)


            # 서비스 호출
            future = self.spawn_client.call_async(request)
            future.add_done_callback(self.callback_spawn_response)


        sleep(3)

        for i in range(4):
            request = SpawnEntity.Request()
        

            box_temp = BoxObject(
                name=f"chair${random.uniform(1000, 9999)}", x_pose = 7.1 + (random.random() / 5 - (1 / 5)), y_pose = 1.95 + (0.9 * i) + (random.random() / 5 - (1 / 5)), z_pose = 1.1,
                xml_string = self.chair_sdf_contents
            )


            request.name = box_temp.name  # 생성할 물체 이름
            request.xml = box_temp.xml_string  # SDF 내용
            request.initial_pose.position.x = box_temp.x_pose  # 초기 위치 설정 (x)
            request.initial_pose.position.y = box_temp.y_pose  # 초기 위치 설정 (y)
            request.initial_pose.position.z = box_temp.z_pose  # 초기 위치 설정 (z)


            euler_angles = [0.0, 0.0, 1.6]  # radians

            # 오일러 각을 쿼터니언으로 변환
            r = R.from_euler('xyz', euler_angles)  # 'xyz'는 회전 순서
            quaternion = r.as_quat()  # [x, y, z, w]


            request.initial_pose.orientation.x = quaternion[0]  # 방향 설정 (쿼터니언)
            request.initial_pose.orientation.y = quaternion[1]  # 방향 설정 (쿼터니언)
            request.initial_pose.orientation.z = quaternion[2]  # 방향 설정 (쿼터니언)
            request.initial_pose.orientation.w = quaternion[3]  # 방향 설정 (쿼터니언)


            # 서비스 호출
            future = self.spawn_client.call_async(request)
            future.add_done_callback(self.callback_spawn_response)
        
        sleep(3)
        
        for i in range(5):
            request = SpawnEntity.Request()
        

            box_temp = BoxObject(
                name=f"chair${random.uniform(1000, 9999)}", x_pose = 5.63 - (0.9 * i) + (random.random() / 5 - (1 / 5)), y_pose = 5.94 + (random.random() / 5 - (1 / 5)), z_pose = 1.1,
                xml_string = self.chair_sdf_contents
            )


            request.name = box_temp.name  # 생성할 물체 이름
            request.xml = box_temp.xml_string  # SDF 내용
            request.initial_pose.position.x = box_temp.x_pose  # 초기 위치 설정 (x)
            request.initial_pose.position.y = box_temp.y_pose  # 초기 위치 설정 (y)
            request.initial_pose.position.z = box_temp.z_pose  # 초기 위치 설정 (z)


            euler_angles = [0.0, 0.0, 3.14]  # radians

            # 오일러 각을 쿼터니언으로 변환
            r = R.from_euler('xyz', euler_angles)  # 'xyz'는 회전 순서
            quaternion = r.as_quat()  # [x, y, z, w]


            request.initial_pose.orientation.x = quaternion[0]  # 방향 설정 (쿼터니언)
            request.initial_pose.orientation.y = quaternion[1]  # 방향 설정 (쿼터니언)
            request.initial_pose.orientation.z = quaternion[2]  # 방향 설정 (쿼터니언)
            request.initial_pose.orientation.w = quaternion[3]  # 방향 설정 (쿼터니언)


            # 서비스 호출
            future = self.spawn_client.call_async(request)
            future.add_done_callback(self.callback_spawn_response)


        sleep(3)

        for i in range(6):
            request = SpawnEntity.Request()
        
            1.35
            box_temp = BoxObject(
                name=f"chair${random.uniform(1000, 9999)}", x_pose = 0.65 - (0.9 * i) + (random.random() / 5 - (1 / 5)), y_pose = 5.94 + (random.random() / 5 - (1 / 5)), z_pose = 1.1,
                xml_string = self.chair_sdf_contents
            )


            request.name = box_temp.name  # 생성할 물체 이름
            request.xml = box_temp.xml_string  # SDF 내용
            request.initial_pose.position.x = box_temp.x_pose  # 초기 위치 설정 (x)
            request.initial_pose.position.y = box_temp.y_pose  # 초기 위치 설정 (y)
            request.initial_pose.position.z = box_temp.z_pose  # 초기 위치 설정 (z)


            euler_angles = [0.0, 0.0, 3.14]  # radians

            # 오일러 각을 쿼터니언으로 변환
            r = R.from_euler('xyz', euler_angles)  # 'xyz'는 회전 순서
            quaternion = r.as_quat()  # [x, y, z, w]


            request.initial_pose.orientation.x = quaternion[0]  # 방향 설정 (쿼터니언)
            request.initial_pose.orientation.y = quaternion[1]  # 방향 설정 (쿼터니언)
            request.initial_pose.orientation.z = quaternion[2]  # 방향 설정 (쿼터니언)
            request.initial_pose.orientation.w = quaternion[3]  # 방향 설정 (쿼터니언)


            # 서비스 호출
            future = self.spawn_client.call_async(request)
            future.add_done_callback(self.callback_spawn_response)

        sleep(3)

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

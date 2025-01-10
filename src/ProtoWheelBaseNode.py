from rclpy import Node
from std_msgs.msg import String

import xml.etree.ElementTree as ET


class ProtoWheelBaseNode(Node):

    def __init__(self, node_name, *, 
                context = None, cli_args = None, namespace = None, 
                use_global_arguments = True, enable_rosout = True, 
                start_parameter_services = True, parameter_overrides = None, 
                allow_undeclared_parameters = False, 
                automatically_declare_parameters_from_overrides = False,
                wheel_data = ""):
        
        super().__init__(node_name, context=context, cli_args=cli_args, namespace=namespace, use_global_arguments=use_global_arguments, enable_rosout=enable_rosout, start_parameter_services=start_parameter_services, parameter_overrides=parameter_overrides, allow_undeclared_parameters=allow_undeclared_parameters, automatically_declare_parameters_from_overrides=automatically_declare_parameters_from_overrides)

        self.wheel_data = wheel_data

        self.robot_desc_subscriber = self.create_subscription(
            String,
            'robot_description',
            self.callback_robot_desc_parse,
            10,
        )

    def callback_robot_desc_parse(self, msg):
        """
        TEST LOGGER CODE 
        """

        self.get_logger().info(f"message recv : ${msg.data}")
        
        """
        IT COULD BE ERASED
        chw1119
        """

        temp_root = ET.fromstring(msg.data)

        wheel_temp = []

        for i in range(len(self.wheel_data)):
            
            pass



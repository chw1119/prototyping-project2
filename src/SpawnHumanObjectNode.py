#!/usr/bin/env python3

from BoxObject import BoxObject


class HumanObject(BoxObject):


    def __init__(self, name="OBJECT__", x_pose=0, y_pose=0, z_pose=0, yaw=0, pitch=0, roll=0, x_vector=0, y_vector=0, z_vector=0, xml_string=""):
        super().__init__(name, x_pose, y_pose, z_pose, yaw, pitch, roll, x_vector, y_vector, z_vector, xml_string)

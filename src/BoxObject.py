import os
import xml.etree.ElementTree as ET

class BoxObject:

    def __init__(self,
        name = "OBJECT__", 
        x_pose = 0, y_pose = 0, z_pose = 0, 
        yaw = 0, pitch = 0, roll = 0,
        x_vector = 0, y_vector = 0, z_vector = 0):
        
        self.name = name
        
        self.x_pose = x_pose
        self.y_pose = y_pose
        self.z_pose = z_pose

        self.yaw = yaw
        self.pitch = pitch
        self.roll = roll

        self.x_vector = x_vector
        self.y_vector = y_vector
        self.z_vector = z_vector

    def animate(self):

        self.x_pose += self.x_vector
        self.y_pose += self.y_vector
        self.z_pose += self.z_vector
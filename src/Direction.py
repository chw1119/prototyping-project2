
class AbsoluteDirection:
    def __init__(self, rel_direction = None, current_xyz = (0,0,0)):

        self.x = current_xyz[0]
        self.y = current_xyz[1]
        self.z = current_xyz[2]
        
        if rel_direction is not None:
        
            self.x_vec = self.x + rel_direction.x_vec
            self.y_vec = self.y + rel_direction.y_vec
            self.z_vec = self.z + rel_direction.z_vec
            

class Direction:
    def __init__(self,
        x_vec = 0.0, y_vec = 0.0, z_vec = 0.0):
        
        self.x_vec = x_vec
        self.y_vec = y_vec
        self.z_vec = z_vec
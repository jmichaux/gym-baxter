class Nullspace():
    def __init__(self):
        self.ll = [-2] * 19
        self.ul = [2] * 19
        self.rp = [0] * 19
        self.jr = [2] * 19
        self.jd = None

class PyBulletConfig():
    def __init__(self):
        self.max_force = 0
        self.max_velocity = 0
        self.position_gain = 0
        self.velocity_gain = 0
        self.nullspace = Nullspace()
        self.initial_pose = [
            # right arm
            [12, 0.00], [13, -0.55], [14, 0.00],[15, 0.75],
            [16, 0.00], [18, 1.26], [19, 0.00],
            # right gripper
            [27, -0.0052], [29, -0.0052],
            # left arm
            [34, 0.00], [35, -0.55], [36, 0.00], [37, 0.75],
            [38, 0.00], [40, 1.26], [41, 0.00],
            # left gripper
            [49, -0.0052],
            [51, -0.0052]]

    def set_random_pose(self):
        pass

class ROSConfig():
    def __init__(self):
        self.all_joint_ranges = {
            'left_s0' : {'min': -1.7016, 'max': 1.7016},
            'left_s1' : {'min': -2.147, 'max': 2.147},
            'left_e0' : {'min': -3.0541, 'max': 3.0541 },
            'left_e1' : {'min': -0.05, 'max': 2.618},
            'left_w0' : {'min': -3.059, 'max': 3.059 },
            'left_w1' : {'min': -1.5707, 'max': 2.094},
            'left_w2' : {'min': -3.059, 'max': 3.059},
            'right_s0' : {'min': -1.7016, 'max': 1.7016},
            'right_s1' : {'min': -2.147, 'max': 2.147},
            'right_e0' : {'min': -3.0541, 'max': 3.0541},
            'right_e1' : {'min': -0.05, 'max': 2.618},
            'right_w0' : {'min': -3.059, 'max': 3.059 },
            'right_w1' : {'min': -1.5707, 'max': 2.094},
            'right_w2' : {'min': -3.059, 'max': 3.059 }}

        self.left_joint_ranges = {
            'left_s0' : {'min': -1.7016, 'max': 1.7016},
            'left_s1' : {'min': -2.147, 'max': 2.147},
            'left_e0' : {'min': -3.0541, 'max': 3.0541 },
            'left_e1' : {'min': -0.05, 'max': 2.618},
            'left_w0' : {'min': -3.059, 'max': 3.059 },
            'left_w1' : {'min': -1.5707, 'max': 2.094},
            'left_w2' : {'min': -3.059, 'max': 3.059}}

        self.right_joint_ranges = {
            'right_s0' : {'min': -1.7016, 'max': 1.7016},
            'right_s1' : {'min': -2.147, 'max': 2.147},
            'right_e0' : {'min': -3.0541, 'max': 3.0541},
            'right_e1' : {'min': -0.05, 'max': 2.618},
            'right_w0' : {'min': -3.059, 'max': 3.059 },
            'right_w1' : {'min': -1.5707, 'max': 2.094},
            'right_w2' : {'min': -3.059, 'max': 3.059 }}
        self.initial_pose = None

    def set_random_pose(self):
        pass

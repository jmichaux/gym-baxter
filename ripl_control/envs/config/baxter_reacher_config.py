class Nullspace():
    def __init__(self):
        # self.ll = [-2] * 19
        # self.ul = [2] * 19
        # self.rp = [0] * 19
        # self.jr = [2] * 19
        # self.jd = None
        self.ll = 0
        self.ul = 0
        self.rp = 0
        self.jr = 0
        self.jd = None

class PyBulletConfig():
    def __init__(self):
        self.max_force = 0
        self.max_velocity = 0
        self.position_gain = 0
        self.velocity_gain = 0
        self.target_velocity = 0
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
    def __init__(self,
                 ee_ranges=None,
                 joint_position_ranges=None,
                 joint_velocity_ranges=None):

        self.right_joint_names = ['right_s0','right_s1','right_e0','right_e1','right_w0','right_w1','right_w2']
        self.left_joint_names = ['left_s0','left_s1','left_e0','left_e1','left_w0','left_w1','left_w2']
        self.joint_names = {'right': self.right_joint_names,
                            'left': self.left_joint_names,
                            'both': self.left_joint_names + self.right_joint_names}

        self.joint_position_ranges = {}
        self.joint_position_ranges['left'] = {
            'left_s0' : {'min': -1.7016, 'max': 1.7016},
            'left_s1' : {'min': -2.147, 'max': 2.147},
            'left_e0' : {'min': -3.0541, 'max': 3.0541 },
            'left_e1' : {'min': -0.05, 'max': 2.618},
            'left_w0' : {'min': -3.059, 'max': 3.059 },
            'left_w1' : {'min': -1.5707, 'max': 2.094},
            'left_w2' : {'min': -3.059, 'max': 3.059}}

        self.joint_position_ranges['right'] = {
            'right_s0' : {'min': -1.7016, 'max': 1.7016},
            'right_s1' : {'min': -2.147, 'max': 2.147},
            'right_e0' : {'min': -3.0541, 'max': 3.0541 },
            'right_e1' : {'min': -0.05, 'max': 2.618},
            'right_w0' : {'min': -3.059, 'max': 3.059 },
            'right_w1' : {'min': -1.5707, 'max': 2.094},
            'right_w2' : {'min': -3.059, 'max': 3.059}}

        self.joint_position_ranges['both'] = {
            'left_s0' : {'min': -1.7016, 'max': 1.7016},
            'left_s1' : {'min': -2.147, 'max': 2.147},
            'left_e0' : {'min': -3.0541, 'max': 3.0541 },
            'left_e1' : {'min': -0.05, 'max': 2.618},
            'left_w0' : {'min': -3.059, 'max': 3.059 },
            'left_w1' : {'min': -1.5707, 'max': 2.094},
            'left_w2' : {'min': -3.059, 'max': 3.059},
            'right_s0' : {'min': -1.7016, 'max': 1.7016},
            'right_s1' : {'min': -2.147, 'max': 2.147},
            'right_e0' : {'min': -3.0541, 'max': 3.0541 },
            'right_e1' : {'min': -0.05, 'max': 2.618},
            'right_w0' : {'min': -3.059, 'max': 3.059 },
            'right_w1' : {'min': -1.5707, 'max': 2.094},
            'right_w2' : {'min': -3.059, 'max': 3.059}}

        self.initial_joint_positions = {
            'left': [0.0, -0.55, 0.0, 0.75, 0.0, 1.26, 0.0],
            'right': [0.0, -0.55, 0.0, 0.75, 0.0, 1.26, 0.0],
            'both': [0.0, -0.55, 0.0, 0.75, 0.0, 1.26, 0.0] * 2
        }

        # max speeds
        #s0,s1,e0,e1 - 2
        #w0,w1,w2 - 4
        self.joint_velocity_ranges = {}
        self.joint_velocity_ranges['left'] = {
            'left_s0' : {'min': -2.0, 'max': 2.0},
            'left_s1' : {'min': -2.0, 'max': 2.0},
            'left_e0' : {'min': -2.0, 'max': 2.0 },
            'left_e1' : {'min': -2.0, 'max': 2.0},
            'left_w0' : {'min': -4.0, 'max': 4.0 },
            'left_w1' : {'min': -4.0, 'max': 4.0},
            'left_w2' : {'min': -4.0, 'max': 4.0}}

        self.joint_velocity_ranges['right'] = {
            'right_s0' : {'min': -2.0, 'max': 2.0},
            'right_s1' : {'min': -2.0, 'max': 2.0},
            'right_e0' : {'min': -2.0, 'max': 2.0},
            'right_e1' : {'min': -2.0, 'max': 2.0},
            'right_w0' : {'min': -4.0, 'max': 4.0},
            'right_w1' : {'min': -4.0, 'max': 4.0},
            'right_w2' : {'min': -4.0, 'max': 4.0}}

        self.joint_velocity_ranges['both'] = {
            'left_s0' : {'min': -2.0, 'max': 2.0},
            'left_s1' : {'min': -2.0, 'max': 2.0},
            'left_e0' : {'min': -2.0, 'max': 2.0 },
            'left_e1' : {'min': -2.0, 'max': 2.0},
            'left_w0' : {'min': -4.0, 'max': 4.0 },
            'left_w1' : {'min': -4.0, 'max': 4.0},
            'left_w2' : {'min': -4.0, 'max': 4.0},
            'right_s0' : {'min': -2.0, 'max': 2.0},
            'right_s1' : {'min': -2.0, 'max': 2.0},
            'right_e0' : {'min': -2.0, 'max': 2.0},
            'right_e1' : {'min': -2.0, 'max': 2.0},
            'right_w0' : {'min': -4.0, 'max': 4.0},
            'right_w1' : {'min': -4.0, 'max': 4.0},
            'right_w2' : {'min': -4.0, 'max': 4.0}}

        self.max_joint_speed = 0
        self.max_joint_accel = 0

    def set_ee_ranges(self):
        pass

    def set_joint_position_ranges(self):
        pass

    def set_joint_velocity_ranges(self):
        pass

    def get_random_pose(self):
        pass

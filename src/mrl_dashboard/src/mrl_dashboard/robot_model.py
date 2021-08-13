class RobotModel:
    IDLE = 0
    MANUAL = 1
    AUTONOMOUS = 2

    def __init__(self):
        self.status = True
        self.name = ''
        self.type = 'p3at'
        self.init_pos_x = 0
        self.init_pos_y = 0
        self.init_pos_z = 0
        self.init_rot_x = 0
        self.init_rot_y = 0
        self.init_rot_z = 0
        self.rgb_camera_topic = ''
        self.thermal_camera_topic = ''
        self.nav_strategy = ''
        self.job_type = ''

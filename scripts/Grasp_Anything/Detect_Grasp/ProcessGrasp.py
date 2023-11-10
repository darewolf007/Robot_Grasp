class Robot_param(object):
    def __init__(self):
        pass
    def _read_eyehand(self):
        pass
    def _read_camera_intrinsics(self):
        pass
    def _read_robot_param(self):
        pass


class Process_Grasp(object):
    def __init__(self):
        self.camera_K = None
        self.camera_D = None
        self.eyehand_T = None
        self.eyehand_R = None
        self.image_index = 0

    def run(self):
        pass
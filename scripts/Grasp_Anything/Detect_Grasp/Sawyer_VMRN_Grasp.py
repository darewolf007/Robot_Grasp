from Grasp_Anything.Detect_Grasp.ProcessGrasp import Process_Grasp
from Grasp_Anything.Scene_Detector.VMRN_detector import VMRN_detector
from Grasp_Anything.utils.Robot_state_transform import transform_uv_to_xy, transfrom_angle_to_ri
from Grasp_Anything.Robot_Tools.Camera_Ros1 import KinectDK
from Grasp_Anything.utils.python_function import init_logging, read_yaml_file
from Grasp_Anything.utils import Robot_state_transform
from scipy.spatial.transform import Rotation
import scipy.io as scio
import cv2
import numpy as np
import os
import Grasp_Anything.Configs
import logging

class VMRN_Grasp(Process_Grasp):
    def __init__(self, config_path):
        super().__init__()
        self.config_param = read_yaml_file(config_path)
        self.camera = KinectDK()
        self.set_path()
        self.get_camera_param()
        self.init_log()
        self.load_eyehand()
        self.vmrn = VMRN_detector(self.vmrn_path, self.image_path)

    def init_log(self):
        init_logging(self.config_param.get("detector_name"), self.log_path)
        self.logger = logging.getLogger(self.config_param.get("detector_name"))
        self.logger.info("Starting VMRN Grasp")

    def set_path(self):
        base_path = os.path.dirname(Grasp_Anything.Configs.__file__)
        self.log_path = base_path + self.config_param.get("log_file_path")
        self.image_path = base_path + self.config_param.get("image_path")
        self.vmrn_path = base_path + self.config_param.get("vmrn_detector_yaml")
        self.eye_hand_path = base_path + self.config_param.get("eye-hand_path")

    def reset(self):
        self.image_index = 0

    def load_eyehand(self):
        eye_hand_info = read_yaml_file(self.eye_hand_path)
        self.eyehand_mode = 'eye_on_hand' if eye_hand_info['parameters']['eye_on_hand'] else 'eye_on_base'
        self.eyehand_T = np.array([eye_hand_info['transformation']['x'], eye_hand_info['transformation']['y'], eye_hand_info['transformation']['z']]).reshape([3])
        self.eyehand_R = np.array([eye_hand_info['transformation']['qx'], eye_hand_info['transformation']['qy'], eye_hand_info['transformation']['qz'], eye_hand_info['transformation']['qw']])
        self.eyehand_R = np.array(Robot_state_transform.transform_quaternion_to_matrix(self.eyehand_R))

    def get_image(self, index = 0, is_write = True):
        color = self.camera.queue_color.get(timeout=5.0)
        depth = self.camera.queue_depth.get(timeout=5.0)
        color_remap = self.remap_img(color, 'rgb')
        depth_remap = self.remap_img(depth, 'depth')
        if is_write:
            cv2.imwrite(self.vmrn.image_path + 'example_' + str(index) + '.jpg', color_remap)
        return color_remap, depth_remap

    def get_camera_param(self):
        self.camera_K = np.asarray(self.camera.rgb_info.K).reshape(3, 3)
        self.camera_D = np.asarray(self.camera.rgb_info.D)

    def remap_img(self, img, type):
        size = img.shape[:2][::-1]
        map1, map2 = cv2.initUndistortRectifyMap(self.camera_K, self.camera_D, None, None, size, cv2.CV_32FC1)
        if type == 'rgb':
            img_remap = cv2.remap(img, map1, map2, cv2.INTER_CUBIC)
        else:
            img_remap = cv2.remap(img, map1, map2, cv2.INTER_NEAREST)
        return img_remap

    def _run_once(self):
        color_img, depth_img = self.get_image(self.image_index)
        self.vmrn.run_detector(self.image_index)
        if self.vmrn.relation_result is not None and self.vmrn.relation_result.current_target.id != -1:
            if len(self.vmrn.grasp_result)>0:
                obj_id = self.vmrn.relation_result.current_target.id
                grasps = self.vmrn.grasp_result[obj_id].obj_grasp.grasp
                uv = [int(grasps[0]), int(grasps[1]), 1]
                depth = depth_img[uv[1]][uv[0]].astype(np.float32)
                grasp_center = transform_uv_to_xy(self.eyehand_R, self.eyehand_T, self.camera_K, uv, depth)
                grasp_ori = transfrom_angle_to_ri(grasps[2])
                self.image_index += 1
                return grasp_center, grasp_ori

    def run(self, request):
        if request.flag_detect:
            return self._run_once()



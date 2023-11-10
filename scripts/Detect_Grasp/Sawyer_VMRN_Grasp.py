from .ProcessGrasp import Process_Grasp
from Scene_Detector.VMRN_detector import VMRN_detector
from utils.Robot_state_transform import transform_uv_to_xy, transfrom_angle_to_ri
from Robot_Tools.Camera_Ros1 import KinectDK
import argparse
import scipy.io as scio
import cv2
import numpy as np

def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--limb", type=str, default="right", help="which robot arm to process grasp")
    args = parser.parse_args()
    return args

class VMRN_Grasp(Process_Grasp):
    def __init__(self):
        self.vmrn = VMRN_detector()
        self.camera = KinectDK()
        self.eye_hand_path = "/home/haowensun/robot/vmrn/change_vmrn/dataset/calibrate_para/extpara.mat"
        self.camera_K = None
        self.camera_D = None
        self.eyehand_T = None
        self.eyehand_R = None
        self.get_camera_param()
        self.image_index = 0

    def rest(self):
        self.image_index = 0

    def load_eyehand(self):
        self.eye_hand_info = scio.loadmat(self.eye_hand_path)
        self.eyehand_T, self.eyehand_R = self.eye_hand_info["T"].reshape([3]), self.eye_hand_info["R"].reshape([3, 3])

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
        self.vmrn.run_detector()
        if self.vmrn.relation_result.current_target.id is not -1:
            if len(self.vmrn.grasp_result)>0:
                id = self.vmrn.relation_result.current_target.id
                grasps = self.vmrn.grasp_result[id].obj_grasp.grasp
                uv = [int(grasps[0]), int(grasps[1]), 1]
                depth = depth_img[uv[1]][uv[0]].astype(np.float32)
                grasp_center = transform_uv_to_xy(self.eyehand_T, self.eyehand_R, self.camera_K, uv, depth)
                grasp_ori = transfrom_angle_to_ri(grasps[2])
                self.image_index += 1
                return grasp_center, grasp_ori

    def run(self, request):
        if request.flag_detect:
            return self._run_once()



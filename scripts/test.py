from Grasp_Anything.Scene_Detector.VMRN_detector import VMRN_detector
import Grasp_Anything.Configs
import os
if __name__ == "__main__":
    test_vmrn = VMRN_detector("./Grasp_Anything/Configs/VMRN_Detector.yaml")
    print(os.path.dirname(Grasp_Anything.Configs.__file__))

    # test_vmrn.run_detector(0)
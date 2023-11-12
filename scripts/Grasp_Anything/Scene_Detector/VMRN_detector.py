from Grasp_Anything.Scene_Detector.Base_detector import Scene_Detector
from Grasp_Anything.Scene_Detector.Scene_Data.VMRN_data import VMRN_Grasp_Result, VMRN_Relation_Result, Relation_Target, bbox2d, VMRN_Grasp
from Grasp_Anything.utils.python_function import init_logging, read_yaml_file
import httpx
import asyncio
import logging


class VMRN_detector(Scene_Detector):

    def __init__(self, config_path, image_path = None):
        super().__init__()
        self.config_param = read_yaml_file(config_path)
        if image_path is None:
            self.image_path = self.config_param.get("image_path")
        else:
            self.image_path = image_path

    def data_process(self, data):
        grasp_result = []
        for i in range(len(data['obj_cls'])):
            grasp_result.append(VMRN_Grasp_Result(data['obj_cls'][i], bbox2d(data['obj_bbox_xyxy'][i]), VMRN_Grasp(data['obj_grasp_grasp'][i])))
        relation_result = VMRN_Relation_Result(Relation_Target(data['relation_result'][-1]['current_target'][0], data['relation_result'][-1]['current_target'][1]))
        logging.info("VMRN_Grasp_Result obj_cls is {}".format(grasp_result.obj_cls))
        logging.info("VMRN_Grasp_Result obj_bbox_xyxy is {}".format(grasp_result.obj_bbox))
        logging.info("VMRN_Grasp_Result obj_grasp_grasp is {}".format(grasp_result.obj_grasp))
        logging.info("VMRN_Relation_Result relation_result is {}".format(relation_result.current_target))
        return grasp_result, relation_result

    async def connect_network(self, index):
        async with httpx.AsyncClient(verify=False) as client:
            img_path = self.image_path + 'example_' + str(index) + '.jpg'
            files = {"image": (img_path, open(img_path, "rb"))}
            timeout = httpx.Timeout(self.config_param.get("wait_time"))
            server_host = self.config_param.get("server_host")
            server_port = self.config_param.get("server_port")
            username = self.config_param.get("username")
            password = self.config_param.get("password")
            response = await client.post(f"{server_host}:{server_port}/upload", files=files,
                                         auth=(username, password), timeout=timeout)
            if response.status_code == 200:
                data = response.json()
                logging.info("Response data obtained from the server")
                self.grasp_result, self.relation_result = self.data_process(data)
            else:
                logging.info("POST request failed. status code: {}".format(response.status_code))

    def run_detector(self, index):
        asyncio.run(self.connect_network(index))


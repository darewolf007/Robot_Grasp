from .Base_detector import Scene_Detector
from .Scene_Data.VMRN_data import VMRN_Grasp_Result, VMRN_Relation_Result
import httpx
import asyncio
from utils.python_function import init_logging
import logging


class VMRN_detector(Scene_Detector):

    def __init__(self):
        super().__init__()
        self.server_host = "http://10.184.17.177"
        self.server_port = 8000
        self.username = "sunhaowen"
        self.password = "sunhaowen"
        self.wait_time = 60
        self.image_path = ""
        self.detector_name = "VMRN"
        self.log_file_path = "/home/haowen/hw_Robot_code/Robot_Grasp/tmp/log/"
        init_logging(self.detector_name, self.log_file_path)
        self.grasp_result = None
        self.relation_result = None

    def data_process(self, data):
        grasp_result = VMRN_Grasp_Result(data['obj_cls'], data['obj_bbox_xyxy'], data['obj_grasp_grasp'])
        relation_result = VMRN_Relation_Result(data['relation_result'][-1]['current_target'][0])
        return grasp_result, relation_result

    async def connect_network(self):
        async with httpx.AsyncClient(verify=False) as client:
            files = {"image": (self.image_path, open(self.image_path, "rb"))}
            timeout = httpx.Timeout(self.wait_time)
            response = await client.post(f"{self.server_host}:{self.server_port}/upload", files=files,
                                         auth=(self.username, self.password), timeout=timeout)
            if response.status_code == 200:
                data = response.json()
                self.grasp_result, self.relation_result = self.data_process(data)
                logging.info("Response data obtained from the server: {}".join(data))
            else:
                logging.info("POST request failed. status code: {}".join(response.status_code))

    def run_detector(self):
        asyncio.run(self.connect_network())


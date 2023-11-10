from dataclasses import dataclass
from typing import List

@dataclass
class Grasp_Detection:
    detect_grasp: bool
    grasp_position: List[float]
    rest_obj_num: int
    move_flag: bool
    detect_time:float
    obj_class:int
    stand_flag:bool

@dataclass
class bbox2d:
    xyxy: List[float]

@dataclass
class Result:
    prob: float
    bbox: bbox2d
    label: str
    id: int

class Relation_Target:
    id: int
    obj_cls: str
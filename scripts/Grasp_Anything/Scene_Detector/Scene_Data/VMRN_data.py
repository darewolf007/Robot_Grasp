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

@dataclass
class Relation_Target:
    id: int
    obj_cls: str

@dataclass
class VMRN_Grasp:
    grasp: List[float]

@dataclass
class VMRN_Grasp_Result:
    obj_cls: str
    obj_bbox: bbox2d
    obj_grasp: VMRN_Grasp

@dataclass
class VMRN_Relation_Result:
    current_target: Relation_Target


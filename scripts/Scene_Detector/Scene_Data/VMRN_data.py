from dataclasses import dataclass
from typing import List
from .Base_data import bbox2d, Relation_Target

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


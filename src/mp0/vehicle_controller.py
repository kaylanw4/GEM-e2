from enum import Enum, auto
import copy
from typing import List

class PedestrianMode(Enum):
    Normal=auto()

class VehicleMode(Enum):
    Normal = auto()
    Brake = auto()
    Accel = auto()
    HardBrake = auto()

class State:
    x: float 
    y: float 
    theta: float 
    v: float 
    agent_mode: VehicleMode 

    def __init__(self, x, y, theta, v, agent_mode: VehicleMode):
        pass 

def decisionLogic(ego: State, other: State):
    output = copy.deepcopy(ego)

    # breakpoint()
    # TODO: Edit this part of decision logic
    threshold = 10
    
    if ego.agent_mode == VehicleMode.Normal:
        output.agent_mode = VehicleMode.Accel
    elif ego.agent_mode == VehicleMode.Accel:
        if other.dist <= ego.v ** 2 / 10 + threshold:
            output.agent_mode = VehicleMode.HardBrake
    elif ego.agent_mode == VehicleMode.Brake:
        if other.dist > 20 + threshold:
            output.agent_mode = VehicleMode.Accel
    elif ego.agent_mode == VehicleMode.HardBrake:
        if other.dist > 10 + threshold:
            output.agent_mode = VehicleMode.Accel
    # output.agent_mode = VehicleMode.HardBrake
    # if other.dist < 21 and ego.x > 165: 
    #     output.agent_mode = VehicleMode.HardBrake
    ###########################################

    assert other.dist > 2.0
    # safety distance is larger than 8
    # assert other.dist > 5.0

    return output 
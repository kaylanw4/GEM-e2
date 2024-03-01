from wsgiref.validate import PartialIteratorWrapper
from mp0 import VehicleAgent, PedestrianAgent, VehiclePedestrianSensor, verify_refine, eval_velocity, sample_init
from verse import Scenario, ScenarioConfig
from vehicle_controller import VehicleMode, PedestrianMode

from verse.plotter.plotter2D import *
from verse.plotter.plotter3D_new import *
import plotly.graph_objects as go
import copy

def get_corner_cases():
    cases = []
    for a in [-5, 5]:
        for b in [-5, 5]:
            for c in [5, 10]:
                for d in [165, 175]:
                    for e in [-55, -50]:
                        cases.append({
                            "car": [[a,b,0,c],[a,b,0,c]],
                            "pedestrian": [[d,e,0,3],[d,e,0,3]]
                        })
    print(cases)
    return cases

def single_run(init_car, init_pedestrian, scenario):
    scenario.set_init_single(
        'car', init_car,(VehicleMode.Normal,)
    )
    scenario.set_init_single(
        'pedestrian', init_pedestrian, (PedestrianMode.Normal,)
    )

    return scenario.simulate(50, 0.1)

if __name__ == "__main__":
    import os 
    script_dir = os.path.realpath(os.path.dirname(__file__))
    input_code_name = os.path.join(script_dir, "vehicle_controller.py")
    vehicle = VehicleAgent('car', file_name=input_code_name)
    pedestrian = PedestrianAgent('pedestrian')

    scenario = Scenario(ScenarioConfig(init_seg_length=1, parallel=False))

    scenario.add_agent(vehicle) 
    scenario.add_agent(pedestrian)
    scenario.set_sensor(VehiclePedestrianSensor())

    traces = []

    cases = get_corner_cases()

    for corner in cases:    
        init_car = corner["car"]
        init_pedestrian = corner["pedestrian"]
        trace = single_run(init_car, init_pedestrian, scenario)
        # fig = go.Figure()
        # fig = simulation_tree_3d(trace, fig,\
        #                           0,'time', 1,'x',2,'y')
        # fig.show()
        traces.append(trace)

    avg_vel, unsafe_frac, unsafe_init = eval_velocity(traces)
    print(f"###### Average velocity {avg_vel}, Unsafe fraction {unsafe_frac}, Unsafe init {unsafe_init}")
    # -----------------------------------------

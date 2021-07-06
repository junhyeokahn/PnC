import sys
import os
sys.path.append(os.getcwd() + '/build')
sys.path.append(os.getcwd() + 'plot')
sys.path.append(os.getcwd())
from pinocchio.visualize import MeshcatVisualizer
import pinocchio as pin

model, collision_model, visual_model = pin.buildModelsFromUrdf(
    "robot_model/draco/draco.urdf", "robot_model/draco",
    pin.JointModelFreeFlyer())
viz = MeshcatVisualizer(model, collision_model, visual_model)
try:
    viz.initViewer(open=True)
except ImportError as err:
    print(
        "Error while initializing the viewer. It seems you should install python meshcat"
    )
    print(err)
    exit()
viz.loadViewerModel()
vis_q = pin.neutral(model)
while True:
    viz.display(vis_q)

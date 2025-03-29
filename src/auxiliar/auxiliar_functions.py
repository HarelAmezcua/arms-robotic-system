# This file contains auxiliary functions that are used in the main code.
import numpy as np
import pydot
from IPython.display import SVG, display

from pydrake.solvers import Solve
from pydrake.math import RotationMatrix

from pydrake.multibody.plant import MultibodyPlant
from pydrake.multibody.inverse_kinematics import InverseKinematics

from src.auxiliar.importing_objects import (
    add_custom_robot,
    add_table,
    add_cylinder_obstacle,
)


def LoadRobot(plant: MultibodyPlant):
    """Setup your plant, and return the body corresponding to your
    end-effector."""
    
    robot_model = add_custom_robot(plant)[0]
    table_model = add_table(plant)[0]
    cylinder_model = add_cylinder_obstacle(plant)[0]    


    return plant.GetBodyByName("tool_center_point")

def get_default_position():
    plant = MultibodyPlant(0.0)
    LoadRobot(plant)
    plant.Finalize()
    context = plant.CreateDefaultContext()
    return plant.GetPositions(context)


def VisualizeConnectivity(iris_regions):
    graph = pydot.Dot("IRIS region connectivity")
    keys = list(iris_regions.keys())
    for k in keys:
        graph.add_node(pydot.Node(k))
    for i in range(len(keys)):
        v1 = iris_regions[keys[i]]
        for j in range(i + 1, len(keys)):
            v2 = iris_regions[keys[j]]
            if v1.IntersectsWith(v2):
                graph.add_edge(pydot.Edge(keys[i], keys[j], dir="both"))
    display(SVG(graph.create_svg()))
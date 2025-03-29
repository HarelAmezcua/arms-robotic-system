import numpy as np
import time
from typing import Dict
from collections import namedtuple
from copy import copy

from pydrake.all import Context, Meshcat, SceneGraph, MeshcatPoseSliders
from pydrake.geometry import Rgba, Sphere
from pydrake.geometry.optimization import HPolyhedron
from pydrake.math import RigidTransform
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph
from pydrake.systems.framework import DiagramBuilder
from pydrake.visualization import AddDefaultVisualization

from src.auxiliar.auxiliar_functions import LoadRobot
from src.auxiliar.gcs_helper import GenerateRegion

from pydrake.multibody.plant import MultibodyPlant
from pydrake.multibody.inverse_kinematics import InverseKinematics
from pydrake.solvers import Solve

from pydrake.math import RigidTransform, RotationMatrix


RpyXyz = namedtuple("RpyXyz", ("roll", "pitch", "yaw", "x", "y", "z"))


# Creates an indicator "light" (a colored sphere in the workspace)
# TODO(russt): use meshcat.SetProperty(path, "color", rgba) instead
def IrisIndicator(
    scene_graph: SceneGraph,
    scene_graph_context: Context,
    meshcat: Meshcat,
    iris_regions: Dict[str, HPolyhedron],
    p_WIndicator: list,
    q
):
    radius = 0.1

    meshcat.SetTransform("iris_indicator", RigidTransform(p_WIndicator))

    query_object = scene_graph.get_query_output_port().Eval(scene_graph_context)
    has_collisions = query_object.HasCollisions()
    if has_collisions:
        meshcat.SetObject("iris_indicator", Sphere(radius), Rgba(1, 0, 0, 1))

    in_any_region = False
    for r in iris_regions.values():
        if r.PointInSet(q):
            if has_collisions:
                print("You found a counter-example!")
                # TODO(russt): Automatically shrink the iris region.
            elif not in_any_region:
                in_any_region = True
                meshcat.SetObject("iris_indicator", Sphere(radius), Rgba(0, 1, 0, 1))

    if not has_collisions and not in_any_region:
        meshcat.SetObject("iris_indicator", Sphere(radius), Rgba(0.5, 0.5, 0.5, 1))


def MyInverseKinematics(X_WE, plant=None, context=None):
    if not plant:
        plant = MultibodyPlant(0.0)
        LoadRobot(plant)
        plant.Finalize()
    if not context:
        context = plant.CreateDefaultContext()
    
    # Get the end-effector frame
    E = plant.GetBodyByName("tool_center_point").body_frame()

    ik = InverseKinematics(plant, context)

    # Define a larger cuboid for the position constraint
    lower_bound = X_WE.translation() - np.array([0.05, 0.05, 0.05])
    upper_bound = X_WE.translation() + np.array([0.05, 0.05, 0.05])
    p_BQ = np.array([0.0, 0.0, 0.0])  # Constrained point in the end-effector frame

    ik.AddPositionConstraint(
        frameB=E,
        p_BQ=p_BQ,
        frameA=plant.world_frame(),
        p_AQ_lower=lower_bound,
        p_AQ_upper=upper_bound
    )

    # Add a rotation cost instead of a strict orientation constraint
    desired_rotation_matrix = X_WE.rotation()
    ik.AddOrientationCost(
        frameAbar=plant.world_frame(),
        R_AbarA=desired_rotation_matrix,
        frameBbar=E,
        R_BbarB=RotationMatrix.Identity(),
        c=0.0000001
    )

    prog = ik.get_mutable_prog()
    q = ik.q()

    q0 = plant.GetPositions(context)
    prog.AddQuadraticErrorCost(np.identity(len(q)) * 0.001, q0, q)
    prog.SetInitialGuess(q, q0)

    result = Solve(ik.prog())
    if not result.is_success():
        print("IK failed")
        return None
    plant.SetPositions(context, result.GetSolution(q))
    return result.GetSolution(q)


def EndEffectorTeleop(meshcat=None, q=None, iris_regions=None):
    meshcat.Delete()
    meshcat.DeleteAddedControls()

    builder = DiagramBuilder()
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, 0.0)
    ee_body = LoadRobot(plant)
    plant.Finalize()

    # Set up teleop widgets.
    sliders = builder.AddSystem(
        MeshcatPoseSliders(
            meshcat,
            lower_limit=RpyXyz(roll=0, pitch=-0.5, yaw=-np.pi, x=-0.6, y=-0.8, z=0.0),
            upper_limit=RpyXyz(
                roll=2 * np.pi, pitch=np.pi, yaw=np.pi, x=0.8, y=0.3, z=1.1
            ),
        )
    )
    print(
        """
Control the end-effector position/orientation with the sliders or the keyboard:
  ┌───────┬┬───────┬┬───────┐      ┌───────┬┬───────┬┬───────┐
  │   Q   ││   W   ││   E   │      │   U   ││   I   ││   O   │
  │ roll- ││ pitch+││ roll+ │      │   z-  ││   y+  ││   z+  │
  ├───────┼┼───────┼┼───────┤      ├───────┼┼───────┼┼───────┤
  ├───────┼┼───────┼┼───────┤      ├───────┼┼───────┼┼───────┤
  │   A   ││   S   ││   D   │      │   J   ││   K   ││   L   │
  │  yaw- ││ pitch-││  yaw+ │      │   x-  ││   y-  ││   x+  │
  └───────┴┴───────┴┴───────┘      └───────┴┴───────┴┴───────┘
"""
    )

    AddDefaultVisualization(builder, meshcat)

    diagram = builder.Build()
    context = diagram.CreateDefaultContext()
    plant_context = plant.GetMyMutableContextFromRoot(context)
    scene_graph_context = scene_graph.GetMyContextFromRoot(context)
    sliders_context = sliders.GetMyContextFromRoot(context)
    
    if len(q) == 0:
        q = plant.GetPositions(plant_context)
    plant.SetPositions(plant_context, q)
    X_WE = plant.EvalBodyPoseInWorld(plant_context, ee_body)
    sliders.SetPose(X_WE)

    iris_button_name = "Compute new IRIS region"
    meshcat.AddButton(iris_button_name)
    iris_button_clicks = 0

    stop_button_name = "Stop End Effector Teleop"
    print(
        f"Press the '{stop_button_name}' button in Meshcat to continue or press 'Escape'"
    )
    meshcat.AddButton(stop_button_name, "Escape")

    diagram.ForcedPublish(context)

    while meshcat.GetButtonClicks(stop_button_name) < 1:
        # Check if the sliders have changed.
        new_X_WE = copy(sliders.get_output_port().Eval(sliders_context))

        if meshcat.GetButtonClicks(iris_button_name) > iris_button_clicks:
            iris_button_clicks = meshcat.GetButtonClicks(iris_button_name)
            # TODO(russt): Get the name from meshcat (#19666)
            region_name = "EETeleopRegion"
            region_num = 0
            while f"{region_name}{region_num}" in iris_regions.keys():
                region_num += 1
            meshcat.AddButton("Generating region (please wait)")
            GenerateRegion(
                f"{region_name}{region_num}", plant.GetPositions(plant_context)
            )
            meshcat.DeleteButton("Generating region (please wait)")
        elif X_WE.IsExactlyEqualTo(new_X_WE):
            time.sleep(1e-3)
            continue

        X_WE = new_X_WE
        q_ik = MyInverseKinematics(X_WE, plant, plant_context)
        if q_ik is not None:
            q = q_ik
            plant.SetPositions(plant_context, q)
            IrisIndicator(
                scene_graph,
                scene_graph_context,
                meshcat,
                iris_regions,
                p_WIndicator=[1, 1, 1],
                q=q,
            )
            diagram.ForcedPublish(context)

    meshcat.DeleteAddedControls()
    q = plant.GetPositions(plant_context)
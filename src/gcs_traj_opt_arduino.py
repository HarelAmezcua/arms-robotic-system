import time
import numpy as np

from pydrake.multibody.plant import AddMultibodyPlantSceneGraph
from pydrake.systems.framework import DiagramBuilder
from pydrake.geometry.optimization import GraphOfConvexSetsOptions, Point  # type: ignore
from pydrake.planning import GcsTrajectoryOptimization
from pydrake.visualization import AddDefaultVisualization

from src.load_scene_objects import LoadRobot

import random

from src.data_streamer import DataStreamer



def PublishPositionTrajectory(
    trajectory, root_context, plant, visualizer, time_step=1.0 / 33.0
):
    """
    Args:
        trajectory: A Trajectory instance.
    """
    
    print("running animation")
    plant_context = plant.GetMyContextFromRoot(root_context)
    visualizer_context = visualizer.GetMyContextFromRoot(root_context)

    visualizer.StartRecording(False)

    trajectory_np = np.zeros([5,1])

    for t in np.append(
        np.arange(trajectory.start_time(), trajectory.end_time(), time_step),
        trajectory.end_time(),
    ):
        root_context.SetTime(t)
        plant.SetPositions(plant_context, trajectory.value(t))   
        # Ensure trajectory.value(t) is a NumPy array and reshape it
        trajectory_value_np = np.array(trajectory.value(t)).reshape(-1, 1)
        trajectory_np = np.append(trajectory_np, trajectory_value_np,1)        
        visualizer.ForcedPublish(visualizer_context)

    visualizer.StopRecording()
    visualizer.PublishRecording()
    print("Trajectory shape: ", trajectory_np.shape)

    time.sleep(trajectory.end_time()-trajectory.start_time() + 4.0)

    return trajectory_np


def GcsTrajOpt(q_start, q_goal, iris_regions, meshcat):
    if not iris_regions:
        print(
            "No IRIS regions loaded. Make some IRIS regions then come back and try this again."
        )
        return
    assert len(q_start) == len(q_goal)
    assert len(q_start) == iris_regions[next(iter(iris_regions))].ambient_dimension()

    builder = DiagramBuilder()
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=0.0)
    LoadRobot(plant)
    plant.Finalize()
    AddDefaultVisualization(builder, meshcat)
    diagram = builder.Build()

    gcs = GcsTrajectoryOptimization(len(q_start))
    # TODO(russt): AddRegions should take named regions.
    regions = gcs.AddRegions(list(iris_regions.values()), order=1)
    source = gcs.AddRegions([Point(q_start)], order=0)
    target = gcs.AddRegions([Point(q_goal)], order=0)
    gcs.AddEdges(source, regions)
    gcs.AddEdges(regions, target)
    gcs.AddTimeCost()
    gcs.AddVelocityBounds(
        plant.GetVelocityLowerLimits(), plant.GetVelocityUpperLimits()
    )

    options = GraphOfConvexSetsOptions()
    options.preprocessing = True
    options.max_rounded_paths = 5
    start_time = time.time()
    traj, result = gcs.SolvePath(source, target, options)
    print(f"GCS solved in {time.time() - start_time} seconds")
    if not result.is_success():
        print("Could not find a feasible path from q_start to q_goal")
        return

    PublishPositionTrajectory(
        traj,
        diagram.CreateDefaultContext(),
        plant,
        diagram.GetSubsystemByName("meshcat_visualizer(illustration)"),
    )


def stream_trajectory(trajectory_np, streamer):
    try:
        for i in range(trajectory_np.shape[1]):
            value = trajectory_np[:, i]  # Extract the (5,) value for the current column
            value = np.concatenate(([1], value, [90]))  # Combine scalars and array into one 1D array
            streamer.update_struct(value)
            streamer.send_data()
            time.sleep(1/16)
            streamer.receive_data()

    except KeyboardInterrupt:
        streamer.close_connection()


def demo_trajectory_optimization(iris_regions, iris_seeds, meshcat):
    if not iris_regions:
        print(
            "No IRIS regions loaded. Make some IRIS regions then come back and try this again."
        )
        return
    assert len(iris_regions) > 0

    builder = DiagramBuilder()
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=0.0)
    LoadRobot(plant)
    plant.Finalize()
    AddDefaultVisualization(builder, meshcat)
    diagram = builder.Build()

    for key in iris_seeds:
        iris_seeds[key] = np.array(iris_seeds[key])

    ketchup_on_top = np.array([0.0, 0.65, -1.23, -0.96, 0.0])

    previous_seed = ketchup_on_top

    in_random_seed = False

    try:
        streamer = DataStreamer('/dev/ttyACM1')
        streamer.open_connection()
    except Exception as e:
        print(f"Error opening connection: {e}")        

    for i in range(10):
        random_seed = random.sample(list(iris_seeds.keys()), 1)

        if in_random_seed:
            q_start = ketchup_on_top
            q_goal = iris_seeds[random_seed[0]]
            previous_seed = q_goal
            in_random_seed = False
        else:
            q_start = previous_seed
            q_goal = ketchup_on_top
            in_random_seed = True

        gcs = GcsTrajectoryOptimization(5) # 5 is the dimension of the state space
        # TODO(russt): AddRegions should take named regions.
        regions = gcs.AddRegions(list(iris_regions.values()), order=1)
        source = gcs.AddRegions([Point(q_start)], order=0)
        target = gcs.AddRegions([Point(q_goal)], order=0)
        gcs.AddEdges(source, regions)
        gcs.AddEdges(regions, target)
        gcs.AddTimeCost()
        gcs.AddVelocityBounds(
            plant.GetVelocityLowerLimits(), plant.GetVelocityUpperLimits()
        )

        options = GraphOfConvexSetsOptions()
        options.preprocessing = True
        options.max_rounded_paths = 5
        start_time = time.time()
        traj, result = gcs.SolvePath(source, target, options)
                
        print(f"GCS solved in {time.time() - start_time} seconds")
        if not result.is_success():
            print("Could not find a feasible path from q_start to q_goal")
            return

        print("time: ",traj.start_time(), traj.end_time())

        trajectory_i = PublishPositionTrajectory(
            traj,
            diagram.CreateDefaultContext(),
            plant,
            diagram.GetSubsystemByName("meshcat_visualizer(illustration)"),
        )      

        stream_trajectory(trajectory_i, streamer)


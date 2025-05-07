import time
import numpy as np
import random

from pydrake.multibody.plant import AddMultibodyPlantSceneGraph
from pydrake.systems.framework import DiagramBuilder
from pydrake.geometry.optimization import GraphOfConvexSetsOptions, Point  # type: ignore
from pydrake.planning import GcsTrajectoryOptimization
from pydrake.visualization import AddDefaultVisualization

from src.load_scene_objects import LoadRobot

def PublishPositionTrajectory(
    trajectory, root_context, plant, visualizer, time_step=1.0 / 33.0
):
    """
    Args:
        trajectory: A Trajectory instance.
    """
    plant_context = plant.GetMyContextFromRoot(root_context)
    visualizer_context = visualizer.GetMyContextFromRoot(root_context)

    visualizer.StartRecording(False)
    

    for t in np.append(
        np.arange(trajectory.start_time(), trajectory.end_time(), time_step),
        trajectory.end_time(),
    ):
        root_context.SetTime(t)
        plant.SetPositions(plant_context, trajectory.value(t))  
        visualizer.ForcedPublish(visualizer_context)

    visualizer.StopRecording()
    visualizer.PublishRecording()    
    time.sleep(trajectory.end_time()-trajectory.start_time() + 2.0)

    """
        Call with this:
        PublishPositionTrajectory(
            traj,
            diagram.CreateDefaultContext(),
            plant,
            diagram.GetSubsystemByName("meshcat_visualizer(illustration)"),
        )
    """


def generate_trajectory_array(trajectory):    
    
    delta_t = 0.01

    trajectory_np = np.empty([5,0])

    for t in np.append(
        np.arange(trajectory.start_time(), trajectory.end_time(), delta_t),
        trajectory.end_time(),
    ):
        trajectory_value_np = np.array(trajectory.value(t)).reshape(-1, 1)
        trajectory_np = np.append(trajectory_np, trajectory_value_np,1)        

    print("Trajectory shape: ", trajectory_np.shape)

    return trajectory_np

def stream_trajectory(trajectory_np, streamer, plant, visualizer, root_context):
    """
    Streams the trajectory data to the connected device.

    Args:
        trajectory_np: A numpy array containing the trajectory data.
        streamer: An instance of DataStreamer for sending and receiving data.
    """
    plant_context = plant.GetMyContextFromRoot(root_context)
    visualizer_context = visualizer.GetMyContextFromRoot(root_context)
    try:        
        for i in range(1, trajectory_np.shape[1]):  # Iterate over trajectory columns starting from the second
            value = trajectory_np[:, i].copy()  # Extract and copy the current column to avoid modifying the original
            value[3] = -value[3]  # Invert the 4th element
            value = (value + 1.5708) / 3.14159  # Apply transformations to all elements
            value = np.concatenate(([1], value, [0.5]))  # Prepend and append scalars to the array
            streamer.update_struct(value)  # Update the streamer structure with the transformed value
            streamer.send_data()  # Send the data
            plant.SetPositions(plant_context, trajectory_np[:, i])  
            visualizer.ForcedPublish(visualizer_context)
            time.sleep(1 / 100)  # Wait for a short interval
            streamer.receive_data()  # Receive data from the device


        
        time.sleep(3)  # Wait for a short interval after sending the entire trajectory

    except KeyboardInterrupt:
        print("Streaming interrupted by user.")
        streamer.close_connection()
    except Exception as e:
        print(f"Error during streaming: {e}")
        streamer.close_connection()


def setup_gcs(q_start, q_goal, iris_regions, plant):
    gcs = GcsTrajectoryOptimization(len(q_start))
    regions = gcs.AddRegions(list(iris_regions.values()), order=1)
    source = gcs.AddRegions([Point(q_start)], order=0)
    target = gcs.AddRegions([Point(q_goal)], order=0)
    gcs.AddEdges(source, regions)
    gcs.AddEdges(regions, target)
    gcs.AddTimeCost()
    gcs.AddVelocityBounds(
        plant.GetVelocityLowerLimits(), plant.GetVelocityUpperLimits()
    )
    return gcs, source, target


def demo_trajectory_optimization(iris_regions, iris_seeds, meshcat, streamer):
    """
    Optimizes and streams trajectories between various positions using GCS.

    Args:
        iris_regions: Dictionary of IRIS regions.
        iris_seeds: Dictionary of IRIS seed positions.
        meshcat: Meshcat visualizer instance.
        streamer: DataStreamer instance for sending and receiving data.

    Returns:
        history: A numpy array containing all generated trajectories.
        indexes_list: List of indices marking the end of each trajectory in history.
    """
    if not iris_regions or len(iris_regions) == 0:
        raise ValueError("IRIS regions must not be empty. Load IRIS regions and try again.")

    # Build the diagram with the robot and visualization
    builder = DiagramBuilder()
    plant, _ = AddMultibodyPlantSceneGraph(builder, time_step=0.0)
    LoadRobot(plant)
    plant.Finalize()
    AddDefaultVisualization(builder, meshcat)
    diagram = builder.Build()

    # Convert IRIS seeds to numpy arrays
    iris_seeds = {key: np.array(value) for key, value in iris_seeds.items()}
    ketchup_in_front = np.array([0.0, 0.5, -1.2, -0.8, 0.0])

    previous_seed = iris_seeds["zero"].copy()
    history = np.empty([5, 0])
    indexes_list = [0]

    for i in range(8):
        random_seed = random.choice(list(iris_seeds.keys()))
        q_start, q_goal = (ketchup_in_front, iris_seeds[random_seed]) if i % 2 != 0 else (previous_seed, ketchup_in_front)
        previous_seed = q_goal

        intermediate_goal = iris_seeds['zero'] if random_seed in ['ketchup_left', 'ketchup_right'] else iris_seeds['home']
        trajectory_np = demo_trajectory_between_positions(q_start, intermediate_goal, iris_regions, meshcat, streamer)

        history, indexes_list = _append_trajectory_to_history(trajectory_np, history, indexes_list)
        gcs, source, target = setup_gcs(intermediate_goal, q_goal, iris_regions, plant)

        traj = _solve_gcs(gcs, source, target)
        trajectory_np = generate_trajectory_array(traj)

        history, indexes_list = _append_trajectory_to_history(trajectory_np, history, indexes_list)
        _stream_trajectory_safe(trajectory_np, streamer, plant, diagram)

    final_trajectory = demo_trajectory_between_positions(previous_seed, iris_seeds['zero'], iris_regions, meshcat, streamer)
    history, indexes_list = _append_trajectory_to_history(final_trajectory, history, indexes_list)

    return history, indexes_list


def _append_trajectory_to_history(trajectory_np, history, indexes_list):
    """
    Appends a trajectory to the history and updates the indexes list.

    Args:
        trajectory_np: A numpy array containing the trajectory.
        history: A numpy array containing all trajectories.
        indexes_list: List of indices marking the end of each trajectory in history.

    Returns:
        Updated history and indexes_list.
    """
    history = np.append(history, trajectory_np, axis=1)
    indexes_list.append(history.shape[1])
    return history, indexes_list


def _solve_gcs(gcs, source, target):
    """
    Solves the GCS optimization problem.

    Args:
        gcs: GcsTrajectoryOptimization instance.
        source: Source region.
        target: Target region.

    Returns:
        The solved trajectory.
    """
    options = GraphOfConvexSetsOptions()
    options.preprocessing = True
    options.max_rounded_paths = 5

    start_time = time.time()
    traj, result = gcs.SolvePath(source, target, options)
    print(f"GCS solved in {time.time() - start_time} seconds")

    if not result.is_success():
        raise RuntimeError("Could not find a feasible path from source to target.")

    return traj


def _stream_trajectory_safe(trajectory_np, streamer, plant, diagram):
    """
    Streams a trajectory safely, handling exceptions.

    Args:
        trajectory_np: A numpy array containing the trajectory.
        streamer: DataStreamer instance.
        plant: MultibodyPlant instance.
        diagram: Diagram instance.
    """
    try:
        stream_trajectory(
            trajectory_np,
            streamer,
            plant,
            diagram.GetSubsystemByName("meshcat_visualizer(illustration)"),
            diagram.CreateDefaultContext(),
        )
        time.sleep(1)
    except Exception as e:
        print(f"Error while streaming trajectory: {e}")
    

def demo_trajectory_between_positions(q_start, q_goal, iris_regions, meshcat, streamer):
    """
    Generates and streams a trajectory between two positions using GCS.

    Args:
        q_start: Starting position as a numpy array.
        q_goal: Goal position as a numpy array.
        iris_regions: Dictionary of IRIS regions.
        meshcat: Meshcat visualizer instance.
        streamer: DataStreamer instance for sending and receiving data.

    Returns:
        trajectory_np: A numpy array containing the generated trajectory.
    """
    if not iris_regions:
        raise ValueError("No IRIS regions loaded. Please load IRIS regions and try again.")
    
    assert len(q_start) == len(q_goal), "q_start and q_goal must have the same dimensions."
    assert len(q_start) == iris_regions[next(iter(iris_regions))].ambient_dimension(), \
        "q_start and q_goal dimensions must match the IRIS regions' ambient dimension."

    # Build the diagram with the robot and visualization
    builder = DiagramBuilder()
    plant, _ = AddMultibodyPlantSceneGraph(builder, time_step=0.0)
    LoadRobot(plant)
    plant.Finalize()
    AddDefaultVisualization(builder, meshcat)
    diagram = builder.Build()

    # Setup GCS for trajectory optimization
    gcs, source, target = setup_gcs(q_start, q_goal, iris_regions, plant)

    # Configure GCS options
    options = GraphOfConvexSetsOptions()
    options.preprocessing = True
    options.max_rounded_paths = 5

    # Solve for the trajectory
    start_time = time.time()
    traj, result = gcs.SolvePath(source, target, options)
    print(f"GCS solved in {time.time() - start_time} seconds")

    if not result.is_success():
        raise RuntimeError("Could not find a feasible path from q_start to q_goal.")

    # Generate the trajectory array
    trajectory_np = generate_trajectory_array(traj)

    # Stream the trajectory
    try:
        stream_trajectory(
            trajectory_np,
            streamer,
            plant,
            diagram.GetSubsystemByName("meshcat_visualizer(illustration)"),
            diagram.CreateDefaultContext(),
        )
    except Exception as e:
        print(f"Error while streaming trajectory: {e}")

    return trajectory_np


def record_np_trajectory(list_of_positions, meshcat):
    """
    Creates a linear interpolation between a list of positions and records the trajectory.

    Args:
        list_of_positions: A list of lists, where each inner list represents a position.
        meshcat: Meshcat visualizer instance.
    """
    # Convert list of positions to numpy arrays
    list_of_positions = [np.array(pos) for pos in list_of_positions]

    # Build the diagram with the robot and visualization
    builder = DiagramBuilder()
    plant, _ = AddMultibodyPlantSceneGraph(builder, time_step=0.0)
    LoadRobot(plant)
    plant.Finalize()
    AddDefaultVisualization(builder, meshcat)
    diagram = builder.Build()
    root_context = diagram.CreateDefaultContext()

    plant_context = plant.GetMyContextFromRoot(root_context)
    visualizer = diagram.GetSubsystemByName("meshcat_visualizer(illustration)")
    visualizer_context = visualizer.GetMyContextFromRoot(root_context)

    # Create the interpolated trajectory
    trajectory_np = np.empty((len(list_of_positions[0]), 0))
    for i in range(len(list_of_positions) - 1):
        start_pos = list_of_positions[i]
        end_pos = list_of_positions[i + 1]
        num_steps = 100  # Number of interpolation steps
        for alpha in np.linspace(0, 1, num_steps):
            interpolated_pos = (1 - alpha) * start_pos + alpha * end_pos
            trajectory_np = np.append(trajectory_np, interpolated_pos.reshape(-1, 1), axis=1)

    print("Generated interpolated trajectory with shape:", trajectory_np.shape)

    # Record the trajectory
    visualizer.StartRecording(False)
    time_step = 1.0 / 33.0  # Recording time step
    for t in range(trajectory_np.shape[1]):
        root_context.SetTime(t * time_step)
        plant.SetPositions(plant_context, trajectory_np[:, t])
        visualizer.ForcedPublish(visualizer_context)

    visualizer.StopRecording()
    visualizer.PublishRecording()
    print("Recording complete.")
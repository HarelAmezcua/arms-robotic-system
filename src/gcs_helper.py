import time
import numpy as np
from IPython.display import display

from pydrake.geometry import QueryObject, Rgba, Role, MeshcatVisualizer
from pydrake.geometry.optimization import HPolyhedron, IrisInConfigurationSpace, SaveIrisRegionsYamlFile # type: ignore
from pydrake.solvers import MathematicalProgram, Solve
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph
from pydrake.systems.framework import DiagramBuilder

from src.load_scene_objects import LoadRobot

def ScaleHPolyhedron(hpoly, scale_factor): # Function for scaling an HPolyhedron
    # Shift to the center.
    xc = hpoly.ChebyshevCenter()
    A = hpoly.A()
    b = hpoly.b() - A @ xc
    # Scale
    b = scale_factor * b
    # Shift back
    b = b + A @ xc
    return HPolyhedron(A, b)

def _CheckNonEmpty(region): # Function for checking if a region is non-empty
    prog = MathematicalProgram()
    x = prog.NewContinuousVariables(region.ambient_dimension())
    region.AddPointInSetConstraints(prog, x)
    result = Solve(prog)
    assert result.is_success()

def _CalcRegion(name, seed, config, iris_regions=None, iris_options = None):
    """
    Compute a region based on the provided seed and configuration.
    Returns:
        Reduced region.
    """
    builder = DiagramBuilder()
    plant = AddMultibodyPlantSceneGraph(builder, 0.0)[0]
    LoadRobot(plant)
    plant.Finalize()
    diagram = builder.Build()
    diagram_context = diagram.CreateDefaultContext()
    plant_context = plant.GetMyContextFromRoot(diagram_context)
    plant.SetPositions(plant_context, seed)

    if config["use_existing_regions_as_obstacles"]:        
        iris_options.configuration_obstacles = [
            ScaleHPolyhedron(r, config["regions_as_obstacles_scale_factor"])
            for k, r in iris_regions.items()
            if k != name and r is not None
        ]
        for h in iris_options.configuration_obstacles:
            _CheckNonEmpty(h)

    try:
        display(f"Computing region for seed: {name}")
        start_time = time.time()
        hpoly = IrisInConfigurationSpace(plant, plant_context, iris_options)
        display(
            f"Finished seed {name}; Computation time: {(time.time() - start_time):.2f} seconds"
        )

        _CheckNonEmpty(hpoly)
        reduced = hpoly.ReduceInequalities()
        _CheckNonEmpty(reduced)
        return reduced

    except Exception as e:
        display(f"Skipping seed {name} due to error: {e}")
        return None


def GenerateRegion(name, seed, config, iris_regions=None, iris_options=None):
    """
    Generate a single region and save it to the iris_regions in config.
    """
    iris_regions[name] = _CalcRegion(name, seed, config, iris_regions, iris_options)
    SaveIrisRegionsYamlFile(f"{config['iris_filename']}.autosave", iris_regions)


def GenerateRegions(seed_dict, config, iris_regions, iris_options, verbose=True):
    """
    Generate regions for all seeds serially.
    """
    loop_time = time.time()
    for k, v in seed_dict.items():        
        iris_regions[k] = _CalcRegion(k, v, config, iris_regions, iris_options)                

    if verbose:
        print("Loop time:", time.time() - loop_time)


def DrawRobot(query_object: QueryObject, meshcat_prefix: str, draw_world: bool = True, meshcat=None):
    rgba = Rgba(0.7, 0.7, 0.7, 0.3)
    role = Role.kProximity
    # This is a minimal replication of the work done in MeshcatVisualizer.
    inspector = query_object.inspector()
    for frame_id in inspector.GetAllFrameIds():
        if frame_id == inspector.world_frame_id():
            if not draw_world:
                continue
            frame_path = meshcat_prefix
        else:
            frame_path = f"{meshcat_prefix}/{inspector.GetName(frame_id)}"
        frame_path.replace("::", "/")
        frame_has_any_geometry = False
        for geom_id in inspector.GetGeometries(frame_id, role):
            path = f"{frame_path}/{geom_id.get_value()}"
            path.replace("::", "/")
            meshcat.SetObject(path, inspector.GetShape(geom_id), rgba)
            meshcat.SetTransform(path, inspector.GetPoseInFrame(geom_id))
            frame_has_any_geometry = True

        if frame_has_any_geometry:
            X_WF = query_object.GetPoseInWorld(frame_id)
            meshcat.SetTransform(frame_path, X_WF)

def VisualizeRegion(region_name, meshcat=None, iris_regions=None):
    """
    A simple hit-and-run-style idea for visualizing the IRIS regions:
    1. Start at the center. Pick a random direction and run to the boundary.
    2. Pick a new random direction; project it onto the current boundary, and run along it. Repeat
    """

    num_to_draw=30
    draw_illustration_role_once=True

    builder = DiagramBuilder()
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=0.0)
    LoadRobot(plant)
    plant.Finalize()
    if draw_illustration_role_once:
        MeshcatVisualizer.AddToBuilder(builder, scene_graph, meshcat)
    diagram = builder.Build()
    context = diagram.CreateDefaultContext()
    plant_context = plant.GetMyMutableContextFromRoot(context)
    scene_graph_context = scene_graph.GetMyContextFromRoot(context)

    region = iris_regions[region_name]

    q = region.ChebyshevCenter()
    plant.SetPositions(plant_context, q)
    diagram.ForcedPublish(context)

    query = scene_graph.get_query_output_port().Eval(scene_graph_context)
    DrawRobot(query, f"{region_name}/0", True, meshcat)

    rng = np.random.default_rng()
    nq = plant.num_positions()
    prog = MathematicalProgram()
    qvar = prog.NewContinuousVariables(nq, "q")
    prog.AddLinearConstraint(region.A(), 0 * region.b() - np.inf, region.b(), qvar)
    cost = prog.AddLinearCost(np.ones((nq, 1)), qvar)

    for i in range(1, num_to_draw):
        direction = rng.standard_normal(nq)
        cost.evaluator().UpdateCoefficients(direction)

        result = Solve(prog)
        assert result.is_success()

        q = result.GetSolution(qvar)
        plant.SetPositions(plant_context, q)
        query = scene_graph.get_query_output_port().Eval(scene_graph_context)
        DrawRobot(query, f"{region_name}/{i}", False, meshcat)


def VisualizeRegions(meshcat, iris_regions):
    """
    Visualize all IRIS regions sequentially using Meshcat.

    Args:
        meshcat: Meshcat instance for visualization.
        iris_regions (dict): Dictionary of IRIS regions.
        config (dict): Configuration dictionary.
    """
    for region_name in iris_regions.keys():
        meshcat.Delete()
        VisualizeRegion(region_name, meshcat, iris_regions)
        
        button_name = f"Visualizing {region_name}; Press for next region"
        meshcat.AddButton(button_name, "Enter")
        print(f"Visualizing {region_name}. Press Enter to proceed to the next region.")
        
        while meshcat.GetButtonClicks(button_name) < 1:
            time.sleep(1.0)
        
        meshcat.DeleteButton(button_name)
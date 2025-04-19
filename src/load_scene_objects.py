import os

import numpy as np
from pydrake.all import (
    Parser,
    RigidTransform,
    RotationMatrix
)
from pydrake.multibody.plant import MultibodyPlant


def LoadRobot(plant: MultibodyPlant):
    """Setup your plant, and return the body corresponding to your
    end-effector."""

    add_custom_robot(plant)[0]
    add_table(plant)[0]    
    add_ketchup(plant)[0]
    add_static_objects(plant)
    return plant.GetBodyByName("tool_center_point")

def get_parent_directory():
    # Get the absolute path of the current working directory
    current_dir = os.getcwd()
    parent_dir = os.path.abspath(os.path.join(current_dir, os.pardir))
    return parent_dir

def add_custom_robot(plant):
    """
    Adds a custom robot to the plant from an SDF file.

    Args:
        plant: The MultibodyPlant to which the robot will be added.
        sdf_path: The path to the SDF file of the robot.

    Returns:
        The model instance of the added robot.
    """
    # Get the path to the robot SDF file
    sdf_path = os.path.join(get_parent_directory(), 'common-files','models','robot_v2','my_robot.sdf')

    if not os.path.exists(sdf_path):
        raise FileNotFoundError(f"SDF file not found at {sdf_path}")
    
    parser = Parser(plant)
    robot_model = parser.AddModels(sdf_path)
    plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("BASE"))
    return robot_model

def add_table(plant):
    """
    Adds a table to the plant from an SDF file.
    """
    # Get the path to the table SDF file
    table_path = os.path.join(get_parent_directory(), 'common-files','scene_objects','table','table_wide.sdf')

    if not os.path.exists(table_path):
        raise FileNotFoundError(f"SDF file not found at {table_path}")
    
    parser = Parser(plant)
    table_model = parser.AddModels(table_path)
    translation_vector = RigidTransform([0.0, 0.0, 0.0])
    plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("table_body"),translation_vector)
    return table_model


def add_ketchup(plant):
    """
    Adds a ketchup bottle to the plant from an SDF file.
    """
    # Get the path to the ketchup SDF file
    ketchup_path = os.path.join(get_parent_directory(), 'common-files','models','Ketchup','Ketchup.sdf')

    if not os.path.exists(ketchup_path):
        raise FileNotFoundError(f"SDF file not found at {ketchup_path}")
    
    parser = Parser(plant)
    ketchup_model = parser.AddModels(ketchup_path)
    translation_vector = RigidTransform([0.3, 0.0, 0.15])
    plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("body_ketchup"),translation_vector)
    return ketchup_model


def add_static_objects(plant):
    """
    Adds static objects to the plant from an SDF file.
    """
    
    ketchup_stand_path = os.path.join(get_parent_directory(), 'common-files','scene_objects','ketchup_stand','ketchup_stand.sdf')
    mini_shelf_l_path = os.path.join(get_parent_directory(), 'common-files','scene_objects','mini_shelf','mini_shelf_l.sdf')
    mini_shelf_r_path = os.path.join(get_parent_directory(), 'common-files','scene_objects','mini_shelf','mini_shelf_r.sdf')

    # Check if the SDF files exist
    paths = [ketchup_stand_path, mini_shelf_l_path, mini_shelf_r_path]
    for path in paths:
        if not os.path.exists(path):
            raise FileNotFoundError(f"SDF file not found at {path}")
        
    # Add the models to the plant
    parser = Parser(plant)
    parser.AddModels(ketchup_stand_path)[0]

    # Add two mini_shelves
    parser.AddModels(mini_shelf_l_path)[0]
    parser.AddModels(mini_shelf_r_path)[0]

    # Set the position of the mini_shelves
    rotation_r = RotationMatrix.MakeZRotation(np.pi/2)
    rotation_l = RotationMatrix.MakeZRotation(-np.pi/2)

    transform_l = RigidTransform(rotation_l,[0.0, 0.25, 0.15])
    transform_r = RigidTransform(rotation_r,[0.0, -0.25, 0.15])

    transform_ks = RigidTransform([0.3, 0.0, 0.075])
    plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("body_mini_shelf_l"), transform_l)
    plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("body_mini_shelf_r"), transform_r)
    plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("body_ketchup_stand"), transform_ks)
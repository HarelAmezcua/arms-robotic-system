import yaml
from pydrake.geometry.optimization import IrisOptions  # type: ignore

def load_config(file_path):
    with open(file_path, "r") as file:
        return yaml.safe_load(file)

def create_iris_options(config):
    """
    Create the IRIS options and configuration for the IRIS algorithm.
    """
    iris_options = IrisOptions()
    iris_config = config["iris_options"]

    iris_options.iteration_limit = iris_config["iteration_limit"]
    iris_options.num_collision_infeasible_samples = iris_config["num_collision_infeasible_samples"]
    iris_options.require_sample_point_is_contained = iris_config["require_sample_point_is_contained"]
    iris_options.relative_termination_threshold = iris_config["relative_termination_threshold"]
    iris_options.termination_threshold = iris_config["termination_threshold"]
    iris_options.configuration_space_margin = iris_config["configuration_space_margin"]

    return iris_options
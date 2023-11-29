# launch_pal

Utilities for simplifying some common ROS2 launch operations.


## robot_config
Contains utilities to read launch argument settings directly from a YAML file, grouped per robot type. Used to to automate argument declaration for the robots by ```launch_arg_factory```.

## arg_utils
Contains utilities for declaring launch arguments and removing boiler plate. 

`launch_arg_factory`: Combines an existing `LaunchArgumentsBase` dataclass with robot specific launch arguments. These robot specific arguments are directly loaded from a yaml file. 

An example of the use can be found in the launch_tutorial package that compares the standard [boilerplate](https://gitlab/davidterkuile/launch_tutorial/-/blob/main/launch/robot_args_example_boilerplate.launch.py) with the an [updated version](https://gitlab/davidterkuile/launch_tutorial/-/blob/main/launch/robot_args_example.launch.py) of the launch file. This updated version also provides a structured architecture that seperates launch arguments from other launch actions. 

`LaunchArgumentsBase`: A dataclass that contains only `DeclareLaunchArgument` objects. The class is used to ease the process of adding launch arguments to the launch description.

`read_launch_argument`: Used in Opaque functions to read the value of a launch argument and substitute it to text.


## param_utils
Contains utilities for merging yaml parameter files or replace parametric variables in a param file.

`parse_parametric_yaml`: Checks yaml files for variables of layout `${VAR_NAME} `and parses them. Parsing is done by giving a dictionary as input:

```
parse_dict = { VAR_NAME_1: value_1,
               VAR_NAME_2: value_2}
```

`merge_param_files`: Merges multiple yaml files into one single file to be loaded by a node.



## include_utils
Contains utilities to reduce the boilerplate necessary for including files.

`include_launch_py_description`: Include a python launch file.

`include_scoped_launch_py_description`: Include a python launch file but avoid  all launch arguments to be passed on by default. Any required launch arguments have to explicitly passed on to the launch file. 

**NOTE:**
This mimics the behavior of including launch files in ROS 1. Helpful in large launch files structures to avoid launch arguments to be overwritten by accident.


## robot_utils (DEPRECATED)
Declare a single launch argument given by the robot name. 

Example:

```
robot_name = 'tiago'
laser_model_arg = get_laser_model(robot_name)
```
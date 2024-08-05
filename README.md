# launch_pal

Utilities for simplifying some common ROS2 launch operations.

## get_pal_configuration

Implementation of the PAL's PAPS-007 standard for configuration management.

Retrieves all the parameters, remappings and arguments for a given node by
looking for `ament_index`-registered YAML configurations file. It properly handle
overloading of parameters, enabling for instance to have a default configuration
and a specific configuration for a given robot family or robot unit.

### User overrides

Users can provide local overrides via configuration files in
`$HOME/.pal/config`.

For instance, creating a file `~/.pal/config/default_volume.yml` with the
content:

```yaml
/volume:
  ros__parameters:
    default_volume: 75
```

would override the ROS parameter `default_volume` for the node `/volume`.

This is useful for eg persist user configuration across robot reboots.

The default location of user configuration is `$HOME/.pal/config`. It can by
changed by setting the environment variable `$PAL_USER_PARAMETERS_PATH`.

### Usage

```python
#...
from launch_pal import get_pal_configuration

def generate_launch_description():

    ld = LaunchDescription()

    config = get_pal_configuration(pkg='pkg_name',
                                   node='node_name', 
                                   ld=ld, # optional; only used for logging
                                   )
    my_node = Node(
        name='node_name',
        namespace='',
        package='pkg_name',
        executable='node_executable',
        parameters=config['parameters'],
        remappings=config['remappings'],
        arguments=config['arguments'],
    )

    # ...

    ld.add_action(my_node)

    return ld
```

## robot_arguments
Contains classes to read launch argument settings directly from a YAML file, grouped per robot type. For each argument the name, the description, default value and possible choices are provided. The classes can be imported to remove boilerplate of robot launch arguments. 

One special class, ```RobotArgs```, contains all available
launch arguments for PAL Robots. These arguments consist of only a name and description.

Example:
```python
from launch_pal.arg_utils import LaunchArgumentsBase
from launch_pal.robot_arguments import TiagoArgs, RobotArgs
from launch.actions import DeclareLaunchArgument
from dataclasses import dataclass

@dataclass(frozen=True)
class LaunchArguments(LaunchArgumentsBase):

    # Tiago specific settings
    wheel_model: DeclareLaunchArgument = TiagoArgs.wheel_model
    # Robot agnostic argument
    base_type: DeclareLaunchArgument = RobotArgs.base_type
```


## arg_utils
Contains utilities for declaring launch arguments and removing boiler plate. 

An example of the use can be found in the launch_tutorial package that compares the standard [boilerplate](https://gitlab/davidterkuile/launch_tutorial/-/blob/main/launch/robot_args_example_boilerplate.launch.py) with the an [updated version](https://gitlab/davidterkuile/launch_tutorial/-/blob/main/launch/robot_args_example.launch.py) of the launch file. This updated version also provides a structured architecture that seperates launch arguments from other launch actions. 

`LaunchArgumentsBase`: A dataclass that contains only `DeclareLaunchArgument` objects. The class is used to ease the process of adding launch arguments to the launch description. Has member function `add_to_launch_description` to automatically add all launch arguments to the launch description.

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

```
    scoped_launch_file = include_scoped_launch_py_description(pkg_name='my_pkg', 
    paths=['launch','my_file.launch.py'],
    launch_arguments={ 'arg_a': DeclareLaunchArgument('arg_a'),
                       'arg_2': DeclareLaunchArgument('arg_b'),
                       'arg_c': LaunchConfiguration('arg_c'),
                       'arg_d': "some_value' }
    env_vars=[SetEnvironmentVariable("VAR_NAME", 'value)]
    condition=IfCondition(LaunchConfiguration('arg_a')))
```

**NOTE:**
This mimics the behavior of including launch files in ROS 1. Helpful in large launch files structures to avoid launch arguments to be overwritten by accident.

## composition_utils
Contains utilities to reduce the boilerplate necessary for using ROS 2 components

`generate_component_list`: generates a list of composable nodes from a YAML and a package name, ready to be added or loaded into a ComposableNodeContainer: 

```yaml
components:
  <COMPONENT-NAME>:
    type: <DIAGNOSTIC-TYPE>
      ros__parameters: 
        name: <FULL-DIAGNOSTIC_DESCRIPTOR>
        [<REST-OF-PARAMS>]
```

It can be used from a launch file like:

```python
component_list = generate_component_list(components_yaml, pkg_name)
```

And then added normally to a container:

```python
container = ComposableNodeContainer(
    name="container_name",
    namespace="",
    package="rclcpp_components",
    executable="component_container",
    composable_node_descriptions=component_list,
)
```

## robot_utils (DEPRECATED)
Declare a single launch argument given by the robot name. 

Example:

```
robot_name = 'tiago'
laser_model_arg = get_laser_model(robot_name)
```

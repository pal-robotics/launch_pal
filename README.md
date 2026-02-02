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

### Variables

Configuration files can cointain variables in the form `${VAR}`. 

```yaml
/costmap:
  ros__parameters:
    robot_radius: ${robot_radius}
```

These variables are resolved by using the [robot_info_publisher](https://gitlab.pal-robotics.com/apps/robot_info_publisher).

```yaml
/robot_info_publisher:
  ros__parameters:
    robot_radius: 0.275
```

By default, variables values are stored in `/etc/robot_info/conf.d` and can be overriden by the user in `$HOME/.pal/robot_info/conf.d`.


### Templates

Templates define configuration presets of a Node. They define a set of parameters, remappings and arguments
that define a way to achieve a certain functionality of a node.
Templates are independent from the node they are used in, and can be used in multiple nodes.

**rgb_camera_low_contrast.yaml**
```yaml
ros__parameters:
  ...
  color:
    contrast": 20
    gain": 10
    gamma": 100
    saturation": 10
  ...
```

**rgb_camera_high_contrast.yaml**
```yaml
ros__parameters:
  ...
  color:
    contrast": 50
    gain": 60
    gamma": 300
    saturation": 60
  ...
```

To use a Template, the `template` key must be set in the Node's configuration file.

**head_camera.yaml**
```yaml
/head_camera_node:
  template: "rgb_camera_low_contrast.yaml"
  ...
```

**right_hand_camera.yaml**
```yaml
/right_hand_camera_node:
  template: "rgb_camera_low_contrast.yaml"
  ...
```


### Automatic command-line arguments

If `get_pal_configuration` is called with `cmdline_args=True` (default), it will
automatically add command-line launch arguments for all parameters in the
configuration file. This allows for easy configuration of the node via the
command line (see for instance `--show-args`).

Alternatively, the user can provide a list of parameters to be exposed as
command-line arguments. This is useful for instance to expose only a subset of
parameters.

This behavior can be disabled by setting `cmdline_args=False`.

### Usage

```python
#...
from launch_pal import get_pal_configuration

def generate_launch_description():

    ld = LaunchDescription()

    config = get_pal_configuration(pkg='pkg_name',
                                   node='node_name', 
                                   ld=ld, # optional if cmdline_args = False
                                   cmdline_args=[Bool|list]) # optional, True by default
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
from launch_pal.robot_arguments import CommonArgs, RobotArgs
from launch.actions import DeclareLaunchArgument
from dataclasses import dataclass

@dataclass(frozen=True)
class LaunchArguments(LaunchArgumentsBase):

    # Frequently used LaunchArguments
    wheel_model: DeclareLaunchArgument = CommonArgs.robot_name
    # PAL common Robot specific  LaunchArguments
    base_type: DeclareLaunchArgument = RobotArgs.base_type
```

## arg_utils

Contains utilities for declaring launch arguments and removing boiler plate.

`LaunchArgumentsBase`: A dataclass that contains only `DeclareLaunchArgument` objects. The class is used to ease the process of adding launch arguments to the launch description. Has member function `add_to_launch_description` to automatically add all launch arguments to the launch description.

`read_launch_argument`: Used in Opaque functions to read the value of a launch argument and substitute it to text.

## param_utils

Contains utilities for merging yaml parameter files or replace parametric variables in a param file.

`parse_parametric_yaml`: Checks yaml files for variables of layout `${VAR_NAME} `and parses them. Parsing is done by giving a dictionary as input:

```python
parse_dict = { VAR_NAME_1: value_1,
               VAR_NAME_2: value_2}
```

`merge_param_files`: Merges multiple yaml files into one single file to be loaded by a node.

## include_utils

Contains utilities to reduce the boilerplate necessary for including files.

`include_launch_py_description`: Include a python launch file.

`include_scoped_launch_py_description`: Include a python launch file but avoid  all launch arguments to be passed on by default. Any required launch arguments have to explicitly passed on to the launch file.

```python
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

## Actions

### CheckPublicSim

Raises an exception if the *is_public_sim* argument is being used correctly, that is, ensure that when using a simulation outside PAL the argument is set to true. 

## ValidateLaunchArgs

Checks that all the passed arguments using ros2 launch are declared in the launch file. This prevents passing uncorrectly typed arguments to a launch file, which would result in unexpected behaviours as the defaults would be used without warning. 

```python
validate_launch_args = ValidateLaunchArgs(launch_args=launch_args)
launch_description.add_action(validate_launch_args)
```

## ValidateXacroArgs

It does two things: 

* Checks that all the arguments that are being passed from the launch file to the xacro are declared in it, failing if not. 
* Checks that all the declared arguments in the xacro are receiving a value from the launch file, giving a warning if not. This allows to know exactly which argument is being used in the robot description. 

```python
validate_xacro_args = ValidateXacroArgs(xacro_path=xacro_file_path, xacro_input_args=xacro_input_args)
launch_description.add_action(validate_xacro_args)
```

## robot_utils (DEPRECATED)

Declare a single launch argument given by the robot name.

Example:

```python
robot_name = 'tiago'
laser_model_arg = get_laser_model(robot_name)
```

## Conditions

### IfNodeRunning

Condition that checks if a node is running. It can be used to conditionally execute actions based on the state of a node.

```python
from launch_pal.conditions import IfNodeRunning
from launch_ros.actions import Node
node_a = Node(
  package='my_package',
  executable='my_executable',
  name='node_a',
  condition=IfNodeRunning(node_name='node_b'),
)
```

In the example above, `node_a` will be launched only if `node_b` is running.

### UnlessNodeRunning
Condition that checks if a node is not running. It can be used to conditionally execute actions based on the state of a node.

```python
from launch_pal.conditions import UnlessNodeRunning
from launch_ros.actions import Node
node_a = Node(
  package='my_package',
  executable='my_executable',
  name='node_a',
  condition=UnlessNodeRunning(node_name='node_b'),
)
```
In the example above, `node_a` will be launched only if `node_b` is **NOT** running.


## Substitutions

### RobotInfoFile

Creates a temporary file with the robot information which can be used by the [`robot_info_publisher`](https://gitlab.pal-robotics.com/apps/robot_info_publisher).

```python
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_pal.substitutions import RobotInfoFile
robot_info_file = RobotInfoFile(
  content={
    'robot_name': 'tiago',
    'robot_family': 'tiago',
    'robot_unit': 'tiago_unit',
    'robot_type': DeclareLaunchArgument('robot_type'),
    'robot_radius': 0.275,
    'robot_height': LaunchConfiguration('robot_height'),
  }
)
```
This will create a temporary file with the robot information in the format expected by the `robot_info_publisher`.
The file path can then be used to set an environment variable that is accessed by the `robot_info_publisher`.

```python
from launch.actions import SetEnvironmentVariable
robot_info_env = SetEnvironmentVariable(
    name='ROBOT_INFO_PATH',
    value=robot_info_file
)
```

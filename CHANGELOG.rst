^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package launch_pal
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.14.0 (2025-06-05)
-------------------
* introduce robot_info_file
* Contributors: antoniobrandi

0.13.0 (2025-06-05)
-------------------
* introduce IfNodeRunning and UnlessNodeRunning
* Contributors: antoniobrandi

0.12.1 (2025-04-09)
-------------------
* Added argument for the side an arm is mounted
* Contributors: oscarmartinez

0.12.0 (2025-04-02)
-------------------
* read ROBOT_INFO_PATH env if available
* fix log color resets
* linters
* restore _parse_config and mark as deprecated
* add filtering to get_pal_configuration
* use consistent messages is get_pal_configuration
* rename pal configuration templates to presets
* rename pal configuration templates to presets
* Improve logging and variables substitution
* fix pal configuration logging in case of no arguments specified
* pal_parameter configuration files support paths splitting also with ;
* add variables support to pal_parameters
* use flattened parameter names in pal_parameters
* add template support; rework test_pal_parameters
* Contributors: Luka Juricic, antoniobrandi

0.11.0 (2025-02-04)
-------------------
* Add comments
* fix ament_pep8 linting
* add generic string substitution concat
* Contributors: thomasung

0.10.0 (2025-01-20)
-------------------
* Update wrong return value
* Apply suggestion after review
* Apply 1 suggestion(s) to 1 file(s)
* Add jinja2 dependency
* Add master calibration implementation
* Contributors: David ter Kuile, antoniobrandi

0.9.0 (2024-12-02)
------------------
* Merge branch 'abr/feat/docking' into 'master'
  added docking args
  See merge request common/launch_pal!73
* added docking args
* Merge branch 'upt/tun/readme' into 'master'
  update robot_arguments readme section
  See merge request common/launch_pal!71
* remove internal gitlab links
* fix linting issues
* update robot_arguments readme section
* Contributors: antoniobrandi, davidterkuile, thomasung

0.8.0 (2024-11-11)
------------------
* Suggestions apply
* Added xacro missing arg warning
* Removing unnecesary parsing
* Launch arguments check
* Xacro args checker
* Contributors: oscarmartinez

0.7.0 (2024-10-16)
------------------
* update README iwith get_pal_configuration automatic arguments
* [pal_get_params] ensure we get the default values for nested parameters
* [get_pal_parm] automatically creates cmdline arguments for node params
  This is controlled by the 'cmdline_args' param of :
  - cmdline_args=True (default): create cmd line arguments for all params
  - cmdline_args=[...]: create cmdline arguments for the listed params
  - cmdline_args=False: do not create cmdline arguments
* [get_pal_param] show config files from high to lower precedence
  This is a more natural order in practise
* Contributors: Séverin Lemaignan

0.6.0 (2024-09-26)
------------------
* Add tool_changer arg
* Contributors: thomas.peyrucain

0.5.0 (2024-09-17)
------------------
* add deprecation note to the robot_utils methods
* Contributors: Sai Kishor Kothakota

0.4.0 (2024-08-28)
------------------
* add tests for namespace
* add namespaces option when including launch file
* Contributors: thomasung

0.3.0 (2024-08-06)
------------------
* Add talos and kangaroo as robots
* Contributors: Adria Roig

0.2.0 (2024-08-05)
------------------
* [test] make sure changing AMENT_PREFIX_PATH does not spill out of the tests
* PAPS-007: better logging for invalid user configuration files
* get_pal_parameters: improved logging
  In particular, list all the configuration files found for the node, by order of precedence
* PAPS-007 - get_pal_parameters: add support for user configuration in ~/.pal/config
  The location of user configuration can be overridden via envvar
  $PAL_USER_PARAMETERS_PATH.
* Contributors: Séverin Lemaignan

0.1.15 (2024-07-04)
-------------------
* Merge branch 'omm/feat/composition_utils' into 'master'
  Composition utils
  See merge request common/launch_pal!38
* Added package field for extended usability
* Readme and new type variable name
* Removing pal_computer_monitor dep
* Composition utils to generate containers from a yaml files
* Merge branch 'paps007' into 'master'
  Add implementation of PAPS-007 'get_pal_configuration'
  See merge request common/launch_pal!57
* add impl of PAPS-007 'get_pal_configuration'
* Merge branch 'abr/feat/advanced-navigation' into 'master'
  added advanced navigation
  See merge request common/launch_pal!58
* added advanced navigation
* Contributors: Noel Jimenez, Oscar, Séverin Lemaignan, antoniobrandi, davidterkuile

0.1.14 (2024-07-03)
-------------------
* Merge branch 'air/feat/add_slam' into 'master'
  add slam param
  See merge request common/launch_pal!60
* fix declare slam
* add slam param
* Merge branch 'feature/tiago-dual-support' into 'master'
  feat: add robot_name arg to CommonArgs
  See merge request common/launch_pal!59
* feat: add robot_name to CommonArgs
* feat: tiago dual support
* Contributors: Aina, davidterkuile, josegarcia

0.1.13 (2024-06-26)
-------------------
* Merge branch 'dtk/move-robot-args' into 'master'
  Dtk/move robot args
  See merge request common/launch_pal!56
* Remove robot configurations
* ArgFactory class to create launch args from yaml
* Move common args
* Contributors: David ter Kuile, davidterkuile

0.1.12 (2024-06-11)
-------------------
* Update Changelog
* Merge branch 'tpe/upate_std_and_launch_arg' into 'master'
  update lauch args for the omni base
  See merge request common/launch_pal!55
* update lauch args for the omni base
* Contributors: David ter Kuile, davidterkuile, thomas.peyrucain

* Merge branch 'tpe/upate_std_and_launch_arg' into 'master'
  update lauch args for the omni base
  See merge request common/launch_pal!55
* update lauch args for the omni base
* Contributors: davidterkuile, thomas.peyrucain

0.1.11 (2024-05-28)
-------------------
* Merge branch 'feat/aca/find-pkg-share-yaml' into 'master'
  Feat/aca/find pkg share yaml
  See merge request common/launch_pal!54
* linters
* linters
* linters
* added import, modified explication
* extend _parse_config functionality
* Contributors: andreacapodacqua, davidterkuile

0.1.10 (2024-05-17)
-------------------
* Merge branch 'omm/feat/rgdb_sensors_rename' into 'master'
  Proper courier_rgbd_sensor name
  See merge request common/launch_pal!52
* Proper courier_rgbd_sensor name
* Contributors: davidterkuile, oscarmartinez

0.1.9 (2024-05-16)
------------------
* Merge branch 'VKG/fix/screen-parameters' into 'master'
  fixed screen parameters, edited configuration and robot argument files
  See merge request common/launch_pal!51
* typo fixed
* fixed screen parameters, edited configuration and robot argument files
* Contributors: Vamsi GUDA, davidterkuile

0.1.8 (2024-05-15)
------------------
* Merge branch 'omm/common_pos_args' into 'master'
  Robot position args added to common
  See merge request common/launch_pal!50
* Robot position args added to common
* Contributors: davidterkuile, oscarmartinez

0.1.7 (2024-05-09)
------------------
* Merge branch 'dtk/fix/bool-args' into 'master'
  Set all boolean robot args to capital value
  See merge request common/launch_pal!49
* Set all boolean robot args to capital value
* Contributors: Noel Jimenez, davidterkuile

0.1.6 (2024-05-08)
------------------
* added tuck arm parameter
* Contributors: sergiacosta

0.1.5 (2024-04-26)
------------------
* fix tests
* fix _parse_config to be able to have a variable between text
* Contributors: Aina Irisarri

0.1.4 (2024-04-12)
------------------
* Added is_public_sim action check
* Add wheel model
* Remove wrong realsense camera arg name
* Contributors: David ter Kuile, Oscar, davidterkuile

0.1.3 (2024-04-09)
------------------
* Changed arm name from sea to tiago-sea for standarization
* Contributors: Oscar

0.1.2 (2024-04-08)
------------------
* Avoid breaking tiago pro tests
* Update realsense name in camera rgument
* Contributors: David ter Kuile, davidterkuile

0.1.1 (2024-03-21)
------------------
* Fix flake test
* Add sensor manager as common arg
* Contributors: David ter Kuile, davidterkuile

0.1.0 (2024-03-20)
------------------
* Update default values
* Remove unsupported lasers for now
* Change common param to is_public_sim
* Add extra common launch args
* Add wrist model for spherical wrist
* Add tiago pro config
* Fixed base_type and arm_type
* Suggested changess
* Standarized config names
* Configs for tiago_sea
* Removed has_screen from tiago_sea
* Update config to tiago sea specific arguments
* Fixing tiago_dual_configuration
* Velodyne param added
* Tiago sea dual params
* Tiago sea params
* Create a class that contains frequently used Launch arguments to avoid mismatching Uppercase/lowercase
* Contributors: David ter Kuile, Oscar, Oscar Martinez, davidterkuile

0.0.18 (2024-01-31)
-------------------
* Remove right-arm option for tiago
* Contributors: Noel Jimenez

0.0.17 (2024-01-29)
-------------------
* tiago_pro robot_name added in the possible choices
* Contributors: ileniaperrella

0.0.16 (2024-01-18)
-------------------
* removing epick
* adding robotiq as end effector for tiago dual
* Adding pal_robotiq grippers as part of choises for the end_effector in ros2
* Contributors: Aina Irisarri

0.0.15 (2024-01-17)
-------------------
* Add right-arm as arm type for backwards compability
* Change arm type from right-arm to tiago-arm
* Remove unecessary whitelines
* Update README
* Contributors: David ter Kuile

0.0.14 (2023-12-04)
-------------------
* Update style errors
* fix typo and add type hint
* update typo
* Update configuration file keywords
* Enable autocomplete for robot arguments
* Use assertDictEqual in test
* Type hint and use get_share_directory function
* update readme
* Add tests
* Update include scoped launch for more intuitive use
* Contributors: David ter Kuile

0.0.13 (2023-11-29)
-------------------
* Remove triple quotes
* Add docstrings and update README
* Change yaml file to single quotes
* change to double quotes to be consistent in robot config yaml
* Update linting
* Update tiaog config and add tiago_dual config
* Add launch arg factory
* Update linting
* Add get_configuration function to robotConfig
* Update tiago configuration
* Add base dataclass with for launch args
* update linting
* Update types
* loop over value instead of items
* A bit of documentation
* Add scoped launch file inclusion
* Create function to translate setting to launch arg
* Create initial version of robot configuration
* Contributors: David ter Kuile

0.0.12 (2023-11-14)
-------------------
* Add website tag
* added support for omni_base
* Contributors: Noel Jimenez, andreacapodacqua

0.0.11 (2023-11-09)
-------------------
* Initial ARI support
* autopep8 line wrapping
* Contributors: Séverin Lemaignan

0.0.10 (2023-10-10)
-------------------
* Merge branch 'yen/feat/pmb3_robot' into 'master'
  Add pmb3 utils
  See merge request common/launch_pal!18
* feat: Add pmb3 utils
* Contributors: YueErro

0.0.9 (2023-07-07)
------------------
* Remove not supported choices
* Contributors: Noel Jimenez

0.0.8 (2023-06-13)
------------------
* fix cast when bool equals False
* Contributors: antoniobrandi

0.0.7 (2023-04-04)
------------------
* added parse_parametric_yaml utils
* Contributors: antoniobrandi

0.0.6 (2022-10-19)
------------------
* Merge branch 'update_copyright' into 'master'
  Update copyright
  See merge request common/launch_pal!6
* update copyright
* Merge branch 'update_maintainers' into 'master'
  Update maintainers
  See merge request common/launch_pal!5
* update maintainers
* Merge branch 'arg_robot_name' into 'master'
  Add get_robot_name argument to choose default value
  See merge request common/launch_pal!4
* add get_robot_name arg to choose default value
* Merge branch 'robot_utils' into 'master'
  Robot utils
  See merge request common/launch_pal!3
* pal-gripper as default end_effector
* launch methods for tiago
* linters
* rm unused import
* robot utils for pmb2
* Merge branch 'fix_slash_warns' into 'master'
  Fix slash warns
  See merge request common/launch_pal!2
* fix slash warns
* Contributors: Jordan Palacios, Noel Jimenez

0.0.5 (2021-08-13)
------------------
* Merge branch 'change_include_utils_to_substitutions' into 'master'
  Change Text type to substitutions for include utils
  See merge request common/launch_pal!1
* change Text type to substitutions
* Contributors: cescfolch, victor

0.0.4 (2021-07-21)
------------------
* Linter fixes
* Add load file substitution
* Contributors: Victor Lopez

0.0.3 (2021-06-30)
------------------
* Add arg_utils.py
* Contributors: Victor Lopez

0.0.2 (2021-03-15)
------------------
* Added missing dependencies
* Contributors: Jordan Palacios

0.0.1 (2021-03-15)
------------------
* Add CONTRIBUTING and LICENSE
* Apply linter fixes
* Add param_utils
* PAL utils for ROS2 launch files
* Contributors: Victor Lopez

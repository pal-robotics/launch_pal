^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package launch_pal
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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

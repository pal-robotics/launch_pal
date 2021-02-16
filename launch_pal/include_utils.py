# Copyright (c) 2021 PAL Robotics S.L.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from ament_index_python.packages import get_package_share_directory

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

import os

from typing import List
from typing import Text


def include_launch_py_description(
        pkg_name: Text,
        paths: List[Text],
        **kwargs) -> Text:
    """
    Return IncludeLaunchDescription for the file inside pkg at paths.

    example:
     include_launch_py_description('my_pkg', ['launch', 'my_file.launch.py'])
     returns file IncludeLaunchDescription from PATH_TO_MY_PKG_SHARE/launch/my_file.launch.py
    """
    pkg_dir = get_package_share_directory(pkg_name)
    full_path = os.path.join(pkg_dir, *paths)

    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            full_path),
        **kwargs)

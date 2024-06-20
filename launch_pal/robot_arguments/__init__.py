# Copyright (c) 2023 PAL Robotics S.L. All rights reserved.
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


"""robot_arguments Module."""

from .ari import AriArgs
from .tiago_dual import TiagoDualArgs
from .tiago_sea import TiagoSEAArgs
from .tiago_sea_dual import TiagoSEADualArgs
from .tiago_pro import TiagoProArgs
from .omni_base import OmniBaseArgs
from .pmb2 import PMB2Args
from .robot import RobotArgs
from .common import CommonArgs

__all__ = [
    'AriArgs',
    'TiagoDualArgs',
    'TiagoSEAArgs',
    'TiagoSEADualArgs',
    'TiagoProArgs',
    'OmniBaseArgs',
    'PMB2Args',
    'RobotArgs',
    'CommonArgs'
]

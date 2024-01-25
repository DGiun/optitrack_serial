# Copyright 2021 Institute for Robotics and Intelligent Machines,
#                Georgia Institute of Technology
# Copyright 2019 Intelligent Robotics Lab
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
#
# Author: Christian Llanes <christian.llanes@gatech.edu>
# Author: David Vargas Frutos <david.vargas@urjc.es>

import os

from launch import LaunchDescription
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory
import launch

from launch.actions import EmitEvent
from launch.actions import SetEnvironmentVariable

def generate_launch_description():
    params_file_path = os.path.join(get_package_share_directory(
          'optitrack_serial'), 'config', 'mocap_optitrack_driver_params.yaml')
    
    return LaunchDescription([
        Node(
            name='rf_receiver_node',
            namespace='',
            package='optitrack_serial',
            executable='rf_receiver',
            output='screen',
            parameters=[params_file_path],
        )
    ])

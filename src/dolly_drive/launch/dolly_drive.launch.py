# Copyright 2019 Louise Poubel
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

"""Launch Gazebo with a world that has Dolly, as well as the follow node."""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():

    ld = LaunchDescription()

    pkg_dolly_drive = get_package_share_directory('planner')

    cmd_topic_name="/simu/cmd"
    scan_topic_name="/simu/scan"
    # Follow node

    
    for i in range(5):
        crr_cmd=cmd_topic_name + '_' + str(i+1)
        crr_scan=scan_topic_name + '_' + str(i+1)
        crr_node_name='follower'
        follow = Node(
            package='planner',
            name=crr_node_name,
            node_executable='dolly_planner',                
            remappings=[
                ('cmd_vel', crr_cmd),
                ('laser_scan', crr_scan)],
            output='screen',        
            arguments=[('__log_level:=info')]
        )
        ld.add_action(follow)



    return ld

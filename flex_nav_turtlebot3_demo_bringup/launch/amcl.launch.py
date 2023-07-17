# Copyright 2022 CHRISLab, Christopher Newport University
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
# Author: David Conner

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from nav2_common.launch import RewrittenYaml


def generate_launch_description():

    turtlebot_desc_dir = get_package_share_directory('flex_nav_turtlebot3_demo_bringup')
    map_dir = get_package_share_directory('turtlebot3_navigation2')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')  # For simulations
    autostart = LaunchConfiguration('autostart', default='true')
    namespace = LaunchConfiguration('namespace')

    amcl_name = "amcl"
    ms_name = 'map_server'  # Presumes map server launched earlier

    lifecycle_nodes = [amcl_name, ms_name]

    param_substitutions = {
        'use_sim_time': use_sim_time,
        'yaml_filename': os.path.join(map_dir, 'map', 'map.yaml'), }

    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]

    params_file = os.path.join(turtlebot_desc_dir, 'param', 'amcl.yaml')
    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key=namespace,
        param_rewrites=param_substitutions,
        convert_types=True)

    return LaunchDescription([
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),

        DeclareLaunchArgument(
            'namespace', default_value='',
            description='Top-level namespace'),

        Node(
            package='nav2_amcl',
            executable='amcl',
            name=amcl_name,
            output='screen',
            parameters=[configured_params],
            remappings=remappings),

        Node(
            package='nav2_map_server',
            executable='map_server',
            name=ms_name,
            output='screen',
            parameters=[param_substitutions]),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager',
            output='screen',
            emulate_tty=True,  # https://github.com/ros2/launch/issues/188
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': autostart},
                        {'node_names': lifecycle_nodes}])

    ])

# Copyright 2022, CHRISLab, Christopher Newport University
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
import xacro
import yaml

from ament_index_python.packages import get_package_share_directory
import launch
import launch_ros.actions
from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch.actions import SetLaunchConfiguration
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from nav2_common.launch import RewrittenYaml

def generate_launch_description():

    bringup_dir = get_package_share_directory('flex_nav_turtlebot3_demo_bringup')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true') # For simulations
    autostart = LaunchConfiguration('autostart', default='true')

    hlp_name = "high_level_planner"
    gcm_name = 'global_costmap'
    mlp_name = 'mid_level_planner'
    mcm_name = 'middle_costmap'
    lmlp_name = 'low_mid_level_planner'
    lmcm_name = 'low_middle_costmap'
    llp_name = 'low_level_planner'
    lcm_name = 'local_costmap'
    bhs_name = 'behavior_server'

    lifecycle_nodes  = [ hlp_name, mlp_name, lmlp_name, llp_name, bhs_name]

    param_substitutions = {
        'use_sim_time': use_sim_time,
        'autostart': autostart}

    # Load the high_level planner and costmap params
    high_level_params = RewrittenYaml(
            source_file=os.path.join(bringup_dir, 'param', 'high_level_planner_params.yaml'),
            root_key="",
            param_rewrites=param_substitutions,
            convert_types=True)

    global_costmap_params = RewrittenYaml(
            source_file=os.path.join(bringup_dir, 'param', 'global_costmap_params.yaml'),
            root_key="",
            param_rewrites=param_substitutions,
            convert_types=True)

    mid_level_params = RewrittenYaml(
            source_file=os.path.join(bringup_dir, 'param', 'mid_level_planner_params.yaml'),
            root_key="",
            param_rewrites=param_substitutions,
            convert_types=True)

    middle_costmap_params = RewrittenYaml(
            source_file=os.path.join(bringup_dir, 'param', 'middle_costmap_params.yaml'),
            root_key="",
            param_rewrites=param_substitutions,
            convert_types=True)

    low_mid_level_params = RewrittenYaml(
            source_file=os.path.join(bringup_dir, 'param', 'low_mid_level_planner_params.yaml'),
            root_key="",
            param_rewrites=param_substitutions,
            convert_types=True)

    low_middle_costmap_params = RewrittenYaml(
            source_file=os.path.join(bringup_dir, 'param', 'low_middle_costmap_params.yaml'),
            root_key="",
            param_rewrites=param_substitutions,
            convert_types=True)


    # Low level controller params
    low_level_params = RewrittenYaml(
            source_file=os.path.join(bringup_dir, 'param', 'low_level_planner_params.yaml'),
            root_key="",
            param_rewrites=param_substitutions,
            convert_types=True)

    local_costmap_params = RewrittenYaml(
            source_file=os.path.join(bringup_dir, 'param', 'local_costmap_params.yaml'),
            root_key="",
            param_rewrites=param_substitutions,
            convert_types=True)

    behavior_params = RewrittenYaml(
            source_file=os.path.join(bringup_dir, 'param', 'behavior_server.yaml'),
            root_key="",
            param_rewrites=param_substitutions,
            convert_types=True)

    # Set up the nodes for launch
    high_level_planner_node = Node(package='flex_nav_planners',
                                   executable='flex_nav_planners_get_path_node',
                                   name=hlp_name,
                                   output='screen',
                                   parameters=[high_level_params, global_costmap_params],
                                  )

    mid_level_planner_node = Node(package='flex_nav_planners',
                                   executable='flex_nav_planners_follow_path_node',
                                   name=mlp_name,
                                   output='screen',
                                   parameters=[mid_level_params, middle_costmap_params],
                                  )

    low_mid_level_planner_node = Node(package='flex_nav_planners',
                                   executable='flex_nav_planners_follow_topic_node',
                                   name=lmlp_name,
                                   output='screen',
                                   parameters=[low_mid_level_params, low_middle_costmap_params],
                                  )

    low_level_planner_node = Node(package='flex_nav_controllers',
                                   executable='flex_nav_controllers_follow_topic_node',
                                   name=llp_name,
                                   output='screen',
                                   parameters=[low_level_params, local_costmap_params],
                                  )

    behavior_server_node = Node(package='flex_nav_behaviors',
                                executable='flex_nav_behaviors_behavior_server_node',
                                name='behavior_server',
                                output='screen',
                                parameters=[behavior_params],
                               )

    lifecycle_manager = Node(package='nav2_lifecycle_manager',
                              executable='lifecycle_manager',
                              name='flex_lifecycle_manager',
                              output='screen',
                              #emulate_tty=True,  # https://github.com/ros2/launch/issues/188
                              parameters=[{'use_sim_time': use_sim_time,
                                          'autostart': autostart,
                                          'bond_timeout': 5.0,
                                          'node_names': lifecycle_nodes}])

    ld = LaunchDescription()
    ld.add_action(high_level_planner_node)
    ld.add_action(mid_level_planner_node)
    ld.add_action(low_mid_level_planner_node)
    ld.add_action(low_level_planner_node)
    ld.add_action(behavior_server_node)
    ld.add_action(lifecycle_manager)

    return ld

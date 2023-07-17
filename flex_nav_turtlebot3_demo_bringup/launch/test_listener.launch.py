# Copyright 2023 CHRISLab, Christopher Newport University
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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')  # For simulations
    timer_period_s = LaunchConfiguration('timer_period_s', default=0.0333)  # 30 hz

    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]

    return LaunchDescription([
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),

        DeclareLaunchArgument(
            'namespace', default_value='',
            description='Top-level namespace'),

        # This requires ground truth topic from simulation!
        Node(
            package='flex_nav_turtlebot3_demo_bringup',
            executable='test_listener',
            name='listener_tester',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time},
                        {'timer_period_s': timer_period_s}],
            remappings=remappings),


    ])

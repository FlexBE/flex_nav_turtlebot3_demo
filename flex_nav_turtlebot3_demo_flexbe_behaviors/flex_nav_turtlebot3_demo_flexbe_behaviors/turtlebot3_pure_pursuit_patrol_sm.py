#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Copyright 2022 David Conner
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

###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

"""
Define Turtlebot3 Pure Pursuit Patrol.

Created on Mon 1-Aug-2022
@author: David Conner
"""


from flexbe_core import Autonomy
from flexbe_core import Behavior
from flexbe_core import ConcurrencyContainer
from flexbe_core import Logger
from flexbe_core import OperatableStateMachine
from flexbe_core import PriorityContainer
from flex_nav_flexbe_states.follow_path_state import FollowPathState
from flex_nav_flexbe_states.get_path_by_name_state import GetPathByNameState
from flex_nav_flexbe_states.pure_pursuit_state import PurePursuitState
from flex_nav_flexbe_states.set_indice_state import SetIndiceState
from flex_nav_flexbe_states.timed_stop_state import TimedStopState
from flexbe_states.log_state import LogState
from flexbe_states.operator_decision_state import OperatorDecisionState

# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


class Turtlebot3PurePursuitPatrolSM(Behavior):
    """
    Define Turtlebot3 Pure Pursuit Patrol.

    Uses Flexible Navigation to control the Turtlebot 3 robot using basic pure pursuit and pre-defined paths.
    Demonstrates using a PurePursuitState where FlexBE published command and FollowPath state using a PurePursuitPath node.

    """

    def __init__(self, node):
        super().__init__()
        self.name = 'Turtlebot3 Pure Pursuit Patrol'

        # parameters of this behavior

        # references to used behaviors
        OperatableStateMachine.initialize_ros(node)
        ConcurrencyContainer.initialize_ros(node)
        PriorityContainer.initialize_ros(node)
        Logger.initialize(node)
        FollowPathState.initialize_ros(node)
        GetPathByNameState.initialize_ros(node)
        LogState.initialize_ros(node)
        OperatorDecisionState.initialize_ros(node)
        PurePursuitState.initialize_ros(node)
        SetIndiceState.initialize_ros(node)
        TimedStopState.initialize_ros(node)

        # Additional initialization code can be added inside the following tags
        # [MANUAL_INIT]

        # [/MANUAL_INIT]

        # Behavior comments:

    def create(self):
        # x:97 y:468, x:1167 y:605
        _state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

        # Additional creation code can be added inside the following tags
        # [MANUAL_CREATE]

        # [/MANUAL_CREATE]
        with _state_machine:
            # x:151 y:48
            OperatableStateMachine.add('Continue',
                                       OperatorDecisionState(outcomes=["patrol", "loop", "quit"], hint="Continue patrolling ?",
                                                             suggestion="patrol"),
                                       transitions={'patrol': 'GetPath', 'loop': 'GetLoop', 'quit': 'finished'},
                                       autonomy={'patrol': Autonomy.High, 'loop': Autonomy.Full, 'quit': Autonomy.Full})

            # x:1063 y:466
            OperatableStateMachine.add('EStop',
                                       TimedStopState(timeout=0.25, cmd_topic='cmd_vel', odom_topic='odom', cmd_topic_stamped=""),
                                       transitions={'done': 'Log Success', 'failed': 'failed'},
                                       autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})

            # x:506 y:578
            OperatableStateMachine.add('FollowPath',
                                       FollowPathState(topic="pure_pursuit_node"),
                                       transitions={'done': 'Stop', 'failed': 'LogFailed', 'canceled': 'LogFailed'},
                                       autonomy={'done': Autonomy.High, 'failed': Autonomy.Off, 'canceled': Autonomy.Off},
                                       remapping={'plan': 'plan'})

            # x:286 y:473
            OperatableStateMachine.add('GetLoop',
                                       GetPathByNameState(action_server_name='get_path_by_name', path_name='sim_loop'),
                                       transitions={'success': 'FollowPath', 'empty': 'LogFail', 'failed': 'LogFail'},
                                       autonomy={'success': Autonomy.Off, 'empty': Autonomy.Off, 'failed': Autonomy.Off},
                                       remapping={'plan': 'plan'})

            # x:529 y:56
            OperatableStateMachine.add('GetPath',
                                       GetPathByNameState(action_server_name='get_path_by_name', path_name='sim_patrol'),
                                       transitions={'success': 'StartNdx', 'empty': 'LogFail', 'failed': 'LogFail'},
                                       autonomy={'success': Autonomy.High, 'empty': Autonomy.Off, 'failed': Autonomy.Off},
                                       remapping={'plan': 'plan'})

            # x:617 y:510
            OperatableStateMachine.add('Log Success',
                                       LogState(text="Success!", severity=Logger.REPORT_HINT),
                                       transitions={'done': 'Continue'},
                                       autonomy={'done': Autonomy.Off})

            # x:549 y:166
            OperatableStateMachine.add('LogFail',
                                       LogState(text="No path found!", severity=Logger.REPORT_HINT),
                                       transitions={'done': 'Continue'},
                                       autonomy={'done': Autonomy.Off})

            # x:552 y:246
            OperatableStateMachine.add('LogFailed',
                                       LogState(text="Failed to follow path!", severity=Logger.REPORT_HINT),
                                       transitions={'done': 'Continue'},
                                       autonomy={'done': Autonomy.Off})

            # x:1069 y:305
            OperatableStateMachine.add('LogLoop',
                                       LogState(text="Completed segment of path", severity=Logger.REPORT_HINT),
                                       transitions={'done': 'PurePursuitState'},
                                       autonomy={'done': Autonomy.Off})

            # x:999 y:186
            OperatableStateMachine.add('PurePursuitState',
                                       PurePursuitState(desired_velocity=0.2, max_rotation_rate=10.0, lookahead_distance=0.25,
                                                        timeout=0.2, cmd_topic='cmd_vel', marker_topic='pure_pursuit_markers',
                                                        marker_size=0.05, cmd_topic_stamped='cmd_vel_stamped'),
                                       transitions={'done': 'Stop', 'continue': 'LogLoop', 'failed': 'SegmentFail'},
                                       autonomy={'done': Autonomy.Off, 'continue': Autonomy.Off, 'failed': Autonomy.Off},
                                       remapping={'indice': 'indice', 'plan': 'plan'})

            # x:684 y:277
            OperatableStateMachine.add('Retry',
                                       OperatorDecisionState(outcomes=['retry', 'fail'],
                                                             hint="Retry to attempt same segment again",
                                                             suggestion='retry'),
                                       transitions={'retry': 'PurePursuitState', 'fail': 'LogFailed'},
                                       autonomy={'retry': Autonomy.High, 'fail': Autonomy.Off})

            # x:864 y:308
            OperatableStateMachine.add('SegmentFail',
                                       LogState(text="Segment failed!", severity=Logger.REPORT_HINT),
                                       transitions={'done': 'Retry'},
                                       autonomy={'done': Autonomy.Off})

            # x:729 y:52
            OperatableStateMachine.add('StartNdx',
                                       SetIndiceState(),
                                       transitions={'done': 'PurePursuitState'},
                                       autonomy={'done': Autonomy.Off},
                                       remapping={'plan': 'plan', 'indice': 'indice'})

            # x:799 y:578
            OperatableStateMachine.add('Stop',
                                       TimedStopState(timeout=0.25, cmd_topic='cmd_vel', odom_topic='odom',
                                                      cmd_topic_stamped=""),
                                       transitions={'done': 'Log Success', 'failed': 'EStop'},
                                       autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})

        return _state_machine

    # Private functions can be added inside the following tags
    # [MANUAL_FUNC]

    # [/MANUAL_FUNC]

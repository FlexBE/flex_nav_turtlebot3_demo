#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Copyright 2023 Christopher Newport University
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the Christopher Newport University nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.


###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from flex_nav_flexbe_states.clear_costmaps_state import ClearCostmapsState
from flex_nav_flexbe_states.follow_path_state import FollowPathState
from flex_nav_flexbe_states.get_path_state import GetPathState
from flex_nav_flexbe_states.get_pose_state import GetPoseState
from flexbe_states.log_state import LogState
from flexbe_states.operator_decision_state import OperatorDecisionState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


"""
Created on Sat Jan 15 2022
@author: Josh Zutell
"""


class Turtlebot3FlexPlannerSM(Behavior):
    """
    Use Flexible Navigation to control the Turtlebot 3 robot.

    high-level: Map only
    low-level: sensors only controller to follow high-level path
    """

    def __init__(self, node):
        super(Turtlebot3FlexPlannerSM, self).__init__()
        self.name = 'Turtlebot3 Flex Planner'

        # parameters of this behavior

        # references to used behaviors
        OperatableStateMachine.initialize_ros(node)
        ConcurrencyContainer.initialize_ros(node)
        PriorityContainer.initialize_ros(node)
        Logger.initialize(node)
        ClearCostmapsState.initialize_ros(node)
        FollowPathState.initialize_ros(node)
        GetPathState.initialize_ros(node)
        GetPoseState.initialize_ros(node)
        LogState.initialize_ros(node)
        OperatorDecisionState.initialize_ros(node)

        # Additional initialization code can be added inside the following tags
        # [MANUAL_INIT]

        # [/MANUAL_INIT]

        # Behavior comments:

    def create(self):
        # x:943 y:286, x:1073 y:20
        _state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

        # Additional creation code can be added inside the following tags
        # [MANUAL_CREATE]

        # [/MANUAL_CREATE]

        with _state_machine:
            # x:193 y:26
            OperatableStateMachine.add('ClearCostmap',
                                       ClearCostmapsState(costmap_topics=['high_level_planner/clear_costmap',
                                                                          'low_level_planner/clear_costmap'],
                                                          timeout=5.0),
                                       transitions={'done': 'Receive Goal', 'failed': 'failed'},
                                       autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})

            # x:435 y:146
            OperatableStateMachine.add('Continue',
                                       OperatorDecisionState(outcomes=["yes", "no", "clearcostmap"],
                                                             hint="Continue planning to new goal?", suggestion="yes"),
                                       transitions={'yes': 'Receive Goal', 'no': 'finished', 'clearcostmap': 'ClearCostmap'},
                                       autonomy={'yes': Autonomy.High, 'no': Autonomy.Full, 'clearcostmap': Autonomy.Full})

            # x:435 y:299
            OperatableStateMachine.add('Execute Path',
                                       FollowPathState(topic="low_level_planner"),
                                       transitions={'done': 'Log Success', 'failed': 'AutoReplan', 'canceled': 'Continue'},
                                       autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off, 'canceled': Autonomy.Off},
                                       remapping={'plan': 'plan'})

            # x:194 y:301
            OperatableStateMachine.add('ExecutePlan',
                                       OperatorDecisionState(outcomes=["yes", "no"],
                                                             hint="Execute the current plan?", suggestion="yes"),
                                       transitions={'yes': 'Execute Path', 'no': 'Continue'},
                                       autonomy={'yes': Autonomy.High, 'no': Autonomy.Full})

            # x:960 y:70
            OperatableStateMachine.add('Log Recovered',
                                       LogState(text="Re-plan after recovery", severity=Logger.REPORT_HINT),
                                       transitions={'done': 'New Plan'},
                                       autonomy={'done': Autonomy.Off})

            # x:708 y:293
            OperatableStateMachine.add('Log Success',
                                       LogState(text="Success!", severity=Logger.REPORT_HINT),
                                       transitions={'done': 'Continue'},
                                       autonomy={'done': Autonomy.Off})

            # x:728 y:65
            OperatableStateMachine.add('New Plan',
                                       GetPathState(planner_topic="high_level_planner"),
                                       transitions={'planned': 'Execute Path', 'empty': 'Receive Goal', 'failed': 'Continue'},
                                       autonomy={'planned': Autonomy.Off, 'empty': Autonomy.Off, 'failed': Autonomy.Off},
                                       remapping={'goal': 'goal', 'plan': 'plan'})

            # x:203 y:113
            OperatableStateMachine.add('Receive Goal',
                                       GetPoseState(topic='flex_nav_global/goal'),
                                       transitions={'done': 'Receive Path'},
                                       autonomy={'done': Autonomy.Low},
                                       remapping={'goal': 'goal'})

            # x:205 y:207
            OperatableStateMachine.add('Receive Path',
                                       GetPathState(planner_topic="high_level_planner"),
                                       transitions={'planned': 'ExecutePlan', 'empty': 'Continue', 'failed': 'Continue'},
                                       autonomy={'planned': Autonomy.Off, 'empty': Autonomy.Low, 'failed': Autonomy.Low},
                                       remapping={'goal': 'goal', 'plan': 'plan'})

            # x:875 y:162
            OperatableStateMachine.add('AutoReplan',
                                       OperatorDecisionState(outcomes=["yes", "no"],
                                                             hint="Re-plan to current goal?",
                                                             suggestion="yes"),
                                       transitions={'yes': 'Log Recovered', 'no': 'Continue'},
                                       autonomy={'yes': Autonomy.High, 'no': Autonomy.Full})

        return _state_machine

    # Private functions can be added inside the following tags
    # [MANUAL_FUNC]

    # [/MANUAL_FUNC]

#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Copyright 2021 Josh Zutell
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
Define SimpleFlexNavTB3LoopDemo.

Created on Mon Oct 25 2021
@author: Josh Zutell
"""


from flexbe_core import Autonomy
from flexbe_core import Behavior
from flexbe_core import ConcurrencyContainer
from flexbe_core import Logger
from flexbe_core import OperatableStateMachine
from flexbe_core import PriorityContainer
from flex_nav_flexbe_states.timed_stop_state import TimedStopState
from flex_nav_flexbe_states.timed_twist_state import TimedTwistState

# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


class SimpleFlexNavTB3LoopDemoSM(Behavior):
    """
    Define SimpleFlexNavTB3LoopDemo.

    A simple demo using Flexible Navigation states to navigation a TurtleBot3 in a loop

    """

    def __init__(self, node):
        super().__init__()
        self.name = 'SimpleFlexNavTB3LoopDemo'

        # parameters of this behavior

        # references to used behaviors
        OperatableStateMachine.initialize_ros(node)
        ConcurrencyContainer.initialize_ros(node)
        PriorityContainer.initialize_ros(node)
        Logger.initialize(node)
        TimedStopState.initialize_ros(node)
        TimedTwistState.initialize_ros(node)

        # Additional initialization code can be added inside the following tags
        # [MANUAL_INIT]

        # [/MANUAL_INIT]

        # Behavior comments:

    def create(self):
        # x:1254 y:208, x:398 y:127
        _state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

        # Additional creation code can be added inside the following tags
        # [MANUAL_CREATE]

        # [/MANUAL_CREATE]
        with _state_machine:
            # x:59 y:28
            OperatableStateMachine.add('Forward1',
                                       TimedTwistState(target_time=2.5, velocity=0.20, rotation_rate=0.0, cmd_topic='cmd_vel',
                                                       cmd_topic_stamped=''),
                                       transitions={'done': 'Stop'},
                                       autonomy={'done': Autonomy.Off})

            # x:653 y:19
            OperatableStateMachine.add('Forward2',
                                       TimedTwistState(target_time=5, velocity=0.2, rotation_rate=0.0, cmd_topic='cmd_vel',
                                                       cmd_topic_stamped=''),
                                       transitions={'done': 'Turn2'},
                                       autonomy={'done': Autonomy.Off})

            # x:1020 y:17
            OperatableStateMachine.add('Forward3',
                                       TimedTwistState(target_time=2.5, velocity=0.2, rotation_rate=0.0, cmd_topic='cmd_vel',
                                                       cmd_topic_stamped=''),
                                       transitions={'done': 'Turn3'},
                                       autonomy={'done': Autonomy.Off})

            # x:321 y:203
            OperatableStateMachine.add('Forward4',
                                       TimedTwistState(target_time=2.5, velocity=0.2, rotation_rate=0.0, cmd_topic='cmd_vel',
                                                       cmd_topic_stamped=''),
                                       transitions={'done': 'Turn4'},
                                       autonomy={'done': Autonomy.Off})

            # x:725 y:201
            OperatableStateMachine.add('Forward5',
                                       TimedTwistState(target_time=5, velocity=0.2, rotation_rate=0.0, cmd_topic='cmd_vel',
                                                       cmd_topic_stamped=''),
                                       transitions={'done': 'Turn5'},
                                       autonomy={'done': Autonomy.Off})

            # x:1073 y:198
            OperatableStateMachine.add('Forward6',
                                       TimedTwistState(target_time=2.5, velocity=0.2, rotation_rate=0.0, cmd_topic='cmd_vel',
                                                       cmd_topic_stamped=''),
                                       transitions={'done': 'finished'},
                                       autonomy={'done': Autonomy.Off})

            # x:266 y:27
            OperatableStateMachine.add('Stop',
                                       TimedStopState(timeout=0.25, cmd_topic='cmd_vel', odom_topic='odom', cmd_topic_stamped=''),
                                       transitions={'done': 'Turn1', 'failed': 'failed'},
                                       autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})

            # x:460 y:22
            OperatableStateMachine.add('Turn1',
                                       TimedTwistState(target_time=3.73064127614, velocity=0.2, rotation_rate=-0.4,
                                                       cmd_topic='cmd_vel', cmd_topic_stamped=''),
                                       transitions={'done': 'Forward2'},
                                       autonomy={'done': Autonomy.Off})

            # x:831 y:14
            OperatableStateMachine.add('Turn2',
                                       TimedTwistState(target_time=3.73064127614, velocity=0.2, rotation_rate=-0.4,
                                                       cmd_topic='cmd_vel', cmd_topic_stamped=''),
                                       transitions={'done': 'Forward3'},
                                       autonomy={'done': Autonomy.Off})

            # x:49 y:186
            OperatableStateMachine.add('Turn3',
                                       TimedTwistState(target_time=15.7079632679, velocity=0.2, rotation_rate=0.4,
                                                       cmd_topic='cmd_vel', cmd_topic_stamped=''),
                                       transitions={'done': 'Forward4'},
                                       autonomy={'done': Autonomy.Off})

            # x:540 y:204
            OperatableStateMachine.add('Turn4',
                                       TimedTwistState(target_time=3.73064127614, velocity=0.2, rotation_rate=-0.4,
                                                       cmd_topic='cmd_vel', cmd_topic_stamped=''),
                                       transitions={'done': 'Forward5'},
                                       autonomy={'done': Autonomy.Off})

            # x:908 y:201
            OperatableStateMachine.add('Turn5',
                                       TimedTwistState(target_time=3.73064127614, velocity=0.2, rotation_rate=-0.4,
                                                       cmd_topic='cmd_vel', cmd_topic_stamped=''),
                                       transitions={'done': 'Forward6'},
                                       autonomy={'done': Autonomy.Off})

        return _state_machine

    # Private functions can be added inside the following tags
    # [MANUAL_FUNC]

    # [/MANUAL_FUNC]

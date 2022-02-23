#!/usr/bin/env python
# -*- coding: utf-8 -*-
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


'''
Created on Sat Jan 15 2022
@author: Josh Zutell
'''
class TurtlebotFlexPlannerSM(Behavior):
	'''
	Uses Flexible Navigation to control the Turtlebot robot
	'''


	def __init__(self, node):
		super(TurtlebotFlexPlannerSM, self).__init__()
		self.name = 'Turtlebot Flex Planner'

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
										ClearCostmapsState(costmap_topics=['high_level_planner/clear_costmap','low_level_planner/clear_costmap'], timeout=5.0),
										transitions={'done': 'Receive Goal', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})

			# x:435 y:146
			OperatableStateMachine.add('Continue',
										OperatorDecisionState(outcomes=["yes","no","clearcostmap"], hint="Continue planning to new goal?", suggestion="yes"),
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
										OperatorDecisionState(outcomes=["yes","no"], hint="Execute the current plan?", suggestion="yes"),
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
										OperatorDecisionState(outcomes=["yes","no"], hint="Re-plan to current goal?", suggestion="yes"),
										transitions={'yes': 'Log Recovered', 'no': 'Continue'},
										autonomy={'yes': Autonomy.High, 'no': Autonomy.Full})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]

	# [/MANUAL_FUNC]

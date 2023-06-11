#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from flex_nav_flexbe_states.follow_path_state import FollowPathState
from flex_nav_flexbe_states.get_path_by_name_state import GetPathByNameState
from flex_nav_flexbe_states.pure_pursuit_state import PurePursuitState
from flex_nav_flexbe_states.set_indice_state import SetIndiceState
from flexbe_states.log_state import LogState
from flexbe_states.operator_decision_state import OperatorDecisionState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Mon 1-Aug-2022
@author: David Conner
'''
class Turtlebot3PurePursuitPatrolSM(Behavior):
	'''
	Uses Flexible Navigation to control the Turtlebot 3 robot using basic pure pursuit and pre-defined paths.  
	Demonstrates using a PurePursuitState where FlexBE published command and FollowPath state using a PurePursuitPath node.
	'''


	def __init__(self, node):
		super(Turtlebot3PurePursuitPatrolSM, self).__init__()
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

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]

		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:97 y:468, x:995 y:482
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]

		# [/MANUAL_CREATE]


		with _state_machine:
			# x:151 y:48
			OperatableStateMachine.add('Continue',
										OperatorDecisionState(outcomes=["patrol","loop", "quit"], hint="Continue patrolling ?", suggestion="patrol"),
										transitions={'patrol': 'GetPath', 'loop': 'GetLoop', 'quit': 'finished'},
										autonomy={'patrol': Autonomy.High, 'loop': Autonomy.Full, 'quit': Autonomy.Full})

			# x:757 y:472
			OperatableStateMachine.add('FollowPath',
										FollowPathState(topic="pure_pursuit_node"),
										transitions={'done': 'Log Success', 'failed': 'LogFailed', 'canceled': 'LogFailed'},
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

			# x:551 y:336
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

			# x:752 y:157
			OperatableStateMachine.add('LogLoop',
										LogState(text="Completed patrol loop", severity=Logger.REPORT_HINT),
										transitions={'done': 'StartNdx'},
										autonomy={'done': Autonomy.Off})

			# x:999 y:186
			OperatableStateMachine.add('PurePursuitState',
										PurePursuitState(desired_velocity=0.2, max_rotation_rate=10.0, target_frame='map', target_x=1.0, target_y=0.1, target_type='line', lookahead_distance=0.25, timeout=0.08, recover_mode=True, center_x=0.0, center_y=0.0, cmd_topic='cmd_vel', odometry_topic='odom', marker_topic='pure_pursuit_marker', marker_size=0.05, cmd_topic_stamped='cmd_vel_stamped'),
										transitions={'done': 'Log Success', 'continue': 'LogLoop', 'failed': 'LogFailed'},
										autonomy={'done': Autonomy.Off, 'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'indice': 'indice', 'plan': 'plan'})

			# x:729 y:52
			OperatableStateMachine.add('StartNdx',
										SetIndiceState(),
										transitions={'done': 'PurePursuitState'},
										autonomy={'done': Autonomy.Off},
										remapping={'plan': 'plan', 'indice': 'indice'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]

	# [/MANUAL_FUNC]

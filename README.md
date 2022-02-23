Flex Nav Turtlebot3 Demo
================================

## Introduction

Custom behaviors and launch files for demonstrations of the [ROS 2] and [FlexBE]-based [Flexible Navigation] ([Wiki]) for use with the ROBOTIS-based Turtlebot.

This repository contains code that interfaces with the ROS 2 versions of the [ROBOTIS Turtlebot3] models, the [FlexBE Behavior Engine], [FlexBE App], and the [Flexible Navigation] system.

Installation and Setup
----------------------

This package has a number of dependencies.  

The quickest and easiest method to get a demonstration up and running is to follow the instructions below.

1) Ensure that you are running a recent ROS 2 version

  * This system is tested on `ros-foxy-desktop-full`.  
  * See [ROS 2 Installation] for more information.

2) Install the [ROBOTIS Turtlebot3] packages (standard ROS install)

3) Install the [FlexBE App] and [FlexBE Behavior Engine] into a ROS 2 workspace
  * If the official ROS 2 release versions are not yet available, then clone both repositories
  into a ROS 2 workspace

4) Clone the [Flexible Navigation] system into a ROS 2 workspace

5) Clone this repository into a ROS 2 workspace

 This version presumes use of the [FlexBE App] for the operator interface, which depends on states and behaviors that are exported as part of individual package.xml.

6) Build all the cloned packages with `colcon` in the ROS 2 workspace

## Operation
---------

The following directions are for a simple demonstration of Flexible Navigation using [ROS 2 Cartographer] as the map.

### Start the simulated robot
`export TURTLEBOT3_MODEL=burger`
 * This defines which version of the Turtlebot3 will be simulated

`ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py`
 * Starts the simulated environment of the ROBOTIS Turtlebot3 world with the simulated Turtlebot3

### Start map server
 With an unknown map, ROS 2 Cartographer will build a map of the simulated environment
 `export TURTLEBOT3_MODEL=burger`
  * Need to export the Turtlebot3 model variable again in the separate terminal tab

 `ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True`
  * Starts ROS 2 Cartographer to build a map
  * Sets use_sim_time to true instead of the robot running in real time

### Visualization
 Is started alongside ROS 2 Cartographer with the previous command
 Displays a standard view of transforms of Turtlebot3, sensor data, with maps, and paths displayed

 Change the nav goal topic to `flex_nav_global/goal`

### Startup of Flexible Navigation
Flexible Navigation requires startup of planning and control nodes, as well as the FlexBE behavior engine and UI.
 `ros2 launch flexbe_app flexbe_full.launch.py`
 * This starts the FlexBE behavior engine and FlexBE App UI

Then start one (and only one) of the following:
 `ros2 launch flex_nav_turtlebot3_demo_bringup flex.launch`
 * This starts the planning and control nodes.
 * This version uses a 2-level planner as a demonstration.
  * The global planner plans over the full map, with sensor data
  * The local planner plans over smaller window trying to follow the global path

or

`ros2 launch flex_nav_turtlebot3_demo_bringup flex_multi_level.launch`
 * This starts the planning and control nodes.
 * This version uses a 3-level planner as a demonstration.
  * The high-level planner is based only on the static map
  * The mid-level planner using only local obstacle sensing
  * The low-level planner using the [ROS 2 Navigation2] DWBLocalPlanner

 *  The mid- and low-level planners run concurrently as they try to follow the global path defined by the high-level planner.

### FlexBE Operation
After startup, all control is through the FlexBE App operator interface and RViz.  
* First load the desired behavior through the `FlexBE Behavior Dashboard` tab.
  * The behavior should match the flex launch started above.
    * 'flex.launch' --> `Turtlebot Flex Planner`
    * 'flex_multi_level.launch' --> `Turtlebot Multi Level Flex Planner`
* Execute the behavior via the `FlexBE Runtime Control` tab.
* The system requires the operator to input a `2D Nav Goal` via the `RViz` screen
  * If the system is in `low` autonomy or higher, the system will request a global plan as soon as the goal is received
  * If the autonomy level is `off`, then the operator will need to confirm receipt by clicking the `done` transition.
* After requesting a path to the goal, the resulting plan will be visualized in the `RViz` window.  
  * If the system is not in full autonomy mode, the operator must confirm that the system should execute the plan via the `FlexBE UI`  
  * If the operator sets the `Runtime Executive` to `full` autonomy, the plan will automatically be executed.  
  * In less than `full` autonomy, the operator can request a recovery behavior at this point.
* Once execution of this plan is complete, `FlexBE` will seek permission to continue planning
  * In `full` autonomy, the system will automatically transition to requesting a new goal
  * In any autonomy level less than `full`, the system will require an operator decision to continue

Whenever a plan is being executed, the `FlexBE` state machine transitions to a concurrent node that uses on line  planners to refine the plans as the robot moves, and also monitors the Turtlebot bumper status for collision.  The operator can terminate the execution early by selecting the appropriate transition in the `FlexBE UI`.  If this low level plan fails, the robot will request permission to initiate a recovery behavior; in `full` autonomy the system automatically initiates the recovery.

[ROS 2]: https://docs.ros.org/en/foxy/index.html
[FlexBE]: https://flexbe.github.io
[FlexBE App]: https://github.com/FlexBE/flexbe_app
[FlexBE Behavior Engine]: https://github.com/FlexBE/flexbe_behavior_engine
[Flexible Navigation]: https://github.com/FlexBE/flexible_navigation
[Wiki]: http://wiki.ros.org/flexible_navigation
[ROBOTIS Turtlebot3]: https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/
[ROS 2 Cartographer]: https://ros2-industrial-workshop.readthedocs.io/en/latest/_source/navigation/ROS2-Cartographer.html
[ROS 2 Installation]: https://docs.ros.org/en/foxy/Installation.html
[ROS 2 Navigation2]: https://navigation.ros.org/

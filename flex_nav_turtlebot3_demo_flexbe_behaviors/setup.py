#!/usr/bin/env python

import os
from glob import glob
from setuptools import setup

package_name = 'flex_nav_turtlebot3_demo_flexbe_behaviors'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='David Conner',
    maintainer_email='robotics@cnu.edu',
    description='Behaviors related to demonstration of flexible_navigation system Turtlebot3 models',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'turtlebot_flex_planner_sm = flex_nav_turtlebot3_demo_flexbe_behaviors.turtlebot_flex_planner_sm',
            'turtlebot_multi_level_flex_planner_sm = flex_nav_turtlebot3_demo_flexbe_behaviors.turtlebot_multi_level_flex_planner_sm',
            'turtlebot_simple_recovery_sm = flex_nav_turtlebot3_demo_flexbe_behaviors.turtlebot_simple_recovery_sm',
        ],
    },
)

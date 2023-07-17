#!/usr/bin/env python

import os
from glob import glob
from setuptools import setup

PACKAGE_NAME = 'flex_nav_turtlebot3_demo_flexbe_behaviors'

setup(
    name=PACKAGE_NAME,
    version='0.0.1',
    packages=[PACKAGE_NAME],
    data_files=[
        (os.path.join('share', PACKAGE_NAME), glob('launch/*.launch.py')),
        ('share/ament_index/resource_index/packages',
            ['resource/' + PACKAGE_NAME]),
        ('share/' + PACKAGE_NAME, ['package.xml']),
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
            'turtlebot_multi_level_flex_planner_sm = flex_nav_turtlebot3_demo_flexbe_behaviors.turtlebot_multi_level_flex_planner_sm',  # noqa
            'turtlebot_simple_recovery_sm = flex_nav_turtlebot3_demo_flexbe_behaviors.turtlebot_simple_recovery_sm',  # noqa
        ],
    },
)

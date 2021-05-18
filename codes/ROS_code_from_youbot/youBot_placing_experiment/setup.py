#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['youBot_placing_experiment', 'youBot_placing_experiment_ros'],
    package_dir={'youBot_placing_experiment': 'common/src/youBot_placing_experiment',
                 'youBot_placing_experiment_ros': 'ros/src/youBot_placing_experiment_ros'}
)

setup(**d)

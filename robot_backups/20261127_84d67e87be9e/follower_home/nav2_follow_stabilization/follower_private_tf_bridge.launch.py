"""Pose-only bridge using private follower TF; never creates TF or motion input."""
from pathlib import Path
import sys
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    source=Path(__file__).resolve().parent/'candidate/robot_ws/src/cnu_capstone_2026/convoy_robot/real_follower_bridge_tf_candidate.py'
    if not source.is_file():raise RuntimeError('Missing candidate bridge: '+str(source))
    return LaunchDescription([Node(executable=sys.executable,
        arguments=['-B',str(source)],name='real_follower_bridge',output='screen',
        remappings=[('/tf','/follower/tf_internal'),('tf','/follower/tf_internal'),
                    ('/tf_static','/follower/tf_static_internal'),('tf_static','/follower/tf_static_internal')])])

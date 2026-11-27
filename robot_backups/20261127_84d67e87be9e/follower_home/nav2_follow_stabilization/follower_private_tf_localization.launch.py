"""Follower AMCL on the same isolated TF topics as its sensors. No motion nodes."""
from pathlib import Path
import sys
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

HERE=Path(__file__).resolve().parent
if str(HERE) not in sys.path:sys.path.insert(0,str(HERE))
from private_tf_launch_core import private_tf_group


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('enable_localization',default_value='false'),
        DeclareLaunchArgument('map',default_value=str(Path.home()/
            'robot_ws/maps/leader_mapping_20261107_143101_9R24Wc/leader_map.yaml')),
        private_tf_group(Path.home()/'follower_diagnostics/follower_localization.launch.py',
                         dict(enable_localization=LaunchConfiguration('enable_localization'),
                              map=LaunchConfiguration('map')))])

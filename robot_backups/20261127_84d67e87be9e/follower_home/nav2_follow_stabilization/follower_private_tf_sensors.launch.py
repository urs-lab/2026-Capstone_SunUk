"""Opt-in stationary TF-isolation pilot; includes existing sensor launch unchanged."""
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
        DeclareLaunchArgument('enable_position_profile',default_value='false'),
        DeclareLaunchArgument('ekf_frequency',default_value='20.0'),
        private_tf_group(Path.home()/'follower_diagnostics/follower_position_fixed_lidar.launch.py',
                         dict(enable_position_profile=LaunchConfiguration('enable_position_profile'),
                              ekf_frequency=LaunchConfiguration('ekf_frequency')))])

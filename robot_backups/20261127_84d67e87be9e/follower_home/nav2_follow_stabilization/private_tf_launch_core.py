"""Use public launch APIs to scope follower TF remaps; frame IDs are unchanged."""
from pathlib import Path
from launch.actions import GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import SetRemap

TF_TOPIC='/follower/tf_internal'
STATIC_TOPIC='/follower/tf_static_internal'


def private_tf_group(source, arguments):
    source=Path(source)
    if not source.is_file():raise RuntimeError('Missing verified launch: '+str(source))
    # SetRemap is scoped to this group and overrides Node-local remap rules.
    # Cover absolute tf2 names as well as relative names in /follower nodes.
    return GroupAction(actions=[
        SetRemap(src='/tf',dst=TF_TOPIC),SetRemap(src='tf',dst=TF_TOPIC),
        SetRemap(src='/tf_static',dst=STATIC_TOPIC),SetRemap(src='tf_static',dst=STATIC_TOPIC),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(source)),
                                 launch_arguments=arguments.items())])

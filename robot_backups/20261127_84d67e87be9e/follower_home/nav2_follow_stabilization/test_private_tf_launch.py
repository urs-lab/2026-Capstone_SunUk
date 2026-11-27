import importlib.util
from pathlib import Path
import tempfile
import unittest
from launch.launch_context import LaunchContext
from launch_ros.actions import Node, SetRemap
from private_tf_launch_core import private_tf_group,TF_TOPIC,STATIC_TOPIC


class PrivateTFTests(unittest.TestCase):
    def test_group_remaps_precede_existing_amcl_rules_and_keep_non_tf_topics(self):
        with tempfile.TemporaryDirectory() as folder:
            source=Path(folder)/'sample.launch.py';source.write_text('# no execution')
            group=private_tf_group(source,dict(enable_localization='false'))
            actions=group.get_sub_entities()
            remaps=[a for a in actions if isinstance(a,SetRemap)]
            self.assertEqual(len(remaps),4)
            context=LaunchContext()
            for action in remaps:action.execute(context)
            node=Node(package='tf2_ros',executable='static_transform_publisher',namespace='follower',
                      remappings=[('tf','/tf'),('tf_static','/tf_static'),('map','/follower/localization_map')])
            node._perform_substitutions(context)
            rules=node.expanded_remapping_rules
            self.assertEqual(rules[:4],[('/tf',TF_TOPIC),('tf',TF_TOPIC),
                                         ('/tf_static',STATIC_TOPIC),('tf_static',STATIC_TOPIC)])
            self.assertEqual(next(dst for src,dst in rules if src=='tf'),TF_TOPIC)
            self.assertIn(('map','/follower/localization_map'),rules)

    def test_missing_launch_does_not_create_a_process(self):
        with self.assertRaises(RuntimeError):private_tf_group('/no/such/launch.py',{})

    def test_both_wrappers_generate_without_starting_hardware(self):
        for name in ('follower_private_tf_sensors.launch.py','follower_private_tf_localization.launch.py'):
            p=Path(__file__).with_name(name)
            spec=importlib.util.spec_from_file_location('wrapper',p)
            module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
            description=module.generate_launch_description()
            self.assertGreaterEqual(len(description.entities),3)

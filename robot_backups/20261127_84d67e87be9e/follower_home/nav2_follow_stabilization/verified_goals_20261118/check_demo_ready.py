"""Read-only live graph/resource audit; never publishes or calls arm/services."""
import json
import os
from pathlib import Path
import subprocess
import sys
import time
import argparse


def goal3_graph_issues(node,command_topic):
    issues=[]
    for topic in ('/cmd_vel',command_topic):
        names=[x.node_name for x in node.get_publishers_info_by_topic(topic)]
        if names:issues.append(dict(topic=topic,endpoint='publisher',expected_nodes=[],actual_nodes=names,
                                    reason='GOAL3_REQUIRES_OTHER_COMMAND_PUBLISHERS_STOPPED'))
        expected=['md200t_controller' if topic=='/cmd_vel' else 'follower_drive_candidate']
        names=[x.node_name for x in node.get_subscriptions_info_by_topic(topic)]
        if names!=expected:issues.append(dict(topic=topic,endpoint='subscriber',expected_nodes=expected,actual_nodes=names))
    for topic in ('/follower/odom_raw','/follower/scan','/leader/odom','/follower/motor_candidate/status',
                  '/follower/odometry/filtered','/follower/imu/data','/follower/odom_map'):
        names=[x.node_name for x in node.get_publishers_info_by_topic(topic)]
        if len(names)!=1:issues.append(dict(topic=topic,endpoint='publisher',expected_count=1,actual_nodes=names))
    return issues


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--goal3',action='store_true',help='Audit goal3 with evaluation stopped; Nav2 gate must be stopped')
    args=parser.parse_args()
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy
    from nav_msgs.msg import Odometry,Path as RosPath
    from sensor_msgs.msg import LaserScan,Imu
    from std_msgs.msg import String
    from action_msgs.msg import GoalStatusArray
    from tf2_msgs.msg import TFMessage
    source=Path.home()/'nav2_follow_stabilization/goal1_operator_20261118/candidate/follower_diagnostics'
    sys.path.insert(0,str(source))
    from local_trial_preflight import graph_issues,laser_mount_issue,mount_snapshot
    from local_avoidance_drive import COMMAND_TOPIC
    if args.goal3:COMMAND_TOPIC='/follower/motor_candidate/continuous20_command'
    rclpy.init(args=[]);node=rclpy.create_node('goal_demo_readonly_audit')
    node._use_direct_unicast=True;node._require_nav2_plan=False;node._require_nav2_goal_status=True
    executor=SingleThreadedExecutor();executor.add_node(node)
    state={};qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT);subscriptions=[]
    def receive(topic,msg):
        row=state.setdefault(topic,dict(count=0));row['count']+=1;row['last_rx']=time.monotonic()
        if hasattr(msg,'header'):
            row['source_ns']=msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
            row['frame']=msg.header.frame_id
            if hasattr(msg,'poses'):row['points']=len(msg.poses)
        if topic.endswith('/status') and hasattr(msg,'data'):
            try:
                s=json.loads(msg.data);row['motor']={k:s.get(k) for k in ('armed','fault','trial_max_rpm','trial_max_seconds','continuous20','feedback_fresh','telemetry_isolated')}
                row['source_ns']=s.get('status_ros_ns')
            except (ValueError,TypeError):row['parse_error']=True
        if hasattr(msg,'status_list'):row['goal_statuses']=[int(s.status) for s in msg.status_list]
    for topic,kind in (('/follower/odom_raw',Odometry),('/follower/odometry/filtered',Odometry),
                       ('/follower/odom_map',Odometry),('/follower/scan',LaserScan),
                       ('/leader/odom_unicast',Odometry),('/leader/path_unicast',RosPath),
                       ('/leader/odom',Odometry),('/leader/path',RosPath),('/follower/imu/data',Imu),
                       ('/follower/motor_candidate/status',String),('/leader/nav2_goal_status_unicast',GoalStatusArray)):
        subscriptions.append(node.create_subscription(kind,topic,lambda m,t=topic:receive(t,m),qos))
    mount=[]
    def static_callback(msg):
        for tf in msg.transforms:
            if tf.child_frame_id=='follower/laser':mount[:]=[tf]
    subscriptions.append(node.create_subscription(TFMessage,'/tf_static',static_callback,
        QoSProfile(depth=20,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)))
    print('8초 조회 전용: 실제 노드·발행자·입력·부하 확인. 주행·arm·설정 변경 없음.',flush=True)
    start=time.monotonic()
    try:
        while rclpy.ok() and time.monotonic()-start<8.:executor.spin_once(timeout_sec=.1)
        issues=graph_issues(node,nav2_leader=True,command_topic=COMMAND_TOPIC,
            node_name='local_avoidance_trial_operator',plan_only=False,pose_source='filtered',follow_only=True)
        # The operator is deliberately stopped for this audit. Do not report
        # its missing command publisher as a missing supporting node.
        expected_absence=[x for x in issues if x['topic']==COMMAND_TOPIC and x.get('endpoint')=='publisher' and x.get('actual_nodes')==[]]
        blockers=[x for x in issues if x not in expected_absence]
        existing_operator=node.get_publishers_info_by_topic(COMMAND_TOPIC)
        if existing_operator:
            blockers.append(dict(topic=COMMAND_TOPIC,endpoint='publisher',
                reason='OPERATOR_EXPECTED_STOPPED_FOR_AUDIT',expected_nodes=[],
                actual_nodes=[x.node_name for x in existing_operator]))
        mount_problem=laser_mount_issue(mount[0] if mount else None)
        if args.goal3:
            blockers=goal3_graph_issues(node,COMMAND_TOPIC);expected_absence=[]
            if mount_problem:blockers.append(dict(topic='/tf_static',reason=mount_problem,
                expected='follower/base_link -> follower/laser; confirmed mount'))
        topics=['/cmd_vel','/cmd_vel_smoothed',COMMAND_TOPIC,'/leader/odom','/leader/nav2_goal_status_unicast',*state]
        endpoints={}
        for topic in dict.fromkeys(topics):
            endpoints[topic]={direction:[dict(node=p.node_name,namespace=p.node_namespace,gid=list(p.endpoint_gid)) for p in getter(topic)]
                for direction,getter in (('publishers',node.get_publishers_info_by_topic),('subscribers',node.get_subscriptions_info_by_topic))}
        mono=time.monotonic();now=node.get_clock().now().nanoseconds
        for row in state.values():
            row['receive_age_s']=mono-row.pop('last_rx')
            if type(row.get('source_ns')) is int:row['source_age_s']=(now-row['source_ns'])/1e9
        process=subprocess.run(['ps','-eo','pid,comm,pcpu,pmem','--sort=-pcpu'],capture_output=True,text=True)
        result=dict(read_only=True,mode='goal3' if args.goal3 else 'goal1',graph_blockers=blockers,static_mount_problem=mount_problem,static_mount=mount_snapshot(mount[0] if mount else None),operator_absent_expected=expected_absence,
            inputs=state,endpoints=endpoints,nodes=node.get_node_names_and_namespaces(),
            load_average=os.getloadavg(),cpu_count=os.cpu_count(),top_processes=process.stdout.splitlines()[:16])
        directory=Path(__file__).parent/'reports';directory.mkdir(exist_ok=True)
        report=directory/('demo_readiness_'+str(time.time_ns())+'.json')
        report.write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
        print('GRAPH_BLOCKERS='+json.dumps(blockers,ensure_ascii=False),flush=True)
        print('INPUTS='+json.dumps(state,ensure_ascii=False),flush=True)
        print('REPORT='+str(report),flush=True)
    finally:
        executor.remove_node(node);executor.shutdown(timeout_sec=0.);node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()

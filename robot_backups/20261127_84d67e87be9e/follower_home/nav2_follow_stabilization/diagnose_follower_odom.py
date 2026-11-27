#!/usr/bin/env python3
"""Read-only follower pose diagnosis. No command publishers or state changes."""
import json,time
from collections import Counter,deque


def diagnose(inputs,publishers,state,lookup):
    if not publishers.get('/follower/odom_map'):return 'FOLLOWER_BRIDGE_NOT_DISCOVERED'
    ekf=inputs.get('/follower/odometry/filtered')
    if not ekf:return 'FILTERED_ODOM_NOT_RECEIVED'
    if ekf['frame']!='follower/odom' or ekf['child_frame']!='follower/base_link':return 'FILTERED_ODOM_FRAME_MISMATCH'
    if not 0<=ekf['source_age_s']<=.5 or ekf['receive_age_s']>.5:return 'FILTERED_ODOM_STALE_OR_FUTURE'
    if state is not None and state!='active':return 'FOLLOWER_AMCL_'+state.upper()
    if not lookup.get('available'):return 'PRIVATE_MAP_TO_BASE_TF_MISSING'
    if not 0<=lookup['source_age_s']<=.5:return 'PRIVATE_MAP_TO_BASE_TF_STALE_OR_FUTURE'
    if '/follower/odom_map' not in inputs:return 'BRIDGE_INPUTS_PRESENT_BUT_NO_OUTPUT_SEE_WARNINGS'
    return 'FOLLOWER_MAP_ODOM_RECEIVED'


def main():
    import argparse
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--tf-scope',choices=('private','public'),default='private')
    parser.add_argument('--seconds',type=int,choices=(8,15,30),default=8)
    args=parser.parse_args()
    selected_topics=('/follower/tf_internal','/follower/tf_static_internal') if args.tf_scope=='private' else ('/tf','/tf_static')
    import rclpy
    from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy
    from rclpy.time import Time
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import LaserScan
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from tf2_msgs.msg import TFMessage
    from lifecycle_msgs.srv import GetState
    from rcl_interfaces.msg import Log
    from tf2_ros import Buffer
    rclpy.init(args=[]);node=rclpy.create_node('follower_odom_readonly_diagnostic')
    buffer=Buffer();counts=Counter();latest={};edges={};warnings=deque(maxlen=5)
    qos=QoSProfile(depth=5,reliability=ReliabilityPolicy.BEST_EFFORT)
    static=QoSProfile(depth=20,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    def ns():return node.get_clock().now().nanoseconds
    def stamp(m):return m.header.stamp.sec*10**9+m.header.stamp.nanosec
    def received(topic,m):
        counts[topic]+=1;latest[topic]=(stamp(m),time.monotonic(),m.header.frame_id,getattr(m,'child_frame_id',None))
    def tf(topic,msg,is_static=False):
        for t in msg.transforms:
            if 'follower' not in t.child_frame_id and 'follower' not in t.header.frame_id:continue
            key=topic+':'+t.header.frame_id+' -> '+t.child_frame_id
            previous=edges.get(key);edges[key]={'count':1 if previous is None else previous['count']+1,'stamp_ns':stamp(t)}
            if topic in selected_topics:
                try:
                    if is_static:buffer.set_transform_static(t,'diagnostic')
                    else:buffer.set_transform(t,'diagnostic')
                except Exception as exc:warnings.append('TF_RECEIVE: '+str(exc))
    def log(m):
        if m.name in ('real_follower_bridge','follower.amcl','amcl') and m.level>=30:warnings.append(m.name+': '+m.msg)
    types={'/follower/odom_raw':Odometry,'/follower/odometry/filtered':Odometry,
           '/follower/odom_map':Odometry,'/follower/amcl_pose':PoseWithCovarianceStamped,'/follower/scan':LaserScan}
    subs=[node.create_subscription(kind,topic,lambda m,t=topic:received(t,m),qos) for topic,kind in types.items()]
    subs += [node.create_subscription(TFMessage,topic,lambda m,t=topic,s=s:tf(t,m,s),static if s else qos)
             for topic,s in [('/follower/tf_internal',False),('/follower/tf_static_internal',True),('/tf',False),('/tf_static',True)]]
    subs.append(node.create_subscription(Log,'/rosout',log,qos))
    client=node.create_client(GetState,'/follower/amcl/get_state');future=None;state=None
    print(f'{args.seconds}초 조회 전용: {args.tf_scope} TF·오뎀·AMCL·Bridge 진단. 주행·arm·설정 변경 없음.',flush=True)
    try:
        end=time.monotonic()+args.seconds
        while time.monotonic()<end:
            rclpy.spin_once(node,timeout_sec=.04)
            if future is None and client.service_is_ready():future=client.call_async(GetState.Request())
            if future is not None and future.done():
                try:state=future.result().current_state.label
                except Exception:pass
        now=ns();mono=time.monotonic()
        inputs={k:dict(count=counts[k],source_age_s=(now-v[0])/1e9,receive_age_s=mono-v[1],frame=v[2],child_frame=v[3]) for k,v in latest.items()}
        publishers={topic:[dict(node=p.node_name,namespace=p.node_namespace) for p in node.get_publishers_info_by_topic(topic)] for topic in types}
        try:
            t=buffer.lookup_transform('map','follower/base_link',Time())
            lookup=dict(available=True,source_age_s=(now-stamp(t))/1e9)
        except Exception as exc:lookup=dict(available=False,error=str(exc))
        for row in edges.values():row['source_age_s']=(now-row.pop('stamp_ns'))/1e9
        tf_subscribers={topic:[dict(node=p.node_name,namespace=p.node_namespace,reliability=str(p.qos_profile.reliability)) for p in node.get_subscriptions_info_by_topic(topic)] for topic in ('/tf','/follower/tf_internal')}
        reason=diagnose(inputs,publishers,state,lookup)
        if args.tf_scope=='public':reason=reason.replace('PRIVATE_','PUBLIC_')
        print(json.dumps(dict(reason=reason,tf_scope=args.tf_scope,amcl_state=state,
            inputs=inputs,publishers=publishers,tf_edges=edges,map_to_base_lookup=lookup,
            tf_subscribers=tf_subscribers,recent_warnings=list(warnings)),ensure_ascii=False,indent=2))
    finally:node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()

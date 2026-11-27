#!/usr/bin/env python3
"""Portable read-only local-vs-remote input audit; no project imports or commands."""
import argparse
from collections import defaultdict
import json
from pathlib import Path
import socket
import time


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds',type=int,choices=(15,30),default=15)
    parser.add_argument('--tf-topic',choices=('/tf','/follower/tf_internal'),default='/tf')
    parser.add_argument('--trace',action='store_true',help='Save original source and receipt times after observation')
    parser.add_argument('--stream-only',action='store_true',help='Only best-effort subscriptions; no lifecycle clients or graph queries')
    args=parser.parse_args()
    import rclpy
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import LaserScan
    from tf2_msgs.msg import TFMessage
    from lifecycle_msgs.srv import GetState
    from rclpy.qos import QoSProfile,ReliabilityPolicy,qos_profile_sensor_data
    rclpy.init();node=rclpy.create_node('convoy_host_audit')
    ages=defaultdict(list);stamps=defaultdict(list);rx=defaultdict(list)
    trace=[]
    subscriptions=[]
    def record(key,stamp):
        ns=stamp.sec*10**9+stamp.nanosec
        received_ns=node.get_clock().now().nanoseconds
        ages[key].append((received_ns-ns)/1e9 if ns else None)
        stamps[key].append(ns);rx[key].append(time.monotonic())
        if args.trace:
            trace.append(dict(topic=key,source_ns=ns,received_ros_ns=received_ns,
                              received_monotonic=rx[key][-1]))
    topics=[('/odom',Odometry),('/odometry/filtered',Odometry),('/scan',LaserScan),
            ('/leader/odom',Odometry),('/follower/odometry/filtered',Odometry),
            ('/follower/scan',LaserScan),('/follower/odom_map',Odometry)]
    for topic,kind in topics:
        subscriptions.append(node.create_subscription(kind,topic,
            lambda m,t=topic:record(t,m.header.stamp),qos_profile_sensor_data))
    edges={('odom','base_link'),('map','odom'),('follower/odom','follower/base_link'),
           ('map','follower/odom')}
    def tf_callback(msg,label):
        for tf in msg.transforms:
            edge=(tf.header.frame_id,tf.child_frame_id)
            if edge in edges:record(label+':'+edge[0]+'->'+edge[1],tf.header.stamp)
    for label,qos in [('tf_best_effort',qos_profile_sensor_data),
                      ('tf_reliable',QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE))]:
        if args.stream_only and label=='tf_reliable':continue
        subscriptions.append(node.create_subscription(TFMessage,args.tf_topic,
            lambda m,l=label:tf_callback(m,l),qos))
    clients={} if args.stream_only else {name:node.create_client(GetState,name+'/get_state') for name in ('/amcl','/follower/amcl')}
    pending={};states={};last_query=float('-inf')
    print('조회 전용: '+socket.gethostname()+'에서 15/30초 관측. 주행·설정 변경 없음.',flush=True)
    start=time.monotonic()
    loop_previous=start;loop_gap_max=0.;loop_stalls=[]
    try:
        while time.monotonic()-start<args.seconds:
            loop_now=time.monotonic();loop_gap=loop_now-loop_previous;loop_previous=loop_now
            loop_gap_max=max(loop_gap_max,loop_gap)
            if loop_gap>.1:loop_stalls.append(dict(elapsed_s=loop_now-start,gap_s=loop_gap))
            rclpy.spin_once(node,timeout_sec=.02)
            if time.monotonic()-last_query>1.:
                last_query=time.monotonic()
                for name,client in clients.items():
                    if name not in pending and client.service_is_ready():pending[name]=client.call_async(GetState.Request())
            for name,future in list(pending.items()):
                if future.done():
                    try:states[name]=future.result().current_state.label
                    except Exception as exc:states[name]=str(exc)
                    del pending[name]
        result={}
        for name,values in ages.items():
            valid=sorted(v for v in values if v is not None)
            result[name]=dict(received=len(values),rate_hz=len(values)/(time.monotonic()-start),
                age_min_p95_max_s=[valid[0],valid[int(.95*(len(valid)-1))],valid[-1]] if valid else None,
                repeated_stamps=sum(a==b for a,b in zip(stamps[name],stamps[name][1:])),
                maximum_receive_gap_s=max((b-a for a,b in zip(rx[name],rx[name][1:])),default=None))
        graph={} if args.stream_only else {topic:[dict(node=e.node_name,namespace=e.node_namespace,
                    reliability=e.qos_profile.reliability.name) for e in node.get_publishers_info_by_topic(topic)]
               for topic,_ in topics}
        report=dict(host=socket.gethostname(),tf_topic=args.tf_topic,stream_only=args.stream_only,
                    observer_loop_gap_max_s=loop_gap_max,observer_stalls=loop_stalls,
                    amcl_states=states,inputs=result,publishers=graph)
        path=Path.home()/f'convoy_host_audit_{time.time_ns()}.json'
        path.write_text(json.dumps(report,indent=2)+'\n')
        if args.trace:
            trace_path=path.with_suffix('.jsonl')
            with trace_path.open('w') as stream:
                for row in trace:stream.write(json.dumps(row)+'\n')
            print('TRACE='+str(trace_path),flush=True)
        print(json.dumps(report,ensure_ascii=False,indent=2),flush=True)
        print('REPORT='+str(path),flush=True)
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()

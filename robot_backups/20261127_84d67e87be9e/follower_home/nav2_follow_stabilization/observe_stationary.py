#!/usr/bin/env python3
"""Read-only 30s readiness audit. Never publishes or calls motion services."""
import argparse
from collections import defaultdict
import hashlib
import json
from pathlib import Path
import time
from stability_core import Readiness


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds',type=int,default=30,choices=(30,60))
    parser.add_argument('--private-follower-tf',action='store_true')
    args=parser.parse_args()
    root=Path(__file__).resolve().parent
    protected=Path.home()/'final_robot'
    baseline=json.loads((root/'protected_baseline_sha256.json').read_text())
    altered=[name for name,sha in baseline.items() if not (protected/name).is_file()
             or hashlib.sha256((protected/name).read_bytes()).hexdigest()!=sha]
    if altered:
        raise SystemExit('BACKUP_CHANGED: '+json.dumps(altered))
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import Imu,LaserScan
    from std_msgs.msg import String
    from tf2_msgs.msg import TFMessage
    from lifecycle_msgs.srv import GetState
    required={name:(.3,.05) for name in ('leader_pose','follower_pose','filtered',
              'position','imu','scan','leader_scan','motor','leader_tf','follower_tf')}
    required.update({name:(.3,1.1) for name in ('leader_map_tf','follower_map_tf')})
    lifecycle_topics={
        'leader_amcl':'/amcl/get_state','follower_amcl':'/follower/amcl/get_state',
        'planner':'/planner_server/get_state','controller':'/controller_server/get_state',
        'navigator':'/bt_navigator/get_state','velocity_smoother':'/velocity_smoother/get_state'}
    required.update({name:(2.,0.) for name in (*lifecycle_topics,'graph')})
    policy=Readiness(required)
    counts=defaultdict(int);ages=defaultdict(list);rows=[];subs=[]
    missing_stamps=defaultdict(int)
    rclpy.init();node=rclpy.create_node('convoy_stationary_audit')
    def accept(name,stamp,valid=True):
        now_ns=node.get_clock().now().nanoseconds
        policy.observe(name,stamp,time.monotonic(),valid)
        counts[name]+=1
        if stamp>0:ages[name].append((now_ns-stamp)/1e9)
        else:missing_stamps[name]+=1
    def measurement(name,msg,frame,child=None):
        valid=msg.header.frame_id==frame and (child is None or msg.child_frame_id==child)
        if child is not None:
            twist=msg.twist.twist
            valid=valid and abs(twist.linear.x)<=.01 and abs(twist.angular.z)<=.02
        accept(name,msg.header.stamp.sec*10**9+msg.header.stamp.nanosec,valid)
    sources=[('leader_pose','/leader/odom',Odometry,'map','base_link'),
             ('follower_pose','/follower/odom_map',Odometry,'map','follower/base_link'),
             ('filtered','/follower/odometry/filtered',Odometry,'follower/odom','follower/base_link'),
             ('position','/follower/odom_position_candidate',Odometry,'follower/odom','follower/base_link'),
             ('imu','/follower/imu/data',Imu,'follower/imu_link',None),
             ('scan','/follower/scan',LaserScan,'follower/laser',None),
             ('leader_scan','/scan',LaserScan,'laser',None)]
    for name,topic,kind,frame,child in sources:
        subs.append(node.create_subscription(kind,topic,
            lambda msg,n=name,f=frame,c=child:measurement(n,msg,f,c),qos_profile_sensor_data))
    def motor(msg):
        try:
            status=json.loads(msg.data)
            valid=(status.get('armed') is False and status.get('fault') is None
                   and status.get('feedback_fresh') is True and status.get('io_failed') is False
                   and status.get('measured_rpm')==[0,0])
            accept('motor',status.get('status_ros_ns',0),valid)
        except (ValueError,TypeError,AttributeError):
            accept('motor',0,False)
    subs.append(node.create_subscription(String,'/follower/motor_candidate/status',motor,qos_profile_sensor_data))
    edges={('odom','base_link'):'leader_tf',('map','odom'):'leader_map_tf',
           ('follower/odom','follower/base_link'):'follower_tf',
           ('map','follower/odom'):'follower_map_tf'}
    def transforms(msg, private=False):
        for tf in msg.transforms:
            name=edges.get((tf.header.frame_id,tf.child_frame_id))
            if args.private_follower_tf and name and ('follower' in name)!=private:continue
            if name:accept(name,tf.header.stamp.sec*10**9+tf.header.stamp.nanosec)
    subs.append(node.create_subscription(TFMessage,'/tf',transforms,qos_profile_sensor_data))
    if args.private_follower_tf:
        subs.append(node.create_subscription(TFMessage,'/follower/tf_internal',
                    lambda msg:transforms(msg,True),qos_profile_sensor_data))
    clients={n:node.create_client(GetState,t) for n,t in lifecycle_topics.items()}
    pending={};lifecycle_states={};last_check=float('-inf')
    def graph():
        valid=all(len(node.get_publishers_info_by_topic(topic))==1 for _,topic,*_ in sources)
        writers=node.get_publishers_info_by_topic('/cmd_vel')
        valid=valid and len(writers)==1 and writers[0].node_name=='nav2_convoy_gate'
        accept('graph',node.get_clock().now().nanoseconds,valid)
    print('정지 상태 조회 전용: 주행·arm·목표·설정 변경 없이 30/60초 관측합니다.',flush=True)
    start=time.monotonic();last_report=None;ready_at_end=False
    try:
        while time.monotonic()-start<args.seconds:
            rclpy.spin_once(node,timeout_sec=.02)
            mono=time.monotonic();ns=node.get_clock().now().nanoseconds
            for name,future in list(pending.items()):
                if future.done():
                    try:
                        state=future.result().current_state
                        lifecycle_states[name]=dict(id=state.id,label=state.label,observed_ros_ns=ns)
                        valid=state.id==3
                    except Exception as exc:
                        lifecycle_states[name]=dict(error=str(exc));valid=False
                    accept(name,ns,valid);del pending[name]
            if mono-last_check>=1.:
                last_check=mono;graph()
                for name,client in clients.items():
                    if name not in pending and client.service_is_ready():
                        pending[name]=client.call_async(GetState.Request())
            # Graph and service processing can advance the clock. Compare all
            # observations against a timestamp taken AFTER that processing.
            mono=time.monotonic();ns=node.get_clock().now().nanoseconds
            state=policy.evaluate(ns,mono);ready_at_end=state['ready']
            rows.append(dict(elapsed_s=mono-start,**state))
            signature=(state['ready'],tuple(state['issues']))
            if signature!=last_report:
                print(json.dumps(state),flush=True);last_report=signature
        stats={}
        for name,values in ages.items():
            values=sorted(values)
            stats[name]=dict(count=counts[name],age_min_s=values[0],
                            age_p95_s=values[int(.95*(len(values)-1))],age_max_s=values[-1])
        out=root/'reports';out.mkdir(exist_ok=True)
        path=out/f'stationary_{time.time_ns()}.json'
        path.write_text(json.dumps(dict(ready_at_end=ready_at_end,
            baseline_files_checked=len(baseline),private_follower_tf=args.private_follower_tf,
            lifecycle_states=lifecycle_states,
            missing_source_stamps=dict(missing_stamps),
            stats=stats,timeline=rows),indent=2)+'\n')
        print('REPORT='+str(path),flush=True)
        return 0 if ready_at_end else 2
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':
    raise SystemExit(main())

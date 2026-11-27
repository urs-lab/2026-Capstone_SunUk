"""Observe lifecycle and map odom. Creates no publishers or state-change calls."""
import time
from readiness_policy import ready,odom_ready


def main():
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from lifecycle_msgs.srv import GetState
    from std_srvs.srv import Trigger
    from nav_msgs.msg import Odometry
    from rclpy.qos import QoSProfile,ReliabilityPolicy
    rclpy.init();node=rclpy.create_node('follower_localization_readiness')
    executor=SingleThreadedExecutor();executor.add_node(node)
    clients={key:node.create_client(GetState,'/follower/'+key+'/get_state') for key in ('map_server','amcl')}
    clients['manager']=node.create_client(Trigger,'/follower/localization_manager/is_active')
    states={};manager=None;odom=None;pending={};queried={};active=False;odom_seen=False;last_report=0.
    def receive(msg):
        nonlocal odom
        if msg.header.frame_id=='map' and msg.child_frame_id=='follower/base_link':
            odom=(msg.header.stamp.sec*10**9+msg.header.stamp.nanosec,time.monotonic())
    sub=node.create_subscription(Odometry,'/follower/odom_map',receive,
        QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT))
    try:
        while rclpy.ok():
            executor.spin_once(timeout_sec=.1);mono=time.monotonic()
            for key,client in clients.items():
                if key in pending:
                    future,began=pending[key]
                    if future.done():
                        try:
                            response=future.result()
                            if key=='manager':manager=(bool(response.success),mono)
                            else:states[key]=(int(response.current_state.id),mono)
                        except Exception as exc:node.get_logger().warning('Readiness query failed: '+str(exc))
                        del pending[key]
                    elif mono-began>3.:
                        future.cancel();del pending[key]
                if key not in pending and mono-queried.get(key,0.)>=1. and client.service_is_ready():
                    request=Trigger.Request() if key=='manager' else GetState.Request()
                    pending[key]=(client.call_async(request),mono);queried[key]=mono
            current=ready(states,manager,mono)
            if current and not active:
                print('[LOCALIZATION_ACTIVE] 지도 서버·AMCL active 및 manager bond 기동 완료. 이제 RViz /follower/initialpose로 실제 위치·방향을 지정하세요.',flush=True)
            elif not current and mono-last_report>=5.:
                print('[LOCALIZATION_NOT_READY] states='+str({k:v[0] for k,v in states.items()})+
                      ', manager_active='+str(manager[0] if manager else None)+'. 주행 준비 미완료.',flush=True)
                last_report=mono
            active=current
            current_odom=current and odom_ready(odom,node.get_clock().now().nanoseconds,mono)
            if current_odom and not odom_seen:print('[FOLLOWER_MAP_ODOM_READY] 새 map 오뎀 수신 확인.',flush=True)
            odom_seen=current_odom
    except KeyboardInterrupt:pass
    finally:
        executor.remove_node(node);executor.shutdown(timeout_sec=0.);node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()

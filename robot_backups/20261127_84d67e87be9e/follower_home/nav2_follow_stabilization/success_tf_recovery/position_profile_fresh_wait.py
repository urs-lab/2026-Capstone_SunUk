"""Wait for fresh position input before starting sensors/EKF. No motor APIs."""
import math
import time


def readiness(source_count, filtered_count, sample, now_ns, mono):
    if source_count > 1:
        raise RuntimeError('Require exactly one /follower/odom_position_candidate publisher')
    if filtered_count:
        raise RuntimeError('Existing follower filtered odom publisher; stop old observation/EKF first')
    if sample is None:
        return False, '위치 데이터 수신 대기'
    stamp_ns, received_mono, frame, child = sample
    if frame != 'follower/odom' or child != 'follower/base_link':
        raise RuntimeError('Wrong candidate frames')
    age = (now_ns - stamp_ns) / 1e9
    receive_age = mono - received_mono
    ready = source_count == 1 and math.isfinite(age) and 0 <= age <= .3 and 0 <= receive_age <= .3
    return ready, f'발행자={source_count}, 원본 나이={age:.3f}s, 수신 나이={receive_age:.3f}s'


def preflight():
    import rclpy
    from rclpy.context import Context
    from rclpy.node import Node
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from nav_msgs.msg import Odometry
    context = Context()
    rclpy.init(args=[], context=context)
    node = Node('position_profile_preflight', context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    latest = [None]

    def received(message):
        latest[0] = (message.header.stamp.sec * 10**9 + message.header.stamp.nanosec,
                     time.monotonic(), message.header.frame_id, message.child_frame_id)

    subscription = node.create_subscription(
        Odometry, '/follower/odom_position_candidate', received,
        QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
    next_notice = 0.
    try:
        while rclpy.ok(context=context):
            executor.spin_once(timeout_sec=.1)
            sources = len(node.get_publishers_info_by_topic('/follower/odom_position_candidate'))
            filtered = len(node.get_publishers_info_by_topic('/follower/odometry/filtered'))
            mono = time.monotonic()
            ready, detail = readiness(sources, filtered, latest[0], node.get_clock().now().nanoseconds, mono)
            if ready:
                print('[POSITION_READY] 신선한 위치 입력 확인. 센서·EKF 실행을 계속합니다.', flush=True)
                return
            if mono >= next_notice:
                print('[POSITION_WAIT] ' + detail + '; 새 입력까지 대기. 주행·arm 없음.', flush=True)
                next_notice = mono + 3.
        raise RuntimeError('Position input wait interrupted; sensors not started')
    finally:
        executor.remove_node(node)
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok(context=context):
            rclpy.shutdown(context=context)

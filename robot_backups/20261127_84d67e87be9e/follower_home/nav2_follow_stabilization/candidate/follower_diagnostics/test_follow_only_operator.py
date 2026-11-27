import json,math,sys,tempfile,unittest
from pathlib import Path
from types import SimpleNamespace as S
from unittest.mock import patch
from test_local_avoidance_operator import Flow
import run_local_avoidance_trial as runner
from directed_path_tracker import DirectedPathTracker

class Tests(unittest.TestCase):
    def flow(self,plan_only=False):
        f=Flow(plan_only=plan_only);modules,core=f.modules();Base=modules['follower_pursuit_preview'].PreviewCore
        class Nav(Base):
            def __init__(self,lookahead=.8):super().__init__();self.path_tracker=DirectedPathTracker();self.lookahead=lookahead;self.heading_limit=math.pi/4;self.last_selection=None
            def path_callback(self,m,ns,rx,key='path',persistent=False):return super().path_callback(m,ns,rx)
            def command(self,*args):
                if 'path' not in self.inputs:return 0.,0.,'STOP_MISSING_OR_INVALID_PATH'
                self.last_selection=self.path_tracker.select(self.inputs['path'][2],self.inputs['follower'][2],.8)
                return super().command(*args)
        modules['tracked_pursuit_candidate']=S(TrackedPreviewCore=Nav)
        def submit(fn,*args):return S(done=lambda:True,result=lambda:fn(*args))
        with tempfile.TemporaryDirectory() as d:
            root=Path(d);ref=root/'reference.json';ref.write_text('{}')
            with patch.dict(sys.modules,modules),patch.dict('os.environ',{'ROS_DOMAIN_ID':'10'}),patch('sys.argv',['local','--plan-only' if plan_only else '--run','--ack-clear-both-power-cut-ready','--reference-plan',str(ref),'--pose-source','filtered','--apf-pivot','--gentle-pivot','--map-rejoin','--follow-only']),patch.object(runner,'ROOT',root),patch.object(runner,'LocalCore',core),patch.object(runner,'ThreadPoolExecutor',return_value=S(submit=submit,shutdown=lambda **kw:None)),patch.object(runner,'reference_from_plan',return_value=((0,0),0.,0.,0.)),patch.object(runner.time,'monotonic',side_effect=lambda:f.t),patch.object(runner,'wait_for_start',return_value=True):
                code=runner.main()
            rows=list(map(json.loads,next((root/'logs').glob('*.jsonl')).read_text().splitlines()))
        return f,code,rows

    def auto_flow(self,prepare_only=False,leader_rotation=False,leader_reverse=False,front_obstacle_x=None,disable_clearance_stop=False,missing_filtered=False,reference_loss=False,persistent_reference_loss=False,goal_end_during_wait=False,motor_fault_during_wait=False):
        class Nav2Flow(Flow):
            def create_subscription(self,kind,topic,callback,qos):
                if topic=='/navigate_to_pose/_action/status':
                    original=callback
                    end=100. if persistent_reference_loss else (12. if reference_loss and not goal_end_during_wait else 8.)
                    callback=lambda msg:original(S(status_list=[S(status=2 if self.t<end and not prepare_only else 4)]))
                elif reference_loss and topic=='/leader/path':
                    original=callback
                    def callback(msg):
                        self.leader_v=.04 if self.armed else 0.
                        msg.poses=[S(pose=S(position=S(x=x,y=0.))) for x in (0.,2.)]
                        age=(self.ros()-self.arm_ns)/1e9 if self.arm_ns else 0.
                        if self.armed and age>=.4 and (persistent_reference_loss or age<1.):
                            old=self.ros()-2_000_000_000
                            msg.header.stamp=S(sec=old//10**9,nanosec=old%10**9)
                        original(msg)
                elif (leader_rotation or leader_reverse) and topic=='/leader/path':
                    original=callback
                    def callback(msg):
                        msg.poses=[S(pose=S(position=S(x=x,y=0.))) for x in (0.,2.)]
                        original(msg)
                elif missing_filtered and topic=='/follower/odometry/filtered':
                    original=callback
                    def callback(msg):
                        if self.armed:
                            msg.twist.twist.linear.x=.2
                            self.events.append(('filtered_rejected_sample',self.t))
                        original(msg)
                return super().create_subscription(kind,topic,callback,qos)
            def get_publishers_info_by_topic(self,topic):
                names={
                    '/cmd_vel':['nav2_convoy_gate'],
                    '/cmd_vel_smoothed':['velocity_smoother'],
                    '/navigate_to_pose/_action/status':['bt_navigator'],
                    runner.COMMAND_TOPIC:['local_avoidance_trial_operator'],
                }
                return [S(node_name=name) for name in names.get(topic,['source'])]
            def get_subscriptions_info_by_topic(self,topic):
                names={
                    '/cmd_vel':['md200t_controller'],
                    '/cmd_vel_smoothed':['nav2_convoy_gate'],
                    runner.COMMAND_TOPIC:['follower_drive_candidate'],
                }
                return [S(node_name=name) for name in names.get(topic,['source'])]

        f=Nav2Flow();modules,core=f.modules();Base=modules['follower_pursuit_preview'].PreviewCore
        if front_obstacle_x is not None:
            modules['watch_apf_case3']=S(project=lambda *args:[(front_obstacle_x,y) for y in (-.04,-.02,0.,.02,.04)])
        if prepare_only:
            original_state=f.state
            f.state=lambda:dict(original_state(),enabled=False)
        if disable_clearance_stop:
            original_clearance_state=f.state
            f.state=lambda:dict(original_clearance_state(),clearance_stop_enabled=False)
        if motor_fault_during_wait:
            original_fault_state=f.state
            f.state=lambda:dict(original_fault_state(),fault='DEVICE_STATUS_NONZERO'
                if f.arm_ns and f.armed and (f.ros()-f.arm_ns)/1e9>.8 else None)
        class Nav(Base):
            def __init__(self,lookahead=.8):
                super().__init__();self.path_tracker=DirectedPathTracker();self.lookahead=lookahead
                self.heading_limit=math.pi/4;self.last_selection=None
            def path_callback(self,m,ns,rx,key='path',persistent=False):
                if not m.poses:self.inputs.pop(key,None);return
                return super().path_callback(m,ns,rx)
            def command(self,ns,mono):
                if leader_rotation and f.armed and .3<=(f.ros()-f.arm_ns)/1e9<=.8:
                    return 0.,0.,'STOP_LEADER_ROTATING'
                if leader_reverse and f.armed and .3<=(f.ros()-f.arm_ns)/1e9<=.8:
                    return 0.,0.,'STOP_LEADER_REVERSE'
                if 'path' not in self.inputs:return 0.,0.,'STOP_MISSING_OR_INVALID_PATH'
                self.last_selection=self.path_tracker.select(self.inputs['path'][2],
                    self.inputs['follower'][2],self.lookahead)
                return super().command(ns,mono)
        modules['tracked_pursuit_candidate']=S(TrackedPreviewCore=Nav)
        with tempfile.TemporaryDirectory() as d:
            root=Path(d);ref=root/'unused_reference.json';ref.write_text('{}')
            argv=['local','--run','--ack-clear-both-power-cut-ready','--follow-only',
                  '--auto-start-on-goal','--follow-check-seconds','360',
                  '--pose-source','filtered','--nav2-leader','--seconds','30']
            if prepare_only:argv.append('--prepare-only')
            if disable_clearance_stop:argv.append('--disable-clearance-stop')
            with patch.dict(sys.modules,modules),patch.dict('os.environ',{'ROS_DOMAIN_ID':'10'}),\
                 patch('sys.argv',argv),patch.object(runner,'ROOT',root),\
                 patch.object(runner,'LocalCore',core),\
                 patch.object(runner,'ThreadPoolExecutor',return_value=S(submit=lambda *args:S(done=lambda:True,result=lambda:None),shutdown=lambda **kw:None)),\
                 patch.object(runner.time,'monotonic',side_effect=lambda:f.t),\
                 patch.object(runner,'wait_for_start',side_effect=AssertionError('auto mode must not prompt')):
                code=runner.main()
            rows=[json.loads(x) for x in next((root/'logs').glob('*.jsonl')).read_text().splitlines()]
        return f,code,rows
    def test_follow_check_stops_after_ten_seconds_with_both_zeros(self):
        f,code,rows=self.flow();self.assertEqual(code,0)
        end=rows[-1];self.assertEqual(end['reason'],'FOLLOW_CHECK_TIME_LIMIT');self.assertTrue(end['follower_zero_confirmed']);self.assertTrue(end['leader_zero_confirmed'])
        arm=next(e[1] for e in f.events if e[0]=='arm');disarm=next(e[1] for e in f.events if e[0]=='disarm')
        self.assertLessEqual(disarm-arm,10.1)
        self.assertTrue(all(e[1]['w']==0. for e in f.events if e[0]=='envelope'))
    def test_plan_only_has_no_publishers_or_arm(self):
        f,code,rows=self.flow(True);self.assertEqual(code,0)
        self.assertFalse(any(e[0] in ('arm','leader','envelope') for e in f.events))
        self.assertTrue(any(r['event']=='follow_check_preview' for r in rows))

    def test_nav2_goal_auto_starts_without_operator_start_and_stops_at_goal_terminal(self):
        f,code,rows=self.auto_flow();self.assertEqual(code,1)  # No fake wheel motion is injected.
        end=rows[-1]
        self.assertEqual(end['reason'],'NAV2_GOAL_FINISHED')
        self.assertTrue(end['leader_started']);self.assertTrue(end['arm_requested'])
        self.assertTrue(end['leader_zero_confirmed']);self.assertTrue(end['follower_zero_confirmed'])
        self.assertTrue(end['auto_start_on_plan'])
        self.assertTrue(end['auto_start_on_goal'])
        self.assertTrue(any(e[0]=='arm' for e in f.events))
        self.assertGreaterEqual(next(e[1] for e in f.events if e[0]=='arm'),5.)
        self.assertTrue(any(e[0]=='disarm' for e in f.events))
        controls=[json.loads(e[1].data) for e in f.events
                  if e[0]=='status']
        self.assertTrue(any(c.get('follower_follow_reason')=='STOP_LEADER_STOPPED'
                            and c.get('reason')=='FOLLOW_WAITING_LEADER'
                            for c in controls))

    def test_prepare_only_with_disabled_driver_never_arms_or_sends_motion(self):
        f,code,rows=self.auto_flow(prepare_only=True)
        self.assertEqual(code,0)
        self.assertEqual(rows[-1]['reason'],'FOLLOW_INPUTS_READY_NO_ARM')
        self.assertFalse(rows[-1]['arm_requested'])
        self.assertFalse(any(e[0] in ('arm','leader','envelope') for e in f.events))

    def test_rotation_wait_does_not_end_nav2_session(self):
        f,code,rows=self.auto_flow(leader_rotation=True)
        self.assertEqual(rows[-1]['reason'],'NAV2_GOAL_FINISHED')
        controls=[json.loads(e[1].data) for e in f.events if e[0]=='status']
        waiting=[c for c in controls if c.get('reason')=='FOLLOW_WAITING_LEADER_ROTATION']
        self.assertTrue(waiting)
        self.assertTrue(all(c['v']==0. and c['w']==0. for c in waiting))
        self.assertEqual(sum(e[0]=='arm' for e in f.events),1)
        self.assertEqual(sum(e[0]=='disarm' for e in f.events),1)

    def test_reverse_wait_does_not_disarm_or_end_nav2_session(self):
        f,code,rows=self.auto_flow(leader_reverse=True)
        self.assertEqual(rows[-1]['reason'],'NAV2_GOAL_FINISHED')
        controls=[json.loads(e[1].data) for e in f.events if e[0]=='status']
        waiting=[c for c in controls if c.get('reason')=='FOLLOW_WAITING_LEADER_REVERSE']
        self.assertTrue(waiting)
        self.assertTrue(all(c['v']==0. and c['w']==0. for c in waiting))
        self.assertEqual(sum(e[0]=='arm' for e in f.events),1)
        self.assertEqual(sum(e[0]=='disarm' for e in f.events),1)

    def test_nav2_follow_ignores_legacy_point9m_obstacle_trial_trigger(self):
        f,code,rows=self.auto_flow(front_obstacle_x=.95)
        self.assertEqual(rows[-1]['reason'],'NAV2_GOAL_FINISHED')
        self.assertIsNone(rows[-1]['obstacle_front_trigger_m'])

    def test_nav2_follow_retains_actual_clearance_stop(self):
        f,code,rows=self.auto_flow(front_obstacle_x=.50)
        self.assertEqual(rows[-1]['reason'],'STOP_FOLLOW_CLEARANCE')
        self.assertTrue(rows[-1]['leader_zero_confirmed'])
        self.assertTrue(rows[-1]['follower_zero_confirmed'])

    def test_nav2_follow_explicit_distance_stop_disable_keeps_session(self):
        f,code,rows=self.auto_flow(front_obstacle_x=.5,disable_clearance_stop=True)
        self.assertEqual(rows[-1]['reason'],'NAV2_GOAL_FINISHED')
        self.assertFalse(rows[-1]['clearance_stop_enabled'])
        self.assertIsNone(rows[-1]['stop_clearance_m'])

    def test_nav2_map_follow_does_not_read_legacy_filtered_cache(self):
        f,code,rows=self.auto_flow(missing_filtered=True)
        self.assertTrue(any(e[0]=='filtered_rejected_sample' for e in f.events))
        self.assertEqual(rows[-1]['reason'],'NAV2_GOAL_FINISHED')
        self.assertFalse(any(r.get('event')=='exception' for r in rows))

    def test_remote_reference_loss_holds_zero_then_resumes_same_session(self):
        f,code,rows=self.auto_flow(reference_loss=True)
        self.assertEqual(rows[-1]['reason'],'NAV2_GOAL_FINISHED')
        waiting=[r for r in rows if r['event']=='follow_reference_wait']
        resumed=[r for r in rows if r['event']=='follow_reference_resumed']
        self.assertTrue(waiting)
        self.assertEqual(len(resumed),1)
        first=waiting[0]['ros_ns'];last=resumed[0]['ros_ns']
        commands=[e[1] for e in f.events if e[0]=='envelope' and first<=e[1]['stamp_ns']<last]
        self.assertTrue(commands)
        self.assertTrue(all(c['motion']=='STOP' and c['v']==c['w']==0. for c in commands))
        self.assertTrue(any(e[0]=='envelope' and e[1]['motion']=='DRIVE'
                            and e[1]['stamp_ns']<first for e in f.events))
        self.assertTrue(any(e[0]=='envelope' and e[1]['motion']=='DRIVE'
                            and e[1]['stamp_ns']>=last for e in f.events))
        self.assertTrue(all(0<=(c['stamp_ns']-c['source_stamp_ns'])/1e9<=.8
                            for c in commands))
        self.assertEqual(sum(e[0]=='arm' for e in f.events),1)
        self.assertEqual(sum(e[0]=='disarm' for e in f.events),1)

    def test_persistent_remote_loss_exits_at_bounded_wait_timeout(self):
        f,code,rows=self.auto_flow(reference_loss=True,persistent_reference_loss=True)
        self.assertEqual(rows[-1]['reason'],'FOLLOW_REFERENCE_RECOVERY_TIMEOUT')
        self.assertFalse(any(r['event']=='follow_reference_resumed' for r in rows))
        self.assertTrue(rows[-1]['follower_zero_confirmed'])

    def test_goal_terminal_during_wait_does_not_resume(self):
        f,code,rows=self.auto_flow(reference_loss=True,goal_end_during_wait=True)
        self.assertEqual(rows[-1]['reason'],'NAV2_GOAL_FINISHED')
        self.assertFalse(any(r['event']=='follow_reference_resumed' for r in rows))

    def test_motor_fault_during_wait_still_terminates(self):
        f,code,rows=self.auto_flow(reference_loss=True,persistent_reference_loss=True,
                                  motor_fault_during_wait=True)
        self.assertEqual(rows[-1]['reason'],'MOTOR_FAULT_OR_FEEDBACK_STALE')
        self.assertFalse(any(r['event']=='follow_reference_resumed' for r in rows))

if __name__=='__main__':unittest.main()

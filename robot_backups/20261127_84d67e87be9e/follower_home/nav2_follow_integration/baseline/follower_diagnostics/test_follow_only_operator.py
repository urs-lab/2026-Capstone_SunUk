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

    def auto_flow(self):
        class Nav2Flow(Flow):
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
        class Nav(Base):
            def __init__(self,lookahead=.8):
                super().__init__();self.path_tracker=DirectedPathTracker();self.lookahead=lookahead
                self.heading_limit=math.pi/4;self.last_selection=None
            def path_callback(self,m,ns,rx,key='path',persistent=False):
                if not m.poses:self.inputs.pop(key,None);return
                return super().path_callback(m,ns,rx)
            def command(self,ns,mono):
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
        self.assertTrue(any(e[0]=='disarm' for e in f.events))
        controls=[json.loads(e[1].data) for e in f.events
                  if e[0]=='status']
        self.assertTrue(any(c.get('follower_follow_reason')=='STOP_LEADER_STOPPED'
                            and c.get('reason')=='FOLLOW_WAITING_LEADER'
                            for c in controls))

if __name__=='__main__':unittest.main()

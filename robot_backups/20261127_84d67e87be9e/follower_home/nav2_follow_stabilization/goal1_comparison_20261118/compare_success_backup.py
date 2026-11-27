"""Audit every goal1 backup file and replay the actual time guard offline."""
import ast
import collections
import hashlib
import importlib.util
import json
from pathlib import Path
import sys

ROOT=Path(__file__).resolve().parent
BACKUP=Path('/home/sunuk/robot_github_backup_20261113/staging/capstone_goal1')
LIVE=Path('/home/sunuk')
REMOTE_COMMIT='b50321217157ebafbc302c7d028ddb7c0bb24591'
REMOTE_TREE='e1864296496d98ce091ac3fc37640b58c91ffb9f'


def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def dump(name,data):(ROOT/name).write_text(json.dumps(data,ensure_ascii=False,indent=2)+'\n')


def git_object(kind,data):return hashlib.sha1(kind.encode()+b' '+str(len(data)).encode()+b'\0'+data).digest()
def git_tree(folder):
    result=[]
    for p in folder.iterdir():
        if p.is_dir():mode=b'40000';value=git_tree(p);key=p.name.encode()+b'/'
        else:mode=b'100755' if p.stat().st_mode&0o111 else b'100644';value=git_object('blob',p.read_bytes());key=p.name.encode()+b'\0'
        result.append((key,mode+b' '+p.name.encode()+b'\0'+value))
    return git_object('tree',b''.join(v for _,v in sorted(result)))


def load(path,name):
    spec=importlib.util.spec_from_file_location(name,path)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    return module


def main():
    local_tree=git_tree(BACKUP).hex()
    assert local_tree==REMOTE_TREE,(local_tree,REMOTE_TREE)
    rows=[]
    for p in sorted(BACKUP.rglob('*')):
        if not p.is_file():continue
        rel=p.relative_to(BACKUP);entry=dict(path=str(rel),backup_sha256=sha(p),bytes=p.stat().st_size)
        if rel.parts[0]=='home':
            q=LIVE/Path(*rel.parts[1:]);entry['current_path']=str(q)
            if q.is_file():entry.update(status='IDENTICAL' if q.read_bytes()==p.read_bytes() else 'CHANGED',current_sha256=sha(q))
            else:entry['status']='NOT_PRESENT_LOCALLY'
        elif rel.parts[0]=='leader_home':
            entry['status']='LEADER_CURRENT_NOT_READ'
        else:entry['status']='REPOSITORY_METADATA'
        rows.append(entry)
    dump('all_backup_files.json',dict(remote_commit=REMOTE_COMMIT,remote_tree=REMOTE_TREE,
        locally_reconstructed_tree=local_tree,all_backup_bytes_match_remote=True,
        counts=dict(collections.Counter(x['status'] for x in rows)),files=rows))
    profile='nav2_follow_stabilization/smooth010/candidate/follower_diagnostics'
    src=BACKUP/'home'/profile;current=LIVE/profile
    core=[]
    for p in sorted(src.iterdir()):
        if p.is_file():
            q=current/p.name;core.append(dict(file=p.name,identical=q.is_file() and p.read_bytes()==q.read_bytes()))
    dump('goal1_profile_files.json',dict(files=len(core),different=[x for x in core if not x['identical']],rows=core))
    new=LIVE/'nav2_follow_stabilization/goal1_transport_20261118/driver'
    driver=[]
    for p in sorted(src.iterdir()):
        if p.is_file():
            q=new/p.name
            driver.append(dict(file=p.name,identical=q.is_file() and p.read_bytes()==q.read_bytes()))
    dump('active_driver_vs_success.json',dict(files=len(driver),different=[x for x in driver if not x['identical']],rows=driver))
    sys.path.insert(0,str(current))
    original=load(src/'nav2_follow_safety.py','success_goal1_safety')
    now=load(current/'nav2_follow_safety.py','current_goal1_safety')
    old_recovery=load(src/'follow_reference_recovery.py','success_reference_recovery')
    recovery=load(current/'follow_reference_recovery.py','current_reference_recovery')
    cases=[];measured_pose=1794989624079753805;exception_ns=1794989624636099884
    for delta_ns in (216624195,319567195,390000000,400000000,400000001,410000000,500000000):
        scan=measured_pose+delta_ns;outcomes=[]
        for mod in (original,now):
            try:
                s=mod.build_scene(pose_row=(measured_pose,10.,(0.,0.,0.,0.,0.)),
                    scan_row=(scan,10.,((2.,0.),)),now_ns=exception_ns,now_mono=10.1)
                outcomes.append(dict(reason='SCENE_READY',delta_s=s['pose_scan_delta_s']))
            except ValueError as exc:outcomes.append(dict(reason=str(exc)))
        assert outcomes[0]==outcomes[1],outcomes
        cases.append(dict(pose_scan_delta_ns=delta_ns,pose_age_s=(exception_ns-measured_pose)/1e9,
                          scan_age_s=(exception_ns-scan)/1e9,success_backup=outcomes[0],current=outcomes[1]))
    assert cases[3]['current']['reason']=='SCENE_READY'
    assert cases[4]['current']['reason']=='FOLLOW_POSE_SCAN_TIME_MISMATCH'
    recovery_outcomes=[mod.recoverable_reference_loss('FOLLOW_POSE_SCAN_TIME_MISMATCH') for mod in (old_recovery,recovery)]
    assert recovery_outcomes==[False,False]
    dump('time_guard_replay.json',dict(no_ros_or_hardware=True,cases=cases,
        mismatch_is_recoverable_reference_loss=recovery_outcomes))
    log=current/'logs/local_avoidance_trial_1794989578439735516.jsonl'
    logs=[json.loads(x) for x in log.open()]
    end=next(x for x in reversed(logs) if x['event']=='end')
    keep=[]
    for x in logs:
        if x['event']=='local_control':
            keep.append({k:v for k,v in x.items() if k in ('event','ros_ns','reason','motion','source_stamp_ns','scan_stamp_ns','input_source_stamps','pose_scan_delta_s')})
    motor=[x for x in logs if x['event']=='motor']
    evidence=dict(log=str(log),log_sha256=sha(log),exception_ns=exception_ns,
        last_controls=keep[-4:],last_follower_map_trace=[x for x in logs if x['event']=='map_pose_trace' and x.get('robot')=='follower'][-4:],
        last_motor_states=[dict(ros_ns=x['ros_ns'],fault=x['status'].get('fault'),armed=x['status'].get('armed'),
                               telemetry_isolated=x['status'].get('telemetry_isolated'),valid_feedback=x['status'].get('valid_feedback')) for x in motor[-6:]],end=end)
    dump('latest_failure_evidence.json',evidence)
    # Show the actual catch policy and callback dispatch differences in a compact artifact.
    callback={}
    for label,p in (('success_goal1',src/'run_local_avoidance_trial.py'),('current_goal1',current/'run_local_avoidance_trial.py'),
                    ('success_goal3_current',LIVE/'nav2_follow_stabilization/goal3_success_tf02524/evaluation3_ekf_wait/candidate/follower_diagnostics/run_local_avoidance_trial.py')):
        tree=ast.parse(p.read_text())
        callback[label]=[dict(line=n.lineno,call=ast.unparse(n)) for n in ast.walk(tree)
            if isinstance(n,ast.Call) and isinstance(n.func,ast.Attribute) and n.func.attr=='spin_once']
    dump('callback_dispatch_comparison.json',callback)
    print(json.dumps(dict(remote_commit=REMOTE_COMMIT,remote_tree_verified=True,
        backup_file_counts=dict(collections.Counter(x['status'] for x in rows)),
        goal1_profile_files=len(core),goal1_profile_differences=[x['file'] for x in core if not x['identical']],
        active_driver_differences=[x['file'] for x in driver if not x['identical']],time_guard_scenarios_compared=len(cases)),ensure_ascii=False))


if __name__=='__main__':main()

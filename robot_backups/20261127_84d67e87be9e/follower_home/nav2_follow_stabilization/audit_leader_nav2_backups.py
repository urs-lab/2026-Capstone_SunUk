#!/usr/bin/env python3
"""Read-only leader Nav2 backup inventory. No ROS, settings or file writes."""
from pathlib import Path
from datetime import datetime
import hashlib,json,tarfile
home=Path.home();root=home/'leader_diagnostics';files=[];archives=[]
if not root.is_dir():raise SystemExit('leader_diagnostics directory missing')
for p in sorted(root.rglob('*')):
    if not p.is_file():continue
    name=p.name.lower()
    if ('nav2' in name and p.suffix in ('.yaml','.yml','.bash','.py')) or name=='leader_nav_yaw_calibrated.launch.py':
        data=p.read_bytes()
        files.append(dict(path=str(p),modified=datetime.fromtimestamp(p.stat().st_mtime).isoformat(timespec='seconds'),sha256=hashlib.sha256(data).hexdigest()))
    if name.endswith(('.tar.gz','.tgz','.tar')):
        try:
            with tarfile.open(p) as archive:
                entries=[m.name for m in archive if 'nav2' in m.name.lower() and m.name.lower().endswith(('.yaml','.yml','.bash','.launch.py'))]
            if entries:archives.append(dict(path=str(p),modified=datetime.fromtimestamp(p.stat().st_mtime).isoformat(timespec='seconds'),nav2_entries=entries))
        except (OSError,tarfile.TarError) as e:archives.append(dict(path=str(p),error=str(e)))
processes=[]
for proc in Path('/proc').iterdir():
    if not proc.name.isdigit():continue
    try:
        args=(proc/'cmdline').read_bytes().replace(b'\0',b' ').decode(errors='replace')
        if ('nav2_bringup' in args or 'start_leader_nav2' in args or 'navigation_launch.py' in args) and 'audit_leader_nav2' not in args:
            processes.append(dict(pid=int(proc.name),command=args))
    except (OSError,PermissionError):pass
print(json.dumps(dict(read_only=True,host=__import__('socket').gethostname(),files=files,archives=archives,nav2_processes=processes),ensure_ascii=False,indent=2))

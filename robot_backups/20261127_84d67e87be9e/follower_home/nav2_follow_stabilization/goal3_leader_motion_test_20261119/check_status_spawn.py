"""Fresh interpreter/helper/spawn check; no ROS, DDS, motors, or serial."""
from pathlib import Path
import os,sys,time
sys.path.insert(0,str(Path(__file__).resolve().parent/'candidate/follower_diagnostics'))
from operator_status_process import OperatorStatusPublisher
from isolated_telemetry import LatestSlot

def sink(path,parent_pid,*args):
    slot=LatestSlot(path)
    try:
        slot.mark(ready=True)
        while os.getppid()==parent_pid:
            meta=slot.metadata()
            if meta and meta['stop']:return
            packet=slot.snapshot(0)
            if packet:
                assert packet[1]==b'smoke-test'
                slot.mark(published=1)
                while os.getppid()==parent_pid:
                    meta=slot.metadata()
                    if meta and meta['stop']:return
                    time.sleep(.003)
            time.sleep(.003)
    finally:slot.close()

def main():
    pub=OperatorStatusPublisher(serialize=lambda x:x,target=sink)
    try:
        end=time.monotonic()+3
        while time.monotonic()<end:
            pub.publish(b'smoke-test')
            if pub.worker.stats().get('published')==1:
                print('ISOLATED_IMPORT_AND_REAL_SPAWN_OK');return
            time.sleep(.005)
        raise RuntimeError('NO_CHILD_RECEIPT')
    finally:pub.close()
if __name__=='__main__':main()

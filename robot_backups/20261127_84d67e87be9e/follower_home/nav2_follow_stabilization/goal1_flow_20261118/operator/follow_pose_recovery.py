"""Zero-only support for a delayed map pose; never makes a motion scene."""


def recoverable_pose_delay(reason,pose_row,now_ns,mono,rejection=None):
    if reason not in {'FOLLOW_POSE_SCAN_TIME_MISMATCH','FOLLOW_POSE_STALE',
                      'FOLLOW_POSE_RECEIVE_STALE','FOLLOW_MAP_POSE_MISSING'}:
        return False
    if pose_row is None:
        return (reason=='FOLLOW_MAP_POSE_MISSING' and isinstance(rejection,dict)
                and rejection.get('key')=='follower' and rejection.get('reason')=='INVALID_SOURCE_TIME'
                and type(rejection.get('source_ns')) is int and 0<rejection['source_ns']<=now_ns)
    stamp=pose_row[0].sec*10**9+pose_row[0].nanosec
    return stamp>0 and now_ns>=stamp and mono>=pose_row[1]


def zero_support(core,now_ns,mono,timeout_s=.8):
    stamps={}
    for key,row in (('raw',core.inputs.get('raw')),('scan',core.scan)):
        if row is None:raise ValueError('FOLLOW_ZERO_'+key.upper()+'_MISSING')
        stamp,received=row[:2]
        if (type(stamp) is not int or stamp<=0 or not 0<=(now_ns-stamp)/1e9<=timeout_s
                or not 0<=mono-received<=timeout_s):
            raise ValueError('FOLLOW_ZERO_'+key.upper()+'_STALE')
        stamps[key]=stamp
    return dict(source=min(stamps.values()),scan_stamp=stamps['scan'],source_stamps=stamps)

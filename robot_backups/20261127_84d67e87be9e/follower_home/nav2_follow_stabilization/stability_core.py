"""Read-only readiness policy. No ROS, ports, commands or automatic recovery."""
import math


def source_time_issue(stamp, now_ns, max_age=.3, future_allowance=.05):
    if type(stamp) is not int or stamp <= 0:
        return 'MISSING_SOURCE_STAMP'
    age = (now_ns-stamp)/1e9
    if age < -future_allowance:
        return 'SOURCE_IN_FUTURE'
    if age > max_age:
        return 'SOURCE_STALE'
    return None


class Readiness:
    def __init__(self, required, stable_seconds=5.):
        self.required = dict(required)  # name -> (source age, future allowance)
        self.stable_seconds = stable_seconds
        self.samples = {}
        self.stable_since = None
        self.last_evaluation = None

    def observe(self, name, stamp_ns, receive_mono, valid=True):
        previous = self.samples.get(name)
        backwards = previous is not None and stamp_ns < previous[0]
        self.samples[name] = (stamp_ns, receive_mono, valid, backwards)

    def evaluate(self, now_ns, mono):
        issues = []
        if self.last_evaluation is not None and (mono < self.last_evaluation or mono-self.last_evaluation > .3):
            issues.append('OBSERVER_SCHEDULING_GAP')
        self.last_evaluation = mono
        for name, (age, future) in self.required.items():
            row = self.samples.get(name)
            if row is None:
                issues.append(name+':MISSING');continue
            stamp, rx, valid, backwards = row
            if not valid:
                issues.append(name+':INVALID')
            if backwards:
                issues.append(name+':SOURCE_BACKWARDS')
            reason = source_time_issue(stamp, now_ns, age, future)
            if reason:
                issues.append(name+':'+reason)
            if not math.isfinite(rx) or not 0 <= mono-rx <= max(.3, age):
                issues.append(name+':RECEIVE_STALE')
        if issues:
            self.stable_since = None
        elif self.stable_since is None:
            self.stable_since = mono
        duration = 0. if self.stable_since is None else mono-self.stable_since
        return dict(ready=not issues and duration >= self.stable_seconds,
                    stable_seconds=duration, issues=issues)

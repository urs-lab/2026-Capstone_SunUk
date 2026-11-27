"""Read-only readiness decision; no state transitions or guessed TF."""


def ready(states,manager,mono):
    return (all(key in states and states[key][0]==3 and 0<=mono-states[key][1]<=3.
                for key in ('map_server','amcl'))
            and manager is not None and manager[0] is True and 0<=mono-manager[1]<=3.)


def odom_ready(row,now_ns,mono):
    return row is not None and 0<=(now_ns-row[0])/1e9<=.8 and 0<=mono-row[1]<=.8

# Reproduced convoy speed-gate interaction

## Established software issue

Using the actual `PreviewCore.speed_gate`, `Nav2ConvoyGateCore`, and an ideal
leader speed measurement equal to the preceding gate output, hold the robot
spacing fixed at 1.51 m to isolate the interaction:

1. Distance-only follower speed is `0.5 * (1.51 - 1.50) = 0.005 m/s`.
2. Leader gate limits forward speed to follower request + 0.003 = 0.008 m/s.
3. The follower's leader-stop threshold is 0.01 m/s. It consequently waits.
4. The gate's startup cap returns to 0.06 m/s. The cycle repeats.

The deterministic example toggles follower drive/stop 19 times in 20 updates,
without network jitter or hardware faults. At fixed spacing 1.70 m, the same
cores remain moving after startup. This establishes a software interaction;
it does not reproduce physical motion, prove encoder-fault causation, or
exclude any hardware/driver issue.

## Saved-log comparison

In the prior approximately 114 s run, nonzero logged command median was
0.04 m/s and no logged motor samples requested 1–2 RPM. In the incident run,
nonzero command median was approximately 0.0137 m/s and small wheel commands
were present. The increased 0.06 m/s cap was never reached in that incident.

## Explicit candidate, disabled by default

The candidate operator accepts `--follow-min-linear 0.04`. Its default remains
0.0, preserving the prior behavior. It changes only positive TRACKED_PURSUIT
commands, preserves curvature when raising speed, and retains wheel RPM bounds.
It does not override wait/fault/zero commands or an enabled obstacle stop.
When enabled, the gate-facing follower speed is the final bounded command.

This removes the fixed-gap counterexample, but a fixed-gap example and replay
of recorded trajectories are not physical closed-loop validation. A minimum
body speed does not guarantee a minimum inner-wheel speed in sharp curves.
Changing spacing, delay, braking, driver internal behavior and encoder health
remain unverified. No robot runtime was launched or live parameter changed.

Validation: 87 relevant runtime regressions plus 8 offline policy/interaction
tests passed. Reports: `minimum_speed_candidate_regression.log`,
`low_speed_policy_tests.log`, `convoy_speed_interaction_counterexample.json`,
`low_speed_open_loop_comparison.json`.

The M2 unintended-motion incident remains open. Do not use this candidate as
grounds to clear an encoder alarm or resume ground trials. Retain motor
protection and independently establish physical stopping and sensor/driver
behavior before any further motion.

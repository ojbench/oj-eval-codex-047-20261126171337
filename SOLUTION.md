# Robot controller

`src.hpp` implements only the permitted controller extension points. It uses
each robot's own target and speed limit, plus observable positions, radii and
previous velocities. There are no global/static variables or shared-area calls.

The preferred velocity points toward the target, capped to avoid overshooting.
Tangential steering selects a consistent passing side around a blocking disk.
Reciprocal velocity-obstacle half-planes anticipate collisions over two seconds.
Additional separating half-planes reserve half of each pair's current clearance
for each robot during the next step. These contain zero velocity and guarantee
that the separating axis remains valid throughout the step. If the anticipatory
constraints are infeasible, the controller retains the separating constraints.
An incremental two-dimensional linear program projects the preferred velocity
onto the constraints and the maximum-speed disk in O(N^2) time per robot.

Persistent per-robot progress tracking detects local deadlocks. A temporary
outward detour creates space before returning to the target; its duration scales
with robot size and speed. This is a heuristic planner, not a proof of eventual
arrival in every possible arrangement.

## Local validation

Run `bash tests/run_samples.sh`. It compiles against an unchanged copy of the
provided simulator in a temporary directory and checks that every supplied case
finishes without collision or speeding warnings.

All eight supplied cases finish. Their scores range from 90.431 to 100 out of
100. No provided framework source is modified.

# cornering-opt

Single-level minimum-time cornering optimization with a three-point parameterized racing line.

## Core idea
- Path variables: turn-in, apex, exit-opening
- Speed variables: discretized squared speeds q = v^2
- Objective: minimize corner traversal time
- Constraints: ordering, track width, friction, acceleration, braking, entry speed
- Solver: interior-point style constrained optimization (scipy.optimize.trust-constr)

## Suggested workflow
1. Start with a toy corner and verify the geometry pipeline.
2. Verify curvature and lateral-offset calculations.
3. Run the single-level optimizer on the toy case.
4. Replace toy geometry with real track data.
5. Add plots and experiments over different entry speeds.

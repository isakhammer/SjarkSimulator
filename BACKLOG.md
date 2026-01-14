# Backlog (Nautomatic)

Priorities and work items for the simulation + control stack.

## P0: 3-DOF monohull sim + steerable stern rotor (MVP)

Goal: replace the current sim with a monohull 3-DOF model and a single stern rotor/propulsor that can steer over a 180° sweep (assume -90°..+90° about the hull x-axis).

- Define the 3-DOF state, frames, and sign conventions (NED/ENU, body axes, yaw sign).
- Define actuator interface: thrust magnitude and rotor angle (plus optional rate limits / dynamics).
- Implement the model and actuator in `src/na_sim/na_sim`.
- Keep topic compatibility where possible (or provide a compatibility node).
- Add fast unit tests: dynamics sanity, actuator saturation, regression for a known maneuver.

Acceptance checks
- With a constant forward thrust and rotor angle 0°, surge speed increases and yaw stays near 0.
- With non-zero rotor angle, the craft generates yaw in the expected direction.
- Rotor limits and (if present) rate limits are enforced.

## P0: LOS controller updated for steerable rotor

Goal: guidance/control reflects thrust-vectoring (rotor angle) rather than only differential thrust.

- Define controller output(s): either publish `(thrust, rotor_angle)` or map to `/cmd_thrust` in a compatibility layer.
- Update LOS logic to compute desired heading/track and convert to rotor angle + thrust (respect limits).
- Add controller-side saturation/rate limits so the sim and controller agree on what is achievable.

Acceptance checks
- In sim, `vx > 0` within a few seconds and the craft follows the published path (bounded cte).
- Controller publishes valid commands continuously (finite values, within limits).

## P1: Path regression tests (quick, CI-friendly)

Goal: validate the full pipeline across multiple path types.

- Add planner options for `CIRCLE`, `SQUARE`, `STRAIGHT` (and keep defaults).
- Add a short “path sweep” CI test mode (no viz) that runs each path type for ~5–10 s.
- Record simple metrics: max/mean cross-track error, heading error, forward speed.

Acceptance checks
- Each scenario produces a valid path, sim state, controller command, and observed motion.
- Metrics remain below defined thresholds (tunable per scenario).

## P1: Launch flags for visualization (rviz/plotjuggler)

Goal: make `lsc` usable headless and interactive by toggling visualization nodes.

- Add launch arguments to `na_launch/sim_controller_launch.py`:
  - `rviz:=true|false`
  - `plotjuggler:=true|false`
- Support combinations: both, either one, or none (default should be none for CI/headless).
- Update `common_scripts.sh` aliases (or add new ones) to pass flags conveniently.

## P2: Upgrade to Thor Fossen 6-DOF model (extension)

Goal: extend the MVP to a 6-DOF Fossen-style marine craft model while reusing the same actuator abstraction.

- Choose the 6-DOF formulation, parameters, and frames; document assumptions.
- Implement and validate against known maneuvers / sanity cases.
- Keep the controller interface stable so LOS works on both 3-DOF and 6-DOF.

Acceptance checks
- LOS + rotor works on both 3-DOF and 6-DOF backends on the regression scenarios.

## Notes / Decisions Needed

- Rotor angle convention: 0° = forward along +x, + = port/CCW? (pick and document).
- Actuator command representation: absolute rotor angle vs relative to hull heading.
- Whether to keep `/cmd_thrust` as the primary command topic long-term or migrate to a dedicated message.

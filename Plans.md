# Agent Plans

This document is a lightweight planning ledger for work done in this repository. Use it to track goals, execution steps, and decisions for ongoing tasks so contributors can quickly understand what is in flight and why.

## When to Use

- Start a new plan when a task spans multiple files or requires coordination across packages.
- Update the plan after completing a meaningful step or when scope changes.
- Close a plan once the task is merged or abandoned.

## Plan Format

Use the template below for each new plan. Keep steps small and verifiable, and mark status as `pending`, `in_progress`, or `done`.

```
### Plan: <short title>

Owner: <name or handle>
Target: <branch / issue / milestone>
Status: <active | paused | closed>

Goals
- <what success looks like in one or two bullets>

Steps
- [pending] <step 1>
- [pending] <step 2>
- [pending] <step 3>

Notes
- <risks, constraints, or dependencies>

Decisions
- <decision and rationale>
```

## Current Focus

- Plan: Monohull 3-DOF sim with steerable rotor (see below).
- Plan: Control system (LOS with steerable rotor) (see below).
- Plan: Monohull 6-DOF Fossen craft model (later) (see below).
- Plan: State estimation system (later phase) (see below).

### Plan: Monohull 3-DOF sim with steerable rotor

Owner: isak
Target: TBD
Status: active

Goals
- Replace the current sim with a monohull 3-DOF model (surge/sway/yaw).
- Model a single stern rotor/propulsor with 180° steering sweep (assume -90° to +90°).
- Expose actuator limits and dynamics so control can respect them.

Steps
- [pending] Define 3-DOF equations, frames, and sign conventions.
- [pending] Define actuator interface: thrust magnitude + rotor angle (with rate/limits).
- [pending] Implement the new model in `src/na_sim/na_sim` and keep topics stable where possible.
- [pending] Add fast unit tests for dynamics sanity (stability, limits, basic responses).
- [pending] Add/extend launch smoke test that confirms sim + planner + controller run and motion occurs.

Notes
- Confirm angle convention for the stern rotor (0 = forward, + = port/CCW?) and document it.
- Decide if the rotor command is absolute angle or relative to hull heading.

Decisions
- Start with a minimal 3-DOF monohull model, then extend to Fossen 6-DOF later.

### Plan: Control system (LOS with steerable rotor, later NMPC with Acados)

Owner: isak
Target: TBD
Status: active

Goals
- Implement LOS guidance for a steerable stern rotor (thrust vectoring).
- Prepare the model and interfaces for an NMPC variant using Acados.

Steps
- [pending] Define control inputs/outputs and tuning parameters for LOS + rotor (angle + thrust).
- [pending] Update the controller to publish rotor angle and thrust magnitude (or left/right mapping).
- [pending] Validate LOS on multiple path types (circle, square, straight) with basic tracking metrics.
- [pending] Identify NMPC state/control constraints and required model exports.
- [pending] Prototype an Acados setup once the model stabilizes.

Notes
- Keep controller output backwards-compatible if possible (for example still publishing `/cmd_thrust`).

Decisions
- Start with LOS for a baseline, then iterate toward NMPC.

### Plan: Monohull 6-DOF Fossen craft model

Owner: isak
Target: TBD
Status: paused

Goals
- Extend the 3-DOF monohull model to a Thor Fossen-style 6-DOF model.
- Reuse the same steerable stern rotor actuator model and limits.
- Document assumptions, frames, and parameters so the team can review the model.

Steps
- [pending] Map 3-DOF parameters/frames to the 6-DOF formulation.
- [pending] Implement 6-DOF equations and validate against known maneuvers.
- [pending] Update controller interfaces/tests to support both models.

Notes
- Keep a clean abstraction boundary so the controller can run against 3-DOF and 6-DOF.

Decisions
- Defer until the 3-DOF + rotor control loop is stable.

### Plan: State estimation system (later phase)

Owner: isak
Target: TBD
Status: paused

Goals
- Define a state estimation stack that complements the 6-DOF model and control loops.
- Identify sensor inputs and required filters (for example, EKF/UKF).

Steps
- [pending] Specify required state outputs and expected sensor topics.
- [pending] Choose estimation approach and reference implementation.
- [pending] Integrate estimator into the simulation and launch files.
- [pending] Validate estimation accuracy against simulated ground truth.

Notes
- Planned for later in the project, after the model and control baseline stabilize.

Decisions
- TBD (defer until model/control architecture is finalized).

## Backlog

- Implement a steerable stern rotor command message/topic (angle + thrust) and keep `/cmd_thrust` as a compatibility layer if needed.
- Add controller-side saturation and rate limits for rotor angle and thrust.
- Add path regression scenarios (circle/square/straight) with basic metrics: average cte, heading error, and steady forward speed.
- Add a "path sweep" test mode that runs multiple path types in CI (short duration, no viz) and reports pass/fail.
- Set up a `colcon test`-compatible unit testing workflow; current `pytest` flow does not integrate with `colcon build`.
- Add a short smoke test that launches the system for 5-10 seconds and ensures core nodes do not crash.
- Add launch tests (for example, `launch_testing`) to exercise core launch files.
- Add a general launch that starts the core stack and allows toggling visualization on/off via a flag (planner stays in the core set).
- Add a flag to choose the map/trajectory type (circle, straight line, or default/normal).
- Implement trajectory-type handling in the planner so map selection stays active.
- Fix the 90-degree B-spline projection issue.
  Notes
  - Potential fallback: when projection jumps, prefer global minimal-distance search instead of local Newton refinement.
  - Use vehicle heading to bias the search window (restrict candidate samples to headings within a threshold).
  - Add a continuity gate: reject t jumps beyond a threshold and fall back to closest sample in a bounded window.
  - Add a max-jump sanity check on projection t and clamp or hold when exceeded.

## Maintenance Rules

- Keep content concise; avoid dumping logs or large outputs.
- Prefer linking to paths (for example, `src/na_sim/na_sim/sim_node.py`) rather than duplicating code.
- Remove completed plans after they have been closed for a while to keep this file short.

# SjarkSimulator
Why make drones so cool when boats are even more useful!

## Documentation
- LaTeX overview (Planning / Control / Simulator): `latex/overview.tex`
- Build via Docker (recommended): `./build_overview_docker.sh`
- Build locally (requires full TeXLive + TikZ/PGF): `make -C latex pdf`

## Simulator integrators (6DOF)
The 6DOF simulator supports multiple attitude integrators and a small benchmark:

- Benchmark tool: `PYTHONPATH=src/sj_sim python3 src/sj_sim/tools/benchmark_integrators.py`
- Typical outcome: expmap on SO(3) preserves orthonormality best; RK4+normalize is accurate;
  Euler is fastest but drifts.

6DOF launch example:
- `ros2 launch sj_launch sim_controller_6dof_launch.py`
URDF is published via `robot_state_publisher` in the 6DOF launch.

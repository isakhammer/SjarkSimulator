# Nautomatic
Why make drones so cool when boats are even more useful!

## Documentation
- LaTeX overview (Planning / Control / Simulator): `latex/overview.tex`
- Build via Docker (recommended): `./build_overview_docker.sh`
- Build locally (requires full TeXLive + TikZ/PGF): `make -C latex pdf`

## Simulator integrators (6DOF)
The 6DOF simulator supports multiple attitude integrators and a small benchmark:

- Benchmark tool: `PYTHONPATH=src/na_sim python3 src/na_sim/tools/benchmark_integrators.py`
- Typical outcome: expmap on SO(3) preserves orthonormality best; RK4+normalize is accurate;
  Euler is fastest but drifts.

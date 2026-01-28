# SjarkSimulator

![Sjark](sjark.jpg)

I got curious about what people use for boat simulators in hobby robotics, but then I realized Gazebo and Isaac Sim are
kind of the everything engines. Great tools, but also GPU hungry (which I dont have money for).

I mostly care about the math and dynamics, so I built my own fast 6 DOF boat model with quaternions and a Fossen style marine
dynamics formulation. It includes approximations for hydrodynamic damping, Coriolis forces, gravity and buoyancy, and full attitude dynamics in yaw, pitch, and roll. It is a nice middle ground between “toy model” and “too much”. So make sure the time integration was stable in the spherical manifold, I also added exponential integrators to ensure stability and accuracy.

To test the simulator I also added a basic Line-Of-Sight control system, which follows a custom made B Spline path.

While this small project is incredible fn It is still a wildly ambitious hobby project, so I use Codex to write all the tests and harder debugging so I can focus on the fun parts.


# Sjark
Just to have a reference, I will use a basic small wooden fishing boat from the 1970s (also called Sjark in Norwegian).

After searching a bit around these are typical specifications:

### Known specs
- Length overall: 10 m
- Beam: 4 m
- Hull depth: 5m
- Mass with full cargo, estimate: 16.5 ton
- Reported operating speed: 7 kn
- Propulsion note: basic rotating propellar


### Inertia estimate

Using a rectangular box as a model we can simply write a CG-centered bodyframe intertia setup. Using a box estimate with length `L`, beam `B`,
depth `T` we get this:

- `Ixx = (1/12) * m * (B^2 + T^2) ≈ 5.64e4 kg m^2`
- `Iyy = (1/12) * m * (L^2 + T^2) ≈ 1.72e5 kg m^2`
- `Izz = (1/12) * m * (L^2 + B^2) ≈ 1.60e5 kg m^2`


# Equations of Motion

Derived from first principles using the Lagrangian method.

## Cart equation (horizontal motion)
(M+m)ẍ - mlθ̈cosθ + mlθ̇²sinθ = F(t)

## Pendulum equation (rotational motion)
l²θ̈ - ẍcosθ - gsinθ - bθ̇ + u = 0

Where:
- M = cart/wheel mass (kg)
- m = pendulum body mass (kg)
- l = distance from pivot to centre of mass (m)
- θ = angle from vertical (rad)
- F(t) = applied motor force (N)
- b = viscous damping coefficient (N.m.s/rad)
- u = control torque input (N.m)

These are the full nonlinear coupled equations.
The simulation uses a simplified single-body approximation as a 
first step. Full coupled equations will be implemented in Stage 1.

Full working derivation available on request.
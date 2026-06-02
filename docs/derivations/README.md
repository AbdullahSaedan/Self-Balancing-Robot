*Note: mathematical notation in this file uses Unicode characters 
which may not render clearly in all editors. Refer to the 
derivation photos for the clearest representation of the equations.*

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
first step. Full coupled equations are implemented in Stage 1.

Full working derivation available on request.

## Explicit form for simulation

The coupled equations above were solved simultaneously to eliminate the coupling terms and giving explicit expressions for both ẍ and θ̈ .

A shared denominator δ appears naturally from the algebra:

δ = (M+m)(I+mL²) − (mLcosθ)²

This represents the coupled inertia of the system. It varies with θ because the mechanical coupling between the cart and the pendulum changes with the angle.

ẍ = [(I+mL²)(F + mLθ̇²sinθ) + mLcosθ(mgLsinθ + bθ̇ − u)] / δ

θ̈ = [mLcosθ(F + mLθ̇²sinθ) + (M+m)(mgLsinθ + bθ̇ − u)] / δ

Where:
- δ = coupled inertia denominator
- I = moment of inertia (kg.m²)
- All other variables as defined above

Note: the full derivation separates cart force F and pendulum torque u as different inputs. In this implementation both are combined into a single control input u, as the motor drives both cart motion and pendulum correction through the same signal.
# Simulation Plots with Different Parameters

## Nominal - Kp = 50, Ki = 1, Kd = 5

Starting from a 5 degree initial tilt, the controller should drive the angle back to zero and hold it there without clear overshoot or oscillation.

<img src="kp50_ki1_kd5_nominal.png" width="600">
<br>
The tilt angle converges to the setpoint within 0.5 seconds with no overshoot. The **P** term starts negative and converges to 0 since the controller output needs to push the system back towards the setpoint. The **D** term initially spikes since the rate of change of measurement is highest at the start, then dies away as the system slows down approaching the setpoint. The **I** term is negligible since there is not enough accumulation of error, indicating that **P** and **D** alone are sufficient for these conditions.
<br>

## Low Proportional Term - Kp = 5, Ki = 1, Kd = 5

Reducing Kp caused the correction to be a lot weaker. Because of this the controller struggles to reach exactly zero and settles with a persistent steady state error.

<img src="kp5_ki1_kd5_low_proportional.png" width="600">
<br>
The steady state error is almost 1 degree and the torque remains small throughout since **P** is small. **P** correction is proportional to error so as the error decreases the correction becomes weaker and gravity takes over, causing the system to settle before reaching the setpoint. The **I** term grows slowly after 2 seconds because the error is constant and accumulates over time. The **D** term is present at the start since measurements are changing, then fades away as the angle becomes constant, because the rate of change of measurement decreases exponentially eventually reaching zero.
<br>

## No Derivative Term - Kp = 50, Ki = 1, Kd = 0

With **Kd** set to 0 the braking force is removed. The system is expected to overshoot continuously and never settle.

<img src="kp50_ki1_kd0_no_derivative.png" width="600">
<br>

Since there is no derivative term there is nothing to dampen the overshoot, so the angle and torque oscillate continuously while accumulating error over time. The **P** term oscillates in phase with the tilt angle, continuously commanding corrective torque but never damping the motion due to the absence of the **D** term. This is why the **I** term grows steadily. The oscillation also grows over time, meaning on real hardware the robot would wobble with increasing amplitude until eventually becoming unstable.
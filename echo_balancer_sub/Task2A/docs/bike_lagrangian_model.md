# Corrected bike model: velocity input and front contact constraint

Status: corrected numerical A/B candidate was entered in the LQR Tuner on 15 Oct 2026. User reports immediate stabilization at default Q=I, R=1. Treat as strongly supported, pending a full evaluator-duration run.

## Coordinates

Use the official state order:

`q = [theta, psi]^T`, where `theta` is chassis lean about the wheelbase/front motor axis `+Y`, `psi` is yaw about world `+Z`. Let `phi` be the front motor joint angle relative to the chassis. The tuner sends the target relative joint velocity `u = phi_dot,target`.

Measured geometry gives wheelbase-like pivot-to-front-contact-center distance `L ≈ 0.200 m` and wheel radius `r ≈ 0.015 m`, so `k=L/r≈13.333`.

## Contact kinematics

At the front wheel bottom, yaw of the wheel center contributes lateral velocity `-L psi_dot`. The wheel's absolute spin about `+Y` is `theta_dot + phi_dot`; at the bottom point it contributes `-r(theta_dot+phi_dot)`. For ideal no-slip contact in that lateral direction:

`-L psi_dot - r(theta_dot + phi_dot) = 0`

Therefore:

`phi_dot = -theta_dot - k psi_dot`.

This is the important bike-specific mapping. The earlier draft omitted `theta_dot` and wrongly treated the motor joint speed as absolute wheel spin. Its numeric A/B must not be used.

## Motor velocity servo as generalized force

The scene's velocity controller is provisionally approximated near zero speed error by `tau = Kv(u - phi_dot)`, with `Kv≈0.1 N m/(rad/s)` and torque saturation at `0.5 N m`. Substituting the rolling relation:

`tau = Kv(u + theta_dot + k psi_dot)`.

The motor acts on the relative coordinate `phi`. Virtual work is `deltaW=tau delta_phi`; because `delta_phi=-delta_theta-k delta_psi`, the generalized motor force in coordinates `[theta,psi]` is:

`Qmotor = [-tau, -k tau]^T`.

Thus the reduced small-signal equations take the form:

`M q_ddot + G q + D q_dot = b u`

with

`D = Kv [[1,k],[k,k^2]]`,

`b = Kv [-1,-k]^T`.

The signs above assume the stated positive-axis choices and torque convention. Verify these against the tuner and a nonsaturated live response.

## Validation status and remaining checks

- The candidate A/B was entered in the LQR Tuner on 15 Oct 2026; the bike stabilized immediately with default `Q=I`, `R=1`.
- This supports the corrected contact constraint, target-velocity input mapping, and signs for the current scene.
- The full evaluator-duration run and robustness to larger disturbances are still to be recorded.
- The earlier passive capture is saturated in roughly 86% of samples, so it does not identify the unsaturated motor gain from data.

## Numerical candidate added after the corrected reduction

The existing geometry gives the pre-constraint matrix `M_cluster` by adding the body about-pivot inertia, wheel-center translation, and each dynamic sphere's rigid-body rotational inertia:

`M_cluster = [[0.0036549, -0.0026676], [-0.0026676, 0.0146332]] kg m^2`.

This baseline includes the front pair's `J_front_spin = 0.00054 kg m^2` in its lean-axis rigid-body inertia. Under the corrected constraint, their absolute motor-axis rate is `theta_dot + phi_dot = -k*psi_dot`; therefore replace their baseline lean-spin term, rather than simply adding another inertia:

`M = M_cluster + diag(-J_front_spin, J_front_spin*k^2)`

`M = [[0.0031149, -0.0026676], [-0.0026676, 0.1106332]] kg m^2`.

Using `G = diag(-m_body*g*h_body,0)` with `m_body=0.6 kg`, `h_body=0.03176 m`; `Kv=0.1 N m/(rad/s)`; `L=0.2 m`; `r=0.015 m`; and the servo/contact equations above gives this continuous-time candidate in official state order `[theta_dot, theta, psi_dot, psi]`:

```
A = [[ -43.3195,  61.2800, -577.5931, 0],
     [   1.0000,   0.0000,    0.0000, 0],
     [ -13.0964,   1.4776, -174.6182, 0],
     [   0.0000,   0.0000,    1.0000, 0]]

B = [[-43.3195],
     [  0.0000],
     [-13.0964],
     [  0.0000]]
```

These values are a reproducible model candidate and were entered in the tuner. The bike stabilized immediately at default `Q=I`, `R=1`. They assume ideal no-slip contact, the measured constrained wheel assembly, the configured velocity-servo P gain, and the stated axis mapping. Full evaluator-duration validation remains. The calculation is reproduced by `../scripts/derive_bike_lqr_model.py`.

## Live controller setting check (read-only)

A passive query of the running scene verified the front motor dynamic velocity PID settings: `P=0.1`, `I=0`, `D=0`. The reported joint max force remained `0.5 N m`. The scene also reports max velocity and acceleration as approximately `2*pi` in SI units; these are motion-profile limits and do not explain the tuner command's much larger observed target values in velocity-control mode. This verifies the configured proportional gain magnitude used by the candidate, but not the signed applied-torque convention or how the capped controller behaves while saturated.

## Official coordinate wording checked

The saved Task 2A instruction page states that the reference frame has the bike upright facing `+Y`; tilt is lean to either side, and yaw is turning about the rear wheel. This supports using the chassis longitudinal direction as the lean axis and world vertical as yaw axis. It still does not state the positive sensor signs explicitly, so the A/B signs remain conditional on the tuner's orientation convention and motor-sign setting.

## Live tuner result (15 Oct 2026)

The user entered the candidate A/B above in the Task 2A LQR Tuner and reports that the bike stabilized immediately with the default `Q=I`, `R=1`, without changing the weights. This is strong empirical support for the corrected model structure, input mapping, and sign convention in the current scene. It is not yet a full-duration/evaluator validation; record the complete run duration, peak tilt/yaw, and any eventual drift or saturation before calling it fully validated.



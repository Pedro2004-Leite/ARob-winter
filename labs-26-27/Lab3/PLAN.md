# Lab 3 (26/27) - preparation plan

Status: **plan updated 2026-11-16, 2026/27 guide and devkit received**.
Nothing is solved here on purpose.
This file is a roadmap so the theoretical questions can be attacked in a calm, ordered way, and reviewed together later.

## Source material

The 2026/27 guide (`26_27_UAVs_Lab3.pdf`) and devkit (`DevKit_Lab3/`) are now in this folder.
Checked on 2026-11-16: the guide is identical to the 2025/26 one (`labs-25-26/25_26_UAVs_Lab3.pdf`) except for the dates and some cross-references that show as "??" in the PDF text.
The devkit is identical to `labs-25-26/DevKit_Lab3/` (`diff -rq` shows no differences).
So the notes in `labs-25-26/notes/03-lab3.tex` apply as they are.

## What the lab is about

Trajectory tracking control of a quadrotor (outer-loop position control on top of the inner-loop attitude control).
The theory (T) part is Section 2 of the guide, questions 2.1 to 2.8.
The lab (L) part is Section 3, questions 3.1 to 3.7, and is **simulation and MATLAB code**, not data collection.
The 2025/26 guide does not ask for any real flight data, although the devkit contains a Wi-Fi model (`ARDroneTT.slx`).

## Theory questions: order, what each needs, where to look

Suggested order is the guide's order, because each question builds on the previous one.

| Q | Task in one line | Key idea to recall | Where to look |
|---|---|---|---|
| 2.1 | Change of coordinates to get a double-integrator-like model with virtual input u*, then write T, theta_r, phi_r as functions of u* | R = Rz Ry Rx, so Rz(-psi) can be moved to the other side; the thrust is the norm of the transformed vector | Ch2 (rigid-body dynamics) and Ch3 (quadrotor model) in `lectures-22-23/COURSE_NOTES.pdf` |
| 2.2 | Error system, input transformation u to u*, LQR gain minimizing J | Error state is position and velocity error, so A and B are the double-integrator pair; K = (1/rho) B'P | Ch4, LQR theorem |
| 2.3 | Show V1 = x'Px has derivative -x'Q*x with Q* > 0 | Substitute K into the Lyapunov derivative and use the Riccati equation; the resulting Q* is Q plus a positive semi-definite term | Ch4, Lyapunov and LQR proof |
| 2.4 | Add a disturbance w to the model, show the new closed-loop error dynamics | Plug the new law u* (with the w-hat term) into the model; the mismatch appears as B times w-tilde | Ch7 part II (adaptive control) |
| 2.5 | Propose an adaptation law for w-hat from V2 = V1 + (1/kw)|w-tilde|^2 | Choose the law that cancels the cross term in V2-dot; w is constant, so w-hat-dot equals minus w-tilde-dot | Ch7 part II, `lectures-22-23/notes/04-guidance-nonlinear.pdf` |
| 2.6 | Show the augmented origin is globally asymptotically stable | V2-dot is only negative **semi**-definite, so you need LaSalle or Barbalat, not plain Lyapunov | Ch7 (invariance principle) |
| 2.7 | Redraw the closed loop as Fig. 1, write K-bar(s), discuss the link to a PID | The adaptive state is an integrator of the position/velocity error, so the structure is PID-like | Ch3/Ch4 on PD plus integral action |
| 2.8 | Design a yaw controller with zero steady-state error for a first-order yaw model | Zero steady-state error to a constant reference needs integral action (internal model principle) | Ch3 |

Open points to settle while solving, not answered here:

- In 2.1, check the sign and frame conventions against what the course uses (z down or up) before writing the final expressions.
- In 2.2, decide the explicit form of the Q matrix block structure (position and velocity weights) so it can be reused in the lab.
- In 2.6, decide which of LaSalle or Barbalat is cleaner for this system, and write the argument in full.

## Lab questions: what can be prepared in advance

These can be prepared before the session without the drone.

| Q | Task | Preparation |
|---|---|---|
| 3.1 | Adapt the controller to the real inputs (phi_r, theta_r, w_r, psi-dot_r); vertical channel is w_r = -kw (z - zd) | Done on paper in 2.1 and 2.8, then coded in the `tt_controller` block |
| 3.2 | `desired_trajectory` for a horizontal line at 0.1 m/s, constant yaw, starting at the drone's current position | Write pd, pd-dot, pd-ddot as functions of time and the start position |
| 3.3 | Test and tune the gains | Compute LQR gains in MATLAB from the 2.2 result, then adjust Q and rho |
| 3.4, 3.5 | Circular trajectory, radius 1 m, 0.1 rad/s, constant yaw | pd, pd-dot, pd-ddot for a circle; check the acceleration magnitude |
| 3.6 | Yaw pointing at the circle centre; is that enough for zero tracking error? | The 2.8 remark that yaw is not constrained by the linear dynamics; if not enough, propose a controller change (likely the adaptive term from 2.5) |
| 3.7 | Line-of-sight straight-line following: psi_d = arctan(-(y - yd)/Delta), xd-dot = V cos(psi_d), yd-dot = V sin(psi_d) | Choose Delta and V; discuss their effect on convergence (see Ch8 on LOS guidance) |

Devkit starting point: `DevKit_Lab3/ARDroneSimulinkDevKit_V1.1/` (start with `start_here_CTRL.m`, option (4) Trajectory Tracking, models `ARDroneTTSim.slx` and `ARDroneTT.slx`).
The two empty MATLAB Function blocks to fill are `desired_trajectory` and `tt_controller`.
The 25/26 notes flag that the exact signal units (rad or normalized command) are not documented and must be checked inside the `.slx`.

## Suggested work sequence

1. Re-read the relevant lecture chapters listed above (Ch2, Ch3, Ch4, Ch7 part II, Ch8 LOS).
2. Solve 2.1 to 2.3 on paper (LQR chain), compare with the LQR derivation in the course notes.
3. Solve 2.4 to 2.6 on paper (adaptive chain), with the LaSalle/Barbalat step done carefully.
4. Solve 2.7 and 2.8.
5. Review the written solutions together (Pedro asked for this review later).
6. Devkit is already in `DevKit_Lab3/` and unchanged from 25/26 (done).
7. Write and test the two Simulink blocks in simulation (3.1 to 3.7).
8. The 2026/27 guide asks for no real flights, only simulation and MATLAB code (the Wi-Fi model `ARDroneTT.slx` exists but is not required).

## Report constraints (2026/27 guide, same as 25/26)

- Single-column PDF, at most 15 pages, with the MATLAB scripts.
- Cover page from the course website (`../Lab1/26_27_UAVs_Lab_report_cover_page.docx`, not yet copied here).
- Theoretical questions should be solved before the lab session, simulations done during the session.

## Notes for future self

- Do not edit the 25/26 files.
- Keep a solutions file like the Lab 1 one (`labs-26-27/Lab1/solutions/`), kept as a self-check, and extend that one file instead of creating new ones.

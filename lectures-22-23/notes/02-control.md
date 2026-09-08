# Control: Quadrotor Control Intro & Modern Control Design

Source files:
- 22_23_UAVs_Ch3_Quadrotor_Control_Intro.pdf (44 slides, "Quadrotor Trajectory Tracking Control - An Introduction")
- 22_23_UAVs_Ch4_Modern_Control_Design.pdf (43 slides, "Modern Control Design")

Course: UAVs, MEAer (Técnico Lisboa), Autumn Semester 2022/2023. Instructors: Rita Cunha / J.R. Azinheira.
Ch4's first part follows "Modern Control Design" lecture notes by Pedro Batista (Feb 2018); the LQR/pole-placement worked example is adapted from J. Hespanha, *Linear Systems Theory*, Princeton Univ. Press, 2009 (aircraft roll dynamics example).

## Chapter 3 - Quadrotor Control Introduction

### Starting point: the quadrotor dynamic model
From Ch1/Ch2 (rigid body + quadrotor modeling), the equations of motion used throughout are:

- Translational dynamics: `m*p_ddot = -T*r3 + m*g*e3`, with `r3 = R*e3` (third column of the rotation matrix, i.e. the body z-axis expressed in the inertial frame).
- Attitude kinematics: `R_dot = R*S(omega)` (S(.) = skew-symmetric operator, so `S(omega)*v = omega x v`).
- Rotational dynamics: `J*omega_dot = -S(omega)*J*omega + n`.

Inputs to the system are total thrust `T` (scalar) and torque `n` (3-vector). Outputs of interest are position/velocity `(p, p_dot)` and attitude/angular velocity `(R, omega)`.

Small-angle/near-hover exercise: linearizing `p_ddot` near hover with roll `phi` and pitch `theta` gives `p_ddot ≈ [-g*theta, g*phi, 0]^T` — this is the classical result that pitch controls x-acceleration and roll controls y-acceleration (sign convention as defined by the course's rotation order).

### Control objective and hierarchical (cascade) structure
Objective: **trajectory tracking**. Given a desired flat-output trajectory `(p*(t), psi*(t))` (position + yaw), find `(T, n)` such that `(p(t), psi(t)) -> (p*(t), psi*(t))` as `t -> infinity`.

Because of the block-diagram dependency structure (rotational dynamics feed into translational dynamics via `r3`), the natural design is **hierarchical/cascade control**:
- **Outer loop (position controller)**: takes `(p*, p_dot*, p_ddot*)` and `(p, p_dot)`, produces desired thrust `T` and a desired direction `r3d` for the body z-axis. Goal: track position.
- **Inner loop (attitude controller)**: takes the desired attitude (derived from `r3d` and desired yaw `psi*`) and current `(R, omega)`, produces torque `n`. Goal: track attitude. Runs faster (5-10x bandwidth) than the outer loop, which is the standard justification for treating the loops as (approximately) independent SISO problems.

Design recipe (4 steps):
1. **Neglect rotational dynamics**: replace `r3` by a desired `r3d`, define virtual input `u_T = -(T/m)*r3d`. Then `T = m*||u_T||` and `r3d = -u_T/||u_T||` (T is magnitude, r3d is direction).
2. **Design a control law for `u_T`** assuming `p_ddot = u_T + g*e3` (or `= u_T + g` in the slide's shorthand).
3. **Combine `r3d` with `psi*`** to get a full desired attitude `R_d` (or Euler-angle equivalent `lambda_d = (phi_d, theta_d, psi*)`). One explicit formula given: `r3d = Rz(psi*) Ry(theta_d) Rx(phi_d) e3`, leading to `[cos(phi_d)sin(theta_d), -sin(phi_d), cos(phi_d)cos(theta_d)]^T = Rz(-psi*) r3d` — i.e. invert this to solve for `(phi_d, theta_d)` given `r3d` and `psi*`.
4. **Design a control law for `n`** to track `R_d`/`lambda_d`, using either the rotation-matrix kinematics `R_dot = R*S(omega)` or the Euler-angle form `lambda_dot = Q(phi,theta)*omega` together with `J*omega_dot = -S(omega)*J*omega + n`.

### Position (translational) controller design
Model: `p_ddot = u_T + g` (per-axis double integrator with gravity offset), reference `(p*(t), p_dot*(t), p_ddot*(t))`.

Error variables: `e = p - p*`, `e_dot = p_dot - p_dot*`, giving `e_ddot = u_T + g - p_ddot*`.

**PD control law**: `u_T(t) = -kP*e(t) - kD*e_dot(t) - g + p_ddot*(t)`, which yields the clean closed-loop error dynamics `e_ddot(t) = -kP*e(t) - kD*e_dot(t)` (feedback term) `+ p_ddot*` (feedforward term, cancels out in the error equation). This is a linear 2nd-order system:
- Stable (asymptotically) iff `kP > 0` and `kD > 0` — `kP` acts like a spring, `kD` like a damper.
- Standard 2nd order form comparison: closed-loop transfer function `P(s)/P*(s) = (kP + kD*s) / (s^2 + kD*s + kP)`, compared against `G(s) = wn^2/(s^2 + 2*xi*wn*s + wn^2)`.
- Damping ratio: `xi = kD / (2*sqrt(kP))`. Increasing `kD` increases `xi` (less overshoot); increasing `kP` decreases `xi`. Settling time improves as `kD` increases (`xi*wn` increases).

### Classical SISO analysis tools used
- **Root locus**: rewrite `C(s) = kP + kD*s = k*(s+z)/z` with `z = kP/kD`, `k = kP`, open-loop `G(s) = k*(s+z)/(z*s^2)`. Increasing gain `k` (with fixed zero `z`) pulls the poles further into the left half-plane -> faster, less oscillatory response (compare `k=1,z=1` vs `k=24,z=2` step responses).
- **Bode / loop-shaping**: with the open-loop `L(s) = k*(s+z)/(z*s^2)`, read off:
  - Gain margin `GM` and phase margin `PM` from the Bode plot (example: `k=24, z=2` gives `GM = +inf`, `PM = 80.7°` at crossover `wc = 12.2 rad/s`).
  - Maximum tolerable pure time delay from phase margin: `tau < tau_c = PM / wc` (numerically `0.116 s` in the example).
  - **Noise attenuation** spec: require `|Y(jwn)/N(jwn)|_dB <= -10 dB` for `wn` in a high-frequency band (e.g. `[100, 1000] rad/s`), i.e. the loop gain must roll off enough at high frequency.
  - **Disturbance attenuation** spec: require `|Y(jwd)/D(jwd)|_dB <= -80 dB` for low-frequency disturbances `wd` in e.g. `[0.01, 0.1] rad/s`, i.e. `|G(jwd)|_dB >= 80 dB` (high loop gain at low frequency).
  - **Reference-following** spec similarly requires `|G(jwr)|_dB >= 100 dB` for the reference-frequency band.

### Disturbance rejection and integral action
Question posed: does the PD-controlled position loop reject a **constant wind disturbance** modeled as an acceleration input `w`? Using the final value theorem on `Y(s)/W(s) = 1/(s^2+C(s))` with `W(s) = w0/s`: **No** — steady-state error `y*(t) -> r(t) + w0/k` (a constant offset proportional to disturbance magnitude and inversely proportional to gain `k = kP`).

**Fix: add integral action -> PID controller.** Strategy shown as adding a zero and an integrator to go from PD to PID:
`C_PD(s) = k*(s+z)/z` -> `C_PID(s) = k*(s+z)(s+z1) / (z*z1*s)`, equivalently `C(s) = kP + kI/s + kD*s = (kD*s^2 + kP*s + kI)/s`.
With integral action, `Y(s)/W(s) = s / (s^2 + kD*s^2 + kP*s + kI)` (note the extra `s` in the numerator from the integrator), so `lim y(t) = lim s*Y(s) = 0` for a step disturbance — the constant wind offset is rejected. Design guidance: keep the existing zero at `-z`, add the new integrator/zero pair at `-z1` (example: `k=16, z=2, z1=0.5`).

Exercise left in the slides (unsolved): repeat the wind-rejection analysis for a **velocity** disturbance input instead of acceleration.

An experimental result slide shows a real quadrotor holding position in front of a desk fan (physical demonstration of disturbance rejection).

### Extending hierarchical control to full state feedback (bridge to Ch4)
- Adding integral action to the position loop is generalized as **state feedback with an augmented integrator state**: `u_T(t) = -kP*(p-p*) - kD*(p_dot-p_dot*) - kI*Integral(p-p*) - g*e3 + p_ddot*(t)`.
- The **attitude controller** is derived by linearizing about hover: equilibrium `lambda_0 = [0,0,psi*]^T`, `omega_0 = 0`, `n_0 = 0`. Standard linearization `delta_xdot = (df/dx)|_eq * delta_x + (df/du)|_eq * delta_u` applied to `lambda_dot = Q(phi,theta)*omega`, `J*omega_dot = -S(omega)*J*omega + n` gives, when `J` is diagonal, a decoupled double-integrator per axis: `delta_lambda_dot = delta_omega`, `J*delta_omega_dot = delta_n`. A **PD controller** is then used: `n = delta_n = -k1*(lambda - lambda_d) - k2*omega`.
- **Full hierarchical block diagram** (position PID -> attitude PD -> rotational dynamics -> translational dynamics) is reduced, for design purposes, to a simplified nested SISO model: outer loop `kp + kv*s` around an inner loop `(k_lambda + k_omega*s)` feeding two integrators `1/s * 1/s` (attitude+angular rate) into the position double integrator `1/s^2`. Design rule of thumb: make the inner loop 5-10x faster (higher bandwidth) than the outer loop so they can be tuned quasi-independently. Root-locus examples compare `C1(s)=5(s+1)` inner vs `C2(s)=10(s+2.5)` outer (well-separated, clean step response) against `C2(s)=3(s+2.5)` (insufficient separation -> oscillatory response).
- **Full pole-placement version (last slides, bridging into Ch4's state-space viewpoint)**: model the whole cascade (position PID error states + inner-loop dynamics) as one augmented state-space system and place poles directly with a single gain vector `K`, e.g. `K = place(A2, B2, [-0.5+0.2i, -0.5-0.2i, -0.1, roots([1, k_omega, k_lambda])])`, giving a 5-element state feedback gain `u = theta_r = [0.029, 0.419, 1.4973, 1.1975, 0.275] * [x_I; y-r; ydot-rdot; theta; thetadot]`. Explicitly noted: **this is no longer hierarchical** — it's a single full-state feedback law over the combined state, which is exactly the "modern control design" (state-space) approach developed in Ch4.

## Chapter 4 - Modern Control Design

Two parts: (1) general linear state-space control/estimation theory (controllability, observability, pole placement, observers, LQR), following P. Batista's notes; (2) a worked LQR design example on aircraft roll dynamics (from Hespanha's textbook), connecting LQR back to loop-shaping/Bode ideas used in Ch3.

### Linear state-space models
General (possibly time-varying) form: `x_dot(t) = A(t)x(t) + B(t)u(t)`, `y(t) = C(t)x(t) + D(t)u(t)`. The course always sets `D(t) = 0` (no direct feedthrough). When `A, B, C` are constant -> **LTI system**: `x_dot = Ax + Bu`, `y = Cx`.

- **Transfer function matrix** (LTI only): Laplace transform with zero initial conditions gives `Y(s) = C(sI-A)^-1 B U(s)`, i.e. `G(s) = C(sI-A)^-1 B`.
- **Time-domain solution** (LTI, initial state `x0` at `t0`): `x(t) = e^{A(t-t0)} x0 + Integral_{t0}^{t} e^{A(t-sigma)} B u(sigma) dsigma`, and correspondingly for `y(t)`.

### Controllability
Definition: the system is controllable on `[t0, tf]` if for any `x0`, there exists a continuous input `u(t)` driving `x(tf) = 0` (note the slide's convention: driving state to **zero**, not to an arbitrary target — equivalent statement for LTI systems since the reachable set is a subspace).

**Theorem (LTI)**: the system is controllable iff the controllability matrix `C := [B, AB, A^2 B, ..., A^{n-1} B]` has full rank `n` (n = number of states).
Worked 2-state examples: `A=[[0,1],[0,0]]`, `B=[0,1]` (i.e. `xdot1=x2, xdot2=u`) is controllable (`C` full rank); `A=[[0,1],[0,0]]`, `B=[1,0]` (i.e. `xdot1 = x2+u, xdot2=0`) is **not** controllable (rank-deficient `C`) — intuition: `x2` is never affected by `u`, so it cannot be driven.

### Observability
Definition: observable on `[t0,tf]` if the initial state `x0` is uniquely determined from `y(t)` for `t` in `[t0,tf]`. Explicitly noted: for **known-input** LTV systems, observability does not depend on whether `u` is zero or not (subtract the known-input contribution from the output first) — for nonlinear systems this does NOT hold in general (much harder analysis).

**Theorem (LTI)**: observable iff the observability matrix `O := [C; CA; ...; CA^{n-1}]` has full rank `n`.
Worked examples mirror controllability: `C=[1,0]` on the same `A` is observable; `C=[0,1]` is not (can't tell `x1` apart from the output).

### State feedback and pole placement
Linear state feedback: `u(t) = r(t) - K*x(t)` (block diagram: reference `r`, minus `Kx` feedback, into `xdot=Ax+Bu`).

**Pole placement theorem**: if `(A,B)` is controllable, then for **any** desired real monic degree-n polynomial `p(lambda)`, there exists a constant gain `K` such that `det(lambda*I - A + BK) = p(lambda)` — i.e. the closed-loop eigenvalues can be placed arbitrarily.

### State observers (Luenberger observer)
Motivation: state feedback needs the full state, which usually isn't measured directly -> build a state estimate.

**Luenberger observer**: `xhat_dot(t) = A(t)xhat(t) + B(t)u(t) + L(t)[y(t) - C(t)xhat(t)]` — a copy of the plant driven by the actual input, corrected by the output estimation error weighted by gain `L`.

Estimation error `xtilde := x - xhat` obeys `xtilde_dot(t) = [A(t) - L(t)C(t)] xtilde(t)` — autonomous linear dynamics independent of `u` and `y`; convergence requires `A-LC` to be stable (eigenvalues negative real part / Hurwitz).

**Duality**: eigenvalues of `A - LC` equal eigenvalues of `(A-LC)^T = A^T - C^T L^T`. This maps the observer design problem onto the pole-placement problem under the substitution `A <-> A^T`, `B <-> C^T`, `K <-> L^T` — i.e. designing `L` for observability is mathematically identical to designing `K` for controllability, just transposed. Observer poles chosen via `L = place(A', C', pCL)'` in the worked example (reusing the same closed-loop pole set as the LQR design, `pCL = eig(A-B*K)`).

### Combining state feedback + observer: separation principle
Using estimated state in the feedback law: `u(t) = r(t) - K*xhat(t)`. Substituting gives the augmented `[x; xtilde]` system:
`[xdot; xtildedot] = [[A-BK, BK],[0, A-LC]] [x; xtilde] + [B;0] r(t)`, `y = [C, 0][x; xtilde]`.

**Theorem/Separation principle**: because the augmented system matrix is block upper-triangular, its eigenvalues are exactly the union of the eigenvalues of `A-BK` and `A-LC`. **Conclusion: controller gain `K` and observer gain `L` can be designed completely independently** by pole placement (or, more strongly, independently as *optimal* LQR/estimator problems — "more on this later" per the slides). This does **not** generalize to nonlinear systems (no general separation result).

### Optimal control: Linear Quadratic Regulator (LQR)
Motivation: pole placement lets you "shape" closed-loop dynamics but doesn't optimize anything; LQR minimizes a cost.

**LQR problem**: for `xdot=Ax+Bu`, minimize `J = Integral_0^inf [x^T(t) Q x(t) + u^T(t) R u(t)] dt`, with `Q` positive semi-definite and `R` positive definite.

**Theorem**: if `(A,B)` is *stabilizable* and `(A,G)` is *detectable* (with `Q = G^T G`), then:
- There is a unique positive-definite solution `P` to the **algebraic Riccati equation (ARE)**: `A^T P + P A + Q - P B R^-1 B^T P = 0`.
- The optimal feedback is `u(t) = -Kx(t)` with `K := R^-1 B^T P`, achieving minimum cost `J = x^T(0) P x(0)`.
- All closed-loop eigenvalues of `A - BK` have negative real part (guaranteed stability, not just optimality).

Stabilizability/detectability are **relaxations** of full controllability/observability: `(A,B)` stabilizable means only the *uncontrollable modes* need to be stable (don't need to control already-stable modes); `(A,G)` detectable means only the *unobservable modes* need to be stable.

**Tuning `Q`/`R` — Bryson's rule** (rule of thumb, "trial and error" per the slides): `Q_ii = 1 / (max acceptable value of x_i^2)`, `R_ii = 1 / (max acceptable value of u_i^2)`. Larger `Q` relative to `R` -> more control effort spent to keep the state small; larger `R` relative to `Q` -> conserve control effort at the cost of larger state excursions.

### LQR worked example: aircraft roll dynamics (bridges LQR back to loop-shaping)
Linearized model (from Hespanha's book): `phi_dot = omega`, `omega_dot = 0.8*omega - 20*tau`, `tau_dot = -50*tau + 50*u`; state `x=[phi,omega,tau]`, output `y=phi=Cx` with `C=[1,0,0]`,
`A = [[0,1,0],[0,-0.8,-20],[0,0,-50]]`, `B = [0,0,50]^T`.

Cost: `J = Integral (x^T Q x + rho*u^2) dt`, with `Q = G^T G`, `G = [[1,0,0],[0,gamma,0]]` (design weights `rho>0`, `gamma>0`), so the "regulated output" is `z(t) = G*x(t) = [phi; gamma*omega]` — i.e. penalize roll angle and (scaled) roll rate, plus control effort weighted by `rho`.

Loop-shaping connection: define `P(s) = (sI-A)^-1 B` (plant, u to x), `L(s) = K*P(s) = (sI-A)^-1 B K` (open-loop, scalar since SISO from `-u_bar` to `x` through `K`). Standard `GM`, `PM`, noise/disturbance/reference-tracking specs from Ch3 all carry over directly to this `L(s)`.

**Kalman's equality** (derivable from the ARE): `|1 + L(jw)|^2 = |1 + KP(jw)|^2 = 1 + ||G*P(jw)||^2 / rho`. With `P(s) = [P1(s); s*P1(s); P3(s)]` at low frequency, `|L(jw)| ≈ |1 + j*gamma*w| * |P1(jw)| / sqrt(rho)`.

Effects of the two design weights (matched to simulated Bode/step-response families in the slides):
- **Decreasing `rho`** (cheaper control) moves the Bode magnitude plot up -> higher loop gain -> faster response, less sensitivity to disturbances, but at the cost of more control effort (intuition: "input becomes cheaper").
- **Increasing `gamma`** (penalizing roll rate more) increases the phase margin -> less overshoot but slower response (intuition: increasing the cost of angular velocity discourages fast maneuvers).

### Luenberger observer worked example (same roll-dynamics system)
MATLAB-style snippet shown:
```
R = 1;
G = [1 0 0; 0 .3 0]; Q = G'*G;
K = lqr(A,B,Q,R);            % K = [-1.0000, -0.4105, 0.1526]
damp(A-B*K)                  % closed-loop eigenvalues/damping/freq listed
pCL = eig(A-B*K);
L = place(A',C',pCL)';       % choose same poles for the observer (duality)
esR = estim(ss(A,B,C,0), L, 1, 1);   % build the observer/estimator object
```
Closed-loop LQR poles: a complex pair `-4.40 ± 0.905i` (damping ≈0.979, freq ≈4.49 rad/s) and a fast real pole `-49.6`. Observer gain obtained by placing the *same* pole locations via the transposed (dual) problem: `L = [7.6275, 29.0897, 37.9793]^T`.

A Simulink block-diagram slide contrasts **"LQR ideal feedback"** (full-state feedback using the true state `x`, i.e. `-Kx`) against **"LQR + observer feedback"** (feedback using the estimated state `xhat` from a noisy sensor, i.e. `-K*xhat`). Simulation plots (step response) show: (1) outputs — the observer-based response tracks the ideal-feedback response closely after a brief transient; (2) measurement — a noisy sensor signal converging to -1; (3) control input — a transient spike near t≈1 (when the step hits) then settling to a small noisy value around 0, demonstrating the observer-based controller performs comparably to ideal state feedback despite only having noisy output measurements.

## Key equations reference

- Quadrotor translational/rotational dynamics: `m*p_ddot = -T*r3 + m*g*e3`, `R_dot = R*S(omega)`, `J*omega_dot = -S(omega)*J*omega + n` — the plant being controlled throughout both chapters.
- Hierarchical control step 1: `u_T = -(T/m)*r3d` ⇒ `T = m||u_T||`, `r3d = -u_T/||u_T||` — decouples thrust magnitude/direction from the rotational subproblem.
- Position PD/PID law: `u_T = -kP*e - kD*e_dot [- kI*Integral(e)] - g + p_ddot*` — the position-loop control law; integral term needed for zero steady-state error under constant wind.
- Damping ratio from gains: `xi = kD/(2*sqrt(kP))` — lets you pick `(kP,kD)` for a target overshoot/settling-time spec.
- Phase-margin time-delay bound: `tau_c = PM/wc` — max pure delay the loop tolerates before instability.
- Steady-state wind offset (PD only): `y*(t) = r(t) + w0/kP` — quantifies why integral action is needed.
- Attitude reference extraction: `[cos(phi_d)sin(theta_d), -sin(phi_d), cos(phi_d)cos(theta_d)]^T = Rz(-psi*) r3d` — converts a desired thrust direction into desired roll/pitch given desired yaw.
- Controllability matrix: `C = [B, AB, ..., A^{n-1}B]`, full rank `n` ⟺ controllable — decides whether pole placement/LQR is achievable.
- Observability matrix: `O = [C; CA; ...; CA^{n-1}]`, full rank `n` ⟺ observable — decides whether a convergent observer exists.
- Pole placement theorem: `det(lambda*I - A + BK) = p(lambda)` solvable for any monic `p` iff `(A,B)` controllable.
- Luenberger observer: `xhat_dot = A*xhat + B*u + L(y - C*xhat)`; error dynamics `xtilde_dot = (A-LC)*xtilde`.
- Duality: eigenvalues of `A-LC` = eigenvalues of `A^T - C^T L^T` ⇒ observer design = transposed controller design (`A↔A^T, B↔C^T, K↔L^T`).
- Separation principle: eigenvalues of the combined state-feedback+observer system = eig(A-BK) ∪ eig(A-LC) — justifies designing `K` and `L` independently (LTI only).
- Algebraic Riccati equation: `A^T P + P A + Q - P B R^-1 B^T P = 0`, optimal gain `K = R^-1 B^T P`, minimizing `J = Integral(x^T Q x + u^T R u) dt`, min cost `= x^T(0) P x(0)`.
- Bryson's rule: `Q_ii = 1/(max acceptable x_i^2)`, `R_ii = 1/(max acceptable u_i^2)` — practical starting point for LQR weight tuning.
- Kalman's equality: `|1+L(jw)|^2 = 1 + ||G*P(jw)||^2/rho` — links the LQR weighting choice directly to open-loop Bode magnitude/phase margin.

## Open questions / unclear points

- Ch3 leaves an explicit **unsolved exercise**: redo the wind-disturbance-rejection analysis with the disturbance entering as a velocity input rather than an acceleration input.
- The exact numeric values of `kw` (`k_omega`) and `kl` (`k_lambda`) used in the Ch3 slide 43-44 inner-loop pole-placement example are referenced (`roots([1,kw,kl])`) but not stated explicitly on the slides read — only the resulting 5-element gain vector is given.
- Ch4's `damp(A-B*K)` MATLAB output and the observer gain `L` are shown as pre-computed results (presumably from a live MATLAB/Simulink demo); the underlying `.m`/`.slx` files are not included in this PDF, only screenshots of the console/Simulink model.
- The `estim(ss(A,B,C,0), L, 1, 1)` call's last two arguments (sensor/noise indices) are shown without explanation in the slide — standard MATLAB Control System Toolbox usage, but not elaborated in the lecture.
- Ch4 defers the "optimal" version of the observer (Kalman filter) with "more on this later" — that material is presumably covered in Ch5/Ch6 (Sensors and State Estimation), not in this file.

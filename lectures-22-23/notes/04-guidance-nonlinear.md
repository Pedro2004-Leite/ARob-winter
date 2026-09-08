# Nonlinear Control, Path Following, and Path Management

Source files:
- `22_23_UAVs_Ch7_Intro_Nonlinear_Control.pdf` (46 pages)
- `22_23_UAVs_Ch7_Intro_Nonlinear_Control_Part_II.pdf` (48 pages)
- `22_23_UAVs_Ch8_Path_Following.pdf` (19 pages, = Beard & McLain Ch.10)
- `22_23_UAVs_Ch9_PathManager.pdf` (28 pages, = Beard & McLain Ch.11)
- `ar23-ch10.pdf` (11 pages)

Course: UAVs / Aeronaves Robotizadas, MEAer, Instituto Superior Técnico (Lisboa), Autumn Semester 2022/2023. Instructors: Rita Cunha, J. R. Azinheira, José Raul Azinheira. Ch7 slides are "based partially on original material by A. Pascoal" (DSOR lab, IST/ISR). Ch8/Ch9 are taken almost verbatim from Beard & McLain, *Small Unmanned Aircraft: Theory and Practice*, Princeton University Press, 2012 (their Chapters 10 and 11).

## Chapter 7 (Part I) - Introduction to Lyapunov Stability Theory

Motivation: nonlinear feedback linearization control laws (e.g. AUV speed control `T = m_T(dv_r/dt + Ke) + βv + fv²`) give clean exponential error decay `e(t)=e(0)e^{-kt}` **only if the nonlinear dynamics are known exactly**. Real systems have uncertain parameters, so a more robust theoretical tool is needed: Lyapunov theory.

**Core idea**: for `dx/dt = f(x)`, `f(0)=0`, propose an energy-like scalar function `V(x)` (positive definite) and check the sign of its time derivative along trajectories, without solving the ODE.

**Stability definitions** (equilibrium `x_e=0` of `dx/dt=f(x)`):
- *Stable*: ∀ε>0 ∃δ(ε)>0 : ‖x(0)‖<δ ⇒ ‖x(t)‖<ε ∀t≥0.
- *Unstable*: not stable.
- *Asymptotically stable*: stable AND ‖x(0)‖<δ ⇒ lim_{t→∞} x(t)=0. (Attractiveness alone does NOT imply stability - both conditions are needed.)

**Lyapunov's Stability Theorem**: if ∃ continuously differentiable `V(x)>0` on domain D containing the equilibrium, and:
- `V̇(x) ≤ 0` in D ⇒ origin is stable.
- `V̇(x) < 0` in D ⇒ origin is asymptotically stable.
- If additionally `V(x)→∞` as `‖x‖→∞` (radially unbounded) and `V̇<0` everywhere ⇒ origin is **globally** asymptotically stable (GAS).

**Krasovskii-LaSalle invariance principle**: if `V̇(x) ≤ 0` and Ω = {x : V̇(x)=0}, and the only trajectory entirely contained in Ω is the null trajectory, then the origin is asymptotically stable (more generally, trajectories converge to the largest invariant set M inside Ω). Useful when V̇ is only negative *semi*-definite (a common practical case, e.g. damped pendulum, position/velocity trackers where V̇ vanishes at v=0 but not at e=0).

**Choosing V - three general methods**:
1. **Krasovskii's method**: try `V(x) = f(x)ᵀ f(x)` (energy of the velocity field itself). Compute the Jacobian `J = ∂f/∂x`; if `F = J + Jᵀ` is negative definite everywhere, the origin is GAS (since `V(x)→∞` as `‖x‖→∞` for polynomial-type f).
2. **Variable gradient method**: assume a linear-in-x form for `∇V` (e.g. `∇V₁ = a₁₁x₁+a₁₂x₂`), impose the curl condition `∂∇V₁/∂x₂ = ∂∇V₂/∂x₁` (needed for ∇V to be a true gradient), choose free coefficients to make `V̇ = ∇Vᵀf(x) < 0`, then integrate `V(x) = ∫₀ˣ ∇V·dy` along any path. Worked example: nonlinear spring-mass system `ẋ₁=x₂, ẋ₂=-h(x₁)-βx₂` yields `V(x) = ∫₀^{x₁} h(y)dy + ½x₂²` (physically: potential + kinetic energy) - this generalizes energy-based Lyapunov functions to nonlinear stiffness/damping.
3. **Linear systems (`ẋ=Ax`)**: propose `V=xᵀPx` with P symmetric positive definite. Then `V̇ = xᵀ(AᵀP+PA)x = -xᵀQx`. Solving the **Lyapunov equation** `AᵀP + PA = -Q` for P>0 given any Q>0 (with A Hurwitz) proves stability. MATLAB: `P = lyap(A,Q)`.

**Worked examples covered**: multiple-equilibria polynomial systems (equilibrium classification via Jacobian eigenvalues - stable node vs. saddle), a 4th-order polynomial system stabilized via Krasovskii's method, and a variable-gradient example matching Slotine & Li's textbook.

## Chapter 7 (Part II) - Nonlinear Control Design (Backstepping, Adaptive Control, Quadrotor Control)

Slide deck by Rita Cunha / J.R. Azinheira, "additional notes" continuing directly from Part I.

### 2nd-order phase-portrait gallery
Illustrates qualitatively different behaviors solvable via Lyapunov theory: asymptotically stable spiral, marginally stable center (V̇=0 exactly, only Lyapunov *stability*, not asymptotic), unstable saddle, Van der Pol limit cycle (no equilibrium is stable, but a stable periodic orbit exists - Lyapunov theory as taught here doesn't directly capture limit cycles), and multiple-equilibria basins of attraction.

### Lyapunov analysis and adaptive control
Pattern for systems with an unknown constant parameter θ: `ẋ = f(x) + g(x,u)θ`. Introduce estimate `θ̂`, estimation error `θ̃ = θ-θ̂`, choose control law `u=h(x,θ̂)` and **augment** the Lyapunov function with the parameter error: `V₂ = V(x) + (1/k_θ)θ̃²`. Differentiating and picking the **adaptation law** `θ̇̂ = k_θ (∂V/∂x) g(x,u)` cancels the cross term, giving `V̇₂ = -W(x) ≤ 0`. Worked scalar example `ẋ=θx+u`: control `u=-(k+θ̂)x`, adaptation `θ̇̂=x²`, closed loop `ẋ=(θ̃-k)x`. LaSalle gives `x(t)→0`, but θ̂ is generally only bounded (not guaranteed to converge to true θ), unless the reference/signal is "persistently exciting."

### Backstepping (constructive nonlinear design)
Recursive Lyapunov-based procedure for **strict-feedback systems**:
```
ż₁ = f₁(z₁) + g₁(z₁) z₂
ż₂ = f₂(z₁,z₂) + g₂(z₁,z₂) z₃
  ⋮
żk = fk(z₁,...,zk) + g₂(z₁,...,zk) u
```
Worked example (double integrator with disturbance, e.g. 1-D position tracking `ë = u_T + g - p̈_d`):
1. `z₁=e`, `V₁=½z₁ᵀz₁`, add/subtract `k₁z₁` to define `z₂ = v_e + k₁z₁` (virtual control substitution), giving `V̇₁ = -k₁z₁ᵀz₁ + z₁ᵀz₂` (a leftover cross term to cancel at the next step).
2. Augment `V₂ = k₁²V₁ + ½z₂ᵀz₂`, differentiate, and choose the actual control `u_T = -g + p̈_d - k₂z₂` to make `V̇₂ = -k₁²W₁(z₁) - W₂(z₂) < 0` (needs `k₂>k₁`).
3. Result: closed-loop is a stable 2nd-order linear-like error system; back-substituting gives `p̈ = -k₁k₂(p-p_d) - k₂(ṗ-ṗ_d) + p̈_d` - i.e., backstepping here reproduces a PD-like tracking law but derived constructively with a Lyapunov certificate. Backstepping generalizes recursively to k-dimensional chains and can be combined with adaptive estimation ("adaptive backstepping"); robustness against disturbances is flagged as an open question in the slides.

### Quadrotor trajectory tracking via nonlinear/hierarchical (cascade) control
Standard **inner-outer (cascade) loop architecture**:
- Outer loop (position controller): computes thrust magnitude `T` and desired body z-axis direction `r_3d` from position error.
- Inner loop (attitude controller): computes torque `n` (or `τ`) to drive actual attitude `R` to the desired `R_d`.

**Step 1 - translational (position) control**: neglect rotational dynamics, replace `r_3` with virtual desired direction `r_3d`. Model `p̈ = u_T + g`. PD-style Lyapunov design (same backstepping-style construction as above) gives `u_T = -k_P e - k_D v_e - g + p̈_d`, proven GAS via LaSalle on the autonomous closed-loop error system.

**Step 2 - decompose `u_T` into thrust + direction**:
- `T_d = m‖-k_1z_1-k_2z_2-g+p̈*‖` (desired thrust magnitude)
- `r_3d = (-k_1z_1-k_2z_2-g+p̈*)/‖·‖` (desired thrust direction, i.e. desired body z-axis)
- Actual thrust command: `T = T_d(r_3ᵀ r_3d)` (projection of desired thrust onto the *current* body z-axis `r_3`, since thrust can only be applied along `r_3`, not `r_3d`, until attitude catches up).
- This introduces a new error `z_3 = r_3 - r_3d` with dynamics `ż_3 = -S(r_3)RΠ_{e3}ω - ṙ_3d` (independent of the yaw component `ω_3`, meaning yaw is a free/decoupled DOF used later to track a desired heading `ψ*`).
- Augmented Lyapunov `V = [z_1 z_2]P[z_1;z_2] + ½z_3ᵀz_3` yields `V̇ = -W(z_1,z_2) + T_d[z_1 z_2]PΠ_{r3}z_3 + z_3ᵀ(S(r_3)ω - ṙ_3d)`; with adequate choice of angular velocity command, `V̇ = -W_3 ≤ 0`.
- Alternative: reconstruct the *whole* desired rotation matrix `R_d = R_z(ψ*)R_y(θ_d)R_x(φ_d)` from `(r_3d, ψ*)` via `[cosφ_d sinθ_d; -sinφ_d; cosφ_d cosθ_d] = R_z(-ψ*) r_3d`, then design an attitude controller to drive `R → R_d`.

### Geometric control for attitude tracking (SE(3)/SO(3) control, Lee-Leok-McClamroch style)
Attitude dynamics: `Ṙ = RS(ω)`, `Jω̇ = -S(ω)Jω + τ`. Rotation error `R_e = R_dᵀR`, error dynamics `Ṙ_e = R_e S(ω)` (assuming `Ṙ_d=0`).

Represent `R_e` via angle-axis `(θ,n)`: `R_e(θ,n) = I₃ + sinθ S(n) + (1-cosθ)S(n)²`. Rotation-error "potential" `V₁(R_e) = 1-cosθ = tr(I-R_e) > 0`.

Lyapunov candidate `V = k_1 tr(I-R_e) + ½ωᵀJω`. With `θ̇ = nᵀω`, `V̇ = ω^T(2k_1 sinθ·n + τ)`. Choosing **control law** `τ = -k_1(2sinθ·n) - k_2ω` gives `V̇ = -k_2ωᵀω ≤ 0`.

Equivalent forms of the rotation error vector `e_R`:
- `e_R = 2 sinθ · n` (angle-axis form)
- `e_R = S⁻¹(R_e - R_eᵀ)` (matrix "vee-map" form, standard in geometric control literature)

Control law: `τ = -k_1 e_R - k_2ω`.

**Important caveat (topological limitation)**: LaSalle only guarantees convergence to `ω=0` and `θ∈{0,π}` - i.e., there is a second, undesired equilibrium at `θ=π` (180° rotation error) which is also stationary under this law. A cross term `ce_Rᵀ Jω` must be added to the Lyapunov function to prove *local* asymptotic stability of the true `(θ,ω)=(0,0)` equilibrium and shrink the basin of attraction away from `θ=π`. Fundamental result quoted: **no continuous feedback law can globally stabilize an attitude system**, because rotation matrices evolve on a compact manifold (SO(3)) that is not diffeomorphic to Euclidean space (topological obstruction, "hairy ball"-type argument) - this is why attitude control is only *almost-global* or *local*, never truly global, unlike position control.

**Bibliography given**: H. Khalil, *Nonlinear Systems* 3rd ed. (2001); Slotine & Li, *Applied Nonlinear Control* (1991); T. Lee, M. Leok, N.H. McClamroch, "Geometric tracking control of a quadrotor UAV on SE(3)," CDC 2010; R. Mahony, V. Kumar, P. Corke, "Multirotor Aerial Vehicles," IEEE RAM 2012; D. Cabecinhas, R. Cunha, C. Silvestre, "A nonlinear quadrotor trajectory tracking controller with disturbance rejection," Control Engineering Practice, 2014 (this last one is by the course's own instructor, Rita Cunha).

## Chapter 8 - Path Following (= Beard & McLain Ch. 10)

Context: small UAVs are strongly affected by wind relative to their airspeed, making classical *trajectory tracking* (chase a moving reference point with a time parametrization) fragile - it requires exact real-time wind knowledge to compute the needed airspeed. **Path following** is more robust: the goal is only to converge onto and stay on a *geometric* path, without a time schedule.

Two path primitives are treated (more complex paths are built by concatenating these):

### Straight-line following
Line defined by origin point **r** and unit direction **q**. UAV position **p**. Path error `e_p = p - r`. Rotating into the path frame (rotation `R_i^P` by course angle `χ_q` of the line) gives along-track error `e_px` and **cross-track error** `e_py`.

Error dynamics: `ė_py = V_g sin(χ - χ_q)` where `χ` is the current course angle and `V_g` ground speed.

**Vector-field / course-command law**: command course angle
`χ^c = χ_q - χ^∞ (2/π) atan(k_path · e_py)`
- `χ^∞` = the course offset commanded far from the line (typically 90°, i.e. fly perpendicular to intercept it).
- `k_path` tunes how sharply the commanded course transitions from `χ^∞` (far away) to `χ_q` (on the line); rule of thumb `k_path ≈ 1/R_min` (R_min = minimum turn radius of the aircraft).
- Lyapunov proof: `W(e_py)=½e_py²`, `Ẇ = -V_a e_py sin(χ^∞ (2/π)atan(k_path e_py)) < 0` ⇒ `e_py→0` asymptotically.

Practical "smallest angle turn logic": `χ_q = atan2(q_e,q_n) + 2πm`, wrapped so that `-π ≤ χ_q-χ ≤ π` (avoids commanding a >180° turn due to angle wraparound). This is **Algorithm 3 (followStraightLine)** in Beard & McLain.

### Circular orbit following
Orbit defined by center **c**, radius ρ, direction λ (+1 = CW, -1 = CCW). In polar coordinates relative to the center, letting d = distance to center, φ = bearing angle:
`ḋ = V_g cos(χ-φ)`, `φ̇ = (V_g/d) sin(χ-φ)`.

Desired approach angle offset `χ^o = φ + λπ/2` (tangent direction). **Commanded course**:
`χ^d(d-ρ,λ) = χ^o + λ·atan(k_orbit·(d-ρ)/ρ)`, i.e. full form
`χ^c(t) = φ + λ[π/2 + atan(k_orbit(d-ρ)/ρ)]`.

Lyapunov proof: `W = ½(d-ρ)²`, `Ẇ = -V_g(d-ρ) sin(atan(k_orbit(d-ρ)/ρ)) < 0` for `d≠ρ` ⇒ `d→ρ` asymptotically. This is **Algorithm 4 (followOrbit)**.

### Lookahead-distance alternatives
Two other popular guidance approaches mentioned (not derived in detail): **Line-of-sight (LOS)** guidance and the **L1 guidance law**, both suited to straight or circular segments, using a lookahead point at fixed distance ahead on the path rather than the direct cross-track vector-field law above.

## Chapter 9 - Path Manager (= Beard & McLain Ch. 11)

Handles sequencing a list of waypoints into continuous straight-line/orbit path-following commands, i.e. sits directly above the path-following layer in the control architecture (path planner → **path manager** → path following → autopilot → aircraft, with a state estimator feeding measurements back to all layers).

### Waypoint path definition
`𝒲 = {w_1,...,w_N}`, each `w_i ∈ ℝ³`.

### Waypoint switching logic (when to move from segment i-1→i to segment i→i+1)
Two methods:
1. **b-ball**: switch when UAV enters a sphere of radius b around `w_i`.
2. **Half-plane** `𝓗(r,n) = {p : (p-r)ᵀn ≥ 0}` through `w_i`: preferred method. Direction unit vectors `q_i = (w_{i+1}-w_i)/‖w_{i+1}-w_i‖`. Normal to the switching plane bisects the incoming/outgoing legs: `n_i = (q_{i-1}+q_i)/‖q_{i-1}+q_i‖`. UAV tracks line `w_{i-1}→w_i` until entering `𝓗(w_i,n_i)`, then switches to tracking `w_i→w_{i+1}`. This is **Algorithm 5 (followWpp)**.

Straight waypoint-to-waypoint switching produces visible "bulges"/overshoot at each corner in simulation (seen in the "Waypoint Following Results" plot).

### Fillets (smoothing corners with circular arcs)
A circular arc of radius R is inserted at each waypoint corner, tangent to both incoming and outgoing legs, to avoid a sharp course-angle discontinuity. Given half-turn angle ϱ (angle between `q_{i-1}` and `q_i`):
- Fillet center: `c = w_i - (R/sin(ϱ/2)) · (q_{i-1}-q_i)/‖q_{i-1}-q_i‖`
- Entry half-plane `𝓗_1`: location `r_1 = w_i - (R/tan(ϱ/2))q_{i-1}`, normal `q_{i-1}`.
- Exit half-plane `𝓗_2`: location `r_2 = w_i + (R/tan(ϱ/2))q_i`, normal `q_i`.
- A 3-state state machine (straight line → orbit-fillet → next straight line) governs switching; this is **Algorithm 1 (followWppFillet)**, producing visibly smoother corner-cutting trajectories than plain waypoint switching.
- Extra path length from fillets: `|𝒲|_F = |𝒲| + Σ(Rϱ_i - 2R/tan(ϱ_i/2))` (fillets always *shorten* the path length relative to the straight-corner distance sum, since the arc cuts the corner).

### Dubins paths (when heading/course at each waypoint also matters)
For a unicycle-like kinematic model `ṗ_n=V cosϑ, ṗ_e=V sinϑ, ϑ̇=u`, `u∈[-ū,ū]`, the **time-optimal path between two configurations (position+heading)** is a **turn-straight-turn** path: circular arc (radius `R=V/ū`) → straight segment → circular arc. This is the classical **Dubins path**.

Four combinatorial cases based on turn directions at start/end:
- Case I: R-S-R (right-straight-right)
- Case II: R-S-L
- Case III: L-S-R
- Case IV: L-S-L

The actual Dubins path is the **shortest of the 4 candidate lengths** `L_1..L_4` (each derived via circle-tangent geometry and wrapped angular-distance formulas `|θ_2-θ_1|_CW = ⟨2π+θ_2-θ_1⟩`, `⟨φ⟩ ≜ φ mod 2π`). Full derivations with tangent-line lengths (`ℓ` = distance between circle centers) given for all 4 cases (slides 19-22).

**Algorithm 7 (findDubinsParameters)**: computes start/end turn circles, evaluates all 4 case lengths, picks the minimum, and derives the 3 half-planes (`z_1,q_1`), (`z_2,q_2`), (`z_3,q_3`) marking the turn→straight and straight→turn transition boundaries.

**Algorithm 8 (followWppDubins)**: a 5-state machine cycling through (1) first turn, (2) wait-for-half-plane-1, (3) straight segment, (4) second turn, (5) wait-for-half-plane-2 → advance to next waypoint pair and recompute Dubins parameters. Produces Dubins-connected waypoint paths that respect a minimum turn radius and match a desired heading at each waypoint (shown in "Dubins Path Following Results" simulation, tighter/cleaner corners than fillets when heading matters).

### Simulation examples shown
- Flying wing (2.5 kg, 2 m span), 20 m/s airspeed, 8 m/s wind, R_min=100 m, L1 guidance: straight-line + circular orbit path, roll/sideslip/deviation plots.
- Quadrotor (2.9 kg), 7 m/s wind from North with moderate turbulence, R_min=20 m: rounded-rectangle path tracking under wind disturbance.

## Key equations reference

- Lyapunov stability theorem: `V>0, V̇≤0 ⇒ stable`; `V>0, V̇<0 ⇒ asymptotically stable`; add radial unboundedness of V for global result.
- Lyapunov equation (linear systems): `AᵀP + PA = -Q`, solve for `P≻0` given `Q≻0` and A Hurwitz.
- Krasovskii's method: `V=fᵀf`; GAS if `J+Jᵀ` negative definite everywhere.
- Backstepping virtual-control substitution: define `z_{k+1} = ż_k(actual) - ż_k(desired-from-Lyapunov)`, augment `V_{k+1}=V_k+½z_{k+1}ᵀz_{k+1}`, cancel cross term with next-level control.
- Quadrotor cascade split: `T_d = m‖u_T‖`, `r_3d = -u_T/‖u_T‖`, actual `T = T_d(r_3ᵀr_3d)` (thrust projected onto current, not desired, body axis).
- Attitude tracking control law: `τ = -k_1 e_R - k_2ω`, with `e_R = S⁻¹(R_e-R_eᵀ) = 2 sinθ·n`; global attitude stabilization is topologically impossible (SO(3) is compact) - only almost-global/local results hold.
- Straight-line path-following command: `χ^c = χ_q - χ^∞(2/π)atan(k_path·e_py)`, `k_path ≈ 1/R_min`.
- Orbit-following command: `χ^c = φ + λ[π/2 + atan(k_orbit(d-ρ)/ρ)]`.
- Fillet corner-smoothing radius geometry: center offset `R/sin(ϱ/2)`, tangent-point offset `R/tan(ϱ/2)`.
- Dubins path = shortest of 4 turn-straight-turn combinations (RSR, RSL, LSR, LSL), each length from circle-tangent geometry with angle-wrapping.

## Open questions / unclear points

- The PDF page-index-vs-printed-slide-number in `Ch7_Part_II` do not match 1:1 (e.g. PDF page 23 shows printed slide number "29"); this looks like intentional slide renumbering by the instructors (a merged/edited deck) rather than missing content - all 48 PDF pages were read, so no material gap, just a note in case future page citations look inconsistent.
- `ar23-ch10.pdf` is **not** really "Chapter 10" of new material - it is a short (11-page) *course wrap-up / conclusion* talk by José Raul Azinheira (Jan 2023) that (a) recaps the Beard-McLain block diagram and lab exercises (Matlab/Simulink modeling, estimation, path control, a lab-3 circle-path example), and (b) lists the full course syllabus for confirmation: Ch0 Introduction, Ch1 Rigid body EOM, Ch2 Quadrotor modeling, Ch3 Quadrotor control intro, Ch4 Modern control design, Ch5 Sensors, Ch6 State estimation, Ch7 Nonlinear control, Ch8 Path following, Ch9 Path management. This confirms the course's own chapter numbering (0-9) differs from the Beard & McLain textbook's numbering (their Ch.10=our Ch.8, their Ch.11=our Ch.9) - worth remembering if cross-referencing page/chapter numbers later.
- Chapter 7 Part II's geometric-attitude-control section ends mid-derivation on the "extra cross term" needed for local asymptotic stability (slide 47/48) - the exact value of constant `c` and the full domain of attraction are asserted to exist but not spelled out numerically in the slides (left as an exercise/reference to Lee-Leok-McClamroch 2010).

# UAVs / Aeronaves Robotizadas - Course Knowledge Base (2022/23)

This file is the entry point into a deep technical analysis of every lecture PDF in this folder.
Detailed, equation-by-equation notes for each group of chapters live in `notes/` (linked below).
This file gives the course overview, the chapter map, and how the pieces fit together.

## Course identity

- **Course**: Unmanned Aerial Vehicles / Aeronaves Robotizadas (ARob), MEAer, Instituto Superior Técnico, Lisboa.
- **Edition**: Autumn Semester 2022/2023.
- **Faculty**: José Raul Azinheira (jraz@dem.ist.utl.pt, responsible), Rita Cunha (rita@isr.tecnico.ulisboa.pt).
- **Structure**: 2h/week lectures + 1h30/week lab (12h total), 3 lab assignments (L1 modelling/identification, L2 motion-variable estimation, L3 motion control), 1 report per lab.
- **Grading**: `E = max(E1, E2)` (exam, 40%), `L = 0.2*L1 + 0.4*L2 + 0.4*L3` (labs, 60%), `F = 0.4*E + 0.6*L`.
- **Main textbook**: Beard & McLain, *Small Unmanned Aircraft: Theory and Practice*, Princeton University Press, 2012 - directly reused (sometimes verbatim) for Ch5, Ch6, Ch8, Ch9.
- **Other references**: Astrom & Murray, *Feedback Systems*; Mahony/Kumar/Corke, "Multirotor Aerial Vehicles" (IEEE RAM 2012); Khalil, *Nonlinear Systems*; Slotine & Li, *Applied Nonlinear Control*; Lee/Leok/McClamroch, "Geometric tracking control of a quadrotor UAV on SE(3)" (CDC 2010); Cabecinhas/Cunha/Silvestre (2014, by the course's own instructor Rita Cunha).

**Important numbering gotcha**: the course's own chapter numbers (0-9) do **not** match Beard & McLain's book chapter numbers. Course Ch5→book Ch7, course Ch6→book Ch8, course Ch8→book Ch10, course Ch9→book Ch11. `ar23-ch10.pdf` is *not* a 10th technical chapter - it's an 11-page course wrap-up/conclusion talk that recaps the syllabus and lab exercises.

## Chapter map and detailed notes

| # | Topic | Source PDFs | Detailed notes |
|---|---|---|---|
| Ch0 | Course intro, UAV motivation, vehicle-type tradeoffs, generic control-loop architecture | `Ch0_Intro_Part_I/II` | [`notes/01-foundations.md`](notes/01-foundations.md) |
| Ch1 | Rigid-body kinematics & dynamics (rotations, Newton-Euler) | `Ch1_Rigid_Body` | [`notes/01-foundations.md`](notes/01-foundations.md) |
| Ch2 | Quadrotor dynamic modelling, actuation, differential flatness, SWaP design | `Ch2_Quadrotor_Modeling` | [`notes/01-foundations.md`](notes/01-foundations.md) |
| Ch3 | Quadrotor control intro: cascade position/attitude control, root-locus/Bode loop-shaping, integral action | `Ch3_Quadrotor_Control_Intro` | [`notes/02-control.md`](notes/02-control.md) |
| Ch4 | Modern control design: controllability/observability, pole placement, Luenberger observers, LQR | `Ch4_Modern_Control_Design` | [`notes/02-control.md`](notes/02-control.md) |
| Ch5 | Sensors (accelerometer, gyro, magnetometer, barometer, GPS) + AR.Drone specifics (sonar, vision) | `Ch5-6_Sensors_and_State_Estimation_1`, `Ch5_Sensors_Beard_McLain` | [`notes/03-sensors-estimation.md`](notes/03-sensors-estimation.md) |
| Ch6 | State estimation: Kalman filter, EKF, complementary filters, GPS smoothing, measurement gating | `Ch5-6_Sensors_and_State_Estimation_1`, `Ch6_State_Estimation_Beard_McLain` | [`notes/03-sensors-estimation.md`](notes/03-sensors-estimation.md) |
| Ch7 | Nonlinear control: Lyapunov theory, LaSalle, backstepping, adaptive control, geometric SO(3) attitude control | `Ch7_Intro_Nonlinear_Control`, `..._Part_II` | [`notes/04-guidance-nonlinear.md`](notes/04-guidance-nonlinear.md) |
| Ch8 | Path following: straight-line and orbit guidance laws (= book Ch.10) | `Ch8_Path_Following` | [`notes/04-guidance-nonlinear.md`](notes/04-guidance-nonlinear.md) |
| Ch9 | Path manager: waypoint switching, fillets, Dubins paths (= book Ch.11) | `Ch9_PathManager` | [`notes/04-guidance-nonlinear.md`](notes/04-guidance-nonlinear.md) |
| (wrap-up) | Course conclusion talk, confirms syllabus/chapter numbering | `ar23-ch10` | [`notes/04-guidance-nonlinear.md`](notes/04-guidance-nonlinear.md) |

## The big picture: how it all fits together

The course is organized around one recurring control-loop diagram (introduced in Ch0):

```
Mission objective -> Planning -> r -> (+/-) -> e -> Controller -> Model Dynamics (drone + disturbances) -> x -> Sensors (+ noise) -> y
                                  ^                                                                     |
                                  |______________________ State Estimator <------------------------------|
```

Every later chapter fills in one block of this diagram:

1. **Model Dynamics** (Ch1-Ch2): derive the general rigid-body equations of motion, then specialize them to the quadrotor. Result is the base plant used everywhere downstream:
   `ṗ = Rv`, `Ṙ = RS(ω)`, `mv̇ = -S(ω)mv + f`, `Jω̇ = -S(ω)Jω + n`, with quadrotor-specific `f = -Te₃ + mgRᵀe₃` and mixer matrix mapping 4 rotor thrusts to `(T, n)`. The quadrotor is **underactuated** (12 states, 4 inputs) but **differentially flat** in `(p, ψ)` - this flatness property is exactly what makes the cascade control design in Ch3/Ch7 possible (a desired thrust direction can always be recovered algebraically from a desired acceleration).

2. **Controller** (Ch3, Ch4, Ch7): three increasingly rigorous ways to design the same cascade (outer position loop → inner attitude loop) controller:
   - Ch3: classical SISO PD/PID + root-locus/Bode loop-shaping, treating the two loops as independent because the inner loop is designed 5-10x faster.
   - Ch4: the state-space/"modern" reformulation of the same problem - pole placement and LQR replace hand-tuned PID, and the same architecture (state feedback + observer) is shown to collapse the hierarchical structure into one full-state feedback law when desired.
   - Ch7: the nonlinear/Lyapunov-rigorous version of the *same* cascade - backstepping reproduces the Ch3 PD law but with a stability proof that doesn't require linearization, and geometric SO(3) control handles the attitude loop in a way that respects the topology of rotations (revealing the fundamental result that *global* attitude stabilization is impossible with continuous feedback).

3. **Sensors + State Estimator** (Ch5-Ch6): the accelerometer/gyro/magnetometer/GPS models feed a Kalman filter / EKF, whose gain-design math (`L = ΣCᵀN⁻¹`, Riccati equation) is the exact **dual** of the LQR gain design from Ch4 (`K = R⁻¹BᵀP`) - the course makes this LQR↔KF duality explicit. The complementary filter used in the AR.Drone labs is shown to be a fixed-gain special case of the Kalman filter.

4. **Planning / Guidance** (Ch8-Ch9): sits *above* the control loop, generating the reference `r(t)` that the controller tracks. Path following (Ch8) gives robust-to-wind vector-field laws for single line/orbit segments; the path manager (Ch9) sequences waypoints into continuous paths using half-plane switching, fillets, or Dubins paths, feeding its output as the position reference into the Ch3/Ch4/Ch7 controllers.

**Course ↔ lab mapping** (from Ch0 logistics): L1 = Modelling/identification (→ Ch1/Ch2), L2 = Estimation (→ Ch5/Ch6), L3 = Motion control (→ Ch3/Ch4/Ch7). The AR.Drone platform used in labs has **no GPS** - it substitutes sonar (height) and downward-camera vision (velocity via optical flow / corner tracking), which is why the generic Beard & McLain GPS-smoothing material (Ch6) doesn't directly transfer to the labs and is explicitly flagged as "generic background" in the sensors notes.

## Consolidated gotchas / things to double-check before relying on this material

- The quadrotor mixer matrix `M` sign/index convention (which rotor pair drives `n_x` vs `n_y`) is convention-dependent and left as an open in-class exercise for the × configuration and hexarotor case - don't trust a specific sign pattern without checking the rotor-numbering diagram in use.
- Ch3's inner-loop pole-placement example references gains `k_ω`, `k_λ` symbolically without stating numeric values on the slides.
- There's a typo in the Ch5-6 course slides: the stochastic output equation is written `y = Cy + n` but should read `y = Cx + n`.
- The LQR↔Kalman-filter duality slide leaves "(A, B̄) controllable?" as an open question mark in the source material itself.
- The geometric SO(3) attitude-control derivation (Ch7 Part II) ends mid-derivation on the exact constant needed for local (as opposed to almost-global) stability - referred to Lee/Leok/McClamroch (2010) rather than spelled out.
- Quaternions are mentioned once (Ch1) as an alternative attitude representation but never developed - if a lab or exam question needs quaternion kinematics, it's not in these slides.

See each `notes/0N-*.md` file for full equation-by-equation detail, worked examples, and chapter-specific open questions.

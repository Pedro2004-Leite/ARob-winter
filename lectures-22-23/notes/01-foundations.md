# Foundations: Course Intro, Rigid Body Dynamics, Quadrotor Modeling

Source files:
- 22_23_UAVs_Ch0_Intro_Part_I.pdf (17 pages)
- 22_23_UAVs_Ch0_Intro_Part_II.pdf (40 pages)
- 22_23_UAVs_Ch1_Rigid_Body.pdf (31 pages)
- 22_23_UAVs_Ch2_Quadrotor_Modeling.pdf (30 pages)

Course: **Unmanned Aerial Vehicles / Aeronaves Robotizadas (ARob)**, MEAer, IST Lisboa, Autumn Semester 2022/2023.
Faculty: José Raul Azinheira (jraz@dem.ist.utl.pt, responsible), Rita Cunha (rita@isr.tecnico.ulisboa.pt).
Slides are in English; occasional Portuguese course-title text ("Aeronaves Robotizadas").

## Chapter 0 - Course Introduction

### Logistics
- Contact hours: Lectures/Exercises (T) 2h/week (Tue 9h30-11h30); Laboratory (L) 1h30/week, total 12h (Wed 10h30-12h and 12h30-14h).
- Lab: groups of 3, 2 shifts of 4 groups; 3 lab assignments (L1, L2, L3), each with 4 sessions:
  1. Modelling and identification of the drone
  2. Estimation of motion variables
  3. Motion control of the drone
- 1 report per assignment, submitted before the following lab session.
- Grading: `E = max(E1, E2)` (exam, 40%), `L = 0.2*L1 + 0.4*L2 + 0.4*L3` (labs, 60%), `F = 0.4*E + 0.6*L`. Exam dates 26/1/2023 and 6/2/2023 (2022/23 edition).
- Recommended bibliography:
  - Beard & McLain, *Small Unmanned Aircraft: Theory and Practice*, Princeton University Press, 2012 (ISBN 9781400840601) - this is the main reference used for Euler-angle diagrams and later sensor/estimation chapters.
  - Astrom & Murray, *Feedback Systems*, online at Caltech CDS wiki.
  - Additional: "Flying Robots" chapter in *Handbook of Robotics* (Siciliano/Khatib, 2016); *Handbook of Unmanned Aerial Vehicles* (Valavanis/Vachtsevanos, 2015); *Introduction to UAV Systems* (Fahlstrom/Gleason, 2012).

### Syllabus (full course roadmap)
1. Introduction to UAVs - motivation, system architecture, UAV configuration types, applications.
2. Modelling - rigid body dynamic modelling; quadrotor dynamic modelling and design considerations.
3. Control systems design - quadrotor trajectory tracking control; root-locus/loop-shaping design; inner-outer loop control structure; linear systems and state-space models; linear state feedback + linear state observers, duality and separation principle, LQR design example.
4. Sensors for UAVs - accelerometers, rate gyros, pressure sensors, digital compasses, GNSS, proximity sensors.
5. State observers and Kalman filtering - position/velocity/attitude estimation including bias correction; connection to complementary filters.
6. Nonlinear systems - stability analysis and nonlinear control design; application to quadrotor trajectory tracking control.
7. Path following and path planning for UAVs - Voronoi graphs, RRTs, coverage algorithms, Dubins paths, waypoint transitions, optimization-based methods.

### Weekly schedule (as planned 2022/23, useful to map chapter numbers to weeks)
| Week | Date | Lecture | Lab |
|---|---|---|---|
|1|19-Sep|Ch0 Intro Part I|enroll|
|2|26-Sep|Ch0 Intro Part II|L1|
|3|03-Oct|Ch1 Rigid Body| |
|4|10-Oct|Ch2 Quadrotor Modeling|L1|
|5|17-Oct|Ch3 Quadrotor Control Intro|L1|
|6|24-Oct|Ch4 Modern Control Design|RL1|
|7|31-Oct|(1st November holiday)|L2|
|8-9|07/14-Nov|mid term| |
|10|21-Nov|Ch5 Sensors (Beard & McLain)|L2|
|11|28-Nov|Ch6 State Estimation (Beard & McLain)|L2|
|12|05-Dec|Q&A|RL2|
|13|12-Dec|Ch7 Intro Nonlinear Control|L3|
|14|19-Dec|Ch8 Path Following|L3|
|(Christmas holiday)| | | |
|15|02-Jan|Ch9 PathManager|L3|
|16|09-Jan|Q&A|RL3|

Exams: E1 26/1, E2 6/2.

### Why UAVs / vehicle type tradeoffs
- Example applications: search & rescue / de-mining area survey, smuggling/forest-fire detection and tracking, parcel delivery.
- UAV must: stay airborne, navigate point-to-point, follow a road/path, avoid obstacles.
- Vehicle types compared (flying-principle comparison table, 1=bad, 3=good, criteria: power cost, control cost, payload/volume, maneuverability, stationary flight, low-speed flight, vulnerability, VTOL, endurance, miniaturization, indoor usage): Airplane (total 19), **Helicopter (25, best)**, Bird/ornithopter (24), Autogiro (18), Blimp (25, tied-best but different profile - very good hover/low power, poor payload/maneuverability).
- Quadrotor rationale: 4-DOF actuation (3-DOF omnidirectional horizontal + vertical), VTOL capable, simple/robust mechanics, good controllability, small/low-cost sensors and control hardware, but limited endurance (typically 15-30 min).
- Portuguese UAV manufacturer examples given: UAVision (fixed-wing + Spyro multirotor), Tekever (AR4 hand-launched, AR5 fixed-wing, military/defense), Spinworks (fixed-wing UAV imaging, forestry & agriculture).
- Broader application domains shown: aerial inspection, precision agriculture, entertainment, and military (ISR defense, forest-fire surveillance, maritime surveillance, search & rescue, crowd control).

### Generic UAV control-loop architecture (recurring diagram throughout the course)
```
Mission objective -> Planning -> r -> (+/-) -> e -> Controller -> Model Dynamics (drone, w/ disturbances) -> x -> Sensors (w/ noise) -> y
                                  ^                                                                  |
                                  |______________________ State Estimator <-------------------------|
```
- General topics of the course map onto this loop: Modelling (L1), Estimation (L2), Control (L3), and Guidance/Path Planning/Trajectory generation.
- Control objective throughout: **track a trajectory**.

### Sensors and onboard architecture (preview, expanded later in Ch5/Ch6)
- GPS - position estimate.
- Barometer or sonar - height estimate.
- IMU (accelerometers, gyroscopes, magnetometers) - attitude estimate.
- Cameras and LiDARs - mainly obstacle avoidance and task execution (surveying, inspection).
- Onboard computers, communication systems (LTE/WiFi/RC), batteries.
- Biggest limitation to enhanced autonomy: **flight time vs. available payload** tradeoff (empirical DJI Matrice 600 Pro curve: flight time drops from ~35min at low payload to ~16min at 6kg payload).
- There is a theoretical optimum: normalized flight time vs. battery-mass fraction of total mass is a concave curve peaking around battery fraction ≈ 0.65-0.7 (derived formally in Ch2, design considerations, where the optimum is shown to be exactly 2/3).

### Research showcase (ISR/IDMEC lab video examples - context only, not testable content)
- Stabilization with wind-disturbance rejection (fan test).
- Aggressive trajectory tracking.
- Landing on a moving platform using **vision-based control**: uses optical flow / a spherical camera projection model. Key relation: image point $p_i = P_i / \|P_i\|$, giving $P_i/d \to \chi/d$ (bearing-like measurements normalized by unknown distance $d$); control objective $\chi \to 0$. Optical flow field $\phi = \iint_{\mathcal{W}^2} \dot p \, dp \to \dot\chi/d$. Assumptions: planar target, translating platform, non-aggressive maneuvers.
- Multi-vehicle **leader-following formation control**: "trailer-like" behavior for followers. In the inertial frame the followers trace translated identical paths; in the "trailer frame" they trace different, non-overlapping paths.
- **Slung-load transportation using quadrotors**: extends the state to include load position/velocity. Quadrotor-only state $(p_Q,v_Q,R_Q,\omega_Q)$, input $(T_Q,n_Q)\in\mathbb{R}^4$, flat output $(p_Q,\psi_Q)$. Quadrotor+load state adds $(p_L,v_L)$; flat output becomes $(p_L,\psi_Q)$ (i.e., you now control the *load's* position, not the quadrotor's).

## Chapter 1 - Rigid Body Kinematics and Dynamics

**Goal of the chapter**: derive the general 3-D rigid-body equations of motion that will later be specialized to the quadrotor.

### Reference frames and configuration
- A reference frame is defined by an origin and 3 orthonormal right-handed axes.
- Two frames used throughout: inertial/world frame $\{I\}$ (flat-earth model) and body-fixed frame $\{B\}$.
- Configuration of $\{B\}$ w.r.t. $\{I\}$: $({}^I p_B, {}^I_B R) \in \mathbb{R}^3 \times SO(3)$.
  - ${}^I p_B \in \mathbb{R}^3$: position of the origin of $\{B\}$ expressed in $\{I\}$ (column vector).
  - ${}^I_B R \in SO(3)$: rotation matrix from $\{B\}$ to $\{I\}$; its columns are the coordinates of $\{B\}$'s principal axes expressed in $\{I\}$: ${}^A_B R = [{}^A x_B \; {}^A y_B \; {}^A z_B]$.
- Notation convention: leading superscript = frame the vector is expressed in; sub/superscript on R = "from {B} to {A}".
- Coordinate transform: ${}^A p = {}^A_B R \, {}^B p$.

### Rotation matrices - properties (Special Orthogonal group SO(3))
- $R=[r_1\,r_2\,r_3]\in\mathbb{R}^{3\times3}$ with columns mutually orthonormal ($r_i^Tr_i=1$, $r_i^Tr_j=0$) and $\det(R)=r_1^T(r_2\times r_3)=\pm1$; right-handed ⇒ $\det(R)=1$ ("special").
- $SO(3) = \{R\in\mathbb{R}^{3\times3} : RR^T=R^TR=I_3,\ \det(R)=1\}$ is a **group** under matrix multiplication: closure, identity $I_3$, inverse $R^{-1}=R^T$ (unique), associativity — but **not commutative**.
- Inverse/relabeling: ${}^B_AR=({}^A_BR)^{-1}=({}^A_BR)^T$; composition rule ${}^A_BR\,{}^B_CR={}^A_CR$.
- Rotations are rigid isometries: preserve distances $\|R(p-q)\|=\|p-q\|$, preserve cross product $R(p\times q)=(Rp)\times(Rq)$, preserve inner product across frames.

### Skew-symmetric matrix $S(\cdot)$
- $S(\mathbf a)\mathbf b = \mathbf a\times\mathbf b$, with
$$S(\mathbf a)=\begin{bmatrix}0&-a_3&a_2\\a_3&0&-a_1\\-a_2&a_1&0\end{bmatrix},\qquad S^T=-S.$$
- $R$ preserves orientation: $RS(\mathbf p)=S(R\mathbf p)R$ (equivalently $RS(\mathbf p)R^T=S(R\mathbf p)$). This identity is used repeatedly to move $R$ across a skew term.

### Rotation representations
- **Exponential map / Euler's rotation theorem**: any rotation = angle $\theta\in[0,2\pi)$ about a unit axis $\mathbf n$: $R(\theta,\mathbf n)=e^{\theta S(\mathbf n)}=I_3+\theta S(\mathbf n)+\tfrac{\theta^2}{2}S(\mathbf n)^2+\dots$
- **Rodrigues' formula** (closed form of the series): $R(\theta,\mathbf n)=I_3+\sin\theta\,S(\mathbf n)+(1-\cos\theta)S(\mathbf n)^2 = \mathbf n\mathbf n^T+\sin\theta\,S(\mathbf n)-\cos\theta\,S(\mathbf n)^2$.
  Geometric decomposition $\mathbf p=\mathbf p_\parallel+\mathbf p_\perp$ (parallel/perpendicular to $\mathbf n$) gives $R\mathbf p=\mathbf p_\parallel+\sin\theta\,S(\mathbf n)\mathbf p+\cos\theta\,\mathbf p_\perp$.
- **Unit quaternions**: $q=(\cos(\theta/2),\sin(\theta/2)\mathbf n)$ - mentioned as an alternative, singularity-free (double-cover) representation; not derived in depth here.
- **Euler angles (Z-Y-X / yaw-pitch-roll)**: any rotation decomposes into 3 elementary rotations. IST convention: $\lambda=[\phi\ \theta\ \psi]^T\in\mathbb{R}^3$ (roll $\phi$, pitch $\theta$, yaw $\psi$), and
$${}^I_BR=R(\lambda)=R_z(\psi)R_y(\theta)R_x(\phi).$$
  Elementary matrices:
$$R_z(\psi)=\begin{bmatrix}c_\psi&-s_\psi&0\\s_\psi&c_\psi&0\\0&0&1\end{bmatrix},\ R_y(\theta)=\begin{bmatrix}c_\theta&0&s_\theta\\0&1&0\\-s_\theta&0&c_\theta\end{bmatrix},\ R_x(\phi)=\begin{bmatrix}1&0&0\\0&c_\phi&-s_\phi\\0&s_\phi&c_\phi\end{bmatrix}.$$
  Intermediate frames: $\{B\}\to\{B_1\}$ roll about body x-axis; $\{B_1\}\to\{B_2\}$ pitch about $\{B_1\}$'s y-axis; $\{B_2\}\to\{B_3\}=\{I\}$ (same orientation, different origin) yaw about $\{B_2\}$'s z-axis.
  **Key caveat**: any 3-parameter attitude representation necessarily has singularities (gimbal lock) - stated explicitly in the slides as unavoidable for Euler angles.

### Rigid-body kinematics
- Linear motion (velocity expressed in inertial frame): $\dot{{}^Ip_B}={}^Iv_B$.
- Angular motion (angular velocity in inertial frame): ${}^I_B\dot R = S({}^I\omega_B)\,{}^I_BR$.
  - *Derivation sketch*: columns $r_i$ of $R$ satisfy $r_i^Tr_i=1\Rightarrow r_i^T\dot r_i=0$, so $\dot r_i = {}^I\omega_B\times r_i$ for some common $\omega$ (this is exactly what defines angular velocity); stacking columns gives $[\dot r_1\,\dot r_2\,\dot r_3]=S({}^I\omega_B)[r_1\,r_2\,r_3]$.
- Rewriting with angular velocity expressed in the **body** frame ($\omega:={}^B_IR\,{}^I\omega_B$, i.e. $\omega = R^T\,{}^I\omega_B$) gives the more commonly used body-rate form:
$$\dot R = R\,S(\omega).$$
  Both are consistent: $RR^T=I \Rightarrow \dot R R^T = S({}^I\omega_B)$ (inertial-frame skew) while $R^T\dot R = S(\omega)$ (body-frame skew).
- **Kinematics with velocities in body frame** (the form used everywhere afterward for the quadrotor):
$$\dot p = Rv,\qquad \dot R = RS(\omega),$$
  where $v$ is linear velocity expressed in $\{B\}$ and $\omega$ is angular velocity expressed in $\{B\}$.
- With position also expressed in body frame ${}^Bp:={}^B_IRp$: ${}^B\dot p = -S(\omega)\,{}^Bp+v$.
- **Worked exercise (circular motion)**: given $p(t)=r[\cos\psi,\sin\psi,0]^T$, $R(t)=R_z(\psi(t))$, one derives $v=R^T\dot p = S(\omega)p_0 = [0,\ r\dot\psi,\ 0]^T$ and ${}^I\omega_B = S^{-1}(\dot R R^T) = [0,0,\dot\psi]^T$, i.e. $\omega=R\omega$ in this planar case (illustrates how to move between inertial/body representations of $\omega$).
- **Attitude kinematics via Euler-angle rates**: $\dot\lambda = Q(\lambda)\,\omega$ where
$$Q(\lambda)=\begin{bmatrix}1&\sin\phi\tan\theta&\cos\phi\tan\theta\\0&\cos\phi&-\sin\phi\\0&\sin\phi/\cos\theta&\cos\phi/\cos\theta\end{bmatrix}.$$
  Its inverse: $\omega = Q^{-1}(\lambda)\dot\lambda = \begin{bmatrix}1&0&-s_\theta\\0&c_\phi&s_\phi c_\theta\\0&-s_\phi&c_\phi c_\theta\end{bmatrix}\dot\lambda$.
  **Important note explicitly called out**: $Q$ is *not* a rotation matrix (not orthogonal) — it's the Jacobian relating Euler-angle rates to body angular velocity, and it is singular at $\theta=\pm\pi/2$ (matches the gimbal-lock caveat above; $\cos\theta$ appears in denominators of $Q$).

### Rigid-body dynamics
- A particle of the body at body-frame offset $r$: $q = p + Rr$ (inertial position), $\dot q = \dot p + RS(\omega)r$.
- **Mass** $m=\int \rho(r)\,dV$; **center of mass** $\bar q = \frac1m\int q(r)\rho(r)\,dV$. If $\{B\}$'s origin is placed at the center of mass, $\int r\rho(r)\,dV = 0$.
- **Linear momentum**: $\int \dot q(r)\rho(r)\,dV = m\dot p$.
- **Tensor of inertia** (about center of mass, expressed in $\{B\}$): $J = -\int \rho(r)S(r)^2\,dV = \int\rho(r)(r^TrI_3-rr^T)\,dV$, giving the standard symmetric matrix with entries $J_{xx}=\int\rho(r)(r_y^2+r_z^2)dV$, off-diagonals $J_{xy}=-\int\rho(r) r_xr_y\,dV$, etc.
  - Angular momentum about the center of mass: $\int\rho(r)(q(r)-\bar q)\times\dot q(r)\,dV = RJ\omega$.
  - Principal axes = eigenvectors of $J$; rotating about a principal axis keeps the angular-momentum direction fixed.
  - Worked example (solid cylinder, radius $r$, height $h$): $J_{zz}=\tfrac12mr^2$, $J_{xx}=J_{yy}=\tfrac1{12}m(3r^2+h^2)$, off-diagonals zero (axes of symmetry = principal axes).
- **Newton-Euler equations of motion** (derived from conservation of linear/angular momentum in the inertial frame, then transformed into the body frame using $\dot R = RS(\omega)$):
  - Translational: $\frac{d}{dt}(m\dot p)={}^If \Rightarrow m\dot v + mS(\omega)v = f$ with $f:=R^T\,{}^If$ (forces expressed in body frame), i.e.
  $$m\dot v = -S(\omega)mv+f.$$
  - Rotational: $\frac{d}{dt}(RJ\omega)={}^I\tau \Rightarrow J\dot\omega+S(\omega)J\omega=\tau$ with $\tau:=R^T\,{}^I\tau$, i.e.
  $$J\dot\omega = -S(\omega)J\omega+n \quad(n \text{ used later in place of } \tau\text{ for the applied moment}).$$

### Final result of Ch1 - the general 3-D rigid-body model (this is the base model reused everywhere later)
$$
\dot p = Rv,\qquad \dot R = RS(\omega),\qquad m\dot v=-S(\omega)mv+f,\qquad J\dot\omega=-S(\omega)J\omega+n,
$$
with state $X=(p,R,v,\omega)$ and input $U=(f,\tau)$ [force and moment, expressed in body frame].

## Chapter 2 - Quadrotor Dynamic Modeling

**Goal**: specialize the rigid-body model of Ch1 to a quadrotor, i.e. determine the actual input $u$ and how forces $f$/moments $n$ are generated as $f=f(p,R,v,\omega,u)$, $n=n(p,R,v,\omega,u)$.

### Actuation chain (per rotor)
Each rotor = electric motor + propeller: input voltage $u_i$ → motor dynamics → spin rate $\omega_i$ → propeller aerodynamics → thrust $T_i$ and reaction torque $Q_i$.

**DC motor model**:
- Electrical: $L\dot i_i(t) = u_i(t) - Ri_i(t) - k_b\omega_i(t)$ ($k_b\omega_i$ = back-EMF, proportional to spin speed).
- Mechanical: $I\dot\omega_i(t) = k_ti_i(t) - \tau_i^{load}(t)$ (generated torque proportional to coil current $i_i$, minus load torque from the propeller).

**Propeller aerodynamics (blade-element theory)**:
- Lift/drag at a blade element at radius $r$: $l(r)=\tfrac{\rho}{2}v_{air}^2 c\,c_l$, $d(r)=\tfrac{\rho}{2}v_{air}^2c\,c_d$, with $v_{air}=[\omega_i r,\ v_{ind}]^T$ (rotational speed component + induced/inflow velocity), $c$ = chord, $c_l\approx a_0\alpha$ (lift coeff. ≈ linear in angle of attack $\alpha$) drives thrust, $c_d$ (drag coeff.) drives torque.
- Integrating over the blade span and summing over $N_b$ blades gives closed forms quadratic in spin speed:
$$T_i = N_b\frac{\rho}{2}R^3a_0\Big(\frac{\theta}{3}-\frac{v_{ind}}{2}\Big)\omega_i^2 = c_T\omega_i^2,\qquad Q_i = c_Q\omega_i^2.$$
- $c_T,c_Q$ (thrust/torque constants) are typically identified experimentally via static thrust-stand tests with different payloads (example: APC propeller datasheets giving RPM→Thrust/Power/Torque/$C_P$/$C_T$ tables; non-dimensional coefficients $C_T=T/(\rho n^2D^4)$, $C_P=P/(\rho n^3D^5)$ as functions of advance ratio $J=V/(nD)=2\pi V/(\omega D)$ for forward-flight/airspeed effects).
- Neglected higher-order effects mentioned: momentum-theory slipstream contraction (upstream/disk/downstream sections with $p_1<p_2$, $v_\infty<v<v_d$), and **ground effect** at low height.

### Quadrotor input forces and moments (near-hover, negligible air velocity)
- Standard **+ configuration** with two pairs of counter-rotating rotors (1&3 vs 2&4, or similar) cancels reaction-torque yaw bias at equal speeds. Total thrust and body-frame moments:
$$T=\sum_iT_i,\qquad n_x=l(T_2-T_4),\qquad n_y=l(T_1-T_3),\qquad n_z=\frac{c_Q}{c_T}(T_1-T_2+T_3-T_4),$$
  i.e. in matrix form $[T,n_x,n_y,n_z]^T = M\,[T_1,T_2,T_3,T_4]^T$ with the **mixer matrix**
$$M=\begin{bmatrix}1&1&1&1\\0&l&0&-l\\l&0&-l&0\\c&-c&c&-c\end{bmatrix},\quad c=c_Q/c_T.$$
  ($l$ = arm length; the exact row/sign pattern depends on rotor numbering/geometry — an in-class exercise asks students to redo this for the × configuration and for a hexarotor, where $M$ becomes non-square/over-actuated: "6 inputs... but M=?" is posed as an open exercise, hinting at pseudo-inverse allocation.)
- With $T,n$ expressed in $\{B\}$ and near-hover aerodynamics negligible, plug directly into the Ch1 rigid-body dynamics:
$$f = -Te_3+mgR^Te_3,\qquad n=[n_x,n_y,n_z]^T.$$

### Full quadrotor dynamic model
$$
\dot p = Rv,\quad m\dot v = -S(\omega)mv - Te_3+mgR^Te_3,\quad \dot R=RS(\omega),\quad J\dot\omega=-S(\omega)J\omega+n,
$$
equivalently in inertial-frame form $m\ddot p = -TRe_3+mge_3$.
Compact form $\dot x=f(x,u)$ with $x=(p,v,R,\omega)$, $u=(T,n_x,n_y,n_z)$.
**Underactuated system: 12 states, 4 inputs** — can only directly prescribe 4 DOF, chosen as $y=(p,\psi)\in\mathbb{R}^4$ (3D position + yaw). Block-diagram causality: $(T,n)\to$ Angular Dynamics $\to r_3$ (thrust direction unit vector) $\to$ Translation Dynamics $\to (p,v)$, with $(R,\omega)$ fed back.
This is explicitly framed as *the* control-design model: no wind, no aerodynamics beyond thrust/reaction-torque, ideal actuation, valid for slow flight.

### Differential flatness of the quadrotor
- A system is **flat** if state and input $(x(t),u(t))$ can be written purely as functions of a flat output $y(t)$ and a finite number of its time derivatives, and $\dim(y)=\dim(u)$.
- For the quadrotor, $y=(p,\psi)\in\mathbb{R}^4$ (matches $u=(T,n_x,n_y,n_z)\in\mathbb{R}^4$), and it can be shown $(x,u)=f(p,\dot p,\ddot p,p^{(3)},p^{(4)},\psi,\dot\psi,\ddot\psi)$.
- **Exercise worked in class**: from $m\ddot p = -TRe_3+mge_3$, define $f_T=-TRe_3$ (thrust force) and $f_g=mge_3$ (gravity); then $m\ddot p=f_T+f_g$, so $T$ and the thrust *direction* $Re_3$ are recovered algebraically from the desired acceleration $\ddot p$ and yaw $\psi$ (given $\psi$, roll/pitch are solved from $R_y(\theta)R_x(\phi)e_3 = [\cos\phi\sin\theta,\ -\sin\phi,\ \cos\phi\cos\theta]^T$ matched against $-m\ddot p - f_g$ direction). This is the standard "differential flatness" trick used in most quadrotor trajectory-tracking controllers (e.g. Mellinger & Kumar-style approaches), though that paper isn't cited by name in these slides.
- Special case (hover): $\ddot p=0 \Rightarrow f_T+f_g=0$ (thrust exactly cancels gravity, vertical).
- Circular-motion exercise: given $p(t)=r[\cos\psi,\sin\psi,0]^T$ with $\dot\psi=c$ constant, students derive roll $\phi$ and pitch $\theta$ from the same force-balance identity — illustrating that even for a simple circular flat trajectory, non-trivial roll/pitch are required (banking into the turn).

### Design considerations (SWaP tradeoffs)
- **Thrust/weight ratio**: $T_{hover}=\tfrac14mg$ per rotor at hover; $T_{max}=\tfrac{c_T}{c_Q}Q_{max}$ (limited by max motor torque); ratio $T_{max}/T_{hover}=4\frac{c_T}{c_Q}\frac{Q_{max}}{mg}$. Typically **not greater than ~2**; lower values → slower response, less control authority. Measured examples: Intel Aero drone ratio ≈1.36 (arctan-shaped thrust curve vs input), Snapdragon-based platform ≈3.2 (much more linear thrust curve).
- **Flight time**: mechanical power per rotor $p_{motor}=\omega Q=c_Q\omega^3=c_Q(T/c_T)^{3/2}$, i.e. **power scales as $T^{3/2}$**. Flight time $t_{flight}=\dfrac{E_{batt}}{4p_{motor}/\eta+p_{payload}}$ with motor efficiency $\eta\approx70\%$; neglecting payload, $t_{flight}\propto m_{batt}/m^{3/2}$.
  - Optimal battery mass fraction $\rho=m_{batt}/m$: with $t_{flight}\propto \rho\sqrt{1-\rho}/m_0$ (where $m_0=m-m_{batt}$ is the fixed airframe mass), the flight-time-vs-battery-fraction curve is concave and maximized at $\rho^*=2/3$ — i.e. **about two-thirds of total mass should be battery** for max endurance. (This is the same normalized curve teased in Ch0.)
  - Battery chemistry tradeoff (specific power vs specific energy): supercapacitors/NiCd/NiMH favor high power (fast discharge, low energy density), Lithium-ion/Li-Polymer favor high energy density (longer flight time) — standard Ragone-plot style comparison.
- **Agility vs size**: characteristic length $d$; mass $m\propto d^3$, inertia $J\propto d^5$. Rotor tip-speed scaling gives $\omega_i\propto 1/\sqrt d$ (Mach scaling) or $\omega_i\propto 1/d$ (Froude scaling); since $T\propto\omega_i^2d^4$ and $J\propto d^5$, angular acceleration $\dot\omega\propto Td/J\propto \omega_i^2/d$, which reduces (with Froude scaling) to $\dot\omega\propto1/d^2$ or (with the $\omega_i^2 d^4/d^5$ grouping shown) $\dot\omega=\omega_i^2$ scaling directly. **Conclusion stated explicitly in the slides: reducing size increases agility.** (Source cited: Mahony, Kumar, Corke, "Multirotor Aerial Vehicles," IEEE Robotics & Automation Magazine, vol 19, no 3, 2012 — a good canonical reference for this whole chapter's design-consideration section.)

## Key equations reference

- Rotation composition / inverse: ${}^A_BR\,{}^B_CR={}^A_CR$; $({}^A_BR)^{-1}=({}^A_BR)^T$.
- Skew-symmetric cross product: $S(a)b=a\times b$; $RS(p)R^T=S(Rp)$.
- Rodrigues' formula: $R(\theta,n)=I+\sin\theta S(n)+(1-\cos\theta)S(n)^2$.
- Euler ZYX: ${}^I_BR=R_z(\psi)R_y(\theta)R_x(\phi)$; rate map $\dot\lambda=Q(\lambda)\omega$ (singular at $\theta=\pm90°$).
- Rigid-body kinematics (body-frame velocities): $\dot p=Rv$, $\dot R=RS(\omega)$.
- Rigid-body dynamics (Newton-Euler, body frame): $m\dot v=-S(\omega)mv+f$, $J\dot\omega=-S(\omega)J\omega+n$.
- Quadrotor mixer: $[T,n_x,n_y,n_z]^T=M[T_1,T_2,T_3,T_4]^T$, $T_i=c_T\omega_i^2$, $Q_i=c_Q\omega_i^2(-1)^{i+1}$.
- Quadrotor translational dynamics with thrust input: $m\ddot p=-TRe_3+mge_3$.
- Underactuation: 12 states $(p,v,R,\omega)$, 4 inputs $(T,n_x,n_y,n_z)$; flat output $y=(p,\psi)$.
- Thrust/weight ratio: $T_{max}/T_{hover}=4(c_T/c_Q)(Q_{max}/mg)$, practical max ≈2.
- Power/endurance: $p_{motor}\propto T^{3/2}$; optimal battery mass fraction $\rho^*=2/3$.
- Agility scaling: $m\propto d^3$, $J\propto d^5$; smaller $d$ → higher angular acceleration → more agile.

## Open questions / unclear points

- The exact sign/index convention of the mixer matrix $M$ (which rotor pairs give $n_x$ vs $n_y$) depends on a rotor-numbering diagram that differs between the "quadrotor forces and moments" slide (Ch0 Part II, p.13) and the "+ / × configuration" exercise slide (Ch2, p.13) — the course explicitly leaves the × configuration and hexarotor allocation as unsolved in-class exercises, so no canonical answer is in the slides themselves.
- The propeller aerodynamic derivation (Ch2 pp.6-7) elides the induced-velocity model $v_{ind}$ (momentum theory) needed to get from blade-element integrals to the closed-form $c_T,c_Q$ — likely covered by the cited textbook (Beard & McLain) rather than fully re-derived on slides.
- Quaternions are mentioned once (Ch1 p.12) as "an alternative representation" but never developed further in these four files (no quaternion kinematics/composition formulas given) — likely deferred to a lab handout or not used at all in this edition of the course.
- The DC motor's electrical time constant $L/R$ vs mechanical time constant is not compared quantitatively; slides state the equations but do not discuss which dynamics dominate (relevant for later control design in Ch3/Ch4).

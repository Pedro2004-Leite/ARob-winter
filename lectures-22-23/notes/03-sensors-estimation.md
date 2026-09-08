# Sensors and State Estimation

Source files:
- `22_23_UAVs_Ch5-6_Sensors_and_State_Estimation_1.pdf` (28 pages) - course lecture, Técnico Lisboa, MEAer UAVs, Autumn 2022/2023, by Rita Cunha / J.R. Azinheira
- `22_23_UAVs_Ch5_Sensors_Beard_McLain.pdf` (29 pages) - reference chapter (Ch. 7 of the source textbook)
- `22_23_UAVs_Ch6_State_Estimation_Beard_McLain.pdf` (47 pages) - reference chapter (Ch. 8 of the source textbook)

Both reference PDFs are slide adaptations of Randal Beard & Timothy McLain, *Small Unmanned Aircraft: Theory and Practice*, Princeton University Press, 2012 (Ch. 7 "Sensors" and Ch. 8 "State Estimation"). All pages processed (28/28, 29/29, 47/47).

## Course lecture (Ch5-6 Sensors and State Estimation)

The lecture is built directly on top of the Beard & McLain slide deck and adds annotations specific to the AR.Drone platform used in the course labs, plus a worked observability example and a treatment of complementary filters.

### AR.Drone-specific sensors (course addition, not in the textbook)
- The AR.Drone has **no GPS**. Its extra sensors compared to the generic textbook list are:
  - **Sonar** (ultrasonic): 40 kHz resonance, range up to 6 m, sampled at 25 Hz. Used to estimate height/vertical displacement and depth of the scene seen by the vertical (downward-facing) camera.
  - **Vision** (downward camera, hovering flight): speed estimation via two alternative algorithms:
    1. Multi-resolution optical-flow scheme over the whole image.
    2. "Corner tracking" - track displacement of a set of interest points (trackers).

### Accelerometers - basic principle (spring-mass-damper derivation)
Derived from a proof mass on a spring/damper inside a casing:
- `h`: inertial height of casing; `h1 = h + δ`: inertial height of proof mass; `δ`: relative displacement (what is actually measured).
- Newton's 2nd law on the proof mass: `m·ḧ1 = -k(h1-h) - β(ḣ1-ḣ) - mg`.
- Substituting `z = -h`, this becomes the accelerometer 2nd-order dynamics: `m·δ̈ + β·δ̇ + k·δ = m(z̈ - g)`.
- Laplace/transfer function: `D(s) = 1/(s² + (β/m)s + k/m) · (Az(s) - g/s)`.
- Low-frequency approximation: `δ ≈ (m/k)(z̈ - g)` - i.e. below the sensor's natural frequency, deflection is proportional to (acceleration minus gravity).
- For a 3-axis accelerometer rigidly attached to the vehicle body frame {B}: `δ ∝ Rᵀ(p̈ - g·e3) = S(ω)v + v̇ - g·Rᵀe3` (S(ω) is the skew-symmetric matrix of the angular velocity - same notation as the rigid-body chapter).
- Near hover (small p̈, small ω): `δ ∝ -g·Rᵀe3 = -g·[-sinθ, cosθ·sinφ, cosθ·cosφ]ᵀ`.
- **Key takeaway**: accelerometers alone give roll/pitch information only near hover / unaccelerated flight - they are "typically used for attitude estimation in combination with other sensors" (fused with gyros/GPS).

### Rate gyros - basic principle (Coriolis / MEMS vibrating mass)
- A vibrating proof mass with velocity `v` on a rotating body experiences a Coriolis acceleration `aC`. For a package rotating with `ω = [0,0,Ω]ᵀ` and vibration velocity `v = [vx,0,0]ᵀ`, `p̈1 = -[0, 2Ω·vx, 0]ᵀ + Rᵀp̈` - the second term is proportional to the angular rate Ω being measured. (Detailed kinematic derivation: `p1 = Rᵀp`, `ṗ1 = -S(ω)Rᵀp + Rᵀṗ`, `p̈1 = -S(ω̇)Rᵀp + S(ω)²Rᵀp - 2S(ω)Rᵀṗ + Rᵀp̈`.)

### Magnetometers - heading recovery
- `Im0 = [cosδ·cosγ, sinδ·cosγ, sinγ]ᵀ` is the known magnetic field vector in the inertial frame (δ = declination, γ = inclination, location-dependent).
- Body-frame reading: `Bm0 = BR_I · Im0`, and roll/pitch (φ,θ) are assumed known from other sensors.
- Compensating for roll and pitch: `B1m0 = Ry(θ)Rx(φ)·Bm0 = Rz(-ψ)·Im0`, which reduces to a 2D rotation by `(ψ-δ)` in the horizontal plane.
- Closed form for heading: `(ψ - δ) = atan2(-B1my, B1mx)`.

### Kalman filter (course treatment)
- Continuous-time stochastic LTI model: `ẋ = Ax + Bu + w`, `y = Cy + n` (should read `y = Cx + n`), with `w`, `n` zero-mean white noise, `E[w(t)w(τ)ᵀ] = δ(t-τ)W`, `E[n(t)n(τ)ᵀ] = δ(t-τ)N`.
- Objective: minimize `J = lim_{t→∞} E[x̃(t)ᵀx̃(t)] = lim tr[Σ(t)]`, with error covariance `Σ(t) = E[x̃(t)x̃(t)ᵀ]`.
- Observer form: `x̂̇ = Ax̂ + Bu - L(y - Cx̂)`, gain `L = Σ∞·Cᵀ·N⁻¹`, where `Σ∞` solves the steady-state Riccati-like equation `(A-LC)Σ∞ + Σ∞(Aᵀ-CᵀLᵀ) + W + LNLᵀ = 0`.
- **Duality LQR ↔ Kalman filter** made explicit side by side:
  - LQR: `ẋ=Ax+Bu`, `u=-Kx`, cost `J=∫(xᵀQx+uᵀRu)dt`, `Q=CᵀC≥0, R>0`, gain `K=R⁻¹BᵀP`, `PA+AᵀP+Q-PBR⁻¹BᵀP=0`. Requires `(A,B)` stabilizable, `(A,C)` detectable.
  - KF: `ẋ=Ax+Bu+B̄w`, `y=Cx+n`, gain `L=ΣCᵀN⁻¹`, `ΣAᵀ+AΣ+W-ΣCᵀN⁻¹CΣ=0`. Requires `(A,C)` detectable, `(A,B̄)` controllable (marked "?" as an open point in slides, i.e. worth double-checking case by case).

### Worked example: roll-angle estimation for a fixed-wing aircraft
Assumptions: coordinated turn, no wind, no sideslip. From force balance: `Flift·cosφ = mg`, `Flift·sinφ = mV²/R = mV·ψ̇`, giving `tanφ = Vψ̇/g`, i.e. `ψ̇ = (g/V)tanφ`.

The course walks through **three progressively richer Kalman filter setups** for this example, illustrating the role of observability:

1. **Rate-gyro only, state = [φ, bp, br]**, output `y = rm` (yaw-rate measurement only): model
   `φ̇ = pm - bp - np`, `ḃp = n1`, `ḃr = n2`, output `y = rm = (g/V)φ + br + nr`.
   → **Not observable** (pair (A,C) fails the observability test) - additional sensor is needed. This is a nice didactic "what goes wrong" case.

2. **Rate gyros + accelerometer, state = [φ, p, bp, br]**, outputs `y = [rm, pm, φm]ᵀ` (adds roll-rate measurement `pm` and a (noisy) direct roll measurement `φm` from the accelerometer, derived from `am ≈ -g[0, sinφ, cosφ]ᵀ` under the coordinated-turn assumption, i.e. `am = -g[0,0,√(1+tan²φ)]` after substitution).
   → **Observable** - this is the version that actually works.

3. A **second alternative** with the same state/output structure but using the dynamic model for the time evolution of `φ̇` directly (equivalent result, different parametrization).

### Complementary filters
- **Context**: different sensors are reliable over different (complementary) frequency bands - e.g. GPS velocity is accurate at low frequency but slow/noisy at high frequency, while integrated accelerometer data is accurate at high frequency but drifts (bias accumulates) at low frequency.
- **Structure**: `x̂ = GL(s)·xmL + GH(s)·xmH`, with `GL(s) = l/(s+l)` (low-pass) and `GH(s) = s/(s+l)` (high-pass), satisfying `GL(s) + GH(s) = 1` for all s.
- **Worked example - altitude-rate estimation**: `ḣGPS = ḣ(t) + n1(t)` (low-frequency-good) and `ḧACC` integrated once to `ḣACC = ḣ(t) + n2(t)` (high-frequency-good). Combined estimate:
  `Ĥ(s) = l/(s+l)·Ḣ1(s) + s/(s+l)·Ḣ2(s) = Ḣ(s) + l/(s+l)·N1(s) + s/(s+l)·N2(s)`
  → keeps the whole true signal, attenuates N1 at high frequency and N2 at low frequency.
- **Connection to Kalman filter**: shown to be an exact special case. With `x = ḣ`, `u = ḧACC`, `y = ḣGPS`, the KF observer `x̂̇ = Ax̂+Bu+L(y-Cx̂)` reduces to exactly the complementary filter structure - i.e. the complementary filter is a Kalman filter with fixed (non-adaptive) gain `l`.
- **More advanced case shown**: a state observer for **roll angle + roll-rate bias**, `x=[φ,bp]`, `u=pm`, `y=φm`, producing a second-order complementary filter `Φ̂(s) = GL(s)Φm(s) + GH(s)Pm(s)/s` with `GL(s) = (l1s+l2)/(s²+l1s+l2)`.

## Textbook reference - Sensors (Beard & McLain Ch5, "Chapter 7" in the original book numbering)

Covers, in order: accelerometers, rate gyros, pressure sensors (absolute + differential), magnetometers/compasses, GPS.

### Accelerometers (MEMS)
- Physical model: proof mass `m` on spring `k`, `mẍ + kx = k(y-x)` where `y` is casing displacement, `x` proof-mass deflection. Laplace: `X(s)/Y(s) = 1/((m/k)s²+1)`, equivalently in terms of acceleration `AX(s)/AY(s) = 1/((m/k)s²+1)`.
- Model with bias + noise: `Υaccel = kaccel·a + βaccel + η'accel`.
- **Key conceptual point (repeated, "tricky concept")**: an accelerometer measures **specific force**, i.e. `a = (1/m)(Ftotal - Fgravity)`, or equivalently `ameasured = (1/m)(ΣFnon-gravitational)`. **Accelerometers do not measure gravity** - a stationary accelerometer on a table reads +g upward (reaction force), not zero, because the proof mass and casing are NOT both free-falling together in that scenario (unlike free fall, where both experience gravity identically and the reading is zero).
- For a fixed-wing aircraft: `ameasured = (1/m)(Flift + Fdrag + Fthrust)` (gravity cancels analytically).
- Full body-frame accelerometer equations (from Ch.3 rigid-body dynamics `m(dv/dt_b + ω_{b/i}×v) = Ftotal`):
  - `ax = u̇ + qw - rv + g·sinθ`
  - `ay = v̇ + ru - pw - g·cosθ·sinφ`
  - `az = ẇ + pv - qu - g·cosθ·cosφ`
  With sensor noise added: `y_accel,i = (kinematic term) + η_accel,i`.
- Also expressed in terms of aerodynamic coefficients (`CX, CY, CZ`, angle of attack `α`, sideslip `β`, control surfaces `δa, δe, δr`, motor throttle `δt`) - the full nonlinear aerodynamic force model, useful for high-fidelity simulation.
- Compact "specific force" form: `y_accel,x = fx/m + g·sinθ + η`, etc. (fx,fy,fz = non-gravitational body forces).

### MEMS rate gyro
- Physical principle: a point translating on a rotating rigid body experiences Coriolis acceleration `aC = 2Ω×v`. A resonating proof mass with `|v| = A·ωn·sin(ωn·t)` yields a sensor output `Vgyro = kC|aC| = 2kC·Ω·|Aωn·sin(ωnt)| → 2kC·A·ωn·Ω = KC·Ω` (proportional to angular rate).
- Model: `Υgyro = kgyro·Ω + βgyro (bias, from manufacturing/drift) + η'gyro` (zero-mean Gaussian noise). Calibrated: `y_gyro,x = p + βgyro,x + ηgyro,x` (and similarly for q,r).

### Pressure sensors
- Piezoresistive diaphragm: external pressure deflects a thin diaphragm, measured via a doped piezoresistor bridge.
- **Absolute pressure → altitude**: hydrostatics `P2-P1 = ρg(z2-z1)`. Using ground as reference: `P - Pground = -ρg·h_AGL`. Below 11 km, barometric formula with temperature lapse rate accounts for density-with-altitude: `P = P0·[T0/(T0+L0·h_ASL)]^(gM/RL0)`, with `L0 = -0.0065 K/m`. Simplified constant-density sensor model: `y_abs_pres = ρg·h_AGL + βabs + ηabs`. (Slides show that the constant-density approximation diverges badly from the ideal-gas-law barometric formula above ~2000 m, but is fine below ~500 m.)
- **Differential (dynamic) pressure → airspeed**: Bernoulli, `Pt = Ps + ρVa²/2` ⇒ `ρVa²/2 = Pt - Ps`. Sensor model: `y_diff_pres = ρVa²/2 + βdiff + ηdiff`. Implemented via a pitot-static tube (measures total pressure Pt at the tube tip and static pressure Ps from side ports, feeding a differential-pressure diaphragm sensor).

### Magnetometers / digital compasses
- Heading = declination + magnetic heading: `ψ = δ + ψm`.
- `m0v1 = Rbv1(φ,θ)·m0b = Rv2v1(θ)·Rbv2(φ)·m0b`, explicit rotation-matrix expansion given in slides using (cθ,sθ,cφ,sφ) shorthand.
- Closed form: `ψm = -atan2(m0y^v1, m0x^v1)`.
- Magnetic **inclination** (dip angle): angle of the field vector below horizontal; positive = pointing into the Earth. Both declination and inclination vary geographically (world maps shown) - relevant for accurate heading estimation depending on operating location.

### GPS
- Constellation: 24 satellites, altitude ~20,180 km; any point on Earth sees ≥4 at all times. 4 pseudorange measurements needed (3 position unknowns + 1 receiver clock-offset unknown) → 4 nonlinear equations in 4 unknowns (lat, lon, alt, clock offset).
- **Error sources**: ephemeris data, satellite clock drift, ionospheric delay, tropospheric delay, multipath, receiver measurement noise. Rule of thumb: 10 ns timing error → ~3 m pseudorange error.
- **UERE** (User Equivalent Range Error) combines bias + random components per source (table given: ephemeris 2.1/0.0, satellite clock 2.0/0.7, ionosphere 4.0/0.5, troposphere 0.5/0.5, multipath 1.0/1.0, receiver 0.5/0.2 → total UERE rms ≈ 5.3 m, filtered ≈ 5.1 m).
- **DOP** (Dilution of Precision): satellite geometry effect on position accuracy; close-together satellites → high DOP (bad), spread-out → low DOP (good). Nominal HDOP=1.3, VDOP=1.8.
- Total error: `E_n-e,rms = HDOP × UERE_rms` (≈6.6 m), `E_h,rms = VDOP × UERE_rms` (≈9.2 m).
- **Gauss-Markov error model** (Rankin) for GPS error time-evolution: `ν[n+1] = e^(-kGPS·Ts)·ν[n] + ηGPS[n]`, first-order correlated random walk. Parameters given per axis (North/East/Altitude): nominal bias/random 1-σ errors, `ηGPS` std dev, correlation time `1/kGPS = 1100 s`, `Ts = 1.0 s`. Measurement model: `yGPS,n[n] = pn[n] + νn[n]` (similarly for east, altitude with sign flip since altitude = -pd).

## Textbook reference - State Estimation (Beard & McLain Ch6, "Chapter 8" in the original book numbering)

### Motivation and architecture
- Standard UAV autopilot architecture chain: **Path planner → Path manager → Path following → Autopilot → Unmanned Vehicle**, all fed by a **State estimator** block that consumes on-board sensor data and outputs `x̂(t)` to every other block.
- Not all states are directly measured. Table of what's measured vs. estimated:
  - `pn, pe`: measured (GPS) and smoothed.
  - `pd` (altitude): measured (absolute pressure, GPS).
  - `u, v, w` (body velocities): estimated with EKF.
  - `φ, θ, ψ` (attitude): estimated with EKF.
  - `p, q, r` (angular rates): measured directly (rate gyro).

### Sensor-model inversion (cheap "quasi-estimation" before resorting to filtering)
- Angular rates: `p̂ = LPF(y_gyro,x)` etc. (just low-pass filter the raw gyro, correcting scaling).
- Altitude: `ĥ = LPF(y_static_pres)/(ρg)`.
- Airspeed: `V̂a = sqrt((2/ρ)·LPF(y_diff_pres))`.
- Roll/pitch under **steady, level (unaccelerated) flight assumption** (`u̇=v̇=ẇ=p=q=r=0`): the accelerometer equations reduce to `LPF(y_accel,x)=g sinθ`, `LPF(y_accel,y)=-g cosθ sinφ`, `LPF(y_accel,z)=-g cosθ cosφ`, solved as:
  `φ̂accel = atan⁻¹(LPF(y_accel,y)/LPF(y_accel,z))`, `θ̂accel = sin⁻¹(LPF(y_accel,x)/g)`.
  Demonstrated in simulation to work reasonably for p,q,r,h,Va, but to fail badly for φ,θ during maneuvers (the steady-flight assumption breaks) - motivating the EKF.
- GPS-derived states (`pn,pe,χ,Vg`) can also be obtained by direct LPF inversion, but GPS update rate (~1 Hz) is too slow, leaving gaps to fill between updates → motivates GPS smoothing (below).

### Dynamic observer theory (build-up to Kalman filter)
- LTI continuous observer: `x̂̇ = Ax̂+Bu + L(y-Cx̂)` ("copy of the model" + "correction due to sensor reading"). Error dynamics `x̃̇=(A-LC)x̃` → error → 0 iff eig(A-LC) in the left half-plane.
- **Predictor-corrector** structure for sampled-data / nonlinear systems: predict between measurements (`x̂̇=Ax̂+Bu` or `x̂̇=f(x̂,u)`), correct at a measurement (`x̂⁺ = x̂⁻ + L(y(tn)-Cx̂⁻)` or `...-h(x̂⁻)`).

### Kalman filter - full derivation (continuous-discrete form)
Model: `ẋ = Ax+Bu+ξ`, `y[n]=Cx[n]+η[n]`, `ξ~N(0,Q)` process noise, `η~N(0,R)` measurement noise. R usually comes from sensor calibration; **Q is the main tuning parameter**.

- **Prediction (between measurements)**: error `x̃̇ = Ax̃+ξ` ⇒ covariance ODE `Ṗ = AP+PAᵀ+Q` (derived by differentiating `E[x̃x̃ᵀ]` and using `E[ξ(τ)ξᵀ(t)]=Qδ(t-τ)`, taking half the delta-function area since integration is up to `t`).
- **Correction (at a measurement)**: `x̃⁺ = (I-LC)x̃⁻ - Lη`, `P⁺ = (I-LC)P⁻(I-LC)ᵀ + LRLᵀ` (Joseph form). Minimizing `tr(P⁺)` over `L` (using trace-derivative identities `∂/∂A tr(BAD)=BᵀDᵀ` and `∂/∂A tr(ABAᵀ)=2AB`) gives the **optimal Kalman gain**:
  `L* = P⁻Cᵀ(R + CP⁻Cᵀ)⁻¹`, and substituting back: `P⁺ = (I-L*C)P⁻(I-L*C)ᵀ + L*RL*ᵀ`.
- **Discretization for implementation** (Euler/2nd-order expansion): `Ad = e^(A·Ts) ≈ I + A·Ts + A²Ts²/2`, giving discrete-time covariance propagation `P_{k+1} = Ad·Pk·Adᵀ + Ts²·Q` (this form is guaranteed to keep P positive-definite, unlike naive first-order Euler `Pk+1 = Pk + Ts(APk+PkAᵀ+Q)` which can lose positive-definiteness numerically).
- **Full continuous-discrete KF summary** (for multiple sensors i, each processed sequentially if R is diagonal/uncorrelated):
  - Prediction: `x̂̇=Ax̂+Bu`, `Ad=I+ATs+A²Ts²/2`, `P_{k+1}=AdPkAdᵀ+Ts²Q`.
  - Correction at sensor i: `Li = P⁻Ciᵀ(Ri+CiP⁻Ciᵀ)⁻¹`, `x̂⁺=x̂⁻+Li(yi-Cix̂⁻)`, `P⁺=(I-LiCi)P⁻(I-LiCi)ᵀ+LiRiLiᵀ`.

### Extended Kalman Filter (EKF)
- For nonlinear system `ẋ=f(x,u)+ξ`, `yi[n]=hi(x[n],u[n])+ηi[n]`. Linearize about the current estimate: `A=∂f/∂x(x̂,u)`, `Ci=∂hi/∂x(x̂⁻)`, then apply the same discretized KF recursion as above using these Jacobians in place of constant A, C.
- **Algorithm box (pseudocode, given verbatim in slides)** - "Continuous-Discrete Extended Kalman Filter":
  1. Initialize `x̂=0`.
  2. Pick output rate `Tout ≪` sensor sample rates.
  3. At each `Tout`: run N prediction sub-steps with `Tp = Tout/N` (this inner-loop sub-stepping improves numerical accuracy of the discretization).
  4. On receipt of a measurement from sensor i: compute `Ci`, `Li`, update `P` and `x̂` (correction step).
  - Note: if R is diagonal (sensors uncorrelated), corrections can be applied one measurement at a time sequentially rather than as a batch.

### Worked example: attitude (φ, θ) estimation via EKF
- Nonlinear propagation model (from Euler-angle kinematics): `φ̇ = p + q·sinφ·tanθ + r·cosφ·tanθ + ξφ`, `θ̇ = q·cosφ - r·sinφ + ξθ`.
- Output = accelerometer, `y_accel = (u̇+qw-rv+g sinθ, v̇+ru-pw-g cosθ sinφ, ẇ+pv-qu-g cosθ cosφ)ᵀ + η_accel`.
- Problem: no direct measurement of `u̇,v̇,ẇ,u,v,w`. Solution: assume `u̇≈v̇≈ẇ≈0` and approximate `(u,v,w)ᵀ ≈ Va(cosα cosβ, sinβ, sinα cosβ)ᵀ ≈ Va(cosθ, 0, sinθ)ᵀ` (small-sideslip, α≈θ approximation) — substituting collapses the output equation to a function of `(φ,θ)` and the measured inputs `(p,q,r,Va)` only:
  `y_accel = (qVa sinθ + g sinθ, rVa cosθ - pVa sinθ - g cosθ sinφ, -qVa cosθ - g cosθ cosφ)ᵀ + η_accel`.
- State-space form: `x=(φ,θ)ᵀ`, `u=(p,q,r,Va)ᵀ`, `ξ=(ξφ,ξθ)ᵀ`, `η=(ηφ,ηθ)ᵀ`; noise on the gyro/pressure inputs `u_actual=u+ξu` propagates through the input-noise Jacobian `G(x)`:
  `ẋ=f(x,u)+G(x)ξu+ξ`, `y=h(x,u)+η`, with `G(x) = [[1, sinφ·tanθ, cosφ·tanθ, 0],[0, cosφ, -sinφ, 0]]`, and combined process noise covariance `G·Qu·Gᵀ + Q`.
- Jacobians for the EKF: `A=∂f/∂x` and `C=∂h/∂x` given explicitly (2×2 and 3×2 matrices, trig-heavy - see slide 36 for exact entries if re-deriving).
- **Result**: EKF attitude tracking shown in simulation to be "not perfect, but significantly better" than the pure sensor-inversion approach (matches roll/pitch through maneuvers where the level-flight assumption previously broke down).

### GPS smoothing
- Goal: fill in estimates between slow (~1 Hz) GPS updates, and estimate wind `(wn, we)`.
- Assuming level flight, position kinematics: `ṗn = Vg·cosχ`, `ṗe = Vg·sinχ`.
- Ground-speed dynamics derived from the wind triangle `Vg = |Va·(cosψ,sinψ) + (wn,we)|`, differentiated (assuming constant Va, wind) to: `V̇g = [(Va cosψ+wn)(-Va ψ̇ sinψ) + (Va sinψ+we)(Va ψ̇ cosψ)] / Vg`.
- Course-angle dynamics (from a coordinated-turn assumption): `χ̇ = (g/Vg)·tanφ·cos(χ-ψ)`.
- Wind assumed constant: `ẇn=0`, `ẇe=0`. Heading kinematics: `ψ̇ = q·sinφ/cosθ + r·cosφ/cosθ`.
- Full state `x=(pn,pe,Vg,χ,wn,we,ψ)ᵀ`, input `u=(Va,q,r,φ,θ)ᵀ`; `f(x,u)` and its Jacobian `∂f/∂x` given explicitly in slides (sparse structure - wind and ψ evolve independently of position/speed states at first order).
- **Pseudo-measurements from the wind-triangle constraint**: since `(pn,pe,Vg,χ,wn,we,ψ)` are not independent (related by the wind triangle), two synthetic zero-valued measurements are added to make the system observable:
  `y_windtri,n = Va cosψ + wn - Vg cosχ` (≡0), `y_windtri,e = Va sinψ + we - Vg sinχ` (≡0).
- Combined measurement vector `yGPS = (yGPS,n, yGPS,e, yGPS,Vg, yGPS,χ, ywind,n, ywind,e)`, with `h(x,u)` and its Jacobian given explicitly (slide 43) - a 6×7 matrix, mostly identity/trig blocks.
- **Result**: GPS-smoothed estimates of position, ground speed, course, wind, and heading track the true trajectory closely in simulation (small errors, a few meters / few m/s / few degrees).

### Measurement gating (outlier rejection)
- Innovation sequence `vk = yk - Cx̂k⁻ = Cx̃k⁻ + ηk`, with covariance `Sk = R + CPk⁻Cᵀ`.
- Test statistic `zk = (yk-h(xk))ᵀ·Sk⁻¹·(yk-h(xk))` is **chi-squared distributed with m degrees of freedom** (m = measurement dimension). A measurement update is only accepted if `zk` falls below a chi-squared threshold at a chosen confidence level (e.g. code snippet shown uses `scipy.stats.chi2.isf(q=0.01, df=3)` for a 3-dof accelerometer measurement) - this rejects statistical outliers before they corrupt the filter.

### Practical tuning advice (given verbatim, useful checklist)
1. Implement the EKF in stages: attitude estimator first, then GPS smoother.
2. Test/tune each component independently and thoroughly.
3. First verify the filter tracks states correctly with **noise-free** simulated sensors (exposes coding bugs, shows the filter's best-case performance ceiling).
4. Keep wind at zero until the base filter is confidently tuned.
5. `R` is usually well known from sensor datasheets/calibration; **tune primarily via `Q`**.
6. Tune state-by-state: excite one state at a time, keep others quiet, adjust the corresponding `Q` entry by trial and error.
7. Avoid extreme/abrupt inputs while tuning (large steps) - the filter's dynamic response has physical limits; jerky truth trajectories are inherently hard to estimate.
8. Re-check your equations - repeatedly.

## Key equations reference

- Accelerometer (spring-mass): `m·δ̈ + β·δ̇ + k·δ = m(z̈-g)`; specific-force interpretation `a = (1/m)(Ftotal - Fgravity)` - **accelerometers never read gravity directly**.
- Body-frame accelerometer output: `[ax,ay,az] = [u̇+qw-rv+g sinθ, v̇+ru-pw-g cosθ sinφ, ẇ+pv-qu-g cosθ cosφ]`.
- Rate gyro (Coriolis): `aC = 2Ω×v`; sensor output `∝ K·Ω` plus bias+noise `y_gyro = p (or q,r) + β + η`.
- Barometric altitude: `y_abs_pres = ρg·h_AGL + β + η`; airspeed from Bernoulli: `y_diff_pres = ρVa²/2 + β + η`.
- Magnetometer heading: `ψ = δ + ψm`, `ψm = -atan2(m0y^v1, m0x^v1)` after compensating for roll/pitch.
- GPS: 4 satellites solve for (lat,lon,alt,clock offset); error ≈ `DOP × UERE`; error modeled as first-order Gauss-Markov process `ν[n+1]=e^{-kTs}ν[n]+η[n]`.
- LQR/Kalman duality: LQR gain `K=R⁻¹BᵀP` solves `PA+AᵀP+Q-PBR⁻¹BᵀP=0`; Kalman gain `L=ΣCᵀN⁻¹` solves `AΣ+ΣAᵀ+W-ΣCᵀN⁻¹CΣ=0`.
- Continuous-discrete Kalman filter: prediction `x̂̇=Ax̂+Bu`, `Ad=I+ATs+A²Ts²/2`, `P_{k+1}=AdPkAdᵀ+Ts²Q`; correction `Li=P⁻Ciᵀ(Ri+CiP⁻Ciᵀ)⁻¹`, `x̂⁺=x̂⁻+Li(yi-Cix̂⁻)`, `P⁺=(I-LiCi)P⁻(I-LiCi)ᵀ+LiRiLiᵀ`.
- EKF = same recursion with `A=∂f/∂x(x̂,u)` and `Ci=∂hi/∂x(x̂⁻)` (local linearization at each step).
- Complementary filter: `x̂ = GL(s)·xmL + GH(s)·xmH`, `GL(s)=l/(s+l)`, `GH(s)=s/(s+l)`, `GL+GH=1` - provably a special (fixed-gain) case of the Kalman filter.
- Measurement gating: reject a measurement if `(y-h(x))ᵀS⁻¹(y-h(x))` exceeds a chi-squared threshold for the measurement's degrees of freedom.
- Observability matters concretely: a rate-gyro-only roll estimator (state `[φ,bp,br]`, output `rm` only) is **not observable**; adding an accelerometer-derived roll measurement and the roll rate `pm` makes it observable. Always check `(A,C)` before trusting an EKF/KF design.

## Open questions / unclear points

- Course lecture slide 12 has a typo in the stochastic system definition: `y = Cy + n` should read `y = Cx + n` (confirmed by every other occurrence of the same model in the deck and in the textbook chapter, which consistently write `y=Cx+n`).
- The "(A, B̄) Controllable?" annotation in the LQR/KF duality slide (course lecture, slide 14) is left as an open question mark in the source material itself - the course does not give a definitive general answer for when the noise-input pair needs to be controllable versus merely stabilizable; treat as a nuance to raise with the instructor if it becomes relevant to an assignment.
- The two Beard & McLain chapters are the *generic fixed-wing MAV* treatment (states `pn,pe,pd,u,v,w,φ,θ,ψ,p,q,r`); the course's own AR.Drone lab platform has no GPS and adds sonar+vision instead, so the GPS-smoothing and airspeed/pitot material in Ch6/Ch8 does not directly transfer to the drone labs - it's provided as generic background for the wider "UAV" concept (likely also relevant to a fixed-wing part of the course not covered by these three files). See `lectures-22-23/notes/` for how this connects to the rigid-body/quadrotor modeling chapters, once those are summarized in sibling notes files.
- Slide "Sensors: AR.Drone" (course lecture) does not give a quantitative model for the vision-based velocity estimate (no equations, just qualitative algorithm names) - if quantitative modeling of optical flow is needed later, it isn't in this material and would need to come from the AR.Drone devkit docs or additional papers.

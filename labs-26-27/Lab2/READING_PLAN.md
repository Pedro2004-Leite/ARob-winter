# Lab 2 (26/27) - theory reading plan

Goal: arrive at Sections 3 to 5 of the Lab 2 guide knowing every concept the questions rely on.
Nothing in the repository or in memory recorded an earlier reading plan, so this one was built on 2026-10-18 from the guide, the 25/26 Lab 2 notes (`labs-25-26/notes/02-lab2.tex`) and the lecture slides.
Page numbers refer to the lecture PDFs in `lectures-22-23/`.

## How the reading maps to the lab

| Lab section | What it needs | Read |
|---|---|---|
| 3.1 to 3.3 | Sensor models, bias, noise, covariance | Block 1 |
| 3.4 | Inclinometer formulas for roll and pitch | Block 1 |
| 4.1 to 4.3 | Steady-state Kalman filter, Q and R tuning, transfer functions | Blocks 2 and 3 |
| 4.4 | Filter with gyro bias state | Blocks 3 and 4 |
| 4.5 | Complementary filters | Block 4 |
| 4.6, 4.7 | Interpretation over experiments A to E, roll version | Blocks 1 to 4 |
| 5 | Batista, Silvestre, Oliveira partial attitude and bias filter | Block 5 |

## Block 0 - Prerequisites (skim, 30 min)

Only to make sure the notation is fresh.

- Attitude and Euler angles: `22_23_UAVs_Ch1_Rigid_Body.pdf`.
- Quadrotor kinematics and the relation between the body rates (p, q, r) and the Euler-angle rates: `22_23_UAVs_Ch2_Quadrotor_Modeling.pdf`.
- Linear state space, observability, observers: `22_23_UAVs_Ch4_Modern_Control_Design.pdf`, pages 3 to 30 (state-space), pages 41 to 43 (Luenberger observer example).

Self-check: write the small-angle relation between the gyro measurements and phi-dot, theta-dot, and say when it breaks down.

## Block 1 - Sensors and their errors (core, 1.5 h)

Source: `22_23_UAVs_Ch5-6_Sensors_and_State_Estimation_1.pdf`.

- Pages 3 and 4: the AR.Drone sensor suite (this is the course's own addition, not in the textbook).
- Pages 6 to 8: accelerometer principle, including the "tricky concept" page on measured acceleration.
- Page 9: rate gyro principle.
- Pages 10 and 11: magnetometers (context only, not used in Lab 2).

Textbook companion: `reference-book/uavbook.pdf`, the sensors chapter (Ch.7 in the 2nd edition numbering), accelerometers and MEMS gyro sections.
In `lectures-22-23/notes/03-sensors-estimation.pdf`, read "Accelerometers - basic principle", "Rate gyros", and the "Sensor-model inversion" section.

Self-checks, all tied to Section 3:

1. Why does a resting accelerometer read about -g on the z axis, and why is that not bias?
2. What does the accelerometer measure in a manoeuvre, and why does that corrupt the inclinometer?
3. What is the difference between the bias, the noise standard deviation and the noise covariance of a sensor, and what does a diagonal covariance imply?
4. Derive the inclinometer: theta from ax, phi from ay and az, under the steady-flight assumption.

## Block 2 - Kalman filter machinery (core, 2 h)

Source: same Ch5-6 slides, pages 12 to 14.

- Pages 12 and 13: the Kalman filter equations and the meaning of Q and R.
- Page 14: duality between LQR and the Kalman filter (K = R^-1 B' P versus L = Sigma C' N^-1, dual Riccati equations).

Textbook companion: the state-estimation chapter (Ch.8 in the 2nd edition numbering), "Dynamic observer theory" and "Kalman filter - full derivation" sections.
Notes: `lectures-22-23/notes/03-sensors-estimation.pdf`, sections "Kalman filter (course treatment)" and "Practical tuning advice".

Also review LQR in `22_23_UAVs_Ch4_Modern_Control_Design.pdf`, pages 26 and 31 to 40, because the Kalman gain is its dual.

Self-checks, tied to Sections 4.1 to 4.3:

1. What does the steady-state Kalman filter assume that a time-varying one does not?
2. How do Q and R change the gain, and what does a very large R/Q ratio do to the filter?
3. In MATLAB, what do `kalman()` and `estim()` return, and what inputs do they need (plant, Q, R, N)?

## Block 3 - The worked roll example (core, 1.5 h)

Source: same slides, pages 15 to 23.

This is the template for Section 4.1.
Follow the model, the measurement equation, the Riccati solution and the resulting filter step by step.
Reproduce the numbers on paper or in MATLAB before the session.

Self-checks, tied to 4.1, 4.2 and 4.7:

1. What is the state, the input and the measurement in the roll example, and which sensor plays which role?
2. For constant gains, write the two transfer functions from the inclinometer measurement and from the gyro measurement to the estimate.
3. Do these two transfer functions add to one? Why does that matter for Section 4.5?

## Block 4 - Complementary filters and gyro bias (core, 1.5 h)

Source: same slides, pages 24 to 28.

- Pages 24 to 26: complementary filters, with low-pass G_L(s) on the inclinometer and high-pass G_H(s) on the integrated gyro.
- Pages 27 and 28: the link between complementary and Kalman filters (a complementary filter is a Kalman filter with fixed gain).

Notes: `lectures-22-23/notes/03-sensors-estimation.pdf`, section "Complementary filters".

Self-checks, tied to 4.4 and 4.5:

1. With the bias state added (x = [phi, b_p], input p_m, output phi_m), what is the new transfer function structure?
2. How does the bias estimate converge, and when is the bias observable from the available measurements?
3. State precisely what "complementary" means (G_L + G_H = 1) and where it fails.

## Block 5 - The paper for Section 5 (2 h)

Source: `labs-25-26/Partial attitude and rate gyro bias estimation observability analysis filter design and performance evaluation.pdf` (Batista, Silvestre, Oliveira, Int. J. Control 2011, 9 pages).

Read in this order:

1. Abstract and introduction, to see the problem: one vector observation (gravity from the accelerometer) plus a biased three-axis rate gyro.
2. The system model (2) and the observability theorem (Theorem 3.3 in the 25/26 notes): the vector observation must not stay parallel to a fixed direction. A drone motionless in hover is therefore not observable for the full bias.
3. The filter design: Riccati equation dP/dt = AP + PA' + Xi - P C' Theta^-1 C P and gain K = P C' Theta^-1.
4. The performance section, for the tuning ideas.

Key point to carry into the report: the filter state is the vector observation and the bias, not roll and pitch directly, so roll and pitch are recovered with the same atan2 and arcsin inversion as the inclinometer in Block 1.

Self-checks, tied to Section 5:

1. Why can a stationary drone not determine the bias on the axis along the gravity direction?
2. How does this tie to what we saw in the data (experiments A, B, C versus E)?

## Block 6 - The course's closing synthesis (30 min)

- `lectures-22-23/COURSE_NOTES.pdf`, the cross-chapter synthesis on how modelling, control, estimation and guidance connect.
- Guide Sections 1.1 and 3.4: the whole point of the "raw" pitch and roll is to be the baseline for the filters.

## Suggested schedule

| Step | Block | Time |
|---|---|---|
| 1 | 0 and 1 | about 2 h |
| 2 | 2 | about 2 h |
| 3 | 3 | about 1.5 h |
| 4 | 4 | about 1.5 h |
| 5 | 5 | about 2 h |
| 6 | 6, then review of self-checks | about 1 h |

The times are rough estimates, not measurements.

## Questions I would like to check you on later

If you want a review, these are the questions that most reveal gaps:

- Why is the z accelerometer reading at rest not a bias?
- Why does the inclinometer fail in manoeuvres and under vibration (our experiment B shows this)?
- What do Q and R mean physically in the pitch filter, and which one would you raise for experiment B?
- Why is a complementary filter a fixed-gain Kalman filter?
- Why is the gyro bias not observable in hover?

## Link to our data

Section 3 uses experiments A, B and C, which are validated in `LAB_NOTES.md` and stored in `data/`.
Experiment D (pitch step) and E (mission) are for the filters in Section 4.

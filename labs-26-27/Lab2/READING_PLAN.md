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

## Best resources per area (rewritten 2026-10-18)

There are no recorded video lectures in the repository.
Every item below is one of three types, always written in capitals:

- **SLIDES**: a lecture deck PDF in `lectures-22-23/`. The page given is the PDF page number of the deck (page 1 is the cover).
- **BOOK**: `reference-book/uavbook.pdf` (Beard and McLain). Two page numbers are given, the one printed on the page and the PDF page. PDF page = printed page + 8.
- **NOTES**: a written summary PDF in `lectures-22-23/notes/` or `labs-25-26/notes/` (not slides, not the book).

Slide decks used (all in `lectures-22-23/`):

- **Course Ch5-6 deck**: `22_23_UAVs_Ch5-6_Sensors_and_State_Estimation_1.pdf` (28 pages, the course's own slides, tailored to the lab).
- **Beard Sensors deck**: `22_23_UAVs_Ch5_Sensors_Beard_McLain.pdf` (29 pages, the book's Chapter 7 slides).
- **Beard Estimation deck**: `22_23_UAVs_Ch6_State_Estimation_Beard_McLain.pdf` (47 pages, the book's Chapter 8 slides).
- **Course Ch4 deck**: `22_23_UAVs_Ch4_Modern_Control_Design.pdf` (43 pages).

### Area 1 - Sensor models (accelerometer, rate gyro, bias, noise), for guide 3.1 to 3.3

1. SLIDES, Beard Sensors deck, pages 3 to 13.
   Page 5 and 6: measured acceleration is specific force. Pages 9 and 10: accelerometer model. Page 13: rate gyro model with bias.
2. BOOK, Chapter 7 "Sensors for MAVs", sections 7.1 Accelerometers and 7.2 Rate Gyros.
   Printed pages 121 to 127, PDF pages 129 to 135.
3. SLIDES, Course Ch5-6 deck, pages 6 to 9 (accelerometer principle and rate gyro principle), and pages 3 and 4 for the AR.Drone sensor suite.

### Area 2 - Raw roll and pitch (inclinometer), for guide 3.4

1. SLIDES, Beard Estimation deck, pages 6 to 12 ("Inverting sensor measurement model").
   Page 9 and 10 are the roll and pitch inversion under steady, level flight.
2. BOOK, Chapter 8 "State Estimation", sections 8.2 Low-pass Filters and 8.3 State Estimation by Inverting the Sensor Model.
   Printed pages 148 to 153, PDF pages 156 to 161.

### Area 3 - Kalman filter and observers, for guide 4.1 to 4.3

1. SLIDES, Beard Estimation deck, pages 13 to 29.
   Pages 13 to 15: dynamic observer. Pages 16 to 18: quadratic forms and Gaussian. Pages 19 to 29: continuous-discrete Kalman filter derivation and covariance update.
2. BOOK, Chapter 8, sections 8.4 Dynamic-observer Theory and 8.5 Derivation of the Continuous-discrete Kalman Filter.
   Printed pages 154 to 160, PDF pages 162 to 168.
3. SLIDES, Course Ch5-6 deck, pages 12 and 13 (Kalman filter) and page 14 (duality between LQR and Kalman filter).

### Area 4 - LQR and its dual, for the Kalman gain (guide 4.1) and Lab 3

1. SLIDES, Course Ch4 deck, page 26 (optimal control) and pages 31 to 40 (LQR design example).
2. BOOK, Chapter 6 "Autopilot Design", section 6.3 LQR Control.
   Printed pages 115 to 117, PDF pages 123 to 125.

### Area 5 - Attitude filter and tuning Q and R, for guide 4.1, 4.3 and 4.7

1. SLIDES, Course Ch5-6 deck, pages 15 to 23 (worked roll-angle example). This is the template for the lab.
2. SLIDES, Beard Estimation deck, pages 33 to 37 (attitude estimation with EKF) and page 47 (tuning advice).
3. BOOK, Chapter 8, section 8.8 Attitude Estimation.
   Printed pages 164 to 166, PDF pages 172 to 174.

### Area 6 - Complementary filters, for guide 4.5

1. SLIDES, Course Ch5-6 deck, pages 24 to 26 (complementary filters) and pages 27 and 28 (relation to the Kalman filter).
2. NOTES, `lectures-22-23/notes/03-sensors-estimation.pdf`, section "Complementary filters".
3. The book table of contents has no dedicated section on this.

### Area 7 - Gyro bias and partial attitude estimation, for guide 4.4 and Section 5

1. The paper itself: `labs-25-26/Partial attitude and rate gyro bias estimation observability analysis filter design and performance evaluation.pdf` (9 pages).
2. NOTES, `labs-25-26/notes/02-lab2.tex` (the paper walkthrough, including the observability theorem).
3. The book table of contents has no dedicated section on bias estimation.

### Lab 3 pointers

- Path following: BOOK, Chapter 10, section 10.1 Straight-line Path Following. Printed pages 198 to 203, PDF pages 206 to 211.
- LQR: Area 4 above.
- Lyapunov and adaptive control: SLIDES `22_23_UAVs_Ch7_Intro_Nonlinear_Control.pdf` and `22_23_UAVs_Ch7_Intro_Nonlinear_Control_Part_II.pdf`. I have not read their page titles, so the pages are still to be chosen.

### Minimal path if short on time

1. SLIDES, Beard Sensors deck, pages 3 to 13.
2. SLIDES, Beard Estimation deck, pages 6 to 12, then 13 to 29.
3. SLIDES, Course Ch5-6 deck, pages 15 to 23, then 24 to 28.
4. The Batista paper.

Correction versus the first version of this section: the earlier BOOK ranges ended one page too late (for example 7.2 was given as ending at printed page 128, but 7.3 starts there).
The ranges above end on the last page of the last listed section.

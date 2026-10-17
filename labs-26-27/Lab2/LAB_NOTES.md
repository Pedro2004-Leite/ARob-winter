# Lab 2 (26/27) - working notes

Status: **paused again 2026-10-17** at Pedro's request (waiting). Data moved to `data/`, wifiControl synced, data quality verified. Next, when told to go: `sensor_characterization.m` (Section 3).

## Data files (in `data/`, updated 2026-10-17)

- Location: `labs-26-27/Lab2/data/` (right next to this file, no per-person folder).
- Source: `data(1).zip` downloaded 2026-10-17, byte-identical to the 2026-10-13 `data.zip`. Six Simulink "To Workspace" exports from the Lab 2 model `ARDroneHover_V2`.
- Cleaned up: the duplicate `E_final` was dropped, `E_1` renamed to `dados_alinea_E.mat`, and the typo `aliean` in D fixed. Names are now `dados_alinea_{A,B,C,D,E}.mat`.
- Variables: `navdata` and `states` are structs with `.time`, `.signals.values`.
  - `navdata.signals.values` is (N, 2186) uint8 at 200 Hz.
  - `states.signals.values` is N x 9 = [phi theta psi u v w x y z].
  - `time` is only the last 1000 samples (a scope artifact); `time_1` and `navdata.time` are the full time vector. Do not use `time`.
  - `sampleTime` = 0.005. `ignore` (and `Y_ref_elias` in E) are MATLAB timeseries objects, readable inside MATLAB only.
- The format matches Replay mode: `start_here_NAV` option (3), `load('<file>')`, `setupReplay`.

| File | Experiment | Duration | Verified story |
|---|---|---|---|
| `dados_alinea_A.mat` | A - at rest, motors off | 31.9 s | ctrl LANDED throughout, alt 0, noise floor |
| `dados_alinea_B.mat` | B - held down, takeoff commanded | 37.4 s | takeoff cmd at 7.3 s; see B clarification below |
| `dados_alinea_C.mat` | C - hover | 41.2 s | takeoff cmd 2.3 s, hovering 8.1-38.9 s at ~0.75 m, landing 38.9-40 s |
| `dados_alinea_D.mat` | D - pitch steps | 37.8 s | takeoff 3.1 s, hover 9.8 s, pitch maneuvers, landing 35.8-36.8 s |
| `dados_alinea_E.mat` | E - waypoint mission | 68.3 s | takeoff 2.2 s, hover 6.3 s, y to 1.0 m, back to 0, landing 66.5-67.5 s |

## Models and scripts (wifiControl, updated 2026-10-17)

`DevKit_Lab2/ARDroneSimulinkDevKit_NAV/wifiControl/` now matches the `wifiControl.zip` Pedro downloaded.
New files vs the original devkit: `ARDroneHover_V2_ALTER_D.slx`, `ARDroneHover_V2_ALTER_E.slx` (+ `.r2017b`), `ARDroneHover_V2_ALTER.slx.r2017b`, `Copy_of_setupHover_EXEC_D.m`, `Copy_of_setupHover_EXEC_E.m`.
The two EXEC scripts are identical to `setupHover.m` except for the last line, which opens the `_ALTER_D` / `_ALTER_E` model instead of `ARDroneHover_V2`.
The editor `.autosave` and `.DS_Store` from the zip were deliberately not copied.

## Data quality check (2026-10-17)

Own Python decoder over all five files. Verdict: **good, usable for Section 3**, with the caveats below.

- Integrity: no NaN/Inf in `states`, `dt` exactly 5 ms everywhere, `states` and `navdata` time vectors identical, no truncated or corrupt packets (every option walk parses to the end).
- The first 1-5 rows of every file are all-zero (stream not started yet): drop rows before ~0.03 s.
- The 200 Hz navdata is oversampled: only ~185-190 distinct IMU updates/s (5-7% of consecutive IMU samples are exact repeats, in bursts up to ~34 samples = 170 ms in A, 14-31 in B-E).
  Zero-order-hold artifacts bias std slightly low and whiteness tests (autocorrelation) will show spikes; consider de-duplicating before the whiteness/PSD analysis.
- `vbat` jumps in the first seconds of B (42 to 57 %), D (39 to 50 %), E (30 to 42 %): stale first packets / reading settling; do not use as a battery log.
- Experiment A: clean. Accs [-3.4, -16.9, -956.7] mg, ultrasound 240-245 mg quantization steps. The robust-outlier counts on acc y/z are only a quantization artifact (MAD ~ 0).
- Experiment B: consistent with the clarification (altraw flat 238-239 mm, `demo.altitude` diverges -413..1066 mm). `states` z (col 9) shows 4.29e6 (a uint32 wrap), so never use `states` z or x from B.
- Experiments C, D, E: the large excursions (acc z to -2.4..-2.7 g, gyro up to 100-210 deg/s, ultrasound 0 and 4613 mm) are all in takeoff/landing transitions.
  Inside the flight windows the ultrasound is clean (median 746-751 mm, no zeros, no jumps >300 mm).
- Clean flight-window stats (excluding 2 s after takeoff and 1.5 s before landing):

| Window | accs std [mg] | gyros std [deg/s] |
|---|---|---|
| C hover 10.1-37.4 s | [6.1, 7.7, 7.7] | [2.3, 1.9, 1.9] |
| B held 16.2-33.5 s | [53.5, 89.4, 25.9] | [5.5, 5.0, 6.9] |

  This supersedes the C row of the older table below (that one included the transitions and inflated the z accel std to 69.5 mg and gyro std to 3.8/6.6): hover noise is ~2x calmer than I first reported, and B is the vibration-dominated case.
- D has a one-tick `ctrl_state` = 8 (looping) at 16.8 s, together with a 3 flicker; probably a glitch from the pitch-step maneuver, ignore for Section 3 (use A, B, C).
- Duplicate-model note: D and E were recorded with the `_ALTER` models (the zip suggests this), so their references may differ from plain `ARDroneHover_V2`; check before using them for Section 4/5 comparisons.

## Navdata decoding facts (for the .m parser)

- Each row of `navdata.signals.values` is one 2186-byte packet: 12 bytes of unknown framing, then AR.Drone 2.0 options starting at offset 12.
- Options walk: little-endian uint16 tag, uint16 size (size includes the 4-byte header).
- demo option (tag 0, 148 B): ctrl_state uint32 @0; vbat uint32 @4 (%); theta/phi/psi float32 @8/12/16 (milli-deg); altitude int32 @20 (mm, fused estimate); vx/vy/vz float32 @24/28/32.
- phys_measures option (tag 3, 46 B): gyro_temp uint16 @4; phys_accs float32 @6/10/14 (mg); phys_gyros float32 @18/22/26 (deg/s).
- Units confirmed against the data: z accel at rest ~ -957 mg ~ -g; gyro pitch rate matches the derivative of theta.
- Convert to SI for the report: accs x 9.81/1000 m/s2, gyros x pi/180 rad/s.
- The official decode path is the extended navdata block in `ARBlocks.slx` via Replay mode.
- The alternative is a small MATLAB parser replicating the walk above (a direct port of the Python decoder used to validate the files in the 2026-10-13 session).

## Experiment B clarification (Pedro confirmed)

- The drone never left the ground: it was held down the whole run, and takeoff was commanded only so the control loop engaged.
- `demo.altitude` (the drone's fused estimate) diverges to ~1 m because the onboard estimator integrates accelerations; it is NOT physical truth here.
- `altitude_raw` (ultrasound, option 10, int32 @ offset 12 of the option body) stays flat at ~238 mm = resting height = ground truth.
- Lesson: never use `demo.altitude` to segment B; segment by ctrl_state transitions instead.
- B windows: 0-7 s motors off (sanity check vs A); 7.5-35 s held down at full throttle.

## Measured statistics (own Python parser - recompute with the official decode for the report)

| Window | accs mean [mg] | accs std [mg] | gyros bias [deg/s] | gyros std [deg/s] |
|---|---|---|---|---|
| A (rest) | [-3.4, -16.9, -956.7] | [3.6, 2.4, 2.3] | [-0.04, -0.03, +0.01] | [0.05, 0.04, 0.04] |
| B (held, full throttle) | [-26.1, -55.7, -923.3] | [47.8, 84.8, 29.4] | [0.15, 0.15, -0.30] | [4.7, 4.5, 6.5] |
| C (hover) | [14.8, -33.9, -954.8] | [14.6, 13.0, 69.5] | [-0.06, 0.05, -0.11] | [3.8, 6.6, 2.1] |

Story to discuss in the report:

- A is the noise floor.
- B amplifies ~20x (accs) and ~100x (gyros) via propeller vibration at full throttle plus hand contact.
- C is calmer in x/y (hover throttle, no contact) but noisier in z (height controller + airflow).
- Gyro bias shifts between runs (thermal drift) - the argument for explicit bias compensation (4.4, Section 5) instead of one-time calibration.

## Section 3 plan (per the lab guide)

Tasks 3.1-3.3 (A/B/C): per window, per axis, compute mean (bias) and std, plus the 6x6 sample covariance.
Report off-diagonals ~ 0 to justify diagonal Q/R in Section 4.
Report z-accel bias as deviation from -g (the -957 mg is gravity, not bias).
For C, exclude takeoff/landing (use ~10-38 s, ctrl flying and altitude_raw ~750 mm) so dynamic accelerations do not inflate the stats.

Task 3.4 (raw inclinometer), from the lecture notes (steady, level flight assumption):

- theta_accel = asin(ax/g)
- phi_accel = atan2(ay, az)

Optional LPF on the accel signals first, as the notes write.

- Expected: A gives theta ~ +0.2 deg, phi ~ -1 deg (table tilt + accel bias) = the raw measurements theta_m/phi_m.
- B gives std ~ 5 deg (unusable under vibration).
- C is in between and breaks down in maneuvers (the notes' explicit caveat).
- Compare against the block's theta_a/phi_a outputs as a check.
- This A vs B/C contrast motivates the whole Kalman section (gyro fast but drifts, inclinometer slow but unbiased).

Deliverables suggested: one `sensor_characterization.m` producing a stats table, histograms of the 6 signals (Gaussianity), optional autocorrelation/PSD (whiteness), plus a ~2 page report section (comparison table + one figure with the three noise regimes).

## Open items

- Write `sensor_characterization.m` (waiting for go-ahead).
- Sections 4 and 5 come after Section 3.

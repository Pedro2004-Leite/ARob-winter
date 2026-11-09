%% Lab 2 - Section 4: steady-state Kalman filters for pitch and roll (4.1 - 4.7)
%
% Run this script from anywhere; it finds the data in ../data.
% It prints every number used in the report and saves the figures to
% ./figures. All filters are designed in continuous time with kalman()
% (Control System Toolbox) and run with lsim(), first in simulation (known
% true attitude, simulated sensors) and then on the flight data.
%
% Filters (per axis, pitch shown; roll is the same with p and phi):
%   KF1 (4.1)  x = theta              theta_dot = w_ym + n_g
%   KF2 (4.4)  x = [theta; b_y]       theta_dot = w_ym - b_y + n_g,  b_y_dot = n_b
%   input  u = w_ym  (rate gyro, assumed theta_dot ~ q for small angles)
%   output y = theta_m (inclinometer from the accelerometers, Section 3.4)
%
% Noise intensities come from Section 3 (experiment C, hover): the sampled
% variance of a white sequence at Ts corresponds to the continuous intensity
%   Q = var(gyro)*Ts,  R = var(inclinometer)*Ts.
% KF2 adds the bias random-walk intensity Qb = cfg.qb^2 as its tuning knob.
% "Tuned" filters multiply R by the rho that fits the flight experiment best
% (4.3): in flight the inclinometer error is model error, not sensor noise.
%
% Simulation: the attitude comes from a closed-loop model driven by reference
% steps (pitch: Lab 1 identification; roll: DevKit ssRoll), and the sensors are
% that attitude and its rate plus white noise with the hover variances. In 4.4
% an artificial bias cfg.bArt is added to the simulated gyro to tune qb.
%
% On the flight data the bias is the sensor's own, and every filter is compared
% with the measurement (inclinometer) and with the state recorded in flight:
% the AR.Drone's own onboard attitude estimate
% (states(:,1:2)), which also uses the camera/velocity aiding. It is not the
% truth, so the comparison is on the shape (std of the difference), and the
% constant offset between the two is reported separately.

clear; close all; clc;

here = fileparts(mfilename('fullpath'));
addpath(fullfile(here, '..', 'data'));
figdir = fullfile(here, 'figures');
if ~isfolder(figdir), mkdir(figdir); end

%% Configuration
cfg.Ts    = 0.005;      % navdata sample time [s] (200 Hz), only for variance -> intensity
cfg.g     = 9.81;
cfg.noise = 'C';        % experiment whose statistics give Q and R (hover)
cfg.qb    = 1e-4;       % KF2: bias random-walk intensity [rad/s/sqrt(s)]
cfg.qbSweep = [1e-4 1e-3 1e-2];     % 4.4: values tried in simulation and on the real gyro
cfg.qbSim = 1e-3;       % KF2 in simulation: lowest error of cfg.qbSweep (4.4)
cfg.bArt  = 0.05;       % artificial gyro bias in simulation [rad/s] (~2.9 deg/s)
cfg.tSettle = 5;        % simulation: start of the metrics, after the initial transient [s]
cfg.tLate = 30;         % simulation: start of the steady-state bias metrics [s]
cfg.rhoSweep = logspace(-2, 4, 13); % 4.3: R is multiplied by rho
cfg.Rtune = [];         % R multiplier of the tuned filters ([] = take the 4.3 optimum)
cfg.inclSource = 'accel';  % inclinometer: 'accel', 'state' or 'estimated' (see the header)
cfg.inclRho = 31.6;     % 'estimated': R multiplier of the first-pass KF1 (about the 4.3 optimum)

% Closed-loop attitude models for the simulation (reference -> angle): pitch
% from the Lab 1 identification (report Q4.2), roll from the DevKit
s = tf('s');
cfg.model.pitch = ss(0.9088*(s + 21.19) / (s^2 + 2*0.4916*6.822*s + 6.822^2));
D = load(fullfile(here, '..', 'DevKit_Lab2', 'ARDroneSimulinkDevKit_NAV', 'lib', 'ssRoll.mat'));
cfg.model.roll = D.ssRoll;

% Experiments and evaluation windows [s] (see LAB_NOTES.md; the takeoff and
% landing transients are excluded)
exps = { 'A', 'dados_alinea_A.mat',  1.0, 31.0;    % at rest, motors off
         'B', 'dados_alinea_B.mat', 16.2, 33.5;    % held down, rotors spinning
         'C', 'dados_alinea_C.mat', 10.1, 37.4;    % hover
         'D', 'dados_alinea_D.mat', 11.8, 34.3;    % pitch steps
         'E', 'dados_alinea_E.mat',  8.3, 65.0 };  % y waypoint mission

%% Load the experiments (decode_navdata + raw inclinometer of Section 3.4)
for e = 1:size(exps,1)
    EX(e) = load_experiment(exps{e,1}, exps{e,2}, [exps{e,3} exps{e,4}], cfg); %#ok<SAGROW>
end

%% Section 3 statistics that feed Q and R
fprintf('Noise statistics in the evaluation windows (std, deg and deg/s)\n');
fprintf('%-3s %10s %10s %10s %10s\n', '', 'gyro_x', 'gyro_y', 'phi_m', 'theta_m');
for e = 1:numel(EX)
    w = EX(e).win;
    fprintf('%-3s %10.3f %10.3f %10.3f %10.3f\n', EX(e).name, ...
        rad2deg(std(EX(e).gyro(w,1))), rad2deg(std(EX(e).gyro(w,2))), ...
        rad2deg(std(EX(e).phi_m(w))),  rad2deg(std(EX(e).theta_m(w))));
end
fprintf('Q and R are taken from experiment %s.\n\n', cfg.noise);

%% Pitch (4.1 - 4.6) and roll (4.7)
pitch = axis_study('pitch', EX, cfg, figdir);
roll  = axis_study('roll',  EX, cfg, figdir);

save(fullfile(here, 'kalman_results.mat'), 'pitch', 'roll', 'cfg');
fprintf('\nFigures in %s\n', figdir);


%% ======================================================================
function R = axis_study(axisName, EX, cfg, figdir)
% Sections 4.1 - 4.6 for one axis ('pitch' or 'roll'). Calling it with
% 'roll' is Section 4.7.

    Ts = cfg.Ts;
    if strcmp(axisName, 'pitch')
        gi = 2;  ang = 'theta';  sym = '\theta';  rate = 'q';  flight = 'D';
    else
        gi = 1;  ang = 'phi';    sym = '\phi';    rate = 'p';  flight = 'E';
    end
    fprintf('==================== %s ====================\n', upper(axisName));
    EF = EX(strcmp({EX.name}, flight));     % flight experiment of this axis
    ref = EF.([ang '_ob']);                 % state recorded in flight
    xl = EF.t([find(EF.win,1) find(EF.win,1,'last')]);

    % Q, R from the hover experiment (Section 3)
    N0 = EX(strcmp({EX.name}, cfg.noise));
    sg = std(N0.gyro(N0.win, gi));          % gyro noise std [rad/s]
    sm = std(N0.([ang '_m'])(N0.win));      % inclinometer noise std [rad]
    Q = sg^2*Ts;  Rm = sm^2*Ts;             % continuous intensities
    fprintf('sigma_gyro = %.4f rad/s (%.2f deg/s), sigma_incl = %.4f rad (%.2f deg)\n', ...
        sg, rad2deg(sg), sm, rad2deg(sm));

    %% 4.1 Steady-state Kalman filter without bias
    kf1 = design_kf(Q, Rm, []);
    fprintf('\n4.1  KF1: Q = %.3g (rad/s)^2 s, R = %.3g rad^2 s\n', Q, Rm);
    fprintf('     Riccati: 0 = Q - P^2/R  ->  P = sqrt(Q R), L = P/R = sqrt(Q/R)\n');
    fprintf('     L = %.3f rad/s (kalman), sqrt(Q/R) = %.3f rad/s\n', kf1.L, sqrt(Q/Rm));
    fprintf('     filter pole: s = %.3f rad/s (%.2f Hz)\n', -kf1.L, kf1.L/2/pi);

    % Simulation phase: known true attitude, simulated white-noise sensors
    S0 = simulate_truth(cfg.model.(axisName), Ts, sg, sm, 0);
    th1s = run_kf(kf1, S0.t, S0.wm, S0.ym);
    ws = S0.t >= cfg.tSettle;
    R.sim.rmsIncl = rms(S0.ym(ws) - S0.ang(ws));
    R.sim.rmsKF1 = rms(th1s(ws) - S0.ang(ws));
    fprintf('     simulation: rms(incl - true) = %.3f deg, rms(KF1 - true) = %.3f deg, sqrt(P) = %.3f deg\n', ...
        rad2deg(R.sim.rmsIncl), rad2deg(R.sim.rmsKF1), rad2deg(sqrt(kf1.P)));

    f = figure('Name', ['4.1 sim ' axisName], 'Position', [50 50 1000 600]);
    subplot(2,1,1);
    plot(S0.t, rad2deg(S0.ym), 'Color', [0.75 0.75 0.75]); hold on;
    plot(S0.t, rad2deg(S0.ang), 'k', 'LineWidth', 1.5);
    plot(S0.t, rad2deg(th1s), 'b');
    ylabel([sym ' [deg]']); grid on;
    legend('measurement (inclinometer)', 'true state', 'KF1', 'Location', 'best');
    title(['4.1 Simulation, ' axisName ', KF1 (no bias)']);
    subplot(2,1,2);
    plot(S0.t, rad2deg(S0.ym - S0.ang), 'Color', [0.75 0.75 0.75]); hold on;
    plot(S0.t, rad2deg(th1s - S0.ang), 'b');
    ylabel('error [deg]'); xlabel('t [s]'); grid on;
    legend('measurement (inclinometer)', 'KF1', 'Location', 'best');
    save_fig(f, figdir, ['4_1_sim_' axisName]);

    % Flight data
    th1 = run_kf(kf1, EF.t, EF.gyro(:,gi), EF.([ang '_m']));
    fprintf('     experiment %s: std(incl - flight) = %.3f deg, std(KF1 - flight) = %.3f deg\n', flight, ...
        rad2deg(std(EF.([ang '_m'])(EF.win) - ref(EF.win))), rad2deg(std(th1(EF.win) - ref(EF.win))));

    f = figure('Name', ['4.1 ' axisName], 'Position', [50 50 1000 450]);
    plot(EF.t, rad2deg(EF.([ang '_m'])), 'Color', [0.75 0.75 0.75]); hold on;
    plot(EF.t, rad2deg(ref), 'k', 'LineWidth', 1.5);
    plot(EF.t, rad2deg(th1), 'b');
    xlim(xl); ylabel([sym ' [deg]']); xlabel('t [s]'); grid on;
    legend('measurement (inclinometer)', 'flight state (onboard)', 'KF1', 'Location', 'best');
    title(['4.1 Experiment ' flight ', ' axisName ', KF1 (no bias)']);
    save_fig(f, figdir, ['4_1_exp' flight '_' axisName]);

    %% 4.2 Transfer functions for constant gains
    % theta_hat_dot = w_ym + L (theta_m - theta_hat)
    %   Theta_hat = L/(s+L) Theta_m + 1/(s+L) W_ym
    fprintf('\n4.2  KF1 transfer functions (constant gain):\n');
    fprintf('     F_m(s) = %.3f / (s + %.3f)     (inclinometer -> estimate, low-pass)\n', kf1.L, kf1.L);
    fprintf('     F_w(s) =    1  / (s + %.3f)     (gyro -> estimate)\n', kf1.L);
    fprintf('     s F_w(s) = s / (s + %.3f)       (high-pass on the integrated gyro)\n', kf1.L);
    fprintf('     crossover = %.2f Hz\n', kf1.L/2/pi);

    %% 4.3 Impact of Q and R: sweep R -> rho R on the flight experiment
    sdev = zeros(size(cfg.rhoSweep));  snoise = sdev;  ssim = sdev;
    EC = EX(strcmp({EX.name}, 'C'));
    for i = 1:numel(cfg.rhoSweep)
        k = design_kf(Q, cfg.rhoSweep(i)*Rm, []);
        th = run_kf(k, EF.t, EF.gyro(:,gi), EF.([ang '_m']));
        sdev(i) = std(th(EF.win) - ref(EF.win));
        thc = run_kf(k, EC.t, EC.gyro(:,gi), EC.([ang '_m']));
        snoise(i) = std(thc(EC.win));
        ths = run_kf(k, S0.t, S0.wm, S0.ym);
        ssim(i) = rms(ths(ws) - S0.ang(ws));
    end
    [~, iBest] = min(sdev);
    rhoT = cfg.Rtune;
    if isempty(rhoT), rhoT = cfg.rhoSweep(iBest); end
    fprintf('\n4.3  R sweep on experiment %s (std of KF1 - flight state, deg) and in simulation (rms of KF1 - true, deg):\n', flight);
    for i = 1:numel(cfg.rhoSweep)
        fprintf('     rho = %8.3g  L = %7.3f rad/s  std(err) = %.3f  std in hover C = %.3f  rms sim = %.3f\n', ...
            cfg.rhoSweep(i), sqrt(Q/(cfg.rhoSweep(i)*Rm)), rad2deg(sdev(i)), rad2deg(snoise(i)), rad2deg(ssim(i)));
    end
    [~, iSim] = min(ssim);
    fprintf('     best rho in simulation = %.3g\n', cfg.rhoSweep(iSim));
    fprintf('     tuned: rho = %.3g (R x %.3g), L = %.3f rad/s\n', rhoT, rhoT, sqrt(Q/(rhoT*Rm)));

    f = figure('Name', ['4.3 QR ' axisName], 'Position', [50 50 1000 420]);
    subplot(1,2,1);
    w = logspace(-2, 2.5, 400);
    for rho = [1e-2 1 1e2 1e4]
        k = design_kf(Q, rho*Rm, []);
        [mFm, ~] = bode(k.Fm, w);
        semilogx(w, 20*log10(squeeze(mFm)), 'DisplayName', sprintf('Q/R x %g', 1/rho)); hold on;
    end
    grid on; xlabel('\omega [rad/s]'); ylabel('|F_m| [dB]'); legend('Location', 'southwest');
    title('4.3 Inclinometer path F_m(s) vs Q/R');
    subplot(1,2,2);
    loglog(cfg.rhoSweep, rad2deg(sdev), 'o-', cfg.rhoSweep, rad2deg(snoise), 's-', cfg.rhoSweep, rad2deg(ssim), 'd-');
    grid on; xlabel('\rho  (R \rightarrow \rho R)'); ylabel('error measure [deg]');
    legend(['std(KF1 - flight state), exp. ' flight], 'std(KF1) in hover C', 'rms(KF1 - true), simulation', 'Location', 'northoutside');
    title('4.3 Sweep of R');
    save_fig(f, figdir, ['4_3_QR_' axisName]);

    kf1t = design_kf(Q, rhoT*Rm, []);
    th1t = run_kf(kf1t, EF.t, EF.gyro(:,gi), EF.([ang '_m']));

    f = figure('Name', ['4.3 exp ' axisName], 'Position', [50 50 1000 450]);
    plot(EF.t, rad2deg(EF.([ang '_m'])), 'Color', [0.75 0.75 0.75]); hold on;
    plot(EF.t, rad2deg(ref), 'k', 'LineWidth', 1.5);
    plot(EF.t, rad2deg(th1), 'Color', [0.9 0.5 0.1]);
    plot(EF.t, rad2deg(th1t), 'b');
    xlim(xl); ylabel([sym ' [deg]']); xlabel('t [s]'); grid on;
    legend('measurement (inclinometer)', 'flight state (onboard)', 'KF1 nominal R', ...
        sprintf('KF1 R x %.3g', rhoT), 'Location', 'best');
    title(['4.3 Experiment ' flight ', ' axisName ', effect of R']);
    save_fig(f, figdir, ['4_3_exp' flight '_' axisName]);

    %% 4.4 Kalman filter with explicit gyro bias state
    Qb = cfg.qb^2;
    kf2 = design_kf(Q, Rm, Qb);
    fprintf('\n4.4  KF2: x = [%s; b], Q = diag(%.3g, %.3g), R = %.3g\n', ang, Q, Qb, Rm);
    fprintf('     observability rank = %d of 2\n', rank(obsv(kf2.A, kf2.C)));
    fprintf('     L = [%.3f; %.4f]\n', kf2.L);
    pc = eig(kf2.A - kf2.L*kf2.C);
    fprintf('     poles s = %s rad/s, slow (bias) time constant = %.2f s\n', mat2str(pc.', 4), 1/min(abs(pc)));
    % kalman() returns L(2) < 0 because the bias enters with a minus sign;
    % with l2 = -L(2) > 0 the filter is the 2nd-order complementary pair
    l1 = kf2.L(1);  l2 = -kf2.L(2);
    fprintf('     F_m(s) = (%.3f s + %.4f) / (s^2 + %.3f s + %.4f)\n', l1, l2, l1, l2);
    fprintf('     F_w(s) =  s / (s^2 + %.3f s + %.4f)   -> F_w(0) = 0, a constant bias is rejected\n', l1, l2);
    kf2t = design_kf(Q, rhoT*Rm, Qb);
    fprintf('     KF2 tuned: R x %.3g, qb = %g\n', rhoT, cfg.qb);

    % Simulation phase: artificial bias on the simulated gyro, nominal R,
    % sweep of qb to guide the tuning (b vs b_hat). KF1 should keep an offset
    % b/L and KF2 should remove it.
    Sb = simulate_truth(cfg.model.(axisName), Ts, sg, sm, cfg.bArt);
    wsb = Sb.t >= cfg.tSettle;  late = Sb.t >= cfg.tLate;
    t1b = run_kf(kf1, Sb.t, Sb.wm, Sb.ym);
    fprintf('     simulation with artificial bias %.3f rad/s (%.2f deg/s):\n', cfg.bArt, rad2deg(cfg.bArt));
    fprintf('       KF1 nominal: mean error = %.3f deg (b/L = %.3f deg)\n', ...
        rad2deg(mean(t1b(late) - Sb.ang(late))), rad2deg(cfg.bArt/kf1.L));

    f = figure('Name', ['4.4 sim ' axisName], 'Position', [50 50 1000 800]);
    subplot(3,1,1);
    plot(Sb.t, rad2deg(Sb.b)*ones(size(Sb.t)), 'k', 'LineWidth', 1.5, 'DisplayName', 'true (artificial) bias'); hold on;
    for qb = cfg.qbSweep
        k = design_kf(Q, Rm, qb^2);
        [th, bh] = run_kf(k, Sb.t, Sb.wm, Sb.ym);
        plot(Sb.t, rad2deg(bh), 'DisplayName', sprintf('b_{hat}, q_b = %g', qb));
        fprintf('       KF2 qb = %-7g bias time const = %6.2f s  rms(KF2 - true) = %.3f deg  b_hat mean = %.3f deg/s  std = %.3f deg/s (t >= %g s)\n', ...
            qb, 1/min(abs(eig(k.A - k.L*k.C))), rad2deg(rms(th(wsb) - Sb.ang(wsb))), ...
            rad2deg(mean(bh(late))), rad2deg(std(bh(late))), cfg.tLate);
    end
    grid on; ylabel(['b_' rate ' [deg/s]']); legend('Location', 'southeast');
    title(['4.4 Simulation, artificial gyro bias ' num2str(rad2deg(cfg.bArt), 3) ' deg/s, ' axisName]);
    kf2s = design_kf(Q, Rm, cfg.qbSim^2);
    t2b = run_kf(kf2s, Sb.t, Sb.wm, Sb.ym);
    fprintf('       KF2 qb = %g: mean error = %.3f deg, rms(KF1 - true) = %.3f deg, rms(KF2 - true) = %.3f deg (t >= %g s)\n', ...
        cfg.qbSim, rad2deg(mean(t2b(late) - Sb.ang(late))), rad2deg(rms(t1b(late) - Sb.ang(late))), ...
        rad2deg(rms(t2b(late) - Sb.ang(late))), cfg.tLate);
    subplot(3,1,2);
    plot(Sb.t, rad2deg(Sb.ym), 'Color', [0.75 0.75 0.75]); hold on;
    plot(Sb.t, rad2deg(Sb.ang), 'k', 'LineWidth', 1.5);
    plot(Sb.t, rad2deg(t1b), 'r', Sb.t, rad2deg(t2b), 'b');
    grid on; ylabel([sym ' [deg]']);
    legend('measurement (inclinometer)', 'true state', 'KF1 (no bias state)', ...
        sprintf('KF2 (q_b = %g)', cfg.qbSim), 'Location', 'best');
    subplot(3,1,3);
    plot(Sb.t, rad2deg(t1b - Sb.ang), 'r', Sb.t, rad2deg(t2b - Sb.ang), 'b');
    grid on; ylabel('error [deg]'); xlabel('t [s]'); legend('KF1', 'KF2', 'Location', 'best');
    save_fig(f, figdir, ['4_4_sim_' axisName]);

    % On the flight data the bias is the sensor's own. Its reference is the slope
    % of (integrated gyro - flight state), compared at rest (experiment A,
    % where the true rate is zero) and in the flight experiment. KF1 cannot
    % remove this bias and should keep an offset b/L against KF2.
    EA = EX(strcmp({EX.name}, 'A'));
    y = EF.([ang '_m']);
    bRefA = bias_reference(EA, gi, ang);  bRefF = bias_reference(EF, gi, ang);
    t2t = run_kf(kf2t, EF.t, EF.gyro(:,gi), y);
    iA = find(EA.win, 1, 'last');  iF = find(EF.win, 1, 'last');
    fprintf('     sensor bias reference: exp. A %.4f deg/s (mean gyro at rest %.4f), exp. %s %.4f deg/s\n', ...
        rad2deg(bRefA), rad2deg(mean(EA.gyro(EA.win,gi))), flight, rad2deg(bRefF));
    fprintf('     exp. %s: bias/L of KF1 tuned = %.3f deg, mean(KF1 tuned - KF2 tuned) = %.3f deg\n', ...
        flight, rad2deg(bRefF/kf1t.L), rad2deg(mean(th1t(EF.win) - t2t(EF.win))));

    f = figure('Name', ['4.4 ' axisName], 'Position', [50 50 1000 850]);
    sA = subplot(3,1,1);  sF = subplot(3,1,2);
    plot(sA, EA.t, rad2deg(bRefA)*ones(size(EA.t)), 'k', 'LineWidth', 1.5, 'DisplayName', 'sensor bias (reference)');
    hold(sA, 'on');
    plot(sF, EF.t, rad2deg(bRefF)*ones(size(EF.t)), 'k', 'LineWidth', 1.5, 'DisplayName', 'sensor bias (reference)');
    hold(sF, 'on');
    for qb = cfg.qbSweep
        k = design_kf(Q, rhoT*Rm, qb^2);
        [~, bA] = run_kf(k, EA.t, EA.gyro(:,gi), EA.([ang '_m']));
        [th, bF] = run_kf(k, EF.t, EF.gyro(:,gi), y);
        plot(sA, EA.t, rad2deg(bA), 'DisplayName', sprintf('b_{hat}, q_b = %g', qb));
        plot(sF, EF.t, rad2deg(bF), 'DisplayName', sprintf('b_{hat}, q_b = %g', qb));
        fprintf('       KF2 qb = %-7g bias time const = %6.2f s  std(KF2 - flight) = %.3f deg  b_hat end: A %.4f, %s %.4f deg/s\n', ...
            qb, 1/min(abs(eig(k.A - k.L*k.C))), rad2deg(std(th(EF.win) - ref(EF.win))), ...
            rad2deg(bA(iA)), flight, rad2deg(bF(iF)));
    end
    xlim(sA, EA.t([1 iA])); grid(sA, 'on'); ylabel(sA, ['b_' rate ' [deg/s]']); legend(sA, 'Location', 'best');
    title(sA, ['4.4 Sensor bias, experiment A (at rest), ' axisName]);
    xlim(sF, [EF.t(1) xl(2)]); grid(sF, 'on'); ylabel(sF, ['b_' rate ' [deg/s]']); legend(sF, 'Location', 'best');
    title(sF, ['4.4 Sensor bias, experiment ' flight]);
    subplot(3,1,3);
    plot(EF.t, rad2deg(y), 'Color', [0.75 0.75 0.75]); hold on;
    plot(EF.t, rad2deg(ref), 'k', 'LineWidth', 1.5);
    plot(EF.t, rad2deg(th1t), 'r', EF.t, rad2deg(t2t), 'b');
    xlim(xl); grid on; ylabel([sym ' [deg]']); xlabel('t [s]');
    legend('measurement (inclinometer)', 'flight state (onboard)', 'KF1 tuned', ...
        sprintf('KF2 tuned (q_b = %g)', cfg.qb), 'Location', 'best');
    save_fig(f, figdir, ['4_4_exp' flight '_' axisName]);

    %% 4.5 Complementarity check: F_m + s F_w = 1
    w = logspace(-3, 3, 500);
    for k = {kf1, kf2}
        kk = k{1};
        H = squeeze(freqresp(kk.Fm + tf('s')*kk.Fw, w));
        fprintf('\n4.5  %s: max |F_m + s F_w - 1| = %.2e', kk.name, max(abs(H - 1)));
    end
    fprintf('\n');
    f = figure('Name', ['4.5 compl ' axisName], 'Position', [50 50 900 420]);
    w = logspace(-2, 2.5, 400);
    for k = {kf1, kf2}
        kk = k{1};
        [mL, ~] = bode(kk.Fm, w);  [mH, ~] = bode(tf('s')*kk.Fw, w);
        semilogx(w, 20*log10(squeeze(mL)), 'LineWidth', 1.2, 'DisplayName', [kk.name ' F_m (low-pass)']); hold on;
        semilogx(w, 20*log10(squeeze(mH)), '--', 'LineWidth', 1.2, 'DisplayName', [kk.name ' s F_w (high-pass)']);
    end
    ylim([-60 10]); grid on; xlabel('\omega [rad/s]'); ylabel('magnitude [dB]'); legend('Location', 'southwest');
    title(['4.5 Complementary pairs, ' axisName]);
    save_fig(f, figdir, ['4_5_compl_' axisName]);

    %% 4.6 Evaluation on experiments A - E
    fprintf('\n4.6  Experiments (deg). offset = mean(est - flight state), std = std(est - flight state)\n');
    fprintf('     %-3s | %-15s | %-15s | %-15s | %-15s | %-15s | %s\n', 'exp', 'inclinometer', ...
        'gyro integ.', 'KF1 nominal', 'KF1 tuned', 'KF2 tuned', 'b_hat at window end [deg/s]');
    filters = {'incl', 'gyro', 'kf1', 'kf1t', 'kf2t'};
    for e = 1:numel(EX)
        X = EX(e);  wv = X.win;  u = X.gyro(:,gi);  y = X.([ang '_m']);  ob = X.([ang '_ob']);
        est.incl = y;
        i0 = find(wv, 1);
        gint = cumtrapz(X.t, u);  est.gyro = gint - gint(i0) + ob(i0);
        est.kf1  = run_kf(kf1,  X.t, u, y);
        est.kf1t = run_kf(kf1t, X.t, u, y);
        [est.kf2t, bh] = run_kf(kf2t, X.t, u, y);
        fprintf('     %-3s |', X.name);
        for j = 1:numel(filters)
            d = est.(filters{j})(wv) - ob(wv);
            R.exp.(X.name).(filters{j}) = [mean(d) std(d)];
            fprintf(' %6.2f / %6.3f |', rad2deg(mean(d)), rad2deg(std(d)));
        end
        bRef = bias_reference(X, gi, ang);
        R.exp.(X.name).bhat = bh(find(wv, 1, 'last'));
        R.exp.(X.name).bref = bRef;
        fprintf(' %6.3f (reference %6.3f)\n', rad2deg(R.exp.(X.name).bhat), rad2deg(bRef));
        R.est.(X.name) = est;  R.est.(X.name).bhat = bh;
    end

    % Time plots of every experiment: measurement, filters, flight state
    for e = 1:numel(EX)
        X = EX(e);  est = R.est.(X.name);  xw = X.t([find(X.win,1) find(X.win,1,'last')]);
        f = figure('Name', ['4.6 ' X.name ' ' axisName], 'Position', [50 50 1000 650]);
        subplot(3,1,[1 2]);
        plot(X.t, rad2deg(est.incl), 'Color', [0.8 0.8 0.8]); hold on;
        plot(X.t, rad2deg(X.([ang '_ob'])), 'k', 'LineWidth', 1.5);
        plot(X.t, rad2deg(est.kf1), 'Color', [0.9 0.5 0.1]);
        plot(X.t, rad2deg(est.kf1t), 'r', X.t, rad2deg(est.kf2t), 'b');
        xlim(xw); grid on; ylabel([sym ' [deg]']);
        legend('measurement (inclinometer)', 'flight state (onboard)', 'KF1 nominal', 'KF1 tuned', ...
            'KF2 tuned', 'Location', 'best');
        title(['4.6 Experiment ' X.name ', ' axisName]);
        subplot(3,1,3);
        plot(X.t, rad2deg(est.bhat), 'b'); grid on; xlim(xw);
        ylabel(['b_' rate ' estimate [deg/s]']); xlabel('t [s]');
        save_fig(f, figdir, ['4_6_exp' X.name '_' axisName]);
    end

    R.kf1 = kf1;  R.kf1t = kf1t;  R.kf2 = kf2;  R.kf2t = kf2t;  R.rhoT = rhoT;
    R.sweep = struct('rho', cfg.rhoSweep, 'std_err', sdev, 'std_hover', snoise, 'rms_sim', ssim);
    R.sigma = [sg sm];
end


%% ======================================================================
function kf = design_kf(Q, R, Qb)
% Continuous steady-state Kalman filter for one attitude angle.
%   Q   gyro noise intensity [(rad/s)^2 s]        -> process noise on the angle
%   R   inclinometer noise intensity [rad^2 s]    -> measurement noise
%   Qb  [] for KF1; bias random-walk intensity [(rad/s)^2/s] for KF2
    if isempty(Qb)
        A = 0;  B = 1;  G = 1;  C = 1;
        Qn = Q;
        kf.name = 'KF1';
    else
        A = [0 -1; 0 0];  B = [1; 0];  G = eye(2);  C = [1 0];
        Qn = diag([Q, Qb]);
        kf.name = 'KF2';
    end
    n = size(A,1);

    % kalman() returns the estimator with inputs [u; y] and outputs
    % [y_hat; x_hat], the gain L and the steady-state error covariance P
    sys = ss(A, [B G], C, zeros(1, 1 + size(G,2)));
    [kf.kest, kf.L, kf.P] = kalman(sys, Qn, R);

    % Transfer functions u -> angle_hat (F_w) and y -> angle_hat (F_m)
    F = tf(ss(A - kf.L*C, [B kf.L], [1 zeros(1, n-1)], 0));
    kf.Fw = minreal(F(1,1));  kf.Fm = minreal(F(1,2));

    kf.A = A;  kf.B = B;  kf.C = C;  kf.Q = Qn;  kf.R = R;
end

function [ang, b] = run_kf(kf, t, u, y)
% Run the continuous filter on the sampled gyro u and inclinometer y
% (lsim, inputs held between samples). Initial state: first inclinometer
% sample, zero bias.
    x0 = zeros(size(kf.A,1), 1);  x0(1) = y(1);
    out = lsim(kf.kest, [u(:) y(:)], t - t(1), x0, 'zoh');
    ang = out(:,2);
    if size(out,2) > 2, b = out(:,3); else, b = zeros(size(ang)); end
end

function S = simulate_truth(G, Ts, sg, sm, b)
% Simulated experiment with known truth: the closed-loop attitude model G is
% driven by reference steps (as in experiment D) and gives the true angle and
% its exact rate. The sensors add white noise with the hover standard
% deviations, and the gyro also the bias b:
%   w_m = rate + b + n_g,   angle_m = angle + n_m
    t = (0:Ts:60)';
    r = 0.2*(t >= 10 & t < 15) - 0.2*(t >= 15 & t < 20) ...
      + 0.1*(t >= 30 & t < 35) - 0.1*(t >= 45 & t < 50);
    x = lsim(ss(G.A, G.B, eye(size(G.A)), 0), r, t, 'zoh');
    S.t = t;
    S.ang = x*G.C';
    S.rate = x*(G.C*G.A)' + r*(G.C*G.B);
    rng(1);
    S.wm = S.rate + b + sg*randn(size(t));
    S.ym = S.ang + sm*randn(size(t));
    S.b = b;
end

function b = bias_reference(X, gi, ang)
% Gyro bias from the data: the measured rate is rate + b, so integrated gyro
% minus flight state grows with slope b over the evaluation window.
    gint = cumtrapz(X.t, X.gyro(:,gi));
    p = polyfit(X.t(X.win), gint(X.win) - X.([ang '_ob'])(X.win), 1);
    b = p(1);
end

function [theta_m, phi_m] = inclinometer(acc, g, theta, phi)
% Raw pitch and roll from the accelerometers. With no angles given the
% formulas use the accelerometers only; otherwise each axis uses the angle
% of the other axis:
%   ax/(-az) = tan(theta)/cos(phi),   -ay = g sin(phi) cos(theta)
    if nargin < 3
        theta_m = asin(max(min(acc(:,1)/g, 1), -1));
        phi_m   = atan2(-acc(:,2), -acc(:,3));
    else
        theta_m = atan2(acc(:,1).*cos(phi), -acc(:,3));
        phi_m   = asin(max(min(-acc(:,2)./(g*cos(theta)), 1), -1));
    end
end

function X = load_experiment(name, file, win, cfg)
% Decoded sensors, raw inclinometer (Section 3.4) and the onboard attitude
% interpolated on the navdata time.
    m = decode_navdata(file);
    X.name = name;
    S = load(file, 'states');
    st = interp1(S.states.time(:), double(S.states.signals.values), m.t, 'linear', 'extrap');
    X.t = m.t(1) + (0:numel(m.t)-1)'*cfg.Ts;   % uniform grid for lsim (dt is exactly Ts)
    X.gyro = m.gyro;
    X.acc = m.acc;
    % Specific force at rest is -g along z_B (sensor reads az ~ -g)
    [X.theta_m, X.phi_m] = inclinometer(m.acc, cfg.g);
    X.phi_ob   = st(:,1);
    X.theta_ob = st(:,2);
    X.win = X.t >= win(1) & X.t <= win(2);
end

function save_fig(f, figdir, name)
    exportgraphics(f, fullfile(figdir, [name '.png']), 'Resolution', 150);
end

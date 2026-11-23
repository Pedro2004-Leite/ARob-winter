function r = run_batista_kf(expName, opts)
% RUN_BATISTA_KF  Replay an experiment through ARDroneReplay_BatistaKF.slx.
%
%   r = run_batista_kf('D')
%   r = run_batista_kf('C', bias=[3 -2 1], pacing=10)
%
% Loads ../data/dados_alinea_<expName>.mat (A..E), replays it through the
% Batista et al. Kalman filter model and, while it runs, shows a scope with
% [phi_hat, theta_hat] vs [phi, theta] (phi, theta = the AR.Drone onboard
% estimate, which is NOT ground truth).
%
% Options:
%   bias     artificial rate gyro bias [deg/s] added to the gyros (default 0),
%            the way the Lab 2 guide suggests tuning the bias estimate
%   calibrate  seconds of ground data, from t = 0, used to calibrate the
%            accelerometer (ax, ay) assuming the drone starts level at rest
%            (default 0 = off). Causal: running mean over the first
%            calibrate seconds, frozen afterwards. It must end before takeoff.
%   thfactor measurement noise factor of Th (default 1e3), see kf_tuning
%   bq       bias random-walk intensity in Xi (default 1e-5), see kf_tuning
%   pacing   simulation speed relative to real time (default 5, 0 = as fast
%            as possible)
%   scopes   open the scope (default true)
%   quiet    do not print the summary (default false)
%
% Output: struct with t [s], phi_hat, theta_hat, phi_ref, theta_ref [deg],
% b_hat (N x 3) [deg/s], acc_cal (N x 6, accelerometer offset removed, m/s^2)
% and the summary: rms (1x2, offset removed), offset (1x2), both [deg],
% computed for t > 10 s and t < end - 3 s (no takeoff or landing).

arguments
    expName (1,1) string = "D"
    opts.bias (1,3) double = [0 0 0]
    opts.calibrate (1,1) double = 0
    opts.thfactor (1,1) double = 1e3
    opts.bq (1,1) double = 1e-5
    opts.pacing (1,1) double = 5
    opts.scopes (1,1) logical = true
    opts.quiet (1,1) logical = false
end

here = fileparts(mfilename('fullpath'));
dk   = fullfile(here, '..', 'DevKit_Lab2', 'ARDroneSimulinkDevKit_NAV');
addpath(fullfile(dk, 'lib'));
m = 'ARDroneReplay_BatistaKF';
load_system(fullfile(here, [m '.slx']));

% The Replay block reads the variable "navdata" from the base workspace
S = load(fullfile(here, '..', 'data', sprintf('dados_alinea_%s.mat', expName)), 'navdata');
assignin('base', 'navdata', S.navdata);

set_param(m, 'StopTime', num2str(S.navdata.time(end)));
if opts.pacing > 0
    set_param(m, 'EnablePacing', 'on', 'PacingRate', num2str(opts.pacing));
else
    set_param(m, 'EnablePacing', 'off');
end
mw = get_param(m, 'ModelWorkspace');
assignin(mw, 'b_art_degps', opts.bias);
assignin(mw, 'acc_cal_T', opts.calibrate);
[Xi, Th] = kf_tuning(opts.thfactor, opts.bq);
assignin(mw, 'Xi', Xi);
assignin(mw, 'Th', Th);
% The scope window opens at the window manager's default size: resize it by
% hand if it comes up small.
sc = get_param([m '/Scope angles'], 'ScopeConfiguration');
sc.OpenAtSimulationStart = opts.scopes;
sc.TimeSpan = num2str(S.navdata.time(end));

% The Replay block prints its internal time vector on every step: silence it
[~, out] = evalc('sim(m)');

%% Collect and summarise against the onboard estimate (constant offset removed)
[t, ang]  = kf_getlog(out, 'ang_ref');
[~, angh] = kf_getlog(out, 'ang_hat');
[~, b]    = kf_getlog(out, 'b_hat');
[~, ac]   = kf_getlog(out, 'acc_cal');
r = struct('t', t, 'phi_hat', angh(:,1), 'theta_hat', angh(:,2), ...
    'phi_ref', ang(:,1), 'theta_ref', ang(:,2), 'b_hat', b, 'acc_cal', ac);
win = t > 10 & t < t(end) - 3;
e   = angh(win,:) - ang(win,:);
r.rms    = std(e);
r.offset = mean(e);
if ~opts.quiet
    fprintf('Experiment %s (%.1f s), artificial bias [%g %g %g] deg/s\n', expName, t(end), opts.bias);
    if opts.calibrate > 0
        fprintf('  accelerometer calibrated on the first %.1f s: removed ax %+.3f, ay %+.3f m/s^2\n', ...
            opts.calibrate, ac(end, 1), ac(end, 2));
    else
        fprintf('  accelerometer NOT calibrated\n');
    end
    fprintf('  RMS(KF - onboard), offset removed [deg]: phi %.2f, theta %.2f\n', r.rms);
    fprintf('  constant offset KF - onboard [deg]:      phi %+.2f, theta %+.2f\n', r.offset);
    fprintf('  gyro bias estimate at the end [deg/s]:   [%.2f %.2f %.2f]\n', b(end, :));
end
end

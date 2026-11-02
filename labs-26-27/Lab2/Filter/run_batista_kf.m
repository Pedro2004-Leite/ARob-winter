function out = run_batista_kf(expName, opts)
% RUN_BATISTA_KF  Replay an experiment through ARDroneReplay_BatistaKF.slx.
%
%   out = run_batista_kf('D')
%   out = run_batista_kf('C', bias=[3 -2 1], pacing=10)
%
% Loads ../data/dados_alinea_<expName>.mat (A..E), replays it through the
% Batista et al. Kalman filter model and, while it runs, shows two scopes:
% phi_hat vs phi and theta_hat vs theta (phi, theta = the AR.Drone onboard
% estimate, which is NOT ground truth).
%
% Options:
%   bias     artificial rate gyro bias [deg/s] added to the gyros (default 0),
%            the way the Lab 2 guide suggests tuning the bias estimate
%   calibrate  seconds of ground data, from t = 0, used to calibrate the
%            accelerometer (ax, ay) assuming the drone starts level at rest
%            (default 0 = off). Causal: running mean over the first
%            calibrate seconds, frozen afterwards. It must end before takeoff.
%   pacing   simulation speed relative to real time (default 5, 0 = as fast
%            as possible)
%   scopes   open the scopes (default true)
%
% Output: Simulink.SimulationOutput with the timeseries phi_hat, theta_hat,
% b_hat [deg/s], phi_ref, theta_ref [deg]. A summary is printed at the end.

arguments
    expName (1,1) string = "D"
    opts.bias (1,3) double = [0 0 0]
    opts.calibrate (1,1) double = 0
    opts.pacing (1,1) double = 5
    opts.scopes (1,1) logical = true
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
% The scope windows open at the window manager's default size: resize them
% by hand if they come up small.
for blk = {'phi', 'theta'}
    sc = get_param([m '/Scope ' blk{1}], 'ScopeConfiguration');
    sc.OpenAtSimulationStart = opts.scopes;
    sc.TimeSpan = num2str(S.navdata.time(end));
end

% The Replay block prints its internal time vector on every step: silence it
[~, out] = evalc('sim(m)');

%% Summary against the onboard estimate (constant offset removed)
t   = out.phi_hat.Time;
win = t > 10 & t < t(end) - 3;                 % skip takeoff and landing
e   = [out.phi_hat.Data(win) - out.phi_ref.Data(win), ...
       out.theta_hat.Data(win) - out.theta_ref.Data(win)];
fprintf('Experiment %s (%.1f s), artificial bias [%g %g %g] deg/s\n', expName, t(end), opts.bias);
if opts.calibrate > 0
    ac = squeeze(out.acc_cal.Data);            % 3 x N, accelerometer bias removed [m/s^2]
    fprintf('  accelerometer calibrated on the first %.1f s: removed ax %+.3f, ay %+.3f m/s^2\n', ...
        opts.calibrate, ac(1, end), ac(2, end));
else
    fprintf('  accelerometer NOT calibrated\n');
end
fprintf('  RMS(KF - onboard), offset removed [deg]: phi %.2f, theta %.2f\n', ...
    std(e(:,1)), std(e(:,2)));
fprintf('  constant offset KF - onboard [deg]:      phi %+.2f, theta %+.2f\n', ...
    mean(e(:,1)), mean(e(:,2)));
b = squeeze(out.b_hat.Data);                   % 3 x N
fprintf('  gyro bias estimate at the end [deg/s]:   [%.2f %.2f %.2f]\n', b(:, end));
end

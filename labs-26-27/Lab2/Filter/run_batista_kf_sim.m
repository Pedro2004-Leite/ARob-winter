function r = run_batista_kf_sim(opts)
% RUN_BATISTA_KF_SIM  Run the Batista et al. filter on a synthetic experiment
% with known ground truth (ARDroneSim_BatistaKF.slx).
%
%   r = run_batista_kf_sim
%   r = run_batista_kf_sim(accbias=[0 -0.166 0])     % accelerometer bias, [m/s^2]
%
% Truth: roll = A1 sin(2 pi f1 t), pitch = A2 sin(2 pi f2 t + pi/3), yaw rate 0.
% Sensors: exact accelerometer and rate gyro readings + white Gaussian noise
% + a constant rate gyro bias (and, optionally, a constant accelerometer bias).
% While it runs, two scopes show [phi_hat, theta_hat] vs the truth and the
% bias estimate vs the true bias.
%
% Options:
%   bias     rate gyro bias [deg/s]            (default [2 -3 1], the paper's)
%   accbias  accelerometer bias [m/s^2]        (default 0)
%   noise    multiplier of the noise std       (default 1: 0.07 m/s^2, 0.01 rad/s)
%   amp      roll, pitch amplitude [deg]       (default [8 6])
%   freq     roll, pitch frequency [Hz]        (default [0.30 0.45])
%   T        duration [s]                      (default 60)
%   seed     noise seed offset                 (default 0)
%   thfactor measurement noise factor of Th    (default 1e3), see kf_tuning
%   bq       bias random-walk intensity in Xi  (default 1e-5), see kf_tuning
%   pacing   speed relative to real time       (default 5, 0 = as fast as possible)
%   scopes   open the scopes                   (default true)
%   quiet    do not print the summary          (default false)
%
% Output: struct with t [s], phi_hat, theta_hat, phi_true, theta_true [deg],
% b_hat (N x 3) [deg/s] and the summary after a 10 s transient: rms (1x2)
% and offset (1x2) of the attitude error [deg], bias_end (1x3, mean of the
% last 20 s) [deg/s].

arguments
    opts.bias (1,3) double = [2 -3 1]
    opts.accbias (1,3) double = [0 0 0]
    opts.noise (1,1) double = 1
    opts.amp (1,2) double = [8 6]
    opts.freq (1,2) double = [0.30 0.45]
    opts.T (1,1) double = 60
    opts.seed (1,1) double = 0
    opts.thfactor (1,1) double = 1e3
    opts.bq (1,1) double = 1e-5
    opts.pacing (1,1) double = 5
    opts.scopes (1,1) logical = true
    opts.quiet (1,1) logical = false
end

here = fileparts(mfilename('fullpath'));
m = 'ARDroneSim_BatistaKF';
load_system(fullfile(here, [m '.slx']));

mw = get_param(m, 'ModelWorkspace');
assignin(mw, 'b_true_degps', opts.bias);
assignin(mw, 'acc_bias',     opts.accbias);
assignin(mw, 'sigma_a',      0.07*opts.noise);
assignin(mw, 'sigma_g',      0.01*opts.noise);
assignin(mw, 'sim_A_deg',    opts.amp);
assignin(mw, 'sim_f_hz',     opts.freq);
assignin(mw, 'seed0',        opts.seed);
[Xi, Th] = kf_tuning(opts.thfactor, opts.bq);
assignin(mw, 'Xi', Xi);
assignin(mw, 'Th', Th);

set_param(m, 'StopTime', num2str(opts.T));
if opts.pacing > 0
    set_param(m, 'EnablePacing', 'on', 'PacingRate', num2str(opts.pacing));
else
    set_param(m, 'EnablePacing', 'off');
end
for blk = {'angles', 'bias'}
    sc = get_param([m '/Scope ' blk{1}], 'ScopeConfiguration');
    sc.OpenAtSimulationStart = opts.scopes;
    sc.TimeSpan = num2str(opts.T);
end

out = sim(m);

%% Collect and summarise against the truth
[t, ang]  = kf_getlog(out, 'ang_true');
[~, angh] = kf_getlog(out, 'ang_hat');
[~, b]    = kf_getlog(out, 'b_hat');
r = struct('t', t, 'phi_hat', angh(:,1), 'theta_hat', angh(:,2), ...
    'phi_true', ang(:,1), 'theta_true', ang(:,2), 'b_hat', b);
win = t > 10;
e   = angh(win,:) - ang(win,:);
r.rms      = sqrt(mean(e.^2));
r.offset   = mean(e);
r.bias_end = mean(b(t > t(end) - 20, :), 1);
if ~opts.quiet
    fprintf('Synthetic experiment, %.0f s, gyro bias [%g %g %g] deg/s, accelerometer bias [%g %g %g] m/s^2, noise x%g\n', ...
        opts.T, opts.bias, opts.accbias, opts.noise);
    fprintf('  attitude error KF - truth [deg]:  RMS phi %.2f, theta %.2f | mean phi %+.2f, theta %+.2f\n', ...
        r.rms, r.offset);
    fprintf('  gyro bias estimate, mean of the last 20 s [deg/s]: [%.2f %.2f %.2f]  (true [%g %g %g])\n', ...
        r.bias_end, opts.bias);
end
end

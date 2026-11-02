function out = run_batista_kf_sim(opts)
% RUN_BATISTA_KF_SIM  Run the Batista et al. filter on a synthetic experiment
% with known ground truth (ARDroneSim_BatistaKF.slx).
%
%   out = run_batista_kf_sim
%   out = run_batista_kf_sim(accbias=[0 -0.166 0])     % accelerometer bias, [m/s^2]
%
% Truth: roll = A1 sin(2 pi f1 t), pitch = A2 sin(2 pi f2 t + pi/3), yaw rate 0.
% Sensors: exact accelerometer and rate gyro readings + white Gaussian noise
% + a constant rate gyro bias (and, optionally, a constant accelerometer bias).
% While it runs, three scopes show phi_hat vs phi, theta_hat vs theta and the
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
%   pacing   speed relative to real time       (default 5, 0 = as fast as possible)
%   scopes   open the scopes                   (default true)
%
% Output: Simulink.SimulationOutput with phi_hat, theta_hat, b_hat [deg/s],
% phi_true, theta_true [deg]. The summary is computed after a 10 s transient.

arguments
    opts.bias (1,3) double = [2 -3 1]
    opts.accbias (1,3) double = [0 0 0]
    opts.noise (1,1) double = 1
    opts.amp (1,2) double = [8 6]
    opts.freq (1,2) double = [0.30 0.45]
    opts.T (1,1) double = 60
    opts.seed (1,1) double = 0
    opts.pacing (1,1) double = 5
    opts.scopes (1,1) logical = true
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

set_param(m, 'StopTime', num2str(opts.T));
if opts.pacing > 0
    set_param(m, 'EnablePacing', 'on', 'PacingRate', num2str(opts.pacing));
else
    set_param(m, 'EnablePacing', 'off');
end
for blk = {'phi', 'theta', 'bias'}
    sc = get_param([m '/Scope ' blk{1}], 'ScopeConfiguration');
    sc.OpenAtSimulationStart = opts.scopes;
    sc.TimeSpan = num2str(opts.T);
end

out = sim(m);

%% Summary against the truth
t   = out.phi_hat.Time;
win = t > 10;
e   = [out.phi_hat.Data(win) - out.phi_true.Data(win), ...
       out.theta_hat.Data(win) - out.theta_true.Data(win)];
b   = squeeze(out.b_hat.Data);                 % 3 x N
fprintf('Synthetic experiment, %.0f s, gyro bias [%g %g %g] deg/s, accelerometer bias [%g %g %g] m/s^2, noise x%g\n', ...
    opts.T, opts.bias, opts.accbias, opts.noise);
fprintf('  attitude error KF - truth [deg]:  RMS phi %.2f, theta %.2f | mean phi %+.2f, theta %+.2f\n', ...
    sqrt(mean(e(:,1).^2)), sqrt(mean(e(:,2).^2)), mean(e(:,1)), mean(e(:,2)));
fprintf('  gyro bias estimate, mean of the last 20 s [deg/s]: [%.2f %.2f %.2f]  (true [%g %g %g])\n', ...
    mean(b(:, t > t(end) - 20), 2), opts.bias);
end

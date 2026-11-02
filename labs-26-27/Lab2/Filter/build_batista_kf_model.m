function build_batista_kf_model()
% BUILD_BATISTA_KF_MODEL  Create the two Simulink models of Lab 2, Section 5.
%
%   ARDroneReplay_BatistaKF.slx   the filter on the logged navdata of an
%                                 experiment (run with run_batista_kf.m)
%   ARDroneSim_BatistaKF.slx      the same filter on a synthetic experiment
%                                 with known ground truth (run with
%                                 run_batista_kf_sim.m)
%
% The filter is the continuous-time Kalman(-Bucy) filter of Batista,
% Silvestre and Oliveira (Int. J. Control, 2011), system (2) with the noise
% model (10):
%
%   state   x = [x1; x2] in R^6   x1 = y (accelerometer, body frame, m/s^2)
%                                 x2 = rate gyro bias (rad/s)
%   system  x_dot = A(t) x,  A = [-S(w_m) -S(y); 0 0],  output  y = [I 0] x
%   filter  x_hat_dot = A x_hat + K (y - C x_hat),  K = P C' Th^-1
%           P_dot     = A P + P A' + Xi - P C' Th^-1 C P
%
% Both x_hat and P are integrated by Simulink Integrator blocks (fixed step
% ode1 at the 5 ms navdata period). Re-run this script only to regenerate
% the models.

here = fileparts(mfilename('fullpath'));
dk   = fullfile(here, '..', 'DevKit_Lab2', 'ARDroneSimulinkDevKit_NAV');
addpath(fullfile(dk, 'lib'));
load_system('ARBlocks');

build_replay_model(here);
build_sim_model(here);
end

%% ------------------------------------------------------------------------
function build_replay_model(here)
m = new_model(here, 'ARDroneReplay_BatistaKF');
assignin(get_param(m, 'ModelWorkspace'), 'b_art_degps', [0 0 0]);   % artificial gyro bias [deg/s]
assignin(get_param(m, 'ModelWorkspace'), 'acc_cal_T', 0);            % ground calibration window [s], 0 = off

% Replay block: outputs used are 1 roll (deg), 2 pitch (deg),
% 11 phys_accs (mg), 12 phys_gyros (deg/s)
add_block('ARBlocks/ARDrone Wi-Fi  Block NAV Replay', [m '/Replay'], 'Position', [40 60 260 760]);
for p = setdiff(1:26, [1 2 11 12])
    t = sprintf('%s/T%d', m, p);
    add_block('simulink/Sinks/Terminator', t, 'Position', [300 40+26*p 315 52+26*p]);
    add_line(m, sprintf('Replay/%d', p), sprintf('T%d/1', p), 'autorouting', 'on');
end

add_block('simulink/Math Operations/Gain', [m '/mg2ms2'],    'Gain', '9.81/1000', 'Position', [340 360 400 390]);
add_block('simulink/Math Operations/Gain', [m '/degs2rads'], 'Gain', 'pi/180',    'Position', [340 420 400 450]);
add_block('simulink/Sources/Constant', [m '/ArtificialBias'], 'Value', 'b_art_degps*pi/180', 'Position', [340 470 420 500]);
add_block('simulink/Math Operations/Add', [m '/AddBias'], 'Inputs', '++', 'Position', [450 425 480 465]);
add_line(m, 'Replay/11', 'mg2ms2/1',    'autorouting', 'on');
add_line(m, 'Replay/12', 'degs2rads/1', 'autorouting', 'on');
add_line(m, 'degs2rads/1',      'AddBias/1', 'autorouting', 'on');
add_line(m, 'ArtificialBias/1', 'AddBias/2', 'autorouting', 'on');

% Optional accelerometer calibration on the ground: causal running mean of ax and
% ay over the first acc_cal_T seconds (drone at rest, assumed level), frozen after.
% acc_cal_T = 0 (default) switches it off.
add_block('simulink/Sources/Clock', [m '/Clock'], 'Position', [340 300 370 320]);
add_block('simulink/Sources/Constant', [m '/CalT'], 'Value', 'acc_cal_T', 'Position', [340 330 400 350]);
add_block('simulink/User-Defined Functions/MATLAB Function', [m '/CalibAcc'], 'Position', [440 300 540 350]);
set_fcn([m '/CalibAcc'], [ ...
'function [yc, b] = calib_acc(y, t, Tcal)' newline ...
'% Level-on-the-ground calibration of the accelerometer (ax, ay), causal.' newline ...
'% Running mean over t in [0.05, Tcal], frozen afterwards. Tcal = 0: off.' newline ...
'persistent s n' newline ...
'if isempty(s), s = zeros(2,1); n = 0; end' newline ...
'if Tcal > 0 && t < Tcal && t >= 0.05' newline ...
'    s = s + y(1:2);  n = n + 1;' newline ...
'end' newline ...
'if Tcal > 0 && n > 0, b = [s/n; 0]; else b = zeros(3,1); end' newline ...
'yc = y - b;' newline ...
'end']);
ch = sfroot().find('-isa', 'Stateflow.EMChart', 'Path', [m '/CalibAcc']);
ch.ChartUpdate = 'DISCRETE';             % it keeps state: needs a discrete sample time
ch.SampleTime  = '0.005';
add_line(m, 'mg2ms2/1', 'CalibAcc/1', 'autorouting', 'on');
add_line(m, 'Clock/1',  'CalibAcc/2', 'autorouting', 'on');
add_line(m, 'CalT/1',   'CalibAcc/3', 'autorouting', 'on');

add_filter(m);
add_line(m, 'CalibAcc/1', 'KalmanBucy/1', 'autorouting', 'on');
add_line(m, 'AddBias/1',  'KalmanBucy/2', 'autorouting', 'on');

% scopes: the filter against the AR.Drone onboard estimate
add_scope_pair(m, 'phi',   'Attitude/1', 'Replay/1', 'phi_hat (Batista KF)',   'phi (onboard estimate)',   [1060 330]);
add_scope_pair(m, 'theta', 'Attitude/2', 'Replay/2', 'theta_hat (Batista KF)', 'theta (onboard estimate)', [1060 420]);

add_logs(m, {'phi_hat',   'Attitude', 1; 'theta_hat', 'Attitude', 2; 'b_hat', 'Attitude', 3; ...
             'phi_ref',   'Replay',   1; 'theta_ref', 'Replay',   2; 'acc_cal', 'CalibAcc', 2});
save_system(m, fullfile(here, [m '.slx']));
fprintf('Saved %s\n', fullfile(here, [m '.slx']));
end

%% ------------------------------------------------------------------------
function build_sim_model(here)
% Synthetic experiment with known ground truth (same idea as the Sim_Bias_gyro
% model of other reports): sinusoidal roll and pitch, exact kinematics, noisy
% sensors with a constant rate gyro bias (and, optionally, an accelerometer bias).
m = new_model(here, 'ARDroneSim_BatistaKF');
mw = get_param(m, 'ModelWorkspace');
assignin(mw, 'sim_A_deg',    [8 6]);        % roll, pitch amplitude [deg]
assignin(mw, 'sim_f_hz',     [0.30 0.45]);  % roll, pitch frequency [Hz]
assignin(mw, 'b_true_degps', [2 -3 1]);     % rate gyro bias [deg/s] (the paper's simulation value)
assignin(mw, 'acc_bias',     [0 0 0]);      % accelerometer bias [m/s^2]
assignin(mw, 'sigma_a',      0.07);         % accelerometer noise std [m/s^2] (hover, Section 3)
assignin(mw, 'sigma_g',      0.01);         % rate gyro noise std [rad/s]
assignin(mw, 'seed0',        0);
set_param(m, 'StopTime', '60');

% Ground truth: roll = A1 sin(2 pi f1 t), pitch = A2 sin(2 pi f2 t + pi/3), yaw rate 0.
% Body rates from the Euler-angle kinematics and the accelerometer reading
% y = R' [0 0 -g]' (sensor convention: az = -g at rest) are exact.
add_block('simulink/Sources/Clock', [m '/Clock'], 'Position', [40 100 70 120]);
add_block('simulink/Sources/Constant', [m '/Amp'],  'Value', 'sim_A_deg*pi/180', 'Position', [40 140 110 160]);
add_block('simulink/Sources/Constant', [m '/Freq'], 'Value', 'sim_f_hz',        'Position', [40 180 110 200]);
add_block('simulink/Sources/Constant', [m '/Grav'], 'Value', '9.81',            'Position', [40 220 110 240]);
add_block('simulink/User-Defined Functions/MATLAB Function', [m '/Truth'], 'Position', [160 110 300 250]);
set_fcn([m '/Truth'], [ ...
'function [phi, theta, w, y] = truth(t, A, f, g)' newline ...
'% Exact attitude, body rates [rad/s] and accelerometer reading [m/s^2].' newline ...
'ph = A(1)*sin(2*pi*f(1)*t);              dph = A(1)*2*pi*f(1)*cos(2*pi*f(1)*t);' newline ...
'th = A(2)*sin(2*pi*f(2)*t + pi/3);       dth = A(2)*2*pi*f(2)*cos(2*pi*f(2)*t + pi/3);' newline ...
'w = [dph; dth*cos(ph); -dth*sin(ph)];' newline ...
'y = g*[sin(th); -sin(ph)*cos(th); -cos(ph)*cos(th)];' newline ...
'phi = ph*180/pi;  theta = th*180/pi;' newline ...
'end']);
add_line(m, 'Clock/1', 'Truth/1', 'autorouting', 'on');
add_line(m, 'Amp/1',   'Truth/2', 'autorouting', 'on');
add_line(m, 'Freq/1',  'Truth/3', 'autorouting', 'on');
add_line(m, 'Grav/1',  'Truth/4', 'autorouting', 'on');

% Sensors: truth + noise (+ bias)
add_block('simulink/Sources/Random Number', [m '/NoiseAcc'],  'Mean', '0', 'Variance', 'sigma_a^2', ...
    'Seed', 'seed0+[1 2 3]', 'SampleTime', '0.005', 'Position', [340 290 400 320]);
add_block('simulink/Sources/Random Number', [m '/NoiseGyro'], 'Mean', '0', 'Variance', 'sigma_g^2', ...
    'Seed', 'seed0+[4 5 6]', 'SampleTime', '0.005', 'Position', [340 400 400 430]);
add_block('simulink/Sources/Constant', [m '/AccBias'],  'Value', 'acc_bias', 'Position', [340 330 400 350]);
add_block('simulink/Sources/Constant', [m '/GyroBias'], 'Value', 'b_true_degps*pi/180', 'Position', [340 440 400 460]);
add_block('simulink/Math Operations/Add', [m '/SumAcc'],  'Inputs', '+++', 'Position', [440 190 470 250]);
add_block('simulink/Math Operations/Add', [m '/SumGyro'], 'Inputs', '+++', 'Position', [440 310 470 370]);
add_line(m, 'Truth/4', 'SumAcc/1',  'autorouting', 'on');
add_line(m, 'NoiseAcc/1', 'SumAcc/2', 'autorouting', 'on');
add_line(m, 'AccBias/1',  'SumAcc/3', 'autorouting', 'on');
add_line(m, 'Truth/3', 'SumGyro/1', 'autorouting', 'on');
add_line(m, 'NoiseGyro/1', 'SumGyro/2', 'autorouting', 'on');
add_line(m, 'GyroBias/1',  'SumGyro/3', 'autorouting', 'on');

add_filter(m);
add_line(m, 'SumAcc/1',  'KalmanBucy/1', 'autorouting', 'on');
add_line(m, 'SumGyro/1', 'KalmanBucy/2', 'autorouting', 'on');

% scopes: the filter against the truth, and the bias estimate against the true bias
add_scope_pair(m, 'phi',   'Attitude/1', 'Truth/1', 'phi_hat (Batista KF)',   'phi (truth)',   [1060 330]);
add_scope_pair(m, 'theta', 'Attitude/2', 'Truth/2', 'theta_hat (Batista KF)', 'theta (truth)', [1060 420]);
add_block('simulink/Sources/Constant', [m '/BiasTruth'], 'Value', 'b_true_degps', 'Position', [880 520 950 540]);
add_scope_pair(m, 'bias', 'Attitude/3', 'BiasTruth/1', 'b_hat [deg/s]', 'b true [deg/s]', [1060 510], 'deg/s');

add_logs(m, {'phi_hat', 'Attitude', 1; 'theta_hat', 'Attitude', 2; 'b_hat', 'Attitude', 3; ...
             'phi_true', 'Truth', 1;   'theta_true', 'Truth', 2});
save_system(m, fullfile(here, [m '.slx']));
fprintf('Saved %s\n', fullfile(here, [m '.slx']));
end

%% ------------------------------------------------------------------------
function m = new_model(here, name)
% Fresh model with the solver and the filter parameters in its workspace.
m = name;
if bdIsLoaded(m), close_system(m, 0); end
if isfile(fullfile(here, [m '.slx'])), delete(fullfile(here, [m '.slx'])); end
new_system(m);
Ts = 0.005;                  % navdata period [s]
g  = 9.81;
mw = get_param(m, 'ModelWorkspace');
assignin(mw, 'Xi', blkdiag((g*0.035)^2*Ts*eye(3), 1e-5*eye(3)));  % process noise intensity
assignin(mw, 'Th', 1e3*(0.07^2)*Ts*eye(3));                       % sensor noise intensity
assignin(mw, 'x0', [0; 0; -g; 0; 0; 0]);                          % level, zero bias
assignin(mw, 'P0', blkdiag(eye(3), 1e-2*eye(3)));
set_param(m, 'SolverType', 'Fixed-step', 'Solver', 'ode1', 'FixedStep', num2str(Ts), ...
    'StopTime', '30', 'EnablePacing', 'on', 'PacingRate', '5');
end

function add_filter(m)
% Kalman-Bucy filter + attitude. Inputs to wire: KalmanBucy/1 = y [m/s^2],
% KalmanBucy/2 = w_m [rad/s]. Outputs: Attitude/1..3 = phi, theta [deg], bias [deg/s].
kb = [m '/KalmanBucy'];
add_block('simulink/User-Defined Functions/MATLAB Function', kb, 'Position', [560 380 700 480]);
set_fcn(kb, [ ...
'function [xdot, Pdot] = kalman_bucy(y, w, x, P, Xi, Th)' newline ...
'% Continuous-time Kalman filter for the Batista et al. system (2)/(10).' newline ...
'% y: accelerometer [m/s^2], w: rate gyro [rad/s], x: [x1; bias], P: 6x6.' newline ...
'Sw = [0 -w(3) w(2); w(3) 0 -w(1); -w(2) w(1) 0];' newline ...
'Sy = [0 -y(3) y(2); y(3) 0 -y(1); -y(2) y(1) 0];' newline ...
'A  = [-Sw -Sy; zeros(3) zeros(3)];' newline ...
'C  = [eye(3) zeros(3)];' newline ...
'K  = P*C'' / Th;' newline ...
'xdot = A*x + K*(y - C*x);' newline ...
'Pdot = A*P + P*A'' + Xi - K*(C*P);' newline ...
'end']);
add_block('simulink/Continuous/Integrator', [m '/Int_x'], 'InitialCondition', 'x0', 'Position', [760 395 790 425]);
add_block('simulink/Continuous/Integrator', [m '/Int_P'], 'InitialCondition', 'P0', 'Position', [760 450 790 480]);
add_block('simulink/Sources/Constant', [m '/Xi'],    'Value', 'Xi', 'Position', [480 520 520 540]);
add_block('simulink/Sources/Constant', [m '/Theta'], 'Value', 'Th', 'Position', [480 560 520 580]);
add_line(m, 'Int_x/1', 'KalmanBucy/3', 'autorouting', 'on');
add_line(m, 'Int_P/1', 'KalmanBucy/4', 'autorouting', 'on');
add_line(m, 'Xi/1',    'KalmanBucy/5', 'autorouting', 'on');
add_line(m, 'Theta/1', 'KalmanBucy/6', 'autorouting', 'on');
add_line(m, 'KalmanBucy/1', 'Int_x/1', 'autorouting', 'on');
add_line(m, 'KalmanBucy/2', 'Int_P/1', 'autorouting', 'on');

att = [m '/Attitude'];
add_block('simulink/User-Defined Functions/MATLAB Function', att, 'Position', [880 380 1000 440]);
set_fcn(att, [ ...
'function [phi, theta, b] = attitude(x)' newline ...
'% Roll/pitch [deg] from the filtered body-frame vector (sensor convention' newline ...
'% az = -g at rest) and gyro bias estimate [deg/s].' newline ...
'x1 = x(1:3);' newline ...
'n  = norm(x1);' newline ...
'phi = atan2(-x1(2), -x1(3))*180/pi;' newline ...
'if n > eps' newline ...
'    theta = asin(x1(1)/n)*180/pi;' newline ...
'else' newline ...
'    theta = 0;' newline ...
'end' newline ...
'b = x(4:6)*180/pi;' newline ...
'end']);
add_line(m, 'Int_x/1', 'Attitude/1', 'autorouting', 'on');
end

function add_scope_pair(m, name, estPort, refPort, estName, refName, pos, ylab)
% Scope with two traces: the estimate and its reference, named for the legend.
if nargin < 8, ylab = 'deg'; end
mux = ['Mux_' name]; scp = ['Scope ' name];
add_block('simulink/Signal Routing/Mux', [m '/' mux], 'Inputs', '2', 'Position', [pos(1) pos(2) pos(1)+5 pos(2)+40]);
add_block('simulink/Sinks/Scope', [m '/' scp], 'Position', [pos(1)+60 pos(2) pos(1)+100 pos(2)+40]);
add_line(m, estPort, [mux '/1'], 'autorouting', 'on');
add_line(m, refPort, [mux '/2'], 'autorouting', 'on');
add_line(m, [mux '/1'], [scp '/1'], 'autorouting', 'on');
set_param(line_of(m, estPort), 'Name', estName);
set_param(line_of(m, refPort), 'Name', refName);
sc = get_param([m '/' scp], 'ScopeConfiguration');
sc.OpenAtSimulationStart = true;
sc.ShowLegend = true;
sc.Title = [name ' [' ylab ']'];
sc.YLabel = ylab;
sc.TimeSpan = 'Auto';
end

function add_logs(m, logs)
% To Workspace timeseries for post-processing: {variable, block, port; ...}
for i = 1:size(logs, 1)
    nm = ['Log_' logs{i,1}];
    add_block('simulink/Sinks/To Workspace', [m '/' nm], 'VariableName', logs{i,1}, ...
        'SaveFormat', 'Timeseries', 'Position', [1120 540+40*i 1180 570+40*i]);
    add_line(m, sprintf('%s/%d', logs{i,2}, logs{i,3}), [nm '/1'], 'autorouting', 'on');
end
end

function h = line_of(m, port)
% Line handle of an output port given as 'Block/N'.
parts = strsplit(port, '/');
lh = get_param([m '/' parts{1}], 'LineHandles');
h = lh.Outport(str2double(parts{2}));
end

function set_fcn(blk, code)
rt = sfroot;
ch = rt.find('-isa', 'Stateflow.EMChart', 'Path', blk);
ch.Script = code;
end

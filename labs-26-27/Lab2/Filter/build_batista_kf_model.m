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
% ode1 at the 5 ms navdata period).
%
% Both models share the same four-block core (KalmanBucy -> Int_x / Int_P ->
% Attitude), with the sensors as one 6-vector u = [accelerometer (m/s^2);
% rate gyro (rad/s)]. Constants (Xi, Th, amplitudes, ...) are parameters of the
% MATLAB Function blocks, read from the model workspace, so there are no
% constant blocks to wire. Signals are logged with Simulink signal logging
% (no To Workspace blocks): read them with kf_getlog.
%
% Re-run this script only to regenerate the models.

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
mw = get_param(m, 'ModelWorkspace');
assignin(mw, 'b_art_degps', [0 0 0]);   % artificial gyro bias [deg/s]
assignin(mw, 'acc_cal_T', 0);            % ground calibration window [s], 0 = off

% Navdata replay: one subsystem hides the 26-port Replay block of the DevKit
% and its unused outputs. Outputs: 1 = onboard [phi; theta] (deg),
% 2 = u = [accelerometer (m/s^2); rate gyro (rad/s)].
sub = [m '/Navdata replay'];
add_block('built-in/Subsystem', sub, 'Position', [60 60 190 180]);
add_block('ARBlocks/ARDrone Wi-Fi  Block NAV Replay', [sub '/Replay'], 'Position', [60 40 260 740]);
for p = setdiff(1:26, [1 2 11 12])
    add_block('simulink/Sinks/Terminator', sprintf('%s/T%d', sub, p), 'Position', [320 40+26*p 335 52+26*p]);
    add_line(sub, sprintf('Replay/%d', p), sprintf('T%d/1', p));
end
add_block('simulink/Signal Routing/Mux', [sub '/MuxAng'], 'Inputs', '2', 'Position', [420 60 425 100]);
add_block('simulink/Signal Routing/Mux', [sub '/MuxSens'], 'Inputs', '2', 'Position', [420 300 425 340]);
add_block('simulink/Math Operations/Gain', [sub '/Units'], 'Gain', '[9.81/1000*ones(3,1); pi/180*ones(3,1)]', ...
    'Multiplication', 'Element-wise(K.*u)', 'Position', [470 300 540 340]);   % mg -> m/s^2, deg/s -> rad/s
add_block('simulink/Sinks/Out1', [sub '/ang_ref'], 'Position', [620 70 650 84]);
add_block('simulink/Sinks/Out1', [sub '/u_raw'],   'Port', '2', 'Position', [620 312 650 326]);
add_line(sub, 'Replay/1', 'MuxAng/1');  add_line(sub, 'Replay/2', 'MuxAng/2');
add_line(sub, 'Replay/11', 'MuxSens/1'); add_line(sub, 'Replay/12', 'MuxSens/2');
add_line(sub, 'MuxAng/1', 'ang_ref/1');
add_line(sub, 'MuxSens/1', 'Units/1');
add_line(sub, 'Units/1', 'u_raw/1');

mk = Simulink.Mask.create(sub);
mk.Display = "disp('Replay')";

% Ground calibration of the accelerometer (causal running mean of ax, ay over the
% first acc_cal_T seconds, frozen afterwards; acc_cal_T = 0 switches it off),
% then the optional artificial gyro bias.
add_block('simulink/Sources/Clock', [m '/Clock'], 'Position', [60 230 90 250]);
add_block('simulink/User-Defined Functions/MATLAB Function', [m '/CalibAcc'], 'Position', [260 110 360 170]);
set_fcn([m '/CalibAcc'], [ ...
'function [uc, ab] = calib_acc(u, t)' newline ...
'% Level-on-the-ground calibration of the accelerometer (ax, ay), causal.' newline ...
'% Running mean over t in [0.05, acc_cal_T], frozen afterwards. acc_cal_T = 0: off.' newline ...
'persistent s n' newline ...
'if isempty(s), s = zeros(2,1); n = 0; end' newline ...
'if acc_cal_T > 0 && t < acc_cal_T && t >= 0.05' newline ...
'    s = s + u(1:2);  n = n + 1;' newline ...
'end' newline ...
'ab = zeros(6,1);' newline ...
'if acc_cal_T > 0 && n > 0, ab(1:2) = s/n; end' newline ...
'uc = u - ab;' newline ...
'end'], {'acc_cal_T'});
ch = sfroot().find('-isa', 'Stateflow.EMChart', 'Path', [m '/CalibAcc']);
ch.ChartUpdate = 'DISCRETE';             % it keeps state: needs a discrete sample time
ch.SampleTime  = '0.005';
add_block('simulink/Sources/Constant', [m '/GyroBias'], 'Value', '[0 0 0 b_art_degps*pi/180]', 'Position', [260 210 360 230]);
add_block('simulink/Math Operations/Add', [m '/Sum'], 'Inputs', '++', 'Position', [420 120 440 180]);
link(m, 'Navdata replay/2', 'CalibAcc/1');
link(m, 'Clock/1', 'CalibAcc/2');
link(m, 'CalibAcc/1', 'Sum/1');
link(m, 'GyroBias/1', 'Sum/2');

add_filter(m, [520 80]);
link(m, 'Sum/1', 'KalmanBucy/1');

% scope: the filter against the AR.Drone onboard estimate
add_block('simulink/Sinks/Scope', [m '/Scope angles'], 'NumInputPorts', '2', 'Position', [1080 60 1120 120]);
link(m, 'Attitude/1', 'Scope angles/1');
link(m, 'Navdata replay/1', 'Scope angles/2');
config_scope([m '/Scope angles'], 'Roll, pitch [deg]: KF (1) vs onboard (2)');

name_line(m, 'Navdata replay/2', 'u_raw');
name_line(m, 'CalibAcc/1', 'u_cal');
name_line(m, 'Sum/1', 'u');
log_line(m, 'Navdata replay/1', 'ang_ref');
log_line(m, 'Attitude/1', 'ang_hat');
log_line(m, 'Attitude/2', 'b_hat');
log_line(m, 'CalibAcc/2', 'acc_cal');
save_system(m, fullfile(here, [m '.slx']));
fprintf('Saved %s\n', fullfile(here, [m '.slx']));
end

%% ------------------------------------------------------------------------
function build_sim_model(here)
% Synthetic experiment with known ground truth: sinusoidal roll and pitch, exact
% kinematics, noisy sensors with a constant rate gyro bias (and, optionally, an
% accelerometer bias).
m = new_model(here, 'ARDroneSim_BatistaKF');
mw = get_param(m, 'ModelWorkspace');
assignin(mw, 'sim_A_deg',    [8 6]);        % roll, pitch amplitude [deg]
assignin(mw, 'sim_f_hz',     [0.30 0.45]);  % roll, pitch frequency [Hz]
assignin(mw, 'b_true_degps', [2 -3 1]);     % rate gyro bias [deg/s] (the paper's simulation value)
assignin(mw, 'acc_bias',     [0 0 0]);      % accelerometer bias [m/s^2]
assignin(mw, 'sigma_a',      0.07);         % accelerometer noise std [m/s^2] (hover, Section 3)
assignin(mw, 'sigma_g',      0.01);         % rate gyro noise std [rad/s]
assignin(mw, 'seed0',        0);
assignin(mw, 'g0',           9.81);
set_param(m, 'StopTime', '60');

% Truth: roll = A1 sin(2 pi f1 t), pitch = A2 sin(2 pi f2 t + pi/3), yaw rate 0.
% Body rates from the Euler-angle kinematics and the accelerometer reading
% y = R' [0 0 -g]' (sensor convention: az = -g at rest) are exact.
add_block('simulink/Sources/Clock', [m '/Clock'], 'Position', [40 70 70 90]);
add_block('simulink/User-Defined Functions/MATLAB Function', [m '/Truth'], 'Position', [130 50 250 110]);
set_fcn([m '/Truth'], [ ...
'function [ang, u] = truth(t)' newline ...
'% Exact [roll; pitch] [deg] and u = [accelerometer (m/s^2); rate gyro (rad/s)].' newline ...
'A = sim_A_deg*pi/180;  f = sim_f_hz;' newline ...
'ph = A(1)*sin(2*pi*f(1)*t);              dph = A(1)*2*pi*f(1)*cos(2*pi*f(1)*t);' newline ...
'th = A(2)*sin(2*pi*f(2)*t + pi/3);       dth = A(2)*2*pi*f(2)*cos(2*pi*f(2)*t + pi/3);' newline ...
'w = [dph; dth*cos(ph); -dth*sin(ph)];' newline ...
'y = g0*[sin(th); -sin(ph)*cos(th); -cos(ph)*cos(th)];' newline ...
'ang = [ph; th]*180/pi;' newline ...
'u = [y; w];' newline ...
'end'], {'sim_A_deg', 'sim_f_hz', 'g0'});
link(m, 'Clock/1', 'Truth/1');

% Sensors: truth + noise + bias (one 6-vector each)
add_block('simulink/Sources/Random Number', [m '/Noise'], 'Mean', '0', ...
    'Variance', '[sigma_a^2*ones(1,3) sigma_g^2*ones(1,3)]', 'Seed', 'seed0+(1:6)', ...
    'SampleTime', '0.005', 'Position', [130 130 190 160]);
add_block('simulink/Sources/Constant', [m '/Bias'], 'Value', '[acc_bias b_true_degps*pi/180]', 'Position', [130 190 220 210]);
add_block('simulink/Math Operations/Add', [m '/Sum'], 'Inputs', '+++', 'Position', [300 70 320 160]);
link(m, 'Truth/2', 'Sum/1');
link(m, 'Noise/1', 'Sum/2');
link(m, 'Bias/1',  'Sum/3');

add_filter(m, [420 70]);
link(m, 'Sum/1', 'KalmanBucy/1');

% scopes: the filter against the truth, and the bias estimate against the true bias
add_block('simulink/Sinks/Scope', [m '/Scope angles'], 'NumInputPorts', '2', 'Position', [980 40 1020 100]);
add_block('simulink/Sinks/Scope', [m '/Scope bias'],   'NumInputPorts', '2', 'Position', [980 150 1020 210]);
add_block('simulink/Sources/Constant', [m '/BiasTruth'], 'Value', 'b_true_degps', 'Position', [860 215 930 235]);
link(m, 'Attitude/1', 'Scope angles/1');
link(m, 'Truth/1',    'Scope angles/2');
link(m, 'Attitude/2', 'Scope bias/1');
link(m, 'BiasTruth/1', 'Scope bias/2');
config_scope([m '/Scope angles'], 'Roll, pitch [deg]: KF (1) vs truth (2)');
config_scope([m '/Scope bias'],   'Gyro bias [deg/s]: KF (1) vs true (2)');

name_line(m, 'Truth/2', 'u_true');
name_line(m, 'Sum/1', 'u');
log_line(m, 'Truth/1', 'ang_true');
log_line(m, 'Attitude/1', 'ang_hat');
log_line(m, 'Attitude/2', 'b_hat');
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
[Xi, Th] = kf_tuning;
assignin(mw, 'Xi', Xi);                                            % process noise intensity
assignin(mw, 'Th', Th);                                            % sensor noise intensity
assignin(mw, 'x0', [0; 0; -g; 0; 0; 0]);                          % level, zero bias
assignin(mw, 'P0', blkdiag(eye(3), 1e-2*eye(3)));
set_param(m, 'SolverType', 'Fixed-step', 'Solver', 'ode1', 'FixedStep', num2str(Ts), ...
    'StopTime', '30', 'EnablePacing', 'on', 'PacingRate', '5', ...
    'SignalLogging', 'on', 'SignalLoggingName', 'logsout', 'ReturnWorkspaceOutputs', 'on');
end

function add_filter(m, p)
% Kalman-Bucy filter + attitude block. p = [x y] of the KalmanBucy block.
% Inputs to wire: KalmanBucy/1 = u (6-vector). Outputs: Attitude/1 = [phi; theta]
% (deg), Attitude/2 = gyro bias estimate (deg/s).
kb = [m '/KalmanBucy'];
add_block('simulink/User-Defined Functions/MATLAB Function', kb, 'Position', [p(1) p(2) p(1)+140 p(2)+90]);
set_fcn(kb, [ ...
'function [xdot, Pdot] = kalman_bucy(u, x, P)' newline ...
'% Continuous-time Kalman filter for the Batista et al. system (2)/(10).' newline ...
'% u = [accelerometer (m/s^2); rate gyro (rad/s)], x = [x1; bias], P: 6x6.' newline ...
'y = u(1:3);  w = u(4:6);' newline ...
'Sw = [0 -w(3) w(2); w(3) 0 -w(1); -w(2) w(1) 0];' newline ...
'Sy = [0 -y(3) y(2); y(3) 0 -y(1); -y(2) y(1) 0];' newline ...
'A  = [-Sw -Sy; zeros(3) zeros(3)];' newline ...
'C  = [eye(3) zeros(3)];' newline ...
'K  = P*C'' / Th;' newline ...
'xdot = A*x + K*(y - C*x);' newline ...
'Pdot = A*P + P*A'' + Xi - K*(C*P);' newline ...
'end'], {'Xi', 'Th'});
add_block('simulink/Continuous/Integrator', [m '/Int_x'], 'InitialCondition', 'x0', ...
    'Position', [p(1)+230 p(2)-60 p(1)+260 p(2)-30]);
add_block('simulink/Continuous/Integrator', [m '/Int_P'], 'InitialCondition', 'P0', ...
    'Position', [p(1)+230 p(2)+120 p(1)+260 p(2)+150]);
link(m, 'KalmanBucy/1', 'Int_x/1');
link(m, 'KalmanBucy/2', 'Int_P/1');
link(m, 'Int_x/1', 'KalmanBucy/2');
link(m, 'Int_P/1', 'KalmanBucy/3');
name_line(m, 'KalmanBucy/1', 'x_dot');
name_line(m, 'KalmanBucy/2', 'P_dot');
name_line(m, 'Int_P/1', 'P');

att = [m '/Attitude'];
add_block('simulink/User-Defined Functions/MATLAB Function', att, ...
    'Position', [p(1)+350 p(2)-75 p(1)+470 p(2)-15]);
set_fcn(att, [ ...
'function [ang, b] = attitude(x)' newline ...
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
'ang = [phi; theta];' newline ...
'b = x(4:6)*180/pi;' newline ...
'end']);
link(m, 'Int_x/1', 'Attitude/1');
end

function link(m, a, b)
% Connect two ports with automatic routing (keeps feedback lines off the blocks).
add_line(m, a, b, 'autorouting', 'on');
end

function config_scope(blk, ylab)
sc = get_param(blk, 'ScopeConfiguration');
sc.OpenAtSimulationStart = true;
sc.ShowLegend = true;
sc.TimeSpan = 'Auto';
sc.YLabel = ylab;
end

function name_line(m, port, nm)
% Name the signal leaving an output port given as 'Block/N' (shown on the diagram).
set_param(port_of(m, port), 'Name', nm);
end

function log_line(m, port, nm)
% Name and log (signal logging) the signal leaving an output port 'Block/N'.
h = port_of(m, port);
set_param(h, 'Name', nm);
set_param(h, 'DataLogging', 'on');
end

function h = port_of(m, port)
parts = strsplit(port, '/');
n  = str2double(parts{end});
ph = get_param([m '/' strjoin(parts(1:end-1), '/')], 'PortHandles');
h  = ph.Outport(n);
end

function set_fcn(blk, code, params)
% Script of a MATLAB Function block; params = names of model-workspace
% parameters the function reads (declared as Parameter data of the block).
rt = sfroot;
ch = rt.find('-isa', 'Stateflow.EMChart', 'Path', blk);
ch.Script = code;
if nargin > 2
    for i = 1:numel(params)
        d = Stateflow.Data(ch);
        d.Name  = params{i};
        d.Scope = 'Parameter';
    end
end
end

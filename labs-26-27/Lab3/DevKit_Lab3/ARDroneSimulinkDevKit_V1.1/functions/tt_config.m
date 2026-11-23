function cfg = tt_config()
%TT_CONFIG  Experiment selection and all tuning parameters for Lab 3.
%
%   This is the only file you need to touch between runs / flights.
%   It is read by desired_trajectory_self.m and tt_controller_self.m.
%
%   How to switch experiment or retune:
%       1) change EXPERIMENT (or any parameter) below,
%       2) save the file,
%       3) press Run in Simulink.
%   The MATLAB Function blocks notice the changed file and rebuild
%   themselves, so the model does not have to be closed or reloaded.
%
%   Experiments (questions of Section 3 of the lab guide):
%       0  hover at the position where tracking was enabled
%       1  Q3.2/3.3  straight line, 0.1 m/s, constant yaw
%       2  Q3.4/3.5  circle, R = 1 m, 0.1 rad/s, constant yaw
%       3  Q3.6      same circle, yaw pointing to the centre
%       4  Q3.6      same as 3, but the disturbance is estimated in the
%                    body frame (the suggested controller change)
%       5  Q3.7      line-of-sight (LOS) straight-line path following
%
%   Numeric codes are used instead of strings so that the file stays
%   compatible with code generation (MATLAB Function blocks).
%#codegen

EXPERIMENT = 1;   % <-- select the experiment here

% ---------------------------------------------------------------------
% Codes (do not change)
%   traj.type : 0 hover, 1 line, 2 circle, 3 LOS
%   traj.yaw  : 0 constant (traj.psi), 1 pointing to the circle centre
%   ctrl.adapt: 0 off (pure LQR / PD), 1 inertial-frame estimate of w,
%               2 body-frame estimate of w
% ---------------------------------------------------------------------

% ======================= trajectory planner ==========================
traj.type   = 0;
traj.yaw    = 0;
traj.psi    = 0;      % constant desired yaw [rad] (0 = heading at enable)
traj.height = 1.0;    % flight height [m] (pd(3) = -height, NED)

% straight line (Q3.2)
traj.line_speed  = 0.1;   % [m/s]
traj.line_dir    = 0;     % direction of the line in the x-y plane [rad]
traj.line_length = 3.0;   % [m] after this distance pd stops (Inf = never)

% circle (Q3.4, Q3.6)
traj.circ_radius = 1.0;   % [m]
traj.circ_omega  = 0.1;   % [rad/s] (negative = clockwise seen from above)
traj.circ_phase0 = pi;    % angle of the start point seen from the centre;
                          % pi puts the centre 1 m ahead (+x) of the start,
                          % so "yaw to centre" starts with psi_d = 0

% line of sight (Q3.7):  psi_d = dir + atan(-e/Delta), v_d = V [cos; sin]
traj.los_V      = 0.1;    % [m/s] desired speed V
traj.los_Delta  = 1.0;    % [m]   look-ahead distance Delta
traj.los_dir    = 0;      % direction of the path [rad]
traj.los_point  = [0; 1]; % a point of the path [m]
traj.los_rel    = 1;      % 1: los_point is relative to the position where
                          %    tracking was enabled, 0: absolute
traj.los_length = Inf;    % [m] along-track length, then stop (Inf = never)

% ======================= controller =================================
% LQR on the error system  x~ = [p~; v~],  x~' = A x~ + B u,  u = -K x~
% with Q = blkdiag(q_p I, q_v I) and weight rho on the input.
ctrl.q_p   = 1.0;
ctrl.q_v   = 1.0;
ctrl.rho   = 1.0;
ctrl.g     = 9.81;     % [m/s^2]

ctrl.adapt = 1;        % see the codes above
ctrl.k_w   = 0.5;      % adaptation gain k_w:  w^' = k_w B'P x~
ctrl.w_max = 2.0;      % [m/s^2] bound on each component of w^ (anti-windup)

ctrl.max_tilt = 0.20;  % [rad] bound on the roll and pitch references

ctrl.k_z      = 1.0;   % vertical channel  w_r = -k_z (h - h_d)  [1/s]
ctrl.k_psi    = 1.5;   % yaw PI: dot_psi_r = -k_psi psi_e - ki_psi int(psi_e)
ctrl.ki_psi   = 0.5;

% Conversion from physical references to the normalised AR.Drone inputs
% [-1, 1]. Values = static gains of the identified models in lib/
% (ssRoll, ssPitch, ssYaw, ssH). Check them against the drone settings
% (euler_angle_max, control_yaw, control_vz_max) before flying.
ctrl.roll_per_unit    = 0.2777;  % [rad]   roll reached for input 1
ctrl.pitch_per_unit   = 0.4082;  % [rad]   pitch reached for input 1
ctrl.yawrate_per_unit = 1.265;   % [rad/s] yaw rate for input 1
ctrl.vz_per_unit      = 0.885;   % [m/s]   climb rate for input 1

% ======================= experiments ================================
switch EXPERIMENT
    case 0  % hover
        traj.type = 0;
    case 1  % Q3.2 / Q3.3
        traj.type = 1;
    case 2  % Q3.4 / Q3.5
        traj.type = 2;  traj.yaw = 0;
    case 3  % Q3.6
        traj.type = 2;  traj.yaw = 1;
    case 4  % Q3.6, suggested controller change
        traj.type = 2;  traj.yaw = 1;  ctrl.adapt = 2;
    case 5  % Q3.7
        traj.type = 3;
    otherwise
        traj.type = 0;
end

cfg.experiment = EXPERIMENT;
cfg.traj = traj;
cfg.ctrl = ctrl;
end

function [pd, dot_pd, ddot_pd, psi_d, type] = desired_trajectory_self(dt, p0, states)
%#codegen
% ------------------------------------------------------------------------
% Trajectory planner (Q3.2, Q3.4, Q3.6, Q3.7). Selected in tt_config.m.
%
% inputs:
% dt     - time since tracking was enabled [s]
% p0     - position when tracking was enabled, [x0; y0; z0] (NED)
% states - [phi theta psi, u v h_dot, x y h] (only used by the LOS planner)
%
% outputs (NED, z = -h):
% pd, dot_pd, ddot_pd - desired position, velocity and acceleration
% psi_d               - desired yaw angle [rad]
% ------------------------------------------------------------------------

persistent pd_los dt_prev

cfg = tt_config();
tr = cfg.traj;

pd      = [p0(1:2); 1];
dot_pd  = [0;0;0];
ddot_pd = [0;0;0];
psi_d   = tr.psi;
type = tr.type;
if isempty(pd_los)
    pd_los  = p0(1:2);
    dt_prev = 0;
end

switch tr.type
    case 1
        % ------------- straight line, constant speed and yaw ------------
        t_dir = [cos(tr.line_dir); sin(tr.line_dir)];
        s = tr.line_speed*dt;              % travelled distance
        if s < tr.line_length
            pd(1:2)     = p0(1:2) + s*t_dir;
            dot_pd(1:2) = tr.line_speed*t_dir;
        else                               % end of the line: hold
            pd(1:2)     = p0(1:2) + tr.line_length*t_dir;
        end

    case 2
        % ------------- circle through the start point -------------------
        R  = tr.circ_radius;
        w  = tr.circ_omega;
        a0 = tr.circ_phase0;
        a  = a0 + w*dt;
        c  = p0(1:2) - R*[cos(a0); sin(a0)];   % centre
        pd(1:2)      = c + R*[cos(a); sin(a)];
        dot_pd(1:2)  = R*w*[-sin(a); cos(a)];
        ddot_pd(1:2) = -R*w^2*[cos(a); sin(a)];
        if tr.yaw == 1                         % look at the centre
            psi_d = wrap_pi(a + pi);
        end

    case 3
        % ------------- line of sight, straight-line path following ------
        if dt <= 0 || dt < dt_prev              % tracking (re)started
            pd_los  = p0(1:2);
            dt_prev = dt;
        end
        h = dt - dt_prev;                       % step since last call
        dt_prev = dt;

        t_dir = [cos(tr.los_dir); sin(tr.los_dir)];
        n_dir = [-sin(tr.los_dir); cos(tr.los_dir)];
        pL = tr.los_point;
        if tr.los_rel == 1
            pL = pL + p0(1:2);
        end

        psi = states(3);
        p   = states(7:8);
        Rz2 = [cos(psi) -sin(psi); sin(psi) cos(psi)];
        v   = Rz2*states(4:5);                  % inertial horizontal velocity

        e     = n_dir'*(p - pL);                % cross-track error
        e_dot = n_dir'*v;
        D     = tr.los_Delta;
        psi_d = wrap_pi(tr.los_dir + atan(-e/D));          % eq. (5)
        psi_d_dot = -D*e_dot/(D^2 + e^2);

        V = tr.los_V;
        if t_dir'*(p - pL) >= tr.los_length      % end of the path: stop
            V = 0;
        end
        vel = V*[cos(psi_d); sin(psi_d)];                    % eqs. (6), (7)
        acc = V*psi_d_dot*[-sin(psi_d); cos(psi_d)];

        pd_los = pd_los + h*vel;                % integrate x_d, y_d
        pd(1:2)      = pd_los;
        dot_pd(1:2)  = vel;
        ddot_pd(1:2) = acc;

    otherwise
        % ------------- hover at p0 (defaults above) ---------------------
end

end

function a = wrap_pi(a)
%#codegen
a = mod(a + pi, 2*pi) - pi;
end

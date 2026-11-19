function [w_r, dot_psi_r, theta_r, phi_r, dot_xi] = tt_controller_self(states, pd, dot_pd, ddot_pd,psi_d,xi)
%#codegen

% ------------------------------------------------------------------------
% inputs:
% states - structure with state variables
% pd - desired position
% dot_pd - desired velocity
% ddot_pt - desired acceleration
% psi_d - desired yaw angle
% xi - integral state (four variables)
%
% outputs:
% w_r - vertical velocity reference
% dot_psi_r - yaw rate reference
% theta_r - pitch angle reference
% phi_r - roll angle reference
% dot_xi - integral state derivative
%
% All four references are normalised AR.Drone inputs in [-1, 1].
% xi(1:2) - disturbance estimate w^ (x, y), xi(3) unused,
% xi(4)   - integral of the yaw error.
% Gains and options: tt_config.m
% ------------------------------------------------------------------------

cfg = tt_config();
c = cfg.ctrl;

lambda = states(1:3,1); % Euler angles
psi = lambda(3);
Rz = [cos(psi) -sin(psi) 0; sin(psi) cos(psi) 0; 0 0 1];
psi_e = normalize_angle(psi-psi_d);

p = states(7:9,1); % position vector [x; y; h]
p(3) = -p(3); % height is symmetric of z (NED: z = -h)
vb = states(4:6,1); % [u; v; h_dot], u and v in the yaw-rotated frame
dot_p = Rz*[vb(1:2); 0]; % same rotation as the position estimator
dot_p(3) = -vb(3);

%% ---------------- horizontal channel (Q2.2 - Q2.5, Q3.1) ---------------
% LQR for x~' = [0 I; 0 0] x~ + [0; I] u, per axis, closed form of the
% Riccati solution P = [p1 p2; p2 p3]:  K = (1/rho) B'P = [kp kd].
kp = sqrt(c.q_p/c.rho);
kd = sqrt(c.q_v/c.rho + 2*kp);
BtP = c.rho*[kp kd];

p_e = p(1:2) - pd(1:2);
v_e = dot_p(1:2) - dot_pd(1:2);
s = BtP(1)*p_e + BtP(2)*v_e;            % B'P x~

% disturbance estimate (adaptation law w^' = k_w B'P x~)
Rz2 = Rz(1:2,1:2);
xi_w = min(max(xi(1:2), -c.w_max), c.w_max);
if c.adapt == 1                          % w^ in the inertial frame
    w_hat = xi_w;
    dw = c.k_w*s;
elseif c.adapt == 2                      % w^ = Rz(psi) w^_b, body frame
    w_hat = Rz2*xi_w;
    dw = c.k_w*(Rz2'*s);
else
    w_hat = [0;0];
    dw = [0;0];
end
for i = 1:2                              % stop integrating at the bound
    if abs(xi(i)) >= c.w_max && dw(i)*xi(i) > 0
        dw(i) = 0;
    end
end

% eq. (4): u* = Rz(-psi) (K x~ + g e3 + w^ - pdd_d) = Rz(-psi) (g e3 - a)
a = -kp*p_e - kd*v_e - w_hat + ddot_pd(1:2);   % desired acceleration
u_star = Rz'*([0; 0; c.g] - [a; 0]);

% Q2.1: u* = (T/m) [cos(phi) sin(theta); -sin(phi); cos(phi) cos(theta)]
T_m   = norm(u_star);                    % thrust/mass (not an AR.Drone input)
theta = atan2(u_star(1), u_star(3));
phi   = asin(-u_star(2)/T_m);

theta = sat(theta, c.max_tilt);
phi   = sat(phi,   c.max_tilt);
theta_r = sat(theta/c.pitch_per_unit, 1);
phi_r   = sat(phi/c.roll_per_unit, 1);

%% ---------------- vertical channel (Q3.1) ------------------------------
% w_r = -k_z (z - z_d) with z read as the height (the AR.Drone vertical
% speed input is positive upwards)
h   = -p(3);
h_d = -pd(3);
w_r = sat(-c.k_z*(h - h_d)/c.vz_per_unit, 1);

%% ---------------- yaw channel (Q2.8): PI -------------------------------
dot_psi_r = sat((-c.k_psi*psi_e - c.ki_psi*xi(4))/c.yawrate_per_unit, 1);
if c.ki_psi > 0
    dpsi = psi_e;
else
    dpsi = 0;
end

dot_xi = [dw; 0; dpsi];


function y = sat(x, m)
%#codegen
y = min(max(x, -m), m);

function R = euler2R(l)
%#codegen
% returns the rotation matrix for a given vector of euler angles

phi = l(1); theta = l(2); psi = l(3);
R = [cos(theta)*cos(psi)...
      sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi)...
      cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi);
   cos(theta)*sin(psi)...
   sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi)...
   cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi);
   -sin(theta)...
   sin(phi)*cos(theta)...
   cos(phi)*cos(theta)];

function angleOut = normalize_angle(angleIn)
%#codegen

angleOut = mod(angleIn, 2*pi) ;
if angleOut > pi
    angleOut = -2*pi + angleOut ;
end

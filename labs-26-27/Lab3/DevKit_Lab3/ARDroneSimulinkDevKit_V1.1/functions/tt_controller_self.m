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
% ------------------------------------------------------------------------

lambda = states(1:3,1); % Euler angles
psi = lambda(3);
R=euler2R(lambda);
Rz = [cos(psi) -sin(psi) 0; sin(psi) cos(psi) 0; 0 0 1];
psi_e = normalize_angle(psi-psi_d);

p = states(7:9,1); % position vector
p(3) = -p(3); % height is symmetric of z
vb = states(4:6,1); % velocity vector in body coordinates
dot_p = R*vb; % body velocity rotated to inertial frame 
dot_p(3) = -dot_p(3);

%!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
% replace with the necessary code to implement the trajectory tracking
% controller
w_r= 0;
dot_psi_r = 0;
theta_r = 0;
phi_r = 0;
dot_xi = [0;0;0;0];
%!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!


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


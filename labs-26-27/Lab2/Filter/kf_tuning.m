function [Xi, Th] = kf_tuning(thfactor, bq)
% KF_TUNING  Noise intensities of the Batista et al. filter (Lab 2, Section 5).
%
%   [Xi, Th] = kf_tuning            % the values used in the report
%   [Xi, Th] = kf_tuning(1e2, 1e-6) % sweep: measurement-noise factor, bias random walk
%
%   Xi = blkdiag((g*sigma_g)^2*Ts*I3, bq*I3)   process noise intensity
%   Th = thfactor*sigma_a^2*Ts*I3              measurement noise intensity
%
% sigma_g = 0.035 rad/s and sigma_a = 0.07 m/s^2 are the hover values of
% Section 3; Ts = 5 ms is the navdata period.

arguments
    thfactor (1,1) double = 1e3
    bq       (1,1) double = 1e-5
end
Ts = 0.005;
g  = 9.81;
Xi = blkdiag((g*0.035)^2*Ts*eye(3), bq*eye(3));
Th = thfactor*0.07^2*Ts*eye(3);
end

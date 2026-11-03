function p = quadrotor_parameters()
%QUADROTOR_PARAMETERS  Physical parameters of the quadrotor UAV.
%
% All values are taken exactly from Section IV of the paper:
%   Lee, Leok, McClamroch, "Geometric Tracking Control of a Quadrotor
%   UAV on SE(3)", IEEE CDC 2010.
%
% Coordinate convention (paper, Fig. 1):
%   - Inertial frame {i1,i2,i3}.
%   - Body-fixed frame {b1,b2,b3}; b3 points downward, opposite to thrust.
%   - R : body-to-inertial rotation matrix, R in SO(3).
%   - Total thrust acts along -b3, i.e. inertial force = -f*R*e3.

p.m      = 4.34;              % total mass            [kg]
p.J      = diag([0.0820, 0.0845, 0.1377]); % inertia matrix  [kg m^2]
p.d      = 0.315;             % rotor distance from COM [m]
p.c_tau_f = 8.004e-4;         % thrust-to-torque coeff [m]
p.g      = 9.81;              % gravity               [m/s^2]
p.e3     = [0; 0; 1];         % inertial vertical unit vector

end

function des = case2_trajectory(t)
%CASE2_TRAJECTORY  Numerical Example II: upside-down recovery (Section IV).
%
%   Desired position:
%       xd(t) = [0; 0; 0]
%
%   Desired heading:
%       b1d(t) = [1; 0; 0]
%
%   Simulation duration: 0 to 6 s.
%   Initial conditions: x(0)=[0;0;0], v(0)=[0;0;0],
%       R(0) = [ 1   0       0
%                0  -0.9995 -0.0314
%                0   0.0314 -0.9995 ],
%       Omega(0) = [0;0;0].

xd      = [0; 0; 0];
xd_dot  = [0; 0; 0];
xd_ddot = [0; 0; 0];
xd_ddd  = [0; 0; 0];
xd_dddd = [0; 0; 0];

b1d     = [1; 0; 0];
b1d_dot = [0; 0; 0];
b1d_ddot= [0; 0; 0];

des.xd      = xd;
des.xd_dot  = xd_dot;
des.xd_ddot = xd_ddot;
des.xd_ddd  = xd_ddd;
des.xd_dddd = xd_dddd;
des.b1d     = b1d;
des.b1d_dot = b1d_dot;
des.b1d_ddot= b1d_ddot;

end

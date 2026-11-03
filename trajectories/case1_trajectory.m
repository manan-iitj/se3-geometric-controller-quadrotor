function des = case1_trajectory(t)
%CASE1_TRAJECTORY  Numerical Example I: elliptical helix (Section IV).
%
%   Desired position:
%       xd(t) = [ 0.4*t
%                 0.4*sin(pi*t)
%                 0.6*cos(pi*t) ]
%
%   Desired heading:
%       b1d(t) = [ cos(pi*t)
%                  sin(pi*t)
%                  0 ]
%
%   Simulation duration: 0 to 10 s.
%   Initial conditions: x(0)=[0;0;0], v(0)=[0;0;0], R(0)=I, Omega(0)=[0;0;0].

pi = 3.141592653589793;

% ----- position and derivatives -------------------------------------------
xd      = [ 0.4*t;
            0.4*sin(pi*t);
            0.6*cos(pi*t) ];

xd_dot  = [ 0.4;
            0.4*pi*cos(pi*t);
           -0.6*pi*sin(pi*t) ];

xd_ddot = [ 0;
           -0.4*pi^2*sin(pi*t);
           -0.6*pi^2*cos(pi*t) ];

xd_ddd  = [ 0;
           -0.4*pi^3*cos(pi*t);
            0.6*pi^3*sin(pi*t) ];

xd_dddd = [ 0;
            0.4*pi^4*sin(pi*t);
            0.6*pi^4*cos(pi*t) ];

% ----- heading and derivatives --------------------------------------------
b1d     = [ cos(pi*t);
            sin(pi*t);
            0 ];

b1d_dot = [ -pi*sin(pi*t);
             pi*cos(pi*t);
             0 ];

b1d_ddot = [ -pi^2*cos(pi*t);
             -pi^2*sin(pi*t);
             0 ];

des.xd      = xd;
des.xd_dot  = xd_dot;
des.xd_ddot = xd_ddot;
des.xd_ddd  = xd_ddd;
des.xd_dddd = xd_dddd;
des.b1d     = b1d;
des.b1d_dot = b1d_dot;
des.b1d_ddot= b1d_ddot;

end

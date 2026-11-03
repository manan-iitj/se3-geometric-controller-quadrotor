function [f, M, dbg] = geometric_controller(t, x, v, R, Om, p, K, casefun)
%GEOMETRIC_CONTROLLER  Geometric tracking control on SE(3) (Eqs. (6)-(16)).
%
%   Tracking errors (Eqs. (6)-(11)):
%       ex     = x - xd
%       ev     = v - xd_dot
%       Psi    = 0.5 * trace(I - Rd' * R)                         (8)
%       eR     = 0.5 * vee( Rd' * R - R' * Rd )                   (10)
%       eOm    = Om - R' * Rd * Omega_d                           (11)
%
%   Desired attitude (Eq. (12)):
%       b3d = -a / ||a||,   a = -kx*ex - kv*ev - m*g*e3 + m*xdd
%       b2d = cross(b3d,b1d)/||cross(b3d,b1d)||
%       Rd  = [ cross(b2d,b3d), b2d, b3d ]
%
%   Control inputs (Eqs. (15)-(16)):
%       f = -a' * R * e3
%       M = -kR*eR - kOmega*eOm + Om x (J Om)
%           - J*( hat(Om) * R' * Rd * Omega_d - R' * Rd * Omega_dot_d )
%
%   Omega_d and Omega_dot_d are computed consistently from the desired
%   attitude matrix and its analytic time derivatives.
%
%   Outputs:
%     f     - collective thrust (scalar)
%     M     - control moment (3x1) in body frame
%     dbg   - debug struct (errors, Psi, Rd, rotor thrusts, ...)

% --- Desired trajectory and derivatives -----------------------------------
des = casefun(t);
xd      = des.xd;
xd_dot  = des.xd_dot;
xd_ddot = des.xd_ddot;
xd_ddd  = des.xd_ddd;
xd_dddd = des.xd_dddd;
b1d     = des.b1d;
b1d_dot = des.b1d_dot;
b1d_ddot= des.b1d_ddot;

% --- Tracking errors ------------------------------------------------------
ex = x - xd;
ev = v - xd_dot;

% --- Vector a and desired third body axis (Eq. (12)) ----------------------
a = -K.kx*ex - K.kv*ev - p.m*p.g*p.e3 + p.m*xd_ddot;
anorm = norm(a);
b3d = -a / anorm;

% --- Collective thrust (Eq. (15)) -----------------------------------------
f = -a' * (R * p.e3);

% --- First/second time derivatives (needed for Omega_d, Omega_dot_d) ------
% v_dot from Eq. (3)
v_dot = p.g*p.e3 - (f/p.m) * (R * p.e3);

% a_dot = -kx*ev - kv*(v_dot - xdd) + m*xdd_dot
a_dot = -K.kx*ev - K.kv*(v_dot - xd_ddot) + p.m*xd_ddd;

% f_dot = -( a_dot' R e3 + a' R_dot e3 )
%        = -( a_dot' R e3 + a' R hat(Om) e3 )
R_Om_e3 = R * (hat_map(Om) * p.e3);
f_dot = -(a_dot' * (R * p.e3) + a' * R_Om_e3);

% v_dot_dot = -(f_dot/m) R e3 - (f/m) R_dot e3
v_dot_dot = -(f_dot/p.m) * (R * p.e3) - (f/p.m) * R_Om_e3;

% a_ddot = -kx*(v_dot - xdd) - kv*v_dot_dot + kv*xdd_dot + m*xdd_dddd
a_ddot = -K.kx*(v_dot - xd_ddot) - K.kv*v_dot_dot ...
         + K.kv*xd_ddd + p.m*xd_dddd;

% --- Desired attitude Rd and derivatives ----------------------------------
% Projector onto plane normal to b3d
P  = eye(3) - b3d*b3d';
b3d_dot = -(P * a_dot) / anorm;

P_dot = -(b3d_dot*b3d' + b3d*b3d_dot');
b3d_ddot = -( (P_dot*a_dot + P*a_ddot) / anorm ...
            - (P*a_dot)*(a'*a_dot)/anorm^3 );

% b2d = cross(b3d,b1d)/||cross(b3d,b1d)||
c  = cross(b3d, b1d);
nc = norm(c);
b2d = c / nc;

c_dot = cross(b3d_dot, b1d) + cross(b3d, b1d_dot);

% b2d_dot = Pc * c_dot / nc   (projector form, equivalent)
Pc = eye(3) - b2d*b2d';
b2d_dot = (Pc * c_dot) / nc;

c_ddot = cross(b3d_ddot, b1d) + 2*cross(b3d_dot, b1d_dot) ...
       + cross(b3d, b1d_ddot);

% Differentiate b2d_dot = Pc * c_dot / nc
nc_dot = b2d' * c_dot;
Pc_dot = -(b2d_dot*b2d' + b2d*b2d_dot');
b2d_ddot = (Pc_dot*c_dot + Pc*c_ddot) / nc - (Pc*c_dot)*nc_dot / nc^2;

% Rd = [ b2d x b3d,  b2d,  b3d ]
Rd = [ cross(b2d, b3d), b2d, b3d ];

% Rd_dot columns
Rd_dot = [ cross(b2d_dot, b3d) + cross(b2d, b3d_dot), b2d_dot, b3d_dot ];

% Rd_ddot columns
Rd_ddot = [ cross(b2d_ddot, b3d) + 2*cross(b2d_dot, b3d_dot) ...
            + cross(b2d, b3d_ddot), b2d_ddot, b3d_ddot ];

% --- Desired angular velocity and its derivative --------------------------
% hat(Omega_d) = Rd' * Rd_dot
Omega_d = vee_map(Rd' * Rd_dot);

% hat(Omega_dot_d) = Rd' * Rd_ddot - hat(Omega_d)^2
Omega_dot_d = vee_map( Rd' * Rd_ddot - (Rd' * Rd_dot)*(Rd' * Rd_dot) );

% --- Attitude / angular velocity tracking errors --------------------------
eR = 0.5 * vee_map( Rd'*R - R'*Rd );                     % Eq. (10)
eOm = Om - (R'*Rd) * Omega_d;                            % Eq. (11)

% --- Control moment (Eq. (16)) --------------------------------------------
RtRd = R' * Rd;
M = -K.kR*eR - K.kOmega*eOm + cross(Om, p.J*Om) ...
    - p.J * ( hat_map(Om) * RtRd * Omega_d - RtRd * Omega_dot_d );

% --- Debug info -----------------------------------------------------------
dbg.t      = t;
dbg.x      = x;  dbg.v = v;  dbg.R = R;  dbg.Om = Om;
dbg.xd     = xd; dbg.xd_dot = xd_dot; dbg.xd_ddot = xd_ddot;
dbg.ex     = ex; dbg.ev = ev; dbg.eR = eR; dbg.eOm = eOm;
dbg.f      = f; dbg.M = M;
dbg.Rd     = Rd; dbg.b3d = b3d; dbg.b1d = b1d;
dbg.Psi    = 0.5*trace(eye(3) - Rd'*R);                 % Eq. (8)
dbg.Omega_d   = Omega_d;
dbg.Omega_dot_d = Omega_dot_d;

% Rotor thrusts (Eq. (1) inverse allocation)
[~, ~, dbg.f_rotors] = rotor_allocation(f, M, p);

end

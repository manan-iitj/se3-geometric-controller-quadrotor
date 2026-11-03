function dxdt = quadrotor_dynamics(t, state, p, K, casefun)
%QUADROTOR_DYNAMICS  Equations of motion (Eqs. (2)-(5) of the paper).
%
%   x_dot  = v                                          (2)
%   m v_dot = m g e3 - f R e3                           (3)
%   R_dot  = R hat(Omega)                               (4)
%   J Om_dot + Om x (J Om) = M                          (5)
%
%   state = [x(3); v(3); R(9) column-major; Om(3)]  (15x1)
%
%   Control inputs f, M come from the geometric tracking controller
%   (Eqs. (6)-(16)); individual rotor thrusts from Eq. (1).

% --- Unpack state ---------------------------------------------------------
x  = state(1:3);
v  = state(4:6);
R  = reshape(state(7:15), 3, 3);     % column-major
Om = state(16:18);

% --- Geometric tracking controller (Eqs. (6)-(16)) ------------------------
[f, M, dbg] = geometric_controller(t, x, v, R, Om, p, K, casefun);

% --- Rotor allocation (Eq. (1)) -------------------------------------------
[f_alloc, M_alloc, f_rotors] = rotor_allocation(f, M, p);

% --- Equations of motion --------------------------------------------------
x_dot   = v;                                                       % (2)
v_dot   = p.g*p.e3 - (f_alloc/p.m) * (R * p.e3);                   % (3)
R_dot   = R * hat_map(Om);                                         % (4)
Om_dot  = p.J \ ( M_alloc - cross(Om, p.J*Om) );                   % (5)

dxdt = [x_dot; v_dot; R_dot(:); Om_dot];

% Expose debug via persistent for the caller (simulations capture via
% their own dbg recording; here we also keep a copy).
persistent lastdbg
lastdbg = dbg;

end

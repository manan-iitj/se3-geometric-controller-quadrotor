function [f, M, f_rotors] = rotor_allocation(f_cmd, M_cmd, p)
%ROTOR_ALLOCATION  Control allocation (Eq. (1) of the paper).
%
%   [f; M1; M2; M3] = A * [f1; f2; f3; f4]
%
%       A = [ 1   1   1   1
%             0  -d   0   d
%             d   0  -d   0
%            -c  c  -c   c ]
%
%   where c = c_tau_f.
%
%   Given the desired total thrust f_cmd and moment M_cmd, computes the
%   individual rotor thrusts f_rotors = A^{-1} * [f_cmd; M_cmd] using the
%   inverse allocation matrix (assumes directly controlled rotor thrusts,
%   no rotor/propeller dynamics, as in the paper).
%
%   Outputs:
%     f        - total thrust (scalar)
%     M        - total moment (3x1) in body frame
%     f_rotors - individual rotor thrusts f1..f4 (4x1)

d   = p.d;
c   = p.c_tau_f;

% Allocation matrix A (Eq. (1))
A = [ 1,   1,   1,   1;
      0,  -d,   0,   d;
      d,   0,  -d,   0;
     -c,   c,  -c,   c ];

cmd = [f_cmd; M_cmd];          % 4x1

% Determinant of A is 8*c*d^2 (paper), invertible for d ~= 0, c ~= 0.
A_inv = inv(A);

f_rotors = A_inv * cmd;        % [f1; f2; f3; f4]

% Recompute total thrust and moment from allocated rotor thrusts
alloc   = A * f_rotors;
f       = alloc(1);
M       = alloc(2:4);

end

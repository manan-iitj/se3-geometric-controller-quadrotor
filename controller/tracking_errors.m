function [tracking, att] = tracking_errors(x, v, R, Om, xd, xd_dot, Rd, Omega_d, p, K)
%TRACKING_ERRORS  Compute tracking errors (Eqs. (6)-(11)).
%
%   ex = x - xd
%   ev = v - xd_dot
%   Psi = 0.5 * trace(I - Rd' * R)                                 (8)
%   eR  = 0.5 * vee( Rd' * R - R' * Rd )                           (10)
%   eOm = Om - R' * Rd * Omega_d                                   (11)

tracking.ex  = x - xd;
tracking.ev  = v - xd_dot;

att.Psi = 0.5 * trace(eye(3) - Rd'*R);                      % Eq. (8)
att.eR  = 0.5 * vee_map( Rd'*R - R'*Rd );                   % Eq. (10)
att.eOm = Om - (R'*Rd) * Omega_d;                           % Eq. (11)

end

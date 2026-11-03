function [Omega_d, Omega_dot_d] = desired_angular_velocity(Rd, Rd_dot, Rd_ddot)
%DESIRED_ANGULAR_VELOCITY  Desired angular velocity from attitude matrix.
%
%   hat(Omega_d)     = Rd' * Rd_dot      => Omega_d = vee(Rd' * Rd_dot)
%   hat(Omega_dot_d) = Rd' * Rd_ddot - hat(Omega_d)^2
%                    = Rd' * Rd_ddot - (Rd'*Rd_dot)^2

Omega_d     = vee_map( Rd' * Rd_dot );
Omega_dot_d = vee_map( Rd' * Rd_ddot - (Rd' * Rd_dot)*(Rd' * Rd_dot) );

end

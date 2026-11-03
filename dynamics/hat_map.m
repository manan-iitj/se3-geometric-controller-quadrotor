function Omega_hat = hat_map(Omega)
%HAT_MAP  Hat map  hat : R^3 -> so(3).
%
%   For x, y in R^3, hat(x)*y = x x y (cross product).
%
%   Given Omega = [w1; w2; w3], returns the skew-symmetric matrix
%
%       [ 0   -w3   w2
%         w3   0   -w1
%        -w2   w1    0 ]

Omega_hat = [ 0,      -Omega(3),  Omega(2);
              Omega(3),  0,      -Omega(1);
             -Omega(2), Omega(1),  0 ];

end

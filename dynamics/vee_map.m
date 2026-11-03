function Omega = vee_map(A)
%VEE_MAP  Vee map  vee : so(3) -> R^3, inverse of the hat map.
%
%   For a skew-symmetric matrix A = hat(Omega), returns Omega such that
%   vee(hat(Omega)) = Omega. Uses the identity
%       -1/2 * trace( hat(x) hat(y) ) = x' y
%   so that Omega = [A(3,2); A(1,3); A(2,1)].

Omega = [ A(3,2);
          A(1,3);
          A(2,1) ];

end

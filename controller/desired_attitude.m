function [b3d, b2d, Rd] = desired_attitude(a, b1d)
%DESIRED_ATTITUDE  Construct desired attitude (Eq. (12) of the paper).
%
%   b3d = -a / ||a||
%   b2d = cross(b3d,b1d) / ||cross(b3d,b1d)||
%   Rd  = [ cross(b2d,b3d), b2d, b3d ]

b3d = -a / norm(a);
c   = cross(b3d, b1d);
b2d = c / norm(c);
Rd  = [ cross(b2d, b3d), b2d, b3d ];

end

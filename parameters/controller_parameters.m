function K = controller_parameters(p)
%CONTROLLER_PARAMETERS  Geometric tracking controller gains.
%
% Values taken exactly from Section IV of the paper.
%
%   kx = 16*m
%   kv = 5.6*m
%   kR = 8.81
%   kOmega = 2.54

K.kx     = 16 * p.m;
K.kv     = 5.6 * p.m;
K.kR     = 8.81;
K.kOmega = 2.54;

end

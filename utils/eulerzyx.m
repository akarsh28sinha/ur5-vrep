function [ rot_mat ] = eulerzyx( theta )
% EULERZYX - Long Qian
%   The rotational matrix can be represented by:
%	R = Rx(t1) * Ry(t2) * Rz(t3)
    rot_mat = ROTX(theta(1)) * ROTY(theta(2)) * ROTZ(theta(3));
end


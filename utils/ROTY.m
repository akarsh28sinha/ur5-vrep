function [ rot_mat ] = ROTY( theta )
%ROTX Summary of this function goes here
%   Detailed explanation goes here
    rot_mat = [cos(theta) 0 sin(theta); 0 1 0; -sin(theta) 0 cos(theta)];
end


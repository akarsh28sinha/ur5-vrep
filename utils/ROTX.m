function [ rot_mat ] = ROTX( theta )
%ROTX Summary of this function goes here
%   Detailed explanation goes here
    rot_mat = [1 0 0; 0 cos(theta) -sin(theta); 0 sin(theta) cos(theta)];
end


function [ rot_mat ] = ROTZ( theta )
%ROTX Summary of this function goes here
%   Detailed explanation goes here
    rot_mat = [cos(theta) -sin(theta) 0; sin(theta) cos(theta) 0; 0 0 1];
end


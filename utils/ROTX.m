function [ rot_mat ] = ROTX( theta )
%ROTX - Long Qian
%   Calculate 3*3 rotation matrix along x axis with theta radian
    rot_mat = [1 0 0; 0 cos(theta) -sin(theta); 0 sin(theta) cos(theta)];
end


function [ rot_mat ] = ROTY( theta )
%ROTY - Long Qian
%   Calculate 3*3 rotation matrix along y axis with theta radian
    rot_mat = [cos(theta) 0 sin(theta); 0 1 0; -sin(theta) 0 cos(theta)];
end


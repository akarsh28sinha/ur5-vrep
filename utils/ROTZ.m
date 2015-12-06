function [ rot_mat ] = ROTZ( theta )
%ROTZ - Long Qian
%   Calculate 3*3 rotation matrix along z axis with theta radian
    rot_mat = [cos(theta) -sin(theta) 0; sin(theta) cos(theta) 0; 0 0 1];
end


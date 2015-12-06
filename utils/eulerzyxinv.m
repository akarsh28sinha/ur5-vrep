function [ theta ] = eulerzyxinv( rot_mat )
% EULERZYXINV - Long Qian
%	The rotational matrix can be represented by:
%	R = Rx(t1) * Ry(t2) * Rz(t3)
%   theta - inverse eulerzyx angle. 1*3 vector
    theta = [0,0,0];
    % If the rotational matrix represents a singularity
    if abs(rot_mat(3,3)) < eps && abs(rot_mat(2,3)) < eps
        theta(1) = 0;
        theta(2) = atan2(rot_mat(1,3), rot_mat(3,3));
        theta(3) = atan2(rot_mat(2,1), rot_mat(2,2));
    % Normal case
    else
        theta(1) = atan2(-rot_mat(2,3), rot_mat(3,3));
        sinr = sin(theta(1));
        cosr = cos(theta(1));
        theta(2) = atan2(rot_mat(1,3), cosr * rot_mat(3,3) - sinr * rot_mat(2,3));
        theta(3) = atan2(-rot_mat(1,2), rot_mat(1,1));
    end
end


function movef( id, vrep, g, h, hrel )
%MOVEF Summary of this function goes here
%   Detailed explanation goes here
    % Get the rotational matrix from the transformation matrix
    rot_mat = g(1:3, 1:3);
    % Get the position offset from the transformation matrix
    pos = g(1:3, 4);
    % Compute the euler angle from the rotational matrix
    theta = eulerzyxinv(rot_mat);
    % Set the current object to the desired euler angle relative to hrel
    res = vrep.simxSetObjectOrientation(id, h, hrel, theta, vrep.simx_opmode_oneshot);
    vrchk(vrep, res, true);
    % Set the current object to the desired position relative to hrel
    res = vrep.simxSetObjectPosition(id, h, hrel, pos, vrep.simx_opmode_oneshot);
    vrchk(vrep, res, true);
end


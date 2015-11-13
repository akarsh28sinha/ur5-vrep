function [hnew] = copyf( id, vrep, g, h, hrel )
%COPYF Summary of this function goes here
%   Detailed explanation goes here
    % Get the rotational matrix from the transformation matrix
    rot_mat = g(1:3, 1:3);
    % Get the position offset from the transformation matrix
    pos = g(1:3, 4);
    % Compute the euler angle from the rotational matrix
    theta = eulerzyxinv(rot_mat);
    % Copy the object and use hnew to record it
    % simx_opmode_oneshot_wait is used to ensure the copy paste is done
    [returnCode, hnew] = vrep.simxCopyPasteObjects(id, h, vrep.simx_opmode_oneshot_wait);
    vrchk(vrep, returnCode, true);
    % Set the hnew object to the desired euler angle relative to hrel
    res = vrep.simxSetObjectOrientation(id, hnew, hrel, theta, vrep.simx_opmode_oneshot);
    vrchk(vrep, res, true);
    % Set the hnew object to the desired position relative to hrel
    res = vrep.simxSetObjectPosition(id, hnew, hrel, pos, vrep.simx_opmode_oneshot);
    vrchk(vrep, res, true);
end


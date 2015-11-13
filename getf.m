function [ g ] = getf( id, vrep, h, hrel )
%GETF Summary of this function goes here
%   Detailed explanation goes here
    g = eye(4); % initialize the G matrix with identity
    % Get the object orientation with respect to the hrel object
    % simx_opmode_oneshot_wait is used in order to get the exact data
    [returnCode, theta] = vrep.simxGetObjectOrientation(id, h, hrel, vrep.simx_opmode_oneshot_wait );
    vrchk(vrep, returnCode, true);
    % Compute the rotational matrix from the euler angle read from vrep
    g(1:3, 1:3) = eulerzyx(theta);
    % Get the object position with respect to the hrel object
    % simx_opmode_oneshot_wait is used in order to get the exact data
    [returnCode, g(1:3, 4)] = vrep.simxGetObjectPosition(id, h, hrel, vrep.simx_opmode_oneshot_wait );
    vrchk(vrep, returnCode, true);
end


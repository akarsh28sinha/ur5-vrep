function [ currentJoints ] = ur5_ptp_jnt( vrep, id, handles, res, startingJoints, threshold, theta )
%UR5_PTP - Long Qian
%   Move the UR5 robot so that the end-effector is at joint vector theta
    targetJointsActual = reshape(theta, [1,6]) + startingJoints;
    
    % Set target joints
    for j = 1:6,
        vrep.simxSetJointTargetPosition(id, handles.ur5Joints(j),...
            targetJointsActual(j), ...
            vrep.simx_opmode_oneshot);
        vrchk(vrep, res);
    end
    
    % Check if the robot arrives at the desired configuration
    currentJoints = zeros(1,6);
    reached = false;
    while ~reached
        % Get current joint angles for each joint
        for i = 1:6
            [ret, currentJoints(i)] = vrep.simxGetJointPosition(id, handles.ur5Joints(i),...
                vrep.simx_opmode_oneshot_wait);
            vrchk(vrep, res, true);
        end
        Gcurrent = ur5fwdtrans(currentJoints-startingJoints, 6);
        movef( id, vrep, Gcurrent, handles.FrameEnd, handles.base);
        % Check whether all joints fall into the threshold
        % If yes, then we think the target is reached
        diffJoints = currentJoints - targetJointsActual;
        for j = 1:6
            if diffJoints(j) > pi
                diffJoints(j) = diffJoints(j) - 2*pi;
            elseif diffJoints(j) < -pi
                diffJoints(j) = diffJoints(j) + 2*pi;
            end
        end
        if max(abs(diffJoints)) < threshold
            reached = true;
        end
    end
    currentJoints = currentJoints - startingJoints;
end


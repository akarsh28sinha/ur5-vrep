function [ currentJoints ] = ur5_ptp_cart( vrep, id, handles, res, startingJoints, threshold, G )
%UR5_PTP - Long Qian
%   Move the UR5 robot so that the end-effector is at transformation G
    currentJoints = zeros(1,6);
    for i = 1:6
        [ret, currentJoints(i)] = vrep.simxGetJointPosition(id, handles.ur5Joints(i),...
            vrep.simx_opmode_oneshot_wait);
        vrchk(vrep, res, true);
    end
    
    % Find the closest joint configuration
	theta = ur5inv(G);
    targetJointsActual = zeros(1,6);
    minDist = 1000;
    for i = 1:8
        targetJointsTemp = theta(:,i)'+startingJoints;
        for j = 1:6
            if targetJointsTemp(j) > pi
                targetJointsTemp(j) = targetJointsTemp(j) - 2*pi;
            elseif targetJointsTemp(j) < -pi
                targetJointsTemp(j) = targetJointsTemp(j) + 2*pi;
            end
        end
        dist = norm(targetJointsTemp - currentJoints);
        if dist < minDist
            minDist = dist;
            targetJointsActual = targetJointsTemp;
        end
    end
    
    currentJoints = ur5_ptp_jnt( vrep, id, handles, res, startingJoints, threshold, targetJointsActual - startingJoints );
end


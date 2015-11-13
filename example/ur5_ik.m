% The first block of code is from ur5_example.m from Lab 0:
vrep=remApi('remoteApi');
vrep.simxFinish(-1);
id = vrep.simxStart('127.0.0.1', 19997, true, true, 2000, 5);
if id < 0,
    disp('Failed connecting to remote API server. Exiting.');
    vrep.delete();
    return;
end
fprintf('Connection %d to remote API server open.\n', id);
% If your main code is run as a function, and not a script,
% you can use this command to ensure that cleanup_vrep is
% automatically run when there is a failure:
% cleanupObj = onCleanup(@() cleanup_vrep(vrep, id));

%% Construct the joints frame for UR5
handles = struct('id', id);
jointNames={'UR5_joint1','UR5_joint2','UR5_joint3','UR5_joint4',...
    'UR5_joint5','UR5_joint6'};
ur5Joints = -ones(1,6); 
for i = 1:6
    [res, ur5Joints(i)] = vrep.simxGetObjectHandle(id, ...
        jointNames{i}, vrep.simx_opmode_oneshot_wait); 
    vrchk(vrep, res);
end
handles.ur5Joints = ur5Joints;

[res, ur5Ref] = vrep.simxGetObjectHandle(id, 'UR5', ...
    vrep.simx_opmode_oneshot_wait); 
vrchk(vrep, res);

[res, ur5Gripper] = vrep.simxGetObjectHandle(id, 'UR5_connection', ...
    vrep.simx_opmode_oneshot_wait);
vrchk(vrep, res);

handles.ur5Ref = ur5Ref;
handles.ur5Gripper = ur5Gripper;

%% Construct the Base frame
[res, handles.base] = vrep.simxGetObjectHandle(id, ...
    'Frame0', vrep.simx_opmode_oneshot_wait);
vrchk(vrep, res);

%% Construct coordinates for each joint and endeffctor
% And compute the transformation from base to each frame at default
handles.FrameEnd = copyf( id, vrep, eye(4), handles.base, handles.base);
handles.FrameEndTarget = copyf( id, vrep, eye(4), handles.base, handles.base);
vrchk(vrep, res);

%% Other Initialization Stuff

% Stream wheel angles, Hokuyo data, and robot pose (see usage below)
% Wheel angles are not used in this example, but they may/will be necessary in
% your project.
for i = 1:6,
  res = vrep.simxGetJointPosition(id, handles.ur5Joints(i),...
      vrep.simx_opmode_streaming); 
  vrchk(vrep, res, true);
end
res = vrep.simxGetObjectPosition(id, handles.ur5Ref, -1,...
    vrep.simx_opmode_streaming); 
vrchk(vrep, res, true);
res = vrep.simxGetObjectOrientation(id, handles.ur5Ref, -1,...
    vrep.simx_opmode_streaming); 
vrchk(vrep, res, true);

% Stream the arm joint angles and the tip position/orientation
res = vrep.simxGetObjectPosition(id, handles.ur5Gripper, handles.ur5Ref, vrep.simx_opmode_streaming);
vrchk(vrep, res, true);
res = vrep.simxGetObjectOrientation(id, handles.ur5Gripper, handles.ur5Ref, vrep.simx_opmode_streaming);
vrchk(vrep, res, true);
for i = 1:6,
  res = vrep.simxGetJointPosition(id, handles.ur5Joints(i),...
      vrep.simx_opmode_streaming);
  vrchk(vrep, res, true);
end

vrep.simxGetPingTime(id); % make sure that all streaming data has reached the client at least once

res = vrep.simxStartSimulation(id, vrep.simx_opmode_oneshot_wait);

%% Simulation
threshold = 0.01;
startingJoints = [pi/2, pi/2, 0, pi/2, 0, 0];
targetJoints = [pi/2, pi/2, -pi/2, pi/2, pi/2, pi/2;...
                1, 1, 1, 1, 1, 1;...
                pi/2, pi/2, 0, pi/2, 0, 0];

res = vrep.simxPauseCommunication(id, true);
vrchk(vrep, res);
% Set the arm to its starting configuration:
for i = 1:6,
    res = vrep.simxSetJointTargetPosition(id, handles.ur5Joints(i),...
        startingJoints(i),...
        vrep.simx_opmode_oneshot);
    vrchk(vrep, res, true);
end
res = vrep.simxPauseCommunication(id, false);
vrchk(vrep, res);

pause(1);

for k = 1:size(targetJoints,1)
    % Calculate the target frame for each of the joint
    % Visualize the target frames
    finalJoints = targetJoints(k,:);
    fprintf('Final joint: %d.\n', k);
    G = ur5fwdtrans(finalJoints-startingJoints, 6);
    movef( id, vrep, G, handles.FrameEndTarget, handles.base);
    theta = ur5inv(G);
    pause(1);
    for ti = 1:8
        fprintf('Approaching the %d theta solution.\n', ti);
        targetJointsActual = theta(:,ti)'+startingJoints;
        for tj = 1:6
            if targetJointsActual(tj) > pi
                targetJointsActual(tj) = targetJointsActual(tj) - 2*pi;
            elseif targetJointsActual(tj) < -pi
                targetJointsActual(tj) = targetJointsActual(tj) + 2*pi;
            end
        end
        targetJointsActual
        for j = 1:6,
            vrep.simxSetJointTargetPosition(id, handles.ur5Joints(j),...
                targetJointsActual(j), ...
                vrep.simx_opmode_oneshot);
            vrchk(vrep, res);
        end

        currentJoints = zeros(1,6);    
        reached = false;
        while ~reached
            % Get current joint angles for each joint
            for i = 1:6
                [returnCode,currentJoints(i)] = vrep.simxGetJointPosition(id, handles.ur5Joints(i),...
                    vrep.simx_opmode_oneshot_wait);
                vrchk(vrep, res, true);
            end
            Gcurrent = ur5fwdtrans(currentJoints-startingJoints, 6);
            movef( id, vrep, Gcurrent, handles.FrameEnd, handles.base);
            % Check whether all joints fall into the threshold
            % If yes, then we think the target is reached
            diffJoints = currentJoints - targetJointsActual;
            for tj = 1:6
                if diffJoints(tj) > pi
                    diffJoints(tj) = diffJoints(tj) - 2*pi;
                elseif diffJoints(tj) < -pi
                    diffJoints(tj) = diffJoints(tj) + 2*pi;
                end
            end
            if max(abs(diffJoints)) < threshold
                reached = true;
            end
        end
        fprintf('%d theta solution reachead.\n', ti);
        pause(1);

        for i = 1:6,
            res = vrep.simxSetJointTargetPosition(id, handles.ur5Joints(i),...
                startingJoints(i),...
                vrep.simx_opmode_oneshot);
            vrchk(vrep, res, true);
        end
        currentJoints = zeros(1,6);
        reached = false;
        while ~reached
            % Get current joint angles for each joint
            for i = 1:6
                [returnCode,currentJoints(i)] = vrep.simxGetJointPosition(id, handles.ur5Joints(i),...
                    vrep.simx_opmode_oneshot_wait);
                vrchk(vrep, res, true);
            end
            Gcurrent = ur5fwdtrans(currentJoints-startingJoints, 6);
            movef( id, vrep, Gcurrent, handles.FrameEnd, handles.base);
            % Check whether all joints fall into the threshold
            % If yes, then we think the target is reached
            diffJoints = currentJoints - startingJoints;
            for tj = 1:6
                if diffJoints(tj) > pi
                    diffJoints(tj) = diffJoints(tj) - 2*pi;
                elseif diffJoints(tj) < -pi
                    diffJoints(tj) = diffJoints(tj) + 2*pi;
                end
            end
            if max(abs(diffJoints)) < threshold
                reached = true;
            end
        end
        pause(1);
    end
    pause(2);
end

% Delete useless frames
vrep.simxRemoveObject(id, handles.FrameEnd, vrep.simx_opmode_oneshot);
vrep.simxRemoveObject(id, handles.FrameEndTarget, vrep.simx_opmode_oneshot);




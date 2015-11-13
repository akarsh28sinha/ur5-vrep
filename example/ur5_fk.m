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
Frames = -ones(1,6); 
FramesTarget = -ones(1,6); 
Frames(1) = copyf( id, vrep, eye(4), handles.base, handles.base);
Frames(2) = copyf( id, vrep, eye(4), handles.base, handles.base);
Frames(3) = copyf( id, vrep, eye(4), handles.base, handles.base);
Frames(4) = copyf( id, vrep, eye(4), handles.base, handles.base);
Frames(5) = copyf( id, vrep, eye(4), handles.base, handles.base);
Frames(6) = copyf( id, vrep, eye(4), handles.base, handles.base);
FramesTarget(1) = copyf( id, vrep, eye(4), handles.base, handles.base);
FramesTarget(2) = copyf( id, vrep, eye(4), handles.base, handles.base);
FramesTarget(3) = copyf( id, vrep, eye(4), handles.base, handles.base);
FramesTarget(4) = copyf( id, vrep, eye(4), handles.base, handles.base);
FramesTarget(5) = copyf( id, vrep, eye(4), handles.base, handles.base);
FramesTarget(6) = copyf( id, vrep, eye(4), handles.base, handles.base);
handles.FrameEnd = copyf( id, vrep, eye(4), handles.base, handles.base);
handles.FrameEndTarget = copyf( id, vrep, eye(4), handles.base, handles.base);
handles.FramesTarget = FramesTarget;
handles.Frames = Frames;
vrchk(vrep, res);

gRest1 = getf(id, vrep, handles.ur5Joints(1), handles.base);
gRest2 = getf(id, vrep, handles.ur5Joints(2), handles.base);
gRest2(1:3, 1:3) = eye(3);
gRest3 = getf(id, vrep, handles.ur5Joints(3), handles.base);
gRest3(1:3, 1:3) = eye(3);
gRest4 = getf(id, vrep, handles.ur5Joints(4), handles.base);
gRest4(1:3, 1:3) = eye(3);
gRest5 = getf(id, vrep, handles.ur5Joints(5), handles.base);
gRest5(1:3, 1:3) = eye(3);
gRest6 = getf(id, vrep, handles.ur5Joints(6), handles.base);
gRest6(1:3, 1:3) = eye(3);
gRestEnd = getf(id, vrep, handles.ur5Gripper, handles.base);
gRestEnd(1:3, 1:3) = eye(3);

%% Other Initialization Stuff

% Stream wheel angles, Hokuyo data, and robot pose (see usage below)
% Wheel angles are not used in this example, but they may/will be necessary in
% your project.
for i = 1:6,
  res = vrep.simxGetJointPosition(id, ur5Joints(i),...
      vrep.simx_opmode_streaming); 
  vrchk(vrep, res, true);
end
res = vrep.simxGetObjectPosition(id, ur5Ref, -1,...
    vrep.simx_opmode_streaming); 
vrchk(vrep, res, true);
res = vrep.simxGetObjectOrientation(id, ur5Ref, -1,...
    vrep.simx_opmode_streaming); 
vrchk(vrep, res, true);

% Stream the arm joint angles and the tip position/orientation
res = vrep.simxGetObjectPosition(id, ur5Gripper, ur5Ref, vrep.simx_opmode_streaming);
vrchk(vrep, res, true);
res = vrep.simxGetObjectOrientation(id, ur5Gripper, ur5Ref, vrep.simx_opmode_streaming);
vrchk(vrep, res, true);
for i = 1:6,
  res = vrep.simxGetJointPosition(id, ur5Joints(i),...
      vrep.simx_opmode_streaming);
  vrchk(vrep, res, true);
end

vrep.simxGetPingTime(id); % make sure that all streaming data has reached the client at least once

res = vrep.simxStartSimulation(id, vrep.simx_opmode_oneshot_wait);

%% Simulation
threshold = 0.01;
targetPoses = [90*pi/180,90*pi/180,-90*pi/180,90*pi/180,90*pi/180,90*pi/180;...
                -90*pi/180,45*pi/180,90*pi/180,135*pi/180,90*pi/180,90*pi/180;...
                zeros(1,6)];

l2 = 0.0892;
l3 = 0.5143;
l4 = 0.9064;
l5 = 0.1101;
l6 = 1.0012;

for k = 1:size(targetPoses,1)
    % Calculate the target frame for each of the joint
    % Visualize the target frames
    finalJoints = targetPoses(k,:);
    twist1 = transformation([0, 0, finalJoints(1)],...
        [0, 0, 0]);
    twist2 = transformation([-finalJoints(2), 0, 0],...
        [0, sin(-finalJoints(2)), 1-cos(-finalJoints(2))] * l2);
    twist3 = transformation([-finalJoints(3), 0, 0],...
        [0, sin(-finalJoints(3)), 1-cos(-finalJoints(3))] * l3);
    twist4 = transformation([-finalJoints(4), 0, 0],...
        [0, sin(-finalJoints(4)), 1-cos(-finalJoints(4))] * l4);
    twist5 = transformation([0, 0, finalJoints(5)],...
        [cos(finalJoints(5))-1, sin(finalJoints(5)), 0] * l5);
    twist6 = transformation([-finalJoints(6), 0, 0],...
        [0, -sin(finalJoints(6)), 1-cos(-finalJoints(6))] * l6);
    % Compute transformation using twists
    gTarget1 = twist1 * gRest1;
    gTarget2 = twist1 * twist2 * gRest2;
    gTarget3 = twist1 * twist2 * twist3 * gRest3;
    gTarget4 = twist1 * twist2 * twist3 * twist4 * gRest4;
    gTarget5 = twist1 * twist2 * twist3 * twist4 * twist5 * gRest5;
    gTarget6 = twist1 * twist2 * twist3 * twist4 * twist5 * twist6 * gRest6;
    gTargetEnd = twist1 * twist2 * twist3 * twist4 * twist5 * twist6 * gRestEnd;
    % Move the frames
    movef( id, vrep, gTarget1, handles.FramesTarget(1), handles.base);
    movef( id, vrep, gTarget2, handles.FramesTarget(2), handles.base);
    movef( id, vrep, gTarget3, handles.FramesTarget(3), handles.base);
    movef( id, vrep, gTarget4, handles.FramesTarget(4), handles.base);
    movef( id, vrep, gTarget5, handles.FramesTarget(5), handles.base);
    movef( id, vrep, gTarget6, handles.FramesTarget(6), handles.base);
    movef( id, vrep, gTargetEnd, handles.FrameEndTarget, handles.base);
    
    % Get current joint configuration
    currentJoints = zeros(1,6);    
    reached = false;
    while ~reached
        % Get current joint angles for each joint
        for i = 1:6
            [returnCode,currentJoints(i)] = vrep.simxGetJointPosition(id, handles.ur5Joints(i),...
                vrep.simx_opmode_oneshot_wait);
            vrchk(vrep, res, true);
        end
        % Calculating current twists associated with joint value
        twist1 = transformation([0, 0, currentJoints(1)],...
            [0, 0, 0]);
        twist2 = transformation([-currentJoints(2), 0, 0],...
            [0, sin(-currentJoints(2)), 1-cos(-currentJoints(2))] * l2);
        twist3 = transformation([-currentJoints(3), 0, 0],...
            [0, sin(-currentJoints(3)), 1-cos(-currentJoints(3))] * l3);
        twist4 = transformation([-currentJoints(4), 0, 0],...
            [0, sin(-currentJoints(4)), 1-cos(-currentJoints(4))] * l4);
        twist5 = transformation([0, 0, currentJoints(5)],...
            [cos(currentJoints(5))-1, sin(currentJoints(5)), 0] * l5);
        twist6 = transformation([-currentJoints(6), 0, 0],...
            [0, -sin(currentJoints(6)), 1-cos(-currentJoints(6))] * l6);
        % Compute current transformations
        g1 = twist1 * gRest1;
        g2 = twist1 * twist2 * gRest2;
        g3 = twist1 * twist2 * twist3 * gRest3;
        g4 = twist1 * twist2 * twist3 * twist4 * gRest4;
        g5 = twist1 * twist2 * twist3 * twist4 * twist5 * gRest5;
        g6 = twist1 * twist2 * twist3 * twist4 * twist5 * twist6 * gRest6;
        gEnd = twist1 * twist2 * twist3 * twist4 * twist5 * twist6 * gRestEnd;
        % Move the current frames
        movef( id, vrep, g1, handles.Frames(1), handles.base);
        movef( id, vrep, g2, handles.Frames(2), handles.base);
        movef( id, vrep, g3, handles.Frames(3), handles.base);
        movef( id, vrep, g4, handles.Frames(4), handles.base);
        movef( id, vrep, g5, handles.Frames(5), handles.base);
        movef( id, vrep, g6, handles.Frames(6), handles.base);
        movef( id, vrep, gEnd, handles.FrameEnd, handles.base);
        
        % Check whether all joints fall into the threshold
        % If yes, then we think the target is reached
        if max(abs(currentJoints - finalJoints)) < threshold
            reached = true;
        end
    end
end

% Delete useless frames
for i=1:6
    vrep.simxRemoveObject(id, handles.Frames(i), vrep.simx_opmode_oneshot);
    vrep.simxRemoveObject(id, handles.FramesTarget(i), vrep.simx_opmode_oneshot);
end
vrep.simxRemoveObject(id, handles.FrameEnd, vrep.simx_opmode_oneshot);
vrep.simxRemoveObject(id, handles.FrameEndTarget, vrep.simx_opmode_oneshot);




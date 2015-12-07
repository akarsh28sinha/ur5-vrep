% ur5_draw.m
% Author: Long Qian
% Adapted from Prof. Noah Cowan @Johns Hopkins University
%% Add path and initialization stuff
addpath('..');
addpath('../utils', '../vrep', '../ur5');

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

startingJoints = [pi/2, pi/2, 0, pi/2, 0, 0];
threshold = 0.005;


%% Initializing the drawing plane
pLeftUp = [0.5, 0.1, 0.3];
pLeftDown = [0.3, 0.1, 0.3];
pRightDown = [0.3, -0.1, 0.3]; % pRightDown can be ignored
pRightUp = [0.5, -0.1, 0.3];
xdir = pRightUp - pLeftUp;
ydir = pLeftDown - pLeftUp;
zdir = cross(xdir, ydir);
zdir = zdir / norm(zdir);
pRot = [xdir/norm(xdir); ydir/norm(ydir); zdir/norm(zdir)]';

%% Initializing lines
% each column is contains start_x, start_y, end_x, end_y
% 2D plane coordinates is used here
% (0, 0) is the leftup corner
% (1, 0) is the rightup corner
% (0, 1) is the leftdown corner
numLines = 10;
lines = zeros(4,numLines);
for k = 1:numLines
    lines(1,k) = 0;
    lines(2,k) = k/10;
    lines(3,k) = 1;
    lines(4,k) = k/10;
end

numMids = 100;
numMidsSafe = 10;
for k = 1:numLines
    pStart = pLeftUp + xdir * lines(1,k) + ydir * lines(2,k);
    pEnd = pLeftUp + xdir * lines(3,k) + ydir * lines(4,k);
    pPreStart = pStart - zdir * 0.03;
    pPostEnd = pEnd - zdir * 0.03;
    GStart = G(pRot, pStart);
    GEnd = G(pRot, pEnd);
    GPreStart = G(pRot, pPreStart);
    GPostEnd = G(pRot, pPostEnd);
    
    % Move the robot to the GPreStart configuration
    movef( id, vrep, GStart, handles.FrameEndTarget, handles.base);
    jointsVec = ur5_ptp_cart( vrep, id, handles, res, startingJoints, threshold, GPreStart );
    % Move the robot linearly down the touch point
    fprintf('Moving Down to the plane.\n');
    jointsVec = ur5_lin_jb( vrep, id, handles, res, startingJoints, threshold, pPreStart, pStart, pRot, jointsVec );
    % Move the robot linearly to the end point of the line
    movef( id, vrep, GEnd, handles.FrameEndTarget, handles.base);
    fprintf('Moving to the end point.\n');
    jointsVec = ur5_lin_jb( vrep, id, handles, res, startingJoints, threshold, pStart, pEnd, pRot, jointsVec );
    % Move the robot up to leave the drawing plane
    fprintf('Moving up the leave the plane.\n');
    jointsVec = ur5_lin_jb( vrep, id, handles, res, startingJoints, threshold, pEnd, pPostEnd, pRot, jointsVec );
    
    pause(1);
end

% for k = 1:numLines
%     pStart = pLeftUp + xdir * lines(1,k) + ydir * lines(2,k);
%     pEnd = pLeftUp + xdir * lines(3,k) + ydir * lines(4,k);
%     pPreStart = pStart - zdir * 0.03;
%     pPostEnd = pEnd - zdir * 0.03;
%     GStart = G(pRot, pStart);
%     GEnd = G(pRot, pEnd);
%     GPreStart = G(pRot, pPreStart);
%     GPostEnd = G(pRot, pPostEnd);
%     
%     % Move the robot to the GPreStart configuration
%     movef( id, vrep, GStart, handles.FrameEndTarget, handles.base);
%     jointsVec = ur5_ptp_cart( vrep, id, handles, res, startingJoints, threshold, GPreStart );
%     % Move the robot linearly down the touch point
%     ur5_lin_ik( vrep, id, handles, res, startingJoints, threshold, pPreStart, pStart, pRot, numMidsSafe );
%     % Move the robot linearly to the end point of the line
%     movef( id, vrep, GEnd, handles.FrameEndTarget, handles.base);
%     ur5_lin_ik( vrep, id, handles, res, startingJoints, threshold, pStart, pEnd, pRot, numMids );
%     % Move the robot up to leave the drawing plane
%     ur5_lin_ik( vrep, id, handles, res, startingJoints, threshold, pEnd, pPostEnd, pRot, numMidsSafe );
%     
%     pause(1);
% end


%% Delete useless frames
vrep.simxRemoveObject(id, handles.FrameEnd, vrep.simx_opmode_oneshot);
vrep.simxRemoveObject(id, handles.FrameEndTarget, vrep.simx_opmode_oneshot);






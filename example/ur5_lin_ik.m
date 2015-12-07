function [ currentJoints ] = ur5_lin_ik( vrep, id, handles, res, startingJoints, threshold, p1, p2, pRot, numMids )
%UR5_LIN_IK - Long Qian
%   Move the UR5 robot linearly with inverse-kinematics method
%   vrep, id, handles, res - VREP related variables
%   startingJoints - the difference between joint values read from
%   remoteAPI and my calculating system
%   threshold - the maximum difference to say a point is close enough to
%   target
%   p1 - position of the start
%   p2 - position of the end
%   pRot - the rotation matrix of start point and end point
%   numMids - the number of points to interpolate the line
    for kk = 1:numMids
        pMid = p1 * (1 - kk / numMids) + p2 * kk / numMids;
        GMid = G(pRot, pMid);
        currentJoints = ur5_ptp_cart( vrep, id, handles, res, startingJoints, threshold, GMid );
    end

end


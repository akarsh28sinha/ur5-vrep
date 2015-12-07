function [ jointVecCurrent ] = ur5_lin_jb( vrep, id, handles, res, startingJoints, threshold, p1, p2, pRot, jointsVec )
%UR5_LIN_JB - Long Qian
%   Move the UR5 robot linearly with jacobian method
%   vrep, id, handles, res - VREP related variables
%   startingJoints - the difference between joint values read from
%   remoteAPI and my calculating system
%   threshold - the maximum difference to say a point is close enough to
%   target
%   p1 - position of the start
%   p2 - position of the end
%   pRot - the rotation matrix of start point and end point
%   jointsVec - the current joint-space vector

    stopflag = false;
    % catesian space vector
    euler = eulerzyxinv(pRot);
    cartVecCurrent = [p1, euler]';
    jointVecCurrent = reshape(jointsVec, [6,1]);
    
    cartVecTarget = [p2, euler]';
    cartVecDif = cartVecTarget - cartVecCurrent;
    
    while ~stopflag
        jb_inv = ur5_jacobian_inv(jointVecCurrent);
        jointVecTarget = jointVecCurrent + jb_inv * cartVecDif / norm(cartVecDif) * 0.002;
        jointVecCurrent = ur5_ptp_jnt( vrep, id, handles, res, startingJoints, threshold, jointVecTarget );
        jointVecCurrent = reshape(jointVecCurrent, [6,1]);
        cartVecCurrent = cartvec(ur5fwdtrans(jointVecCurrent, 6));
        cartVecDif = cartVecTarget - cartVecCurrent;
        stopflag = true;
        
        for i = 4:6 
            while cartVecDif(i) > pi
                cartVecDif(i) = cartVecDif(i) - 2*pi;
            end
            while cartVecDif(i) < -pi
                cartVecDif(i) = cartVecDif(i) + 2*pi;
            end
        end
        for i = 1:6
            if abs(cartVecDif(i)) > threshold
                stopflag = false;
            end
        end
    end
    
end


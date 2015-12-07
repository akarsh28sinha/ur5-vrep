function [ jb_inv ] = ur5_jacobian_inv( jointsVec )
%UR5_JACOBIAN_INV - Long Qian
%   Calculate the inverse jacobian of UR5 at a certain joint configuration
    stepsize = 0.0001;
    G = ur5fwdtrans(jointsVec, 6);
    cartVec = cartvec(G);
    jb = zeros(6, 6);
    for i = 1:6
        jointsVecTemp = jointsVec;
        jointsVecTemp(i) = jointsVecTemp(i) + stepsize;
        cartVecTemp = cartvec(ur5fwdtrans(jointsVecTemp, 6));
        cartVecDif = cartVecTemp - cartVec;
        if i > 3
            while cartVecDif(i) > pi
                cartVecDif(i) = cartVecDif(i) - 2*pi;
            end
            while cartVecDif(i) < -pi
                cartVecDif(i) = cartVecDif(i) + 2*pi;
            end
        end
        jb(:,i) = (cartVecTemp - cartVec) / stepsize;
    end
    jb_inv = inv(jb);
end


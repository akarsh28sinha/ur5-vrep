function [ cartVec ] = cartvec( G )
%CARTVEC - Long Qian
%   Get cartesian vector from transformation matrix
    cartVec = zeros(6,1);
    cartVec(1) = G(1, 4);
    cartVec(2) = G(2, 4);
    cartVec(3) = G(3, 4);
    cartVec(4:6) = eulerzyxinv(G(1:3, 1:3));
end


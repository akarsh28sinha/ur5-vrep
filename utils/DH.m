function [ G ] = DH( a, alpha, d, theta )
%DH Summary of this function goes here
%   Detailed explanation goes here
    Td = eye(4);
    Td(3,4) = d;
    Ta = eye(4);
    Ta(1,4) = a;
    Rtheta = eye(4);
    Rtheta(1:3, 1:3) = ROTZ(theta);
    Ralpha = eye(4);
    Ralpha(1:3, 1:3) = ROTX(alpha);
    G = Td * Rtheta * Ta * Ralpha;
end


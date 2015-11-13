function [ G ] = twist( omega, p, theta )
%TWIST Summary of this function goes here
%   Detailed explanation goes here
    G = eye(4);
    omega_hat = zeros(3,3);
    omega_hat(1,2) = -omega(3);
    omega_hat(2,1) = omega(3);
    omega_hat(1,3) = omega(2);
    omega_hat(3,1) = -omega(2);
    omega_hat(2,3) = -omega(1);
    omega_hat(3,2) = omega(1);
    v = -omega_hat * p;
    G(1:3, 1:3) = eye(3) + omega_hat * sin(theta) + (1-cos(theta)) * omega_hat * omega_hat;
    G(1:3, 4) = (eye(3)*theta + (1-cos(theta))*omega_hat + (theta-sin(theta))*omega_hat*omega_hat) * v;
end


function [ G ] = twist( omega, p, theta )
%TWIST - Long Qian
%   Calculate the twist matrix from omega, p and theta
%   omega - the twist axis, 1*3 or 3*1 matrix
%   p - any point on the twist axis, 3*1 vector
%   theta - twist angle, in radian
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


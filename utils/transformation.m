function [ g ] = transformation( theta, pos )
%TRANSFORMATION - Long Qian
%   Composite a transformation matrix from euler angle theta and
%   translation vector
%   theta - eulerzyx angle, 1*3 or 3*1 vector
%   pos - translation vector, 1*3 or 3*1 vector
    g = eye(4); % initialize the G matrix with identity
    g(1:3, 1:3) = eulerzyx(theta);
    g(1, 4) = pos(1);
    g(2, 4) = pos(2);
    g(3, 4) = pos(3);
end


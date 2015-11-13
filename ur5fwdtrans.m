function [ G ] = ur5fwdtrans( targetJoints, i )
%UR5FWDTRANS Summary of this function goes here
%   Detailed explanation goes here
    d1 = 0.089159;
    d2 = 0;
    d3 = 0;
    d4 = 0.10915;
    d5 = 0.09465;
    d6 = 0.0823;
    a1 = 0;
    a2 = -0.425;
    a3 = -0.39225;
    a4 = 0;
    a5 = 0;
    a6 = 0;
    alpha1 = pi/2;
    alpha2 = 0;
    alpha3 = 0;
    alpha4 = pi/2;
    alpha5 = -pi/2;
    alpha6 = 0;
    T01 = DH(a1, alpha1, d1, targetJoints(1));
    T12 = DH(a2, alpha2, d2, targetJoints(2));
    T23 = DH(a3, alpha3, d3, targetJoints(3));
    T34 = DH(a4, alpha4, d4, targetJoints(4));
    T45 = DH(a5, alpha5, d5, targetJoints(5));
    T56 = DH(a6, alpha6, d6, targetJoints(6));
    if i==1
        G = T01;
    elseif i==2
        G = T01 * T12;
    elseif i==3
        G = T01 * T12 * T23;
    elseif i==4
        G = T01 * T12 * T23 * T34;
    elseif i==5
        G = T01 * T12 * T23 * T34 * T45;
    elseif i==6
        G = T01 * T12 * T23 * T34 * T45 * T56;
    else
        G = eye(4);
    end
end


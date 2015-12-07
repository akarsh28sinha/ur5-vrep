function [ ret ] = G( v1, v2 )
%G - Long Qian
%   Composite transformation
    ret = zeros(4);
    ret(1:3, 1:3) = v1;
    ret(1, 4) = v2(1);
    ret(2, 4) = v2(2);
    ret(3, 4) = v2(3);
    ret(4, 4) = 1;
end


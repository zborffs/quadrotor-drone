function ret = Rx(theta)
%Rx returns rotation matrix about 'x'-axis
%   Detailed explanation goes here
ret = [1 0 0; 0 cos(theta) sin(theta); 0 -sin(theta) cos(theta)];
end
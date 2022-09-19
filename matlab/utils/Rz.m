function ret = Rz(theta)
%Rz returns rotation matrix about 'x'-axis
ret = [cos(theta) sin(theta) 0; -sin(theta) cos(theta) 0; 0 0 1];
end
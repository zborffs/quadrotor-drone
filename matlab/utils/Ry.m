function ret = Ry(theta)
%Ry returns rotation matrix about 'y'-axis
ret = [cos(theta) 0 -sin(theta); 0 1 0; sin(theta) 0 cos(theta)];
end
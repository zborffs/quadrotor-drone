function u_control = compute_control(x, parameters, controller_gains)
%compute_control(x, parameters) asdf 
%   asdfasdfasdf asdf 

xhat = x(13:24,:);
z = x(25:28,:);

u0 = sqrt(parameters.g * (parameters.M + 4 * parameters.m) / (4 * parameters.k1));
u_control = -(controller_gains.K1 * xhat + controller_gains.K2 * z) + [u0; -u0; u0; -u0];

end
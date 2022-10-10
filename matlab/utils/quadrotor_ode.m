function out = quadrotor_ode(t, state, parameters, controller_gains)
% quadrotor_ode(t, x, p) is the 

% state variables
u = state(4);
v = state(5);
w = state(6);
phi = state(7);
theta = state(8);
psi = state(9);
p = state(10);
q = state(11);
r = state(12);

x = state(1:12);
xhat = state(13:24);
z = state(25:28);

% model parameters
M = parameters.M;
m = parameters.m;
R = parameters.R;
L = parameters.L;
l = parameters.l;
g = parameters.g;
k1 = parameters.k1;
k2 = parameters.k2;

A = [
    0         0         0    1.0000         0         0         0         0         0         0         0         0;
    0         0         0         0    1.0000         0         0         0         0         0         0         0;
    0         0         0         0         0    1.0000         0         0         0         0         0         0;
    0         0         0         0         0         0         0   -9.8100         0         0         0         0;
    0         0         0         0         0         0    9.8100         0         0         0         0         0;
    0         0         0         0         0         0         0         0         0         0         0         0;
    0         0         0         0         0         0         0         0         0    1.0000         0         0;
    0         0         0         0         0         0         0         0         0         0    1.0000         0;
    0         0         0         0         0         0         0         0         0         0         0    1.0000;
    0         0         0         0         0         0         0         0         0         0         0         0;
    0         0         0         0         0         0         0         0         0         0         0         0;
    0         0         0         0         0         0         0         0         0         0         0         0
];
B = [
                   0                   0                   0                   0;
                   0                   0                   0                   0;
                   0                   0                   0                   0;
                   0                   0                   0                   0;
                   0                   0                   0                   0;
  -1.400714103591450   1.400714103591450  -1.400714103591450   1.400714103591450;
                   0                   0                   0                   0;
                   0                   0                   0                   0;
                   0                   0                   0                   0;
                   0  -2.918154382482188                   0   2.918154382482188;
   2.918154382482188                   0  -2.918154382482188                   0;
   0.227272727272727   0.227272727272727   0.227272727272727   0.227272727272727
];

C = [
     1     0     0     0     0     0     0     0     0     0     0     0;
     0     1     0     0     0     0     0     0     0     0     0     0;
     0     0     1     0     0     0     0     0     0     0     0     0;
     0     0     0     0     0     0     1     0     0     0     0     0;
     0     0     0     0     0     0     0     1     0     0     0     0;
     0     0     0     0     0     0     0     0     1     0     0     0;
     0     0     0     0     0     0     0     0     0     1     0     0;
     0     0     0     0     0     0     0     0     0     0     1     0;
     0     0     0     0     0     0     0     0     0     0     0     1
];

C2 = [
     1     0     0     0     0     0     0     0     0     0     0     0;
     0     1     0     0     0     0     0     0     0     0     0     0;
     0     0     1     0     0     0     0     0     0     0     0     0;
     0     0     0     0     0     0     0     0     1     0     0     0    
];


% control
u_control = compute_control(state, parameters, controller_gains);
u1 = u_control(1);
u2 = u_control(2);
u3 = u_control(3);
u4 = u_control(4);

% stable system responses are invariant to use of extended over vanilla
% kalman filter, but using nonlinear kalman filter does nothing to
% stabilize the response where the vanilla kalman filter is unsuccessful
Anonlinear = [0, 0, 0, cos(psi)*cos(theta), cos(psi)*sin(phi)*sin(theta) - cos(phi)*sin(psi), sin(phi)*sin(psi) + cos(phi)*cos(psi)*sin(theta), v*(sin(phi)*sin(psi) + cos(phi)*cos(psi)*sin(theta)) + w*(cos(phi)*sin(psi) - cos(psi)*sin(phi)*sin(theta)), cos(psi)*(w*cos(phi)*cos(theta) - u*sin(theta) + v*cos(theta)*sin(phi)), w*(cos(psi)*sin(phi) - cos(phi)*sin(psi)*sin(theta)) - v*(cos(phi)*cos(psi) + sin(phi)*sin(psi)*sin(theta)) - u*cos(theta)*sin(psi), 0, 0, 0; 0, 0, 0, cos(theta)*sin(psi), cos(phi)*cos(psi) + sin(phi)*sin(psi)*sin(theta), cos(phi)*sin(psi)*sin(theta) - cos(psi)*sin(phi), - v*(cos(psi)*sin(phi) - cos(phi)*sin(psi)*sin(theta)) - w*(cos(phi)*cos(psi) + sin(phi)*sin(psi)*sin(theta)), sin(psi)*(w*cos(phi)*cos(theta) - u*sin(theta) + v*cos(theta)*sin(phi)), w*(sin(phi)*sin(psi) + cos(phi)*cos(psi)*sin(theta)) - v*(cos(phi)*sin(psi) - cos(psi)*sin(phi)*sin(theta)) + u*cos(psi)*cos(theta), 0, 0, 0; 0, 0, 0, -sin(theta), cos(theta)*sin(phi), cos(phi)*cos(theta), cos(theta)*(v*cos(phi) - w*sin(phi)), - u*cos(theta) - w*cos(phi)*sin(theta) - v*sin(phi)*sin(theta), 0, 0, 0, 0; 0, 0, 0, 0, r, -q, 0, -(981*cos(theta))/100, 0, 0, -w, v; 0, 0, 0, -r, 0, p, (981*cos(phi)*cos(theta))/100, -(981*sin(phi)*sin(theta))/100, 0, w, 0, -u; 0, 0, 0, q, -p, 0, -(981*cos(theta)*sin(phi))/100, -(981*cos(phi)*sin(theta))/100, 0, -v, u, 0; 0, 0, 0, 0, 0, 0, (sin(theta)*(q*cos(phi) - r*sin(phi)))/cos(theta), (r*cos(phi) + q*sin(phi))/cos(theta)^2, 0, 1, sin(phi)*tan(theta), cos(phi)*tan(theta); 0, 0, 0, 0, 0, 0, - r*cos(phi) - q*sin(phi), 0, 0, 0, cos(phi), -sin(phi); 0, 0, 0, 0, 0, 0, (q*cos(phi) - r*sin(phi))/cos(theta), (sin(theta)*(r*cos(phi) + q*sin(phi)))/cos(theta)^2, 0, 0, sin(phi)/cos(theta), cos(phi)/cos(theta); 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -(5*r)/6, -(5*q)/6; 0, 0, 0, 0, 0, 0, 0, 0, 0, (5*r)/6, 0, (5*p)/6; 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
Bnonlinear = [0, 0, 0, 0; 0, 0, 0, 0; 0, 0, 0, 0; 0, 0, 0, 0; 0, 0, 0, 0; -(2*u1)/5, -(2*u2)/5, -(2*u3)/5, -(2*u4)/5; 0, 0, 0, 0; 0, 0, 0, 0; 0, 0, 0, 0; 0, (5*u2)/6, 0, -(5*u4)/6; (5*u1)/6, 0, -(5*u3)/6, 0; 5/22, 5/22, 5/22, 5/22];

% system 
out = [
    w*(sin(phi)*sin(psi) + cos(phi)*cos(psi)*sin(theta)) - v*(cos(phi)*sin(psi) - cos(psi)*sin(phi)*sin(theta)) + u*cos(psi)*cos(theta);
    v*(cos(phi)*cos(psi) + sin(phi)*sin(psi)*sin(theta)) - w*(cos(psi)*sin(phi) - cos(phi)*sin(psi)*sin(theta)) + u*cos(theta)*sin(psi);
    w*cos(phi)*cos(theta) - u*sin(theta) + v*cos(theta)*sin(phi);
    r*v - q*w - g*sin(theta);
    p*w - r*u + g*cos(theta)*sin(phi);
    q*u - p*v - k1*(u1^2 + u2^2 + u3^2 + u4^2)/(M + 4*m) + g*cos(phi)*cos(theta);
    p + r*cos(phi)*tan(theta) + q*sin(phi)*tan(theta);
    q*cos(phi) - r*sin(phi);
    (r*cos(phi))/cos(theta) + (q*sin(phi))/cos(theta);
    (l*(k1*u2^2 - k1*u4^2))/((2*M*R^2)/5 + 2*m*l^2) - (2*l^2*m*q*r)/((2*M*R^2)/5 + 2*m*l^2);
    (l*(k1*u1^2 - k1*u3^2))/((2*M*R^2)/5 + 2*m*l^2) + (2*l^2*m*p*r)/((2*M*R^2)/5 + 2*m*l^2);
    (k2*(u1 + u2 + u3 + u4))/((2*M*R^2)/5 + 4*m*l^2);
    Anonlinear * xhat + Bnonlinear * -(controller_gains.K1 * xhat + controller_gains.K2 * z) + controller_gains.H * C * (x - xhat);
%     A * xhat + B * u_control + controller_gains.H * C * (x - xhat);
    C2 * x - [1; 1; -1.0; 0] % reference is 0...
];
end
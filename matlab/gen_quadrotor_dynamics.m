%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This script does the following                                          %
%     1. Generates the dynamics of a quadrotor drone symbolically,        %
%     2. linearizes the nonlinear system of differential equations that   %
%     precipitates about the hover equilibrium point (where the drone     %
%     stays at a fixed attitude and alitude, no velocities, and no        %
%     accelerations).                                                     %
%     3. Designs an observer-based state-feedback controller with integral%
%     action using LQR for both the Luenberger observer and the controller%
%     gains.                                                              %
%     4. Simulates the linear system with feedback controller to          %
%     demonstrate it working                                              %
%     5. Plots Bode / Sigma plots of the effective loop gain, output      %
%     sensitivity, and complementary output sensitivity functions.        %
%                                                                         %
% Takeaways: sensitivity functions demonstrate that the system exhibits   %
% performance limitations most likely due to the observer in the feedback %
% loop.                                                                   %
%                                                                         %
% Next steps: create both nonlinear and linear simulations / animations of%
%     1. Create both nonlinear and linear simulations / animations of the %
%     system, capturing all state variables not just observables          %
%     2. See how well the controller performs in nonlinear case           %
%     3. Incorporate control signal saturation                            %
%     3. Randomize all the system parameters, perfom a simulation of the  %
%     system, collect observables. Use system identification toolbox to   %
%     compute unknown, randomized parameters                              %
%                                                                         %
% Author: Zach Bortoff                                                    %
% Last Updated: 09/19/2022                                                %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Add functions in 'utils' folder to path
clear; clc;
addpath('utils')

%% Quadrotor State Variables (6 DOF, 12 state variables)
% Position state variables are all w.r.t. inertial frame, namely NED frame
syms p_n real % inertial (North) position of the quadrotor in NED frame [m]
syms p_e real % inertial (East) position of the quadrotor in NED frame [m]
syms p_d real % inertial (Down) position of the quadrotor in NED frame [m]
h = -p_d; % altitude state variables (used over p_d) is negative of 'down'

% Velocity state variables are all w.r.t. body-frame
syms u real % body-frame velocity 'forward' [m/s]
syms v real % body-frame velocity 'starboard' (right wing) [m/s]
syms w real % body-frame velocity 'down' [m/s]

% Euler angles (roll, pitch, yaw) are defined in different frames 
syms phi real % roll defined w.r.t. vehicle-2 frame [rad]
syms theta real % pitch  defined w.r.t. vehicle-1 frame [rad]
syms psi real % yaw defined w.r.t. vehicle frame [rad]

% Angular rates are defined w.r.t. body-frame
syms p real % roll rate [rad/s]
syms q real % pitch rate [rad/s]
syms r real % yaw rate [rad/s]

%% Quadrotor Model Parameters
syms M real % mass of fuselage (kg)
syms m real % mass of rotor (kg)
syms R real % 'radius' of solid sphere representing inertia of fuselage (m)
syms L real % length of strut from fuselage to rotor (m)
syms l real % length of rotor (m)
syms g real % acceleration due to gravity near surface of Earth (m/s^2)
syms k1 k2 real % experimentally determined coeffs

% Assumption: Quadrotor inertia matrix constant, diagonal matrix modeled by
% solid sphere representing fuselage and 4 point masses representing rotors
Jx = 2 / 5 * M * R^2 + 2 * m * l^2;
Jy = Jx;
Jz = 2 / 5 * M * R^2 + 4 * m * l^2;
J = diag([Jx Jy Jz]);

%% Control inputs
syms u1 real % angular velocity of rotor 1 (front)
syms u2 real % angular velocity of rotor 2 (right)
syms u3 real % angular velocity of rotor 3 (back)
syms u4 real % angular velocity of rotor 4 (left)

F1 = k1 * u1^2;
F2 = k1 * u2^2;
F3 = k1 * u3^2;
F4 = k1 * u4^2;
tau_phi = l * (F2 - F4);
tau_theta = l * (F1 - F3);
tau_psi = k2 * (u1 + u2 + u3 + u4);

%% Define basic frame transformations
% Vehicle frame to body frame (must be in the order below)
Rb_v = Rx(phi) * Ry(theta) * Rz(psi);

%% Quadrotor Kinematics
% Kinematics deals with the relationship between the position state
% variables and the body-frame velocity as well as the relationship between
% the Euler angles and the angular rates

% Relationship between position state variables and body-frame velocity
pos_dot = Rb_v' * [u; v; w];

% Relationship between Euler angles and angular rates
ang_dot = [1 sin(phi) * tan(theta) cos(phi) * tan(theta); 0 cos(phi) -sin(phi); 0 sin(phi) * sec(theta) cos(phi) * sec(theta)] * [p; q; r];

%% Quadrotor Dynamics
% Linear accelerations
pos_ddot = [r * v - q * w; p * w - r * u; q * u - p * v] + [-g * sin(theta); g * cos(theta) * sin(phi); g * cos(theta) * cos(phi)] + 1 / (4 * m + M) * [0; 0; -(F1 + F2 + F3 + F4)];

% Angular accelerations
ang_ddot = [(Jy - Jz) / Jx * q * r; (Jz - Jx) / Jy * p * r; (Jx - Jy) / Jz * p * q] + [ 1 / Jx * tau_phi; 1 / Jy * tau_theta; 1 / Jz * tau_psi];

%% Nonlinear State-Space Realization
A_nonlinear = [pos_dot; pos_ddot; ang_dot; ang_ddot];
x_state = [p_n; p_e; p_d; u; v; w; phi; theta; psi; p; q; r];
u_control = [u1; u2; u3; u4];
u0 = sqrt(g * (M + 4 * m) / (4 * k1));

%% Simplified Model for Control Purposes
p_n_dot = u;
p_e_dot = v;
p_d_dot = w;
phi_dot = p;
theta_dot = q;
psi_dot = r;
p_n_ddot = (-cos(phi) * sin(theta) * cos(psi) - sin(phi) * sin(psi)) * (F1 + F2 + F3 + F4) / (M + 4 * m);
p_e_ddot = (-cos(phi) * sin(theta) * sin(psi) + sin(phi) * cos(psi)) * (F1 + F2 + F3 + F4) / (M + 4 * m);
p_d_ddot = g - (cos(phi) * cos(theta)) * (F1 + F2 + F3 + F4) / (M + 4 * m);
phi_ddot = 1 / Jx * tau_phi;
theta_ddot = 1 / Jy * tau_theta;
psi_ddot = 1 / Jz * tau_psi;

%% State-Space Representation of Simplified Model
A_simple_nonlinear = [p_n_dot; p_e_dot; p_d_dot; p_n_ddot; p_e_ddot; p_d_ddot; phi_dot; theta_dot; psi_dot; phi_ddot; theta_ddot; psi_ddot];

%% Linearize about the hover equilibrium (don't incorporate psi)
A = subs(jacobian(A_simple_nonlinear, x_state'), [u, v, w, p, q, r, theta, phi, u1, u2, u3, u4], [0, 0, 0, 0, 0, 0, 0, 0, u0, -u0, u0, -u0]);
B = subs(jacobian(A_simple_nonlinear, u_control'), [u, v, w, p, q, r, theta, phi, u1, u2, u3, u4], [0, 0, 0, 0, 0, 0, 0, 0, u0, -u0, u0, -u0]);
equilibrium_poles = eig(A)

%% Controllability of linearized model (A, B)
Mc = [B A*B A^2*B A^3*B A^4*B A^5*B A^6*B A^7*B A^8*B A^9*B A^10*B A^11*B A^12*B];
controllability = rank(Mc) % controllability is 12, so fully controllable

%% Define Measurement Model (C,D)
% GPS measures (p_n, p_e, p_d)
% gyros measure (p, q, r)
% accelerometer + magnotometer measures (phi, theta, psi)
% -> N.B. we can definitely do better here... I'm just not sure how exactly
% to do so... 
C = [
    1 0 0 0 0 0 0 0 0 0 0 0;
    0 1 0 0 0 0 0 0 0 0 0 0;
    0 0 1 0 0 0 0 0 0 0 0 0;
    0 0 0 0 0 0 1 0 0 0 0 0;
    0 0 0 0 0 0 0 1 0 0 0 0;
    0 0 0 0 0 0 0 0 1 0 0 0;
    0 0 0 0 0 0 0 0 0 1 0 0;
    0 0 0 0 0 0 0 0 0 0 1 0;
    0 0 0 0 0 0 0 0 0 0 0 1;
];

C2 = [
    1 0 0 0 0 0 0 0 0 0 0 0;
    0 1 0 0 0 0 0 0 0 0 0 0;
    0 0 1 0 0 0 0 0 0 0 0 0;
    0 0 0 0 0 0 0 0 1 0 0 0;
];
D = zeros(size(C, 1), size(B, 2));

%% Observability of linearized model (A,C)
Mob = [C; C*A; C*A^2; C*A^3; C*A^4; C*A^5; C*A^6; C*A^7; C*A^8; C*A^9; C*A^10; C*A^11; C*A^12];
observability = rank(Mob) % observability is 12, so fully observable
Mob2 = [C2; C2*A; C2*A^2; C2*A^3; C2*A^4; C2*A^5; C2*A^6; C2*A^7; C2*A^8; C2*A^9; C2*A^10; C2*A^11; C2*A^12];
observability2 = rank(Mob2) % observability is 12, so fully observable

%% Poles and Zeros of the Linear Open-Loop System
psi_eval = 0;
% psi_eval = pi/2;
A_realized = double(simplify(subs(A, [M m R L l R k1 k2 g psi], [1 1 1 1 1 1 1 1 9.81 psi_eval])));
B_realized = double(simplify(subs(B, [M m R L l R k1 k2 g psi], [1 1 1 1 1 1 1 1 9.81 psi_eval])));
assert(rank(ctrb(A_realized, B_realized)) == 12); % rank(ctrb) must be 12 for controllability
sys_realized = ss(A_realized, B_realized, C, D);
sys_zeros = tzero(sys_realized); % no open-loop zeros
sys_poles = pole(sys_realized); % all poles are unstable

%% Define performance specifications (to be modified later)
% phase margin specs? gain margin specs?

%% Controller Design
% We will use observer-based state-feedback controller with integral action
% to stabilize the system about the equilibrium condition and meet
% performance specifications using LQR technique...

% Step 1. Augment the plant with integral action...
Atilde = [
    A_realized zeros(size(A, 1), size(C, 1));
    C zeros(size(C, 1), size(C, 1))
];
Btilde = [
    B_realized;
    zeros(size(C, 1), size(B, 2))
];
assert(size(Atilde, 1) == size(Atilde, 2));
assert(size(Btilde, 1) == size(Atilde, 1));
controllability_tilde = rank(ctrb(Atilde, Btilde)) % 16 => not controllable
size(Atilde) % 21 x 21

Atilde2 = [
    A_realized zeros(size(A, 1), size(C2, 1));
    C2 zeros(size(C2, 1), size(C2, 1))
];
Btilde2 = [
    B_realized;
    zeros(size(C2, 1), size(B, 2))
];
assert(size(Atilde2, 1) == size(Atilde2, 2));
assert(size(Btilde2, 1) == size(Atilde2, 1));

controllability_tilde2 = rank(ctrb(Atilde2, Btilde2)) % 16 => controllable
size(Atilde2) % 16 x 16

%% Design integral action controller
% from the results above, it's clear that the system is only controllable
% if we add integrators just on the (p_n, p_e, p_d, yaw) state variables...
Qtilde2 = eye(16, 16); Rtilde2 = eye(4,4);
Qtilde2(10:12,10:12) = eye(3,3); % change cost to make go faster/slower to these dynamics
Qtilde2(16,16) = 1; % reduce cost of bringing closing reference gap (slow down these dynamics)
Ktilde2 = lqr(Atilde2, Btilde2, Qtilde2, Rtilde2);

% 1:12 because that's the # of state vars; 13:16 b/c 13 = 12 + 1, 16 = 12
% + 4 and 4 is # of virtual integrator signals for (p_n, p_e, p_d, yaw)
Ktilde2_1 = Ktilde2(:,1:12); Ktilde2_2 = Ktilde2(:,13:16); 

% Observer 'H' gain
Q = eye(12,12); R = eye(9,9);
H = lqr(A_realized', C', Q, R)';
% H = place(A_realized', C', [-16, -16.1, -16.2, -16.3, -16.4, -16.5, -16.6, -16.7, -16.8, -16.9, -16.11, -16.12])';

% Note: This controller design works because of the separation principle...
% we use 'C' in devising the Luenburger observer 'H', but 'C2' in devising
% the controller gains... Ktilde2_1 represents gains transforming the state
% estimate into control inputs and Ktilde2_2 represents gains transforming
% the integral virtual signals for the (pn,pe,pd,yaw) signals...

%% Simulate full controller...
T = 0:0.01:25;
num_references = size(C2, 1); % this is because (p_n, p_e, p_d, yaw)

Acl = [
    A_realized-B_realized*Ktilde2_1 -B_realized*Ktilde2_2 B_realized*Ktilde2_1; 
    C2 zeros(size(C2, 1), size(-B*Ktilde2_2, 2)) zeros(size(C2, 1), size(B*Ktilde2_1, 2));
    zeros(size(A-H*C, 1), size(C2, 2)) zeros(size(A-H*C, 1), size(-B*Ktilde2_2, 2)) A_realized-H*C
];
Bcl = [zeros(size(A, 1), num_references); -eye(num_references, num_references); zeros(size(A, 1), num_references)];
Ccl = [C zeros(size(C, 1), num_references) zeros(size(C, 1), size(A, 1))];
Dcl = zeros(size(Ccl, 1), size(Bcl, 2));
cls = ss(Acl, Bcl, Ccl, Dcl);
cls_poles = eig(cls);
observer_poles = cls_poles(end-(size(H',2)-1):end); % -1 just cuz of MATLAB indexing
system_poles = cls_poles(1:size(Ktilde2, 2));
cls_zeros = tzero(cls);
controllability_gramian = gram(cls, 'c');
observability_gramian = gram(cls, 'o');

%% Simulate linear closed-loop system...
% the output vector y_pn or y_pe or y_pd or y_psi from the function call to
% 'lsim' is what we measure y = Cx, so we only return a subset of the
% state-variables... the inputs are reference signals to the pn pe pd or
% psi variables. the final argument to the function 'lsim' are the state
% initial conditions!
y_pn = lsim(cls, [10*ones(length(T), 1) zeros(length(T), 3)], T, zeros(28,1));
plot(T, y_pn, 'LineWidth', 2); legend('p_n', 'p_e', 'p_d', '\phi', '\theta', '\psi', 'p', 'q', 'r'); grid on; title('Response to Step Reference for p_n'); xlabel('Time [s]');

y_pe = lsim(cls, [zeros(length(T), 1) ones(length(T), 1) zeros(length(T), 2)], T, zeros(28,1));
plot(T, y_pe); grid on;

y_pd = lsim(cls, [zeros(length(T), 2) ones(length(T), 1) zeros(length(T), 1)], T, zeros(28,1));
plot(T, y_pd); grid on;

y_psi = lsim(cls, [zeros(length(T), 3) ones(length(T), 1)], T, zeros(28,1));
plot(T, y_psi); grid on;

y_all = lsim(cls, [ones(length(T), 4)], T, zeros(28,1));
plot(T, y_all); grid on;

%% Perform another set of simulations, but this time with error != 0
% make the initial condition of the estimated state != actual state...
y_pn = lsim(cls, [ones(length(T), 1) zeros(length(T), 3)], T, [zeros(28-12,1); 0.1*ones(28-12-4,1)]);
plot(T, y_pn); grid on;

y_pe = lsim(cls, [zeros(length(T), 1) ones(length(T), 1) zeros(length(T), 2)], T, [zeros(28-12,1); 0.1*ones(28-12-4,1)]);
plot(T, y_pe); grid on;

y_pd = lsim(cls, [zeros(length(T), 2) ones(length(T), 1) zeros(length(T), 1)], T, [zeros(28-12,1); 0.1*ones(28-12-4,1)]);
plot(T, y_pd); grid on;

y_psi = lsim(cls, [zeros(length(T), 3) ones(length(T), 1)], T, [zeros(28-12,1); 0.1*ones(28-12-4,1)]);
plot(T, y_psi); grid on;

%% Determine the Input Effective Loop Gain (L_i) (i.e. the open-loop gain)
K1 = Ktilde2_1;
K2 = Ktilde2_2;
A_Li = [
    A_realized zeros(size(A_realized, 1), size(-B*K2, 2)) zeros(size(A_realized, 1), size(A_realized - B * K1 - H * C, 2));
    C2 zeros(size(C2, 1), size(-B_realized*K2, 2)) zeros(size(C2, 1), size(A_realized - B * K1 - H * C, 2));
    H * C -B_realized * K2 A_realized - B_realized * K1 - H * C
];

B_Li = [B_realized; zeros(size(C2, 1), size(B, 2)); zeros(size(A, 1), size(B, 2))];
C_Li = [zeros(size(K2, 1), size(A_realized, 2)) K2 K1];
D_Li = zeros(size(C_Li, 1), size(B_Li, 2));

L_i = minreal(ss(A_Li, B_Li, C_Li, D_Li)); % input effective Loop gain

%% Plot the Bode / Sigma plot of the Input effective loop gain
[s_Li, w_Li] = sigma(L_i); grid on;
% bode(L_i); grid on;

%% Plot the Bode / Sigma plot of the Input sensitivity function
S_i = minreal(inv(minreal(eye(4,4) + L_i))); % input sensitivity function
% bode(S_i); grid on;
[s_Si, w_Si] = sigma(minreal(eye(4,4) + L_i), [], 1); grid on; % humps in sensitivity function represent frequency regions of bad tracking/disturbance rejection performance

%% Plot the Bode / Sigma plot of the Input  complementary sensitivity function
T_i = minreal(L_i * S_i); % input complementary sensitivity function -> multiplication of states -> numerical inaccuracies
% bode(T_i); grid on;
[s_Ti, w_Ti] = sigma(T_i); grid on;

%% Input Disk margin (quantification of robustness)
[dmi, mmi] = diskmargin(L_i);

%% Determine the Output Effective Loop Gain (L_o) (i.e. the open-loop gain)
A_Lo = [
    A_realized -B_realized*K2 -B_realized*K1;
    zeros(size(C2, 1), size(A,2)) zeros(size(C2, 1), size(-B*K2,2)) zeros(size(C2,1), size(-B*K1,2));
    zeros(size(-B_realized*K2, 1), size(A_realized, 2)) -B_realized*K2 A_realized - H*C - B_realized*K1
];
B_Lo = [
    zeros(size(A_realized, 1), size(H, 2));
    -eye(4,9);
    -H
];

% the selection below is the same as the one above, except we make some
% unstable pole-zero cancellations in the one below... this causes some
% singular values to get cancelled out so the plots are prettier, but the
% singular value values are also -inf, so that's not good...
% A_Lo = [ 
%     A_realized -B_realized*K2 -B_realized*K1;
%     zeros(size(C2, 1), size(A,2)) zeros(size(C2, 1), size(-B*K2,2)) zeros(size(C2,1), size(-B*K1,2));
%     H*C -B_realized*K2 A_realized - H*C - B_realized*K1
% ];
% B_Lo = [
%     zeros(size(A_realized, 1), size(H, 2));
%     -eye(4,9);
%     zeros(size(H))
% ];
C_Lo = [C zeros(size(C, 1), 4) zeros(size(C, 1), 12)];
D_Lo = zeros(size(C_Lo, 1), size(B_Lo, 2));

L_o = minreal(ss(A_Lo, B_Lo, C_Lo, D_Lo)); % might be wrong... there is literially 0 phase margin and 1 gain margin... so literally no margin

%% Plot the Bode / Sigma plot of the output effective loop gain
[s_Lo, w_Lo] = sigma(L_o); grid on;
% bode(L_o); grid on;

%% Plot the Bode / Sigma plot of the Output sensitivity function
S_o = minreal(inv(minreal(eye(9,9) + L_o))); % output sensitivity function
% bode(S_o); grid on;
[s_So, w_So] = sigma(S_o); grid on;

%% Plot the Bode / Sigma plot of the Output complementary sensitivity function
T_o = minreal(L_o * S_o); % output complementary sensitivity function
% bode(T_o); grid on;
[s_To, w_To] = sigma(T_o); grid on;

%% Output Disk margin (quantification of robustness)
[dmo, mmo] = diskmargin(L_o);


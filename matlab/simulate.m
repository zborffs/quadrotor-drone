%% Add functions in 'utils' folder to path
% clear; clc;
% addpath('utils')

%% Define the Quadrotor model parameters
K1 = [
   -2.1446   -0.0000   -1.0163   -2.8985   -0.0000   -0.7828   -0.0000   20.5626    2.3984   -0.0000    7.5529    5.5024;
   -0.0000   -2.1446    1.0163   -0.0000   -2.8985    0.7828  -20.5626    0.0000    2.3984   -7.5529   -0.0000    5.5024;
    2.1446    0.0000   -1.0163    2.8985    0.0000   -0.7828    0.0000  -20.5626    2.3984   -0.0000   -7.5529    5.5024;
    0.0000    2.1446    1.0163    0.0000    2.8985    0.7828   20.5626   -0.0000    2.3984    7.5529   -0.0000    5.5024
];

K2 = [
   -0.7071   -0.0000   -0.5000    0.5000;
   -0.0000   -0.7071    0.5000    0.5000;
    0.7071    0.0000   -0.5000    0.5000;
   -0.0000    0.7071    0.5000    0.5000
];
H = [
    4.0061    0.0000   -0.0000   -0.0000   -0.8180    0.0000    0.0000   -0.1674   -0.0000;
   -0.0000    4.0061   -0.0000    0.8180    0.0000    0.0000    0.1674   -0.0000   -0.0000;
   -0.0000    0.0000    1.7321   -0.0000    0.0000   -0.0000    0.0000   -0.0000   -0.0000;
    7.8728    0.0000   -0.0000   -0.0000   -3.9852    0.0000    0.0000   -1.1515   -0.0000;
   -0.0000    7.8728   -0.0000    3.9852    0.0000    0.0000    1.1515   -0.0000   -0.0000;
   -0.0000    0.0000    1.0000   -0.0000    0.0000   -0.0000   -0.0000   -0.0000   -0.0000;
   -0.0000    0.8180    0.0000    0.9878    0.0000    0.0000    0.4040   -0.0000   -0.0000;
   -0.8180   -0.0000    0.0000   -0.0000    0.9878   -0.0000   -0.0000    0.4040    0.0000;
   -0.0000   -0.0000    0.0000   -0.0000    0.0000    1.2872    0.0000    0.0000    0.4142;
   -0.0000    0.1674   -0.0000    0.4040    0.0000    0.0000    0.8993    0.0000    0.0000;
   -0.1674   -0.0000    0.0000   -0.0000    0.4040   -0.0000   -0.0000    0.8993    0.0000;
    0.0000    0.0000    0.0000   -0.0000   -0.0000    0.4142   -0.0000   -0.0000    0.9102
];

quadrotor_params = QuadrotorParameters(1, 1, 1, 1, 1, 9.81, 1, 1);
controller_gains = struct('K1', K1, 'K2', K2, 'H', H);

%% 
% Observation: when we increase the magnitude of any of the elements of the
% reference step vector beyond a certain (yet undetermined) point, then the
% system goes unstable... in effect, we are trying to command a position or
% orientation of the drone (or both simultaneously), which, for one reason
% or another, causes the drone to go unstable
% Interpretation: This observation can be understand as follows: the drone
% gets sufficiently far from the equilibrium point that the assumptions
% under which the controller stabilizes the system are no longer true and
% therefore the controller is no longer guaranteed to be stabilizing... 
% Thought: There is no difference in the amount of time it takes the linear
% closed-loop system to go to a commanded step of 10 or a commanded step of
% 1, because the closed-loop poles are fixed in the linear system
% simulation and the time-constants governing the response of the system to
% a step-reference don't change as a function of the magnitude of the step
% references, because the time-constants are really just the poles of the
% closed-loop system and the poles of the closed-loop system are fixed in
% the linear case... However, when we command the drone to go to position 
% (10,10) vs position (1,1) it fails, because the Euler angles, angular 
% rates, and linear
% velocities necessary to get to 50% of the way to (10, 10) in the same
% amount of time as it takes to get 50% of the way to (1,1) deviates too
% far from the equilibrium point (which by definition assumes 0 angular
% rates and 0 linear velocities. There is another related assumption made
% during the design of the controller, namely that the angular rates were
% sufficiently small that the product of angular rates was negligible...
% This assumption was made in order to reduce the complexity of the
% nonlinear model so that the controller could be simpler... This produces
% another reason for enforcing the same constraint that the rates and
% velocities of the drone must not exceed a certain magnitude for fear of
% violating the assumptions that guarantee stabilization...
% Thought: There are three ways I see of solving this problem... (1) never
% drive the system to a reference for which we know the system will go
% unstable. The pros of this solution are that we can go on using the
% underlying feedback control law, the cons are that, there is no
% theoretical grounds for knowing which references actually cause the
% instability or how such reference might change if we change the system
% parameters or the control law itself. Therefore, in order to get this to
% work, it would seem one would have to determine the border line between
% the destabilizing references and the stable references for a given
% controller design and for given system parameters and hope that the
% simulation results would be the same or more conservative than the real
% world results. (2) 
tspan = [0 20];
x0 = [
    0; 0; -1.0; 0; 0; 0; 0; 0; 0; 0; 0; 0;
    0; 0; -1.0; 0; 0; 0; 0; 0; 0; 0; 0; 0;
    0; 0; 0; 0
];
[t, y] = ode45(@(t, x)quadrotor_ode(t, x, quadrotor_params, controller_gains), tspan, x0);
u_control = compute_control(y', quadrotor_params, controller_gains);

% figure(1); plot(t,y(:,1:12)); grid on;
figure(2); plot(t,y); grid on;
figure(3); plot(t,u_control); grid on;

Ts = 0.02;
k = 1;
tvec = [t(k)];
yvec = [y(k,:)];
for i = 1:length(t)-1
    if (t(i) - t(k) >= Ts)
        tvec = [tvec; t(i)]; % inefficient way to add elements...
        yvec = [yvec; y(i,:)]; % inefficient way to add elements...
        k = i;
    end
end

drone_animation(yvec(:,1), yvec(:,2), -yvec(:,3), yvec(:,7), yvec(:,8), yvec(:,9));

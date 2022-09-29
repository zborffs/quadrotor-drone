%% Add functions in 'utils' folder to path
clear; clc;
addpath('utils')

%% Define the Quadrotor model parameters
quadrotor_params = QuadrotorParameters(1, 1, 1, 1, 1, 9.81, 1, 1);

%% 
tspan = [0 5];
x0 = [
    0; 0; -1.0; 0; 0; 0; 0; 0; 0; 0; 0; 0;
    0; 0; -1.0; 0; 0; 0; 0; 0; 0; 0; 0; 0;
    0; 0; 0; 0
];
[t, y] = ode45(@(t, x)quadrotor_ode(t, x, quadrotor_params), tspan, x0);
u_control = compute_control(y', quadrotor_params);

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

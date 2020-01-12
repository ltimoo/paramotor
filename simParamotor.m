%% Simulate the simplified paramotor dynamics
% NOTE: You have to first run system_parameters.m in order to generate the 
%       sys_param and trim_eq files. This is used to generate the
%       linearized models for the MPC controller

clear all
%% Load all necessary parameters into the workspace
model_parameters();

%% Define a function handler for the function that defines the paramotor dynamics
dynamics = @(states, input) NonlinearDynamicsParamotor(states, input, m, g, rho, Sc, b, c, Rbp, Rbc, Iyy_pert, Iyy_inv_pert, C_aero_pert, d, tau_m, tau_alpha, tau_delta);

%% Initialize time and state vectors/matrices
T = 30; % Define the simulation length (in seconds)
t = [];
t_input = 0:Ts:T;
states = [];
states_meas = [states_init(1:12)'];
input = [input_init'];
input_long_save = [];
input_lat_save = [];
L = size(input,1);
%options = odeset('AbsTol', 1e-6, 'RelTol',1e-6);

%% Generate reference signal for altitude and heading angle
z_ref_nom = z_ref;
yaw_ref_nom = 0;
yaw_ref_signal = yaw_ref_nom*ones(T/Ts + 1,1);
z_ref_signal = z_ref_nom*ones(T/Ts + 1,1);
% Step input at timestep 20: 2s
%z_ref_signal(100:end) = z_ref_signal(100:end) - 1;
% Step input at timestep 100: 10s
%z_ref_signal(200:end) = z_ref_signal(200:end) - 1;
% Step input at timestep 150: 15s
% z_ref_signal(50:end) = z_ref_signal(50:end) - 0.5;
% z_ref_signal(350:end) = z_ref_signal(350:end) - 0.1;
% z_ref_signal(650:end) = z_ref_signal(650:end) + 0.1;
% z_ref_signal(950:end) = z_ref_signal(950:end) + 0.5;
z_ref_signal(50:end) = z_ref_signal(50:end) - 0;
z_ref_signal(350:end) = z_ref_signal(350:end) - 0;
z_ref_signal(650:end) = z_ref_signal(650:end) + 0;
z_ref_signal(950:end) = z_ref_signal(950:end) + 0;

yaw_ref_signal(50:end) = yaw_ref_signal(50:end) + 0;
yaw_ref_signal(500:end) = yaw_ref_signal(500:end) + 0;
yaw_ref_signal(700:end) = yaw_ref_signal(700:end) + 0;

%% Actual Simulation (might be able to be implemented slightly more efficiently)
% Choose if the combined optimization problem should be used (including a
% constraint for the combination of alpha_i and delta) or if seperate
% optimzation problems should be solved.
seperate = 1; % 0: Combined; 1: Seperated
for i = 1:T/Ts
    dyn_handler = @(t, states) dynamics(states, input(i,:));
    [t_temp, states_temp] = ode23s(dyn_handler, [t_input(i) t_input(i+1)], states_init);
    t = [t;t_temp(1:end-1)];
    states = [states; states_temp(1:end-1,:)];
    states_init = states_temp(end,:);
    e = 0*randn(1,12)*var;                                                                      % Add measurement noise if desired (var is defined in model_parameters)
    states_meas = [states_meas; states_init(1:12) + e];
    states_long = states_meas(i+1,[3,4,6,8,11]);
    states_lat = states_meas(i+1,[5,7,9,10,12]);
    if(seperate == 1)
        input_long = mpc_controller_long_v2(states_long', z_ref_signal(i), input_init(2:3));
        input_long_save = [input_long_save; input_long];                                        % Save the whole predicted input sequence of the MPC controller
        input_lat = mpc_controller_lat(states_lat', yaw_ref_signal(i), input_init(1));
        input_lat_save = [input_lat_save; input_lat];                                           % Save the whole predicted input sequence of the MPC controller
        input = [input; input_lat(1), input_long(:,1)'];
    else
        input_total = mpc_controller([states_long'; states_lat'], z_ref_signal(i), input_init);
        input = [input; input_total']; 
    end
end

%% Plotting and Visualisation
figure(1)
subplot(5,3,1)
plot(t, states(:,1))
hold on
plot(t_input, states_meas(:,1))
hold off
title('Position: x')
xlabel('time [s]')
ylabel('[m]')

subplot(5,3,2)
plot(t, states(:,2))
hold on
plot(t_input, states_meas(:,2))
hold off
title('Position: y')
xlabel('time [s]')
ylabel('[m]')

subplot(5,3,3)
plot(t, states(:,3))
hold on
plot(t_input, states_meas(:,3))
stairs(t_input, z_ref_signal, 'LineStyle', '--')
hold off
title('Position: z')
xlabel('time [s]')
ylabel('[m]')

subplot(5,3,4)
plot(t, states(:,4))
hold on
plot(t_input, states_meas(:,4))
hold off
title('Velocity: x')
xlabel('time [s]')
ylabel('[m/s]')

subplot(5,3,5)
plot(t, states(:,5))
hold on
plot(t_input, states_meas(:,5))
hold off
title('Velocity: y')
xlabel('time [s]')
ylabel('[m/s]')

subplot(5,3,6)
plot(t, states(:,6))
hold on
plot(t_input, states_meas(:,6))
hold off
title('Velocity: z')
xlabel('time [s]')
ylabel('[m/s]')

subplot(5,3,7)
plot(t, states(:,7)*180/pi)
hold on
plot(t_input, states_meas(:,7)*180/pi)
hold off
title('Euler Angle: yaw')
xlabel('time [s]')
ylabel('[deg]')

subplot(5,3,8)
plot(t, states(:,8)*180/pi)
hold on
plot(t_input, states_meas(:,8)*180/pi)
hold off
title('Euler Angle: pitch')
xlabel('time [s]')
ylabel('[deg]')

subplot(5,3,9)
plot(t, states(:,9)*180/pi)
hold on
plot(t_input, states_meas(:,9)*180/pi)
hold off
title('Euler Angle: roll')
xlabel('time [s]')
ylabel('[deg]')

subplot(5,3,10)
plot(t, states(:,10)*180/pi)
hold on
plot(t_input, states_meas(:,10)*180/pi)
hold off
title('Body rate: p')
xlabel('time [s]')
ylabel('[deg/s]')

subplot(5,3,11)
plot(t, states(:,11)*180/pi)
hold on
plot(t_input, states_meas(:,11)*180/pi)
hold off
title('Body rate: q')
xlabel('time [s]')
ylabel('[deg/s]')

subplot(5,3,12)
plot(t, states(:,12)*180/pi)
hold on
plot(t_input, states_meas(:,12)*180/pi)
hold off
title('Body rate: r')
xlabel('time [s]')
ylabel('[deg/s]')

subplot(5,3,13)
stairs(t_input, input(:,1))
hold on
plot(t, states(:,15))
hold off
title('Input: \delta')
xlabel('time [s]')


subplot(5,3,14)
stairs(t_input, input(:,2)*180/pi)
hold on
plot(t, states(:,14)*180/pi)
hold off
title('Input: \alpha_i')
xlabel('time [s]')
ylabel('[deg]')

subplot(5,3,15)
stairs(t_input, input(:,3))
hold on
plot(t, states(:,13))
hold off
title('Input: F_t')
xlabel('time [s]')
ylabel('[N]')


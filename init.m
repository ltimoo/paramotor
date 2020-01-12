%% Run this script if any system parameter was changed (aerodynamic coefficient, moment of inirtia, mass, etc.)
% Calculate a new linearized model for the given parameters in 'parameter.m'
% Note: Vb in 'model_parameter' is a quantity that might have to be tuned
%       in order to get meaningful values for the trim equlibrium. This is
%       due to the fact that it is a degree of freedom that we have to
%       calculate the trim equlibrium. 

% Generated files:  
%       sys_param: Contains the linearized model
%       trim_eq:   Contains trim equilibrium values for states and input

params = parameter();

%% Set up symbolic equations of the nonlinear dynamics
[pos_dot, Vb_dot, euler_dot, omega_dot, pos, V_B, euler, omega, input] = nonLinDyn();

%% Calculate the Jacobian 
[J_A, J_B] = linDyn(pos_dot, Vb_dot, euler_dot, omega_dot, pos, V_B, euler, omega, input);

%% Evaluate the trim equilibrium for numerical values of the parameters and
% evaluate the jacobian at these values
[A, B] = numTrimEquilibrium(Vb_dot, omega_dot, params.Vb, J_A, J_B, params);


%% Split up the states into longitudinal and lateral states and exclude x-y-position
% Get longitudinal dynamics [V_x, V_z, yr, omega_y]
A_long = A([3,4,6,8,11],[3,4,6,8,11]);
B_long = B([3,4,6,8,11],2:3);

% Get lateral dynamics
A_lat = A([5,7,9,10,12],[5,7,9,10,12]);
B_lat = B([5,7,9,10,12],1);

%% Save the linearized model into sys_param
sys_c_long = ss(A_long,B_long,eye(size(A_long)),zeros(size(A_long,1),size(B_long,2)));
sys_c_lat = ss(A_lat,B_lat,eye(size(A_lat)), zeros(size(A_lat,2),size(B_lat,2)));
sys_param.sys_c_long = sys_c_long;
sys_param.sys_c_lat = sys_c_lat;

save sys_param -v7.3 sys_param;
clear
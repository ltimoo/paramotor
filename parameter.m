function params = parameter()
%% Generates a struct with all relevant system parameters (excluding control parameters)
% If any system parameters should be changed, change them here!

params.g = 9.81; % [m/s^2]
params.m_fuselage = 0.9; % [kg] 
params.m_wing = 0.136;
params.m = params.m_fuselage + params.m_wing;
params.Rfw = -1.56; 
params.IMUoffset_z = params.m_wing/(params.m)*params.Rfw;
params.Sc = 1.1; % [m^2]
params.rho = 1.225; % [kg/m^3]
params.b = 2.36; % [m]
params.c = 0.4661; % [m]
params.bpz = -params.IMUoffset_z - 0.08; % [m]
params.bpx = 0;
params.Rbp = [0, -params.bpz, 0; params.bpz, 0, -params.bpx; 0, params.bpx, 0];
Iyy = params.m_fuselage*params.IMUoffset_z^2 + params.m_wing*(params.Rfw - params.IMUoffset_z)^2;
params.Iyy = Iyy;
params.Iyy_inv = 1/Iyy;

params.I = [0.23 0 0; 0 Iyy 0; 0 0 0.03];
params.I_inv = inv(params.I);

%params.Vb = 3.63;


params.Vb = 3.15;

% Lift
%syms C_L_0 real positive
params.C_L_0 = 1.1244; % [-]
params.C_L_alpha = 5.1180; % [-]

% Drag
%syms C_D_0 real positive
params.C_D_0 = 0.2099; % [-]
params.C_D_alpha = 2.2478; % [-]

% Pitching moment
params.C_m_0 = -0.0816; % [-]
params.C_m_alpha = -0.2403; % [-]
params.C_m_q = -1.6482;








% % Lift
% %syms C_L_0 real positive
% params.C_L_0 = 0.82; % [-]
% params.C_L_alpha = 4.40; % [-]
% 
% % Drag
% %syms C_D_0 real positive
% params.C_D_0 = 0.16; % [-]
% params.C_D_alpha = 1.47; % [-]
% 
% % Pitching moment
% params.C_m_0 = -0.0729; % [-]
% params.C_m_alpha = 0.047; % [-]
% params.C_m_q = -1.72;
% % params.C_m_0 = -0.0876; % [-]
% % params.C_m_alpha = 0.3577; % [-]
% % params.C_m_q = -2;

% Rolling moment
params.C_l_delta = 0.0249; % [-]
params.C_l_phi = -0.0298;
params.C_l_p = 0;

% Yawing moment
params.C_n_delta = 0.0533; % [-]
params.C_n_r = -0.0502;


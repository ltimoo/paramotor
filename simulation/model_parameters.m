%% Script used to load all relevant parameters for simulation into the workspace
% Can be used for simulations in MATLAB or SIMULINK
clear all
params = parameter();
control_param = control_parameter();


Ts = control_param.Ts;
g = params.g; % [m/s^2]
m = params.m; % [kg] 
Sc = params.Sc; % [m^2]
rho = params.rho; % [kg/m^3]
b = params.b; % [m]
c = params.c; % [m]
bpz = params.bpz; % [m]
I = params.I;
I_inv = params.I_inv;
IxxI = I_inv(1,1);
IyyI = I_inv(2,2);
IzzI = I_inv(3,3);
Iyy = params.Iyy;
Iyy_inv = params.Iyy_inv;
Iyy_pert = 0.5*Iyy;
Iyy_inv_pert = 1/Iyy_pert;

% Lift
%syms C_L_0 real positive
C_L_0 = params.C_L_0; % [-]
C_L_0_pert = C_L_0*1.2;
C_L_alpha = params.C_L_alpha; % [-]
C_L_alpha_pert = C_L_alpha*1.2;

% Drag
%syms C_D_0 real positive
C_D_0 = params.C_D_0; % [-]
C_D_0_pert = C_D_0*1.2;
C_D_alpha = params.C_D_alpha; % [-]
C_D_alpha_pert = C_D_alpha*1.2;

% Pitching moment
C_m_0 = params.C_m_0; % [-]
C_m_0_pert = C_m_0*1;
C_m_alpha = params.C_m_alpha; % [-]
C_m_alpha_pert = C_m_alpha*1.0;
C_m_q = params.C_m_q;
C_m_q_pert = C_m_q*1.0;

% Rolling moment
C_l_delta = params.C_l_delta; % [-]
C_l_delta_pert = C_l_delta*1;
C_l_phi = params.C_l_phi;
C_l_phi_pert = C_l_phi*1;
C_l_p = params.C_l_p;
C_l_p_pert = C_l_p*1;

% Yawing moment
C_n_delta = params.C_n_delta; % [-]
C_n_delta_pert = C_n_delta*1;
C_n_r = params.C_n_r;
C_n_r_pert = C_n_r*1;

% Time constant of motor --> rise time = 1/tau_m
tau_m = 10;
tau_alpha = 20;
tau_delta = 20;

% Distances from CM to fuselage and canopy center
bpx = 0; % [m] 
bcx = 0; % [m]
bcz = -1; % [m]
Rbp = [0, -bpz, 0; bpz, 0, -bpx; 0, bpx, 0];
Rbc = [0, -bcz, 0; bcz, 0, -bcx; 0, bcx, 0];

% % Rolling moment
C_l_0 = 0; % [-]

% Yawing moment
C_n_0 = 0; % [-]

C_aero = [C_L_0; C_L_alpha; ...
    C_D_0; C_D_alpha; ...
    C_m_0; C_m_alpha; C_m_q; ...
    C_l_delta; C_l_phi; C_l_p; ...
    C_n_delta; C_n_r];

C_aero_pert = [C_L_0_pert; C_L_alpha_pert; ...
    C_D_0_pert; C_D_alpha_pert; ...
    C_m_0_pert; C_m_alpha_pert; C_m_q_pert; ...
    C_l_delta_pert; C_l_phi_pert; C_l_p_pert; ...
    C_n_delta_pert; C_n_r_pert];


% Trimmed equilibrium
delta_eq = 0;
load('trim_eq.mat');
Vb_eq = trim_eq.Vb;
alpha_i_eq = trim_eq.alpha_i;
Ft_eq = trim_eq.Ft;
yr_eq = trim_eq.yr;
u_eq = cos(yr_eq)*Vb_eq;
w_eq = sin(yr_eq)*Vb_eq;
z_ref = -50;

z_init = -50;
yr_init = yr_eq;
zr_init = 0;
Vb_init = Vb_eq;
u_init = cos(yr_init)*Vb_init;
w_init = sin(yr_init)*Vb_init;
Ft_init = Ft_eq;
alpha_i_init = alpha_i_eq;
delta_init = 0;

% Disturbance: 
d = [0;0;0];

% Measurement Noise variance
var = diag([0.1, 0.1, 0.1, 0.001, 0.001, 0.001, pi/180*[1, 0.1, 0.1, 5, 1, 5]]);

% Initial states
states_init = ...
    [0; 0; z_init; ...
    u_init; 0; w_init; ...
    zr_init; yr_init; 0; ...
    0; 0; 0; ...
    Ft_init; alpha_i_init; delta_init];

input_init = [delta_init; alpha_i_init; Ft_init];

% Controller Parameter (for SimuLink Model in case we want to use a PID controller)
P_control_pitch = -0.5;
I_control_pitch = 0;
D_control_pitch = -2;

P_control_altitude = 0.001;
I_control_altitude = 0;
D_control_altitude = 0;



function [A, B, u_eq, x_eq] = numTrimEquilibrium(Vb_dot, omega_dot, Vb, J_A, J_B, params)
%% Input:   
%       Vb_dot:     Symbolic expression for Vb_dot
%       omega_dot:  Symbolic expression for omega_dot
%       Vb:         Desired trim equilibrium velocity
%       J_A:        Jacobian of the nonlinear equations for the states
%       J_B:        Jacobian of the nonlinear equations for the inputs
% Output:
%       A, B:       Linearized dynamics around the trim equilibrium
%       u_eq:       Equilibrium values for inputs
%       x_eq:       Equlibriums values for states
% Generated files:
%       trim_eq:    Struct including all relevant equilibriums values

% Desired heading
ref = 0;


% Omega_eq equals to zero
p = 0;
q = 0;
r = 0;

% Assume steady state flight in theta heading direction
syms yr
u = Vb*cos(yr);
v = 0;
w = Vb*sin(yr);
beta = 0;

% Euler_eq 
xr = 0;
zr = ref;


% Assumptions on certain parameters
delta = 0;
IxyI = 0;
IxzI = 0;
IyzI = 0;
IzxI = 0;
IzyI = 0;
IyxI = 0;
bpy = 0;
bpx = 0;

%% Solve for alpha_i_eq, Ft_eq, u_eq
syms ref_p

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

% Lift
%syms C_L_0 real positive
C_L_0 = params.C_L_0; % [-]
C_L_alpha = params.C_L_alpha; % [-]

% Drag
%syms C_D_0 real positive
C_D_0 = params.C_D_0; % [-]
C_D_alpha = params.C_D_alpha; % [-]

% Pitching moment
C_m_0 = params.C_m_0; % [-]
C_m_alpha = params.C_m_alpha; % [-]
C_m_q = params.C_m_q;

% Rolling moment
C_l_delta = params.C_l_delta; % [-]
C_l_phi = params.C_l_phi;
C_l_p = params.C_l_p;

% Yawing moment
C_n_delta = params.C_n_delta; % [-]
C_n_r = params.C_n_r;




% Solve omega_dot = 0
syms Ft
syms alpha_i
eqn_omega_dot = subs(subs(omega_dot(2))) == 0;
S_omega_dot = solve(eqn_omega_dot, Ft);
Ft = simplify(S_omega_dot, 'IgnoreAnalyticConstraints', true);

% Solve Vb_dot = 0
eqn_Vb_x_dot = subs(Vb_dot(1)) == 0;
S_Vb_x_dot = solve(eqn_Vb_x_dot, alpha_i);
alpha_i = simplify(S_Vb_x_dot, 'IgnoreAnalyticConstraints', true);

eqn_Vb_z_dot = subs(Vb_dot(3)) == 0;
S_Vb_z_dot = solve(eqn_Vb_z_dot, yr);
yr = simplify(S_Vb_z_dot, 'IgnoreAnalyticConstraints', true);


yr_eq = double(yr);
alpha_i_eq = double(subs(alpha_i));
Ft_eq = double(subs(subs(Ft)));
trim_eq.yr = real(yr_eq);
trim_eq.alpha_i = real(alpha_i_eq);
trim_eq.Ft = real(Ft_eq);
trim_eq.Vb = Vb;
%save('trim_eq.mat', '-struct', 'trim_eq');
save trim_eq -v7.3 trim_eq;


A = double(subs(subs(J_A)));
B = double(subs(subs(J_B)));
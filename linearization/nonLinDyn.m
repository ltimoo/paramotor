function [pos_dot, Vb_dot, euler_dot, omega_dot, pos, V_B, euler, omega, input] = nonLinDyn()
%% Set up symbolic expressions for the nonlinear dynamics

% Express dynamics with symbolic toolbox

syms g real positive
syms m real positive
syms Sc real positive
syms rho real positive
syms b real positive
syms t real 
syms d real
syms c real
syms bpx real
syms bpy real
syms bpz real
syms bcx real
syms bcz real

% Lift
syms C_L_0 real positive
syms C_L_alpha real

% Drag
syms C_D_0 real positive
syms C_D_alpha real

% Pitching moment
syms C_m_0 real
syms C_m_alpha real
syms C_m_q real

% Rolling moment
syms C_l_delta real
syms C_l_phi real
syms C_l_p real

% Yawing moment
syms C_n_delta real
syms C_n_r real

syms Ixx real
syms Ixz real
syms Izx real
syms Iyy real
syms Izz real
syms IxxI real
syms IxyI real
syms IxzI real
syms IyxI real
syms IzxI real
syms IyyI real
syms IzzI real
syms IyzI real
syms IzyI real
syms x real
syms y real
syms z real
syms xd real
syms yd real
syms zd real
syms xr real
syms yr real
syms zr real
syms u real positive
syms v real
syms w real
syms p real
syms q real
syms r real

syms delta real
syms alpha_i real
syms Ft real positive


% Define all important states and inputs
euler = [zr; yr; xr];
omega = [p; q; r];
pos = [x; y; z];
V_B = [u; v; w];
input = [delta; alpha_i; Ft];



C_IB = [cos(yr)*cos(zr), cos(zr)*sin(xr)*sin(yr) - cos(xr)*sin(zr), sin(xr)*sin(zr)+cos(xr)*cos(zr)*sin(yr); ...
    cos(yr)*sin(zr), cos(xr)*cos(zr)+sin(xr)*sin(yr)*sin(zr), cos(xr)*sin(yr)*sin(zr)-cos(zr)*sin(xr);...
    -sin(yr), cos(yr)*sin(xr), cos(xr)*cos(yr)];

T_IB = [cos(zr)*sin(yr)/cos(yr), sin(yr)*sin(zr)/cos(yr), 1; ...
    -sin(zr), cos(zr), 0; ...
    cos(zr)/cos(yr), sin(zr)/cos(yr), 0];



% Velocity of the paraglider: Important for aerodynamics
Vb = sqrt(u^2+v^2+w^2);

% In case we have wind: Include it here
Va = Vb;
V_A = V_B;

gamma = atan(V_A(3)/V_A(1));
alpha = gamma + alpha_i;
beta = asin(V_A(2)/Va);

I = [Ixx, 0, Ixz; 0, Iyy, 0; Izx, 0, Izz];
I_inv = [IxxI, IxyI, IxzI; IyxI, IyyI, IyzI; IzxI, IzyI, IzzI];
Omega = [0, -omega(3), omega(2); omega(3), 0, -omega(1); -omega(2), omega(1), 0];
Rbp = [0, -bpz, bpy; bpz, 0, -bpx; -bpy, bpx, 0];

C_L = C_L_0 + C_L_alpha*alpha;
C_D = C_D_0 + C_D_alpha*alpha;
C_m = C_m_0 + C_m_alpha*alpha + C_m_q*q*c/(2*Va);       % pitch
C_l = C_l_delta*delta + C_l_phi*xr + C_l_p*p*b/(2*Va);                     % roll
C_n = C_n_delta*delta + C_n_r*r*b/(2*Va);               % yaw

%% Acting Forces and Moments:
% Gravity
G = C_IB'*[0; 0; m*g];


% Aerodynamic forces 
% Assume small angles beta, resulting in no aerodynamic force due to C_Y
F_A = 0.5*rho*Sc*Va^2* ...
    [C_L*sin(gamma)*cos(beta) - C_D*cos(gamma)*cos(beta); ...
    -C_D*sin(beta)*cos(gamma) + C_L*sin(beta)*sin(gamma); ...
    -C_L*cos(gamma)*cos(beta) - C_D*sin(gamma)*cos(beta)];

% Aerodynamic Moments
M_A = 0.5*rho*Sc*Va^2 * ...
    [b*C_l;...
    IyyI*c*C_m;...
    b*C_n];


% Propeller force
F_T = [Ft;0;0];


%% Dynamics (Vb_dot and omega_dot are both defined in the body frame) --> neglect M_A_ind in comparison to M_A
Vb_dot = simplify(-Omega*V_B + 1/m*(G + F_A + F_T));
omega_dot = simplify(M_A + IyyI*Rbp*F_T);

euler_dot = simplify(T_IB * C_IB * omega);
pos_dot = simplify(C_IB * V_B);
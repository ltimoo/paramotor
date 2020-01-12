function states_dot  = NonlinearDynamicsParamotor(states, input, m, g, rho, Sc, b, c, Rbp, Rbc, Iyy, Iyy_inv, C_aero, d, tau_m, tau_alpha, tau_delta)
%% Aerodynamic Coefficients
% Force
C_L_0 = C_aero(1);
C_L_alpha = C_aero(2);
C_D_0 = C_aero(3);
C_D_alpha = C_aero(4);
% Moment
C_m_0 = C_aero(5);
C_m_alpha = C_aero(6);
C_m_q = C_aero(7);

C_l_delta = C_aero(8);
C_l_phi = C_aero(9);
C_l_p = C_aero(10);

C_n_delta = C_aero(11);
C_n_r = C_aero(12);


u = states(4);
v = states(5);
w = states(6);
zr = states(7);
yr = states(8);
xr = states(9);
p = states(10);
q = states(11);
r = states(12);
Fm = states(13);
alpha_m = states(14);
delta_m = states(15);
delta = input(1);
alpha_i = input(2);
Ft = input(3);

omega = [p; q; r];
V_B = [u; v; w];

C_IB = [cos(yr)*cos(zr), cos(zr)*sin(xr)*sin(yr) - cos(xr)*sin(zr), sin(xr)*sin(zr)+cos(xr)*cos(zr)*sin(yr); ...
    cos(yr)*sin(zr), cos(xr)*cos(zr)+sin(xr)*sin(yr)*sin(zr), cos(xr)*sin(yr)*sin(zr)-cos(zr)*sin(xr);...
    -sin(yr), cos(yr)*sin(xr), cos(xr)*cos(yr)];

T_IB = [cos(zr)*sin(yr)/cos(yr),    sin(yr)*sin(zr)/cos(yr),    1; ...
        -sin(zr),                   cos(zr),                    0; ...
        cos(zr)/cos(yr),            sin(zr)/cos(yr),            0];



Vb = sqrt(u^2+v^2+w^2);
V_A = V_B + C_IB'*d;
Va = norm(V_A,2);

gamma = atan(V_A(3)/V_A(1));
alpha = gamma + alpha_m;
beta = asin(V_A(2)/Va);

Omega = [0, -omega(3), omega(2); omega(3), 0, -omega(1); -omega(2), omega(1), 0];

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
    Iyy_inv*c*C_m;...
    b*C_n];  

% Propeller force
F_T = [Fm;0;0];


%% Dynamics (Vb_dot and omega_dot are both defined in the body frame) --> neglect M_A_ind in comparison to M_A
states_dot = zeros(size(states));
states_dot(13) = tau_m*(Ft - Fm);
states_dot(14) = tau_alpha*(alpha_i - alpha_m);
states_dot(15) = tau_delta*(delta - delta_m);
states_dot(4:6) = -Omega*V_B + 1/m*(G + F_A + F_T);
states_dot(10:12) = (M_A + Iyy_inv*Rbp*F_T);
%states_dot(10:12) = I_inv*(-Omega*I*omega + M_A + Rbp*F_T);    % Neglect
%inertial moments as we don't know the real moment of inertias for x and z
%axis

% Note that we first have to transform omega from body to inertial
% frame, then we can apply the well known relationship between \chi_dot and
% omega
states_dot(7:9) = T_IB * C_IB * omega;
states_dot(1:3) = C_IB * V_B;





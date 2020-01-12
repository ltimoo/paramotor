function param = control_parameter()
%% Generates a struct containing all relevant parameters for control purposes

% Execute system_parameter() if these files don't exist yet, or if we have
% changes in model parameters
load('sys_param.mat');
load('trim_eq.mat');


% Sampling time of MPC controller
Ts = 0.1;
N_long = 30;
N_lat = 30;

% Discretize the linearized longitudinal and lateral models
sys_d_long = c2d(sys_param.sys_c_long, Ts);
[A_long,B_long,~,~] = ssdata(sys_d_long);
sys_d_lat = c2d(sys_param.sys_c_lat, Ts);
[A_lat,B_lat,~,~] = ssdata(sys_d_lat);



% Cost function weighting functions for the longitudinal mpc controller
Q_long = diag([1 1e-1 1e-1 0.1 0.01]);
%Q_long = diag([100 1e-2 1e-2 0.1 0.01]);
R_long = diag([1e3 1]);
R_d_long = diag([10 100]);
[F_inf_long, P_inf_long] = dlqr(A_long, B_long, Q_long, R_long);

% Constraints for the longitudinal mpc controller
u_eq = trim_eq.Vb*cos(trim_eq.yr);
w_eq = trim_eq.Vb*sin(trim_eq.yr);
X_long_ss = [0; u_eq; w_eq; trim_eq.yr; 0];
Xcons_long_lower = -X_long_ss + [-1e3; -1e1; -2; -30*pi/180; -1]; % [z, u, w, pitch, q]
Xcons_long_upper = -X_long_ss + [1e3; 1e1; 2; 30*pi/180; 1];
Xcons_long = [Xcons_long_lower, Xcons_long_upper];
    
U_ss = [trim_eq.alpha_i; trim_eq.Ft];
Ucons_long = -U_ss + [[-0.1274; 0],[0.1274; 8]];
servo_rate_limit = 0.1274*0.2*10;
%servo_rate_limit = 1/(0.12/60) * pi/180 * 0.1; % Speed limit = 0.12 sec/60?
%UrateCons_long = Ts*[[-servo_rate_limit; -0.3],[servo_rate_limit; 0.3]];
UrateCons_long = Ts*[[-servo_rate_limit; -8],[servo_rate_limit; 8]];

gamma_min = -20*pi/180; 
gamma_max = 25*pi/180;
Xcons_long_add = [0 -gamma_max, 1, 0, 0; ...
    0 gamma_min, -1, 0, 0];

% Cost function weighting functions for the lateral mpc controller
Q_lat = diag([0, 0.3, 0.5, 0.1, 0.5]); % Vb,y   yaw  roll  p(roll)  r(yaw)
R_lat = 2;
R_d_lat = 10;
[F_inf_lat, P_inf_lat, CLP_lat] = dlqr(A_lat, B_lat, Q_lat, R_lat);
%P_inf_lat = Q_lat;

% Constraints for the lateral dynamics
v_ss = 0;
zr_ref = 0;
X_lat_ss = [v_ss; zr_ref; 0; 0; 0];
Xcons_lat_lower = -X_lat_ss + [-1e3; -10; -20*pi/180; -1; -1];
Xcons_lat_upper = -X_lat_ss + [1e3; 10; 20*pi/180; 1; 1];
Xcons_lat = -X_lat_ss + [Xcons_lat_lower, Xcons_lat_upper];
Ucons_lat = [-1,1];
UrateCons_lat = [-0.2,0.2];


% Trim equiliriums values 
states_eq = [trim_eq.Vb*cos(trim_eq.yr); 0; trim_eq.Vb*sin(trim_eq.yr); 0; trim_eq.yr; 0; 0; 0; 0];
states_eq_long = states_eq([1,3,5,8]);
states_eq_lat = states_eq([2,4,6,7,9]);
inputs_eq = [0; trim_eq.alpha_i; trim_eq.Ft];
inputs_eq_long = inputs_eq([2,3]);
inputs_eq_lat = inputs_eq(1);

% Soft constraint coefficients
S_long = diag([1, 1, 1, 1, 1]);
v_long = 1e4;

S_lat = diag([1,1,1,1,1]);
v_lat = 1e4;




% Save all necessary values into a struct called "param"
param.Ts = Ts;

param.A_long = A_long;
param.B_long = B_long;

param.A_lat = A_lat;
param.B_lat = B_lat;

param.u_eq_long = inputs_eq_long;
param.u_eq_lat = inputs_eq_lat;

param.states_eq_long = states_eq_long;
param.states_eq_lat = states_eq_lat;

param.Xcons_long = Xcons_long;
param.Ucons_long = Ucons_long;
param.UrateCons_long = UrateCons_long;
param.Xcons_long_add = Xcons_long_add;

param.Xcons_lat = Xcons_lat;
param.Ucons_lat = Ucons_lat;
param.UrateCons_lat = UrateCons_lat;

param.Q_long = Q_long;
param.R_long = R_long;
param.P_long = P_inf_long;
param.F_long = -F_inf_long;
param.R_d_long = R_d_long;

param.Q_lat = Q_lat;
param.R_lat = R_lat;
param.P_lat = P_inf_lat;
param.F_lat = -F_inf_lat;
param.R_d_lat = R_d_lat;

param.S_long = S_long;
param.v_long = v_long;

param.S_lat = S_lat;
param.v_lat = v_lat;

param.N_long = N_long;
param.N_lat = N_lat;

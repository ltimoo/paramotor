function [A,B] = linDyn(pos_dot, Vb_dot, euler_dot, omega_dot, pos, V_B, euler, omega, input)
%% Generate the Jacobian for the nonlinear dynamics

%% Linearization of the dynamics
E1 = jacobian(pos_dot, pos);
E2 = jacobian(pos_dot, V_B);
E3 = jacobian(pos_dot, euler);
E4 = jacobian(pos_dot, omega);
F1 = jacobian(Vb_dot, pos);
F2 = jacobian(Vb_dot, V_B);
F3 = jacobian(Vb_dot, euler);
F4 = jacobian(Vb_dot, omega);
G1 = jacobian(euler_dot, pos);
G2 = jacobian(euler_dot, V_B);
G3 = jacobian(euler_dot, euler);
G4 = jacobian(euler_dot, omega);
H1 = jacobian(omega_dot, pos);
H2 = jacobian(omega_dot, V_B);
H3 = jacobian(omega_dot, euler);
H4 = jacobian(omega_dot, omega);

I = jacobian(Vb_dot, input);
J = jacobian(omega_dot, input);

A = [E1, E2, E3, E4; ...
    F1, F2, F3, F4; ...
    G1, G2, G3, G4; ...
    H1, H2, H3, H4];

B = [zeros(3,3); I; zeros(3,3); J];
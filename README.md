# paramotor
Simulation environment in MATLAB and a working MPC controller for longitudinal and lateral controllers.

How to use:
1. Run 'init.m' in order to generate 'sys_param.mat' and 'trim_eq.mat'.
2. Run 'simParamotor.m' in order to run the simulation

Notes:
1. If any system parameter should be changed (aerodynamic coefficient, moment of inertia, etc.), change them in 'parameter.m' and run 'init.m' to linearize the system around the new trim equilibrium. You might need to slightly change 'Vb' in 'parameter.m' in order to get realistic values for the trim equilibrium.
3. If you want to change the dynamics of the system you have to change 'linearization/nonLinDyn.m' which defines the symbolic expressions of the nonlinear dynamics, and 'simulation/NonlinearDynamicsParamotor.m' which is the function used to simulate the dynamics. Afterwards, you have to run 'init.m' in order to generate the new linearized model (similar to point 2 you might need to change Vb).
4. If you want to change the number of states or something in the same scope, you might need to change more files.
5. To check the robustness of the MPC controller you can simply change the '..._pert' variables in 'simulation/model_parameters.m' to change parameters only in the simulation.

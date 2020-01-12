function inputs = mpc_controller_long_v2(states_long, z_ref, u_long_init)
%% MPC Controller for longitudial dynamics
% Inputs:
%           states_long: States of the longitudinal dynamics (see report for more info)
%           z_ref:  Reference signal for the altitude (minus sign)
%           u_long_init: Initial input
% NOTE: Compared to 'mpc_controller_long.m' this version includes a method
%       to achieve offset-free tracking. This however needs tuning of the 
%       parameters err_tol and fac_corr.

    % controller variables
    persistent param yalmip_optimizer_long input_long err_z z_curr offset counter err_tol fac_corr

    % initialize controller, if not done already
    if isempty(yalmip_optimizer_long)
        [param, yalmip_optimizer_long] = init();
        input_long = u_long_init;
        err_z = 0;
        z_curr = z_ref;
        offset = 0;
        counter = 0;
        err_tol = 40;
        fac_corr = 1/4;
    end
    
    if(abs(z_ref - z_curr) > 1e-3)
       err_z = 0; % Reset error calculation
    end
    %offset = err_z
    if(abs(err_z) > err_tol)
        offset = offset + err_z/(param.Ts*counter)*fac_corr; 
        err_z = 0;
        counter = 0;
    end
    err_z = err_z + param.Ts*(states_long(1) - z_ref);
    counter = counter + 1;
    states_long(1) = states_long(1) + offset;

    %% evaluate control action by solving MPC problem, e.g.
    [u_mpc,errorcode] = yalmip_optimizer_long( [states_long - [z_ref; param.states_eq_long]; input_long-param.u_eq_long] );
    if (errorcode ~= 0)
          warning('MPC infeasible');
    end
    u_mpc = cell2mat(u_mpc);
    inputs = round(u_mpc, 5) + param.u_eq_long;
    input_long = inputs(:,1);
end

function [param, yalmip_optimizer] = init()
    % initializes the controller on first call and returns parameters and
    % Yalmip optimizer object

    param = control_parameter(); % get basic controller parameters
    A = param.A_long;
    B = param.B_long;

    %% implement your MPC using Yalmip here, e.g.
    N = param.N_long;
    nx = size(A,1);
    nu = size(B,2);

    U = sdpvar(repmat(nu,1,N-1),repmat(1,1,N-1),'full');
    X = sdpvar(repmat(nx,1,N),repmat(1,1,N),'full');
    eps1 = sdpvar(repmat(nx,1,N-1),repmat(1,1,N-1), 'full');
    eps2 = sdpvar(repmat(nx,1,N-1),repmat(1,1,N-1), 'full');

    objective = 0;
    constraints = []; % Constraints
    for k = 1:N-2
      %constraints = [constraints, X{k+1} == A*X{k}+B*U{k}, ...
      constraints = [constraints, X{k+1} == A*X{k}+B*U{k}, eps1{k} == 0, eps2{k} == 0, ...
          param.Xcons_long(:,1) + eps1{k} <= X{k+1} <= param.Xcons_long(:,2) + eps2{k}, ...
          param.Ucons_long(:,1) <= U{k} <= param.Ucons_long(:,2), ...
          param.UrateCons_long(:,1) <= U{k+1} - U{k} <= param.UrateCons_long(:,2)];%, ...
          %param.Xcons_long_add*X{k+1} <= 0]; 
      objective = objective + X{k}'*param.Q_long*X{k} + U{k}'*param.R_long*U{k} + ...
          (U{k+1} - U{k})'*param.R_d_long*(U{k+1} - U{k}) + ...
          eps1{k}'*param.S_long*eps1{k} + eps2{k}'*param.S_long*eps2{k} + ...
          param.v_long * norm(eps1{k},1) + param.v_long * norm(eps2{k},1); 
    end
    constraints = [constraints, ...
        param.Xcons_long(:,1) + eps1{N-1} <= X{N} <= param.Xcons_long(:,2) + eps2{N-1},  ...
         X{N} == A*X{N-1}+B*U{N-1}, ...
         param.Ucons_long(:,1) <= U{N-1} <= param.Ucons_long(:,2)];
%     constraints = [constraints, ...
%          X{N} == A*X{N-1}+B*U{N-1}, eps1{N-1} == 0, eps2{N-1} == 0, param.Xcons_long(:,1) + eps1{N-1} <= X{N} <= param.Xcons_long(:,2) + eps2{N-1}];
    objective = objective + X{N-1}'*param.Q_long*X{N-1} + U{N-1}'*param.R_long*U{N-1} + ...
       eps1{N-1}'*param.S_long*eps1{N-1} + eps2{N-1}'*param.S_long*eps2{N-1} + ...
       param.v_long * norm(eps1{N-1},1) + param.v_long * norm(eps2{N-1},1) + ...
       X{end}'*param.P_long*X{end}; % Final cost
    
    % parameter for initial condition
    info = sdpvar(nx + nu,1);
    constraints = [constraints, X{1} == info(1:nx), param.UrateCons_long(:,1) <= U{1} - info(nx+1:nx+nu) <= param.UrateCons_long(:,2)];
    %constraints = [constraints, X{1} == info(1:nx)];

    ops = sdpsettings('solver','quadprog');
    yalmip_optimizer = optimizer(constraints,objective,ops, info, U);
end
function inputs = mpc_controller_long(states_long, z_ref, u_long_init)
%% MPC Controller for longitudial dynamics
% Inputs:
%           states_long: States of the longitudinal dynamics (see report for more info)
%           z_ref:  Reference signal for the altitude (minus sign)
%           u_long_init: Initial input

    % controller variables
    persistent param yalmip_optimizer_long input_long

    % initialize controller, if not done already
    if isempty(yalmip_optimizer_long)
        [param, yalmip_optimizer_long] = init();
        input_long = u_long_init;
    end

    %% evaluate control action by solving MPC problem, e.g.
    [u_mpc,errorcode] = yalmip_optimizer_long( [states_long - [z_ref; param.states_eq_long]; input_long-param.u_eq_long] );
    if (errorcode ~= 0)
          warning('MPC infeasible');
    end
    u_mpc = cell2mat(u_mpc);
    inputs = round(u_mpc(:,1), 5) + param.u_eq_long;
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
      constraints = [constraints, X{k+1} == A*X{k}+B*U{k}, eps1{k} == 0, eps2{k} == 0, ...
          param.Xcons_long(:,1) + eps1{k} <= X{k+1} <= param.Xcons_long(:,2) + eps2{k}, ...
          param.Ucons_long(:,1) <= U{k} <= param.Ucons_long(:,2), ...
          param.UrateCons_long(:,1) <= U{k+1} - U{k} <= param.UrateCons_long(:,2)];%, ...
          %param.Xcons_long_add*X{k+1} <= 0]; 
      objective = objective + X{k}'*param.Q_long*X{k} + U{k}'*param.R_long*U{k} + ...
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
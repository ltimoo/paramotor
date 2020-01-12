function inputs = mpc_controller_lat(states_lat, yaw_ref, u_lat_init)
%% MPC Controller for lateral dynamics
% Inputs:
%           states_lat: States of the lateral dynamics (see report for more info)
%           yaw_ref:  Reference signal for the heading angle
%           u_lat_init: Initial input

    % controller variables
    persistent param yalmip_optimizer_lat input_lat

    % initialize controller, if not done already
    if isempty(yalmip_optimizer_lat)
        [param, yalmip_optimizer_lat] = init();
        input_lat = u_lat_init;
    end

    %% evaluate control action by solving MPC problem, e.g.
    [u_mpc,errorcode] = yalmip_optimizer_lat([states_lat - [param.states_eq_lat(1); -yaw_ref; param.states_eq_lat(3:end)]; input_lat-param.u_eq_lat]);
    if (errorcode ~= 0)
          warning('MPC infeasible');
    end
    u_mpc = cell2mat(u_mpc);
    inputs = round(u_mpc, 5) + param.u_eq_lat;
    input_lat = inputs(1);
end

function [param, yalmip_optimizer] = init()
    % initializes the controller on first call and returns parameters and
    % Yalmip optimizer object

    param = control_parameter(); % get basic controller parameters
    A_lat = param.A_lat;
    B_lat = param.B_lat;

    %% implement your MPC using Yalmip here, e.g.
    N = param.N_lat;
    nx_lat = size(A_lat,1);
    nu_lat = size(B_lat,2);

    U_lat = sdpvar(repmat(nu_lat,1,N-1),repmat(1,1,N-1),'full');
    X_lat = sdpvar(repmat(nx_lat,1,N),repmat(1,1,N),'full');
    eps1_lat = sdpvar(repmat(nx_lat,1,N-1),repmat(1,1,N-1), 'full');
    eps2_lat = sdpvar(repmat(nx_lat,1,N-1),repmat(1,1,N-1), 'full');

    objective = 0;
    constraints = []; % Constraints for x0
    for k = 1:N-2
      constraints = [constraints, X_lat{k+1} == A_lat*X_lat{k}+B_lat*U_lat{k}, ...
          param.Xcons_lat(:,1) + eps1_lat{k} <= X_lat{k+1} <= param.Xcons_lat(:,2) + eps2_lat{k}, ...
          param.Ucons_lat(:,1) <= U_lat{k} <= param.Ucons_lat(:,2), ...
          param.UrateCons_lat(:,1) <= U_lat{k+1} - U_lat{k} <= param.UrateCons_lat(:,2)];%, ...
          %param.Xcons_lat_add*X{k+1} <= 0]; 
      objective = objective + X_lat{k}'*param.Q_lat*X_lat{k} + U_lat{k}'*param.R_lat*U_lat{k} + ...
          (U_lat{k+1} - U_lat{k})'*param.R_d_lat*(U_lat{k+1} - U_lat{k}) + ...
          eps1_lat{k}'*param.S_lat*eps1_lat{k} + eps2_lat{k}'*param.S_lat*eps2_lat{k} + ...
          param.v_lat * norm(eps1_lat{k},1) + param.v_lat * norm(eps2_lat{k},1); 
      %constraints = [constraints, eps1_lat{k} == 0, eps2_lat{k} == 0];
    end
    constraints = [constraints, param.Xcons_lat(:,1) + eps1_lat{N-1} <= X_lat{N} <= param.Xcons_lat(:,2) + eps2_lat{N-1},  X_lat{N} == A_lat*X_lat{N-1}+B_lat*U_lat{N-1}, param.Ucons_lat(:,1) <= U_lat{N-1} <= param.Ucons_lat(:,2)];
    objective = objective + X_lat{N-1}'*param.Q_lat*X_lat{N-1} + U_lat{N-1}'*param.R_lat*U_lat{N-1} + ...
        eps1_lat{N-1}'*param.S_lat*eps1_lat{N-1} + eps2_lat{N-1}'*param.S_lat*eps2_lat{N-1} + ...
          param.v_lat * norm(eps1_lat{N-1},1) + param.v_lat * norm(eps2_lat{N-1},1)...
        + X_lat{end}'*param.P_lat*X_lat{end}; % Final cost
    %constraints = [constraints, eps1_lat{N-1} == 0, eps2_lat{N-1} == 0];
    
    % parameter for initial condition
    info = sdpvar(nx_lat+nu_lat,1);
    constraints = [constraints, X_lat{1} == info(1:nx_lat), param.UrateCons_lat(:,1) <= U_lat{1} - info(nx_lat+1:nx_lat+nu_lat) <= param.UrateCons_lat(:,2)];
    objective = objective + (U_lat{1} - info(nx_lat+1:nx_lat+nu_lat))'*param.R_d_lat*(U_lat{1} - info(nx_lat+1:nx_lat+nu_lat));
    
    ops = sdpsettings('solver','quadprog');
    yalmip_optimizer = optimizer(constraints,objective,ops, info, U_lat);
end
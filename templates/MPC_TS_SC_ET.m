%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (c) 2025, Amon Lahr, Simon Muntwiler, Antoine Leeman, Fabian Flürenbrock & Marco Heim Institute for Dynamic Systems and Control, ETH Zurich.
%
% All rights reserved.
%
% Please see the LICENSE file that has been included as part of this package.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

classdef MPC_TS_SC_ET
    properties
        yalmip_optimizer
    end

    methods
        function obj = MPC_TS_SC_ET(Q, R, N, H, h, S, v, S_t, v_t, params)
            % ADD STUFF HERE
            A = params.model.A;
            B = params.model.B;
            nx = params.model.nx;
            nu = params.model.nu;
            H_x = params.constraints.StateMatrix;
            H_u = params.constraints.InputMatrix;
            h_x = params.constraints.StateRHS;
            n_constr_x = size(h_x, 1);
            n_t = size(h, 1);
            h_u = params.constraints.InputRHS;
            P_inf = LQR(Q, R, params).P_inf;

            X = sdpvar(repmat(nx, 1, N + 1), ones(1, N + 1));
            U = sdpvar(repmat(nu, 1, N), ones(1, N));
            eps_i = sdpvar(repmat(n_constr_x, 1, N), ones(1, N));
            eps_t = sdpvar(n_t, 1);

            constraints = [];
            objective = 0;
            for i = 1:N
                constraints = [constraints, ...
                                X{i + 1} == A * X{i} + B * U{i}, ...
                                H_x * X{i} <= h_x + eps_i{i}, ...
                                H_u * U{i} <= h_u, ...
                                eps_i{i} >= 0];
                objective = objective + X{i}' * Q * X{i} + U{i}' * R * U{i} + eps_i{i}' * S * eps_i{i} + v * norm(eps_i{i}, 1);
            end
            constraints = [constraints, ...
                        H * X{N+1} <= h + eps_t, ...
                        eps_t >= 0];
            objective = objective + X{N+1}' * P_inf * X{N+1} + eps_t' * S_t * eps_t + v_t * norm(eps_t, 1);

            X0 = X(:, 1);
            
            opts = sdpsettings('verbose', 1, 'solver', 'quadprog', 'quadprog.TolFun', 1e-8);
            obj.yalmip_optimizer = optimizer(constraints, objective, opts, X0, {U{1} objective});
        end

        function [u, ctrl_info] = eval(obj,x)
            % evaluate control action by solving MPC problem, e.g.
            tic
            [optimizer_out, errorcode] = obj.yalmip_optimizer(x);
            solvetime = toc;
            [u, objective] = optimizer_out{:};

            feasible = true;
            if (errorcode ~= 0)
                feasible = false;
            end

            ctrl_info = struct('ctrl_feas', feasible, 'objective', objective, 'solvetime', solvetime);
        end
    end
end

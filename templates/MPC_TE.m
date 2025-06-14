%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (c) 2025, Amon Lahr, Simon Muntwiler, Antoine Leeman, Fabian Flürenbrock & Marco Heim Institute for Dynamic Systems and Control, ETH Zurich.
%
% All rights reserved.
%
% Please see the LICENSE file that has been included as part of this package.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

classdef MPC_TE
    properties
        yalmip_optimizer
    end

    methods
        function obj = MPC_TE(Q, R, N, params)
            % ADD STUFF HERE
            nu = params.model.nu;
            H_x = params.constraints.StateMatrix;
            H_u = params.constraints.InputMatrix;
            h_x = params.constraints.StateRHS;
            h_u = params.constraints.InputRHS;
            A = params.model.A;
            B = params.model.B;
            X0 = sdpvar(params.model.nx, 1);
            U = sdpvar(repmat(nu, 1, N), ones(1, N));
            P_inf = LQR(Q, R, params).P_inf;

            constraints = [];
            objective = 0;
            x = X0;
            for k = 1:N
                constraints = [constraints, H_x * x <= h_x, H_u * U{k} <= h_u];
                objective = objective + trace(x' * Q * x) + trace(U{k}' * R * U{k});
                x = A * x + B * U{k};
            end
            objective = objective + trace(x' * P_inf * x);
            constraints = [constraints, x == 0];

            opts = sdpsettings('verbose', 1, 'solver', 'quadprog');
            obj.yalmip_optimizer = optimizer(constraints, objective, opts, X0, {U{1} objective});
        end

        function [u, ctrl_info] = eval(obj, x)
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
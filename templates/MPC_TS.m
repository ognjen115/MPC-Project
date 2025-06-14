%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (c) 2025, Amon Lahr, Simon Muntwiler, Antoine Leeman, Fabian Flürenbrock & Marco Heim Institute for Dynamic Systems and Control, ETH Zurich.
%
% All rights reserved.
%
% Please see the LICENSE file that has been included as part of this package.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

classdef MPC_TS
    properties
        yalmip_optimizer
    end

    methods
        function obj = MPC_TS(Q, R, N, H, h, params)
            % ADD STUFF HERE
            A = params.model.A;
            B = params.model.B;
            nx = params.model.nx;
            nu = params.model.nu;
            H_x = params.constraints.StateMatrix;
            H_u = params.constraints.InputMatrix;
            h_x = params.constraints.StateRHS;
            h_u = params.constraints.InputRHS;
            P_inf = LQR(Q, R, params).P_inf;
            X0 = sdpvar(nx, 1);
            U = sdpvar(repmat(nu, 1, N), ones(1, N));

            constraints = [];
            objective = 0;
            x = X0;
            for i = 1:N
                constraints = [constraints, ...
                                H_x * x <= h_x, ...
                                H_u * U{i} <= h_u];
                objective = objective + x' * Q * x + U{i}' * R * U{i};
                x = A * x + B * U{i};
            end
            constraints = [constraints, H * x <= h];
            objective = objective + x' * P_inf * x;

            opts = sdpsettings('verbose', 1, 'solver', 'quadprog');
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

            ctrl_info = struct('ctrl_feas',feasible,'objective',objective,'solvetime',solvetime);
        end
    end
end

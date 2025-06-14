%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (c) 2025, Amon Lahr, Simon Muntwiler, Antoine Leeman, Fabian Flürenbrock & Marco Heim Institute for Dynamic Systems and Control, ETH Zurich.
%
% All rights reserved.
%
% Please see the LICENSE file that has been included as part of this package.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

classdef MPC_TUBE
    properties
        yalmip_optimizer
        K_tube
    end

    methods
        function obj = MPC_TUBE(Q, R, N, H_N, h_N, H_tube, h_tube, K_tube, params_robust_tube)
            obj.K_tube = K_tube;

            % YOUR CODE HERE
            A = params_robust_tube.model.A;
            B = params_robust_tube.model.B;
            nx = params_robust_tube.model.nx;
            nu = params_robust_tube.model.nu;
            H_x = params_robust_tube.constraints.StateMatrix;
            h_x = params_robust_tube.constraints.StateRHS;
            H_u = params_robust_tube.constraints.InputMatrix;
            h_u = params_robust_tube.constraints.InputRHS;
            P_inf = LQR(Q, R, params_robust_tube).P_inf;
            
            Z = sdpvar(repmat(nx, 1, N + 1), ones(1, N + 1));
            V = sdpvar(repmat(nu, 1, N), ones(1, N));
            X0 = sdpvar(nx, 1); % initial state

            constraints = [H_tube * (X0 - Z{1}) <= h_tube]; % initial error in tube
            objective = 0;
            for i = 1:N
                constraints = [constraints, ...
                                Z{i + 1} == A * Z{i} + B * V{i}, ...
                                H_x * Z{i} <= h_x, ...
                                H_u * V{i} <= h_u];
                objective = objective + Z{i}' * Q * Z{i} + V{i}' * R * V{i};
            end
            constraints = [constraints, ...
                        H_N * Z{N + 1} <= h_N];
            objective = objective + Z{N + 1}' * P_inf * Z{N + 1};

            opts = sdpsettings('verbose', 1, 'solver', 'quadprog');
            obj.yalmip_optimizer = optimizer(constraints, objective, opts, X0, {V{1} Z{1} objective});
        end

        function [u, ctrl_info] = eval(obj, x)
            %% evaluate control action by solving MPC problem, e.g.
            tic;
            [optimizer_out, errorcode] = obj.yalmip_optimizer(x);
            solvetime = toc;
            
            % YOUR CODE HERE
            [v0, z0, objective] = optimizer_out{:};
            u = v0 + obj.K_tube * (x - z0);
            
            feasible = true;
            if (errorcode ~= 0)
                feasible = false;
            end

            ctrl_info = struct('ctrl_feas', feasible,'objective', objective,'solvetime', solvetime);
        end
    end
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (c) 2025, Amon Lahr, Simon Muntwiler, Antoine Leeman, Fabian Flürenbrock & Marco Heim Institute for Dynamic Systems and Control, ETH Zurich.
%
% All rights reserved.
%
% Please see the LICENSE file that has been included as part of this package.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%  TASK18

classdef MPC_TS
    properties
        yalmip_optimizer
    end

    methods
        function obj = MPC_TS(Q,R,N,H,h,params)
            
            % ADD STUFF HERE
                        % Extract system and constraint data
            A = params.model.A;
            B = params.model.B;
            H_x = params.constraints.StateMatrix;
            h_x = params.constraints.StateRHS;
            H_u = params.constraints.InputMatrix;
            h_u = params.constraints.InputRHS;

            nx = size(A, 1);
            nu = size(B, 2);

            % Terminal cost matrix from LQR
            [~, P, ~] = dlqr(A, B, Q, R);

            % YALMIP variables
            x = sdpvar(nx, N+1);
            U = cell(1, N);
            for i = 1:N
                U{i} = sdpvar(nu, 1);
            end
            X0 = sdpvar(nx,1);

            % Objective
            objective = 0;
            for i = 1:N
                objective = objective + x(:,i)' * Q * x(:,i) + U{i}' * R * U{i};
            end
            objective = objective + x(:,N+1)' * P * x(:,N+1);  % terminal cost

            % Constraints
            constraints = [];
            constraints = [constraints, x(:,1) == X0];
            for i = 1:N
                constraints = [constraints, x(:,i+1) == A * x(:,i) + B * U{i}];
                constraints = [constraints, H_x * x(:,i) <= h_x];
                constraints = [constraints, H_u * U{i} <= h_u];
            end
            constraints = [constraints, H * x(:,N+1) <= h];  % terminal set constraint


            opts = sdpsettings('verbose',1,'solver','quadprog');
            obj.yalmip_optimizer = optimizer(constraints,objective,opts,X0,{U{1} objective});
        end

        function [u, ctrl_info] = eval(obj,x)
            %% evaluate control action by solving MPC problem, e.g.
            tic
            [optimizer_out,errorcode] = obj.yalmip_optimizer(x);
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
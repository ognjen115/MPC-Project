%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (c) 2025, Amon Lahr, Simon Muntwiler, Antoine Leeman, Fabian Flürenbrock & Marco Heim Institute for Dynamic Systems and Control, ETH Zurich.
%
% All rights reserved.
%
% Please see the LICENSE file that has been included as part of this package.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% TASK14

classdef MPC
    properties
        yalmip_optimizer
    end

    methods
        function obj = MPC(Q,R,N,params)
           
            % ADD STUFF HERE

            % Extract system matrices from params
            A = params.model.A; 
            B = params.model.B;
            H_x = params.constraints.StateMatrix; 
            h_x = params.constraints.StateRHS; 
            H_u = params.constraints.InputMatrix; 
            h_u = params.constraints.InputRHS; 

            % Get system dimensions
            nx = size(A, 1);
            nu = size(B, 2);

            % Compute LQR infinite-horizon cost for terminal cost l(x_N)
            [~, P, ~] = dlqr(A, B, Q, R); 


            %% Define YALMIP decision variables
            x = sdpvar(nx, N+1); % States x_0, ..., x_N
            U = cell(1, N);      % Cell array for control actions
            for i = 1:N
                U{i} = sdpvar(nu, 1); % Each U{i} is a control action u_i
            end
            x0 = sdpvar(nx, 1);  % Parameter for initial state

            %% Define the cost function
            objective = 0;
            for i = 1:N
                objective = objective + x(:,i)' * Q * x(:,i) + U{i}' * R * U{i};
            end
            objective = objective + x(:,N+1)' * P * x(:,N+1); % Terminal cost l(x_N)


            %% Define constraints
            constraints = [];
            % Initial condition
            constraints = [constraints, x(:,1) == x0];
            % Dynamics
            for i = 1:N
                constraints = [constraints, x(:,i+1) == A * x(:,i) + B * U{i}];
            end
            % State constraints
            for i = 1:N+1
                constraints = [constraints, H_x * x(:,i) <= h_x];
            end
            % Input constraints
            for i = 1:N
                constraints = [constraints, H_u * U{i} <= h_u];
            end

            opts = sdpsettings('verbose',1,'solver','quadprog');
            obj.yalmip_optimizer = optimizer(constraints,objective,opts,x0,{U{1} objective});
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
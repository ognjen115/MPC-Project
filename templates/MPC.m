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
            A = params.model.A; % [nx x nx]
            B = params.model.B; % [nx x nu]
            H_x = params.constraints.StateMatrix; % [n_cx x nx]
            h_x = params.constraints.StateRHS; % [n_cx x 1]
            H_u = params.constraints.InputMatrix; % [n_cu x nu]
            h_u = params.constraints.InputRHS; % [n_cu x 1]

            % Get system dimensions
            nx = size(A, 1);
            nu = size(B, 2);

            % Debug: Check dimensions
            disp(['Size of A: ', mat2str(size(A))]);
            disp(['Size of B: ', mat2str(size(B))]);
            disp(['Size of Q: ', mat2str(size(Q))]);
            disp(['Size of R: ', mat2str(size(R))]);
            disp(['nx: ', num2str(nx), ', nu: ', num2str(nu)]);

            % Compute LQR infinite-horizon cost for terminal cost l(x_N)
            [~, P, ~] = dlqr(A, B, Q, R); 
            disp(['Size of P: ', mat2str(size(P))]);

            % Verify dimensions match
            if size(Q, 1) ~= nx || size(Q, 2) ~= nx
                error('Dimension mismatch: Q should be [%d x %d]', nx, nx);
            end
            if size(R, 1) ~= nu || size(R, 2) ~= nu
                error('Dimension mismatch: R should be [%d x %d]', nu, nu);
            end
            if size(P, 1) ~= nx || size(P, 2) ~= nx
                error('Dimension mismatch: P should be [%d x %d]', nx, nx);
            end

            % Define YALMIP decision variables
            x = sdpvar(nx, N+1); % States x_0, ..., x_N
            U = cell(1, N);      % Cell array for control actions
            for i = 1:N
                U{i} = sdpvar(nu, 1); % Each U{i} is a control action u_i
            end
            x0 = sdpvar(nx, 1);  % Parameter for initial state

            % Define the cost function
            objective = 0;
            for i = 1:N
                state_cost = x(:,i)' * Q * x(:,i);
                input_cost = U{i}' * R * U{i};
                % Ensure scalar by using trace or explicit scalarization
                if ~isscalar(state_cost)
                    state_cost = trace(state_cost);
                end
                if ~isscalar(input_cost)
                    input_cost = trace(input_cost);
                end
                objective = objective + state_cost + input_cost;
            end
            terminal_cost = x(:,N+1)' * P * x(:,N+1);
            if ~isscalar(terminal_cost)
                terminal_cost = trace(terminal_cost);
            end
            objective = objective + terminal_cost; % Terminal cost l(x_N)

            % Define constraints
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
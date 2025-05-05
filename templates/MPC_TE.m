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
        function obj = MPC_TE(Q,R,N,params)

            % ADD STUFF HERE

            % Extract system
            A = params.model.A;
            B = params.model.B;
            H_x = params.constraints.StateMatrix;
            h_x = params.constraints.StateRHS;
            H_u = params.constraints.InputMatrix;
            h_u = params.constraints.InputRHS;
        
            nx = size(A, 1);
            nu = size(B, 2);
        
            % Declare decision variables
            x = sdpvar(nx, N+1);  % x0 to xN
            U = cell(1, N);       % u0 to u_{N-1}
            for i = 1:N
                U{i} = sdpvar(nu, 1);
            end
            X0 = sdpvar(nx, 1);   % initial state parameter
        
            % Objective function
            objective = 0;
            for i = 1:N
                objective = objective + x(:,i)'*Q*x(:,i) + U{i}'*R*U{i};
            end
        
            % No terminal cost (instead we add x_N = 0)
            % Constraints
            constraints = [];
            constraints = [constraints, x(:,1) == X0];  % initial condition
        
            for i = 1:N
                constraints = [constraints, x(:,i+1) == A*x(:,i) + B*U{i}];  % dynamics
                constraints = [constraints, H_x * x(:,i) <= h_x];           % state constraints
                constraints = [constraints, H_u * U{i} <= h_u];             % input constraints
            end
        
            % Terminal state constraint: x_N = 0
            constraints = [constraints, x(:,N+1) == zeros(nx,1)];
        
            % Also enforce state constraint at final step (if needed)
            constraints = [constraints, H_x * x(:,N+1) <= h_x];

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
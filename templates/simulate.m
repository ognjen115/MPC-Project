%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (c) 2025, Amon Lahr, Simon Muntwiler, Antoine Leeman, Fabian Flürenbrock & Marco Heim Institute for Dynamic Systems and Control, ETH Zurich.
%
% All rights reserved.
%
% Please see the LICENSE file that has been included as part of this package.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [Xsim, Usim, ctrl_info] = simulate(x0, ctrl, params)
    % Extract system
    A = params.model.A;
    B = params.model.B;
    N = params.exercise.SimHorizon;

    nx = size(A, 1);
    nu = size(B, 2);

    % Preallocate
    Xsim = zeros(nx, N+1);  % x(0)...x(N)
    Usim = zeros(nu, N);    % u(0)...u(N-1)
    ctrl_info(1:N) = struct('ctrl_feas', false);  % preallocate struct array

    % Initial state
    Xsim(:,1) = x0;

    % Loop
    for k = 1:N
        xk = Xsim(:,k);

        % Evaluate controller
        [uk, info] = ctrl.eval(xk);

        % Store
        Usim(:,k) = uk;
        ctrl_info(k) = info;  


        % State update
        Xsim(:,k+1) = A * xk + B * uk;
    end
end

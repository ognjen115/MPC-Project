%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (c) 2025, Amon Lahr, Simon Muntwiler, Antoine Leeman, Fabian Flürenbrock & Marco Heim Institute for Dynamic Systems and Control, ETH Zurich.
%
% All rights reserved.
%
% Please see the LICENSE file that has been included as part of this package.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% TASK5

function [x_s, u_s] = compute_steady_state(params, d)
    r = params.exercise.T_ref;
    H = params.model.C_ref;

    % Extract system parameters
    A = params.model.A;
    B = params.model.B;
    Bd = params.model.Bd;
    C = params.model.C;
    Cd = params.model.Cd;

    nx = params.model.nx;
    nu = params.model.nu;

    % Define optimization variables
    xs = sdpvar(nx, 1, 'full');
    us = sdpvar(nu, 1, 'full');

    % Build linear equality system
    ss_LHS = [A - eye(nx), B;
              H * C,       zeros(size(H, 1), size(B, 2))];

    ss_RHS = [-Bd * d;
              r - H * Cd * d];

    % Solve
    constr = (ss_LHS * [xs; us] == ss_RHS);
    optimize(constr, [], []);

    % Extract and fix output shapes/types
    x_s = full(double(reshape(value(xs), [], 1)));
    u_s = full(double(reshape(value(us), [], 1)));

end















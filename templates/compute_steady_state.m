%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (c) 2025, Amon Lahr, Simon Muntwiler, Antoine Leeman, Fabian Flürenbrock & Marco Heim Institute for Dynamic Systems and Control, ETH Zurich.
%
% All rights reserved.
%
% Please see the LICENSE file that has been included as part of this package.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [x_s, u_s] = compute_steady_state(params, d)
    C = params.model.C;
    C_ref = params.model.C_ref;
    T_ref = params.exercise.T_ref;
    Cd = params.model.Cd;
    A = params.model.A;
    B = params.model.B;
    Bd = params.model.Bd;
    nx = params.model.nx;
    nu = params.model.nu;
    I = eye(nx);

    lhs = [C_ref * C,   zeros(size(C_ref, 1), size(B, 2));
           A - I,       B];
    rhs = [T_ref - C_ref * Cd * d;
           -Bd * d];
    % compute the Ax=b solution
    sol = lhs \ rhs;
    x_s = sol(1:nx);
    u_s = sol(nx + 1:nx + nu);
end

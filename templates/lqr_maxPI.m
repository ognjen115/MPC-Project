%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (c) 2025, Amon Lahr, Simon Muntwiler, Antoine Leeman, Fabian Flürenbrock & Marco Heim Institute for Dynamic Systems and Control, ETH Zurich.
%
% All rights reserved.
%
% Please see the LICENSE file that has been included as part of this package.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%  TASK12

function [H_LQR, h_LQR] = lqr_maxPI(Q, R, params)
    % Extract system matrices from params
    A = params.model.A; 
    B = params.model.B; 
    H_x = params.constraints.StateMatrix; 
    h_x = params.constraints.StateRHS; 
    H_u = params.constraints.InputMatrix; 
    h_u = params.constraints.InputRHS; 

    % Compute LQR feedback gain K
    [K, ~, ~] = dlqr(A, B, Q, R); 

    % Define F_inf: u = F_inf * x
    F_inf = -K; 

    % Define closed-loop system matrix
    A_cl = A + B * F_inf; 

    % Define constraint polyhedron
    H_input = H_u * F_inf; 
    h_input = h_u; 

    % Combine state and input constraints
    H_total = [H_x; H_input]; 
    h_total = [h_x; h_input]; 

    % Create polyhedron for constraints
    P = Polyhedron('A', H_total, 'b', h_total);

    % Compute maximum positively invariant set iteratively
    X_LQR = computeMaxPosInvariantSet(P, A_cl);

    % Extract H_LQR and h_LQR from the invariant set
    H_LQR = X_LQR.A;
    h_LQR = X_LQR.b;
end

function X = computeMaxPosInvariantSet(P, A_cl)
    max_iter = 100; % Prevent infinite loop

    X = P; % Start with the constraint set
    for i = 1:max_iter
        % Compute the pre-image of X
        H_prev = X.A * A_cl;
        h_prev = X.b;

        % Define the pre-image polyhedron
        X_prev = Polyhedron('A', H_prev, 'b', h_prev);

        % Intersect with the original constraint set
        X_new = X_prev & P;

        % Check for convergence
        if X_new == X
            break;
        end

        X = X_new;
    end

    if i == max_iter
        warning('MaxPosInvariantSet: Did not converge within max iterations');
    end
end







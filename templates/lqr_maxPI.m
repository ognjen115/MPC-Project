%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (c) 2025, Amon Lahr, Simon Muntwiler, Antoine Leeman, Fabian Flürenbrock & Marco Heim Institute for Dynamic Systems and Control, ETH Zurich.
%
% All rights reserved.
%
% Please see the LICENSE file that has been included as part of this package.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [H_LQR, h_LQR] = lqr_maxPI(Q, R, params)
    % Extract system matrices from params
    A = params.model.A; % [nx x nx]
    B = params.model.B; % [nx x nu]
    H_x = params.constraints.StateMatrix; % [n_cx x nx]
    h_x = params.constraints.StateRHS; % [n_cx x 1]
    H_u = params.constraints.InputMatrix; % [n_cu x nu]
    h_u = params.constraints.InputRHS; % [n_cu x 1]

    % Debug: Print dimensions to verify
    disp(['Size of A: ', mat2str(size(A))]);
    disp(['Size of B: ', mat2str(size(B))]);

    % Compute LQR feedback gain K
    [K, ~, ~] = dlqr(A, B, Q, R); % K is [nu x nx]
    disp(['Size of K: ', mat2str(size(K))]);

    % Define F_inf as per document: u = F_inf * x
    F_inf = -K; % [nu x nx]

    % Define closed-loop system matrix
    A_cl = A + B * F_inf; % [nx x nx], equivalent to A - B * K

    % Define constraint polyhedron
    H_input = H_u * F_inf; % [n_cu x nx]
    h_input = h_u; % [n_cu x 1]

    % Combine state and input constraints
    H_total = [H_x; H_input]; % [ (n_cx + n_cu) x nx ]
    h_total = [h_x; h_input]; % [ (n_cx + n_cu) x 1 ]

    % Create polyhedron for constraints
    P = Polyhedron('A', H_total, 'b', h_total);

    % Compute maximum positively invariant set iteratively
    X_LQR = computeMaxPosInvariantSet(P, A_cl);

    % Extract H_LQR and h_LQR from the invariant set
    H_LQR = X_LQR.A;
    h_LQR = X_LQR.b;
end

function X = computeMaxPosInvariantSet(P, A_cl)
    % Iterative computation of the maximum positively invariant set
    max_iter = 100; % Maximum iterations to prevent infinite loop
    tol = 1e-6; % Tolerance for convergence

    X = P; % Start with the constraint set
    for i = 1:max_iter
        % Compute the pre-image manually: X_prev = { x | A_cl * x \in X }
        H = X.A; % Constraints of current set X
        h = X.b;
        % Pre-image constraints: H * (A_cl * x) <= h => (H * A_cl) * x <= h
        H_prev = H * A_cl; % New constraint matrix
        h_prev = h; % Same RHS

        % Define the pre-image polyhedron
        X_prev = Polyhedron('A', H_prev, 'b', h_prev);

        % Intersect with the original constraint set
        X_new = X_prev & P; % Intersection using MPT3

        % Check for convergence
        if X_new == X % MPT3 equality check for polyhedra
            break;
        end

        X = X_new;
    end

    if i == max_iter
        warning('MaxPosInvariantSet: Did not converge within max iterations');
    end
end






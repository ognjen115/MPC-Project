%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (c) 2025, Amon Lahr, Simon Muntwiler, Antoine Leeman, Fabian Flürenbrock & Marco Heim Institute for Dynamic Systems and Control, ETH Zurich.
%
% All rights reserved.
%
% Please see the LICENSE file that has been included as part of this package.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [T1_max, T2_min, T2_max, P1_min, P1_max, P2_min, P2_max, input_cost, cstr_viol] = traj_constraints_cc(Xsim, Usim, params)
    % Extract constraints
    c = params.constraints;

    % Extract state and input components
    T1 = Xsim(1, :);  % all time steps for T1
    T2 = Xsim(2, :);  % all time steps for T2
    P1 = Usim(1, :);  % all time steps for P1
    P2 = Usim(2, :);  % all time steps for P2

    % Temperature extrema
    T1_max = max(T1);
    T2_min = min(T2);
    T2_max = max(T2);

    % Input extrema
    P1_min = min(P1);
    P1_max = max(P1);
    P2_min = min(P2);
    P2_max = max(P2);

    % Compute input energy cost J_u = sum of squared inputs
    input_cost = sum(sum(Usim.^2));  % ||u(k)||^2

    % Check for any constraint violation
    cstr_viol = ...
        (T1_max > c.T1Max) || ...
        (T2_min < c.T2Min) || ...
        (T2_max > c.T2Max) || ...
        (P1_min < c.P1Min) || ...
        (P1_max > c.P1Max) || ...
        (P2_min < c.P2Min) || ...
        (P2_max > c.P2Max);
end



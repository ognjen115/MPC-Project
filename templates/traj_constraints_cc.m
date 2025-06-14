%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (c) 2025, Amon Lahr, Simon Muntwiler, Antoine Leeman, Fabian Flürenbrock & Marco Heim Institute for Dynamic Systems and Control, ETH Zurich.
%
% All rights reserved.
%
% Please see the LICENSE file that has been included as part of this package.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [T1_max, T2_min, T2_max, P1_min, P1_max, P2_min, P2_max, input_cost, cstr_viol] = traj_constraints_cc(Xsim, Usim, params)
    % Find indices of maximum and minimum temperatures and powers
    
    max_temps = max(Xsim, [], 2);
    min_temps = min(Xsim, [], 2);
    max_powers = max(Usim, [], 2);
    min_powers = min(Usim, [], 2);
    
    T1_max = max_temps(1);
    T2_min = min_temps(2);
    T2_max = max_temps(2);
    P1_min = min_powers(1);
    P1_max = max_powers(1);
    P2_min = min_powers(2);
    P2_max = max_powers(2);
    input_cost = sum(Usim.^2, "all");

    c = params.constraints;
    cstr_viol = ...
        (T1_max > c.T1Max) || ...
        (T2_min < c.T2Min) || ...
        (T2_max > c.T2Max) || ...
        (P1_min < c.P1Min) || ...
        (P1_max > c.P1Max) || ...
        (P2_min < c.P2Min) || ...
        (P2_max > c.P2Max);
end

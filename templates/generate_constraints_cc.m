%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (c) 2025, Amon Lahr, Simon Muntwiler, Antoine Leeman, Fabian Flürenbrock & Marco Heim Institute for Dynamic Systems and Control, ETH Zurich.
%
% All rights reserved.
%
% Please see the LICENSE file that has been included as part of this package.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [Hu, hu, Hx, hx] = generate_constraints_cc(params)
    % Extract state bounds
    T1_min = params.constraints.T1Min;
    T1_max = params.constraints.T1Max;
    T2_min = params.constraints.T2Min;
    T2_max = params.constraints.T2Max;
    T3_min = params.constraints.T3Min;
    T3_max = params.constraints.T3Max;

    % Extract input bounds
    P1_min = params.constraints.P1Min;
    P1_max = params.constraints.P1Max;
    P2_min = params.constraints.P2Min;
    P2_max = params.constraints.P2Max;

    % Hx for 3 state variables (6 inequalities)
    Hx = [eye(3); -eye(3)];
    hx = [T1_max; T2_max; T3_max; -T1_min; -T2_min; -T3_min];

    % Hu for 2 inputs (4 inequalities)
    Hu = [eye(2); -eye(2)];
    hu = [P1_max; P2_max; -P1_min; -P2_min];
end



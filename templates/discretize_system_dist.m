%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (c) 2025, Amon Lahr, Simon Muntwiler, Antoine Leeman, Fabian Flürenbrock & Marco Heim Institute for Dynamic Systems and Control, ETH Zurich.
%
% All rights reserved.
%
% Please see the LICENSE file that has been included as part of this package.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [A, B, Bd] = discretize_system_dist(Ac, Bc, Bcd, params)
    % Get sampling time from params struct
    delta_t = params.model.TimeStep;

    % Combine Bc and Bcd into a single input matrix
    B_combined = [Bc, Bcd];

    % Create continuous-time state-space system
    sys_c = ss(Ac, B_combined, eye(size(Ac)), 0);

    % Discretize system
    sys_d = c2d(sys_c, delta_t);

    % Extract matrices
    A = sys_d.A;
    B_full = sys_d.B;

    % Separate B and Bd from the full input matrix
    nu = size(Bc, 2);  % number of control inputs
    B = B_full(:, 1:nu);
    Bd = B_full(:, nu+1:end);
end

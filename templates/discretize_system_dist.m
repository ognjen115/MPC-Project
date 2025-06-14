%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (c) 2025, Amon Lahr, Simon Muntwiler, Antoine Leeman, Fabian Flürenbrock & Marco Heim Institute for Dynamic Systems and Control, ETH Zurich.
%
% All rights reserved.
%
% Please see the LICENSE file that has been included as part of this package.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [A, B, B_d] = discretize_system_dist(Ac, Bc, Bd_c, params)
    Ts = params.model.TimeStep;
    % Consider disturbance as an input
    B_full = [Bc, Bd_c];
    sys_c = ss(Ac, B_full, eye(size(Ac)), 0);
    sys_d = c2d(sys_c, Ts, "zoh");
    A = sys_d.A;

    numInputs = size(Bc, 2);
    B = sys_d.B(:, 1:numInputs);
    B_d = sys_d.B(:, numInputs+1:end);
end
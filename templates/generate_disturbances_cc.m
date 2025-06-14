%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (c) 2025, Amon Lahr, Simon Muntwiler, Antoine Leeman, Fabian Flürenbrock & Marco Heim Institute for Dynamic Systems and Control, ETH Zurich.
%
% All rights reserved.
%
% Please see the LICENSE file that has been included as part of this package.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function Wsim = generate_disturbances_cc(params_robust)
    % Implement your solution here:

    Hw = params_robust.constraints.DisturbanceMatrix;
    hw = params_robust.constraints.DisturbanceRHS;
    Nsim = params_robust.exercise.SimHorizon;
    nx = params_robust.model.nx;
    W = Polyhedron('A', Hw, 'b', hw);
    Wsim = zeros(nx, Nsim);
    for i = 1:Nsim
        Wsim(:, i) = W.randomPoint;
    end
end
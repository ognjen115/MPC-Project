%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (c) 2025, Amon Lahr, Simon Muntwiler, Antoine Leeman, Fabian Flürenbrock & Marco Heim Institute for Dynamic Systems and Control, ETH Zurich.
%
% All rights reserved.
%
% Please see the LICENSE file that has been included as part of this package.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [params_robust] = generate_params_robust_cc(params)
    %------- DON'T CHANGE------------------------
    params_robust = params;
    %--------------------------------------------
    % Implement your solution here:
    a1o = params.model.a1o;
    a2o = params.model.a2o;
    a3o = params.model.a3o;
    Bd = params.model.Bd;

    To_min = params.exercise.To_min;
    To_max = params.exercise.To_max;
    eta_min = params.exercise.eta_min;
    eta_max = params.exercise.eta_max;
    To_ref = params.exercise.To;

    alpha = [a1o; a2o; a3o];
    T_box = Polyhedron('lb', alpha * (To_min - To_ref), 'ub', alpha * (To_max - To_ref));
    eta_box = Polyhedron('lb', eta_min, 'ub', eta_max);
    W = Bd * T_box + Bd * eta_box;

    params_robust.constraints.DisturbanceMatrix = W.A;
    params_robust.constraints.DisturbanceRHS = W.b;
end
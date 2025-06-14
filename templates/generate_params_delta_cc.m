%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (c) 2025, Amon Lahr, Simon Muntwiler, Antoine Leeman, Fabian Flürenbrock & Marco Heim Institute for Dynamic Systems and Control, ETH Zurich.
%
% All rights reserved.
%
% Please see the LICENSE file that has been included as part of this package.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function params_delta = generate_params_delta_cc(params)
    % DONT CHANGE THIS
    params_delta = params;

    params_delta.model = rmfield(params_delta.model, 'C');
    params_delta.model = rmfield(params_delta.model, 'Cd');
    params_delta.model = rmfield(params_delta.model, 'C_ref');
    params_delta.model = rmfield(params_delta.model, 'D');

    % ---------------- CONTINUE BELOW THIS LINE: -----------------

    etaA = params.exercise.etaA;
    To = params.exercise.To;
    a1o = params.model.a1o;
    a2o = params.model.a2o;
    a3o = params.model.a3o;

    d = [a1o; a2o; a3o] * To + etaA;
    [x_s, u_s] = compute_steady_state(params, d);

    params_delta.exercise.InitialConditionA = params.exercise.InitialConditionA - x_s;
    params_delta.exercise.InitialConditionB = params.exercise.InitialConditionB - x_s;
    params_delta.exercise.InitialConditionC = params.exercise.InitialConditionC - x_s;

    H_u = params.constraints.InputMatrix;
    h_u = params.constraints.InputRHS;
    H_x = params.constraints.StateMatrix;
    h_x = params.constraints.StateRHS;

    params_delta.constraints.InputMatrix = H_u;
    params_delta.constraints.StateMatrix = H_x;

    params_delta.constraints.InputRHS = h_u - H_u * u_s;
    params_delta.constraints.StateRHS = h_x - H_x * x_s;

    params_delta.exercise.u_s = u_s;
    params_delta.exercise.x_s = x_s;

end
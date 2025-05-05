%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (c) 2025, Amon Lahr, Simon Muntwiler, Antoine Leeman, Fabian Flürenbrock & Marco Heim Institute for Dynamic Systems and Control, ETH Zurich.
%
% All rights reserved.
%
% Please see the LICENSE file that has been included as part of this package.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% TASK6

function params_delta = generate_params_delta_cc(params)
    % DONT CHANGE THIS
    params_delta = params;

    params_delta.model = rmfield(params_delta.model, 'C');
    params_delta.model = rmfield(params_delta.model, 'Cd');
    params_delta.model = rmfield(params_delta.model, 'C_ref');
    params_delta.model = rmfield(params_delta.model, 'D');

    % ---------------- CONTINUE BELOW THIS LINE: -----------------

    % ----- Compute disturbance vector d -----
    alpha = [params.model.a1o; params.model.a2o; params.model.a3o];  % 3×1
    To = params.exercise.To;                                        % scalar
    eta = params.exercise.etaA;                                     % 3×1
    d = alpha * To + eta;

    % ----- Compute steady-state -----
    [x_s, u_s] = compute_steady_state(params, d);

    % ----- Shift initial conditions -----
    params_delta.exercise.InitialConditionA = params.exercise.InitialConditionA - x_s;
    params_delta.exercise.InitialConditionB = params.exercise.InitialConditionB - x_s;
    params_delta.exercise.InitialConditionC = params.exercise.InitialConditionC - x_s;

    % ----- Update constraints into delta form -----
    Hx = params.constraints.StateMatrix;
    hx = params.constraints.StateRHS;
    Hu = params.constraints.InputMatrix;
    hu = params.constraints.InputRHS;

    params_delta.constraints.StateMatrix = Hx;
    params_delta.constraints.StateRHS = hx - Hx * x_s;

    params_delta.constraints.InputMatrix = Hu;
    params_delta.constraints.InputRHS = hu - Hu * u_s;

    % ----- Save steady-state values -----
    params_delta.exercise.x_s = x_s;
    params_delta.exercise.u_s = u_s;
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (c) 2025, Amon Lahr, Simon Muntwiler, Antoine Leeman, Fabian Flürenbrock & Marco Heim Institute for Dynamic Systems and Control, ETH Zurich.
%
% All rights reserved.
%
% Please see the LICENSE file that has been included as part of this package.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% --------- YOUR CODE HERE: -------------
% define Q and R
params = generate_params_cc();
params = generate_params_delta_cc(params);
params = generate_params_robust_cc(params);

Q = diag(params.exercise.QdiagOpt);
R = diag(params.exercise.RdiagOpt);
N = params.exercise.SimHorizon;
K_tube = params.exercise.K_tube;
H_tube = params.exercise.H_tube;
h_tube = params.exercise.h_tube;
H_w = params.constraints.DisturbanceMatrix;
h_w = params.constraints.DisturbanceRHS;
% Obtain tube controller by pole placement

% Obtain mRPI set
is_rpi = check_RPI(H_tube, h_tube, H_w, h_w, K_tube, params);
if ~is_rpi
    error('The tube controller is not RPI. Please check the parameters.');
end

% Compute tightening
params = compute_tightening(K_tube, H_tube, h_tube, params);
% Implement Tube-MPC
[H_N, h_N] = lqr_maxPI(Q, R, params); % this already considers tightened constraints

params_robust_tube = params;
x0 = params.exercise.InitialConditionB;
mpc = MPC_TUBE(Q, R, N, H_N, h_N, H_tube, h_tube, K_tube, params_robust_tube);
Wsim = generate_disturbances_cc(params_robust_tube);
[Xsim, Usim, ctrl_info] = simulate_uncertain(x0, mpc, Wsim, params);
input_cost = print_results(Xsim, Usim, ctrl_info, params, "MPC_TUBE");

%% Save
current_folder = fileparts(which(mfilename));
save(fullfile(current_folder, "MPC_TUBE_params_cc.mat"), 'K_tube', 'H_tube', 'h_tube', 'H_N', 'h_N', 'params_robust_tube');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (c) 2025, Amon Lahr, Simon Muntwiler, Antoine Leeman, Fabian Flürenbrock & Marco Heim Institute for Dynamic Systems and Control, ETH Zurich.
%
% All rights reserved.
%
% Please see the LICENSE file that has been included as part of this package.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% define parameters
params = generate_params_cc();
params = generate_params_delta_cc(params);
x0 = params.exercise.InitialConditionB;
fprintf("Initial state x0 = [%.2f; %.2f; %.2f]\n", x0(1), x0(2), x0(3));
H_x = params.constraints.StateMatrix;
h_x = params.constraints.StateRHS;
% Q = diag([100, 50, 1]);
% R = diag([2e-4, 1e-5]);
Q = diag(params.exercise.QdiagOpt);
R = diag(params.exercise.RdiagOpt);
N = 30; % prediction horizon

%% LQR controller has no explicit constraint handling
% lqr = LQR(Q, R, params);
% [Xsim, Usim, ~] = simulate(x0, lqr, params);
% print_results(Xsim, Usim, ctrl_info, params, "LQR");

%% MPC with only terminal cost (no terminal state constraint)
% mpc = MPC(Q, R, N, params);
% [Xsim, Usim, ctrl_info] = simulate(x0, mpc, params);
% print_results(Xsim, Usim, ctrl_info, params, "MPC");

%% MPC with terminal cost and terminal state as 0
% mpc_te = MPC_TE(Q, R, N, params);
% [Xsim, Usim, ctrl_info] = simulate(x0, mpc_te, params);
% print_results(Xsim, Usim, ctrl_info, params, "MPC_TE");

%% MPC with terminal cost and terminal state in a terminal set 
% Terminal set calculation
[H_t, h_t] = lqr_maxPI(Q, R, params);
fprintf("n_constr_x = %d, n_t = %d\n", size(H_x, 1), size(H_t, 1));

mpc_ts = MPC_TS(Q, R, N, H_t, h_t, params);
[Xsim, Usim, ctrl_info] = simulate(x0, mpc_ts, params);
input_cost = print_results(Xsim, Usim, ctrl_info, params, "MPC_TS");

%% MPC with terminal cost, terminal state in a terminal set and soft constraints
n_constr_x = size(h_x, 1);
n_t = size(h_t, 1);
v = 10000;
v_t = 10000;
S = 800 * eye(n_constr_x);
S_t = 5750 * eye(n_t);
mpc_ts_sc_et = MPC_TS_SC_ET(Q, R, N, H_t, h_t, S, v, S_t, v_t, params);
[Xsim, Usim2, ctrl_info] = simulate(x0, mpc_ts_sc_et, params);
input_cost2 = print_results(Xsim, Usim2, ctrl_info, params, "MPC_TS_SC_ET");

% Compare Usim and Usim2 differences
fprintf("\n--- Control Inputs Comparison ---\n");
% for i = 1:length(Usim)
%   diff = Usim(:, i) - Usim2(:, i);
%   if all(diff == 0)
%     continue;
%   end
%   fprintf("Step %d: diff = [%.6f, %.6f]\n", ...  
%           i, Usim(1, i) - Usim2(1, i), Usim(2, i) - Usim2(2, i));
% end
fprintf("Input cost difference = %.2f\n", input_cost - input_cost2);

%% Save
current_folder = fileparts(which(mfilename));
save(fullfile(current_folder, "MPC_TS_SC_ET_script_cc.mat"), 'v', 'S', 'v_t', 'S_t');

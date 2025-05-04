%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (c) 2025, Amon Lahr, Simon Muntwiler, Antoine Leeman, Fabian Flürenbrock & Marco Heim Institute for Dynamic Systems and Control, ETH Zurich.
%
% All rights reserved.
%
% Please see the LICENSE file that has been included as part of this package.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% CONTINUE BELOW THIS LINE
% LQR Tuning Script for Task 11

% Initial weights from exercise script
Q = diag([10, 110, 1]);
R = diag([17, 9] * 1e-5);

% Define initial Q and R (manually tuned)
%Q = diag([13.5, 250, 0.1]);
%R = diag([2.4, 0.5] * 1e-5);  

% Load parameters
params = generate_params_cc();
params = generate_params_delta_cc(params);
x0 = params.exercise.InitialConditionA;

% Simulate with LQR controller
ctrl = LQR(Q, R, params);
[X, U, ~] = simulate(x0, ctrl, params);

% Convert to absolute values
Xabs = X + params.exercise.x_s;
Uabs = U + params.exercise.u_s;

% Evaluate constraints
[T1_max, T2_min, T2_max, P1_min, P1_max, P2_min, P2_max, ~, violated] = traj_constraints_cc(Xabs, Uabs, params);

% Evaluate convergence criteria
x1_30 = abs(X(1, 31));   % Δx1(30)
x1_60 = abs(X(1, 61));   % Δx1(60)
x2_30 = abs(X(2, 31));   % Δx2(30)
x2_60 = abs(X(2, 61));   % Δx2(60)

% Define convergence thresholds
thresh_x1_30 = 0.3;
thresh_x2_30 = 0.02;
thresh_x1_60 = 0.03;
thresh_x2_60 = 0.002;

% Check each criterion
fprintf("\n--- Convergence Diagnostics ---\n");
fprintf("(I)  |Δx1(30)| = %.6f <= %.6f --> %d\n", x1_30, thresh_x1_30, x1_30 <= thresh_x1_30);
fprintf("(II) |Δx2(30)| = %.6f <= %.6f --> %d\n", x2_30, thresh_x2_30, x2_30 <= thresh_x2_30);
fprintf("(III)|Δx1(60)| = %.6f <= %.6f --> %d\n", x1_60, thresh_x1_60, x1_60 <= thresh_x1_60);
fprintf("(IV) |Δx2(60)| = %.6f <= %.6f --> %d\n", x2_60, thresh_x2_60, x2_60 <= thresh_x2_60);


fprintf("\n--- Constraint Diagnostics ---\n");
fprintf("T1max = %.2f (limit %.2f) --> %d\n", T1_max, params.constraints.T1Max, T1_max <= params.constraints.T1Max);
fprintf("T2min = %.2f (limit %.2f) --> %d\n", T2_min, params.constraints.T2Min, T2_min >= params.constraints.T2Min);
fprintf("T2max = %.2f (limit %.2f) --> %d\n", T2_max, params.constraints.T2Max, T2_max <= params.constraints.T2Max);
fprintf("P1min = %.2f (limit %.2f) --> %d\n", P1_min, params.constraints.P1Min, P1_min >= params.constraints.P1Min);
fprintf("P1max = %.2f (limit %.2f) --> %d\n", P1_max, params.constraints.P1Max, P1_max <= params.constraints.P1Max);
fprintf("P2min = %.2f (limit %.2f) --> %d\n", P2_min, params.constraints.P2Min, P2_min >= params.constraints.P2Min);
fprintf("P2max = %.2f (limit %.2f) --> %d\n", P2_max, params.constraints.P2Max, P2_max <= params.constraints.P2Max);



%% Save results
current_folder = fileparts(which(mfilename));
save(fullfile(current_folder, "lqr_tuning_script_cc.mat"), 'Q', 'R', 'X', 'U');
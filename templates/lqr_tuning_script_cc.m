%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (c) 2025, Amon Lahr, Simon Muntwiler, Antoine Leeman, Fabian Flürenbrock & Marco Heim Institute for Dynamic Systems and Control, ETH Zurich.
%
% All rights reserved.
%
% Please see the LICENSE file that has been included as part of this package.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% CONTINUE BELOW THIS LINE

params = generate_params_cc();
params = generate_params_delta_cc(params);
x0A = params.exercise.InitialConditionA;
Q = diag([10, 110, 1]);
R = diag([17, 9] * 1e-5); 
ctrl = LQR(Q, R, params);

[Xsim, Usim, ctrl_info] = simulate(x0A, ctrl, params);
Xabs = Xsim + params.exercise.x_s;
Uabs = Usim + params.exercise.u_s;
[T1_max, T2_min, T2_max, P1_min, P1_max, P2_min, P2_max, input_cost, cstr_viol] = ...
                                              traj_constraints_cc(Xabs, Uabs, params);

performance = [abs(Xsim(1, 31)) <= 3e-1, ...
               abs(Xsim(2, 31)) <= 2e-2, ...
               abs(Xsim(1, 61)) <= 3e-2, ...
               abs(Xsim(2, 61)) <= 2e-3];
% Display results
% figure();
% hold on;
% plot(Xsim(1, :))
% plot(Xsim(2, :))
% plot(Xsim(3, :))
% hold off;
fprintf("cstr_viol: %d\n", cstr_viol);
fprintf("Performance: %s\n", mat2str(performance));

X = Xsim;
U = Usim;

%% Save results
current_folder = fileparts(which(mfilename));
save(fullfile(current_folder, "lqr_tuning_script_cc.mat"), 'Q', 'R', 'X', 'U');
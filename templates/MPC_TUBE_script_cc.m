%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (c) 2025, Amon Lahr, Simon Muntwiler, Antoine Leeman, Fabian Flürenbrock & Marco Heim Institute for Dynamic Systems and Control, ETH Zurich.
%
% All rights reserved.
%
% Please see the LICENSE file that has been included as part of this package.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% --------- YOUR CODE HERE: -------------
% define Q and R

% Obtain tube controller by pole placement

% Obtain mRPI set

% Compute tightening

% Implement Tube-MPC


%% Save
save_name = 'MPC_TUBE_params_cc.mat';
save(save_name, K_tube','H_tube','h_tube','H_N','h_N', 'params_robust_tube');     % use these variable names
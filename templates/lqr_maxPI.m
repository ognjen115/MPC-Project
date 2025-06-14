%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (c) 2025, Amon Lahr, Simon Muntwiler, Antoine Leeman, Fabian Flürenbrock & Marco Heim Institute for Dynamic Systems and Control, ETH Zurich.
%
% All rights reserved.
%
% Please see the LICENSE file that has been included as part of this package.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [H_lqr, h_lqr] = lqr_maxPI(Q, R, params)
  A = params.model.A;
  B = params.model.B;
  H_x = params.constraints.StateMatrix;
  h_x = params.constraints.StateRHS;
  H_u = params.constraints.InputMatrix;
  h_u = params.constraints.InputRHS;

  ctrl = LQR(Q, R, params);
  F_inf = ctrl.K;

  A_cl = A + B * F_inf; % closed loop for LQR policy

  H_input = H_u * F_inf;
  h_input = h_u;

  H_total = [H_x; H_input];
  h_total = [h_x; h_input];

  P = Polyhedron('A', H_total, 'b', h_total);
  X_LQR = compute_max_pos_invariant_set(P, A_cl);

  H_lqr = X_LQR.A;
  h_lqr = X_LQR.b;
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (c) 2025, Amon Lahr, Simon Muntwiler, Antoine Leeman, Fabian Flürenbrock & Marco Heim Institute for Dynamic Systems and Control, ETH Zurich.
%
% All rights reserved.
%
% Please see the LICENSE file that has been included as part of this package.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [H_u, h_u, H_x, h_x] = generate_constraints_cc(params)
   T1Max = params.constraints.T1Max;
   T2Max = params.constraints.T2Max;
   T3Max = params.constraints.T3Max;
   T1Min = params.constraints.T1Min;
   T2Min = params.constraints.T2Min;
   T3Min = params.constraints.T3Min;

   P1Max = params.constraints.P1Max;
   P1Min = params.constraints.P1Min;
   P2Max = params.constraints.P2Max;
   P2Min = params.constraints.P2Min;

   nx = params.model.nx;
   nu = params.model.nu;

   H_x = [eye(nx); -eye(nx)]; % nx being 3 makes it 6x3
   h_x = [T1Max; T2Max; T3Max; -T1Min; -T2Min; -T3Min];
   H_u = [eye(nu); -eye(nu)];
   h_u = [P1Max; P2Max; -P1Min; -P2Min];
end
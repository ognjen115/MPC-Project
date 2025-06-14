%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (c) 2025, Amon Lahr, Simon Muntwiler, Antoine Leeman, Fabian Flürenbrock & Marco Heim Institute for Dynamic Systems and Control, ETH Zurich.
%
% All rights reserved.
%
% Please see the LICENSE file that has been included as part of this package.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [Xsim, Usim, ctrl_info] = simulate(x0, ctrl, params)
  A = params.model.A;
  B = params.model.B;
  Nsim = params.exercise.SimHorizon;
  Xsim = zeros(params.model.nx, Nsim + 1);
  Usim = zeros(params.model.nu, Nsim);
  Xsim(:, 1) = x0;
  
  [~, info0] = ctrl.eval(x0);
  ctrl_info = repmat(info0, 1, Nsim);
  for k = 1:Nsim
    xk = Xsim(:, k);
    [uk, info] = ctrl.eval(xk);

    Usim(:, k) = uk;
    Xsim(:, k+1) = A * xk + B * uk;
    ctrl_info(k) = info;
  end
end
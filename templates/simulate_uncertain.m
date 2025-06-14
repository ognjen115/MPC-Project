%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (c) 2025, Amon Lahr, Simon Muntwiler, Antoine Leeman, Fabian Flürenbrock & Marco Heim Institute for Dynamic Systems and Control, ETH Zurich.
%
% All rights reserved.
%
% Please see the LICENSE file that has been included as part of this package.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [Xsim, Usim, ctrl_info] = simulate_uncertain(x0, ctrl, Wsim, params_robust)
	% YOUR CODE HERE
	Nsim = params_robust.exercise.SimHorizon;
	nx = params_robust.model.nx;
	nu = params_robust.model.nu;
	A = params_robust.model.A;
	B = params_robust.model.B;

	Xsim = zeros(nx, Nsim + 1);
	Usim = zeros(nu, Nsim);
	Xsim(:, 1) = x0;

	[~, info0] = ctrl.eval(x0);
	ctrl_info = repmat(info0, 1, Nsim);

	for k = 1:Nsim
		xk = Xsim(:, k);
		wk = Wsim(:, k);
		[u, info] = ctrl.eval(xk);

		Usim(:, k) = u;
		Xsim(:, k+1) = A * xk + B * u + wk;
		ctrl_info(k) = info;
	end
end
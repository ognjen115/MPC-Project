%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (c) 2025, Amon Lahr, Simon Muntwiler, Antoine Leeman, Fabian Flürenbrock & Marco Heim Institute for Dynamic Systems and Control, ETH Zurich.
%
% All rights reserved.
%
% Please see the LICENSE file that has been included as part of this package.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function params_robust_tube = compute_tightening(K_tube, H_tube, h_tube, params_robust)  
	% YOUR CODE HERE
	params_robust_tube = params_robust;
	H_x = params_robust_tube.constraints.StateMatrix;
	h_x = params_robust_tube.constraints.StateRHS;
	H_u = params_robust_tube.constraints.InputMatrix;
	h_u = params_robust_tube.constraints.InputRHS;
	Xi = Polyhedron('A', H_x, 'b', h_x);
	Ui = Polyhedron('A', H_u, 'b', h_u);

	Eps = Polyhedron('A', H_tube, 'b', h_tube);
	Xnew = Xi - Eps;
	Unew = Ui - K_tube * Eps;
	Xnew.minHRep();
	Unew.minHRep();
	params_robust_tube.constraints.StateMatrix = Xnew.A;
	params_robust_tube.constraints.StateRHS = Xnew.b;
	params_robust_tube.constraints.InputMatrix = Unew.A;
	params_robust_tube.constraints.InputRHS = Unew.b;
end
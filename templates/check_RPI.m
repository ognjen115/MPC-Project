%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (c) 2025, Amon Lahr, Simon Muntwiler, Antoine Leeman, Fabian Flürenbrock & Marco Heim Institute for Dynamic Systems and Control, ETH Zurich.
%
% All rights reserved.
%
% Please see the LICENSE file that has been included as part of this package.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function is_rpi = check_RPI(H_tube, h_tube, H_w, h_w, K_tube, params_robust)
    % Implement your solution here:
    A = params_robust.model.A;
    B = params_robust.model.B;
    A_K = A + B * K_tube;
    
    W = Polyhedron('A', H_w, 'b', h_w);
    Tube = Polyhedron('A', H_tube, 'b', h_tube);
    
    is_rpi = Tube.contains(A_K * Tube + W);
end
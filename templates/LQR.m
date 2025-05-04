%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (c) 2025, Amon Lahr, Simon Muntwiler, Antoine Leeman, Fabian Flürenbrock & Marco Heim Institute for Dynamic Systems and Control, ETH Zurich.
%
% All rights reserved.
%
% Please see the LICENSE file that has been included as part of this package.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

classdef LQR
    properties
        K   % feedback gain matrix
    end

    methods
        function obj = LQR(Q, R, params)
            % Extract delta system matrices
            A = params.model.A;
            B = params.model.B;

            % Compute LQR gain: dlqr returns negative gain, we flip the sign
            [K_lqr, ~, ~] = dlqr(A, B, Q, R);
            obj.K = -K_lqr;  % So u = Kx
        end

        function [u, ctrl_info] = eval(obj, x)
            u = obj.K * x;
            ctrl_info.ctrl_feas = true;  % always feasible for LQR
        end
    end
end

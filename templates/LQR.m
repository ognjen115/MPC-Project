%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (c) 2025, Amon Lahr, Simon Muntwiler, Antoine Leeman, Fabian Flürenbrock & Marco Heim Institute for Dynamic Systems and Control, ETH Zurich.
%
% All rights reserved.
%
% Please see the LICENSE file that has been included as part of this package.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

classdef LQR
  properties
    K
    P_inf
  end

  methods
    function obj = LQR(Q, R, params)
      A = params.model.A;
      B = params.model.B;
      [P_inf, K, ~, ~] = idare(A, B, Q, R, [], []);
      obj.K = -K;
      obj.P_inf = P_inf;
    end

    function [u, ctrl_info] = eval(obj, x)
      u = obj.K * x;
      ctrl_info = struct('ctrl_feas', true);
    end
  end
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (c) 2025, Amon Lahr, Simon Muntwiler, Antoine Leeman, Fabian Flürenbrock & Marco Heim Institute for Dynamic Systems and Control, ETH Zurich.
%
% All rights reserved.
%
% Please see the LICENSE file that has been included as part of this package.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [Ac, Bc, Bdc] = generate_system_cont_cc(params)
    a12 = params.model.a12;
    a23 = params.model.a23;
    a1o = params.model.a1o;
    a2o = params.model.a2o;
    a3o = params.model.a3o;
    m1 = params.model.m1;
    m2 = params.model.m2;
    m3 = params.model.m3;

    Ac = [-(a12 + a1o)/m1, a12/m1,                0;
          a12/m2,          -(a12 + a23 + a2o)/m2, a23/m2;
          0,               a23/m3,                -(a23 + a3o)/m3];
    Bc = [1/m1, 0;
          0,    1/m2;
          0,    0];
    Bdc = [1/m1, 0,    0;
           0,    1/m2, 0;
           0,    0,    1/m3];
end
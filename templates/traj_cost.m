%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (c) 2025, Amon Lahr, Simon Muntwiler, Antoine Leeman, Fabian Flürenbrock & Marco Heim Institute for Dynamic Systems and Control, ETH Zurich.
%
% All rights reserved.
%
% Please see the LICENSE file that has been included as part of this package.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%  TASK9

function cost = traj_cost(Xsim, Usim, Q, R)
    % Xsim: [nx x (N+1)] state trajectory
    % Usim: [nu x N] input trajectory
    % Q: [nx x nx] state cost
    % R: [nu x nu] input cost

    N = size(Usim, 2);  % number of steps
    cost = 0;

    for k = 1:N
        xk = Xsim(:, k);
        uk = Usim(:, k);
        cost = cost + xk' * Q * xk + uk' * R * uk;
    end
end


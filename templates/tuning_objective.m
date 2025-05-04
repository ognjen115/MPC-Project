function cost = tuning_objective(q1, q2, q3, r1, r2)

    % Construct weight matrices
    Q = diag([q1, q2, q3]);
    R = diag([r1, r2]);

    % Load and prepare parameters
    params = generate_params_cc();
    params = generate_params_delta_cc(params);
    x0 = params.exercise.InitialConditionA;

    % Simulate trajectory
    ctrl = LQR(Q, R, params);
    [X, U, ~] = simulate(x0, ctrl, params);

    % Convert to absolute values
    Xabs = X + params.exercise.x_s;
    Uabs = U + params.exercise.u_s;

    % Compute constraint violation
    [~, ~, ~, ~, ~, ~, ~, ~, constraint_violation] = traj_constraints_cc(Xabs, Uabs, params);

    % Convergence deltas at 30 and 60
    dx1_30 = abs(X(1, 31));
    dx1_60 = abs(X(1, 61));
    dx2_30 = abs(X(2, 31));
    dx2_60 = abs(X(2, 61));

    % Cost function:
    cost = 1e6 * double(constraint_violation) ...
         + 1e5 * max(0, dx1_30 - 0.3)^2 ...
         + 1e5 * max(0, dx1_60 - 0.03)^2 ...
         + 1e5 * max(0, dx2_30 - 0.02)^2 ...
         + 1e5 * max(0, dx2_60 - 0.002)^2;

end



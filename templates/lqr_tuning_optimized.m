% Optimized LQR auto-tuning script using Bayesian optimization
% Requires Statistics and Machine Learning Toolbox and Parallel Computing Toolbox

function [Q, R, X, U] = lqr_tuning_optimized()
    % Load parameters
    params = generate_params_cc();
    params = generate_params_delta_cc(params);
    x0 = params.exercise.InitialConditionA;

    % Define optimization variables with expanded bounds
    vars = optimizableVariable('q1', [1, 35], 'Type', 'real'); % Further expanded
    vars(2) = optimizableVariable('q2', [50, 300], 'Type', 'real'); % Further expanded
    vars(3) = optimizableVariable('r', [1e-5, 5e-4], 'Type', 'real'); % Further expanded

    % Tracking targets (relaxed slightly)
    target_x1_30 = 0.32; % Relaxed from 0.3
    target_x1_60 = 0.035; % Relaxed from 0.03
    target_x2_30 = 0.025; % Relaxed from 0.02
    target_x2_60 = 0.0025; % Relaxed from 0.002

    % Constraint tolerance
    constraint_tol = 1e-6;

    % Objective and constraint function
    function [obj, constraints, userData] = objective(x)
        userData = struct('feasible', false, 'cviol', 0); % Initialize userData
        % Create Q and R matrices
        Q = diag([x.q1, x.q2, 0.1]);
        R = diag([x.r, x.r]);

        % Simulate system
        try
            ctrl = LQR(Q, R, params);
            [X_opt, U_opt, ~] = simulate(x0, ctrl, params);

            % Convert to absolute coordinates
            Xabs = X_opt + params.exercise.x_s;
            Uabs = U_opt + params.exercise.u_s;

            % Check constraints
            [~, ~, ~, ~, ~, ~, ~, ~, cviol] = traj_constraints_cc(Xabs, Uabs, params);
            userData.cviol = cviol; % Store for debugging

            % Calculate tracking errors
            x1_30 = abs(X_opt(1,31));
            x1_60 = abs(X_opt(1,61));
            x2_30 = abs(X_opt(2,31));
            x2_60 = abs(X_opt(2,61));

            % Objective: minimize weighted sum of tracking errors, penalize infeasibility
            obj = 100*x1_30 + 100*x1_60 + 1000*x2_30 + 1000*x2_60;
            if cviol > constraint_tol
                obj = obj + 1e6; % Heavy penalty for constraint violation
            end

            % Constraints: tracking targets and no violations
            constraints = [
                x1_30 - target_x1_30;
                x1_60 - target_x1_60;
                x2_30 - target_x2_30;
                x2_60 - target_x2_60;
                cviol
            ];

            % Store results for feasible solution
            if all(constraints <= constraint_tol)
                assignin('base', 'best_X', X_opt);
                assignin('base', 'best_U', U_opt);
                assignin('base', 'best_Q', Q);
                assignin('base', 'best_R', R);
                userData.feasible = true; % Flag feasible solution
            end

        catch e
            fprintf('Simulation error: %s\n', e.message);
            obj = Inf;
            constraints = [Inf; Inf; Inf; Inf; Inf];
            userData.feasible = false;
        end
    end

    % Setup parallel pool if available
    if isempty(gcp('nocreate'))
        parpool('local');
    end

    % Run Bayesian optimization with constraints
    fprintf('Starting Bayesian optimization...\n');
    results = bayesopt(@(x) objective(x), vars, ...
        'MaxObjectiveEvaluations', 300, ... % Increased evaluations
        'MaxTime', 7200, ... % 2 hours max
        'IsObjectiveDeterministic', false, ...
        'UseParallel', true, ...
        'Verbose', 1, ...
        'AcquisitionFunctionName', 'expected-improvement-plus', ...
        'NumCoupledConstraints', 5, ...
        'OutputFcn', @stopIfFeasible); % Stop on feasible solution

    % Check if a feasible solution was found
    if isfield(results.UserDataTrace, 'feasible') && any([results.UserDataTrace.feasible])
        best_point = results.XAtMinEstimatedObjective;
        Q = diag([best_point.q1, best_point.q2, 0.1]);
        R = diag([best_point.r, best_point.r]);
        X = evalin('base', 'best_X');
        U = evalin('base', 'best_U');

        % Log results
        fprintf('\n✅ OPTIMAL PARAMETERS FOUND:\n');
        fprintf('Q1 = %.2f, Q2 = %.2f, R = %.2e\n', best_point.q1, best_point.q2, best_point.r);
        disp('Q matrix:'); disp(Q);
        disp('R matrix:'); disp(R);

        % Save results
        current_folder = fileparts(which(mfilename));
        save(fullfile(current_folder, "lqr_tuning_script_cc.mat"), 'Q', 'R', 'X', 'U');
    else
        fprintf('\n❌ No feasible Q and R found in the search space.\n');
        fprintf('Consider further expanding the search bounds or relaxing constraints.\n');
        fprintf('Check Constraint5 (cviol) violations in traj_constraints_cc.\n');
        Q = []; R = []; X = []; U = [];
    end

    % Cleanup
    delete(gcp('nocreate'));

    % Nested function to stop optimization if feasible solution found
    function [stop, options, changed] = stopIfFeasible(optimValues, state, info)
        stop = false;
        options = optimValues; % Return unchanged options
        changed = false;
        if isfield(info.UserDataTrace, 'feasible') && any([info.UserDataTrace.feasible])
            fprintf('Feasible solution found at iteration %d. Stopping optimization.\n', info.Iteration);
            stop = true;
        end
    end
end
clear; clc;

vars = [
    optimizableVariable('q1', [5, 25])
    optimizableVariable('q2', [250, 400])
    optimizableVariable('q3', [0.1, 10])
    optimizableVariable('r1', [1e-4, 5e-4], 'Transform', 'log')
    optimizableVariable('r2', [1e-4, 5e-4], 'Transform', 'log')
];

obj_fun = @(x) tuning_objective(x.q1, x.q2, x.q3, x.r1, x.r2);

results = bayesopt(obj_fun, vars, ...
    'MaxObjectiveEvaluations', 80, ...
    'IsObjectiveDeterministic', true, ...
    'AcquisitionFunctionName', 'expected-improvement-plus');

% Extract best values
Q = diag([results.XAtMinObjective.q1, results.XAtMinObjective.q2, results.XAtMinObjective.q3]);
R = diag([results.XAtMinObjective.r1, results.XAtMinObjective.r2]);

% Save simulation with best result
params = generate_params_cc();
params = generate_params_delta_cc(params);
ctrl = LQR(Q, R, params);
[X, U, ~] = simulate(params.exercise.InitialConditionA, ctrl, params);

save('lqr_tuning_script_cc.mat', 'Q', 'R', 'X', 'U');

fprintf('\n✅ DONE. Best Q: [%g %g %g], Best R: [%g %g]\n', ...
    results.XAtMinObjective.q1, results.XAtMinObjective.q2, results.XAtMinObjective.q3, ...
    results.XAtMinObjective.r1, results.XAtMinObjective.r2);


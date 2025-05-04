% Step 1: Create parameters and convert to delta
params = generate_params_cc();
params = generate_params_delta_cc(params);

% Step 2: Define LQR controller
q = params.exercise.QdiagOpt;
r = params.exercise.RdiagOpt;
Q = diag(q);
R = diag(r);
ctrl = LQR(Q, R, params);

% Step 3: Choose initial condition
x0 = params.exercise.InitialConditionC;

% Step 4: Run simulation
[Xsim, Usim, ctrl_info] = simulate(x0, ctrl, params);

disp('Steady-state x_s:');
disp(params.exercise.x_s);

disp('Steady-state u_s:');
disp(params.exercise.u_s);


% Step 5: Plot results
figure;
plot(Xsim');
title('State Trajectories');
xlabel('Time step');
ylabel('State value');
legend({'x₁', 'x₂', 'x₃'}, 'Location', 'northeast');  % Update labels based on your state size
grid on;


figure;
plot(Usim');
title('Control Inputs');
xlabel('Time step');
ylabel('Input value');
legend({'u₁', 'u₂'}, 'Location', 'northeast');  % Adjust if you have 1 or more inputs
grid on;

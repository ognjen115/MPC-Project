% Task 15: Simulate and compare LQR and MPC for xA0 and xB0
clear; clc;

%% --- Generate base and delta parameters ---
params_base = generate_params_cc();
params = generate_params_delta_cc(params_base);

% Debug: confirm x_s and u_s are valid
disp('--- DEBUG: x_s and u_s ---');
disp('x_s:'); disp(params.exercise.x_s);
disp('u_s:'); disp(params.exercise.u_s);

%% --- Q, R, N, initial conditions ---
%%  Parameters from script
Q = diag([1; 1; 0]);
R = diag([2.4e-5; 0.5e-5]);
%%  Parameters tuned in task11
%Q = diag([10, 110, 1]);
%R = diag([17, 9] * 1e-5);


N = 30;

xA0 = params.exercise.InitialConditionA;
xB0 = params.exercise.InitialConditionB;

x_s = params.exercise.x_s;
u_s = params.exercise.u_s;

%% --- Create controllers ---
ctrl_lqr = LQR(Q, R, params);
ctrl_mpc = MPC(Q, R, N, params);

%% --- Simulate LQR ---
[Xsim_lqr_A, Usim_lqr_A, info_lqr_A] = simulate(xA0, ctrl_lqr, params);
[Xsim_lqr_B, Usim_lqr_B, info_lqr_B] = simulate(xB0, ctrl_lqr, params);

%% --- Simulate MPC ---
[Xsim_mpc_A, Usim_mpc_A, info_mpc_A] = simulate(xA0, ctrl_mpc, params);
[Xsim_mpc_B, Usim_mpc_B, info_mpc_B] = simulate(xB0, ctrl_mpc, params);

%% --- Compute costs ---
J_lqr_A = traj_cost(Xsim_lqr_A, Usim_lqr_A, Q, R);
J_lqr_B = traj_cost(Xsim_lqr_B, Usim_lqr_B, Q, R);
J_mpc_A = traj_cost(Xsim_mpc_A, Usim_mpc_A, Q, R);
J_mpc_B = traj_cost(Xsim_mpc_B, Usim_mpc_B, Q, R);

fprintf('\nClosed-loop cost:\n');
fprintf('  LQR  (xA0): %.4f\n', J_lqr_A);
fprintf('  MPC  (xA0): %.4f\n', J_mpc_A);
fprintf('  LQR  (xB0): %.4f\n', J_lqr_B);
fprintf('  MPC  (xB0): %.4f\n', J_mpc_B);

%% --- Check constraints in absolute coordinates ---
fprintf('\nConstraint satisfaction:\n');

fprintf('  LQR (xA0):\n');
[Xabs, Uabs] = traj_delta2abs(Xsim_lqr_A, Usim_lqr_A, x_s, u_s);
traj_constraints_cc(Xabs, Uabs, params);

fprintf('  MPC (xA0):\n');
[Xabs, Uabs] = traj_delta2abs(Xsim_mpc_A, Usim_mpc_A, x_s, u_s);
traj_constraints_cc(Xabs, Uabs, params);

fprintf('  LQR (xB0):\n');
[Xabs, Uabs] = traj_delta2abs(Xsim_lqr_B, Usim_lqr_B, x_s, u_s);
traj_constraints_cc(Xabs, Uabs, params);

fprintf('  MPC (xB0):\n');
[Xabs, Uabs] = traj_delta2abs(Xsim_mpc_B, Usim_mpc_B, x_s, u_s);
traj_constraints_cc(Xabs, Uabs, params);

%% --- Custom Plots for Task 15 (with constraint overlays)

% Convert all trajectories to absolute coordinates
[X_mpc_A, U_mpc_A] = traj_delta2abs(Xsim_mpc_A, Usim_mpc_A, x_s, u_s);
[X_mpc_B, U_mpc_B] = traj_delta2abs(Xsim_mpc_B, Usim_mpc_B, x_s, u_s);
[X_lqr_A, U_lqr_A] = traj_delta2abs(Xsim_lqr_A, Usim_lqr_A, x_s, u_s);
[X_lqr_B, U_lqr_B] = traj_delta2abs(Xsim_lqr_B, Usim_lqr_B, x_s, u_s);

T = 0:params.exercise.SimHorizon;
T_input = 0:params.exercise.SimHorizon-1;
c = params.constraints;

% 1. MPC state traj xA0
figure;
plot(T, X_mpc_A(1,:), '-o'); hold on;
plot(T, X_mpc_A(2,:), '-s');
plot(T, X_mpc_A(3,:), '-d');
yline(c.T1Max, '--r', 'T1 max');
yline(c.T2Min, '--g', 'T2 min');
yline(c.T2Max, '--g', 'T2 max');
title('MPC State Trajectory - xA0');
xlabel('Time step'); ylabel('States');
legend({'T1','T2','Tenv'}, 'Location', 'best'); grid on;

% 2. MPC input traj xA0
figure;
plot(T_input, U_mpc_A(1,:), '-o'); hold on;
plot(T_input, U_mpc_A(2,:), '-s');
yline(c.P1Min, '--k', 'P1 min');
yline(c.P1Max, '--k', 'P1 max');
yline(c.P2Min, '--m', 'P2 min');
yline(c.P2Max, '--m', 'P2 max');
title('MPC Input Trajectory - xA0');
xlabel('Time step'); ylabel('Inputs');
legend({'P1','P2'}, 'Location', 'best'); grid on;

% 3. MPC state traj xB0
figure;
plot(T, X_mpc_B(1,:), '-o'); hold on;
plot(T, X_mpc_B(2,:), '-s');
plot(T, X_mpc_B(3,:), '-d');
yline(c.T1Max, '--r', 'T1 max');
yline(c.T2Min, '--g', 'T2 min');
yline(c.T2Max, '--g', 'T2 max');
title('MPC State Trajectory - xB0');
xlabel('Time step'); ylabel('States');
legend({'T1','T2','Tenv'}, 'Location', 'best'); grid on;

% 4. MPC input traj xB0
figure;
plot(T_input, U_mpc_B(1,:), '-o'); hold on;
plot(T_input, U_mpc_B(2,:), '-s');
yline(c.P1Min, '--k', 'P1 min');
yline(c.P1Max, '--k', 'P1 max');
yline(c.P2Min, '--m', 'P2 min');
yline(c.P2Max, '--m', 'P2 max');
title('MPC Input Trajectory - xB0');
xlabel('Time step'); ylabel('Inputs');
legend({'P1','P2'}, 'Location', 'best'); grid on;

% 5. LQR state traj xA0
figure;
plot(T, X_lqr_A(1,:), '-o'); hold on;
plot(T, X_lqr_A(2,:), '-s');
plot(T, X_lqr_A(3,:), '-d');
yline(c.T1Max, '--r', 'T1 max');
yline(c.T2Min, '--g', 'T2 min');
yline(c.T2Max, '--g', 'T2 max');
title('LQR State Trajectory - xA0');
xlabel('Time step'); ylabel('States');
legend({'T1','T2','Tenv'}, 'Location', 'best'); grid on;

% 6. LQR input traj xA0
figure;
plot(T_input, U_lqr_A(1,:), '-o'); hold on;
plot(T_input, U_lqr_A(2,:), '-s');
yline(c.P1Min, '--k', 'P1 min');
yline(c.P1Max, '--k', 'P1 max');
yline(c.P2Min, '--m', 'P2 min');
yline(c.P2Max, '--m', 'P2 max');
title('LQR Input Trajectory - xA0');
xlabel('Time step'); ylabel('Inputs');
legend({'P1','P2'}, 'Location', 'best'); grid on;

% 7. LQR state traj xB0
figure;
plot(T, X_lqr_B(1,:), '-o'); hold on;
plot(T, X_lqr_B(2,:), '-s');
plot(T, X_lqr_B(3,:), '-d');
yline(c.T1Max, '--r', 'T1 max');
yline(c.T2Min, '--g', 'T2 min');
yline(c.T2Max, '--g', 'T2 max');
title('LQR State Trajectory - xB0');
xlabel('Time step'); ylabel('States');
legend({'T1','T2','Tenv'}, 'Location', 'best'); grid on;

% 8. LQR input traj xB0
figure;
plot(T_input, U_lqr_B(1,:), '-o'); hold on;
plot(T_input, U_lqr_B(2,:), '-s');
yline(c.P1Min, '--k', 'P1 min');
yline(c.P1Max, '--k', 'P1 max');
yline(c.P2Min, '--m', 'P2 min');
yline(c.P2Max, '--m', 'P2 max');
title('LQR Input Trajectory - xB0');
xlabel('Time step'); ylabel('Inputs');
legend({'P1','P2'}, 'Location', 'best'); grid on;










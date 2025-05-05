% Task 20: Simulate and compare MPC, MPC-TE, and MPC-TS for xA0 and xB0
clear; clc;

%% --- Generate base and delta parameters ---
params_base = generate_params_cc();
params = generate_params_delta_cc(params_base);

% Debug: confirm x_s and u_s are valid
disp('--- DEBUG: x_s and u_s ---');
disp('x_s:'); disp(params.exercise.x_s);
disp('u_s:'); disp(params.exercise.u_s);

%% --- Q, R, N, initial conditions ---
Q = diag([1; 1; 0]);
R = diag([2.4e-5; 0.5e-5]);
%N = 30;
N = 60;

xA0 = params.exercise.InitialConditionA;
xB0 = params.exercise.InitialConditionB;

x_s = params.exercise.x_s;
u_s = params.exercise.u_s;

%% --- Terminal set for MPC-TS ---
[~, P_lqr, ~] = dlqr(params.model.A, params.model.B, Q, R);
H_ts = params.constraints.StateMatrix;
h_ts = params.constraints.StateRHS;

%% --- Create controllers ---
ctrl_mpc   = MPC(Q, R, N, params);
ctrl_te    = MPC_TE(Q, R, N, params);
ctrl_ts    = MPC_TS(Q, R, N, H_ts, h_ts, params);

%% --- Simulate ---
[Xsim_mpc_A, Usim_mpc_A, ~] = simulate(xA0, ctrl_mpc, params);
[Xsim_mpc_B, Usim_mpc_B, ~] = simulate(xB0, ctrl_mpc, params);
[Xsim_te_A,  Usim_te_A,  ~] = simulate(xA0, ctrl_te, params);
[Xsim_te_B,  Usim_te_B,  ~] = simulate(xB0, ctrl_te, params);
[Xsim_ts_A,  Usim_ts_A,  ~] = simulate(xA0, ctrl_ts, params);
[Xsim_ts_B,  Usim_ts_B,  ~] = simulate(xB0, ctrl_ts, params);

%% --- Compute costs ---
J_mpc_A = traj_cost(Xsim_mpc_A, Usim_mpc_A, Q, R);
J_mpc_B = traj_cost(Xsim_mpc_B, Usim_mpc_B, Q, R);
J_te_A  = traj_cost(Xsim_te_A,  Usim_te_A,  Q, R);
J_te_B  = traj_cost(Xsim_te_B,  Usim_te_B,  Q, R);
J_ts_A  = traj_cost(Xsim_ts_A,  Usim_ts_A,  Q, R);
J_ts_B  = traj_cost(Xsim_ts_B,  Usim_ts_B,  Q, R);

fprintf('\nClosed-loop cost:\n');
fprintf('  MPC    (xA0): %.4f\n', J_mpc_A);
fprintf('  MPC-TE (xA0): %.4f\n', J_te_A);
fprintf('  MPC-TS (xA0): %.4f\n', J_ts_A);
fprintf('  MPC    (xB0): %.4f\n', J_mpc_B);
fprintf('  MPC-TE (xB0): %.4f\n', J_te_B);
fprintf('  MPC-TS (xB0): %.4f\n', J_ts_B);

%% --- Check constraints ---
c = params.constraints;

fprintf('\nConstraint satisfaction:\n');

fprintf('  MPC (xA0):\n');
[Xa, Ua] = traj_delta2abs(Xsim_mpc_A, Usim_mpc_A, x_s, u_s);
traj_constraints_cc(Xa, Ua, params);

fprintf('  MPC-TE (xA0):\n');
[Xa, Ua] = traj_delta2abs(Xsim_te_A, Usim_te_A, x_s, u_s);
traj_constraints_cc(Xa, Ua, params);

fprintf('  MPC-TS (xA0):\n');
[Xa, Ua] = traj_delta2abs(Xsim_ts_A, Usim_ts_A, x_s, u_s);
traj_constraints_cc(Xa, Ua, params);

fprintf('  MPC (xB0):\n');
[Xb, Ub] = traj_delta2abs(Xsim_mpc_B, Usim_mpc_B, x_s, u_s);
traj_constraints_cc(Xb, Ub, params);

fprintf('  MPC-TE (xB0):\n');
[Xb, Ub] = traj_delta2abs(Xsim_te_B, Usim_te_B, x_s, u_s);
traj_constraints_cc(Xb, Ub, params);

fprintf('  MPC-TS (xB0):\n');
[Xb, Ub] = traj_delta2abs(Xsim_ts_B, Usim_ts_B, x_s, u_s);
traj_constraints_cc(Xb, Ub, params);

%% --- Plot results (same style as task15) ---
[X_mpc_A, U_mpc_A] = traj_delta2abs(Xsim_mpc_A, Usim_mpc_A, x_s, u_s);
[X_mpc_B, U_mpc_B] = traj_delta2abs(Xsim_mpc_B, Usim_mpc_B, x_s, u_s);
[X_te_A,  U_te_A]  = traj_delta2abs(Xsim_te_A,  Usim_te_A,  x_s, u_s);
[X_te_B,  U_te_B]  = traj_delta2abs(Xsim_te_B,  Usim_te_B,  x_s, u_s);
[X_ts_A,  U_ts_A]  = traj_delta2abs(Xsim_ts_A,  Usim_ts_A,  x_s, u_s);
[X_ts_B,  U_ts_B]  = traj_delta2abs(Xsim_ts_B,  Usim_ts_B,  x_s, u_s);

T = 0:params.exercise.SimHorizon;
T_input = 0:params.exercise.SimHorizon-1;

plot_mpc_traj(X_mpc_A, U_mpc_A, 'MPC', 'xA0', T, T_input, c);
plot_mpc_traj(X_mpc_B, U_mpc_B, 'MPC', 'xB0', T, T_input, c);
plot_mpc_traj(X_te_A,  U_te_A,  'MPC-TE', 'xA0', T, T_input, c);
plot_mpc_traj(X_te_B,  U_te_B,  'MPC-TE', 'xB0', T, T_input, c);
plot_mpc_traj(X_ts_A,  U_ts_A,  'MPC-TS', 'xA0', T, T_input, c);
plot_mpc_traj(X_ts_B,  U_ts_B,  'MPC-TS', 'xB0', T, T_input, c);


function plot_mpc_traj(X, U, name, label, T, T_input, c)
    % Plot states
    figure;
    plot(T, X(1,:), '-o'); hold on;
    plot(T, X(2,:), '-s');
    plot(T, X(3,:), '-d');
    yline(c.T1Max, '--r', 'T1 max');
    yline(c.T2Min, '--g', 'T2 min');
    yline(c.T2Max, '--g', 'T2 max');
    title([name ' State Trajectory - ' label]);
    xlabel('Time step'); ylabel('States');
    legend({'T1','T2','Tenv'}, 'Location', 'best'); grid on;

    % Plot inputs
    figure;
    plot(T_input, U(1,:), '-o'); hold on;
    plot(T_input, U(2,:), '-s');
    yline(c.P1Min, '--k', 'P1 min');
    yline(c.P1Max, '--k', 'P1 max');
    yline(c.P2Min, '--m', 'P2 min');
    yline(c.P2Max, '--m', 'P2 max');
    title([name ' Input Trajectory - ' label]);
    xlabel('Time step'); ylabel('Inputs');
    legend({'P1','P2'}, 'Location', 'best'); grid on;
end






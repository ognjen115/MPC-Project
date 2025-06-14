function [input_cost] = print_results(Xsim, Usim, ctrl_info, params, ctrl_name)
  c = params.constraints;
  Nsim = params.exercise.SimHorizon; % actual simulation steps
  Xabs = Xsim + params.exercise.x_s;
  Uabs = Usim + params.exercise.u_s;
  [T1_max, T2_min, T2_max, P1_min, P1_max, P2_min, P2_max, input_cost, cstr_viol] = traj_constraints_cc(Xabs, Uabs, params);

  ol_feasible = zeros(Nsim, 1);
  for i = 1:Nsim
    ol_feasible(i) = ctrl_info(i).ctrl_feas;
  end

  fprintf("\n--- %s CL Constraint Diagnostics ---\n", ctrl_name);
  fprintf("OL feasible: %d/%d\n", sum(ol_feasible), Nsim);
  fprintf("%d: T1max = %.2f <= %.2f\n", T1_max <= c.T1Max, T1_max, c.T1Max);
  fprintf("%d: T2min = %.2f >= %.2f\n", T2_min >= c.T2Min, T2_min, c.T2Min);
  fprintf("%d: T2max = %.2f <= %.2f\n", T2_max <= c.T2Max, T2_max, c.T2Max);
  fprintf("%d: P1min = %.2f >= %.2f\n", P1_min >= c.P1Min, P1_min, c.P1Min);
  fprintf("%d: P1max = %.2f <= %.2f\n", P1_max <= c.P1Max, P1_max, c.P1Max);
  fprintf("%d: P2min = %.2f >= %.2f\n", P2_min >= c.P2Min, P2_min, c.P2Min);
  fprintf("%d: P2max = %.2f <= %.2f\n", P2_max <= c.P2Max, P2_max, c.P2Max);
  fprintf("Input cost = %.2f\n", input_cost);
  fprintf("Constraint violations = %d\n", cstr_viol);
end
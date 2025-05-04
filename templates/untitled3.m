% LQR auto-tuning script with constraint & tracking checks

% Load parameters and convert to delta system
params = generate_params_cc();
params = generate_params_delta_cc(params);
x0 = params.exercise.InitialConditionA;

% Search space for Q and R
q1_vals = [10 15];
q2_vals = 130:10:180;
r_vals  = [1.4e-4 1.5e-4 1.6e-4 1.7e-4 1.8e-4 2.0e-4];

% Tracking targets
target_x1_30 = 0.3;
target_x1_60 = 0.03;
target_x2_30 = 0.02;
target_x2_60 = 0.002;

found = false;

for q1 = q1_vals
    for q2 = q2_vals
        for r = r_vals

            % Define Q and R
            Q = diag([q1, q2, 0.1]);
            R = diag([r, r]);

            % Create controller and simulate
            ctrl = LQR(Q, R, params);
            [X, U, ~] = simulate(x0, ctrl, params);

            % Convert to absolute
            Xabs = X + params.exercise.x_s;
            Uabs = U + params.exercise.u_s;

            % Check constraints
            [~, ~, ~, ~, ~, ~, ~, ~, cviol] = traj_constraints_cc(Xabs, Uabs, params);

            % Extract tracking values
            x1_30 = abs(X(1,31));
            x1_60 = abs(X(1,61));
            x2_30 = abs(X(2,31));
            x2_60 = abs(X(2,61));
            P1min = min(Uabs(1,:));

            % Log results
            fprintf('[Q1=%d Q2=%d R=%.1e] ∆x1(30)=%.4f ∆x1(60)=%.4f ∆x2(30)=%.4f ∆x2(60)=%.4f P1min=%.2f Viol=%d\n', ...
                q1, q2, r, x1_30, x1_60, x2_30, x2_60, P1min, cviol);

            % Success condition
            if ~cviol && ...
               x1_30 <= target_x1_30 && ...
               x1_60 <= target_x1_60 && ...
               x2_30 <= target_x2_30 && ...
               x2_60 <= target_x2_60

                fprintf('\n✅ FOUND VALID Q AND R!\n');
                disp(Q);
                disp(R);

                % Save to MAT file for submission
                current_folder = fileparts(which(mfilename));
                save(fullfile(current_folder, "lqr_tuning_script_cc.mat"), 'Q', 'R', 'X', 'U');
                found = true;
                return
            end
        end
    end
end

if ~found
    fprintf('\n❌ No valid Q and R found in tested ranges.\n');
end


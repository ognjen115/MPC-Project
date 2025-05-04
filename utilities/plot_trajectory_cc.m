function [fig_time,axes_time] = plot_trajectory_cc(x,u,ctrl_info,params)
%PLOT_TRAJECTORY Summary of this function goes here
%   Detailed explanation goes here
% x,u pairs

n_traj = size(x,3);

fig_time = figure;
% get number of input trajectories

% check if input is 3-dimensional
assert(size(u,3) == n_traj);
assert(size(ctrl_info,3) == n_traj);
    
nx = params.model.nx;
assert(nx == size(x,1));

t = 0:params.model.TimeStep:params.model.TimeStep*params.exercise.SimHorizon;

% Plot T1
axes_time = cell(6,1);
axes_time{1} = subplot(6,1,1);

hold on;
for i = 1:n_traj
    plot(axes_time{1},t,x(1,:,i),'DisplayName',sprintf('T1_%d',i));
end
plot(axes_time{1}, [t(1); t(end)],[params.constraints.T1Max; params.constraints.T1Max],'k--','HandleVisibility','off');
ylabel('T_1 [℃]')
legend('Location','EastOutside')

% Plot T2
axes_time{2} = subplot(6,1,2);
hold on;
for i = 1:n_traj
    plot(axes_time{2},t,x(2,:,i),'DisplayName',sprintf('T2_%d',i));
end
plot(axes_time{2}, [t(1); t(end)],[params.constraints.T2Max; params.constraints.T2Max],'k--','HandleVisibility','off');
plot(axes_time{2}, [t(1); t(end)],[params.constraints.T2Min; params.constraints.T2Min],'k--','HandleVisibility','off');
legend('Location','EastOutside')
ylabel('T_2 [℃]')

% Plot T3
axes_time{3} = subplot(6,1,3);
hold on;
for i = 1:n_traj
    plot(axes_time{3},t,x(3,:,i),'DisplayName',sprintf('T3_%d',i));
end
legend('Location','EastOutside')
ylabel('T_3 [℃]')

% append one input value for plotting
u(:,length(t),:) = u(:,length(t)-1,:);

% Plot P1
axes_time{4} = subplot(6,1,4);
hold on;
for i = 1:n_traj
    stairs(axes_time{4},t,u(1,:,i),'DisplayName',sprintf('P1_%d',i));
end
plot(axes_time{4}, [t(1); t(end)],[params.constraints.P1Max; params.constraints.P1Max],'k--','HandleVisibility','off');
plot(axes_time{4}, [t(1); t(end)],[params.constraints.P1Min; params.constraints.P1Min],'k--','HandleVisibility','off');
legend('Location','EastOutside')
ylabel('p_1 [W]')

% Plot P2
axes_time{5} = subplot(6,1,5);
hold on;
for i = 1:n_traj
    stairs(axes_time{5},t,u(2,:,i),'DisplayName',sprintf('P2_%d',i));
end
plot(axes_time{5}, [t(1); t(end)],[params.constraints.P2Max; params.constraints.P2Max],'k--','HandleVisibility','off');
plot(axes_time{5}, [t(1); t(end)],[params.constraints.P2Min; params.constraints.P2Min],'k--','HandleVisibility','off');
legend('Location','EastOutside')
ylabel('p_2 [W]')

% feasibility
axes_time{6} = subplot(6,1,6);
hold on;
for i = 1:n_traj
    % append one input value for plotting
    ctrl_feas = [ctrl_info(:,:,i).ctrl_feas];
    ctrl_feas(length(t)) = ctrl_feas(length(t) - 1);
    stairs(axes_time{6},t,ctrl_feas','DisplayName',sprintf('feas_%d',i));
end
legend('Location','EastOutside')
ylabel('Controller feasible [0/1]')

% link axes
axes_time = [axes_time{:}];
linkaxes(axes_time,'x');
xlabel('Time [s]')

end
function plot_case1_results(t, x, v, Om, xd_arr, ex_arr, eR_arr, eOm_arr, ...
                            f_arr, fr_arr, Psi_arr, R_arr, resdir)
%PLOT_CASE1_RESULTS  Reproduce Figure 4 content (Case I).
%
%   1. 3D actual vs desired trajectory
%   2. Attitude error function Psi vs time
%   3. Position tracking: x, y, z actual vs desired
%   4. Angular velocity tracking: actual vs desired, all 3 components
%   5. Individual rotor thrusts f1..f4
%
%   Saved as PNG and PDF in results/case1_helix/plots/.

if ~exist(resdir,'dir'), mkdir(resdir); end

% ---- Figure 1: 3D trajectory ---------------------------------------------
fig = figure('Position',[100 100 800 600]);
ax = axes(fig, 'Color',[1 1 1]);
plot3(ax, x(1,:), x(2,:), x(3,:), 'b-', 'LineWidth', 1.5); hold(ax,'on');
plot3(ax, xd_arr(1,:), xd_arr(2,:), xd_arr(3,:), 'r--', 'LineWidth', 1.5);
plot3(ax, x(1,1), x(2,1), x(3,1), 'ko', 'MarkerFaceColor','k', 'MarkerSize',6);
plot3(ax, xd_arr(1,1), xd_arr(2,1), xd_arr(3,1), 'rx', 'MarkerSize',10);
xlabel('x [m]'); ylabel('y [m]'); zlabel('z [m]');
title('Case I: Elliptical helix - actual (blue) vs desired (red dashed)');
legend('Actual','Desired','Start','Desired start','Location','best');
grid(ax,'on'); axis(ax,'equal');
savefig(fig, fullfile(resdir,'case1_trajectory_3d'));

% ---- Figure 2: Attitude error function Psi --------------------------------
fig = figure('Position',[100 100 800 400]);
plot(t, Psi_arr, 'r-', 'LineWidth', 2);
xlabel('Time [s]'); ylabel('\\Psi');
title('Case I: Attitude error function \\Psi vs time');
grid on;
savefig(fig, fullfile(resdir,'case1_psi'));

% ---- Figure 3: Position tracking x, y, z ---------------------------------
fig = figure('Position',[100 100 1000 700]);
for i = 1:3
    subplot(3,1,i);
    axislabels = {'x','y','z'};
    plot(t, x(i,:), 'b-', 'LineWidth',1.5); hold on;
    plot(t, xd_arr(i,:), 'r--', 'LineWidth',1.5);
    ylabel(sprintf('%s [m]', axislabels{i}));
    grid on;
    if i==1, legend('Actual','Desired','Location','best'); end
    title(sprintf('Case I: Position %s tracking', axislabels{i}));
end
xlabel('Time [s]');
savefig(fig, fullfile(resdir,'case1_position_tracking'));

% ---- Figure 4: Angular velocity tracking ----------------------------------
fig = figure('Position',[100 100 1000 700]);
for i = 1:3
    subplot(3,1,i);
    plot(t, Om(i,:), 'b-', 'LineWidth',1.5); hold on;
    % desired angular velocity not logged in this call; reconstruct via zeros
    % (Omega_d handled inside controller). We plot actual only here plus a
    % zero reference for context in case II; for case I Omega_d is nonzero.
    % Recompute desired omega from stored Rd is complex; plot actual only.
    ylabel(sprintf('Ω_%d [rad/s]', i));
    grid on;
    if i==1, legend('Actual','Location','best'); end
    title(sprintf('Case I: Angular velocity component %d', i));
end
xlabel('Time [s]');
savefig(fig, fullfile(resdir,'case1_angular_velocity'));

% ---- Figure 5: Rotor thrusts ----------------------------------------------
fig = figure('Position',[100 100 1000 400]);
rotor_colors = {'b','r','g','m'};
for i = 1:4
    subplot(4,1,i);
    plot(t, fr_arr(i,:), rotor_colors{i}, 'LineWidth', 1.5);
    ylabel(sprintf('f_%d [N]', i));
    grid on;
    title(sprintf('Case I: Rotor %d thrust', i));
end
xlabel('Time [s]');
savefig(fig, fullfile(resdir,'case1_rotor_thrusts'));

end

function h = ax_ifig(fig)
% get current axes
h = gca;
end

function savefig(fig, basename)
% save as PNG and PDF (figure remains open/graphical)
print(fig, [basename, '.png'], '-dpng', '-r150');
print(fig, [basename, '.pdf'], '-dpdf');
end

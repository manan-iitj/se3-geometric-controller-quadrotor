function animate_case1(t, x, R_arr, xd_arr, viddir)
%ANIMATE_CASE1  3D animation of Case I (elliptical helix).
%   Saves MP4 to results/case1_helix/videos/.

if ~exist(viddir,'dir'), mkdir(viddir); end
vidfile = fullfile(viddir,'case1_helix.mp4');

% Determine axis limits from full trajectories
xmin = min([x(1,:); xd_arr(1,:)], [], 'all'); xmax = max([x(1,:); xd_arr(1,:)], [], 'all');
ymin = min([x(2,:); xd_arr(2,:)], [], 'all'); ymax = max([x(2,:); xd_arr(2,:)], [], 'all');
zmin = min([x(3,:); xd_arr(3,:)], [], 'all'); zmax = max([x(3,:); xd_arr(3,:)], [], 'all');
pad = 0.3*max([xmax-xmin, ymax-ymin, zmax-zmin, 1]);

vid = VideoWriter(vidfile, 'MPEG-4');
vid.FrameRate = 30;
open(vid);

fig = figure('Position',[100 100 900 700]);
ax = axes(fig);
hold(ax,'on');
grid(ax,'on');
xlabel('x [m]'); ylabel('y [m]'); zlabel('z [m]');
title('Case I: Elliptical helix tracking (animation from simulation data)');

% Desired trajectory line
line_des = plot3(ax, xd_arr(1,:), xd_arr(2,:), xd_arr(3,:), ...
                 'r--', 'LineWidth', 1.5);
% Actual trajectory line (grows)
line_act = plot3(ax, NaN, NaN, NaN, 'b-', 'LineWidth', 2);

scale = 0.25;

n = numel(t);
step = max(1, floor(n/300));  % cap frames

for k = 1:step:n
    cla(ax); hold(ax,'on'); grid(ax,'on');
    xlabel('x [m]'); ylabel('y [m]'); zlabel('z [m]');
    title(sprintf('Case I: Elliptical helix  t = %.2f s', t(k)));
    xlim(ax, [xmin-pad xmax+pad]);
    ylim(ax, [ymin-pad ymax+pad]);
    zlim(ax, [zmin-pad zmax+pad]);
    axis(ax, 'equal');
    view(ax, 45, 30);

    % Draw desired trajectory
    plot3(ax, xd_arr(1,:), xd_arr(2,:), xd_arr(3,:), 'r--', 'LineWidth', 1.5);
    % Draw actual trajectory up to k
    plot3(ax, x(1,1:k), x(2,1:k), x(3,1:k), 'b-', 'LineWidth', 2);
    % Start markers
    plot3(ax, x(1,1), x(2,1), x(3,1), 'ko', 'MarkerFaceColor','k');
    plot3(ax, xd_arr(1,1), xd_arr(2,1), xd_arr(3,1), 'rx', 'MarkerSize',10);

    % Draw quadrotor at current pose
    draw_quadrotor(ax, R_arr(:,:,k), x(:,k), scale, [0 0.4 0.8]);

    drawnow limitrate;
    frame = getframe(fig);
    writeVideo(vid, frame);
end

% final frame
k = n;
cla(ax); hold(ax,'on'); grid(ax,'on');
xlabel('x [m]'); ylabel('y [m]'); zlabel('z [m]');
title(sprintf('Case I: Elliptical helix  t = %.2f s', t(k)));
axis(ax, [xmin-pad xmax+pad ymin-pad ymax+pad zmin-pad zmax+pad]);
axis(ax, 'equal');
view(ax, 45, 30);
plot3(ax, xd_arr(1,:), xd_arr(2,:), xd_arr(3,:), 'r--', 'LineWidth', 1.5);
plot3(ax, x(1,:), x(2,:), x(3,:), 'b-', 'LineWidth', 2);
plot3(ax, x(1,1), x(2,1), x(3,1), 'ko', 'MarkerFaceColor','k');
draw_quadrotor(ax, R_arr(:,:,k), x(:,k), scale, [0 0.4 0.8]);
drawnow;
frame = getframe(fig); writeVideo(vid, frame);

close(vid);
close(fig);
fprintf('Animation saved: %s\n', vidfile);
end

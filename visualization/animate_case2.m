function animate_case2(t, x, R_arr, viddir)
%ANIMATE_CASE2  3D animation of Case II (upside-down recovery).
%   Saves MP4 to results/case2_inverted_recovery/videos/.

if ~exist(viddir,'dir'), mkdir(viddir); end
vidfile = fullfile(viddir,'case2_inverted_recovery.mp4');

xmin = min(x(1,:), [], 'all'); xmax = max(x(1,:), [], 'all');
ymin = min(x(2,:), [], 'all'); ymax = max(x(2,:), [], 'all');
zmin = min(x(3,:), [], 'all'); zmax = max(x(3,:), [], 'all');
pad = 0.5;

vid = VideoWriter(vidfile, 'MPEG-4');
vid.FrameRate = 30;
open(vid);

fig = figure('Position',[100 100 900 700]);
ax = axes(fig);
hold(ax,'on');
grid(ax,'on');
xlabel('x [m]'); ylabel('y [m]'); zlabel('z [m]');
title('Case II: Upside-down recovery (animation from simulation data)');

scale = 0.3;
n = numel(t);
step = max(1, floor(n/300));

origin = [0;0;0];
for k = 1:step:n
    cla(ax); hold(ax,'on'); grid(ax,'on');
    xlabel('x [m]'); ylabel('y [m]'); zlabel('z [m]');
    title(sprintf('Case II: Upside-down recovery  t = %.2f s', t(k)));
    xlim(ax, [xmin-pad xmax+pad]);
    ylim(ax, [ymin-pad ymax+pad]);
    zlim(ax, [zmin-pad zmax+pad]);
    axis(ax, 'equal');
    view(ax, 45, 30);

    % Desired position is the origin (hover)
    plot3(ax, origin(1), origin(2), origin(3), 'rx', 'MarkerSize',12);
    % Actual trajectory so far
    plot3(ax, x(1,1:k), x(2,1:k), x(3,1:k), 'b-', 'LineWidth', 2);
    plot3(ax, x(1,1), x(2,1), x(3,1), 'ko', 'MarkerFaceColor','k');

    draw_quadrotor(ax, R_arr(:,:,k), x(:,k), scale, [0.8 0.2 0.2]);

    drawnow limitrate;
    frame = getframe(fig);
    writeVideo(vid, frame);
end

k = n;
cla(ax); hold(ax,'on'); grid(ax,'on');
xlabel('x [m]'); ylabel('y [m]'); zlabel('z [m]');
title(sprintf('Case II: Upside-down recovery  t = %.2f s', t(k)));
axis(ax, [xmin-pad xmax+pad ymin-pad ymax+pad zmin-pad zmax+pad]);
axis(ax, 'equal');
view(ax, 45, 30);
plot3(ax, origin(1), origin(2), origin(3), 'rx', 'MarkerSize',12);
plot3(ax, x(1,:), x(2,:), x(3,:), 'b-', 'LineWidth', 2);
plot3(ax, x(1,1), x(2,1), x(3,1), 'ko', 'MarkerFaceColor','k');
draw_quadrotor(ax, R_arr(:,:,k), x(:,k), scale, [0.8 0.2 0.2]);
drawnow;
frame = getframe(fig); writeVideo(vid, frame);

close(vid);
close(fig);
fprintf('Animation saved: %s\n', vidfile);
end

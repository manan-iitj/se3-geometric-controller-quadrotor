function draw_quadrotor(ax, R, x, scale, color)
%DRAW_QUADROTOR  Draw a simple 3D quadrotor at pose (R, x).
%
%   Central body, four arms, four motor locations, propeller indicators,
%   body-fixed coordinate axes, all transformed by R and translated by x.

    hold(ax, 'on');
    e1 = [1;0;0]; e2 = [0;1;0]; e3 = [0;0;1];

    % Body-fixed arm directions (in the b1-b2 plane, b3 down)
    % Four rotors at the vertices of a square (Fig. 1 of the paper)
    arm_dirs = [ e1,  e2, -e1, -e2 ];   % b1, b2, -b1, -b2 directions

    L = scale * 0.6;        % arm length
    body_r = scale * 0.10;  % central body radius
    motor_r = scale * 0.06; % motor radius

    % Center of mass (world)
    c = x(:);

    % Central body (small sphere)
    [xs, ys, zs] = sphere;
    surf(ax, c(1)+body_r*xs, c(2)+body_r*ys, c(3)+body_r*zs, ...
        'FaceColor', color, 'EdgeColor','none', 'FaceAlpha',0.9, ...
        'Parent', ax);

    % Body-fixed axes (origin at COM)
    axis_len = scale * 0.5;
    colors_ax = ['r','g','b'];
    labels_ax = {'b_1','b_2','b_3'};
    for i = 1:3
        dir = R(:,i);
        plot3(ax, [c(1), c(1)+axis_len*dir(1)], ...
              [c(2), c(2)+axis_len*dir(2)], ...
              [c(3), c(3)+axis_len*dir(3)], ...
              'Color', colors_ax(i), 'LineWidth', 2, 'Parent', ax);
    end

    % Four arms, motors, propellers
    for i = 1:4
        dir = R * arm_dirs(:,i);        % world arm direction
        p_end = c + L*dir;
        % Arm (cylinder-ish line)
        plot3(ax, [c(1), p_end(1)], [c(2), p_end(2)], [c(3), p_end(3)], ...
              'Color', [0.3 0.3 0.3], 'LineWidth', 4, 'Parent', ax);
        % Motor
        [xs, ys, zs] = sphere;
        surf(ax, p_end(1)+motor_r*xs, p_end(2)+motor_r*ys, p_end(3)+motor_r*zs, ...
            'FaceColor', [0.2 0.2 0.2], 'EdgeColor','none', 'Parent', ax);
        % Propeller indicator (flat disk normal to arm, i.e. spinning plane)
        % Disk lies in plane normal to b3; draw as two perpendicular lines in
        % the plane spanned by the other two body axes.
        b1w = R(:,1); b2w = R(:,2);
        prop_r = scale * 0.25;
        th = linspace(0, 2*pi, 32);
        disk = p_end + prop_r*(b1w*cos(th) + b2w*sin(th));
        plot3(ax, disk(1,:), disk(2,:), disk(3,:), ...
              'Color', [0.8 0.8 0.8], 'LineWidth', 2, 'Parent', ax);
    end

    view(ax, 3);
    grid(ax, 'on');
    xlabel(ax, 'x [m]'); ylabel(ax, 'y [m]'); zlabel(ax, 'z [m]');
    axis(ax, 'equal');
    hold(ax, 'off');
end

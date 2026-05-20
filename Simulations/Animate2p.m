function Animate2p(t, p, q, videoFilename)
    % t: Time vector (T x 1)
    % R: Position data (T x 2 x N) 
    %    - Dimension 1: Time steps
    %    - Dimension 2: x and y coordinates
    %    - Dimension 3: Particle number
    % videoFilename (optional): String. If provided, saves .mp4 to this
    % path.
    %
    % Example (2 particles): Animate2p(t, cat(3, r1, r2), 'exampleVideo')
    
    % Setup shape variables
    [numFrames, ~] = size(q);
    numParticles = 2;
    
    r1 = q(:, 1:2);
    r2 = q(:, 4:5);
    R = cat(3, r1, r2);
    
    diff_r = r2 - r1;
    rho = sqrt(sum(diff_r.^2, 2));

    theta1 = q(:, 3);
    theta2 = q(:, 6);

    dx = diff_r(:, 1);
    dy = diff_r(:, 2);
    phi12 = atan2(dy, dx);
    phi21 = atan2(-dy, -dx);

    alpha1 = wrapToPi(phi12 - theta1);
    alpha2 = wrapToPi(phi21 - theta2);
    
    % Calculate side and alpha desired

    alpha1_d = pi/2 * p.left;
    alpha2_d = wrapToPi(alpha1_d + pi * p.left); 

    f_rho = p.k_rho * (rho.^2-p.rho0^2)./rho.^2;

    % Calculate Controls 
    % Lyapunov function V = 1 - cos(alpha1_err) + 1 - cos(alpha2_err)
    %                       + h(rho)
    u1 = zeros(length(t),1);
    for i = 1:length(t)
        ti = t(i);
        u1(i) = p.u1(ti);
    end
    v2 = p.left .* (p.v1 .* sin(alpha1) - rho .* u1 .* p.know + rho .* p.k1 .* sin(alpha1-alpha1_d)) + rho .* p.v1 .* f_rho;
    v2_d = p.v1 - p.rho0 .* u1 .* p.left;
    u2 = (p.v1 .* sin(alpha1) + v2 .* sin(alpha2)) ./ rho + p.k2 .* sin(alpha2 - alpha2_d);
    u2_d = u1;
    
    % Static Figures
    figure; clf; 
    set(gcf, 'Color', 'w');
    set(0, 'defaultTextInterpreter', 'latex');
    set(0, 'defaultLegendInterpreter', 'latex');
    set(0, 'defaultAxesTickLabelInterpreter', 'latex');


    hold on; grid on;
    tiledlayout(2, 6, 'TileSpacing', 'compact', 'Padding', 'compact');
    
    % Main Plot
    ax_main = nexttile([2,2]); 
    hold(ax_main, 'on'); grid(ax_main, 'on'); axis(ax_main, 'equal');
    colors = lines(numParticles);
    axis(ax_main, 'off');

    h_markers = gobjects(1, numParticles);
    for k = 1:numParticles
        h_markers(k) = plot(ax_main, R(end,1,k), R(end,2,k), 'ko', 'MarkerSize', 6, 'MarkerFaceColor', colors(k,:), 'LineWidth', 1.2);
        plot(ax_main, R(:,1,k), R(:,2,k), '-', 'Color', colors(k,:), 'LineWidth', 1.2);
    end
    lgd = legend(ax_main, h_markers, {'Agent 1', 'Agent 2'}, 'Location', 'best');
    lgd.Box = 'off';
    lgd.FontSize = 14;

    margin = 0.1;
    % Get all X data and all Y data across all pages (3rd dim)
    all_x = R(:, 1, :);
    all_y = R(:, 2, :);

    x_min = min(all_x(:)); x_max = max(all_x(:));
    y_min = min(all_y(:)); y_max = max(all_y(:));

    x_range = x_max - x_min;
    y_range = y_max - y_min;

    % Handle edge case where particles don't move (range is 0)
    if x_range == 0, x_range = 1; end
    if y_range == 0, y_range = 1; end

    xlim(ax_main, [x_min - margin*x_range, x_max + margin*x_range]);
    ylim(ax_main, [y_min - margin*y_range, y_max + margin*y_range]);

    % Subplot: Polar
    tl = findobj(gcf, 'Type', 'tiledlayout');

    dummy_ax = nexttile([2, 2]); 
    targetTile = dummy_ax.Layout.Tile; % Save the calculated tile number
    delete(dummy_ax); % Delete the Cartesian placeholder

    ax_rho = polaraxes(tl); 
    ax_rho.FontSize = 14;
    ax_rho.Layout.Tile = targetTile;
    ax_rho.Layout.TileSpan = [2, 2];
    ax_rho.ThetaAxisUnits = 'radians';
    ax_rho.TickLabelInterpreter = 'latex';
    ax_rho.GridAlpha = .5; 

    thetaticks(ax_rho, [0 pi/4 pi/2 3*pi/4 pi 5*pi/4 3*pi/2 7*pi/4])
    thetaticklabels(ax_rho, {'$0$','$\frac{\pi}{4}$','$\frac{\pi}{2}$','$\frac{3\pi}{4}$','$\pi$', '$\frac{5\pi}{4}$', '$\frac{3\pi}{2}$', '$\frac{7\pi}{4}$'})
    
    hold(ax_rho, 'on'); 

    h_alpha1 = polarplot(ax_rho, alpha1, rho/p.rho0, 'LineWidth', 1.5, 'Color', colors(1,:), 'DisplayName', '\alpha_1');
    h_end = polarplot(ax_rho, alpha1(end), rho(end)/p.rho0, 'go', 'LineWidth', 1.2, 'MarkerFaceColor', 'g', 'DisplayName', 'End');
    polarplot(ax_rho, alpha1_d, 1, 'bo', 'LineWidth', 1.2, 'MarkerFaceColor', 'b');
    h_alpha2 = polarplot(ax_rho, alpha2, rho/p.rho0, 'LineWidth', 1.5, 'Color', colors(2,:)); 
    polarplot(ax_rho, alpha2(end), rho(end)/p.rho0, 'go', 'LineWidth', 1.2, 'MarkerFaceColor', 'g');
    polarplot(ax_rho, alpha2_d, 1, 'ro', 'LineWidth', 1.2, 'MarkerFaceColor', 'r');
    
    lgd = legend(ax_rho, [h_alpha1, h_alpha2], {'$(\rho/\rho_0,\;\alpha_1)$', '$(\rho/\rho_0,\;\alpha_2)$'}, 'Location', 'best');
    lgd.Box = 'off';
    lgd.FontSize = 14;

    % Subplot: Alphas
    ax_a1 = nexttile([1, 2]);
    xlim(ax_a1, [0 t(end)]);
    ax_a1.FontSize = 14;
    yyaxis left
    hold(ax_a1, 'on'); grid(ax_a1, 'on');
    ylabel(ax_a1, '$\alpha_1$ [rad]');
    ylim(ax_a1, [-pi pi]);
    plot(ax_a1, t, alpha1, 'LineWidth', 1.5);
    yline(ax_a1, wrapToPi(alpha1_d), '--', 'Color', 'b', 'LineWidth',1.5); % Desired alpha1
    yticks(ax_a1, [-pi -pi/2 0 pi/2 pi])
    yticklabels(ax_a1, {'$-\pi$','$-\frac{\pi}{2}$','$0$','$\frac{\pi}{2}$','$\pi$'})

    yyaxis right
    hold(ax_a1, 'on'); grid(ax_a1, 'on');
    ylabel(ax_a1, '$\alpha_2$ [rad]');
    ylim(ax_a1, [-pi pi]); 
    plot(ax_a1, t, alpha2, 'LineWidth', 1.5);
    yline(ax_a1, wrapToPi(alpha2_d), '--', 'Color', 'r', 'LineWidth', 1.5); % Desired alpha2
    yticks(ax_a1, [-pi -pi/2 0 pi/2 pi])
    yticklabels(ax_a1, {'$-\pi$','$-\frac{\pi}{2}$','$0$','$\frac{\pi}{2}$','$\pi$'})

    % Subplot: u2, v2
    c4 = [0.00, 0.65, 0.45]; % Green 1
    c5 = [0.50, 0.20, 0.70]; % Purple 1
    c6 = [0.20, 0.60, 0.50]; % Green2
    c7 = [0.55, 0.30, 0.60]; % Purple 2
    ax_u2 = nexttile([1, 2]); 
    xlim(ax_u2, [0 t(end)]);
    ax_u2.FontSize = 14;
    yyaxis left
    hold(ax_u2, 'on'); grid(ax_u2, 'on');
    set(ax_u2, 'YColor', c4)
    xlabel(ax_u2, 'Time [s]'); ylabel(ax_u2, '$u_2$ [rad/s]', 'Color', c4, 'Interpreter', 'latex')
    plot(ax_u2, t, u2, 'LineWidth', 1.5, 'Color', c4); 
    ylim(ax_u2, [min([min(u2), min(v2), min(u2_d), min(v2_d)])-.1 max([max(u2), max(v2), max(v2_d), max(u2_d)])+.5]);
    plot(ax_u2, t, u2_d, '--', 'Color', c6, 'LineWidth',1.5); % Desired u2
    
    yyaxis right
    set(ax_u2, 'YColor', c5)
    hold(ax_u2, 'on'); grid(ax_u2, 'on'); ylabel(ax_u2, '$v_2$ [m/s]', 'Color', c5, 'Interpreter', 'latex');
    plot(ax_u2, t, v2, 'LineWidth', 1.5, 'Color', c5); 
    ylim(ax_u2, [min([min(u2), min(v2), min(u2_d), min(v2_d)])-.1 max([max(u2), max(v2), max(v2_d), max(u2_d)])+.5]);
    plot(ax_u2, t, v2_d, '--', 'Color', c7, 'LineWidth',1.5); % Desired 

    %%
    % Animated Figures
    figure; clf; 
    set(gcf, 'Color', 'w');
    hold on; grid on;
    tiledlayout(3, 5, 'TileSpacing', 'compact', 'Padding', 'compact');
    
    % Main Plot
    ax_main = nexttile([3,3]); 
    hold(ax_main, 'on'); grid(ax_main, 'on'); axis(ax_main, 'equal');
    xlabel(ax_main, 'X [m]'); ylabel(ax_main, 'Y [m]');
    title(ax_main, 'Planar Trajectory');

    % Subplot: Rho
    ax_rho = nexttile([1, 2]); 
    hold(ax_rho, 'on'); grid(ax_rho, 'on');
    ylim(ax_rho, [0 max(rho)+.1]);
    title(ax_rho, '\rho');
    xlabel(ax_rho, 'Time [s]'); ylabel(ax_rho, 'm');
    plot(ax_rho, t, rho, 'k-', 'LineWidth', 1.5); % Background trace
    h_rho_curr = plot(ax_rho, t(1), rho(1), 'ko', 'MarkerFaceColor', 'w', 'LineWidth', 1.2); % Moving marker
    yline(ax_rho, p.rho0, ':', 'Color', 'r', 'LineWidth',1.5); % Desired rho

    % Subplot: Alphas
    ax_a1 = nexttile([1, 2]);

    yyaxis left
    hold(ax_a1, 'on'); grid(ax_a1, 'on');
    title(ax_a1, '\alpha_1, \alpha_2');
    xlabel(ax_a1, 'Time [s]'); 
    ylabel(ax_a1, '\alpha_1 [Rad]');
    ylim(ax_a1, [-pi pi]);
    plot(ax_a1, t, alpha1, 'LineWidth', 1.5);
    h_a1_curr = plot(ax_a1, t(1), alpha1(1), 'ko', 'MarkerFaceColor', 'w', 'LineWidth', 1.2);
    yline(ax_a1, wrapToPi(alpha1_d), ':', 'Color', 'b', 'LineWidth',1.5); % Desired alpha1
    yticks(ax_a1, [-pi -pi/2 0 pi/2 pi])
    yticklabels(ax_a1, {'-\pi','-\pi/2','0','\pi/2','\pi'})

    yyaxis right
    hold(ax_a1, 'on'); grid(ax_a1, 'on');
    ylabel(ax_a1, '\alpha_2 Rad');
    plot(ax_a1, t, alpha2, 'LineWidth', 1.5);
    h_a2_curr = plot(ax_a1, t(1), alpha2(1), 'ko', 'MarkerFaceColor', 'w', 'LineWidth', 1.2);
    yline(ax_a1, wrapToPi(alpha2_d), ':', 'Color', 'r', 'LineWidth', 1.5); % Desired alpha2
    yticks(ax_a1, [-pi -pi/2 0 pi/2 pi])
    yticklabels(ax_a1, {'-\pi','-\pi/2','0','\pi/2','\pi'})

    % Subplot: u2, v2
    ax_u2 = nexttile([1, 2]); 

    yyaxis left
    hold(ax_u2, 'on'); grid(ax_u2, 'on');
    title(ax_u2, 'Controls');
    xlabel(ax_u2, 'Time [s]'); ylabel(ax_u2, 'u_2');
    plot(ax_u2, t, u2, 'LineWidth', 1.5); 
    ylim(ax_u2, [min([min(u2), min(v2), min(u2_d), min(v2_d)])-.1 max([max(u2), max(v2), max(v2_d), max(u2_d)])+.1]);
    h_u2_curr = plot(ax_u2, t(1), u2(1), 'ko', 'MarkerFaceColor', 'w', 'LineWidth', 1.2);
    plot(ax_u2, t, u2_d, ':', 'Color', 'b', 'LineWidth',1.5); % Desired u2
    
    yyaxis right
    hold(ax_u2, 'on'); grid(ax_u2, 'on');ylabel(ax_u2, 'v_2');
    plot(ax_u2, t, v2, 'LineWidth', 1.5); 
    ylim(ax_u2, [min([min(u2), min(v2), min(u2_d), min(v2_d)])-.1 max([max(u2), max(v2), max(v2_d), max(u2_d)])+.1]);
    h_v2_curr = plot(ax_u2, t(1), v2(1), 'ko', 'MarkerFaceColor', 'w', 'LineWidth', 1.2);
    plot(ax_u2, t, v2_d, ':', 'Color', 'r', 'LineWidth',1.5); % Desired v2
    

    % Generate distinct colors for N particles
    colors = lines(numParticles);
    
    % Initialize arrays to store plot handles
    h_markers = gobjects(1, numParticles);
    h_trails  = gobjects(1, numParticles);
    
    % Initial Plotting loop
    legendEntries = cell(1, numParticles);
    for k = 1:numParticles
        % Plot initial state
        h_markers(k) = plot(ax_main, R(1,1,k), R(1,2,k), 'ko', 'MarkerSize', 6, 'MarkerFaceColor', colors(k,:), 'LineWidth', 1.2);
        h_trails(k)  = plot(ax_main, R(1,1,k), R(1,2,k), '-', 'Color', colors(k,:), 'LineWidth', 1.2);
        legendEntries{k} = sprintf('Agent %d', k);
    end
    legend(ax_main, h_markers, legendEntries, 'Location', 'northeast');

    % Calculate Axis Limits

    margin = 0.1;
    % Get all X data and all Y data across all pages (3rd dim)
    all_x = R(:, 1, :);
    all_y = R(:, 2, :);

    x_min = min(all_x(:)); x_max = max(all_x(:));
    y_min = min(all_y(:)); y_max = max(all_y(:));

    x_range = x_max - x_min;
    y_range = y_max - y_min;

    % Handle edge case where particles don't move (range is 0)
    if x_range == 0, x_range = 1; end
    if y_range == 0, y_range = 1; end

    xlim(ax_main, [x_min - margin*x_range, x_max + margin*x_range]);
    ylim(ax_main, [y_min - margin*y_range, y_max + margin*y_range]);

    % Video Writer Setup
    saveVideo = false;
    if nargin > 2 && ~isempty(videoFilename)
        saveVideo = true;
        writerObj = VideoWriter(videoFilename, 'Motion JPEG AVI');
        % Calculate frame rate based on time vector
        writerObj.FrameRate = 1 / mean(diff(t)); 
        open(writerObj);
        fprintf('Recording video to %s...\n', videoFilename);
    end
    
    % Animation Loop
    trailLength = 500; 
    
    for i = 1:numFrames
        if i < trailLength
            idx = 1:i;
        else
            idx = i - trailLength + 1 : i;
        end
        
        % Update all particles
        for k = 1:numParticles
            % Update Marker
            h_markers(k).XData = R(i, 1, k);
            h_markers(k).YData = R(i, 2, k);
            
            % Update Trail
            h_trails(k).XData = R(idx, 1, k);
            h_trails(k).YData = R(idx, 2, k);
        end

        % Update Subplot Markers (The red dot moving along the curve)
        h_rho_curr.XData = t(i); h_rho_curr.YData = rho(i);
        h_a1_curr.XData = t(i);  h_a1_curr.YData = alpha1(i);
        h_a2_curr.XData = t(i);  h_a2_curr.YData = alpha2(i);
        h_u2_curr.XData = t(i);  h_u2_curr.YData = u2(i);
        h_v2_curr.XData = t(i);  h_v2_curr.YData = v2(i);

        title(ax_main, sprintf('Time: %.2f s', t(i)));
        drawnow limitrate
        
        % Save Video Frame
        if saveVideo
            frame = getframe(gcf);
            writeVideo(writerObj, frame);
        end
    end
    
    if saveVideo
        close(writerObj);
        fprintf('Animation saved successfully.\n');
    end
end

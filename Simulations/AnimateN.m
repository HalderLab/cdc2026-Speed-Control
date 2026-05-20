function AnimateN(t, positions, shapes, controls, videoFilename, animate)
    % INPUTS:
    % t: Time vector (T x 1)
    % positions: Positions vector (1 x 3*N x T)
    % shapes: Shape variables rho, alpha1, alpha2 (1 x 3*(N-1) x T)
    % videoFilename (optional): String. If provided, saves .mp4 to this
    % path.
    %
    % Example (2 particles): Animate2p(t, cat(3, r1, r2), 'exampleVideo')
    
    % Setup shape variables
    [~, numParticles, numFrames] = size(positions);
    numParticles = numParticles/3;
    
    % Controls 
    controls = squeeze(controls);

    % Setup Figure
    figure; clf; 
    set(gcf, 'Color', 'w');
    set(0, 'defaultTextInterpreter', 'latex');
    set(0, 'defaultLegendInterpreter', 'latex');
    set(0, 'defaultAxesTickLabelInterpreter', 'latex');

    hold on; grid on;
    tiledlayout(2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
    
    % Main Plot
    ax_main = nexttile; 
    hold(ax_main, 'on'); grid(ax_main, 'on'); axis(ax_main, 'off');
    xlabel(ax_main, 'X [m]'); ylabel(ax_main, 'Y [m]');
    axis(ax_main, 'equal');

    % Subplot: u
    % ax_u = nexttile; 
    % hold(ax_u, 'on'); grid(ax_u, 'on');
    % title(ax_u, 'u');
    % xlabel(ax_u, 'Time [s]'); 

    % Subplot: v
    % ax_v = nexttile; 
    % hold(ax_v, 'on'); grid(ax_v, 'on');
    % ylabel(ax_v, '$v$');
    % ylim(ax_v, [0, 1.2]);

    % Subplot: u/v
    ax_kappa = nexttile; 
    hold(ax_kappa, 'on'); grid(ax_kappa, 'on');
    xlabel(ax_kappa, 'Time [s]'); ylabel(ax_kappa, '$\kappa$')

    % Generate distinct colors for N particles
    colors = lines(numParticles);
    
    % Initialize arrays to store plot handles
    h_markers = gobjects(1, numParticles);
    h_trails  = gobjects(1, numParticles);
    h_v = gobjects(1, numParticles);
    h_kappa = gobjects(1, numParticles);

    % Initial Plotting loop
    legendEntriesMarkers = cell(1, numParticles);
    numMarkers = 6;
    for k = 0:(numParticles-1)
        % Plot initial state

        % Plot markers
        for i = round(linspace(1, numel(t), numMarkers))
            initX = positions(1, 1:3:(3*numParticles), i);
            initY = positions(1, 2:3:(3*numParticles), i);
            h_links = plot(ax_main, initX(:), initY(:), '-', 'Color', [0.6 0.6 0.6], 'LineWidth', 1.2);
            h_markers(k+1) = plot(ax_main, positions(1,3*k+1,i), positions(1,3*k+2,i), 'ko', 'MarkerSize', 6, 'MarkerFaceColor', colors(k+1,:), 'LineWidth', 1.2);
        end
        h_trails(k+1)  = plot(ax_main, squeeze(positions(1,3*k+1,:)), squeeze(positions(1,3*k+2,:)), '-', 'Color', colors(k+1,:), 'LineWidth', 1.2);
        % h_v(k+1) = plot(ax_v, t(1), controls(2*k+1, 1), 'ko', 'MarkerSize', 6, 'MarkerFaceColor', colors(k+1,:), 'LineWidth', 1.2);
        % plot(ax_v, t, controls(2*k+1, :), '-', 'Color', colors(k+1,:), 'LineWidth', 1.5);
        % h_kappa(k+1) = plot(ax_kappa, t(1), controls(2*k+2, 1)/controls(2*k+1, 1), 'ko', 'MarkerSize', 6, 'MarkerFaceColor', colors(k+1,:), 'LineWidth', 1.2);
        plot(ax_kappa, t, controls(2*k+2, :)./controls(2*k+1, :), '-', 'Color', colors(k+1,:), 'LineWidth', 1.5);
        legendEntriesMarkers{k+1} = sprintf('Agent %d', k+1);
    end
    legend(ax_main, h_markers, legendEntriesMarkers, 'Location', 'northeast', Box='off');

    % Video Writer Setup
    % if animate
    %     saveVideo = true;
    %     writerObj = VideoWriter(videoFilename, 'Motion JPEG AVI');
    %     % Calculate frame rate based on time vector
    %     writerObj.FrameRate = 1 / mean(diff(t)); 
    %     open(writerObj);
    %     fprintf('Recording video to %s...\n', videoFilename);
    % end
    
    % % Animation Loop
    % if animate
    %     trailLength = 500; 
    % 
    %     for i = 1:numFrames
    %         if i < trailLength
    %             idx = 1:i;
    %         else
    %             idx = i - trailLength + 1 : i;
    %         end
    % 
    %         currentX = positions(1, 1:3:(3*numParticles), i);
    %         currentY = positions(1, 2:3:(3*numParticles), i);
    %         h_links.XData = currentX(:);
    %         h_links.YData = currentY(:);
    % 
    %         % Update all particles
    %         for k = 0:(numParticles-1)
    %             % Update Marker
    %             h_markers(k+1).XData = positions(1,3*k+1,i); h_markers(k+1).YData = positions(1,3*k+2,i);
    % 
    %             % Update Trail
    %             h_trails(k+1).XData = positions(1,3*k+1,idx); h_trails(k+1).YData = positions(1,3*k+2,idx);
    %         end
    % 
    %         % Update Subplot Markers
    %         for k = 0:numParticles-1
    %             h_v(k+1).XData = t(i); h_v(k+1).YData = controls(2*k+1, i);
    %             h_kappa(k+1).XData = t(i); h_kappa(k+1).YData = controls(2*k+2, i)/controls(2*k+1, i);
    %         end
    %         title(ax_main, sprintf('Time: %.2f s', t(i)));
    %         drawnow limitrate
    % 
    %         % Save Video Frame
    %         if saveVideo
    %             frame = getframe(gcf);
    %             writeVideo(writerObj, frame);
    %         end
    %     end
    % end
    % if animate
    %     close(writerObj);
    %     fprintf('Animation saved successfully.\n');
    % end
end


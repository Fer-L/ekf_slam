function [motion] = plot_state(xhat, P, landmarks, landmark_map, sensor_data, window, motion)
    % Visualizes the state of the EKF SLAM algorithm.
    %
    % The plot includes:
    % - Ground truth landmarks (black +)
    % - Estimated robot pose (red ellipse)
    % - Estimated landmarks (cyan o)
    % - Current observations (green lines)
    % - Robot trajectory (magenta)
    
    clf;
    hold on;
    grid on;

    % Desenha os marcadores reais (ground truth)
    L = struct2cell(landmarks);
    plot(cell2mat(L(2,:)), cell2mat(L(3,:)), 'ks', 'MarkerSize', 10, 'LineWidth', 2, 'MarkerFaceColor', 'k');
    %for i = 1:numel(landmarks)
    %    text(landmarks(i).x, landmarks(i).y, sprintf('%d', landmarks(i).id), 'Color', 'k', 'FontSize', 12, ...
    %        'HorizontalAlignment', 'center', 'VerticalAlignment', 'top');
    %end

    % Desenha a pose do robô com elipse de incerteza
    drawprobellipse(xhat(1:3), P(1:3,1:3), 0.6, [235, 102, 255]/255);
    drawrobot(xhat(1:3), 'm', 3, 0.3, 0.3);

    % Desenha os marcadores descobertos
    for i = 1:length(landmark_map)
        id = landmark_map(i);
        idx = i; % índice do marcador no vetor de estados
        state_idx = 3 + 2*(idx-1) + (1:2); % índice da posição [x, y] do marcador
        
        if all(state_idx <= size(P,1))
            xm = xhat(state_idx(1));
            ym = xhat(state_idx(2));
            pos_m = [xm; ym];
            cov_m = P(state_idx, state_idx);

            drawprobellipse(pos_m, cov_m, 0.6, [255, 201, 227]/255);
            plot(xm, ym, '*', 'MarkerSize', 10, 'LineWidth', 1.5, 'Color', [255, 0, 123]/255);
            %text(xm, ym, sprintf('%d', id), 'Color', 'b', 'FontSize', 10, ...
            %    'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom');
        end
    end

    % Traça as observações atuais
    xr = xhat(1); yr = xhat(2);
    for i = 1:length(sensor_data)
        id = sensor_data(i).id;
        idx = find(landmark_map == id, 1);
        if ~isempty(idx)
            xm = xhat(3 + 2*(idx-1) + 1);
            ym = xhat(3 + 2*(idx-1) + 2);
            plot([xr, xm], [yr, ym], '-', 'Color', [229, 173, 255]/255);
        end
    end

    % Atualiza trajetória do robô
    motion = [motion; xr, yr];
    plot(motion(:,1), motion(:,2), '.', 'MarkerSize', 5, 'Color', [118, 0, 173]/255);

    xlim([-2, 12]);
    ylim([-2, 12]);
    axis equal;

    if window
        set(gcf, 'Visible', 'on');
        drawnow;
        pause(0.1);
    else
        set(gcf, 'Visible', 'off');
        filename = sprintf('../plots/ekf_%03d.png', timestep);
        print(filename, '-dpng');
    end
end

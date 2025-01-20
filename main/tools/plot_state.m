function [motion] = plot_state(mu, sigma, landmarks, observedLandmarks, z, window, motion)
    % Visualizes the state of the EKF SLAM algorithm.
    %
    % The resulting plot displays the following information:
    % - map ground truth (black +'s)
    % - current robot pose estimate (red)
    % - current landmark pose estimates (blue)
    % - visualization of the observations made at this time step (line between robot and landmark)

    clf;
    hold on;
    grid on;

    % Convert landmarks structure to cell array for easier access
    L = struct2cell(landmarks); 

    % Draw robot's belief ellipse
    drawprobellipse(mu(1:3), sigma(1:3, 1:3), 0.6, 'b');

    % Plot ground truth landmarks (black +'s)
    plot(cell2mat(L(2,:)), cell2mat(L(3,:)), 'k+', 'markersize', 10, 'linewidth', 3);

    for i=1:length(observedLandmarks)
        if(observedLandmarks(i))
            plot(mu(2 * i + 2), mu(2 * i + 3), '*', 'markersize', 5, 'linewidth', 3)
            drawprobellipse(mu(2*i+ 2:2*i+ 3), sigma(2*i+ 2:2*i+ 3,2*i+ 2:2*i+ 3), 0.9, 'm');
        end
    end

    for i=1:size(z,2)
	    mX = mu(2*z(i).id+2);
	    mY = mu(2*z(i).id+3);
    	line([mu(1), mX],[mu(2), mY], 'color', 'k', 'linewidth', 1);
    end

    %for i = 1:size(landmarks, 2)
    %    text(landmarks(i).x, landmarks(i).y, sprintf('%d', landmarks(i).id), 'Color', 'b', 'FontSize', 9, 'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom');
    %end

    drawrobot(mu(1:3), 'r', 3, 0.3, 0.3);
    xlim([-2, 12])
    ylim([-2, 12])

    motion = [ motion; mu(1) mu(2)];
    %plot(motion(:,1), motion(:,2), '.', 'markersize', 5, 'Color', 'm');
    hold off

    if window
      set(1, "visible", "on");
      axis equal
      drawnow;
      pause(0.1);
    else
      set(1, "visible", "off");
      filename = sprintf('../plots/ekf_%03d.png', timestep);
      print(filename, '-dpng');
    end
end


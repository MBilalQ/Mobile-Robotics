%code help taken from gpt after finding the equation for q
t = linspace(0, pi, 50);  % Time
xr = t;  % Desired trajectory for x
yr = zeros(size(t));  % Desired trajectory for y, zero as y is zero all the time
phi_r = t;  % Desired phi

% Initial pose
x = 0.5;
y = -0.5;
phi = -pi/4;

kp = 1.5;  

%storing the values in an array
x_actual = zeros(size(t));
y_actual = zeros(size(t)); 
phi_actual = zeros(size(t));
x_actual(1) = x;
y_actual(1) = y;
phi_actual(1) = phi;

% Set up figure for visualization
figure;
hold on;
axis equal;
xlim([0, pi+1]);
ylim([-1, 1]);
title('Holonomic Robot Tracking');
xlabel('X Position');
ylabel('Y Position');

% Plot the desired trajectory (xr, yr)
plot(xr, yr, '--r', 'DisplayName', 'Desired Trajectory');
legend;

% Simulation loop
robot_plot = plot(x, y, 'bo-', 'MarkerSize', 1, 'DisplayName', 'Actual Trajectory');

for i = 2:length(t)
    % Current desired pose
    q_desired = [xr(i); yr(i); phi_r(i)];
    q_actual = [x_actual(i-1); y_actual(i-1); phi_actual(i-1)];
    
    %Feedforward + Feedback
    q = [1 + kp*(xr(i) - x_actual(i-1)); 
         kp*(yr(i) - y_actual(i-1)); 
         1 + kp*(phi_r(i) - phi_actual(i-1))];
    
    % Update actual pose (simple Euler integration)
    dt = t(i) - t(i-1);
    q_actual = q_actual + q * dt;
    
    % Store results
    x_actual(i) = q_actual(1);
    y_actual(i) = q_actual(2);
    phi_actual(i) = q_actual(3);
    
    % Update robot's path plot in real-time
    set(robot_plot, 'XData', x_actual(1:i), 'YData', y_actual(1:i));
    drawnow;
    
    % Optional: Pause for a smoother animation (adjust as needed)
    pause(0.01);
end

% Plot the final error results
figure;
subplot(3,1,1);
plot(t, xr - x_actual);
title('Error in X');
xlabel('Time');
ylabel('X Error');

subplot(3,1,2);
plot(t, yr - y_actual);
title('Error in Y');
xlabel('Time');
ylabel('Y Error');

subplot(3,1,3);
plot(t, phi_r - phi_actual);
title('Error in Phi');
xlabel('Time');
ylabel('Phi Error');

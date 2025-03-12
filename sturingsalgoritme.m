current_position = [1, 2];
steering_angle = 0;
max_steering_rate = 0.1745;
max_steering_angle = deg2rad(35);
linear_velocity = 0;
max_velocity = 0.5;
dt = 0.01;

list = [];

command_table = [];

[e, n, u] = coordsConv();
[path_x, path_y, waypoints] = pathplanner([e, n, u]);
current_waypoint_index = 1;

prediction_horizon = 10;
Q = diag([10, 10, 0.1]);
R = 1.5;

yaw = getStartYaw([e, n, u], 2) + 0.1; % In radians
encoder_data = Simulation_model(0, 0, 0, true, false);
positionCalculation(encoder_data, yaw, true);
current_position = [1, 2];

figure;
hold on;
grid on;
axis equal;
% title('Position Tracking with Waypoints');
xlabel('X positie [m]', FontSize=11);
ylabel('Y positie [m]', FontSize=11);

plot(waypoints(:, 1), waypoints(:, 2), 'kx-', 'LineWidth', 1.5, 'DisplayName', 'Waypoints');
legend();

current_position_plot = plot(current_position(1), current_position(2), 'bo', ...
    'MarkerFaceColor', 'b', 'MarkerSize', 8, 'DisplayName', 'Current Position');

position_history = [];

while true
    if current_waypoint_index > size(waypoints, 1)
        disp('Reached the final waypoint!');
        plot(position_history(:, 1), position_history(:, 2), 'b-', 'LineWidth', 1.2);
        legend('Waypoints', 'Huidige positie', 'Gereden pad');
        Simulation_model(0, steering_angle, dt, false, true);
        break;
    end

    % Find the closest waypoint dynamically
    distances = vecnorm(waypoints(:, 1:2) - current_position, 2, 2);
    
    % Only consider waypoints ahead of the current waypoint index
    valid_indices = current_waypoint_index:size(waypoints, 1);
    [~, closest_valid_index] = min(distances(valid_indices));
    closest_waypoint_index = valid_indices(closest_valid_index);
    
    % Update current waypoint index if it moves forward
    if closest_waypoint_index > current_waypoint_index
        current_waypoint_index = closest_waypoint_index;
    end

    target_waypoint = waypoints(current_waypoint_index, :);
    distance_to_waypoint = norm(target_waypoint - current_position);

    % Move to the next waypoint if within a threshold
    if distance_to_waypoint < 0.2
        current_waypoint_index = current_waypoint_index + 1;
        continue;
    end
    
    [current_position, current_yaw] = positionCalculation(encoder_data, yaw, false);
       
    % disp(current_position);
    yaw = current_yaw;

    [steering_angle, linear_velocity, optimal_path, list] = solve_mpc(current_position, yaw, steering_angle, waypoints, current_waypoint_index, prediction_horizon, Q, R, linear_velocity, max_steering_rate, max_steering_angle, max_velocity, 0.01, list);
    
    command_table = [command_table; steering_angle, linear_velocity];

    position_history = [position_history; current_position];
    set(current_position_plot, 'XData', current_position(1), 'YData', current_position(2));    
    %plot(position_history(:, 1), position_history(:, 2), 'b-', 'LineWidth', 1.2);
    %drawnow;

    encoder_data = Simulation_model(linear_velocity, steering_angle, dt, false, false);
    %pause(0.01);
end

function [optimal_steering_angle, optimal_velocity, optimal_path, delta_list] = solve_mpc(current_position, yaw, current_steering_angle, waypoints, waypoint_index, horizon, Q, R, v, max_steering_rate, max_steering_angle, max_velocity, dt, delta_list)
    horizon_waypoints = waypoints(waypoint_index:min(waypoint_index + horizon - 1, size(waypoints, 1)), :);
    if waypoint_index + horizon - 1 > size(waypoints, 1)
        horizon = size(waypoints, 1) - waypoint_index + 1;
    end

    x = current_position(1);
    y = current_position(2);
    theta = yaw;
    delta = current_steering_angle;
    
    L1=1.2;
    L2=0.8;

    optimal_path = zeros(horizon, 3);
    cost = 0;
    optimal_velocity = max_velocity; % Default to max forward velocity

    for t = 1:horizon
        if t <= size(horizon_waypoints, 1)
            target = horizon_waypoints(t, :);
        else
            target = horizon_waypoints(end, :);
        end

        % Bereken gewenste yaw en yaw-fout
        desired_yaw = atan2(target(2) - y, target(1) - x);
        yaw_error = mod(desired_yaw - theta + pi, 2 * pi) - pi;

        % Bereken stuurhoekverandering op basis van yaw-fout        
        desired_yaw_rate = yaw_error;
        delta_desired = (desired_yaw_rate * (L2 + L1 * cos(delta)) - v * sin(delta))/L2;
        
        steering_rate_change = max(-max_steering_rate * dt, min(max_steering_rate * dt, delta_desired - delta));
        delta = delta + steering_rate_change;
        delta = max(-max_steering_angle, min(max_steering_angle, delta));

        % Update positie en oriëntatie
        x_next = x + optimal_velocity * cos(theta) * dt;
        y_next = y + optimal_velocity * sin(theta) * dt;

        % Calculate the change in heading (delta_theta) based on the new formula
        delta_theta = (L2 * steering_rate_change + optimal_velocity * sin(delta)) / (L2 + L1 * cos(delta));
        theta_next = theta + delta_theta * dt;

        % theta_next = theta + (optimal_velocity / 1.5) * tan(delta) * dt;

        % Bereken fouten en kosten
        pos_error = [x_next - target(1); y_next - target(2)];
        state_error = [pos_error; yaw_error];
        cost = cost + state_error' * Q * state_error + R * delta^2;

        % Sla resultaten op
        optimal_path(t, :) = [x_next, y_next, theta_next];

        % Update state voor de volgende iteratie
        x = x_next;
        y = y_next;
        theta = theta_next;
    end
    
    delta_list = [delta_list; rad2deg(delta)];

    optimal_steering_angle = delta;
end


function [e, n, u] = coordsConv()
    inputFileName = 'coordinates.csv';
    opts = detectImportOptions(inputFileName);
    opts = setvartype(opts, {'Latitude', 'Longitude', 'Height'}, 'double');
    data = readtable(inputFileName, opts);
    
    lat = data.Latitude;
    lon = data.Longitude;
    h = data.Height; 
    wgs84 = wgs84Ellipsoid('meter');
    
    [x,y,z] = geodetic2ecef(wgs84, rad2deg(lat), rad2deg(lon), h);
    
    refLat = rad2deg(lat(1));
    refLon = rad2deg(lon(1));
    refH = h(1);
    
    [e, n, u] = ecef2enu(x, y, z, refLat, refLon, refH, wgs84);
    plot(e, n, '.');
    axis equal;
    % e = e / 100;
    % n = n / 100;
    % u = u / 100;
end

function yaw = getStartYaw(enu_coords, min_distance)
    if size(enu_coords, 1) < 2
        error('Not enough coordinates to calculate yaw.');
    end

    start_point = enu_coords(1, :);

    for i = 2:size(enu_coords, 1)
        current_point = enu_coords(i, :);
        distance = norm(current_point - start_point);
        if distance > min_distance
            delta_x = current_point(1) - start_point(1);
            delta_y = current_point(2) - start_point(2);
            yaw = atan2(delta_y, delta_x);
            return;
        end
    end

    error('No points found more than the specified distance apart.');
end


current_position = [0, 0];
L1 = 1.2;
L2 = 0.8;
steer_cmd = 0;
steering_angle = 0;
max_steering_speed = 0.29;
max_steering_angle = deg2rad(35);
linear_velocity = 0;
max_velocity = 0.5;
dt = 0.05;
steps = 10000;
current_waypoint_index = 1;

waypoint_horizon = 10;
prediction_horizon = 6;
Ts = 0.5;

list = [];
command_table = [];
states_table = [];

%[e, n, u] = coordsConv_old();
%[path_x, path_y, waypoints] = pathplanner([e, n, u]);
%yaw = getStartYaw([e, n, u], 2) + 0.1;

coords = coordsConv();
[path_x, path_y, waypoints] = pathplanner_new(coords);
yaw = getStartYaw(coords, 2);

encoder_data = Simulation_model(0, 0, 0, true, false);
positionCalculation(encoder_data, yaw, true);

u0 = [linear_velocity, steer_cmd];
mpcobj = initialise_mpc(L1, L2, prediction_horizon, Ts, max_steering_speed, [current_position, yaw, steering_angle], u0);

figure;
hold on;
grid on;
axis equal;
% title('Position Tracking with Waypoints');
xlabel('X positie [m]', FontSize=11);
ylabel('Y positie [m]', FontSize=11);

plot(waypoints(:, 1), waypoints(:, 2), 'kx-', 'LineWidth', 1.5, 'DisplayName', 'Waypoints');
% legend();

current_position_plot = plot(current_position(1), current_position(2), 'bo', ...
    'MarkerFaceColor', 'b', 'MarkerSize', 8, 'DisplayName', 'Current Position');

position_history = [];

% tic
for i = 1:1:steps
    if current_waypoint_index > size(waypoints, 1) - 1 
        disp('Reached the final waypoint!');
        plot(position_history(:, 1), position_history(:, 2), 'b-', 'LineWidth', 1.2);
        legend('Waypoints', 'Huidige positie', 'Gereden pad');
        Simulation_model(0, steer_cmd, dt, false, true);
        break;
    end
    
    if i == steps
        disp('Unable to reach final waypoint, max number of steps reached');
        plot(position_history(:, 1), position_history(:, 2), 'b-', 'LineWidth', 1.2);
        legend('Waypoints', 'Huidige positie', 'Gereden pad');
        Simulation_model(0, steer_cmd, dt, false, true);
        break;
    end

    tic
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
    if distance_to_waypoint < 0.5
        current_waypoint_index = current_waypoint_index + 1;
        continue;
    end
    delta_t(4) = toc;
    
    tic
    [current_position, current_yaw, steering_angle] = positionCalculation(encoder_data, yaw, false);
       
    % disp(current_position);
    yaw = current_yaw;
    
    x = [current_position, yaw, steering_angle];
    delta_t(5) = toc;

    tic
    ref = generateReference(waypoints(min(current_waypoint_index, length(waypoints)),:), waypoints, waypoint_horizon);
    [u,x,info] = nlmpcmove(mpcobj, x, u0, ref);
    % disp(info);
    linear_velocity = u(1);
    steer_cmd = u(2);
    delta_t(1) = toc;
    
    tic
    command_table = [command_table; steer_cmd, linear_velocity];
    states_table = [states_table; x];

    position_history = [position_history; current_position];
    delta_t(2) = toc;
    
    tic
    if mod(i, 100) == 0
        set(current_position_plot, 'XData', current_position(1), 'YData', current_position(2));    
        plot(position_history(:, 1), position_history(:, 2), 'b-', 'LineWidth', 1.2);

        % Bereken de richting van de pijl
        arrow_length = 0.5; % Lengte van de pijl, pas aan indien nodig
        dx = arrow_length * cos(current_yaw);
        dy = arrow_length * sin(current_yaw);
        
        % Teken de pijl
        quiver(current_position(1), current_position(2), dx, dy, 0, ...
               'Color', 'r', 'LineWidth', 1.5, 'MaxHeadSize', 2);

        drawnow limitrate;
    end
    delta_t(3) = toc;

    tic
    encoder_data = Simulation_model(linear_velocity, steer_cmd, dt, false, false);
    delta_t(4) = toc;
    %pause(0.01);

    %disp(delta_t);
end
% toc

function coords = coordsConv()
    %inputFileName = 'straight_path.csv';
    %inputFileName = 'turning_path.csv';
    %inputFileName = 'circle_path.csv';
    %inputFileName = 'figure8_path_scaled.csv';
    inputFileName = 'turning_path_edit.csv';

    opts = detectImportOptions(inputFileName);
    opts = setvartype(opts, {'x', 'y'}, 'double');
    coords = readtable(inputFileName, opts);
    coords = table2array(coords);
end

function [e, n, u] = coordsConv_old()
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

function ref = generateReference(currentPos, waypoints, horizon)
    % currentPos: [x, y] current robot position
    % waypoints: Nx2 matrix of [x, y] waypoints
    % horizon: number of steps to look ahead (MPC prediction horizon)
    
    % Only consider waypoints ahead of the current waypoint index
    % Make new array with only waypoints ahead
    % Set a max view check
    % valid_indices = current_waypoint_index:size(waypoints, 1);
    % [~, closest_valid_index] = min(distances(valid_indices));
    % closest_waypoint_index = valid_indices(closest_valid_index);

    % Find the closest waypoint index to the current position
    distances = vecnorm(waypoints - currentPos, 2, 2);
    [~, idx] = min(distances);

    % Make sure we don't go out of bounds
    lastIdx = min(idx + horizon - 1, size(waypoints, 1) - 1);

    % Initialize reference
    ref = zeros(horizon, 4); % [x y theta gamma]

    for i = 1:horizon
        currentIdx = min(idx + i - 1, size(waypoints,1) - 1);
        nextIdx = currentIdx + 1;

        % Get position
        pos = waypoints(currentIdx, :);

        % Estimate heading
        delta = waypoints(nextIdx, :) - waypoints(currentIdx, :);
        theta = atan2(delta(2), delta(1)); % heading toward the next point

        % Fill reference
        ref(i, :) = [pos, theta, 0];
    end
end

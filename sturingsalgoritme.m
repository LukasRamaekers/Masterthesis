current_position = [1, 2];
L1 = 1.2;
L2 = 0.8;
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

prediction_horizon = 50;
Ts = 0.01;

yaw = getStartYaw([e, n, u], 2) + 0.1; % In radians
encoder_data = Simulation_model(0, 0, 0, true, false);
positionCalculation(encoder_data, yaw, true);
current_position = [1, 2];

u0 = [linear_velocity, steering_angle];
mpcobj = initialise_mpc(L1, L2, prediction_horizon, Ts, max_steering_angle, [current_position, yaw], u0);

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

tic
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
    
    x = [current_position, yaw];
    
    [u,~,~] = nlmpcmove(mpcobj, x, u0, [waypoints(min(current_waypoint_index + 10, length(waypoints)),:), yaw]);
    linear_velocity = u(1);
    steering_angle = u(2);

    command_table = [command_table; steering_angle, linear_velocity];

    position_history = [position_history; current_position];
    set(current_position_plot, 'XData', current_position(1), 'YData', current_position(2));    
    %plot(position_history(:, 1), position_history(:, 2), 'b-', 'LineWidth', 1.2);
    %drawnow;

    encoder_data = Simulation_model(linear_velocity, steering_angle, dt, false, false);
    %pause(0.01);
end
toc

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


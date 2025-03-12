function [path_x, path_y, waypoints] = pathplanner(coords)
    % Generate a path using Catmull-Rom splines with a maximum turning angle
    filtered_coords = preprocessCoords(coords, 1);
    [path_x, path_y] = CatmullRomPath(filtered_coords, 35);

    % Generate waypoints from the path
    waypoints = generateWaypoints(path_x, path_y, 0.5); % Specify spacing between waypoints (e.g., 0.5 meters)
    
    % waypoints = limitTurnAngle(waypoints, 35, 2);

    % figure;
    % h1 = plot(path_x, path_y);
    % hold on;
    % h2 = plot(waypoints(:, 1), waypoints(:, 2), 'rx', 'MarkerSize', 4, 'LineWidth', 1.5);
    % axis equal;
    % xlabel('x (m)', 'FontSize', 11);
    % ylabel('y (m)', 'FontSize', 11);
    % legend([h1, h2], 'Catmull-Rom pad', 'Waypoints');
    % grid on;
    % hold off;

    % Write the data to a CSV file with headers
    % outputFileName = 'waypoints.csv';
    % header = {'e', 'n', 'u'};
    % fileID = fopen(outputFileName, 'w');
    % fprintf(fileID, '%s,%s,%s\n', header{:});
    % fclose(fileID);
    % writematrix(waypoints, outputFileName, 'WriteMode', 'append');
    % fprintf('Data has been successfully written to %s\n', outputFileName);
end

function filtered_coords = preprocessCoords(coords, threshold)
    % Reduce noise by clustering close points and selecting representative ones
    filtered_coords = coords(1, :); % Initialize with the first point
    
    for i = 2:size(coords, 1)
        % Calculate the distance to the last filtered point
        dist = norm(coords(i, :) - filtered_coords(end, :));
        
        if dist > threshold
            filtered_coords = [filtered_coords; coords(i, :)]; % Add point if far enough
        end
    end
end

function [path_x, path_y] = CatmullRomPath(coords, max_turn_angle)
    n_points = size(coords, 1);
    t = linspace(0, 1, 100);
    path_x = [];
    path_y = [];
    
    % Add artificial points to ensure first and last points are included
    extended_coords = [2*coords(1,:) - coords(2,:);  % Extrapolated before the first point
                       coords; 
                       2*coords(end,:) - coords(end-1,:)]; % Extrapolated after the last point

    % Iterate through all segments
    for i = 2:n_points
        P0 = extended_coords(i-1, :);
        P1 = extended_coords(i, :);
        P2 = extended_coords(i+1, :);
        P3 = extended_coords(i+2, :);

        % Calculate Catmull-Rom segment
        catmull_rom_segment = 0.5 * (2*P1 + (-P0 + P2) .* t' + ...
                                (2*P0 - 5*P1 + 4*P2 - P3) .* (t'.^2) + ...
                                (-P0 + 3*P1 - 3*P2 + P3) .* (t'.^3));

        % Check steering angle and adjust if needed
        %catmull_rom_segment = limitTurnAngle(catmull_rom_segment, max_turn_angle);

        % Store path coordinates
        path_x = [path_x; catmull_rom_segment(:,1)];
        path_y = [path_y; catmull_rom_segment(:,2)];
    end
end

function waypoints = limitTurnAngle(waypoints, maxAngle, wheelbase)    
    i = 2;
    while i < size(waypoints,1) - 1
        P1 = waypoints(i-1, :);
        P2 = waypoints(i, :);
        P3 = waypoints(i+1, :);
        
        % Compute vectors and the turning angle
        v1 = P2 - P1;
        v2 = P3 - P2;
        
        angle = acos(dot(v1, v2) / (norm(v1) * norm(v2))); % Compute angle in radians
        
        if angle > maxAngle % If angle exceeds constraint, insert a new waypoint
            % Calculate minimum turning radius
            R_min = wheelbase / tan(maxAngle); %Diferent formula needed (for ackerman vehicles) / Calculate once, fixed variable
            
            % Compute direction of the turn
            turnDirection = sign(det([v1; v2])); % +1 left turn, -1 right turn
            
            % Compute new waypoint location
            d = R_min * tan(maxAngle / 2); % Arc-based distance for smooth transition
            
            % Normalize v1 and rotate it slightly to find a valid intermediate point
            v1 = v1 / norm(v1); % Normalize
            v1_rotated = [cos(maxAngle/2), -turnDirection*sin(maxAngle/2); 
                          turnDirection*sin(maxAngle/2), cos(maxAngle/2)] * v1'; 
            newPoint = P2 + d * v1_rotated'; 
            
            % Insert new point and continue checking
            waypoints = [waypoints(1:i, :); newPoint; waypoints(i+1:end, :)];
        else
            i = i + 1; % Move to next set of points
        end
    end
end

function waypoints = generateWaypoints(path_x, path_y, spacing)
    % Create waypoints with uniform spacing along the path
    path = [path_x, path_y];
    waypoints = path(1, :); % Start with the first point
    cumulative_distance = 0;

    for i = 2:length(path_x)
        % Calculate the distance between consecutive points
        distance = norm(path(i, :) - path(i-1, :));
        cumulative_distance = cumulative_distance + distance;

        % Add waypoint if cumulative distance exceeds spacing
        if cumulative_distance >= spacing
            waypoints = [waypoints; path(i, :)];
            cumulative_distance = 0; % Reset cumulative distance
        end
    end
end
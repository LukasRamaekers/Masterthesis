function [path_x, path_y, waypoints] = pathplanner_new(coords)
    % Generate a path using Catmull-Rom splines with a maximum turning angle
    weighted_coords = weightedCoordCluster(coords, 0.75, 1);
    filtered_coords = clusterAverageCoords(weighted_coords, 0.5);
    [path_x, path_y] = CatmullRomPathIterative(filtered_coords, 1/2.6, 50);

    % Generate waypoints from the path
    waypoints = generateWaypoints(path_x, path_y, 0.5); % Specify spacing between waypoints (e.g., 0.5 meters)

    % Write the data to a CSV file with headers
    % outputFileName = 'waypoints.csv';
    % header = {'e', 'n', 'u'};
    % fileID = fopen(outputFileName, 'w');
    % fprintf(fileID, '%s,%s,%s\n', header{:});
    % fclose(fileID);
    % writematrix(waypoints, outputFileName, 'WriteMode', 'append');
    % fprintf('Data has been successfully written to %s\n', outputFileName);
end

function filtered_coords = clusterAverageCoords(coords, threshold)
    % Initialize filtered coordinates with the first point
    filtered_coords = coords(1, :);
    temp_cluster = coords(1, :); % Temporary cluster for averaging

    for i = 2:size(coords, 1)
        % Compute distance to last stored point
        dist = norm(coords(i, :) - filtered_coords(end, :));

        if dist < threshold
            % Add to temporary cluster (points within threshold)
            temp_cluster = [temp_cluster; coords(i, :)];
        else
            % Compute cluster centroid (average)
            filtered_coords(end, :) = mean(temp_cluster, 1);
            
            % Start a new cluster
            temp_cluster = coords(i, :);
            filtered_coords = [filtered_coords; coords(i, :)];
        end
    end

    % Ensure last cluster is averaged
    filtered_coords(end, :) = mean(temp_cluster, 1);
end
 
function filtered_coords = weightedCoordCluster(coords, threshold, sigma)
    filtered_coords = zeros(size(coords));

    for i = 1:size(coords, 1)
        max_lookahead = min(i + 10, size(coords, 1));
        lookahead_coords = coords(1:max_lookahead, :);
        
        % Calculate distances to all points in the lookahead window
        distances = vecnorm(lookahead_coords - coords(i, :), 2, 2);
        
        % Apply the Gaussian weight function: w(x) = exp(-x^2 / (2 * sigma^2))
        weights = exp(-distances.^2 / (2 * sigma^2));
        
        % Only keep points within the threshold distance
        valid_indices = distances < threshold;
        
        % Normalize weights for the selected valid points
        weighted_coords = bsxfun(@times, lookahead_coords(valid_indices, :), weights(valid_indices) / sum(weights(valid_indices)));
        
        % Calculate the weighted average of the valid points
        filtered_coords(i, :) = sum(weighted_coords, 1);
    end
end

% Iterative Catmull-Rom pathplanner
function cost = curvature_cost(P2, P0, P1, P3, t, max_curvature)
    % Generate segment with optimized P2 (keeping P1 fixed)
    segment = catmullRomSegment(P0, P1, P2, P3, t);
    
    % Compute curvature
    K = LineCurvature2D([segment(:,1), segment(:,2)]);
    K(abs(K) > 10) = 0;

    % Cost: Penalize curvature values exceeding the constraint
    cost = mean(max(0, abs(K) - max_curvature).^2);
end

function cost = gradient_curvature_cost(P2, P0, P1, P3, t, max_curvature)
    % Generate segment with optimized P2 (keeping P1 fixed)
    segment = catmullRomSegment(P0, P1, P2, P3, t);
    
    % Compute curvature
    K = LineCurvature2D([segment(:,1), segment(:,2)]);
    K(abs(K) > 10) = 0;

    % Sigmoid-based penalty
    alpha = 0.1; % Controls how steeply the penalty increases
    beta = 10; % Scaling factor to control cost magnitude
    
    penalty = 1 ./ (1 + exp(-alpha * (abs(K) - max_curvature)));
    
    % Cost: Mean penalty across all points
    cost = beta * mean(penalty);
end

function [path_x, path_y] = CatmullRomPathIterative(coords, max_curvature, max_iterations)
    n_points = size(coords, 1);
    t = linspace(0, 1, 100);
    path_x = [];
    path_y = [];
    
    % Add artificial points for boundary conditions
    extended_coords = [2*coords(1,:) - coords(2,:); coords; 2*coords(end,:) - coords(end-1,:)];
    
    options = optimoptions('fmincon', 'Algorithm', 'sqp', 'Display', 'off');
    
    for iter = 1:max_iterations
        modified = false; % Track if any point was modified
        i = 2;
        
        while i < n_points + 2
            P0 = extended_coords(i-1, :);
            P1 = extended_coords(i, :);
            P2 = extended_coords(i+1, :);
            if i+2 <= size(extended_coords, 1)
                P3 = extended_coords(i+2, :);
            else
                P3 = extended_coords(end, :); % Laatste punt herhalen om out-of-bounds te vermijden
            end

            
            % Generate Catmull-Rom segment
            segment = catmullRomSegment(P0, P1, P2, P3, t);
            K = LineCurvature2D([segment(:,1), segment(:,2)]);
            K(abs(K) > 10) = 0;
            
            % Check if curvature exceeds max limit
            if any(abs(K) > max_curvature)
                modified = true;
                segment_length = norm(P2 - P1);
                max_adjustment = 0.1 * segment_length;
                lb = P2 - max_adjustment;
                ub = P2 + max_adjustment;
                
                % Optimize P2
                P2_opt = fmincon(@(P) curvature_cost(P, P0, P1, P3, t, max_curvature), ...
                                P2, [], [], [], [], lb, ub, [], options);
                
                % Adaptive spreading if the change is too large
                P2 = (P2 + P2_opt) / 2;
                extended_coords(i+1, :) = P2;
            end
            
            i = i + 1;
        end
        
        % Stop iterating if no modifications were made
        if ~modified
            break;
        end
    end
    
    % Generate final path
    i = 2;
    while i < n_points + 2
        P0 = extended_coords(i-1, :);
        P1 = extended_coords(i, :);
        P2 = extended_coords(i+1, :);
        if i+2 <= size(extended_coords, 1)
            P3 = extended_coords(i+2, :);
        else
            P3 = extended_coords(end, :);
        end
        
        segment = catmullRomSegment(P0, P1, P2, P3, t);
        path_x = [path_x; segment(:,1)];
        path_y = [path_y; segment(:,2)];
        
        % Compute curvature for visualization
        K = LineCurvature2D([segment(:,1), segment(:,2)]);
        K(abs(K) > 10) = 0;

        % Plot the optimized segment
        % if any(abs(K) > max_curvature)
        %     plot(segment(:,1), segment(:,2), 'r', 'LineWidth', 1.5);
        % else
        %     plot(segment(:,1), segment(:,2), 'g', 'LineWidth', 1.5);
        % end

        i = i + 1;
    end
end

function segment = catmullRomSegment(P0, P1, P2, P3, t)
    segment = 0.5 * (2*P1 + (-P0 + P2) .* t' + (2*P0 - 5*P1 + 4*P2 - P3) .* (t'.^2) + (-P0 + 3*P1 - 3*P2 + P3) .* (t'.^3));
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
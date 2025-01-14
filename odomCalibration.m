[file, path] = uigetfile('*', 'Select a File', 'C:\Users\lukas\Documents\MP_v2\rosbags');
bag = ros2bagreader(path);
encoderTable = getEncoderData(bag);
gpsTable = getGpsData(bag);

lb = [0.20, 0.20, 0.20, 0.20];
ub = [0.25, 0.25, 0.25, 0.25];
initial_guess = [0.249999998269480, 0.235000001701395, 0.23, 0.23, 0, 0, 1.8895];

% rms_error = cost_function(initial_guess, encoderTable, gpsTable);
% optimized_radii = fminsearch(@(r) cost_function(r, encoderTable, gpsTable), initial_guess);

objective_function = @(r) cost_function(r, encoderTable, gpsTable);
% optimized_radii = fmincon(objective_function, initial_guess, [], [], [], [], [], [], [], lb, ub);

options = optimset('MaxIter',inf,'MaxFunEvals',inf,'TolFun',1e-9,'TolX',1e-9);
optimized_radii=fminsearch(objective_function,x,options);

disp(double(optimized_radii));

function rms_error = cost_function(r, encoderTable, gpsTable)
    % Extract radii
    r1 = r(1); r2 = r(2); r3 = r(3); r4 = r(4); start_pos_x = r(5); start_pos_y = r(6); start_yaw = r(7);

    % Compute encoder-based positions
    [x_enc, y_enc] = encoder_position(r1, r2, r3, r4, encoderTable.pos, encoderTable.steer, start_pos_x, start_pos_y, start_yaw);
    encoder_time = encoderTable.time;
    
    gps_time = gpsTable.time;
    e = gpsTable.e;
    n = gpsTable.n;
    
    % Interpolate encoder positions to GPS timestamps
    matched_x_enc = interp1(encoder_time, x_enc, gps_time, 'linear', 'extrap');
    matched_y_enc = interp1(encoder_time, y_enc, gps_time, 'linear', 'extrap');
    
    % plot(matched_x_enc, matched_y_enc, 'o-');
    % hold on;
    % plot(e, n, 'o-');
    % axis equal;
    % grid on;

    % Compute RMS error
    rms_error = sqrt(mean((e - matched_x_enc).^2 + (n - matched_y_enc).^2, 'all'));
    % rms_error = sqrt((e - matched_x_enc).^2 + (n - matched_y_enc).^2);
end

function [x_steer, y_steer] = encoder_position(r1, r2, r3, r4, encoders, steer_encoder, start_pos_x, start_pos_y, start_yaw)
    wheel_f=2*pi*[r1 -r2 r3 -r4]/1952;
    enc2ang=@(x) -0.6711 * (x.^4) + 0.6546 * (x.^3) - 0.3672 * (x.^2) - 1.155 * x + 0.6783;
    
    delta_encoders=diff(encoders);

    delta_pos=delta_encoders.*wheel_f;
    avg_delta_pos=mean(delta_pos,2);
    steer_angle=enc2ang(steer_encoder/32767);
    delta_angle=diff(steer_angle);
    
    L1=1.2;
    L2=0.8;
    
    delta_theta_steer=(L2*delta_angle+avg_delta_pos.*sin(steer_angle(1:end-1)))./(L2+L1*cos(steer_angle(1:end-1)));
    theta_steer=-cumsum([start_yaw;delta_theta_steer]);
    delta_x_steer=avg_delta_pos.*cos(theta_steer(1:end-1));
    delta_y_steer=avg_delta_pos.*sin(theta_steer(1:end-1));
    x_steer=cumsum([0;delta_x_steer]);
    y_steer=cumsum([0;delta_y_steer]);
    
    w = 1.5;
    d_left_front = delta_pos(:, 1);
    d_right_front = -delta_pos(:, 2);
    d_left_rear = delta_pos(:, 3);
    d_right_rear = -delta_pos(:, 4);
    
    d_theta_wheel_front = (d_left_front - abs(d_right_front)) / w;
    d_theta_wheel_rear = (d_left_rear - abs(d_right_rear)) / w;
    
    d_theta_wheel_avg = (d_theta_wheel_front + d_theta_wheel_rear) / 2;
    
    theta_wheel = -cumsum([start_yaw;d_theta_wheel_rear]);
    delta_x_wheel = avg_delta_pos.*cos(theta_wheel(1:end-1));
    delta_y_wheel = avg_delta_pos.*sin(theta_wheel(1:end-1));
    x_wheel=cumsum([0;delta_x_wheel]) + start_pos_x;
    y_wheel=cumsum([0;delta_y_wheel]) + start_pos_y;
end

function encoder_table = getEncoderData(bag)
    encoder_sel = select(bag, "Topic", "/encoder_data");
    encoderData = readMessages(encoder_sel);
    
    MessageList = bag.MessageList;
    filteredMessageList = MessageList(MessageList.Topic == '/encoder_data', :);

    pos = cellfun(@(msg) msg.pos, encoderData, 'UniformOutput', false);
    pos = cell2mat(pos')';
    steerEncoder = cellfun(@(msg) msg.steer, encoderData, 'UniformOutput', true);

    encoder_table.time = double(filteredMessageList{:, 1});
    encoder_table.pos = double(pos);
    encoder_table.steer = double(steerEncoder);
end

function gpsData = getGpsData(bag)
    geodetic_sel = select(bag, "Topic", "/pvtgeodetic");
    geodeticData = readMessages(geodetic_sel);
    
    MessageList = bag.MessageList;
    filteredMessageList = MessageList(MessageList.Topic == '/pvtgeodetic', :);
    gpsData.time = double(filteredMessageList{:, 1});

    lat = cellfun(@(msg) msg.latitude, geodeticData, 'UniformOutput', true);
    lon = cellfun(@(msg) msg.longitude, geodeticData, 'UniformOutput', true);
    h = cellfun(@(msg) msg.height, geodeticData, 'UniformOutput', true);
    wgs84 = wgs84Ellipsoid('meter');
    
    [x,y,z] = geodetic2ecef(wgs84, rad2deg(lat), rad2deg(lon), h);
    
    refLat = rad2deg(lat(1));
    refLon = rad2deg(lon(1));
    refH = (1);
    
    [e, n, u] = ecef2enu(x, y, z, refLat, refLon, refH, wgs84);
    gpsData.e = e;
    gpsData.n = n;
    gpsData.u = u;

    gpsData.refLat = refLat;
    gpsData.refLong = refLon;
    gpsData.refH = refH;

    gpsData.yaw = calculateStartYaw([e, n, u], 2);
end

function start_yaw = calculateStartYaw(enu_coords, min_distance)
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
            start_yaw = atan2(delta_y, delta_x);
            return;
        end
    end

    error('No points found more than the specified distance apart.');
end
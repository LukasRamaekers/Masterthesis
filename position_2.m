[file, path] = uigetfile('*', 'Select a File', 'C:\Users\lukas\Documents\MP_v2\rosbags');
bag = ros2bagreader(path);
encoder_sel = select(bag, "Topic", "/encoder_data");
encoderData = readMessages(encoder_sel);
[encoders, steer_encoder] = getPos(encoderData);

MessageList = bag.MessageList;
filteredMessageList = MessageList(MessageList.Topic == '/encoder_data', :);
time = filteredMessageList{:, 1};
encoder_data = [time, encoders]; 

delta_encoders=diff(encoders);

% Oorspronkelijke metingen
r = [0.245, 0.245, 0.23, 0.23];

% beste resultaat wielencoders
% r = [0.249999526728281,0.233834343351581,0.230000000000000,0.230000000000000];

% r= [0.200000015936536,0.200000008389164,0.200000007090777,0.200000008375007];

% Beste shatting stuurencoder
% r = [0.240000699279350,0.220000310358042,0.230000000000000,0.230000000000000];

wheel_f=2*pi*[r(1), -r(2), r(3), -r(4)]/1952;
enc2ang=@(x) -0.6711 * (x.^4) + 0.6546 * (x.^3) - 0.3672 * (x.^2) - 1.155 * x + 0.6783;

if any(strcmp('/pvtgeodetic', bag.AvailableTopics.Properties.RowNames))
    [e, n, u] = coordsCalc(bag);
    start_yaw = -calculateStartYaw([e, n, u], 0.5) - 0.3

    figure;
    plot(e, n, 'o-');
    %plot(e(1), n(1), 'rx', 'LineWidth', 5);
    xlabel('Oost (m)', 'FontSize', 11);
    ylabel('Noord (m)', 'FontSize', 11);
    axis equal;
    grid on;
    hold on;
else
    start_yaw = pi/2;
end

delta_pos=delta_encoders.*wheel_f;
avg_delta_pos=mean(delta_pos,2);
%avg_delta_pos = mean(delta_pos(:, 3:4), 2);
steer_angle=enc2ang(steer_encoder/32767);
delta_angle=diff(steer_angle);

L1=1.2;
L2=0.8;

delta_theta_steer=(L2*delta_angle+avg_delta_pos.*sin(steer_angle(1:end-1)))./(L2+L1.*cos(steer_angle(1:end-1)));
theta_steer=-cumsum([start_yaw;delta_theta_steer]);
delta_x_steer=avg_delta_pos.*cos(theta_steer(1:end-1));
delta_y_steer=avg_delta_pos.*sin(theta_steer(1:end-1));
x_steer=cumsum([0;delta_x_steer]);
y_steer=cumsum([0;delta_y_steer]);

plot(x_steer,y_steer, 'b', LineWidth=2)
hold on;

w = 1.5;
d_left_front = delta_pos(:, 1);
d_right_front = delta_pos(:, 2);
d_left_rear = delta_pos(:, 3);
d_right_rear = delta_pos(:, 4);

d_theta_wheel_front = (d_left_front - abs(d_right_front)) / w;
d_theta_wheel_rear = (d_left_rear - abs(d_right_rear)) / w;

d_theta_wheel_avg = (d_theta_wheel_front + d_theta_wheel_rear) / 2;

%disp(d_theta_wheel_avg);

theta_wheel = -cumsum([start_yaw;d_theta_wheel_avg]);
delta_x_wheel = avg_delta_pos.*cos(theta_wheel(1:end-1));
delta_y_wheel = avg_delta_pos.*sin(theta_wheel(1:end-1));
x_wheel=cumsum([0;delta_x_wheel]);
y_wheel=cumsum([0;delta_y_wheel]);

plot(x_wheel,y_wheel, 'r', LineWidth=2)
hold on;

theta_wheel = -cumsum([start_yaw;d_theta_wheel_front]);
delta_x_wheel = avg_delta_pos.*cos(theta_wheel(1:end-1));
delta_y_wheel = avg_delta_pos.*sin(theta_wheel(1:end-1));
x_wheel=cumsum([0;delta_x_wheel]);
y_wheel=cumsum([0;delta_y_wheel]);

plot(x_wheel,y_wheel, 'Color', 'green')
hold on;

theta_wheel = -cumsum([start_yaw;d_theta_wheel_rear]);
delta_x_wheel = avg_delta_pos.*cos(theta_wheel(1:end-1));
delta_y_wheel = avg_delta_pos.*sin(theta_wheel(1:end-1));
x_wheel=cumsum([0;delta_x_wheel]);
y_wheel=cumsum([0;delta_y_wheel]);

plot(x_wheel,y_wheel, 'Color', 'black')
legend("ENU coordinaten", "stuurencoder", "wielencoders 4 wielen", "wielencoders voorste wielen", "wielencoders achterste wielen");
axis equal
xlabel("x (m)", 'FontSize', 11)
ylabel("y (m)", 'FontSize', 11)
grid on


function [pos, steerEncoder] = getPos(encoderData)
    pos = cellfun(@(msg) msg.pos, encoderData, 'UniformOutput', false);
    pos = cell2mat(pos')';
    pos = double(pos);

    steerEncoder = cellfun(@(msg) msg.steer, encoderData, 'UniformOutput', true);
    steerEncoder = double(steerEncoder);
end

function [e, n, u] = coordsCalc(bag)
    geodetic_sel = select(bag, "Topic", "/pvtgeodetic");
    geodeticData = readMessages(geodetic_sel);
    
    lat = cellfun(@(msg) msg.latitude, geodeticData, 'UniformOutput', true);
    lon = cellfun(@(msg) msg.longitude, geodeticData, 'UniformOutput', true);
    h = cellfun(@(msg) msg.height, geodeticData, 'UniformOutput', true);
    wgs84 = wgs84Ellipsoid('meter');
    
    MessageList = bag.MessageList;
    filteredMessageList = MessageList(MessageList.Topic == '/pvtgeodetic', :);
    time_gnss = filteredMessageList{:, 1};
    gnss_data = [time_gnss, lat, lon, h]; 
    
    [x,y,z] = geodetic2ecef(wgs84, rad2deg(lat), rad2deg(lon), h);
    
    refLat = rad2deg(lat(1));
    refLon = rad2deg(lon(1));
    refH = (1);
    
    [e, n, u] = ecef2enu(x, y, z, refLat, refLon, refH, wgs84);
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

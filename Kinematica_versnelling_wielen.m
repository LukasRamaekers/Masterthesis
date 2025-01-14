[file, path] = uigetfile('*', 'Select a File', 'C:\Users\lukas\Documents\MP_v2\rosbags');
bag = ros2bagreader(path);
encoder_sel = select(bag, "Topic", "/encoder_data");

encoderData = readMessages(encoder_sel);
leftFrontPosValue = cellfun(@(msg) msg.pos(1), encoderData, 'UniformOutput', true);
rightFrontPosValue = cellfun(@(msg) msg.pos(2), encoderData, 'UniformOutput', true);
leftRearPosValue = cellfun(@(msg) msg.pos(3), encoderData, 'UniformOutput', true);
rightRearPosValue = cellfun(@(msg) msg.pos(4), encoderData, 'UniformOutput', true);

MessageList = bag.MessageList;
filteredMessageList = MessageList(MessageList.Topic == '/encoder_data', :);

encoders = table( ...
    filteredMessageList{:, 1}, ... % Extract the first column from filteredMessageList
    leftFrontPosValue, ...
    rightFrontPosValue, ...
    leftRearPosValue, ...
    rightRearPosValue, ...
    'VariableNames', {'Timestamp', 'LeftFront', 'RightFront', 'LeftRear', 'RightRear'} ...
);

timestamps = encoders.Timestamp;
timestamps_rel = timestamps - timestamps(1);
wheel_encoders = encoders{:, 2:5}; 
wheel_f = 2 * pi * [0.245 0.245 0.23 0.23] / 1952;

% cutoff = timestamps <= 1731920000;
% timestamps = timestamps(cutoff);
% wheel_encoders = wheel_encoders(cutoff, :);

delta_encoders = diff(wheel_encoders);
delta_distance = double(delta_encoders) .* wheel_f;

avg_delta_d = mean(delta_distance, 2);
delta_t = diff(timestamps);

v = avg_delta_d ./ delta_t; 
v_smooth = smooth(timestamps(2:end), v, 0.1, 'loess');

fit_speed = fit(timestamps(2:end), v_smooth, 'smoothingspline');
fitted_v = fit_speed(timestamps(2:end));

fitted_v_diff = diff(fitted_v);
a = fitted_v_diff ./delta_t(2:end);
a_smooth = smooth(timestamps(3:end), a, 0.1, 'loess');

positive_speed_indices = fitted_v(1:end-1) > 0; % Exclude the last point for indexing
filtered_a_smooth = a_smooth(positive_speed_indices);

max_a = max(filtered_a_smooth);

disp(max_a);

figure;
subplot(3, 1, 1);
plot(timestamps_rel(1:1000),wheel_encoders(1:1000,:), '-x');
%title('delta position vs. time (avergae of 4 wheels)');
xlabel('Time (s)', 'FontSize', 11);
ylabel('Encoderwaarde', 'FontSize', 11);

subplot(3, 1, 2);
plot(timestamps_rel(2:1000),v_smooth(1:999,1), '-x');
%title('velocity vs. time (avergae of 4 wheels)');
xlabel('Tijd (s)', 'FontSize', 11);
ylabel('v (m/s)', 'FontSize', 11);

subplot(3, 1, 3);
plot(timestamps_rel(3:1000),a_smooth(1:998,1), '-x');
%title('Acceleration vs. time (avergae of 4 wheels)');
xlabel('Tijd (s)', 'FontSize', 11);
ylabel('a (m/s^2)', 'FontSize', 11);


steerEncoder = cellfun(@(msg) msg.steer, encoderData, 'UniformOutput', true);
steerEncoder = double(steerEncoder);

enc2ang=@(x) -0.6711 * (x.^4) + 0.6546 * (x.^3) - 0.3672 * (x.^2) - 1.155 * x + 0.6783;

steer_angle = enc2ang(steerEncoder/32767);
steer_angle = -steer_angle * 180 / pi;

steer_angle = smooth(timestamps, steer_angle, 0.1, 'loess'); % Of gebruik 'sgolay' als alternatief

delta_steer = diff(steer_angle);

v_steer = delta_steer./delta_t(1:end);
v_steer_smooth = smooth(timestamps(2:end), v_steer, 0.1, 'loess');

fit_v_steer = fit(timestamps(2:end), v_steer_smooth, 'smoothingspline');
fitted_v_steer = fit_v_steer(timestamps(2:end));
delta_v_steer = diff(fitted_v_steer);

a_steer = delta_v_steer./delta_t(2:end);
a_steer_smooth = smooth(timestamps(3:end), a_steer, 0.1, 'loess');

max_a_steer = max(abs(a_steer_smooth));
disp(max_a_steer);

figure;
subplot(3, 1, 1);
plot(timestamps_rel(1:end),angle_deg(:,1), '-x');
%title('Stuurhoek (rad)');
xlabel('Tijd (s)', 'FontSize', 11);
ylabel('Stuurhoek (graden)', 'FontSize', 11);

subplot(3, 1, 2);
plot(timestamps_rel(2:end),v_steer_smooth(:,1), '-x');
%title('omega (rad/s)');
xlabel('Tijd (s)', 'FontSize', 11);
ylabel('Hoeksnelheid (graden/s)', 'FontSize', 11);

subplot(3, 1, 3);
plot(timestamps_rel(3:end),a_steer_smooth(:,1), '-x');
%title('Acceleration vs. time (steer encoder)');
xlabel('Tijd (s)', 'FontSize', 11);
ylabel('Hoekversnelling (graden/s^2)', 'FontSize', 11);

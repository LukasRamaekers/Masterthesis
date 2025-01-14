function encoder_result = Simulation_model(linear_speed, desired_angle, dt, reset, finished)
    % Parameters
    L1 = 1.2; L2 = 0.8;
    W1 = 1.5; W2 = 1.5;
    max_phi = deg2rad(35);
    max_steering_accel = 16.65;
    max_steering_speed = 10;
    max_speed = 0.5;
    max_accel = 1.26;
    dt = 0.01;
    wheel_radius = [0.245, 0.245, 0.23, 0.23];
    encoder_ticks_per_rev = 1952;
    wheel_noise_std = 0.01;
    
    % Hydraulic piston parameters
    min_elongation = 65.5;
    max_elongation = 90;
    min_encoder_value = 2176;
    max_encoder_value = 29568;
    
    
    % Persistent Variables for State
    persistent x y theta v_current steering_angle steering_angle_rate front_left_ticks front_right_ticks rear_left_ticks rear_right_ticks time_elapsed encoder_data perturbed_wheel_radius
    
    time_elapsed = time_elapsed + dt;

    if (reset)
        % Initial state
        x = 0; y = 0; theta = 0; 
        v_current = 0;
        steering_angle = 0;
        steering_angle_rate = 0;
        encoder_data = [];
        front_left_ticks = 77437;
        front_right_ticks = -87542;
        rear_left_ticks = 93329;
        rear_right_ticks = -91575;
        time_elapsed = 0;
        perturbed_wheel_radius = wheel_radius + randn * wheel_noise_std; 
    end

    % Encoder simulation variables
    piston_encoder_value = ang2enc(steering_angle);
    
    % Control inputs
    
    if linear_speed >= 0
        v_current = min(min(v_current + max_accel * dt, linear_speed), max_speed);
    else
        v_current = max(max(v_current - max_accel * dt, linear_speed), -max_speed);
    end

    % Update steering angle with max acceleration
    angle_error = rad2deg(desired_angle) - steering_angle;
    steering_angle_rate = max(-max_steering_speed, ...
                          min(max_steering_speed, ...
                          steering_angle_rate + max_steering_accel * dt * sign(angle_error)));
    steering_angle = max(-35, min(35, steering_angle + steering_angle_rate * dt));
    
    % disp(steering_angle)

    % Update piston encoder value for the current steering angle
    piston_encoder_value = ang2enc(steering_angle);
        
    % Update state for linear motion
    x = x + v_current * cos(theta) * dt;
    y = y + v_current * sin(theta) * dt;
    
    R_pivot = (L1 + L2) / tan(deg2rad(steering_angle));
    if isinf(R_pivot)
        % If no turning (straight line)
        d_FL = v_current * dt;
        d_FR = v_current * dt;
        d_RL = v_current * dt;
        d_RR = v_current * dt;
        
    else
        omega = v_current / R_pivot;

        % Calculate the turning radii for each wheel
        R_FL = R_pivot - W1/2;
        R_FR = R_pivot + W1/2;
        R_RL = R_pivot - W2/2;
        R_RR = R_pivot + W2/2;
            
        % Calculate wheel distances traveled during this time step
        d_FL = R_FL * omega * dt;
        d_FR = R_FR * omega * dt;
        d_RL = R_RL * omega * dt;
        d_RR = R_RR * omega * dt;
    end
    
    % Update encoder ticks for each wheel
    front_left_ticks = front_left_ticks + (d_FL / (2 * pi * perturbed_wheel_radius(1))) * encoder_ticks_per_rev;
    front_right_ticks = front_right_ticks + (d_FR / (2 * pi * perturbed_wheel_radius(2))) * -encoder_ticks_per_rev;
    rear_left_ticks = rear_left_ticks + (d_RL / (2 * pi * perturbed_wheel_radius(3))) * encoder_ticks_per_rev;
    rear_right_ticks = rear_right_ticks + (d_RR / (2 * pi * perturbed_wheel_radius(4))) * -encoder_ticks_per_rev;

    % Store the data in the encoder_data table
    encoder_data = [encoder_data; time_elapsed, front_left_ticks, front_right_ticks, ...
                    rear_left_ticks, rear_right_ticks, piston_encoder_value, ...
                    steering_angle];
    
    encoder_result.Time = time_elapsed;
    encoder_result.FrontLeft = front_left_ticks;
    encoder_result.FrontRight = front_right_ticks;
    encoder_result.RearLeft = rear_left_ticks;
    encoder_result.RearRight = rear_right_ticks;
    encoder_result.PistonEncoder = piston_encoder_value;
    
    % disp(encoder_result.PistonEncoder);
    % plotCoords(encoder_table);
    if finished
        kinematica(encoder_data)
    end    
end

function plotCoords(table)
    encoders = table{:, 2:5};
    steer_encoder = table{:, 6};
    
    delta_encoders=diff(encoders);
    
    wheel_f=2*pi*[0.245 0.245 0.23 0.23]/195.2;
    
    enc2ang=@(x) -0.6711 * (x.^4) + 0.6546 * (x.^3) - 0.3672 * (x.^2) - 1.155 * x + 0.6783;
    
    delta_pos=delta_encoders.*wheel_f;
    avg_delta_pos=mean(delta_pos,2);
    steer_angle=enc2ang(steer_encoder/32767);
    delta_angle=diff(steer_angle);
    
    L1=1.2;
    L2=0.8;
    
    delta_theta_steer=(L2*delta_angle+avg_delta_pos.*sin(steer_angle(1:end-1)))./(L2+L1*cos(steer_angle(1:end-1)));
    theta_steer=-cumsum([-pi/2;delta_theta_steer]);
    delta_x_steer=avg_delta_pos.*cos(theta_steer(1:end-1));
    delta_y_steer=avg_delta_pos.*sin(theta_steer(1:end-1));
    x_steer=cumsum([0;delta_x_steer]);
    y_steer=cumsum([0;delta_y_steer]);
    
    p1 = plot(x_steer,y_steer, 'b');
    p1.LineWidth = 2;
    axis equal
    xlabel("x [m]")
    ylabel("y [m]")
    grid on
    hold on;

    w = 1.5;
    d_left_front = delta_pos(:, 1);
    d_right_front = delta_pos(:, 2);
    d_left_rear = delta_pos(:, 3);
    d_right_rear = delta_pos(:, 4);
    
    d_theta_wheel_front = (d_left_front - abs(d_right_front)) / w;
    d_theta_wheel_rear = (d_left_rear - abs(d_right_rear)) / w;
    
    d_theta_wheel_avg = (d_theta_wheel_front + d_theta_wheel_rear) / 2;
    
    theta_wheel = -cumsum([-pi/2;-d_theta_wheel_avg]);
    delta_x_wheel = avg_delta_pos.*cos(theta_wheel(1:end-1));
    delta_y_wheel = avg_delta_pos.*sin(theta_wheel(1:end-1));
    x_wheel=cumsum([0;delta_x_wheel]);
    y_wheel=cumsum([0;delta_y_wheel]);
    
    p2 = plot(x_wheel,y_wheel, 'r');
    p2.LineWidth = 2;
    axis equal
    xlabel("x [m]")
    ylabel("y [m]")
    grid on
end

function encoder_value = ang2enc(steering_angle)
    % Polynomial coefficients
    coeffs = [-0.6711, 0.6546, -0.3672, -1.155, (0.6783 - deg2rad(steering_angle))];
    
    % Solve the polynomial equation
    roots_x = roots(coeffs);
    
    % Find the real root within the range [-1, 1]
    real_root = roots_x(imag(roots_x) == 0 & roots_x >= -1 & roots_x <= 1);
    
    if isempty(real_root)
        error("No valid encoder value found for the given angle!");
    end

    % Convert x to encoder value
    x = real_root(1); % Use the first valid root (if multiple)
    encoder_value = round(x * 32767); % Scale by 32767 and round
end

function max_a = kinematica(encoder_table)
    timestamps = encoder_table(:, 1);
    wheel_encoders = encoder_table(:, 2:5); 
    wheel_f = 2 * pi * [0.245 -0.245 0.23 -0.23] / 1952;
    
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
    plot(timestamps(1:end),wheel_encoders(:,1), '-x');
    %title('delta position vs. time (avergae of 4 wheels)');
    xlabel('Time (s)', 'FontSize', 11);
    ylabel('Encoderwaarde', 'FontSize', 11);
    
    subplot(3, 1, 2);
    plot(timestamps(2:end),v_smooth(:,1), '-x');
    %title('velocity vs. time (avergae of 4 wheels)');
    xlabel('Tijd (s)', 'FontSize', 11);
    ylabel('v (m/s)', 'FontSize', 11);
    
    subplot(3, 1, 3);
    plot(timestamps(3:end),a_smooth(:,1), '-x');
    %title('Acceleration vs. time (avergae of 4 wheels)');
    xlabel('Tijd (s)', 'FontSize', 11);
    ylabel('a (m/s^2)', 'FontSize', 11);
    
    
    % steerEncoder = encoder_table(:, 6);
    % steerEncoder = double(steerEncoder);
    % 
    % enc2ang=@(x) -0.6711 * (x.^4) + 0.6546 * (x.^3) - 0.3672 * (x.^2) - 1.155 * x + 0.6783;
    % 
    % steer_angle = enc2ang(steerEncoder/32767);
    % 
    % delta_steer = diff(steer_angle);
    
    steer_angle = encoder_table(:, 7);
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
    
    % angle_deg = steer_angle.*180/pi;
    
    figure;
    subplot(3, 1, 1);
    plot(timestamps(1:end),steer_angle(:,1), '-x');
    %title('Stuurhoek (rad)');
    xlabel('Tijd (s)', 'FontSize', 11);
    ylabel('Stuurhoek (graden)', 'FontSize', 11);
    
    subplot(3, 1, 2);
    plot(timestamps(2:end),v_steer_smooth(:,1), '-x');
    %title('omega (rad/s)');
    xlabel('Tijd (s)', 'FontSize', 11);
    ylabel('Hoeksnelheid (graden/s)', 'FontSize', 11);
    
    subplot(3, 1, 3);
    plot(timestamps(3:end),a_steer_smooth(:,1), '-x');
    %title('Acceleration vs. time (steer encoder)');
    xlabel('Tijd (s)', 'FontSize', 11);
    ylabel('Hoekversnelling (graden/s^2)', 'FontSize', 11);
end
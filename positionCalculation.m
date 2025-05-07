function [current_position, current_yaw, steer_angle] = positionCalculation(encoderData, yaw, reset)
    persistent previous_encoder previous_angle x_previous y_previous cum_theta_wheel
    
    if (reset)
        previous_encoder = [encoderData.FrontLeft, encoderData.FrontRight, encoderData.RearLeft, encoderData.RearRight];
        previous_angle = 0;
        x_previous = 0; 
        y_previous = 0;
        cum_theta_wheel = yaw;
    end

    r = [0.245, 0.245, 0.23, 0.23];
    % r = [0.23, 0.23, 0.23, 0.23];
    
    [encoders, steer_encoder] = getPos(encoderData);
    
    delta_encoders=encoders-previous_encoder;

    previous_encoder = encoders;

    wheel_f=2*pi*[r(1), -r(2), r(3), -r(4)]/1952;
    enc2ang=@(x) -0.6711 * (x.^4) + 0.6546 * (x.^3) - 0.3672 * (x.^2) - 1.155 * x + 0.6783;

    delta_pos=delta_encoders.*wheel_f;
    avg_delta_pos=mean(delta_pos,2);
    steer_angle=enc2ang(steer_encoder/32767);
    delta_angle=steer_angle-previous_angle;

    L1=1.2;
    L2=0.8;
    
    delta_theta_steer=(L2*delta_angle+avg_delta_pos.*sin(steer_angle(1:end-1)))./(L2+L1*cos(steer_angle(1:end-1)));
    theta_steer=-yaw + delta_theta_steer;
    delta_x_steer=avg_delta_pos.*cos(theta_steer);
    delta_y_steer=avg_delta_pos.*sin(theta_steer);
    x_steer=x_previous + delta_x_steer;
    y_steer=y_previous + delta_y_steer;
    
    w = 1.5;
    d_left_front = delta_pos(1);
    d_right_front = -delta_pos(2);
    d_left_rear = delta_pos(3);
    d_right_rear = -delta_pos(4);
    
    d_theta_wheel_front = (d_left_front - abs(d_right_front)) / w;
    d_theta_wheel_rear = (d_left_rear - abs(d_right_rear)) / w;
    
    d_theta_wheel_avg = (d_theta_wheel_front + d_theta_wheel_rear) / 2;
    
    cum_theta_wheel = cum_theta_wheel - d_theta_wheel_avg;
    theta_wheel = cum_theta_wheel;
    delta_x_wheel = avg_delta_pos.*cos(theta_wheel);
    delta_y_wheel = avg_delta_pos.*sin(theta_wheel);
    x_wheel=x_previous + delta_x_wheel;
    y_wheel=y_previous + delta_y_wheel;
    
    x_previous = x_wheel;
    y_previous = y_wheel;

    current_position = [x_wheel, y_wheel];
    current_yaw = theta_wheel;

    % disp(current_yaw);
end

function [encoders, steer_encoder] = getPos(encoderData)
    encoders = [encoderData.FrontLeft, encoderData.FrontRight, encoderData.RearLeft, encoderData.RearRight];
    steer_encoder = encoderData.PistonEncoder;
end
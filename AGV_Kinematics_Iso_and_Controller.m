%% AGV Kinematics

%% Main Simulation loop

function AGV_Kinematics_and_Controller

    clc; close all; clear;

    % 1. Initialize System Components
    % Simulation timing parameters
    dt = 0.01; 

    kinematics = AGV_kinematics();
    cmd = velocity_generator(dt);

    t_final = cmd.time(end);
    time = 0:dt:t_final;
    N = length(time);

    % Initial robot state: [x, y, theta]
    q = [0; 0; 0]; 
    XI = zeros(3, N); % Log state history
    
   % Controller setup: Standardize PID struct fields
    PID_gains       = [10, 0.1, 0.5, 2]; % [Kp, Ki, Kd, i_limit]
    PID_motor_gains = [50, 10, 0.1, 50];

    % Initialize using the exact same struct template
    pid_template   = struct('integral', 0, 'prev_error', 0, 'd_filt', 0);
    turret_PIDs    = repmat(pid_template, 1, 4);
    motor_PIDs     = repmat(pid_template, 4, 2);
    omega_m_actual = zeros(4, 2);
    
    % 2. Initialize 8 motor PID controller structs (4 turrets x 2 wheels)
    for i = 1:4
        for j = 1:2
            motor_PIDs(i, j).integral   = 0;
            motor_PIDs(i, j).prev_error = 0;
        end
    end

    % 2. Main Simulation Loop
    % Inside function AGV_Kinematics_and_Controller
    % Replace section inside "for k = 1:N" loop:
    
    for k = 1:N
        t_curr = time(k);
        
        % Direct continuous interpolation lookup
        Vx = cmd.vx(k);
        Vy = cmd.vy(k);
        W  = cmd.w(k); 
        
        % Run Control Loop to calculate steering rate, motor torques, and update physical states
        [phi_dot_cmd, torque_cmd, turret_PIDs, motor_PIDs, kinematics, omega_m_actual, V_body_act] = ...
            control_loop(Vx, Vy, W, kinematics, turret_PIDs, motor_PIDs, PID_gains, PID_motor_gains, omega_m_actual, dt);
        
        % Theoretical state q integration transformed into World Frame
        theta = q(3);
        R = [cos(theta), -sin(theta), 0;
             sin(theta),  cos(theta), 0;
             0,           0,          1];
         
        q_dot = R * [Vx; Vy; W];
        q = q + q_dot * dt;
        
        robot_pose = [kinematics.pose(1); kinematics.pose(2); kinematics.pose(3)];
        % Log state
        XI(:, k)  = robot_pose;
        XII(:, k) = q;
    end

    % 3. Plotting & Animation
    run_animation(time, XI, dt); 

    run_animation(time, XII, dt); 
    
    plot_tracking_errors(time, XI, XII);

end


%% Kinematics of the system

function kinematics_AGV = AGV_kinematics 

    distance_x = 0.657; % m
    distance_y = 0.18625; % m
    kinematics_AGV.k_gear = 7.58;
    kinematics_AGV.pose = [0; 0; 0];

    kinematics_AGV.turrets_position = [
        distance_x, -distance_y;
        -distance_x, -distance_y;
        -distance_x, distance_y;
        distance_x, distance_y];

    kinematics_AGV.wheel_array = [
        1, 0, -kinematics_AGV.turrets_position(1,2);
        0, 1, kinematics_AGV.turrets_position(1,1);
        1, 0, -kinematics_AGV.turrets_position(2,2);
        0, 1, kinematics_AGV.turrets_position(2,1);
        1, 0, -kinematics_AGV.turrets_position(3,2);
        0, 1, kinematics_AGV.turrets_position(3,1);
        1, 0, -kinematics_AGV.turrets_position(4,2);
        0, 1, kinematics_AGV.turrets_position(4,1)];

    kinematics_AGV.radius_wheel = 0.07; % m
    kinematics_AGV.distance_between_wheels = 0.15; % m

    kinematics_AGV.turret_angles = [0, 0, 0, 0];

end

%% Controller for the AGV

function [u, PID] = PID_controller(PID, Gains, error_val, dt, u_max)
    if nargin < 5
        u_max = Inf; % Default unconstrained
    end

    Kp = Gains(1);
    Ki = Gains(2);
    Kd = Gains(3);
    i_limit = Gains(4);

    % Derivative with low-pass filter
    alpha = 0.1;
    d_raw = (error_val - PID.prev_error) / dt;
    PID.d_filt = alpha * d_raw + (1 - alpha) * PID.d_filt;

    % Proportional + Derivative output
    u_pd = Kp * error_val + Kd * PID.d_filt;
    
    % Unclamped control signal using existing integral state
    u_raw = u_pd + Ki * PID.integral;
    
    % Saturate control signal
    u = min(max(u_raw, -u_max), u_max);
    
    % Anti-windup conditional integration (only accumulate integral if not saturated)
    if u_raw == u
        PID.integral = PID.integral + error_val * dt;
        PID.integral = min(max(PID.integral, -i_limit), i_limit);
    end

    PID.prev_error = error_val;
end

%% Control loop

function [phi_dot_cmd, omega_cmd, turret_PIDs, motor_PIDs, kinematics, omega_m_actual, V_body_act] = ...
    control_loop(Vx, Vy, W, kinematics, turret_PIDs, motor_PIDs, Gains, Gains_motor, omega_m_actual, dt)
    
    velocity = [Vx; Vy; W];

    turrets_velocities = reshape(kinematics.wheel_array * velocity, 2,4)';

    phi_dot_cmd = zeros(4, 1);
    omega_wheels = zeros(4, 2);

    omega_max = 50;

    for i = 1:4
        
        % Extract velocities 
        vx_i = turrets_velocities(i, 1);
        vy_i = turrets_velocities(i, 2);

        v_bar = sqrt(vx_i^2 + vy_i^2);
        raw_phi = atan2(vy_i, vx_i);

        phi_limit = 105 * (pi / 180);
        is_stopped = (abs(Vx) < 1e-3 && abs(Vy) < 1e-3 && abs(W) < 1e-3);

        % Swerve optimization: flip wheel direction if angle exceeds mechanical limit (+/-105 deg)
        hysteresis = 5*pi/180;
        if abs(raw_phi) > phi_limit + hysteresis
            phi_desired = wrapToPi(raw_phi + pi);
            v_bar = -v_bar;
            turret_PIDs(i).integral = 0;      
        else
            phi_desired = raw_phi;
        end

        % Enforce strict physical limit saturation
        phi_desired = min(max(phi_desired, -phi_limit), phi_limit);

        % Actual data of the robot
        phi_actual = kinematics.turret_angles(i);
        error_phi = wrapToPi(phi_desired - phi_actual);
        v_bar = v_bar * max(0, cos(error_phi));


        % Heading PID controllers definition
        phi_dot_max = 3;
        [phi_dot, turret_PIDs(i)] = PID_controller(turret_PIDs(i), Gains, error_phi, dt, phi_dot_max);
        phi_dot_cmd(i) = phi_dot;

        % Differential robot wheel conversion 
        v_left  = v_bar - 0.5 * kinematics.distance_between_wheels * phi_dot;
        v_right = v_bar + 0.5 * kinematics.distance_between_wheels * phi_dot;
        
        omega_left  = v_left  / kinematics.radius_wheel;
        omega_right = v_right / kinematics.radius_wheel;
        kinematics.omega_wheels(i,:) = [omega_left, omega_right];
        omega_cmd_m = [omega_left, omega_right];

        % Reset integrals when stopped to eliminate windup drift
        if is_stopped
            omega_cmd_m = [0, 0];
            turret_PIDs(i).integral = 0;
            turret_PIDs(i).d_filt   = 0;
            motor_PIDs(i, 1).integral = 0;
            motor_PIDs(i, 2).integral = 0;
        end

        % Wheel motor PID controllers
        for j = 1:2 
            error_omega = omega_cmd_m(j) - omega_m_actual(i, j);
            [omega_cmd(i, j), motor_PIDs(i,j)] = PID_controller(motor_PIDs(i,j), Gains_motor, error_omega, dt, omega_max);
        end 

        W_mL  = omega_m_actual(i, 1);
        W_mR  = omega_m_actual(i, 2);
        omega_L = omega_cmd(i, 1);
        omega_R = omega_cmd(i, 2);

        % Stop residual motor creep at rest
        if is_stopped && norm(omega_m_actual(i, :)) < 0.1
            omega_m_actual(i, :) = [0, 0];
        end
    
    
        % Integrate to update physical states for next step
        omega_m_actual(i, 1) = omega_cmd(i,1);
        omega_m_actual(i, 2) = omega_cmd(i,2);

        omega_m_actual(i, 1) = omega_left;
        omega_m_actual(i, 2) = omega_right;

        tau_motor = 0.05; % Constante de tiempo aproximada del motor
        omega_m_actual(i, 1) = omega_m_actual(i, 1) + (omega_cmd(i, 1) - omega_m_actual(i, 1)) * (dt / tau_motor);
        omega_m_actual(i, 2) = omega_m_actual(i, 2) + (omega_cmd(i, 2) - omega_m_actual(i, 2)) * (dt / tau_motor);
        
        phi_dot_act = kinematics.radius_wheel * (omega_m_actual(i, 2) - omega_m_actual(i, 1)) / kinematics.distance_between_wheels;
        kinematics.turret_angles(i) = wrapToPi(phi_actual + phi_dot_act * dt);   % was: phi_actual + phi_dot*dt
    
        % Average wheel speed to get center linear speed v_i
        v_i_act = kinematics.radius_wheel * ((omega_m_actual(i, 1) + omega_m_actual(i, 2)) / 2);
        
        % Store x and y actual components in 8x1 vector for pinv()
        phi_curr = kinematics.turret_angles(i);
        V_turrets_act(2*i - 1) = v_i_act * cos(phi_curr);
        V_turrets_act(2*i)     = v_i_act * sin(phi_curr);

    end
    
    % 1. Solve actual chassis body velocities [Vx_act; Vy_act; W_act] (3x1)
    V_body_act = pinv(kinematics.wheel_array) * V_turrets_act(:);
    
    Vx_act = V_body_act(1);
    Vy_act = V_body_act(2);
    W_act  = V_body_act(3);
    
    % 2. Extract current global orientation theta
    theta = kinematics.pose(3);
    
    % 3. Transform body frame velocities to world frame velocities
    x_dot     = Vx_act * cos(theta) - Vy_act * sin(theta);
    y_dot     = Vx_act * sin(theta) + Vy_act * cos(theta);
    theta_dot = W_act;
    
    % 4. Integrate world frame velocity over dt to update global AGV pose
    kinematics.pose(1) = kinematics.pose(1) + x_dot * dt;     % Global X (m)
    kinematics.pose(2) = kinematics.pose(2) + y_dot * dt;     % Global Y (m)
    kinematics.pose(3) = wrapToPi(kinematics.pose(3) + theta_dot * dt); % Global Theta (rad)



end


%% Warehouse enviroment

function warehouse = warehouse_function()

    warehouse.width = 30; % m
    warehouse.length = 50; % m
    
    % Obstables = [x y width length
    warehouse.obstacles = [
        5 5 10 2;
        5 15 10 2
        25 5 10 2;
        25 15 10 2];

    warehouse.startPosition = [0, 0];
    warehouse.goalPosition = [warehouse.length, warehouse.width];

end

%% Velocity generator
function command = velocity_generator(dt)
    % Your keyframe arrays (regardless of their individual lengths)
    vx_key = [1.0, 0.8, 0.6, 0.4, 0.2, 0.0, -0.2, -0.4, -0.6, -0.8, -1.0, -0.8, -0.6, -0.4, -0.2, 0.0, ...
              0.2, 0.4, 0.6, 0.8, 1.0, 0.6, 0.2, -0.2, -0.6, -1.0, -0.8, -0.6, -0.4, -0.2, 0.0, ...
              0.0, 0.2, 0.4, 0.6, 0.8, 0.8, 0.6, 0.4, 0.2, 0.0, -0.2, -0.4, -0.6, -0.8, -1.0, ...
              -0.8, -0.6, -0.4, -0.2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
          
    vy_key = [0.0, 0.2, 0.4, 0.6, 0.8, 1.0, 0.8, 0.6, 0.4, 0.2, 0.0, -0.2, -0.4, -0.6, -0.8, -1.0, ...
              -0.8, -0.6, -0.4, -0.2, 0.0, 0.2, 0.4, 0.6, 0.8, 1.0, 0.8, 0.6, 0.4, 0.2, 0.0, ...
              0.0, 0.0, 0.0, 0.0, 0.0, 0.2, 0.4, 0.6, 0.8, 1.0, 0.8, 0.6, 0.4, 0.2, 0.0, ...
              -0.2, -0.4, -0.6, -0.8, -1.0, -0.8, -0.6, -0.4, -0.2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
          
    w_key  = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, ...
              0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, ...
              1.0, 0.8, 0.6, 0.4, 0.2, 0.0, 0.0, -0.2, -0.4, -0.6, -0.8, -1.0, -1.0, -0.8, -0.6, -0.4, -0.2, 0.0, ...
              0.0, 0.2, 0.4, 0.6, 0.8, 1.0, 1.0, 0.8, 0.6, 0.4, 0.2];

    % Create individual time axes matched strictly to each vector's length
    t_vx = 0:(length(vx_key) - 1);
    t_vy = 0:(length(vy_key) - 1);
    t_w  = 0:(length(w_key) - 1);

    % Global simulation time vector up to the maximum keyframe index available
    max_t = max([t_vx(end), t_vy(end), t_w(end)]);
    command.time = 0:dt:max_t;
    
    % Interpolate each trajectory independently using its own coordinate map
    command.vx = interp1(t_vx, vx_key, command.time, 'linear', 'extrap');
    command.vy = interp1(t_vy, vy_key, command.time, 'linear', 'extrap');
    command.w  = interp1(t_w,  w_key,  command.time, 'linear', 'extrap');
end

%% --- AGV Animation Plotting ---
function run_animation(t, XI, dt)
    figure('Name', 'AGV Simulation', 'NumberTitle', 'off');

    XI(3,:) = unwrap(XI(3,:));
    
    % Subplot 1: Trajectory
    subplot(2, 1, 1);
    plot(t, XI', 'LineWidth', 1.5);
    xlabel('Time (s)'); ylabel('State Vector');
    legend('x (m)', 'y (m)', '\theta (rad)', 'Location', 'best');
    grid on; title('State vs Time');

    % Subplot 2: 2D Motion Animation
    subplot(2, 1, 2);
    grid on; hold on; axis equal;
    title('AGV Motion Trajectory');
    xlabel('X Position (m)'); ylabel('Y Position (m)');
    
    trajplot = plot(XI(1,1), XI(2,1), '--k', 'LineWidth', 1);
    
    % Draw initial AGV body triangle
    BV = [-0.3, 0, 0.3; -0.2, 0.4, -0.2];
    q0 = XI(:, 1);
    R0 = [cos(q0(3)-pi/2), -sin(q0(3)-pi/2); sin(q0(3)-pi/2), cos(q0(3)-pi/2)];
    IV = R0 * BV;
    bodyplot = fill(IV(1,:) + q0(1), IV(2,:) + q0(2), [0.3, 0.7, 0.9]);

    % Downsampled animation loop for performance
    step_stride = max(1, round(0.05 / dt));
    for n = 1:step_stride:length(t)
        q = XI(:, n);
        x = q(1); y = q(2); theta = q(3);
        
        trajplot.XData = XI(1, 1:n);
        trajplot.YData = XI(2, 1:n);
        
        R = [cos(theta-pi/2), -sin(theta-pi/2); sin(theta-pi/2), cos(theta-pi/2)];
        IV = R * BV;
        bodyplot.XData = IV(1,:) + x;
        bodyplot.YData = IV(2,:) + y;
        
        drawnow limitrate;
    end
end

%% Error plots
% --- Plot Position Tracking Errors ---
function plot_tracking_errors(t, XI, XII)
    % Compute coordinate errors (q - actual)
    err_x = XII(1, :) - XI(1, :);
    err_y = XII(2, :) - XI(2, :);
    err_theta = wrapToPi(XII(3, :) - XI(3, :));
    err_distance = sqrt(err_x.^2 + err_y.^2); % Total Euclidean distance error

    figure('Name', 'Position Tracking Errors', 'NumberTitle', 'off');
    
    % Subplot 1: Component Errors (e_x, e_y, e_theta)
    subplot(2, 1, 1);
    plot(t, err_x, '-b', 'LineWidth', 1.5); hold on;
    plot(t, err_y, '-r', 'LineWidth', 1.5);
    plot(t, err_theta, '-m', 'LineWidth', 1.5);
    yline(0, '--k', 'Alpha', 0.5);
    xlabel('Time (s)'); ylabel('Error');
    legend('e_x = q_x - x_{act} (m)', 'e_y = q_y - y_{act} (m)', 'e_{\theta} = q_{\theta} - \theta_{act} (rad)', 'Location', 'best');
    grid on; title('Component Tracking Errors over Time');

    % Subplot 2: Total Distance Error Magnitude
    subplot(2, 1, 2);
    plot(t, err_distance, '-k', 'LineWidth', 1.5);
    xlabel('Time (s)'); ylabel('Position Error (m)');
    grid on; title('Euclidean Distance Error ||e_{pos}|| = \sqrt{e_x^2 + e_y^2}');
end
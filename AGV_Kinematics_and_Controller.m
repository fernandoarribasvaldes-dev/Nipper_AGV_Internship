%% AGV Kinematics

%% Main Simulation loop

function main_simulation

% Initial conditions
% Initial postion = [x y theta]
initial_position = [0 0 0];

pid.kp = 10;
pid.kd = 0;
pid.ki = 0;

% Simulation
%[t, x] ode45(@(t,x) AGV_kinematics(), [0,60], initial_position); 


end


%% Kinematics of the system

function AGV_kinematics

    distance_x = 0.657; % m
    distance_y = 0.18625; % m

    kinematics_AGV.turrets_position = [
        distance_x, -distance_y;
        -distance_x, -distance_y;
        -distance_x, distance_y;
        distance_x, distance_y];

    kinematics_AGV.wheel_array = [
        1, 0, turrets_position(1,2);
        0, 1, turrets_position(1,1);
        1, 0, turrets_position(2,1);
        0, 1, turrets_position(2,2);
        1, 0, turrets_position(3,1);
        0, 1, turrets_position(3,2);
        1, 0, turrets_position(4,1);
        0, 1, turrets_position(4,2)];

    kinematics_AGV.radius_wheel = 0.07; % m
    kinematics_AGV.distance_between_wheels = 0.15; % m

    kinematics_AGV.turret_angles = [0, 0, 0, 0];

end

%% Controller for the AGV

function PID


end

%% Control loop

function control_loop (Vx, Vy, W, kinematics)

    velocity = [Vx, Vy, W];

    turrets_velocities = dot(kinematics.turrets_postions, velocity);

    wheel_commands = [];

    [rows, columns] = size(turrets_velocities);

    for i = 1:size(turret_velocities, 1)

        % Extract velocities 
        vx_i = turrets_velocities(i, 1);
        vy_i = turrets_velocities(i, 2);

        v_bar = sqrt(vx_i^2, vy_i^2);
        raw_phi = atan2(vy_i, vx_i);


        % Angle protection
        if abs(wrapToPi(raw_phi) > pi/2)
            phi_desired = wrapToPi(raw_phi + pi);
            v_bar = -v_bar;
            
        else
            phi_desired = raw_phi;

        end

        % Actual data of the robot
        phi_actual = kinematics.turret_angles(i);
        error_phi = wrapToPi(phi_desired - phi_actual);


        % Differential robot wheel conversion
        v_left = v_bar - 0.5 * kinematic.distance_between_wheels * phi_dot;
        v_right = v_bar + 0.5 * kinematic.distance_between_wheels * phi_dot;

        omega_left = v_left/kinematics.radius_wheel;
        omega_right = v_right/kinematics.radius_wheel;

    end



end


%% Warehouse enviroment

function warehouse_function

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

function velocity_generator

    command.time = [0 5 10 15 20 25 30 35 40 45 50 55 60];

    command.vx = [1 0 -1 0 1 -1 0 1 -1 0 0];
    command.vy = [0 1 0 -1 1 -1 0 0 0 1 -1];
    command.w  = [0 0 0 0 0 0 1 -1 1 -1 1 -1];

end
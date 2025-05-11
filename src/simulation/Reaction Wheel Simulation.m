% Reaction Wheel Simulation

%{
This script simulates the dynamics of a spacecraft with a reaction wheel. Additionally, it demonstrates the interaction
between the spacecraft and the reaction wheel when a torque is applied to the wheel. The simulation runs for 10 seconds
with a time step of 0.01 seconds. The initial angular velocities of the spacecraft and the reaction wheel are both set to zero.

Summary:
The spacecraft has a moment of inertia of 0.10833 kg*m^2, and the reaction wheel has a moment of inertia of 0.0069121818 kg*m^2.
The simulation runs for 10 seconds with a time step of 0.01 seconds. The initial angular velocities of the spacecraft and
the reaction wheel are both set to zero.

Purpose:
The purpose of this simulation is to demonstrate the interaction between the spacecraft and the reaction wheel when a torque is
applied to the wheel. The torque applied to the wheel is controlled by a simple strategy: a constant torque of 0.5 N*m is applied
for the first 3 seconds, and then the torque is set to zero.

Inputs:
    - Moment of inertia of the spacecraft (I_spacecraft)
    - Moment of inertia of the reaction wheel (I_wheel)
    - Total simulation time (total_time)
    - Time step (dt)

Outputs:
    - Angular velocity of the spacecraft over time (omega_spacecraft_data)
    - Angular velocity of the reaction wheel over time (omega_wheel_data)
%}

function reaction_wheel_simulation
    % Define constants
    I_spacecraft = 0.10833; % Moment of inertia of the spacecraft (kg*m^2)
    I_wheel = 0.0069121818; % Moment of inertia of the reaction wheel (kg*m^2)
    total_time = 10;        % Total time for simulation (seconds)
    dt = 0.01;              % Time step (seconds)

    % Initialize arrays for storing data
    time_steps = 0:dt:total_time;
    omega_spacecraft_data = zeros(size(time_steps));
    omega_wheel_data = zeros(size(time_steps));

    % Initial conditions
    omega_spacecraft = 0; % Initial angular velocity of spacecraft (rad/s)
    omega_wheel = 0;      % Initial angular velocity of reaction wheel (rad/s)

    % Simulation loop
    for i = 1:length(time_steps)
        t = time_steps(i);

        % Calculate torque applied by the reaction wheel
        tau = calculate_torque(t);

        % Update angular velocity of the wheel
        alpha_wheel = tau / I_wheel; % Angular acceleration of the wheel
        omega_wheel = omega_wheel + alpha_wheel * dt;

        % Update angular velocity of the spacecraft
        omega_spacecraft = omega_spacecraft - (tau / I_spacecraft) * dt;

        % Store data for plotting
        omega_spacecraft_data(i) = omega_spacecraft;
        omega_wheel_data(i) = omega_wheel;
    end

    % Plot results
    figure;
    plot(time_steps, omega_spacecraft_data, 'b-', 'LineWidth', 2);
    hold on;
    plot(time_steps, omega_wheel_data, 'r--', 'LineWidth', 2);
    hold off;
    xlabel('Time (s)');
    ylabel('Angular Velocity (rad/s)');
    legend('Spacecraft Angular Velocity', 'Wheel Angular Velocity');
    title('Spacecraft and Wheel Angular Velocity vs Time');
end

function tau = calculate_torque(t)
    % Define the control strategy for the reaction wheel torque
    if t < 3
        tau = 0.5; % Constant torque (N*m)
    else
        tau = 0;   % No torque
    end
end
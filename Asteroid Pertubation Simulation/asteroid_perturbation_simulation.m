function asteroid_perturbation_simulation()
    % Constants
    G = 6.67430e-11; % Gravitational constant
    deltaTime = 1; % Time step for the simulation in seconds
    totalSimulationTime = 3600; % Total simulation time in seconds
    initialQuaternion = [1, 0, 0, 0]; % Initial quaternion (no rotation)

    % Mass and distance parameters
    massAsteroid = 2.58945e10; % Mass of asteroid in kg
    massObject = 0.8; % Mass of the object/spacecraft in kg
    distance = 0.8; % Distance between asteroid and object in meters

    % Inertia matrix components for a 6U CubeSat
    dimensions = [0.3, 0.2, 0.1]; % Dimensions in meters
    m = massObject; % Mass in kg
    Ixx = m/12 * (dimensions(2)^2 + dimensions(3)^2);
    Iyy = m/12 * (dimensions(1)^2 + dimensions(3)^2);
    Izz = m/12 * (dimensions(1)^2 + dimensions(2)^2);
    InertiaMatrix = diag([Ixx, Iyy, Izz]); % Diagonal inertia matrix

    % Time vector
    timeVector = 0:deltaTime:totalSimulationTime;

    % Initialize quaternion data for plotting
    quaternionData = zeros(length(timeVector), 4); % Initialize with zeros

    % Initial conditions
    angularVelocity = [0, 0, 0];
    quaternion = initialQuaternion;

    % Main simulation loop
    for t = 1:length(timeVector)
        % Oscillating lever arm dynamics
        leverArmX = 0.05 * sin(2 * pi * t / 3600);
        leverArmY = 0.05 * cos(2 * pi * t / 3600);
        leverArmZ = 0.1 * sin(2 * pi * t / 1800);

        % Calculate gravitational force
        F_grav = G * massAsteroid * massObject / distance^2;

        % Compute torques for each axis
        torque = [leverArmX * F_grav, leverArmY * F_grav, leverArmZ * F_grav];

        % Update angular velocity based on torque and inertia
        angularAcceleration = InertiaMatrix \ torque';
        angularVelocity = angularVelocity + angularAcceleration' * deltaTime;

        % Update quaternion using the derived angular velocity
        omega_quat = [0, angularVelocity];
        quaternion_dot = 0.5 * quatmultiply(quaternion, omega_quat);
        quaternion = quaternion + quaternion_dot * deltaTime;

        % Normalize quaternion to maintain unit length
        quaternion = quaternion / norm(quaternion);

        % Store quaternion data
        quaternionData(t, :) = quaternion;
    end

    % Convert to timeseries object
    ts = timeseries(quaternionData, timeVector, 'Name', 'Quaternion');

    % Save data for use in Simulink
    save('quaternionData.mat', 'ts', '-v7.3');
end

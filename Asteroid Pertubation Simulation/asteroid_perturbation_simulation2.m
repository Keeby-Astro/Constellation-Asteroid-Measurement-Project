function asteroid_perturbation_simulation2()
    % Constants
    G = 6.67430e-11; % Gravitational constant in m^3 kg^-1 s^-2
    deltaTime = 1; % Time step for the simulation in seconds
    totalSimulationTime = 3600; % Total simulation time in seconds
    initialQuaternion = [1, 0, 0, 0]; % Initial quaternion (no rotation), as row vector
    minDistance = 200; % Minimum distance in meters
    maxDistance = 800; % Maximum distance in meters
    numDistances = 200; % Number of distance steps
    asteroidMasses = [2.58945e10, 5.45147e10, 3.03797e10, 6.39573e10]; % Masses for A1_S, A1_E, B1_S, B1_E
    asteroidLabels = {'A1_S', 'A1_E', 'B1_S', 'B1_E'}; % Labels for asteroids

    % Mass of the object/spacecraft
    massObject = 0.8; % Mass in kg
    distances = linspace(minDistance, maxDistance, numDistances); % Array of distances

    % Inertia matrix components based on a 6U CubeSat dimensions
    dimensions = [0.3, 0.2, 0.1]; % meters
    m = massObject; % mass in kg
    Ixx = m/12 * (dimensions(2)^2 + dimensions(3)^2);
    Iyy = m/12 * (dimensions(1)^2 + dimensions(3)^2);
    Izz = m/12 * (dimensions(1)^2 + dimensions(2)^2);
    InertiaMatrix = diag([Ixx, Iyy, Izz]); % Diagonal inertia matrix

    % Loop over each asteroid mass
    for massIdx = 1:length(asteroidMasses)
        massAsteroid = asteroidMasses(massIdx); % Current asteroid mass
        quaternionData = zeros(totalSimulationTime / deltaTime + 1, 4, numDistances);
        timeVector = 0:deltaTime:totalSimulationTime;

        % Loop over each distance
        for idx = 1:numDistances
            % Current distance for this simulation
            distance = distances(idx);

            % Reset initial conditions for each distance
            angularVelocity = [0, 0, 0];
            quaternion = initialQuaternion;

            % Simulation loop
            for t = 1:length(timeVector)
                % Oscillating lever arm on Cartesian axes
                leverArmX = 0.05 * sin(2 * pi * t / 3600);
                leverArmY = 0.05 * cos(2 * pi * t / 3600);
                leverArmZ = 0.1 * sin(2 * pi * t / 1800);

                % Calculate gravitational force at current distance
                F_grav = G * massAsteroid * massObject / distance^2;

                % Calculate torque for each axis
                torque = [leverArmX * F_grav, leverArmY * F_grav, leverArmZ * F_grav];

                % Update angular velocity based on torque and inertia
                angularAcceleration = InertiaMatrix \ torque'; % Solve for angular acceleration
                angularVelocity = angularVelocity + angularAcceleration' * deltaTime;

                % Update quaternion
                omega_quat = [0, angularVelocity];
                quaternion_dot = 0.5 * quatmultiply(quaternion, omega_quat);
                quaternion = quaternion + quaternion_dot * deltaTime;

                % Normalize quaternion to avoid drift
                quaternion = quaternion / norm(quaternion);

                % Store quaternion for plotting
                quaternionData(t, :, idx) = quaternion;
            end
        end

        % Plot the quaternion components in 3D
        figure;
        for q_idx = 1:4
            subplot(2, 2, q_idx);
            mesh(timeVector, distances, squeeze(quaternionData(:, q_idx, :))');
            xlabel('Time (s)');
            ylabel('Distance (m)');
            zlabel(['Quaternion Component q_', num2str(q_idx - 1)]);
            title(['Quaternion Component q_', num2str(q_idx - 1), ' for ', asteroidLabels{massIdx}]);
        end
    end
end

function q = quatmultiply(q1, q2)
    % Quaternion multiplication
    w1 = q1(1);
    x1 = q1(2);
    y1 = q1(3);
    z1 = q1(4);

    w2 = q2(1);
    x2 = q2(2);
    y2 = q2(3);
    z2 = q2(4);

    % Resulting quaternion
    q = [w1*w2 - x1*x2 - y1*y2 - z1*z2, ...
         w1*x2 + x1*w2 + y1*z2 - z1*y2, ...
         w1*y2 - x1*z2 + y1*w2 + z1*x2, ...
         w1*z2 + x1*y2 - y1*x2 + z1*w2];
end
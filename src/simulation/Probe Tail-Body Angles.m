% Tail-Body Angles for CubeSat Dynamics

%{
The function tb_angles() is used to simulate the Tail-Body angles for a CubeSat.

Purpose:
    - To simulate the Tail-Body angles for a CubeSat for two different cases.
    - To plot the Tail-Body angles for the two cases.

Inputs:
    - I: Inertial Matrix of the CubeSat.
    - tspan: Time span for the simulation.
    - y0_1: Initial conditions for Case 1.
    - y0_2: Initial conditions for Case 2.

Outputs:
    - Plot of the Tail-Body angles for the two cases.
%}

% Function to simulate the Tail-Body angles for a CubeSat
function tb_angles()
    % Given Inertial Matrix
    I = [10, 0, -0.1; 
         0, 5, 0; 
         -0.1, 0, 10];

    % Time Span for 30 seconds
    tspan = [0 30];

    % Case 1
    y0_1 = [0; 0; 0; 0; 0; 10];
    % ODE 45 for Case 1
    [t1, y1] = ode45(@(t,y) aircraft_dynamics(t, y, I, [0; 0; 0]), tspan, y0_1);

    % Case 2
    y0_2 = [pi/4; pi/3; 0; -1; 0; 10];
    % ODE 45 for Case 2
    [t2, y2] = ode45(@(t,y) aircraft_dynamics(t, y, I, [0; 1; 0]), tspan, y0_2);
    
    % Plot results
    % Case 1
    subplot(2,1,1);
    plot(t1,y1(:,1:6));
    title('T-B Angles for Case 1');
    xlabel('Time (s)');
    ylabel('Angles (rad)');
    legend('Phi', 'Theta', 'Psi');


    % Case 2
    subplot(2,1,2);
    plot(t2,y2(:,1:6));
    title('T-B Angles for Case 2');
    xlabel('Time (s)');
    ylabel('Angles (rad)');
    legend('Phi', 'Theta', 'Psi');
end

% Function to calculate the dynamics of the CubeSat
function dydt = aircraft_dynamics(t, y, I, M_ext)
    % Extracting states
    phi = y(1);     % Roll angle
    theta = y(2);   % Pitch angle
    psi = y(3);     % Yaw angle
    p = y(4);       % Rate of change of phi
    q = y(5);       % Rate of change of theta
    r = y(6);       % Rate of change of psi
    M = M_ext;      % Use the passed external moment directly from cases

    % Equations of Motion
    % Angles
    d_angles = [p + q*sin(phi)*tan(theta) + r*cos(phi)*tan(theta);
                q*cos(phi) - r*sin(phi);
                q*sin(phi)/cos(theta) + r*cos(phi)/cos(theta)];
    % Rates
    d_rates = inv(I) * (M - cross([p; q; r], I*[p; q; r]));

    % Time derivative
    dydt = [d_angles; d_rates];
end
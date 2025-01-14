% Super-Twist Algorithm (STA) with Integral Sliding Mode Control (ISMC) for a 3D system with disturbance
%{
    The system is a 3D system with a disturbance term d = 2*sin(t) + 3

    The system is defined by the following differential equations:
        r1_dot = r2
        r2_dot = r3
        r3_dot = -k*sin(acos(-r3/k))*r3*tan(asin(-r2)) + k*sin(acos(-r3/k))*u
        zeta_dot = u + d - u_nom
        theta_dot = -a6*sign(zeta)
        where k = 1.0/56.7, f = -k*sin(acos(-r3/k))*r3*tan(asin(-r2)), g = k*sin(acos(-r3/k)),
        d = 2*sin(t) + 3, u_nom = -a1*sign(r1)*(abs(r1))^(b1) - a2*sign(r2)*(abs(r2))^(b2) - a3*sign(r3)*(abs(r3))^(b3),
        u_stc = -a5*(zeta^(1/2))*sign(zeta) + theta, u = u_nom + u_stc, and i2 = (u - f) / g

    The control law is defined by the following equations:
        u_nom = -a1*sign(r1)*(abs(r1))^(b1) - a2*sign(r2)*(abs(r2))^(b2) - a3*sign(r3)*(abs(r3))^(b3)
        u_stc = -a5*(zeta^(1/2))*sign(zeta) + theta
        u = u_nom + u_stc
        
    The control input to the system is given by:
        i2 = (u - f) / g
%}
% Define the main function
function ismc_sta
    % Define the time span for the simulation
    tspan = [0 10];
    % Initial conditions for the state variables
    r0 = [0.3; 0.5; 0.2; 0; 0];

    % Solve the differential equations using ode45
    [t,r] = ode45(@myodefun,tspan,r0);
    % Extract the Zeta and Theta states from the solution
    Zeta = r(:,4);
    Theta = r(:, 5);

    % Define constants for the control law
    a1 = 15;
    a2 = 23;
    a3 = 9;
    b1 = 1/4;
    b2 = 1/3;
    b3 = 1/2;
    a5 = 0.13;

    % Compute the nominal control input
    U_nom = -a1*sign(r(:,1)).*(abs(r(:,1))).^(b1) - a2*sign(r(:,2)).*(abs(r(:,2))).^(b2) - a3*sign(r(:,3)).*(abs(r(:,3))).^(b3);
    % Compute the sliding mode control input
    U_stc = -a5*sign(Zeta).*(abs(Zeta)).^(1/2) + Theta;
    % Total control input
    U = U_nom + U_stc;

    % Plot the first state variable over time
    plot(t, r(:,1), 'b')
    title('ISMC + STA With Disturbance')
    xlabel('Time (t) (secs.)')
    ylabel('Distance away from plane (r1 = y) (cms.)')

    % Plot the second and third state variables over time
    figure
    plot(t, r(:,2), 'r', t, r(:,3), 'b')
    title('ISMC + STA With Disturbance')
    xlabel('Time (t) (secs.)')
    ylabel('States denoting rotations (r2 and r3)')
    legend('r2 state', 'r3 state')

    % 3D plot of the first three state variables
    figure
    plot3(r(:,1), r(:,2), r(:,3))
    xlabel('r1')
    ylabel('r2')
    zlabel('r3')

    % Plot the sliding surface over time
    figure
    plot(t, Zeta, 'b')
    title('ISMC + STA With Disturbance')
    xlabel('Time (t) (secs.)')
    ylabel('Sliding Surface')

    % Plot the total control input over time
    figure
    plot(t, U, 'b')
    title('ISMC + STA With Disturbance')
    xlabel('Time (t) (secs.)')
    ylabel('Control Input')

    % Plot the nominal control input over time
    figure
    plot(t, U_nom, 'b')
    title('ISMC + STA With Disturbance')
    xlabel('Time (t) (secs.)')
    ylabel('Nominal Control Input')
    hold on
    axis image

    % Nested function defining the system of differential equations
    function xdot = myodefun(t,x)
        % Define constants for the system dynamics
        k = 1.0/56.7;
        f = -k*sin(acos(-x(3)/k))*x(3)*tan(asin(-x(2)));
        g = k*sin(acos(-x(3)/k));
        d = 2*sin(t) + 3;
        a1 = 15;
        a2 = 23;
        a3 = 9;
        b1 = 1/4;
        b2 = 1/3;
        b3 = 1/2;
        UM = 6;

        % Compute the nominal control input
        u_nom = -a1*sign(x(1))*(abs(x(1)))^(b1) - a2*sign(x(2))*(abs(x(2)))^(b2) - a3*sign(x(3))*(abs(x(3)))^(b3);
        % Extract the Zeta and Theta states
        zeta = x(4);
        theta = x(5);
        a5 = 0.13;
        a6 = 300;

        % Compute the sliding mode control input
        u_stc = -a5*(zeta^(1/2))*sign(zeta) + theta;
        % Total control input
        u = u_nom + u_stc;
        % Compute the control input to the system
        i2 = (u - f) / g;

        % Define the system of differential equations
        xdot(1,1) = x(2);
        xdot(2,1) = x(3);
        % Without Disturbance
        % xdot(3,1) = f + g*i2;
        % With Disturbance d = 2*sin(t) + 3
        xdot(3,1) = f + g*i2 + d;
        xdot(4,1) = u + d - u_nom;
        xdot(5,1) = -a6*sign(zeta);

        % If conditioned:
        % If you take big enough UM (say 10 or above), it will be same as unconditioned case
        % if abs(u_stc) > UM
        %   xdot(5,1) = -u_stc;
        % else
        %   xdot(5,1) = -a6*sign(zeta);
        % end
    end
end
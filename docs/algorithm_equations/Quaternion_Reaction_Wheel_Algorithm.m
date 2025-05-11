%--------------------------------------------------------------------------
%                                                                         -
% QUATERNION REACTION WHEEL ALGORITHM                                     -
%                                                                         -
%--------------------------------------------------------------------------
% This is a function file list for the purpose of compiling all necessary
% equations for Probe CubeSat Attitude and Determination Dynamics.
% The zero constants will be varying in accordance to IMU/Motor encoding.

% Purpose: This file contains the necessary equations for the Probe CubeSat
% Attitude and Determination Dynamics. The equations are organized by
% category and are intended to be used as a reference for the Probe CubeSat
% Attitude and Determination Dynamics.

% Summary of Equations:
%   1. Body Frame Rotation
%   2. New Frame Rotation
%   3. Mapping Rotational Relation
%   4. General Mapping Relation
%   5. Inertial Rotational Frame
%   6. Quaternion Scalar Kinematics
%   7. Quaternion Vector Kinematics
%   8. Compact Quaternion Matrix
%   9. Eulerian-based Quaternion
%  10. Cross Product Matrix
%  11. Redundant Kinetic Equations
%  12. Angular Momentum Body Frame
%  13. Circle Plate Angular Momentum
%  14. Angular Momentum Relative Frame
%  15. Angular Momentum Mapping Relation
%  16. Total Angular Momentum
%  17. Total Angular Momentum Transformation
%  18. Equivalent Moment of Inertia Matrix
%  19. Force Distribution Matrix (FDM)
%  20. Axial Moment of Inertia Matrix
%  21. Total Angular Moment Inertial Relation
%  22. Euler Equation of Motion
%  23. External Control Torque
%  24. Reaction Torque
%  25. Angular Momentum Torque Relation
%  26. Torque Rotation Matrix
%  27. FDM Geometric Constraint Definition 1
%  28. FDM Geometric Constraint Definition 2
%  29. Geometrically Constrained Force Distribution Model
%  30. Static Optimization
%  31. Static Optimization Subjections
%  32. Lagrangian Definition
%  33. Static Optimization First-Order Condition Definition 1
%  34. Static Optimization First-Order Condition Definition 2
%  35. Static Optimization First-Order Implied Condition
%  36. Augmented Distribution Matrix
%  37. Inverse Mapping Augmented Distribution Matrix
%  38. Linear Dynamic Reaction Torque
%  39. Control Input Voltage
%  40. Closed-Loop Sliding Dynamics
%  41. Closed-Loop Sliding Dynamics State Space Form
%  42. Closed-Loop Sliding Dynamics State Space Form Implied Condition
%  43. Variable Transformation
%  44. Variable Transformation Time Derivative
%  45. Variable Transformation Time Derivative Matrix Form
%  46. Disturbance Transformation
%--------------------------------------------------------------------------

%--------------------------------------------------------------------------
% GEOMETRY CONFIGURATION ANALYSIS
%--------------------------------------------------------------------------
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Angles
beta_x  = 0;
beta_y  = 0;
beta_z  = 0;
beta    = [ beta_x;  beta_y;  beta_z];
alpha_x = 0;
alpha_y = 0;
alpha_z = 0;
alpha   = [alpha_x; alpha_y; alpha_z];
omega_x = 0;
omega_y = 0;
omega_z = 0;
omega = [omega_x; omega_y; omega_z];
% Coordinates
x = 0;
y = 0;
z = 0;
v = [x, y, z];
q1 = 0;
q2 = 0;
q3 = 0;
q4 = 0;
q0 = 0;
q = quaternion(q1,q2,q3,q4);
Q = [q0; q1; q2; q3];
% Operations
a1 = 0;
a2 = 0;
a3 = 0;
a = [a1; a2; a3];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Eq.1 Body Frame Rotation
function BodyFrameRotation
% Define the angle beta in radians
R = [cos(beta) sin(beta) 0;
    -sin(beta) cos(beta) 0;
    0 0 1];
v = [x; y; z];
v_prime = R * v;
x_prime = v_prime(1);
y_prime = v_prime(2);
z_prime = v_prime(3);
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Eq. 2 New Frame Rotation
function NewFrameRotation
R_prime = [cos(-alpha) 0 -sin(-alpha);
    0 1 0;
    sin(-alpha) 0 cos(-alpha)];
v_double_prime = R_prime * v_prime;
x_double_prime = v_double_prime(1);
y_double_prime = v_double_prime(2);
z_double_prime = v_double_prime(3);
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Eq. 3 Mapping Rotational Relation
function MappingRotationalRelation
R_mapping = R * R_prime * v;
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Eq. 5 Inertial Rotational Frame
function InertialRotationalFrame
beta_i = zeros(1, 4);
for i = 1:4
    beta_i(i) = beta + 0.5 * (i - 1) * pi;
end
R_rotation_i = cell(1, 4);
for i = 1:4
    R_i = [cos(beta_i(i)), sin(beta_i(i)), 0;
           -sin(beta_i(i)), cos(beta_i(i)), 0;
            0,               0,              1];
    R_rotation_i{i} = R_prime * [R_i, [0; 0; 1]];
end
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Eq. 4 General Mapping Relation
function GeneralMappingRelation
v_i = R_rotation_i * v;
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Eq. 6 Quaternion Scalar Kinematics
function QuaternionScalarKinematics
q0 = q(1);
q_vec = q(2:4);
q0_dot = -0.5 * q_vec' * omega;
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Eq. 7 Quaternion Vector Kinematics
function QuaternionVectorKinematics
q_dot = [q0_dot; -0.5 * (q0 * omega + cross(q_vec, omega))];
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Eq. 8 Compact Quaternion Matrix
function CompactQuaternionMatrix
Q_dot = 0.5 * E_Q * omega;
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Eq. 9 Eulerian-based Quaternion
function EulerianBasedQuaternion
E_Q = [ -Q(2), -Q(3), -Q(4);
         Q(1), -Q(4),  Q(3);
         Q(4),  Q(1), -Q(2);
        -Q(3),  Q(2),  Q(1)];
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Eq. 10 Cross Product Matrix
function CrossProductMatrix
a_x = [ 0,     -a(3),  a(2);
        a(3),   0,    -a(1);
       -a(2),  a(1),   0 ];
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Eq. 11 Redundant Kinetic Equations
function RedundantKineticEquations
Omega_i = 0;
ex_i = [1; 0; 0];
Ri = eye(3);
omega_w_i = Ri * omega + Omega_i * ex_i;
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Eq. 12 Angular Momentum Body Frame
function AngularMomentumBodyFram
I = [Ixx, Ixy, Ixz;
     Ixy, Iyy, Iyz;
     Ixz, Iyz, Izz];
omega = [omega_x; omega_y; omega_z];
H = I * omega;
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Eq. 13 Circle Plate Angular Momentum
function CirclePlateAngularMomentum
r = 0;
m = 0;
omega = [omega_x; omega_y; omega_z];
I_circle_plate = 0.5 * m * r^2;
H_circle_plate = I_circle_plate * omega;
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Eq. 14 Angular Momentum Relative Frame
function AngularMomentumRelativeFrame
I = [Ixx, Ixy, Ixz;
     Ixy, Iyy, Iyz;
     Ixz, Iyz, Izz];
omega = [omega_x; omega_y; omega_z];
H_rel = I * omega;
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Eq. 15 Angular Momentum Mapping Relation
function AngularMomentumMappingRelation
R = [cos(beta) sin(beta) 0;
    -sin(beta) cos(beta) 0;
    0 0 1];
H_body = I * omega;
H_new = R * H_body;
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Eq. 16 Total Angular Momentum
function TotalAngularMomentum
I = [Ixx, Ixy, Ixz;
     Ixy, Iyy, Iyz;
     Ixz, Iyz, Izz];
omega = [omega_x; omega_y; omega_z];
H_body = I * omega;
R = [cos(beta) sin(beta) 0;
    -sin(beta) cos(beta) 0;
    0 0 1];
H_new = R * H_body;
H_total = H_body + H_new;
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Eq. 17 Total Angular Momentum Transformation 
function TotalAngularMomentumTransformation
I = [Ixx, Ixy, Ixz;
     Ixy, Iyy, Iyz;
     Ixz, Iyz, Izz];
omega = [omega_x; omega_y; omega_z];
H_body = I * omega;
R = [cos(beta) sin(beta) 0;
    -sin(beta) cos(beta) 0;
    0 0 1];
H_new = R * H_body;
H_total = H_body + H_new;
T = [1, 0, 0;
     0, cos(alpha), -sin(alpha);
     0, sin(alpha),  cos(alpha)];
H_transformed = T * H_total;
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Eq. 18 Equivalent Moment of Inertia Matrix
function EquivalentMomentofInertiaMatrix
I = [Ixx, Ixy, Ixz;
     Ixy, Iyy, Iyz;
     Ixz, Iyz, Izz];
omega = [omega_x; omega_y; omega_z];
I_eq = I + diag([Ixx, Iyy, Izz]);
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Eq. 19 Force Distribution Matrix (FDM)
function ForceDistributionMatrix
FDM = [1, 0, 0;
       0, 1, 0;
       0, 0, 1];
Fx = 0; Fy = 0; Fz = 0;
F = [Fx; Fy; Fz];
F_dist = FDM * F;
Fx_dist = F_dist(1);
Fy_dist = F_dist(2);
Fz_dist = F_dist(3);
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Eq. 20 Axial Moment of Inertia Matrix
function AxialMomentofInertiaMatrix
I_axial = [Ixx, 0,   0;
           0,   Iyy, 0;
           0,   0,   Izz];
omega = [omega_x; omega_y; omega_z];
H_axial = I_axial * omega;
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Eq. 21 Total Angular Moment Inertial Relation
function TotalAngularMomentInertialRelation
I = [Ixx, Ixy, Ixz;
     Ixy, Iyy, Iyz;
     Ixz, Iyz, Izz];
omega = [omega_x; omega_y; omega_z];
R = [cos(beta) sin(beta) 0;
    -sin(beta) cos(beta) 0;
    0 0 1];
H_body = I * omega;
H_inertial = R' * H_body;
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Eq. 22 Euler Equation of Motion
function EulerEquationofMotion
I = [Ixx, Ixy, Ixz;
     Ixy, Iyy, Iyz;
     Ixz, Iyz, Izz];
omega = [omega_x; omega_y; omega_z];
tau = [tau_x; tau_y; tau_z];
omega_dot = inv(I) * (tau - cross(omega, I * omega));
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Eq. 23 External Control Torque
function ExternalControlTorque
K = [Kxx, Kxy, Kxz;
     Kxy, Kyy, Kyz;
     Kxz, Kyz, Kzz];
omega_d = [omega_dx; omega_dy; omega_dz];
omega = [omega_x; omega_y; omega_z];
tau_control = -K * (omega - omega_d);
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Eq. 24 Reaction Torque
function ReactionTorque
I = [Ixx, Ixy, Ixz;
     Ixy, Iyy, Iyz;
     Ixz, Iyz, Izz];
omega = [omega_x; omega_y; omega_z];
tau = [tau_x; tau_y; tau_z];
reaction_torque = I * omega_dot + cross(omega, I * omega) - tau;
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Eq. 25 Angular Momentum Torque Relation
function AngularMomentumTorqueRelation
I = [Ixx, Ixy, Ixz;
     Ixy, Iyy, Iyz;
     Ixz, Iyz, Izz];
omega = [omega_x; omega_y; omega_z];
tau = [tau_x; tau_y; tau_z];
H_body = I * omega;
H_dot = tau - cross(omega, H_body);
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%--------------------------------------------------------------------------
% ACTUATOR ANALYSIS
%--------------------------------------------------------------------------
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Eq. 26 Torque Rotation Matrix
function [tcx, tcy, tcz] = calculate_actuator_forces(tw1, tw2, tw3, tw4, a, b)
R = [cos(a)*cos(b), -cos(a)*sin(b), -cos(a)*cos(b),  cos(a)*sin(b);
     cos(a)*sin(b),  cos(a)*cos(b), -cos(a)*sin(b), -cos(a)*cos(b);
     sin(a),         sin(a),        sin(a),         sin(a)];
tw = [tw1; tw2; tw3; tw4];
tc = R * tw;
tcx = tc(1);
tcy = tc(2);
tcz = tc(3);
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Eq. 27 FDM Geometric Constraint Definition 1
function [a,b] = alpha_and_beta_calculations()
b = pi/4;
a = asind(sqrt(3)/3);
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Eq. 28 FDM Geometric Constraint Definition 2
function [c,d] = geometric_constraint()
c = cosd(45);
d = tand(30);
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Eq. 29 Geometrically Constrained Force Distribution Model
function A = FDMMatrix()
A = sqrt(3)/3*[1, -1, -1,  1;
               1,  1, -1,  1;
               1,  1,  1,  1];
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Eq. 30 Static Optimization
function static_optimization()
desired_torque = [0.0; 0.0; 0.0];

% Initial guess for wheel torques
tw0 = [0; 0; 0; 0];

% Call optimizer
options = optimoptions('fmincon','Display','none');
[tw_opt, fval] = fmincon(@(tw) norm(tw,2), tw0, [], [], [], [], [], [], ...
                            @(tw) torqueConstraint(tw, desired_torque), options);

% Display results
disp('Optimized reaction wheel torques:');
disp(tw_opt);
disp('Objective function value (norm of wheel torques):');
disp(fval);

% Nested constraint function
function [c, ceq] = torqueConstraint(tw, desTorque)
    % Use eq. 29 FDMMatrix for example
    A = FDMMatrix();
    % The difference between desired torque and generated torque
    torqueError = desTorque - A*tw;
    % No inequality constraint
    c = [];
    % We want torqueError == 0
    ceq = torqueError;
end
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Eq. 31 Static Optimization Subjections
function static_optimization_subjections()
desired_torque = [0.0; 0.0; 0.0];

% Initial guess for wheel torques
tw0 = [0; 0; 0; 0];

% Call optimizer with constraints
options = optimoptions('fmincon','Display','none');
[tw_opt, fval] = fmincon(@(tw) norm(tw,2), tw0, [], [], [], [], [], [], ...
                            @(tw) torqueConstraint(tw, desired_torque), options);

% Display results
disp('Optimized reaction wheel torques with subjections:');
disp(tw_opt);
disp('Objective function value (norm of wheel torques):');
disp(fval);
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Eq. 32 Lagrangian Definition
function lagrangian_definition()
syms x y lambda real
% Simple objective: f(x,y) = x^2 + y^2
f = x^2 + y^2;
% Simple constraint: g(x,y)= x + y - 1 = 0
g = x + y - 1;
% Lagrangian
L = f + lambda*g;

disp('Lagrangian:');
disp(L);
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Eq. 33 Static Optimization First-Order Condition Definition 1
function static_optimization_first_order_condition1()
syms x y lambda real
f = x^2 + y^2;
g = x + y - 1;
L = f + lambda*g;

dL_dx = diff(L, x);
dL_dy = diff(L, y);
dL_dlambda = diff(L, lambda);

disp('First-order conditions:');
disp(dL_dx);
disp(dL_dy);
disp(dL_dlambda);
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Eq. 34 Static Optimization First-Order Condition Definition 2
function static_optimization_first_order_condition2()
syms x y lambda real
f = x^2 + y^2;
g = x + y - 1;
L = f + lambda*g;

eqs = [diff(L,x) == 0, diff(L,y) == 0, diff(L,lambda) == 0];
sol = solve(eqs, [x, y, lambda], 'Real', true);

disp('Solution to first-order conditions:');
disp(sol);
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Eq. 35 Static Optimization First-Order Implied Condition
function static_optimization_first_order_implied_condition()
disp('Implied condition from first-order conditions:');
disp('The gradient of the Lagrangian is orthogonal to the constraint');
disp('function at the optimal conditions');
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Eq. 36 Augmented Distribution Matrix
function augmented_distribution_matrix()
Augmented matrix combining FDM with additional constraints
A = sqrt(3)/3*[1, -1, -1,  1;
                1,  1, -1,  1;
                1,  1,  1,  1];

additional_constraints = [0, 0, 0, 0]; 

% Augmented distribution matrix
augmented_A = [A; additional_constraints];

disp('Augmented Distribution Matrix:');
disp(augmented_A);
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Eq. 37 Inverse Mapping Augmented Distribution Matrix
function inverse_mapping_augmented_distribution_matrix()
    % Placeholder for computing the inverse of the augmented distribution matrix.

    disp('Inverse mapping augmented distribution matrix placeholder.');
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Eq. 38 Linear Dynamic Reaction Torque
function linear_dynamic_reaction_torque()
omega_wheel = [omega_x; omega_y; omega_z];
omega_ref = [omega_ref_x; omega_ref_y; omega_ref_z];

K_lin = [1, 0, 0, 0;
        0, 1, 0, 0;
        0, 0, 1, 0];

tau_reaction = K_lin * (omega_wheel - omega_ref);
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Eq. 39 Control Input Voltage
function control_input_voltage()
V_max = 0.0; % Maximum voltage
K_v = 0.0; % Motor velocity constant

% Desired torque
tau_desired = [tau_x; tau_y; tau_z];

% Calculate the required wheel speeds
omega_wheel_desired = inv(K_lin) * tau_desired;

% Calculate the control input voltage
V_control = K_v * omega_wheel_desired;

% Ensure the voltage does not exceed the maximum allowed voltage
V_control = min(max(V_control, -V_max), V_max);
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Eq. 40 Closed-Loop Sliding Dynamics
function closed_loop_sliding_dynamics()
% Define system parameters
J = [Jxx, Jxy, Jxz;
    Jxy, Jyy, Jyz;
    Jxz, Jyz, Jzz];
K = [Kp, 0, 0;
    0, Kp, 0;
    0, 0, Kp];
C = [Kd, 0, 0;
    0, Kd, 0;
    0, 0, Kd];

% Define state variables
omega = [omega_x; omega_y; omega_z];
omega_ref = [omega_ref_x; omega_ref_y; omega_ref_z];
e = omega - omega_ref;

% Define sliding surface
s = C * e + J * (omega - omega_ref);

% Define control law
tau_control = -K * sign(s);

% Compute closed-loop dynamics
omega_dot = inv(J) * (tau_control - cross(omega, J * omega));
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Eq. 41 Closed-Loop Sliding Dynamics State Space Form
function closed_loop_sliding_dynamics_state_space_form()
% Define system parameters
J = [Jxx, Jxy, Jxz;
    Jxy, Jyy, Jyz;
    Jxz, Jyz, Jzz];
K = [Kp, 0, 0;
    0, Kp, 0;
    0, 0, Kp];
C = [Kd, 0, 0;
    0, Kd, 0;
    0, 0, Kd];

% Define state variables
omega = [omega_x; omega_y; omega_z];
omega_ref = [omega_ref_x; omega_ref_y; omega_ref_z];
e = omega - omega_ref;

% Define sliding surface
s = C * e + J * (omega - omega_ref);

% Define control law
tau_control = -K * sign(s);

% State-space representation
A = -inv(J) * C;
B = inv(J);
u = tau_control;

% State-space form
omega_dot = A * omega + B * u;
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Eq. 42 Closed-Loop Sliding Dynamics State Space Form Implied Condition
function closed_loop_sliding_dynamics_state_space_form_implied_condition()
% Define system parameters
J = [Jxx, Jxy, Jxz;
    Jxy, Jyy, Jyz;
    Jxz, Jyz, Jzz];
K = [Kp, 0, 0;
    0, Kp, 0;
    0, 0, Kp];
C = [Kd, 0, 0;
    0, Kd, 0;
    0, 0, Kd];

% Define state variables
omega = [omega_x; omega_y; omega_z];
omega_ref = [omega_ref_x; omega_ref_y; omega_ref_z];
e = omega - omega_ref;

% Define sliding surface
s = C * e + J * (omega - omega_ref);

% Implied condition: Sliding surface should converge to zero
% This implies that the control law should be designed such that
% the sliding surface s and its derivative s_dot converge to zero.
s_dot = C * (omega_dot - omega_ref_dot) + J * (omega_dot - omega_ref_dot);
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Eq. 43 Variable Transformation
function variable_transformation()
% Define the state variables
x = [x1; x2; x3];
x_ref = [x1_ref; x2_ref; x3_ref];

% Define the error
e = x - x_ref;

% Define the sliding surface
s = C * e + J * (x_dot - x_ref_dot);

% Define the variable transformation
z = T * e;
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Eq. 44 Variable Transformation Time Derivative
function variable_transformation_time_derivative()
% Define the state variables
x = [x1; x2; x3];
x_ref = [x1_ref; x2_ref; x3_ref];

% Define the error
e = x - x_ref;

% Define the sliding surface
s = C * e + J * (x_dot - x_ref_dot);

% Define the variable transformation
z = T * e;

% Compute the time derivative of the transformed variables
z_dot = T * (x_dot - x_ref_dot);
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Eq. 45 Variable Transformation Time Derivative Matrix Form
function variable_transformation_time_derivative_matrix_form()
% Define the state variables
x = [x1; x2; x3];
x_ref = [x1_ref; x2_ref; x3_ref];

% Define the error
e = x - x_ref;

% Define the sliding surface
s = C * e + J * (x_dot - x_ref_dot);

% Define the variable transformation
T = eye(3);
z = T * e;

% Compute the time derivative of the transformed variables in matrix form
z_dot = T * (x_dot - x_ref_dot);
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Eq. 46 Disturbance Transformation
function disturbance_transformation()
% Define the state variables
x = [x1; x2; x3];
x_ref = [x1_ref; x2_ref; x3_ref];

% Define the error
e = x - x_ref;

% Define the sliding surface
s = C * e + J * (x_dot - x_ref_dot);

% Define the disturbance
disturbance = [d1; d2; d3];

% Define the variable transformation
T = eye(3);
z = T * e;

% Transform the disturbance
disturbance_transformed = T * disturbance;

% Compute the time
t = 0:0.01:10; % Time vector

% Compute the disturbance at each time step
disturbance_t = zeros(3, length(t));
for i = 1:length(t)
    disturbance_t(:, i)

    % Compute the transformed disturbance
    disturbance_transformed_t = T * disturbance_t(:, i);
end
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%--------------------------------------------------------------------------
% QUATERNION REACTION WHEEL ALGORITHM                                     -
%--------------------------------------------------------------------------
% This is a function file list for the purpose of compiling all necessary
% equations for Probe CubeSat Attitude and Determination Dynamics.
% The zero constants will be varying in accordance to IMU/Motor encoding.

% Purpose: This file contains the necessary equations for the Probe CubeSat
% Attitude and Determination Dynamics. The equations are organized by
% category and are intended to be used as a reference for the Probe CubeSat
% Attitude and Determination Dynamics.

% Super-Twisting Sliding Mode Attitude Control Simulation
function super_twisting_sliding_mode
    %% --- Physical and configuration parameters ---
    mu = 398600.4418; % Earth's gravitational parameter (km^3/s^2)
    
    [alpha, beta] = eq28_solve_constraints();
    
    J = [35, 3, -1.5; 3, 28, 2; -1.5, 2, 30];
    J_m_list = [0.126, 0.063, 0.063, 0.063];
    J_w_list = cell(1,4);
    for i = 1:4
        J_w_list{i} = eq13_J_w_i(J_m_list(i), J_m_list(i));
    end
    
    R_list = cell(1,4);
    for i = 1:4
        R_list{i} = eq5_rotation_R_i(alpha, eq5_beta_i(beta, i));
    end
    
    J_eq        = eq18_J_eq(J, R_list, J_w_list);
    Gamma       = eq29_Gamma_specific();
    J_m_mat     = eq20_J_m_matrix(J_m_list);
    Pi_m        = Gamma * J_m_mat;
    lambda_gain = 0.05;
    
    K1        = diag([0.547, 0.547, 0.547]);
    K2        = diag([0.05, 0.05, 0.05]);
    a_i       = [0.02, 0.02, 0.02, 0.02];
    b_i       = [3.0, 3.0, 3.0, 3.0];
    r0        = [8000.0; 0.0; 6000.0];
    v0        = [0.0; 7.0; 0.0];
    omega0    = deg2rad([7.0; -8.0; -7.0]);
    Q0        = [0.999; 0.017; -0.035; 0.026];
    Omega0    = zeros(4,1);
    int_sign0 = zeros(3,1);
    
    % Step 1: Propagate the orbital trajectory
    t_span_orbit = [0, 100];
    t_eval_orbit = linspace(t_span_orbit(1), t_span_orbit(2), 101);
    y0_orbit = [r0; v0];
    opts = odeset('RelTol',1e-9,'AbsTol',1e-12);
    [t_orbit, y_orbit] = ode113(@(t, y) orbital_dynamics(t, y, mu), t_eval_orbit, y0_orbit, opts);
    
    % Step 2: Compute reference quaternion Qd(t) from the TNB frame
    Q_ref = zeros(length(t_orbit), 4);
    for i = 1:length(t_orbit)
        Q_ref(i,:) = compute_Q_ref(y_orbit(i,1:3)', y_orbit(i,4:6)', mu);
    end
    Qd_interp = @(t) interp1(t_orbit, Q_ref, t, 'pchip');
    
    % Step 3: Combined attitude + wheel dynamics
    y0 = [Q0; omega0; Omega0; int_sign0];
    t_span_att = [0, 250];
    t_eval_att = linspace(t_span_att(1), t_span_att(2), 2501);
    opts2 = odeset('RelTol',1e-6,'AbsTol',1e-9);
    [t_att, y_att] = ode45(@(t, y) combined_dynamics(t, y, Qd_interp, J_eq, Pi_m, K1, K2, lambda_gain, J_m_list, a_i, b_i, Gamma, J_m_mat), t_eval_att, y0, opts2);
    
    % --- Plotting results ---
    figure;
    plot3(y_orbit(:,1), y_orbit(:,2), y_orbit(:,3));
    xlabel('X (km)'); ylabel('Y (km)'); zlabel('Z (km)');
    title('Spacecraft 3D Trajectory');
    
    figure;
    plot(t_att, y_att(:,1), t_att, y_att(:,2), t_att, y_att(:,3), t_att, y_att(:,4));
    legend('q0e','q1e','q2e','q3e'); xlabel('Time (s)'); ylabel('Quaternion Error');
    title('Quaternion Tracking Error');
    S_hist = zeros(length(t_att), 3);
    for idx = 1:length(t_att)
        q = y_att(idx,1:4)';
        omega = y_att(idx,5:7)';
        qd = Qd_interp(t_att(idx))';
        rot_q = quat2rotm([q(1), q(2), q(3), q(4)]);
        rot_qd = quat2rotm([qd(1), qd(2), qd(3), qd(4)]);
        rot_e = rot_qd' * rot_q;
        qe = rotm2quat(rot_e);
        q_vec_e = qe(2:4);
        S_hist(idx,:) = eq72_qe_dot(qe(1), q_vec_e, omega) + lambda_gain * q_vec_e;
    end
    
    figure;
    plot(t_att, S_hist(:,1), t_att, S_hist(:,2), t_att, S_hist(:,3));
    legend('S1','S2','S3'); xlabel('Time (s)'); ylabel('Sliding Surface');
    title('Sliding Surface vs Time');
    
    Omega_hist = y_att(:,8:11)';
    Omega_dot_fd = gradient(Omega_hist, t_att);
    tau_w = -J_m_mat * Omega_dot_fd;
    figure;
    hold on;
    for i = 1:4
        plot(t_att, tau_w(i,:), 'DisplayName', sprintf('tau_{w%d}', i));
    end
    hold off;
    legend; xlabel('Time (s)'); ylabel('Reaction Torque');
    title('Evolution of Reaction Torque');
end

function dy = orbital_dynamics(~, y, mu)
    r = y(1:3);
    v = y(4:6);
    a = -mu * r / norm(r)^3;
    dy = [v; a];
end

function q = compute_Q_ref(r_vec, v_vec, mu)
    e_t = v_vec / norm(v_vec);
    acc = -mu * r_vec / norm(r_vec)^3;
    a_t = dot(acc, e_t) * e_t;
    a_n = acc - a_t;
    e_n = a_n / norm(a_n);
    e_b = cross(e_t, e_n);
    C = [e_t, e_n, e_b];
    q = rotm2quat(C');
end

function dy = combined_dynamics(t, y, Qd_interp, J_eq, Pi_m, K1, K2, lambda_gain, J_m_list, a_i, b_i, Gamma, J_m_mat)
    q = y(1:4);
    omega = y(5:7);
    Omega = y(8:11);
    int_s = y(12:14);
    qd = Qd_interp(t)';
    rot_q = quat2rotm([q(1), q(2), q(3), q(4)]);
    rot_qd = quat2rotm([qd(1), qd(2), qd(3), qd(4)]);
    rot_e = rot_qd' * rot_q;
    qe = rotm2quat(rot_e);
    q0e = qe(1); qv = qe(2:4);
    qv_dot = eq72_qe_dot(q0e, qv, omega);
    S = qv_dot + lambda_gain * qv;
    xi = 0.5 * lambda_gain * (q0e * eye(3) + eq10_cross_matrix(qv)) * omega;
    tau_c0 = eq90_tau_c0(omega, J_eq, Pi_m, Omega, xi);
    tau_cN = eq90_tau_cN(J_eq, K1, S, K2, int_s);
    tau_c = tau_c0 + tau_cN;
    Omega_dot = zeros(4,1);
    for i = 1:4
        Omega_dot(i) = eq38_motor_dynamics(J_m_list(i), a_i(i), Omega(i), b_i(i), eq39_control_voltage(a_i(i), b_i(i), Omega(i), -J_m_list(i)*0));
    end
    H_total = eq21_H_T_simplified(J_eq, omega, Gamma, J_m_mat, Omega);
    q_dot = 0.5 * eq9_E_of_Q(q(1), q(2:4)) * omega;
    omega_dot = J_eq \ (tau_c - cross(omega, H_total));
    int_s_dot = sign(S);
    dy = [q_dot; omega_dot; Omega_dot; int_s_dot];
end

% --- Equation 1: Rotation matrix around z-axis by angle beta ---
function Rz = eq1_rotation_z(beta)
    Rz = [cos(beta),  sin(beta), 0;
         -sin(beta), cos(beta), 0;
          0,         0,         1];
end

% --- Equation 2: Rotation matrix around -y-axis by angle alpha ---
function Ry_minus = eq2_rotation_y_minus(alpha)
    Ry_minus = [cos(-alpha), 0, -sin(-alpha);
                0,          1,  0;
                sin(-alpha), 0,  cos(-alpha)];
end

% --- Equation 3: Rotation matrix around y-axis by angle alpha ---
function Ry = eq3_rotation_y(alpha)
    Ry = [cos(alpha), 0, -sin(alpha);
           0,         1, 0;
           sin(alpha), 0, cos(alpha)];
end

% --- Equation 4: General mapping xi = R * x ---
function xi = eq4_map(R, x)
    xi = R * x;
end

% --- Equation 5: Rotation matrix R_i defined by alpha and beta_i ---
function R = eq5_rotation_R_i(alpha, beta_i)
    R = [cos(alpha)*cos(beta_i),  cos(alpha)*sin(beta_i),  sin(alpha);
        -sin(beta_i),            cos(beta_i),             0;
        -cos(beta_i)*sin(alpha), -sin(alpha)*sin(beta_i), cos(alpha)];
end

% --- Equation 5 (aux): beta_i = beta + 0.5*(i-1)*pi ---
function beta_i = eq5_beta_i(beta, i)
    beta_i = beta + 0.5*(i-1)*pi;
end

% --- Equation 6: Quaternion scalar derivative ---
function q0_dot = eq6_q0_dot(q_vec, omega)
    q0_dot = -0.5 * dot(q_vec, omega);
end

% --- Equation 7: Quaternion vector derivative ---
function q_dot_vec = eq7_q_dot_vec(q0, q_vec, omega)
    q_dot_vec = 0.5 * (q0 * eye(3) + eq10_cross_matrix(q_vec)) * omega;
end

% --- Equation 8: Quaternion kinematics in matrix form ---
function q_dot = eq8_q_dot_matrix(E, omega)
    q_dot = 0.5 * E * omega;
end

% --- Equation 9: Construct E(Q) matrix from quaternion ---
function E = eq9_E_of_Q(q0, q_vec)
    E = [-q_vec'; q0*eye(3) + eq10_cross_matrix(q_vec)];
end

% --- Equation 10: Cross-product matrix a^x for vector a ---
function X = eq10_cross_matrix(a)
    X = [ 0,   -a(3),  a(2);
          a(3), 0,    -a(1);
         -a(2), a(1),  0];
end

% --- Equation 11: Angular velocity of i-th reaction wheel in local frame ---
function omega_w_i = eq11_omega_w_i(R_i, omega, Omega_i)
    e_x = [1; 0; 0];
    omega_w_i = R_i * omega + Omega_i * e_x;
end

% --- Equation 12: Angular momentum H = J * omega ---
function H = eq12_H_spacecraft(J, omega)
    H = J * omega;
end

% --- Equation 13: Moment of inertia of i-th reaction wheel in its local frame ---
function Jw = eq13_J_w_i(Jm, Jp)
    Jw = diag([Jm, Jp, Jp]);
end

% --- Equation 14: Angular momentum of i-th reaction wheel in local frame ---
function H_w_local = eq14_H_w_local(J_w_i, R_i, omega, J_m_i, Omega_i)
    e_x = [1; 0; 0];
    H_w_local = J_w_i * (R_i * omega) + J_m_i * Omega_i * e_x;
end

% --- Equation 15: Angular momentum of i-th reaction wheel in body frame ---
function H_w_body = eq15_H_w_body(R_i, J_w_i, omega, J_m_i, Omega_i)
    e_x = [1; 0; 0];
    H_w_body = R_i' * (J_w_i * (R_i * omega)) + R_i' * (J_m_i * Omega_i * e_x);
end

% --- Equation 16: Total angular momentum H_T ---
function H_T = eq16_H_total(J, R_list, J_w_list, omega, J_m_list, Omega_list)
    H_sc = J * omega;
    H_rw_sum = zeros(3,1);
    for i = 1:4
        H_rw_sum = H_rw_sum + eq15_H_w_body(R_list{i}, J_w_list{i}, omega, J_m_list(i), Omega_list(i));
    end
    H_T = H_sc + H_rw_sum;
end

% --- Equation 17: Sum of reaction wheel torques ---
function out = eq17_control_term(Gamma, J_m_mat, Omega_vec)
    out = Gamma * (J_m_mat * Omega_vec);
end

% --- Equation 18: Equivalent moment of inertia ---
function J_eq = eq18_J_eq(J, R_list, J_w_list)
    J_eq = J;
    for i = 1:4
        J_eq = J_eq + R_list{i}' * J_w_list{i} * R_list{i};
    end
end

% --- Equation 19: Force distribution matrix Gamma ---
function Gamma = eq19_Gamma(alpha, beta)
    Gamma = [ cos(alpha)*cos(beta), -cos(alpha)*sin(beta), -cos(alpha)*cos(beta),  cos(alpha)*sin(beta);
              cos(alpha)*sin(beta),  cos(alpha)*cos(beta), -cos(alpha)*sin(beta), -cos(alpha)*cos(beta);
              sin(alpha),            sin(alpha),            sin(alpha),            sin(alpha)];
end

% --- Equation 20: Axial moment of inertia matrix for reaction wheels ---
function Jm_mat = eq20_J_m_matrix(J_m_list)
    Jm_mat = diag(J_m_list);
end

% --- Equation 21: Simplified total angular momentum ---
function H_T = eq21_H_T_simplified(J_eq, omega, Gamma, J_m_mat, Omega_vec)
    H_T = J_eq * omega + Gamma * (J_m_mat * Omega_vec);
end

% --- Equation 22: Euler's equation of motion ---
function M_G = eq22_euler_equation(H_T_dot, omega, H_T)
    M_G = H_T_dot + cross(omega, H_T);
end

% --- Equation 23: Full attitude dynamics with control and disturbance ---
function out = eq23_attitude_dynamics(J_eq, omega_dot, Gamma, J_m_mat, Omega_dot, omega, Gamma_J_m_Omega, tau_a, tau_c, d)
    out = tau_a + d - (J_eq * omega_dot + Gamma * (J_m_mat * Omega_dot) + cross(omega, J_eq * omega + Gamma_J_m_Omega));
end

% --- Equation 24: Reaction wheel torque vector tau_w = -J_m * dOmega/dt ---
function tau_w = eq24_tau_w(J_m_mat, Omega_dot)
    tau_w = -J_m_mat * Omega_dot;
end

% --- Equation 24: Control torque tau_c = Gamma * tau_w ---
function tau_c = eq24_tau_c(Gamma, tau_w)
    tau_c = Gamma * tau_w;
end

% --- Equation 25: Simplified attitude dynamics ---
function out = eq25_simplified_dynamics(J_eq, omega_dot, tau_a, tau_c, d, omega, H_total)
    out = J_eq * omega_dot - (tau_a + tau_c + d - cross(omega, H_total));
end

% --- Equation 26: Expanded form of control torque ---
function tau_c = eq26_control_expanded(Gamma, tau_w)
    tau_c = Gamma * tau_w;
end

% --- Equation 27: Constraint functions ---
function [g1, g2] = eq27_geometric_constraints(alpha, beta)
    g1 = sin(beta) - cos(beta);
    g2 = cos(alpha)*sin(beta) - sin(alpha);
end

% --- Equation 28: Solutions alpha, beta under constraints ---
function [alpha, beta] = eq28_solve_constraints()
    beta = pi/4;
    alpha = asin(sqrt(3)/3);
end

% --- Equation 29: Specific Gamma under alpha, beta constraints ---
function Gamma = eq29_Gamma_specific()
    factor = sqrt(3)/3;
    Gamma = factor * [1, -1, -1, 1;
                      1,  1, -1, -1;
                      1,  1,  1,  1];
end

% --- Equation 30: Performance index T = sum(tau_wi^2) ---
function T = eq30_performance_index(tau_w)
    T = sum(tau_w.^2);
end

% --- Equation 31: Constraint equations g1, g2, g3 ---
function [g1, g2, g3] = eq31_constraint_equations(tau_w, tau_c)
    factor = sqrt(3)/3;
    g1 = factor*(tau_w(1) - tau_w(2) - tau_w(3) + tau_w(4)) - tau_c(1);
    g2 = factor*(tau_w(1) + tau_w(2) - tau_w(3) - tau_w(4)) - tau_c(2);
    g3 = factor*(tau_w(1) + tau_w(2) + tau_w(3) + tau_w(4)) - tau_c(3);
end

% --- Equation 32: Lagrangian L = T + sum(lambda_i * g_i) ---
function L = eq32_lagrangian(tau_w, lambdas, tau_c)
    T = eq30_performance_index(tau_w);
    [g1, g2, g3] = eq31_constraint_equations(tau_w, tau_c);
    L = T + lambdas(1)*g1 + lambdas(2)*g2 + lambdas(3)*g3;
end

% --- Equation 33: Partial derivatives of L w.r.t tau_w ---
function dL_dtau = eq33_grad_L_tau(tau_w, lambdas)
    factor = sqrt(3)/3;
    dL_dtau = [2*tau_w(1) + factor*(lambdas(1) + lambdas(2) + lambdas(3));
               2*tau_w(2) + factor*(-lambdas(1) + lambdas(2) + lambdas(3));
               2*tau_w(3) + factor*(-lambdas(1) - lambdas(2) + lambdas(3));
               2*tau_w(4) + factor*( lambdas(1) - lambdas(2) + lambdas(3))];
end

% --- Equation 34: Partial derivatives of L w.r.t lambdas ---
function [g1, g2, g3] = eq34_grad_L_lambda(tau_w, tau_c)
    [g1, g2, g3] = eq31_constraint_equations(tau_w, tau_c);
end

% --- Equation 35: tau_w1 - tau_w2 + tau_w3 - tau_w4 = 0 ---
function val = eq35_implicit_constraint(tau_w)
    val = tau_w(1) - tau_w(2) + tau_w(3) - tau_w(4);
end

% --- Equation 36: Augmented mapping matrix (4x4) ---
function M = eq36_augmented_mapping_matrix()
    factor = sqrt(3)/3;
    M = factor * [1, -1, -1,  1;
                  1,  1, -1, -1;
                  1,  1,  1,  1;
                  1, -1,  1, -1];
end

% --- Equation 37: Inverse of augmented mapping matrix ---
function Minv = eq37_inverse_mapping_matrix()
    factor = 1/4;
    sqrt3 = sqrt(3);
    Minv = factor * [ sqrt3,  sqrt3,  sqrt3,  1;
                     -sqrt3,  sqrt3,  sqrt3, -1;
                     -sqrt3, -sqrt3,  sqrt3,  1;
                      sqrt3, -sqrt3,  sqrt3, -1];
end

% --- Equation 38: Motor dynamics: J_m_i * dOmega = -a_i*Omega + b_i*V_in ---
function Jm = eq38_motor_dynamics(J_m_i, a_i, Omega_i, b_i, V_in_i)
    Jm = (-a_i * Omega_i + b_i * V_in_i) / J_m_i;
end

% --- Equation 39: Input voltage for wheel i ---
function V = eq39_control_voltage(a_i, b_i, Omega_i, tau_w_i)
    V = (-tau_w_i + a_i * Omega_i) / b_i;
end

% --- Equation 40: Super-twisting sliding mode dynamics dot(s) ---
function s_dot = eq40_super_twisting_dot(s, integral_sign_s, k1, k2, d)
    s_dot = -k1 * sign(s) * sqrt(abs(s)) - k2 * integral_sign_s + d;
end

% --- Equation 41: z1 and z2 definitions ---
function [z1, z2] = eq41_state_vars(s, integral_sign_s, d, k2)
    z1 = s;
    z2 = -k2 * integral_sign_s + d;
end

% --- Equation 42: State-space form of super-twisting algorithm ---
function [dz1, dz2] = eq42_super_twisting_ss(z1, z2, k1, k2, rho)
    dz1 = -k1 * abs(z1)^0.5 * sign(z1) + z2;
    dz2 = -k2 * sign(z1) + rho;
end

% --- Equation 43: Variable transformation to zeta ---
function [zeta1, zeta2] = eq43_var_transform(z1, z2)
    zeta1 = sign(z1) * sqrt(abs(z1));
    zeta2 = z2;
end

% --- Equation 44: Dynamics in transformed variables ---
function [dzeta1, dzeta2] = eq44_transformed_dynamics(zeta1, zeta2, k1, k2, rho)
    dzeta1 = (1/zeta1) * (-0.5*k1*zeta1 + 0.5*zeta2);
    dzeta2 = (1/zeta1) * (-k2*zeta1 + zeta1*rho);
end

% --- Equation 45: Matrices A and B for transformed dynamics ---
function [A, B] = eq45_AB_matrices(k1, k2)
    A = [-0.5*k1, 0.5; -k2, 0];
    B = [0; 1];
end

% --- Equation 46: Bound on hat_rho ---
function val = eq46_hat_rho_bound(rho, delta, zeta1)
    val = abs(rho) * abs(zeta1) <= delta * abs(zeta1);
end

% --- Equation 47: Construct LMI block matrix ---
function M = eq47_LMI_matrix(P, Qc, A, B, c)
    top = [P*A + A'*P + Qc + [1,0;0,0], P*B];
    bottom = [B'*P, -c^-2];
    M = [top; bottom];
end

% --- Equation 48: Lyapunov function V = zeta^T P zeta ---
function V = eq48_lyapunov(P, zeta)
    V = zeta' * P * zeta;
end

% --- Equation 49: Rayleigh bounds for V ---
function [lam_min, lam_max, norm2] = eq49_rayleigh_bounds(P, zeta)
    lam_min = min(eig(P));
    lam_max = max(eig(P));
    norm2 = norm(zeta)^2;
end

% --- Equation 50: Time derivative of V ---
function dV = eq50_dV_dt(P, A, zeta, rho, zeta1)
    term1 = zeta' * (P*A + A'*P) * zeta;
    term2 = 2 * zeta' * P * [0;1] * rho;
    dV = (term1 + term2) / abs(zeta1);
end

% --- Equation 51: Bound on |hat_rho|^2 ---
function val = eq51_hat_rho_sq_bound(rho, delta, zeta_norm_sq)
    val = rho^2 <= delta^2 * zeta_norm_sq;
end

% --- Equation 52: Check positive definiteness ---
function val = eq52_inequality(zeta_norm_sq, rho_sq, delta)
    val = zeta_norm_sq - (rho_sq / delta^2) > 0;
end

% --- Equation 53: Refined derivative bound matrix form ---
function val = eq53_dV_bound(P, Qc, A, B, C, zeta, rho, c)
    M = [P*A + A'*P + Qc + C'*C, P*B; B'*P, -c^-2];
    vec = [zeta; rho];
    val = vec' * M * vec;
end

% --- Equation 54: Upper bound on V_dot ---
function val = eq54_upper_bound(Qc, P, zeta_norm_sq, zeta1)
    lam_q = min(eig(Qc));
    lam_p = max(eig(P));
    val = -(lam_q / lam_p) * (zeta_norm_sq / abs(zeta1));
end

% --- Equation 55: Bound on 1/|zeta1| ---
function val = eq55_reciprocal_bound(zeta1, P, V)
    lam_p_min = min(eig(P));
    val = 1/abs(zeta1) <= sqrt(lam_p_min) / sqrt(V);
end

% --- Equation 56: Differential inequality dot(V) <= -alpha V^1/2 ---
function val = eq56_diff_inequality(alpha, V)
    val = -alpha * sqrt(V);
end

% --- Equation 57: Integrated differential inequality ---
function val = eq57_integral_inequality(V_t, V0, t, alpha)
    lhs = 2 * (sqrt(V0) - sqrt(V_t));
    rhs = alpha * t;
    val = lhs <= rhs;
end

% --- Equation 58: Lyapunov decay V(t) ---
function Vt = eq58_V_decay(V0, alpha, t)
    Vt = (sqrt(V0) - (alpha/2)*t)^2;
end

% --- Equation 59: Finite-time convergence bound ---
function tf = eq59_tf_bound(V0, alpha)
    tf = 2 * sqrt(V0) / alpha;
end

% --- Equation 60: Gain conditions set 1 ---
function val = eq60_feasibility_case1(k1, k2, delta)
    val = (k2 > delta) && (k1^2 > 4*k2);
end

% --- Equation 61: Gain conditions set 2 ---
function val = eq61_feasibility_case2(k1, k2, delta)
    lhs = (k1^2)/(2*k2) - (k1^4)/16;
    val = (lhs < delta^2) && (4*k2 > k1^2) && (k1 ~= 0) && (k2 ~= 0);
end

% --- Equation 62: Transfer function G(s) ---
function G = eq62_transfer_function(s, k1, k2)
    G = 0.5 / (s^2 + 0.5*k1*s + 0.5*k2);
end

% --- Equation 63: Bounded-real condition ---
function val = eq63_bounded_real(delta, max_gain)
    val = max_gain < 1/delta;
end

% --- Equation 64: |G(jw)|^2 magnitude squared ---
function val = eq64_G_mag_sq(omega, k1, k2)
    val = 1 / ((k2 - 2*omega^2)^2 + (k1*omega)^2);
end

% --- Equation 65: Derivative of |G(jw)|^2 ---
function val = eq65_dG_mag_sq(omega, k1, k2)
    num = 16*omega*(omega^2 + (k1^2)/8 - k2/2);
    den = ((k2 - 2*omega^2)^2 + (k1*omega)^2)^2;
    val = num / den;
end

% --- Equation 66: Extremum frequency omega ---
function omega_ext = eq66_extremum_frequency(k1, k2)
    if 4*k2 - k1^2 < 0
        omega_ext = 0;
    else
        omega_ext = sqrt((4*k2 - k1^2)/8);
    end
end

% --- Equation 67: Maximum gain expression ---
function val = eq67_max_gain_expr(k1, k2)
    if 4*k2 - k1^2 < 0
        val = 0;
    else
        val = 1 / (k1^2 * (0.5*k2 - (k1^2)/16));
    end
end

% --- Equation 68: (Not defined in original, stub for completeness) ---
function out = eq68_stub(varargin)
    out = [];
end

% --- Equation 69: (Not defined in original, stub for completeness) ---
function out = eq69_stub(varargin)
    out = [];
end

% --- Equation 70: Quaternion-based angular velocity error ---
function err = eq70_omega_error(omega, omega_d, omega_q)
    err = omega - omega_d;
    % omega_q is optional, not used here
end

% --- Equation 71: Quaternion scalar error derivative ---
function q0e_dot = eq71_q0e_dot(q_e_vec, omega_e)
    q0e_dot = -0.5 * dot(q_e_vec, omega_e);
end

% --- Equation 72: Quaternion vector error derivative ---
function out = eq72_qe_dot(q0e, q_e_vec, omega_e)
    out = 0.5 * (q0e * eye(3) + eq10_cross_matrix(q_e_vec)) * omega_e;
end

% --- Equation 73: Angular velocity error derivative ---
function omega_e_dot = eq73_omega_e_dot(J_eq_inv, tau_c, d, omega, H_eq_omega, Pi_m, Omega)
    omega_e_dot = J_eq_inv * (-tau_c + d - cross(omega, H_eq_omega + Pi_m * Omega));
end

% --- Equation 74: Sliding surface S = e_dot + lambda * q_e ---
function S = eq74_sliding_surface(e_dot, lam, q_e)
    S = e_dot + lam * q_e;
end

% --- Equation 75: Sliding condition S = S_dot = 0 for t>=t_f ---
function val = eq75_sliding_condition(S, S_dot)
    val = all(abs(S) < 1e-8) && all(abs(S_dot) < 1e-8);
end

% --- Equation 76: (Not defined in original, stub for completeness) ---
function out = eq76_stub(varargin)
    out = [];
end

% --- Equation 77: (Not defined in original, stub for completeness) ---
function out = eq77_stub(varargin)
    out = [];
end

% --- Equation 78: Check unit quaternion identity q_e^T q_e = 1 ---
function val = eq78_unit_quaternion_identity(q_e)
    val = abs(norm(q_e)^2 - 1.0) < 1e-8;
end

% --- Equation 79: Scalar quaternion error dynamics under sliding ---
function val = eq79_q0e_scalar_dynamics(q0e)
    val = 0.5 * (1 - q0e^2);
end

% --- Equation 80: Change of variable zeta for stability analysis ---
function zeta = eq80_change_of_variable(q0e)
    zeta = 1 / (q0e - 1);
end

% --- Equation 81: Derivative of new variable y ---
function y_dot = eq81_y_dot(y)
    y_dot = y - 0.5;
end

% --- Equation 82: (Not defined in original, stub for completeness) ---
function out = eq82_stub(varargin)
    out = [];
end

% --- Equation 83: First-order linear ODE: y_dot - y = -1/2 ---
function val = eq83_linear_ode(y_dot, y)
    val = abs(y_dot - y + 0.5) < 1e-8;
end

% --- Equation 84: General solution y(t) = C e^t - 1/2 ---
function y = eq84_general_solution_y(C, t)
    y = C * exp(t) - 0.5;
end

% --- Equation 85: Inverse mapping of change of variable ---
function q0e = eq85_q0e_from_y(y)
    q0e = 1 + 1 / y;
end

% --- Equation 86: Constant C from initial condition q0e(0) ---
function C = eq86_constant_from_initial(q0e0)
    C = 0.5 + 1 / (q0e0 - 1);
end

% --- Equation 87: Condition to avoid singularity ---
function val = eq87_avoid_singularity(q0e_tf, t_f)
    val = -q0e_tf + 1 + (q0e_tf + 1) * exp(-t_f) ~= 0;
end

% --- Equation 88: Sliding surface derivative ---
function S_dot = eq88_S_dot(omega_e_dot, lam, q_e_dot, D1)
    S_dot = omega_e_dot + lam * q_e_dot + D1;
end

% --- Equation 89: Auxiliary variable xi ---
function xi = eq89_xi_def(omega_d_dot, lam, q0e, q_e_vec, omega_e)
    xi = -omega_d_dot + 0.5 * lam * (q0e * eye(3) + eq10_cross_matrix(q_e_vec)) * omega_e;
end

% --- Equation 90: Super-twisting control law tau_c = tau_c0 + tau_cN ---
function tau_c = eq90_control_law(tau_c0, tau_cN)
    tau_c = tau_c0 + tau_cN;
end

% --- Equation 90 (cont): Continuous part of control torque ---
function tau_c0 = eq90_tau_c0(omega, J_eq, Pi_m, Omega, xi)
    tau_c0 = cross(omega, J_eq*omega + Pi_m*Omega) - J_eq*xi;
end

% --- Equation 90 (cont): Nonlinear part of control torque ---
function tau_cN = eq90_tau_cN(J_eq, K1, S, K2, integral_sign_S)
    norm_S = norm(S);
    if norm_S < 1e-12
        S_scaled = S;
    else
        S_scaled = S / sqrt(norm_S);
    end
    tau_cN = -J_eq * (K1*S_scaled + K2*integral_sign_S);
end

% --- Equation 91: Gain matrices K1 and K2 ---
function [K1, K2] = eq91_gain_matrices(k11, k12, k13, k21, k22, k23)
    K1 = diag([k11, k12, k13]);
    K2 = diag([k21, k22, k23]);
end

% --- Equation 92: Closed-loop sliding dynamics ---
function out = eq92_closed_loop_dynamics(S, integral_sign_S, K1, K2, D)
    out = -K1 * (S / sqrt(norm(S))) - K2 * integral_sign_S + D;
end

% --- Equation 93: Sign function definition ---
function sgn = eq93_sign_function(s_i)
    if s_i > 0
        sgn = 1;
    elseif s_i < 0
        sgn = -1;
    else
        sgn = 0;
    end
end

% --- Equation 94: Gain conditions for stability ---
function val = eq94_gain_conditions(k1i, k2i, delta_i)
    val = (k2i > delta_i) && (k1i^2 > 4*k2i);
end

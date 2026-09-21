function dx = droneDynamics(t, state, coeffs_all, mass, gravity, Kp, Kd, Kr, Kw, I, t_leg_start)
%   Given the drone's current 12-state, returns the rate of change of all nine states, using a two-loop position → attitude → force control structure

    num_legs = size(coeffs_all, 3);

    leg = find(t >= t_leg_start, 1, "last");
    leg = min(leg, num_legs);

    t_local = t - t_leg_start(leg);

    pos = state(1:3);
    vel = state(4:6);

    % orientation
    phi = state(7); % roll
    theta = state(8); % pitch
    psi = state(9); % yaw

    % speed of rotation around x, y and z-axes
    p = state(10); % roll rate
    q = state(11); % pitch rate
    r = state(12); % yaw rate

    target_pos = zeros(3,1);
    target_vel = zeros(3,1);
    target_acc = zeros(3,1);

    for axis = 1:3
        [target_pos(axis), target_vel(axis), target_acc(axis)] = evalCubic(coeffs_all(:, axis, leg), t_local);
    end

    e_pos = target_pos - pos;
    e_vel = target_vel - vel;
    
    % Desired force
    F_des = (-mass * gravity) + (Kp * e_pos) + (Kd * e_vel) + (mass * target_acc);  % 3x1 column matrix

    % Desired attitude from F_des
    g = 9.81;
    max_tilt = 0.5; % ±0.5 radians (~28°) of tilt

    % For small angles, sin(θ) ≈ θ and cos(θ) ≈ 1
    % Fx = mass * g * θ

    theta_des = max(min((F_des(1)/(mass*g)), max_tilt), -max_tilt);
    phi_des = max(min((-F_des(2)/(mass*g)), max_tilt), - max_tilt);
    psi_des = 0;

    % Torque needed
    angle_error = [phi_des; theta_des; psi_des] - [phi; theta; psi];
    omega = [p; q; r];

    torque = (Kr * angle_error) - (Kw * omega); % how hard is it to twist around x, y and z
    
    % Angular Acceleration
    omega_dot = I \ torque; % multiply by the inverse of I (shows how fast the spin-rate is changing)

    % Actual thrust direction
    zB = bodyZAxis(phi, theta, psi); % zB is the direction being pushed towards
    u1 = dot(F_des, zB); % u1 is the scalar magnitude (how hard the motors push)

    accel = (u1 * zB) / mass + gravity;

    dx = [vel; accel; p; q; r; omega_dot];

end
%   Sets constants, waypoints, and initial conditions, runs the whole simulation through ode45, and plots the results

% Waypoints

waypoints = [
    1, 10, 15, 20;
    0, 10,  5, 15;
    3,  6,  4,  8
];

%{ 
Random Waypoint Approach
x_range = [0, 20];
y_range = [0, 20];
z_range = [3, 10];

waypoints = [
    x_range(1) + diff(x_range) * rand(1, 4);
    y_range(1) + diff(y_range) * rand(1, 4);
    z_range(1) + diff(z_range) * rand(1, 4)
];
%}

% Constants
mass = 0.5; % kg (AscTec Hummingbird class)
speed = 2; % m/s

gravity = [0; 0; -9.81];
Kp = 0.1089; % proportional
Kd = 0.1087; % derivative

Kr = 12.6583; % angle-error (how hard to correct attitude)
Kw = 0.7257; % spin-damping (how hard to resist current spin)
I = diag([0.02, 0.02, 0.04]); % inertia grid with numbers down diagonal (PLACEHOLDER VALS)

num_legs = size(waypoints, 2); % # of waypoints (4)

prev_point = [0;0;0]; % starting pos
coeffs_all = zeros(4, 3, num_legs); %  4 coeffs, 3 axes, leg segments

waypoints_full = [prev_point, waypoints]; % waypoints_full(:,1) = origin, then 4 waypoints
num_points = size(waypoints_full, 2);

t_segment = zeros(1, num_legs);

for leg = 1:num_legs
    start_point = waypoints_full(:, leg);
    end_point = waypoints_full(:, leg + 1);

    dist = norm(end_point - start_point);
    t_segment(leg) = dist / speed;
end

t_leg_start = [0, cumsum(t_segment)];

velocities = zeros(3, num_points);

% central-difference formula
for i = 2:num_points - 1
    velocities(:, i) = (waypoints_full(:, i+1) - waypoints_full(:, i-1)) / (t_segment(i - 1) + t_segment(i));
end
% velocities(:, 1) and velocities(:, end) stay zero

for leg = 1:num_legs
    start_point = waypoints_full(:, leg);
    end_point = waypoints_full(:, leg + 1);

    v_start = velocities(:, leg);
    v_end = velocities(:, leg + 1);

    for axis = 1:3
        coeffs_all(:, axis, leg) = cubicTrajectory(start_point(axis), end_point(axis), v_start(axis), v_end(axis), t_segment(leg));
    end

end

% x, y, z, v_x, v_y, v_z, phi, theta, psi, p, q, r
initial_state = zeros(12, 1);

% Simulate drone dynamics
[t,state] = ode45(@(t, state) droneDynamics(t, state, coeffs_all, mass, gravity, Kp, Kd, Kr, Kw, I, t_leg_start), [0 t_leg_start(end)], initial_state);

% Gate radius
gate_radius = 1.5;

figure
hold on

% Plot drone trajectory
plot3(state(:,1), state(:,2), state(:,3));

% Plot waypoints
plot3(waypoints(1,:), waypoints(2,:), waypoints(3,:), ...
    'ro', 'MarkerSize', 6, 'MarkerFaceColor', 'r');

% Draw oriented circular gates
theta = linspace(0, 2*pi, 100);

for i = 1:size(waypoints, 2)

    % Current gate center
    center = waypoints(:,i);

    % Determine direction of travel through the gate
    if i == 1
        % First gate: direction from origin to first waypoint
        direction = waypoints(:,i) - [0; 0; 0];

    elseif i == size(waypoints, 2)
        % Last gate: direction from previous waypoint to last waypoint
        direction = waypoints(:,i) - waypoints(:,i-1);

    else
        % Middle gates: average incoming and outgoing directions
        direction_in = waypoints(:,i) - waypoints(:,i-1);
        direction_out = waypoints(:,i+1) - waypoints(:,i);

        direction = direction_in/norm(direction_in) + ...
                    direction_out/norm(direction_out);
    end

    % Normalize gate normal
    normal = direction / norm(direction);

    % Find two vectors perpendicular to the normal
    % First choose a reference vector that isn't parallel to normal
    reference = [0; 0; 1];

    if abs(dot(normal, reference)) > 0.9
        reference = [0; 1; 0];
    end

    % First vector in the gate plane
    u = cross(normal, reference);
    u = u / norm(u);

    % Second vector in the gate plane
    v = cross(normal, u);
    v = v / norm(v);

    % Generate circle
    circle = center + gate_radius * ...
        (u * cos(theta) + v * sin(theta));

    % Plot gate
    plot3(circle(1,:), circle(2,:), circle(3,:), ...
        'g-', 'LineWidth', 2);

end

xlabel("x (m)");
ylabel("y (m)");
zlabel("z (m)");

grid on
axis equal
view(3)

figure
plot(t, state(:,1), t, state(:,2), t, state(:,3))
xlabel('Time (s)');
ylabel('Position (m)');
legend ('x', 'y', 'z');
grid on

figure
plot(t, state(:,7), t, state(:,8))
xlabel('Time (s)');
ylabel('Angle (rad)');
legend ('phi (roll)', 'theta(pitch)');
grid on

figure
plot(t, state(:,10), t, state(:,11))
xlabel('Time (s)');
ylabel('Spin rate (rad/s)');
legend('p','q');
grid on

disp("Time = "+ t(end))
disp(state(end, 1:3))

% Check whether the actual (controlled) flight passed through each
% waypoint treated as a racing gate, and by how much it missed if not
gate_radius = 1.5; % meters -- keep this in sync with the viewer call below
gateResults = evaluateGatePasses(t, state, waypoints, gate_radius);

uavFlightViewer(t, state, waypoints, gateResults, 2, 3, gate_radius);
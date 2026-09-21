function uavFlightViewer(t, state, waypoints, gateResults, arm_scale, ground_margin, gate_radius)
% Interactive UAV Toolbox flight replay with play/pause and camera control
    if nargin < 4, gateResults = []; end
    if nargin < 5 || isempty(arm_scale), arm_scale = 2; end
    if nargin < 6 || isempty(ground_margin), ground_margin = 3; end
    if nargin < 7 || isempty(gate_radius), gate_radius = 1.5; end

    pos   = state(:, 1:3);
    phi   = state(:, 7);   % Roll
    theta = state(:, 8);   % Pitch
    psi   = state(:, 9);   % Yaw

    % --- Precompute uniform time steps (30 FPS) ---
    fps = 30; dt = 1/fps;
    t_uniform = (t(1):dt:t(end))';
    pos_u   = interp1(t, pos,   t_uniform);
    phi_u   = interp1(t, phi,   t_uniform);
    theta_u = interp1(t, theta, t_uniform);
    psi_u   = interp1(t, psi,   t_uniform);
    N = length(t_uniform);

    vel_u = gradient2(pos_u, dt);
    acc_u = gradient2(vel_u, dt);

    % Convert Euler angles directly to quaternions using standard 'ZYX' sequence
    % eul2quat expects angles in [yaw (psi), pitch (theta), roll (phi)] order
    eul_angles = [psi_u, theta_u, phi_u];
    quat_u = eul2quat(eul_angles, 'ZYX');

    % --- Build UAV Scenario ---
    scenario = uavScenario('UpdateRate', fps);
    all_pts = [pos_u; waypoints'];
    xmin = min(all_pts(:,1)) - ground_margin; xmax = max(all_pts(:,1)) + ground_margin;
    ymin = min(all_pts(:,2)) - ground_margin; ymax = max(all_pts(:,2)) + ground_margin;

    % Drone platform setup
    quad = uavPlatform('Drone', scenario, 'ReferenceFrame', 'ENU', ...
        'InitialPosition', pos_u(1,:), ...
        'InitialOrientation', quaternion(quat_u(1,:)));
    updateMesh(quad, 'quadrotor', {arm_scale}, [0.1 0.6 0.9], eye(4));

    fig = figure('Name', 'UAV Flight Viewer', 'NumberTitle', 'off');
    ax = axes('Parent', fig);
    show3D(scenario, 'Parent', ax);
    hold(ax, 'on'); axis(ax, 'equal');
    grid(ax, 'off'); box(ax, 'off');

    % Procedural ground plane
    drawGroundTexture(ax, xmin, xmax, ymin, ymax, -0.02, 2.0);

    % Waypoint Gate rings
    M = size(waypoints, 2);
    for i = 1:M
        wp = waypoints(:, i);
        if isempty(gateResults)
            n = estimateGateNormal(waypoints, i);
            color = [1.0 0.55 0.0]; % Orange
        else
            n = gateResults(i).normal;
            if gateResults(i).passed
                color = [1.0, 0.843, 0.0]; % Yellow
            else
                color = [0.9 0.15 0.15]; % Red
            end
        end
        drawGateRing(ax, wp, n, gate_radius, 0.12, color);
    end

    trailLine = plot3(ax, nan, nan, nan, 'Color', [1 1 0.2], 'LineWidth', 2);

    % Scene lighting
    set(ax, 'Color', [0.53 0.80 0.92]); 
    lighting(ax, 'gouraud');
    material(ax, 'dull');
    headlight = camlight(ax, 'headlight');
    light(ax, 'Style', 'infinite', 'Position', [100 100 200], 'Color', [1 1 0.95]);
    xlabel(ax, 'x (m)'); ylabel(ax, 'y (m)'); zlabel(ax, 'z (m)');
    set(fig, 'Color', 'w');

    ax.Units = 'normalized';
    ax.Position = [0.08 0.16 0.88 0.78];
    panel = uipanel(fig, 'Units', 'normalized', 'Position', [0 0 1 0.12], ...
        'BackgroundColor', [0.15 0.15 0.15]);

    % Playback control state
    k = 1;
    isPlaying = false;
    camMode = 'Isometric';

    playBtn = uicontrol(panel, 'Style', 'pushbutton', 'String', 'Play', ...
        'Units', 'normalized', 'Position', [0.02 0.2 0.08 0.6], ...
        'Callback', @playPauseCallback);
    uicontrol(panel, 'Style', 'pushbutton', 'String', 'Restart', ...
        'Units', 'normalized', 'Position', [0.11 0.2 0.08 0.6], ...
        'Callback', @restartCallback);
    slider = uicontrol(panel, 'Style', 'slider', 'Min', 1, 'Max', N, 'Value', 1, ...
        'Units', 'normalized', 'Position', [0.21 0.3 0.45 0.4], ...
        'Callback', @sliderCallback);
    uicontrol(panel, 'Style', 'text', 'String', 'Camera:', ...
        'Units', 'normalized', 'Position', [0.68 0.2 0.06 0.6], ...
        'BackgroundColor', [0.15 0.15 0.15], 'ForegroundColor', 'w');
    uicontrol(panel, 'Style', 'popupmenu', ...
        'String', {'Isometric', 'Top-Down', 'Side', 'Front', 'Follow (chase)'}, ...
        'Units', 'normalized', 'Position', [0.74 0.3 0.24 0.4], ...
        'Callback', @camCallback);

    applyCamera('Isometric', pos_u(1,:));
    renderFrame(1);

    % Playback loop
    while isvalid(fig)
        if isPlaying
            k = k + 1;
            if k > N
                k = N;
                isPlaying = false;
                set(playBtn, 'String', 'Play');
            end
            renderFrame(k);
        else
            drawnow;
            pause(0.02);
        end
    end

    % Nested Callbacks
    function playPauseCallback(~, ~)
        isPlaying = ~isPlaying;
        if isPlaying
            set(playBtn, 'String', 'Pause');
        else
            set(playBtn, 'String', 'Play');
        end
    end

    function restartCallback(~, ~)
        k = 1;
        isPlaying = true;
        set(playBtn, 'String', 'Pause');
        set(slider, 'Value', 1);
    end

    function sliderCallback(src, ~)
        k = round(get(src, 'Value'));
        isPlaying = false;
        set(playBtn, 'String', 'Play');
        renderFrame(k);
    end

    function camCallback(src, ~)
        modes = get(src, 'String');
        camMode = modes{get(src, 'Value')};
        applyCamera(camMode, pos_u(k,:));
    end

    function applyCamera(mode, dronePos)
        switch mode
            case 'Isometric'
                view(ax, 45, 25);
            case 'Top-Down'
                view(ax, 0, 90);
            case 'Side'
                view(ax, 90, 0);
            case 'Front'
                view(ax, 0, 0);
            case 'Follow (chase)'
                camtarget(ax, dronePos);
                campos(ax, dronePos + [-6 -6 3]);
                camup(ax, [0 0 1]);
        end
        camlight(headlight, 'headlight');
    end

    function renderFrame(kk)
        % Motion state expected by uavPlatform.move: [pos, vel, acc, quat, angularVelocity]
        motion = [pos_u(kk,:), vel_u(kk,:), acc_u(kk,:), quat_u(kk,:), 0 0 0];
        move(quad, motion);
        
        set(trailLine, 'XData', pos_u(1:kk,1), 'YData', pos_u(1:kk,2), 'ZData', pos_u(1:kk,3));
        
        if strcmp(camMode, 'Follow (chase)')
            applyCamera(camMode, pos_u(kk,:));
        end
        
        show3D(scenario, 'Parent', ax, 'FastUpdate', true);
        title(ax, sprintf('t = %.2f s  (%d/%d)', t_uniform(kk), kk, N));
        set(slider, 'Value', kk);
        drawnow limitrate;
    end
end

function g = gradient2(x, dt)
    g = gradient(x')' / dt;
end

function n = estimateGateNormal(waypoints, i)
    M = size(waypoints, 2);
    if i == 1
        tangent = waypoints(:, min(i+1, M)) - waypoints(:, i);
    elseif i == M
        tangent = waypoints(:, i) - waypoints(:, i-1);
    else
        tangent = waypoints(:, i+1) - waypoints(:, i-1);
    end
    n = tangent / norm(tangent);
end

function h = drawGateRing(ax, center, normal, R, r, faceColor)
    n = normal(:) / norm(normal);
    arbitrary = [0; 0; 1];
    if abs(dot(n, arbitrary)) > 0.95
        arbitrary = [0; 1; 0];
    end
    e1 = cross(n, arbitrary); e1 = e1 / norm(e1);
    e2 = cross(n, e1); e2 = e2 / norm(e2);
    nb = 30; na = 12;
    bAngles = linspace(0, 2*pi, nb);
    aAngles = linspace(0, 2*pi, na);
    [B, A] = meshgrid(bAngles, aAngles);
    X = zeros(size(B)); Y = X; Z = X;
    for ii = 1:numel(B)
        eR = cos(B(ii)) * e1 + sin(B(ii)) * e2;
        pt = center + (R + r * cos(A(ii))) * eR + r * sin(A(ii)) * n;
        X(ii) = pt(1); Y(ii) = pt(2); Z(ii) = pt(3);
    end
    h = surf(ax, X, Y, Z, 'FaceColor', faceColor, 'EdgeColor', 'none', ...
        'FaceLighting', 'gouraud');
end

function drawGroundTexture(ax, xmin, xmax, ymin, ymax, z, tileSize)
    nx = max(2, ceil((xmax - xmin) / tileSize));
    ny = max(2, ceil((ymax - ymin) / tileSize));
    xEdges = linspace(xmin, xmax, nx + 1);
    yEdges = linspace(ymin, ymax, ny + 1);
    [X, Y] = meshgrid(xEdges, yEdges);
    Z = ones(size(X)) * z;
    [Cx, Cy] = meshgrid(0:nx-1, 0:ny-1);
    checker = mod(Cx + Cy, 2);
    darkColor  = [0.10 0.35 0.12];
    lightColor = [0.16 0.45 0.19];
    C = zeros(ny, nx, 3);
    for iy = 1:ny
        for ix = 1:nx
            if checker(iy, ix) == 0
                C(iy, ix, :) = darkColor;
            else
                C(iy, ix, :) = lightColor;
            end
        end
    end
    surf(ax, X, Y, Z, C, 'FaceColor', 'flat', 'EdgeColor', 'none', ...
        'FaceLighting', 'gouraud');
end
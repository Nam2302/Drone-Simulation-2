function results = evaluateGatePasses(t, state, waypoints, gate_radius)
%  Check whether the *actual flown* trajectory passes

    if nargin < 4 || isempty(gate_radius), gate_radius = 1.5; end

    pos = state(:, 1:3);
    M = size(waypoints, 2);

    results = struct('index', {}, 'position', {}, 'normal', {}, ...
        'missDistance', {}, 'passed', {}, 'crossTime', {});

    fprintf('\n--- Gate Pass Report (radius = %.2f m) ---\n', gate_radius);

    for i = 1:M
        if i == 1
            tangent = waypoints(:, min(i+1, M)) - waypoints(:, i);
        elseif i == M
            tangent = waypoints(:, i) - waypoints(:, i-1);
        else
            tangent = waypoints(:, i+1) - waypoints(:, i-1);
        end
        n = tangent / norm(tangent);
        wp = waypoints(:, i);

        % Signed distance of every sample along the gate normal; the
        % sample closest to zero is when the drone crosses the gate plane
        s = (pos - wp') * n;
        [~, idx] = min(abs(s));
        crossPos = pos(idx, :)';

        inPlaneVec = (crossPos - wp) - dot(crossPos - wp, n) * n;
        missDist = norm(inPlaneVec);
        passed = missDist <= gate_radius;

        results(i).index = i; %#ok<AGROW>
        results(i).position = wp;
        results(i).normal = n;
        results(i).missDistance = missDist;
        results(i).passed = passed;
        results(i).crossTime = t(idx);

        status = "MISS";
        if passed, status = "PASS"; end
        fprintf('Gate %d: %-4s | miss distance = %.3f m | crossed at t = %.2f s\n', ...
            i, status, missDist, t(idx));
    end
end
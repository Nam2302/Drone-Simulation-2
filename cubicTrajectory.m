function coeffs = cubicTrajectory(x_start, x_end, v_start, v_end, T)
%   Solve for a0, a1, a2 and a3
    A = [1, 0,  0,      0;      % x(0) = a0
         1, T,  T^2,    T^3;    % x(T) = a0 + a1*T + a2*T^2 + a3*T^3
         0, 1,  0,      0;      % xdot(0) = a1
         0, 1,  2*T,    3*T^2]; % xdot(T) = a1 + 2*a2*T + 3*a3*T^2
  
    b = [x_start; x_end; v_start; v_end];

    coeffs = A \ b;

end


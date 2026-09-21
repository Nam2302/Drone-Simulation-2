function [pos, vel, acc] = evalCubic(coeffs, t)
%   Determine positive, velocity and acceleration at one instance
    a0 = coeffs(1);
    a1 = coeffs(2);
    a2 = coeffs(3); 
    a3 = coeffs(4);

    pos = a0 + a1*t + a2*t^2 + a3*t^3;
    vel = a1 + 2*a2*t + 3*a3*t^2;
    acc = 2*a2 + 6*a3*t;

end
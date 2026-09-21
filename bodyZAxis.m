function zB = bodyZAxis(phi, theta, psi)
%   Shows which world-frame direction is the drone's thrust currently pointing to using the current tilt and rotation matrix
    R = rotationalMatrix(phi, theta, psi);
    zB = R * [0; 0; 1];

end

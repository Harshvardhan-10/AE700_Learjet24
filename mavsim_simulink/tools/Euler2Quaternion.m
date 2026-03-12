
function quaternion = Euler2Quaternion(phi, theta, psi)
    % Converts Euler angles (roll, pitch, yaw) to quaternion
    % Rotation sequence: ZYX (yaw -> pitch -> roll)

    cphi = cos(phi/2);
    sphi = sin(phi/2);

    cth = cos(theta/2);
    sth = sin(theta/2);

    cpsi = cos(psi/2);
    spsi = sin(psi/2);

    e0 = cphi*cth*cpsi + sphi*sth*spsi;
    e1 = sphi*cth*cpsi - cphi*sth*spsi;
    e2 = cphi*sth*cpsi + sphi*cth*spsi;
    e3 = cphi*cth*spsi - sphi*sth*cpsi;

    quaternion = [e0; e1; e2; e3];
end

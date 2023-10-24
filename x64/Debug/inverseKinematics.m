function qp = inverseKinematics(q, xp)
    % Assuming q, xe, and xp are column vectors
    J = jacobiann(q); % Assuming jacobian function computes the Jacobian matrix
    Jdagger = pinv(J); % Pseudo-inverse of Jacobian matrix
    qp = Jdagger *  xp';
end
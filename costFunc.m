function J = costFunc(robot, q, pd, z_desired)
    T = robot.fkine(q);
    pos_err = norm(T.t - pd);

    R = T.R;         % matriz de rotación 3x3
    z_act = R(:, 3); % tercera columna = eje Z actual del efector

    angle_err_rad = acos(dot(z_act, z_desired));
    w_pos = 1;
    w_ori = 100;
    J = w_pos * pos_err + w_ori * angle_err_rad^2;
end

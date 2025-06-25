function main_trayectoria()
    clc; clear;

    % Crear robot
    robot = createRobot();

    % Parámetros del área
    x_const = 200;
    y_min = -50; y_max = 50;
    z_min = 150; z_max = 250;

    % Resolución del barrido
    pasos_y = 10;
    pasos_z = 10;

    ys_fwd = linspace(y_min, y_max, pasos_y);
    ys_bwd = linspace(y_max, y_min, pasos_y);
    zs = linspace(z_min, z_max, pasos_z);

    % Construir trayectoria de barrido
    tray_cart = zeros(3, pasos_y * pasos_z);
    idx = 1;

    for i = 1:pasos_z
        z = zs(i);
        ys = ys_fwd;
        if mod(i, 2) == 0
            ys = ys_bwd;
        end
        for j = 1:pasos_y
            tray_cart(:, idx) = [x_const; ys(j); z];
            idx = idx + 1;
        end
    end

    % Eje Z deseado (apunta en X)
    z_desired = [1; 0; 0];

    % Resolver IK para cada punto
    num_pasos = size(tray_cart, 2);
    q_tray = zeros(num_pasos, 5);

    % Para registrar errores
    pos_errores = zeros(1, num_pasos);
    ang_errores = zeros(1, num_pasos);
    puntos_fallidos = [];

    for i = 1:num_pasos
        pd_i = tray_cart(:, i);

        if i == 1
            q0 = deg2rad([0 0 0 0 0]);
        else
            q0 = q_tray(i-1, :);
        end

        try
            q_tray(i, :) = solveIK(robot, pd_i, z_desired, q0);

            T_i = robot.fkine(q_tray(i, :));
            R = T_i.R;
            z_act = R(:, 3);

            pos_err = norm(T_i.t - pd_i);
            angle_err = acosd(dot(z_act, z_desired));

            pos_errores(i) = pos_err;
            ang_errores(i) = angle_err;

            fprintf('Paso %d | Pos err: %.6f mm | Ang err: %.4f°\n', i, pos_err, angle_err);
        catch
            fprintf('⚠️  Error en paso %d\n', i);
            q_tray(i, :) = NaN;
            puntos_fallidos(end+1) = i;
        end
    end

    % Visualización
    figure;
    robot.plot(q_tray, 'trail', {'m', 'LineWidth', 2});
    title('Barrido en área 10x10 cm en plano Y-Z');

    % Inicializar puerto serial
    s = serialport("COM4", 115200);

    % Enviar ángulos al microcontrolador
    for i = 1:num_pasos
        if any(isnan(q_tray(i, :)))
            continue;  % Saltar puntos fallidos
        end
        offsets = [60, 0, 90, 160, 0];
        angles_adj = round(rad2deg(q_tray(i, :)) + offsets);

        cmd = sprintf('%d,%d,%d,%d,%d', angles_adj);
        writeline(s, cmd);
        fprintf('Enviado: %s\n', cmd);

        robot.plot(q_tray(i, :));
        pause(0.1);
    end

    % Último envío por seguridad
    if exist('angles_adj', 'var')
        cmd = sprintf('%d,%d,%d,%d,%d', angles_adj);
        writeline(s, cmd);
    end

    % Mostrar resumen de errores
    fprintf('\nResumen de errores:\n');
    fprintf('Máximo error de posición: %.2f mm\n', max(pos_errores));
    fprintf('Máximo error angular: %.2f°\n', max(ang_errores));
    if ~isempty(puntos_fallidos)
        fprintf('Puntos fallidos: %s\n', mat2str(puntos_fallidos));
    else
        fprintf('Todos los puntos se resolvieron exitosamente.\n');
    end
end

function robot = createRobot()
    L(1) = RevoluteMDH('a', 0,   'alpha', 0,       'd', 110);
    L(2) = RevoluteMDH('a', 0,   'alpha', pi/2,    'd', 0);
    L(3) = RevoluteMDH('a', 100, 'alpha', pi,      'd', 0);
    L(4) = RevoluteMDH('a', 95,  'alpha', 0,       'd', 0);
    L(5) = RevoluteMDH('a', 30,  'alpha', -pi/2,   'd', 130);
    robot = SerialLink(L, 'name', 'MiRobot');
end
function q_sol = solveIK(robot, pd, z_desired, q0)
    cost = @(q) costFunc(robot, q, pd, z_desired);
    [lb, ub] = jointLimits();

    opts = optimoptions('fmincon', 'Display', 'off', ...
        'Algorithm', 'sqp', 'MaxFunctionEvaluations', 3000);

    try
        q_sol = fmincon(cost, q0, [], [], [], [], lb, ub, [], opts);
    catch ME
        disp('Error con el punto:');
        disp(pd');
        disp('q0 usado:');
        disp(rad2deg(q0));
        rethrow(ME);
    end
end
function [lb, ub] = jointLimits()
    lb = deg2rad([-60, 0, -50, -160, 0]);
    ub = deg2rad([ 120, 80, 130,   0, 0]);
end
function J = costFunc(robot, q, pd, z_desired)
    T = robot.fkine(q);
    pos_err = norm(T.t - pd);

    R = T.R;         % matriz de rotación 3x3
    z_act = R(:, 3); % tercera columna = eje Z actual del efector

    dot_val = max(min(dot(z_act, z_desired), 1), -1);
    angle_err_rad = acos(dot_val);

    w_pos = 1;
    w_ori = 100;
    J = w_pos * pos_err + w_ori * angle_err_rad^2;
end


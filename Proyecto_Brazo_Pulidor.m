function Proyecto_Brazo_Pulidor()
    clc; clear;

    % Inicialización del robot mediante función externa
    robot = createRobot();

    % Configuración de posición HOME del robot
    % Ángulos en grados para cada articulación del robot de 5 DOF
    angles_home = [0, 140, 80, -110, 0];
    q_home = deg2rad(angles_home)'; % Conversión a radianes

    % Cálculo de la posición cartesiana HOME mediante cinemática directa
    T_home = robot.fkine(q_home);
    pos_origen = T_home.t; % Extrae vector de posición (x,y,z)
    fprintf('Posición HOME calculada: [%.1f, %.1f, %.1f] mm\n', pos_origen);

    % Definición de punto de acercamiento previo al barrido
    pos_acercamiento = [100; 0; 170];

    % Parámetros del área de barrido rectangular
    x_const = 200;          % Coordenada X fija del plano de barrido
    y_min = -50; y_max = 50; % Límites en Y (ancho de barrido: 100mm)
    z_min = 200; z_max = 300; % Límites en Z (altura de barrido: 100mm)

    % Configuración del patrón de barrido zigzag
    num_lineas_barrido = 12; % Número total de líneas horizontales
    separacion_lineas = (z_max - z_min) / (num_lineas_barrido - 1); % Espaciado uniforme en Z

    % Vector de orientación deseada del end-effector (apuntando en X)
    z_desired = [1; 0; 0];

    % Offsets progresivos en X para crear efecto de barrido inclinado
    U_max = 30; U_min = -5;
    offsets_x = linspace(U_max, U_min, num_lineas_barrido); % Decremento lineal

    % Generación de puntos de barrido en patrón zigzag
    puntos_barrido = [];
    
    for i = 1:num_lineas_barrido
        % Cálculo de coordenadas Z progresivas
        z_actual = z_min + (i-1) * separacion_lineas;
        x_actual = x_const + offsets_x(i);

        % Alternancia de dirección Y para crear patrón zigzag
        if mod(i, 2) == 1  % Líneas impares: izquierda a derecha
            y_inicio = y_min;
            y_fin = y_max;
        else               % Líneas pares: derecha a izquierda
            y_inicio = y_max;
            y_fin = y_min;
        end

        % Adición de puntos inicial y final de cada línea de barrido
        puntos_barrido = [puntos_barrido, [x_actual; y_inicio; z_actual]];
        puntos_barrido = [puntos_barrido, [x_actual; y_fin; z_actual]];
    end

    % Construcción de secuencia completa de waypoints cartesianos
    waypoints_cartesianos = [
        pos_acercamiento, ...  % Punto de aproximación inicial
        puntos_barrido, ...    % Todos los puntos de barrido
        pos_acercamiento       % Retorno al punto de aproximación
    ];

    % Resolución de cinemática inversa para todos los waypoints
    n_waypoints_cartesianos = size(waypoints_cartesianos, 2);
    waypoints_articulares_ik = zeros(5, n_waypoints_cartesianos);

    fprintf('Resolviendo cinemática inversa para %d waypoints...\n', n_waypoints_cartesianos);

    % Resolución iterativa usando configuración previa como semilla
    q0 = q_home; % Semilla inicial
    for i = 1:n_waypoints_cartesianos
        try
            % solveIK: función externa que resuelve cinemática inversa
            % Parámetros: robot, posición_deseada, orientación_deseada, configuración_inicial
            waypoints_articulares_ik(:, i) = solveIK(robot, waypoints_cartesianos(:, i), z_desired, q0);
            q0 = waypoints_articulares_ik(:, i); % Actualiza semilla para continuidad
            fprintf('Waypoint %d: OK\n', i);
        catch ME
            fprintf('Error en waypoint %d: %s\n', i, ME.message);
            fprintf('Posición problemática: [%.1f, %.1f, %.1f]\n', waypoints_cartesianos(:, i));
            return;
        end
    end

    % Construcción de trayectoria articular completa incluyendo HOME
    waypoints_articulares = [
        q_home, ...                    % Posición inicial HOME
        waypoints_articulares_ik, ...  % Waypoints de barrido
        q_home                         % Retorno a HOME
    ];

    n_waypoints_total = size(waypoints_articulares, 2);

    % Definición de tiempos de segmento para diferentes tipos de movimiento
    time_segments = [3, 2]; % HOME->acercamiento(3s), acercamiento->primer_punto(2s)
    
    % Tiempos alternados para barrido: movimientos horizontales lentos, transiciones rápidas
    for i = 1:(2*num_lineas_barrido-1)
        if mod(i, 2) == 1
            time_segments = [time_segments, 2.0]; % Movimientos de barrido horizontal: 2s
        else
            time_segments = [time_segments, 0.3]; % Transiciones verticales: 0.3s
        end
    end
    time_segments = [time_segments, 2, 3]; % Retorno y HOME final

    % Generación de trayectoria suave mediante polinomios cúbicos
    dt = 0.15; % Paso de tiempo para discretización (segundos)
    fprintf('Generando trayectoria cúbica para barrido zigzag...\n');
    [q_trajectory, time_points] = generateCubicTrajectory(waypoints_articulares, time_segments, dt);
    fprintf('Trayectoria generada: %d puntos en %.1f segundos\n', size(q_trajectory, 1), max(time_points));

    % Inicialización de comunicación serial con servomotores
    try
        s = serialport("COM4", 115200); % Puerto COM4, velocidad 115200 baud
        fprintf('Puerto serial COM4 inicializado correctamente.\n');
        puerto_disponible = true;
    catch ME
        fprintf('Error al inicializar puerto serial: %s\n', ME.message);
        fprintf('Continuando en modo simulación...\n');
        s = [];
        puerto_disponible = false;
    end

    % Ejecución de la trayectoria generada
    executeTrajectoryWithServos_Barrido(robot, q_trajectory, time_points, s, puerto_disponible, num_lineas_barrido, angles_home);

    % Cierre seguro del puerto serial
    if puerto_disponible && ~isempty(s)
        try
            clear s;
            fprintf('Puerto serial cerrado correctamente.\n');
        catch ME
            fprintf('Error al cerrar puerto serial: %s\n', ME.message);
        end
    end

    fprintf('Barrido zigzag completado exitosamente!\n');
end

%% FUNCIÓN: Generación de Trayectorias Cúbicas
function [q_trajectory, time_points] = generateCubicTrajectory(waypoints, time_segments, dt)
    % Genera trayectoria suave mediante polinomios cúbicos por segmentos
    % 
    % Parámetros:
    %   waypoints: [n_joints x n_waypoints] - Configuraciones articulares objetivo
    %   time_segments: [1 x n_segments] - Duración de cada segmento
    %   dt: escalar - Paso de tiempo para discretización
    %
    % Salidas:
    %   q_trajectory: [n_points x n_joints] - Trayectoria discretizada
    %   time_points: [1 x n_points] - Instantes de tiempo correspondientes
    
    [n_joints, n_waypoints] = size(waypoints);
    n_segments = n_waypoints - 1;
    
    % Validación de dimensiones
    if length(time_segments) ~= n_segments
        error('El número de segmentos de tiempo debe ser n_waypoints - 1');
    end
    
    q_trajectory = [];
    time_points = [];
    current_time = 0;
    
    % Procesamiento de cada segmento entre waypoints consecutivos
    for segment = 1:n_segments
        % Configuraciones inicial y final del segmento actual
        q_start = waypoints(:, segment);
        q_end = waypoints(:, segment + 1);
        T = time_segments(segment); % Duración del segmento
        
        % Condiciones de frontera: velocidades nulas en extremos
        % Esto garantiza movimientos suaves sin sacudidas bruscas
        qd_start = zeros(n_joints, 1);
        qd_end = zeros(n_joints, 1);
        
        % Vector de tiempo discretizado para el segmento
        t_segment = 0:dt:T;
        if t_segment(end) ~= T
            t_segment = [t_segment, T]; % Garantiza llegada exacta al tiempo final
        end
        n_points_segment = length(t_segment);
        
        % Matriz para almacenar trayectoria del segmento
        q_segment = zeros(n_points_segment, n_joints);
        
        % Cálculo de polinomio cúbico para cada articulación independientemente
        for joint = 1:n_joints
            % Coeficientes del polinomio cúbico: q(t) = a0 + a1*t + a2*t² + a3*t³
            % Sistema de ecuaciones basado en condiciones de frontera:
            % q(0) = q_start, q(T) = q_end, q'(0) = 0, q'(T) = 0
            
            a0 = q_start(joint);                                                    % Posición inicial
            a1 = qd_start(joint);                                                  % Velocidad inicial
            a2 = (3*(q_end(joint) - q_start(joint))/T^2) - (2*qd_start(joint)/T) - (qd_end(joint)/T);
            a3 = (2*(q_start(joint) - q_end(joint))/T^3) + (qd_start(joint)/T^2) + (qd_end(joint)/T^2);
            
            % Evaluación del polinomio en cada instante de tiempo
            for i = 1:n_points_segment
                t = t_segment(i);
                q_segment(i, joint) = a0 + a1*t + a2*t^2 + a3*t^3;
            end
        end
        
        % Concatenación con trayectoria global
        if segment == 1
            % Primer segmento: incluir todos los puntos
            q_trajectory = [q_trajectory; q_segment];
            time_points = [time_points, current_time + t_segment];
        else
            % Segmentos posteriores: evitar duplicación del punto inicial
            q_trajectory = [q_trajectory; q_segment(2:end, :)];
            time_points = [time_points, current_time + t_segment(2:end)];
        end
        
        current_time = current_time + T; % Actualización del tiempo acumulado
    end
end

%% FUNCIÓN: Ejecución de Trayectoria con Visualización
function executeTrajectoryWithServos_Barrido(robot, q_trajectory, time_points, serial_port, puerto_disponible, num_lineas_barrido, angles_home)
    % Ejecuta trayectoria de barrido con control de servomotores y visualización 3D
    %
    % Parámetros:
    %   robot: objeto robot para cinemática directa y visualización
    %   q_trajectory: [n_points x n_joints] - Trayectoria articular discretizada
    %   time_points: [1 x n_points] - Instantes de tiempo correspondientes
    %   serial_port: objeto de puerto serial para comunicación con servos
    %   puerto_disponible: booleano - indica si hay comunicación serial activa
    %   num_lineas_barrido: número de líneas del patrón de barrido
    %   angles_home: configuración HOME en grados
    
    n_points = size(q_trajectory, 1);
    offsets = [60, -40, 50, 130, 0]; % Offsets de calibración para servomotores MG996R
    
    fprintf('Ejecutando trayectoria de barrido zigzag con %d puntos...\n', n_points);
    fprintf('Posición HOME: [%d, %d, %d, %d, %d] grados\n', angles_home);
    
    % Configuración de ventana de visualización 3D
    figure;
    hold on;
    grid on;
    axis equal;
    title('Ejecución de Trayectoria Cúbica - Barrido Zigzag de Pizarra');
    xlabel('X (mm)');
    ylabel('Y (mm)');
    zlabel('Z (mm)');
    
    % Matriz para almacenar trayectoria del end-effector
    tray_end_effector = zeros(3, n_points);
    
    % Paleta de colores para visualización de diferentes líneas de barrido
    colores_barrido = ['r', 'b', 'm', 'c', 'g', 'y', 'k', 'r'];
    
    start_time = tic; % Inicio del cronómetro para sincronización temporal
    
    % Bucle principal de ejecución punto a punto
    for i = 1:n_points
        % Sincronización temporal: esperar hasta el instante objetivo
        target_time = time_points(i);
        while toc(start_time) < target_time
            pause(0.001); % Pausa mínima para evitar saturación del CPU
        end
        
        % Conversión de ángulos articulares para servomotores
        angles_deg = rad2deg(q_trajectory(i, :));      % Radianes a grados
        angles_adj = round(angles_deg + offsets);      % Aplicación de offsets de calibración
        angles_adj = max(0, min(180, angles_adj));     % Limitación a rango físico [0,180]°
        
        % Creación de comando de comunicación serial
        cmd = sprintf('%d,%d,%d,%d,%d', angles_adj);
        
        % Envío de comando a servomotores (si hay comunicación disponible)
        if puerto_disponible
            try
                writeline(serial_port, cmd);
                
                % Notificación para comandos especiales (HOME)
                if i == 1
                    fprintf('Enviando comando HOME inicial: %s\n', cmd);
                elseif i == n_points
                    fprintf('Enviando comando HOME final: %s\n', cmd);
                end
                
            catch ME
                fprintf('Error enviando comando en punto %d: %s\n', i, ME.message);
            end
        else
            % Modo simulación: mostrar comandos HOME sin envío físico
            if i == 1
                fprintf('HOME inicial (simulación): %s\n', cmd);
            elseif i == n_points
                fprintf('HOME final (simulación): %s\n', cmd);
            end
        end
        
        % Cálculo de posición cartesiana del end-effector mediante cinemática directa
        T = robot.fkine(q_trajectory(i, :));
        tray_end_effector(:, i) = T.t;
        
        % Visualización de configuración actual del robot
        robot.plot(q_trajectory(i, :), 'trail', {'k', 'LineWidth', 1});
        
        % Generación de estela visual con codificación por colores según fase de movimiento
        if i > 1
            % Clasificación del tipo de movimiento actual
            color_actual = 'k';
            estilo_actual = '-';
            ancho_linea = 2;
            fase_nombre = 'Desconocida';
            
            % Cálculo de límites aproximados de fases
            punto_inicio_barrido = 3;
            punto_fin_barrido = punto_inicio_barrido + 2*num_lineas_barrido;
            
            if i <= punto_inicio_barrido * 10
                % Fase de movimientos iniciales (HOME -> acercamiento)
                color_actual = 'g';
                estilo_actual = '--';
                fase_nombre = 'Movimiento inicial';
            elseif i >= punto_fin_barrido * 10
                % Fase de retorno a HOME
                color_actual = 'g';
                estilo_actual = '--';
                fase_nombre = 'Regreso a home';
            else
                % Fase de barrido activo
                proporcion_barrido = (i - punto_inicio_barrido*10) / ((punto_fin_barrido - punto_inicio_barrido)*10);
                linea_actual = min(num_lineas_barrido, ceil(proporcion_barrido * num_lineas_barrido));
                
                if mod(floor(proporcion_barrido * 2 * num_lineas_barrido), 2) == 0
                    % Movimiento horizontal de barrido (trabajo efectivo)
                    color_actual = colores_barrido(mod(linea_actual-1, length(colores_barrido)) + 1);
                    estilo_actual = '-';
                    ancho_linea = 3;
                    fase_nombre = sprintf('Barrido línea %d', linea_actual);
                else
                    % Movimiento vertical de transición entre líneas
                    color_actual = [0.5, 0.5, 0.5]; % Gris
                    estilo_actual = ':';
                    ancho_linea = 1;
                    fase_nombre = 'Transición entre líneas';
                end
            end
            
            % Dibujo del segmento de estela con estilo correspondiente
            plot3(tray_end_effector(1, i-1:i), ...
                  tray_end_effector(2, i-1:i), ...
                  tray_end_effector(3, i-1:i), ...
                  'Color', color_actual, 'LineStyle', estilo_actual, 'LineWidth', ancho_linea);
        end
        
        % Indicador de progreso cada 30 puntos
        if mod(i, 30) == 0
            fprintf('Progreso: %d/%d puntos (%.1f%%)\n', i, n_points, 100*i/n_points);
        end
    end
    
    % Generación de leyenda explicativa
    h_handles = [];
    h_labels = {};
    
    % Entrada para movimientos HOME/acercamiento
    h_handles(end+1) = plot3(NaN, NaN, NaN, 'g--', 'LineWidth', 2);
    h_labels{end+1} = 'Movimientos HOME/Acercamiento';
    
    % Entradas para líneas de barrido (máximo 4 en leyenda para claridad)
    for i = 1:min(4, num_lineas_barrido)
        color_linea = colores_barrido(mod(i-1, length(colores_barrido)) + 1);
        h_handles(end+1) = plot3(NaN, NaN, NaN, 'Color', color_linea, 'LineStyle', '-', 'LineWidth', 3);
        h_labels{end+1} = sprintf('Línea de barrido %d', i);
    end
    
    % Entrada para transiciones
    h_handles(end+1) = plot3(NaN, NaN, NaN, 'Color', [0.5, 0.5, 0.5], 'LineStyle', ':', 'LineWidth', 1);
    h_labels{end+1} = 'Transiciones';
    
    legend(h_handles, h_labels, 'Location', 'best');
    
    % Información de área barrida como texto en el gráfico
    text(0, 0, 350, sprintf('Área barrida: %.0f x %.0f mm\nLíneas de barrido: %d', ...
                           100, 100, num_lineas_barrido), ...
         'FontSize', 10, 'BackgroundColor', 'white', 'EdgeColor', 'black');
    
    fprintf('Barrido zigzag completado exitosamente!\n');
    fprintf('Área total barrida con %d líneas horizontales\n', num_lineas_barrido);
end
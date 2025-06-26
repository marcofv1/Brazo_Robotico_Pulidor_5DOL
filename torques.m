function tau_exp = torques()
    % Definir variables simbólicas
    syms th1 th2 th3 th4 th5 real
    syms Fx Fy Fz real

    % Posición simbólica del efector final
    px = (125 + 110*sin(th3 - th4) + 100*cos(th4)) / 1000;
    py = 130 / 1000;
    pz = (-110*cos(th3 - th4) - 100*sin(th4)) / 1000;
    p = [px; py; pz];

    % Vector de variables articulares
    q = [th1; th2; th3; th4; th5];

    % Jacobiana traslacional
    Jv = jacobian(p, q);

    % Fuerza simbólica
    F = [Fx; Fy; Fz];

    % Torques simbólicos: τ = Jᵗ * F
    tau = simplify(transpose(Jv) * F);

    % Configuración articular (en grados)
    q_val_deg = [60; 0; 90; 100; 0];
    q_val = deg2rad(q_val_deg);  % a radianes

    % Sustitución de ángulos en los torques
    vars = [th1, th2, th3, th4, th5];
    tau_eval = subs(tau, vars, q_val.');

    % Valores experimentales de fuerza vertical (gramos-fuerza)
    Fz_gN = [650.1; 685.4; 725.6; 710.3; 685.9];

    % Conversión a Newtons (1 g ≈ 9.81 m/s² / 1000)
    Fz_avg_N = mean(Fz_gN) * 9.81 / 1000;

     % Aplicar signo negativo porque el robot empuja hacia abajo
    F_num = [0; 0; -Fz_avg_N];

    % Sustitución de fuerzas en la expresión de torque (convertido a fila)
    tau_exp = double(subs(tau_eval, [Fx, Fy, Fz], F_num.'));


    % Mostrar resultados
    fprintf('--- RESULTADOS EXPERIMENTALES ---\n');
    fprintf('Configuración q = [60; 0; 90; 100; 0] grados\n');
    fprintf('Fuerza promedio medida: %.3f N (dirección -Z)\n\n', Fz_avg_N);
    fprintf('Torques resultantes en las articulaciones:\n');
    for i = 1:5
        fprintf('τ_%d = %.4f Nm\n', i, tau_exp(i));
    end
end

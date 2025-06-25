function J_sym = getSymbolicJacobianWithPartialDerivatives()
%getSymbolicJacobianWithPartialDerivatives Calcula y devuelve la matriz Jacobiana simbólica
%   e imprime cada una de sus derivadas parciales constituyentes.
%
%   Salida:
%     J_sym: Matriz Jacobiana de 3x5 en formato simbólico.

    % Declarar variables simbólicas
    syms th1 th2 th3 th4 th5 real

    % Definir las ecuaciones de posición x, y, z
    x_sym = 110*sin(th3 - th4) + 100*cos(th4) + 125;
    y_sym = 130;
    z_sym = - 110*cos(th3 - th4) - 100*sin(th4);

    % Crear un vector simbólico de las coordenadas de posición
    position_vector_sym = [x_sym; y_sym ; z_sym];

    % Crear un vector simbólico de las variables articulares (thetas)
    joint_variables_sym = [th3, th4];

    % Calcular el Jacobiano usando la función 'jacobian'
    J_sym = jacobian(position_vector_sym, joint_variables_sym);

    % --- Imprimir derivadas parciales individuales ---
    fprintf('--- Derivadas Parciales Individuales ---\n\n');

    % Derivadas de x
    fprintf('Derivadas de x:\n');
    fprintf('  dx/d(th1) = %s\n', char(diff(x_sym, th1)));
    fprintf('  dx/d(th2) = %s\n', char(diff(x_sym, th2)));
    fprintf('  dx/d(th3) = %s\n', char(diff(x_sym, th3)));
    fprintf('  dx/d(th4) = %s\n', char(char(diff(x_sym, th4)))); % Usamos char(char()) para una mejor visualización de expresiones complejas
    fprintf('  dx/d(th5) = %s\n', char(char(diff(x_sym, th5)))); % Usamos char(char()) para una mejor visualización de expresiones complejas
    fprintf('\n');

    % Derivadas de y
    fprintf('Derivadas de y:\n');
    fprintf('  dy/d(th1) = %s\n', char(diff(y_sym, th1)));
    fprintf('  dy/d(th2) = %s\n', char(diff(y_sym, th2)));
    fprintf('  dy/d(th3) = %s\n', char(diff(y_sym, th3)));
    fprintf('  dy/d(th4) = %s\n', char(diff(y_sym, th4)));
    fprintf('  dy/d(th5) = %s\n', char(diff(y_sym, th5)));
    fprintf('\n');

    % Derivadas de z
    % fprintf('Derivadas de z:\n');
    % fprintf('  dz/d(th1) = %s\n', char(diff(z_sym, th1)));
    % fprintf('  dz/d(th2) = %s\n', char(diff(z_sym, th2)));
    % fprintf('  dz/d(th3) = %s\n', char(char(diff(z_sym, th3)))); % Usamos char(char())
    % fprintf('  dz/d(th4) = %s\n', char(char(diff(z_sym, th4)))); % Usamos char(char())
    % fprintf('  dz/d(th5) = %s\n', char(char(diff(z_sym, th5)))); % Usamos char(char())
    % fprintf('\n');

    fprintf('--- Matriz Jacobiana Simbólica Completa ---\n');
    
    % det_J = simplify(det(J_sym)); % Calculate the determinant of your 3x3 symbolic Jacobian
    % % Correct way to display text and the symbolic determinant:
    % fprintf('Determinante del J:\n'); % Use fprintf for formatted text
    % disp(det_J);                     % Use disp to display the symbolic expression
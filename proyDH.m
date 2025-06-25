clc; clear; close all;


L(1) = RevoluteMDH('a', 0,     'alpha', 0,      'd', 110);
L(2) = RevoluteMDH('a', 0,     'alpha', pi/2,   'd', 0);
L(3) = RevoluteMDH('a', 100, 'alpha', pi,      'd', 0);
L(4) = RevoluteMDH('a', 95,     'alpha', 0,  'd', 0);
L(5) = RevoluteMDH('a', 30,    'alpha', -pi/2,      'd', 130);


% Crear el modelo del robot con DH Modificado
robot = SerialLink(L, 'name', 'robot1');

% Configuración de los ángulos articulares en grados
q = deg2rad([0 0 0 0 0 ]); % Vector fila de 6 elementos
% Graficar el robot
figure;
robot.plot(q, 'workspace', [-500 500 -500 500 -500 500], 'scale', 0.5, 'jaxes'); 
hold on;
% Graficar los sistemas de referencia de cada articulación
T = eye(4); % Matriz identidad para la base
for i = 1:5
    T = T * robot.A(i, q).T; 
    trplot(T, 'frame', num2str(i), 'length', 50, 'color', 'k'); 
end
hold off;
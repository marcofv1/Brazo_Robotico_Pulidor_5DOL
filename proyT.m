clc; clear;
syms theta1 theta2 theta3 theta4 theta5 real

% Parámetros MDH: [theta, d, a, alpha]
MDH = [theta1, 110,   0,     0;
       theta2, 0,    0,     pi/2;
       theta3, 0,    100,   pi;
       theta4, 0,    95,   0;
       0, 130,    30,    -pi/2];

% Función para matriz de transformación homogénea MDH
mdh_transform = @(theta, d, a, alpha) ...
    [cos(theta),             -sin(theta),            0,              a;
     sin(theta)*cos(alpha),  cos(theta)*cos(alpha), -sin(alpha),    -sin(alpha)*d;
     sin(theta)*sin(alpha),  cos(theta)*sin(alpha), cos(alpha),     cos(alpha)*d;
     0,                      0,                      0,              1];

% Matrices individuales
A1 = simplify(mdh_transform(MDH(1,1), MDH(1,2), MDH(1,3), MDH(1,4)));
A2 = simplify(mdh_transform(MDH(2,1), MDH(2,2), MDH(2,3), MDH(2,4)));
A3 = simplify(mdh_transform(MDH(3,1), MDH(3,2), MDH(3,3), MDH(3,4)));
A4 = simplify(mdh_transform(MDH(4,1), MDH(4,2), MDH(4,3), MDH(4,4)));
A5 = simplify(mdh_transform(MDH(5,1), MDH(5,2), MDH(5,3), MDH(5,4)));

% Imprimir matrices individuales
disp('A1 (A_1^0) ='); disp(A1);
disp('A2 (A_2^1) ='); disp(A2);
disp('A3 (A_3^2) ='); disp(A3);
disp('A4 (A_4^3) ='); disp(A4);
disp('A5 (A_5^4) ='); disp(A5);

% Multiplicación desde la 5 a la 1: T^5_0
T_5_0 = simplify(A5 * A4 * A3 * A2 * A1);
disp('T^5_0 (transformación del efector hacia la base) =');
disp(T_5_0);

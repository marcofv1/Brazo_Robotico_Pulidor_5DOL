function robot = createRobot()
    L(1) = RevoluteMDH('a', 0,   'alpha', 0,       'd', 110);
    L(2) = RevoluteMDH('a', 0,   'alpha', pi/2,    'd', 0);
    L(3) = RevoluteMDH('a', 100, 'alpha', pi,      'd', 0);
    L(4) = RevoluteMDH('a', 95,  'alpha', 0,       'd', 0);
    L(5) = RevoluteMDH('a', 30,  'alpha', -pi/2,   'd', 130);
    robot = SerialLink(L, 'name', 'MiRobot');
end

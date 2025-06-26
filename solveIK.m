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

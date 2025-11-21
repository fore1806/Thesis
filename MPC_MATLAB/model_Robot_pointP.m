function [ xkp1, yk ] = model_Robot_pointP(xk, uk, tau)
    % model_step
    % Function implementing the discrete-time evolution of the system state x(k+1) and output y(k).
    % Obtained discretizing the system dynamics via Forward Euler.
    %
    % x(k+1) = f(x(k), u(k))
    % y(k) = x(k)
    

    % Discretization via Forward Euler
    xkp1 = [xk(1) + tau*uk(1);
            xk(2) + tau*uk(2)];
    yk = xk;

end
function u_opt = FHOCP(xk, Q, R, P, Y, N, n, n_u, tau_s, pose_ref, theta_ref, robot_params)
    % FHOCP
    % Solve the Finite Horizon Control Optimization Problem (FHOCP)
    %
    % Arguments
    % - xk: The current state
    % - Q: The state weight
    % - R: The input weight
    % - P: The final state weight
    % - Y: Weight to the feasibility recursivity
    % - N: The prediction horizon
    % - n: System Order
    % - n_u: Numer of Inputs
    % - tau_s: Discretization time-step
    % - pose_ref: Pose reference for the Robot
    % - theta_ref: Heading angle for the Robot
    % - robot_params: Vector containing [d l r]
    %
    % Returns
    % - u_opt: The optimal control move at the current time step (u^*(k))
       
    % Import the CasADi toolkit and instantiate the optimization problem (opti)
    import casadi.*;
    opti = casadi.Opti();
    
    
    %%%%%% Declare the optimization variables %%%%%%
    u = opti.variable(n_u, N);
    epsilon = opti.variable(n_u, N); % Slack Variable
    rho = 1e9*eye(n_u); % Weight on the Slack Variable
    x = opti.variable(n, N+1);
    x_ter_end = opti.variable(n, 1);

    %%%%%%% Limit on the angular speed
    w_physical_limit = 100; 
    
    % We induce a 10% discount on the 
    safety_factor = 0.9; 
    w_safe_limit = w_physical_limit * safety_factor;
    
    % Usamos este límite reducido para las restricciones
    omega_min = -w_safe_limit * ones(n_u, 1); 
    omega_max =  w_safe_limit * ones(n_u, 1);

   
    %%%%%% Impose the constraints %%%%%%
    
    % Initial state constraint: x(k) == xk
    opti.subject_to(x(:,1) == xk);

    % Positive value on the Slack Variables
    opti.subject_to(epsilon(:) >= 0);

    % Input constraints:  u_min <= u(k+i) <= u_max
    % This constraint has to be computed for the model
    for k = 1:N
        M = transformationMatrix(theta_ref(k), robot_params);
        omegas = M*u(:,k);
        %opti.subject_to(omega_min <= omegas <= omega_max);
        % opti.subject_to(omega_min - epsilon(:,k) <= omegas <= omega_max + epsilon(:,k));
        %opti.subject_to(M\omega_min <= u(:, k) <= M\omega_max)
        opti.subject_to(M\omega_min - epsilon(:,k) <= u(:, k) <= M\omega_max + epsilon(:,k));
    end
    
    % Terminal constraint: x(k+N+1) = x(:,end-1) We stopped
    opti.subject_to(x(:,end) == x_ter_end);
    
    %%%%%% System dynamics and cost function %%%%%%
    
    % Cost function initialization
    J = 0;
    
    for ii=1:N      % Note that the MATLAB index ii = 1 corresponds to i = 0 in the formula
        % Set the system dynamics as a constraint:  x(k+i+1) == f(x(k+i), u(k+i))
        opti.subject_to(x(:,ii+1)==model_Robot_pointP(x(:, ii), u(:, ii), tau_s));
        
        % Add the i-th term to the cost function:
        J = J + (u(:,ii))' * R * (u(:,ii)) + (x(:,ii)-pose_ref(:,ii))' * Q * (x(:,ii)-pose_ref(:,ii)) + (epsilon(:,ii)'* rho * epsilon(:,ii));
    end

    J = J + (x(:,end)-pose_ref(:,end))' * P * (x(:,end)-pose_ref(:,end)) + (x_ter_end-pose_ref(:,end))' * Y * (x_ter_end-pose_ref(:,end));
    
    
    %%%%%% Set the initial guess of the optimization variables %%%%%%
    
    % Possible initial guess for x:    x(k+i) = xk  ∀i = 0, ..., N
    opti.set_initial(x, repmat(xk, 1, N+1));   
    
    % Possible initial guess for u:    u(k+i) = u_min  ∀i = 0, ..., N-1
    M = transformationMatrix(theta_ref(1), robot_params);
    opti.set_initial(u, repmat((M\omega_min), 1, N)); 
    opti.set_initial(epsilon, zeros(n_u, N));
    
       
    %%%%%% CasADi Settings (do not change) %%%%%%
    % Declare the cost function
    opti.minimize(J);
    
    % Options of the optimization problem
    prob_opts = struct;
    prob_opts.expand = true;
    prob_opts.ipopt.print_level = 0;    % Disable printing
    prob_opts.print_time = false;       % Do not print the timestamp
    
    % Options of the solver
    ip_opts = struct;
    ip_opts.print_level = 0;            % Disable printing
    ip_opts.max_iter = 1e5;             % Maximum iterations
    ip_opts.compl_inf_tol = 1e-6;
    
    % Set the solver
    opti.solver('ipopt', prob_opts, ip_opts);
    
    
    %%%%%% Solve the optimization problem %%%%%%
    try
        sol = opti.solve();

        % Diagnostic
        eps_vals = sol.value(epsilon); % We extract the values
        max_slack = max(abs(eps_vals(:))); % We find the highest value
        
        if max_slack > 1e-4
            fprintf(2, '¡ALERTA! Active Slack variables. Máx val: %.4f\n', max_slack);
        else
            disp('Restrictions were met.');
        end
        
        % Extract the optimal con;
        u_opt = sol.value(u(:,1));       
    catch EX
        warning("Solver failed");
        u_opt = [0;0]; % fallback: we stop the robot
        %keyboard; % Enter the debug mode
    end

    %disp("------------------------------------")
    %disp(" La Pose Actual ")
    %disp(xk)
    %disp(" Señal de Control ")
    %disp(u_opt)
end
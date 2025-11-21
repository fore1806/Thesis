import casadi.*
opti = casadi.Opti();

% Dummy example

x = opti.variable(2,1);
y = opti.variable(1,1);

opti.subject_to([0;0] <= x <= [1;1]);
opti.subject_to(-1 <= y <= 2);
opti.subject_to(x(1)*y-x(2) == 1);

J = x.'*x + y^2;
opti.minimize(J);

% Optional
opti.set_initial(x,[0.5;0.5]);
opti.set_initial(y, 0);

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

try
    sol = opti.solve();
        
catch EX
    keyboard; % Enter the debug mode
end
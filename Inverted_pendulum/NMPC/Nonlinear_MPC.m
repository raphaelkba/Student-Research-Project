%% Nonlinear MPC for an Inverted Pendulum
% This code presents an implementation of a Nonlinear Model Predictive
% Controller for a Inverted Pendulum carried by a car. The force applied
% to the car swings the pendulum and controls its angle.
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% Inputs:
% inverted_pendulum: inverted pendulum system dynamics
% constraints: R^(n*N) matrix with the desired equalties and inequalities
% contraints for x, u and t for the entire optimization process. n = number
% of inputs, N = horizon
% terminalconstraints: same as contraints, but only for the terminal time
% step
% mpciterations: number of iterations for the optimization process
% N: time horizon
% T: time step
% t_measured: time measured intialy before the mpc optimization
% x_measured: states measured intialy before the mpc optimization
%             x(1): x position of the cart
%             x(2): velocity of the cart in the x direction
%             x(3): angle between the pendulum and the y-axis
%             x(4): angular velocity of the pendulum
% u_0: initial guess for the controller
% desired_states: vector with the final desired states
%             x(1): x position of the cart
%             x(2): velocity of the cart in the x direction
%             x(3): angle between the pendulum and the y-axis
%             x(4): angular velocity of the pendulum
% options: options for fmincon
% sys: system parameters
%   sys.m: pendulum mass (kg)
%   sys.M: car mass (kg)
%   sys.g: gravity force (m/s^2)
%   sys.l: pendulum lenght (m)
%   sys.dt: timestep (s)
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% Output:
% t_all: vector R^m, m: mpciterations, stores the time for each iteration
% step
% x_all: vector R^(m*4), m: mpciterations, stores the states for each
% iteration step
% u_all: vector R^m, m: mpciterations, stores the optimal control input for
% each iteration step
function [t_all, x_all, u_all] = Nonlinear_MPC(inverted_pendulum, constraints, terminalconstraints, ...
    linearconstraints, mpciterations, N, T, t_measured, ...
    x_measured, u_0, desired_states, options, sys, varargin)
if (nargin>=14)
    iprint = varargin{1};
else
    iprint = 0;
end
if (nargin>=15)
    printHeader = varargin{2};
else
    printHeader = @printHeaderDummy;
end
if (nargin>=16)
    printClosedloopData = varargin{3};
else
    printClosedloopData = @printClosedloopDataDummy;
end
if (nargin>=17)
    plotTrajectories = varargin{4};
else
    plotTrajectories = @plotTrajectoriesDummy;
end

% Initialize variables
t_all = [];
x_all = [];
u_all = [];


% Start of the NMPC iteration
mpciter = 0;

while(mpciter < mpciterations)
    %     disp(['Iteration step: ', num2str(mpciter)]);
    % Get initial states
    x_0 = x_measured(end,:);
    t_0 = t_measured(end,:);
    t_Start = tic;
    [u_new, V, exitflag, output, x, t] = solveOptimalControlProblem(inverted_pendulum,constraints, terminalconstraints, ...
        linearconstraints, N, t_0, x_0, u_0, T,options, sys, desired_states);
    t_Elapsed = toc( t_Start );
    %   Print solution
    if ( iprint >= 1 )
        printSolution(printHeader, printClosedloopData, plotTrajectories, mpciter...
            ,iprint, t_Elapsed, u_new, T, t_0,x_0,V,exitflag,output,x,t,lb,ub);
    end
    %   Store closed loop data
    t_all = [ t_all; t_measured ];
    x_all = [ x_all; x_measured];
    u_all = [ u_all; u_new(:,1) ];
    
    %   Shift the input vector to the left (restart)
    %  Without zero-terminal constraints
%             u_0 = [u_new(:,2:size(u_new,2)) u_new(size(u_new,1))];
    %  With zero terminal constraint
    u_0 = [u_new(:,2:size(u_new,2)) 0];
    
    %   Apply control to the system
    [t_measured, x_measured] = ode45(@(t,x) inverted_pendulum(t, x ,u_new(1), sys), [t_0:T/10:t_0+T], x_0);
    mpciter = mpciter+1;
end

% Solves the optimal control problem taking into account the problem
% contraints
%  Inputs:
% inverted_pendulum: inverted pendulum system dynamics
% constraints: R^(n*N) matrix with the desired equalties and inequalities
% contraints for x, u and t for the entire optimization process. n = number
% of inputs, N = horizon
% terminalconstraints: same as contraints, but only for the terminal time
% step
% mpciterations: number of iterations for the optimization process
% N: time horizon
% T: time step
% t_0: time measured intialy before the mpc optimization
% x_0: states measured intialy before the mpc optimization
%             x(1): x position of the cart
%             x(2): velocity of the cart in the x direction
%             x(3): angle between the pendulum and the y-axis
%             x(4): angular velocity of the pendulum
% u_0: initial guess for the controller
% desired_states: vector with the final desired states
%             x(1): x position of the cart
%             x(2): velocity of the cart in the x direction
%             x(3): angle between the pendulum and the y-axis
%             x(4): angular velocity of the pendulum
% options: options for fmincon
% sys: system parameters
%   sys.m: pendulum mass (kg)
%   sys.M: car mass (kg)
%   sys.g: gravity force (m/s^2)
%   sys.l: pendulum lenght (m)
%   sys.dt: timestep (s)
% Output:
% u: optimal control input
    function [u, V, fminconflag, output , x ,t] = solveOptimalControlProblem ...
            (inverted_pendulum, constraints, terminalconstraints, ...
            linearconstraints, N, t_0, x_0, u_0, T, options, sys, desired_states)
        
        % Initialize x
        x = zeros(N+1, length(x_0));
        [x t] = computeOpenloopSolution(inverted_pendulum,N, T, t_0, x_0, u_0, sys);
        

        % Set control and linear bounds
        A = [];
        b = [];
        Aeq = [];
        beq = [];
        lb = [];
        ub = [];
        
        % Separate the linear constraints in different matrices
        for k=1:N
            [Anew, bnew, Aeqnew, beqnew, lbnew, ubnew] = ...
                linearconstraints(t_0+k*T,x(k,:),u_0(:,k));
            A = blkdiag(A,Anew);
            b = [b, bnew];
            Aeq = blkdiag(Aeq,Aeqnew);
            beq = [beq, beqnew];
            lb = [lb, lbnew];
            ub = [ub, ubnew];
        end
        
        cost_function = @(u) pendulumFunction(u,x_0',desired_states,N,sys,inverted_pendulum);
        
        % Solve optimization problem
        [u, V, fminconflag, output] = fmincon(cost_function, u_0, A, b, Aeq, beq, lb, ...
            ub, @(u,x) nonlinearconstraints(inverted_pendulum,constraints, terminalconstraints, ...
            N, T, t_0, x_0, u, sys), options);
        if fminconflag < 1
            error('The optimization problem could not be solved properly')
        end
        
    end

% Gets the nonlinear inquality and equality constraints of the problem
%  Inputs:
% inverted_pendulum: inverted pendulum system dynamics
% constraints: R^(n*N) matrix with the desired equalties and inequalities
% contraints for x, u and t for the entire optimization process. n = number
% of inputs, N = horizon
% terminalconstraints: same as contraints, but only for the terminal time
% step
% mpciterations: number of iterations for the optimization process
% N: time horizon
% T: time step
% t_0: time measured intialy before the mpc optimization
% x_0: states measured intialy before the mpc optimization
%             x(1): x position of the cart
%             x(2): velocity of the cart in the x direction
%             x(3): angle between the pendulum and the y-axis
%             x(4): angular velocity of the pendulum
% u: initial guess for the controller
% sys: system parameters
%   sys.m: pendulum mass (kg)
%   sys.M: car mass (kg)
%   sys.g: gravity force (m/s^2)
%   sys.l: pendulum lenght (m)
%   sys.dt: timestep (s)
% Output:
% c: inequality constraints
% ceq: equality constraints
    function [c,ceq] = nonlinearconstraints(inverted_pendulum,constraints, ...
            terminalconstraints,N, T, t_0, x_0, u, sys)
        % Initialize x matrix
        x_constraints = zeros(N+1, length(x_0));
        [x_constraints t_constraints] = computeOpenloopSolution(inverted_pendulum,N, T, t_0, x_0, u, sys);
        
        c = [];
        ceq = [];
        for k=1:N
            [cnew, ceqnew] = constraints(t_0+k*T,x_constraints(k,:),u(:,k));
            c = [c cnew];
            ceq = [ceq ceqnew];
        end
        [cnew, ceqnew] = terminalconstraints(t_0+(N+1)*T,x_constraints(N+1,:));
        c = [c cnew];
        ceq = [ceq ceqnew];
    end



% Computes the open loop solution for the problem
%  Inputs:
% inverted_pendulum: inverted pendulum system dynamics
% N: time horizon
% T: time step
% t_0: time measured intialy before the mpc optimization
% x_0: states measured intialy before the mpc optimization
%             x(1): x position of the cart
%             x(2): velocity of the cart in the x direction
%             x(3): angle between the pendulum and the y-axis
%             x(4): angular velocity of the pendulum
% u: initial guess for the controller
% sys: system parameters
%   sys.m: pendulum mass (kg)
%   sys.M: car mass (kg)
%   sys.g: gravity force (m/s^2)
%   sys.l: pendulum lenght (m)
%   sys.dt: timestep (s)
% Output:
% x_open: R^(N*4): matrix with the states computed by the open loop
% solution
    function [x_open, t_open] = computeOpenloopSolution(inverted_pendulum,N, T, t_0, x_0, u, sys)
        x_open(1,:) = x_0;
        t_open(1,:) = t_0;
        for k=1:N
            [t_0, x_open_] = ode45(@(t,x) inverted_pendulum(t, x ,u(k),sys), [t_0(end):T/10:t_0(end)+T], x_0); %[t_0(end):0.05:t_0(end)+T], x_0)
            x_open(k+1,:) = x_open_(end,:);
            t_open(k+1,:) = t_0(end,:);
            x_0 = x_open_(end,:);
   
        end

    end
end

function printHeaderDummy(varargin)
end

function printClosedloopDataDummy(varargin)
end

function plotTrajectoriesDummy(varargin)
end

function printSolution(printHeader, printClosedloopData, plotTrajectories...
    ,mpciter, iprint, t_Elapsed, u, T, t0,x0,V,exitflag,output,x,t,lb,ub)
if (mpciter == 0)
    printHeader();
end
printClosedloopData(mpciter, u, x0, t_Elapsed);
if ( iprint >= 1 )
     plotTrajectories(T, t0, x0, u, x, t,lb,ub,mpciter)
end
end


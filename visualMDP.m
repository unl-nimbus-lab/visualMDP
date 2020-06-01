% This lists the steps to create, solve and visualize a Markov Decision
% Process. This uses Matlab's Reinforcement library and a MDP Toolbox
% developed by Marie-Josee Cros. Link : https://www.mathworks.com/matlabcentral/fileexchange/25786-markov-decision-processes-mdp-toolbox
% Please follow the steps 
% Note : This supports only 

% The probability of getting the intended outcome and going to the correct state, or ending up in a
% different state instead.
% defines as [Forward, Left, Right, Stay]
reliability_of_actions = [0.9, 0.04, 0.04, 0.02];

% Step 1: Create the grid object and assign states
grid_rows = 6 ;
grid_columns = 6 ;

grid = createGridWorld(grid_rows,grid_columns,"Standard");
grid.CurrentState = "[2,1]";
grid.TerminalStates = "[5,5]";
grid.ObstacleStates = ["[3,3]";"[3,4]";"[3,5]";"[4,3]"]; 

% Step 2: Set the transition probabilites
util = gridWorldUtilities ;
util.setTransitionPorbabilities(grid,reliability_of_actions);
util.adjustTransitionsForObstacles(grid);

% Step 3: Now add the rewards 
nS = numel(grid.States);
nA = numel(grid.Actions);
grid.R = -10*ones(nS,nS,nA);
grid.R(:,state2idx(grid,grid.TerminalStates),:) = 10;

% Step 4: Now solve for the optimal policy
T = grid.T ;
R = grid.R ;
discount = 0.8 ;
epsilon = 0.001 ;
max_iter = 1000 ;
V0 = zeros(grid_rows*grid_columns,1); 

[policy, iter, cpu_time] = mdp_value_iteration(T, R, discount, epsilon, max_iter, V0);

% Step 5: Plot the policy
util.plotOptimalPath(grid,policy);
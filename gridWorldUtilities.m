classdef gridWorldUtilities
    % A class that implements utility functions to enhance Matlab's MDP to
    % include 1. Transition probabilities, 
    % 2. Adjust transitions for obstacles
    % 3. Visualize the optimal path
    
    
    %Public Methods
    methods
        
        function uMDP = setTransitionPorbabilities(~,MDP,reliability)
            
            % This function adjusts the transition model accounting for the reliability of each action. 
            
            % MDP: MDP model from Matlab's GridWorld object or GenericMDP object or one with a similar structure.  
            % reliability: 4x1 vector to match the reliability of each
            % actions. 
            
            uMDP = MDP; % updated MDP
            uT = uMDP.T ; % Updated transition matrix
            actions = MDP.Actions ;
            
            for j = 1:numel(actions)
                action = actions(j);
                
                m = uMDP.GridSize(1);
                n = uMDP.GridSize(2);
                uTT = zeros(m*n,m*n); % Updated Transition Tree for  each action. 
 
                switch action
                    case "N"
                        % Move North
                        for c = 1:n
                            r = 1;
                            if uMDP.Border
                                if c == 1
                                    uTT(r+(c-1)*m,r+(c-1)*m) = 1 - reliability(3); %Try to go N, ends up at the same block
                                    uTT(r+(c-1)*m,r+(c+0)*m) = reliability(3); %Try to go N, ends up moving right
                                elseif c == n
                                    uTT(r+(c-1)*m,r+(c-1)*m) = 1 - reliability(2); %Try to go N, ends up at the same block
                                    uTT(r+(c-1)*m,r+(c-2)*m) = reliability(2); %Try to go N, ends up moving left
                                else
                                    uTT(r+(c-1)*m,r+(c-2)*m) = reliability(2); %Try to go N, ends up moving left
                                    uTT(r+(c-1)*m,r+(c+0)*m) = reliability(3); %Try to go N, ends up moving right
                                    uTT(r+(c-1)*m,r+(c-1)*m) = 1 - (reliability(2)+reliability(3)); %Try to go N, ends up at the same block
                                end
                            end

                            for r = 2:m
                                if c == 1
                                    uTT(r+(c-1)*m,(r-1)+(c-1)*m) = 1 - (reliability(2)+reliability(3)+reliability(4)); %Try to go N, goes one block Forward
                                    uTT(r+(c-1)*m,r+(c+0)*m) = reliability(3); %Try to go N, ends up moving right
                                    uTT(r+(c-1)*m,r+(c-1)*m) = reliability(4)+reliability(2); %Try to go N, ends up at the same block
                                elseif c == n
                                    uTT(r+(c-1)*m,(r-1)+(c-1)*m) = 1 - (reliability(2)+reliability(3)+reliability(4)); %Try to go N, goes one block Forward
                                    uTT(r+(c-1)*m,r+(c-1)*m) = reliability(3)+reliability(4); %Try to go N, ends up at the same block
                                    uTT(r+(c-1)*m,r+(c-2)*m) = reliability(2); %Try to go N, ends up moving left
                                else
                                    uTT(r+(c-1)*m,(r-1)+(c-1)*m) = 1 - (reliability(2)+reliability(3)+reliability(4)); %Try to go N, goes one block Forward
                                    uTT(r+(c-1)*m,r+(c-2)*m) = reliability(2); %Try to go N, ends up moving left
                                    uTT(r+(c-1)*m,r+(c+0)*m) = reliability(3); %Try to go N, ends up moving right
                                    uTT(r+(c-1)*m,r+(c-1)*m) = reliability(4); %Try to go N, ends up at the same block
                                end                            
                            end
                        end

                    case "S"
                        % Move South
                        for c = 1:n
                            for r = 1:m-1

                                if c == 1
                                    uTT(r+(c-1)*m,(r+1)+(c-1)*m) = 1 - (reliability(2)+reliability(3)+reliability(4)); %Try to go S, goes one block Forward
                                    uTT(r+(c-1)*m,r+(c+0)*m) = reliability(2); %Try to go S, ends up moving left
                                    uTT(r+(c-1)*m,r+(c-1)*m) = reliability(3)+reliability(4); %Try to go S, ends up at the same block
                                elseif c == n
                                    uTT(r+(c-1)*m,(r+1)+(c-1)*m) = 1 - (reliability(2)+reliability(3)+reliability(4)); %Try to go S, goes one block Forward
                                    uTT(r+(c-1)*m,r+(c-2)*m) = reliability(3); %Try to go S, ends up moving right
                                    uTT(r+(c-1)*m,r+(c-1)*m) = reliability(2) + reliability(4); %Try to go S, ends up at the same block
                                else
                                    uTT(r+(c-1)*m,r+(c+0)*m) = reliability(2); %Try to go S, ends up moving left
                                    uTT(r+(c-1)*m,r+(c-2)*m) = reliability(3); %Try to go S, ends up moving right
                                    uTT(r+(c-1)*m,r+(c-1)*m) = reliability(4); %Try to go S, ends up at the same block
                                    uTT(r+(c-1)*m,(r+1)+(c-1)*m) = 1 - (reliability(2)+reliability(3)+reliability(4)); %Try to go S, goes one block Forward
                                end

                            end
                            r = m;
                            if uMDP.Border

                                if c == 1
                                    uTT(r+(c-1)*m,r+(c+0)*m) = reliability(2); %Try to go S, ends up moving left
                                    uTT(r+(c-1)*m,r+(c-1)*m) = 1 - reliability(2); %Try to go S, ends up at the same block
                                elseif c == n
                                    uTT(r+(c-1)*m,r+(c-2)*m) = reliability(3); %Try to go S, ends up moving right
                                    uTT(r+(c-1)*m,r+(c-1)*m) = 1 - reliability(3); %Try to go S, ends up at the same block
                                else
                                    uTT(r+(c-1)*m,r+(c+0)*m) = reliability(2); %Try to go S, ends up moving left
                                    uTT(r+(c-1)*m,r+(c-2)*m) = reliability(3); %Try to go S, ends up moving right
                                    uTT(r+(c-1)*m,r+(c-1)*m) = 1 - (reliability(2)+reliability(3)); %Try to go S, ends up at the same block

                                end    
                            end
                        end

                    case "E"
                        % Move East
                        for r = 1:m
                            for c = 1:n-1

                                if r == 1
                                    uTT(r+(c-1)*m,r+(c+0)*m) = 1 - (reliability(2)+reliability(3)+reliability(4)); %Try to go E, goes one block Forward
                                    uTT(r+(c-1)*m,(r+1)+(c-1)*m) = reliability(3); %Try to go E, ends up moving right
                                    uTT(r+(c-1)*m,r+(c-1)*m) = reliability(2)+reliability(4) ; %Try to go E, ends up at the same block
                                elseif r == m
                                    uTT(r+(c-1)*m,r+(c+0)*m) = 1 - (reliability(2)+reliability(3)+reliability(4)); %Try to go E, goes one block Forward
                                    uTT(r+(c-1)*m,r+(c-1)*m) = reliability(3)+reliability(4) ; %Try to go E, ends up at the same block
                                    uTT(r+(c-1)*m,(r-1)+(c-1)*m) = reliability(2); %Try to go E, ends up moving left
                                else
                                    uTT(r+(c-1)*m,r+(c+0)*m) = 1 - (reliability(2)+reliability(3)+reliability(4)); %Try to go E, goes one block Forward
                                    uTT(r+(c-1)*m,(r+1)+(c-1)*m) = reliability(3); %Try to go E, ends up moving right
                                    uTT(r+(c-1)*m,(r-1)+(c-1)*m) = reliability(2); %Try to go E, ends up moving left
                                    uTT(r+(c-1)*m,r+(c-1)*m) = reliability(4) ; %Try to go E, ends up at the same block
                                end
                            end

                            c = n;
                            if uMDP.Border
                                if r == 1
                                    uTT(r+(c-1)*m,r+(c-1)*m) = 1 - reliability(3) ; %Try to go E, ends up at the same block
                                    uTT(r+(c-1)*m,(r+1)+(c-1)*m) = reliability(3); %Try to go E, ends up moving right
                                elseif r == m
                                    uTT(r+(c-1)*m,r+(c-1)*m) = 1 - reliability(2) ; %Try to go E, ends up at the same block
                                    uTT(r+(c-1)*m,(r-1)+(c-1)*m) = reliability(2); %Try to go E, ends up moving left
                                else
                                    uTT(r+(c-1)*m,r+(c-1)*m) = 1 - (reliability(2)+reliability(3)) ; %Try to go E, ends up at the same block
                                    uTT(r+(c-1)*m,(r-1)+(c-1)*m) = reliability(2); %Try to go E, ends up moving left
                                    uTT(r+(c-1)*m,(r+1)+(c-1)*m) = reliability(3); %Try to go E, ends up moving right
                                end    
                            end
                        end

                    case "W"
                        % Move West
                        for r = 1:m
                            c = 1;
                            if uMDP.Border
                                if r == 1
                                    uTT(r+(c-1)*m,r+(c-1)*m) = 1 - reliability(2); %Try to go W, ends up at the same block
                                    uTT(r+(c-1)*m,(r+1)+(c-1)*m) = reliability(2); %Try to go W, ends up moving left
                                elseif r == m
                                    uTT(r+(c-1)*m,r+(c-1)*m) = 1 - reliability(3); %Try to go W, ends up at the same block
                                    uTT(r+(c-1)*m,(r-1)+(c-1)*m) = reliability(3); %Try to go W, ends up moving right
                                else
                                    uTT(r+(c-1)*m,r+(c-1)*m) = 1 - (reliability(2)+reliability(3)); %Try to go W, ends up at the same block
                                    uTT(r+(c-1)*m,(r+1)+(c-1)*m) = reliability(2); %Try to go W, ends up moving left
                                    uTT(r+(c-1)*m,(r-1)+(c-1)*m) = reliability(3); %Try to go W, ends up moving right
                                end

                            end

                            for c = 2:n
                                if r == 1
                                    uTT(r+(c-1)*m,r+(c-2)*m) = 1 - (reliability(2)+reliability(3)+reliability(4)); %Try to go W, goes one block Forward
                                    uTT(r+(c-1)*m,(r+1)+(c-1)*m) = reliability(2) ; %Try to go W, ends up moving left
                                    uTT(r+(c-1)*m,r+(c-1)*m) = reliability(3)+reliability(4); %Try to go W, ends up at the same block
                                elseif r == m
                                    uTT(r+(c-1)*m,r+(c-2)*m) = 1 - (reliability(2)+reliability(3)+reliability(4)); %Try to go W, goes one block Forward
                                    uTT(r+(c-1)*m,(r-1)+(c-1)*m) = reliability(3); %Try to go W, ends up moving right
                                    uTT(r+(c-1)*m,r+(c-1)*m) = reliability(2)+reliability(4); %Try to go W, ends up at the same block
                                else
                                    uTT(r+(c-1)*m,r+(c-2)*m) = 1 - (reliability(2)+reliability(3)+reliability(4)); %Try to go W, goes one block Forward
                                    uTT(r+(c-1)*m,(r+1)+(c-1)*m) = reliability(2) ; %Try to go W, ends up moving left
                                    uTT(r+(c-1)*m,(r-1)+(c-1)*m) = reliability(3); %Try to go W, ends up moving right
                                    uTT(r+(c-1)*m,r+(c-1)*m) = reliability(4); %Try to go W, ends up at the same block
                                end                            

                            end
                        end

                end
                
                index = action2idx(uMDP,action);
                uT(:,:,index) = uTT ;

            end
            
            uMDP.T = uT;
            
        end
        
        
        function uMDP = adjustTransitionsForObstacles(~,MDP)
            
            % Change the transition model to avoid into moving into
            % obstacles. 
            
            % MDP: MDP model from Matlab's GridWorld object or GenericMDP object or one with a similar structure.  

            T = MDP.T ; % Extract the Transitions
            obs = MDP.ObstacleStates ; % Extract the obstacles
            actions = MDP.Actions ; 

            numOfObs = numel(obs) ; % number of obstacles
            m = MDP.GridSize(1); % number of rows 
            n = MDP.GridSize(2); % number of columns

            for l = 1:numOfObs

                o = eval(obs(l)); % convert the "[1,1]" to [1,1] . 
                o_r = o(1) ; % obstacle's row
                o_c = o(2) ; % obstacle's column
                stateIdx = o_r + (o_c-1)*m ; % convert the obstacle into the state tarnsition matrix's numerical index

                for a = 1:numel(actions)

                    switch actions(a)
                        case "N"
                            actionIdx = action2idx(MDP,"N");
                            T(stateIdx,:,actionIdx) = 0 ; % remove the obstacle box transitions 

                            %consider left side of the obstacle exists: 0_r,o_c-1]
                            if(o_c-1 ~= 0)
                                r = o_r ; c = o_c-1 ;
                                T(r+(c-1)*m,r+(c-1)*m,actionIdx) = T(r+(c-1)*m,r+(c-1)*m,actionIdx) + T(r+(c-1)*m,r+(c+0)*m,actionIdx); % Add ending up right probability
                                T(r+(c-1)*m,r+(c+0)*m,actionIdx) = 0; % Set ending up right to 0
                            end
                            %consider bottom of the obstacle exists: o_r+1 o_c
                            if(o_r+1 ~= m+1)
                                r = o_r+1 ; c = o_c ;
                                T(r+(c-1)*m,r+(c-1)*m,actionIdx) = T(r+(c-1)*m,r+(c-1)*m,actionIdx) + T(r+(c-1)*m,(r-1)+(c-1)*m,actionIdx); % Add moving forward probability
                                T(r+(c-1)*m,(r-1)+(c-1)*m,actionIdx) = 0 ; % Set moving forward to 0
                            end
                             %consider right side of the obstacle exists: o_r o_c+1
                            if(o_c+1 ~= n+1)
                                r = o_r ; c = o_c+1 ;
                                T(r+(c-1)*m,r+(c-1)*m,actionIdx) = T(r+(c-1)*m,r+(c-1)*m,actionIdx) + T(r+(c-1)*m,r+(c-2)*m,actionIdx) ; % Add ending up left probability
                                T(r+(c-1)*m,r+(c-2)*m,actionIdx) = 0 ; % Set ending up left to 0
                            end

                        case "S"
                            actionIdx = action2idx(MDP,"S");
                            T(stateIdx,:,actionIdx) = 0 ; % remove the obstacle box transitions 

                            %consider left side of the obstacle exists: 0_r,o_c-1]
                            if(o_c-1 ~= 0)
                                r = o_r ; c = o_c-1 ;
                                T(r+(c-1)*m,r+(c-1)*m,actionIdx) = T(r+(c-1)*m,r+(c-1)*m,actionIdx) + T(r+(c-1)*m,r+(c+0)*m,actionIdx); % Add ending up left probability
                                T(r+(c-1)*m,r+(c+0)*m,actionIdx) = 0 ; % Set ending up left to 0 
                            end
                            %consider top of the obstacle exists: o_r-1 o_c
                            if(o_r-1 ~= 0)
                                r = o_r-1 ; c = o_c ;
                                T(r+(c-1)*m,r+(c-1)*m,actionIdx) = T(r+(c-1)*m,r+(c-1)*m,actionIdx) + T(r+(c-1)*m,(r+1)+(c-1)*m,actionIdx); % Add moving forward probability
                                T(r+(c-1)*m,(r+1)+(c-1)*m,actionIdx) = 0 ; % Set moving forward to 0
                            end
                            %consider right side of the obstacle exists: o_r o_c+1
                            if(o_c+1 ~= n+1)
                                r = o_r ; c = o_c+1 ;
                                T(r+(c-1)*m,r+(c-1)*m,actionIdx) = T(r+(c-1)*m,r+(c-1)*m,actionIdx) + T(r+(c-1)*m,r+(c-2)*m,actionIdx); % Add ending up right probability
                                T(r+(c-1)*m,r+(c-2)*m,actionIdx) = 0 ; % Set ending up right to 0
                            end

                        case "E"
                            actionIdx = action2idx(MDP,"E");
                            T(stateIdx,:,actionIdx) = 0 ; % remove the obstacle box transitions 

                            %consider top of the obstacle exists: o_r-1 o_c
                            if(o_r-1 ~= 0)
                                r = o_r-1 ; c = o_c ;
                                T(r+(c-1)*m,r+(c-1)*m,actionIdx) = T(r+(c-1)*m,r+(c-1)*m,actionIdx) + T(r+(c-1)*m,(r-1)+(c-1)*m,actionIdx);  % Add ending up left probability
                                T(r+(c-1)*m,(r-1)+(c-1)*m,actionIdx) = 0 ; % Set ending up left to 0 
                            end

                            %consider bottom of the obstacle exists: o_r+1 o_c
                            if(o_r+1 ~= m+1)
                                r = o_r+1 ; c = o_c ;
                                T(r+(c-1)*m,r+(c-1)*m,actionIdx) = T(r+(c-1)*m,r+(c-1)*m,actionIdx) + T(r+(c-1)*m,(r+1)+(c-1)*m,actionIdx); % Add ending up right probability
                                T(r+(c-1)*m,(r+1)+(c-1)*m,actionIdx) = 0 ; % Set ending up right to 0
                            end

                            %consider left side of the obstacle exists: o_r o_c+1
                            if(o_c-1 ~= 0)
                                r = o_r ; c = o_c-1 ;
                                T(r+(c-1)*m,r+(c-1)*m,actionIdx) = T(r+(c-1)*m,r+(c-1)*m,actionIdx) + T(r+(c-1)*m,r+(c+0)*m,actionIdx); % Add moving forward probability
                                T(r+(c-1)*m,r+(c+0)*m,actionIdx) = 0 ; % Set moving forward to 0
                            end

                        case "W"
                            actionIdx = action2idx(MDP,"W");
                            T(stateIdx,:,actionIdx) = 0 ; % remove the obstacle box transitions 

                            %consider top of the obstacle exists: o_r-1 o_c
                            if(o_r-1 ~= 0)
                                r = o_r-1 ; c = o_c ;
                                T(r+(c-1)*m,r+(c-1)*m,actionIdx) = T(r+(c-1)*m,r+(c-1)*m,actionIdx) + T(r+(c-1)*m,(r-1)+(c-1)*m,actionIdx); % Add ending up right probability
                                T(r+(c-1)*m,(r-1)+(c-1)*m,actionIdx) = 0 ; % Set ending up right to 0                    
                            end

                            %consider bottom of the obstacle exists: o_r+1 o_c
                            if(o_r+1 ~= m+1)
                                r = o_r+1 ; c = o_c ;     
                                T(r+(c-1)*m,r+(c-1)*m,actionIdx) = T(r+(c-1)*m,r+(c-1)*m,actionIdx) + T(r+(c-1)*m,(r+1)+(c-1)*m,actionIdx);  % Add ending up left probability
                                T(r+(c-1)*m,(r+1)+(c-1)*m,actionIdx) = 0 ; % Set ending up left to 0 
                            end

                            %consider right side of the obstacle exists: 0_r,o_c-1]
                            if(o_c-1 ~= 0)
                                r = o_r ; c = o_c+1 ;                     
                                T(r+(c-1)*m,r+(c-1)*m,actionIdx) = T(r+(c-1)*m,r+(c-1)*m,actionIdx) + T(r+(c-1)*m,r+(c-2)*m,actionIdx); % Add moving forward probability
                                T(r+(c-1)*m,r+(c-2)*m,actionIdx) = 0 ; % % Set moving forward to 0
                            end

                    end

                end

            end

            MDP.T = T ;
            uMDP = MDP ;

        end


        function uMDP = plotOptimalPath(~,MDP,path)
            
            % Plots the optimal path in a grid
            
            % MDP: MDP model from Matlab's GridWorld object or GenericMDP object or one with a similar structure.  
            % path : MDPGridHeight*MDPGridWidth x 1 vector of elemets 1-4. 
            % 1 - Go North;  2 - Go South;  3 - Go East;  4 - Go West 
            
            MDP.Viewer = rl.env.GridWorldVisualizer(MDP.GridSize,[]);
            setTerminalStates(MDP.Viewer,MDP.TerminalStates)
            setObstacles(MDP.Viewer,MDP.ObstacleStates)

            % Draw the arrows 
            optimalPath = path ;
            obs = MDP.state2idx(MDP.ObstacleStates); % Extract the obstacles
%             terms = obj.state2idx(obj.TerminalStates); % Extract the obstacles
            optimalPath(obs) = 0 ; % set obstacle states to zero
%             optimalPath(terms) = 0 ; % set obstacle states to zero
            
            numOfPathStates = numel(optimalPath) ; % number of obstacles
            m = MDP.GridSize(1); % number of rows 
            n = MDP.GridSize(2); % number of columns
            
            arrowColor = [0.6 0 1] ; % set the arrow color
            ofst = 0.2 ; %offset to shrink the arrows 
            Ax = MDP.Viewer.Ax ;
            
            for l = 1:numOfPathStates
                action = optimalPath(l) ; 
                state = eval(MDP.idx2state(l)); % get the grid indexes of the optimal path states
                r = state(1); % row 
                c = state(2) ; % column

                switch action
                    case 1
                        % go north arrow
                        X = [c-ofst, c, c+ofst];
                        Y = -[r+ofst, r-ofst, r+ofst];
                        patch(Ax,X,Y,arrowColor);
                    case 2
                        % go south arrow
                        X = [c-ofst, c, c+ofst];
                        Y = -[r-ofst, r+ofst, r-ofst];
                        patch(Ax,X,Y,arrowColor);                       
                    case 3
                        % go east arrow
                        X = [c-ofst, c-ofst, c+ofst];
                        Y = -[r+ofst, r-ofst, r];
                        patch(Ax,X,Y,arrowColor);
                    case 4
                        % go west arrow
                        X = [c-ofst, c+ofst, c+ofst];
                        Y = -[r, r-ofst, r+ofst];
                        patch(Ax,X,Y,arrowColor);
                end
                
            end
        
        end
        
        
    end
    
    
    
    
end
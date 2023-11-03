clear
clc
close

load('my_variables.mat')

% % Define q_init and q_goal
% q_init = [0; 0; 0];
% q_goal = [4; -1; pi/6];
% 
% % Create random robot configuration
% %A = rand(2, 5);
% 
% % Create n random obstacles
% n = 10;
% %B = createEnvironment(n);
% 
% all_points = [q_init(1:2), q_goal(1:2)];
% 
% % % Add robot vertices
% % for i = 1:size(A, 2)
% %     all_points = [all_points, A(:, i)];
% % end
% 
% % Add obstacle vertices
% for i = 1:numel(B)
%     obstacle = B{i};
%     all_points = [all_points, obstacle];
% end
% 
% % Define the bounds of the Euclidean space
% min_x = min(all_points(1,:));
% max_x = max(all_points(1,:));
% min_y = min(all_points(2,:));
% max_y = max(all_points(2,:));
% 
% %% define q_init
% upperBoundx = min_x-1;
% lowerBoundx = min_x-1.25;
% randomx = rand();
% x_for_qinit = lowerBoundx + randomx * (upperBoundx - lowerBoundx);
% 
% upperBoundy = max_y;
% lowerBoundy = min_y;
% randomy = rand();
% y_for_qinit = lowerBoundy + randomy * (upperBoundy - lowerBoundy);
% 
% randomTheta = rand() * 2 * pi;
% q_init = [x_for_qinit; y_for_qinit; randomTheta];
% 
% %% define q_goal
% upperBoundxg = max_x+0.5;
% lowerBoundxg = max_x+0.25;
% randomxg = rand();
% x_for_qgoal = lowerBoundxg + randomxg * (upperBoundxg - lowerBoundxg);
% 
% upperBoundy = max_y;
% lowerBoundy = min_y;
% randomy = rand();
% y_for_qgoal = lowerBoundy + randomy * (upperBoundy - lowerBoundy);
% 
% randomTheta = rand() * 2 * pi;
% q_goal = [x_for_qgoal; y_for_qgoal; randomTheta];
% 
% %% define bounds
% bounds = [x_for_qinit, min_y; 
%           x_for_qgoal, min_y;
%           x_for_qgoal, max_y;
%           x_for_qinit, max_y]';

%% call vCell
path = vCellGraph(q_init, q_goal, B, bounds,A);


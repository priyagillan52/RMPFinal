clear
clc
close

load('my_variables.mat')
% 
% % % Define q_init and q_goal
% % q_init = [0; 0; 0];
% % q_goal = [4; -1; pi/6];
% 
% % Create random robot configuration
% A = rand(2, 6);
% 
% % Create 3 random obstacles
% n = 10;
% B = createEnvironment(n);
% 
% 
% all_points = [];
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
% bounds = [min_x, min_y; 
%           max_x, min_y;
%           max_x, max_y;
%           min_x, max_y]';
% 
% % % Define a threshold distance to check for intersection
% % thres = 0.5;
% % 
% % % Generate obstacles and check for collisions
% % for i = 1:10
% %     while true
% %         newObstacle = rand(2, n) + repmat(5 * (rand(2, 1) - 0.5), 1, n);
% %         collisionDetected = false;
% %         
% %         % Check for collisions with q_init, q_goal, or A
% %         for j = 1:n
% %             if checkCollision(newObstacle(:, j), [q_init(1:2), q_goal(1:2), A], thres)
% %                 collisionDetected = true;
% %                 break;
% %             end
% %         end
% %         
% %         if collisionDetected
% %             continue;
% %         else
% %             B{i} = newObstacle;
% %             break;
% %         end
% %     end
% % end
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
%% call RRT
RRT(A, B, q_init, q_goal, bounds)

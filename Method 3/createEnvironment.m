function [CB, q_init,q_goal] = createEnvironment(n_obstacles)

% n_obstacles = 10;
maxSides = 10;
% axs = gca;
% hold(axs,'on');
% daspect(axs, [1 1 1]);

i = 1;
while true

    centerPoint = [-5 + 10 * rand(); -5 + 10 * rand()]; % Define your desired center point coordinates
    offsetMatrix = 2 * (repmat(centerPoint, 1, maxSides) - repmat([1; 1], 1, maxSides));
    CB{i} = rand(2, maxSides) + offsetMatrix;

%     CB{i} = rand(2,maxSides) + 10*(repmat(rand(2,1),1,maxSides) - repmat(1,2,maxSides));
    %         CB{i} = rand(2,n_obstacles) + repmat(5 * (rand(2,1) - .5), 1 ,n_obstacles);
    % ensure all numbers are positive
    %     CB{i}(CB{i} < 0) = -CB{i}(CB{i} < 0);
    %     idx = convhull(CB{i}(1,:), CB{i}(2,:));
    %     CB{i} = CB{i}(:,idx(1:end-1))*10;


    i_empty = [];
    for j = 1:i
        for k = 1:i
            if j == k
                continue % don't chek obstacle against itself
            end
            if isempty(CB{k})
                continue % obstacle is already bad
            end
            if isempty(CB{j})
                continue % obstacle is already bad
            end

            phsp_a = polyshape(CB{j}(1,:), CB{j}(2,:));
            phsp_b = polyshape(CB{k}(1,:), CB{k}(2,:));

            polyout = intersect(phsp_a,phsp_b);

            if isempty(polyout.Vertices)
                % good polygon
            else
                CB{k} = [];
                i_empty(end+1) = k;
            end
        end
    end

    % delete bad cb obstacles
    if ~isempty(i_empty)
        CB(i_empty) = [];
    end

    i = numel(CB);
    if i >= n_obstacles
        break
    end

    i = i+1;
end

radius = 3; % Define the radius of the circles

% Find valid q_init and q_goal
while true
    % Generate random positions for q_init and q_goal
    q_init = [-10 + 20 * rand(); -10 + 20 * rand()];
    q_goal = [-10 + 20 * rand(); -10 + 20 * rand()];
    
    % Check if q_init and q_goal are not inside any obstacle
    valid_q_init = true;
    valid_q_goal = true;
    for k = 1:numel(CB)
        [in_q_init,~] = inpolygon(q_init(1), q_init(2), CB{k}(1,:), CB{k}(2,:));
        [in_q_goal,~] = inpolygon(q_goal(1), q_goal(2), CB{k}(1,:), CB{k}(2,:));
        if in_q_init
            valid_q_init = false;
        end
        if in_q_goal
            valid_q_goal = false;
        end
    end
    
    % Check if circles centered at q_init and q_goal with radius do not intersect obstacles
    valid_circles = true;
    for k = 1:numel(CB)
        circle_q_init = [q_init(1); q_init(2)]; % Center of the circle
        circle_q_goal = [q_goal(1); q_goal(2)]; % Center of the circle
        distance_q_init = norm(CB{k} - circle_q_init, 'fro'); % Frobenius norm to calculate distance
        distance_q_goal = norm(CB{k} - circle_q_goal, 'fro'); % Frobenius norm to calculate distance
        if distance_q_init < radius || distance_q_goal < radius
            valid_circles = false;
        end
    end
    
    if valid_q_init && valid_q_goal && valid_circles
        break;
    end
end

% %% plot obstacles
% for i = 1:numel(CB)
%     plotCObstacle(CB{i},i);
% end
end
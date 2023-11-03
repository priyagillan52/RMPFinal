function q_path = RRT(A, B, q_init, q_goal, bounds)

%% create environment
% plot robot
idx = convhull(A(1,:), A(2,:));
A = A(:,idx(1:end-1));
% plot obstacle
for i =1:numel(B)
    idx = convhull(B{i}(1,:),B{i}(2,:));
    B{i} = B{i}(:,idx(1:end-1));
    % plot obstacle
    pB(i) = plotObstacle(B{i}, i);
end
% Get current axes
axs = gca;
hold(axs,'on');
%          plot q_init and q_goal
plt_i = plot(axs, q_init(1), q_init(2), 'go','MarkerSize', 8, 'LineWidth',2);
plt_h = plot(axs, q_goal(1), q_goal(2), 'rx','MarkerSize', 8, 'LineWidth',2);

plotRobot(A, q_init);
plotRobot(A, q_goal);

%% RRT algorithm
closestVertexIndex = 1;
tree = {[q_init;closestVertexIndex]}; % tree to be build
q_path = []; % empty path

% Get the x-axis limits
xLimits = xlim(axs);
xMin = min(bounds(1,:));
xMax = max(bounds(1,:));

% Get the y-axis limits
yMin = min(bounds(2,:));
yMax = max(bounds(2,:));

closeEnough = false;
while ~closeEnough
    % Generate random x and y coordinates within the specified range
    randomX = xMin + (xMax - xMin) * rand();
    randomY = yMin + (yMax - yMin) * rand();
    randomT = 2 * pi * rand;

    % Create the random vertex as a 2-element vector
    randomVertex = [randomX; randomY; randomT];
    % plot where the random point is
    [robot_visualization1, h_o_a1]= plotRobot(A, randomVertex);
    pause(0.1)

    % get the random vertices
    theta = randomT;
    R = [cos(theta), -sin(theta);
        sin(theta), cos(theta)];
    randomVerts = R*A;
    for i = 1:(numel(A)/2)
        randomVerts(1,i) = randomVerts(1,i) + randomVertex(1);
        randomVerts(2,i) = randomVerts(2,i) + randomVertex(2);
    end

    % get the closest vertices
    closestVertexIndex = findClosestVertex(tree, [randomVertex(1);randomVertex(2)]);
    closestVertex = tree{closestVertexIndex}(1:3);
    theta = closestVertex(3);
    [robot_visualization2, h_o_a2]= plotRobot(A, closestVertex);
    R = [cos(theta), -sin(theta);
        sin(theta), cos(theta)];

    closestVerts = R*A;
    for i = 1:(numel(A)/2)
        closestVerts(1,i) = closestVerts(1,i) + closestVertex(1);
        closestVerts(2,i) = closestVerts(2,i) + closestVertex(2);
    end

    newVertSeg = zeros(2, numel(A)/2 * 2)+2;  % Preallocate the array for line segment vertices
    % draw segments from random to closest to see if they collide
    for i = 1:(numel(A)/2)
        % Update newVertSeg with newVerts
        newVertSeg(1, 2*i - 1) = closestVerts(1,i);
        newVertSeg(2, 2*i - 1) = closestVerts(2,i);

        % Update newVertSeg with A vertices
        newVertSeg(1, 2*i) = randomVerts(1,i);
        newVertSeg(2, 2*i) = randomVerts(2,i);
    end

    newVertSeg(:, end+1) = randomVertex(1:2);
    newVertSeg(:, end+1) = closestVertex(1:2);

    path_line(:,1) = randomVertex(1:2);
    path_line(:,2) = closestVertex(1:2);

    % Plot the line segments
    temp_lines = plot(axs, newVertSeg(1,:), newVertSeg(2,:), 'c', 'LineWidth', 1);
    pause(0.1);

    % check if segments collide
    goodseg = true;
    for j = 2:(numel(newVertSeg)/2)/2
        for i = 1:numel(B)
            [tf,~,~] = intersectSegmentPolygon([newVertSeg(1,2*j-1);newVertSeg(2,2*j-1)],[newVertSeg(1,2*j);newVertSeg(2,2*j)],B{i});
            if tf
                goodseg = false;
                break;
            end
        end
        if ~goodseg
            delete(temp_lines)
            delete(h_o_a1)
            delete(robot_visualization1)
            delete(h_o_a2)
            delete(robot_visualization2)
            break
        end
    end

    if goodseg
        dist = norm([q_goal(1), q_goal(2)] - [randomVertex(1), randomVertex(2)]);
        % Check if the distance is below the threshold
        if dist < 0.75
            closeEnough = true;
        end
        tree{end+1} = [randomVertex; closestVertexIndex];
        % Plot the random vertex
        plt_v = plot(axs, randomVertex(1), randomVertex(2), 'k.','MarkerSize', 8, 'LineWidth',2);
        pause(0.1);
        plot(axs, path_line(1,:), path_line(2,:), 'k','LineWidth', 1);
        pause(0.1);
        delete(temp_lines)
        delete(h_o_a1)
        delete(robot_visualization1)
        delete(h_o_a2)
        delete(robot_visualization2)
    end
end
if closeEnough
    tree{end+1} = [q_goal;closestVertexIndex];
    % Backtrack to generate the final path
    finalPath = [];
    currentVertexIndex = numel(tree); % Start from the last vertex added to the tree
    
    counter = 1;
    while currentVertexIndex ~= 1
        if counter >= 500
            path = [];
            return;
        end
        currentVertex = tree{currentVertexIndex}; % Get the current vertex
        finalPath = [finalPath, currentVertex(1:2)]; % Append the x and y coordinates
        q_path = [q_path, currentVertex(1:3)];
        currentVertexIndex = currentVertex(4); % Move to the parent index
    end
    
    % Append the start vertex
    finalPath = [finalPath, q_init(1:2)];
    q_path = [q_path, q_init(1:3)];
    
    % Reverse the path to get the correct order
    finalPath = fliplr(finalPath);

    q_path = fliplr(q_path);
    
    % Plot the final path
    plot(axs, finalPath(1,:), finalPath(2,:), 'm', 'LineWidth', 2);
end

end
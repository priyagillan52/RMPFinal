function q_path = PRM(A, B, q_init, q_goal, bounds)
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
% plot q_init and q_goal
plot(axs, q_init(1), q_init(2), 'go','MarkerSize', 8, 'LineWidth',2);
plot(axs, q_goal(1), q_goal(2), 'rx','MarkerSize', 8, 'LineWidth',2);

center = q_init(1:2);
radius = 0.5;

% Create a square around the center with sides equal to twice the radius
rectangle('Position', [center(1)-radius, center(2)-radius, 2*radius, 2*radius], ...
    'Curvature', [1, 1], 'EdgeColor', 'r'); % Curvature [1, 1] makes it a circle
axis equal; % Ensure equal scaling for x and y axes

center = q_goal(1:2);
% Create a square around the center with sides equal to twice the radius
rectangle('Position', [center(1)-radius, center(2)-radius, 2*radius, 2*radius], ...
    'Curvature', [1, 1], 'EdgeColor', 'r'); % Curvature [1, 1] makes it a circle
axis equal; % Ensure equal scaling for x and y axes
pause(0.1)

%% Sampling
tree = {};
validNodes = {q_init(1:2)};
q_path = []; % empty path

% Get the x-axis limits
xMin = min(bounds(1,:));
xMax = max(bounds(1,:));

% Get the y-axis limits
yMin = min(bounds(2,:));
yMax = max(bounds(2,:));

numSamples = 60;
for i = 1:numSamples
    randomVerts = [];
    randomPointSeg = [];

    % Generate random x and y coordinates within the specified range
    randomX = xMin + (xMax - xMin) * rand();
    randomY = yMin + (yMax - yMin) * rand();
    % Create the random vertex as a 2-element vector
    randomVertex = [randomX; randomY];

    % plot where the random point is
    center = randomVertex(1:2);
    % Create a square around the center with sides equal to twice the radius
    circ = rectangle('Position', [center(1)-radius, center(2)-radius, 2*radius, 2*radius], ...
        'Curvature', [1, 1], 'EdgeColor', 'r'); % Curvature [1, 1] makes it a circle
    axis equal; % Ensure equal scaling for x and y axes
    point = plot(randomVertex(1), randomVertex(2), 'kx');
    pause(0.1);

    validConfig = true;
    % check if sample point is in collision
    for k = 1:size(B,2)
        [in,~] = inpolygon(randomX, randomY,B{k}(1,:)',B{k}(2,:)');
        if in
            validConfig = false;
        end
    end
    for k = 1:numel(B)
        tf = circlePolygonIntersect([randomVertex(1); randomVertex(2)], radius, B{k});
        if tf
            validConfig = false;
        end
    end

    if ~validConfig
        delete(circ)
        delete(point)
    else
        validNodes = [validNodes, randomVertex];
    end
end
validNodes = [validNodes, q_goal(1:2)];

%% Connect nodes to make graph
% Create a weighted adjacency matrix
num_nodes = numel(validNodes);
adjacency_matrix = zeros(num_nodes, num_nodes);

for i = 1:numel(validNodes)
    neighborIndices = []; % Initialize neighbor indices for this node
    
    for j = i+1:numel(validNodes)
        validConfig = true;
        if i == j
            continue;  % Skip the same node
        end
        x1 = validNodes{i}(1);
        y1 = validNodes{i}(2);

        x2 = validNodes{j}(1);
        y2 = validNodes{j}(2);
        segments = [[x2+radius; y2], [x1+radius; y1], [x2; y2+radius], [x1; y1+radius], [x2-radius; y2], [x1-radius; y1], [x2; y2-radius],[x1; y1-radius],[x2; y2], [x1; y1]];

        % Calculate the distance between nodes i and j
        distance = norm(validNodes{i}(1:2) - validNodes{j}(1:2));

        for k = 1:size(segments,2)/2

            for p = 1:numel(B)
                [tf, ~, ~] = intersectSegmentPolygon(segments(:,2*k-1), segments(:,2*k), B{p}); 
                if tf
                    validConfig = false;
                    break
                end
            end
        end

        if validConfig
%             neighborIndices = [neighborIndices, j]; % Store the neighbor index
            adjacency_matrix(i, j) = 1;
            adjacency_matrix(j, i) = 1;

            % Plot the connection between nodes
            plot([validNodes{i}(1), validNodes{j}(1)], [validNodes{i}(2), validNodes{j}(2)], 'k', 'LineWidth', 1);
            pause(0.1);
        end
    end
    % Store this node's information in the tree cell array
%     tree{i} = {validNodes{i}, i, neighborIndices};
end

% plot q_init and q_goal so you can see
plot(axs, q_init(1), q_init(2), 'go','MarkerSize', 8, 'LineWidth',2);
plot(axs, q_goal(1), q_goal(2), 'rx','MarkerSize', 8, 'LineWidth',2);

%% Path Planning using Dijkstra's algorithm
start_index = 1; % Index of the start node
goal_index = size(validNodes,2); % Index of the goal node (assuming it's the last node)

shortest_path_indices = bfs(adjacency_matrix,start_index, goal_index);

if isempty(shortest_path_indices)
    fprintf("No possible path")
    path = [];
    return
end


% Extract the actual path from the indices
path = {};

for i = 1:numel(shortest_path_indices)
    path{end+1} = validNodes{shortest_path_indices(i)}(1:2);
end

% Convert path to a 3xN matrix
q_path = cell2mat(path);

path = q_path;

% Plot the final path
plot(axs, path(1, :), path(2, :), 'c', 'LineWidth', 3);


end

% Function to check if a circle intersects with a polygon
function intersects = circlePolygonIntersect(circleCenter, radius, polygon)
intersects = false;
for i = 1:size(polygon, 2)
    segment = [polygon(:,i), polygon(:,mod(i, size(polygon, 2)) + 1)];
    distance = pointToSegmentDistance(circleCenter, segment);
    if distance < radius
        intersects = true;
        return;
    end
end
end

function distance = pointToSegmentDistance(point, segment)
% Calculate the vector from the segment's start to end
segmentVector = segment(:, 2) - segment(:, 1);

% Calculate the vector from the segment's start to the point
pointVector = point - segment(:, 1);

% Calculate the projection of the point vector onto the segment vector
projection = dot(pointVector, segmentVector) / dot(segmentVector, segmentVector);

if projection <= 0
    % The closest point on the segment is the start point
    closestPoint = segment(:, 1);
elseif projection >= 1
    % The closest point on the segment is the end point
    closestPoint = segment(:, 2);
else
    % The closest point on the segment is the projection of the point
    closestPoint = segment(:, 1) + projection * segmentVector;
end

% Calculate the distance between the point and the closest point
distance = norm(point - closestPoint);
end
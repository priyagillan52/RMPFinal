function shortestPath = DFS(graph, startNode, endNode)
    % Perform depth-first search to find the shortest path from startNode to endNode
    % graph: Weighted adjacency matrix
    % startNode: Index of the starting node
    % endNode: Index of the ending node
    
    visited = false(size(graph, 1), 1); % Initialize the visited array
    stack = startNode; % Initialize the stack with the starting node
    
    paths = cell(size(graph, 1), 1); % Initialize paths for each node
    paths{startNode} = startNode;
    
    while ~isempty(stack)
        currentNode = stack(end); % Get the top node from the stack
        stack(end) = []; % Pop the top node
        
        if currentNode == endNode
            shortestPath = paths{currentNode}; % Return the shortest path
            return;
        end
        
        neighbors = find(graph(currentNode, :) ~= inf); % Find neighbors of the current node
        
        for i = 1:length(neighbors)
            neighbor = neighbors(i);
            if ~visited(neighbor)
                visited(neighbor) = true; % Mark the neighbor as visited
                stack(end+1) = neighbor; % Push the neighbor onto the stack
                
                paths{neighbor} = [paths{currentNode}, neighbor]; % Update the path for the neighbor
            end
        end
    end
    
    shortestPath = []; % If no path is found, return an empty array
end

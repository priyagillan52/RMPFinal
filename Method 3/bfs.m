function path = bfs(G, q_init, q_goal)
    n = size(G, 1);

    if q_init == q_goal
        path = q_init;
        return
    end

    closed_set = false(1, n);
    queue = q_init;
    prev_node = zeros(1, n);

    while ~isempty(queue)
        current = queue(1);
        queue(1) = []; % Dequeue

        if current == q_goal
            path = reconstructPath(prev_node, q_goal);
            return
        end

        closed_set(current) = true;

        neighbors = find(G(current, :));

        for neighbor = neighbors
            if ~closed_set(neighbor) && isempty(find(queue == neighbor, 1))
                prev_node(neighbor) = current;
                queue(end+1) = neighbor;
            end
        end
    end

    path = []; % No path found
    disp('Could not reach goal :(')
end

function path = reconstructPath(prev_node, n_goal)
    path = n_goal;
    while prev_node(path(1)) ~= 0
        path = [prev_node(path(1)), path];
    end
end

function isCollision = isVertexInPoly(vertex, obstacles)
    % Check if the vertex is in collision with any of the obstacles
    isCollision = false;
    for i = 1:numel(obstacles)
        if inpolygon(vertex(1), vertex(2), obstacles{i}(1,:), obstacles{i}(2,:))
            isCollision = true;
            break;
        end
    end
end
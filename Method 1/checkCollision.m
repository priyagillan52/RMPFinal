function collision = checkCollision(point, referencePoints, thres)
    collision = false;
    for k = 1:size(referencePoints, 2)
        % Calculate Euclidean distance between points
        dist = norm(point - referencePoints(:, k));
        
        % Check if the distance is below the threshold
        if dist < thres
            collision = true;
            return;
        end
    end
end
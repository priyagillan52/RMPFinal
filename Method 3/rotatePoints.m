function rotatedPoints = rotatePoints(points, angle)
    % points: 2xN matrix of points (each column represents a point)
    % angle: rotation angle in radians
    
    rotationMatrix = [cos(angle), -sin(angle); sin(angle), cos(angle)];
    rotatedPoints = rotationMatrix * points;
end

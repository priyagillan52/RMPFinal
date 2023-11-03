function closestVertexIndex = findClosestVertex(tree, point)
closestVertexIndex = 0;
closestDistance = Inf;

for i = 1:length(tree)
    vertex = tree{i}(1:2);
    distance = norm(vertex - point');

    if distance < closestDistance
        closestDistance = distance;
        closestVertexIndex = i;
    end
end
end
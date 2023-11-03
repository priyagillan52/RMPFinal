function [ob,txt] = plotObstacle(B,i)
    axs = gca;
    hold(axs,'on');
    daspect(axs,[1,1,1]);

    [n,m] = size(B);

    if i <= 0
        error('i must be positive')
    end
    
    if n ~= 2 
        error('vertices should be 2 by n matrix')
    end

    idx = convhull(B(1,:), B(2,:));
    idx(end) = [];

    if numel(idx) ~= size(B,2)
        error('polygon not convex')
    end
    if max(diff(idx)) ~= 1
        error('polygon vertices are not ccw')
    end

    n = size(B,2);
    faces = 1:n;
    vertices = B.';
    ob = patch('Vertices', vertices, 'Faces', faces, 'FaceColor', 'g', 'EdgeColor', 'g', 'FaceAlpha', 0.5);
    
    ps = polyshape(B(1,:),B(2,:));
    [x,y] = centroid(ps);
    txt = text(x,y, sprintf('B_{%d}',i));
end


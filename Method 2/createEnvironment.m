function CB = createEnvironment(n_obstacles)

% n_obstacles = 10;
maxSides = 12;
% axs = gca;
% hold(axs,'on');
% daspect(axs, [1 1 1]);

i = 1;
while true
    CB{i} = rand(2,maxSides) + 10*(repmat(rand(2,1),1,maxSides) - repmat(1,2,maxSides));
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

% %% plot obstacles
% for i = 1:numel(CB)
%     plotCObstacle(CB{i},i);
% end
end
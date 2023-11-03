function path = vCellGraph(q_init, q_goal, CB, bounds, A)


%% making the figure
fig = figure;
axs = axes('Parent', fig);
hold(axs, 'on')
daspect(axs, [1 1 1])

%% initialized output values
verts = [];
%% keep track of objects and object index
obsts = [];

%% fill in values from CB
for i =1:numel(CB)
    idx = convhull(CB{i}(1,:),CB{i}(2,:));
    CB{i} = CB{i}(:,idx(1:end-1));
    % plot obastacles
    pCB(i) = plotObstacle(CB{i}, i);
    verts = [verts, CB{i}];
    obsts = [obsts, repmat(i,1,size(CB{i},2))];

end

%% plot q_init and q_goal
plt_i = plot(axs, q_init(1), q_init(2), 'go','MarkerSize', 8, 'LineWidth',2);
plt_h = plot(axs, q_goal(1), q_goal(2), 'rx','MarkerSize', 8, 'LineWidth',2);
plotRobot(A,q_init);
plotRobot(A,q_goal);

%% vertical scale
ymin = min(bounds(2,:));
ymax = max(bounds(2,:));

%% vertical segments
plt_SegU = {};
plt_SegL = {};
pnt_SegU = {};
pnt_SegL = {};

for k = 1:numel(CB)
    % kth obstacle
    for i = 1:size(CB{k},2)
        segU{k}{i} = [...
            CB{k}(1,i) CB{k}(1,i);
            ymax CB{k}(2,i) ];
        segL{k}{i} = [...
            CB{k}(1,i) CB{k}(1,i);
            CB{k}(2,i) ymin ];
        plt_SegU{k}(i) = plot(axs, segU{k}{i}(1,:), segU{k}{i}(2,:),'r');
        plt_SegL{k}(i) = plot(axs, segL{k}{i}(1,:), segL{k}{i}(2,:),'b');
        pnt_SegU{k}(i) = plot(axs, segU{k}{i}(1,2), segU{k}{i}(2,2),'xr');
        pnt_SegL{k}(i) = plot(axs, segL{k}{i}(1,2), segL{k}{i}(2,2),'xb');
    end
end

%% remove edge that intersect with self
pltEdge = plot(axs,0,0,'b','LineWidth',2);

for k = 1:numel(CB)
    for i = 1:numel(segU{k})
        X1_U = segU{k}{i};
        X1_L = segL{k}{i};

        nCB = size(CB{k},2);

        bad_U = false;
        bad_L = false;

        for j = 1:nCB

            j0 = j;
            if j == nCB
                j1 = 1;
            else
                j1 = j+1;
            end
            X2 = [CB{k}(:,j0), CB{k}(:,j1)];

            set(pltEdge, 'XData', X2(1,:),'YData',X2(2,:));
            set(plt_SegU{k}(i), 'LineWidth',3);
            set(plt_SegL{k}(i), 'LineWidth',3);

            %             drawnow
            %pause
            %%pause(0.1)

            [tf,~,~,~] = intersectSegmentSegment(X1_U,X2);

            if tf
                set(plt_SegU{k}(i),'LineStyle',':');
                bad_U = true;
                segU{k}{i} = [];
            end

            [tf,~,~,~] = intersectSegmentSegment(X1_L,X2);

            if tf
                set(plt_SegL{k}(i),'LineStyle',':');
                bad_L = true;
                segL{k}{i} = [];
            end
            %             drawnow
            %pause
            %%pause(0.1)
            if bad_U && bad_L
                break
            end
        end
        set(plt_SegU{k}(i),'LineWidth',0.5);
        set(plt_SegL{k}(i),'LineWidth',0.5);
    end
end

%% hide dotted lines

for k = 1:numel(segU)
    for i = 1:numel(segU{k})
        if isempty(segU{k}{i})
            set(plt_SegU{k}(i), 'Visible','off');
        end
        if isempty(segL{k}{i})
            set(plt_SegL{k}(i), 'Visible','off');
        end
    end
end

%% cut segments that intersect with others
plt_pnt = plot(axs,0,0,'*b', 'Visible', 'off');

for k = 1:numel(segU)
    for i = 1:numel(segU{k})
        X1_U = segU{k}{i};
        X1_L = segL{k}{i};

        for kk = 1:numel(CB)
            if kk == k
                continue
            end

            nCB = size(CB{kk},2);

            for j = 1:nCB
                j0 = j;

                if j == nCB
                    j1 = 1;
                else
                    j1 = j +1;
                end
                X2 = [CB{kk}(:,j0), CB{kk}(:,j1) ];

                set(pltEdge, 'Xdata', X2(1,:), 'YData',X2(2,:));

                set(plt_SegU{k}(i),'LineWidth',3);
                set(plt_SegL{k}(i),'LineWidth',3);

                %                 drawnow
                % check upper segment
                if ~isempty(X1_U) && size(X1_U,2) == 2
                    [tf,~,~,pnt] = intersectSegmentSegment(X1_U,X2);
                    if tf
                        set(plt_pnt, 'XData', pnt(1), 'YData', pnt(2), 'Visible', 'on');
                        if pnt(2) < segU{k}{i}(2,1)
                            segU{k}{i}(:,1) = pnt;
                        end
                        set(plt_SegU{k}(i), 'XData', segU{k}{i}(1,:), 'YData',segU{k}{i}(2,:));
                    end
                end
                %                 drawnow
                set(plt_pnt, 'Visible','off');

                % check lower segment
                if ~isempty(X1_L) && size(X1_L,2) == 2
                    [tf,~,~,pnt] = intersectSegmentSegment(X1_L,X2);
                    if tf
                        set(plt_pnt, 'XData', pnt(1), 'YData', pnt(2), 'Visible', 'on');
                        if pnt(2) > segL{k}{i}(2,2)
                            segL{k}{i}(:,2) = pnt;
                            segL{k}{i}(1,2);
                        end
                        set(plt_SegL{k}(i), 'XData', segL{k}{i}(1,:), 'YData',segL{k}{i}(2,:));

                    end
                end
                %                 drawnow
                %%pause
                set(plt_pnt, 'Visible','off');
                set(plt_SegU{k}(i), 'LineWidth',0.5);
                set(plt_SegL{k}(i), 'LineWidth',0.5);
            end
        end
    end
end

%% find mid points
verts = [];
for k = 1:numel(CB)
    for i = 1:numel(segL{k})
        if ~isempty(segU{k}{i})
            verts(:,end+1) = [mean(segU{k}{i}(1,:)); mean(segU{k}{i}(2,:))];
        end
        if ~isempty(segL{k}{i})
            verts(:,end+1) = [mean(segL{k}{i}(1,:)); mean(segL{k}{i}(2,:))];
        end
    end
end
verts = [q_init(1:2,1), verts, q_goal(1:2,1)];
verts = (sortrows([verts(1,:)',verts(2,:)'], [1,2]))';

labels = cellstr(num2str((1:size(verts,2))'));
plot(axs,verts(1,:),verts(2,:),'xbl');
pause(0.1)
text(axs,verts(1,:),verts(2,:),labels);


%% find random robots at the mid points
validNodes = zeros(3, size(verts,2)); % Initialize a matrix to store valid nodes

validNodes(:,1) = q_init;
for i = 2:size(verts,2)-1

    theta = rand() * 2 * pi; % Generate a random theta value
    step = pi/4;

    validConfig = true;
    for h = 0:8
        randomTheta = theta + h*step;
        [a,b] = plotRobot(A, [verts(1,i); verts(2,i); randomTheta]); % Use the midpoint coordinates

        R = [cos(theta), -sin(theta);
            sin(theta), cos(theta)];
        randomVerts = R*A;
        for j = 1:(numel(A)/2)
            randomVerts(1,j) = randomVerts(1,j) + verts(1,i);
            randomVerts(2,j) = randomVerts(2,j) + verts(2,i);
        end
        randomVerts(1:2, end+1) = verts(1:2,i);

        randomPointSeg = [];
        for j = 2:(size(randomVerts,2))
            randomPointSeg = [randomPointSeg, randomVerts((1:2),j-1), randomVerts((1:2),j)];
        end
        randomPointSeg = [randomPointSeg, randomVerts((1:2),size(randomVerts,2)), randomVerts((1:2),1)];

        % check if segments are in collision with obstacles
        for j = 1:(size(randomPointSeg,2)/2)
            for k = 1:numel(CB)
                [tf,~,~] = intersectSegmentPolygon(randomPointSeg(1:2, 2*j - 1),randomPointSeg(1:2, 2*j),CB{k});

                if tf
                    validConfig = false;
                    break;
                end
            end
            if ~validConfig
                validNodes(:,i) = [inf;inf;inf];
                delete(a)
                delete(b)
                break

            end
        end
        if validConfig
            validNodes(:,i)=[verts(1,i); verts(2,i); randomTheta]; % Store valid configuration
            break;
        end
    end
end
validNodes(:,size(verts,2)) = q_goal;
%% connect what can be connected in a tree
wAdj = inf(size(validNodes, 2)); % Initialize the weighted adjacency matrix

for i = 1:size(validNodes,2)
    if validNodes(1,i) == inf
        wAdj(i,j) = inf;
        wAdj(j,i) = inf;
        continue
    end

    % Get first configuration
    R1 = [cos(validNodes(3,i)), -sin(validNodes(3,i));
        sin(validNodes(3,i)), cos(validNodes(3,i))];
    config1 = R1*A;
    config1(1:2, end+1) = validNodes(1:2,i);

    for j = i+1:size(validNodes, 2)
        if validNodes(1,j) == inf
            wAdj(i,j) = inf;
            wAdj(j,i) = inf;
            continue
        end

        % Get second configuration
        R2 = [cos(validNodes(3,j)), -sin(validNodes(3,j));
            sin(validNodes(3,j)), cos(validNodes(3,j))];
        config2 = R2*A;
        config2(1:2, end+1) = validNodes(1:2,j);
        goodseg = true;

        for d = 1:size(A, 2) + 1
            distance = norm(validNodes(1:2,i) - validNodes(1:2,j)); % Calculate distance

            for k = 1:size(CB, 2)
                [tf, ~, ~] = intersectSegmentPolygon(config1(1:2,d), config2(1:2,d), CB{k});
                if tf
                    goodseg = false;
                    break;
                end
            end

            if goodseg
                wAdj(i,j) = distance;
                wAdj(j,i) = distance;
            else
                wAdj(i,j) = inf;
                wAdj(j,i) = inf;
                break
            end
        end
    end
end

startNode = 1; % Choose the starting node index
endNode = size(validNodes, 2);   % Choose the ending node index

% Find the shortest path
shortest_path = DFS(wAdj, startNode, endNode);

if isempty(shortest_path)
    path = [];
    return;
end

path = zeros(3,numel(shortest_path));
for i=1:numel(shortest_path)
    path(1:3, i) = validNodes(1:3,shortest_path(i));
end
plot(path(1,:), path(2,:), 'c', 'LineWidth',2);
pause(0.1)
end

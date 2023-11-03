function [robot_visualization, h_o_a] = plotRobot(A, q)
    [m,n] = size(A);
    [qm, qn] = size(q);

    if m ~=2
        error("A isn't a 2 by x matrix")
    elseif n < 3
        error("need more vertices for the robot")
    elseif qm ~= 3
        error("q doesn't have the right number of rows")
    elseif qn ~= 1
        error("q has too many columns")
    end

    x = A(1,:);
    y = A(2,:);

    qSE3 = [q(1,1); q(2,1); 0; 0; 0; q(3,1)];

    k = qSE3(4:6, 1);
    
    r = wedge(k);

    R = expm(r);
    
    H = eye(4);
    H(1:3,1:3) = R;
    H(1:3,4) = qSE3(1:3,1);

    axs = gca;
    hold(axs,'on');
    daspect(axs,[1,1,1]);
  
    robot_color = 'red';
    h_o_a = triad('Parent', axs);
    robot_visualization = patch(x,y,robot_color,'Parent', h_o_a);
    set(h_o_a, 'matrix', H);

end

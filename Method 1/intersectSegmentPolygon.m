function [tf,intPt,vPG,vSeg] = intersectSegmentPolygon(v1,v2,PG)
% INTERSECTSEGMENTPOLYGON finds the intersection(s) between a 2D segment 
% defined by two end-points, and a polygon defined as an ordered set of
% non-repeating vertices.
%   [tf,intPt,onVertex] = INTERSECTSEGMENTPOLYGON(v1,v2,PG) defines the
%   segment as having end-points v1 and v2, and the polygon as an ordered
%   set of vertices contained in PG
%
%   Inputs:
%       v1 - 2x1 array containing the x/y coordinates of a segment 
%            end-point of
%       v2 - 2x1 array containing the x/y coordinates of a segment 
%            end-point of
%       PG - 2xN array containing an ordered set of vertices defining a 
%            polygon (where N > 2)
%
%   Outputs:
%       tf - binary indicating whether the segment intersects the polygon
%            at least once and/or if the segment is entirely contained 
%            within the polygon
%       intPt - 2xN array containing all points of intersect between
%            the obstacle and polygon including intersections with polygon
%            vertices.
%
%   M. Kutzer, 24Jul2020, USNA

%% Check inputs
narginchk(3,3);

if numel(v1) ~= 2
    error('First end-point must be two-dimensional.');
end
if numel(v2) ~= 2
    error('Second end-point must be two-dimensional.');
end
if size(PG,1) ~= 2
    error('Polygon vertices must be two-dimensional.');
end
if size(PG,2) < 3
    error('Polygon must have at least three vertices.');
end

%% Define "zero"
ZERO = 1e-6;    % TODO - use zeroFPError.m

%% Set defaults
tf = false;
intPt = [];
vPG = [];
vSeg = [];

%% Combine end-points
v1 = reshape(v1,[],1);
v2 = reshape(v2,[],1);
p1 = [v1,v2];

%% Check for intersections
N = size(PG,2);
for i = 1:N
    % Define wrap-around condition
    if i < N
        j = i+1;
    else
        j = 1;
    end
    % Define polygon edge
    p2 = [PG(:,i),PG(:,j)];
    % Find intersection(s)
    [intEE,intEV,intVV,intPt_i] = intersectSegmentSegment(p1,p2);
    % Check intersect condition
    switch nnz([intEE,intEV,intVV])
        case 1
            % Standard intersect condition
            if intEE
                % Edge/Edge intersection
                tf    = true;
                intPt = [intPt, intPt_i];
                vPG   = [ vPG, false];
                vSeg  = [vSeg, false];
            end
            if intEV
                % Edge/Vertex intersection
                tf    = true;
                intPt = [intPt, intPt_i];
                if any( sum((p1 - intPt_i).^2, 1) < ZERO )
                    vPG   = [ vPG, false];
                    vSeg  = [vSeg, true];
                else
                    vPG   = [ vPG, true];
                    vSeg  = [vSeg, false];
                end
            end
            if intVV
                % Vertex/Vertex intersection
                tf    = true;
                intPt = [intPt, intPt_i];
                vPG   = [ vPG, true];
                vSeg  = [vSeg, true];
            end
        case 2
            warning('Segment is co-linear with and overlapping edge %d of the polygon.',i);
            if intEE && intEV
                % Segments overlap but do not share vertices
                % -> NOT SURE WHAT TO DO WITH THIS CONDITION!
                tf    = false;
                intPt = [intPt, intPt_i];
                vPG   = [ vPG, false(1,size(intPt_i,2))];
                vSeg  = [vSeg, false(1,size(intPt_i,2))];
            elseif intEE && intVV
                % Segments share both vertices
                % -> NOT SURE WHAT TO DO WITH THIS CONDITION!
                tf    = false;
                intPt = [intPt, intPt_i];
                vPG   = [ vPG, true(1,size(intPt_i,2))];
                vSeg  = [vSeg, true(1,size(intPt_i,2))];
            else
                error('~intEE && intEV && intVV is not possible!');
            end
        case 3
            warning('Segment is co-linear with and overlapping edge %d of the polygon.',i);
            % Segments share one vertex, and another vertex lies on the
            % edge
            
    end
end
%% Check if either of the points are inside of the polygon
[in,on] = inpolygon(v1(1),v1(2),PG(1,:),PG(2,:));
if in && ~on
    tf = true;
    return;
end

[in,on] = inpolygon(v2(1),v2(2),PG(1,:),PG(2,:));
if in && ~on
    tf = true;
    return;
end

%% Check each edge individually
s0 = 0;
sf = 1;
S = [s0,sf;1,1];
invS = S^(-1);

% Fit line to vertex pair
AB = [v1,v2]*invS;
A = AB(:,1);
B = AB(:,2);

% Fit line to each edge of the polygon
intPt = [];    % Initialize intersection array
onVertex = [];

ZERO = 1e-6;
n = size(PG,2);
for i = 1:n
    if i < n
        idx1 = i;
        idx2 = i+1;
    else
        idx1 = i;
        idx2 = 1;
    end
    CD = [PG(:,idx1),PG(:,idx2)]*invS;
    C = CD(:,1);
    D = CD(:,2);
    
    % Find possible intersection
    ss = [A, -C]^(-1) * (D-B);
    
    
    % Check if intersection occurs within the bounds of each segment
    if ss(1) >= 0 && ss(1) <= 1
        if ss(2) >= 0 && ss(2) <= 1
            
            intPt(:,end+1) = AB*[ss(1); 1];  % Calculate intersection point
            if norm(intPt(:,end)-v1) < ZERO || norm(intPt(:,end)-v2) < ZERO
                % Start and/or end-points of the segments are shared
                onVertex(:,end+1) = true;
                if debugON
                    fprintf('One vertex is the same as the polygon vertex... IGNORING\n');
                end
            else
                % Check if intersection occurs 
                if abs(ss(2)) < ZERO || abs(ss(2) - 1) < ZERO
                    % -> On polygon vertex
                    onVertex(:,end+1) = true;
                    
                    % Check for and remove possible vertex redundancy
                    % -> Ignore current value when checking
                    v_tmp = intPt;
                    v_tmp(:,end) = inf;
                    % -> Get current value
                    v_now = intPt(:,end);
                    % -> Calculate Euclidean norm
                    deltaV = v_tmp - repmat(v_now,1,size(v_tmp,2));
                    norm_dV = sqrt( sum( deltaV.^2, 1) );
                    
                    % If the vertex is redundant, remove it
                    if any( norm_dV < ZERO )
                        intPt(:,end) = [];
                        onVertex(:,end) = [];
                    end
                else
                    % -> On polygon edge
                    onVertex(:,end+1) = false;
                end
                
                tf = true;          % Return true
                %return
            end
            
        end
    end
end


%% Check for redundant intersections
% % Define Euclidean norm between adjacent vertices
% d_vInt = diff(intPt,1,2);
% norm_d_vInt = sqrt( sum(d_vInt.^2,1) );
% % Find all values that are larger than our ZERO tolerance
% bin = norm_d_vInt > ZERO;
% 
% % Keep only non-redundant values
% intPt = intPt(:,bin);
% onVertex = onVertex(:,bin);

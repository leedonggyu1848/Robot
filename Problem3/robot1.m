clc; clear all; close all; 

% parameter
cur = [0, 0].';
goal = [10, 10].';
obstacles = [[6,4].', [4,6].', [6,6].'];
epsilon = 1; % attractive distance
zeta = 0.03;    % attractive tunning parameter
delta = 3;   % repulsive distance
eta = 0.2;     % repulsive tunning paramter

% calculate force function
function force = cal_attractive(cur_wp, goal_wp, epsilon, zeta)
    % pre-calculating values
    diff = cur_wp - goal_wp;
    norm_diff = norm(diff);
    
    if norm_diff > epsilon
        % condition 1
        force = -epsilon * zeta * diff / norm_diff;
    else
        % condition 2
        force = -zeta * diff;
    end
end

function force = cal_repulsive(cur_wp, o, delta, eta)
 dim = getDimVec(cur_wp);
    n_o = getNumVec(o);
    force = zeros(dim, 1);
    for i=1:n_o
        diff = cur_wp - getNthVec(o, i);
        norm_diff = norm(diff);
        if norm_diff < delta
            % case3
            force = force + eta ...
                            * (1/norm_diff - 1/delta)...
                            * (1/(norm_diff^2))...
                            * (diff / norm_diff);
        else
            % case 4
        end
    end
end

% Functions

function rst = getNumVec(matrix)
    [~, rst] = size(matrix);
end

function rst = getDimVec(matrix)
    [rst, ~] = size(matrix);
end

function rst = getNthVec(matrix, n)
    rst = matrix(:, n);
end

function rst = getNthDim(matrix, n)
    rst = matrix(n, :);
end

function rst = isReached(a, b)
 rst = getDistance(a, b) < 0.5;
end

function rst = getDistance(a, b)
    rst = norm(a - b);
end

obstacles_size = getNumVec(obstacles);

% Configuration figure
hfig = figure(1);
axis equal;
axis_max = 10;
axis([-axis_max, axis_max, -axis_max, axis_max]);

cur_dot = line(inf, inf);
goal_dot = line(goal(1), goal(2));
obstacle_dots = gobjects(1, obstacles_size);

for i=1:obstacles_size
    obstacle = getNthVec(obstacles, i);
    obstacle_dots(i) = line(obstacle(1), obstacle(2));
    set(obstacle_dots(i),'Marker', 'o', 'MarkerSize', 10,...
    'MarkerEdgeColor', 'k', 'MarkerFaceColor', 'k');
end

set(cur_dot,'Marker', 'o', 'MarkerSize', 10,...
'MarkerEdgeColor', 'g', 'MarkerFaceColor', 'g');
set(goal_dot,'Marker', 'o', 'MarkerSize', 10,...
'MarkerEdgeColor', 'r', 'MarkerFaceColor', 'r');

% calculate setting
while ~isReached(cur, goal)
    set(cur_dot, 'XData', getNthDim(cur, 1), 'YData', getNthDim(cur, 2));
    drawnow;

    force = cal_attractive(cur, goal, epsilon, zeta)...
            + cal_repulsive(cur, obstacles, delta, eta);
    
    if norm(force) >= 0.00000001
        cur = cur + force;
    end
end

clc; clear all; close all; 

% parameter
cur = [0, 0];
goal = [10, 10];
obstacles = [6,4; 4,6; 6,6];
epsilon = 0.5; % attractive distance
zeta = 0.01;    % attractive tunning parameter
delta = 3;   % repulsive distance
eta = 0.03;     % repulsive tunning paramter

% calculate force function
function force = cal_attractive(q, q_goal, epsilon, zeta)
    % pre-calculating values
    diff = q - q_goal;
    norm_diff = norm(diff);
    
    if norm_diff > epsilon
        % condition 1
        force = -epsilon * zeta * diff / norm_diff;
    else
        % condition 2
        force = -zeta * diff;
    end
end

function force = cal_repulsive(q, o, n_o, delta, eta)
    [~, col] = size(q);
    force = zeros(1, col);
    for i=1:n_o
        diff = q - o(i,:);
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
function rst = isReached(a, b)
 rst = getDistance(a, b) < 0.5;
end

function rst = getDistance(a, b)
    rst = norm(a - b);
end

[n_obstacles, ~] = size(obstacles);
% 
% Configuration figure
hfig = figure(1);
axis equal;
axis_max = 10;
axis([-axis_max, axis_max, -axis_max, axis_max]);

cur_dot = line(inf, inf);
goal_dot = line(goal(1), goal(2));
obstacle_dots = gobjects(1, n_obstacles);
for i=1:n_obstacles
    obstacle_dots(i) = line(obstacles(i, 1), obstacles(i, 2));
    set(obstacle_dots(i),'Marker', 'o', 'MarkerSize', 10,...
    'MarkerEdgeColor', 'k', 'MarkerFaceColor', 'k');
end

set(cur_dot,'Marker', 'o', 'MarkerSize', 10,...
'MarkerEdgeColor', 'g', 'MarkerFaceColor', 'g');
set(goal_dot,'Marker', 'o', 'MarkerSize', 10,...
'MarkerEdgeColor', 'r', 'MarkerFaceColor', 'r');

% calculate setting
while ~isReached(cur, goal)
    set(cur_dot, 'XData', cur(1), 'YData', cur(2));
    drawnow;

    force = cal_attractive(cur, goal, epsilon, zeta)...
            + cal_repulsive(cur, obstacles, n_obstacles, delta, eta);
    
    if norm(force) >= 0.00000001
        cur = cur + force;
    end
end

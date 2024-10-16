clc;
clear all;
close all;

start = [-7, -7, -7];
goal = [7, 7, 7];
obstacles = [3,3,3; 1,2,3; 4,5,6; -1,-2,-3];
vec_tri_point = [0, 3, 0; -1, 0, 0; 1, 0, 0];
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

function h = calRotation(ax, ay, az, dx, dy, dz)

    roll = [cos(az), -sin(az), 0;
            sin(az), cos(az), 0;
            0, 0, 1];
    pitch = [cos(ay), 0, sin(ay);
             0, 1, 0
             -sin(ay), 0, cos(ay)];
    yaw = [1, 0, 0;
           0, cos(ax), -sin(ax);
           0, sin(ax), cos(ax)];

    R = roll * pitch * yaw;
    d = [dx, dy, dz].';
    h = [R, d;
         zeros(1, 3), 1];
end

% Functions
function rst = isReached(a, b)
 rst = getDistance(a, b) < 0.5;
end

function rst = getDistance(a, b)
    rst = norm(a - b);
end

% initializing
cur_wp = [start; start; start] + vec_tri_point;
goal_wp = [goal;goal;goal] + vec_tri_point;
h = calRotation(pi/6, pi/6, pi/6, 0, 0, 0);
goal_wp = h * [goal_wp.'; ones(1,3)];
goal_wp = goal_wp(1:3, :).';

% Configuration figure
figure(1);
grid on;
axis equal;
view(3);
axis_max = 11;
axis([-axis_max, axis_max, -axis_max, axis_max, -axis_max, axis_max]);
hold on;

[obstacles_size, ~] = size(obstacles);
obstacle_dots = gobjects(1, obstacles_size);
for i=1:obstacles_size
    obstacle = obstacles(i, :);
    obstacle_dots(i) = scatter3(obstacle(1), obstacle(2), obstacle(3), 100, 'r', 'filled');
end

cur_tri = fill3(cur_wp(:, 1), cur_wp(:, 2), cur_wp(:, 3), 'b');
goal_tri = fill3(goal_wp(:, 1), goal_wp(:, 2), goal_wp(:, 3), 'g');

while ~isReached(cur_wp, goal_wp)
    set(cur_tri, 'XData', cur_wp(:, 1), 'YData', cur_wp(:, 2), 'ZData', cur_wp(:, 3));
    drawnow;
    
end
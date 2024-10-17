clc;
clear all;
close all;

% configuration values
start = [-7, -7, -7].';
goal = [7, 7, 7].';
obstacles = [[-4, -3, -4].', [-2, -1, -1].', [-6, -4, -6].', [3, 4, 5].'];
tri_vec = [[0, 3, 0].', [-1, 0, 0].', [1, 0, 0].'];
epsilon = 1; % attractive distance
zeta = 0.05;    % attractive tunning parameter
delta = 2;   % repulsive distance
eta = 0.9;     % repulsive tunning paramter


% calculating force function
function force = calAttractive(cur_wp, goal_wp, epsilon, zeta)
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

function force = calRepulsive(cur_wp, o, delta, eta)
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
 rst = true;
 for i=1:3
    if getDistance(getNthVec(a, i), getNthVec(b, i)) >= 0.1
        rst = false;
    end
 end
end

function rst = getDistance(a, b)
    rst = norm(a - b);
end

% initializing values
syms ax ay az dx dy dz;
% current
cur_wp = tri_vec;
h = calRotation(ax, ay, az, dx + start(1), dy + start(2), dz + start(3));
cur_wx = h * [cur_wp; ones(1,3)];
cur_wx = cur_wx(1:3, :);
cur_cp = [0, 0, 0, 0, 0, 0]; % ax, ay, az, dx, dy, dz
cur_wp =  double(subs(cur_wx, [ax, ay, az, dx, dy, dz], cur_cp));
x_jaco = {jacobian(getNthVec(cur_wx, 1), [ax, ay, az, dx, dy, dz]),...
     jacobian(getNthVec(cur_wx, 2), [ax, ay, az, dx, dy, dz]),...
     jacobian(getNthVec(cur_wx, 3), [ax, ay, az, dx, dy, dz])};
% goal
goal_wp = tri_vec;
h = calRotation(pi/6, pi/6, pi/6, goal(1), goal(2), goal(3));
goal_wp = h * [goal_wp; ones(1,3)];
goal_wp = goal_wp(1:3, :);


% Configuration figure
figure(1);
grid on;
axis equal;
view(3);
axis_max = 11;
axis([-axis_max, axis_max, -axis_max, axis_max, -axis_max, axis_max]);
hold on;

obstacles_size = getNumVec(obstacles);
obstacle_dots = gobjects(1, obstacles_size);

for i=1:obstacles_size
    obstacle = getNthVec(obstacles, i);
    obstacle_dots(i) = scatter3(obstacle(1), obstacle(2), obstacle(3), 100, 'r', 'filled');
end

cur_tri = fill3(getNthDim(cur_wp, 1), getNthDim(cur_wp, 2), getNthDim(cur_wp, 3), 'b');
fill3(getNthDim(goal_wp, 1), getNthDim(goal_wp, 2), getNthDim(goal_wp, 3), 'g');

% main loop
while ~isReached(cur_wp, goal_wp)
    cur_wp = double(subs(cur_wx, [ax, ay, az, dx, dy, dz], cur_cp))
    set(cur_tri, 'XData', getNthDim(cur_wp, 1), 'YData', getNthDim(cur_wp, 2), 'ZData', getNthDim(cur_wp, 3));
    drawnow;
    force = zeros(6, 1);
    for i=1:3
        jaco = subs(x_jaco{i}, [ax, ay, az, dx, dy, dz], cur_cp);
        jaco = double(jaco);
        force = force + jaco.' * calAttractive(getNthVec(cur_wp, i), getNthVec(goal_wp, i), epsilon, zeta);
        force = force + jaco.' * calRepulsive(getNthVec(cur_wp, i), obstacles, delta, eta);
    end
    cur_cp = cur_cp + force.';
end
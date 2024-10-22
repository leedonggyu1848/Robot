clc;
clear all;
close all;

% configuration values
goal = [[6, -1, 4].', [5, -2, 6].', [6, -4, 3].', [4, 1, 5].', [3, 2, 1].', [0, -6, 0].', [-3, -5, 2].', [-4, 6, 3].', [-5, 3, 2].', [-6, 0, 5].'];
link_length = 5;

epsilon = 1; % attractive distance
zeta = 0.01;    % attractive tunning parameter

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

% Base homogeneous matrix
function h = transX(d)
    h = [
        1, 0, 0, d;
        0, 1, 0, 0;
        0, 0, 1, 0;
        0, 0, 0, 1
    ];
end

function h = transZ(d)
    h = [
        1, 0, 0, 0;
        0, 1, 0, 0;
        0, 0, 1, d;
        0, 0, 0, 1
    ];
end

function h = rotX(a)
    h = [
        1, 0, 0, 0;
        0, cos(a), -sin(a), 0;
        0, sin(a), cos(a), 0;
        0, 0, 0, 1
    ];
end

function h = rotZ(a)
    h = [
        cos(a), -sin(a), 0, 0;
        sin(a), cos(a), 0, 0;
        0, 0, 1, 0;
        0, 0, 0, 1
    ];
end

% Calculate DH transform
function h = calTranslation(theta, d, a, alpha)
    h = rotZ(theta) * transZ(d) * transX(a) * rotX(alpha);
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
if getDistance(a, b) >= 0.1
    rst = false;
end
end

function rst = getDistance(a, b)
    rst = norm(a - b);
end

% initializing values
syms th1 th2 th3; % configuration

% current
a = {};
a{1} = calTranslation(th1, link_length, 0, pi/2);
a{2} = calTranslation(th2, 0, link_length, 0);
a{3} = calTranslation(th3, 0, link_length, 0);
base = [0,0,0,1].'; % p1
cur_wx = [
    base,...
    a{1} * base,...
    a{1} * a{2} * base,...
    a{1} * a{2} * a{3} * base];
cur_wx = cur_wx(1:3, :);

cur_cp = [0, 0, 0]; % th1, th2, th3
cur_wp =  double(subs(cur_wx, [th1, th2, th3], cur_cp));
x_jaco = jacobian(getNthVec(cur_wx, 4), [th1, th2, th3]);

% goal

goal_wp = goal;

hfig = figure(1);
grid on;
axis equal;
view(3);
axis_max = 10;
axis([-axis_max, axis_max, -axis_max, axis_max, 0, 8]);
hold on;

dots = gobjects(1, 4);
for i = 1:4
    cur = getNthVec(cur_wp, i);
    dots(i) = scatter3(cur(1), cur(2), cur(3), 'b', 'filled');
end
links = gobjects(1, 3);
for i = 1:3
    cur = [getNthVec(cur_wp, i), getNthVec(cur_wp, i+1)];
    links(i) = line(getNthDim(cur, 1), getNthDim(cur, 2), getNthDim(cur, 3), 'Color', 'k');
end

goal_dots = gobjects(1,getNumVec(goal));
for t=1:getNumVec(goal)
    goal_wp = getNthVec(goal, t);
    goal_dots(t) = scatter3(goal_wp(1), goal_wp(2), goal_wp(3), 'r', 'filled');
end
for t=1:getNumVec(goal)
    goal_wp = getNthVec(goal, t);
    while ~isReached(getNthVec(cur_wp, 4), goal_wp)
        cur_wp = double(subs(cur_wx, [th1, th2, th3], cur_cp));
        for i = 1:4
            cur = getNthVec(cur_wp, i);
            set(dots(i), 'XData', cur(1), 'YData', cur(2), 'ZData', cur(3));
        end
        for i = 1:3
            cur = [getNthVec(cur_wp, i), getNthVec(cur_wp, i+1)];
            set(links(i), 'XData', getNthDim(cur, 1), 'YData', getNthDim(cur, 2), 'ZData', getNthDim(cur, 3));
        end
        drawnow;
    
        jaco = subs(x_jaco, [th1, th2, th3], cur_cp);
        jaco = double(jaco);
        force = jaco.' * calAttractive(getNthVec(cur_wp, 4), goal_wp, epsilon, zeta);
        cur_cp = cur_cp + force.';
    end
    delete(goal_dots(t));
end
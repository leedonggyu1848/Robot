clc;
clear all;
close all; 

% parameter
cur = [0, 0].';
goal = [8, 8].';
ax = 2;
ay = 3;
epsilon = 0.5; % attractive distance
zeta = 0.1;    % attractive tunning parameter

% calculate force function
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

syms x y theta;
cur_wx = [[x, y].',...
     [x+ax*cos(theta),y+ax*sin(theta)].'...
     [x+ax*cos(theta)-ay*sin(theta), y+ax*sin(theta)+ay*cos(theta)].'];
goal_wp = [[goal(1), goal(2)].', [goal(1), goal(2)+ax].', [goal(1)-ay, goal(2)+ax].'];
cur_cp = [0,0,0]; % [x, y, theta]
x_jaco = {jacobian(getNthVec(cur_wx, 1), [x,y,theta]),...
     jacobian(getNthVec(cur_wx, 2), [x,y,theta]),...
     jacobian(getNthVec(cur_wx, 3), [x,y,theta])};
cur_wp = zeros(2,3);

% Configuration figure
hfig = figure(1);
grid on;
axis equal;
axis_max = 13;
axis([0, axis_max, 0, axis_max]);

dot = gobjects(1, 3);
dot_goals = gobjects(1, 3);
for i = 1:3
    dot(i) = line(inf, inf);
    set(dot(i),'Marker', 'o', 'MarkerSize', 10,...
    'MarkerEdgeColor', 'k', 'MarkerFaceColor', 'k');
    dot_goals(i) = line(inf, inf);
    set(dot_goals(i),'Marker', 'o', 'MarkerSize', 10,...
    'MarkerEdgeColor', 'g', 'MarkerFaceColor', 'g');
end

for i = 1:3
    set(dot_goals(i), 'XData', goal_wp(1, i), 'YData', goal_wp(2, i));
end

% calculate setting
while ~isReached(cur_wp, goal_wp)
    cur_wp = double(subs(cur_wx, [x, y, theta], cur_cp));
    for i = 1:3
        set(dot(i), 'XData', cur_wp(1, i), 'YData', cur_wp(2,i));
    end
    drawnow;

    force = zeros(3, 1);
    for i=1:3
        jaco = subs(x_jaco{i}, [x, y, theta], cur_cp);
        jaco = double(jaco);
        force = force + jaco.' * calAttractive(getNthVec(cur_wp, i), getNthVec(goal_wp, i), epsilon, zeta);
    end
    cur_cp = cur_cp + force.';
end
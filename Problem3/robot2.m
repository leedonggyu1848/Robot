clc;
clear all;
close all; 

% parameter
cur = [0, 0];
goal = [8, 8];
ax = 2;
ay = 3;
epsilon = 0.5; % attractive distance
zeta = 0.1;    % attractive tunning parameter

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

% Functions
function rst = isReached(a, b)
 rst = true;
 for i=1:3
    if getDistance(a{i}, b{i}) >= 0.1
        rst = false;
    end
 end
end

function rst = getDistance(a, b)
    rst = norm(a - b);
end

syms x y theta;
wx_cur = {[x, y],...
     [x+ax*cos(theta),y+ax*sin(theta)],...
     [x+ax*cos(theta)-ay*sin(theta), y+ax*sin(theta)+ay*cos(theta)]};
wp_goal = {[goal(1), goal(2)], [goal(1), goal(2)+ax], [goal(1)-ay, goal(2)+ax]};
cp_cur = [0,0,0]; % [x, y, theta]
x_jaco = {jacobian(wx_cur{1}, [x,y,theta]),...
     jacobian(wx_cur{2}, [x,y,theta]),...
     jacobian(wx_cur{3}, [x,y,theta])};
wp_cur = {[0, 0], [0, 0], [0, 0]};

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
    set(dot_goals(i), 'XData', wp_goal{i}(1), 'YData', wp_goal{i}(2));
end
% calculate setting
while ~isReached(wp_cur, wp_goal)
    for i = 1:3
        wp_cur{i} = subs(wx_cur{i}, [x, y, theta], cp_cur);
        set(dot(i), 'XData', wp_cur{i}(1), 'YData', wp_cur{i}(2));
    end
    drawnow;

    force = zeros(3, 1);
    for i=1:3
        jaco = subs(x_jaco{i}, [x, y, theta], cp_cur);
        force = force + jaco.' * cal_attractive(wp_cur{i}, wp_goal{i}, epsilon, zeta).';
        force = vpa(force);
    end
    cp_cur = cp_cur + force.';
end
clc;
clear all;
close all; 

% Base homogeneous matrix
function h=transX(d)
    h = [
        1, 0, 0, d;
        0, 1, 0, 0;
        0, 0, 1, 0;
        0, 0, 0, 1
    ];
end

function h=transZ(d)
    h = [
        1, 0, 0, 0;
        0, 1, 1, 0;
        0, 0, 1, d;
        0, 0, 0, 1
    ];
end

function h=rotX(a)
    h = [
        1, 0, 0, 0;
        0, cos(a), -sin(a), 0;
        0, sin(a), cos(a), 0;
        0, 0, 0, 1
    ];
end

function h=rotZ(a)
    h = [
        cos(a), -sin(a), 0, 0;
        sin(a), cos(a), 0, 0;
        0, 0, 1, 0;
        0, 0, 0, 1
    ];
end

% Calculate DH transform
function h = calDHTransform(theta, d, a, alpha)
    h = rotZ(theta) * transZ(d) * transX(a) * rotX(alpha);
end

% Functions
function rst = isSame(a, b)
 rst = abs(a - b) < 0.5;
end

% Configuration figure
hfig = figure(1);
grid on;
axis equal;
axis_max = 10;
axis([-axis_max, axis_max, -axis_max, axis_max]);

link = gobjects(1, 2);
for i = 1:2
    link(i) = line([inf, inf], [inf, inf]);
    set (link(i), 'Color', 'g', 'LineWidth', 7);
end

dot = gobjects(1, 3);
for i = 1:3
    dot(i) = line(inf, inf);
    set(dot(i),'Marker', 'o', 'MarkerSize', 10,...
    'MarkerEdgeColor', 'k', 'MarkerFaceColor', 'k');
end

% calculate setting
cur = [0, 0];
tars = [90, 60;
        30, -30;
        70, -30;
        0, 0];
[ros, cols] = size(tars);
speed = 0.5;

for t=1:ros
    while ~isSame(cur(1), tars(t,1)) || ~isSame(cur(2), tars(t,2))
        % DH parameter
        a = {};
        a{1} = calDHTransform(deg2rad(cur(1)), 0, 2, 0);
        a{2} = calDHTransform(deg2rad(cur(2)), 0, 3, 0);
        p = {};
        p{1} = [0, 0, 0, 1].';
        p{2} = a{1} * p{1};
        p{3} = a{1} * a{2} * p{1};
    
        for i = 1:3
            set(dot(i), 'XData', p{i}(1), 'YData', p{i}(2));
        end
        for i = 1:2
            set(link(i), 'XData', [p{i}(1), p{i+1}(1)],...
                'YData', [p{i}(2), p{i+1}(2)]);
        end

        drawnow;
        div = tars(t, :) - cur;
        div = div / norm(div);
        cur = cur + (div * speed);
    end
end
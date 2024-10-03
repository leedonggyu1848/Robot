clc;
clear all;
close all; 

% base homogeneous matrix
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

% calulate
function h = calDHTransform(theta, d, a, alpha)
    h = rotZ(theta) * transZ(d) * transX(a) * rotX(alpha);
end

% function
function rst = isSame(a, b)
 rst = abs(a - b) < 0.5;
end

% configuration figure
hfig = figure(1);
grid on;
axis equal;
view(3);
axis_max = 10
axis([-axis_max, axis_max, -axis_max, axis_max, -axis_max, axis_max]);

link = gobjects(1, 5);
for i = 1:5
    link(i) = line([inf, inf], [inf, inf],+ [inf, inf]);
    set (link(i), 'color', 'g', 'linewidth', 7);
end

dot = gobjects(1, 6);
for i = 1:6
    dot(i) = line(inf, inf, inf);
    set (dot(i), 'marker', 'o', 'markersize', 10,...
    'markeredgecolor', 'k', 'markerfacecolor', 'k');
end

% calculate setting
cur = [0, 0, 0, 0, 0];
tars = [30, 0, 0, 0, 0;];
        % 150, 0, 0, 0, 0;];

[ros, cols] = size(tars);
speed = 0.5;

for t=1:ros
    while ~isSame(cur(1), tars(t,1)) || ~isSame(cur(2), tars(t,2))
        % DH parameter
        %{
            theta  d      a      alpha
            i1,    0.077, 0,     90
            i2+90, 0,     0.128, 0
            -90,   0,     0.024, 0
            i4,    0,     0.124, 0
            i5,    0,     0.126, 0
        %}
        a = {};
        a{1} = calDHTransform(deg2rad(cur(1)), 2, 0, deg2rad(90));
        a{2} = calDHTransform(deg2rad(cur(2)+90), 0, 2, 0);
        a{3} = calDHTransform(deg2rad(-90), 0, 0.5, 0);
        a{4} = calDHTransform(deg2rad(cur(4)), 0, 2, 0);
        a{5} = calDHTransform(deg2rad(cur(5)), 0, 2, 0);
    
        p = {};
        p{1} = [0, 0, 0, 1].';
        p{2} = a{1} * p{1};
        p{3} = a{1} * a{2} * p{1};
        p{4} = a{1} * a{2} * a{3} * p{1};
        p{5} = a{1} * a{2} * a{3} * a{4} * p{1};
        p{6} = a{1} * a{2} * a{3} * a{4} * a{5} * p{1};


        for i = 1:5
            set(link(i), ...
                'xdata', [p{i}(1), p{i+1}(1)], ...
                'ydata', [p{i}(2), p{i+1}(2)], ...
                'zdata', [p{i}(3), p{i+1}(3)]);
        end
        for i = i:6
            set(dot(i), ...
                'xdata', p{i}(1), ...
                'ydata', p{i}(2), ...
                'zdata', p{i}(3));
        end

        drawnow;
        div = tars(t, :) - cur;
        div = div / norm(div);
        cur = cur + (div * speed);
    end
end
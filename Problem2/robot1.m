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
axis([-10, 10, -10, 10, -10, 10]);
line1 = line([inf, inf], [inf, inf], [inf, inf]);
line2 = line([inf, inf], [inf, inf], [inf, inf]);
line3 = line([inf, inf], [inf, inf], [inf, inf]);
line4 = line([inf, inf], [inf, inf], [inf, inf]);
line5 = line([inf, inf], [inf, inf], [inf, inf]);

dot1 = line(inf, inf, inf);
dot2 = line(inf, inf, inf);
dot3 = line(inf, inf, inf);
dot4 = line(inf, inf, inf);
dot5 = line(inf, inf, inf);
dot6 = line(inf, inf, inf);

set (dot1, 'marker', 'o', 'markersize', 10,...
    'markeredgecolor', 'k', 'markerfacecolor', 'k');
set (dot2, 'marker', 'o', 'markersize', 10,...
    'markeredgecolor', 'k', 'markerfacecolor', 'k');
set (dot3, 'marker', 'o', 'markersize', 10,...
    'markeredgecolor', 'k', 'markerfacecolor', 'k');
set (dot4, 'marker', 'o', 'markersize', 10,...
    'markeredgecolor', 'k', 'markerfacecolor', 'k');
set (dot5, 'marker', 'o', 'markersize', 10,...
    'markeredgecolor', 'k', 'markerfacecolor', 'k');
set (dot6, 'marker', 'o', 'markersize', 10,...
    'markeredgecolor', 'k', 'markerfacecolor', 'k');
set (line1, 'color', 'g', 'linewidth', 7);
set (line2, 'color', 'g', 'linewidth', 7);
set (line3, 'color', 'g', 'linewidth', 7);
set (line4, 'color', 'g', 'linewidth', 7);
set (line5, 'color', 'g', 'linewidth', 7);

% calculate setting
cur = [0, 0, 0, 0, 0];
tars = [90, 60;
        30, -30;
        70, 30;
        0, 0];
[ros, cols] = size(tars);
speed = 0.5;

for i=1:ros
    while ~isSame(cur(1), tars(i,1)) || ~isSame(cur(2), tars(i,2))
        % DH parameter
        %{
            theta  d      a      alpha
            i1,    0.077, 0,     90
            i2+90, 0,     0.128, 0
            -90,   0,     0.024, 0
            i4,    0,     0.124, 0
            i5,    0,     0.126, 0
        %}
        t01 = calDHTransform(deg2rad(cur(1)), 1, 0, deg2rad(90));
        t12 = calDHTransform(deg2rad(cur(2)+90), 0, 1, 0);
        t23 = calDHTransform(deg2rad(-90), 0, 0.2, 0);
        t34 = calDHTransform(deg2rad(cur(4)), 0, 1, 0);
        t45 = calDHTransform(deg2rad(cur(5)), 0, 1, 0);

        p1 = [0, 0, 0, 1].';
        p2 = t01 * p1;
        p3 = t12 * p2;
        p4 = t23 * p3;
        p5 = t34 * p4;
        p6 = t45 * p5;
    
        set(dot1, 'xdata', p1(1), 'ydata', p1(2), 'zdata', p1(3));
        set(dot2, 'xdata', p2(1), 'ydata', p2(2), 'zdata', p2(3));
        set(dot3, 'xdata', p3(1), 'ydata', p3(2), 'zdata', p3(3));
        dot4 = plot3(p4(1), p4(2), p4(3), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
        dot5 = plot3(p5(1), p5(2), p5(3), 'bo', 'MarkerSize', 10, 'MarkerFaceColor', 'b');
        dot6 = plot3(p6(1), p6(2), p6(3), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
% 선 위치 업데이트 (각 선의 시작과 끝점에 대한 x, y, z 데이터 설정)
set(line1, 'xdata', [p1(1), p2(1)], 'ydata', [p1(2), p2(2)], 'zdata', [p1(3), p2(3)]);
set(line2, 'xdata', [p2(1), p3(1)], 'ydata', [p2(2), p3(2)], 'zdata', [p2(3), p3(3)]);

% p4에서 p6까지 선 추가 (추가로 line3, line4 설정)
line3 = line([p3(1), p4(1)], [p3(2), p4(2)], [p3(3), p4(3)], 'Color', 'g');
line4 = line([p4(1), p5(1)], [p4(2), p5(2)], [p4(3), p5(3)], 'Color', 'm');
line5 = line([p5(1), p6(1)], [p5(2), p6(2)], [p5(3), p6(3)], 'Color', 'c');

% p4, p5, p6에 대한 새로운 점 추가 (옵션)

        set(dot1, 'xdata', p1(1), 'ydata', p1(2), 'zdata', 0);
        set(dot2, 'xdata', p2(1), 'ydata', p2(2), 'zdata', 0);
        set(dot3, 'xdata', p3(1), 'ydata', p3(2), 'zdata', 0);
        set(line1, 'xdata', [p1(1), p2(1)], 'ydata', [p1(2), p2(2)]);
        set(line2, 'xdata', [p2(1), p3(1)], 'ydata', [p2(2), p3(2)]);
        drawnow;
        div = tars(i, :) - cur;
        div = div / norm(div);
        cur = cur + (div * speed);
    end
end
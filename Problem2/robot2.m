clc;
clear all;
close all; 

% Calculate DH transform
function h = calDHTransform(az, ay, ax, dx, dy, dz)
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
function rst = isSame(a, b)
    rst = abs(a - b) < 0.5;
end

function rst = isAllSame(a, b, n)
    rst = true;
    for i=1:n
        if (~isSame(a(i), b(i)))
            rst = false;
            return
        end
    end
end

% Configuration figure
hfig = figure(1);
grid on;
axis equal;
view(3);
axis_max = 20;
axis([-axis_max, axis_max, -axis_max, axis_max, -axis_max, axis_max]);
hold on;

plane = patch('Vertices', [[0,0,0]; [0,0,0]; [0,0,0]], ... % 세 점의 좌표
          'Faces', [1, 2, 3], ... % 세 점을 잇는 면
          'FaceColor', 'r', ... % 색상 설정
          'EdgeColor', 'k'); % 가장자리 색상

cur = [0,0,0,0,0,0]; % az, ay, ax, dx, dy, dz
tars = [0, 0, 3600, 20, 0, 0];
[ros, cols] = size(tars);
speed = 2;
for t = 1:ros
    while ~isAllSame(cur, tars(t, :), 6)
        % DH parameter
        h = calDHTransform( ...
            deg2rad(cur(1)), deg2rad(cur(2)), deg2rad(cur(3)), ...
            cur(4), cur(5), cur(6));
        p1 = [2, 0, 0].';
        p2 = [-4, 3, 0].';
        p3 = [-4, -3, 0].';
        p = [p1, p2, p3; ones(1,3)];
        p = h * p;
        p = p.';
        set(plane, 'Vertices', p(:, 1:3));

        drawnow;
        div = tars(t, :) - cur;
        div = div / norm(div);
        cur = cur + (div * speed);
    end
end

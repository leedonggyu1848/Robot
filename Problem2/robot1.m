clc;
clear all;
close all; 

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
function h = calDHTransform(theta, d, a, alpha)
    h = rotZ(theta) * transZ(d) * transX(a) * rotX(alpha);
end

% Functions
function rst = isSame(a, b)
    rst = abs(a - b) < 0.5;
end

% Draw a box
function h = drawBox(p1, p2, width)
    % p1, p2 are 3D points [x, y, z], width is the box thickness
    % Calculate vector between points
    v = p2 - p1;
    % Normalize the vector
    v = v / norm(v);
    % Generate a perpendicular vector
    u = cross(v, [0 0 1]);
    if norm(u) == 0
        u = cross(v, [0 1 0]);
    end
    u = u / norm(u);
    % Calculate another perpendicular vector
    w = cross(v, u);
    w = w / norm(w);

    % Calculate the box vertices
    half_w = width / 2;
    corners = [
        p1 + half_w*u + half_w*w;
        p1 + half_w*u - half_w*w;
        p1 - half_w*u - half_w*w;
        p1 - half_w*u + half_w*w;
        p2 + half_w*u + half_w*w;
        p2 + half_w*u - half_w*w;
        p2 - half_w*u - half_w*w;
        p2 - half_w*u + half_w*w;
    ];

    % Draw the box faces using patch
    faces = [
        1, 2, 6, 5;
        2, 3, 7, 6;
        3, 4, 8, 7;
        4, 1, 5, 8;
        1, 2, 3, 4;
        5, 6, 7, 8;
    ];

    h = patch('Vertices', corners, 'Faces', faces, 'FaceColor', 'g', 'EdgeColor', 'k', 'FaceAlpha', 0.5);
end

% Configuration figure
hfig = figure(1);
grid on;
axis equal;
view(3);
axis_max = 10;
axis([-axis_max, axis_max, -axis_max, axis_max, -axis_max, axis_max]);
hold on;

%Create graphic components
[XS, YS, ZS] = sphere(20);
links = gobjects(1, 5);
dots = gobjects(1, 6);

for i = 1:5
    links(i) = drawBox([0, 0, 0], [0, 0, 0], 0.2);
end

for i = 1:6
    dots(i) = surf(XS*0.3, YS*0.3, ZS*0.3, 'FaceColor', 'k'); % Create empty spheres
end

% Calculate setting
cur = [0, 0, 0, 0, 0];
tars = [360, 90, 0, 90, 60;];
[ros, cols] = size(tars);
speed = 0.5;

for t = 1:ros
    while ~isSame(cur(1), tars(t, 1)) || ~isSame(cur(2), tars(t, 2))
        % DH parameter
        a = {};
        a{1} = calDHTransform(deg2rad(cur(1)), 2, 0, deg2rad(90));
        a{2} = calDHTransform(deg2rad(cur(2) + 90), 0, 2, 0);
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
            link_pos1 = p{i}(1:3)';
            link_pos2 = p{i+1}(1:3)';
            delete(links(i));
            links(i) = drawBox(link_pos1, link_pos2, 0.2);
        end
        for i = 1:6
            set(dots(i), 'XData', XS*0.3 + p{i}(1), ...
                'YData', YS*0.3 + p{i}(2), ...
                'ZData', ZS*0.3 + p{i}(3));
        end

        drawnow;
        div = tars(t, :) - cur;
        div = div / norm(div);
        cur = cur + (div * speed);
    end
end

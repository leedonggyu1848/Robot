clc; clear all; close all; % reset env

obstacles_pos = [[-7,13].', [8, 13].', [0, 10].', [-10, 10].', [10, 10].'...
    [-5,5].', [-10,5].', [5,5].', [10,5].'...
    [-7,0].',[-2,0].',[3,0].',[8,0].'];
obstacle_length = 2; % 장애물 한 변 길이

link_length = 5; % 링크 길이
start_pos = [0, 15].'; % robot start position

coord_min = [-15, -10];  % (xmin, ymin)
coord_max = [15, 20];  % (xmax, ymax)

n_1d = 70; % 한 차원 당 샘플 개수
conn_dist = 0.1; % sample 연결 거리

hifg = figure(1); grid on; axis equal, hold on;
axis([coord_min(1), coord_max(1), coord_min(2), coord_max(2)]);

robot = LinkRobot(start_pos, [0, 0, 0], link_length);
obstacle = Obstacle(obstacles_pos, obstacle_length, [-10, 10], [0, 16]);
robot.draw;
drawnow;

% generate samples
[A, B, C] = ndgrid(linspace(-pi/2, pi/2, n_1d/2),...
    linspace(-pi, pi, n_1d),...
    linspace(-pi, pi, n_1d));
samples = [A(:), B(:), C(:)].';
sample_size = Utils.getNumVec(samples)

% filter out collision samples
valid_samples = true(1, sample_size);
for i = 1 : sample_size
    if mod(i, 100000) == 0
        i
    end
    cur_sample = Utils.getNthVec(samples, i);
    wp = robot.calPoints(cur_sample);
    if (obstacle.isCollisionPoints(wp))
        valid_samples(i) = false;
    end
end

function rst = isConnectableInConfig(start, goal, robot, obstacle)
    step = 0.1;
    dir = (goal - start) / norm(goal - start);
    cur = start;
    rst = true;
    while Utils.calDistance(cur, goal) > step
        cur = cur + dir * step;
        wp = robot.calPoints(cur);
        if obstacle.isCollisionPoint(wp)
            rst = false;
            return
        end
    end
end

valid_samples = samples(:, valid_samples);
valid_samples_size = Utils.getNumVec(valid_samples)
adjust_matrix = false(valid_samples_size, valid_samples_size);
tree = KDTree(valid_samples);
for i = 1:valid_samples_size
    adjust_matrix(i, i) = true;
    if mod(i, 1000) == 0
        i
        Utils.getNumVec(rst)
    end
    rst = tree.findWithinDistance(conn_dist, Utils.getNthVec(valid_samples, i));
    for j = 1:Utils.getNumVec(rst)
        if (rst(j).index <= i)
            continue;
        end
        if isConnectableInConfig(Utils.getNthVec(valid_samples, i), rst(j).point, robot, obstacle)
            adjust_matrix(i, rst(j).index) = true;
            adjust_matrix(rst(j).index, i) = true;
        end
    end
end

G = graph(adjust_matrix);
dist = distances(G);
dist(~isfinite(dist)) = -1;
max_dist = max(max(dist))
[y, x] = size(dist);
src = 0;
tar = 0;
max_v = 0;
for i = 1:y
    for j = i:x
        if dist(i, j) ==max_dist
            src = i;
            tar = j;
        end
    end
end

path = shortestpath(G, src, tar);
dist(src, tar)
for i = 1:Utils.getNumVec(path) - 1
    Utils.getNthVec(valid_samples, path(i))
    robot = robot.update(Utils.getNthVec(valid_samples, path(i)));
    robot.draw;
    drawnow;
    pause
end

% TODO: 근처에 n개의 점을 찾는 것으로 변경 고려
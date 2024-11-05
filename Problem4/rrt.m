clc; clear all; close all; % reset env

space_size = 20;
hole_size = 6;
box_size = 3;
start_point = [-5, 5, 5, 0, 0, 0].';
dest_point = [5, -5, -5, 0, 0, 0].';
max_pos = 5;
min_pos = -5;
step = 0.5;

wall_long = space_size;
wall_short = (space_size - hole_size)/2;
wall_move = hole_size/2 + wall_short/2;
wall = {};
wall{1} = collisionBox(1, wall_long, wall_short);
wall{2} = collisionBox(1, wall_long, wall_short);
wall{3} = collisionBox(1, wall_short, wall_long);
wall{4} = collisionBox(1, wall_short, wall_long);
wall{1}.Pose = trvec2tform([0, 0, wall_move]);
wall{2}.Pose = trvec2tform([0, 0, -wall_move]);
wall{3}.Pose = trvec2tform([0, -wall_move, 0]);
wall{4}.Pose = trvec2tform([0, wall_move, 0]);

global box_size
cur_box = collisionBox(box_size, box_size, 7);
tar_box = collisionBox(box_size, box_size, 7);
cur_box.Pose = trvec2tform(start_point(1:3).');
tar_box.Pose = trvec2tform(dest_point(1:3).');

figure;
hold on; axis equal; view(3); grid on; hold on;
axis([-space_size/2, space_size/2, -space_size/2, space_size/2, -space_size/2, space_size/2]);

for i = 1 : 4
	[~, g_wall] = show(wall{i});
	g_wall.FaceColor = 'r';
end
[~, g_cur] = show(cur_box);
[~, g_tar] = show(tar_box);
g_cur.FaceColor = 'b';
g_tar.FaceColor = 'g';

function rst = randBetween(min, max)
	rst = (max - min) * rand + min;
end

% generate random point
function point = generateRandomPoint(min_pos, max_pos)
	point = [
		randBetween(min_pos, max_pos),...
		randBetween(min_pos, max_pos),...
		randBetween(min_pos, max_pos),...
		randBetween(-pi*3, pi*3),...
		randBetween(-pi*3, pi*3),...
		randBetween(-pi*3, pi*3)].';
end

function pose = calPose(q)
	q = q.';
	pose = trvec2tform(q(1:3)) * axang2tform([0, 0, 1, q(4)/3]) * axang2tform([0, 1, 0, q(5)/3]) * axang2tform([1, 0, 0, q(6)/3]);
end

function rst = isCollision(wall, q)
	global box_size;
	cur = collisionBox(box_size, box_size, box_size*2);
	cur.Pose = calPose(q);
	rst = false;
	for i = 1 : 4
		if checkCollision(wall{i}, cur)
			rst = true;
			return;
		end
	end
end

function rst = isConnectable(wall, start, goal)
	step = 0.1;
	dir = (goal - start) / norm(goal - start);
	cur = start;
	rst = true;
	while norm(cur - goal) > step
		cur = cur + dir * step;
		if isCollision(wall, cur)
			rst = false;
			return;
		end
	end
end

count = 1;
tree = KDTree;
tree = tree.insert(start_point, count);
nodes = [start_point; -1];
to_target = true;
while true
	if to_target
		rand_point = dest_point;
		to_target = false;
	else
		rand_point = generateRandomPoint(min_pos, max_pos);
	end
	[nearest_node, ~] = tree.closestPoint(rand_point);
	nearest = nearest_node.point;
	dir = (rand_point - nearest) / norm(rand_point - nearest);
	new_point = nearest + dir * step;
	if isConnectable(wall, nearest, new_point)
		count = count + 1;
		to_target = true;
		tree = tree.insert(new_point, count);
		nodes = [nodes, [new_point; nearest_node.index]];
		scatter3(new_point(1), new_point(2), new_point(3), 'k', 'filled');
		drawnow;
		if norm(new_point - dest_point) < step
			break;
		end
	end
end

path = [];
[cur, ~] = tree.closestPoint(dest_point);
node = cur.index;

while true
	if node == -1
		break;
	end
	path = [nodes(1:6, node), path];
	node = nodes(7, node);
end

for i = 1 : size(path, 2)
	cur = path(:, i);
	cur_box.Pose = calPose(cur);
	pause(0.1);
	show(cur_box);
	drawnow;
end
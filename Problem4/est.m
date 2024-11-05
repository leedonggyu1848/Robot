clc; clear all; close all; % reset env

space_size = 20;
hole_size = 6;
box_size = 3;
start_point = [-5, 5, 5, 0, 0, 0].';
dest_point = [5, -5, -5, 0, 0, 0].';
max_pos = 5;
min_pos = -5;
step = 1;

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

count_dest = 1;
count_start = 1;
tree_dest = KDTree;
tree_start = KDTree;
tree_dest = tree_dest.insert(dest_point, count_dest);
tree_start = tree_start.insert(start_point, count_start);
nodes_start = [start_point; -1];
nodes_dest = [dest_point; -1];
while true
	rand_point = generateRandomPoint(min_pos, max_pos);
	[nearest_node_start, ~] = tree_start.closestPoint(rand_point);
	[nearest_node_dest, ~] = tree_dest.closestPoint(rand_point);
	nearest_start = nearest_node_start.point;
	nearest_dest = nearest_node_dest.point;
	dir_start = (rand_point - nearest_start) / norm(rand_point - nearest_start);
	dir_dest = (rand_point - nearest_dest) / norm(rand_point - nearest_dest);
	new_point_start = nearest_start + dir_start * step;
	new_point_dest = nearest_dest + dir_dest * step;
	if isConnectable(wall, nearest_start, new_point_start)
		count_start = count_start + 1;
		tree_start = tree_start.insert(new_point_start, count_start);
		nodes_start = [nodes_start, [new_point_start; nearest_node_start.index]];
		scatter3(new_point_start(1), new_point_start(2), new_point_start(3), 'y', 'filled');
		drawnow;
		if norm(new_point_start - dest_point) < step
			break;
		end
		[p, ~] = tree_dest.closestPoint(new_point_start);
		if norm(p.point - new_point_start) < step
			break;
		end
	end

	if isConnectable(wall, nearest_dest, new_point_dest)
		count_dest = count_dest + 1;
		tree_dest = tree_dest.insert(new_point_dest, count_dest);
		nodes_dest = [nodes_dest, [new_point_dest; nearest_node_dest.index]];
		scatter3(new_point_dest(1), new_point_dest(2), new_point_dest(3), 'k', 'filled');
		drawnow;
		if norm(new_point_dest - start_point) < step
			break;
		end
		[p, ~] = tree_start.closestPoint(new_point_dest);
		if norm(p.point - new_point_dest) < step
			break;
		end
	end
end

path_start = [];
[cur_start, ~] = tree_start.closestPoint(p.point);
node = cur_start.index;

while true
	if node == -1
		break;
	end
	path_start = [nodes_start(1:6, node), path_start];
	node = nodes_start(7, node);
end

path_dest = [];
[cur_dest, ~] = tree_dest.closestPoint(p.point);
node = cur_dest.index;

while true
	if node == -1
		break;
	end
	path_dest = [path_dest, nodes_dest(1:6, node)];
	node = nodes_dest(7, node);
end

for i = 1 : size(path_start, 2)
	cur = path_start(:, i);
	cur_box.Pose = calPose(cur);
	pause(0.1);
	show(cur_box);
	drawnow;
end

for i = 1 : size(path_dest, 2)
	cur = path_dest(:, i);
	tar_box.Pose = calPose(cur);
	pause(0.1);
	show(tar_box);
	drawnow;
end
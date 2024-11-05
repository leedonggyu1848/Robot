classdef Obstacle
	properties
		obstacles
		obstacles_size
		obstacle_length
		g_obstacles
		range_x
		range_y
	end

	methods
		function obj = Obstacle(obstacles_pos, obstacle_length, range_x, range_y)
			% obstacles_pos: [[x1, y1].'; [x2, y2].'; ...]
			obj.obstacles = obstacles_pos;
			obj.obstacle_length = obstacle_length;
			obj.g_obstacles = gobjects(1, Utils.getNumVec(obstacles_pos));
			obj.obstacles_size = Utils.getNumVec(obstacles_pos);
			obj.range_x = range_x;
			obj.range_y = range_y;
			for i = 1 : obj.obstacles_size
				obstacle = Utils.getNthVec(obstacles_pos, i);
				circle = Circle.getCoordinates(obstacle, obj.obstacle_length);
				obj.g_obstacles(i) = fill(circle(1, :), circle(2, :), 'r');
			end
		end

		function rst = isOutsideRange(obj, point)
			if obj.range_x(1) <= point(1) && point(1) <= obj.range_x(2)...
				&& obj.range_y(1) <= point(2) && point(2) <= obj.range_y(2)
				rst = false;
			else
				rst = true;
			end
		end

		function rst = isCollisionLine(obj, start, goal)
			if obj.isOutsideRange(start) || obj.isOutsideRange(goal)
				rst = true;
				return
			end
			rst = false;
			for i = 1 : obj.obstacles_size
				obstacle = Utils.getNthVec(obj.obstacles, i);
				if Circle.isCollisionLine(start, goal, obstacle, obj.obstacle_length)
					rst = true;
					return
				end
			end
		end

		function rst = isCollisionPoints(obj, points)
			rst = false;
			for i = 1 : Utils.getNumVec(points) - 1
				start = Utils.getNthVec(points, i);
				goal = Utils.getNthVec(points, i+1);
				if obj.isOutsideRange(start) || obj.isOutsideRange(goal)
					rst = true;
					return
				end
				if obj.isCollisionLine(start, goal)
					rst = true;
					return
				end
				if obj.isCollisionLine(start, goal)
					rst = true;
					return
				end
			end
		end

		function rst = isCollisionPoint(obj, point)
			if obj.isOutsideRange(point)
				rst = true;
				return
			end
			rst = false;
			for i = 1 : obj.obstacles_size
				obstacle = Utils.getNthVec(obj.obstacles, i);
				if Circle.isCollisionPoint(point, obstacle, obj.obstacle_length)
					rst = true;
					return
				end
			end
		end
	end
end
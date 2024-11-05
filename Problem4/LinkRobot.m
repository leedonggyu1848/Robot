classdef LinkRobot
	properties
		g_dots;
		g_links;
		dots;
		dot_size;
		a;
		link_length;
	end

	methods(Static)
		function h = transX(d)
			h = [
				1, 0, d;
				0, 1, 0;
				0, 0, 1
			];
		end

		function h = rotZ(a)
			h = [
				cos(a), -sin(a), 0;
				sin(a), cos(a), 0;
				0, 0, 1
			];
		end

		function h = calTranslation(theta, d)
			h = LinkRobot.rotZ(theta) * LinkRobot.transX(d);
		end
	end

	methods
		function obj = LinkRobot(start, config, link_length)
			% start: [x, y].'
			% config: [th1, th2, th3]

			obj.link_length = link_length;
			obj.dot_size = size(config, 2) + 1;
			obj.dots = [];
			obj.a = {};

			obj.a{1} = LinkRobot.calTranslation(0, start(1)) ...
			  * LinkRobot.calTranslation(pi/2, start(2)) ...
			  * LinkRobot.calTranslation(pi, 0);
			obj = obj.update(config);

			obj.g_dots = gobjects(1, obj.dot_size);
			for i = 1 : obj.dot_size
				dot = Utils.getNthVec(obj.dots, i);
				obj.g_dots(i) = scatter(dot(1), dot(2), 'b', 'filled');
			end
			obj.g_links = gobjects(1, obj.dot_size - 1);
			for i = 1 : obj.dot_size - 1
				link = [ Utils.getNthVec(obj.dots, i), Utils.getNthVec(obj.dots, i+1) ];
				obj.g_links(i) = line(Utils.getNthDim(link, 1), Utils.getNthDim(link, 2), 'Color', 'k');
			end
		end

		function obj = update(obj, config)
			for i = 1 : obj.dot_size - 1
				obj.a{i+1} = obj.a{i} * LinkRobot.calTranslation(config(i), obj.link_length);
			end
			idots = [];
			base = [0, 0, 1].';
			for i = 1 : obj.dot_size
				idots = [
					idots...
					obj.a{i} * base
				];
			end
			obj.dots = idots(1:2, 1:obj.dot_size);
		end

		function dots = calPoints(obj, config)
			obj = obj.update(config);
			dots = obj.dots;
		end

		function draw(obj)
			for i = 1 : obj.dot_size
				dot = Utils.getNthVec(obj.dots, i);
				set(obj.g_dots(i), 'XData', dot(1), 'YData', dot(2));
			end
			for i = 1 : obj.dot_size - 1
				link = [ Utils.getNthVec(obj.dots, i), Utils.getNthVec(obj.dots, i+1) ];
				set(obj.g_links(i), 'XData', Utils.getNthDim(link, 1), 'YData', Utils.getNthDim(link, 2));
			end
		end

	end
end
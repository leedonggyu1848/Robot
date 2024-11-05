classdef Circle
	methods(Static)
		function rst = getCoordinates(center, d)
			% center를 중심으로 r 반지름의 원의 좌표를 반환
			% center: [x, y]
			% d: scalar 지름

			r = d / 2; % 반지름 계산
			theta = linspace(0, 2*pi, 100); % 각도 범위 설정
			x = r * cos(theta) + center(1); % x 좌표 계산
			y = r * sin(theta) + center(2); % y 좌표 계산
			rst = [x; y];
		end

		function rst = isOutside(target, center, d)
			% target이 center를 중심으로 d 지름의 원 밖에 있는지 확인
			% target: [x, y]
			% center: [x, y]
			% d: scalar 지름

			r = d / 2; % 반지름 계산
			if norm(target - center) > r
				rst = true;
			else
				rst = false;
			end
		end

		function rst = isCollisionPoint(target, center, d)
			rst = ~Circle.isOutside(target, center, d);
		end

		function rst = isCollisionLine(start, goal, center, d)
			% start에서 end로 이동하는 선분이 obstacles와 충돌하는지 확인
			% start: [x, y]
			% end: [x, y]
			% obstacles: [[x1, y1].'; [x2, y2].'; ...]
			% d: scalar 지름

			if d >= Utils.calDistanceLine(start, goal, center)
				rst = true;
			else
				rst = false;
			end
		end
	end
end
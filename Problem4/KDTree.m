classdef KDTree
    properties
        point % 저장된 데이터 포인트
        left % 왼쪽 서브트리
        right % 오른쪽 서브트리
        axis % 분할 축
        index % 인덱스
    end

    methods
        function obj = KDTree(all_points, depth, points)
            if nargin < 2
                depth = 0;
            end
            if nargin < 3
                points = [all_points; (1:Utils.getNumVec(all_points))];
            end
            if isempty(points)
                obj.point = [];
                obj.left = [];
                obj.right = [];
                obj.axis = [];
                obj.index = [];
            else
                [dim, num_points] = size(points);
                dim = dim - 1;
                % 분할 축 선택
                axis = mod(depth, dim) + 1;
                % 정렬하여 중간값을 선택
                sorted_points = sortrows(points.', axis).';
                median_index = floor(num_points / 2) + 1;
                obj.point = sorted_points(1:dim, median_index);
                obj.axis = axis;
                obj.index = sorted_points(dim+1, median_index);
                obj.left = KDTree(all_points, depth+1, sorted_points(:, 1:median_index-1));
                obj.right = KDTree(all_points, depth+1, sorted_points(:, median_index+1:end));
            end
        end

        function rst = findWithinDistance(obj, distance, target)
            % 거리 내에 있는 데이터 포인트를 찾음
            % obj: KDTree 객체
            % distance: scalar
            % target: [x, y]

            rst = [];
            if isempty(obj.point)
                return;
            end
            if norm(obj.point - target) <= distance
                rst = [rst, obj];
            end

            if target(obj.axis) - distance <= obj.point(obj.axis)
                rst = [rst, obj.left.findWithinDistance(distance, target)];
            end
            if target(obj.axis) + distance > obj.point(obj.axis)
                rst = [rst, obj.right.findWithinDistance(distance, target)];
            end
        end
    end
end

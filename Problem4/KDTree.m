classdef KDTree
    properties
        point % 저장된 데이터 포인트
        left % 왼쪽 서브트리
        right % 오른쪽 서브트리
        axis % 분할 축
        index % 인덱스
        parent % 부모 노드
    end

    methods
        function obj = KDTree(points, indexs, depth, parent)
            if nargin < 3
                depth = 0;
            end
            if nargin ~= 0 && depth == 0
                points = [points; indexs];
            end
            if nargin < 4
                parent = [];
            end

            if nargin < 1 || isempty(points)
                obj.point = [];
                obj.left = [];
                obj.right = [];
                obj.axis = [];
                obj.index = [];
                obj.parent = [];
            else
                [dim, num_points] = size(points);
                dim = dim - 1;
                % 분할 축 선택
                axis = mod(depth, dim) + 1;
                % 정렬하여 중간값을 선택
                points = sortrows(points.', axis).';
                median_index = floor(num_points / 2) + 1;
                obj.point = points(1:dim, median_index);
                obj.axis = axis;
                obj.parent = parent;
                obj.index = points(dim+1, median_index);
                obj.left = KDTree(points(:, 1:median_index-1), indexs, depth+1, obj);
                obj.right = KDTree(points(:, median_index+1:end), indexs, depth+1, obj);
            end
        end

        function obj = insert(obj, point, index, depth)
            if nargin < 4
                depth = 0;
            end
            if isempty(obj.point)
                obj.point = point;
                obj.index = index;
                obj.axis = mod(depth, length(point)) + 1;
                obj.left = [];
                obj.right = [];
                return;
            end
            if point(obj.axis) < obj.point(obj.axis)
                if isempty(obj.left)
                    obj.left = KDTree();
                    obj.left.parent = obj;
                end
                obj.left = obj.left.insert(point, index, depth+1);
            else
                if isempty(obj.right)
                    obj.right = KDTree();
                    obj.right.parent = obj;
                end
                obj.right = obj.right.insert(point, index, depth+1);
            end
        end

        function [closest, closestDist] = closestPoint(obj, target, closest, closestDist)
            % 가장 가까운 데이터 포인트를 찾음
            % obj: KDTree 객체
            % target: [x, y]와 같은 형태의 타겟 좌표
            % closest: 현재까지 가장 가까운 점
            % closestDist: 현재까지 가장 가까운 점까지의 거리

            if isempty(obj) || isempty(obj.point)
                return;
            end
            currentDist = norm(obj.point - target);
            if nargin < 3 || currentDist < closestDist
                closest = obj;
                closestDist = currentDist;
            end

            if target(obj.axis) < obj.point(obj.axis)
                primary = obj.left;
                secondary = obj.right;
            else
                primary = obj.right;
                secondary = obj.left;
            end

            if ~isempty(primary)
                [closest, closestDist] = primary.closestPoint(target, closest, closestDist);
            end
            if ~isempty(secondary) && abs(target(obj.axis) - obj.point(obj.axis)) < closestDist
                [closest, closestDist] = secondary.closestPoint(target, closest, closestDist);
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

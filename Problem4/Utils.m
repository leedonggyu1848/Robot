classdef Utils
    methods(Static)
        function rst = getNumVec(matrix)
            rst = size(matrix, 2);
        end

        function rst = getNemdim(matrix)
            rst = size(matrix, 1);
        end

        function rst = getNthVec(matrix, n)
            rst = matrix(:, n);
        end

        function rst = getNthDim(matrix, n)
            rst = matrix(n, :);
        end

        function rst = generatePos(start, goal, spacing)
            % start에서 goal로 이동하는 선분을 spacing 간격으로 나누어 점을 생성
            % start: [x, y]
            % goal: [x, y]
            % spacing: scalar

            distance = norm(goal - start);
            numPoints = floor(distance / spacing) + 1;
            x_values = linspace(start(1), goal(1), numPoints);
            y_values = linspace(start(2), goal(2), numPoints);
            rst = [x_values; y_values];
        end

        function rst = calDistance(start, goal)
            % start에서 goal까지의 거리를 계산
            % start: [x, y]
            % goal: [x, y]

            rst = norm(goal - start);
        end

        function rst = calDirection(start, goal)
            % start에서 goal까지의 방향을 계산
            % start: [x, y]
            % goal: [x, y]

            rst = (goal - start) / norm(goal - start);
        end

        function rst = calDistanceLine(start, goal, point)
            % start에서 goal로 이동하는 선분과 point 사이의 거리를 계산
            % start: [x, y]
            % goal: [x, y]
            % point: [x, y]
            A = start;
            B = goal;
            P = point;
            % 벡터 AP와 AB 계산
            AP = P - A;
            AB = B - A;

            % 내적과 벡터 AB의 크기 제곱
            dotProduct = dot(AP, AB);
            AB_squared = dot(AB, AB);

            % 투영 비율 t 계산
            t = dotProduct / AB_squared;

            % 점 P가 선분 AB의 범위 내에 있는지 확인
            if t < 0
                % 점 A가 더 가까움
                rst = norm(P - A);
            elseif t > 1
                % 점 B가 더 가까움
                rst = norm(P - B);
            else
                % 수직 거리 계산
                projection = A + t * AB;
                rst = norm(P - projection);
            end
        end
    end
end

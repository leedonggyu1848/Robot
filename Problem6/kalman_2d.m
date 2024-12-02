clc; clear; close all;

global pos_size;

% 설정 값
pos_size = [15, 15]; % x, y
doors_pos = [1, 1; 6, 6; 10, 10].';
actions = [1, 2, 1, 2, 1, 2, 1, 2, 1, 2, 1, 2, 1, 2, 1, 2, 1, 2, 1, 2, 1, 2, 1, 2, 1, 2, 1, 2, 1, 2];


% 예측 단계
function [next_x, next_P] = prediction_step(x, P, G, F, V, u)
    next_x = F*x + G*u;
    next_P = F*P*F' + V;
end

% 보정 단계
function [next_x, next_P] = correction_step(x, P, z, H, W)
    S = H*P*H' + W;
    R = P*H'/S;
    next_x = x + R*(z - H*x);
    next_P = (eye(size(P)) - R*H)*P;
end

% helper function
function rst = cvt_action(action)
	if action == 0
		rst = [-1; 0];
	elseif action == 1
		rst = [1; 0];
	elseif action == 2
		rst = [0; 1];
	elseif action == 3
		rst = [0; -1];
	end
end

function rst = gausian(pos_size, mu, sigma)
	mu = mu.';
	[x, y] = meshgrid(0:pos_size(1), 0:pos_size(2));
	pdf = mvnpdf([x(:), y(:)], mu, sigma);
	rst = reshape(pdf, pos_size + 1);
end


% 시각화 함수
function plot_figure(doors_pos, true_pos, belief)
	global pos_size;

	figure(1); clf;
	% 첫 번째 플롯 (Position)
	subplot(1, 2, 1);
	axis equal; hold on; grid on;
	plot(doors_pos(1, :), doors_pos(2, :), 'rs', 'MarkerSize', 12, 'MarkerFaceColor', 'r');
	plot(true_pos(1), true_pos(2), 'bd', 'MarkerSize', 12, 'MarkerFaceColor', 'b');
	title('real world');
	xlim([0, pos_size(1)]);
	ylim([0, pos_size(2)]);

	% 세 번째 플롯 (Belief)
	subplot(2, 2, 2);
	axis equal; hold on; grid on;
	contour(0:pos_size(1), 0:pos_size(2), belief);
	xlim([0, pos_size(1)]);
	ylim([0, pos_size(2)]);
	title('Belief');

	subplot(2, 2, 4);
	axis equal; grid on;
	surf(0:pos_size(1), 0:pos_size(2), belief);
	xlim([0, pos_size(1)]);
	ylim([0, pos_size(2)]);
	zlim([0, 0.5]);
	title('Belief surf');

	drawnow;
end


% 초기값 설정
x = [0; 0]; % 초기 위치 [x_0; y_0]
P = 0.01 * eye(2); % 초기 분산 (2x2 공분산 행렬)

% 초기 분포
belief = gausian(pos_size, x, P);
true_pos = [0; 0];
plot_figure(doors_pos, true_pos, belief);

% 칼만 필터 설정
G = eye(2);
F = eye(2);
V = 0.5 * eye(2);
H = eye(2);
W = 0.5 * eye(2);
u = [0; 0];

% 시뮬레이션
for i = 1:size(actions, 2)
	action = actions(i);
	u = cvt_action(action);
	true_pos = true_pos + u;
    [x, P] = prediction_step(x, P, G, F, V, u);

    % 센싱된 문 위치가 있는지 확인
	sensored = sum(sum(doors_pos == true_pos) == 2) > 0;
    if sensored
        [x, P] = correction_step(x, P, true_pos, H, W);
    end

	belief = gausian(pos_size, x, P);
    plot_figure(doors_pos, true_pos, belief);
    pause(0.05);
end

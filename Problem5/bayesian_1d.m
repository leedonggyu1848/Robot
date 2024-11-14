clc; clear; close all;

global epsilon;
global step;

% 설정 값
pos_size = 100;
doors_pos = [10, 30, 60];
epsilon = 0.00001;
step = 1;

% 기본 값 정의
doors_size = size(doors_pos, 2);

% measuremetn model 정의 -> p(z|x)
measurement_model = zeros(1, pos_size);
for i = 1:doors_size
	measurement_model = measurement_model + normpdf(1:pos_size, doors_pos(i), 1);
end
measurement_model = measurement_model / sum(measurement_model);

% 다음 belief 계산
function rst = next_belief(prev_belief, measurement_model, sensored)
	global epsilon;
	global step;

	position_size = size(prev_belief, 2);
	% step: 로봇이 한번에 움직이는 거리, 1: index가 1부터 시작하여 +1
	move_noise = normpdf(1:position_size, floor(position_size/2)+1+step, 0.5);

	if sensored == 1
		z_given_x = measurement_model;
	else
		z_given_x = 1 - measurement_model;
	end

	rst = conv(move_noise, prev_belief, 'same');
	rst = rst + epsilon;
	pause
	rst = rst.*z_given_x;
	rst = rst / sum(rst);
end

function plot_figure(doors_pos, true_pos, z_given_x, belief)
	% 시각화
	figure(1); clf;
	pos_size = size(belief, 2);

	% 첫 번째 서브플롯: 실제 환경
	subplot(3, 1, 1);
	hold on;
	plot(doors_pos, ones(size(doors_pos)), 'rs', 'MarkerSize', 12, 'MarkerFaceColor', 'r'); % 문 위치 표시
	plot(true_pos, 0, 'bd', 'MarkerSize', 12, 'MarkerFaceColor', 'b'); % 실제 로봇 위치 표시
	xlim([0 pos_size]);
	ylim([0 2]);
	title('Real World');
	hold off;

	% 두 번째 서브플롯: 센서 모델 p(z|x)
	subplot(3, 1, 2);
	plot(z_given_x, 'r', 'LineWidth', 2);
	xlim([0 pos_size]);
	ylim([0 0.5]);
	title('p(z|x)');

	% 세 번째 서브플롯: 확률 분포 bel(x)
	subplot(3, 1, 3);
	plot(belief, 'b', 'LineWidth', 2);
	xlim([0 pos_size]);
	ylim([0 1]);
	title('bel(x)');

	drawnow;
end

belief = ones(1, pos_size) * epsilon;
true_pos = 0;
plot_figure(doors_pos, true_pos, measurement_model, belief);
for i = 1:pos_size
	true_pos = true_pos + step;
	sensored = sum(doors_pos == true_pos) > 0;
	belief = next_belief(belief, measurement_model, sensored);
	plot_figure(doors_pos, true_pos, measurement_model, belief);
	pause(0.01);
end
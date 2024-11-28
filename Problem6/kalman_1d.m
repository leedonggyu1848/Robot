clc; clear; close all;

global step;

% 설정 값
pos_size = 100;
doors_pos = [10, 30, 60, 80];
step = 1;

% initial value
x = 0; % 위치 x_0
P = 0.01; % 위치의 분산 P_0

function [next_x, next_P] = prediction_step(x, P, G, F, V, u)
	next_x = F*x + G*u;
	next_P = F*P*F' + V;
end

% prediction에서 계산한 next_x, next_P를 이용하여 update step을 수행
function [next_x, next_P] = correction_step(x, P, z, H, W)
	S = H*P*H' + W;
	R = P*H'/S;
	next_x = x + R*(z - H*x);
	next_P = (eye(size(P)) - R*H)*P;
end

function plot_figure(doors_pos, true_pos, belief)
	% 시각화
	figure(1); clf;
	pos_size = size(belief, 2);

	% 첫 번째 서브플롯: 실제 환경
	subplot(2, 1, 1);
	hold on;
	plot(doors_pos, ones(size(doors_pos)), 'rs', 'MarkerSize', 12, 'MarkerFaceColor', 'r'); % 문 위치 표시
	plot(true_pos, 0, 'bd', 'MarkerSize', 12, 'MarkerFaceColor', 'b'); % 실제 로봇 위치 표시
	xlim([0 pos_size]);
	ylim([0 2]);
	title('Real World');
	hold off;

	% 세 번째 서브플롯: 확률 분포 bel(x)
	subplot(2, 1, 2);
	plot(0:pos_size-1, belief, 'b', 'LineWidth', 2);
	xlim([0 pos_size]);
	ylim([0 1]);
	title('bel');
	drawnow;
end

belief = normpdf(0:pos_size-1, x, sqrt(P));
true_pos = 0;
plot_figure(doors_pos, true_pos, belief);

G = 1;        % 제어 입력 계수
F = 1;        % 상태 전이 계수
V = 0.5;     % 프로세스 잡음 분산
H = 1;        % 관측 모델 계수
W = 0.5;      % 관측 잡음 분산
u = 1;        % 제어 입력 (한 칸씩 이동)

for i = 1:pos_size
	true_pos = true_pos + step;
	[x, P] = prediction_step(x, P, G, F, V, u);
	sensored = sum(doors_pos == true_pos) > 0;
	if sensored
		[x, P] = correction_step(x, P, true_pos, H, W);
	end
	belief = normpdf(0:pos_size-1, x, sqrt(P));
	plot_figure(doors_pos, true_pos, belief);
	pause(0.1);
end
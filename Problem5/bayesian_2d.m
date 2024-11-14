clc; clear; close all;

global epsilon;
global step;
global pos_size;

% 설정 값
pos_size = [10, 10]; % x, y
doors_pos = [1, 1; 4, 4; 8, 8].';
epsilon = 0.00001;
step = 1;
% 0: left, 1: right, 2: up, 3: down
actions = [1, 2, 1, 2, 1, 2, 1, 2, 1, 2, 1, 2, 1, 2, 1, 2, 1, 2, 1, 2];

% 기본 값 정의
doors_size = size(doors_pos, 2);

function rst = gausian(pos_size, mu, sigma)
	[x, y] = meshgrid(1:1:pos_size(1), 1:1:pos_size(2));
	pdf = mvnpdf([x(:), y(:)], mu, sigma);
	rst = reshape(pdf, pos_size);
end

function rst = cvt_action(action)
	if action == 0
		rst = [-1, 0];
	elseif action == 1
		rst = [1, 0];
	elseif action == 2
		rst = [0, 1];
	elseif action == 3
		rst = [0, -1];
	end
end

% measuremetn model 정의 -> p(z|x)
measurement_model = zeros(pos_size);
sigma = [1, 0.5; 0.5, 1];
for i = 1:doors_size
	door_pos = doors_pos(:, i);
	pdf = gausian(pos_size, door_pos.', sigma);
	measurement_model = measurement_model + pdf;
end
measurement_model = measurement_model / sum(measurement_model(:));

function rst = next_belief(prev_belief, measurement_model, sensored, action)
	global epsilon;
	global step;

	position_size = size(prev_belief);
	if sensored == 1
		z_given_x = measurement_model;
	else
		z_given_x = 1 - measurement_model;
	end

	dir = cvt_action(action);
	sigma = [0.5, 0; 0, 0.5];

	move_noise = gausian(position_size, floor([10, 10]/2)+1 +dir * step, sigma);
	rst = conv2(move_noise, prev_belief, 'same');
	rst = rst + epsilon;
	rst = rst.*z_given_x;
	rst = rst / sum(rst(:));
end

function plot_figure(doors_pos, true_pos, z_given_x, belief)
	global pos_size;

	figure(1); clf;
	% 첫 번째 플롯 (Position)
	subplot(1, 3, 1);
	axis equal; hold on; grid on;
	plot(doors_pos(1, :), doors_pos(2, :), 'rs', 'MarkerSize', 12, 'MarkerFaceColor', 'r');
	plot(true_pos(1), true_pos(2), 'bd', 'MarkerSize', 12, 'MarkerFaceColor', 'b');
	title('real world');
	xlim([0, pos_size(1)]);
	ylim([0, pos_size(2)]);

	% 두 번째 플롯 (P(z|s))
	subplot(1, 3, 2);
	axis equal; hold on; grid on;
	contour([0:pos_size(1)-1], [0:pos_size(2)-1], z_given_x);
	xlim([0, pos_size(1)]);
	ylim([0, pos_size(2)]);
	title('P(z|s)');

	% 세 번째 플롯 (Belief)
	subplot(2, 3, 3);
	axis equal; hold on; grid on;
	contour([0:pos_size(1)-1], [0:pos_size(2)-1], belief);
	xlim([0, pos_size(1)]);
	ylim([0, pos_size(2)]);
	title('Belief');

	subplot(2, 3, 6);
	axis equal; grid on;
	surf([0:pos_size(1)-1], [0:pos_size(2)-1], belief);
	xlim([0, pos_size(1)]);
	ylim([0, pos_size(2)]);
	zlim([0, 0.5]);
	title('Belief surf');

end

belief = ones(pos_size) * epsilon;
true_pos = [0, 0].';
plot_figure(doors_pos, true_pos, measurement_model, belief);
for i = 1:size(actions, 2)
	action = actions(i);
	dir = cvt_action(action);
	true_pos = true_pos + dir.';
	sensored = sum(sum(doors_pos == true_pos) == 2) > 0;
	belief = next_belief(belief, measurement_model, sensored, action);
	plot_figure(doors_pos, true_pos, measurement_model, belief);
	pause(0.5);
end
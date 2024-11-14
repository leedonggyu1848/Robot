% 매개변수 설정
mu1 = 20;          % 첫 번째 가우시안 평균
mu2 = 40;          % 두 번째 가우시안 평균
mu3 = 70;          % 세 번째 가우시안 평균

sigma1 = 1;        % 첫 번째 가우시안 표준편차
sigma2 = 1;        % 두 번째 가우시안 표준편차
sigma3 = 1;        % 세 번째 가우시안 표준편차

w1 = 1/3;          % 첫 번째 가우시안의 가중치
w2 = 1/3;          % 두 번째 가우시안의 가중치
w3 = 1/3;          % 세 번째 가우시안의 가중치

% x 값의 범위 설정
x = linspace(0, 100, 1000);

% 각 가우시안 함수 정의
f1 = (1 / (sigma1 * sqrt(2 * pi))) * exp(- (x - mu1).^2 / (2 * sigma1^2));
f2 = (1 / (sigma2 * sqrt(2 * pi))) * exp(- (x - mu2).^2 / (2 * sigma2^2));
f3 = (1 / (sigma3 * sqrt(2 * pi))) * exp(- (x - mu3).^2 / (2 * sigma3^2));

% 가우시안 분포 합성
f_combined = w1 * f1 + w2 * f2 + w3 * f3;

% 그래프 그리기
figure;
plot(x, f_combined, 'LineWidth', 2);
hold on;
plot(x, f1, '--', 'LineWidth', 1);
plot(x, f2, '--', 'LineWidth', 1);
plot(x, f3, '--', 'LineWidth', 1);
legend('Combined', 'Gaussian 1 (Mean 20)', 'Gaussian 2 (Mean 40)', 'Gaussian 3 (Mean 70)');
title('Combination of Three Gaussian Distributions');
xlabel('x');
ylabel('Probability Density');
grid on;

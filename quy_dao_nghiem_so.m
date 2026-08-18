% =========================================================================
% FILE CẤU HÌNH THÔNG SỐ IMC-PID CHO HỆ THỐNG 2 BÌNH MỨC NƯỚC MỚI
% Điểm làm việc mới: h1_bar = 3.0m, h2_bar = 1.5m
% =========================================================================

clear all;
clc;

% 1. Thông số vật lý cơ sở của hệ thống bồn chứa
A = 1;              % Tiết diện bình chứa (m2)
Cv = 0.0025;        % Hệ số cỡ van
g = 9.81;           % Gia tốc trọng trường (m/s2)
alpha = (Cv / A) * sqrt(g); 

% 2. Điểm làm việc tĩnh mới (Cột áp bình 1 cao hơn bình 2)
h1_bar = 3.0;       % Mức nước tĩnh bình 1 (m)
h2_bar = 1.5;       % Mức nước tĩnh bình 2 (m)
delta_h = h1_bar - h2_bar; % Độ chênh lệch cột áp xuôi

% 3. Tính toán các hệ số khuếch đại tuyến tính hóa mới
K3 = alpha * sqrt(delta_h);
K4 = alpha * sqrt(h2_bar);

% Tại điểm làm việc này K3 = K4 nên độ mở hai van bằng nhau ở trạng thái tĩnh
l1_bar = 0.5;       % Độ mở van 1 tại điểm cân bằng (50%)
l2_bar = 0.5;       % Độ mở van 2 tại điểm cân bằng (50%)

K1 = (alpha * l1_bar) / (2 * sqrt(delta_h));
K2 = (alpha * l2_bar) / (2 * sqrt(h2_bar));

% 4. Thiết lập Đa thức mẫu số chung cho hệ thống: D(s) = s^2 + a1*s + a2
a1 = 2*K1 + K2;
a2 = K1*K2;
den = [1, a1, a2]; % Mẫu số nạp vào các khối Transfer Fcn

% 5. Hằng số thời gian bộ lọc mong muốn thiết kế (IMC Lambda)
lambda1 = 500;       % Vòng 1
lambda2 = 500;       % Vòng 2

% 6. Tính toán thông số bộ điều khiển IMC-PID vòng 1 (L1 -> H1)
Kc1 = -(2*K1 + K2) / (K2 * K3 * lambda1);
Ti1 = (2*K1 + K2) / (K1 * K2);
Td1 = 1 / (2*K1 + K2);
Tf1 = 1 / K2;

% 7. Tính toán thông số bộ điều khiển IMC-PID vòng 2 (L2 -> H2)
Kc2 = -(2*K1 + K2) / (K1 * K4 * lambda2);
Ti2 = Ti1;
Td2 = Td1;
Tf2 = 1 / K1;

% 8. Chuyển đổi sang hệ số dạng song song (Parallel) để điền vào Simulink
P1 = Kc1;  I1 = Kc1/Ti1;  D1 = Kc1*Td1;  N1 = 1/Tf1;
P2 = Kc2;  I2 = Kc2/Ti2;  D2 = Kc2*Td2;  N2 = 1/Tf2;

% 9. Cấu hình giá trị biên độ cho các khối biến sai lệch Step trong Simulink
% Mục tiêu thực tế: h1 từ 3m -> 3.2m  |  h2 từ 1.5m -> 1.6m
step_delta_h1 = 3.2 - h1_bar;   % = 0.2 (Điền vào Final Value của Step 1)
step_delta_h2 = 1.6 - h2_bar;   % = 0.1 (Điền vào Final Value của Step 2)

% 10. Giới hạn Saturation vật lý đầu ra cho khối PID (Δl = L - l_bar)
sat_upper_PID1 = 1 - l1_bar;    % = 0.5
sat_lower_PID1 = -l1_bar;       % = -0.5

sat_upper_PID2 = 1 - l2_bar;    % = 0.5
sat_lower_PID2 = -l2_bar;       % = -0.5

% Hiển thị thông báo kiểm tra kết quả trên Command Window
fprintf('=== THÔNG SỐ ĐÃ ĐƯỢC CẬP NHẬT THÀNH CÔNG ===\n');
fprintf('K1 = %.6f | K2 = %.6f | K3 = %.6f | K4 = %.6f\n', K1, K2, K3, K4);
fprintf('PID 1 -> P: %.4f, I: %.4f, D: %.4f, N: %.4f\n', P1, I1, D1, N1);
fprintf('PID 2 -> P: %.4f, I: %.4f, D: %.4f, N: %.4f\n', P2, I2, D2, N2);
fprintf('Biên độ Step đặt mới -> Delta H1: %.1f | Delta H2: %.1f\n', step_delta_h1, step_delta_h2);
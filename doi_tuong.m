

clear all; clc; close all;

%% BƯỚC 1: KHAI BÁO THÔNG SỐ VẬT LÝ CỦA HỆ THỐNG
A  = 1;                 % Tiết diện bình chứa (m^2)
Cv = 0.0025;            % Hệ số cỡ van (m^3 / s.kPa^1/2)
g  = 9.81;              % Gia tốc trọng trường (m/s^2)
gs = 1;                 % Tỷ trọng chất lỏng (nước)
% Lưu ý: Áp suất P = rho*g*h (Pa). Đổi sang kPa chia 1000 -> P(kPa) = 9.81*h

%% BƯỚC 2: XÁC ĐỊNH ĐIỂM LÀM VIỆC (TRẠNG THÁI XÁC LẬP)
% Theo đề bài và điều kiện cân bằng lưu lượng: F10 = F20 = F30
h20 = 1.5;              % Mức nước bình 2 theo yêu cầu (m)
h10 = 3.0;              % Mức nước bình 1 (m) (Tính từ h10 - h20 = h20)
L20 = 0.5;              % Độ mở van 2 cố định 50%
L30 = 0.5;              % Độ mở van 3 cố định 50%

% Lưu lượng đầu vào tại trạng thái cân bằng
F10 = Cv * L20 * sqrt((g * (h10 - h20)) / gs);
fprintf('--- THONG SO TAI DIEM LAM VIEC ---\n');
fprintf('Luu luong vao F10 = %g (m^3/s)\n', F10);
fprintf('Muc nuoc h10 = %g (m), h20 = %g (m)\n\n', h10, h20);

%% BƯỚC 3: TUYẾN TÍNH HÓA BẰNG KHAI TRIỂN TAYLOR
% Các hệ số đạo hàm riêng (tương ứng với k3, k4, k5, k6 trong giấy nháp)
Kh = (Cv * L20 * g) / (2 * sqrt(g * h20 / gs)); % Đạo hàm theo mức nước (dF/dh)
KL = Cv * sqrt(g * h20 / gs);                   % Đạo hàm theo độ mở van (dF/dL)

fprintf('--- HE SO TUYEN TINH HOA ---\n');
fprintf('He so k_h (Dao ham theo h) = %g\n', Kh);
fprintf('He so k_L (Dao ham theo L) = %g\n\n', KL);

%% BƯỚC 4: XÂY DỰNG MÔ HÌNH KHÔNG GIAN TRẠNG THÁI (STATE-SPACE)
% Hệ phương trình vi phân:
% d(dh1)/dt = (1/A)*[ dF1 - KL*dL2 - Kh*dh1 + Kh*dh2 ]
% d(dh2)/dt = (1/A)*[ KL*dL2 - KL*dL3 + Kh*dh1 - 2*Kh*dh2 ]

% Vector trạng thái x = [dh1; dh2], Vector điều khiển u = [dL2; dL3], Nhiễu d = [dF1]
As = [-Kh/A,   Kh/A;
       Kh/A, -2*Kh/A];

Bs = [-KL/A,      0;     % Ma trận cho ngõ vào điều khiển u (van 2, van 3)
       KL/A,  -KL/A];

Bd = [1/A;               % Ma trận cho ngõ vào nhiễu d (lưu lượng F1)
      0  ];

Cs = [1 0;               % Ma trận ngõ ra (Đo cả h1 và h2)
      0 1];

Ds = zeros(2,2);         
Dd = zeros(2,1);

%% BƯỚC 5: XUẤT MA TRẬN HÀM TRUYỀN (TRANSFER FUNCTION)
% Chuyển từ State-Space sang Transfer Function
sys_p = ss(As, Bs, Cs, Ds); 
sys_d = ss(As, Bd, Cs, Dd); 

% Dùng hàm minreal để triệt tiêu các nghiệm tử/mẫu dư thừa (nếu có), giúp hàm gọn nhất
Gp = minreal(tf(sys_p)); 
Gd = minreal(tf(sys_d)); 

fprintf('--- MA TRAN HAM TRUYEN DOI TUONG Gp(s) ---\n');
disp('Cot 1: Tac dong tu Van L2 | Cot 2: Tac dong tu Van L3');
disp('Hang 1: Len muc nuoc h1   | Hang 2: Len muc nuoc h2');
Gp;

fprintf('--- MA TRAN HAM TRUYEN NHIEU Gd(s) ---\n');
disp('Tac dong tu luu luong vao F1 len h1 va h2');
Gd;

%% BƯỚC 5.1: CHIẾT XUẤT CHI TIẾT TỬ SỐ VÀ MẪU SỐ (Dùng cho Simulink)
% Sử dụng hàm tfdata với tham số 'v' để lấy vector hệ số trực tiếp

% --- Đối với ma trận Đối tượng Gp(s) ---
[num_G11, den_G11] = tfdata(Gp(1,1), 'v'); % Van 2 -> h1
[num_G12, den_G12] = tfdata(Gp(1,2), 'v'); % Van 3 -> h1
[num_G21, den_G21] = tfdata(Gp(2,1), 'v'); % Van 2 -> h2
[num_G22, den_G22] = tfdata(Gp(2,2), 'v'); % Van 3 -> h2

% --- Đối với ma trận Nhiễu Gd(s) ---
[num_Gd1, den_Gd1] = tfdata(Gd(1,1), 'v'); % Nhiễu F1 -> h1
[num_Gd2, den_Gd2] = tfdata(Gd(2,1), 'v'); % Nhiễu F1 -> h2

%% BƯỚC 5.2: HIỂN THỊ KẾT QUẢ DẠNG SỐ (Dễ chép vào báo cáo)
fprintf('--- CHI TIET TU SO VA MAU SO CAC HAM TRUYEN ---\n');
fprintf('G11: num = [%s], den = [%s]\n', num2str(num_G11, ' %g'), num2str(den_G11, ' %g'));
fprintf('G12: num = [%s], den = [%s]\n', num2str(num_G12, ' %g'), num2str(den_G12, ' %g'));
fprintf('G21: num = [%s], den = [%s]\n', num2str(num_G21, ' %g'), num2str(den_G21, ' %g'));
fprintf('G22: num = [%s], den = [%s]\n', num2str(num_G22, ' %g'), num2str(den_G22, ' %g'));
fprintf('Gd1: num = [%s], den = [%s]\n', num2str(num_Gd1, ' %g'), num2str(den_Gd1, ' %g'));
fprintf('Gd2: num = [%s], den = [%s]\n', num2str(num_Gd2, ' %g'), num2str(den_Gd2, ' %g'));G22 = Gp(2,2);




clc; clear all; close all;

% ===== 1. Freq Response ===== %

% 주파수 범위 설정 
Freq_Range = 4.30:0.40:7.10 ; 
% Freq_Range = 8.00:1.00:12.00 ; 

% Freq_Range = [2.90, 3.00 , 3.10, 3.15 ,3.20, 3.30];
N_Freq = length(Freq_Range) ; 

% 주파수 반응 (Freq ; Gain ; Phase) 
Freq_Response = zeros(3,N_Freq) ; 

fs = 200;         

% 입/출력 Offset 고려
OUT_eBias  = zeros(1,N_Freq) ; 
IN_eBias   = zeros(1,N_Freq) ; 

% ====== 모델 함수 정의 ======
SysResp = @(x, fTime) x(1)*sin(2*pi*fTime + x(2)) + x(3);  % [진폭, 위상, 바이어스]



for idx = 1:N_Freq
    % ====== 데이터 불러오기 ======
    filename = sprintf('../../data/freq_response/Frequency_Response_%2.2fHz.out', Freq_Range(idx));
    data = load(filename);

    time  = data(:,1);
    w_in  = data(:,2);  % 입력 (Vcmd)
    w_out = data(:,4);  % 출력 (Vg or Vout)

    % ====== 초기 추정 ======
    period_samples = round(fs / Freq_Range(idx));  % 1주기 샘플 수

    [max_Vout, max_idx_out] = max(w_out(1:period_samples));
    [min_Vout, ~] = min(w_out(1:period_samples));
    [max_Vin, max_idx_in] = max(w_in(1:period_samples));

    A_out_guess = (max_Vout - min_Vout) / 2;
    td          = (max_idx_out - max_idx_in) / fs;
    phi_out_guess = -2 * pi * Freq_Range(idx) * td;
    bias_out_guess = (max_Vout + min_Vout) / 2;

    % ====== 입력 신호 적합 ======
    x_in = lsqcurvefit(SysResp, [1.0; 0.0; mean(w_in)], Freq_Range(idx)*time, w_in);
    IN_eMag  = x_in(1);
    IN_ePhs  = x_in(2);
    IN_eBias = x_in(3);

    if IN_eMag < 0
        IN_eMag = abs(IN_eMag);
        IN_ePhs = IN_ePhs + pi;
    end

    % ====== 출력 신호 적합 ======
    x_out = lsqcurvefit(SysResp, ...
        [A_out_guess; phi_out_guess; bias_out_guess], ...
        Freq_Range(idx)*time, w_out);

    OUT_eMag = x_out(1);
    OUT_ePhs = x_out(2);
    OUT_eBias = x_out(3);

    if OUT_eMag < 0
        OUT_eMag = abs(OUT_eMag);
        OUT_ePhs = OUT_ePhs + pi;
    end

    % ====== Gain / Phase 저장 ======
    gain = OUT_eMag / IN_eMag;
    phase_deg = asin(sin(OUT_ePhs - IN_ePhs)) * 180 / pi;

    Freq_Response(:, idx) = [Freq_Range(idx); gain; phase_deg];

    % ====== (옵션) 모델 적합 시각화 ======
    eVout = OUT_eMag * sin(2*pi*Freq_Range(idx)*time + OUT_ePhs) + OUT_eBias ;
    figure;
    plot(time, w_out - OUT_eBias, 'b', 'LineWidth', 1); hold on;
    plot(time, eVout - OUT_eBias, 'r', 'LineWidth', 1.2);
    xlabel('time [sec]');
    ylabel('response [V]');
    legend('true', 'model');
    title(sprintf('%2.2f [Hz] - Model Fit', Freq_Range(idx)));
    grid on;
    
end


% 데이터 확인 

figure ;
semilogx((Freq_Response(1,:)), 20*log10(Freq_Response(2,:)));
grid on;
xlabel('frequency [Hz]');
ylabel('magnitude [dB]');


figure ;
semilogx((Freq_Response(1,:)), Freq_Response(3,:));
grid on ;
xlabel('frequency [Hz]');
ylabel('phase [deg]') 


%% 

% ===== 2. Transfer Function ===== %

Freq_Model   = Freq_Response(2,:).*exp(1j*Freq_Response(3,:)*pi/180); % 주파수 응답
Omega_Model  = Freq_Response(1,:)*2*pi ; % 주파수 (rad/s)

Nnum = 0 ; % 분자의 차수
Nden = 2 ; % 분모의 차수

% 전달 함수 추정
[num, den] = invfreqs(Freq_Model, Omega_Model, Nnum, Nden);
Gm_s = tf(num,den);

% Bode Plot 
[mag,phase,wout] = bode(Gm_s); 


% Mangnitude 비교
figure; 
semilogx(wout, 20*log10(squeeze(mag))), hold on; 
semilogx(2*pi*Freq_Response(1,:), 20*log10(Freq_Response(2,:)),'rx');
title('Magnitude Comparing') ; 
xlim([0.01 1000]), ylim([-45 5]) ; 
legend({'estimated', 'measured'}); 
grid on ; 


% Phase 비교
figure; 
semilogx(wout, squeeze(phase)), hold on ; 
semilogx(2*pi*Freq_Response(1,:), Freq_Response(3,:),'rx'); 
title('Phase Comparing') ; 
xlim([0.01 1000]), ylim([-180 0]); 
legend({'estimated', 'measured'}); 
grid on ; 

figure; 
pzmap(Gm_s);
grid on ; 
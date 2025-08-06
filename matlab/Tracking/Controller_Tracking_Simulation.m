clc; clear all; close all;

%%
% ==== 0. Simulation Setting ==== %

Tstart = 0.0   ; 
Tend   = 100.0  ;
Tsim   = 0.001 ;
Ts     = 0.005 ; 


time = Tstart : Tsim : Tend ; 
time_s = Tstart : Ts : Tend ; 

% Disturbance input  
Dist_b = zeros(length(time),2);
Dist_b(:,1) = time ;

Dist_scale = 200 ;  
Dist_freq = 5 ; 

for i=1:length(time) 
    Dist_b(i,2) = Dist_scale * sin(Dist_freq * time(i)); % [deg/sec] 
end 

Angle_c = zeros(length(time),2);
Angle_c(:,1) = time ;
input_vel = 30 ; 

% Step input [deg] 
for i=1:length(time) 
    Angle_c(i,2) = input_vel ; 
end 



% [V] to [rad/sec]
Kg = 1/0.00067 ; 

%%
% === 1. Gm (모터 전달함수) === %
% 06.12 

Pm    = 31.1403       ;  
tau_m = 1 / 31.1403   ;  
Km    = 10.88 / Pm  ; 

Gm_num = [Km * Pm]  ;
Gm_den = [1 Pm] ; 
Gm =tf(Gm_num,Gm_den) ; 


%%
% === 2. Gcl_Stability === %  
Kt = 8 ; 

zeta = 0.707 ;   
wn   = 35.5 ;     

Kp = ((2 * wn * zeta / Pm) - 1) / (Km * Kg) ; 
Ki = (wn^2 /(Km * Kg * Pm)) ; 

% WN = ((((0.002226)* Km * Kg) + 1) * Pm )/(2 * zeta) ;
% 0.002226 


Gcl_num = [ Kp*Km*Pm , Ki*Km*Pm] ;  
Gcl_den = [1 , Pm * (1 + Km * Kg * Kp) ,  Ki * Km * Kg * Pm ] ;
Gcl = tf(Gcl_num , Gcl_den) ;   

bw = bandwidth(Gcl);
fprintf('Bandwidth = %.2f rad/s\n', bw);

Gc_num = [ Kp, Ki ] ; 
Gc_den = [ 1 , 0] ;
Gc     = tf(Gc_num , Gc_den) ; 

Go = Gm * Gc * Kg ;


figure();
nyquist(Go); hold on;

theta = 0 : 0.01 : 2*pi;
x = cos(theta);
y = sin(theta);
plot(x, y, 'r--', 'LineWidth', 1.5);

legend('Nyquist plot','Unit circle');
title('Nyquist Plot of Designation Go');

xlim([-1.3 1.3]); 
ylim([-1.3 1.3]);

[re, im, w] = nyquist(Go, logspace(-1, 2, 2000));  % 주파수 범위 확장
nyq = squeeze(re + 1i*im);

% 단위원과 가장 가까운 점 찾기
dist_to_unit_circle = abs(abs(nyq) - 1);
[~, idx] = min(dist_to_unit_circle);




%% ===== 4. Plot ==== %%

out = sim('Controller_Tracking_Simul_Final.slx'); 

% ==== 1. Simultaion result ==== %
figure ; 
plot(out.time, out.output, 'LineWidth', 2); 
hold on ;
plot(out.time, out.Dist, 'LineWidth', 2); 
title('Simulation Result'); 
xlabel('Time [sec]');
ylabel('\omega_g [deg/sec]'); 
legend('Disturbance','Output');
xlim([0,5]);
ylim([0,50]);


% data = load('stabilization_LPF.txt');
% time  = data(:,1); 
% V_cmd = data(:,3); 
% W_g   = data(:,5); 
% K_i   = data(:,6); 
% K_p   = data(:,7); 
% Error = data(:,8); 

figure ; 
plot(out.time, out.Signal_Ki, LineWidth=2); 
% hold on ; 
% plot(time-2,K_i, LineWidth=2); 
grid on; 
title('Signal Ki of Controller Stabilization'); 
xlabel('Time [sec]');
ylabel('Signal Ki [V]'); 
legend('simulation','result');
xlim([0 2]);

figure ; 
plot(out.time, out.Signal_Kp, LineWidth=2); 
% hold on ; 
% plot(time-2,K_p, LineWidth=2); 
grid on; 
title('Signal Kp of Controller Stabilization '); 
xlabel('Time [sec]');
ylabel('Signal Kp [V]'); 
legend('simulation','result');
xlim([0 2]);

figure ; 
plot(out.time, out.Signal_Total, LineWidth=2); 
% hold on ; 
% plot(time-2,V_cmd, LineWidth=2); 
grid on; 
title('Signal (Ki + Kp) of Controller Stabilization');
xlabel('Time [sec]');
ylabel('Vcmd  [V]'); 
legend('simulation','result');
xlim([0 2]);


figure ; 
plot(time_s, out.Signal_Vcmd, LineWidth=2); 
% hold on ; 
% plot(time-2,V_cmd, LineWidth=2); 
grid on; 
title('Vcmd of Controller Stabilization');
xlabel('Time [sec]');
ylabel('Vcmd  [V]'); 
legend('simulation','result');
xlim([0 2]);
% ylim([0.8 2.6]);


figure ; 
plot(out.time, out.output, LineWidth=2); 
% hold on ; 
% plot(time-2,W_g, LineWidth=2); 
grid on; 
title('Dg of Controller Stabilization');  
xlabel('Time [sec]'); 
ylabel('Dg [deg]'); 
legend('simulation','result');
xlim([0 2]);
ylim([0 40]);

figure ; 
plot(out.time, out.Error, LineWidth=2); 
% hold on ; 
% plot(time-2,500-W_g, LineWidth=2); 
grid on; 
title('Error'); 
xlabel('Time [sec]');
ylabel('Error [deg/sec]'); 
legend('simulation','result');
xlim([0 2]);


%% === Step Info === %% 

info = stepinfo(Gcl);
disp(info);

%% 

Gb_num = [ 1, Pm, 0 ] ; 
Gb_den = [ 1, Pm*(1+Km*Kp*Kg), Pm*Km*Kg*Ki ] ; 
Ge = tf(Gb_num , Gb_den) ; 

wb = 5;  % [rad/s]
[mag, phase] = bode(Ge, wb);  % mag: gain, phase: 위상 (deg)


% 결과 출력
fprintf('\n') ;
fprintf('\n ===== Ge ==== \n') ;
fprintf('Gain at w = 5 rad/s: %.6f\n', mag);
figure ; 
bode(Gcl);
grid on ; 

figure ;
bode(Ge);
title('Bode Plot of G_e ');
grid on ;

[mag, phase, wout] = bode(Ge, {0.1, 100});
mag = squeeze(mag);
wout = squeeze(wout);

% Normalized magnitude
mag_norm = mag / max(mag);
idx_cutoff = find(mag_norm >= 1/sqrt(2), 1, 'first'); 
cutoff_freq = wout(idx_cutoff); 
fprintf("Cut off Frequnecy of Ge ≈ %.2f [rad/sec]\n", cutoff_freq);




%%

% s = tf('s');
% 
% G1 = (0.01016*s + 0.1824) / (s^2 + 24.98*s + 272.3);
% G2 = (0.1824) / (s^2 + 24.98*s + 272.3);
% 
% % Step 응답 비교
% t = 0:0.001:2;
% [y1, t1] = step(G1, t);
% [y2, t2] = step(G2, t);
% 
% figure;
% plot(t1, y1, 'b-', 'LineWidth', 2); hold on;
% plot(t2, y2, 'r--', 'LineWidth', 2);
% legend('with Zero', 'no Zero');
% title('Step Response Comparison');
% xlabel('Time [sec]');
% ylabel('Amplitude');
% grid on;

fprintf('\n ===== Nyquist Plot ==== \n') ;
% 주파수 성분 출력
fprintf("단위원 통과 지점 주파수 ≈ %.4f rad/s (%.2f Hz)\n", w(idx), w(idx)/(2*pi));
fprintf("응답: Re = %.4f, Im = %.4f, 위상 = %.2f deg\n", ...
    real(nyq(idx)), imag(nyq(idx)), angle(nyq(idx)) * 180/pi);


fprintf('\n') ;
fprintf('\n ===== Bandwidth ==== \n') ;
fprintf('Bandwidth = %.2f rad/s\n', bw);

fprintf('\n') ; 
fprintf('\n ===== PI Constroller Coefficient ==== \n') ;
fprintf('Kp : %.4f  \n', Kp); 
fprintf('Ki : %.4f  \n', Ki); 



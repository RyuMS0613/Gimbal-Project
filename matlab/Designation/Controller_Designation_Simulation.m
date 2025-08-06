clc; clear all; close all;

% Data Set%

Data = load('../../data/designation/Designation.out');

Time         = Data(1:5*200,1) ; 
Angle_Input  = Data(1:5*200,2) ; 
Vcmd         = Data(1:5*200,3) ; 
Angle_Output = Data(1:5*200,4) ; 
Error_1      = Data(1:5*200,5) ; 
Error_2      = Data(1:5*200,6) ; 


% Simulation Set % 
Tstart = 0.0   ; 
Tend   = 50.0  ;
Tsim   = 0.001 ;
Ts     = 0.005 ; 


time   = Tstart : Tsim : Tend ; 
D_time = Tstart :  Ts  : Tend ; 

% Why? 
input_degree = 30 ; 
input_deg = zeros(length(time),2) ;
input_deg(:,1) = time ;

for i=1:length(time) 
    input_deg(i,2) =input_degree ;
end 

%% 
% === Controller Set === %
zeta = 0.707 ; 
wn   = 35.5 ; 

% === Gm === %
% 06.30. 
tau_m = 1 / 31.1403 ; 
Km    = 10.88 * tau_m ; 

Gm_num = [Km/tau_m]  ; 
Gm_den = [1 1/tau_m] ; 

Gm =tf(Gm_num,Gm_den) ;

% === Ccl === %

Kg = 1/0.00067 ; 

Kd = (tau_m * 2 * wn * zeta - 1) / (Km*Kg) ; 
Kp = (wn^2 * tau_m) / (Km*Kg) ; 

% Gcl 
Gcl_num = [ Km * Kg * Kd / tau_m , Km * Kg * Kp / tau_m] ; 
Gcl_den = [ 1 , (1+Km * Kg* Kd) / tau_m , (Kp * Km * Kg) / tau_m ] ;  
 
Gcl = tf(Gcl_num , Gcl_den) ;  

K2 = Kd ; 
K1 = Kp / Kd ; 

out = sim('Controller_Designation_Simul.slx'); 


% === Margin for Gcl_Desingation === %  

Integ_num = [ 1 ] ;  
Integ_den = [ 1,0 ] ; 
Integ     = tf(Integ_num , Integ_den) ; 


Gc_num = [ Kd, Kp ] ; 
Gc_den = [1] ;
Gc = tf(Gc_num , Gc_den) ; 


s=tf('s');

Go = Gm * Gc * Kg *Integ ; 
figure() ; 
margin(Go);  

fc = 65;                     
BW = 2*pi*fc;                
LPF = tf([BW], [1 BW]);

Go_LPF = Go*LPF ;

figure() ; 
margin(Go_LPF) ; 
title('LPF Effect');


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





%%
% Input = out.input;

figure ; 
plot(out.time, out.output,'.'); 
title('Simulation of Controller Designation');
xlabel('Time [sec]');
ylabel('\psi_g [deg]'); 
legend('Simulation');
xlim([0 2]);
ylim([0 40]);

figure();
plot(Time-0.04,Angle_Output);
hold on ; 
plot(out.time+2, out.output,'.'); 
xlim([0 4]);

figure();
plot(Time-0.04,Angle_Output);
hold on ;
plot(out.time+2, out.output,'.'); 


figure ; 
pzmap(Gcl); 
grid on ; 

figure ; 
bode(Gcl); 
grid on ; 


bw = bandwidth(Gcl);

    
%%

% s = tf('s');
% 
% % 전달함수 정의
% G1 = (8.004*s + 272.2) / (s^2 + 17.82*s + 272.2);
% G2 = 272.2 / (s^2 + 17.82*s + 272.2);
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
fprintf('\n ===== PP Constroller Coefficient ==== \n') ;
fprintf('Kp : %.4f  \n', Kp); 
fprintf('Kd : %.4f  \n', Kd); 
fprintf('K1 : %.4f  \n', K1); 
fprintf('K2 : %.4f  \n', K2); 

clc; clear all; close all;


Tstart = 0.0   ; 
Tend   = 100.0  ;
Tsim   = 0.001 ;
Ts     = 0.005 ;

Vstd = 1.511293  ; 

data  =  load('../../data/freq_response/Motor_Validation.out');


time = data(:,1) ; 
Vs   = data(:,2) ; 
Vcmd = data(:,3) ; 
Vout = data(:,4) - Vstd ;

% 시간 벡터 생성
t = (Tstart:Tsim:Tend)';

% 입력값 초기화
u = zeros(size(t));

% 원하는 구간에 맞게 값 설정

input_data = [time, Vcmd];  % time, value


% === Transfer Function === %

% Pole 
p = 31.1403 ;
K = 0.34924330616996507566938300349243 ;

% 1st Order
num = [K * p];
den = [1, p]; 

% 2nd Order
% num = [1800];
% den = [1 196.6 5154]; 

G = tf(num, den);


out = sim('Step.slx'); 

figure ; 
plot(out.time-33, out.output,'b');

hold on;
plot(time-33,Vout,'g');
title('System Output');
xlabel('Time [s]');
ylabel('Output [V]');
xlim([0 40]);
grid on;



% figure ; 
% % plot(out.time, out.output,'b');
% 
% % hold on;
% plot(time,Vout,'g');
% title('System Output');
% xlabel('Time [s]');
% ylabel('Output [V]');
% xlim([0 40]);
% grid on;

% 
% 
% figure ; 
% % plot(out.time-15, out.output,'b');
% % hold on ; 
% % plot(time,Vout,'g'); 
% plot(time,Vout ,'g'); 
% title('System Output');
% xlabel('Time [s]');
% ylabel('Output [V]');
% grid on;
% xlim([0 5]);
% ylim([-0.5 0]);


% figure ; 
% plot(out.time-5, out.output,'b') ;
% hold on ; 
% % plot(time,Vout,'g') ; 
% % plot(time-5,Vout ,'g') ; 
% title('System Output') ;
% xlabel('Time [s]') ;
% ylabel('Output [V]') ; 
% grid on ;
% xlim([0 5]) ;
% ylim([0 0.5]) ;


figure ; 
plot(time, Vcmd,'b');
hold on ; 
plot(time, Vout);

figure ; 
plot(time, Vout);
hold on ; 
plot(out.time, out.output,'b');





%% 
% 2 order 
% 1800
% 1 196.6 5154

% 1 order 
% 10.88 (0.34924330616996507566938300349243 * 31.1403 )
% 1 31.1403 
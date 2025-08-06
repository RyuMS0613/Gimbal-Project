clc; clear all; close all;

%% 필터 사양 설정
Fs = 200;          % 샘플링 주파수 [Hz]
Fc = 60;            % 차단 주파수 (대역폭) [Hz]
Wn = 2*pi*Fc;       % 차단 각주파수 [rad/s]
Ts = 1/Fs;          % 샘플링 주기

%% 1. 아날로그 (연속시간) Butterworth 필터 설계 (2차)

[b, a] = butter(2, Wn, 's');                % s-domain (analog) 2차 Butterworth 필터

Hs = tf(b, a);                              % 전달함수 표현

disp('--- Continuous-time LPF (Analog) ---');
Hs

%% 2. 이산화 (Tustin 방법)
Hz = c2d(Hs, Ts, 'tustin');                % 이산화된 전달함수 (bilinear transform)

disp('--- Discrete-time LPF (Digital) ---');
Hz

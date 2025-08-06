clc; clear all; close all;

%% === Controller Set (Adjustable)=== %% 

zeta = 0.707 ; 
wn   = 35.5 ; 

% === Constant === %
Kg = 1/0.00067 ; 

% === 1/s === %
Integ_num = [ 1 ] ;  
Integ_den = [ 1,0 ] ; 
Integ     = tf(Integ_num , Integ_den) ; 


% === Gm  (07/29) === %
Gm_num = [10.88]   ; 
Gm_den = [1 31.1403] ; 

Pm    = Gm_den(2) ; 
tau_m = 1 / Gm_den(2) ; 
Km    = Gm_num(1) * tau_m ; 

Gm =tf(Gm_num, Gm_den) ;

% === Constant (Kp,Ki) === %
Kp = ((2 * wn * zeta / Pm) - 1) / (Km * Kg) ; 
Ki = (wn^2 /(Km * Kg * Pm)) ; 


% === Gcl === %
Gcl_num = [ Kp * Km * Pm , Ki * Km * Pm] ;  
Gcl_den = [1 , Pm * (1 + Km * Kg * Kp) ,  Ki * Km * Kg * Pm ] ;
Gcl     = tf(Gcl_num , Gcl_den) ;   

% === Ge === %
Gb_num = [ 1, Pm, 0 ] ; 
Gb_den = [ 1, Pm*(1+Km*Kp*Kg), Pm*Km*Kg*Ki ] ; 
Ge     = tf(Gb_num , Gb_den) ; 

Wb = 5;  % [rad/s]
[mag, phase] = bode(Ge, Wb);  % mag: gain, phase: 위상 (deg)


%% === Controller Design === %%

w_c    = 10: 0.01 : 100 ;
zeta_c = 0 : 0.001  : 1 ;

zero   = (Ki / Kp)./w_c;

PM_LIST = zeros(length(w_c),1);
Tr_90   = zeros(length(w_c),1);
OS      = zeros(length(w_c),1);
Ge      = zeros(length(w_c),1);

Wb = 5;

idx = 1; 

for Wp = 10: 0.01 : 100 
    
    Kp_PM = ((2 * Wp * zeta / Pm) - 1) / (Km * Kg) ; 
    Ki_PM = (Wp^2 /(Km * Kg * Pm)) ;  
    
    % === Gc_PM === %
    Gc_num_PM = [ Kp_PM, Ki_PM ] ; 
    Gc_den_PM = [1] ;
    Gc_PM = tf(Gc_num_PM , Gc_den_PM) ; 

    
    Gcl_num_PM = [ Kp_PM * Km * Pm , Ki_PM * Km * Pm] ;  
    Gcl_den_PM = [1 , Pm * (1 + Km * Kg * Kp_PM) ,  Ki_PM * Km * Kg * Pm ] ;
 
    Gcl_PM = tf(Gcl_num_PM , Gcl_den_PM) ;  

    Go_PM = Gm * Gc_PM * Kg * Integ ; 
    [~, PM, ~, ~] = margin(Go_PM);    
    PM_LIST(idx) = PM ;


    Gb_num = [ 1, Pm, 0 ] ; 
    Gb_den = [ 1, Pm*(1 + Km * Kp_PM * Kg), Pm * Km * Ki_PM * Kg ] ; 
    Gb     = tf(Gb_num , Gb_den) ; 

  % [rad/s]
    [mag, phase] = bode(Gb, Wb);  % mag: gain, phase: 위상 (deg)
    mag = squeeze(mag);
    Ge(idx)    = mag ; 

    info = stepinfo(Gcl_PM, 'RiseTimeLimits', [0, 0.9]);
    Tr_90(idx) = info.RiseTime  ;
    OS(idx)    = info.Overshoot ;  


    idx = idx + 1 ;

end


% === Zero === %
figure;
plot(w_c/Pm ,zero,'.');
title('\omega_c-zero interaction dependancy on bandwidth (\zeta = 0.707)')
ylabel('$\frac{zero}{\omega_c}$','Interpreter','latex','FontSize',20, 'FontWeight','bold');
xlabel('$\frac{\omega_c}{\omega_m}$','Interpreter','latex','FontSize',20, 'FontWeight','bold');
legend('zero/\omega_c');
grid on;

% === Rising Time === %
figure;
plot(w_c/Pm,Tr_90,'.');
title('Rising time dependancy on bandwidth (\zeta = 0.707)');
ylabel('rising time$[s]$','Interpreter','latex','FontSize',20, 'FontWeight','bold');
xlabel('$\frac{\omega_c}{\omega_m}$','Interpreter','latex');
legend('rising time [sec]');
grid on; 


% === Overshoot === %
figure;
plot(w_c/Pm,OS,'.'); 
title('Overshoot dependency on zeta'); 
legend('Overshoot');
ylabel('OS$[\%]$','Interpreter','latex','FontSize',20, 'FontWeight','bold')
xlabel('$\frac{\omega_c}{\omega_m}$','Interpreter','latex','FontSize',20, 'FontWeight','bold');
grid on;


% === Phase Margin === %
figure;
plot(w_c/Pm,PM_LIST,'.'); 
title('Phase Margin dependency on $\omega_c$ (zeta = 0.707)','Interpreter','latex');
legend('Phase Margin');
ylabel('PM $[deg]$','Interpreter','latex','FontSize',20, 'FontWeight','bold');
xlabel('$\frac{\omega_c}{\omega_m}$','Interpreter','latex');
grid on; 




% === Phase Margin === %
figure;
plot(w_c/Pm,Ge,'.'); 
title('Gain (at 5 [rad/sec]) dependency on $\omega_c$ (zeta = 0.707)','Interpreter','latex');
legend('Gain at 5 [rad/sec]');
ylabel('Gain','FontSize',20, 'FontWeight','bold');
xlabel('$\frac{\omega_c}{\omega_m}$','Interpreter','latex');
grid on; 





%% === Margin === %% 


% === Gc === %
% Gc_num = [ Kp, Ki ] ; 
% Gc_den = [ 1 , 0] ;
% Gc     = tf(Gc_num , Gc_den) ; 

% === Go === %
% figure();
% Go = Gc * Gm * Kg; 
% margin(Go); 

% === LPF === % 
% fc = 65;                     
% BW = 2*pi*fc;                
% LPF = tf([BW], [1 BW]);
% 
% Go_LPF = Go * LPF ;
% 
% figure() ; 
% margin(Go_LPF) ; 
% title('LPF Effect');

% === Nyquist Plot === %

% theta = 0 : 0.01 : 2*pi;
% x = cos(theta);
% y = sin(theta);
% 
% figure();
% nyquist(Go); hold on;
% 
% plot(x, y, 'r--', 'LineWidth', 1.5);
% 
% legend('Nyquist plot','Unit circle');
% title('Nyquist Plot of Stabilization Go');
% 
% xlim([-1.3 1.3]); 
% ylim([-1.3 1.3]);

% === Crossover Frequency === %
% 
% [re, im, w] = nyquist(Go, logspace(-1, 2, 2000));  % 주파수 범위 확장
% nyq = squeeze(re + 1i*im);
% 
% % 단위원과 가장 가까운 점 찾기
% dist_to_unit_circle = abs(abs(nyq) - 1);
% [~, idx] = min(dist_to_unit_circle);
% 
% % 주파수 성분 출력
% fprintf("단위원 통과 지점 주파수 ≈ %.4f rad/s (%.2f Hz)\n", w(idx), w(idx)/(2*pi));
% fprintf("응답: Re = %.4f, Im = %.4f, 위상 = %.2f deg\n", ...
%     real(nyq(idx)), imag(nyq(idx)), angle(nyq(idx)) * 180/pi);
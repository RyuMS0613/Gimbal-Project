clc; clear all; close all;

%% 
% === Controller Set (Adjustable)=== %
zeta = 0.707 ; 
wn   = 20.0 ; 

% === Constant === %
Kg = 1/0.00067 ; 

% === 1/s === %
Integ_num = [ 1 ] ;  
Integ_den = [ 1,0 ] ; 
Integ     = tf(Integ_num , Integ_den) ; 


% === Gm  (07/29) === %
Gm_num = [10.88]   ; 
Gm_den = [1 31.1403] ; 

Wm    = Gm_den(2) ; 
tau_m = 1 / Gm_den(2) ; 
Km    = Gm_num(1) * tau_m ; 

Gm =tf(Gm_num, Gm_den) ;

% === Constant (Kd,Kp) === %
Kd = (tau_m * 2 * wn * zeta - 1) / (Km * Kg) ; 
Kp = (wn^2 * tau_m) / (Km * Kg) ; 


% === Gcl === %
Gcl_num = [ Km * Kg * Kd / tau_m , Km * Kg * Kp / tau_m] ; 
Gcl_den = [ 1 , (1+Km * Kg* Kd) / tau_m , (Kp * Km * Kg) / tau_m ] ;  
 
Gcl = tf(Gcl_num , Gcl_den) ;  

% === Constant (K1,K2) === % 
K2 = Kd ; 
K1 = Kp / Kd ; 


%% 

% === Controller Design === %

w_c    = 10: 0.01 : 100 ;
zeta_c = 0 : 0.001  : 1 ;

zero   = -(Kp / Kd)./w_c ;

PM_LIST = zeros(length(w_c),1);
Tr_90   = zeros(length(w_c),1);
OS      = zeros(length(w_c),1);

idx = 1; 

for Wp = 10: 0.01 : 100 
    
    Kd_PM = (tau_m * 2 * Wp * zeta - 1) / (Km * Kg) ; 
    Kp_PM = (Wp^2 * tau_m) / (Km * Kg) ; 
    
    % === Gc_PM === %
    Gc_num_PM = [ Kd_PM, Kp_PM ] ; 
    Gc_den_PM = [1] ;
    Gc_PM = tf(Gc_num_PM , Gc_den_PM) ; 

    
    Gcl_num_PM = [ Km * Kg * Kd_PM / tau_m , Km * Kg * Kp_PM / tau_m] ; 
    Gcl_den_PM = [ 1 , (1+Km * Kg* Kd_PM) / tau_m , (Kp_PM * Km * Kg) / tau_m ] ;  
 
    Gcl_PM = tf(Gcl_num_PM , Gcl_den_PM) ;  

    Go_PM = Gm * Gc_PM * Kg * Integ ; 
    [~, PM, ~, ~] = margin(Go_PM);    
    PM_LIST(idx) = PM ;

    info = stepinfo(Gcl_PM, 'RiseTimeLimits', [0, 0.9]);
    Tr_90(idx) = info.RiseTime  ;
    OS(idx)    = info.Overshoot ;  

    idx = idx + 1 ;

end


% === Zero === %
figure;
plot(w_c/Wm ,zero,'.');
title('\omega_c-zero interaction dependancy on bandwidth (\zeta = 0.707)')
ylabel('$\frac{zero}{\omega_c}$','Interpreter','latex','FontSize',20, 'FontWeight','bold');
xlabel('$\frac{\omega_c}{\omega_m}$','Interpreter','latex','FontSize',20, 'FontWeight','bold');
legend('zero/\omega_c');
grid on;

% === Rising Time === %
figure;
plot(w_c/Wm,Tr_90,'.');
title('Rising time dependancy on bandwidth (\zeta = 0.707)');
ylabel('rising time$[s]$','Interpreter','latex','FontSize',20, 'FontWeight','bold');
xlabel('$\frac{\omega_c}{\omega_m}$','Interpreter','latex');
legend('rising time [sec]');
grid on; 


% === Overshoot === %
figure;
plot(w_c/Wm,OS,'.'); 
title('Overshoot dependency on zeta'); 
legend('Overshoot');
ylabel('OS$[\%]$','Interpreter','latex','FontSize',20, 'FontWeight','bold')
xlabel('$\frac{\omega_c}{\omega_m}$','Interpreter','latex','FontSize',20, 'FontWeight','bold');
grid on;


% === Phase Margin === %
figure;
plot(w_c/Wm,PM_LIST,'.'); 
title('Phase Margin dependency on $\omega_c$ (zeta = 0.707)','Interpreter','latex');
legend('Phase Margin');
ylabel('PM $[deg]$','Interpreter','latex','FontSize',20, 'FontWeight','bold');
xlabel('$\frac{\omega_c}{\omega_m}$','Interpreter','latex');
grid on; 



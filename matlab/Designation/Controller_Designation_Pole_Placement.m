clc; clear all; close all;


%% ===== Controller ==== %% 
zeta = 0.707 ; 
wn   = 35.5 ; 

% === Constant === %
Kg = 1/0.00067 ; 

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

%% ===== Pole Placement ==== %% 


OS_std = 15 ; % [%]
zeta_std = sqrt(1 / (1 + (pi / log(100 / OS_std))^2));

OS = exp(-zeta * pi / sqrt(1 - zeta^2)) * 100 ; 


% Pole-Zero Map 
zeta_max = 0.9 ;
theta = acos(zeta_max); 
theta_std = acos(zeta_std); 

% zeta 범위
r= 200 ; 
x_pos_std = [-r * cos(theta_std), 0] ;
y_pos_std = [ r * sin(theta_std), 0] ; 
x_neg_std = [-r * cos(theta_std), 0] ;
y_neg_std = [-r * sin(theta_std), 0] ;

x_pos = [-r * cos(theta), 0];
y_pos = [ r * sin(theta), 0];
x_neg = [-r * cos(theta), 0];
y_neg = [-r * sin(theta), 0];

% Wn 범위 (zeta = 0.707) 
theta_circle = linspace(0, 2*pi, 300);   

% rising time without zero (bigger than this)
tr_std = 2.67/0.1 ; 
circle_x = tr_std * cos(theta_circle); 
circle_y = tr_std * sin(theta_circle);  

% rising time with zero (bigger than this)
wn_tr_std = 0.797038 * 31.1403; 
tr_circle_x = wn_tr_std * cos(theta_circle); 
tr_circle_y = wn_tr_std * sin(theta_circle);  
 
% zero (less than this)
wn_bw_std = 1.5 * 31.1403 ; 
bw_circle_x = wn_bw_std * cos(theta_circle); 
bw_circle_y = wn_bw_std * sin(theta_circle);  

figure();

pzmap(Gcl);

hold on ;

% Zeta
plot(x_pos_std, y_pos_std, 'r--', 'LineWidth', 1.5, 'HandleVisibility','off');
plot(x_neg_std, y_neg_std, 'r--', 'LineWidth', 1.5, 'HandleVisibility','off');
plot(x_pos, y_pos, 'r--', 'LineWidth', 1.5, 'HandleVisibility','off');
plot(x_neg, y_neg, 'r--', 'LineWidth', 1.5, 'HandleVisibility','off');

% plot(circle_x, circle_y, 'r--', 'LineWidth', 1.5);
plot(tr_circle_x, tr_circle_y, 'k--', 'LineWidth', 1.5);
plot(bw_circle_x, bw_circle_y, 'b--', 'LineWidth', 1.5);
xlim([-100 0]);
ylim([-70 70]);
grid on; 

title('Pole-Zero Map'); 
legend('Pole-Zero','> Rising Time (0.1 [sec])','< Bandwidth (Bw of Motor x 1.5 )');

  
fprintf('\n ===== Pole Placement ==== \n') ;
fprintf('Zeta : %f [-] \n',zeta) ;
fprintf('Wn   : %f [rad/sec] \n',wn) ;
fprintf('Wn Limit for Rising Time : %f [rad/sec]\n',wn_tr_std) ;
fprintf('Wn Limit for Bandwidth   : %f [rad/sec]\n',wn_bw_std) ;

figure();
grid on;  
step(Gcl); 
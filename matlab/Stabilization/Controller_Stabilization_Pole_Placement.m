clc; clear all; close all;


%% ===== Controller ==== %% 
zeta = 0.707 ; 
wn   = 35.5 ; 

% === Constant === %
Kg = 1/0.00067 ; 

% === Gm  (07/29) === % 
Gm_num = [10.88]   ; 
Gm_den = [1 31.1403] ; 

Pm    = Gm_den(2) ;  
tau_m = 1 / Gm_den(2) ;  
Km    = Gm_num(1) * tau_m ;  

Gm =tf(Gm_num, Gm_den) ;

% === Constant (Kd,Kp) === %
Kp = ((2 * wn * zeta / Pm) - 1) / (Km * Kg) ; 
Ki = (wn^2 /(Km * Kg * Pm)) ; 


% === Gcl === %
Gcl_num = [ Kp*Km*Pm , Ki*Km*Pm] ;  
Gcl_den = [1 , Pm * (1 + Km * Kg * Kp) ,  Ki * Km * Kg * Pm ] ;
Gcl     = tf(Gcl_num , Gcl_den) ;    


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

% Disturbance Rejection at 5 [rad/sec] (bigger than this)
tr_dist_std = 15.57848702981203580556 ; 
dist_circle_x = tr_dist_std * cos(theta_circle); 
dist_circle_y = tr_dist_std * sin(theta_circle);  

% rising time with zero (bigger than this)
wn_tr_std = 24.8200024314 ; 
tr_circle_x = wn_tr_std * cos(theta_circle); 
tr_circle_y = wn_tr_std * sin(theta_circle);  
 
% zero (less than this) 
wn_bw_std = 1.5 * 31.1403 ; % 46.71045
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
% plot(tr_circle_x   , tr_circle_y   , 'k--', 'LineWidth', 1.5);
plot(bw_circle_x   , bw_circle_y   , 'b--', 'LineWidth', 1.5);
plot(dist_circle_x , dist_circle_y , 'b--', 'LineWidth', 1.5);

grid on; 
xlim([-100 0]);
ylim([-70 70]);
title('Pole-Zero Map'); 
legend('Pole-Zero','> Rising Time (0.1 [sec])','< Bandwidth (Bw of Motor x 1.5 )');

fprintf('\n ===== Pole Placement ==== \n') ;
fprintf('Zeta : %f [-] \n',zeta) ;
fprintf('Wn   : %f [rad/sec] \n',wn) ;
fprintf('Wn Limit for Rising Time : %f [rad/sec]\n',wn_tr_std) ;
fprintf('Wn Limit for Bandwidth   : %f [rad/sec]\n',wn_bw_std) ;


figure();
step(Gcl);
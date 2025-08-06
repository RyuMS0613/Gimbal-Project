clc; clear all; close all;

% ======== 1. input/output ======== %

data_1 = load('../../data/linearization/Linearization_1.out');  
time = data_1(:,1) ;
Vs_1 = data_1(:,2) ; 
Vg_1 = data_1(:,3) ; 

data_2 = load('../../data/linearization/Linearization_2.out');  
Vs_2 = data_2(:,2) ; 
Vg_2 = data_2(:,3) ;

data_3 = load('../../data/linearization/Linearization_3.out');  
Vs_3 = data_3(:,2) ; 
Vg_3 = data_3(:,3) ; 

data_4 = load('../../data/linearization/Linearization_4.out');  
Vs_4 = data_4(:,2) ; 
Vg_4 = data_4(:,3) ; 

data_5 = load('../../data/linearization/Linearization_5.out');  
Vs_5 = data_5(:,2) ; 
Vg_5 = data_5(:,3) ; 


data = load('../../data/linearization/Linearization.out');  
Vs = data(:,2) ; 
Vg = data(:,3) ; 

% Vs = (Vs_1 + Vs_2 + Vs_3 + Vs_4 + Vs_5)/5 ; 
% Vg = (Vg_1 + Vg_2 + Vg_3 + Vg_4 + Vg_5)/5 ; 

Offset_output = zeros(100,1) ; 

for k = 1:100
    Offset_output(k) = mean(Vg(200*(k)-100 : 200*(k)));
end

% Vstd = sum(Offset_output)/100 ; 
Vstd =  1.517933 ; 

Omega = (Vg-Vstd)/0.00067 ; 


Vo_output = zeros(51,1)  ;
Wg_output  = zeros(51,1) ; 
Vs_input = zeros(51,1)   ; 

Vo_output(1)  = mean(Vg(100:200)) ; 
Wg_output(1)  = mean(Omega(100:200)); 
Vs_input(1)   = 2.5 ; 

for i = 1:50 
    
    avg_Vo = mean(Vg(400*(i)-100 : 400*(i)));
    Vo_output(i+1)= avg_Vo;

    avg_Wg = mean(Omega(400*(i)-100 : 400*(i)));
    Wg_output(i+1)= avg_Wg; 

    Vs_input(i+1) = Vs(400*(i)-100); 

end


% 오름차순 정렬
Vo_output = sort(Vo_output);
Wg_output = sort(Wg_output);
Vs_input  = sort(Vs_input);



%% 
% Plot - result 

figure ; 

plot(time,Vs,'LineWidth',1);
hold on ; grid on ; 
plot(time,Vg,'LineWidth',1);

xlabel("Time [sec]");
ylabel("Voltage [V]");
legend('Vs','Vo');
title('Steady State I/O Relationship');

%% 
% Plot - Vs to Wg 

figure ; 

plot(Vs_input,Wg_output,'.','LineWidth',1); 

xlabel("Vs [V]"); 
ylabel("\omega_{g} [rad/sec]"); 
legend('\omega_{g}') ;
title('Steady State I/O Relationship') ;

grid on ; 

%% 
% Plot - Vs to Vo 

figure ; 

plot(Vs_input,Vo_output,'.','LineWidth',1); 

xlabel("Vs [V]"); 
ylabel("Vo [V]"); 
legend('Vo [V]') ;
title('Steady State I/O Relationship') ;

grid on ; 

%% 

% ======== 2. Linearzation ======== %

Vcmd = -2.5 : 0.01 : 2.5 ; 

Wg_max = 1200 ; 
Wg_min = -1200  ; 

Wg = (Wg_max-Wg_min)/(5)*(Vcmd-2.5) + Wg_max ; 

%% 
% Plot - Linearization Model (Vcmd-> Wg) 
figure ; 

plot(Vcmd,Wg,'.','LineWidth',1); 

xlabel("Vcmd [V]"); 
ylabel("\omega_{g} [rad/sec]"); 
legend('\omega_{g}'); 
title('Linearization'); 

grid on ; 


%%

Vg = 0.00067 *Wg ; 

% Plot - Linearization Model (Vcmd-> Vg) 
figure ; 

plot(Vcmd,Vg,'.','LineWidth',1); 

xlabel("Vcmd [V]"); 
ylabel("V_g []"); 
legend('v_g'); 
title('Linearization'); 

grid on ; 

%% 
% ======== 3. Mapping ======== %


Vcmd_map_pre = zeros(51,1) ;
Vs_map   = 0.8 : 0.1 : 4.2 ; 

for k=1:51
    
    Vcmd_map_pre(k)= (5 * (Wg_output(k) - Wg_max)) / (Wg_max - Wg_min) + 2.5 ;
    
end

Vcmd_map = zeros(35,1) ;
Vcmd_map = Vcmd_map_pre(9:43) ; 


%%

% Configuration %

figure ; 

plot(Vcmd_map_pre,Vs_input,'.','LineWidth',1); 

xlabel("Vcmd [V]"); 
ylabel("Vs   [V]"); 
legend('Vs [V]'); 
title('Configuration'); 

grid on ; 

%% 
% Plot - Fcmd
figure ; 

plot(Vcmd_map,Vs_map,'.','LineWidth',1);

xlabel("Vcmd [V]"); 
ylabel("Vs [V] "); 
legend('Vcmd to Vs [V]'); 
title('f(cmd)'); 

grid on ; 

%% 
% ======== P/M & Plot ======== %

n = 4 ; 

Vs_map_minus = Vs_map(n:16);
Vcmd_map_minus = Vcmd_map(n:16);

Vs_map_plus = Vs_map(20:36-n);
Vcmd_map_plus = Vcmd_map(20:36-n);

figure ; 
plot(Vcmd_map_plus,Vs_map_plus,'.','LineWidth',1);

xlabel("Vcmd-plus [V]"); 
ylabel("Vs_plus [V] "); 
legend('Vs [V]'); 
title('f-plus(cmd)'); 

grid on ; 

figure ; 
plot(Vcmd_map_minus,Vs_map_minus,'.','LineWidth',1);

xlabel("Vcmd-minus [V]"); 
ylabel("Vs_minus [V] "); 
legend('Vs [V]'); 
title('f-minus(cmd)'); 

grid on ; 


% 계수 찾기 
p = fit(Vcmd_map_plus, Vs_map_plus', 'poly2');

m = fit(Vcmd_map_minus, Vs_map_minus', 'poly2'); 






%%
% ====== f(cmd) ====== %

Vcmd_fcmd = -2.5 : 0.1 : 2.5 ; 

m_final = 24 ;
p_start = 28 ;
p_final = length(Vcmd_fcmd);

Vcmd_fcmd_minus = Vcmd_fcmd(1:m_final); 
Vcmd_fcmd_plus = Vcmd_fcmd(p_start:p_final); 


p1 = p.p1 ; 
p2 = p.p2 ; 
p3 = p.p3 ; 
% p4 = p.p4 ; 

fcmd_plus =  p1 * Vcmd_fcmd_plus.^2 +  p2 * Vcmd_fcmd_plus + p3 ;
% fcmd_plus =  p1 * Vcmd_fcmd_plus.^3 +  p2 * Vcmd_fcmd_plus.^2 + p3*Vcmd_fcmd_plus +p4  ;

m1 = m.p1 ; 
m2 = m.p2 ; 
m3 = m.p3 ; 
% m4 = m.p4 ; 

fcmd_minus =  m1 * Vcmd_fcmd_minus.^2 +  m2 * Vcmd_fcmd_minus + m3 ; 
% fcmd_minus =  m1 * Vcmd_fcmd_minus.^3 +  m2 * Vcmd_fcmd_minus.^2 + m3 * Vcmd_fcmd_minus + m4;

fcmd = zeros(p_final,1);

fcmd(1:m_final)       = fcmd_minus ; 
fcmd(m_final+1)       = 2.5        ;
fcmd(m_final+2)       = 2.5        ;
fcmd(m_final+3)       = 2.5        ;
fcmd(p_start:p_final) = fcmd_plus  ; 



%% 
% Plot - Fcmd 

figure; 

plot(Vcmd_map,Vs_map,'LineWidth',1);

hold on ; grid on ; 
plot(Vcmd_fcmd,fcmd,'.','LineWidth',1); 

xlabel("Vcmd [V]"); 
ylabel("Vs [V] "); 
legend('Vs [V]','fcmd'); 
title('f(cmd)'); 


%% 


Vcm_test_plus = 0:0.001:2.5 ;

p1_test = p.p1 ; 
p2_test = p.p2 ; 
p3_test = p.p3 ; 
% p4_test = p.p4 ; 

fcmd_test_plus =  p1_test * Vcm_test_plus.^2 +  p2_test * Vcm_test_plus + p3_test ;
% fcmd_test_plus =  p1_test * Vcm_test_plus.^3 +  p2_test * Vcm_test_plus.^2 + p3_test* Vcm_test_plus+ p4_test;

Vcm_test_minus = -2.5:0.001:0 ;

m1_test = m.p1 ; 
m2_test = m.p2 ; 
m3_test = m.p3 ; 
% m4_test = m.p4 ; 

fcmd_test_minus =  m1_test * Vcm_test_minus.^2 +  m2 * Vcm_test_minus + m3 ; 
% fcmd_test_minus =  m1_test * Vcm_test_minus.^3 +  m2 * Vcm_test_minus.^2 + m3_test * Vcm_test_minus + m4_test ; 

figure ; 
plot(Vcm_test_plus,fcmd_test_plus);

figure ; 
plot(Vcm_test_minus,fcmd_test_minus);

%%

fprintf('double p1 = %.15f ; \n', p1)  ;   
fprintf('double p2 = %.15f ; \n', p2)  ;   
fprintf('double p3 = %.15f ; \n', p3)  ;   
% fprintf('double p4 = %.15f ; \n', p4)  ;   
fprintf('\n')  ;   
fprintf('double m1 = %.15f ; \n', m1)  ;   
fprintf('double m2 = %.15f ; \n', m2)  ;   
fprintf('double m3 = %.15f ; \n', m3)  ;   
% fprintf('double m4 = %.15f ; \n', m4)  ;   


clc; clear all; close all;

% ======== input/output ======== %

data_1 = load('../../data/linearization/Validation_Result_1.out');
data_2 = load('../../data/linearization/Validation_Result_2.out'); 
data = load('../../data/linearization/Validation_Result.out');

time = (data_1(:,1)+data_2(:,1))/2 ; 
% Vs   = (data_1(:,2)+data_2(:,2))/2 ; 
% Vcmd = (data_1(:,3)+data_2(:,3))/2 ; 
% Vg   = (data_1(:,4)+data_2(:,4))/2 ; 
% Wg   = (data_1(:,5)+data_2(:,5))/2 ; 

Vs   = data(:,2); 
Vcmd = data(:,3); 
Vg   = data(:,4); 
Wg   = data(:,5); 



Vg_output  = zeros(51,1) ; 
Wg_output  = zeros(51,1) ; 
Vcmd_input = zeros(51,1) ; 
Vs_input   = zeros(51,1) ; 


Vg_output(1)  = mean(Vg(10:20));
Vcmd_input(1) = 0 ; 
Vs_input(1)   = 2.5 ; 
Wg_output(1)  = mean(Wg(10:20));

for i = 1:50 
    
    avg_Vg = mean(Vg(400*(i)-100 : 400*(i)));
    Vg_output(i+1)= avg_Vg;

    avg_Vcmd = mean(Vcmd(400*(i)-100 : 400*(i)));
    Vcmd_input(i+1)= avg_Vcmd; 

    avg_Wg = mean(Wg(400*(i)-100 : 400*(i)));
    Wg_output(i+1)= avg_Wg; 

    Vs_input(i+1) = Vs(400*(i)-100); 

end

% 오름차순 정렬
Vg_output = sort(Vg_output);
Wg_output = sort(Wg_output);
Vs_input  = sort(Vs_input);
Vcmd_input = sort(Vcmd_input);

figure ; 

plot(Vcmd_input,Vs_input,'.','LineWidth',1); 

xlabel("Vcmd [V]"); 
ylabel("Vs [V]"); 
legend('f(Vcmd) [V]') ;
title('f(Vcmd)') ;


% ======== Linearization ========= %
Vcmd = -2.5 : 0.01 : 2.5 ; 

Wg_max = 1200 ; 
Wg_min = -1200  ; 

Wg = (Wg_max-Wg_min)/(5)*(Vcmd-2.5) + Wg_max ; 

figure ; 

plot(Vcmd,Wg,'.','LineWidth',1); 
hold on ;
plot(Vcmd_input,Wg_output,'.','LineWidth',1); 

xlabel("Vcmd [V]"); 
ylabel("\omega_{g} [rad/sec]"); 
legend('\omega_{g}'); 
title('Linearization'); 


Wg_linear = (Wg_max - Wg_min)/5 * (Vcmd_input - 2.5) + Wg_max;

% 오차 계산
Wg_error = Wg_linear - Wg_output;

% 오차 플롯
figure;
plot(Vcmd_input, Wg_error, 'r.-', 'LineWidth', 1.5);
xlabel("Vcmd [V]");
ylabel("Wg Error [rad/sec]");
title("Error (Linear Model vs Measured Wg)");
grid on;
legend('Model Wg - Measured Wg');


figure;
plot(time, Vs, 'r.-', 'LineWidth', 1.5);
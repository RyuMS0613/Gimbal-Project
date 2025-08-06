clc; clear all; close all;

data = load('../../data/Potentio/Potentiometer_Model_final.out');

Angle = data(:,1) ;  
Vp    = data(:,2) ;

% ====  ==== %
p = fit(Vp,Angle, 'poly1'); 

p1 = p.p1 ; 
p2 = p.p2 ; 


Vp_Model     =  1.67 : 0.01 : 3.38 ; 
Angle_Model  =  p1 * Vp_Model + p2 ; 

figure ;
plot(Vp,Angle);  
ylim([-70 70]); 
xlabel('Vp [V]');
ylabel('Angle [deg]');
title('Petentio Modeling (Experiment)');

figure ;

plot(Vp_Model,Angle_Model); 
ylim([-70 70]); 
xlabel('Vp [V]');
ylabel('Angle [deg]');
title('Petentio Model');



fprintf("p1 : %f \n",p1 );
fprintf("p2 : %f \n",p2 );
fprintf("\n");

fprintf(" Example \n");
fprintf("p1 * Angle_Model + p2 \n");
fprintf("%f * Angle_Model + %f \n",p1,p2);
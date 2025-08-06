clc; clear all; close all;

data = load('../../data/DOA/DOA_Model_data.out');

DOA  = data(:,1) ;  
Vp = data(:,2) ;

figure ; 

plot(DOA,Vp); 

Vp_p  = Vp(1:3);
DOA_p = DOA(1:3);

Vp_m   = Vp(3:5);
DOA_m  = DOA(3:5);

p = fit(DOA_p,Vp_p, 'poly1'); 

p1 = p.p1 ; 
p2 = p.p2 ; 


plus = p1 * DOA_p + p2 ; 


figure ;

plot(DOA_p,Vp_p); 
hold on ;
plot(DOA_p,plus);





% ====  ==== %
m = fit(DOA_m,Vp_m, 'poly1'); 

m1 = m.p1 ; 
m2 = m.p2 ; 


% m1 = 68.085163299236939  ; 
% m2 = -169.8648388361487 ; 


minus  = m1 * DOA_m + m2 ; 

figure ;

plot(DOA_m,Vp_m); 
hold on ;
plot(DOA_m,minus);


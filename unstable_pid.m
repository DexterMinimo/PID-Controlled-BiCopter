clc;
clear;
close all;

k = 66.886;
tau = 0.42026;
theta = .03;

lambda = 0.1




alpha1 = (2*lambda*tau + lambda*theta + 2*tau*theta)/(2*tau-theta); 

kc = (2*alpha1 + theta)/(2*k*(alpha1 - lambda - theta));
tau_i = alpha1 + (theta/2);
tau_d = alpha1*theta/(2*alpha1+theta);

kp = kc
ki = kc/tau_i
kd = kc*tau_d


s=tf('s');
G = k*exp(-theta*s)/(s);
C=pid(kp,ki,kd,0.01);
step(feedback(G*C,1))

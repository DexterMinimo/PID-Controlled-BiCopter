clc;
clear;
close all;

k = 66.886;
tau = 0.42026;
theta = 0.3;

% Define lambda values
lambda_values = [0.1, 0.2, 0.3, 0.4 , 0.6, 0.8, 1, 1.5, 3, 5];

for i = 1:length(lambda_values)
    
    lambda = lambda_values(i);
    
    % Calculate PID gains
    alpha1 = (2*lambda*tau + lambda*theta + 2*tau*theta)/(2*tau-theta); 
    kc = (2*alpha1 + theta)/(2*k*(alpha1 - lambda - theta));
    tau_i = alpha1 + (theta/2);
    tau_d = alpha1*theta/(2*alpha1+theta);
    kp = kc;
    ki = kc/tau_i;
    kd = kc*tau_d;

    % Create system transfer function and PID controller
    s=tf('s');
    G = k*exp(-theta*s)/(s);
    C=pid(kp,ki,kd,0.01);

    % Generate and plot step response
    t = linspace(0, 10, 5003);
    u = ones(size(t));
    y = lsim(feedback(G*C,1), u, t);
    plot(y, 'DisplayName', sprintf('\\lambda=%.1f', lambda))
    hold on

end
legend('show')
title('Step Response for Various Values of \lambda')

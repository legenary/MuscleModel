clear; close all;

muslce_activation_tau_a = 0.08;
muslce_activation_tau_d = 0.08;

phase = 120;
ratio = 0.6;

excitation = [ones(phase*ratio, 1); zeros(phase*(1-ratio), 1); 
    ones(phase*ratio, 1); zeros(phase*(1-ratio), 1)];
a = zeros(phase*4, 1);
for t = 2:(phase*2)
    if (excitation(t)>a(t-1))
        tau = muslce_activation_tau_a * (0.5 + 1.5*a(t-1));
    else
        tau = muslce_activation_tau_d / (0.5 + 1.5*a(t-1));
    end
    da = (excitation(t)-a(t-1))/tau;
    a(t) = a(t-1) + da * (1/phase);
end

figure; hold on;
plot(excitation);
plot(a);

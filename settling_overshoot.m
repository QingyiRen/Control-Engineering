function [wn,t_s] = settling_overshoot(ratio,band)
t_s=4/band/ratio/ratio*sqrt(1-2*ratio*ratio+sqrt(ratio^4-4*ratio*ratio+2));
wn=-log(0.02*sqrt(1-ratio*ratio))/t_s/ratio;
end
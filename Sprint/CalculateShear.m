clear all; close all; clc;

z_r = 150 ; % [m] HH
u_r = 12.5; % [m/s] HH wind speed 
alfa = 0.2;

z = 0:1:270;

u = (z / z_r).^alfa * u_r;

grad = gradient(u, z);
m = grad(z == z_r);

figure;
hold on; grid on; box on
plot(u, z);
close all;
clear all;

A = csvread('output.txt');

i = A(:,1);
q = A(:,2);

plot(i, 'color', 'blue');
hold on;
plot(q, 'color', 'red');
hold off;
grid on;
grid minor;
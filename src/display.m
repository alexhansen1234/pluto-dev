close all;
clear all;

A = csvread('output.txt');
B = csvread('frame.txt');

i = A(:,1);
q = A(:,2);

i_corr = A(:,3);
q_corr = A(:,4);

freq_offset = A(:,5);

error_p = A(:,6);

fs = 2.5e6;

fig1 = figure(1);

subplot(3,1,1);
plot(i);
hold on;
plot(q);
title('Original Signal');
hold off;
grid on;
grid minor;

subplot(3,1,2);
plot(i_corr);
hold on;
plot(q_corr);
title('Frequency Corrected Signal');
hold off;
grid on;
grid minor;

subplot(3,1,3);
plot(freq_offset);
title('Digital Frequency Offset');
grid on;
grid minor;

offset = 100000
fft_len = 2048;
fig2 = figure(2);
spec = fftshift(fft(i(offset:offset+fft_len-1) + 1j * q(offset:offset+fft_len-1))) / fft_len;
freq = (-fft_len/2 : fft_len/2 -1) / fft_len * fs;
stem(freq, abs(spec));

hold on;
spec2 = fftshift(fft(i_corr(offset:offset+fft_len-1) + 1j * q_corr(offset:offset+fft_len-1))) / fft_len;
stem(freq, abs(spec2));

fig3 = figure(3);
plot(B);
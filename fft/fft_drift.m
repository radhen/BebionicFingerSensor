Fs = 340;            % Sampling frequency                    
T = 1/Fs;             % Sampling period       
L = length(Nrl);             % Length of signal
t = (0:L-1)*T;        % Time vector

Yrl = fft(Nrl);
P2 = abs(Yrl/L);
P1 = P2(1:L/2+1);
P1(2:end-1) = 2*P1(2:end-1);f = Fs*(0:(L/2))/L;
plot(f,P1) 
title('Single-Sided Amplitude Spectrum of X(t)')
xlabel('f (Hz)')
ylabel('|P1(f)|')

% X1000C2 = rmoutliers(IDCpokes9993.baro);
% N1000C2 = normalize(X1000C2);
% hold on
% fft_drift
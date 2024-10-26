filename = 'LoggedData.csv';

M = readmatrix(filename);

M = M(:, 1:3);

X = M(:, 1);
Y = M(:, 2);
Z = M(:, 3);

N = length(X);

T = 0:(N-1);

dt = 0.02;  % 20 ms sample rate

T = T * dt;
T = T';

Ta = T(end);

f = 1/dt;   % 10 Hz

fs = 0:f/(N):f/2;
% fs = -f/2:f/(N-1):f/2;

X_fft = fftshift(fft(X));
X_fft = X_fft(N/2:end);

plot(fs, abs(X_fft));

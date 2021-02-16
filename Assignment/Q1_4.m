%% PSK Constellation
clc                       %for clearing the command window
close all                 %for closing all the window except command window

% symbol = [1, i, -1, -i]; QPSK symobls
% fprintf("M is %d" , length(C))
% scatterplot(symbol, [], [], 'r*')
% grid

M=8;
k = log2(M);               % Bits per symbol

data = randi([0, 1], 1000*k, 1); % Generate binary data
txData = reshape(data, [], k);   % Reshape and chunk in two k bits per symbol
txsym = bi2de(txData);         % Convert bits to tranmitted symbols
modSig = qammod(txsym, M);  % MPSK modulation
scatterplot(modSig, [], [], 'r*');
grid on;
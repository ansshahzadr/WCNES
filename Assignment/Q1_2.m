%% PSK Constellation
clc                       %for clearing the command window
close all                 %for closing all the window except command window

% symbol = [1, i, -1, -i]; QPSK symobls
% fprintf("M is %d" , length(C))
% scatterplot(symbol, [], [], 'r*')
% grid

M=16;                        % M=4 for QPSK
k = log2(M);               % Bits per symbol

data = randi([0, 1], 1000*k, 1); % Generate binary data
txData = reshape(data, [], k);   % Reshape and chunk in two k bits per symbol
txsym = bi2de(txData);         % Convert bits to tranmitted symbols
%% PSK Modulation
modSig = pskmod(txsym, M);  % MPSK modulation
scatterplot(modSig, [], [], 'r*');

%% AWGN Channel
snr=10;
rxSig = awgn(modSig, snr);    % AWGN channel with snr
scatterplot(rxSig, [], [], 'r*');

%snr=20;
%rxSig = awgn(modSig, snr);    % AWGN channel with snr
%scatterplot(rxSig, [], [], 'r*');


%% Demodulation
rxSym = pskdemod(rxSig, M);   % MPSK demodulation
rxData = de2bi(rxSym, k);     % Convert received symbols to bits
rxData = reshape(rxData, [], 1);
scatterplot(rxData, [], [], 'r*');


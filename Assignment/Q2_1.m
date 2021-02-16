clc                       %for clearing the command window
close all                 %for closing all the window except command window
clear all

% Modulation order M
M = 2;
k = log2(M);            % Bits per symbol
EbNoVec = (5:15)';      % Eb/No values (dB)
numSymPerFrame = 100;   % Number of QAM symbols per frame

berEst_ps_8 = zeros(size(EbNoVec)); 
berEst_ps_16 = zeros(size(EbNoVec)); 
berEst_qam_16 = zeros(size(EbNoVec)); 

for n = 1:length(EbNoVec)
	% Convert Eb/No to SNR
	snrdB_ps_8 = EbNoVec(n) + 10*log10(2);
    snrdB_ps_16 = EbNoVec(n) + 10*log10(4);
    snrdB_qam_16 = EbNoVec(n) + 10*log10(4);
    
    %snrdB
	% Reset the error and bit counters
	numErrs_ps_8 = 0;
    numErrs_ps_16 = 0;
	numErrs_qam_16 = 0;
	
    numBits_ps_8 = 0;
	numBits_ps_16 = 0;
	numBits_qam_16 = 0;
	while numErrs_ps_8 < 200 && numBits_ps_8 < 1e7
	%while numErrs_ps_8 < 200 && numBits_ps_8 < 1e7
        % Generate binary data and convert to symbols
		dataIn = randi([0 1],numSymPerFrame,k);
		dataSym = bi2de(dataIn);
		
		% Your modulator here: 
        modSig_ps_8 = pskmod(dataSym, 8);  % MPSK modulation
        
		
		% Pass through AWGN channel: 
		rxSig_ps_8 = awgn(modSig_ps_8, snrdB_ps_8);    % AWGN channel with snr
        

		% Your demodulator here:
        rxSym_ps_8 = pskdemod(rxSig_ps_8, 8);   % MPSK demodulation
        
		
		% Convert received symbols to bits
		dataOut_ps_8 = de2bi(rxSym_ps_8,3);
        
		
		% Calculate the number of bit errors
		nErrors_ps_8 = biterr(dataIn,dataOut_ps_8);
        
        
		
		% Increment the error and bit counters
		numErrs_ps_8 = numErrs_ps_8 + nErrors_ps_8;
        
		numBits_ps_8 = numBits_ps_8 + numSymPerFrame*k;
    end
    
    while numErrs_ps_16 < 200 && numBits_ps_16 < 1e7
		% Generate binary data and convert to symbols
		dataIn = randi([0 1],numSymPerFrame,k);
		dataSym = bi2de(dataIn);
		
		% Your modulator here: 
        modSig_ps_16 = pskmod(dataSym, 16);  % MPSK modulation
       
		
		% Pass through AWGN channel: 
		rxSig_ps_16 = awgn(modSig_ps_16, snrdB_ps_16);    % AWGN channel with snr
        

		% Your demodulator here:
        rxSym_ps_16 = pskdemod(rxSig_ps_16, 16);   % MPSK demodulation
        
		
		% Convert received symbols to bits
		dataOut_ps_16 = de2bi(rxSym_ps_16,4);
        
		
		% Calculate the number of bit errors
		nErrors_ps_16 = biterr(dataIn,dataOut_ps_16);
        
        
		
		% Increment the error and bit counters
		numErrs_ps_16 = numErrs_ps_16 + nErrors_ps_16;
		
		numBits_ps_16 = numBits_ps_16 + numSymPerFrame*k;
    end
	
    while numErrs_qam_16 < 200 && numBits_qam_16 < 1e7
		% Generate binary data and convert to symbols
		dataIn = randi([0 1],numSymPerFrame,k);
		dataSym = bi2de(dataIn);
		
		% Your modulator here: 
        modSig_qam_16 = qammod(dataSym, 16);  % MPSK modulation	
		
		
		% Pass through AWGN channel: 
		rxSig_qam_16 = awgn(modSig_qam_16, snrdB_qam_16);    % AWGN channel with snr


		% Your demodulator here:
        rxSym_qam_16 = pskdemod(rxSig_qam_16, 16);   % MPSK demodulation
        
		
		% Convert received symbols to bits
		dataOut_qam_16 = de2bi(rxSym_qam_16,4);
        
		
		% Calculate the number of bit errors
		nErrors_qam_16 = biterr(dataIn,dataOut_qam_16);
        
        
		
		% Increment the error and bit counters
		numErrs_qam_16 = numErrs_qam_16 + nErrors_qam_16;
		
		numBits_qam_16 = numBits_qam_16 + numSymPerFrame*k;
	end
	
	
	% Estimate the BER
	berEst_ps_8(n) = numErrs_ps_8/numBits_ps_8;
    berEst_ps_16(n) = numErrs_ps_16/numBits_ps_16;
    berEst_qam_16(n) = numErrs_qam_16/numBits_qam_16;
    
end
semilogy(EbNoVec,berEst_ps_8,EbNoVec,berEst_ps_16,EbNoVec,berEst_qam_16);

figure(1);
semilogy(EbNoVec,berEst_qam_16,'b*');
hold on;
figure(1);
semilogy(EbNoVec,berEst_ps_8,'g*');
hold on;
figure(1);
semilogy(EbNoVec,berEst_ps_16,'r*');
grid on
xlabel('SNR (dB)'); ylabel('BER')
legend('Blue = 16QAM','Green = 8psk','Red = 16psk');
    












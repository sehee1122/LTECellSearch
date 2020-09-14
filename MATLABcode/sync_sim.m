clear;


%Parameter setting ------------------------------------

%synchronization signal
timing_offset = 20;         %timing offset
NID = 355;                  %Physical Cell ID
Snr = 20;                                           %signal to noise ratio, dB
freqOffset = 0;                                     %frequency offset (unit : %)


%frame structure parameters
Nslot = 20;                                         %Num. of slots
NOFDMsym = 7;                                       %Num. of OFDM symbols in a slot
FFTsize = 1024;


Nsym = FFTsize*NOFDMsym*(Nslot+2);                  %number of symbols to be sent
Qm = 2;                                             %2 - QPSK, 4 - 16QAM, 6 - 64QAM
Nbit = Nsym*Qm;                                     %number of bits to be sent
NsymPSS = 62;
NsymSSS = 62;
NsymZeroPadding = 5;
subOffset_SS = FFTsize/2-36+1;
symOffset_pss0 = (NOFDMsym-1)*FFTsize + subOffset_SS;           %slot 0 last OFDM symbol
symOffset_pss5 = (NOFDMsym*10+NOFDMsym-1)*FFTsize + subOffset_SS;            %slot 10 last OFDM symbol



%other parameters (should be fixed)
rate_LowPass = 3/20;            %Low pass ratio
Noversample = 8;                %oversampling rate
RFfreq = 2*pi/Noversample;      %RF frequency (rad)







%TX Process ---------------------------------

%1. bit generation (random)
bit_send = randi(2,1,Nbit) - ones(1,Nbit);


%2. modulation (I Q signal generation)------------------------ 
if Qm == 2 % --- QPSK
    Nsym = Nbit/Qm;                             % number of symbols determination
    sym_mapper = [1+i 1-i -1+i -1-i]/sqrt(2);   %symbol mapper definition (Table in TS 36.211) 

    %symbol mappging
    sym_send = zeros(1,Nsym);
    for k = 1 : Nsym
        sym_send(k) = sym_mapper(bin2dec(num2str(bit_send(Qm*(k-1)+1:Qm*k)))+1 );
    end

elseif Qm == 4 % --- 16QAM

    Nsym = Nbit/Qm;                             % number of symbols determination
    sym_mapper = [1+i 1+3*i 3+i 3+3*i ...
                  1-i 1-3*i 3-i 3-3*i ...
                  -1+i -1+3*i -3+i -3+3*i ...
                  -1-i -1-3*i -3-i -3-3*i]/sqrt(10); %symbol mapper definition (Table in TS 36.211)

    %symbol mappging              
    sym_send = zeros(1,Nsym);
    for k = 1 : Nsym
        sym_send(k) = sym_mapper(bin2dec(num2str(bit_send(Qm*(k-1)+1:Qm*k)))+1 );
    end 

elseif Qm == 6 % --- 64QAM
    Nsym = Nbit/Qm;
    sym_mapper = [3+3*i 3+1*i 1+3*i 1+i 3+5*i 3+7*i 1+5*i 1+7*i...
                  5+3*i 5+1*i 7+3*i 7+i 5+5*i 5+7*i 7+5*i 7+7*i...
                  3-3*i 3-1*i 1-3*i 1-i 3-5*i 3-7*i 1-5*i 1-7*i...
                  5-3*i 5-1*i 7-3*i 7-i 5-5*i 5-7*i 7-5*i 7-7*i...
                  -3+3*i -3+1*i -1+3*i -1+i -3+5*i -3+7*i -1+5*i -1+7*i...
                  -5+3*i -5+1*i -7+3*i -7+i -5+5*i -5+7*i -7+5*i -7+7*i...
                  -3-3*i -3-1*i -1-3*i -1-i -3-5*i -3-7*i -1-5*i -1-7*i...
                  -5-3*i -5-1*i -7-3*i -7-i -5-5*i -5-7*i -7-5*i -7-7*i...
                  ]/sqrt(42); %TS 36.211 7.1.3

    %symbol mappging
    sym_send = zeros(1,Nsym);
    for k = 1 : Nsym
        sym_send(k) = sym_mapper(bin2dec(num2str(bit_send(Qm*(k-1)+1:Qm*k)))+1 );
    end 

else % wrong Qm value :: regard as QPSK
    printf("Qm is weird! setting Qm as 2...\n");
    Qm = 2;

    Nsym = Nbit/Qm;
    sym_mapper = [1+i 1-i -1+i -1-i]/sqrt(2);
    sym_send = zeros(1,Nsym);
    for k = 1 : Nsym
        sym_send(k) = sym_mapper(bin2dec(num2str(bit_send(Qm*(k-1)+1:Qm*k)))+1 );
    end
end



%2.5 synch signal insertion
%Calculating NID1, 2
NID2 = mod(NID,3);
NID1 = (NID-NID2)/3;

%PSS generation
sig_PSS = gen_PSS(NID2);
%mapping to RE
sym_send(symOffset_pss0 : symOffset_pss0+(NsymPSS+2*NsymZeroPadding)-1 ) = [zeros(1,NsymZeroPadding) sig_PSS zeros(1,NsymZeroPadding)];
sym_send(symOffset_pss5 : symOffset_pss5+(NsymPSS+2*NsymZeroPadding)-1 ) = [zeros(1,NsymZeroPadding) sig_PSS zeros(1,NsymZeroPadding)];

%SSS generation
[sig_SSS] = gen_SSS(NID1, NID2);
%mapping to RE
symOffset_sss0 = symOffset_pss0 - FFTsize;
symOffset_sss5 = symOffset_pss5 - FFTsize;
sym_send(symOffset_sss0 : symOffset_sss0+(NsymSSS+2*NsymZeroPadding)-1 ) = [zeros(1,NsymZeroPadding) sig_SSS(1,:) zeros(1,NsymZeroPadding)];
sym_send(symOffset_sss5 : symOffset_sss5+(NsymSSS+2*NsymZeroPadding)-1 ) = [zeros(1,NsymZeroPadding) sig_SSS(2,:) zeros(1,NsymZeroPadding)];



%3. OFDM modulation (IFFT)
if FFTsize > 0
    symf_send = [];
    for k = 1 : Nsym/FFTsize
        symf_send = [symf_send ifft(sym_send((k-1)*FFTsize + 1 : k*FFTsize))];
    end
end


%3.5 timing offset insertion (wrap around)
%inserting 1 slot at the front
N1slotSym = FFTsize*NOFDMsym;
symf_send = [symf_send(Nsym-N1slotSym+1:Nsym) symf_send(1:Nsym-N1slotSym)];
%inserting the additional timing offset
symf_send = [symf_send(Nsym-timing_offset+1:Nsym) symf_send(1:Nsym-timing_offset)];

%printing the actual timing information
timing_offset_insertion = timing_offset + 1 + FFTsize*NOFDMsym






%Channel modeling ---------------------------------

%1. AWGN Noise insertion
N0 = randn(1,Nsym)/(10^(Snr/10));
rx_demod = symf_send + N0;

%2. frequency offset insertion
n = 1:Nsym;
rx_demod = rx_demod .* exp(i*RFfreq*freqOffset/100*n);







%RX Process -------------------------
%1. frequency offset compensation
%rx_sym = rx_demod .* exp(-i*RFfreq*freqOffset/100*n);

%PSS detection
max_timing = -1;
max_metric = -100000;
max_Nid = 0;
for testNid = 0 : 2
    PSSpattern = gen_PSS(testNid);
    timePssPattern = zeros(1,FFTsize);
    timePssPattern(subOffset_SS+NsymZeroPadding : subOffset_SS+NsymZeroPadding+NsymPSS-1 ) = PSSpattern;
    timePssPattern = ifft(timePssPattern);
    
    for testTiming = 1 : Nsym-FFTsize
        metric = dot(timePssPattern, rx_demod(testTiming:testTiming+FFTsize-1));
        if max_metric < metric
            max_metric = metric;
            max_timing = testTiming;
            max_Nid = testNid;
        end
    end
end

estimated_timing_offset = max_timing;

estimatedNID2 = max_Nid;

%SSS detection
max_seq = 1;
max_metric = -100000;
max_Nid = 0;

SSSsym = rx_demod(estimated_timing_offset-FFTsize:estimated_timing_offset-1);
SSSsymf = FFT_imp(SSSsym, FFTsize);
SSSrx = SSSsymf(subOffset_SS+NsymZeroPadding : subOffset_SS+NsymZeroPadding + NsymSSS-1);
for testNid = 0 : 167
    SSSpattern = gen_SSS(testNid, estimatedNID2);
    
    for seq = 1 : 2
        metric = dot(SSSrx, SSSpattern(seq,:));
        if metric > max_metric
            max_metric = metric;
            max_Nid = testNid;
            max_seq = seq;
        end
    end
end

estimatedNID = max_Nid*3 + estimatedNID2
if max_seq == 1
    estimated_timing_offset = estimated_timing_offset - (NOFDMsym-1)*FFTsize
else
    estimated_timing_offset = estimated_timing_offset - (NOFDMsym*10+NOFDMsym-1)*FFTsize
end



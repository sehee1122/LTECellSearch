function fft_out = FFT_imp(Xin, N)

%N-point FFT by recursive method
%FFT_imp call itself with FFT size N/2 until N becomes 2

if mod(N,2) == 0
    if N == 2 %2-point FFT
        fft_out(1) = Xin(1)+Xin(2);
        fft_out(2) = Xin(1)-Xin(2);
    else %call itself with FFT size N/2
        
        n = 1:N;
        W = exp(-i*2*pi/N*(n-1)); %butterfly weights
        n_perm = 0:2:(N-1); %N/2 point FFT input selection

        %fft radix N/2
        fftHighHalf = FFT_imp(Xin(n_perm+1), N/2);
        fftLowHalf = FFT_imp(Xin(n_perm+2), N/2);

        if length(fftHighHalf) == 0 || length(fftLowHalf) == 0
            fft_out = [];
        else
            %butterfly Radix-N
            fft_out = bflyRadixN(fftHighHalf, fftLowHalf, W, N);
        end
    end
else
    printf("ERROR in FFT_imp : wrong fft size %i\n", N);
    fft_out = [];
end


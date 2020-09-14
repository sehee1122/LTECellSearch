function bfly_out = bflyRadixN(xIn, yIn, W, N)

%fft by butterfly calculation (radix N=2^n)
n = 0 : N-1;
stepSize = length(W)/N;

bfly_out = zeros(1,N);
for i = n
    bfly_out(i+1) = xIn(mod(i,N/2)+1)+W(i*stepSize+1)*yIn(mod(i,N/2)+1);
end
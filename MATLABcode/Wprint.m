function res = Wprint(N)

if mod(N,4) ~= 0
    printf('N is not valid (%i)\n', N);
    res = 0;
else

    Wbase = exp(-j*2*pi/N);
    fp = fopen('./outputArray.txt','w');
    
    
    
    %original arrays
    for stepsize = 0 : 3
        W0 = zeros(1,N);
        for k = 1 : N
            W0(k) = Wbase^(stepsize * (k-1));
        end

        varname = sprintf('W%i_%i', N, stepsize);
        outputArray(W0, varname, fp);

        fprintf(fp, '\n\n');
    
    end
    
    
    %shuffle arrays
    for stepsize = 0 : 3
        cnt = 0;
        W0 = zeros(1,N);
        for k1 = 1 : N/4
            for k2 = 1 : 4
                W0(cnt+1) = Wbase^(stepsize * cnt);
                cnt = cnt + 1;
            end
        end

        varname = sprintf('shuffle_W%i_%i', N, stepsize);
        outputArrayShuffle(W0, varname, fp);

        fprintf(fp, '\n\n');
    
    end
    
    
    
    %minus arrays
    for stepsize = 0 : 3
        cnt = 0;
        W0 = zeros(1,N);
        for k1 = 1 : N/4
            for k2 = 1 : 4
                W0(cnt+1) = Wbase^(stepsize * cnt);
                cnt = cnt + 1;
            end
        end

        varname = sprintf('minus_W%i_%i', N, stepsize);
        outputArrayConj(W0, varname, fp);

        fprintf(fp, '\n\n');
    
    end
    
    
    
    

    res = 1;
end
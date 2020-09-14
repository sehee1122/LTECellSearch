function res = outputArray(x, varname, fp)

N = length(x);
row = 4;
col = N/4;

cnt = 1;
fprintf(fp, 'const static int16_t __attribute__((aligned(16))) %s[] = {\n', varname);
for k1 = 1 : row   
    for k2 = 1 : col
        if real(x(cnt)) > 0
            out_real = fix(real(x(cnt))*32767);
        else
            out_real = fix(real(x(cnt))*32768);
        end
        
        if imag(x(cnt)) > 0
            out_imag = fix(imag(x(cnt))*32767);
        else
            out_imag = fix(imag(x(cnt))*32768);
        end
        cnt=cnt+1;
        fprintf(fp, '%i, %i', out_real, out_imag);
        if (k2 ~= col)
            fprintf(fp, ', ');
        end
    end
    if (k1 ~= row)
        fprintf(fp, ', \n');
    end
end

fprintf(fp, '};\n');
res = 1;
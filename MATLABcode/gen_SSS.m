function out_sig = gen_SSS(NID1, NID2)

%------------------------------------------------
%Generating Secondary Synchronization Signal (SSS)
%out_sig : output SSS (2,62)
%       row 1 : subframe 0 sequence
%       row 2 : subframe 5 sequence
%NID1 : PCI (NID1 part, 0 ~ 167)
%NID2 : PCI (NID2 part, 0 ~ 2)
%------------------------------------------------

%m0, m1 determination
mTableThres = [0   1; %start gap
               30  2;
               59  3;
               87  4;
               114 5;
               140 6;
               165 7;
               168 8];
[mSize dum] = size(mTableThres);

for i = 1:mSize-1
    if NID1 >= mTableThres(i,1) && NID1 < mTableThres(i+1,1)
        startNum = mTableThres(i,1);
        gap = mTableThres(i,2);
    end
end

m0 = NID1 - startNum;
m1 = m0 + gap;

%z determination
x_z = [0 0 0 0 1];
for i = 6 : 31
    x_z(i) = mod( x_z(i-1)+x_z(i-3)+x_z(i-4)+x_z(i-5) ,2);
end

for i = 1 : 31
    z_tilde(i) = 1-2*x_z(i);
end

for i = 1 : 31
    zm0(i) = z_tilde( mod( (i-1)+mod(m0,8), 31)+1 );
    zm1(i) = z_tilde( mod( (i-1)+mod(m1,8), 31)+1 );
end


%c determination
x_c = [0 0 0 0 1];
for i = 6 : 31
    x_c(i) = mod( x_c(i-2) + x_c(i-5) ,2);
end

for i = 1 : 31
    c_tilde(i) = 1-2*x_c(i);
end

for i = 1 : 31
    c0(i) = c_tilde( mod( (i-1)+NID2 ,31)+1 );
    c1(i) = c_tilde( mod( (i-1)+NID2+3 ,31)+1 );
end


%s determination
x_s = [0 0 0 0 1];
for i = 6 : 31
    x_s(i) = mod( x_s(i-3) + x_s(i-5) ,2);
end

for i = 1 : 31
    s_tilde(i) = 1-2*x_s(i);
end

for i = 1 : 31
    sm0(i) = s_tilde( mod( (i-1)+m0 ,31)+1 );
    sm1(i) = s_tilde( mod( (i-1)+m1 ,31)+1 );
end

out_sig = zeros(2,62);
for i = 1 : 31
    %subframe 0
    out_sig(1,2*i-1) = sm0(i)*c0(i);
    out_sig(1,2*i) = sm1(i)*c1(i)*zm0(i);
    
    %subframe 5
    out_sig(2,2*i-1) = sm1(i)*c0(i);
    out_sig(2,2*i) = sm0(i)*c1(i)*zm1(i);
end





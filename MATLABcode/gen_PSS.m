function out_sig = gen_PSS(NID2)

%------------------------------------------------
%Generating Primary Synchronization Signal (SSS)
%out_sig : output PSS (1,62)
%NID2 : PCI (NID2 part, 0 ~ 2)
%------------------------------------------------

RootIndex = [25 29 34];
u = RootIndex(NID2+1);

n1 = 0:30;
n2 = 31:61;

out_sig = [exp(-j*pi*u.*n1.*(n1+1)/63) exp(-j*pi*u.*(n2+1).*(n2+2)/63)];
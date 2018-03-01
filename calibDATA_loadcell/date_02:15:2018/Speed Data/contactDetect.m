[b,a] = butter(1,0.3, 'high')
dataOut = filter(b,a,me2.ir);
yyaxis left; plot(me2.ir); hold on; plot(dataOut); yyaxis right; plot(me2.baro,'g-');
% [p_b,loc_b]=findpeaks(deg2030N.baro,'MinPeakHeight', 1554)
% [p_n,loc_n]=findpeaks(deg2030N.newton,'MinPeakHeight', 30)
% [p_i,loc_i]=findpeaks(deg2030N.ir,'MinPeakHeight', 2000, 'MinPeakWidth', 25)

w_size = 75;

for i=1:5
    figure (i)
    yyaxis left; 
    plot(deg2030N.newton(loc_n(i)-w_size:loc_n(i)+w_size)); 
    yyaxis right; 
    plot(deg2030N.baro(loc_b(i)-w_size:loc_b(i)+w_size));
end    
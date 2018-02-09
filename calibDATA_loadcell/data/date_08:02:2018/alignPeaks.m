[p,loc]=findpeaks(zeroDeg30N1.baro,'MinPeakHeight', 1570)
[p_1,loc_1]=findpeaks(zeroDeg30N1.newton,'MinPeakHeight', 30)

for i=1:5
    yyaxis left; 
    plot(zeroDeg30N1.newton(loc_1(i)-75:loc_1(i)+75)); 
    yyaxis right; 
    plot(zeroDeg30N1.baro(loc(i)-75:loc(i)+75));
end    
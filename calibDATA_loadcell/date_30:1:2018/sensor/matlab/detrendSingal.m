[p,s,mu] = polyfit((1:numel(N50.baro))',N50.baro,6);
f_y = polyval(p,(1:numel(N50.baro))',[],mu);

baro_50 = N50.baro - f_y;  
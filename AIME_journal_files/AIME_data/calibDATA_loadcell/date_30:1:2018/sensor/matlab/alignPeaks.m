% load('test_deg20.mat')

% [p_b50,loc_b50]=findpeaks(baro_50,'MinPeakHeight', 1537, 'MinPeakProminence',0.5)
% [p_n50,loc_n50]=findpeaks(n_50,'MinPeakHeight', 0.9)
% [p_ir50,loc_ir50]=findpeaks(ir_50,'MinPeakHeight', 9500, 'MinPeakWidth', 20)

% invertedY = max(y) - y;
% [peakValues, indexes] = findpeaks(invertedY);
% tValues = t(indexes);

w_size = 100;
% 
% N=[];
% B=[];
% IR=[];

for i=1:10
    N_50(:,i) = (n_50(loc_n50(i)-w_size:loc_n50(i)+w_size));
    B_50(:,i) = (baro_50(loc_b50(i)-w_size:loc_b50(i)+w_size));
    IR_50(:,i) = (ir_50(loc_ir50(i)-w_size:loc_ir50(i)+w_size));
%     
%     
%     
%     plotting stuff for un-normalized values
%     figure(1)
%     yyaxis left; 
%     plot(deg2030N.newton(loc_n(i)-w_size:loc_n(i)+w_size)); 
%     yyaxis right; 
%     plot(deg2030N.baro(loc_b(i)-w_size:loc_b(i)+w_size));
%     hold on 
    
end    

% plotting stuff for un-normalized values continued...
% hold off

% N_mean = mean(N,2);
% B_mean = mean(B,2);
% IR_mean = mean(IR,2);
% 
% % calculating normalized values between 0 and 1
% norm_N_mean = N_mean - min(N_mean(:));
% norm_N_mean = norm_N_mean ./ max(norm_N_mean(:));
% 
% norm_B_mean = B_mean - min(B_mean(:));
% norm_B_mean = norm_B_mean ./ max(norm_B_mean(:));
% 
% norm_IR_mean = IR_mean - min(IR_mean(:));
% norm_IR_mean = norm_IR_mean ./ max(norm_IR_mean(:));

% plotting for mean of number of trials (noramlized values) 
% figure(2)
% hold on
% plot(norm_N_mean);
% hold on 
% plot(norm_B_mean);
% hold on 
% plot(norm_IR_mean);
% hold off 
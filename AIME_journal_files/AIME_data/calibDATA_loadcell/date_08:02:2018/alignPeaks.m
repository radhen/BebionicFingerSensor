% load('test_deg20.mat')

% [p_b,loc_b]=findpeaks(N1.baro,'MinPeakHeight', 1537, 'MinPeakProminence',0.5)
% [p_n,loc_n]=findpeaks(deg01N1.ForceN,'MinPeakHeight', 4)
% [p_ir,loc_ir]=findpeaks(N1.ir,'MinPeakHeight', 9500, 'MinPeakWidth', 20)

w_size = 75;
% 
% N=[];
% B=[];
% IR=[];
for i=1:10
%     N(:,i) = (deg050N1.ForceN(loc_n(i)-w_size:loc_n(i)+w_size));
    B(:,i) = (N1.baro(loc_b(i)-w_size:loc_b(i)+w_size));
    IR(:,i) = (N1.ir(loc_ir(i)-w_size:loc_ir(i)+w_size));
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
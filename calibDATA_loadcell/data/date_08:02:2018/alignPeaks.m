% load('test_deg20.mat')

% [p_b,loc_b]=findpeaks(deg2030N.baro,'MinPeakHeight', 1554)
% [p_n,loc_n]=findpeaks(deg2030N.newton,'MinPeakHeight', 30)
% [p_i,loc_i]=findpeaks(deg2030N.ir,'MinPeakHeight', 2000, 'MinPeakWidth', 25)

w_size = 75;

N=[];
B=[];
IR=[];
for i=1:5
    N(:,i) = (deg030N.newton(loc_n(i)-w_size:loc_n(i)+w_size));
    B(:,i) = (deg030N.baro(loc_b(i)-w_size:loc_b(i)+w_size));
    IR(:,i) = (deg030N.ir(loc_ir(i)-w_size:loc_ir(i)+w_size));
    
    
    
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

N_mean = mean(N,2);
B_mean = mean(B,2);
IR_mean = mean(IR,2);

% calculating normalized values between 0 and 1
norm_N_mean = N_mean - min(N_mean(:));
norm_N_mean = norm_N_mean ./ max(norm_N_mean(:));

norm_B_mean = B_mean - min(B_mean(:));
norm_B_mean = norm_B_mean ./ max(norm_B_mean(:));

norm_IR_mean = IR_mean - min(IR_mean(:));
norm_IR_mean = norm_IR_mean ./ max(norm_IR_mean(:));

% plotting for mean of number of trials (noramlized values) 
% figure(2)
% hold on
% plot(norm_N_mean);
% hold on 
plot(norm_B_mean);
hold on 
plot(norm_IR_mean);
hold off 
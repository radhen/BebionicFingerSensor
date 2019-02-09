
clear all; close all; clc;

filenames = dir;
names = {filenames(3:14).name};

plot_index = [1 5 9;
    2 6 10;
    3 7 11;
    4 8 12]; % 1N - 5N - 30N - 50N indices

for t=1:4 % loop over loads
    inc=1;
for i=plot_index(t,:) % loop over angles

    
raw = importdata(names{2});

% if t~=2
IR = raw.data(:,2);
Bar = raw.data(:,3);
Loop = raw.data(:,1);
Time = cumsum(raw.data(3:end,1)); % sum loop time in ms
% elseif t==2
% IR = raw(:,2);
% Bar = raw(:,3);     
    
% end
        
        
% Title Building %

forces = {'_1N';'_5N';'_30N';'_50N'}; %identify loads in filenames
forces_names = {'1N - ';'5N - ';'30N - ';'50N - '}; 
spatials = {'0deg';'-20deg';'+20deg'}; %identify angles
spatials_names = {'Center';'-20deg';'+20deg'};


for f=1:4
    for s=1:3
        if strfind(names{i(inc)},spatials{s})
            Titles{2} = spatials_names{s};
        end
    end
    if strfind(names{i(inc)},forces{f})
        Titles{1} = forces_names{f};
    end
end

Title = strcat(Titles{1},Titles{2});

% Visualize

% figure(1)
% % subplot(4,2,i)
% plot(IR);
% title(strcat('IR - ',Title));
% % title(strcat(txt{1,2},' - ',filename{i}));
% figure(100)
% % subplot(4,2,i)
% plot(Bar);
% title(strcat('BAR - ',Title));
% % Peak Finder

% high_IR = max(IR);low_IR = min(IR);range_IR = high_IR-low_IR;
% Don't use the min(Bar)...use the mean instead.  More robust to '0' values
% in array
high_Bar = max(Bar);low_Bar = min(Bar);range_Bar = high_Bar-mean(Bar);
% MPH_IR = high_IR-(0.1*range_IR);
MPH_Bar = high_Bar-(.5*range_Bar);

% [IR_pks,IR_loc] = findpeaks(IR,'MinPeakHeight',MPH_IR,...
%     'NPeaks',10,'MinPeakDistance',100);
[BAR_pks,BAR_loc] = findpeaks(Bar,'MinPeakHeight',MPH_Bar,...
    'NPeaks',10,'MinPeakDistance',100);
[IR_pks] = IR(BAR_loc); [IR_loc] = BAR_loc; % use BAR peaks for IR signal

% figure(i)
% hold on
% % subplot(4,2,i)
% hold on
% scatter(IR_loc,IR_pks)
% 
% figure(i*100)
% hold on
% % subplot(4,2,i)
% hold on
% scatter(BAR_loc,BAR_pks)


% Center on Peaks

% Find distance between peaks
for j=1:size(IR_loc,1)-1
peak_length_IR_array(j) = IR_loc(j+1)-IR_loc(j);
peak_length_Bar_array(j) = BAR_loc(j+1)-BAR_loc(j);
end

peak_length_IR = mean(peak_length_IR_array);
peak_length_BAR = mean(peak_length_Bar_array);
peak_length = round(mean([peak_length_IR,peak_length_BAR]));


% Build Arrays

for j=2:size(IR_loc,1)-1
    start_index = IR_loc(j)-(round(peak_length/2));
    if start_index < 0, continue; end % skip ahead if first press occurs too soon
    end_index = IR_loc(j)+(round(peak_length/2));
    IR_array(:,j)=IR(start_index:end_index);
end
for j=2:size(IR_loc,1)-1
    start_index = BAR_loc(j)-(round(peak_length/2));
    if start_index < 0, continue; end % skip ahead if first press occurs too soon
    end_index = BAR_loc(j)+(round(peak_length/2));
    BAR_array(:,j)=Bar(start_index:end_index);
end


BAR_array_mean = mean(BAR_array');
IR_array_mean = mean(IR_array');

% Find Contact %
%%%%%%%%%%%%%%%%
% Threshold Method %
%%%%%%%%%%%%%%%%%%%

% Diffentiate filtered IR signal
dt = mean(Loop).*ones(size(IR_array_mean,2),1);
IR_diff = abs([0;diff(IR_array_mean')]./dt);

% Low Pass Filter
a=1; % transfer function denom
windowSize = 10; %length of smoothing window
b = (1/windowSize)*ones(1,windowSize);

IR_diff_filt = filter(b,a,IR_diff);


thres = 14; 

contact = zeros(size(IR_diff,1),1);
for k=2:size(IR_diff,1)
if IR_diff(k)>thres && IR_diff(k-1)<thres % contact event
    contact(k) = 18000;
elseif IR_diff(k)<thres && IR_diff(k-1)>thres % release event
    contact(k) = 18000;
end

end
% 
% figure(i*100)
% subplot(2,1,1)
% plot(IR_array_mean)
% hold on
% % plot([contact(6:end);zeros(5,1)])
% subplot(2,1,2)
% plot(IR_diff)
% hold on
% % plot(contact/1000)


% Adjust BAR_array and IR_array for missing trials %

if sum(BAR_array(:,1))==0
    BAR_array(:,1)=[];
end

if sum(IR_array(:,1))==0
    IR_array(:,1)=[];
end



% Plot Means with Shaded Error Bar %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% where to position into subplot
position = [6,4,5];

ylim_IR = {[5500 11000];[6000 15000];[6000 15000];[7000 13000]};
ylim_Bar = {[1580 1680];[1600 2500];[1600 3500];[1600 1900]};

figure(t)
subplot(3,3,position(inc))
yyaxis right
if t~=2
x = 0.001*mean(raw.data(5:end,1))*[1:size(BAR_array,1)]'; %time in seconds
elseif t==2
    x = 0.001*63*[1:size(BAR_array,1)]'; %time in seconds
end
% x = Time;
bar_std = std(BAR_array');
shadedErrorBar(x,mean(BAR_array'),bar_std','lineprops', '-r')
ylabel('Barometer Signal [millibar]')
% ylim(ylim_Bar{t});
% ylim([1566 1595])

% subplot(1,2,2)
yyaxis left
% x = [1:size(IR_array,1)]';
% x = Time;
IR_std = std(IR_array');
shadedErrorBar(x,mean(IR_array'),IR_std','lineprops', '-b')
hold on
plot(contact)
ylabel('IR Signal [counts]')
title(Title)
xlabel('Time (s)')
xlim([0 20]);
% ylim(ylim_IR{t});
% ylim([0.6E4 1.8E4])

% subplot(1,3,2)
% plot(BAR_array)
% subplot(1,3,3)
% plot(IR_array)

clear BAR_array IR_array BAR_loc

inc = inc+1;

end

end
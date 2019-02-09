% Hybrid Fingertip Sensor 
% Jacob Segil
% February 1st, 2018

%% Peaks

for i=1:3
% Load data

%clear all;close all;clc;
%[num,txt,raw] = xlsread('Hybrid_Fingertip_Testing_v1');
filename = {'0deg-50N','0deg-5N','0deg-1N'};
filenames = {'0deg_50N','0deg_5N','0deg_1N'};
[num,txt,raw] = xlsread(filenames{i});

IR = num(:,2);
Bar = num(:,3);

% Visualize

figure(1)
subplot(3,2,((2*i)-1))
plot(IR);
title(strcat(txt{1,2},' - ',filename{i}));

subplot(3,2,(2*i))
plot(Bar);
title(strcat(txt{1,3},' - ',filename{i}));

% Peak Finder

% high_IR = max(IR);low_IR = min(IR);range_IR = high_IR-low_IR;
high_Bar = max(Bar);low_Bar = min(Bar);range_Bar = high_Bar-low_Bar;
% MPH_IR = high_IR-(0.1*range_IR);
MPH_Bar = high_Bar-(0.1*range_Bar);

% [IR_pks,IR_loc] = findpeaks(IR,'MinPeakHeight',MPH_IR,...
%     'NPeaks',10,'MinPeakDistance',100);
[BAR_pks,BAR_loc] = findpeaks(Bar,'MinPeakHeight',MPH_Bar,...
    'NPeaks',10,'MinPeakDistance',250);
[IR_pks] = IR(BAR_loc); [IR_loc] = BAR_loc; % use BAR peaks for IR signal


figure(1)
hold on
subplot(3,2,((2*i)-1))
hold on
scatter(IR_loc,IR_pks)

hold on
subplot(3,2,(2*i))
hold on
scatter(BAR_loc,BAR_pks)

% Center on Peaks

% Find distance between peaks
for j=1:9
peak_length_IR_array(j) = IR_loc(j+1)-IR_loc(j);
peak_length_Bar_array(j) = BAR_loc(j+1)-BAR_loc(j);
end

peak_length_IR = mean(peak_length_IR_array);
peak_length_BAR = mean(peak_length_Bar_array);
peak_length = round(mean([peak_length_IR,peak_length_BAR]));


% Build Arrays

for j=1:9
    start_index = IR_loc(j)-(round(peak_length/2));
    end_index = IR_loc(j)+(round(peak_length/2));
    IR_array(:,j)=IR(start_index+1:end_index);
end
for j=1:9
    start_index = BAR_loc(j)-(round(peak_length/2));
    end_index = BAR_loc(j)+(round(peak_length/2));
    BAR_array(:,j)=Bar(start_index:end_index);
end

% Visualize 
figure(2)
subplot(3,2,((2*i)-1))
plot(IR_array)
title(strcat('IR',' - ',filename{i}))
hold on
subplot(3,2,(2*i))
plot(BAR_array)
title(strcat('BAR',' - ',filename{i}))

clear all
end

%% Hystersis Curves


clear all;close all;clc;

for k=1:3
filename = {'0deg-50N','0deg-5N','0deg-1N'};
filenames = {'0deg_50N','0deg_5N','0deg_1N'};
[num,txt,raw] = xlsread(filenames{k});
cd Instron

filename_I = {'radhen_10cyc_black_0deg_1mms_50N.txt',...
    'radhen_10cyc_black_0deg_1mms_5N.txt',...
    'radhen_10cyc_black_0deg_1mms_1N.txt'};

raw = importdata(filename_I{k});
cd ..

Dist = raw.data(:,4);
Bar = num(:,3);

% Interpolate

a = size(Bar,1); b = size(Dist,1);
factor = ceil(b/a);

Bar_interp = interp(Bar,factor);

% Match size

Bar_interp = Bar_interp(1:b,1);

% Select a single trial and plot

idx = raw.data(:,6)==1; %find first trial

% Peak Finder (Bar)
high_Bar = max(Bar_interp);low_Bar = min(Bar_interp);range_Bar = high_Bar-low_Bar;
MPH_Bar = high_Bar-(0.1*range_Bar);

[BAR_pks,BAR_loc] = findpeaks(Bar_interp,'MinPeakHeight',MPH_Bar,...
    'NPeaks',10,'MinPeakDistance',250);

% Peak Finder (Dist)
high_Dist = max(Dist);low_Dist = min(Dist);range_Dist = high_Dist-low_Dist;
MPH_Dist = high_Dist-(0.1*range_Dist);

[Dist_pks,Dist_loc] = findpeaks(Dist,'MinPeakHeight',MPH_Dist,...
    'NPeaks',10,'MinPeakDistance',250);


% Build Arrays

for j=1%:8
peak_length_Bar_array(j) = BAR_loc(j+1)-BAR_loc(j);
peak_length_Dist_array(j) = Dist_loc(j+1)-Dist_loc(j);
end

peak_length_Bar = mean(peak_length_Bar_array)/3; % base length on Bar
peak_length_Dist = mean(peak_length_Dist_array)/3; % do not use

for j=1%:8
start_index_h = BAR_loc(j)-(round(peak_length_Bar/2));
end_index_h = BAR_loc(j)+(round(peak_length_Bar/2));
BAR_array(:,j)=Bar_interp(start_index_h:end_index_h);

start_index_h_d = Dist_loc(j)-(round(peak_length_Bar/2));
end_index_h_d = Dist_loc(j)+(round(peak_length_Bar/2));
Dist_array(:,j)=Dist(start_index_h_d:end_index_h_d);
end

figure(1)
subplot(3,2,(2*k)-1)
plot(BAR_array)
title(strcat('Bar - ',filename{k}));
subplot(3,2,(2*k))
plot(Dist_array)
title(strcat('Dist - ',filename{k}));


figure(2)
subplot(1,3,k)
line(Dist_array,BAR_array)
title(strcat('Hysteresis Loop - ',filename{k}));
ylabel('Bar'); xlabel('Dist (mm)');

%clear all
end

%% Force vs. Bar vs. IR

clear all;close all;clc;

for k=1:3
    
% Import Data
filename = {'0deg-50N','0deg-5N','0deg-1N'};
filenames = {'0deg_50N','0deg_5N','0deg_1N'};
[num,txt,raw] = xlsread(filenames{k});
cd Instron

filename_I = {'radhen_10cyc_black_0deg_1mms_50N.txt',...
    'radhen_10cyc_black_0deg_1mms_5N.txt',...
    'radhen_10cyc_black_0deg_1mms_1N.txt'};

raw = importdata(filename_I{k});
cd ..

Dist = raw.data(:,4);
Bar = num(:,3);
Load = raw.data(:,3);

% Interpolate

a = size(Bar,1); b = size(Dist,1);
factor = ceil(b/a);

Bar_interp = interp(Bar,factor);

% Match size

Bar_interp = Bar_interp(1:b,1);

% Remove DC offset
Dist_n = Dist-Dist(1);
Bar_interp_n = Bar_interp - Bar_interp(1);
Load_n = Load - Load(1);

% Normalize all curves to [0-1] and remove DC offset
Dist_n = (Dist_n/max(Dist_n));
Bar_interp_n = ((Bar_interp_n/max(Bar_interp_n)));
Load_n = (Load_n/max(Load_n));





% Plot all curves

% subplot(1,3,1)
% plot(Load_n);
% subplot(1,3,2)
% plot(Dist_n);
% subplot(1,3,3)
% plot(Bar_interp_n)

% Select a single trial and plot

idx = raw.data(:,6)==1; %find first trial

% Peak Finder (Bar)
high_Bar = max(Bar_interp_n);low_Bar = min(Bar_interp_n);range_Bar = high_Bar-low_Bar;
MPH_Bar = high_Bar-(0.1*range_Bar);

[BAR_pks,BAR_loc] = findpeaks(Bar_interp_n,'MinPeakHeight',MPH_Bar,...
    'NPeaks',10,'MinPeakDistance',250);

% Peak Finder (Dist)
high_Dist = max(Dist);low_Dist = min(Dist);range_Dist = high_Dist-low_Dist;
MPH_Dist = high_Dist-(0.1*range_Dist);

[Dist_pks,Dist_loc] = findpeaks(Dist,'MinPeakHeight',MPH_Dist,...
    'NPeaks',10,'MinPeakDistance',250);


% Build Arrays

for j=1:8
peak_length_Bar_array(j) = BAR_loc(j+1)-BAR_loc(j);
peak_length_Dist_array(j) = Dist_loc(j+1)-Dist_loc(j);
end

peak_length_Bar = mean(peak_length_Bar_array)/3; % base length on Bar
peak_length_Dist = mean(peak_length_Dist_array)/3; % do not use

for j=1:8
start_index_h = BAR_loc(j)-(round(peak_length_Bar/2));
end_index_h = BAR_loc(j)+(round(peak_length_Bar/2));
BAR_array(:,j)=Bar_interp_n(start_index_h:end_index_h);

start_index_h_d = Dist_loc(j)-(round(peak_length_Bar/2));
end_index_h_d = Dist_loc(j)+(round(peak_length_Bar/2));
Dist_array(:,j)=Dist_n(start_index_h_d:end_index_h_d);
Load_array(:,j)=Load_n(start_index_h_d:end_index_h_d);
end

% Show all
figure(1)
subplot(3,3,(3*k)-2)
plot(BAR_array)
title(strcat('Bar - ',filename{k}));
subplot(3,3,(3*k)-1)
plot(Dist_array)
title(strcat('Dist - ',filename{k}));
subplot(3,3,(3*k))
plot(Load_array)
title(strcat('Load - ',filename{k}));

% Overlap all signals
figure(2)
subplot(1,3,k)
plot(BAR_array)
hold on
plot(Load_array)

clear all

end

%% IR Contact Detection

clear all;close all;clc;

for k=1
    
% Import Data
filename = {'0deg-50N','0deg-5N','0deg-1N'};
filenames = {'0deg_50N','0deg_5N','0deg_1N'};
[num,txt,raw] = xlsread(filenames{k});
cd Instron

filename_I = {'radhen_10cyc_black_0deg_1mms_50N.txt',...
    'radhen_10cyc_black_0deg_1mms_5N.txt',...
    'radhen_10cyc_black_0deg_1mms_1N.txt'};

raw = importdata(filename_I{k});
cd ..

Dist = raw.data(:,4);
Bar = num(:,3);
IR = num(:,2);
loop = num(:,1);
Load = raw.data(:,3);

% Low Pass Filter

a=1; % transfer function denom
windowSize = 50; %length of smoothing window
b = (1/windowSize)*ones(1,windowSize);

IR_filt = filter(b,a,IR);

plot(IR)
hold on
plot(IR_filt)

% Diffentiate filtered IR signal
IR_diff = [0;diff(IR_filt)]./loop;
figure(2)
plot(IR_diff)

end

%% Speed Trials

% Navigate to 
% G:\My Drive\GE+\Teaching\Spring 2018\Humsini Acharya - Independent Study\
% ...Fingertip MTS Data\data\02.15.2018\Speed Data

for i=1:7
filenames = dir;
raw = importdata(filenames(i+3).name);

IR = raw.data(:,2);
Bar = raw.data(:,3);

% Visualize

figure(1)
% subplot(4,2,i)
plot(IR);
title(strcat('IR - ',num2str(raw.data(6,6)),' (mm/s)'));
% title(strcat(txt{1,2},' - ',filename{i}));
figure(2)
subplot(4,2,i)
plot(Bar);
title(strcat('BAR - ',num2str(raw.data(6,6)),' (mm/s)'));
% Peak Finder

% high_IR = max(IR);low_IR = min(IR);range_IR = high_IR-low_IR;
high_Bar = max(Bar);low_Bar = min(Bar);range_Bar = high_Bar-low_Bar;
% MPH_IR = high_IR-(0.1*range_IR);
MPH_Bar = high_Bar-(.5*range_Bar);

% [IR_pks,IR_loc] = findpeaks(IR,'MinPeakHeight',MPH_IR,...
%     'NPeaks',10,'MinPeakDistance',100);
[BAR_pks,BAR_loc] = findpeaks(Bar,'MinPeakHeight',MPH_Bar,...
    'NPeaks',10,'MinPeakDistance',75);
[IR_pks] = IR(BAR_loc); [IR_loc] = BAR_loc; % use BAR peaks for IR signal

figure(1)
hold on
subplot(4,2,i)
hold on
scatter(IR_loc,IR_pks)

figure(2)
hold on
subplot(4,2,i)
hold on
scatter(BAR_loc,BAR_pks)

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

for j=1:size(IR_loc,1)-1
    start_index = IR_loc(j)-(round(peak_length/2));
    end_index = IR_loc(j)+(round(peak_length/2));
    IR_array(:,j)=IR(start_index+1:end_index);
end
for j=1:size(IR_loc,1)-1
    start_index = BAR_loc(j)-(round(peak_length/2));
    end_index = BAR_loc(j)+(round(peak_length/2));
    BAR_array(:,j)=Bar(start_index:end_index);
end

% Visualize 
figure(3)
subplot(4,2,i)
plot(IR_array)
title(strcat('IR - ',num2str(raw.data(6,6)),' (mm/s)'));
ylim([0.5E4 2E4])

figure(4)
subplot(4,2,i)
plot(BAR_array)
title(strcat('BAR - ',num2str(raw.data(6,6)),' (mm/s)'));
ylim([1560 1600])

clear BAR_array IR_array

% Low Pass Filter
loop = raw.data(:,1);
a=1; % transfer function denom
windowSize = 10; %length of smoothing window
b = (1/windowSize)*ones(1,windowSize);

IR_filt = filter(b,a,IR);

figure(i*10)
plot(IR)
hold on
plot(IR_filt)


% Find Contact %
%%%%%%%%%%%%%%%%
% Threshold Method %
%%%%%%%%%%%%%%%%%%%

% Diffentiate filtered IR signal
IR_diff = ([0;diff(IR)]./loop);
IR_2diff = ([0;diff(IR_diff)]./loop);
% Low Pass Filter
loop = raw.data(:,1);
a=1; % transfer function denom
windowSize = 10; %length of smoothing window
b = (1/windowSize)*ones(1,windowSize);

IR_diff_filt = filter(b,a,IR_diff);


thres = 6; 

contact = zeros(size(IR_diff,1),1);
for k=2:size(IR_diff,1)
if IR_diff(k)>thres && IR_diff(k-1)<thres % contact event
    contact(k) = 18000;
elseif IR_diff(k)<thres && IR_diff(k-1)>thres % release event
    contact(k) = 18000;
end

end
figure(i*100)
subplot(3,1,1)
plot(IR)
hold on
plot([contact(6:end);zeros(5,1)])
subplot(3,1,2)
plot(IR_diff)
hold on
plot(contact/1000)
subplot(3,1,3)
plot(IR_2diff)


% Peak of Differential Method %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%





figure(1)
subplot(4,2,i)
hold on
plot(contact)

end

%% Spatial Data

% Navigate to 
% G:\My Drive\GE+\Teaching\Spring 2018\Humsini Acharya - Independent Study\
% Fingertip MTS Data\data\02.22.2018



for i=7%3:20
filenames = dir;
raw = importdata(filenames(i).name);

IR = raw.data(:,2);
Bar = raw.data(:,3);


% Visualize

% figure(1)
% % subplot(4,2,i)
% plot(IR);
% title(strcat('IR - ',num2str(raw.data(6,6)),' (mm/s)'));
% % title(strcat(txt{1,2},' - ',filename{i}));
% figure(2)
% % subplot(4,2,i)
% plot(Bar);
% title(strcat('BAR - ',num2str(raw.data(6,6)),' (mm/s)'));
% % Peak Finder

% high_IR = max(IR);low_IR = min(IR);range_IR = high_IR-low_IR;
high_Bar = max(Bar);low_Bar = min(Bar);range_Bar = high_Bar-low_Bar;
% MPH_IR = high_IR-(0.1*range_IR);
MPH_Bar = high_Bar-(.5*range_Bar);

% [IR_pks,IR_loc] = findpeaks(IR,'MinPeakHeight',MPH_IR,...
%     'NPeaks',10,'MinPeakDistance',100);
[BAR_pks,BAR_loc] = findpeaks(Bar,'MinPeakHeight',MPH_Bar,...
    'NPeaks',10,'MinPeakDistance',75);
[IR_pks] = IR(BAR_loc); [IR_loc] = BAR_loc; % use BAR peaks for IR signal
% 
% figure(1)
% hold on
% % subplot(4,2,i)
% hold on
% scatter(IR_loc,IR_pks)
% 
% figure(2)
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

for j=1:size(IR_loc,1)-1
    start_index = IR_loc(j)-(round(peak_length/2));
    end_index = IR_loc(j)+(round(peak_length/2));
    IR_array(:,j)=IR(start_index+1:end_index);
end
for j=1:size(IR_loc,1)-1
    start_index = BAR_loc(j)-(round(peak_length/2));
    end_index = BAR_loc(j)+(round(peak_length/2));
    BAR_array(:,j)=Bar(start_index:end_index);
end

% Plot Means with Shaded Error Bar %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

figure(10*i)
subplot(1,2,1)
x = [1:size(BAR_array,1)]';
bar_std = std(BAR_array');
shadedErrorBar(x,mean(BAR_array'),bar_std')

subplot(1,2,2)
x = [1:size(IR_array,1)]';
IR_std = std(IR_array');
shadedErrorBar(x,mean(IR_array'),IR_std')

clear BAR_array IR_array

end


%% Figure 3 for IROS for Speed Data

% Navigate to 
% G:\My Drive\GE+\Teaching\Spring 2018\Humsini Acharya - Independent Study\
% Fingertip MTS Data\data\02.15.2018\Speed Data

clear all; close all; clc;

for i=4:10
filenames = dir;
raw = importdata(filenames(i).name);

IR = raw.data(:,2);
Bar = raw.data(:,3);
Time = cumsum(raw.data(3:end,1)); % sum loop time in ms


% Visualize

% figure(1)
% % subplot(4,2,i)
% plot(IR);
% title(strcat('IR - ',num2str(raw.data(6,6)),' (mm/s)'));
% % title(strcat(txt{1,2},' - ',filename{i}));
% figure(2)
% % subplot(4,2,i)
% plot(Bar);
% title(strcat('BAR - ',num2str(raw.data(6,6)),' (mm/s)'));
% % Peak Finder

% high_IR = max(IR);low_IR = min(IR);range_IR = high_IR-low_IR;
high_Bar = max(Bar);low_Bar = min(Bar);range_Bar = high_Bar-low_Bar;
% MPH_IR = high_IR-(0.1*range_IR);
MPH_Bar = high_Bar-(.5*range_Bar);

% [IR_pks,IR_loc] = findpeaks(IR,'MinPeakHeight',MPH_IR,...
%     'NPeaks',10,'MinPeakDistance',100);
[BAR_pks,BAR_loc] = findpeaks(Bar,'MinPeakHeight',MPH_Bar,...
    'NPeaks',10,'MinPeakDistance',75);
[IR_pks] = IR(BAR_loc); [IR_loc] = BAR_loc; % use BAR peaks for IR signal
% 
% figure(1)
% hold on
% % subplot(4,2,i)
% hold on
% scatter(IR_loc,IR_pks)
% 
% figure(2)
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

for j=1:size(IR_loc,1)-1
    start_index = IR_loc(j)-(round(peak_length/2));
    end_index = IR_loc(j)+(round(peak_length/2));
    IR_array(:,j)=IR(start_index:end_index);
end
for j=1:size(IR_loc,1)-1
    start_index = BAR_loc(j)-(round(peak_length/2));
    end_index = BAR_loc(j)+(round(peak_length/2));
    BAR_array(:,j)=Bar(start_index:end_index);
end


% Low Pass Filter
loop = raw.data(:,1);
a=1; % transfer function denom
windowSize = 10; %length of smoothing window
b = (1/windowSize)*ones(1,windowSize);

IR_filt = filter(b,a,IR);

% figure(i*10)
% plot(IR)
% hold on
% plot(IR_filt)

% Find Contact %
%%%%%%%%%%%%%%%%
% Threshold Method %
%%%%%%%%%%%%%%%%%%%

% Diffentiate filtered IR signal
IR_diff = abs([0;diff(IR)]./loop);
IR_2diff = ([0;diff(IR_diff)]./loop);
% Low Pass Filter
loop = raw.data(:,1);
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

% figure(i*100)
% subplot(3,1,1)
% plot(IR)
% hold on
% plot([contact(6:end);zeros(5,1)])
% subplot(3,1,2)
% plot(IR_diff)
% hold on
% plot(contact/1000)
% subplot(3,1,3)
% plot(IR_2diff)


% Plot Means with Shaded Error Bar %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



figure(10)
subplot(2,4,i-3)
yyaxis right
x = 0.001*mean(raw.data(5:end,1))*[1:size(BAR_array,1)]'; %time in seconds
% x = Time;
bar_std = std(BAR_array');
shadedErrorBar(x,mean(BAR_array'),bar_std','lineprops', '-r')
ylabel('Barometer Signal [millibar]')
ylim([1566 1595])

% subplot(1,2,2)
yyaxis left
% x = [1:size(IR_array,1)]';
% x = Time;
IR_std = std(IR_array');
shadedErrorBar(x,mean(IR_array'),IR_std','lineprops', '-b')
ylabel('IR Signal [counts]')
title(filenames(i).name)
xlabel('Time (s)')
ylim([0.6E4 1.8E4])


clear BAR_array IR_array

end

%% Figure 3 for IROS for Spatial Data

% Navigate to 
% G:\My Drive\GE+\Teaching\Spring 2018\Humsini Acharya - Independent Study\
%Fingertip MTS Data\data\02.22.2018

clear all; close all; clc;

filenames = dir;
names = {filenames(3:22).name};

plot_index = [1 2 3 4 5;
    6 7 8 9 10;
    11 12 13 14 15;
    16 17 18 19 20]; % 1N - 5N - 30N - 50N indices

for t=4%:4
    inc=1;
for i=plot_index(t,:)

    
raw = importdata(names{i});

IR = raw.data(:,2);
Bar = raw.data(:,3);
Loop = raw.data(:,1);
Time = cumsum(raw.data(3:end,1)); % sum loop time in ms

% Title Building %

forces = {'Force_1';'Force_5';'Force_30';'Force_50'};
forces_names = {'1N - ';'5N - ';'30N - ';'50N - '};
spatials = {'Spatial_0';'Spatial_1';'Spatial_2';'Spatial_3';'Spatial_4'};
spatials_names = {'Center';'Medial';'Distal';'Lateral';'Proximal'};


for f=1:4
    for s=1:5
        if strfind(names{i},spatials{s})
            Titles{2} = spatials_names{s};
        end
    end
    if strfind(names{i},forces{f})
        Titles{1} = forces_names{f};
    end
end

Title = strcat(Titles{1},Titles{2});

% Visualize

% figure(i)
% % subplot(4,2,i)
% plot(IR);
% title(strcat('IR - ',Title));
% % title(strcat(txt{1,2},' - ',filename{i}));
% figure(i*100)
% % subplot(4,2,i)
% plot(Bar);
% title(strcat('BAR - ',Title));
% Peak Finder

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

figure(i*100)
subplot(2,1,1)
plot(IR_array_mean)
hold on
% plot([contact(6:end);zeros(5,1)])
subplot(2,1,2)
plot(IR_diff)
hold on
% plot(contact/1000)


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
position = [5,4,2,6,8];

ylim_IR = {[5500 11000];[6000 15000];[6000 15000];[7000 13000]};
ylim_Bar = {[1580 1680];[1600 2500];[1600 3500];[1600 1900]};

figure(t)
subplot(3,3,position(inc))
yyaxis right
x = 0.001*mean(raw.data(5:end,1))*[1:size(BAR_array,1)]'; %time in seconds
% x = Time;
bar_std = std(BAR_array');
shadedErrorBar(x,mean(BAR_array'),bar_std','lineprops', '-r')
ylabel('Barometer Signal [millibar]')
ylim(ylim_Bar{t});
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
xlim([0 15]);
ylim(ylim_IR{t});
% ylim([0.6E4 1.8E4])

% subplot(1,3,2)
% plot(BAR_array)
% subplot(1,3,3)
% plot(IR_array)

clear BAR_array IR_array BAR_loc

inc = inc+1;

end

end






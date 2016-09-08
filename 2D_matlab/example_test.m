% Robotics: Estimation and Learning 
% WEEK 3
% 
% This script is to help run your algorithm and visualize the result from it.
% 
% Please see example_lidar first to understand the lidar measurements, 
% and see example_bresenham to understand how to use it.
clear all;
close all;

load practice.mat 
% This will load four variables: ranges, scanAngles, t, pose
% [1] t is K-by-1 array containing time in second. (K=3701)
%     You may not use time info for implementation.
% [2] ranges is 1081-by-K lidar sensor readings. 
%     e.g. ranges(:,k) is the lidar measurement (in meter) at time index k.
% [3] scanAngles is 1081-by-1 array containing at what angles (in radian) the 1081-by-1 lidar
%     values ranges(:,k) were measured. This holds for any time index k. The
%     angles are with respect to the body coordinate frame.
% [4] pose is 3-by-K array containing the pose of the mobile robot over time. 
%     e.g. pose(:,k) is the [x(meter),y(meter),theta(in radian)] at time index k.
param.fig = 1;
% 1. Decide map resolution, i.e., the number of grids for 1 meter.
param.resol = 16;

% 2. Decide the initial map size in pixels
param.size = [ceil(900/25*param.resol), ceil(900/25*param.resol)];

% 3. Indicate where you will put the origin in pixels
param.origin = [ceil(700/25*param.resol),ceil(600/25*param.resol)]'; 

% 4. Log-odd parameters 
param.lo_occ = 0.6;
param.lo_free = 0.1; 
param.lo_max = 100;
param.lo_min = -100;

param.ISM = 'AISM';
param.sigma = 0.03;
param.rho = 0.6;
param.rangelim = 0.1;

% Call your mapping function here.
% Running time could take long depending on the efficiency of your code.
% For a quicker test, you may take some hundreds frames as input arguments as
% shown.
numFrame = 1:120:length(pose);
if param.fig == 1
    figure('units','normalized','outerposition',[0 0 1 1]);
    a = subplot(1,2,1);
end
% subaxis(5,5,i, 'Spacing', 0.03, 'Padding', 0, 'Margin', 0);
[myMap1, H1, IG1]= occGridMapping(ranges(:,numFrame), scanAngles,...
    pose(:,numFrame), param);
% title(a,'Approximate inverse sensor model');

param.ISM = 'EISM';
if param.fig ==1
    b = subplot(1,2,2);
end
[myMap2, H2, IG2]= occGridMapping(ranges(:,numFrame), scanAngles,...
    pose(:,numFrame), param);
% title(b, 'Exact inverse sensor model');

%%


figure;
a = subplot(1,2,1);
imagesc(1-myMap1);colormap('gray');axis equal;ylim([0 ceil(900/25*param.resol)]);
title(a,'Approximate inverse sensor model');
b = subplot(1,2,2);
imagesc(1-myMap2);colormap('gray');axis equal;ylim([0 ceil(900/25*param.resol)]);
title(b,'Exact inverse sensor model')

print('map_comparison','-dpng');
figure;
rangex = ceil(413/25*param.resol):ceil(560/25*param.resol);
rangey = ceil(20/25*param.resol):ceil(190/25*param.resol);
a = subplot(1,2,1);
imagesc(1-myMap1(rangey,rangex));colormap('gray');axis equal;
ylim([0 ceil(170/25*param.resol)]);title(a,'Approximate inverse sensor model')
b = subplot(1,2,2);
imagesc(1-myMap2(rangey,rangex));colormap('gray');axis equal;
ylim([0 ceil(170/25*param.resol)]);title(b,'Exact inverse sensor model')

print('map_zoom','-dpng');

% The final grid map: 
% figure
% imagesc(myMap1); 
% colormap(flipud(gray)); axis equal;
figure;

plot(H1,'b+-');hold on;
plot(H2,'r+-');grid on;
legend('AISM','EISM');title('Entropy comparison');
print('entropy','-dpng');
% information gain
figure;
plot(IG1,'b+-');hold on;
plot(IG2,'r+-');grid on;
legend('AISM','EISM');
print('infogain','-dpng');

% colormap('gray'); axis equal;

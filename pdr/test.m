global simdata;

simdata.path='data_set_1/';
u = load_dataset();


function [u]=load_dataset()


global simdata;

% Load inertial data
data_inert_file = fopen( [simdata.path '2022_1_12_straight_walk_chest2-001.txt'], 'r');

fscanf(data_inert_file, '%s', [1 14]);

data_inert = fscanf(data_inert_file, '%f  %f %f %f  %f %f %f  %f %f  %f  %f %f %f %f %f %f %f', [14 inf])';

fclose(data_inert_file);
clear data_inert_file;

Acc_imu = data_inert(:,3:5);
omega_imu = data_inert(:,6:8);
ori = data_inert(:,9:11);
gps = data_inert(:,12:14);

% %GPS转化为平面坐标系
% n = length(Acc_imu);
% xyz = zeros(3,n);
% original = [gps(1,1), gps(2,1), gps(3,1)];
% for k = 1:n
%     [xyz(1,k),xyz(2,k), xyz(3,k)] = latlon2local(gps(1,k),gps(2,k),gps(3,k),original);
% end
% xyz(1,:) = -xyz(1,:);
% 
% rx0 = 0;
% ry0 = 0;
% angle=-91.5;
% %angle = -252;
% angle=pi/180*angle;
% for i=1:length(xyz)
%     x0=(xyz(2,i)-rx0)*cos(angle)-(xyz(1,i)-ry0)*sin(angle)+rx0;
%     y0=(xyz(2,i)-rx0)*sin(angle)+(xyz(1,i)-ry0)*cos(angle)+ry0;
%     xyz(2,i)=x0;
%     xyz(1,i)=y0;
% end

u=[Acc_imu omega_imu ori gps];

end
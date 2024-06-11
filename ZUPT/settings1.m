function u=settings1()
%常规数据设置
global simdata;
% Rough altitude [m] 
simdata.altitude=100;

% Rough latitude [degrees]
simdata.latitude=39.97;

%重力加速度
simdata.g=9.8015;
%采样频率
simdata.Ts=1/399.5;
%simdata.Ts = 400;

%IMU路径
simdata.path='data_set_1/';
%simdata.path='data_set_2\';

%载入数据
u=load_dataset();

%初始化
simdata.init_heading=0*pi/180;
simdata.init_pos=[0 0 0]';


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%           Detector Settings             %% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Detector type to be used. You can chose between: 
% GLRT - Generalized likelihood ratio test
% MV -  Accelerometer measurements variance test
% MAG - Accelerometer measurements magnitude test
% ARE - Angular rate measurement energy test 
simdata.detector_type='GLRT';
%simdata.detector_type='MV';
%simdata.detector_type='MAG';
%simdata.detector_type='ARE';


%加速度噪音误差
simdata.sigma_a=0.08; 

%陀螺仪噪声误差
simdata.sigma_g=0.044*pi/180;     


%滑动窗口大小
simdata.Window_size=3;

%阈值
%simdata.gamma=600000; 
simdata.gamma=600000; 

%滤波器设置
%用于建模加速计噪声（x、y、z平台坐标轴）和其他加速计误差的过程噪声[m/s^2]。
simdata.sigma_acc =0.15*[1 1 1]';

%建模陀螺仪噪声（x、y、z平台坐标轴）和其他陀螺仪误差的过程噪声[rad/s]。
%simdata.sigma_gyro =1.1*[1 1 1]'*pi/180; 
%simdata.sigma_gyro =1.5*[1 1 1]'*pi/180; 
simdata.sigma_gyro =6.5*[1 1 1]'*pi/180; 

%更新测量噪声协方差R。协方差矩阵假定为对角矩阵。[m/s]
simdata.sigma_vel=[0.012 0.012 0.012];      

%初始状态协方差矩阵P的对角元素   
simdata.sigma_initial_pos=1e-7*ones(3,1);               %位置[m]
simdata.sigma_initial_vel=1e-7*ones(3,1);               % 速度[m/s]
simdata.sigma_initial_att=(pi/180*[0.001 0.001 0.001]');      % 姿态（侧倾、俯仰、航向）(roll,pitch,heading) [rad]    

end

%导入数据的函数
function [u]=load_dataset()


global simdata;

% Load inertial data
%data_inert_file = fopen( [simdata.path 'data_inert_left.txt'], 'r');
%data_inert_file = fopen( [simdata.path '2022_1_12_stair_up_walk_foot2-000.txt'], 'r');
%data_inert_file = fopen( [simdata.path '2022_1_12_playground_walk_foot1-000(1).txt'], 'r');
data_inert_file = fopen( [simdata.path '2022_1_12_playground_walk_foot1-001.txt'], 'r');
%data_inert_file = fopen( [simdata.path '2022_1_12_straight_walk_foot2-000.txt'], 'r');
%data_inert_file = fopen( [simdata.path '2022_1_12_straight_walk_70_foot1-000(1).txt'], 'r');
%data_inert_file = fopen( [simdata.path '2022_1_12_straight_walk_foot3-000(1).txt'], 'r');

%data_inert_file = fopen( [simdata.path '2022_1_14_hallway_walk2-001(1).txt'], 'r');
%data_inert_file = fopen( [simdata.path '2022_1_14_hallway_walk1-002(1).txt'], 'r');
%data_inert_file = fopen( [simdata.path '2022_1_12_straight_walk_foot1-000.txt'], 'r');
%data_inert_file = fopen( [simdata.path '2022_1_13_straight_walk_foot1-000.txt'], 'r');
%data_inert_file = fopen( [simdata.path '2022_1_14_hallway_walk3-001.txt'], 'r');

fscanf(data_inert_file, '%s', [1 11]);

data_inert = fscanf(data_inert_file, '%f  %f %f %f  %f %f %f  %f %f  %f  %f %f %f %f ', [11 inf])';

fclose(data_inert_file); clear data_inert_file;

Acc_imu = data_inert(:,3:5)';
omega_imu = data_inert(:,6:8)';
gps = data_inert(:,9:11)';

%GPS转化为平面坐标系
n = length(Acc_imu);
xyz = zeros(3,n);
original = [gps(1,1), gps(2,1), gps(3,1)];
for k = 1:n
    [xyz(1,k),xyz(2,k), xyz(3,k)] = latlon2local(gps(1,k),gps(2,k),gps(3,k),original);
end
xyz(1,:) = -xyz(1,:);

rx0 = 0;
ry0 = 0;
angle=-91.5;
%angle = -252;
%angle = -80;
%angle = -72;
angle=pi/180*angle;
for i=1:length(xyz)
    x0=(xyz(2,i)-rx0)*cos(angle)-(xyz(1,i)-ry0)*sin(angle)+rx0;
    y0=(xyz(2,i)-rx0)*sin(angle)+(xyz(1,i)-ry0)*cos(angle)+ry0;
    xyz(2,i)=x0;
    xyz(1,i)=y0;
end

u=[Acc_imu; omega_imu;xyz];

end
    

    



function u=settings1()
%������������
global simdata;
% Rough altitude [m] 
simdata.altitude=100;

% Rough latitude [degrees]
simdata.latitude=39.97;

%�������ٶ�
simdata.g=9.8015;
%����Ƶ��
simdata.Ts=1/399.5;
%simdata.Ts = 400;

%IMU·��
simdata.path='data_set_1/';
%simdata.path='data_set_2\';

%��������
u=load_dataset();

%��ʼ��
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


%���ٶ��������
simdata.sigma_a=0.08; 

%�������������
simdata.sigma_g=0.044*pi/180;     


%�������ڴ�С
simdata.Window_size=3;

%��ֵ
%simdata.gamma=600000; 
simdata.gamma=600000; 

%�˲�������
%���ڽ�ģ���ټ�������x��y��zƽ̨�����ᣩ���������ټ����Ĺ�������[m/s^2]��
simdata.sigma_acc =0.15*[1 1 1]';

%��ģ������������x��y��zƽ̨�����ᣩ���������������Ĺ�������[rad/s]��
%simdata.sigma_gyro =1.1*[1 1 1]'*pi/180; 
%simdata.sigma_gyro =1.5*[1 1 1]'*pi/180; 
simdata.sigma_gyro =6.5*[1 1 1]'*pi/180; 

%���²�������Э����R��Э�������ٶ�Ϊ�ԽǾ���[m/s]
simdata.sigma_vel=[0.012 0.012 0.012];      

%��ʼ״̬Э�������P�ĶԽ�Ԫ��   
simdata.sigma_initial_pos=1e-7*ones(3,1);               %λ��[m]
simdata.sigma_initial_vel=1e-7*ones(3,1);               % �ٶ�[m/s]
simdata.sigma_initial_att=(pi/180*[0.001 0.001 0.001]');      % ��̬�����㡢����������(roll,pitch,heading) [rad]    

end

%�������ݵĺ���
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

%GPSת��Ϊƽ������ϵ
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
    

    



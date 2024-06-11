global simdata;
simdata.path='data_set_1/';

%加载数据
disp('导入数据')
u = load_dataset();

NE_ini(1,:) = [0,0]; %起始位置
delta_yaw1=0; %数学 得儿塔yaw
steptime = 0.2; 

deg2rad = pi/180;
%g0 = [0;0;9.8015];
g0 = [9.8015;0;0];

%提取数据
% acc0 = load('acc.txt');
% gyr0 = load('gyr.txt');
% ori0 = load('ori.txt');
% %coo = load('coo.txt');
% acc0 = acc0(:,1:3);
% gyr0 = gyr0(:,1:3);
% ori0 = ori0(:,1:3);

acc0 = u(:,1:3);
gyr0 = u(:,4:6);
ori0 = u(:,7:9);

roll  = ori0(:,1) * deg2rad;
pitch = ori0(:,2) * deg2rad;
yaw   = ori0(:,3) * deg2rad;

N=length(acc0);
t=0:1/400:(N-1)/400;

figure(1111)
plot(t, acc0(:,1));
title('\fontname{Times New Roman}x\fontname{Songti SC}轴加速度');
xlabel('\fontname{Songti SC}时间\fontname{Times New Roman} [s]')
ylabel('\fontname{Songti SC}加速度\fontname{Times New Roman} [m/s^2]')


figure(111)
plot(t,acc0(:,1:3));
title('\fontname{Songti SC}三轴加速度');
xlabel('\fontname{Songti SC}时间\fontname{Times New Roman} [s]')
ylabel('\fontname{Songti SC}加速度\fontname{Times New Roman} [m/s^2]')
legend('\fontname{Times New Roman}x-axis','\fontname{Times New Roman}y-axis','\fontname{Times New Roman}z-axis')

figure(2)
plot(yaw/deg2rad,'b');
legend('\fontname{Songti SC}航向角');
title('\fontname{Songti SC}校正前');
grid on;

%拆分
gyr_x = gyr0(:,1);
gyr_y = gyr0(:,2);
gyr_z=gyr0(:,3);

len_eul = length(ori0(:,1));
g       = zeros(len_eul   ,3); % 重力对acc的影响
a_xyz   = zeros(len_eul   ,3); % acc remove g
x       = zeros(len_eul-2 ,3); % coordinate 坐标

%删除acc中传感器的重力
for i = 1:len_eul
    eul_vect(1) =  roll(i) ;
    eul_vect(2) = pitch(i) ;
    eul_vect(3) =   yaw(i) ;
    DCM_n2b = eulr2dcm(eul_vect);
    C = DCM_n2b;
    g(i,:) = C * g0;%基于MEMS与智能手机电子罗盘的室内定位P50，(3.10)
     a_xyz(i,:) = acc0(i,:) - g(i,:);
%     a_xyz(i,:) = acc0(i,:) + g(i,:);
    a_ned(i,:) = C' * a_xyz(i,:)';
end

%acc = acc0(:,3)-mean(acc0(:,3));
acc = acc0(:,1)-mean(acc0(:,1));

%滤去噪音信号
fs = 50; % Hz采样频率 
n = 10; % 滤波器阶数
f = [0 2 3 50]/fs; % 归一化频率向量,频率系数
a = [1 1 0 0]; % 振幅向量，放大特性
b = firls(n,f,a); % 采用firls设计滤波器
[h,w] = freqz(b); % 计算其频率响应

% figure(3)
% plot(w/pi,abs(h),'r')%画出幅频响应曲线
% xlabel('归一化频率');ylabel('振幅');
% legend('firls'); % 给出图例
% grid on;

% figure(4)%acc滤掉重力图
% plot(a_xyz(:,1:3));
% xlabel('加速度数')
% ylabel('幅度 [m/s^2]')
% title('加速度去除重力影响')
% legend('x-axis','y-axis','z-axis')
% box on
% grid on

% % % % % % % % % % % % %
len = length(a_xyz(:,1));
a_module = zeros(len,1);
for i = 1:len
   a_module(i) = sqrt(a_xyz(i,1)^2+a_xyz(i,2)^2+a_xyz(i,3)^2);
end
% figure (6)
% plot(a_module);
% xlabel('samples')
% ylabel('幅度[m/s^2]')
% title('矢量和加速度信号')

% % % % % % % % % % % % % % % % % % % % 
% figure(7) 
% subplot(1,2,1);
% plot(acc(4000:5000,:));
% xlabel('samples')
% ylabel('幅度[m/s^2]')
% title('原始信号')
% subplot(1,2,2);
%  ACC=fft(acc(4000:5000,:));
%  plot(abs(ACC));
% title('原始信号频谱');
% 
% y = filter(b,1,a_module);
y = filter(b,1,acc);
% y(6512)=-1.65;
% y(6533)=2.1;
% y(6534)=1.7;
% y(6906) =  0.56;
%figure(8)
% plot(y(1200:1550))
% subplot(1,2,1);

% plot(y(4000:5000,:))
% xlabel('samples')
% ylabel('幅度[m/s^2]')
% title('低通滤波信号')
% 
% figure(9)%
% plot(acc(4000:5000))
% xlabel('samples')
% ylabel('幅度[m/s^2]')
% title('矢量和加速度信号')
%

yaw = map_aid(ori0,gyr0)*deg2rad;

figure(10)
plot(yaw/deg2rad,'b');
xlabel('\fontname{Songti SC}加速度数')
legend('\fontname{Songti SC}航向角')
title('\fontname{Songti SC}校正后');
grid on;

%% step number detection
ap = 0.5;
[pk_num, vy_num, stp_num, pk_ti, vy_ti, py_flg] = IMU_firls_stp_num(y, ap);
%% step length detection
[stp_len, dif_acc, s2] = IMU_firls_stp_len(stp_num, py_flg,acc);
% [stp_len, dif_acc, s2] = IMU_firls_stp_len(stp_num, py_flg,a_xyz);
%% step orientation estimation and location
% [stp_ori, pk_pos, vy_pos,NE] = IMU_firls_stp_ori_loc(stp_num, stp_len, py_flg, yaw, gyr0,spl_time, NE_ini);
[stp_ori, STP_ORI, pk_pos, vy_pos,NE] = IMU_firls_stp_ori_loc(stp_num, stp_len, py_flg, yaw, NE_ini);
% len = length(NE(:,1));
% [E_part, N_part] = part(NE(:,2),NE(:,1),len);

%gps转换
n = length(u(:,1));
xyz = zeros(n,3);
original = [u(1,10), u(1,11), u(1,12)];
for k = 1:n
    [xyz(k,1),xyz(k,2), xyz(k,3)] = latlon2local(u(k,10),u(k,11),u(k,12),original);
end

%xyz(:,1) = -xyz(:,1);


pi = 3.1415926;
rx0 = 0;
ry0 = 0;

%angle=150.5;
%angle = -54;
angle=128;
angle=angle*pi/180;
cos(angle)
sin(angle)
for i=1:length(xyz)
    x0=(xyz(i,2)-rx0)*cos(angle)-(xyz(i,1)-ry0)*sin(angle)+rx0;
    y0=(xyz(i,2)-rx0)*sin(angle)+(xyz(i,1)-ry0)*cos(angle)+ry0;
    xyz(i,2)=x0;
    xyz(i,1)=y0;
end



%绘制轨迹图
figure(11);
plot(NE(:,2),NE(:,1),'g-')
hold on
plot(xyz(:,2), xyz(:,1),'b-');
legend('\fontname{Songti SC}行人航位推算\fontname{Times New Roman}PDR\fontname{Songti SC}算法','\fontname{Times New Roman}GPS\fontname{Songti SC}轨迹')
xlabel('\fontname{Times New Roman}x[m]')
ylabel('\fontname{Times New Roman}y[m]')
%axis([-40 20 -10 50]);
title('\fontname{Songti SC}二维平面坐标系')
 
 function [u]=load_dataset()
global simdata;
% Load inertial data
%data_inert_file = fopen( [simdata.path '2022_1_12_straight_walk_chest1-000.txt'], 'r');
%data_inert_file = fopen( [simdata.path '2022_1_12_straight_walk_chest2-001.txt'], 'r');
data_inert_file = fopen( [simdata.path '2022_1_12_straight_walk_chest3-000.txt'], 'r');

fscanf(data_inert_file, '%s', [1 14]);

data_inert = fscanf(data_inert_file, '%f  %f %f %f  %f %f %f  %f %f  %f  %f %f %f %f %f %f %f', [14 inf])';

fclose(data_inert_file);
clear data_inert_file;

Acc_imu = data_inert(:,3:5);
omega_imu = data_inert(:,6:8);
ori = data_inert(:,9:11);
gps = data_inert(:,12:14);

%u_original=[Acc_imu omega_imu ori gps];
u = [Acc_imu omega_imu ori gps];
%去掉前期的无用数据
% n = 150; %无用数据量
% N = length(u_original);
% u = zeros(N-n,12);
% for i=1:N-n
%     u(i,:) = u_original(i+n,:);
% end


end


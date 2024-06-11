global simdata;
simdata.path='data_set_1/';

%��������
disp('��������')
u = load_dataset();

NE_ini(1,:) = [0,0]; %��ʼλ��
delta_yaw1=0; %ǰ����ǵĲ�ֵ
steptime = 0.2; 

deg2rad = pi/180;  %�Ƕ�ת����
%g0 = [0;0;9.8015];
g0 = [9.8015;0;0]; %������������

%��ȡ����(���ٶȼơ������ǡ�����)
acc0 = u(:,1:3);
gyr0 = u(:,4:6);
ori0 = u(:,7:9);

%תΪ����
roll  = ori0(:,1) * deg2rad;
pitch = ori0(:,2) * deg2rad;
yaw   = ori0(:,3) * deg2rad;

N=length(acc0);
t=0:1/400:(N-1)/400;

% figure(1111)
% plot(t, acc0(:,1));
% title('x����ٶ�');
% xlabel('ʱ�� [s]')
% ylabel('���ٶ� [m/s^2]')


figure(111)
plot(t,acc0(:,1:3));
title('\fontname{Songti SC}������ٶ�');
xlabel('\fontname{Songti SC}ʱ��\fontname{Times New Roman} [s]')
ylabel('\fontname{Songti SC}���ٶ�\fontname{Times New Roman} [m/s^2]')
legend('\fontname{Times New Roman}x-axis','\fontname{Times New Roman}y-axis','\fontname{Times New Roman}z-axis')

%����ǰ�����
figure(2)
plot(yaw/deg2rad,'b');
legend('\fontname{Songti SC}�����');
xlabel('\fontname{Songti SC}���ٶ���')
ylabel('\fontname{Songti SC}�Ƕ�\fontname{Times New Roman} [deg]')
title('\fontname{Songti SC}У��ǰ');
grid on;

%���������xyz
gyr_x = gyr0(:,1);
gyr_y = gyr0(:,2);
gyr_z=gyr0(:,3);

%ŷ���ǳ��Ȳ���ʼ��
len_eul = length(ori0(:,1));
g       = zeros(len_eul   ,3); % ������acc��Ӱ��
a_xyz   = zeros(len_eul   ,3); % acc remove g
x       = zeros(len_eul-2 ,3); % coordinate ����

%ɾ�����ٶȼ���x�������
for i = 1:len_eul
    eul_vect(1) =  roll(i) ;
    eul_vect(2) = pitch(i) ;
    eul_vect(3) =   yaw(i) ;
    %ŷ����ת���Ҿ���
    DCM_n2b = eulr2dcm(eul_vect);
    C = DCM_n2b;
    g(i,:) = C * g0;
    a_xyz(i,:) = acc0(i,:) - g(i,:);
%   a_xyz(i,:) = acc0(i,:) + g(i,:);
    a_ned(i,:) = C' * a_xyz(i,:)';
end

%acc = acc0(:,3)-mean(acc0(:,3));
acc = acc0(:,1)-mean(acc0(:,1));

%��ȥ�����ź�
fs = 50; % Hz����Ƶ�� 
n = 10; % �˲�������
f = [0 2 3 50]/fs; % ��һ��Ƶ������,Ƶ��ϵ��
a = [1 1 0 0]; % ����������Ŵ�����
b = firls(n,f,a); % ����firls����˲���
[h,w] = freqz(b); % ������Ƶ����Ӧ

figure(4)%acc�˵�����ͼ
plot(a_xyz(:,1:3));
xlabel('���ٶ���')
ylabel('���� [m/s^2]')
title('���ٶ�ȥ������Ӱ��')
legend('x-axis','y-axis','z-axis')
box on
grid on

% % % % % % % % % % % % %
len = length(a_xyz(:,1));
a_module = zeros(len,1);
for i = 1:len
   a_module(i) = sqrt(a_xyz(i,1)^2+a_xyz(i,2)^2+a_xyz(i,3)^2);
end

% % % % % % % % % % % % % % % % % % % % 
y = filter(b,1,acc);

yaw = map_aid(ori0,gyr0)*deg2rad;

figure(10)
plot(yaw/deg2rad,'b');
xlabel('\fontname{Songti SC}���ٶ���')
ylabel('\fontname{Songti SC}�Ƕ�\fontname{Times New Roman} [deg]')
legend('\fontname{Songti SC}�����')
title('\fontname{Songti SC}У����');
grid on;

%��̬���
ap = 0.6;
[pk_num, vy_num, stp_num, pk_ti, vy_ti, py_flg] = IMU_firls_stp_num(y, ap);

%��¼��ֵ
pk_pos  = find(py_flg(:,1)~=0);  % pos1 ��ʾ����λ��
pk_index = zeros(length(pk_pos),1);
for i = 1: length(pk_pos)
    pk_index(i) = y(pk_pos(i));
end

%��������
[stp_len, dif_acc, s2] = IMU_firls_stp_len(stp_num, py_flg,acc);
% [stp_len, dif_acc, s2] = IMU_firls_stp_len(stp_num, py_flg,a_xyz);

%�������
% [stp_ori, pk_pos, vy_pos,NE] = IMU_firls_stp_ori_loc(stp_num, stp_len, py_flg, yaw, gyr0,spl_time, NE_ini);
[height,stp_ori, STP_ORI, pk_pos, vy_pos,NE, myaw] = IMU_firls_stp_ori_loc(stp_num, stp_len, py_flg, yaw, NE_ini, pk_index);
%������ʼ��
init = [0 0];
NE = [init; NE];
height = [0;height];

% len = length(NE(:,1));
% [E_part, N_part] = part(NE(:,2),NE(:,1),len);


%GPS_get = input('�Ƿ���GPS�ο�������0��1��')

% %gpsת��
% n = length(u(:,1));
% xyz = zeros(n,3);
% original = [u(1,10), u(1,11), u(1,12)];
% for k = 1:n
%     [xyz(k,1),xyz(k,2), xyz(k,3)] = latlon2local(u(k,10),u(k,11),u(k,12),original);
% end
% 
% pi = 3.1415926;
% rx0 = 0;
% ry0 = 0;
% angle=122;  %��˳����
% %angle=-90;
% %angle = -252;
% angle=angle*pi/180;
% cos(angle)
% sin(angle)
% for i=1:length(xyz)
%     x0=(xyz(i,2)-rx0)*cos(angle)-(xyz(i,1)-ry0)*sin(angle)+rx0;
%     y0=(xyz(i,2)-rx0)*sin(angle)+(xyz(i,1)-ry0)*cos(angle)+ry0;
%     xyz(i,2)=x0;
%     xyz(i,1)=y0;
% end
% 
% %���ƹ켣ͼ
% figure(11);
% plot3(NE(:,2),NE(:,1),height,'g-')
% hold on
% plot3(xyz(:,2), xyz(:,1),xyz(:,3),'b-');
% legend('���˺�λ����PDR�㷨','��ʵ�켣')
% xlabel('x[m]')
% ylabel('y[m]')
% zlabel('z[m]')
% %axis([-10 80 -10 80]);
% title('��άƽ������ϵ')
% 
% else
figure(11);
plot3(NE(:,2),NE(:,1),height,'g-')
hold on;
plot3(NE(1,2),NE(1,1),height(1),'rs')
plot3(NE(length(NE),2),NE(length(NE),1),height(length(height)),'gs')
legend('\fontname{Songti SC}���˺�λ����\fontname{Times New Roman}PDR\fontname{Songti SC}�㷨', '\fontname{Times New Roman}Start point', '\fontname{Times New Roman}End point')
xlabel('\fontname{Times New Roman}x[m]')
ylabel('\fontname{Times New Roman}y[m]')
zlabel('\fontname{Times New Roman}z[m]')
axis([-2 2 -5 1 -10 1])
%axis([-10 80 -10 80]);
title('\fontname{Songti SC}��άƽ������ϵ')


 
function [u]=load_dataset()
global simdata;
% Load inertial data
%data_inert_file = fopen( [simdata.path '2022_1_12_stair_down_walk_chest_1-001.txt'], 'r');
%data_inert_file = fopen( [simdata.path '2022_1_12_straight_walk_chest2-001.txt'], 'r');
%data_inert_file = fopen( [simdata.path '2022_1_12_stair_up_walk_chest1-000.txt'], 'r');

data_inert_file = fopen( [simdata.path '2022_1_14_stair_down_walk_chest1-000.txt'], 'r');
%data_inert_file = fopen( [simdata.path '2022_1_14_stair_down_walk_chest2-000.txt'], 'r');
%data_inert_file = fopen( [simdata.path '2022_1_14_stair_down_walk_chest3-000.txt'], 'r');

%data_inert_file = fopen( [simdata.path '2022_1_14_stair_up_walk_chest1-000.txt'], 'r');
%data_inert_file = fopen( [simdata.path '2022_1_14_stair_up_walk_chest2-000.txt'], 'r');
%data_inert_file = fopen( [simdata.path '2022_1_14_stair_up_walk_chest3-000.txt'], 'r');

fscanf(data_inert_file, '%s', [1 11]);

data_inert = fscanf(data_inert_file, '%f  %f %f %f  %f %f %f  %f %f  %f  %f %f %f %f', [11 inf])';

fclose(data_inert_file);
clear data_inert_file;

Acc_imu = data_inert(:,3:5);
omega_imu = data_inert(:,6:8);
ori = data_inert(:,9:11);
%gps = data_inert(:,12:14);

%u_original=[Acc_imu omega_imu ori gps];
u = [Acc_imu omega_imu ori];
%ȥ��ǰ�ڵ���������
% n = 150; %����������
% N = length(u_original);
% u = zeros(N-n,12);
% for i=1:N-n
%     u(i,:) = u_original(i+n,:);
% end


end


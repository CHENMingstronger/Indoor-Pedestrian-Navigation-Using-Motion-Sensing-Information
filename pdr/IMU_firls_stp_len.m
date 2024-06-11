function [stp_len, dif_acc, s2] = IMU_firls_stp_len(stp_num, flg,acc)

%{
%   INPUTS
%       stp_num       :  stp_num_detection()函数计算得到的步数
%       py_flg        :  stp_num_detection()函数获取的波峰波谷幅值

%   OUTPUTS
%       stp_len      ： 估计得到的步长
%       Distance     ： 估计的步长累加值
%}

stp_len    = zeros(stp_num,1);
stp_len1    = zeros(stp_num,1);
stp_len2    = zeros(stp_num,1);
stp_len3    = zeros(stp_num,1);

dif_acc    = zeros(stp_num,1);
pos1 = find(flg(:,1)~=0);  % pos1 表示波峰位置
pos2 = find(flg(:,2)~=0);  % pos1 表示波谷位置
sumpk = 0;
distance = 0;
% % % % % % % % % % % % % % 
for i = 1:length(pos1)
        sumpk = sumpk + flg(pos1(i),1);
end
mPk = sumpk / length(pos1);
% K = 0.087 * mPk + 0.175
K = 0.06 * mPk + 0.41;%mPk为波峰平均数值
K1 = 0.06 * mPk + 0.9
 K2 = 0.06 * mPk + 0.8
% K = 0.47;
% % % % % % % % % % % % % % 
for i = 1:stp_num
    sum = 0;
    for j = pos2(i):pos1(i)
       sum = sum +acc(j); %单步中采样点的合加速度
    end
    mAcc(i) = sum/(pos1(i)-pos2(i));%pos1(i)-pos2(i)是指单步加速度采样点个数
end
% % % % % % % % % % % % % % %  
for i = 1:stp_num
    dif_acc(i) = abs(flg(pos1(i),1) - flg(pos2(i),2));
%     K = 0.087 * dif_acc(i)/2 + 0.25;
    stp_len(i) = K * ((dif_acc(i))^(1/4));%基于经验公式获得的计算公式
    %stp_len(i) = K * ((dif_acc(i)-1.5)^(1/4));
    stp_len1(i) = 0.7; %直接常数
    stp_len2(i) = K1 * (mAcc(i)-flg(pos2(i),2))/dif_acc(i);
    stp_len3(i) = K2 * mAcc(i).^(1/3); %基于行为模型的步长估计经验
   % stp_len4(i)=K*(dif_acc(i)*3.5+(dif_acc(i))^(1/4));
    distance = distance + stp_len1(i);
end
% % % % % % % % % % 
    s2 = distance;
% % % % % % % % % % 
figure(1)
plot(1:stp_num,stp_len,'r',1:stp_num,stp_len2,'b',1:stp_num,stp_len3,'k')
xlabel('\fontname{Songti SC}步伐数');
ylabel('\fontname{Songti SC}步长\fontname{Times New Roman} [m]');
legend('\fontname{Songti SC}基于经验公式的步长估计','\fontname{Songti SC}基于生物模型的步长估计','\fontname{Songti SC}基于行为模型的步长估计');

%% 计算距离    假设是直线运动
 %figure
 %plot(1:stp_num,stp_len,'r--')
 %title('步长');
 %figure
 %plot(1:stp_num,stp_len2,'b--')
 %title('步长');
 %figure
 %plot(1:stp_num,stp_len3,'k--')
 %title('步长');
% Distance = sum(stp_len); 
function [stp_len, dif_acc, s2] = IMU_firls_stp_len(stp_num, flg,acc)

%{
%   INPUTS
%       stp_num       :  stp_num_detection()��������õ��Ĳ���
%       py_flg        :  stp_num_detection()������ȡ�Ĳ��岨�ȷ�ֵ

%   OUTPUTS
%       stp_len      �� ���Ƶõ��Ĳ���
%       Distance     �� ���ƵĲ����ۼ�ֵ
%}

stp_len    = zeros(stp_num,1);
stp_len1    = zeros(stp_num,1);
stp_len2    = zeros(stp_num,1);
stp_len3    = zeros(stp_num,1);

dif_acc    = zeros(stp_num,1);
pos1 = find(flg(:,1)~=0);  % pos1 ��ʾ����λ��
pos2 = find(flg(:,2)~=0);  % pos1 ��ʾ����λ��
sumpk = 0;
distance = 0;
% % % % % % % % % % % % % % 
for i = 1:length(pos1)
        sumpk = sumpk + flg(pos1(i),1);
end
mPk = sumpk / length(pos1);
% K = 0.087 * mPk + 0.175
K = 0.06 * mPk + 0.41;%mPkΪ����ƽ����ֵ
K1 = 0.06 * mPk + 0.9
 K2 = 0.06 * mPk + 0.8
% K = 0.47;
% % % % % % % % % % % % % % 
for i = 1:stp_num
    sum = 0;
    for j = pos2(i):pos1(i)
       sum = sum +acc(j); %�����в�����ĺϼ��ٶ�
    end
    mAcc(i) = sum/(pos1(i)-pos2(i));%pos1(i)-pos2(i)��ָ�������ٶȲ��������
end
% % % % % % % % % % % % % % %  
for i = 1:stp_num
    dif_acc(i) = abs(flg(pos1(i),1) - flg(pos2(i),2));
%     K = 0.087 * dif_acc(i)/2 + 0.25;
    stp_len(i) = K * ((dif_acc(i))^(1/4));%���ھ��鹫ʽ��õļ��㹫ʽ
    %stp_len(i) = K * ((dif_acc(i)-1.5)^(1/4));
    stp_len1(i) = 0.7; %ֱ�ӳ���
    stp_len2(i) = K1 * (mAcc(i)-flg(pos2(i),2))/dif_acc(i);
    stp_len3(i) = K2 * mAcc(i).^(1/3); %������Ϊģ�͵Ĳ������ƾ���
   % stp_len4(i)=K*(dif_acc(i)*3.5+(dif_acc(i))^(1/4));
    distance = distance + stp_len1(i);
end
% % % % % % % % % % 
    s2 = distance;
% % % % % % % % % % 
figure(1)
plot(1:stp_num,stp_len,'r',1:stp_num,stp_len2,'b',1:stp_num,stp_len3,'k')
xlabel('\fontname{Songti SC}������');
ylabel('\fontname{Songti SC}����\fontname{Times New Roman} [m]');
legend('\fontname{Songti SC}���ھ��鹫ʽ�Ĳ�������','\fontname{Songti SC}��������ģ�͵Ĳ�������','\fontname{Songti SC}������Ϊģ�͵Ĳ�������');

%% �������    ������ֱ���˶�
 %figure
 %plot(1:stp_num,stp_len,'r--')
 %title('����');
 %figure
 %plot(1:stp_num,stp_len2,'b--')
 %title('����');
 %figure
 %plot(1:stp_num,stp_len3,'k--')
 %title('����');
% Distance = sum(stp_len); 
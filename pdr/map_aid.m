function [yaw_] = map_aid(ori0,gyr0)
    %%���ڵ�ͼƥ��ķ����˲�
    % ori0 = load('ori.txt');
    % gyr0 = load('gyr.txt');
    yaw = ori0(:,1);
    gyr_x = gyr0(:,1);
    th = 1;                 %%����ת�ǵ���ֵ

    %%�⺽��ǵĲ���
    for i = 1:size(yaw)-1
        if(yaw(i+1) - yaw(i)<-180)
            yaw(i+1) = yaw(i+1) + 360;
        else if(yaw(i+1) - yaw(i)>180)
                yaw(i+1) = yaw(i+1) - 360;  
            end
        end
    end
%      figure
%      plot(yaw)

    %%���������Ƕ�ת��λ��
    [ L, L0] = corner_detect(gyr_x,th);

    %%���õ�ͼƥ��ĽǶȶ�yaw�����˲�
%     yaw_kal = yaw_kal1(yaw,gyr_x,L0);  %
    yaw_kal = yaw;
    yaw_ = yaw_kal;
end
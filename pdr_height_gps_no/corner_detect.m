%x����ٶȼ����
function [L ,LO] = corner_detect(gyr_x,th)
%th ��ֵ
    len = length(gyr_x);
    num = 0;
    for i = 1:len
        if gyr_x(i)>1
            num = num+1;
            L(num) = i;
        end
    end
    num1 = 2;
    for i = 2:num
       if L(i)-L(i-1)>1
            num1 = num1+1;
            LO(num1-1) = L(i-1);
            LO(num1) = L(i);
       end
    end
    LO(1) = L(1);
    LO(num1+1) = L(num);
end
%导入设置信息和IMU数据
disp('导入设置信息和IMU数据')
u=settings1();

%零速区间检测器
disp('运行零速区间检测器')
[zupt, T]=zero_velocity_detector(u);

%卡尔曼滤波器
disp('运行常规卡尔曼滤波器')
[x_closed, cov_closed] = ZUPTaidedINS(u, zupt);

%RTS滤波平滑
disp('运行RTS平滑滤波器')
[x_smoothed, cov_smoothed, segment, P, P_smooth, dx, dx_smooth] = smoothed_ZUPTaidedINS(u, zupt);

%绘制图像
disp('绘制图像中')
view_data_smoothed;



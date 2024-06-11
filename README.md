背景：房屋、高楼、隧道以及其他各种存在各种电子设施的公共场合或封闭场，常规的卫星导航信号会受到干扰，甚至会直接被屏蔽，导致定位精度下降或无法定位。室内行人定位导航可以用于室内或其他封闭环境里面的抢险救援活动等特种工作，例如火灾时用于定位火警救灾人员、或者隧道施工、维修等。

1、国内外室内行人定位技术研究现状：目前通常采用无线信号交汇定位的技术，例如WIFI、RFID射频、UWB、WLAN、磁场、红外线、激光雷达等，通常这些技术需要提前了解环境信息，并布置相关的外设，比较适合特定场合，还有就是机器视觉的实时建图导航，但计算量大、实时性差，且受环境干扰较大。还有常用的惯性系统的捷联惯导解算，单一的惯性导航随着时间积累误差较大，需实时修正才能保证定位的有效性。我们采用人体典型运动模式检测并运动建模，并融合惯导进行定位。

2、研究内容：行人典型运动模式检测与运动建模，融合惯导并进行轨迹解算描绘：
  1）主要研究了IMU置于足部，根据人体运动时足部会存在静止时间，通过零速检测、设计卡尔曼滤波进行零速修正。
  2）IMU置于腰部和胸部处，行人运动时整个生体会随着走动而出现周期性的起伏，据此进行步态检测，在相邻的加速度波峰和波谷之间意味着行人已经走过一步，再根据相关模型不断估计步长，加上航向角估计从而确定位置，完成平面轨迹的描绘
  3）在前两者基础上加入高度估计，基于上下楼梯的过程中加速度计的变化幅度是远大于平面行走时，可得出人体是在楼层平面还是楼梯运动，从而绘制行人运动3D立体轨迹


Please Use main.m in ZUPT

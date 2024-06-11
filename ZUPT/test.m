% A = [1 2 3 4 5;5 6 7 8 9 ;9 10 11 12 13];
% 
% B = A(:,2:4);
% 
% C = zeros(9,9);
% C(1:3,4:6)=eye(3);
% 
% D = zeros(9)
% pos=1e-5*ones(3,1)               % Position (x,y,z navigation coordinate axis) [m]
% vel=1e-5*ones(3,1)               % Velocity (x,y,z navigation coordinate axis) [m/s]
% att=(pi/180*[0.1 0.1 0.1]')
% D(1:3,1:3)=diag(pos.^2)
% D(4:6,4:6)=diag(vel.^2)
% D(7:9,7:9)=diag(att.^2)
% E = eye(size(D))
% F(9,1) = diag(D)
% 
% Ts=3;
% A=eye(6);
% A(1,4)=Ts;
% A(2,5)=Ts;
% A(3,6)=Ts;
% A
% B=[(Ts^2)/2*eye(3);Ts*eye(3)]
% a = 0.1e-3

%%%MATLAB程序实现经纬度转换成平面尔坐标：
A = [1 1 1 1 1 1 ];
B = [0 0 0 0 0 0 ];
C = [2 2 2 2 2 2 ]
D = [A;B;C]



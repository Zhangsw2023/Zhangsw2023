clc;clear all;
%% 轨迹NURBS曲线插补
track = load('track1.txt'); % 位置
xforplot = track(:,1);
yforplot = track(:,2);
zforplot = track(:,3);
% track = track(round(linspace(1,numrows(track),8)),1:3);%linspace函数用于生成一个包含8个等间距元素的向量，这些元素的值介于1和numrows(track)之间（四舍五入取整）。
% % 然后，track变量被创建为具有3列的矩阵，其中每一列都包含上面生成的向量的相应元素。
pose = load('rotvect1.txt'); % 姿态
% pose = pose(round(linspace(1,numrows(pose),8)),1:3);
D_X = track(:,1); D_Y = track(:,2); D_Z = track(:,3);  % TCP 坐标
R_X = pose(:,1); R_Y = pose(:,2); R_Z = pose(:,3);  % 旋转向量
[m,n] = size([track,pose]);% m行n列

row = size(pose, 1); % 获取pose矩阵的行数  
  

U = zeros(row + 4, 1); % 创建U数组  
P_X = zeros(row, 1); % 创建P_X数组  
P_Y = zeros(row, 1); % 创建P_Y数组  
P_Z = zeros(row, 1); % 创建P_Z数组  
W = zeros(row, 1); % 创建W数组  
  
% 初始化p和V_T数组  
p = zeros(6, 1);  
V_T = zeros(3, 1);  
  
% 创建并初始化Tdd、s_m、Sr_m、Ts和u等向量  
Tdd = zeros(row, row);  
s_m = zeros(row, 1);  
Sr_m = zeros(row, 1);  
Sr = zeros(row, 1);  
u = zeros(row, 1);  
Ts = zeros(row, row);  
  
% 设置全局变量  
v0 = 0;  
v1 = 0;  
s0 = 0;  
s1 = 0;  
vmax = 0;  
amax = 0;  
td = 0;  
arcS = 0;  
  

  
% 调用counter_B函数进行计算  
[P_X,P_Y,P_Z,U,w] =counter_B(D_X, D_Y, D_Z);  
  

u=U;

n = length(u);
m = row;
arcs = 0;
s_m = [0];
for i = 1 : n - 1
    arcs = ComputeArcLengthOfBspline(m, P_X, P_Y, P_Z, U, u(i), u(i + 1)); %计算样条曲线弧长
    s_m = [s_m s_m(i) + arcs];
end

St = s_m(end);
s = [];
for i = 1 : length(s_m)
    s = [s s_m(i) / St];
end


% step1  计算四元数
Q = []; % 用来储存四元数
for i = 1:m
    q = rotvec2quaternion(R_X(i), R_Y(i), R_Z(i));
    Q = [Q;q];%每一行q0, q1, q2, q3
end
% step2 根据数据点，反算出位置控制点
[P_X,P_Y,P_Z,U,w] = counter_B(D_X,D_Y,D_Z);  %节点矢量个数为m+2*k,应为14个  ?

%% 轨迹NURBS插补&姿态Squad插补
straj=[];
k=1; %生成位姿的行数
Sn = GetCtlPoints(Q,m);

for i=1:m-1 %pose中的行
    t=0;
    N = ComDisPntsNum(D_X, D_Y, D_Z, i); %插值点数
    t_ = 1/N;
    for j=1:N %两行pose之间插n个点
        Pra = ((1 - t)* w(i) + t* w(i+1)) + 1.0e-6;
        [straj(k,1),straj(k,2),straj(k,3)] = Bspline(P_X,P_Y,P_Z,U,Pra); %NURBS
%         straj(k,4:7) = slerp(Q(i,1:4),Q(i+1,1:4),t);
        straj(k,4:7) = Squad(Q(i,:),Sn(i,:),Q(i+1,:),Sn(i+1,:),t); %Squad
        straj(k,4:6) = quaternion2rpy(straj(k,4:7));
        straj(k,4:6) = rpy2rotvec(straj(k,4),straj(k,5),straj(k,6));
        k = k+1; 
        t = t + t_;
    end    
end
%% 画图 
straj(:,7)=[];
% 导出straj
% writematrix(straj,'straj.txt');
% 数据点
% plot3(D_X,D_Y,D_Z,'.k','LineWidth',4);
plot3(xforplot,yforplot,zforplot,'.k','LineWidth',4);
hold on 
% 控制点
plot3(P_X,P_Y,P_Z,'.g-','LineWidth',1);
hold on
% 样条曲线
plot3(straj(:,1),straj(:,2),straj(:,3),'r','LineWidth',1); %输出末端轨迹
hold on
xlabel('x')
ylabel('y')
zlabel('z')
title('NURBS轨迹插补');
legend('数据点','控制点','样条曲线')
% legend('数据点','样条曲线')
axis equal
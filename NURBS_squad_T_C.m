clc;clear all;
%% ***轨迹NURBS曲线插补+T型速度规划+squad姿态插补***zsw***2023.7.14***
%% 导入数据点的位置与姿态数据
track = load('track1.txt'); % 位置
pt=load('track1.txt'); % 位置
xforplot = track(:,1);
yforplot = track(:,2);
zforplot = track(:,3);
pose = load('rotvect1.txt'); % 姿态
pose = pose(round(linspace(1,numrows(pose),8)),1:3);
D_X = track(:,1); D_Y = track(:,2); D_Z = track(:,3);  % TCP 坐标
R_X = pose(:,1); R_Y = pose(:,2); R_Z = pose(:,3);  % rpy欧拉角（数据由示教器teach出）
m=length(track);
%% 根据数据点计算控制点，并求出nurbs曲线的插值点
[P_X,P_Y,P_Z,U,w] =counter_B(D_X, D_Y, D_Z); 
%% 对曲线C(u)等参数取样
ucount = 0;
pro = 0.001;
u = [];
for i = 1:(1/pro+1)
    u(i) = ucount;
    ucount = ucount + pro;
end
%% 计算对应的弧长参数
n = length(u);
row=m;
s_m = [0];
ss=length(P_X);

for i = 1:n-1
    arcs = ComputeArcLengthOfBspline(row, P_X,P_Y,P_Z, U, u(i), u(i+1),ss);
    s_m(i+1) = s_m(i) + arcs;
end

St = s_m(n);

s = [];
for i = 1:length(s_m)
    s(i) = s_m(i) / St;
end
%% 建立u-s模型（u关于s的五次多项式）
ret=polyFit(s, u, 5);%ret为1*6数组，即为s=a+b*u+c*u^2+d*u^3+.....
%% T型速度插补
v0 = 0;
v1 = 0;
s0 = 0;
s1 = St/100;
vmax = 30;
amax = 30;
td = 0.001; % 采样周期
Ts = [];
V_T = [];

[nn,time, q, qd, qdd] = TSpeedCruveMethod_1(s0, s1, v0, v1, vmax, amax, td); % T型速度规划

Ts=[time', q', qd', qdd'];
%% 画出T型插补的位置、速度、加速度曲线
figure(1)
subplot(311)
plot(time,q,'r','LineWidth',1.5);
grid on;xlabel('time[s]');ylabel('position[mm]');
subplot(312)
plot(time,qd,'b','LineWidth',1.5);
grid on;xlabel('time[s]');ylabel('speed[mm/s]');
subplot(313)
plot(time,qdd,'g','LineWidth',1.5);
grid on;xlabel('time[s]');ylabel('acceleration[mm/s2]');
%% 根据T型速度规划获取B样条离散点
Tdd = [];
u_i = [];

for i = 1:nn-1
    u = Ts(i, 2)/Ts(nn, 2);
    if u> 1.0
        u = 1.0;
    end
    u_i = [u_i, u];
    
     [Arr]= Bspline1(P_X,P_Y,P_Z,U, u); % B样条
     
    Tdd = [Tdd; Arr];
    u = [];
end
%% 画出根据T型速度规划后的nurbs曲线
figure(2)
plot3(Tdd(:,1),Tdd(:,2),Tdd(:,3),'b','LineWidth',0.1); %输出末端轨迹
hold on ;grid on;
% plot3(P_X,P_Y,P_Z,'m-o','LineWidth',0.1); %输出控制线
% hold on ;grid on;
scatter3(xforplot,yforplot,zforplot,'*r','LineWidth',0.1);
hold on ;grid on;
title('T型速度规划后的nurbs曲线'); % 图标题
xlabel('x');     % x轴标签
ylabel('y');     % y轴标签
zlabel('z');     % y轴标签
legend('nurbs曲线', '型值点'); % 添加图例
hold off
%% 计算数据点的弧长
Sr_m = [0];
for i = 1:m-1
    arcS = ComputeArcLengthOfBspline(row, P_X, P_Y, P_Z, U, w(i), w(i+1),ss); % 计算数据点对应的曲线弧长
    Sr_m = [Sr_m, Sr_m(i) + arcS];
end
%% 求对应的弧长参数
mm=length(Sr_m);
Sr = [];
for i = 1:mm
    Sr = [Sr, Sr_m(i) / Sr_m(mm)];
end
%% 计算四元数
Q = []; % 用来储存四元数
for i = 1:m
    qx=rpy2rotvec(R_X(i), R_Y(i), R_Z(i));
    q = rotvec2quaternion(qx(1,1), qx(1,2), qx(1,3));
    Q = [Q;q];%每一行q0, q1, q2, q3
end
%% 求姿态的控制点
Sn = GetCtlPoints(Q,m);
%% 对姿态四元数插值
Quat=[];
Qisum=[];

for i = 1:length(u_i)
    for j = 1:m-1
        if Sr(j) <= u_i(i) && u_i(i) <= Sr(j+1)
            qi1 = Q(j,:);
            qi2 = Q(j + 1,:);
            si1 = Sn(j,:);
            si2 = Sn(j + 1,:);
            pra = (u_i(i) - Sr(j)) / (Sr(j + 1) - Sr(j));
        end
    end
    Qi = Squad(qi1, si1, qi2, si2, pra);%squad插值
    Qisum=[Qisum;Qi];
    Qc=quaternion2rot_vec(Qi);%四元数to旋转矩阵（最好别四元数到rpy到旋转矩阵，求rpy时atan2不能解出大于180度的角）
    Quat = [Quat; Qc];
end
%% 画出四元数q0 q1 q2 q3的插值曲线
figure(3)
plot(1:length(Qisum),Qisum(:,1),'b','LineWidth',1); %q0四元数的变换
grid on;hold on;
plot(1:length(Qisum),Qisum(:,2),'r','LineWidth',1); %q1四元数的变换
grid on;hold on;
plot(1:length(Qisum),Qisum(:,3),'m','LineWidth',1); %q2四元数的变换
grid on;hold on;
plot(1:length(Qisum),Qisum(:,4),'g','LineWidth',1); %q3四元数的变换
grid on;hold on;

xlabel('点数');     % x轴标签
ylabel('q0 q1 q2 q3');     % y轴标签.
title('四元数插值'); % 图标题
legend('q0', 'q1','q2','q3'); % 添加图例
hold off;        % 释放图形保持状态，以便可以绘制新的图形
%% 画出旋转矩阵的插值曲线
figure(7)
plot(1:length(Quat),Quat(:,1),'b','LineWidth',1); %
grid on;hold on;
plot(1:length(Quat),Quat(:,2),'r','LineWidth',1); %
grid on;hold on;
plot(1:length(Quat),Quat(:,3),'m','LineWidth',1); %
grid on;hold on;

% xlabel('点数');     % x轴标签
% ylabel('q0 q1 q2 q3');     % y轴标签.
% title('四元数插值'); % 图标题
% legend('q0', 'q1','q2'); % 添加图例
% hold off;        % 释放图形保持状态，以便可以绘制新的图形
%% 建立机器人模型
% theta d a alpha offset
L1=Link([0 0 0 0 0 ],'modified'); %连杆的D-H参数
L2=Link([0 149.09 0 -pi/2 0 ],'modified');
L3=Link([0 0 431.8 0 0 ],'modified');
L4=Link([0 433.07 20.32 -pi/2 0 ],'modified');
L5=Link([0 0 0 pi/2 0 ],'modified');
L6=Link([0 0 0 -pi/2 0 ],'modified');
robot=SerialLink([L1 L2 L3 L4 L5 L6],'name',' ','base' , ...
transl(0, 0, 0.62)* trotz(0)); %连接连杆，机器人取名
% robot.plot([0,pi/2,0,0,0,pi/2]);%输出机器人模型，后面的六个角为输出时的theta姿态
figure(4)
plot3(Tdd(:,1),Tdd(:,2),Tdd(:,3),'b','LineWidth',0.1); %输出末端轨迹
hold on ;grid on;
scatter3(xforplot,yforplot,zforplot,'*r','LineWidth',0.1);
robot.plot([0,0,0,0,0,0])
hold on
%% 反解关节角并使机器人沿nurbs曲线运动
straj=[Tdd,Quat];
T=straj;%将位置与姿态信息存到T
G = [];  %用来存储新的障碍节点，用于栅格地图的更新
N = size(T,1);
Qj= []; %可行路径上的最终各点的关节角
c = [0 0 0 0 0 0 0 0]; %不可行点判别数组  % A为路径可行判别数，可行为1，不可行为0
t0=[0 0 0 0 0 0];
e0 = [];
temp0=0;
sum = [];
for i =1:N    %分别对各点进行检测
   t = T(i,1:3);
   tt=T(i,4:6);
   qm = transl(t);
   qq= transl(tt);
   qm(1,1)=qq(1,4);qm(2,2)=qq(2,4);qm(3,3)=qq(3,4);
   
%     TH = ikine(robot,q);
    TH = niyundx_change(qm);
    for n=1:8
        for mm=1:6
            if TH(n,mm)>pi
                TH(n,mm)=TH(n,mm)-2*pi;
            elseif TH(n,mm)<-pi
                TH(n,mm)=TH(n,mm)+2*pi;
            end
        end
    end
    for j=1:8
        for k=1:6
           temp0=temp0+abs(TH(j,k)-t0(k));            
        end
        e0(j)=temp0;
        temp0=0;        
    end
    [e1,index]=sort(e0);%变量 e0 进行排序，sort 函数默认按升序排序
    THH = [TH(index(1),:);TH(index(2),:);TH(index(3),:);TH(index(4),:);TH(index(5),:);
        TH(index(6),:);TH(index(7),:);TH(index(8),:)];%将变量 TH 中索引为 1 的行复制到一个新的矩阵,行索引从 0 开始，列索引从 1 开始。因此，索引为 1 的行表示矩阵中的第二行。

   Qj(i,:) = THH(1,:);
    t0=Qj(i,:);
    cc = 1;
    for n = 1:8
        cc = cc*c(n);
    end
    if cc ==1  %判别i坐标处是否所有关节角都不可行
        G = [G;T(i,:)];
        A = 0;  %若都不可行，表示路径不可行，该点纳入障碍节点，需重新规划
        break
    else if i == N  %若可行，判断是否已经最后一个路径点
            A = 1;   %若是，则路径可行，输出各点的最优关节角
            Qj;
            break
        end
    end
end

ll=length(Qj);

for i=1:25:nn
    
    plot3(Tdd(i,1),Tdd(i,2),Tdd(i,3),'m.','LineWidth',0.01);
    hold on;
    robot.plot(Qj(i,:));
    hold on;
end
    hold off;
%% 画出关节角的变化
figure(5)
plot(1:length(Qj),Qj(:,1),'b','LineWidth',1); %theta1的变换
grid on;hold on;
plot(1:length(Qj),Qj(:,2),'r','LineWidth',1); %theta2的变换
grid on;hold on;
plot(1:length(Qj),Qj(:,3),'m','LineWidth',1); %theta3的变换
grid on;hold on;
plot(1:length(Qj),Qj(:,4),'k','LineWidth',1); %theta4的变换
grid on;hold on;
plot(1:length(Qj),Qj(:,5),'c','LineWidth',1); %theta5的变换
grid on;hold on;
plot(1:length(Qj),Qj(:,6),'g','LineWidth',1); %theta6的变换
grid on;hold on;

xlabel('点数');     % x轴标签
ylabel('theta1 theta2 theta3 theta4 theta5 theta6');     % y轴标签.
title('关节角的变化'); % 图标题
legend('theta1', 'theta2','theta3','theta4','theta5','theta6'); % 添加图例
hold off;        % 释放图形保持状态，以便可以绘制新的图形
%% 检验:姿态插补t的变化
% prasum=[];
% for i = 1:length(u_i)
%     for j = 1:m-1
%         if Sr(j) <= u_i(i) && u_i(i) <= Sr(j+1)
%             qi1 = Q(j,:);
%             qi2 = Q(j + 1,:);
%             si1 = Sn(j,:);
%             si2 = Sn(j + 1,:);
%             pra = (u_i(i) - Sr(j)) / (Sr(j + 1) - Sr(j));
%         end
%     end
%     prasum=[prasum;pra];
% end
% figure(6)
% plot(1:length(prasum),prasum(:,1),'b','LineWidth',1); %t的变换
% xlabel('点数');     % x轴标签
% ylabel('姿态插补的t');     % y轴标签.
% title('姿态插补的t的变化'); % 图标题





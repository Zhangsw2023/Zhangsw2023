clc;clear all;
%% 轨迹NURBS曲线插补
track = load('track1.txt'); % 位置
pt=track; % 位置
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

%% ********节点矢量************%
k=3;
x=D_X;
y=D_Y;
z=D_Z ;
temp=zeros(1,m-1);
for i=1:m-1%m个数据点，利用积累弦长法
temp(i)=sqrt((x(i+1)-x(i))^2+(y(i+1)-y(i))^2+(z(i+1)-z(i))^2);
end
for i=1:k+1%前k+1个节点为0
    U(i)=0;
end
for i=n+k-1:n+2*k%后k+1个节点为1
    U(i)=1;
end
for i=k+1:m+k-1%m个数据点，m=n-1,内节点为m-2个，U(k+1)作为初始值
    U(i+1)=U(i)+temp(i-k);%m-1段弦长，U（k+1）到U（m+k）共m个节点
end
xc=U(m+k);%总弦长
for i=k+1:m+k
    U(i)=U(i)/U(m+k);
end
%% ***************反算n+1个控制点**************
dU=zeros(1,m+2*k);%△U
dpt1=[0 1 1]*xc;%给定首数据点切矢
dptm=[-1 0 -1]*xc;%给定末数据点切矢

A=zeros(n-1);
E=zeros(n-1,3);
%
for i=k+1:n+k+1
    dU(i)=U(i+1)-U(i);
end
A(1,1)=1;%切矢条件b1=1,c1=a1=0,e1
E(1,:,:)=pt(1,:,:)+(dU(4)/3)*dpt1;
A(n-1,n-1)=1;%切矢条件cn-1=an-1=0,bn-1=0
E(n-1,:,:)=pt(m,:,:)-(dU(n+1)/3)*dptm;
for i=2:n-2%a,b,c,e的值
    A(i,i-1)=dU(i+3).^2/(dU(i+1)+dU(i+2)+dU(i+3));
    A(i,i)=dU(i+3)*(dU(i+1)+dU(i+2))/(dU(i+1)+dU(i+2)+dU(i+3))+...
        dU(i+2)*(dU(i+3)+dU(i+4))/(dU(i+2)+dU(i+3)+dU(i+4));
    A(i,i+1)=dU(i+2).^2/(dU(i+2)+dU(i+3)+dU(i+4));
   E(i,:,:)=(dU(i+2)+dU(i+3))*pt(i,:,:);
end
D=A\E;
D=[pt(1,:,:);D;pt(m,:,:)];%加上首末端点，控制顶点比数据点多两个
[ss,t]=size(D);

% % [P_X,P_Y,P_Z,U,w] =counter_B(D_X, D_Y, D_Z);  
% % 
% % ucount = 0;
% % pro = 0.001;
% % u = [];
% % for i = 1:(1/pro+1)
% %     u(i) = ucount;
% %     ucount = ucount + pro;
% % end
% % 
% % n = length(u);
% % row=m;
% % s_m = [0];
% % 
% % 
% % for i = 1:n-1
% %     arcs = ComputeArcLengthOfBspline(row,P_X ,P_Y ,P_Z , U, u(i), u(i+1),s);
% %     s_m(i+1) = s_m(i) + arcs;
% % end


ucount = 0;
pro = 0.001;
u = [];
for i = 1:(1/pro+1)
    u(i) = ucount;
    ucount = ucount + pro;
end

n = length(u);
row=m;
s_m = [0];


for i = 1:n-1
    arcs = ComputeArcLengthOfBspline(row, D(:,1), D(:,2), D(:,3), U, u(i), u(i+1),ss);
    s_m(i+1) = s_m(i) + arcs;
end

St = s_m(n);


s = [];
for i = 1:length(s_m)
    s(i) = s_m(i) / St;
end


ret=polyFit(s, u, 5);%ret为1*6数组，即为s=a+b*u+c*u^2+d*u^3+.....


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

% figure(1)
% subplot(311)
% plot(time,q,'r','LineWidth',1.5);
% grid on;xlabel('time[s]');ylabel('position[mm]');
% subplot(312)
% plot(time,qd,'b','LineWidth',1.5);
% grid on;xlabel('time[s]');ylabel('speed[mm/s]');
% subplot(313)
% plot(time,qdd,'g','LineWidth',1.5);
% grid on;xlabel('time[s]');ylabel('acceleration[mm/s2]');


Tdd = [];
u_i = [];

% 报错：（2023.7.14）只能循环到3170次
%数组索引必须为正整数或逻辑值。
%出错 Bspline1 (line 25)
%               if (U(i+j-1)-U(i))==0
for i = 1:nn
    u = Ts(i, 2)/Ts(nn, 2);
    if u> 1.0
        u = 1.0;
    end
    u_i = [u_i, u];
    
     [Arr]= Bspline1(D(:,1), D(:,2), D(:,3),U, u); % B样条
     
    Tdd = [Tdd; Arr];
    u = [];
end





% for i = 1:nn
%     ui = Ts(i, 1);
%     if ui > 1.0
%         ui = 1.0;
%     end
%     u_i = [u_i, ui];
%     x_Arr = Bspline1(D(:,1), D(:,2), D(:,3), U, ui); % B样条
%     for j = 1:3
%         u = [u, x_Arr(j)];
%     end
% %     Tdd = [Tdd; u];
% %     u = [];
% end



plot3(Tdd(:,1),Tdd(:,2),Tdd(:,3),'r','LineWidth',1); %输出末端轨迹
hold on ;grid on;
plot3(xforplot,yforplot,zforplot,'.r','LineWidth',4);
hold on ;grid on;
% P=[];      %Nurbs曲线
% LL=[];
% syms dx;   %定义节点间的递增变量为变量dx
% for i=k+1:ss
%     u=U;
%     d=sym(D);
%     %每次迭代后将d恢复初值
%     for m=1:k
%         for j=i-k:i-m
%             alpha(j)=(dx-u(j+m))/(u(j+k+1)-u(j+m));
%             d(j,:)=(1-alpha(j))*d(j,:)+alpha(j)*d(j+1,:);
%         end
%     end
%     %带入变量dx求出各节点区间[ui,ui+1]内m=3的控制点d
%     for j=1:length(Ts)
%     if Ts(j,2)>=u(i)&&u(i+1)>=Ts(j,2)
%         LL=[LL,Ts(j,2)];
%     end
%     end
%        
% %     LL=sort(LL);
%     M=subs(d(i-k,:),dx,LL');    %节点区间内的插值点替换dx
%     P=[P;double(M)];
% end
% M=subs(d(s-k,:),dx,1);
% P=[P;double(M)];%补上最后一个点








n=length(P);
for i=1:n-1
    L(i)=sqrt((P(i+1,1)-P(i,1))^2+(P(i+1,2)-P(i,2))^2+(P(i+1,3)-P(i,3))^2);%nurbs总弦长
end
w(1)=0;%积累弦长参数化
for i=2:n
    w(i)=w(i-1)+L(i-1);
end
l=zeros(1,n);
for i=1:n
    l(1,i)=w(1,i)/w(1,n);
end  

Ll=w(n);


l=q/q(length(q));%T插补后的弧长参数
 
%计算四元数
QQ = []; % 用来储存四元数
for i = 1:m
    qx=rpy2rotvec(R_X(i), R_Y(i), R_Z(i));
    q = rotvec2quaternion(qx(1,1), qx(1,2), qx(1,3));
    QQ = [QQ;q];%每一行q0, q1, q2, q3
end

[P_X,P_Y,P_Z,U,w] = counter_B(D_X,D_Y,D_Z);  %节点矢量个数为m+2*k,应为14个  ?
%% 轨迹NURBS插补&姿态Squad插补
straj=[];
k=1; %生成位姿的行数
Sn = GetCtlPoints(QQ,m);

for i=1:m-1 %
    t=0;
    t_ = 1/N;
    for j=1:N %两行pose之间插n个点
        Pra = ((1 - t)* w(i) + t* w(i+1)) + 1.0e-6;
        [straj(k,1),straj(k,2),straj(k,3)] = Bspline(P_X,P_Y,P_Z,U,Pra); %NURBS
%         straj(k,4:7) = slerp(Q(i,1:4),Q(i+1,1:4),t);
        straj(k,4:7) = Squad(QQ(i,:),Sn(i,:),QQ(i+1,:),Sn(i+1,:),t); %Squad
        straj(k,4:6) = quaternion2rpy(straj(k,4:7));
        straj(k,4:6) = rpy2rotvec(straj(k,4),straj(k,5),straj(k,6));
        k = k+1; 
        t =  t + t_;
    end  
    
end
%% 画图
straj(:,7)=[];
% 导出straj
% writematrix(straj,'straj.txt');
% 数据点
% plot3(D_X,D_Y,D_Z,'.k','LineWidth',4);
figure(2)
plot3(xforplot,yforplot,zforplot,'.r','LineWidth',4);
hold on 
% 控制点
plot3(P_X,P_Y,P_Z,'-b','LineWidth',1);
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

%建立机器人模型
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
robot.plot([0,0,0,0,0,0])
figure(2)
% robot.teach() 
robot.display(); %显示D-H表
hold on
T1 = transl(track(1,1),track(1,2),track(1,3));	%起点 返回一个表示平移变换的矩阵4*4\
T2 = transl(track(m,1),track(m,2),track(m,2));	%终点

nn=length(straj);
TA = ones(nn,1);%nn行1列元素为1的矩i阵
PA= [straj,TA];
% T = ctraj(T1,T2,nn);%使用ctraj函数在这两个点之间规划了一个匀加速和匀减速的轨迹，4行4列共nn个，均为平移形式。
% for i=1:nn
%     T = replace(T, T(:,3,i), PA(i,:));
% end
Tj =straj;%使用transl函数将轨迹中的每个点转换为末端执行器的平移变换矩阵50*3
%输出末端轨迹%


T=Tj;%*3
G = [];  %用来存储新的障碍节点，用于栅格地图的更新
% N = ndims(T);
N = size(T,1);
Q = []; %可行路径上的最终各点的关节角
c = [0 0 0 0 0 0 0 0]; %不可行点判别数组  % A为路径可行判别数，可行为1，不可行为0
t0=[0 0 0 0 0 0];
e0 = [];
temp0=0;
sum = [];
for i =1:N    %分别对各点进行碰撞检测
   t = T(i,1:3);
   tt=T(i,4:6);
   q = transl(t);
   qq= transl(tt);
   q(1,1)=qq(1,4);q(2,2)=qq(2,4);q(3,3)=qq(3,4);
   
%     TH = ikine(robot,q);
    TH = niyundx_change(q);
    for n=1:8
        for mm=1:6
            if TH(n,mm)>pi
                TH(n,mm)=TH(n,mm)-2*pi;
            elseif TH(n,mm)<-pi
                TH(n,mm)=TH(n,mm)+2*pi;
            end
        end
    end
    TH
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

%    Q(i,:) = TH(1,:); 
   Q(i,:) = THH(1,:);
    t0=Q(i,:);
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
            Q;
            break
        end
    end
end

ll=length(Q);

for i=1:200:nn
    
figure(2)

    plot3(Tj(i,1),Tj(i,2),Tj(i,3),'m.');
    hold on;
    robot.plot(Q(i,:));

end


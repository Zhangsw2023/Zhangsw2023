clc;clear all;
%% �켣NURBS���߲岹
track = load('track1.txt'); % λ��
pt=track; % λ��
xforplot = track(:,1);
yforplot = track(:,2);
zforplot = track(:,3);
% track = track(round(linspace(1,numrows(track),8)),1:3);%linspace������������һ������8���ȼ��Ԫ�ص���������ЩԪ�ص�ֵ����1��numrows(track)֮�䣨��������ȡ������
% % Ȼ��track����������Ϊ����3�еľ�������ÿһ�ж������������ɵ���������ӦԪ�ء�
pose = load('rotvect1.txt'); % ��̬
% pose = pose(round(linspace(1,numrows(pose),8)),1:3);
D_X = track(:,1); D_Y = track(:,2); D_Z = track(:,3);  % TCP ����
R_X = pose(:,1); R_Y = pose(:,2); R_Z = pose(:,3);  % ��ת����
[m,n] = size([track,pose]);% m��n��

%% ********�ڵ�ʸ��************%
k=3;
x=D_X;
y=D_Y;
z=D_Z ;
temp=zeros(1,m-1);
for i=1:m-1%m�����ݵ㣬���û����ҳ���
temp(i)=sqrt((x(i+1)-x(i))^2+(y(i+1)-y(i))^2+(z(i+1)-z(i))^2);
end
for i=1:k+1%ǰk+1���ڵ�Ϊ0
    U(i)=0;
end
for i=n+k-1:n+2*k%��k+1���ڵ�Ϊ1
    U(i)=1;
end
for i=k+1:m+k-1%m�����ݵ㣬m=n-1,�ڽڵ�Ϊm-2����U(k+1)��Ϊ��ʼֵ
    U(i+1)=U(i)+temp(i-k);%m-1���ҳ���U��k+1����U��m+k����m���ڵ�
end
xc=U(m+k);%���ҳ�
for i=k+1:m+k
    U(i)=U(i)/U(m+k);
end
%% ***************����n+1�����Ƶ�**************
dU=zeros(1,m+2*k);%��U
dpt1=[0 1 1]*xc;%���������ݵ���ʸ
dptm=[-1 0 -1]*xc;%����ĩ���ݵ���ʸ

A=zeros(n-1);
E=zeros(n-1,3);
%
for i=k+1:n+k+1
    dU(i)=U(i+1)-U(i);
end
A(1,1)=1;%��ʸ����b1=1,c1=a1=0,e1
E(1,:,:)=pt(1,:,:)+(dU(4)/3)*dpt1;
A(n-1,n-1)=1;%��ʸ����cn-1=an-1=0,bn-1=0
E(n-1,:,:)=pt(m,:,:)-(dU(n+1)/3)*dptm;
for i=2:n-2%a,b,c,e��ֵ
    A(i,i-1)=dU(i+3).^2/(dU(i+1)+dU(i+2)+dU(i+3));
    A(i,i)=dU(i+3)*(dU(i+1)+dU(i+2))/(dU(i+1)+dU(i+2)+dU(i+3))+...
        dU(i+2)*(dU(i+3)+dU(i+4))/(dU(i+2)+dU(i+3)+dU(i+4));
    A(i,i+1)=dU(i+2).^2/(dU(i+2)+dU(i+3)+dU(i+4));
   E(i,:,:)=(dU(i+2)+dU(i+3))*pt(i,:,:);
end
D=A\E;
D=[pt(1,:,:);D;pt(m,:,:)];%������ĩ�˵㣬���ƶ�������ݵ������
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


ret=polyFit(s, u, 5);%retΪ1*6���飬��Ϊs=a+b*u+c*u^2+d*u^3+.....


v0 = 0;
v1 = 0;
s0 = 0;
s1 = St/100;
vmax = 30;
amax = 30;
td = 0.001; % ��������
Ts = [];
V_T = [];

[nn,time, q, qd, qdd] = TSpeedCruveMethod_1(s0, s1, v0, v1, vmax, amax, td); % T���ٶȹ滮

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

% ������2023.7.14��ֻ��ѭ����3170��
%������������Ϊ���������߼�ֵ��
%���� Bspline1 (line 25)
%               if (U(i+j-1)-U(i))==0
for i = 1:nn
    u = Ts(i, 2)/Ts(nn, 2);
    if u> 1.0
        u = 1.0;
    end
    u_i = [u_i, u];
    
     [Arr]= Bspline1(D(:,1), D(:,2), D(:,3),U, u); % B����
     
    Tdd = [Tdd; Arr];
    u = [];
end





% for i = 1:nn
%     ui = Ts(i, 1);
%     if ui > 1.0
%         ui = 1.0;
%     end
%     u_i = [u_i, ui];
%     x_Arr = Bspline1(D(:,1), D(:,2), D(:,3), U, ui); % B����
%     for j = 1:3
%         u = [u, x_Arr(j)];
%     end
% %     Tdd = [Tdd; u];
% %     u = [];
% end



plot3(Tdd(:,1),Tdd(:,2),Tdd(:,3),'r','LineWidth',1); %���ĩ�˹켣
hold on ;grid on;
plot3(xforplot,yforplot,zforplot,'.r','LineWidth',4);
hold on ;grid on;
% P=[];      %Nurbs����
% LL=[];
% syms dx;   %����ڵ��ĵ�������Ϊ����dx
% for i=k+1:ss
%     u=U;
%     d=sym(D);
%     %ÿ�ε�����d�ָ���ֵ
%     for m=1:k
%         for j=i-k:i-m
%             alpha(j)=(dx-u(j+m))/(u(j+k+1)-u(j+m));
%             d(j,:)=(1-alpha(j))*d(j,:)+alpha(j)*d(j+1,:);
%         end
%     end
%     %�������dx������ڵ�����[ui,ui+1]��m=3�Ŀ��Ƶ�d
%     for j=1:length(Ts)
%     if Ts(j,2)>=u(i)&&u(i+1)>=Ts(j,2)
%         LL=[LL,Ts(j,2)];
%     end
%     end
%        
% %     LL=sort(LL);
%     M=subs(d(i-k,:),dx,LL');    %�ڵ������ڵĲ�ֵ���滻dx
%     P=[P;double(M)];
% end
% M=subs(d(s-k,:),dx,1);
% P=[P;double(M)];%�������һ����








n=length(P);
for i=1:n-1
    L(i)=sqrt((P(i+1,1)-P(i,1))^2+(P(i+1,2)-P(i,2))^2+(P(i+1,3)-P(i,3))^2);%nurbs���ҳ�
end
w(1)=0;%�����ҳ�������
for i=2:n
    w(i)=w(i-1)+L(i-1);
end
l=zeros(1,n);
for i=1:n
    l(1,i)=w(1,i)/w(1,n);
end  

Ll=w(n);


l=q/q(length(q));%T�岹��Ļ�������
 
%������Ԫ��
QQ = []; % ����������Ԫ��
for i = 1:m
    qx=rpy2rotvec(R_X(i), R_Y(i), R_Z(i));
    q = rotvec2quaternion(qx(1,1), qx(1,2), qx(1,3));
    QQ = [QQ;q];%ÿһ��q0, q1, q2, q3
end

[P_X,P_Y,P_Z,U,w] = counter_B(D_X,D_Y,D_Z);  %�ڵ�ʸ������Ϊm+2*k,ӦΪ14��  ?
%% �켣NURBS�岹&��̬Squad�岹
straj=[];
k=1; %����λ�˵�����
Sn = GetCtlPoints(QQ,m);

for i=1:m-1 %
    t=0;
    t_ = 1/N;
    for j=1:N %����pose֮���n����
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
%% ��ͼ
straj(:,7)=[];
% ����straj
% writematrix(straj,'straj.txt');
% ���ݵ�
% plot3(D_X,D_Y,D_Z,'.k','LineWidth',4);
figure(2)
plot3(xforplot,yforplot,zforplot,'.r','LineWidth',4);
hold on 
% ���Ƶ�
plot3(P_X,P_Y,P_Z,'-b','LineWidth',1);
hold on
% ��������
plot3(straj(:,1),straj(:,2),straj(:,3),'r','LineWidth',1); %���ĩ�˹켣
hold on
xlabel('x')
ylabel('y')
zlabel('z')
title('NURBS�켣�岹');
legend('���ݵ�','���Ƶ�','��������')
% legend('���ݵ�','��������')
axis equal

%����������ģ��
% theta d a alpha offset
L1=Link([0 0 0 0 0 ],'modified'); %���˵�D-H����
L2=Link([0 149.09 0 -pi/2 0 ],'modified');
L3=Link([0 0 431.8 0 0 ],'modified');
L4=Link([0 433.07 20.32 -pi/2 0 ],'modified');
L5=Link([0 0 0 pi/2 0 ],'modified');
L6=Link([0 0 0 -pi/2 0 ],'modified');
robot=SerialLink([L1 L2 L3 L4 L5 L6],'name',' ','base' , ...
transl(0, 0, 0.62)* trotz(0)); %�������ˣ�������ȡ��
% robot.plot([0,pi/2,0,0,0,pi/2]);%���������ģ�ͣ������������Ϊ���ʱ��theta��̬
robot.plot([0,0,0,0,0,0])
figure(2)
% robot.teach() 
robot.display(); %��ʾD-H��
hold on
T1 = transl(track(1,1),track(1,2),track(1,3));	%��� ����һ����ʾƽ�Ʊ任�ľ���4*4\
T2 = transl(track(m,1),track(m,2),track(m,2));	%�յ�

nn=length(straj);
TA = ones(nn,1);%nn��1��Ԫ��Ϊ1�ľ�i��
PA= [straj,TA];
% T = ctraj(T1,T2,nn);%ʹ��ctraj��������������֮��滮��һ���ȼ��ٺ��ȼ��ٵĹ켣��4��4�й�nn������Ϊƽ����ʽ��
% for i=1:nn
%     T = replace(T, T(:,3,i), PA(i,:));
% end
Tj =straj;%ʹ��transl�������켣�е�ÿ����ת��Ϊĩ��ִ������ƽ�Ʊ任����50*3
%���ĩ�˹켣%


T=Tj;%*3
G = [];  %�����洢�µ��ϰ��ڵ㣬����դ���ͼ�ĸ���
% N = ndims(T);
N = size(T,1);
Q = []; %����·���ϵ����ո���Ĺؽڽ�
c = [0 0 0 0 0 0 0 0]; %�����е��б�����  % AΪ·�������б���������Ϊ1��������Ϊ0
t0=[0 0 0 0 0 0];
e0 = [];
temp0=0;
sum = [];
for i =1:N    %�ֱ�Ը��������ײ���
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
    [e1,index]=sort(e0);%���� e0 ��������sort ����Ĭ�ϰ���������
    THH = [TH(index(1),:);TH(index(2),:);TH(index(3),:);TH(index(4),:);TH(index(5),:);
        TH(index(6),:);TH(index(7),:);TH(index(8),:)];%������ TH ������Ϊ 1 ���и��Ƶ�һ���µľ���,�������� 0 ��ʼ���������� 1 ��ʼ����ˣ�����Ϊ 1 ���б�ʾ�����еĵڶ��С�

%    Q(i,:) = TH(1,:); 
   Q(i,:) = THH(1,:);
    t0=Q(i,:);
    cc = 1;
    for n = 1:8
        cc = cc*c(n);
    end
    if cc ==1  %�б�i���괦�Ƿ����йؽڽǶ�������
        G = [G;T(i,:)];
        A = 0;  %���������У���ʾ·�������У��õ������ϰ��ڵ㣬�����¹滮
        break
    else if i == N  %�����У��ж��Ƿ��Ѿ����һ��·����
            A = 1;   %���ǣ���·�����У������������Źؽڽ�
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


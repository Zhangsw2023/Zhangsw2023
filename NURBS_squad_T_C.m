clc;clear all;
%% ***�켣NURBS���߲岹+T���ٶȹ滮+squad��̬�岹***zsw***2023.7.14***
%% �������ݵ��λ������̬����
track = load('track1.txt'); % λ��
pt=load('track1.txt'); % λ��
xforplot = track(:,1);
yforplot = track(:,2);
zforplot = track(:,3);
pose = load('rotvect1.txt'); % ��̬
pose = pose(round(linspace(1,numrows(pose),8)),1:3);
D_X = track(:,1); D_Y = track(:,2); D_Z = track(:,3);  % TCP ����
R_X = pose(:,1); R_Y = pose(:,2); R_Z = pose(:,3);  % rpyŷ���ǣ�������ʾ����teach����
m=length(track);
%% �������ݵ������Ƶ㣬�����nurbs���ߵĲ�ֵ��
[P_X,P_Y,P_Z,U,w] =counter_B(D_X, D_Y, D_Z); 
%% ������C(u)�Ȳ���ȡ��
ucount = 0;
pro = 0.001;
u = [];
for i = 1:(1/pro+1)
    u(i) = ucount;
    ucount = ucount + pro;
end
%% �����Ӧ�Ļ�������
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
%% ����u-sģ�ͣ�u����s����ζ���ʽ��
ret=polyFit(s, u, 5);%retΪ1*6���飬��Ϊs=a+b*u+c*u^2+d*u^3+.....
%% T���ٶȲ岹
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
%% ����T�Ͳ岹��λ�á��ٶȡ����ٶ�����
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
%% ����T���ٶȹ滮��ȡB������ɢ��
Tdd = [];
u_i = [];

for i = 1:nn-1
    u = Ts(i, 2)/Ts(nn, 2);
    if u> 1.0
        u = 1.0;
    end
    u_i = [u_i, u];
    
     [Arr]= Bspline1(P_X,P_Y,P_Z,U, u); % B����
     
    Tdd = [Tdd; Arr];
    u = [];
end
%% ��������T���ٶȹ滮���nurbs����
figure(2)
plot3(Tdd(:,1),Tdd(:,2),Tdd(:,3),'b','LineWidth',0.1); %���ĩ�˹켣
hold on ;grid on;
% plot3(P_X,P_Y,P_Z,'m-o','LineWidth',0.1); %���������
% hold on ;grid on;
scatter3(xforplot,yforplot,zforplot,'*r','LineWidth',0.1);
hold on ;grid on;
title('T���ٶȹ滮���nurbs����'); % ͼ����
xlabel('x');     % x���ǩ
ylabel('y');     % y���ǩ
zlabel('z');     % y���ǩ
legend('nurbs����', '��ֵ��'); % ���ͼ��
hold off
%% �������ݵ�Ļ���
Sr_m = [0];
for i = 1:m-1
    arcS = ComputeArcLengthOfBspline(row, P_X, P_Y, P_Z, U, w(i), w(i+1),ss); % �������ݵ��Ӧ�����߻���
    Sr_m = [Sr_m, Sr_m(i) + arcS];
end
%% ���Ӧ�Ļ�������
mm=length(Sr_m);
Sr = [];
for i = 1:mm
    Sr = [Sr, Sr_m(i) / Sr_m(mm)];
end
%% ������Ԫ��
Q = []; % ����������Ԫ��
for i = 1:m
    qx=rpy2rotvec(R_X(i), R_Y(i), R_Z(i));
    q = rotvec2quaternion(qx(1,1), qx(1,2), qx(1,3));
    Q = [Q;q];%ÿһ��q0, q1, q2, q3
end
%% ����̬�Ŀ��Ƶ�
Sn = GetCtlPoints(Q,m);
%% ����̬��Ԫ����ֵ
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
    Qi = Squad(qi1, si1, qi2, si2, pra);%squad��ֵ
    Qisum=[Qisum;Qi];
    Qc=quaternion2rot_vec(Qi);%��Ԫ��to��ת������ñ���Ԫ����rpy����ת������rpyʱatan2���ܽ������180�ȵĽǣ�
    Quat = [Quat; Qc];
end
%% ������Ԫ��q0 q1 q2 q3�Ĳ�ֵ����
figure(3)
plot(1:length(Qisum),Qisum(:,1),'b','LineWidth',1); %q0��Ԫ���ı任
grid on;hold on;
plot(1:length(Qisum),Qisum(:,2),'r','LineWidth',1); %q1��Ԫ���ı任
grid on;hold on;
plot(1:length(Qisum),Qisum(:,3),'m','LineWidth',1); %q2��Ԫ���ı任
grid on;hold on;
plot(1:length(Qisum),Qisum(:,4),'g','LineWidth',1); %q3��Ԫ���ı任
grid on;hold on;

xlabel('����');     % x���ǩ
ylabel('q0 q1 q2 q3');     % y���ǩ.
title('��Ԫ����ֵ'); % ͼ����
legend('q0', 'q1','q2','q3'); % ���ͼ��
hold off;        % �ͷ�ͼ�α���״̬���Ա���Ի����µ�ͼ��
%% ������ת����Ĳ�ֵ����
figure(7)
plot(1:length(Quat),Quat(:,1),'b','LineWidth',1); %
grid on;hold on;
plot(1:length(Quat),Quat(:,2),'r','LineWidth',1); %
grid on;hold on;
plot(1:length(Quat),Quat(:,3),'m','LineWidth',1); %
grid on;hold on;

% xlabel('����');     % x���ǩ
% ylabel('q0 q1 q2 q3');     % y���ǩ.
% title('��Ԫ����ֵ'); % ͼ����
% legend('q0', 'q1','q2'); % ���ͼ��
% hold off;        % �ͷ�ͼ�α���״̬���Ա���Ի����µ�ͼ��
%% ����������ģ��
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
figure(4)
plot3(Tdd(:,1),Tdd(:,2),Tdd(:,3),'b','LineWidth',0.1); %���ĩ�˹켣
hold on ;grid on;
scatter3(xforplot,yforplot,zforplot,'*r','LineWidth',0.1);
robot.plot([0,0,0,0,0,0])
hold on
%% ����ؽڽǲ�ʹ��������nurbs�����˶�
straj=[Tdd,Quat];
T=straj;%��λ������̬��Ϣ�浽T
G = [];  %�����洢�µ��ϰ��ڵ㣬����դ���ͼ�ĸ���
N = size(T,1);
Qj= []; %����·���ϵ����ո���Ĺؽڽ�
c = [0 0 0 0 0 0 0 0]; %�����е��б�����  % AΪ·�������б���������Ϊ1��������Ϊ0
t0=[0 0 0 0 0 0];
e0 = [];
temp0=0;
sum = [];
for i =1:N    %�ֱ�Ը�����м��
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
    [e1,index]=sort(e0);%���� e0 ��������sort ����Ĭ�ϰ���������
    THH = [TH(index(1),:);TH(index(2),:);TH(index(3),:);TH(index(4),:);TH(index(5),:);
        TH(index(6),:);TH(index(7),:);TH(index(8),:)];%������ TH ������Ϊ 1 ���и��Ƶ�һ���µľ���,�������� 0 ��ʼ���������� 1 ��ʼ����ˣ�����Ϊ 1 ���б�ʾ�����еĵڶ��С�

   Qj(i,:) = THH(1,:);
    t0=Qj(i,:);
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
%% �����ؽڽǵı仯
figure(5)
plot(1:length(Qj),Qj(:,1),'b','LineWidth',1); %theta1�ı任
grid on;hold on;
plot(1:length(Qj),Qj(:,2),'r','LineWidth',1); %theta2�ı任
grid on;hold on;
plot(1:length(Qj),Qj(:,3),'m','LineWidth',1); %theta3�ı任
grid on;hold on;
plot(1:length(Qj),Qj(:,4),'k','LineWidth',1); %theta4�ı任
grid on;hold on;
plot(1:length(Qj),Qj(:,5),'c','LineWidth',1); %theta5�ı任
grid on;hold on;
plot(1:length(Qj),Qj(:,6),'g','LineWidth',1); %theta6�ı任
grid on;hold on;

xlabel('����');     % x���ǩ
ylabel('theta1 theta2 theta3 theta4 theta5 theta6');     % y���ǩ.
title('�ؽڽǵı仯'); % ͼ����
legend('theta1', 'theta2','theta3','theta4','theta5','theta6'); % ���ͼ��
hold off;        % �ͷ�ͼ�α���״̬���Ա���Ի����µ�ͼ��
%% ����:��̬�岹t�ı仯
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
% plot(1:length(prasum),prasum(:,1),'b','LineWidth',1); %t�ı任
% xlabel('����');     % x���ǩ
% ylabel('��̬�岹��t');     % y���ǩ.
% title('��̬�岹��t�ı仯'); % ͼ����





clc;clear all;
%% �켣NURBS���߲岹
track = load('track1.txt'); % λ��
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



% step1  ������Ԫ��
Q = []; % ����������Ԫ��
for i = 1:m
    q = rotvec2quaternion(R_X(i), R_Y(i), R_Z(i));
    Q = [Q;q];%ÿһ��q0, q1, q2, q3
end
% step2 �������ݵ㣬�����λ�ÿ��Ƶ�
[P_X,P_Y,P_Z,U,w] = counter_B(D_X,D_Y,D_Z);  %�ڵ�ʸ������Ϊm+2*k,ӦΪ14��  ?

%% �켣NURBS�岹&��̬Squad�岹
straj=[];
k=1; %����λ�˵�����
Sn = GetCtlPoints(Q,m);

for i=1:m-1 %pose�е���
    t=0;
    N = ComDisPntsNum(D_X, D_Y, D_Z, i); %��ֵ����
    t_ = 1/N;
    for j=1:N %����pose֮���n����
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
%% ��ͼ 
straj(:,7)=[];
figure(1)
% ����straj
% writematrix(straj,'straj.txt');
% ���ݵ�
% plot3(D_X,D_Y,D_Z,'.k','LineWidth',4);
plot3(xforplot,yforplot,zforplot,'.k','LineWidth',4);
hold on 
% ���Ƶ�
plot3(P_X,P_Y,P_Z,'.g-','LineWidth',1);
hold on
% ��������
plot3(straj(:,1),straj(:,2),straj(:,3),'r','LineWidth',1); %���ĩ�˹켣
hold on
grid on;
xlabel('x')
ylabel('y')
zlabel('z')
title('NURBS�켣�岹');
legend('���ݵ�','���Ƶ�','��������')
% legend('���ݵ�','��������')
axis equal

figure(2)
plot(1:length(straj),straj(:,4),'b','LineWidth',1); %��Ԫ���ı任
grid on;hold on;
plot(1:length(straj),straj(:,5),'r','LineWidth',1); %��Ԫ���ı任
grid on;hold on;
plot(1:length(straj),straj(:,6),'m','LineWidth',1); %��Ԫ���ı任
grid on;hold on;
xlabel('����');     % x���ǩ
ylabel('q0 q1 q2 ');     % y���ǩ.
title('��Ԫ����ֵ'); % ͼ����
legend('q0', 'q1','q2'); % ���ͼ��
hold off;        % �ͷ�ͼ�α���״̬���Ա���Ի����µ�ͼ��



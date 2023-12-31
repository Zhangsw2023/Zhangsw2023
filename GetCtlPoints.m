function [Sn] = GetCtlPoints(Qn,n)
%GETCTLPOINTS 计算控制点，形参n是控制点数，跟姿态个数相同
%Qn为一系列插值点；
% clc
% clear
% Qn=load('datatestdata.txt');
% [n,m]=size(Qn);
Sn(1,:)=Qn(1,:);%第一行q0, q1, q2, q3
Sn(n,:)=Qn(n,:);%最后一行q0, q1, q2, q3
%一系列四元数qi,qi_m1,qi_a1,qi_conj；
%一系列四元数m0,m1,m0_log,m1_log,m_log_sum,k,k_exp；

fun=sys();
for i=2:n-1
    qi=Qn(i,:);%qi
    qi_m1=Qn(i-1,:);%q i-1
    qi_a1=Qn(i+1,:);%q i+1
    if(fun.sys5(qi,qi_m1)<0)%四元数点积小于0,两个四元数为钝角，需将一个四元数取反
        qi_m1=fun.sys4(qi_m1);
    end
    if(fun.sys5(qi,qi_a1)<0)%四元数点积小于0,两个四元数为钝角，需将一个四元数取反
        qi_a1=fun.sys4(qi_a1);
    end
    qi_conj=fun.sys3(qi);%共轭
    m0=fun.sys2(qi_conj,qi_m1); %四元数乘法
    m1=fun.sys2(qi_conj,qi_a1);
    m0_log=fun.sys7(m0);%对数
    m1_log=fun.sys7(m1);
    m_log_sum=fun.sys1(m0_log,m1_log);
    k=Quat_Smupltipy(m_log_sum,(-1)/4);
    k_exp=fun.sys6(k);
    Sn(i,:)=fun.sys5(qi,k_exp);%原代码Sn(i,:)=fun.sys2(qi,k_exp);应为指数运算
end
end

%一系列四元数基本运算
function [q] = sys1(q1,q2)
%SYS1 四元数加法运算
%其中q1和q2为单位四元数
for i=1:4
    q(i)=q1(i)+q2(i);
end
end

function [q] = sys2(q1,q2)
%SYS2 四元数乘积运算
%q1=[a,b,c,d];q2=[x,y,z,t];q=[m,n,b,v];
%运算法则：m=a*x-b*y-c*z-d*t;
%         n=a*y+b*x+c*t-d*z;
%         b=a*z+c*x+d*y-b*t;
%         v=a*t+d*x+b*z-c*y;
q(1)=q1(1)*q2(1)-q1(2)*q2(2)-q1(3)*q2(3)-q1(4)*q2(4);
q(2)=q1(1)*q2(2)+q1(2)*q2(1)+q1(3)*q2(4)-q1(4)*q2(3);
q(3)=q1(1)*q2(3)+q1(3)*q2(1)+q1(4)*q2(2)-q1(2)*q2(4);
q(4)=q1(1)*q2(4)+q1(4)*q2(1)+q1(2)*q2(3)-q1(3)*q2(2);
end

function [q] = sys3(Q)
%SYS3 四元数共轭运算
q(1)=Q(1);
for i=2:4
q(i)=(-1)*Q(i);
end
end

function [q] = sys4(Q)
%SYS4 四元数取反运算
for i=1:4
    q(i)=(-1)*Q(i);
end
end

function [q] = sys5(q1,q2)
%SYS5 四元数点积运算
q=q1(1)*q2(1)+q1(2)*q2(2)+q1(3)*q2(3)+q1(4)*q2(4);
end

function [q] = sys6(Q)
%SYS6 四元数指数函数
theta=sqrt(Q(2)^2+Q(3)^2+Q(4)^2);
cosa=cos(theta);
%当sina很小时，不能作为分子，用theta代替sin(theta)
if(cosa>0.9995)
    for i=1:4
        q(i)=Q(i);
    end
else
    q=Quat_Smupltipy(Q,sin(theta)/theta);
end
q(1)=cosa;
end

function [q] = sys7(Q)
%SYS7 四元数对数运算
sina=sqrt(Q(2)^2+Q(3)^2+Q(4)^2);
cosa=Q(1);
theta=atan2(sina,cosa);
%当sina很小时，不能作分子，用theta代替sin(theta)
if(cosa>0.9995)
    for i=1:4
        q(i)=Q(i);
    end
else
    q=Quat_Smupltipy(Q,theta/sina);
end
q(1)=0;
end
%% 四元数的标量乘数
function [q]=Quat_Smupltipy(Q,scalar)
for i=1:4
    q(i)=Q(i)*scalar;
end
end



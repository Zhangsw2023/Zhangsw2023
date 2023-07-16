function fun=sys
fun.sys1=@sys1;
fun.sys2=@sys2;
fun.sys3=@sys3;
fun.sys4=@sys4;
fun.sys5=@sys5;
fun.sys6=@sys6;
fun.sys7=@sys7;
end

function [q] = sys1(q1,q2)
%SYS1 ��Ԫ���ӷ�����
%����q1��q2Ϊ��λ��Ԫ��
for i=1:4
    q(i)=q1(i)+q2(i);
end
end

function [q] = sys2(q1,q2)
%SYS2 ��Ԫ���˻�����
%q1=[a,b,c,d];q2=[x,y,z,t];q=[m,n,b,v];
%���㷨��m=a*x-b*y-c*z-d*t;
%         n=a*y+b*x+c*t-d*z;
%         b=a*z+c*x+d*y-b*t;
%         v=a*t+d*x+b*z-c*y;
q(1)=q1(1)*q2(1)-q1(2)*q2(2)-q1(3)*q2(3)-q1(4)*q2(4);
q(2)=q1(1)*q2(2)+q1(2)*q2(1)+q1(3)*q2(4)-q1(4)*q2(3);
q(3)=q1(1)*q2(3)+q1(3)*q2(1)+q1(4)*q2(2)-q1(2)*q2(4);
q(4)=q1(1)*q2(4)+q1(4)*q2(1)+q1(2)*q2(3)-q1(3)*q2(2);
end

function [q] = sys3(Q)
%SYS3 ��Ԫ����������
q(1)=Q(1);
for i=2:4
q(i)=(-1)*Q(i);
end
end

function [q] = sys4(Q)
%SYS4 ��Ԫ��ȡ������
for i=1:4
    q(i)=(-1)*Q(i);
end
end

function [q] = sys5(q1,q2)
%SYS5 ��Ԫ���������
q=q1(1)*q2(1)+q1(2)*q2(2)+q1(3)*q2(3)+q1(4)*q2(4);
end

function [q] = sys6(Q)
%SYS6 ��Ԫ��ָ������
theta=sqrt(Q(2)^2+Q(3)^2+Q(4)^2);
cosa=cos(theta);
%��sina��Сʱ��������Ϊ���ӣ���theta����sin(theta)
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
%SYS7 ��Ԫ����������
 sina=sqrt(Q(2)^2+Q(3)^2+Q(4)^2);
cosa=Q(1);
theta=atan2(sina,cosa);
%��sina��Сʱ�����������ӣ���theta����sin(theta)
if(cosa>0.9995)
    for i=1:4
        q(i)=Q(i);
    end
else
    q=Quat_Smupltipy(Q,theta/sina);
end
q(1)=0;
end
%% ��Ԫ���ı�������
function [q]=Quat_Smupltipy(Q,scalar)
for i=1:4
    q(i)=Q(i)*scalar;
end
end

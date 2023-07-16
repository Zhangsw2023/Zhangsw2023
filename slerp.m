function [q] = slerp(p1,p2,t)
%SLERP ��Ԫ�������ֵ
%   ��Ԫ�������ֵ��p1Ϊ��ֵ��㣬p2Ϊ��ֵ�յ㣬tΪ[0,1]��һ����
%   ������Ԫ��֮��ķ�������
fun=sys();
cosa = fun.sys5(p1,p2);
% cosa = sum(p1.*p2);%aΪ�Ƕ�
% �岹֮ǰӦ���ж�������Ԫ���ĽǶȣ��۽���ת����һ����Ԫ��
% if(cosa<0)
%     cosa=-cosa;
%     p2=-p2;
% end
%���н�̫С��ʹ�����Բ�ֵ��
if(cosa>0.9995)
    k0=1-t;
    k1=t;
%����ʹ�õ���Slerp
else
    sina =sqrt(1 - cosa*cosa);
    a=atan2(sina,cosa);
    k0=sin((1 - t)*a) / sina;
    k1=sin(t*a) / sina;
end
%���ݲ�ֵ��һ����t�����ֵ��Ԫ��
q=k0*p1+k1*p2;
end


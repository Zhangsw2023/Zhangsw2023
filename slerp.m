function [q] = slerp(p1,p2,t)
%SLERP 四元数球面插值
%   四元数球面插值，p1为插值起点，p2为插值终点，t为[0,1]归一化量
%   求两四元数之间的方向余弦
fun=sys();
cosa = fun.sys5(p1,p2);
% cosa = sum(p1.*p2);%a为角度
% 插补之前应该判断两个四元数的角度，钝角则反转其中一个四元数
% if(cosa<0)
%     cosa=-cosa;
%     p2=-p2;
% end
%若夹角太小，使用线性插值法
if(cosa>0.9995)
    k0=1-t;
    k1=t;
%否则使用的是Slerp
else
    sina =sqrt(1 - cosa*cosa);
    a=atan2(sina,cosa);
    k0=sin((1 - t)*a) / sina;
    k1=sin(t*a) / sina;
end
%根据插值归一化量t计算插值四元数
q=k0*p1+k1*p2;
end


function [q] = Rotationtoquateration(T)
%ROTATIONTOQUATERATION q��ʾ������Ԫ��;T����ת����;
%̽����Ԫ���е�������
float fourWSquaredMinus1=T(0)+T(4)+T(8);
float fourXSquaredMinus1=T(0)-T(4)-T(8);
float fourYSquaredMinus1=T(4)-T(0)-T(8);
float fourZSquaredMinus1=T(8)-T(0)-T(4);
biggestIndex=0;
fourBiggestSqureMinus1 = fourWSquaredMinusl;
if(fourXSquaredMinusl>fourBiggestSqureMinus1)
	fourBiggestSqureMinus1 = fourXSquaredMinusl;
	biggestIndex=1;
end
if(fourYSquaredMinusl>fourBiggestSqureMinus1)
	fourBiggestSqureMinus1 = fourYSquaredMinusl;
	biggestIndex=2;
end
if(fourZSquaredMinusl>fourBiggestSqureMinus1)
	fourBiggestSqureMinus1 = fourZSquaredMinusl;
	biggestIndex =3;
end
%����ƽ�����ͳ���
biggestVal=sqrt(fourBiggestSqureMinus1+1.0)*0.5;
mult=0.25/biggestVal;
%������Ԫ����ֵ
switch(biggestIndex)
    case 0
        w=biggestVal;
        x=(T(5)-T(7))*mult;
        y=(T(6)-T(2))*mult;
        z=(T(1)-T(4))*mult;
    case 1
        x=biggestVal;
        w=(T(5)-T(7))*mult;
        y=(T(1)+T(4))*mult;
        z=(T(6)+T(2))*mult;
    case 2
        y=biggestVal;
        w=(T(6)-T(2))*mult;
        x=(T(1)+T(4))*mult;
        z=(T(5)+T(7))*mult;
    case 3
        z=biggestVal;
        w=(T(1)-T(4))*mult;
        x=(T(6)+T(2))*mult;
        y=(T(5)+T(7))*mult;
end
%��Ԫ��q=w+xi+yj+zk;
q(0)=w;q(1)=x;q(2)=y;q(3)=z;
end


function [n,time,q, qd, qdd] = ComDisPntsNum1(L)

    %T�Ͳ岹
    v0 = 0;
	v1 = 0;
	q0 = 0;
	q1 = L;
	vmax = 30;
	amax = 30 ;
    td = 0.01;
    %���n�岹������time��ʱ�䣬qÿ�β岹������qd�ٶ�
    [n,time, q, qd, qdd] = TSpeedCruveMethod(q0,q1,v0,v1,vmax,amax,td);
end
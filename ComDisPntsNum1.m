function [n,time,q, qd, qdd] = ComDisPntsNum1(L)

    %T型插补
    v0 = 0;
	v1 = 0;
	q0 = 0;
	q1 = L;
	vmax = 30;
	amax = 30 ;
    td = 0.01;
    %输出n插补次数，time总时间，q每段插补弧长，qd速度
    [n,time, q, qd, qdd] = TSpeedCruveMethod(q0,q1,v0,v1,vmax,amax,td);
end
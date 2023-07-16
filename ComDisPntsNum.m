function [n,time,q, qd, qdd] = ComDisPntsNum(D_X, D_Y, D_Z, i)

    L = sqrt((D_X(i+1)-D_X(i))^2+(D_Y(i+1)-D_Y(i))^2+(D_Z(i+1)-D_Z(i))^2);
    %T型插补
    v0 = 0;
	v1 = 0;
	q0 = 0;
	q1 = L;
	vmax = 30;
	amax = 25 ;
    td = 0.001;
    %输出n插补次数，time总时间，q每段插补弧长，qd速度
    [n,time, q, qd, qdd] = TSpeedCruveMethod(q0,q1,v0,v1,vmax,amax,td);
end
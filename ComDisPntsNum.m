function [n,time,q, qd, qdd] = ComDisPntsNum(D_X, D_Y, D_Z, i)

    L = sqrt((D_X(i+1)-D_X(i))^2+(D_Y(i+1)-D_Y(i))^2+(D_Z(i+1)-D_Z(i))^2);
    %T�Ͳ岹
    v0 = 0;
	v1 = 0;
	q0 = 0;
	q1 = L;
	vmax = 30;
	amax = 25 ;
    td = 0.001;
    %���n�岹������time��ʱ�䣬qÿ�β岹������qd�ٶ�
    [n,time, q, qd, qdd] = TSpeedCruveMethod(q0,q1,v0,v1,vmax,amax,td);
end
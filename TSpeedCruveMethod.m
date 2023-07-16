function [n,time, q, qd, qdd] = TSpeedCruveMethod(q0,q1,v0,v1,vmax,amax,td)
%% input q0,q1,vo,v1,vmax,amax,td
%% output time,q,qd,qdd
 
v_temp = sqrt((2.0*amax*(q1-q0) + (v1^2 + v0^2)) / 2);
 
if(v_temp<vmax)
    vlim = v_temp;
else
    vlim = vmax;
end
 
Ta = (vlim-v0)/amax;
Sa = v0*Ta+amax*Ta^2/2;
 
Tv = (q1-q0-(vlim^2-v0^2)/(2*amax)-(v1^2-vlim^2)/(2*-amax))/vlim;
Sv = vlim*Tv;
 
Td = (vlim-v1)/amax;
Sd = vlim*Td - amax*Td^2/2;
 
T = Ta + Tv +Td;
k = 1;
for t = 0:td:T
    time(k) = td *k;
    if(t >= 0 && t < Ta)
        q(k) = q0 + v0*t + amax*t^2/2;
        qd(k) = v0 + amax*t;
        qdd(k) = amax;
    elseif(t >= Ta && t < Ta+Tv)
        q(k) = q0 + Sa + vlim*(t - Ta);
        qd(k) = vlim;
        qdd(k) = 0;
    elseif(t >= Ta+Tv && t <= T)
        q(k) = q0 + Sa + Sv + vlim*(t - Ta - Tv) - amax*power(t - Ta - Tv, 2)/2;
        qd(k) = vlim - amax*(t - Ta - Tv);
        qdd(k) = -amax;
    end
    k = k + 1;
end
n=k-1;
end
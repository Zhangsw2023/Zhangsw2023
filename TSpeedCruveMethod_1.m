function [n,time, q, qd, qdd] = TSpeedCruveMethod_1(q0,q1,v0,v1,vmax,amax,td)
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

% function [n] = TSpeedCruveMethod_1(q0, q1, v0, v1, vmax, amax, td, V_T)
%     result = [];
%     
%     v_temp = sqrt((2.0 * amax * (q1 - q0) + (v1 * v1 + v0 * v0)) / 2); % 系统允许的最大速度
%     if v_temp < vmax
%         vlim = v_temp;
%     else
%         vlim = vmax;
%     end
%     
%     Ta = (vlim - v0) / amax;
%     Sa = v0 * Ta + amax * Ta * Ta / 2;
%     Tv = (q1 - q0 - (vlim * vlim - v0 * v0) / (2 * amax) - (v1 * v1 - vlim * vlim) / (2 * -amax)) / vlim;
%     Sv = vlim * Tv;
%     Td = (vlim - v1) / amax;
%     Sd = vlim * Td - amax * Td * Td / 2;
%     T = Ta + Tv + Td;
%     V_T(1) = Ta;
%     V_T(2) = Tv;
%     V_T(3) = Td;
%     
%     k = 1;
%     
%     for t = 0:td:T
%         time = td * k;
%         if t >= 0 && t < Ta
%             q = q0 + v0 * t + amax * t * t / 2;
%             qd = v0 + amax * t;
%             qdd = amax;
%         elseif t >= Ta && t < Ta + Tv
%             q = q0 + Sa + vlim * (t - Ta);
%             qd = vlim;
%             qdd = 0;
%         elseif t >= Ta + Tv && t <= T
%             q = q0 + Sa + Sv + vlim * (t - Ta - Tv) - amax * (t - Ta - Tv) * (t - Ta - Tv) / 2;
%             qd = vlim - amax * (t - Ta - Tv);
%             qdd = -amax;
%         end
%         
%         Tdd = [time, q, qd, qdd];
%         result = [result; Tdd];
%         
%         k = k + 1;
%     end
%     
%     n = k - 1;
% end

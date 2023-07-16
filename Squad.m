function [SquadVector]=Squad(Qi,Si,Qia,Sia,t)
%% Qi、Q(ia)是单位四元数；Si、Sia是控制点；t为参数;
k1=slerp(Qi,Qia,t);
k2=slerp(Si,Sia,t);
SquadVector=slerp(k1,k2,2*t*(1-t));
end

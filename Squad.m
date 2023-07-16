function [SquadVector]=Squad(Qi,Si,Qia,Sia,t)
%% Qi��Q(ia)�ǵ�λ��Ԫ����Si��Sia�ǿ��Ƶ㣻tΪ����;
k1=slerp(Qi,Qia,t);
k2=slerp(Si,Sia,t);
SquadVector=slerp(k1,k2,2*t*(1-t));
end

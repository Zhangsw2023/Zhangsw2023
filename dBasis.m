%% ************K�ηǾ���B��������������************
function N_ = dBasis(i,kk,u,t)
%��i��k��B������,Cox-Deboor���Ƶݹ��㷨
% uΪ�ڵ���������һά����
%tΪ����,u(i)<=t<u(i+1),k=0ʱresult=1;
if(kk==0)
    if(u(i)<=t && t<u(i+1))%ע��1=u(i)<=t<u(i+1)=1ʱ�����,����Ҫ��t<=u(i+1);
        N_=0;
        return;
    else
        N_=0;
        return;
    end
else
    if(u(i+kk)-u(i)==0)
        alpha=0;
    else
        alpha=kk/(u(i+kk)-u(i));
    end
    if(u(i+kk+1)-u(i+1)==0)
         beta=0;
    else
        beta=kk/(u(i+kk+1)-u(i+1));
    end
end
N_=alpha*Base(i,kk-1,u,t)+beta*Base(i+1,kk-1,u,t);
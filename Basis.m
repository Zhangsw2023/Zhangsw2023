%% ************K次非均匀B样条基函数************
function N= Basis(i,kk,u,t)
%第i段k次B样条基,Cox-Deboor递推递归算法
% u为节点向量，是一维数组
%t为变量,u(i)<=t<u(i+1),k=0时result=1;
if(kk==0)
    if(u(i)<=t && t<u(i+1))%注意1=u(i)<=t<u(i+1)=1时的情况,这里要用t<=u(i+1);
        N=1;
        return;
    else
        N=0;
        return;
    end
else
    if(u(i+kk)-u(i)==0)
        alpha=0;
    else
        alpha=(t-u(i))/(u(i+kk)-u(i));
    end
    if(u(i+kk+1)-u(i+1)==0)
         beta=0;
    else
        beta=(u(i+kk+1)-t)/(u(i+kk+1)-u(i+1));
    end
end
N=alpha*Base(i,kk-1,u,t)+beta*Base(i+1,kk-1,u,t);
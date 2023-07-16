function [x, y, z,result] = TanVecBspline(row, X, Y, Z, U, u)  
    k = 2;  
    C = 0;  
    na = row;  
      
    for i = k:na-1  
        if (U(i + 2) >= u && u >= U(i+1))  
            C = i + 1;  
        end  
    end  
      
    x = 0;
    y =0;
    z = 0;  
    for m = C:C +k %m = C- k+1:C +1 
        N_ = TanVecB_basis(u, U, m); %Basis(m-1,3,U,u);  calculate basis function  
        x = x + X(m ) * N_;  
        y = y + Y(m ) * N_;  
        z = z + Z(m ) * N_;  
    end  
      
    result = [x, y, z];  


%  确定3次非均匀B样条曲线,给定控制点
% input 控制点和节点矢量和离散点数
% output 离散点坐标

% k=3;% 三次样条曲线
% n=row;
% T=zeros(k+1,n+3);
% 
% % for u=0:(1/N):1
%     i=0;
%     for Count=k+1:n
%          if U(Count+1)>=u && u>=U(Count)
%             T(1,Count)=1;
%             i=Count;
%          end
%     end
%     
% %     Count=i;
%     PP=i;
%     
%      for j=2:k+1
%          for Count=PP-k:PP
%              for i=Count:Count+k+1-j
%                 if (U(i+j-1)-U(i))==0
%                     if (U(i+j)-U(i+1))==0
%                         T(j,i)=0;
%                     else
%                         T(j,i)=(U(i+j)-u)/(U(i+j)-U(i+1))*T(j-1,i+1);
%                     end
%                 else
%                     if (U(i+j)-U(i+1))==0
%                         T(j,i)=(u-U(i))/(U(i+j-1)-U(i))*T(j-1,i);
%                     else
%                         T(j,i)=(u-U(i))/(U(i+j-1)-U(i))*T(j-1,i)+(U(i+j)-u)/(U(i+j)-U(i+1))*T(j-1,i+1);
%                     end
%                 end   
%                 
%             end 
%          end
%      end
%      
%      x=0;
%      y=0;
%      z=0;
%      
%      for m=Count-k:Count
%          x=x+X(m)*T(k+1,m);
%          y=y+Y(m)*T(k+1,m);
%          z=z+Z(m)*T(k+1,m);
%      end
%       result = [x, y, z];  
end
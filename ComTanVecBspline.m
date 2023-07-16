function f = ComTanVecBspline(row, X, Y, Z, U, x0,s)  
%* INPUT B样条曲线的控制点，节点矢量 还有样条参数C(u)'的模长
%* DESCRIPTION:对3次样条曲线进行求导,求其控制点和节点向量
    p = 3;  
    temp=0;
    f=0;
 
for i=1:s
    N_ = dBasis(i,p,U,x0);
   
    temp= sqrt((N_*X(i)).^2 + (N_*Y(i)).^2 + (N_*Z(i)).^2); 
    
    f=f+temp;
    
end
end
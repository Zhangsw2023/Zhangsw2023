function f = ComTanVecBspline(row, X, Y, Z, U, x0,s)  
%* INPUT B�������ߵĿ��Ƶ㣬�ڵ�ʸ�� ������������C(u)'��ģ��
%* DESCRIPTION:��3���������߽�����,������Ƶ�ͽڵ�����
    p = 3;  
    temp=0;
    f=0;
 
for i=1:s
    N_ = dBasis(i,p,U,x0);
   
    temp= sqrt((N_*X(i)).^2 + (N_*Y(i)).^2 + (N_*Z(i)).^2); 
    
    f=f+temp;
    
end
end
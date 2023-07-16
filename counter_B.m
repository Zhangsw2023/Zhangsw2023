function [ b_x,b_y,b_z,U, w] = counter_B( X,Y,Z )
%UNTITLED Summary of this function goes here
%   给定型值点反算3次非均匀B样条控制点
n=length(X);
for i=1:n-1
    L(i)=sqrt((X(i+1)-X(i))^2+(Y(i+1)-Y(i))^2+(Z(i+1)-Z(i))^2);%总弦长
end
w(1)=0;%积累弦长参数化
for i=2:n
    w(i)=w(i-1)+L(i-1);
end
for i=1:n
    w(i)=w(i)/w(n);
end
for i=1:4%前四个节点矢量为0
    U(i)=0;
end
j=3;
for i=5:n
    U(i)=w(j);
% U(i)=roundn(U(i),-2);
    j=j+1;
end
for i=n+1:n+4%后四个节点矢量为0
    U(i)=1;
end
t1=w(2);
t2=w(n-1);
for i=1:length(U)-1
    du(i)=U(i+1)-U(i);
end
for i=3:n-2
    Lmd(i-1)=(du(i+2)^2)/(du(i)+du(i+1)+du(i+2));
    p(i-1)=du(i+2)*(du(i)+du(i+1))/(du(i)+du(i+1)+du(i+2))+du(i+1)*(du(i+2)+du(i+3))/(du(i+1)+du(i+2)+du(i+3));
    mu(i-1)=du(i+1)^2/(du(i+1)+du(i+2)+du(i+3));
    d_x(i-1)=(du(i+1)+du(i+2))*X(i);
    d_y(i-1)=(du(i+1)+du(i+2))*Y(i);
    d_z(i-1)=(du(i+1)+du(i+2))*Z(i);
end
N(1,1)=((U(5)-t1)/du(4))^3;
N(1,2)=(t1-U(4))/du(4)*(((U(5)-t1)/du(4))^2+(U(6)-t1)/(du(4)+du(5))*((U(5)-t1)/du(4)+(U(6)-t1)/(du(4)+du(5))));%论文中这里少了一个平方
N(1,3)=(t1-U(4))^2/(du(4)*(du(4)+du(5)))*((U(5)-t1)/du(4)+(U(6)-t1)/(du(4)+du(5))+(U(7)-t1)/(du(4)+du(5)+du(6)));
N(1,4)=(t1-U(4))^3/(du(4)*(du(4)+du(5))*(du(4)+du(5)+du(6)));
N(2,1)=(U(n+1)-t2)^3/(du(n)*(du(n-1)+du(n))*(du(n-2)+du(n-1)+du(n)));
N(2,2)=(U(n+1)-t2)^2/(du(n)*(du(n-1)+du(n)))*((t2-U(n-2))/(du(n-2)+du(n-1)+du(n))+(t2-U(n-1))/(du(n-1)+du(n))+(t2-U(n))/du(n));
N(2,3)=(U(n+1)-t2)/du(n)*((t2-U(n-1))/(du(n-1)+du(n))*((t2-U(n-1))/(du(n-1)+du(n))+(t2-U(n))/du(n))+((t2-U(n))/du(n))^2);
N(2,4)=((t2-U(n))/du(n))^3;
Lmd(1)=mu(2)*N(1,1);
p(1)=mu(2)*N(1,2)-Lmd(2)*N(1,4);
mu(1)=mu(2)*N(1,3)-p(2)*N(1,4);
Lmd(n-2)=Lmd(n-3)*N(2,2)-p(n-3)*N(2,1);
p(n-2)=Lmd(n-3)*N(2,3)-mu(n-3)*N(2,1);
mu(n-2)=Lmd(n-3)*N(2,4);
d_x(1)=mu(2)*X(2)-(du(4)+du(5))*N(1,4)*X(3);
d_y(1)=mu(2)*Y(2)-(du(4)+du(5))*N(1,4)*Y(3);
d_z(1)=mu(2)*Z(2)-(du(4)+du(5))*N(1,4)*Z(3);
d_x(n-2)=Lmd(n-3)*X(n-1)-(du(n-1)+du(n))*N(2,1)*X(n-2);
d_y(n-2)=Lmd(n-3)*Y(n-1)-(du(n-1)+du(n))*N(2,1)*Y(n-2);
d_z(n-2)=Lmd(n-3)*Z(n-1)-(du(n-1)+du(n))*N(2,1)*Z(n-2);
d_x=[X(1) d_x X(n)];
d_y=[Y(1) d_y Y(n)];
d_z=[Z(1) d_z Z(n)];
p=[1 p 1];
mu=[0 mu];
Lmd=[Lmd 0];
A=diag(p)+diag(mu,1)+diag(Lmd,-1);
b_x=A\d_x';%控制点x坐标
b_y=A\d_y';%控制点y坐标
b_z=A\d_z';%控制点z坐标
end


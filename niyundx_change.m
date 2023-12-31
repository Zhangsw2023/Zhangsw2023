function TH = niyundx_change(A1)

a2=431.8;a3=20.32;d2=149.09;d4=433.07;

nx=A1(1,1); ox=A1(1,2); ax=A1(1,3); px=A1(1,4);
ny=A1(2,1); oy=A1(2,2); ay=A1(2,3); py=A1(2,4);
nz=A1(3,1); oz=A1(3,2); az=A1(3,3); pz=A1(3,4);

th1(1) = atan2(py,px)-atan2(d2,(sqrt(abs(px^2+py^2-d2^2))));
th1(2) = atan2(py,px)-atan2(d2,-(sqrt(abs(px^2+py^2-d2^2))));

k = (px^2+py^2+pz^2-a2^2-a3^2-d2^2-d4^2)/(2*a2);
th3(1) = atan2(a3,d4)-atan2(k,(sqrt(abs(a3^2+d4^2-k^2))));
th3(2) = atan2(a3,d4)-atan2(k,-(sqrt(abs(a3^2+d4^2-k^2))));

%th23(1,1)的两列，第一列指的是第几个th3,第二列指的是第几个th1；
th23(1,1) = atan2(((-a3-a2*cos(th3(1)))*pz + (cos(th1(1))*px+sin(th1(1))*py)*(a2*sin(th3(1))-...
    d4)),((-d4+a2*sin(th3(1)))*pz-(cos(th1(1))*px+sin(th1(1))*py)*(-a2*cos(th3(1))-a3)));
th23(1,2) = atan2(((-a3-a2*cos(th3(1)))*pz + (cos(th1(2))*px+sin(th1(2))*py)*(a2*sin(th3(1))-...
    d4)),((-d4+a2*sin(th3(1)))*pz-(cos(th1(2))*px+sin(th1(2))*py)*(-a2*cos(th3(1))-a3)));
th23(2,1) = atan2(((-a3-a2*cos(th3(2)))*pz + (cos(th1(1))*px+sin(th1(1))*py)*(a2*sin(th3(2))-...
    d4)),((-d4+a2*sin(th3(2)))*pz-(cos(th1(1))*px+sin(th1(1))*py)*(-a2*cos(th3(2))-a3)));
th23(2,2) = atan2(((-a3-a2*cos(th3(2)))*pz + (cos(th1(2))*px+sin(th1(2))*py)*(a2*sin(th3(2))-...
    d4)),((-d4+a2*sin(th3(2)))*pz-(cos(th1(2))*px+sin(th1(2))*py)*(-a2*cos(th3(2))-a3)));

th2(1,1) = th23(1,1) - th3(1);
th2(1,2) = th23(1,2) - th3(1);
th2(2,1) = th23(2,1) - th3(2);
th2(2,2) = th23(2,2) - th3(2);

th4(1,1) = atan2((-ax*sin(th1(1))+ay*cos(th1(1))),(-ax*cos(th1(1))*cos(th23(1,1))-...
    ay*sin(th1(1))*cos(th23(1,1))+az*sin(th23(1,1))));
th4(1,2) = atan2((-ax*sin(th1(2))+ay*cos(th1(2))),(-ax*cos(th1(2))*cos(th23(1,2))-...
    ay*sin(th1(2))*cos(th23(1,2))+az*sin(th23(1,2))));
th4(2,1) = atan2((-ax*sin(th1(1))+ay*cos(th1(1))),(-ax*cos(th1(1))*cos(th23(2,1))-...
    ay*sin(th1(1))*cos(th23(2,1))+az*sin(th23(2,1))));
th4(2,2) = atan2((-ax*sin(th1(2))+ay*cos(th1(2))),(-ax*cos(th1(2))*cos(th23(2,2))-...
    ay*sin(th1(2))*cos(th23(2,2))+az*sin(th23(2,2))));

s5(1,1) = -(ax*(cos(th1(1))*cos(th23(1,1))*cos(th4(1,1))+sin(th1(1))*sin(th4(1,1)))+...
    ay*(sin(th1(1))*cos(th23(1,1))*cos(th4(1,1))-cos(th1(1))*sin(th4(1,1)))-...
    az*(sin(th23(1,1))*cos(th4(1,1))));
c5(1,1) = ax*(-cos(th1(1))*sin(th23(1,1)))+ay*(-sin(th1(1))*sin(th23(1,1)))+...
    az*(-cos(th23(1,1)));
s5(1,2) = -(ax*(cos(th1(2))*cos(th23(1,2))*cos(th4(1,2))+sin(th1(2))*sin(th4(1,2)))+...
    ay*(sin(th1(2))*cos(th23(1,2))*cos(th4(1,2))-cos(th1(2))*sin(th4(1,2)))-...
    az*(sin(th23(1,2))*cos(th4(1,2))));
c5(1,2) = ax*(-cos(th1(2))*sin(th23(1,2)))+ay*(-sin(th1(2))*sin(th23(1,2)))+...
    az*(-cos(th23(1,2)));
s5(2,1) = -(ax*(cos(th1(1))*cos(th23(2,1))*cos(th4(2,1))+sin(th1(1))*sin(th4(2,1)))+...
    ay*(sin(th1(1))*cos(th23(2,1))*cos(th4(2,1))-cos(th1(1))*sin(th4(2,1)))-...
    az*(sin(th23(2,1))*cos(th4(2,1))));
c5(2,1) = ax*(-cos(th1(1))*sin(th23(2,1)))+ay*(-sin(th1(1))*sin(th23(2,1)))+...
    az*(-cos(th23(2,1)));
s5(2,2) = -(ax*(cos(th1(2))*cos(th23(2,2))*cos(th4(2,2))+sin(th1(2))*sin(th4(2,2)))+...
    ay*(sin(th1(2))*cos(th23(2,2))*cos(th4(2,2))-cos(th1(2))*sin(th4(2,2)))-...
    az*(sin(th23(2,2))*cos(th4(2,2))));
c5(2,2) = ax*(-cos(th1(2))*sin(th23(2,2)))+ay*(-sin(th1(2))*sin(th23(2,2)))+...
    az*(-cos(th23(2,2)));

th5(1,1) = atan2(s5(1,1),c5(1,1));
th5(1,2) = atan2(s5(1,2),c5(1,2));
th5(2,1) = atan2(s5(2,1),c5(2,1));
th5(2,2) = atan2(s5(2,2),c5(2,2));

s6(1,1) = -nx*(cos(th1(1))*cos(th23(1,1))*sin(th4(1,1))-sin(th1(1))*cos(th4(1,1)))-...
    ny*(sin(th1(1))*cos(th23(1,1))*sin(th4(1,1))+cos(th1(1))*cos(th4(1,1)))+...
    nz*(sin(th23(1,1))*sin(th4(1,1)));
c6(1,1) = nx*((cos(th1(1))*cos(th23(1,1))*cos(th4(1,1))+sin(th1(1))*sin(th4(1,1)))*cos(th5(1,1))-...
    cos(th1(1))*sin(th23(1,1))*sin(th5(1,1)))+...
    ny*((sin(th1(1))*cos(th23(1,1))*cos(th4(1,1))-cos(th1(1))*sin(th4(1,1)))*cos(th5(1,1))-...
    sin(th1(1))*sin(th23(1,1))*sin(th5(1,1)))-...
    nz*(sin(th23(1,1))*cos(th4(1,1))*cos(th5(1,1))+cos(th23(1,1))*sin(th5(1,1)));
s6(1,2) = -nx*(cos(th1(2))*cos(th23(1,2))*sin(th4(1,2))-sin(th1(2))*cos(th4(1,2)))-...
    ny*(sin(th1(2))*cos(th23(1,2))*sin(th4(1,2))+cos(th1(2))*cos(th4(1,2)))+...
    nz*(sin(th23(1,2))*sin(th4(1,2)));
c6(1,2) = nx*((cos(th1(2))*cos(th23(1,2))*cos(th4(1,2))+sin(th1(2))*sin(th4(1,2)))*cos(th5(1,2))-...
    cos(th1(2))*sin(th23(1,2))*sin(th5(1,2)))+...
    ny*((sin(th1(2))*cos(th23(1,2))*cos(th4(1,2))-cos(th1(2))*sin(th4(1,2)))*cos(th5(1,2))-...
    sin(th1(2))*sin(th23(1,2))*sin(th5(1,2)))-...
    nz*(sin(th23(1,2))*cos(th4(1,2))*cos(th5(1,2))+cos(th23(1,2))*sin(th5(1,2)));
s6(2,1) = -nx*(cos(th1(1))*cos(th23(2,1))*sin(th4(2,1))-sin(th1(1))*cos(th4(2,1)))-...
    ny*(sin(th1(1))*cos(th23(2,1))*sin(th4(2,1))+cos(th1(1))*cos(th4(2,1)))+...
    nz*(sin(th23(2,1))*sin(th4(2,1)));
c6(2,1) = nx*((cos(th1(1))*cos(th23(2,1))*cos(th4(2,1))+sin(th1(1))*sin(th4(2,1)))*cos(th5(2,1))-...
    cos(th1(1))*sin(th23(2,1))*sin(th5(2,1)))+...
    ny*((sin(th1(1))*cos(th23(2,1))*cos(th4(2,1))-cos(th1(1))*sin(th4(2,1)))*cos(th5(2,1))-...
    sin(th1(1))*sin(th23(2,1))*sin(th5(2,1)))-...
    nz*(sin(th23(2,1))*cos(th4(2,1))*cos(th5(2,1))+cos(th23(2,1))*sin(th5(2,1)));
s6(2,2) = -nx*(cos(th1(2))*cos(th23(2,2))*sin(th4(2,2))-sin(th1(2))*cos(th4(2,2)))-...
    ny*(sin(th1(2))*cos(th23(2,2))*sin(th4(2,2))+cos(th1(2))*cos(th4(2,2)))+...
    nz*(sin(th23(2,2))*sin(th4(2,2)));
c6(2,2) = nx*((cos(th1(2))*cos(th23(2,2))*cos(th4(2,2))+sin(th1(2))*sin(th4(2,2)))*cos(th5(2,2))-...
    cos(th1(2))*sin(th23(2,2))*sin(th5(2,2)))+...
    ny*((sin(th1(2))*cos(th23(2,2))*cos(th4(2,2))-cos(th1(2))*sin(th4(2,2)))*cos(th5(2,2))-...
    sin(th1(2))*sin(th23(2,2))*sin(th5(2,2)))-...
    nz*(sin(th23(2,2))*cos(th4(2,2))*cos(th5(2,2))+cos(th23(2,2))*sin(th5(2,2)));

th6(1,1) = atan2(s6(1,1),c6(1,1));
th6(1,2) = atan2(s6(1,2),c6(1,2));
th6(2,1) = atan2(s6(2,1),c6(2,1));
th6(2,2) = atan2(s6(2,2),c6(2,2));
    
TH(1,1)=th1(1);TH(1,2)=th2(1,1);TH(1,3)=th3(1);TH(1,4)=th4(1,1);TH(1,5)=th5(1,1);TH(1,6)=th6(1,1);
TH(2,1)=th1(2);TH(2,2)=th2(1,2);TH(2,3)=th3(1);TH(2,4)=th4(1,2);TH(2,5)=th5(1,2);TH(2,6)=th6(1,2);
TH(3,1)=th1(1);TH(3,2)=th2(2,1);TH(3,3)=th3(2);TH(3,4)=th4(2,1);TH(3,5)=th5(2,1);TH(3,6)=th6(2,1);
TH(4,1)=th1(2);TH(4,2)=th2(2,2);TH(4,3)=th3(2);TH(4,4)=th4(2,2);TH(4,5)=th5(2,2);TH(4,6)=th6(2,2);

TH(5,1)=th1(1);TH(5,2)=th2(1,1);TH(5,3)=th3(1);TH(5,4)=th4(1,1)+pi;TH(5,5)=-th5(1,1);TH(5,6)=th6(1,1)+pi;
TH(6,1)=th1(2);TH(6,2)=th2(1,2);TH(6,3)=th3(1);TH(6,4)=th4(1,2)+pi;TH(6,5)=-th5(1,2);TH(6,6)=th6(1,2)+pi;
TH(7,1)=th1(1);TH(7,2)=th2(2,1);TH(7,3)=th3(2);TH(7,4)=th4(2,1)+pi;TH(7,5)=-th5(2,1);TH(7,6)=th6(2,1)+pi;
TH(8,1)=th1(2);TH(8,2)=th2(2,2);TH(8,3)=th3(2);TH(8,4)=th4(2,2)+pi;TH(8,5)=-th5(2,2);TH(8,6)=th6(2,2)+pi;

for n=1:8
    for m=1:6
        if TH(n,m)>pi
            TH(n,m)=TH(n,m)-2*pi;
        elseif TH(n,m)<-pi
            TH(n,m)=TH(n,m)+2*pi;
        end
    end
end
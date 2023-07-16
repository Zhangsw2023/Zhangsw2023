function [q] = rotvec2quaternion(rx,ry,rz)
% ����ת����������Ԫ��
theta = sqrt(rx*rx + ry*ry + rz*rz);
kx = rx/theta;
ky = ry/theta;
kz = rz/theta;
cth = cos(theta);
sth = sin(theta);
vth = 1-cos(theta);

r11 = kx*kx*vth + cth;
r12 = kx*ky*vth - kz*sth;
r13 = kx*kz*vth + ky*sth;% nx ny nz
r21 = kx*ky*vth + kz*sth;
r22 = ky*ky*vth + cth;
r23 = ky*kz*vth - kx*sth;% ox oy oz
r31 = kx*kz*vth - ky*sth;
r32 = ky*kz*vth + kx*sth;
r33 = kz*kz*vth + cth;% ax ay az

% R = [r11 r12 r13; r21 r22 r23; r31 r32 r33];

   q0 = sqrt(1 + r11 + r22 + r33)/2 ;

if (q0 ~= 0 && (1 + r11 + r22 + r33)>0)
    
    q1 = (r32 - r23)/(4*q0);
    q2 = (r13 - r31)/(4*q0);
    q3 = (r21 - r12)/(4*q0);
elseif (q0 == 0 && (r11 + r22 + r33) == -1)%q0��ֵ�ӽ�0,��Ԫ���鲿������޷�ȷ����������
    if max([r11, r22, r33]) == r11
        t = sqrt(1 + r11 - r22 - r33);
        q0 = (r32 - r23)/t;
        q1 = t/4;
        q2 = (r13 + r31)/t;
        q3 = (r12 + r21)/t;
    elseif max([r11, r22, r33]) == r22
        t = sqrt(1 - r11 + r22 - r33);
        q0 = (r13 - r31)/t;
        q1 = (r12 + r21)/t;
        q2 = t/4;
        q3 = (r32 + r23)/t;
    else
        t = sqrt(1 - r11 - r22 + r33);
        q0 = (r21 - r12)/t;
        q1 = (r13 + r31)/t;
        q2 = (r23 - r32)/t;
        q3 = t/4;
    end
end
    q = [q0, q1, q2, q3];
end
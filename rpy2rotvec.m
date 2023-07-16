function [rotation_vector] = rpy2rotvec(roll,pitch,yaw)
  
alpha = yaw;
beta = pitch;
gamma = roll;

% alpha = deg2rad(yaw);
% beta = deg2rad(pitch);
% gamma = deg2rad(roll);

ca = cos(alpha);
cb = cos(beta);
cg = cos(gamma);
sa = sin(alpha);
sb = sin(beta);
sg = sin(gamma);

r11 = ca*cb;
r12 = ca*sb*sg-sa*cg;
r13 = ca*sb*cg+sa*sg;
r21 = sa*cb;
r22 = sa*sb*sg+ca*cg;
r23 = sa*sb*cg-ca*sg;
r31 = -sb;
r32 = cb*sg;
r33 = cb*cg;

theta = acos((r11+r22+r33-1)/2);
sth = sin(theta);
kx = (r32-r23)/(2*sth);
ky = (r13-r31)/(2*sth);
kz = (r21-r12)/(2*sth);

rx = theta * kx;
ry = theta * ky;
rz = theta * kz;

rotation_vector = [rx, ry, rz];
end

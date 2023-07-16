function rot_vec = quaternion2rot_vec(q)
    % 由四元数构造旋转矩阵
    r11 = 1.0 - 2.0 * q(3) * q(3) - 2.0 * q(4) * q(4);
    r12 = 2.0 * q(2) * q(3) - 2.0 * q(4) * q(1);
    r13 = 2.0 * q(2) * q(4) + 2.0 * q(3) * q(1);
    r21 = 2.0 * q(2) * q(3) + 2.0 * q(4) * q(1);
    r22 = 1.0 - 2.0 * q(2) * q(2) - 2.0 * q(4) * q(4);
    r23 = 2.0 * q(3) * q(4) - 2.0 * q(2) * q(1);
    r31 = 2.0 * q(2) * q(4) - 2.0 * q(3) * q(1);
    r32 = 2.0 * q(3) * q(4) + 2.0 * q(2) * q(1);
    r33 = 1.0 - 2.0 * q(2) * q(2) - 2.0 * q(3) * q(3);
    
    % 转化为rot_vec
    theta = acos((r11 + r22 + r33 - 1) / 2);
    sth = sin(theta);
    kx = (r32 - r23) / (2 * sth);
    ky = (r13 - r31) / (2 * sth);
    kz = (r21 - r12) / (2 * sth);
    
    rot_vec = [theta * kx, theta * ky, theta * kz];
end

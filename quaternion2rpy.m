function [RPY] = quaternion2rpy(q)
% ×ª»¯Îªrot_vec
roll = atan2(2*(q(1)*q(2) + q(3)*q(4)), 1 - 2*(q(2)*q(2) + q(3)*q(3)));
pitch = asin(2*(q(1)*q(3) - q(2)*q(4)));
yaw = atan2(2*(q(1)*q(4) + q(2)*q(3)), 1 - 2*(q(3)*q(3) + q(4)*q(4)));
% if yaw>pi
%     yaw=-yaw;
% end    
RPY = [roll pitch yaw];
end
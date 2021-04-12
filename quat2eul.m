function [Roll Pitch Yaw] = quat2eul(quat)

for i = 1:size(quat,1)
    Roll(i,1) = atan2(2*(quat(i,3)*quat(i,4)+quat(i,1)*quat(i,2)),  quat(i,1)*quat(i,1) - quat(i,2)*quat(i,2) - quat(i,3)*quat(i,3) + quat(i,4)*quat(i,4));
    Pitch(i,1) = -asin(2*(quat(i,2)*quat(i,4)-quat(1)*quat(i,3)));
    Yaw(i,1) = atan2(2*(quat(i,2)*quat(i,3)+quat(1)*quat(i,4)), quat(i,1)*quat(i,1) + quat(i,2)*quat(i,2) - quat(i,3)*quat(i,3) - quat(i,4)*quat(i,4));
end
% 本函数为四元数转换为姿态角
% 输入为四元数向量Quat
% 输出为姿态角Ang列向量（俯仰，横滚，航向）

function a_out = Q2A(q_in)

a_out = zeros(1,3);

a_out(1) = atan2(2*(q_in(1)*q_in(2)+q_in(3)*q_in(4)),1-2*(q_in(2)^2+q_in(3)^2));
a_out(2) = asin(2*(q_in(1)*q_in(3)-q_in(4)*q_in(2)));
a_out(3) = atan2(2*(q_in(1)*q_in(4)+q_in(2)*q_in(3)),1-2*(q_in(3)^2+q_in(4)^2));

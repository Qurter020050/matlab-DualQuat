% 本函数为四元数转换为姿态角
% 输入为四元数向量Quat
% 输出为姿态角Ang列向量（俯仰，横滚，航向）

function a_out = Q2A(q_in)

a_out = M_M2A(Q_Q2M(q_in));

% 本函数用于求取对偶四元数的共轭
% 输入为对偶四元数 dq_in
% 输出为对偶四元数的共轭对偶四元数，dq_Conj_out

function dq_Conj_out = DQ_Conj(dq_in)

dq_Conj = dq_in;
dq_Conj(2:4) = -dq_in(2:4);
dq_Conj(6:8) = -dq_in(6:8);
dq_Conj_out = dq_Conj;
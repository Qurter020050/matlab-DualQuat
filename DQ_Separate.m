% 本函数用于拆分对偶四元数，将其分解为旋转四元数与平移矢量
% 输入为对偶四元数dq_in;
% 输出为旋转四元数q_out 和平移矢量vi_out(相对于惯性参考系),vb_out(相对于载体系)

function [q_out, v_out] = DQ_Separate(dq_in,state)

dq_real = dq_in(1:4);
dq_dual = dq_in(5:8);

dq_real_conj = Q_Conj(dq_real);
% dq_real_conj= dq_real_conj / norm(dq_real_conj);

q_out = dq_real;

vb_out = 2*Q_Mul(dq_real_conj, dq_dual);
vi_out = 2*Q_Mul(dq_dual, dq_real_conj);
vi_out = vi_out(2:4);
vb_out = vb_out(2:4);

if state == 1
    v_out = vi_out;
elseif state == -1
    v_out = vb_out;
end

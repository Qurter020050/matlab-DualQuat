% 本函数用于求取对偶四元数的范数
% 输入为对偶四元数dq_in或旋量s_in
% 输出为对应的范数dq_norm_out（输出形式也为dq的形式，由于对偶四元数或螺旋矢量的表示形式为（p+Ep‘）的形式，故其范数也为（p_n+Ep_n’）的形式，其中实部和对偶部分为对应的模长

function dq_norm_out = D_Norm(dq_in)


dq_norm_out = zeros(1,8);

dq_conj = DQ_Conj(dq_in);
dq_norm_out = DQ_Mul(dq_in,dq_conj);
dq_norm_out(1) = sqrt(dq_norm_out(1));

dq_norm_out(5) = 0.5*dq_norm_out(5)/dq_norm_out(1);
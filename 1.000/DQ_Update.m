

function dq_out = DQ_Update(dq_pre, d_in)

	d_in_norm = D_Norm(d_in);
	d_in_norm_half = 0.5*d_in_norm;
	
	dq_update_cos = zeros(1,8);				%构建更新对偶四元数的余弦、正弦向釿
	dq_update_sin = zeros(1,8);

	dq_update_cos(1) = cos(d_in_norm_half(1));
	dq_update_cos(5) = -d_in_norm_half(5)*sin(d_in_norm_half(1));
	dq_update_sin(1) = sin(d_in_norm_half(1));
	dq_update_sin(5) = d_in_norm_half(5)*cos(d_in_norm_half(1));

	dq_update_unit = DQ_Unit(d_in, d_in_norm);				%生成单位螺旋向量，用以表示螺旋轴

	dq_update = dq_update_cos + DQ_Mul(dq_update_sin, dq_update_unit);			%
	
	dq_out = DQ_Mul(dq_pre,dq_update);
	dq_out = DQ_Unit(dq_out);
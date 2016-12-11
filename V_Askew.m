% 本函数用于求取向量的反对称矩阵；
% 函数输入为列向量Vec；
% 函数输出为列向量的反对称矩阵MatX；

function m_X_out = V_Askew(v_in)

m_X_out = [0,		-v_in(3),	v_in(2);
		v_in(3),		0,		-v_in(1);
		-v_in(2), v_in(1),		0];
		

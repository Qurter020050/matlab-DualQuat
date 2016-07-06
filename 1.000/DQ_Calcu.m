% 本函数用于�?过旋转四元数与平移矢量求取对偶四元数,具体算法参�?《算法研究�?�?���?0�?
%输入为旋转四元数q_in，平移矢量v_in�?
% 输出为对偶四元数dq_out�?

function dq_out = DQ_Calcu(q_in, v_in)

%	q_in = q_in/norm(q_in);
	if(length(v_in)==4&&v_in(1)==0)
	v_in = v_in(2:4);
	end
		
	m_VecMul(:,1) = [0; v_in'];
	m_VecMul(:,2) = [-v_in(1); 0; v_in(3); -v_in(2)];
	m_VecMul(:,3) = [-v_in(2); -v_in(3); 0; v_in(1)];
	m_VecMul(:,4) = [-v_in(3); v_in(2); -v_in(1); 0];

	dq_dual = 0.5*m_VecMul*q_in';
	dq_out = [q_in, dq_dual'];
	
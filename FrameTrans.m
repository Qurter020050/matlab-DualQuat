function v_out = FrameTrans(q_in, v_in, state)

	v_in_dim = length(v_in);
	
	if v_in_dim == 3
		v_in = [0, v_in];
		
	end
	
	if state == 1
	v_out = Q_Mul(Q_Conj(q_in), v_in);
	v_out = Q_Mul(v_out,q_in);
	elseif state == -1
	v_out = Q_Mul(q_in, v_in);
	v_out = Q_Mul(v_out,Q_Conj(q_in));
	else error('FramesTrans is wrong')
	end 
	
	if v_in_dim == 3
		v_out(1) = [];
	end
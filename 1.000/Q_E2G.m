function q_eg_out = Q_E2G(v_pos_in)

	long = v_pos_in(1);
	lat = v_pos_in(2);		%取出纬度信息
	h = v_pos_in(3);

	att = [0.5*pi-lat 0 0.5*pi+long];
	
	q_eg_out = A_A2Q(att);	
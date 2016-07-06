% ���������ڽ��ж�ż��Ԫ����λ��
% ����Ϊ��ż��Ԫ��dq_in��
% ���Ϊ��λ����Ķ�ż��Ԫ��dq_unit_out;

function dq_unit_out = DQ_Unit(dq_in, dq_norm)
	
	if nargin == 1;
		dq_norm = D_Norm(dq_in);
	end

	dq_unit_real = dq_in(1:4)/dq_norm(1);
	dq_unit_dual = dq_norm(1)*dq_in(5:8) - dq_norm(5)*dq_in(1:4);
	dq_unit_dual = dq_unit_dual/(dq_norm(1)^2);

	dq_unit_out = [dq_unit_real, dq_unit_dual];
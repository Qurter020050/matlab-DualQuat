% ���������ڽ���̬����ת��Ϊ��̬��
% ����Ϊ��̬����m_in
% ���Ϊ��̬��a_out
% ��̬�����˳��ΪX-������Y-�����Z-����

function a_out = M_M2A(m_in)

a_out(1) = asin(m_in(3,2));
a_out(2) = atan2(-m_in(3,1),m_in(3,3));
a_out(3) = atan2(-m_in(2,1),m_in(2,2));

 if a_out(3) < 0
	 a_out(3) = a_out(3) + 2*pi;
end
% ���������ڽ���̬����ת��Ϊ��ת��Ԫ��
% ����Ϊ��ת����m_in
% ���Ϊ��Ӧ����ת��Ԫ��q_out

function q_out = M_M2Q(m_in)

q_out(1) = 0.5*sqrt(abs(1.0+m_in(1,1)+m_in(2,2)+m_in(3,3)));
q_out(2) = sign(m_in(3,2)-m_in(2,3))*0.5*sqrt(abs(1.0+m_in(1,1)-m_in(2,2)-m_in(3,3)));
q_out(3) = sign(m_in(1,3)-m_in(3,1))*0.5*sqrt(abs(1.0-m_in(1,1)+m_in(2,2)-m_in(3,3)));
q_out(4) = sign(m_in(2,1)-m_in(1,2))*0.5*sqrt(abs(1.0-m_in(1,1)-m_in(2,2)+m_in(3,3)));
% ������Ϊ��Ԫ��ת��Ϊ��̬��
% ����Ϊ��Ԫ������Quat
% ���Ϊ��̬��Ang�����������������������

function a_out = Q2A(q_in)

a_out = zeros(1,3);

a_out(1) = atan2(2*(q_in(1)*q_in(2)+q_in(3)*q_in(4)),1-2*(q_in(2)^2+q_in(3)^2));
a_out(2) = asin(2*(q_in(1)*q_in(3)-q_in(4)*q_in(2)));
a_out(3) = atan2(2*(q_in(1)*q_in(4)+q_in(2)*q_in(3)),1-2*(q_in(3)^2+q_in(4)^2));

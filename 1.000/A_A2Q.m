% ���������ڽ���̬��ת��Ϊ��̬��Ԫ��
% ����Ϊ��̬������a_In��
% ���Ϊ��̬��Ԫ��q_Out��


function q_out = A_A2Q(a_in)

q_out = M_M2Q(A_A2M(a_in));

q_out = q_out/norm(q_out);				%��׼��
% ������Ϊ��Ԫ���˷�����
% �������Ϊ����Ԫ��������q_in1��q_in2;
% ���Ϊ��Ԫ���˻�������qout��

function q_Multi_out = Q_Mul(q_in1 , q_in2)

q_Multi_out =[	q_in1(1)*q_in2(1) - q_in1(2)*q_in2(2) - q_in1(3)*q_in2(3) - q_in1(4)*q_in2(4) 
				q_in1(1)*q_in2(2) + q_in1(2)*q_in2(1) + q_in1(3)*q_in2(4) - q_in1(4)*q_in2(3) 
				q_in1(1)*q_in2(3) + q_in1(3)*q_in2(1) + q_in1(4)*q_in2(2) - q_in1(2)*q_in2(4) 
				q_in1(1)*q_in2(4) + q_in1(4)*q_in2(1) + q_in1(2)*q_in2(3) - q_in1(3)*q_in2(2)]';

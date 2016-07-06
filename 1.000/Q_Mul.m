% 本程序为四元数乘法程序
% 输入变量为两四元数列向量q_in1、q_in2;
% 输出为四元数乘积列向量qout；

function q_Multi_out = Q_Mul(q_in1 , q_in2)

q_Multi_out =[	q_in1(1)*q_in2(1) - q_in1(2)*q_in2(2) - q_in1(3)*q_in2(3) - q_in1(4)*q_in2(4) 
				q_in1(1)*q_in2(2) + q_in1(2)*q_in2(1) + q_in1(3)*q_in2(4) - q_in1(4)*q_in2(3) 
				q_in1(1)*q_in2(3) + q_in1(3)*q_in2(1) + q_in1(4)*q_in2(2) - q_in1(2)*q_in2(4) 
				q_in1(1)*q_in2(4) + q_in1(4)*q_in2(1) + q_in1(2)*q_in2(3) - q_in1(3)*q_in2(2)]';

% 本函数用于对偶四元数的乘法
% 输入为两个对偶四元数dq_in1、dq_in2;其中对偶四元数
% 输出为对偶四元数乘积dq_out;

% This function is used to do the quaternion multiplication.
% The inputs are two dual quaternion D_Quatin1 and D_Quatin2.
% The output is the multiplication of this two dual quaternion D_Quatout.


function dq_out = DQ_Mul(dq_in1, dq_in2)

dq1_real = dq_in1(1:4);
dq1_dual = dq_in1(5:8);

dq2_real = dq_in2(1:4);
dq2_dual = dq_in2(5:8);

dq_out(1:4) = Q_Mul(dq1_real,dq2_real);								%output the real part of the calculation result
dq_out(5:8) = Q_Mul(dq1_dual,dq2_real) + Q_Mul(dq1_real,dq2_dual);	%output the dual part of the calculation result

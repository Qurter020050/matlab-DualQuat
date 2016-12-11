% 本函数用于将四元数转化为姿态矩阵
% 输入为四元数列向量q_in；
% 输出为姿态矩阵m_out；
% 姿态矩阵为Cnb，由于加表陀螺测量均为相对于载体系b的数据，故需要Cnb矩阵将其转换为参考系n下的值，以便今后计算

function m_out = Q_Q2M(q_in)

m_out(1,1) = q_in(1)^2 + q_in(2)^2 - q_in(3)^2 - q_in(4)^2;
m_out(1,2) = 2*(q_in(2)*q_in(3) - q_in(1)*q_in(4));
m_out(1,3) = 2*(q_in(2)*q_in(4) + q_in(1)*q_in(3));
m_out(2,1) = 2*(q_in(2)*q_in(3) + q_in(1)*q_in(4));
m_out(2,2) = q_in(1)^2 + q_in(3)^2 - q_in(2)^2 - q_in(4)^2;
m_out(2,3) = 2*(q_in(3)*q_in(4) - q_in(1)*q_in(2));
m_out(3,1) = 2*(q_in(2)*q_in(4) - q_in(1)*q_in(3));
m_out(3,2) = 2*(q_in(3)*q_in(4) + q_in(1)*q_in(2));
m_out(3,3) = q_in(1)^2 + q_in(4)^2 - q_in(2)^2 - q_in(3)^2;

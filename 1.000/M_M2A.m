% 本函数用于将姿态矩阵转化为姿态角
% 输入为姿态矩阵m_in
% 输出为姿态角a_out
% 姿态角输出顺序为X-俯仰，Y-横滚，Z-航向

function a_out = M_M2A(m_in)

a_out(1) = asin(m_in(3,2));
a_out(2) = atan2(-m_in(3,1),m_in(3,3));
a_out(3) = atan2(-m_in(2,1),m_in(2,2));

 if a_out(3) < 0
	 a_out(3) = a_out(3) + 2*pi;
end
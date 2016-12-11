% 本函数用于将姿态角转换为姿态矩阵CEb
% 输入为姿态角向量a_in;
% 输出为姿态解算矩阵m_out;
% 姿态角向量a_in顺序为：x-俯仰角，y-横滚角，z-航向角；
% 旋转顺序为：航线、俯仰、横滚
% 姿态矩阵为Cnb，由于加表陀螺测量均为相对于载体系b的数据，故需要Cnb矩阵将其转换为参考系n下的值，以便今后计算

function m_out = A_A2M(a_in)
	
	spitch = sin(a_in(1));	cpitch = cos(a_in(1));
	sroll = sin(a_in(2));	croll = cos(a_in(2));
	syaw = sin(a_in(3));	cyaw = cos(a_in(3));
	
	
	m_out =	[croll*cyaw+spitch*sroll*syaw, syaw*cpitch,	cyaw*sroll-spitch*croll*syaw;
            -croll*syaw+spitch*sroll*cyaw,	cpitch*cyaw,-sroll*syaw-spitch*croll*cyaw;
				-cpitch*sroll,			spitch,		cpitch*croll		];

% 本文件用于定义有关全局变量。

	global glv
	
%	基本单位转化及定义
	glv.deg = pi/180;							%角度转化为弧度
	glv.rad = 180/pi;							%弧度转化为角度
	glv.min = glv.deg/60;          				%角分化为弧度
    glv.sec = glv.min/60;          				%角秒化为弧度
    glv.hur = 3600;            					%小时化为秒
    glv.dph = glv.deg/glv.hur;      			%度/秒（°/s）化为弧度/小时（rad/h）	
	
%	地球相关参数
	glv.Re = 6378137;               			%地球半径(m)(WGS-84系)
	% glv.Re = 6378160;							%学长系 =.=
	glv.f = 1/298.257;                			%地球扁率
	glv.Rp = (1-glv.f)*glv.Re;				
	glv.e = sqrt(2*glv.f-glv.f^2);  			%地球椭圆度(比传统公式少一次乘方)
	glv.e2 = glv.e^2;
	glv.wie = 7.2921151467e-5;					%地球自转角速度(rad)		
	glv.g0 = 9.7803267714;						%重力加速度
	glv.mg = glv.g0 * 1.0e-3;					%毫重力加速度
	glv.ug = glv.g0 * 1.0e-6;					%微重力加速度
	
%	解算程序参数
	glv.Ts = 0.005;								%采样时间
	glv.n = 4;									%解算周期倍数（子样数）
	glv.Tn = glv.n*glv.Ts;

	
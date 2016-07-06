% 本函数用于计算当地的地球相对于导航系的自转角速度以及导航系的角速度
% 输入为经纬度pos（经度、纬度、高程），相对于导航系的速度向量（东向,北向，天向）；
% 输出为当前位置的地球相对于惯性系和相对于导航系的角速度w_nie,w_nen，当地子午圈、卯酉圈曲率半径RNh,RMh，当地重力gn；

function [w_nie, w_nen, retp, gn] = earth(pos, vn)
	global glv
	
	if nargin == 1;
		vn = [0, 0, 0];
	end
	
	sinLo = sin(pos(2));	
	cosLo = cos(pos(2));
	tanLo = sinLo/cosLo;	
	sin2Lo = sinLo^2;		
	sin4Lo = sin2Lo^2;
	
	w_nie = glv.wie*[0, cosLo, sinLo]; 									% 导航坐标系相对于惯性坐标的旋转角速度
	sq = 1- glv.e2*sin2Lo;		sq2 = sqrt(sq);
	RMh = glv.Re * (1-glv.e2)/sq/sq2 + pos(3);		% 参考严恭敏硕 3.1.5中（式3.1.4a/b）
	RNh = glv.Re /sq2 + pos(3);
	w_nen = [-vn(2)/RMh, vn(1)/RNh, vn(1)/RNh*tanLo];
	
	g = glv.g0*(1+5.27094e-3*sin2Lo+2.32718e-5*sin4Lo)-3.086e-6*pos(3); % grs80
	gn = [0, 0, -g];
	
	retp.g = g;
    retp.sinLo = sinLo; retp.cosLo = cosLo; retp.tanLo = tanLo; retp.sin2Lo = sin2Lo;
    retp.RMh = RMh; retp.RNh = RNh;%retp保存当前的一些信息量

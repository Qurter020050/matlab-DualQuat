% ���ļ����ڶ����й�ȫ�ֱ�����

	global glv
	
%	������λת��������
	glv.deg = pi/180;							%�Ƕ�ת��Ϊ����
	glv.rad = 180/pi;							%����ת��Ϊ�Ƕ�
	glv.min = glv.deg/60;          				%�Ƿֻ�Ϊ����
    glv.sec = glv.min/60;          				%���뻯Ϊ����
    glv.hur = 3600;            					%Сʱ��Ϊ��
    glv.dph = glv.deg/glv.hur;      			%��/�루��/s����Ϊ����/Сʱ��rad/h��	
	
%	������ز���
	glv.Re = 6378137;               			%����뾶(m)(WGS-84ϵ)
	% glv.Re = 6378160;							%ѧ��ϵ =.=
	glv.f = 1/298.257;                			%�������
	glv.Rp = (1-glv.f)*glv.Re;				
	glv.e = sqrt(2*glv.f-glv.f^2);  			%������Բ��(�ȴ�ͳ��ʽ��һ�γ˷�)
	glv.e2 = glv.e^2;
	glv.wie = 7.2921151467e-5;					%������ת���ٶ�(rad)		
	glv.g0 = 9.7803267714;						%�������ٶ�
	glv.mg = glv.g0 * 1.0e-3;					%���������ٶ�
	glv.ug = glv.g0 * 1.0e-6;					%΢�������ٶ�
	
%	����������
	glv.Ts = 0.005;								%����ʱ��
	glv.n = 4;									%�������ڱ�������������
	glv.Tn = glv.n*glv.Ts;

	
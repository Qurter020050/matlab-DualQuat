% ���������ڽ���̬��ת��Ϊ��̬����CEb
% ����Ϊ��̬������a_in;
% ���Ϊ��̬�������m_out;
% ��̬������a_in˳��Ϊ��x-�����ǣ�y-����ǣ�z-����ǣ�
% ��ת˳��Ϊ�����ߡ����������
% ��̬����ΪCnb�����ڼӱ����ݲ�����Ϊ���������ϵb�����ݣ�����ҪCnb������ת��Ϊ�ο�ϵn�µ�ֵ���Ա������

function m_out = A_A2M(a_in)
	
	spitch = sin(a_in(1));	cpitch = cos(a_in(1));
	sroll = sin(a_in(2));	croll = cos(a_in(2));
	syaw = sin(a_in(3));	cyaw = cos(a_in(3));
	
	
	m_out =	[croll*cyaw+spitch*sroll*syaw, syaw*cpitch,	cyaw*sroll-spitch*croll*syaw;
            -croll*syaw+spitch*sroll*cyaw,	cpitch*cyaw,-sroll*syaw-spitch*croll*cyaw;
				-cpitch*sroll,			spitch,		cpitch*croll		];

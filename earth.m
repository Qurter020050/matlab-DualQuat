% ���������ڼ��㵱�صĵ�������ڵ���ϵ����ת���ٶ��Լ�����ϵ�Ľ��ٶ�
% ����Ϊ��γ��pos�����ȡ�γ�ȡ��̣߳�������ڵ���ϵ���ٶ�����������,�������򣩣�
% ���Ϊ��ǰλ�õĵ�������ڹ���ϵ������ڵ���ϵ�Ľ��ٶ�w_nie,w_nen����������Ȧ��î��Ȧ���ʰ뾶RNh,RMh����������gn��

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
	
	w_nie = glv.wie*[0, cosLo, sinLo]; 									% ��������ϵ����ڹ����������ת���ٶ�
	sq = 1- glv.e2*sin2Lo;		sq2 = sqrt(sq);
	RMh = glv.Re * (1-glv.e2)/sq/sq2 + pos(3);		% �ο��Ϲ���˶ 3.1.5�У�ʽ3.1.4a/b��
	RNh = glv.Re /sq2 + pos(3);
	w_nen = [-vn(2)/RMh, vn(1)/RNh, vn(1)/RNh*tanLo];
	
	g = glv.g0*(1+5.27094e-3*sin2Lo+2.32718e-5*sin4Lo)-3.086e-6*pos(3); % grs80
	gn = [0, 0, -g];
	
	retp.g = g;
    retp.sinLo = sinLo; retp.cosLo = cosLo; retp.tanLo = tanLo; retp.sin2Lo = sin2Lo;
    retp.RMh = RMh; retp.RNh = RNh;%retp���浱ǰ��һЩ��Ϣ��

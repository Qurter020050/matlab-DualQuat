% ���������ڸ��ݼ�����̬�任����Cnb������ϵͳ�Ľ���ʽ�ֶ�׼
% ����Ϊ���������ϵ������ڵ���ϵ�Ļ�׼���ٶȣ���ת���ٶȣ��Լ�������wbie��gb��wnie��gn�������ڲ���漰����������������������˳��Ӧ��һ��
% ���Ϊ��̬�任����Cnb
% ����ԭ��ο��Ϲ���3.5.1�ֶ�׼

function Cnb = dv2att(varb1, varb2, varn1, varn2)

	varb1 = norm(varn1)/norm(varb1)*varb1;						%��vb1������ȡ����Ϊvn1������
	varb2 = norm(varn2)/norm(varb2)*varb2;						%��vb2������ȡ����Ϊvn2��������������ڱ�׼����
	varbtmp = cross(varb1,varb2);
	varntmp = cross(varn1,varn2);

	%Cnb = [ varn1'; varn2'; varntmp';] \ [ varb1'; varb2'; varbtmp';];		%�Ϲ���3.5.1 �ֶ�׼��ʽ3.5-4
	Cnb = [ varn1; varntmp; cross(varn1,varntmp)] \ [varb1; varbtmp; cross(varb1,varbtmp)];					%ѧ��������ϵѡȡ��������ȫ����������ϵ�����˾�������g��w֮��ĽǶȹ�ϵ���ֶ�׼����Ӧ�ø����Ϲ�����ʽ�ӣ���������׼��Ӧ���޲��

	Cnb = Cnb*(Cnb'*Cnb)^(-1/2);		%Cnb��׼��


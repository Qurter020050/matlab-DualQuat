%	���ļ�Ϊ�����ߵ�ϵͳ����������
%	��������У�ȫ��ʹ�û��ȡ�Сʱ��Ϊ��׼��λ
%	���������У�dqǰ׺��ʾ��ż��Ԫ�����ͣ�q��ʾ��Ԫ�����ͣ�m��ʾ��������
%	v��ʾ�ٶ�������p��ʾλ�ã�a��ʾ��̬�ǣ�����Թ�ϵ�ο������±�

%% %%%%%%%%%%%%%%	����ȫ�ֱ���	���뵼������	%%%%%%%%%%%%%%%%

    load_data;

%     load 'D:\Data\IMU1.mat';
% 	load 'D:\Data\GPS1.mat';
    
%     StaticAlignment_Data=IMU1(1:3000,2:7);%�ֶ�׼����
%     Navigation_Data=IMU1(3000:1200000,2:7);%��������
%     
%     wmsta = [StaticAlignment_Data(:,3) StaticAlignment_Data(:,1) StaticAlignment_Data(:,2)];% �������ݲ���ֵ��rad/s��
%     fmsta = [StaticAlignment_Data(:,6) StaticAlignment_Data(:,4) StaticAlignment_Data(:,5)];% ���ؼ��ٶȼƲ���ֵ��m/s/s)
%     
%     wm = [Navigation_Data(:,3) Navigation_Data(:,1) Navigation_Data(:,2)];% �������ݲ���ֵ��rad/s��
%     fm = [Navigation_Data(:,6) Navigation_Data(:,4) Navigation_Data(:,5)];% ���ؼ��ٶȼƲ���ֵ��m/s/s)
%% %%%%%%%%%%%%%%	���������ʼ��	%%%%%%%%%%%%%%%%

    pos = [GPS_Data(1,6)*glv.deg, GPS_Data(1,4)*glv.deg, GPS_Data(1,5)];	%������ʼλ�õľ�γ���Լ��߳���Ϣ��˳��Ϊ���ȣ�γ�ȣ��߶�
	glv.H_int = pos(3);											%��ȡ��ʼ�߳�
    Vn = zeros(1,3);

%% %%%%%%%%%%%%%%	����ʽ�ֶ�׼	%%%%%%%%%%%%%%%%

% 	disp('�ֶ�׼�����')
% 	[wnie, wnen, retp, gn] = earth(pos);		
% 	gb = sum(fmsta(1:alinum,:))/alinum;			%����ȡǰ5000������ƽ��ֵ�����Խ����ӱ���ƫ
% 	wbie = sum(wmsta(1:alinum,:))/alinum;				%���ٶ�ȡǰ5000������ƽ��ֵ�����Խ���������ƫ
% 	Cnb = dv2att(-gb, wbie, gn, wnie);		%����ʽ�ֶ�׼���������ϵ������ϵ��̬����
% 	att0 = M_M2A(Cnb)*glv.rad						%�����ʼ��̬��att0����λΪrad
% 	qnb = M_M2Q(Cnb);				%��ʼ��̬��ת��Ϊ��Ԫ��������֮��ľ���׼�Լ�֮�����̬�������
	qen = Q_E2G(pos);               %λ�ò������仯ʱ������Ԫ��Ӧ���������仯
    qie = [1 0 0 0];                      %��ʼ״̬�¹���ϵ���������ϵӦ��һ��,Ҳ��qie=1
    
    qnb = [0.7903 0.0022 0.0046 0.6127];
%     qnb = [0.742419491064446, -0.00253448908687747, -0.000320419088466838, 0.669930423987660]
    att0 = Q_Q2A(qnb);
	qib = Q_Mul(qie,Q_Mul(qen,qnb));
	Ra = Geo2Ear(pos);		%������ϵת��Ϊ����ϵ����
	g0 = Gravitation(pos, Ra);
	Vie = [0 cross([0 0 glv.wie],Ra)];

   	d_wie_g = glv.Tn*[0 0 0 glv.wie, g0];
 	d_rie = glv.Tn*[0 0 0 glv.wie, Vie];
 	
	bodystate.d_wie_g = [d_wie_g; d_wie_g];
	bodystate.pos = [pos; pos];
	bodystate.vel = [Vn; Vn];
	bodystate.att = [att0; att0];
	bodystate.d_rie = d_rie;
	bodystate.dq_wvib = DQ_Calcu(qib,Vn);
	bodystate.dq_wvie = DQ_Calcu(qie,Vie);		%������ʼ��ֹ������ϵ������ת�ٶ�
	bodystate.dq_wreb = DQ_Calcu(qie,Ra);
    	
%% %%%%%%%%%%%%%%	����׼	%%%%%%%%%%%%%%%%
% �������������������������趨KalmanFilter��ֵ������ϵͳ���˲������״̬�����������ٶ����2D�����򡢱��򣩣���׼�����3D����������������򣩣��ӱ�Ư��2D�����򡢱��򣩣�����Ʈ��3D������������������ܼ�10D��dX = AX+w(t);XΪ10D������

% 	dVn0 = [0.05;0.05];				%�趨��ʼ�ٶ������򡢱��򣬵�λΪm/s����
% 	datt0 = [0.1;0.1;0.1]*glv.deg;	%�趨��ʼ���ǣ�
% 	eb = [100;100]*glv.ug;			%�趨�ӱ�ֵƯ�ƣ�
% 	Wb = [0.01;0.01;0.01]*glv.dph;	%�趨����Ư�ƣ�
% 	Pk = diag([dVn0;datt0;eb;Wb])^2;%��ʼ������
% 
% 	Web = [50;50]*glv.ug;			%�ӱ�Э�����ʼֵ��
% 	Wdb = [0.01;0.01;0.01]*glv.dph;	%����Э�����ʼֵ��
% 	Qt = 1.0e-4*diag([Web;Wdb;zeros(5,1)])^2;%�趨״̬���Э�������Ĭ���ٶȡ���̬�������أ������������޹أ�
% 	Rk = diag([0.02;0.02])^2;		%�趨��������Э������ڹ۲��������������ٶ������RKΪ2D����������������Ư�����Բ�����Ϊ�۲�ô��
% 	Xk = zeros(10,1);				%�趨��ʼ״̬
% 	Kk = 1;							%�趨��ʼ���������棻
% 
% 	Hk = [eye(2) zeros(2,8)];
% 	F = [0, 2*wnie(3),	0, -retp.g, 0;
% 		-2*wnie(3),	0,	retp.g,	0,	0;
% 		0,	0,	0,	wnie(3), -wnie(2);
% 		0,	0,	-wnie(3),	0,	0;
% 		0,	0,	wnie(2),	0,	0];
% 
% 	C = [Cnb(1:2,1:2),	zeros(2,3);
% 		zeros(3,2),		-Cnb];
% 
% 	Ft = [	F,		C;
% 		  zeros(5,10)];
% 
% 	[Fkk_1,Qk] = KalDis(Ft, Qt, Tkf, 10);	

	kk = 1;

 	Sta_Result = zeros(stalen*0.25,9);
    testres = zeros(0.25*stalen,3);
    
%% %%%%%%%%%%%%%%	��������	%%%%%%%%%%%%%%%%

	% hwait = waitbar(0,'Please wait...'); %������
	% Total_Time = glv.n*length(1:glv.n:stalen);
		
	%% %%%%%%%%%%%%%%	��ż��Ԫ������	%%%%%%%%%%%%%%%%
		wie_update = glv.Tn*[0 0 glv.wie];
      qie_update = A_A2Q(wie_update);		
	% for k=1:4:stalen-glv.n

		% vmm = glv.Ts*(fmsta(k:k+glv.n-1,:));	%��������������
		% wmm = glv.Ts*(wmsta(k:k+glv.n-1,:));
		
		% bodystate = sins(bodystate, wmm, vmm);			%����״̬����
	
		% Vn = bodystate.vel(1,:);
		
		% Zk = Vn(1:2)';				%ȡ���򡢶����ٶ���Ϊ�۲���
		% testres(kk,:)=test;	
		% [Xk, Pk, Kk] = kalman(Fkk_1, Qk, Xk, Pk, Hk, Rk, Zk);%�������˲�
		
		% X(:,kk)=Xk;%�����˲������״̬
		% % ---------------�ջ�����-------------------
		% Cnb=(eye(3)+V_Askew(Xk(3:5)))*Q_Q2M(qnb); % ������̬����
		
		% bodystate.att(2,:) = bodystate.att(1,:);
		% bodystate.att(1,:) = M_M2A(Cnb);     % ��̬���
      % qie = Q_Mul(qie_update,qie);
		% qnb = M_M2Q(Cnb); % ������ϵͳ ���������� sinsQuat()
      % qib = Q_Mul(qie,Q_Mul(qen,qnb));
		
		% Vn = [Vn(1:2) - Xk(1:2)', 0]; %�ٶȷ���
		% Xk(1:5)=zeros(5,1);
		
		% bodystate.vel(2,:) = bodystate.vel(1,:);
		% bodystate.vel(1,:) = Vn;
		
 		% bodystate.dq_wvib = DQ_Calcu(qib,[0 0 0 0]);
 		% bodystate.dq_wvie = DQ_Calcu(qie,Vie);
      % bodystate.dq_wreb = DQ_Calcu(qie,Ra);
        
      % bodystate.d_wie_g = [d_wie_g; d_wie_g];
      % bodystate.d_rie = [d_rie; d_rie];
        
      % d_wie_g = glv.Tn*[0 0 0 glv.wie, g0];
      % d_rie = glv.Tn*[0 0 0 glv.wie, Vie];
        
		% %������ת��Ϊ�Ƕȣ�������̬���ٶ���Ϣ�����ں��ڻ�ͼ
		% Sta_ResTemp = [bodystate.att(1,:)*glv.rad, bodystate.vel(1,:), bodystate.pos(1,1)*glv.rad, bodystate.pos(1,2)*glv.rad, bodystate.pos(3)];
		% Sta_Result(kk,:) = [Sta_ResTemp];
		% kk=kk+1;

		% % waitbar(k/Total_Time);
		
	% end


	kk = 1; 	
	Nav_Result = zeros(0.25 * navlen,9);
	testres = zeros(0.25 * navlen,3);
	
% % 	hwait = waitbar(0,'Please wait...'); %������
% % 	Total_Time = glv.n*length(1:glv.n:navlen);

 %% %%%%%%%%%%%%%%	��ż��Ԫ������	%%%%%%%%%%%%%%%%  
	for k=1:glv.n:navlen-glv.n

		vmm = glv.Ts*(fmnav(k:k+glv.n-1,:));	%��������������
		wmm = glv.Ts*(wmnav(k:k+glv.n-1,:));

		bodystate= sins(bodystate, wmm, vmm);			%����״̬����

		Nav_ResTemp = [bodystate.att(1,:)*glv.rad, bodystate.vel(1,:), bodystate.pos(1,1:2)*glv.rad, bodystate.pos(1,3)];
		Nav_Result(kk,:) = Nav_ResTemp;
        kk=kk+1;
% 		waitbar(k/Total_Time);
	end
% 	close(hwait);

L = length(Navigation_Data);
Gdata=zeros(fix(L/40),6);
for i=1:1:fix(L/40)    
Gdata(i,:)=GPS1(i,5:10);
end
[a,b]=size(Gdata);
tg=1:1:a;
tg=tg/5;

print;
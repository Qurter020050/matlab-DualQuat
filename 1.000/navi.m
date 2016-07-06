%	本文件为捷联惯导系统解算主程序
%	计算过程中，全部使用弧度、小时作为基准单位
%	变量命名中，dq前缀表示对偶四元数类型，q表示四元数类型，m表示矩阵类型
%	v表示速度向量，p表示位置，a表示姿态角；其相对关系参考后附上下标

%% %%%%%%%%%%%%%%	构建全局变量	载入导航数据	%%%%%%%%%%%%%%%%

    load_data;

%     load 'D:\Data\IMU1.mat';
% 	load 'D:\Data\GPS1.mat';
    
%     StaticAlignment_Data=IMU1(1:3000,2:7);%粗对准数据
%     Navigation_Data=IMU1(3000:1200000,2:7);%导航数据
%     
%     wmsta = [StaticAlignment_Data(:,3) StaticAlignment_Data(:,1) StaticAlignment_Data(:,2)];% 加载陀螺测量值（rad/s）
%     fmsta = [StaticAlignment_Data(:,6) StaticAlignment_Data(:,4) StaticAlignment_Data(:,5)];% 加载加速度计测量值（m/s/s)
%     
%     wm = [Navigation_Data(:,3) Navigation_Data(:,1) Navigation_Data(:,2)];% 加载陀螺测量值（rad/s）
%     fm = [Navigation_Data(:,6) Navigation_Data(:,4) Navigation_Data(:,5)];% 加载加速度计测量值（m/s/s)
%% %%%%%%%%%%%%%%	导航程序初始化	%%%%%%%%%%%%%%%%

    pos = [GPS_Data(1,6)*glv.deg, GPS_Data(1,4)*glv.deg, GPS_Data(1,5)];	%输入起始位置的经纬度以及高程信息，顺序为经度，纬度，高度
	glv.H_int = pos(3);											%获取初始高程
    Vn = zeros(1,3);

%% %%%%%%%%%%%%%%	解析式粗对准	%%%%%%%%%%%%%%%%

% 	disp('粗对准结果：')
% 	[wnie, wnen, retp, gn] = earth(pos);		
% 	gb = sum(fmsta(1:alinum,:))/alinum;			%重力取前5000次数据平均值，用以矫正加表零偏
% 	wbie = sum(wmsta(1:alinum,:))/alinum;				%角速度取前5000次数据平均值，用以矫正陀螺零偏
% 	Cnb = dv2att(-gb, wbie, gn, wnie);		%解析式粗对准计算出导航系与载体系姿态矩阵
% 	att0 = M_M2A(Cnb)*glv.rad						%计算初始姿态角att0，单位为rad
% 	qnb = M_M2Q(Cnb);				%初始姿态角转化为四元数，用以之后的精对准以及之后的姿态矩阵更新
	qen = Q_E2G(pos);               %位置不发生变化时，该四元数应当不发生变化
    qie = [1 0 0 0];                      %初始状态下惯性系坐标与地球系应当一致,也即qie=1
    
    qnb = [0.7903 0.0022 0.0046 0.6127];
%     qnb = [0.742419491064446, -0.00253448908687747, -0.000320419088466838, 0.669930423987660]
    att0 = Q_Q2A(qnb);
	qib = Q_Mul(qie,Q_Mul(qen,qnb));
	Ra = Geo2Ear(pos);		%将地理系转换为地球系坐标
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
	bodystate.dq_wvie = DQ_Calcu(qie,Vie);		%假设起始静止，推力系仅有自转速度
	bodystate.dq_wreb = DQ_Calcu(qie,Ra);
    	
%% %%%%%%%%%%%%%%	精对准	%%%%%%%%%%%%%%%%
% ————————————设定KalmanFilter初值，采用系统级滤波，因此状态向量包含：速度误差2D（东向、北向），对准角误差3D（俯仰、横滚、航向），加表漂移2D（东向、北向），陀螺飘逸3D（俯仰、横滚、航向）总计10D，dX = AX+w(t);X为10D列向量

% 	dVn0 = [0.05;0.05];				%设定初始速度误差（东向、北向，单位为m/s）；
% 	datt0 = [0.1;0.1;0.1]*glv.deg;	%设定初始误差角；
% 	eb = [100;100]*glv.ug;			%设定加表常值漂移；
% 	Wb = [0.01;0.01;0.01]*glv.dph;	%设定陀螺漂移；
% 	Pk = diag([dVn0;datt0;eb;Wb])^2;%初始误差矩阵；
% 
% 	Web = [50;50]*glv.ug;			%加表协方差初始值；
% 	Wdb = [0.01;0.01;0.01]*glv.dph;	%陀螺协方差初始值；
% 	Qt = 1.0e-4*diag([Web;Wdb;zeros(5,1)])^2;%设定状态误差协方差矩阵；默认速度、姿态角误差相关，其他与噪声无关？
% 	Rk = diag([0.02;0.02])^2;		%设定量测噪声协方差，由于观测量仅含有两个速度误差，因此RK为2D；陀螺数据由于有漂移所以不能作为观测么？
% 	Xk = zeros(10,1);				%设定初始状态
% 	Kk = 1;							%设定初始卡尔曼增益；
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
    
%% %%%%%%%%%%%%%%	导航解算	%%%%%%%%%%%%%%%%

	% hwait = waitbar(0,'Please wait...'); %进度条
	% Total_Time = glv.n*length(1:glv.n:stalen);
		
	%% %%%%%%%%%%%%%%	对偶四元数计算	%%%%%%%%%%%%%%%%
		wie_update = glv.Tn*[0 0 glv.wie];
      qie_update = A_A2Q(wie_update);		
	% for k=1:4:stalen-glv.n

		% vmm = glv.Ts*(fmsta(k:k+glv.n-1,:));	%构建四子样数据
		% wmm = glv.Ts*(wmsta(k:k+glv.n-1,:));
		
		% bodystate = sins(bodystate, wmm, vmm);			%导航状态更新
	
		% Vn = bodystate.vel(1,:);
		
		% Zk = Vn(1:2)';				%取北向、东向速度作为观测量
		% testres(kk,:)=test;	
		% [Xk, Pk, Kk] = kalman(Fkk_1, Qk, Xk, Pk, Hk, Rk, Zk);%卡尔曼滤波
		
		% X(:,kk)=Xk;%保存滤波的误差状态
		% % ---------------闭环修正-------------------
		% Cnb=(eye(3)+V_Askew(Xk(3:5)))*Q_Q2M(qnb); % 修正姿态矩阵
		
		% bodystate.att(2,:) = bodystate.att(1,:);
		% bodystate.att(1,:) = M_M2A(Cnb);     % 姿态输出
      % qie = Q_Mul(qie_update,qie);
		% qnb = M_M2Q(Cnb); % 反馈回系统 见导航方程 sinsQuat()
      % qib = Q_Mul(qie,Q_Mul(qen,qnb));
		
		% Vn = [Vn(1:2) - Xk(1:2)', 0]; %速度反馈
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
        
		% %将弧度转化为角度，更新姿态、速度信息，用于后期画图
		% Sta_ResTemp = [bodystate.att(1,:)*glv.rad, bodystate.vel(1,:), bodystate.pos(1,1)*glv.rad, bodystate.pos(1,2)*glv.rad, bodystate.pos(3)];
		% Sta_Result(kk,:) = [Sta_ResTemp];
		% kk=kk+1;

		% % waitbar(k/Total_Time);
		
	% end


	kk = 1; 	
	Nav_Result = zeros(0.25 * navlen,9);
	testres = zeros(0.25 * navlen,3);
	
% % 	hwait = waitbar(0,'Please wait...'); %进度条
% % 	Total_Time = glv.n*length(1:glv.n:navlen);

 %% %%%%%%%%%%%%%%	对偶四元数计算	%%%%%%%%%%%%%%%%  
	for k=1:glv.n:navlen-glv.n

		vmm = glv.Ts*(fmnav(k:k+glv.n-1,:));	%构建四子样数据
		wmm = glv.Ts*(wmnav(k:k+glv.n-1,:));

		bodystate= sins(bodystate, wmm, vmm);			%导航状态更新

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
% 本函数用于卡尔曼滤波前的连续系统离散化
% 输入为连续系统的状态转移矩阵Fai(t)，以及连续噪声协方差矩阵Qt,卡尔曼滤波周期Tkf，
% 输出为离散化后的一步状态转移矩阵Fai(k,k-1)以及离散噪声协方差矩阵Qk

function [Faikk_1, Qk] = KalDis(Ft, Qt, Tkf, n)

	
	Tkfi = Tkf;
	Fti = Ft;
	facti = 1.0;
	Mi = Qt;
	In = eye(size(Ft,1));
	Faikk_1 = In + Tkf*Ft;
	Qk = Qt*Tkf;
	
	for i=2:1:n
		Tkfi = Tkfi*Tkf;
		facti = facti*i;
		Fti = Fti*Ft;
		Faikk_1 = Faikk_1 +Tkfi/facti*Fti;
		
		FtMi = Ft*Mi;
		Mi = FtMi + FtMi';
		Qk = Qk + Tkfi/facti*Mi;
	end
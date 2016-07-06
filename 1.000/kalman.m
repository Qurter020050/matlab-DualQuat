function [Xk, Pk, Kk] = kalman(Phikk_1, Qk, Xk_1, Pk_1, Hk, Rk, Zk)
    if nargin<7     % ������״̬����
        Xk = Phikk_1*Xk_1;
        Pk = Phikk_1*Pk_1*Phikk_1'+Qk;
    else            % �в���ʱ�˲�
        Xkk_1=Phikk_1*Xk_1;    
        Pkk_1 = Phikk_1*Pk_1*Phikk_1' + Qk; 
        Pxz = Pkk_1*Hk';
        Pzz = Hk*Pxz + Rk;
        Kk = Pxz*Pzz^(-1);
        Xk = Xkk_1 + Kk*(Zk-Hk*Xkk_1);
		Pk = Pkk_1-Kk*Hk*Pkk_1;
    end

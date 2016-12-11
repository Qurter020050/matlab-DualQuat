%	���ļ�Ϊ�����ߵ�ϵͳ����������
%	��������У�ȫ��ʹ�û��ȡ�Сʱ��Ϊ��׼��λ
%	���������У�dqǰ׺��ʾ��ż��Ԫ�����ͣ�q��ʾ��Ԫ�����ͣ�m��ʾ��������
%	v��ʾ�ٶ�������p��ʾλ�ã�a��ʾ��̬�ǣ�����Թ�ϵ�ο������±�

%% %%%%%%%%%%%%%%	����ȫ�ֱ���	���뵼������	%%%%%%%%%%%%%%%%

load_data;

%% %%%%%%%%%%%%%%	���������ʼ��	%%%%%%%%%%%%%%%%

pos = [gps(1,1:2)*glv.deg, gps(1,3)];	%������ʼλ�õľ�γ���Լ��߳���Ϣ��˳��Ϊ���ȣ�γ�ȣ��߶�
glv.Hint = pos(3);											%��ȡ��ʼ�߳�
Vn = zeros(1,3);

%% %%%%%%%%%%%%%%	����ʽ�ֶ�׼	%%%%%%%%%%%%%%%%

att = [-0.0067    0.0104    1.4041];
glv.qen = Q_E2G(pos);               %λ�ò������仯ʱ������Ԫ��Ӧ���������仯
qie = [1 0 0 0];                      %��ʼ״̬�¹���ϵ���������ϵӦ��һ��,Ҳ��qie=1

qnb = [0.7635, -0.0059, 0.0018, 0.6457];

%     Cnb =  [0.1660   -0.9861   -0.0048
%             0.9861    0.1660    0.0114
%             -0.0104   -0.0067    0.9999];
%   qnb = [0.742419491064446, -0.00253448908687747, -0.000320419088466838, 0.669930423987660]
att0 = Q_Q2A(qnb);
qib = Q_Mul(qie,Q_Mul(glv.qen,qnb));
Ra = Geo2Ear(pos);		%������ϵת��Ϊ����ϵ����
g0 = Gravitation(pos, Ra, glv.qen);
v_ie = [0 cross([0 0 glv.wie],Ra)];

d_wie_g = glv.Tn*[0 0 0 glv.wie, g0];
d_rie = glv.Tn*[0 0 0 glv.wie, v_ie];

bodystate.d_wie_g = [d_wie_g; d_wie_g];
bodystate.pos = [pos; pos];
bodystate.vel = [Vn; Vn];
bodystate.att = [att0; att0];
bodystate.d_rie = d_rie;
bodystate.dq_wvib = DQ_Calcu(qib,Vn);
bodystate.dq_wvie = DQ_Calcu(qie,v_ie);		%������ʼ��ֹ������ϵ������ת�ٶ�
bodystate.dq_wreb = DQ_Calcu(qie,Ra);

kk = 1;

Sta_Result = zeros(stalen*0.25,9);
testres = zeros(0.25*stalen,3);

%% %%%%%%%%%%%%%%	��������	%%%%%%%%%%%%%%%%
wie_update = glv.Tn*[0 0 glv.wie];
qie_update = A_A2Q(wie_update);

navres=zeros(navlen/4,10);
testres = [];

%% %%%%%%%%%%%%%%	��ż��Ԫ������	%%%%%%%%%%%%%%%%
for k=glv.n:glv.n:navlen
    
    vmm = glv.Ts*(fmnav(k-glv.n+1:k,:));	%��������������
    wmm = glv.Ts*(wmnav(k-glv.n+1:k,:));
    
    bodystate = sins(bodystate, wmm, vmm);			%����״̬����
    
    Nav_ResTemp = [bodystate.att(1,:)*glv.rad, bodystate.vel(1,:), bodystate.pos(1,1:2)*glv.rad, bodystate.pos(1,3),timestap(k)];
    testtemp = [bodystate.test1,bodystate.test2,bodystate.test3,bodystate.test4,bodystate.test5];
    navres(k/4,:) = Nav_ResTemp;
    testres(k/4,:) = testtemp;
    kk=kk+1;
end
%%

print;


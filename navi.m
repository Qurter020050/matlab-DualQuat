%	本文件为捷联惯导系统解算主程序
%	计算过程中，全部使用弧度、小时作为基准单位
%	变量命名中，dq前缀表示对偶四元数类型，q表示四元数类型，m表示矩阵类型
%	v表示速度向量，p表示位置，a表示姿态角；其相对关系参考后附上下标

%% %%%%%%%%%%%%%%	构建全局变量	载入导航数据	%%%%%%%%%%%%%%%%

load_data;

%% %%%%%%%%%%%%%%	导航程序初始化	%%%%%%%%%%%%%%%%

pos = [gps(1,1:2)*glv.deg, gps(1,3)];	%输入起始位置的经纬度以及高程信息，顺序为经度，纬度，高度
glv.Hint = pos(3);											%获取初始高程
Vn = zeros(1,3);

%% %%%%%%%%%%%%%%	解析式粗对准	%%%%%%%%%%%%%%%%

att = [-0.0067    0.0104    1.4041];
glv.qen = Q_E2G(pos);               %位置不发生变化时，该四元数应当不发生变化
qie = [1 0 0 0];                      %初始状态下惯性系坐标与地球系应当一致,也即qie=1

qnb = [0.7635, -0.0059, 0.0018, 0.6457];

%     Cnb =  [0.1660   -0.9861   -0.0048
%             0.9861    0.1660    0.0114
%             -0.0104   -0.0067    0.9999];
%   qnb = [0.742419491064446, -0.00253448908687747, -0.000320419088466838, 0.669930423987660]
att0 = Q_Q2A(qnb);
qib = Q_Mul(qie,Q_Mul(glv.qen,qnb));
Ra = Geo2Ear(pos);		%将地理系转换为地球系坐标
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
bodystate.dq_wvie = DQ_Calcu(qie,v_ie);		%假设起始静止，推力系仅有自转速度
bodystate.dq_wreb = DQ_Calcu(qie,Ra);

kk = 1;

Sta_Result = zeros(stalen*0.25,9);
testres = zeros(0.25*stalen,3);

%% %%%%%%%%%%%%%%	导航解算	%%%%%%%%%%%%%%%%
wie_update = glv.Tn*[0 0 glv.wie];
qie_update = A_A2Q(wie_update);

navres=zeros(navlen/4,10);
testres = [];

%% %%%%%%%%%%%%%%	对偶四元数计算	%%%%%%%%%%%%%%%%
for k=glv.n:glv.n:navlen
    
    vmm = glv.Ts*(fmnav(k-glv.n+1:k,:));	%构建四子样数据
    wmm = glv.Ts*(wmnav(k-glv.n+1:k,:));
    
    bodystate = sins(bodystate, wmm, vmm);			%导航状态更新
    
    Nav_ResTemp = [bodystate.att(1,:)*glv.rad, bodystate.vel(1,:), bodystate.pos(1,1:2)*glv.rad, bodystate.pos(1,3),timestap(k)];
    testtemp = [bodystate.test1,bodystate.test2,bodystate.test3,bodystate.test4,bodystate.test5];
    navres(k/4,:) = Nav_ResTemp;
    testres(k/4,:) = testtemp;
    kk=kk+1;
end
%%

print;


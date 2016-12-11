%% 本函数为捷联惯导主程序
% 推力系T与载体系B平行，ip = ib
% 引力系F与地球系E平行，it = ie
% 位置系为地球系，速度、姿态系参考为导航系

function bodystate_out = sins(bodystate_in, wmm, vmm)

global glv;									%载入全局变量
q_en = glv.qen;
%%  -----------------初始参数载入----------------
w_bb(1,:) = wmm(1,:)+wmm(2,:);
w_bb(2,:) = wmm(3,:)+wmm(4,:);

v_bb(1,:) = vmm(1,:)+vmm(2,:);
v_bb(2,:) = vmm(3,:)+vmm(4,:);

dbb1 = [0, wmm(1,:), 0, vmm(1,:)];
dbb2 = [0, wmm(2,:), 0, vmm(2,:)];
dbb3 = [0, wmm(3,:), 0, vmm(3,:)];
dbb4 = [0, wmm(4,:), 0, vmm(4,:)];

dq_wvib_pre = bodystate_in.dq_wvib;
dq_wvie_pre = bodystate_in.dq_wvie;
dq_wreb_pre = bodystate_in.dq_wreb;

%%  ----------------推力系姿态更新----------------
dq_wvib_update_spiral = zeros(1,8);

dq_wvib_update_spiral(2:4) = cross(w_bb(1,:), w_bb(2,:));
dq_wvib_update_spiral(6:8) = cross(w_bb(1,:), v_bb(2,:)) + cross(w_bb(2,:), v_bb(1,:));

d_wib_vib = [0, sum(w_bb,1), 0, sum(v_bb,1)] + 2*dq_wvib_update_spiral/3;	% 二子样螺旋矢量
% d_wib_vib = [0, sum(wmm,1), 0, sum(vmm,1)] + (736*(DQ_Mul(dbb1,dbb2)+DQ_Mul(dbb3,dbb4))+334*(DQ_Mul(dbb1,dbb3)+DQ_Mul(dbb2,dbb4))+526*DQ_Mul(dbb1,dbb4)+654*DQ_Mul(dbb2,dbb3))/945; %张论文参数
% d_wib_vib = [0, sum(wmm,1), 0, sum(vmm,1)] +214*(DQ_Mul(dbb1,dbb2)+DQ_Mul(dbb3,dbb4)+DQ_Mul(dbb2,dbb3))/315+(46*(DQ_Mul(dbb1,dbb3)+DQ_Mul(dbb2,dbb4))+54*DQ_Mul(dbb1,dbb4))/105;%秦老师论文参数

dq_wvib = DQ_Update(dq_wvib_pre,d_wib_vib);				%

[q_ib, v_ip] = DQ_Separate(dq_wvib,1);

%%  ---------------引力系姿态更新----------------
d_wie_g2 = bodystate_in.d_wie_g(2,:);
d_wie_g1 = bodystate_in.d_wie_g(1,:);

dq_wie_update_spiral = DQ_Mul(d_wie_g1, d_wie_g2);
dq_wie_update_spiral(1) = 0;
dq_wie_update_spiral(5) = 0;

d_wvie = d_wie_g1 + dq_wie_update_spiral/12;

dq_wvie = DQ_Update(dq_wvie_pre, d_wvie);

[q_ie, v_ig] = DQ_Separate(dq_wvie,1);

v_i_ib = v_ip + v_ig;

q_eb = Q_Mul(Q_Conj(q_ie),q_ib);
q_nb = Q_Mul(Q_Conj(q_en),q_eb);

v_e_ib = FrameTrans(q_ie, v_i_ib, 1);
% v_nb = FrameTrans(q_en, v_eb, 1);
% v_nb(3) = 0;
% v_eb = FrameTrans(q_en, v_nb, -1);

%% ----------------位置更新---------------------
d_reb2 = bodystate_in.d_rie(1,:);
d_reb1 = glv.Tn*[0 0 0 glv.wie, 0 v_e_ib];

d_wreb_update = DQ_Mul(d_reb2, d_reb1);
d_wreb_update(1) = 0;
d_wreb_update(5) = 0;

d_wreb = d_reb1 + d_wreb_update/12;
d_wreb(1) = 0;
d_wreb(5) = 0;

dq_wreb = DQ_Update(dq_wreb_pre, d_wreb);

[~,r_eb] = DQ_Separate(dq_wreb,-1);

r_nb = Ear2Geo(r_eb);
r_nb(3) = glv.Hint;

v_e_ie = cross([0,0, glv.wie],r_eb);
v_eb = v_e_ib - v_e_ie;

g_eb = Gravitation(r_nb, r_eb);

d_wie_vie = glv.Tn*[0 0 0 glv.wie, g_eb];

v_n_ib = FrameTrans(q_en,v_eb, 1);
v_nb2 = FrameTrans(q_en,v_eb, -1);

% v_nb(3) = 0;
% v_ebfix = FrameTrans(q_en, v_nb, -1);
% v_e_ibfix = v_ebfix+v_ie;
% dq_wreb = DQ_Calcu(q_ie,v_e_ibfix);




%%	----------------对偶四元数输出---------------
bodystate_out.att(2,:) = bodystate_in.att(1,:);
bodystate_out.vel(2,:) = bodystate_in.vel(1,:);
bodystate_out.pos(2,:) = bodystate_in.pos(1,:);

bodystate_out.att(1,:) = Q_Q2A(q_nb);
bodystate_out.vel(1,:) = v_n_ib;
bodystate_out.pos(1,:) = r_nb;
bodystate_out.d_wie_g = [d_wie_vie; d_wie_g1];
bodystate_out.d_rie = [d_reb1; d_reb1];

bodystate_out.dq_wvib = dq_wvib;
bodystate_out.dq_wvie = dq_wvie;
bodystate_out.dq_wreb = dq_wreb;
bodystate_out.test1 = v_i_ib;
bodystate_out.test2 = v_eb;
bodystate_out.test3 = v_nb2;
bodystate_out.test4 = v_e_ib;
bodystate_out.test5 = Q_Q2A(q_ie);


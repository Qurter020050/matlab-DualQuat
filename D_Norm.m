% ������������ȡ��ż��Ԫ���ķ���
% ����Ϊ��ż��Ԫ��dq_in������s_in
% ���Ϊ��Ӧ�ķ���dq_norm_out�������ʽҲΪdq����ʽ�����ڶ�ż��Ԫ��������ʸ���ı�ʾ��ʽΪ��p+Ep��������ʽ�����䷶��ҲΪ��p_n+Ep_n��������ʽ������ʵ���Ͷ�ż����Ϊ��Ӧ��ģ��

function dq_norm_out = D_Norm(dq_in)


dq_norm_out = zeros(1,8);

dq_conj = DQ_Conj(dq_in);
dq_norm_out = DQ_Mul(dq_in,dq_conj);
dq_norm_out(1) = sqrt(dq_norm_out(1));

dq_norm_out(5) = 0.5*dq_norm_out(5)/dq_norm_out(1);
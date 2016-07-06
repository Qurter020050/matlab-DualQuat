% 本程序为四元数共轭程序
% 输入为四元数列向量qin；
% 输出为共轭四元数列向量qConjout；

function q_Conj_out = Q_Conj(q_in)

q_Conj_out = [q_in(1), -q_in(2:4)];

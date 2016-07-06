% 本函数用于将姿态角转换为姿态四元数
% 输入为姿态角向量a_In；
% 输出为姿态四元数q_Out；


function q_out = A_A2Q(a_in)

q_out = M_M2Q(A_A2M(a_in));

q_out = q_out/norm(q_out);				%标准化
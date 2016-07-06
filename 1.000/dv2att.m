% 本函数用于根据计算姿态变换矩阵Cnb，用于系统的解析式粗对准
% 输入为相对于载体系和相对于导航系的基准角速度（自转角速度）以及比力（wbie，gb；wnie，gn），由于叉乘涉及到方向，因此两组变量的输入顺序应当一致
% 输出为姿态变换矩阵Cnb
% 基本原理参考严恭敏3.5.1粗对准

function Cnb = dv2att(varb1, varb2, varn1, varn2)

	varb1 = norm(varn1)/norm(varb1)*varb1;						%在vb1方向上取长度为vn1的向量
	varb2 = norm(varn2)/norm(varb2)*varb2;						%在vb2方向上取长度为vn2的向量（方便后期标准化）
	varbtmp = cross(varb1,varb2);
	varntmp = cross(varn1,varn2);

	%Cnb = [ varn1'; varn2'; varntmp';] \ [ varb1'; varb2'; varbtmp';];		%严恭敏3.5.1 粗对准中式3.5-4
	Cnb = [ varn1; varntmp; cross(varn1,varntmp)] \ [varb1; varbtmp; cross(varb1,varbtmp)];					%学长的坐标系选取，采用完全正交的坐标系，个人觉得少了g和w之间的角度关系，粗对准精度应该高于严恭敏的式子，不过精对准后应该无差别

	Cnb = Cnb*(Cnb'*Cnb)^(-1/2);		%Cnb标准化


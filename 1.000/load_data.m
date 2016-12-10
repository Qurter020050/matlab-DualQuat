%% 数据读取程序
% .imu数据文件格式：
%  iPhasePC  X角速率 Y角速率 Z角速率 X加速度 Y加速度 Z加速度 里程计脉冲
%  
%  iPhasePC 为惯导5ms计数值；
%  XYZ角速度为前上右坐标，单位 rad/s；
%  XYZ加速度              单位 m/s/s;
%  里程计脉冲为5ms实时采集值（每个脉冲当量大约为0.012）
%  
%  .GPS数据文件格式：
%  iPhasePC GPS状态 GPS模式 GPS有效性 GPS经度 GPS纬度 GPS高度 GPS东速 GPS北速 GPS天速
%  iPhasePC 与imu文件的PC值对应，周期200ms；
%  GPS有效性标志，5a表示有效， a5为无效；


% close all;
clear all;
% clearvars -except imu gps ;
glv;							%读取全局变量
clc;
navlen = 400000;
stalen = 5000;
gpslen = navlen/40;
% %% %%%%%%%%%%%%%%	杞藉叆瀵艰埅鏁版嵁	%%%%%%%%%%%%%%%%
% 
% if exist('imu','var')==0
% data=load('D:\Project\Matlab\Data\IMU2.mat');
% imu = data.IMU2;
% data=load('D:\Project\Matlab\Data\GPS2.mat');
% gps = data.GPS2;
% clear IMU1 GPS1;
% end

load('D:\Project\Matlab\Data\test\Data2.mat');
load('D:\Project\Matlab\Data\test\trares.mat');

stadata = imu(1:stalen,1:7);
navdata = imu(1:navlen,1:7);
timestap = imu(1:navlen,1);

wmsta = [ stadata(:,4), stadata(:,2), stadata(:,3)];
fmsta = [ stadata(:,7), stadata(:,5), stadata(:,6)];

wmnav = [ navdata(:,4), navdata(:,2), navdata(:,3)];
fmnav = [ navdata(:,7), navdata(:,5), navdata(:,6)];


clear stadata navdata;

Tkf = 100 * glv.Ts;
%% ���ݶ�ȡ����
% .imu�����ļ���ʽ��
%  iPhasePC  X������ Y������ Z������ X���ٶ� Y���ٶ� Z���ٶ� ��̼�����
%  
%  iPhasePC Ϊ�ߵ�5ms����ֵ��
%  XYZ���ٶ�Ϊǰ�������꣬��λ rad/s��
%  XYZ���ٶ�              ��λ m/s/s;
%  ��̼�����Ϊ5msʵʱ�ɼ�ֵ��ÿ�����嵱����ԼΪ0.012��
%  
%  .GPS�����ļ���ʽ��
%  iPhasePC GPS״̬ GPSģʽ GPS��Ч�� GPS���� GPSγ�� GPS�߶� GPS���� GPS���� GPS����
%  iPhasePC ��imu�ļ���PCֵ��Ӧ������200ms��
%  GPS��Ч�Ա�־��5a��ʾ��Ч�� a5Ϊ��Ч��


close all;
clearvars -except imu gps ;
glv;							%��ȡȫ�ֱ���
clc;
navlen = 600000;
stalen = 5000;
gpslen = navlen/40;
%% %%%%%%%%%%%%%%	载入导航数据	%%%%%%%%%%%%%%%%

if exist('imu','var')==0
data=load('D:\Project\Matlab\Data\IMU1.mat');
imu = data.IMU1;
data=load('D:\Project\Matlab\Data\GPS1.mat');
gps = data.GPS1;
clear IMU1 GPS1;
end

stadata = imu(1:stalen,:);
navdata = imu(stalen+1:navlen+stalen,:);
timestap = imu(stalen+1:navlen+stalen,1);

gpsdata = zeros(gpslen,8);
for i=1:1:gpslen
    gpsdata(i,:)=[gps(i,8:10),gps(i,5:7),gps(i,1),gps(i,4)];
end

wmsta = [ stadata(:,4), stadata(:,2), stadata(:,3)];
fmsta = [ stadata(:,7), stadata(:,5), stadata(:,6)];

wmnav = [ navdata(:,4), navdata(:,2), navdata(:,3)];
fmnav = [ navdata(:,7), navdata(:,5), navdata(:,6)];


clear stadata navdata;

Tkf = 100 * glv.Ts;
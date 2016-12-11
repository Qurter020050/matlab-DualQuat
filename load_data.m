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


% close all;
clear all;
% clearvars -except imu gps ;
glv;							%��ȡȫ�ֱ���
clc;

% load('D:\Project\Matlab\Data\test\trasimu.mat');
% Data = load('D:\Project\Matlab\Copy_of_�������-���\0715.txt');                                                 % ���ļ��ж�ȡIMU����
% Gdata = Data(:,1:9)';
% Gdata = Gdata(:,3000:73000);
% Navigation_Data = Data(3000:73000,10:15);                                     % ��ȡ������������
% wmnav = [Navigation_Data(:,1) Navigation_Data(:,2) Navigation_Data(:,3)];   % �������ݲ���ֵ��rad/s��
% fmnav = [Navigation_Data(:,4) Navigation_Data(:,5) Navigation_Data(:,6)];   % ���ؼ��ٶȼƲ���ֵ��m/s/s)
% 
% 
% timestap = 1:navlen;
% for i=1:navlen/4
%   gps(:,i) = Gdata(:,4*i-2);
%   tg(i)=i/50;
% end

navlen = 400000;
stalen = 5000;
gpslen = navlen/40;

% %% %%%%%%%%%%%%%%	载入导航数据	%%%%%%%%%%%%%%%%
% 
if exist('imu','var')==0
data=load('D:\Project\Matlab\Data\IMU2.mat');
imu = data.IMU2;
data=load('D:\Project\Matlab\Data\GPS2.mat');
gps = data.GPS2;
clear IMU2 GPS2;
end

load('D:\Project\Matlab\Data\test\Data2.mat');
load('D:\Project\Matlab\Data\test\trares.mat');

stadata = imu_load(1:stalen,1:7);
navdata = imu_load(1:navlen,1:7);
timestap = imu_load(1:navlen,1);

wmsta = [ stadata(:,4), stadata(:,2), stadata(:,3)];
fmsta = [ stadata(:,7), stadata(:,5), stadata(:,6)];

wmnav = [ navdata(:,4), navdata(:,2), navdata(:,3)];
fmnav = [ navdata(:,7), navdata(:,5), navdata(:,6)];


clear stadata navdata;

Tkf = 100 * glv.Ts;
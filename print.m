tn=1:length(navres);
tt = 1:length(trares);
[a,b]=size(gps);
tg=1:1:a;
tg=tg*10;
time = datestr(now);

% 	load 'D:\tra_res\trad_nav_result.mat';
% 	
%     figure name 'time'
% 	subplot(311);plot(tn,navres(:,1),'--b',tt,trares(1,:),'r');xlabel('t/s');ylabel('\it\phix\rm/rad');title('俯仰角');grid on;
% 	subplot(312);plot(tn,navres(:,2),'--b',tt,trares(2,:),'r');xlabel('t/s');ylabel('\it\phiy\rm/rad');title('横滚角');grid on;
%  	subplot(313);plot(tn,navres(:,3),'--b',tt,trares(3,:),'r');xlabel('t/s');ylabel('\it\phiz\rm/rad');title('航向角');grid on;
% 	figure name '速度解算'
% 	subplot(311);plot(tn,navres(:,4),'b',tg,gps(:,4),'--r');xlabel('t/s');ylabel('');title('东向速度');grid on;
% 	subplot(312);plot(tn,navres(:,5),'b',tg,gps(:,5),'--r');xlabel('t/s');ylabel('');title('北向速度');grid on;
%  	subplot(313);plot(tn,navres(:,6),'b',tg,gps(:,6),'--r');xlabel('t/s');ylabel('');title('天向速度');grid on;
% 	
    figure name '位置解算'
	subplot(311);plot(tn,navres(:,7),'b',tn,trares(8,:),'y',tg,gps(:,1),'--r');xlabel('t/s');ylabel('');title('经度');grid on;
	subplot(312);plot(tn,navres(:,8),'b',tn,trares(7,:),'y',tg,gps(:,2),'--r');xlabel('t/s');ylabel('');title('纬度');grid on;
 	subplot(313);plot(tn,navres(:,9),'b',tn,trares(9,:),'y',tg,gps(:,3),'--r');xlabel('t/s');ylabel('');title('高度');grid on;
% error = zeros(gpslen,6);
% for i=1:gpslen
%     error(i,:)=gps(i,1:6)-navres(i*10,4:9);
% end


	
% 		for i = 1:1:length(tg)
% 	error_vel(i,:) = [GPS_Data(i,3)-Nav_Result(i*10,4), GPS_Data(i,1)-Nav_Result(i*10,5), GPS_Data(i,2)-Nav_Result(i*10,6), GPS_Data(i,3)-tra_nav_res(4,i*10)', GPS_Data(i,1)-tra_nav_res(5,i*10)', GPS_Data(i,2)-tra_nav_res(6,i*10)'];
% 	error_pos(i,:) = [GPS_Data(i,6)-Nav_Result(i*10,7), GPS_Data(i,4)-Nav_Result(i*10,8), GPS_Data(i,5)-Nav_Result(i*10,9), GPS_Data(i,6)-tra_nav_res(8,i*10)*glv.rad', GPS_Data(i,4)-tra_nav_res(7,i*10)*glv.rad', GPS_Data(i,5)-tra_nav_res(9,i*10)'];
% 	end
% 	
% 	figure name '速度误差'
% 	subplot(311);plot(tg,abs(error_vel(:,1)),tg,abs(error_vel(:,4)));xlabel('t/s');ylabel('');title('东向速度');grid on;
% 	subplot(312);plot(tg,abs(error_vel(:,2)),tg,abs(error_vel(:,5)));xlabel('t/s');ylabel('');title('北向速度');grid on;
%  	subplot(313);plot(tg,abs(error_vel(:,3)),tg,abs(error_vel(:,6)));xlabel('t/s');ylabel('');title('天向速度');grid on;
% % 	
% 	figure name '位置误差'
% 	subplot(311);plot(tg,abs(error_pos(:,1)),tg,abs(error_pos(:,4)));xlabel('t/s');ylabel('');title('经度');grid on;
% 	subplot(312);plot(tg,abs(error_pos(:,2)),tg,abs(error_pos(:,5)));xlabel('t/s');ylabel('');title('纬度');grid on;
%  	subplot(313);plot(tg,abs(error_pos(:,3)),tg,abs(error_pos(:,6)));xlabel('t/s');ylabel('');title('高度');grid on;
% 	
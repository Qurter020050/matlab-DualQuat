% 	load 'D:\tra_res\trad_nav_result.mat';
% 	
%     figure name '角度解算'
% 	subplot(311);plot(tn,Nav_Result(:,1),tn,tra_nav_res(1,:));xlabel('t/s');ylabel('\it\phix\rm/rad');title('俯仰角');grid on;
% 	subplot(312);plot(tn,Nav_Result(:,2),tn,tra_nav_res(2,:));xlabel('t/s');ylabel('\it\phiy\rm/rad');title('横滚角');grid on;
%  	subplot(313);plot(tn,Nav_Result(:,3),tn,tra_nav_res(3,:));xlabel('t/s');ylabel('\it\phiz\rm/rad');title('航向角');grid on;
	figure name '速度解算'
	subplot(311);plot(tn,navres(:,4),'b',tg,gpsdata(:,1),'--r');xlabel('t/s');ylabel('');title('东向速度');grid on;
	subplot(312);plot(tn,navres(:,5),'b',tg,gpsdata(:,2),'--r');xlabel('t/s');ylabel('');title('北向速度');grid on;
 	subplot(313);plot(tn,navres(:,6),'b',tg,gpsdata(:,3),'--r');xlabel('t/s');ylabel('');title('天向速度');grid on;
	
    figure name '位置解算'
	subplot(311);plot(tn,navres(:,7),'b',tg,gpsdata(:,4),'--r');xlabel('t/s');ylabel('');title('经度');grid on;
	subplot(312);plot(tn,navres(:,8),'b',tg,gpsdata(:,5),'--r');xlabel('t/s');ylabel('');title('纬度');grid on;
 	subplot(313);plot(tn,navres(:,9),'b',tg,gpsdata(:,6),'--r');xlabel('t/s');ylabel('');title('高度');grid on;
	
% 		for i = 1:1:length(tg)
% 	error_vel(i,:) = [GPS_Data(i,3)-Nav_Result(i*10,4), GPS_Data(i,1)-Nav_Result(i*10,5), GPS_Data(i,2)-Nav_Result(i*10,6), GPS_Data(i,3)-tra_nav_res(4,i*10)', GPS_Data(i,1)-tra_nav_res(5,i*10)', GPS_Data(i,2)-tra_nav_res(6,i*10)'];
% 	error_pos(i,:) = [GPS_Data(i,6)-Nav_Result(i*10,7), GPS_Data(i,4)-Nav_Result(i*10,8), GPS_Data(i,5)-Nav_Result(i*10,9), GPS_Data(i,6)-tra_nav_res(8,i*10)*glv.rad', GPS_Data(i,4)-tra_nav_res(7,i*10)*glv.rad', GPS_Data(i,5)-tra_nav_res(9,i*10)'];
% 	end
% 	
% 	figure name '速度误差'
% 	subplot(311);plot(tg,abs(error_vel(:,1)),tg,abs(error_vel(:,4)));xlabel('t/s');ylabel('');title('东向速度');grid on;
% 	subplot(312);plot(tg,abs(error_vel(:,2)),tg,abs(error_vel(:,5)));xlabel('t/s');ylabel('');title('北向速度');grid on;
%  	subplot(313);plot(tg,abs(error_vel(:,3)),tg,abs(error_vel(:,6)));xlabel('t/s');ylabel('');title('天向速度');grid on;
% 	
% 	figure name '位置误差'
% 	subplot(311);plot(tg,abs(error_pos(:,1)),tg,abs(error_pos(:,4)));xlabel('t/s');ylabel('');title('经度');grid on;
% 	subplot(312);plot(tg,abs(error_pos(:,2)),tg,abs(error_pos(:,5)));xlabel('t/s');ylabel('');title('纬度');grid on;
%  	subplot(313);plot(tg,abs(error_pos(:,3)),tg,abs(error_pos(:,6)));xlabel('t/s');ylabel('');title('高度');grid on;
	
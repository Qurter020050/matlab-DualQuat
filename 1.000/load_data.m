    close all;
	clearvars -except Data StaticAlignment_Data Navigation_Data GPS_Data;
    glv;							%��ȡȫ�ֱ���	
    clc;
    datalen = 4200000;
%% %%%%%%%%%%%%%%	载入导航数据	%%%%%%%%%%%%%%%%

	if (exist('Data','var') ~= 1)
		load 'E:\Project\Matlab\Data\data2.mat';%可以放入你的出错处理，为空则相当于返�?
    end
	if (exist('StaticAlignment_Data','var') ~= 1)
        StaticAlignment_Data = Data(1:200000,:);		%提取对准数据，采样频�?00HZ，顺序为IPHASEPC、陀螺数据（北天东）、加表数据（前上右）、GPS数据（�?度（前上右）、位置（纬高经）�?

    end
    
    if (exist('Navigation_Data','var') ~= 1)
        Navigation_Data = Data(200001:datalen,:);		%数据少了看不出趋�?
     end
    if (exist('GPS_Data','var') ~= 1)   
    GPS_Data = zeros(datalen-200001,6);
    L = length(Navigation_Data);
    for i=1:1:fix(L/40)    
    GPS_Data(i,:)=Data(200000+(i-1)*40,8:13);
    end
    GPS_Data(i:datalen-200001,:)=[];
    end
    
    wmsta = [ StaticAlignment_Data(:,4), StaticAlignment_Data(:,2), StaticAlignment_Data(:,3)];	%提取�?��数据，顺序为东向、北向�?天向
    fmsta = [ StaticAlignment_Data(:,7), StaticAlignment_Data(:,5), StaticAlignment_Data(:,6)];	%提取加表数据，顺序为右�?前�?�?

    wmnav = [ Navigation_Data(:,4), Navigation_Data(:,2), Navigation_Data(:,3)];	%提取导航数据，提取陀螺数据，顺序为北向�?天向、东�?
    fmnav = [ Navigation_Data(:,7), Navigation_Data(:,5), Navigation_Data(:,6)];	%提取加表数据，顺序为右�?前�?�?
    
	Tkf = 100 * glv.Ts;				%卡尔曼滤波周期，0.5s�?��
	
	stalen = length(StaticAlignment_Data);
	navlen = length(Navigation_Data);
	gpslen = length(GPS_Data);
	
	alinum = 5000;
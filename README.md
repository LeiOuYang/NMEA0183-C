# NMEA0183-C

GPS NMEA0183协议简要解析框架 
*	
*	支持解析 RMC、GGA、VTG、HDT等消息字段信息，由于解析框架比较明了，
*   移植源码者可自行考虑添加新的解析代码。
*   解析方式采用逐个字段解析的方式，能很大程度上节省消耗内存空间。 
*   重新编写数据转换函数，不依赖外部库，移植时一般只需包含NMEA0183.h
*   文件即可。 
*
*   代码示例：
*   	
		char nmea[] = \
		"$GNRMC,102219.00,A,2239.11578,N,11406.59325,E,0.009,,291018,,,D*62\r\n"\
		"$GNVTG,,T,,M,0.009,N,0.017,K,D*37\r\n"\
		"$GNGGA,102220.00,2239.11583,N,11406.59338,E,2,09,1.30,112.7,M,-2.3,M,,0000*52\r\n";
		
		gps_nmea gps_nmea_temp;   /// 定义解析时用到的暂存状态数据,只供解析时使用 
		gps_data gps_data1;       /// GPS解析得到的最终数据，用户可使用 
		unsigned int index = 0;
		
		for(index=0; index<sizeof(nmea); ++index)
		{
			if(nmea_decode(&gps_nmea_temp, &gps_data1, nmea[index])) ///调用解析函数
			{
				/// 解析消息段成功 
				... 
			} 
		} 
		
*   NMEA0183解析代码思想来源于开源飞控代码(Ardupilot)，所以该份代码开源，可免费使用。
*   ( 如有Bug，望能反馈 ! QQ: 2281280195 ) 
*       
*   作者： ouyanglei      	时间：2018-11-16

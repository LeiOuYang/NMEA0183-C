/*
*				GPS NMEA0183协议简要解析框架 
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
*/


#ifndef NMEA0183_H
#define NMEA0183_H

/*************** 宏定义  *******************/ 
#define current_time_ms() 1

/* 检查字符串是否为数字0-9 */ 
#define IS_DIGITAL(x) ( ((x)>='0'&&(x)<='9')||(x)=='-'||(x)=='.' )
#define CHAR_TO_DIGITAL(x) ((x)-'0')
#define DIGITAL_TO_CHAR(x) ( (x)+'0' )

/**************** 枚举量定义 ***************/
	/* 解析字段枚举量 */ 
    typedef enum _sentence_types 
	{   /* 每个枚举量表示该字段的起始位置，有些特殊字段需要10个字段域 */ 
        GPS_SENTENCE_RMC = 32,     /* RMC字段 */
        GPS_SENTENCE_GGA = 64,		/* GGA字段 */
        GPS_SENTENCE_VTG = 96,		/* VTG字段 */
        GPS_SENTENCE_HDT = 128,    /* HDT字段 */
        GPS_SENTENCE_OTHER = 0     /* 默认没有字段 */
    }sentence_types;

/**************** 结构体定义 ***************/    
    /* GPS定位状态枚举量 */ 
    typedef enum _gps_status
	{
        NO_GPS = 0,              /* 无GPS */
        NO_FIX,                  /* 接收到GPS信息，但没有定位 */
        GPS_OK_FIX_2D,           /* 2D定位 */
        GPS_OK_FIX_3D,           /* 3D定位 */ 
        GPS_OK_FIX_3D_DGPS,      /* 优于3D定位 */ 
        GPS_OK_FIX_3D_RTK_FLOAT, /* RTK浮点解状态 */ 
        GPS_OK_FIX_3D_RTK_FIXED, /* RTK固定解状态 */
    }gps_status;
    
    /* GPS点位置 */ 
    typedef struct _location 
	{
	    int alt:24;         /* 海拔高度 meters * 100  单位为厘米 */
	    float lat;            /* 纬度 *  暂时不乘以10**7，原始保存*/
	    float lng;            /* 精度 *  暂时不乘以10**7，原始保存*/
	}Location;
    
    /* 数据解析状态结构体定义 */
    typedef struct _gps_nmea
    {
    	/* 数据解析状态 */ 
    	unsigned char parity;    		   /* 校验计算和 */ 
    	unsigned char is_checksum_term;    /* 当前字段域为校验字 */
    	char term[15];           		   /* 当前字段域缓冲区，','为字段域分割符 */
    	unsigned char sentence_type;       /* 字段类别，见枚举 sentence_types*/
    	unsigned char term_number;         /* 当前字段域序号 */
    	unsigned char term_offset;         /* 字段域数据偏移量 */
    	unsigned char gps_data_good;       /* GPS数据有效状态 */
    	
    	/* 数据解析结果 */
    	unsigned int new_time;                  /* UTC时间 */
    	unsigned int new_date;                  /* UTC日期 */
    	float new_latitude;             		/* 纬度 */
    	float new_longitude;            		/* 经度 */
    	int new_altitude;             			/* 海拔高度 cm*/
    	float new_speed;                		/* 地速 km/h */
    	float new_course;               		/* RMC、VTG字段磁偏角信息*/
    	float new_gps_yaw;              		/* HDT航向信息 */
    	unsigned short new_hdop;                /* 水平精度 *100 */
    	unsigned char new_satellite_count;      /* 当前使用卫星的颗数 */
    	unsigned char new_quality_indicator;    /* 定位状态 */

    	unsigned int last_RMC_ms;         	    /* 最后更新字段时间 */
    	unsigned int last_GGA_ms;
    	unsigned int last_VTG_ms;
   		unsigned int last_HDT_ms;
	}gps_nmea;
	
	/* GPS数据区域，供上层应用调用 */ 
	typedef struct _gps_data {
        unsigned char instance; 		   /* GPS实例个数 */ 
        gps_status status;                 /* GPS定位状态 */
        unsigned int time_week_ms;         /* GPS time (milliseconds from start of GPS week)*/ 
        unsigned short time_week;          /* GPS星期号 */ 
        Location location;                 /* GPS定位到的当前位置 */
        float ground_speed;                /* 地速  m/s */
        float ground_course;               /* 地速航向  度  顺时针 0-360 */ 
        float gps_yaw;                     /* GPHDT字段航向信息 一般用于双天线测向 */
        unsigned short hdop;               /* 水平精度  cm */
        unsigned short vdop;               /* 垂直精度  cm */
        unsigned char num_sats;            /* 可见卫星颗数 */
        unsigned char have_gps_yaw;        /* GPHDT航向字段有效 */
        unsigned int last_gps_time_ms;     /* 最后获取GPS信息时的时间  ms */
    }gps_data;
    
/**************** 函数定义 ***************/

unsigned char nmea_decode(gps_nmea* pnmea, gps_data* pdata, char c);

#endif



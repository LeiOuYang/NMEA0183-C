
#ifndef UBLOX_NMEA0183_H
#define UBLOX_NMEA0183_H
#include <stdbool.h>
#include <stdlib.h>
#include <math.h>

#define IMPORT_FLOAT 0   //支持浮点运算 1支持

	#define GPGSA	 "GPGSA"
	#define GPGGA  "GPGGA"
	#define GPGSV  "GPGSV"
	#define GPRMC  "GPRMC"
	#define GPVTG  "GPVTG"
	#define GPGLL  "GPGLL"
	
	#define GNGSA	 "GNGSA"
	#define GNGGA  "GNGGA"
	#define GNGSV  "GNGSV"
	#define GNRMC  "GNRMC"
	#define GNVTG  "GNVTG"
	#define GNGLL  "GNGLL"
	
	#define GLGSA	 "GLGSA"
	#define GLGGA  "GLGGA"
	#define GLGSV  "GLGSV"
	#define GLRMC  "GLRMC"
	#define GLVTG  "GLVTG"
	#define GLGLL  "GLGLL"
	
	#define GBGSA	 "GBGSA"
	#define GBGGA  "GBGGA"
	#define GBGSV  "GBGSV"
	#define GBRMC  "GBRMC"
	#define GBVTG  "GBVTG"
	#define GBGLL  "GBGLL"
	
	#define BDGSA	 "BDGSA"
	#define BDGGA  "BDGGA"
	#define BDGSV  "BDGSV"
	#define BDRMC  "BDRMC"
	#define BDVTG  "BDVTG"
	#define BDGLL  "BDGLL"
	
	#define NMEA_NO	0
	#define NMEA_START 1 
	#define NMEA_CRC1 2
	#define NMEA_CRC2 3
	#define NMEA_END 4
	
//define enum
	typedef enum _gps_message_flag
	{
		GPS_MSG_NO = 0,
		GPS_MSG_GGA, 	//定位信息
		GPS_MSG_GSA,  //当前卫星信息
		GPS_MSG_GSV,	//可见卫星信息
		GPS_MSG_RMC,	//推荐定位信息数据格式	
		GPS_MSG_VTG,  //地面速度信息
		GPS_MSG_GLL,  //地理定位信息
	}gps_message_flag;
//end enum define 
	
//define the gps message struct	
	typedef struct _gps_message
	{
	#if IMPORT_FLOAT==1  //支持浮点运算
		double longitude;  //经度
		double latitude;   //纬度
		double altitude;	 //高度
		float pdop; //位置精度
		float hdop;	//水平精度
		float vdop;	//垂直精度
	#endif
		float groundSpeed;
		char lat_direc;
		char long_direc;
		/*
		*  B  L  P
		*	 3  2  1
		*  0  0  1  - GPS
		*  0  1  1  - GPS+格林奈斯 
		*  1  0  1  - GPS+BD
		*  0  1  1  - BD+格林奈斯 
		*  0  1  0  - 格林奈斯 
		*  1  1  0  - BD+格林奈斯 
		*  1  0  0  - BD
		*  1  1  1  - GPS+格林奈斯+BD
		*/
		unsigned char sttl_class;           //当前使用的星种 
		unsigned char scan_satellite_count; //收到卫星信号的总数
		unsigned char use_satellite_count; //正在使用的卫星个数
		unsigned char gp_scan_count;       //扫描
		unsigned char gb_scan_count;
		unsigned char gl_scan_count;
//		unsigned char gp_use_count;				//使用
//		unsigned char gb_use_count;
//		unsigned char gl_use_count;
		unsigned char gps_per; 		/*GPS状态 0-不可用fix not，1-单点定位GPS FIX
															*2-差分定位DGPS，3-无效PPS，4-实时差分定位RTK FIX
															*5-RTK FLOAT，6-正在估算*/
		bool data_val;   //定位数据有效
		unsigned char mode; //定位模式、手动或者自动 A  M
		unsigned char fix_mode; //定位类型 1 = 未定位, 2 = 二维定位, 3 = 3维定位
		unsigned char val;     //数据有效
	}gps_message;
//end gps_message struct

//定义保存卫星信息的结构体	
	typedef struct _gps_gsv
	{
		unsigned char enable;  //占用状态  1-占用  0-没有占用，表示新的卫星信息可以保存在其中
		unsigned char prn_number;
		unsigned char db;		
	}gps_gsv;
//end gps_gsv define 
		
	/* 函数接口 */
	bool is_digital(char d);
#if IMPORT_FLOAT==1  //字符串转换浮点函数
	double string_to_float(char* str, unsigned len);
#endif
	static void copy_string(char* dest,char *src,unsigned int num);
	bool init_char_buff(char* buff, unsigned int num, char value);
	bool get_nmea_frame(char usartC, char* frame, unsigned int* cou);//获取一帧数据 开始数据'$' 结束数据'*' 其后为两个字节校验位
	unsigned char hex_to_uint(char c); 			 //字符转换为整数
	unsigned char char_to_int(char c);
	void nema_message_parse(char* frame, gps_message* pos, unsigned int len);		 //解析得到信息段
	
	void rmc_parse(char* frame, gps_message* pos, unsigned int len);  //解析RMC数据，建议最小定位信息
	void gsv_parse(char* frame, gps_message* pos, unsigned int len);  //解析GSV数据，可视卫星格式
	void gsa_parse(char* frame, gps_message* pos, unsigned int len);  //解析GSA数据，GPS精度指针及使用卫星格式
	void gll_parse(char* frame, gps_message* pos, unsigned int len);  //解析GLL数据
	void gga_parse(char* frame, gps_message* pos, unsigned int len);  //解析GGA数据，固定数据输出
	void vtg_parse(char* frame, gps_message* pos, unsigned int len);  //解析VTG数据，地面速度信息
	
	static void update_stll_msg(gps_gsv* gsvMsg, unsigned char len, unsigned char prn, unsigned char dbh);  //更新卫星的信息
	
	gps_message* get_gps_message(void);
	void init_gps_message(gps_message* gpsMsg);
	
		/*获取GPS信息经度、纬度、高度等*/
	char* get_gps_longitude_str(void);  
	char* get_gps_latitude_str(void);
	char* get_gps_altitude_str(void);
	char* get_gps_utc_date_str(void);
	char* get_gps_utc_time_str(void);
	char* get_gps_utc_str(void);
	char* get_utc(char* buff);
	char* get_gps_pdop_str(void);
	char* get_gps_hdop_str(void);
	char* get_gps_vdop_str(void);
	unsigned char get_db_average(void);
	
	void set_gps_longitude_str(char* str);  
	void set_gps_latitude_str(char* str);
	void set_gps_altitude_str(char* str, unsigned int len);
	void set_gps_utc_date_str(char* str);
	void set_gps_utc_time_str(char* str);
	void set_gps_pdop_str(char* str, unsigned char len);
	void set_gps_hdop_str(char* str, unsigned char len);
	void set_gps_vdop_str(char* str, unsigned char len);
	
	void local_time(char* ddmmyy, char* hhmmss, unsigned char localTimeArea);
	
	void reset_gps_gsv(gps_gsv* gsv);
	unsigned char get_gps_msg_val(void);
	
	bool gps_valid(gps_message* gps);
	
	#define NMEA0183_FRAME_MAX_LEN		100   //NMEA数据帧的最大长度
	extern char nmea_buff[NMEA0183_FRAME_MAX_LEN];//外部调用，可将解析成功的一帧数据保存在该缓冲区中get_nmea_frame()和nema_message_parse()函数调用
	extern gps_message gpsMsg;
	extern gps_gsv gpgsv[12];  
	extern gps_gsv glgsv[12];  
	extern gps_gsv gbgsv[12];
	
#endif /* UBLOX_NMEA0183_H */


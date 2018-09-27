/*************************************************************************************
***
*** 		GNSS NEMA0183数据解析，支持对：RMC、GSV、GLL、TVS、GGA、GSA数据包的解析	
***			库函数调用示例如下：
***       char buff[];  //该缓冲区中存有协议数据
***       unsigned int count = 0;  //定义存储帧数据的个数
***				init_gps_message(get_gps_message()); //初始化库内部定义的gps_message
***       for(int i=0; i<val_count; ++i)  //val_count表示缓冲区中有效数据个数
***				{			
***     		if(get_nmea_frame(buff, nmea_buff, &gpscount) //按字符解析，如果得到一帧数据，则为真
***       	{
***						nema_message_parse(nmea_buff, get_gps_message(), gpscount); //对一帧数据进行解析
***					}
***					++buff;
***				}
***       解析成功后，可通过调用内置的gpsMsg、gpgsv、gbgsv、gbgsv，获取定位信息和卫星状态
*** 
***
***     BY Awesome OYL     2017-11-05 
***
**************************************************************************************/


#include "NMEA0183.h"

char nmea_buff[NMEA0183_FRAME_MAX_LEN] = {0};   //一帧数据缓冲区，供外部调用
gps_message gpsMsg; //全局gps数据信息
gps_gsv gpgsv[12];  //gps
gps_gsv glgsv[12];  //格林奈斯
gps_gsv gbgsv[12];  //北斗

/* 判断字符是否为数字字符 */
bool is_digital(char d) 
{
	if(d>='0'&& d<='9'||'.'==d) 
		return true;
	else 
		return false;
}
//end function

/* 字符串转换为浮点数， len需要转换字符串的长度 */
#if IMPORT_FLOAT==1
double string_to_float(char* str,unsigned int len)
{
	unsigned char num[20];
	double d = 0.0;
	unsigned char flag = 1; 
	unsigned char count;
	unsigned int i = 0;
	unsigned int l = 0;
	unsigned char fuhao = 0;
	
	if(0==str) return 0.0;
	if('-'==*str) //识别符号
	{
		fuhao = 1;
		++str;
	}
	
	while('\0'!=*str)
	{
		if(!is_digital(*str)) //识别数字字符
			break;
		if(l==len)
		{
			if(1==flag)
			{
				while(count)
				{
					d += num[i]*pow(10,--count); 
					++i;
				}
			}
			break;
		}
		if('.'==*str)
		{
			flag=0;
			while(count)
			{
				d += num[i]*pow(10,--count); 
				++i;
			}
			if(l==len) 
			{
				break;
			}
		}
		if(1==flag)
		{
			num[count] = hex_to_uint(*str);
			++count;
		}else if(0==flag)
		{
			d += (double)hex_to_uint(*str)*(double)pow(10,-count); 
			++count;
		}
		++str;
		++l;
	}
	if(1==flag) 
	{
		while(count)
		{
			d += num[i]*pow(10,--count); 
			++i;
		}
	}
	if(1==fuhao) d = -d;
	return d;
}
#endif
//end function

/* 十六进制字符转换为数字 */
unsigned char hex_to_uint(char c)
{
	if(c>='0'&& c<='9')
	{
		return (unsigned int)(c-48);
	}else if(c>='A' && c<='F')
	{
		return (unsigned int)(c-55);
	}
	return 0;
}
//end funciton

/* 复制字符串  复制结束条件是'\0'和指定长度*/
static void copy_string(char* dest,char *src,unsigned int num) 
{ 
	if(dest!=0&&src!=0)
	{ while(num--&&'\0'!=*src)
		{
			*dest = *src;
			++dest;
			++src;
		} 
		*dest='\0'; 
	}
} 
//end function

/* 指定长度和初始化值初始化字符缓冲区，成功返回true */
bool init_char_buff(char* buff, unsigned int num, char value)
{
	if(0==buff&&0==num)  return false;
	
	while(num--)
	{
		*buff = value;
		++buff;
	}
	return true;
}
//end function

/* 获取全局gps_message结构体 */
gps_message* get_gps_message(void)
{
	return &gpsMsg;  //定义的全局变量，变量中保存gps的信息。请见结构体gps_message @ublox_NMEA0183.h
}
//end function

/* 指定初始化一个结构体gps_message */
void init_gps_message(gps_message* gpsMsg)
{
	if(0!=gpsMsg)
	{
		gpsMsg->data_val = false;
		gpsMsg->fix_mode = 0;
		gpsMsg->lat_direc = 'N';
		gpsMsg->scan_satellite_count = 0;
		gpsMsg->gps_per = 0;
		gpsMsg->long_direc = 'N';
		gpsMsg->use_satellite_count =0;
		gpsMsg->mode = 0;
		gpsMsg->sttl_class = 0x00;
		gpsMsg->gp_scan_count = 0;       //扫描
		gpsMsg->gb_scan_count = 0;
		gpsMsg->gl_scan_count = 0;
		gpsMsg->val = 0;
	#if IMPORT_FLOAT==1
		gpsMsg->altitude = 0.0;
		gpsMsg->latitude = 0.0;
		gpsMsg->longitude = 0.0;
		gpsMsg->altitude = 0.0;
		gpsMsg->hdop = 0.0;
		gpsMsg->vdop =0.0;
		gpsMsg->pdop = 0.0;
	#endif
	}
}
//end function

/* 逐个字符解析NMEA0183函数，输入参数frame保存返回解析成功的数据帧，
		不带$, 以'*'结束 成功解析一帧数据返回 真 true*/
bool get_nmea_frame(char usartC, char* frame, unsigned int* cou)
{
	static unsigned char flag = NMEA_NO;	
	static unsigned int count = 0;				
	static char crc[2];
	static unsigned int crcSum;
	
	if(0==frame) return false;
		
	switch(flag)
	{
		case NMEA_NO:
		{
			if('$'==usartC)    //识别到开始符'$'    
				flag = NMEA_START;	
			break;
		}
		case NMEA_START:
		{
			if('*'==usartC && count>0)	//识别结束符'*'
			{
				*(frame+count) = '*';			//保存结束符到输出缓冲区中
				++count;
				flag = NMEA_CRC1;					//接下解析第一个校验字符
				*cou = count;							//返回缓冲区中的个数
			}else if(count<NMEA0183_FRAME_MAX_LEN)	//检验帧长是否超出返回
			{
				*(frame+count) = usartC;  
				++count;
				crcSum ^= usartC;		//校验和为'$'和'*'中的所有字符异或结构的字符表示，包括','			 
			}else
			{
				flag = NMEA_NO;
				count = 0;
				crc[0] = crc[1] = 0;
				crcSum = 0;
			}
			break;
		}
		case NMEA_CRC1:	//第一个校验位
		{
			crc[1] = usartC;
			flag = NMEA_CRC2;
			break;
		}
		case NMEA_CRC2:	//第二个校验位
		{
			crc[0] = usartC;
			flag = NMEA_END;
			break;
		}
		default: break;
	}
	if(NMEA_END==flag)  //校验位获取完成，也就是所有数据都解析完成
	{
		flag = NMEA_NO;
		count = 0;
		if(crcSum==(hex_to_uint(crc[0])+hex_to_uint((crc[1]))*16)) //判断检验和
		{
			crc[0] = crc[1] = 0;
			crcSum = 0;
			return true;
		}
	}		
	return false;
}
//end function


/* 解析通筭get_nmea_frame函数获取的正确帧数据  */
void nema_message_parse(char* frame, gps_message* pos, unsigned int len)
{
	char msg[6];
	if(0==frame) return;
	
	pos->val = 0;
	copy_string(msg,frame,2);
	
	switch(msg[1])   //获取星系
	{
		case 'P':
		{
			pos->sttl_class = pos->sttl_class | 0x01;
			break;
		}
		case 'L':
		{
			pos->sttl_class = pos->sttl_class | 0x02;
			break;
		}
		case 'B':
		case 'D':
		{
			pos->sttl_class = pos->sttl_class | 0x04;
			break;
		}
		default: break;
	}
	
	copy_string(msg,frame,5); //获取ID(GPGSA等字符)
	
	if(!strcmp(msg,GNGSA)|| !strcmp(msg,GPGSA) || !strcmp(msg,GLGSA) || !strcmp(msg,GBGSA) )
	{
		gsa_parse(frame, pos, len); //GSA
		
	}else if(!strcmp(msg,GNGGA) || !strcmp(msg,GPGGA) || !strcmp(msg,GLGGA) || !strcmp(msg,GBGGA) )	
	{
		gga_parse(frame, pos, len); //GGA
		
	}else if(!strcmp(msg,GPGSV) || !strcmp(msg,GNGSV) || !strcmp(msg,GLGSV) || !strcmp(msg,GBGSV ))
	{
		gsv_parse(frame, pos, len); //GSV
		
	}else if(!strcmp(msg,GPRMC) || !strcmp(msg,GNRMC) || !strcmp(msg,GLRMC) || !strcmp(msg,GBRMC))	
	{
		rmc_parse(frame, pos, len); //RMC
		
	}else if(!strcmp(msg,GPVTG) || !strcmp(msg,GNVTG) || !strcmp(msg,GLVTG) || !strcmp(msg,GBVTG) )	
	{
		vtg_parse(frame, pos, len);  //VTG
		
	}else if(!strcmp(msg,GPGLL) || !strcmp(msg,GNGLL) || !strcmp(msg,GLGLL) || !strcmp(msg,GBGLL))	
	{
		gll_parse(frame, pos, len); //GLL
		
	}else
	{
		
	}	
	
	pos->scan_satellite_count = pos->gb_scan_count+pos->gp_scan_count+pos->gl_scan_count;
	pos->val = 1;
}
//end function


/* 解析RMC 推介定位信息 UTC时间，经纬度，速度，方位角信息*/
void rmc_parse(char* frame, gps_message* pos, unsigned int len)
{
	char* fr;
	unsigned char count = 0;
	unsigned int index = 0;
	char buff[15];
	unsigned int t = 0;
	
	if(0==frame&&0==pos) return;
	
	fr = frame+6;  //跳过ID字符，和前面的',' 
	while('*'!=*fr&&t<5000) //识别结束符
	{
		++t;
		if(','!=*fr) //如果遇到的不是','，则将字符保存在缓冲区中
		{
			buff[index] = *fr;
			++index;
		}else if(','==*fr) //知道遇到','，再对缓冲区中的数据进行处理
		{

			buff[index] = '\0'; //字符串结束标志，方便后续处理
			++count; 
			if(index>0) //缓冲区中的数据都是有效
			{
				if(1==count)  //字段1，utc-time
				{
					set_gps_utc_time_str(buff); //utc time hhmmss.sss

				}else if(2==count)   //字段2，定位状态
				{
					if(buff[0]=='A')
					{
						pos->data_val = true;				

					}else{
						pos->data_val = false;

					}
				}else if(3==count) //字段3 纬度
				{
					set_gps_latitude_str(buff);
				#if IMPORT_FLOAT==1					
					pos->latitude = string_to_float(buff, index); //stm32f103处理浮点数太慢，不推介使用
				#endif
				}else if(4==count) //字段4 纬度类别
				{
					pos->lat_direc = buff[0];//N or S

				}else if(5==count) //字段5 经度
				{
					set_gps_longitude_str(buff);
				#if IMPORT_FLOAT==1		
					pos->longitude = string_to_float(buff, index);
				#endif	

				}else if(6==count) //字段6 经度类别
				{
					pos->long_direc = buff[0];// E or W

				}else if(7==count) //字段7 速度 节knots
				{

				}else if(8==count) //字段8 方位角 度
				{

				}else if(9==count) //字段9 utc-日期
				{
					set_gps_utc_date_str(buff);

				}else if(10==count) //字段10 磁偏角
				{

				}else if(11==count) //字段11 磁偏角方向  E-东  W-西
				{

				}
		  }
			index = 0;	//获取一个字段结束，将缓冲区标志清0	
		}
		++fr; //获取下个缓冲区的数据
	}
	if(count>0)//字段12 模式 A-自动  D-差分  E-估测  N-数据无效
	{
		
	}
}
//end function

/* 解析GSV  可见卫星信息 当前可见卫星数，PRN编码，卫星仰角，卫星方位角，信噪比
*  卫星信息保存在定义的缓冲区中,具有12个元素的gps_gsv数组缓冲中。
*/
void gsv_parse(char* frame, gps_message* pos, unsigned int len)
{
	if(0!=frame&&0!=pos)
	{
		char* fr;
		unsigned char count = 0;
		unsigned int index = 0;
		char buff[15];
		unsigned int t = 0;
		unsigned int j = 0;
		unsigned int i = 0;
		unsigned char prn = 0;
		unsigned char DBH = 0;
		char msg[6];
		static unsigned char gsv_count = 0;
		static unsigned char gsv_current_count = 0;
			
		
		if(0==frame&&0==pos) return;
		
		fr = frame+6;  

		while('*'!=*fr&&t<5000)
		{
			++t;
			if(','!=*fr)
			{
				buff[index] = *fr;
				++index;
			}else if(','==*fr) 
			{
				buff[index] = '\0';
				++count; 
				if(index>0)
				{
					if(1==count)
					{   
						gsv_count = (buff[0]-0x30)*10+buff[1]-0x30;  //语句个数 
						reset_gps_gsv(gpgsv);
						reset_gps_gsv(glgsv);
						reset_gps_gsv(gbgsv);
					}else if(2==count)
					{
						gsv_current_count = (buff[0]-0x30)*10+buff[1]-0x30;   //语句编号
						if(gsv_current_count==gsv_count) 
						{
							gsv_count = 0;
							gsv_current_count = 0;
						}
					}else if(3==count)  //字段3  可见卫星个数
					{
						unsigned char tmp = 0;
						tmp = (buff[0]-0x30)*10+buff[1]-0x30;  						
						copy_string(msg,frame,2);
						
						switch(msg[1])   //获取星系
						{
							case 'P':
							{
								pos->gp_scan_count = tmp;
								break;
							}
							case 'L':
							{
								pos->gl_scan_count = tmp;
								break;
							}
							case 'B':
							case 'D':
							{
								pos->gb_scan_count = tmp;
								break;
							}
							default: break;
						}
						while('*'!=*fr) 
						{
							for(i=0,j=0; i<13; ++i) //将一个卫星的所有信息去除',',按顺序存储到缓冲区中
							{
								if(','!=*fr) 
								{
									buff[j] = *fr;
									++j;
								}
								++fr;
							}
							prn = (buff[0]-0x30)*10+buff[1]-0x30;  //获取PRN编码
							DBH = (buff[7]-0x30)*10+buff[8]-0x30;  //获取信噪比
							switch(*(frame+1))
							{
								case 'P': //GP
								{								
									update_stll_msg(&gpgsv[0], 12, prn, DBH); //将卫星信息保存到定义的全局gps_msg缓冲中									
									break;
								}
								case 'B': //GB
								{
									update_stll_msg(&gbgsv[0], 12, prn, DBH);
									break;
								}
								case 'L': //GL
								{
									update_stll_msg(&glgsv[0], 12, prn, DBH);
									break;
								}
								default: break;							
							}
						}
						break;  //获取完毕
					}
				}
				index = 0;		
			}
			++fr;
		}
	}
}
//end function

//复位gps_gsv全局变量
void reset_gps_gsv(gps_gsv* gsv)
{
	unsigned char i = 0;
	if(0==gsv) return;
	for( ; i<12; ++i)
	{
		gsv->enable = 0;
	}
}
//end function

//数据有效标志
unsigned char get_gps_msg_val(void)
{
	return gpsMsg.val;
}

//获取可见卫星平均信噪比
unsigned char get_db_average(void)
{
	unsigned char count = 0;
	unsigned int sum = 0;
	unsigned char avr = 0;
	unsigned char i = 0;
	
	for(i=0; i<12; ++i)
	{
		if(gpgsv[i].enable)
		{
			sum += gpgsv[i].db;
			++count;
		}
	}
	
	for(i=0; i<12; ++i)
	{
		if(gbgsv[i].enable)
		{
			sum += gbgsv[i].db;
			++count;
		}
	}
	
	for(i=0; i<12; ++i)
	{
		if(glgsv[i].enable)
		{
			sum += glgsv[i].db;
			++count;
		}
	}
	avr = sum/count;
	return avr;
}

/* 更新卫星信息 */
static void update_stll_msg(gps_gsv* gsvMsg, unsigned char len, unsigned char prn, unsigned char dbh)  
{
	if(0!=gsvMsg&&0!=len) 
	{
		while(len--)
		{
			if(gsvMsg->prn_number==prn) //缓冲区中已经保存有信息
			{
				gsvMsg->db = dbh;
				gsvMsg->enable = 1;
				return;
			}
					
			if(gsvMsg->prn_number!=prn&&0==gsvMsg->enable) //缓冲区中未有保存信息并且存在可存入信息的缓冲区
			{
				gsvMsg->enable = 1; //缓冲区中已经存有数据标志
				gsvMsg->prn_number = prn;
				gsvMsg->db = dbh;
				return;
			}
			++gsvMsg;
		}
	}		
}
//end function

/*解析GGA 定位信息 UTC-time,经纬度，GPS状态，正在使用的卫星数，水平经度，海拔高度*/
void gga_parse(char* frame, gps_message* pos, unsigned int len)
{
	char* fr;
	unsigned char count = 0;
	unsigned int index = 0;
	char buff[15];
	unsigned int t = 0;
	char msg[6];
	
	if(0==frame&&0==pos) return;
	
	fr = frame+6;  

	while('*'!=*fr&&t<5000)
	{
		++t;
		if(','!=*fr)
		{
			buff[index] = *fr;
			++index;
		}else if(','==*fr) 
		{
			buff[index] = '\0';
			++count; 
			if(index>0)
			{
				if(1==count) //字段1 utc time
				{
					set_gps_utc_time_str(buff);	
				}else if(2==count) //字段2 纬度
				{
					set_gps_latitude_str(buff);
				#if IMPORT_FLOAT==1		
					pos->latitude = string_to_float(buff, 9); 
				#endif		
				}else if(3==count)
				{
					pos->lat_direc = buff[0];//N or S
				}else if(4==count) //字段4 经度
				{
					set_gps_longitude_str(buff);
				#if IMPORT_FLOAT==1		
					pos->longitude = string_to_float(buff, index);
				#endif
				}else if(5==count) 
				{
					pos->long_direc = buff[0];// E or W

				}else if(6==count) //字段6 GPS状态 0-不可用 1-单点定位 2-差分定位 
													//	3-无效PPS 4-实时差分定位  5-RTK FLOAT 6-正在估算
				{
					pos->gps_per = buff[0];  

				}else if(7==count) //字段7 正在使用的卫星数
				{
					unsigned char tmp = 0;
					tmp = (buff[0]-0x30)*10+buff[1]-0x30;  						
					copy_string(msg,frame,2);
					
					switch(msg[1])   //获取星系
					{
						case 'P':
						{
							break;
						}
						case 'L':
						{
							break;
						}
						case 'B':
						case 'D':
						{
							break;
						}
						case 'N':
						{
							pos->use_satellite_count = (buff[0]-0x30)*10+buff[1]-0x30;  
							break;
						}
						default: break;
					}												
						
				}else if(8==count) //字段8 水平精度
				{
					set_gps_hdop_str(buff, index+1);
				#if IMPORT_FLOAT==1		
					pos->hdop = string_to_float(buff, index);
				#endif
				}else if(9==count) //字段9 海拔高度
				{
					set_gps_altitude_str(buff, index); //高度字符串是可变长度的
				#if IMPORT_FLOAT==1		
					pos->altitude = string_to_float(buff,index);
				#endif

				}else if(10==count) //字段10  海拔单位
				{
					
				}else if(11==count)
				{
				}
		  }
			index = 0;		
		}
		++fr;
	}
	if(count>0) //字段12
	{
	}
}
//end function

/* 解析GSA 当前卫星信息  定位模式，定位类型，字段3-14为12颗卫星信息的PRN编码，位置、水平、垂直精度  */
void gsa_parse(char* frame, gps_message* pos, unsigned int len)
{
	char* fr;
	unsigned char count = 0;
	unsigned int index = 0;
	char buff[15];
	unsigned int t = 0;
	
	if(0==frame&&0==pos) return;
	
	fr = frame+6;  

	while('*'!=*fr&&t<5000)
	{
		++t;
		if(','!=*fr)
		{
			buff[index] = *fr;
			++index;
		}else if(','==*fr) 
		{

			buff[index] = '\0';
			++count; 
			if(index>0)
			{
				if(1==count) //字段1 定位模式 A-自动  M-手动
				{
					pos->mode = buff[0];  
				}else if(2==count) //字段2 定位类型  1-未定位  2-2D定位  3-3D定位
				{
					pos->fix_mode = buff[0];  
				}else if(3==count)
				{

				}else if(4==count)
				{

				}else if(5==count)
				{

				}else if(6==count)
				{

				}else if(7==count)
				{

				}else if(8==count)
				{

				}else if(9==count)
				{

				}else if(10==count)
				{
					
				}else if(11==count)
				{
					
				}else if(12==count)
				{
				}else if(13==count)
				{
				}else if(14==count)
				{
				}else if(15==count) //字段15 位置精度 0.5-99.9
				{
					set_gps_pdop_str(buff, index+1);
				#if IMPORT_FLOAT==1		
					pos->pdop = string_to_float(buff, index);	
				#endif
				}else if(16==count) //字段16 水平精度
				{
					set_gps_hdop_str(buff, index+1);
				#if IMPORT_FLOAT==1		
					pos->hdop = string_to_float(buff, index);
				#endif
				}
		  }
			index = 0;		
		}
		++fr;
	}
	if(count>0) //字段17 垂直精度
	{
		set_gps_vdop_str(buff, index+1);
	#if IMPORT_FLOAT==1		
		pos->vdop = string_to_float(buff, index);
	#endif
	}
}
//end function

/* 解析GLL 地理定位信息 经纬度，UTC-time，定位状态 */
void gll_parse(char* frame, gps_message* pos, unsigned int len)
{
	char* fr;
	unsigned char count = 0;
	unsigned int index = 0;
	char buff[15];
	unsigned int t = 0;
	
	if(0==frame&&0==pos) return;
	
	fr = frame+6;  

	while('*'!=*fr&&t<5000)
	{
		if(','!=*fr)
		{
			buff[index] = *fr;
			++index;
		}else if(','==*fr) 
		{
			buff[index] = '\0';
			++count; 
			if(index>0)
			{
				if(1==count) //字段1 纬度
				{
					set_gps_latitude_str(buff);
				#if IMPORT_FLOAT==1		
					pos->latitude = string_to_float(buff, 9); 
				#endif
				}else if(2==count)
				{
					pos->lat_direc = buff[0];//N or S
				}else if(3==count) //字段3 经度
				{
					set_gps_longitude_str(buff);
				#if IMPORT_FLOAT==1		
					pos->longitude = string_to_float(buff, index);
				#endif
				}else if(4==count)
				{
					pos->long_direc = buff[0];// E or W
				}else if(5==count) //字段5 utc time
				{
					set_gps_utc_time_str(buff);	
				}
		  }
			index = 0;		
		}
		++fr;
	}
	
	if(count>0) //字段6 定位状态 A-有效  V-未定位
	{
		if(buff[0]=='A')
		{
			pos->data_val = true;
		}else{
			pos->data_val = false;
		}
	}
}
//end function

/* 解析VTG 地理速度信息  运动角度，参照系，水平运动速度*/
void vtg_parse(char* frame, gps_message* pos, unsigned int len)
{
		char* fr;
	unsigned char count = 0;
	unsigned int index = 0;
	char buff[15];
	unsigned int t = 0;
	
	if(0==frame&&0==pos) return;
	
	fr = frame+6;  
	while('*'!=*fr&&t<5000)
	{
		if(','!=*fr)
		{
			buff[index] = *fr;
			++index;
		}else if(','==*fr) 
		{
			buff[index] = '\0';
			++count; 
			if(index>0)
			{
				if(1==count) //字段1 运动角度 000-359
				{
					
				}else if(2==count) //字段2 T 真北参照系
				{
					
				}else if(3==count) //字段3 运动角度
				{
					
				}else if(4==count) //字段4 磁北参照系
				{
					
				}else if(5==count) //字段5 水平运动速度0.00
				{
					
				}else if(6==count) //字段6 速度单位 N=节 Knots
				{
					
				}else if(7==count) //字段7 水平运动速度0.00
				{
					
				}
		  }
			index = 0;		
		}
		++fr;
	}
	if(count>0) //字段8 运动速度单位 K=公里/时
	{
	}
}
//end function

/* GPS信息获取函数 */

//经度获取  字符串总长12 有效数据位为11
char* get_gps_longitude_str(void) 
{
	static char longitudeStr[] ={"dddmm.mmmmm"};
	
	return &longitudeStr[0];	
}
void set_gps_longitude_str(char* str)   
{
	init_char_buff(get_gps_longitude_str(), 12, '!');
	copy_string(get_gps_longitude_str(),str,11);
}
//经度 end

//纬度获取  字符串总长11 有效数据位为10
char* get_gps_latitude_str(void)
{
	static char latitude[]={"ddmm.mmmmm"};
	
	return &latitude[0];
}
void set_gps_latitude_str(char* str)
{
	init_char_buff(get_gps_latitude_str(), 11, '!');
	copy_string(get_gps_latitude_str(),str,10);
}
//纬度 end

//高度获取  字符串总长8 有效数据位为7
char* get_gps_altitude_str(void)
{
	static char altitude[]={7,'9','9','9','9','9','.','9','\0'};  //第一个字符为长度
	static unsigned char len = 0;
	
	return &altitude[0];
}
void set_gps_altitude_str(char* str, unsigned int len)
{
	init_char_buff(get_gps_altitude_str(), 9, '!');
	*(get_gps_altitude_str()) = len;
	copy_string(get_gps_altitude_str()+1,str,len);
}
//高度 end

//UTC时间 年月日有效长度6  时间长度10
char* get_gps_utc_date_str(void)
{
	static char date[]={"DDMMYY"};
	
	return &date[0];
}
char* get_gps_utc_time_str(void)
{
	static char time[]={"hhmmss.sss"};
	
	return &time[0];
}
void set_gps_utc_date_str(char* str)
{
	init_char_buff(get_gps_utc_date_str(), 6, '!');
	copy_string(get_gps_utc_date_str(),str,6);
}
void set_gps_utc_time_str(char* str)
{
	init_char_buff(get_gps_utc_time_str(), 10, '!');
	copy_string(get_gps_utc_time_str(),str,10);
}

//char* get_utc(char* buff, unsigned int len)
//{
//	char* pc;
//	char* date;
//	char* time;
//	unsigned int i;
//	if(0==buff&&0==len) return 0;
//	
//	date = get_gps_utc_date_str() + 5;
//	time = get_gps_utc_time_str();	
//	if(0==date&&0==time) return 0;
//	
//	pc = buff;
//	
//	
//}
//日期、时间 end

//位置经度  有效长度3
char* get_gps_pdop_str(void)
{
	static char pdop[] = {"99.9"};
	
	return &pdop[0];
}
void set_gps_pdop_str(char* str, unsigned char len)
{
	init_char_buff(get_gps_pdop_str(), 5, '-');
	copy_string(get_gps_pdop_str(),str,4);   //字符串复制的长度由字符串结束符号与设置的长度决定 
}
//位置 end

//水平经度  有效长度3
void set_gps_hdop_str(char* str, unsigned char len)
{
	init_char_buff(get_gps_hdop_str(), 4, '-');
	copy_string(get_gps_hdop_str(),str,5);   //字符串复制的长度由字符串结束符号与设置的长度决定
}
char* get_gps_hdop_str(void)
{
	static char hdop[] = {"99.9"};

	return &hdop[0];
}
//水平经度 end

//垂直精度 有效长度3
char* get_gps_vdop_str(void)
{
	static char vdop[] = {"99.9"};
	
	return &vdop[0];
}
void set_gps_vdop_str(char* str,unsigned char len)
{
	init_char_buff(get_gps_vdop_str(), 5, '-');
	copy_string(get_gps_vdop_str(),str,4);   //字符串复制的长度由字符串结束符号与设置的长度决定
}
//垂直经度   end
/* END  */

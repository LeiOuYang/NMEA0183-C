
#include "NMEA0183.h"

static short int from_hex(char a);
static unsigned char nmea_term_complete(gps_nmea* pnmea, gps_data* pdata);
static int sring_to_int(const char* pstr, unsigned char* b);
static int int_pow(int value, unsigned int count);
static double double_pow(double value, unsigned int count);
static unsigned char float_to_string(double value, char* pdest, unsigned int intgr, unsigned int dec);
static unsigned char int_to_string(int value, char* pdest, unsigned int intgr) ;
static double string_to_float(const char* pstr, unsigned char* b); 
static int sring_to_int(const char* pstr, unsigned char* b); 
static int make_date_time(gps_data* pdata, unsigned int date, unsigned int time);

/* GPS数据解析 */ 
unsigned char nmea_decode(NMEA0183* nmea, char c)
{
    unsigned char valid_sentence = 0;
    
	if((void*)0==nmea) return 0;

    switch (c) 
	{
	    case ',':  /* 字段域分割符号 */ 
	        nmea->gpsParse.parity ^= c;
	        /* no break */
	    case '\r':
	    case '\n':
	    case '*':
	        if (nmea->gpsParse.term_offset < sizeof(nmea->gpsParse.term)) 
			{
	            nmea->gpsParse.term[nmea->gpsParse.term_offset] = 0;
	            valid_sentence = nmea_term_complete(&nmea->gpsParse, &nmea->gpsData);
	        }
	        ++nmea->gpsParse.term_number;
	        nmea->gpsParse.term_offset = 0;
	        nmea->gpsParse.is_checksum_term = (c == '*');   /* 后面两个字节为校验字段 */ 
	        return valid_sentence;
	
	    case '$': /* 字段信息开始 */ 
	        nmea->gpsParse.term_number = nmea->gpsParse.term_offset = 0;
	        nmea->gpsParse.parity = 0;
	        nmea->gpsParse.sentence_type = GPS_SENTENCE_OTHER;
	        nmea->gpsParse.is_checksum_term = 0;
	        nmea->gpsParse.gps_data_good = 0;
	        return valid_sentence;
    }

    /* 保存字段域数据 */
    if (nmea->gpsParse.term_offset < sizeof(nmea->gpsParse.term) - 1)
        nmea->gpsParse.term[nmea->gpsParse.term_offset++] = c;
    if (!nmea->gpsParse.is_checksum_term)
        nmea->gpsParse.parity ^= c;

    return valid_sentence;
}

/* 单个十六进制字符转换为十进制 */
static short int from_hex(char a)
{
    if (a >= 'A' && a <= 'F')
        return a - 'A' + 10;
    else if (a >= 'a' && a <= 'f')
        return a - 'a' + 10;
    else
        return a - '0';
}

/* 处理单个字段域数据 */
static unsigned char nmea_term_complete(gps_nmea* pnmea, gps_data* pdata)
{
	if((void*)0==pnmea || (void*)0==pdata) return 0;
    // handle the last term in a message
    if (pnmea->is_checksum_term)
	{
        unsigned char checksum = 16 * from_hex(pnmea->term[0]) + from_hex(pnmea->term[1]);
        
        /* 检验信息段是否完整无错，如果校验通过，则将解析得到的数据保存，并提供给用户调用*/
        if (checksum == pnmea->parity) 
		{
            if (pnmea->gps_data_good) 
			{
                unsigned int now = current_time_ms();
                
                switch (pnmea->sentence_type) 
				{
	                case GPS_SENTENCE_RMC:
	                    pnmea->last_RMC_ms = now;
	                    //time                        = _new_time;
	                    //date                        = _new_date;
	                    pdata->location.lat     = pnmea->new_latitude;
	                    pdata->location.lng     = pnmea->new_longitude;
	                    pdata->ground_speed     = pnmea->new_speed;
	                    pdata->ground_course    = pnmea->new_course;
						make_date_time(pdata, pnmea->new_date, pnmea->new_time);
	                    pdata->last_gps_time_ms = now;
	                    
	                    break;
	                case GPS_SENTENCE_GGA:
	                    pnmea->last_GGA_ms = now;
	                    pdata->location.alt  = pnmea->new_altitude;
	                    pdata->location.lat  = pnmea->new_latitude;
	                    pdata->location.lng  = pnmea->new_longitude;
	                    pdata->num_sats      = pnmea->new_satellite_count;
	                    pdata->hdop          = pnmea->new_hdop;

	                    switch(pnmea->new_quality_indicator) {
							case 0: // Fix not available or invalid
								pdata->status = NO_FIX;
								break;
							case 1: // GPS SPS Mode, fix valid
								pdata->status = GPS_OK_FIX_3D;
								break;
							case 2: // Differential GPS, SPS Mode, fix valid
								pdata->status = GPS_OK_FIX_3D_DGPS;
								break;
							case 3: // GPS PPS Mode, fix valid
								pdata->status = GPS_OK_FIX_3D;
								break;
							case 4: // Real Time Kinematic. System used in RTK mode with fixed integers
								pdata->status = GPS_OK_FIX_3D_RTK_FIXED;
								break;
							case 5: // Float RTK. Satellite system used in RTK mode, floating integers
								pdata->status = GPS_OK_FIX_3D_RTK_FLOAT;
								break;
							case 6: // Estimated (dead reckoning) Mode
								pdata->status = NO_FIX;
								break;
							default://to maintain compatibility with MAV_GPS_INPUT and others
								pdata->status = GPS_OK_FIX_3D;
								break;
						}
	                    break;
	                case GPS_SENTENCE_VTG:
	                    pnmea->last_VTG_ms = now;
	                    pdata->ground_speed  = pnmea->new_speed;
	                    pdata->ground_course = pnmea->new_course;

	                    // VTG has no fix indicator, can't change fix status
	                    break;
	                case GPS_SENTENCE_HDT:
						 pnmea->last_HDT_ms = now;
//						 pdata->gps_yaw = wrap_360(pnmea->new_gps_yaw*0.01f);
						 pdata->have_gps_yaw = 1;
						 break;
                }
            } else {
                switch (pnmea->sentence_type) {
                case GPS_SENTENCE_RMC:
                case GPS_SENTENCE_GGA:
                    // Only these sentences give us information about
                    // fix status.
                    pdata->status = NO_FIX;
                }
            }
            // see if we got a good message
            //return _have_new_message();
            return 1;
        }
        // we got a bad message, ignore it
        return 0;
    }

    // the first term determines the sentence type
    if (pnmea->term_number == 0) {
        /*
          The first two letters of the NMEA term are the talker
          ID. The most common is 'GP' but there are a bunch of others
          that are valid. We accept any two characters here.
         */
        if (pnmea->term[0] < 'A' || pnmea->term[0] > 'Z' ||
            pnmea->term[1] < 'A' || pnmea->term[1] > 'Z') {
            pnmea->sentence_type = GPS_SENTENCE_OTHER;
            return 0;
        }
        const char *term_type = &pnmea->term[2];
        if (strcmp(term_type, "RMC") == 0) {
            pnmea->sentence_type = GPS_SENTENCE_RMC;
        } else if (strcmp(term_type, "GGA") == 0) {;
            pnmea->sentence_type = GPS_SENTENCE_GGA;
        }else if (strcmp(term_type, "HDT") == 0) {
            pnmea->sentence_type = GPS_SENTENCE_HDT;
            // HDT doesn't have a data qualifier
            pnmea->gps_data_good = 1;
        }else if (strcmp(term_type, "VTG") == 0) {
            pnmea->sentence_type = GPS_SENTENCE_VTG;
            // VTG may not contain a data qualifier, presume the solution is good
            // unless it tells us otherwise.
            pnmea->gps_data_good = 1;
        } else {
            pnmea->sentence_type = GPS_SENTENCE_OTHER;
        }
        return 0;
    }

    // 32 = RMC, 64 = GGA, 96 = VTG
    if (pnmea->sentence_type != GPS_SENTENCE_OTHER && pnmea->term[0]) {
        switch (pnmea->sentence_type + pnmea->term_number) {
        // operational status
        //
        case GPS_SENTENCE_RMC + 2: // validity (RMC)
            pnmea->gps_data_good = pnmea->term[0] == 'A';
            break;
        case GPS_SENTENCE_GGA + 6: // Fix data (GGA)
            pnmea->gps_data_good = pnmea->term[0] > '0';
            pnmea->new_quality_indicator = pnmea->term[0] - '0';
            break;
        case GPS_SENTENCE_VTG + 9: // validity (VTG) (we may not see this field)
            pnmea->gps_data_good = pnmea->term[0] != 'N';
            break;
        case GPS_SENTENCE_GGA + 7: // satellite count (GGA)
            pnmea->new_satellite_count = (unsigned char)sring_to_int((const char*)pnmea->term, (void*)0);
            break;
        case GPS_SENTENCE_GGA + 8: // HDOP (GGA)
            pnmea->new_hdop = (unsigned short)(string_to_float((const char*)pnmea->term, (void*)0)*100);
            break;

        // time and date
        //
        case GPS_SENTENCE_RMC + 1: // Time (RMC)
        case GPS_SENTENCE_GGA + 1: // Time (GGA)
            pnmea->new_time = (unsigned int)(string_to_float((const char*)pnmea->term, (void*)0)*1000);  /* ms */ 
            break;
        case GPS_SENTENCE_RMC + 9: // Date (GPRMC)
            pnmea->new_date = (unsigned int)sring_to_int((const char*)pnmea->term, (void*)0);
            break;

        // location
        //
        case GPS_SENTENCE_RMC + 3: // Latitude
        case GPS_SENTENCE_GGA + 2:
            pnmea->new_latitude = string_to_float((const char*)pnmea->term, (void*)0)/100.0;   //dd.mmmm
			pnmea->new_latitude = (float)( ((unsigned int)pnmea->new_latitude) + (pnmea->new_latitude-((unsigned int)pnmea->new_latitude))/60.0 );  //deg
            break;
        case GPS_SENTENCE_RMC + 4: // N/S
        case GPS_SENTENCE_GGA + 3:
            if (pnmea->term[0] == 'S')
                pnmea->new_latitude = -pnmea->new_latitude;
            break;
        case GPS_SENTENCE_RMC + 5: // Longitude
        case GPS_SENTENCE_GGA + 4:
            pnmea->new_longitude = string_to_float((const char*)pnmea->term, (void*)0)/100.0;  //dd.mmmmmm   
			pnmea->new_longitude = (float)(( (unsigned int)pnmea->new_longitude) + (pnmea->new_longitude-((unsigned int)pnmea->new_longitude))/60.0);  //deg
            break;
        case GPS_SENTENCE_RMC + 6: // E/W
        case GPS_SENTENCE_GGA + 5:
            if (pnmea->term[0] == 'W')
                pnmea->new_longitude = -pnmea->new_longitude;
            break;
        case GPS_SENTENCE_GGA + 9: // Altitude (GPGGA)
            pnmea->new_altitude = (int)(string_to_float((const char*)pnmea->term, (void*)0)*100);  /* cm */ 
            break;

        // course and speed
        //
        //case GPS_SENTENCE_RMC + 7: // Speed (GPRMC)
        //case GPS_SENTENCE_VTG + 5: // Speed (VTG)
        //    pnmea->new_speed = (_parse_decimal_100(pnmea->term) * 514) / 1000;       // knots-> m/sec, approximiates * 0.514
        //    break;
        
        case GPS_SENTENCE_VTG + 7: // Speed (VTG)
        	pnmea->new_speed = string_to_float((const char*)pnmea->term, (void*)0);       //km/h
           break;
        
        case GPS_SENTENCE_HDT + 1: // Course (HDT)
			 pnmea->new_gps_yaw = string_to_float((const char*)pnmea->term, (void*)0);
			 break;
        case GPS_SENTENCE_RMC + 8: // Course (GPRMC)
        case GPS_SENTENCE_VTG + 1: // Course (VTG)
            pnmea->new_course = string_to_float((const char*)pnmea->term, (void*)0);
            break;
        }
    }

    return 0;
}

/* 多次方 */
static int int_pow(int value, unsigned int count)
{
	int v = 1;
	
	while(count--)
		v = v*value;
	return v;
} 

static double double_pow(double value, unsigned int count)
{
	double v = 1;
	
	while(count--)
		v = v*value;
	return v;
}

/* 检查字符串是否为数字字符串，是返回1，否则返回0 */ 
static unsigned char string_check_digital(const char* pc)
{
	const char* p;
	if((void*)0==pc || '\0'==*pc) return 0;	
	
	p = pc;
	while('\0'!=*p)
	{
		if(!IS_DIGITAL(*p)) return 0;
		++p;
	}
	return 1;	
} 

/* 数字字符串处理函数,识别符号并且不检查溢出
*  如果是小数，则只返回整数部分,如果转换出错，返回0 
*  传入参数： 原始字符串， 转换成功标志 0-错误  1-正确 
*/
static int sring_to_int(const char* pstr, unsigned char* b)
{
	char nega = 1;    /* 负数为1， 正数为1 */ 
	const char* psrc = (void*)0;
	int value = 0;
	
	if( (void*)0==pstr ) 
	{
		*b = 0; 
		return 0;
	}
	
	if( !string_check_digital(pstr) ) 
	{
		*b = 0; 
		return 0;
	}
	
	psrc = pstr;
	if('-'==*psrc)  /* 符号位 */ 
	{
		nega = -1;
		++psrc;
	}	
	
	while('\0'!=*psrc)
	{
		if('.'==*psrc || '-'==*psrc ) return value;
		value = 10*value + CHAR_TO_DIGITAL(*psrc);	
		++psrc;
	} 
	if( -1==nega )
		value = -value;
		
	return value;
} 

/* 字符串转换成小数 */
static double string_to_float(const char* pstr, unsigned char* b) 
{
	char nega = 1;    /* 负数为1， 正数为1 */ 
	const char* psrc = (void*)0;
	double value = 0.0;
	double lvalue = 0.0; /* 小数点后面数字 */ 
	unsigned char dotflag = 0;  /* 小数点标记  1为之前出现过小数点 */ 
	unsigned char j = 0;
	
	if( (void*)0==pstr ) 
	{
		*b = 0; 
		return 0;
	}
	
	psrc = pstr;
	if( !string_check_digital(psrc) ) 
	{
		*b = 0; 
		return 0;
	}
	
	if('-'==*psrc)  /* 符号位 */ 
	{
		nega = -1;
		++psrc;
	}	
	
	while('\0'!=*psrc)
	{
		if('-'==*psrc ) return value;
		if('.'==*psrc ) 
		{
			dotflag = 1;
			++psrc;
			continue;
		}
		
		if(!dotflag) 
		{
			value = 10*value + CHAR_TO_DIGITAL(*psrc);	
		}else
		{
			++j;
			lvalue = lvalue + CHAR_TO_DIGITAL(*psrc)*double_pow(0.1, j);
		}	
		++psrc;
	} 
	if( -1==nega )
		value = -(value+lvalue);
	else value = value+lvalue;
		
	return value;
}

/* 浮点数转换为字符串，包括整数转换为字符串 
*  intgr指定整数位个数，dec指定小数位个数 
*  自动去除前面的0，小数点后面的0不会舍去 
*/ 
static unsigned char float_to_string(double value, char* pdest, unsigned int intgr, unsigned int dec)
{
	char* pstr = (void*)0;
	double fvalue = 0.0;
	char c = 0;
	int tvalue = 0;
	unsigned char zeroflag = 0;
	
	if( (void*)0==pdest || 0==intgr ) return 0;
	
	if(1==intgr) zeroflag = 1;
	
	pstr = pdest;
	if(value<-0.000000000000000001)
	{
		*pstr = '-';
		++pstr;
		value = -value;
	}
	
	tvalue = (int)value%int_pow(10,intgr);
	while(intgr)
	{
		c = DIGITAL_TO_CHAR(tvalue/int_pow(10,intgr-1));
		
		if( !zeroflag  && '0'==c )
		{
			tvalue = tvalue%int_pow(10,intgr-1);
			--intgr;
			continue;
		}
		
		zeroflag = 1;
		
		*pstr = c;
		tvalue = tvalue%int_pow(10,intgr-1);
		--intgr;
		++pstr;
	}
	
	if( !zeroflag ) *pstr++ = '0';
		
		
	/* 如果小数位数为0，则返回整数部分 */
	if(0==dec)
	{
		*pstr = '\0';
		return 1;		
	}
	
	*pstr++ = '.';	
	tvalue = (int)(value*int_pow(10,dec))%int_pow(10,dec);
	while(dec)
	{
		*pstr = DIGITAL_TO_CHAR(tvalue/int_pow(10,dec-1));
		tvalue = tvalue%int_pow(10,dec-1);
		--dec;
		++pstr;
	}
	*pstr = '\0';
	return 1;	
} 


/* 整数转换成字符串，内部其实调用了 float_to_string */
static unsigned char int_to_string(int value, char* pdest, unsigned int intgr) 
{
	return float_to_string((double)value, pdest, intgr,0);
}

/* 时间处理  传入参数为日月年和毫秒*/ 
static int make_date_time(gps_data* pdata, unsigned int date, unsigned int time)
{
	
	unsigned char year, mon, day, hour, min, sec;
    unsigned short int msec;
    unsigned int v,ret;
    char rmon;
    
    if((void*)0==pdata) return 0;

    year = date % 100;
    mon  = (date / 100) % 100;
    day  = date / 10000;
    
    pdata->date_time.year = year;
    pdata->date_time.month = mon;
    pdata->date_time.day = day;

    v = time;
    msec = v % 1000; v /= 1000;
    sec  = v % 100; v /= 100;
    min  = v % 100; v /= 100;
    hour = v % 100; v /= 100;

    rmon = mon - 2;
    if (0 >= rmon) {    
        rmon += 12;
        year -= 1;
    }

    // get time in seconds since unix epoch
    ret = (year/4) - (GPS_LEAPSECONDS_MILLIS / 1000UL) + 367*rmon/12 + day;
    ret += year*365 + 10501;
    ret = ret*24 + hour;
    ret = ret*60 + min;
    ret = ret*60 + sec;

    // convert to time since GPS epoch
    ret -= 272764785UL;

    // get GPS week and time
    pdata->date_time.week = ret / AP_SEC_PER_WEEK;
    
    return 1;
} 


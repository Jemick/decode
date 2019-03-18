/*------------------------------------------------------------------------------
* ublox.c : ublox receiver dependent functions
*
*          Copyright (C) 2007-2017 by T.TAKASU, All rights reserved.
*          Copyright (C) 2014 by T.SUZUKI, All rights reserved.
*
* reference :
*     [1] ublox-AG, GPS.G3-X-03002-D, ANTARIS Positioning Engine NMEA and UBX
*         Protocol Specification, Version 5.00, 2003
*     [2] ublox-AG, UBX-13003221-R03, u-blox M8 Receiver Description including
*         Protocol Specification V5, Dec 20, 2013
*     [3] ublox-AG, UBX-13003221-R07, u-blox M8 Receiver Description including
*         Protocol Specification V15.00-17.00, Nov 3, 2014
*     [4] ublox-AG, UBX-13003221-R09, u-blox 8 /u-blox M8 Receive]
 * r Description
*         including Protocol Specification V15.00-18.00, January, 2016
*
* version : $Revision: 1.2 $ $Date: 2008/07/14 00:05:05 $
* history : 2007/10/08 1.0  new
*           2008/06/16 1.1  separate common functions to rcvcmn.c
*           2009/04/01 1.2  add range check of prn number
*           2009/04/10 1.3  refactored
*           2009/09/25 1.4  add function gen_ubx()
*           2010/01/17 1.5  add time tag adjustment option -tadj sec
*           2010/10/31 1.6  fix bug on playback disabled for raw data (2.4.0_p9)
*           2011/05/27 1.7  add almanac decoding
*                           add -EPHALL option
*                           fix problem with ARM compiler
*           2013/02/23 1.8  fix memory access violation problem on arm
*                           change options -tadj to -TADJ, -invcp to -INVCP
*           2014/05/26 1.9  fix bug on message size of CFG-MSG
*                           fix bug on return code of decode_alm1()
*           2014/06/21 1.10 support message TRK-MEAS and TRK-SFRBX
*                           support message NAV-SOL and NAV-TIMEGPS to get time
*                           support message GFG-GNSS generation
*           2014/06/23 1.11 support message TRK-MEAS for beidou ephemeris
*           2014/08/11 1.12 fix bug on unable to read RXM-RAW
*                           fix problem on decoding glo ephemeris in TRK-SFRBX
*                           support message TRK-TRKD5
*           2014/08/31 1.13 suppress warning
*           2014/11/04 1.14 support message RXM-RAWX and RXM-SFRBX
*           2015/03/20 1.15 omit time adjustment for RXM-RAWX
*           2016/01/22 1.16 add time-tag in raw-message-type
*           2016/01/26 1.17 support galileo navigation data in RXM-SFRBX
*                           enable option -TADJ for RXM-RAWX
*           2016/05/25 1.18 fix bug on crc-buffer-overflow by decoding galileo
*                           navigation data
*           2016/07/04 1.19 add half-cycle vaild check for ubx-trk-meas
*           2016/07/29 1.20 support RXM-CFG-TMODE3 (0x06 0x71) for M8P
*                           crc24q() -> rtk_crc24q()
*                           check week number zero for ubx-rxm-raw and rawx
*           2016/08/20 1.21 add test of std-dev for carrier-phase valid
*           2016/08/26 1.22 add option -STD_SLIP to test slip by std-dev of cp
*                           fix on half-cyc valid for sbas in trkmeas
*           2017/04/11 1.23 (char *) -> (signed char *)
*                           fix bug on week handover in decode_trkmeas/trkd5()
*                           fix bug on prn for geo in decode_cnav()
*           2017/06/10 1.24 output half-cycle-subtracted flag
*-----------------------------------------------------------------------------*/
#include "StdAfx.h"

#include "rtklib.h"

/* adjust gps week number ------------------------------------------------------
* adjust gps week number using cpu time
* args   : int   week       I   not-adjusted gps week number
* return : adjusted gps week number
*-----------------------------------------------------------------------------*/
extern void trace   (int level, const char *format, ...) {}
extern void traceb  (int level, const unsigned char *p, int n) {}

extern int adjgpsweek(int week)
{
    int w;
    
    (void)time2gpst(utc2gpst(timeget()),&w);
    if (w<1560) w=1560; /* use 2009/12/1 if time is earlier than 2009/12/1 */
    return week+(w-week+512)/1024*1024;
}

#define CLIGHT      299792458.0         /* speed of light (m/s) */

#define FREQ1       1.57542E9           /* L1/E1  frequency (Hz) */
#define FREQ2       1.22760E9           /* L2     frequency (Hz) */
#define FREQ5       1.17645E9           /* L5/E5a frequency (Hz) */
#define FREQ6       1.27875E9           /* E6/LEX frequency (Hz) */
#define FREQ7       1.20714E9           /* E5b    frequency (Hz) */
#define FREQ8       1.191795E9          /* E5a+b  frequency (Hz) */
#define FREQ9       2.492028E9          /* S      frequency (Hz) */
#define FREQ1_GLO   1.60200E9           /* GLONASS G1 base frequency (Hz) */
#define DFRQ1_GLO   0.56250E6           /* GLONASS G1 bias frequency (Hz/n) */
#define FREQ2_GLO   1.24600E9           /* GLONASS G2 base frequency (Hz) */
#define DFRQ2_GLO   0.43750E6           /* GLONASS G2 bias frequency (Hz/n) */
#define FREQ3_GLO   1.202025E9          /* GLONASS G3 frequency (Hz) */
#define FREQ1_CMP   1.561098E9          /* BeiDou B1 frequency (Hz) */
#define FREQ2_CMP   1.20714E9           /* BeiDou B2 frequency (Hz) */
#define FREQ3_CMP   1.26852E9           /* BeiDou B3 frequency (Hz) */

#undef main

const double lam_carr[MAXFREQ]={ /* carrier wave length (m) */
	CLIGHT/FREQ1,CLIGHT/FREQ2,CLIGHT/FREQ5,CLIGHT/FREQ6,CLIGHT/FREQ7,
	CLIGHT/FREQ8,CLIGHT/FREQ9
};

static const unsigned int tbl_CRC24Q[]={
	0x000000,0x864CFB,0x8AD50D,0x0C99F6,0x93E6E1,0x15AA1A,0x1933EC,0x9F7F17,
	0xA18139,0x27CDC2,0x2B5434,0xAD18CF,0x3267D8,0xB42B23,0xB8B2D5,0x3EFE2E,
	0xC54E89,0x430272,0x4F9B84,0xC9D77F,0x56A868,0xD0E493,0xDC7D65,0x5A319E,
	0x64CFB0,0xE2834B,0xEE1ABD,0x685646,0xF72951,0x7165AA,0x7DFC5C,0xFBB0A7,
	0x0CD1E9,0x8A9D12,0x8604E4,0x00481F,0x9F3708,0x197BF3,0x15E205,0x93AEFE,
	0xAD50D0,0x2B1C2B,0x2785DD,0xA1C926,0x3EB631,0xB8FACA,0xB4633C,0x322FC7,
	0xC99F60,0x4FD39B,0x434A6D,0xC50696,0x5A7981,0xDC357A,0xD0AC8C,0x56E077,
	0x681E59,0xEE52A2,0xE2CB54,0x6487AF,0xFBF8B8,0x7DB443,0x712DB5,0xF7614E,
	0x19A3D2,0x9FEF29,0x9376DF,0x153A24,0x8A4533,0x0C09C8,0x00903E,0x86DCC5,
	0xB822EB,0x3E6E10,0x32F7E6,0xB4BB1D,0x2BC40A,0xAD88F1,0xA11107,0x275DFC,
	0xDCED5B,0x5AA1A0,0x563856,0xD074AD,0x4F0BBA,0xC94741,0xC5DEB7,0x43924C,
	0x7D6C62,0xFB2099,0xF7B96F,0x71F594,0xEE8A83,0x68C678,0x645F8E,0xE21375,
	0x15723B,0x933EC0,0x9FA736,0x19EBCD,0x8694DA,0x00D821,0x0C41D7,0x8A0D2C,
	0xB4F302,0x32BFF9,0x3E260F,0xB86AF4,0x2715E3,0xA15918,0xADC0EE,0x2B8C15,
	0xD03CB2,0x567049,0x5AE9BF,0xDCA544,0x43DA53,0xC596A8,0xC90F5E,0x4F43A5,
	0x71BD8B,0xF7F170,0xFB6886,0x7D247D,0xE25B6A,0x641791,0x688E67,0xEEC29C,
	0x3347A4,0xB50B5F,0xB992A9,0x3FDE52,0xA0A145,0x26EDBE,0x2A7448,0xAC38B3,
	0x92C69D,0x148A66,0x181390,0x9E5F6B,0x01207C,0x876C87,0x8BF571,0x0DB98A,
	0xF6092D,0x7045D6,0x7CDC20,0xFA90DB,0x65EFCC,0xE3A337,0xEF3AC1,0x69763A,
	0x578814,0xD1C4EF,0xDD5D19,0x5B11E2,0xC46EF5,0x42220E,0x4EBBF8,0xC8F703,
	0x3F964D,0xB9DAB6,0xB54340,0x330FBB,0xAC70AC,0x2A3C57,0x26A5A1,0xA0E95A,
	0x9E1774,0x185B8F,0x14C279,0x928E82,0x0DF195,0x8BBD6E,0x872498,0x016863,
	0xFAD8C4,0x7C943F,0x700DC9,0xF64132,0x693E25,0xEF72DE,0xE3EB28,0x65A7D3,
	0x5B59FD,0xDD1506,0xD18CF0,0x57C00B,0xC8BF1C,0x4EF3E7,0x426A11,0xC426EA,
	0x2AE476,0xACA88D,0xA0317B,0x267D80,0xB90297,0x3F4E6C,0x33D79A,0xB59B61,
	0x8B654F,0x0D29B4,0x01B042,0x87FCB9,0x1883AE,0x9ECF55,0x9256A3,0x141A58,
	0xEFAAFF,0x69E604,0x657FF2,0xE33309,0x7C4C1E,0xFA00E5,0xF69913,0x70D5E8,
	0x4E2BC6,0xC8673D,0xC4FECB,0x42B230,0xDDCD27,0x5B81DC,0x57182A,0xD154D1,
	0x26359F,0xA07964,0xACE092,0x2AAC69,0xB5D37E,0x339F85,0x3F0673,0xB94A88,
	0x87B4A6,0x01F85D,0x0D61AB,0x8B2D50,0x145247,0x921EBC,0x9E874A,0x18CBB1,
	0xE37B16,0x6537ED,0x69AE1B,0xEFE2E0,0x709DF7,0xF6D10C,0xFA48FA,0x7C0401,
	0x42FA2F,0xC4B6D4,0xC82F22,0x4E63D9,0xD11CCE,0x575035,0x5BC9C3,0xDD8538
};
/* crc-24q parity --------------------------------------------------------------
* compute crc-24q parity for sbas, rtcm3
* args   : unsigned char *buff I data
*          int    len    I      data length (bytes)
* return : crc-24Q parity
* notes  : see reference [2] A.4.3.3 Parity
*-----------------------------------------------------------------------------*/
extern unsigned int rtk_crc24q(const unsigned char *buff, int len)
{
    unsigned int crc=0;
    int i;
    
    trace(4,"rtk_crc24q: len=%d\n",len);
    
    for (i=0;i<len;i++) crc=((crc<<8)&0xFFFFFF)^tbl_CRC24Q[(crc>>16)^buff[i]];
    return crc;
}

#define POLYCRC32   0xEDB88320u /* CRC32 polynomial */
#define POLYCRC24Q  0x1864CFBu  /* CRC24Q polynomial */

static const double gpst0[]={1980,1, 6,0,0,0}; /* gps time reference */
static const double gst0 []={1999,8,22,0,0,0}; /* galileo system time reference */
static const double bdt0 []={2006,1, 1,0,0,0}; /* beidou time reference */

static double leaps[MAXLEAPS+1][7]={ /* leap seconds (y,m,d,h,m,s,utc-gpst) */
	{2017,1,1,0,0,0,-18},
	{2015,7,1,0,0,0,-17},
	{2012,7,1,0,0,0,-16},
	{2009,1,1,0,0,0,-15},
	{2006,1,1,0,0,0,-14},
	{1999,1,1,0,0,0,-13},
	{1997,7,1,0,0,0,-12},
	{1996,1,1,0,0,0,-11},
	{1994,7,1,0,0,0,-10},
	{1993,7,1,0,0,0, -9},
	{1992,7,1,0,0,0, -8},
	{1991,1,1,0,0,0, -7},
	{1990,1,1,0,0,0, -6},
	{1988,1,1,0,0,0, -5},
	{1985,7,1,0,0,0, -4},
	{1983,7,1,0,0,0, -3},
	{1982,7,1,0,0,0, -2},
	{1981,7,1,0,0,0, -1},
	{0}
};

/* gpstime to utc --------------------------------------------------------------
* convert gpstime to utc considering leap seconds
* args   : gtime_t t        I   time expressed in gpstime
* return : time expressed in utc
* notes  : ignore slight time offset under 100 ns
*-----------------------------------------------------------------------------*/
extern gtime_t gpst2utc(gtime_t t)
{
    gtime_t tu;
    int i;
    
    for (i=0;leaps[i][0]>0;i++) {
        tu=timeadd(t,leaps[i][6]);
        if (timediff(tu,epoch2time(leaps[i]))>=0.0) return tu;
    }
    return t;
}
/* utc to gpstime --------------------------------------------------------------
* convert utc to gpstime considering leap seconds
* args   : gtime_t t        I   time expressed in utc
* return : time expressed in gpstime
* notes  : ignore slight time offset under 100 ns
*-----------------------------------------------------------------------------*/
extern gtime_t utc2gpst(gtime_t t)
{
    int i;
    
    for (i=0;leaps[i][0]>0;i++) {
        if (timediff(t,epoch2time(leaps[i]))>=0.0) return timeadd(t,-leaps[i][6]);
    }
    return t;
}
/* gpstime to bdt --------------------------------------------------------------
* convert gpstime to bdt (beidou navigation satellite system time)
* args   : gtime_t t        I   time expressed in gpstime
* return : time expressed in bdt
* notes  : ref [8] 3.3, 2006/1/1 00:00 BDT = 2006/1/1 00:00 UTC
*          no leap seconds in BDT
*          ignore slight time offset under 100 ns
*-----------------------------------------------------------------------------*/
extern gtime_t gpst2bdt(gtime_t t)
{
    return timeadd(t,-14.0);
}

/* bdt to gpstime --------------------------------------------------------------
* convert bdt (beidou navigation satellite system time) to gpstime
* args   : gtime_t t        I   time expressed in bdt
* return : time expressed in gpstime
* notes  : see gpst2bdt()
*-----------------------------------------------------------------------------*/
extern gtime_t bdt2gpst(gtime_t t)
{
    return timeadd(t,14.0);
}
/* time to day and sec -------------------------------------------------------*/
static double time2sec(gtime_t time, gtime_t *day)
{
    double ep[6],sec;
    time2epoch(time,ep);
    sec=ep[3]*3600.0+ep[4]*60.0+ep[5];
    ep[3]=ep[4]=ep[5]=0.0;
    *day=epoch2time(ep);
    return sec;
}
/* utc to gmst -----------------------------------------------------------------
* convert utc to gmst (Greenwich mean sidereal time)
* args   : gtime_t t        I   time expressed in utc
*          double ut1_utc   I   UT1-UTC (s)
* return : gmst (rad)
*-----------------------------------------------------------------------------*/
extern double utc2gmst(gtime_t t, double ut1_utc)
{
    const double ep2000[]={2000,1,1,12,0,0};
    gtime_t tut,tut0;
    double ut,t1,t2,t3,gmst0,gmst;
    
    tut=timeadd(t,ut1_utc);
    ut=time2sec(tut,&tut0);
    t1=timediff(tut0,epoch2time(ep2000))/86400.0/36525.0;
    t2=t1*t1; t3=t2*t1;
    gmst0=24110.54841+8640184.812866*t1+0.093104*t2-6.2E-6*t3;
    gmst=gmst0+1.002737909350795*ut;
    
    return fmod(gmst,86400.0)*PI/43200.0; /* 0 <= gmst <= 2*PI */
}
/* time to string --------------------------------------------------------------
* convert gtime_t struct to string
* args   : gtime_t t        I   gtime_t struct
*          char   *s        O   string ("yyyy/mm/dd hh:mm:ss.ssss")
*          int    n         I   number of decimals
* return : none
*-----------------------------------------------------------------------------*/
extern void time2str(gtime_t t, char *s, int n)
{
    double ep[6];
    
    if (n<0) n=0; else if (n>12) n=12;
    if (1.0-t.sec<0.5/pow(10.0,n)) {t.time++; t.sec=0.0;};
    time2epoch(t,ep);
    sprintf(s,"%04.0f/%02.0f/%02.0f %02.0f:%02.0f:%0*.*f",ep[0],ep[1],ep[2],
            ep[3],ep[4],n<=0?2:n+3,n<=0?0:n,ep[5]);
}

/* string to time --------------------------------------------------------------
* convert substring in string to gtime_t struct
* args   : char   *s        I   string ("... yyyy mm dd hh mm ss ...")
*          int    i,n       I   substring position and width
*          gtime_t *t       O   gtime_t struct
* return : status (0:ok,0>:error)
*-----------------------------------------------------------------------------*/
extern int str2time(const char *s, int i, int n, gtime_t *t)
{
    double ep[6];
    char str[256],*p=str;
    
    if (i<0||(int)strlen(s)<i||(int)sizeof(str)-1<i) return -1;
    for (s+=i;*s&&--n>=0;) *p++=*s++;
    *p='\0';
    if (sscanf(str,"%lf %lf %lf %lf %lf %lf",ep,ep+1,ep+2,ep+3,ep+4,ep+5)<6)
        return -1;
    if (ep[0]<100.0) ep[0]+=ep[0]<80.0?2000.0:1900.0;
    *t=epoch2time(ep);
    return 0;
}
/* get time string -------------------------------------------------------------
* get time string
* args   : gtime_t t        I   gtime_t struct
*          int    n         I   number of decimals
* return : time string
* notes  : not reentrant, do not use multiple in a function
*-----------------------------------------------------------------------------*/
extern char *time_str(gtime_t t, int n)
{
    static char buff[64];
    time2str(t,buff,n);
    return buff;
}
/* convert calendar day/time to time -------------------------------------------
* convert calendar day/time to gtime_t struct
* args   : double *ep       I   day/time {year,month,day,hour,min,sec}
* return : gtime_t struct
* notes  : proper in 1970-2037 or 1970-2099 (64bit time_t)
*-----------------------------------------------------------------------------*/
extern gtime_t epoch2time(const double *ep)
{
    const int doy[]={1,32,60,91,121,152,182,213,244,274,305,335};
    gtime_t time={0};
    int days,sec,year=(int)ep[0],mon=(int)ep[1],day=(int)ep[2];
    
    if (year<1970||2099<year||mon<1||12<mon) return time;
    
    /* leap year if year%4==0 in 1901-2099 */
    days=(year-1970)*365+(year-1969)/4+doy[mon-1]+day-2+(year%4==0&&mon>=3?1:0);
    sec=(int)floor(ep[5]);
    time.time=(time_t)days*86400+(int)ep[3]*3600+(int)ep[4]*60+sec;
    time.sec=ep[5]-sec;
    return time;
}
/* time to calendar day/time ---------------------------------------------------
* convert gtime_t struct to calendar day/time
* args   : gtime_t t        I   gtime_t struct
*          double *ep       O   day/time {year,month,day,hour,min,sec}
* return : none
* notes  : proper in 1970-2037 or 1970-2099 (64bit time_t)
*-----------------------------------------------------------------------------*/
extern void time2epoch(gtime_t t, double *ep)
{
    const int mday[]={ /* # of days in a month */
        31,28,31,30,31,30,31,31,30,31,30,31,31,28,31,30,31,30,31,31,30,31,30,31,
        31,29,31,30,31,30,31,31,30,31,30,31,31,28,31,30,31,30,31,31,30,31,30,31
    };
    int days,sec,mon,day;
    
    /* leap year if year%4==0 in 1901-2099 */
    days=(int)(t.time/86400);
    sec=(int)(t.time-(time_t)days*86400);
    for (day=days%1461,mon=0;mon<48;mon++) {
        if (day>=mday[mon]) day-=mday[mon]; else break;
    }
    ep[0]=1970+days/1461*4+mon/12; ep[1]=mon%12+1; ep[2]=day+1;
    ep[3]=sec/3600; ep[4]=sec%3600/60; ep[5]=sec%60+t.sec;
}
/* gps time to time ------------------------------------------------------------
* convert week and tow in gps time to gtime_t struct
* args   : int    week      I   week number in gps time
*          double sec       I   time of week in gps time (s)
* return : gtime_t struct
*-----------------------------------------------------------------------------*/
extern gtime_t gpst2time(int week, double sec)
{
    gtime_t t=epoch2time(gpst0);
    
    if (sec<-1E9||1E9<sec) sec=0.0;
    t.time+=(time_t)86400*7*week+(int)sec;
    t.sec=sec-(int)sec;
    return t;
}
/* time to gps time ------------------------------------------------------------
* convert gtime_t struct to week and tow in gps time
* args   : gtime_t t        I   gtime_t struct
*          int    *week     IO  week number in gps time (NULL: no output)
* return : time of week in gps time (s)
*-----------------------------------------------------------------------------*/
extern double time2gpst(gtime_t t, int *week)
{
    gtime_t t0=epoch2time(gpst0);
    time_t sec=t.time-t0.time;
    int w=(int)(sec/(86400*7));
    
    if (week) *week=w;
    return (double)(sec-(double)w*86400*7)+t.sec;
}
/* galileo system time to time -------------------------------------------------
* convert week and tow in galileo system time (gst) to gtime_t struct
* args   : int    week      I   week number in gst
*          double sec       I   time of week in gst (s)
* return : gtime_t struct
*-----------------------------------------------------------------------------*/
extern gtime_t gst2time(int week, double sec)
{
    gtime_t t=epoch2time(gst0);
    
    if (sec<-1E9||1E9<sec) sec=0.0;
    t.time+=(time_t)86400*7*week+(int)sec;
    t.sec=sec-(int)sec;
    return t;
}
/* time to galileo system time -------------------------------------------------
* convert gtime_t struct to week and tow in galileo system time (gst)
* args   : gtime_t t        I   gtime_t struct
*          int    *week     IO  week number in gst (NULL: no output)
* return : time of week in gst (s)
*-----------------------------------------------------------------------------*/
extern double time2gst(gtime_t t, int *week)
{
    gtime_t t0=epoch2time(gst0);
    time_t sec=t.time-t0.time;
    int w=(int)(sec/(86400*7));
    
    if (week) *week=w;
    return (double)(sec-(double)w*86400*7)+t.sec;
}
/* beidou time (bdt) to time ---------------------------------------------------
* convert week and tow in beidou time (bdt) to gtime_t struct
* args   : int    week      I   week number in bdt
*          double sec       I   time of week in bdt (s)
* return : gtime_t struct
*-----------------------------------------------------------------------------*/
extern gtime_t bdt2time(int week, double sec)
{
    gtime_t t=epoch2time(bdt0);
    
    if (sec<-1E9||1E9<sec) sec=0.0;
    t.time+=(time_t)86400*7*week+(int)sec;
    t.sec=sec-(int)sec;
    return t;
}
/* time to beidouo time (bdt) --------------------------------------------------
* convert gtime_t struct to week and tow in beidou time (bdt)
* args   : gtime_t t        I   gtime_t struct
*          int    *week     IO  week number in bdt (NULL: no output)
* return : time of week in bdt (s)
*-----------------------------------------------------------------------------*/
extern double time2bdt(gtime_t t, int *week)
{
    gtime_t t0=epoch2time(bdt0);
    time_t sec=t.time-t0.time;
    int w=(int)(sec/(86400*7));
    
    if (week) *week=w;
    return (double)(sec-(double)w*86400*7)+t.sec;
}
/* add time --------------------------------------------------------------------
* add time to gtime_t struct
* args   : gtime_t t        I   gtime_t struct
*          double sec       I   time to add (s)
* return : gtime_t struct (t+sec)
*-----------------------------------------------------------------------------*/
extern gtime_t timeadd(gtime_t t, double sec)
{
    double tt;
    
    t.sec+=sec; tt=floor(t.sec); t.time+=(int)tt; t.sec-=tt;
    return t;
}
/* time difference -------------------------------------------------------------
* difference between gtime_t structs
* args   : gtime_t t1,t2    I   gtime_t structs
* return : time difference (t1-t2) (s)
*-----------------------------------------------------------------------------*/
extern double timediff(gtime_t t1, gtime_t t2)
{
    return difftime(t1.time,t2.time)+t1.sec-t2.sec;
}
/* get current time in utc -----------------------------------------------------
* get current time in utc
* args   : none
* return : current time in utc
*-----------------------------------------------------------------------------*/
static double timeoffset_=0.0;        /* time offset (s) */

extern gtime_t timeget(void)
{
    gtime_t time;
    double ep[6]={0};
#ifdef WIN32
    SYSTEMTIME ts;
    
    GetSystemTime(&ts); /* utc */
    ep[0]=ts.wYear; ep[1]=ts.wMonth;  ep[2]=ts.wDay;
    ep[3]=ts.wHour; ep[4]=ts.wMinute; ep[5]=ts.wSecond+ts.wMilliseconds*1E-3;
#else
    struct timeval tv;
    struct tm *tt;
    
    if (!gettimeofday(&tv,NULL)&&(tt=gmtime(&tv.tv_sec))) {
        ep[0]=tt->tm_year+1900; ep[1]=tt->tm_mon+1; ep[2]=tt->tm_mday;
        ep[3]=tt->tm_hour; ep[4]=tt->tm_min; ep[5]=tt->tm_sec+tv.tv_usec*1E-6;
    }
#endif
    time=epoch2time(ep);
    
#ifdef CPUTIME_IN_GPST /* cputime operated in gpst */
    time=gpst2utc(time);
#endif
    return timeadd(time,timeoffset_);
}
/* set current time in utc -----------------------------------------------------
* set current time in utc
* args   : gtime_t          I   current time in utc
* return : none
* notes  : just set time offset between cpu time and current time
*          the time offset is reflected to only timeget()
*          not reentrant
*-----------------------------------------------------------------------------*/
extern void timeset(gtime_t t)
{
    timeoffset_+=timediff(t,timeget());
}


extern unsigned int getbitu(const unsigned char *buff, int pos, int len)
{
	unsigned int bits=0;
	int i;
	for (i=pos;i<pos+len;i++) bits=(bits<<1)+((buff[i/8]>>(7-i%8))&1u);
	return bits;
}
extern int getbits(const unsigned char *buff, int pos, int len)
{
	unsigned int bits=getbitu(buff,pos,len);
	if (len<=0||32<=len||!(bits&(1u<<(len-1)))) return (int)bits;
	return (int)(bits|(~0u<<len)); /* extend sign */
}

/* set unsigned/signed bits ----------------------------------------------------
* set unsigned/signed bits to byte data
* args   : unsigned char *buff IO byte data
*          int    pos    I      bit position from start of data (bits)
*          int    len    I      bit length (bits) (len<=32)
*         (unsigned) int I      unsigned/signed data
* return : none
*-----------------------------------------------------------------------------*/
extern void setbitu(unsigned char *buff, int pos, int len, unsigned int data)
{
    unsigned int mask=1u<<(len-1);
    int i;
    if (len<=0||32<len) return;
    for (i=pos;i<pos+len;i++,mask>>=1) {
        if (data&mask) buff[i/8]|=1u<<(7-i%8); else buff[i/8]&=~(1u<<(7-i%8));
    }
}
extern void setbits(unsigned char *buff, int pos, int len, int data)
{
    if (data<0) data|=1<<(len-1); else data&=~(1<<(len-1)); /* set sign bit */
    setbitu(buff,pos,len,(unsigned int)data);
}

#define P2_34       5.820766091346740E-11 /* 2^-34 */
#define P2_46       1.421085471520200E-14 /* 2^-46 */
#define P2_59       1.734723475976810E-18 /* 2^-59 */
#define P2_66       1.355252715606881E-20 /* 2^-66 for BeiDou ephemeris */

/* get two component bits ----------------------------------------------------*/
static unsigned int getbitu2(const unsigned char *buff, int p1, int l1, int p2,
                             int l2)
{
    return (getbitu(buff,p1,l1)<<l2)+getbitu(buff,p2,l2);
}
static int getbits2(const unsigned char *buff, int p1, int l1, int p2, int l2)
{
    if (getbitu(buff,p1,1))
        return (int)((getbits(buff,p1,l1)<<l2)+getbitu(buff,p2,l2));
    else
        return (int)getbitu2(buff,p1,l1,p2,l2);
}
/* get three component bits --------------------------------------------------*/
static unsigned int getbitu3(const unsigned char *buff, int p1, int l1, int p2,
                             int l2, int p3, int l3)
{
    return (getbitu(buff,p1,l1)<<(l2+l3))+(getbitu(buff,p2,l2)<<l3)+
            getbitu(buff,p3,l3);
}
static int getbits3(const unsigned char *buff, int p1, int l1, int p2, int l2,
                    int p3, int l3)
{
    if (getbitu(buff,p1,1))
        return (int)((getbits(buff,p1,l1)<<(l2+l3))+
                   (getbitu(buff,p2,l2)<<l3)+getbitu(buff,p3,l3));
    else
        return (int)getbitu3(buff,p1,l1,p2,l2,p3,l3);
}
/* merge two components ------------------------------------------------------*/
static unsigned int merge_two_u(unsigned int a, unsigned int b, int n)
{
    return (a<<n)+b;
}
static int merge_two_s(int a, unsigned int b, int n)
{
    return (int)((a<<n)+b);
}
/* get sign-magnitude bits ---------------------------------------------------*/
static double getbitg(const unsigned char *buff, int pos, int len)
{
    double value=getbitu(buff,pos+1,len-1);
    return getbitu(buff,pos,1)?-value:value;
}


/* satellite system+prn/slot number to satellite number ------------------------
* convert satellite system+prn/slot number to satellite number
* args   : int    sys       I   satellite system (SYS_GPS,SYS_GLO,...)
*          int    prn       I   satellite prn/slot number
* return : satellite number (0:error)
*-----------------------------------------------------------------------------*/
extern int satno(int sys, int prn)
{
    if (prn<=0) return 0;
    switch (sys) {
        case SYS_GPS:
            if (prn<MINPRNGPS||MAXPRNGPS<prn) return 0;
            return prn-MINPRNGPS+1;
        case SYS_GLO:
            if (prn<MINPRNGLO||MAXPRNGLO<prn) return 0;
            return NSATGPS+prn-MINPRNGLO+1;
        case SYS_GAL:
            if (prn<MINPRNGAL||MAXPRNGAL<prn) return 0;
            return NSATGPS+NSATGLO+prn-MINPRNGAL+1;
        case SYS_QZS:
            if (prn<MINPRNQZS||MAXPRNQZS<prn) return 0;
            return NSATGPS+NSATGLO+NSATGAL+prn-MINPRNQZS+1;
        case SYS_CMP:
            if (prn<MINPRNCMP||MAXPRNCMP<prn) return 0;
            return NSATGPS+NSATGLO+NSATGAL+NSATQZS+prn-MINPRNCMP+1;
        case SYS_IRN:
            if (prn<MINPRNIRN||MAXPRNIRN<prn) return 0;
            return NSATGPS+NSATGLO+NSATGAL+NSATQZS+NSATCMP+prn-MINPRNIRN+1;
        case SYS_LEO:
            if (prn<MINPRNLEO||MAXPRNLEO<prn) return 0;
            return NSATGPS+NSATGLO+NSATGAL+NSATQZS+NSATCMP+NSATIRN+
                   prn-MINPRNLEO+1;
        case SYS_SBS:
            if (prn<MINPRNSBS||MAXPRNSBS<prn) return 0;
            return NSATGPS+NSATGLO+NSATGAL+NSATQZS+NSATCMP+NSATIRN+NSATLEO+
                   prn-MINPRNSBS+1;
    }
    return 0;
}
/* satellite number to satellite system ----------------------------------------
* convert satellite number to satellite system
* args   : int    sat       I   satellite number (1-MAXSAT)
*          int    *prn      IO  satellite prn/slot number (NULL: no output)
* return : satellite system (SYS_GPS,SYS_GLO,...)
*-----------------------------------------------------------------------------*/
extern int satsys(int sat, int *prn)
{
    int sys=SYS_NONE;
    if (sat<=0||MAXSAT<sat) sat=0;
    else if (sat<=NSATGPS) {
        sys=SYS_GPS; sat+=MINPRNGPS-1;
    }
    else if ((sat-=NSATGPS)<=NSATGLO) {
        sys=SYS_GLO; sat+=MINPRNGLO-1;
    }
    else if ((sat-=NSATGLO)<=NSATGAL) {
        sys=SYS_GAL; sat+=MINPRNGAL-1;
    }
    else if ((sat-=NSATGAL)<=NSATQZS) {
        sys=SYS_QZS; sat+=MINPRNQZS-1; 
    }
    else if ((sat-=NSATQZS)<=NSATCMP) {
        sys=SYS_CMP; sat+=MINPRNCMP-1; 
    }
    else if ((sat-=NSATCMP)<=NSATIRN) {
        sys=SYS_IRN; sat+=MINPRNIRN-1; 
    }
    else if ((sat-=NSATIRN)<=NSATLEO) {
        sys=SYS_LEO; sat+=MINPRNLEO-1; 
    }
    else if ((sat-=NSATLEO)<=NSATSBS) {
        sys=SYS_SBS; sat+=MINPRNSBS-1; 
    }
    else sat=0;
    if (prn) *prn=sat;
    return sys;
}
/* satellite id to satellite number --------------------------------------------
* convert satellite id to satellite number
* args   : char   *id       I   satellite id (nn,Gnn,Rnn,Enn,Jnn,Cnn,Inn or Snn)
* return : satellite number (0: error)
* notes  : 120-142 and 193-199 are also recognized as sbas and qzss
*-----------------------------------------------------------------------------*/
extern int satid2no(const char *id)
{
    int sys,prn;
    char code;
    
    if (sscanf(id,"%d",&prn)==1) {
        if      (MINPRNGPS<=prn&&prn<=MAXPRNGPS) sys=SYS_GPS;
        else if (MINPRNSBS<=prn&&prn<=MAXPRNSBS) sys=SYS_SBS;
        else if (MINPRNQZS<=prn&&prn<=MAXPRNQZS) sys=SYS_QZS;
        else return 0;
        return satno(sys,prn);
    }
    if (sscanf(id,"%c%d",&code,&prn)<2) return 0;
    
    switch (code) {
        case 'G': sys=SYS_GPS; prn+=MINPRNGPS-1; break;
        case 'R': sys=SYS_GLO; prn+=MINPRNGLO-1; break;
        case 'E': sys=SYS_GAL; prn+=MINPRNGAL-1; break;
        case 'J': sys=SYS_QZS; prn+=MINPRNQZS-1; break;
        case 'C': sys=SYS_CMP; prn+=MINPRNCMP-1; break;
        case 'I': sys=SYS_IRN; prn+=MINPRNIRN-1; break;
        case 'L': sys=SYS_LEO; prn+=MINPRNLEO-1; break;
        case 'S': sys=SYS_SBS; prn+=100; break;
        default: return 0;
    }
    return satno(sys,prn);
}
/* satellite number to satellite id --------------------------------------------
* convert satellite number to satellite id
* args   : int    sat       I   satellite number
*          char   *id       O   satellite id (Gnn,Rnn,Enn,Jnn,Cnn,Inn or nnn)
* return : none
*-----------------------------------------------------------------------------*/
extern void satno2id(int sat, char *id)
{
    int prn;
    switch (satsys(sat,&prn)) {
        case SYS_GPS: sprintf(id,"G%02d",prn-MINPRNGPS+1); return;
        case SYS_GLO: sprintf(id,"R%02d",prn-MINPRNGLO+1); return;
        case SYS_GAL: sprintf(id,"E%02d",prn-MINPRNGAL+1); return;
        case SYS_QZS: sprintf(id,"J%02d",prn-MINPRNQZS+1); return;
        case SYS_CMP: sprintf(id,"C%02d",prn-MINPRNCMP+1); return;
        case SYS_IRN: sprintf(id,"I%02d",prn-MINPRNIRN+1); return;
        case SYS_LEO: sprintf(id,"L%02d",prn-MINPRNLEO+1); return;
        case SYS_SBS: sprintf(id,"%03d" ,prn); return;
    }
    strcpy(id,"");
}


/* decode Galileo I/NAV ephemeris ----------------------------------------------
* decode Galileo I/NAV (ref [5] 4.3)
* args   : unsigned char *buff I Galileo I/NAV subframe bits
*                                  buff[ 0-15]: I/NAV word type 0 (128 bit)
*                                  buff[16-31]: I/NAV word type 1
*                                  buff[32-47]: I/NAV word type 2
*                                  buff[48-63]: I/NAV word type 3
*                                  buff[64-79]: I/NAV word type 4
*                                  buff[80-95]: I/NAV word type 5
*          eph_t    *eph    IO  ephemeris structure
* return : status (1:ok,0:error)
*-----------------------------------------------------------------------------*/
extern int decode_gal_inav(const unsigned char *buff, eph_t *eph)
{
    double tow,toc,tt,sqrtA;
    int i,time_f,week,svid,e5b_hs,e1b_hs,e5b_dvs,e1b_dvs,type[6],iod_nav[4];
    
    i=0; /* word type 0 */
    type[0]    =getbitu(buff,i, 6);              i+= 6;
    time_f     =getbitu(buff,i, 2);              i+= 2+88;
    week       =getbitu(buff,i,12);              i+=12; /* gst-week */
    tow        =getbitu(buff,i,20);
    
    i=128; /* word type 1 */
    type[1]    =getbitu(buff,i, 6);              i+= 6;
    iod_nav[0] =getbitu(buff,i,10);              i+=10;
    eph->toes  =getbitu(buff,i,14)*60.0;         i+=14;
    eph->M0    =getbits(buff,i,32)*P2_31*SC2RAD; i+=32;
    eph->e     =getbitu(buff,i,32)*P2_33;        i+=32;
    sqrtA      =getbitu(buff,i,32)*P2_19;
    
    i=128*2; /* word type 2 */
    type[2]    =getbitu(buff,i, 6);              i+= 6;
    iod_nav[1] =getbitu(buff,i,10);              i+=10;
    eph->OMG0  =getbits(buff,i,32)*P2_31*SC2RAD; i+=32;
    eph->i0    =getbits(buff,i,32)*P2_31*SC2RAD; i+=32;
    eph->omg   =getbits(buff,i,32)*P2_31*SC2RAD; i+=32;
    eph->idot  =getbits(buff,i,14)*P2_43*SC2RAD;
    
    i=128*3; /* word type 3 */
    type[3]    =getbitu(buff,i, 6);              i+= 6;
    iod_nav[2] =getbitu(buff,i,10);              i+=10;
    eph->OMGd  =getbits(buff,i,24)*P2_43*SC2RAD; i+=24;
    eph->deln  =getbits(buff,i,16)*P2_43*SC2RAD; i+=16;
    eph->cuc   =getbits(buff,i,16)*P2_29;        i+=16;
    eph->cus   =getbits(buff,i,16)*P2_29;        i+=16;
    eph->crc   =getbits(buff,i,16)*P2_5;         i+=16;
    eph->crs   =getbits(buff,i,16)*P2_5;         i+=16;
    eph->sva   =getbitu(buff,i, 8);
    
    i=128*4; /* word type 4 */
    type[4]    =getbitu(buff,i, 6);              i+= 6;
    iod_nav[3] =getbitu(buff,i,10);              i+=10;
    svid       =getbitu(buff,i, 6);              i+= 6;
    eph->cic   =getbits(buff,i,16)*P2_29;        i+=16;
    eph->cis   =getbits(buff,i,16)*P2_29;        i+=16;
    toc        =getbitu(buff,i,14)*60.0;         i+=14;
    eph->f0    =getbits(buff,i,31)*P2_34;        i+=31;
    eph->f1    =getbits(buff,i,21)*P2_46;        i+=21;
    eph->f2    =getbits(buff,i, 6)*P2_59;
    
    i=128*5; /* word type 5 */
    type[5]    =getbitu(buff,i, 6);              i+= 6+41;
    eph->tgd[0]=getbits(buff,i,10)*P2_32;        i+=10; /* BGD E5a/E1 */
    eph->tgd[1]=getbits(buff,i,10)*P2_32;        i+=10; /* BGD E5b/E1 */
    e5b_hs     =getbitu(buff,i, 2);              i+= 2;
    e1b_hs     =getbitu(buff,i, 2);              i+= 2;
    e5b_dvs    =getbitu(buff,i, 1);              i+= 1;
    e1b_dvs    =getbitu(buff,i, 1);
    
    /* test word types */
    if (type[0]!=0||type[1]!=1||type[2]!=2||type[3]!=3||type[4]!=4) {
        trace(3,"decode_gal_inav error: type=%d %d %d %d %d\n",type[0],type[1],
              type[2],type[3],type[4]);
        return 0;
    }
    /* test word type 0 time field */
    if (time_f!=2) {
        trace(3,"decode_gal_inav error: word0-time=%d\n",time_f);
        return 0;
    }
    /* test consistency of iod_nav */
    if (iod_nav[0]!=iod_nav[1]||iod_nav[0]!=iod_nav[2]||iod_nav[0]!=iod_nav[3]) {
        trace(3,"decode_gal_inav error: ionav=%d %d %d %d\n",iod_nav[0],
              iod_nav[1],iod_nav[2],iod_nav[3]);
        return 0;
    }
    if (!(eph->sat=satno(SYS_GAL,svid))) {
        trace(2,"decode_gal_inav svid error: svid=%d\n",svid);
        return 0;
    }
    eph->A=sqrtA*sqrtA;
    eph->iode=eph->iodc=iod_nav[0];
    eph->svh=(e5b_hs<<7)|(e5b_dvs<<6)|(e1b_hs<<1)|e1b_dvs;
    eph->ttr=gst2time(week,tow);
    tt=timediff(gst2time(week,eph->toes),eph->ttr); /* week complient to toe */
    if      (tt> 302400.0) week--;
    else if (tt<-302400.0) week++;
    eph->toe=gst2time(week,eph->toes);
    eph->toc=gst2time(week,toc);
    eph->week=week+1024; /* gal-week = gst-week + 1024 */
    eph->code=1;         /* data source = I/NAV E1B */
    
    return 1;
}
/* decode BeiDou D1 ephemeris --------------------------------------------------
* decode BeiDou D1 ephemeris (IGSO/MEO satellites) (ref [3] 5.2)
* args   : unsigned char *buff I beidou D1 subframe bits
*                                  buff[ 0- 37]: subframe 1 (300 bits)
*                                  buff[38- 75]: subframe 2
*                                  buff[76-113]: subframe 3
*          eph_t    *eph    IO  ephemeris structure
* return : status (1:ok,0:error)
*-----------------------------------------------------------------------------*/
extern int decode_bds_d1(const unsigned char *buff, eph_t *eph)
{
    double toc_bds,sqrtA;
    unsigned int toe1,toe2,sow1,sow2,sow3;
    int i,frn1,frn2,frn3;
    
    trace(3,"decode_bds_d1:\n");
    
    i=8*38*0; /* subframe 1 */
    frn1       =getbitu (buff,i+ 15, 3);
    sow1       =getbitu2(buff,i+ 18, 8,i+30,12);
    eph->svh   =getbitu (buff,i+ 42, 1); /* SatH1 */
    eph->iodc  =getbitu (buff,i+ 43, 5); /* AODC */
    eph->sva   =getbitu (buff,i+ 48, 4);
    eph->week  =getbitu (buff,i+ 60,13); /* week in BDT */
    toc_bds    =getbitu2(buff,i+ 73, 9,i+ 90, 8)*8.0;
    eph->tgd[0]=getbits (buff,i+ 98,10)*0.1*1E-9;
    eph->tgd[1]=getbits2(buff,i+108, 4,i+120, 6)*0.1*1E-9;
    eph->f2    =getbits (buff,i+214,11)*P2_66;
    eph->f0    =getbits2(buff,i+225, 7,i+240,17)*P2_33;
    eph->f1    =getbits2(buff,i+257, 5,i+270,17)*P2_50;
    eph->iode  =getbitu (buff,i+287, 5); /* AODE */
    
    i=8*38*1; /* subframe 2 */
    frn2       =getbitu (buff,i+ 15, 3);
    sow2       =getbitu2(buff,i+ 18, 8,i+30,12);
    eph->deln  =getbits2(buff,i+ 42,10,i+ 60, 6)*P2_43*SC2RAD;
    eph->cuc   =getbits2(buff,i+ 66,16,i+ 90, 2)*P2_31;
    eph->M0    =getbits2(buff,i+ 92,20,i+120,12)*P2_31*SC2RAD;
    eph->e     =getbitu2(buff,i+132,10,i+150,22)*P2_33;
    eph->cus   =getbits (buff,i+180,18)*P2_31;
    eph->crc   =getbits2(buff,i+198, 4,i+210,14)*P2_6;
    eph->crs   =getbits2(buff,i+224, 8,i+240,10)*P2_6;
    sqrtA      =getbitu2(buff,i+250,12,i+270,20)*P2_19;
    toe1       =getbitu (buff,i+290, 2); /* TOE 2-MSB */
    eph->A     =sqrtA*sqrtA;
    
    i=8*38*2; /* subframe 3 */
    frn3       =getbitu (buff,i+ 15, 3);
    sow3       =getbitu2(buff,i+ 18, 8,i+30,12);
    toe2       =getbitu2(buff,i+ 42,10,i+ 60, 5); /* TOE 5-LSB */
    eph->i0    =getbits2(buff,i+ 65,17,i+ 90,15)*P2_31*SC2RAD;
    eph->cic   =getbits2(buff,i+105, 7,i+120,11)*P2_31;
    eph->OMGd  =getbits2(buff,i+131,11,i+150,13)*P2_43*SC2RAD;
    eph->cis   =getbits2(buff,i+163, 9,i+180, 9)*P2_31;
    eph->idot  =getbits2(buff,i+189,13,i+210, 1)*P2_43*SC2RAD;
    eph->OMG0  =getbits2(buff,i+211,21,i+240,11)*P2_31*SC2RAD;
    eph->omg   =getbits2(buff,i+251,11,i+270,21)*P2_31*SC2RAD;
    eph->toes  =merge_two_u(toe1,toe2,15)*8.0;
    
    /* check consistency of subframe numbers, sows and toe/toc */
    if (frn1!=1||frn2!=2||frn3!=3) {
        trace(3,"decode_bds_d1 error: frn=%d %d %d\n",frn1,frn2,frn3);
        return 0;
    }
    if (sow2!=sow1+6||sow3!=sow2+6) {
        trace(3,"decode_bds_d1 error: sow=%d %d %d\n",sow1,sow2,sow3);
        return 0;
    }
    if (toc_bds!=eph->toes) {
        trace(3,"decode_bds_d1 error: toe=%.0f toc=%.0f\n",eph->toes,toc_bds);
        return 0;
    }
    eph->ttr=bdt2gpst(bdt2time(eph->week,sow1));      /* bdt -> gpst */
    if      (eph->toes>sow1+302400.0) eph->week++;
    else if (eph->toes<sow1-302400.0) eph->week--;
    eph->toe=bdt2gpst(bdt2time(eph->week,eph->toes)); /* bdt -> gpst */
    eph->toc=bdt2gpst(bdt2time(eph->week,toc_bds));   /* bdt -> gpst */
    return 1;
}
/* decode BeiDou D2 ephemeris --------------------------------------------------
* decode BeiDou D2 ephemeris (GEO satellites) (ref [3] 5.3)
* args   : unsigned char *buff I beidou D2 subframe 1 page bits
*                                  buff[  0- 37]: page 1 (300 bits)
*                                  buff[ 38- 75]: page 2
*                                  ...
*                                  buff[342-379]: page 10
*          eph_t    *eph    IO  ephemeris structure
* return : status (1:ok,0:error)
*-----------------------------------------------------------------------------*/
extern int decode_bds_d2(const unsigned char *buff, eph_t *eph)
{
    double toc_bds,sqrtA;
    unsigned int f1p4,cucp5,ep6,cicp7,i0p8,OMGdp9,omgp10;
    unsigned int sow1,sow3,sow4,sow5,sow6,sow7,sow8,sow9,sow10;
    int i,f1p3,cucp4,ep5,cicp6,i0p7,OMGdp8,omgp9;
    int pgn1,pgn3,pgn4,pgn5,pgn6,pgn7,pgn8,pgn9,pgn10;
    
    trace(3,"decode_bds_d2:\n");
    
    i=8*38*0; /* page 1 */
    pgn1       =getbitu (buff,i+ 42, 4);
    sow1       =getbitu2(buff,i+ 18, 8,i+ 30,12);
    eph->svh   =getbitu (buff,i+ 46, 1); /* SatH1 */
    eph->iodc  =getbitu (buff,i+ 47, 5); /* AODC */
    eph->sva   =getbitu (buff,i+ 60, 4);
    eph->week  =getbitu (buff,i+ 64,13); /* week in BDT */
    toc_bds    =getbitu2(buff,i+ 77, 5,i+ 90,12)*8.0;
    eph->tgd[0]=getbits (buff,i+102,10)*0.1*1E-9;
    eph->tgd[1]=getbits (buff,i+120,10)*0.1*1E-9;
    
    i=8*38*2; /* page 3 */
    pgn3       =getbitu (buff,i+ 42, 4);
    sow3       =getbitu2(buff,i+ 18, 8,i+ 30,12);
    eph->f0    =getbits2(buff,i+100,12,i+120,12)*P2_33;
    f1p3       =getbits (buff,i+132,4);
    
    i=8*38*3; /* page 4 */
    pgn4       =getbitu (buff,i+ 42, 4);
    sow4       =getbitu2(buff,i+ 18, 8,i+ 30,12);
    f1p4       =getbitu2(buff,i+ 46, 6,i+ 60,12);
    eph->f2    =getbits2(buff,i+ 72,10,i+ 90, 1)*P2_66;
    eph->iode  =getbitu (buff,i+ 91, 5); /* AODE */
    eph->deln  =getbits (buff,i+ 96,16)*P2_43*SC2RAD;
    cucp4      =getbits (buff,i+120,14);
    
    i=8*38*4; /* page 5 */
    pgn5       =getbitu (buff,i+ 42, 4);
    sow5       =getbitu2(buff,i+ 18, 8,i+ 30,12);
    cucp5      =getbitu (buff,i+ 46, 4);
    eph->M0    =getbits3(buff,i+ 50, 2,i+ 60,22,i+ 90, 8)*P2_31*SC2RAD;
    eph->cus   =getbits2(buff,i+ 98,14,i+120, 4)*P2_31;
    ep5        =getbits (buff,i+124,10);
    
    i=8*38*5; /* page 6 */
    pgn6       =getbitu (buff,i+ 42, 4);
    sow6       =getbitu2(buff,i+ 18, 8,i+ 30,12);
    ep6        =getbitu2(buff,i+ 46, 6,i+ 60,16);
    sqrtA      =getbitu3(buff,i+ 76, 6,i+ 90,22,i+120,4)*P2_19;
    cicp6      =getbits (buff,i+124,10);
    eph->A     =sqrtA*sqrtA;
    
    i=8*38*6; /* page 7 */
    pgn7       =getbitu (buff,i+ 42, 4);
    sow7       =getbitu2(buff,i+ 18, 8,i+ 30,12);
    cicp7      =getbitu2(buff,i+ 46, 6,i+ 60, 2);
    eph->cis   =getbits (buff,i+ 62,18)*P2_31;
    eph->toes  =getbitu2(buff,i+ 80, 2,i+ 90,15)*8.0;
    i0p7       =getbits2(buff,i+105, 7,i+120,14);
    
    i=8*38*7; /* page 8 */
    pgn8       =getbitu (buff,i+ 42, 4);
    sow8       =getbitu2(buff,i+ 18, 8,i+ 30,12);
    i0p8       =getbitu2(buff,i+ 46, 6,i+ 60, 5);
    eph->crc   =getbits2(buff,i+ 65,17,i+ 90, 1)*P2_6;
    eph->crs   =getbits (buff,i+ 91,18)*P2_6;
    OMGdp8     =getbits2(buff,i+109, 3,i+120,16);
    
    i=8*38*8; /* page 9 */
    pgn9       =getbitu (buff,i+ 42, 4);
    sow9       =getbitu2(buff,i+ 18, 8,i+ 30,12);
    OMGdp9     =getbitu (buff,i+ 46, 5);
    eph->OMG0  =getbits3(buff,i+ 51, 1,i+ 60,22,i+ 90, 9)*P2_31*SC2RAD;
    omgp9      =getbits2(buff,i+ 99,13,i+120,14);
    
    i=8*38*9; /* page 10 */
    pgn10      =getbitu (buff,i+ 42, 4);
    sow10      =getbitu2(buff,i+ 18, 8,i+ 30,12);
    omgp10     =getbitu (buff,i+ 46, 5);
    eph->idot  =getbits2(buff,i+ 51, 1,i+ 60,13)*P2_43*SC2RAD;
    
    /* check consistency of page numbers, sows and toe/toc */
    if (pgn1!=1||pgn3!=3||pgn4!=4||pgn5!=5||pgn6!=6||pgn7!=7||pgn8!=8||pgn9!=9||
        pgn10!=10) {
        trace(3,"decode_bds_d2 error: pgn=%d %d %d %d %d %d %d %d %d\n",
              pgn1,pgn3,pgn4,pgn5,pgn6,pgn7,pgn8,pgn9,pgn10);
        return 0;
    }
    if (sow3!=sow1+6||sow4!=sow3+3||sow5!=sow4+3||sow6!=sow5+3||
        sow7!=sow6+3||sow8!=sow7+3||sow9!=sow8+3||sow10!=sow9+3) {
        trace(3,"decode_bds_d2 error: sow=%d %d %d %d %d %d %d %d %d\n",
              sow1,sow3,sow4,sow5,sow6,sow7,sow8,sow9,sow10);
        return 0;
    }
    if (toc_bds!=eph->toes) {
        trace(3,"decode_bds_d2 error: toe=%.0f toc=%.0f\n",eph->toes,toc_bds);
        return 0;
    }
    eph->f1  =merge_two_s(f1p3  ,f1p4  ,18)*P2_50;
    eph->cuc =merge_two_s(cucp4 ,cucp5 , 4)*P2_31;
    eph->e   =merge_two_s(ep5   ,ep6   ,22)*P2_33;
    eph->cic =merge_two_s(cicp6 ,cicp7 , 8)*P2_31;
    eph->i0  =merge_two_s(i0p7  ,i0p8  ,11)*P2_31*SC2RAD;
    eph->OMGd=merge_two_s(OMGdp8,OMGdp9, 5)*P2_43*SC2RAD;
    eph->omg =merge_two_s(omgp9 ,omgp10, 5)*P2_31*SC2RAD;
    
    eph->ttr=bdt2gpst(bdt2time(eph->week,sow1));      /* bdt -> gpst */
    if      (eph->toes>sow1+302400.0) eph->week++;
    else if (eph->toes<sow1-302400.0) eph->week--;
    eph->toe=bdt2gpst(bdt2time(eph->week,eph->toes)); /* bdt -> gpst */
    eph->toc=bdt2gpst(bdt2time(eph->week,toc_bds));   /* bdt -> gpst */
    return 1;
}
/* test hamming code of glonass ephemeris string -------------------------------
* test hamming code of glonass ephemeris string (ref [2] 4.7)
* args   : unsigned char *buff I glonass navigation data string bits in frame
*                                with hamming
*                                  buff[ 0]: string bit 85-78
*                                  buff[ 1]: string bit 77-70
*                                  ...
*                                  buff[10]: string bit  5- 1 (0 padded)
* return : status (1:ok,0:error)
*-----------------------------------------------------------------------------*/
extern int test_glostr(const unsigned char *buff)
{
    static const unsigned char xor_8bit[256]={ /* xor of 8 bits */
        0,1,1,0,1,0,0,1,1,0,0,1,0,1,1,0,1,0,0,1,0,1,1,0,0,1,1,0,1,0,0,1,
        1,0,0,1,0,1,1,0,0,1,1,0,1,0,0,1,0,1,1,0,1,0,0,1,1,0,0,1,0,1,1,0,
        1,0,0,1,0,1,1,0,0,1,1,0,1,0,0,1,0,1,1,0,1,0,0,1,1,0,0,1,0,1,1,0,
        0,1,1,0,1,0,0,1,1,0,0,1,0,1,1,0,1,0,0,1,0,1,1,0,0,1,1,0,1,0,0,1,
        1,0,0,1,0,1,1,0,0,1,1,0,1,0,0,1,0,1,1,0,1,0,0,1,1,0,0,1,0,1,1,0,
        0,1,1,0,1,0,0,1,1,0,0,1,0,1,1,0,1,0,0,1,0,1,1,0,0,1,1,0,1,0,0,1,
        0,1,1,0,1,0,0,1,1,0,0,1,0,1,1,0,1,0,0,1,0,1,1,0,0,1,1,0,1,0,0,1,
        1,0,0,1,0,1,1,0,0,1,1,0,1,0,0,1,0,1,1,0,1,0,0,1,1,0,0,1,0,1,1,0
    };
    static const unsigned char mask_hamming[][12]={ /* mask of hamming codes */
        {0x55,0x55,0x5A,0xAA,0xAA,0xAA,0xB5,0x55,0x6A,0xD8,0x08},
        {0x66,0x66,0x6C,0xCC,0xCC,0xCC,0xD9,0x99,0xB3,0x68,0x10},
        {0x87,0x87,0x8F,0x0F,0x0F,0x0F,0x1E,0x1E,0x3C,0x70,0x20},
        {0x07,0xF8,0x0F,0xF0,0x0F,0xF0,0x1F,0xE0,0x3F,0x80,0x40},
        {0xF8,0x00,0x0F,0xFF,0xF0,0x00,0x1F,0xFF,0xC0,0x00,0x80},
        {0x00,0x00,0x0F,0xFF,0xFF,0xFF,0xE0,0x00,0x00,0x01,0x00},
        {0xFF,0xFF,0xF0,0x00,0x00,0x00,0x00,0x00,0x00,0x02,0x00},
        {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xF8}
    };
    unsigned char cs;
    int i,j,n=0;
    
    for (i=0;i<8;i++) {
        for (j=0,cs=0;j<11;j++) {
            cs^=xor_8bit[buff[j]&mask_hamming[i][j]];
        }
        if (cs) n++;
    }
    return n==0||(n==2&&cs);
}
/* decode glonass ephemeris strings --------------------------------------------
* decode glonass ephemeris string (ref [2])
* args   : unsigned char *buff I glonass navigation data string bits in frames
*                                (without hamming and time mark)
*                                  buff[ 0- 9]: string #1 (77 bits)
*                                  buff[10-19]: string #2
*                                  buff[20-29]: string #3
*                                  buff[30-39]: string #4
*          geph_t *geph  IO     glonass ephemeris message
* return : status (1:ok,0:error)
* notes  : geph->tof should be set to frame time witin 1/2 day before calling
*          geph->frq is set to 0
*-----------------------------------------------------------------------------*/
extern int decode_glostr(const unsigned char *buff, geph_t *geph)
{
    double tow,tod,tof,toe;
    int P,P1,P2,P3,P4,tk_h,tk_m,tk_s,tb,ln,NT,slot,M,week;
    int i=1,frn1,frn2,frn3,frn4;
    
    trace(3,"decode_glostr:\n");
    
    /* frame 1 */
    frn1        =getbitu(buff,i, 4);           i+= 4+2;
    P1          =getbitu(buff,i, 2);           i+= 2;
    tk_h        =getbitu(buff,i, 5);           i+= 5;
    tk_m        =getbitu(buff,i, 6);           i+= 6;
    tk_s        =getbitu(buff,i, 1)*30;        i+= 1;
    geph->vel[0]=getbitg(buff,i,24)*P2_20*1E3; i+=24;
    geph->acc[0]=getbitg(buff,i, 5)*P2_30*1E3; i+= 5;
    geph->pos[0]=getbitg(buff,i,27)*P2_11*1E3; i+=27+4;
    
    /* frame 2 */
    frn2        =getbitu(buff,i, 4);           i+= 4;
    geph->svh   =getbitu(buff,i, 3);           i+= 3;
    P2          =getbitu(buff,i, 1);           i+= 1;
    tb          =getbitu(buff,i, 7);           i+= 7+5;
    geph->vel[1]=getbitg(buff,i,24)*P2_20*1E3; i+=24;
    geph->acc[1]=getbitg(buff,i, 5)*P2_30*1E3; i+= 5;
    geph->pos[1]=getbitg(buff,i,27)*P2_11*1E3; i+=27+4;
    
    /* frame 3 */
    frn3        =getbitu(buff,i, 4);           i+= 4;
    P3          =getbitu(buff,i, 1);           i+= 1;
    geph->gamn  =getbitg(buff,i,11)*P2_40;     i+=11+1;
    P           =getbitu(buff,i, 2);           i+= 2;
    ln          =getbitu(buff,i, 1);           i+= 1;
    geph->vel[2]=getbitg(buff,i,24)*P2_20*1E3; i+=24;
    geph->acc[2]=getbitg(buff,i, 5)*P2_30*1E3; i+= 5;
    geph->pos[2]=getbitg(buff,i,27)*P2_11*1E3; i+=27+4;
    
    /* frame 4 */
    frn4        =getbitu(buff,i, 4);           i+= 4;
    geph->taun  =getbitg(buff,i,22)*P2_30;     i+=22;
    geph->dtaun =getbitg(buff,i, 5)*P2_30;     i+= 5;
    geph->age   =getbitu(buff,i, 5);           i+= 5+14;
    P4          =getbitu(buff,i, 1);           i+= 1;
    geph->sva   =getbitu(buff,i, 4);           i+= 4+3;
    NT          =getbitu(buff,i,11);           i+=11;
    slot        =getbitu(buff,i, 5);           i+= 5;
    M           =getbitu(buff,i, 2);
    
    if (frn1!=1||frn2!=2||frn3!=3||frn4!=4) {
        trace(3,"decode_glostr error: frn=%d %d %d %d %d\n",frn1,frn2,frn3,frn4);
        return 0;
    }
    if (!(geph->sat=satno(SYS_GLO,slot))) {
        trace(2,"decode_glostr error: slot=%d\n",slot);
        return 0;
    }
    geph->frq=0;
    geph->iode=tb;
    tow=time2gpst(gpst2utc(geph->tof),&week);
    tod=fmod(tow,86400.0); tow-=tod;
    tof=tk_h*3600.0+tk_m*60.0+tk_s-10800.0; /* lt->utc */
    if      (tof<tod-43200.0) tof+=86400.0;
    else if (tof>tod+43200.0) tof-=86400.0;
    geph->tof=utc2gpst(gpst2time(week,tow+tof));
    toe=tb*900.0-10800.0; /* lt->utc */
    if      (toe<tod-43200.0) toe+=86400.0;
    else if (toe>tod+43200.0) toe-=86400.0;
    geph->toe=utc2gpst(gpst2time(week,tow+toe)); /* utc->gpst */
    return 1;
}
/* decode gps/qzss navigation data subframe 1 --------------------------------*/
static int decode_subfrm1(const unsigned char *buff, eph_t *eph)
{
    double tow,toc;
    int i=48,week,iodc0,iodc1,tgd;
    
    trace(4,"decode_subfrm1:\n");
    trace(5,"decode_subfrm1: buff="); traceb(5,buff,30);
    
    tow        =getbitu(buff,24,17)*6.0;           /* transmission time */
    week       =getbitu(buff,i,10);       i+=10;
    eph->code  =getbitu(buff,i, 2);       i+= 2;
    eph->sva   =getbitu(buff,i, 4);       i+= 4;   /* ura index */
    eph->svh   =getbitu(buff,i, 6);       i+= 6;
    iodc0      =getbitu(buff,i, 2);       i+= 2;
    eph->flag  =getbitu(buff,i, 1);       i+= 1+87;
    tgd        =getbits(buff,i, 8);       i+= 8;
    iodc1      =getbitu(buff,i, 8);       i+= 8;
    toc        =getbitu(buff,i,16)*16.0;  i+=16;
    eph->f2    =getbits(buff,i, 8)*P2_55; i+= 8;
    eph->f1    =getbits(buff,i,16)*P2_43; i+=16;
    eph->f0    =getbits(buff,i,22)*P2_31;
    
    eph->tgd[0]=tgd==-128?0.0:tgd*P2_31; /* ref [4] */
    eph->iodc=(iodc0<<8)+iodc1;
    eph->week=adjgpsweek(week); /* week of tow */
    eph->ttr=gpst2time(eph->week,tow);
    eph->toc=gpst2time(eph->week,toc);
    
    return 1;
}
/* decode gps/qzss navigation data subframe 2 --------------------------------*/
static int decode_subfrm2(const unsigned char *buff, eph_t *eph)
{
    double sqrtA;
    int i=48;
    
    trace(4,"decode_subfrm2:\n");
    trace(5,"decode_subfrm2: buff="); traceb(5,buff,30);
    
    eph->iode=getbitu(buff,i, 8);              i+= 8;
    eph->crs =getbits(buff,i,16)*P2_5;         i+=16;
    eph->deln=getbits(buff,i,16)*P2_43*SC2RAD; i+=16;
    eph->M0  =getbits(buff,i,32)*P2_31*SC2RAD; i+=32;
    eph->cuc =getbits(buff,i,16)*P2_29;        i+=16;
    eph->e   =getbitu(buff,i,32)*P2_33;        i+=32;
    eph->cus =getbits(buff,i,16)*P2_29;        i+=16;
    sqrtA    =getbitu(buff,i,32)*P2_19;        i+=32;
    eph->toes=getbitu(buff,i,16)*16.0;         i+=16;
    eph->fit =getbitu(buff,i, 1)?0.0:4.0; /* 0:4hr,1:>4hr */
    
    eph->A=sqrtA*sqrtA;
    
    return 2;
}
/* decode gps/qzss navigation data subframe 3 --------------------------------*/
static int decode_subfrm3(const unsigned char *buff, eph_t *eph)
{
    double tow,toc;
    int i=48,iode;
    
    trace(4,"decode_subfrm3:\n");
    trace(5,"decode_subfrm3: buff="); traceb(5,buff,30);
    
    eph->cic =getbits(buff,i,16)*P2_29;        i+=16;
    eph->OMG0=getbits(buff,i,32)*P2_31*SC2RAD; i+=32;
    eph->cis =getbits(buff,i,16)*P2_29;        i+=16;
    eph->i0  =getbits(buff,i,32)*P2_31*SC2RAD; i+=32;
    eph->crc =getbits(buff,i,16)*P2_5;         i+=16;
    eph->omg =getbits(buff,i,32)*P2_31*SC2RAD; i+=32;
    eph->OMGd=getbits(buff,i,24)*P2_43*SC2RAD; i+=24;
    iode     =getbitu(buff,i, 8);              i+= 8;
    eph->idot=getbits(buff,i,14)*P2_43*SC2RAD;
    
    /* check iode and iodc consistency */
    if (iode!=eph->iode||iode!=(eph->iodc&0xFF)) return 0;
    
    /* adjustment for week handover */
    tow=time2gpst(eph->ttr,&eph->week);
    toc=time2gpst(eph->toc,NULL);
    if      (eph->toes<tow-302400.0) {eph->week++; tow-=604800.0;}
    else if (eph->toes>tow+302400.0) {eph->week--; tow+=604800.0;}
    eph->toe=gpst2time(eph->week,eph->toes);
    eph->toc=gpst2time(eph->week,toc);
    eph->ttr=gpst2time(eph->week,tow);
    
    return 3;
}



/* decode gps/qzss navigation data frame ---------------------------------------
* decode navigation data frame and extract ephemeris and ion/utc parameters
* args   : unsigned char *buff I gps navigation data frame (without parity)
*                                  buff[0-29]: 24 bits x 10 words
*          eph_t *eph    IO     ephemeris message      (NULL: no input)
*          alm_t *alm    IO     almanac                (NULL: no input)
*          double *ion   IO     ionospheric parameters (NULL: no input)
*          double *utc   IO     delta-utc parameters   (NULL: no input)
*          int   *leaps  IO     leap seconds (s)       (NULL: no input)
* return : status (0:no valid, 1-5:subframe id)
* notes  : use cpu time to resolve modulo 1024 ambiguity of the week number
*          see ref [1]
*          utc[3] reference week for utc parameter is truncated in 8 bits
*          ion and utc parameters by qzss indicate local iono and qzst-utc
*          parameters.
*-----------------------------------------------------------------------------*/
extern int decode_frame(const unsigned char *buff, eph_t *eph, alm_t *alm,
                        double *ion, double *utc, int *leaps)
{
    int id=getbitu(buff,43,3); /* subframe id */
    
    trace(3,"decodefrm: id=%d\n",id);
    
    switch (id) {
        case 1: return decode_subfrm1(buff,eph);
        case 2: return decode_subfrm2(buff,eph);
        case 3: return decode_subfrm3(buff,eph);

    }
    return 0;
}
/* initialize receiver raw data control ----------------------------------------
* initialize receiver raw data control struct and reallocate obsevation and
* epheris buffer
* args   : raw_t  *raw   IO     receiver raw data control struct
*          int    format I      stream format (STRFMT_???)
* return : status (1:ok,0:memory allocation error)
*-----------------------------------------------------------------------------*/
extern int init_raw(raw_t *raw, int format)
{
    const double lam_glo[NFREQ]={CLIGHT/FREQ1_GLO,CLIGHT/FREQ2_GLO};
    gtime_t time0={0};
    obsd_t data0={{0}};
    eph_t  eph0 ={0,-1,-1};
    alm_t  alm0 ={0,-1};
    geph_t geph0={0,-1};
//    seph_t seph0={0};
//    sbsmsg_t sbsmsg0={0};
//    lexmsg_t lexmsg0={0};
    int i,j,sys,ret=1;
    
    trace(3,"init_raw: format=%d\n",format);
    
    raw->time=time0;
    raw->ephsat=0;

    for (j=0;j<380;j++) raw->subfrm2[j]=0;

	for (i=0;i<MAXSAT;i++) {
        for (j=0;j<380;j++) raw->subfrm[i][j]=0;
        for (j=0;j<NFREQ+NEXOBS;j++) {
//            raw->tobs [i][j]=time0;
//            raw->lockt[i][j]=0.0;
//            raw->halfc[i][j]=0;
        }
//        raw->icpp[i]=raw->off[i]=raw->prCA[i]=raw->dpCA[i]=0.0;
    }
//    for (i=0;i<MAXOBS;i++) raw->freqn[i]=0;
//    raw->lexmsg=lexmsg0;
//    raw->icpc=0.0;
    raw->nbyte=raw->len=0;
    raw->iod=raw->flag=raw->tbase=raw->outtype=0;
    raw->tod=-1;
    for (i=0;i<MAXRAWLEN;i++) raw->buff[i]=0;
    raw->opt[0]='\0';
    raw->format=-1;
    
    raw->obs.data =NULL;
    raw->obuf.data=NULL;



//     raw->half_cyc =NULL;
//     raw->rcv_data =NULL;
    
    if (!(raw->obs.data =(obsd_t *)malloc(sizeof(obsd_t)*MAXOBS))||
        !(raw->obuf.data=(obsd_t *)malloc(sizeof(obsd_t)*MAXOBS)) ) {
        free_raw(raw);
        return 0;
    }
    raw->obs.n =0;
    raw->obuf.n=0;
    raw->nav.n =MAXSAT;
    raw->nav.na=MAXSAT;
    raw->nav.ng=NSATGLO;
    raw->nav.ns=NSATSBS*2;
    for (i=0;i<MAXOBS   ;i++) raw->obs.data [i]=data0;
    for (i=0;i<MAXOBS   ;i++) raw->obuf.data[i]=data0;



    
    /* initialize receiver dependent data */
    raw->format=format;
    if (!ret) {
        free_raw(raw);
        return 0;
    }
    return 1;
}
/* free receiver raw data control ----------------------------------------------
* free observation and ephemeris buffer in receiver raw data control struct
* args   : raw_t  *raw   IO     receiver raw data control struct
* return : none
*-----------------------------------------------------------------------------*/
extern void free_raw(raw_t *raw)
{

    
    trace(3,"free_raw:\n");
    
    free(raw->obs.data ); raw->obs.data =NULL; raw->obs.n =0;
    free(raw->obuf.data); raw->obuf.data=NULL; raw->obuf.n=0;

    raw->nav.na=0;



}



#define UBXSYNC1    0xB5        /* ubx message sync code 1 */
#define UBXSYNC2    0x62        /* ubx message sync code 2 */
#define UBXCFG      0x06        /* ubx message cfg-??? */

#define ID_NAVSOL   0x0106      /* ubx message id: nav solution info */
#define ID_NAVTIME  0x0120      /* ubx message id: nav time gps */
#define ID_RXMRAW   0x0210      /* ubx message id: raw measurement data */
#define ID_RXMSFRB  0x0211      /* ubx message id: subframe buffer */
#define ID_RXMSFRBX 0x0213      /* ubx message id: raw subframe data */
#define ID_RXMRAWX  0x0215      /* ubx message id: multi-gnss raw meas data */
#define ID_TRKD5    0x030A      /* ubx message id: trace mesurement data */
#define ID_TRKMEAS  0x0310      /* ubx message id: trace mesurement data */
#define ID_TRKSFRBX 0x030F      /* ubx message id: trace subframe buffer */

#define FU1         1           /* ubx message field types */
#define FU2         2
#define FU4         3
#define FI1         4
#define FI2         5
#define FI4         6
#define FR4         7
#define FR8         8
#define FS32        9

#define P2_10       0.0009765625 /* 2^-10 */

#define CPSTD_VALID 5           /* std-dev threshold of carrier-phase valid */

#define ROUND(x)    (int)floor((x)+0.5)

/* get fields (little-endian) ------------------------------------------------*/
#define U1(p) (*((unsigned char *)(p)))
#define I1(p) (*((signed char *)(p)))
static unsigned short U2(unsigned char *p) {unsigned short u; memcpy(&u,p,2); return u;}
static unsigned int   U4(unsigned char *p) {unsigned int   u; memcpy(&u,p,4); return u;}
static int            I4(unsigned char *p) {int            u; memcpy(&u,p,4); return u;}
static float          R4(unsigned char *p) {float          r; memcpy(&r,p,4); return r;}
static double         R8(unsigned char *p) {double         r; memcpy(&r,p,8); return r;}

static double         I8(unsigned char *p) {return I4(p+4)*4294967296.0+U4(p);}

/* set fields (little-endian) ------------------------------------------------*/
static void setU1(unsigned char *p, unsigned char  u) {*p=u;}
static void setU2(unsigned char *p, unsigned short u) {memcpy(p,&u,2);}
static void setU4(unsigned char *p, unsigned int   u) {memcpy(p,&u,4);}
static void setI1(unsigned char *p, signed char    i) {*p=(unsigned char)i;}
static void setI2(unsigned char *p, short          i) {memcpy(p,&i,2);}
static void setI4(unsigned char *p, int            i) {memcpy(p,&i,4);}
static void setR4(unsigned char *p, float          r) {memcpy(p,&r,4);}
static void setR8(unsigned char *p, double         r) {memcpy(p,&r,8);}

/* checksum ------------------------------------------------------------------*/
static int checksum(unsigned char *buff, int len)
{
    unsigned char cka=0,ckb=0;
    int i;
    
    for (i=2;i<len-2;i++) {
        cka+=buff[i]; ckb+=cka;
    }
    return cka==buff[len-2]&&ckb==buff[len-1];
}
static void setcs(unsigned char *buff, int len)
{
    unsigned char cka=0,ckb=0;
    int i;
    
    for (i=2;i<len-2;i++) {
        cka+=buff[i]; ckb+=cka;
    }
    buff[len-2]=cka;
    buff[len-1]=ckb;
}
/* ubx gnss indicator (ref [2] 25) -------------------------------------------*/
static int ubx_sys(int ind)
{
    switch (ind) {
        case 0: return SYS_GPS;
        case 1: return SYS_SBS;
        case 2: return SYS_GAL;
        case 3: return SYS_CMP;
        case 5: return SYS_QZS;
        case 6: return SYS_GLO;
    }
    return 0;
}
/* 8-bit week -> full week ---------------------------------------------------*/
static void adj_utcweek(gtime_t time, double *utc)
{
    int week;
    
    if (utc[3]>=256.0) return;
    time2gpst(time,&week);
    utc[3]+=week/256*256;
    if      (utc[3]<week-128) utc[3]+=256.0;
    else if (utc[3]>week+128) utc[3]-=256.0;
}

/* decode ubx-rxm-rawx: multi-gnss raw measurement data (ref [3]) ------------*/
static int decode_rxmrawx(raw_t *raw)
{
    gtime_t time;
    double tow,cp1,pr1,tadj=0.0,toff=0.0,freq,tn;
    int i,j,sys,prn,sat,n=0,nsat,week,tstat,lockt,slip,halfv,halfc,fcn,cpstd;
    int std_slip=0;
    char *q;
    unsigned char *p=raw->buff+6;
    
    trace(4,"decode_rxmrawx: len=%d\n",raw->len);
    
    nsat=U1(p+11);
    if (raw->len<24+32*nsat) {
        trace(2,"ubx rxmrawx length error: len=%d nsat=%d\n",raw->len,nsat);
        return -1;
    }
    tow=R8(p);
    week=U2(p+8);
    time=gpst2time(week,tow);
    
    if (week==0) {
        trace(3,"ubx rxmrawx week=0 error: len=%d nsat=%d\n",raw->len,nsat);
        return 0;
    }

    /* time tag adjustment option (-TADJ) */
    if ((q=strstr(raw->opt,"-TADJ="))) {
        sscanf(q,"-TADJ=%lf",&tadj);
    }
    /* slip theshold of std-dev of carreir-phase (-STD_SLIP) */
    if ((q=strstr(raw->opt,"-STD_SLIP="))) {
        sscanf(q,"-STD_SLIP=%d",&std_slip);
    }
    /* time tag adjustment */
    if (tadj>0.0) {
        tn=time2gpst(time,&week)/tadj;
        toff=(tn-floor(tn+0.5))*tadj;
        time=timeadd(time,-toff);
    }
    for (i=0,p+=16;i<nsat&&i<MAXOBS;i++,p+=32) {
        
        if (!(sys=ubx_sys(U1(p+20)))) {
            trace(2,"ubx rxmrawx: system error\n");
            continue;
        }
        prn=U1(p+21)+(sys==SYS_QZS?192:0);
        if (!(sat=satno(sys,prn))) {
            trace(2,"ubx rxmrawx sat number error: sys=%2d prn=%2d\n",sys,prn);
            continue;
        }
        cpstd=U1(p+28)&15; /* carrier-phase std-dev */
        tstat=U1(p+30); /* tracking status */
        pr1=tstat&1?R8(p  ):0.0;
        cp1=tstat&2?R8(p+8):0.0;
        if (cp1==-0.5||cpstd>CPSTD_VALID) cp1=0.0; /* invalid phase */
        raw->obs.data[n].sat=sat;
        raw->obs.data[n].time=time;
        raw->obs.data[n].P[0]=pr1;
        raw->obs.data[n].L[0]=cp1;
        
        /* offset by time tag adjustment */
        if (toff!=0.0) {
            fcn=(int)U1(p+23)-7;
            freq=sys==SYS_CMP?FREQ1_CMP:
                 (sys==SYS_GLO?FREQ1_GLO+DFRQ1_GLO*fcn:FREQ1);
            raw->obs.data[n].P[0]-=toff*CLIGHT;
            raw->obs.data[n].L[0]-=toff*freq;
        }
        raw->obs.data[n].D[0]=R4(p+16);
        raw->obs.data[n].SNR[0]=U1(p+26)*4;
        raw->obs.data[n].LLI[0]=0;
        raw->obs.data[n].code[0]=
            sys==SYS_CMP?CODE_L1I:(sys==SYS_GAL?CODE_L1X:CODE_L1C);
        
        lockt=U2(p+24);    /* lock time count (ms) */
//        slip=lockt==0||lockt<raw->lockt[sat-1][0]?1:0;
#if 0
        if (std_slip>0) {
            slip|=(cpstd>=std_slip)?1:0; /* slip by std-dev of cp */
        }
#else
        if (std_slip>0) {
            cp1=0.0;
        }
#endif
        halfv=tstat&4?1:0; /* half cycle valid */
        halfc=tstat&8?1:0; /* half cycle subtracted from phase */
        
        if (cp1!=0.0) { /* carrier-phase valid */
            
            /* LLI: bit1=loss-of-lock,bit2=half-cycle-invalid */
//            raw->obs.data[n].LLI[0]|=slip?LLI_SLIP:0;
#if 0
            raw->obs.data[n].LLI[0]|=halfc!=raw->halfc[sat-1][0]?1:0;
#elif 1
            raw->obs.data[n].LLI[0]|=halfc?LLI_HALFA:0; /* half-cycle subtraced */
#else
            raw->obs.data[n].LLI[0]|=halfc?LLI_HALFS:0; /* half-cycle subtraced */
#endif
            raw->obs.data[n].LLI[0]|=halfv?0:LLI_HALFC;
//            raw->lockt[sat-1][0]=lockt;
//            raw->halfc[sat-1][0]=halfc;
        }
        for (j=1;j<NFREQ+NEXOBS;j++) {
            raw->obs.data[n].L[j]=raw->obs.data[n].P[j]=0.0;
            raw->obs.data[n].D[j]=0.0;
            raw->obs.data[n].SNR[j]=raw->obs.data[n].LLI[j]=0;
            raw->obs.data[n].code[j]=CODE_NONE;
        }
        n++;
    }
    raw->time=time;
    raw->obs.n=n;
    return 1;
}
/* save subframe -------------------------------------------------------------*/
static int save_subfrm(int sat, raw_t *raw)
{
    unsigned char *p=raw->buff+6,*q;
    int i,j,n,id=(U4(p+6)>>2)&0x7;
    
    trace(4,"save_subfrm: sat=%2d id=%d\n",sat,id);
    
    if (id<1||5<id) return 0;
    
    q=raw->subfrm[sat-1]+(id-1)*30;
    
    for (i=n=0,p+=2;i<10;i++,p+=4) {
        for (j=23;j>=0;j--) {
            *q=(*q<<1)+((U4(p)>>j)&1); if (++n%8==0) q++;
        }
    }
    return id;
}
/* decode ephemeris ----------------------------------------------------------*/
static int decode_ephem(int sat, raw_t *raw)
{
    eph_t eph={0};
    int res1,res11;
	int res2,res22;
	int res3,res33;

    trace(4,"decode_ephem: sat=%2d\n",sat);
    
	{
		res1 = decode_frame(raw->subfrm[sat-1]   ,&eph,NULL,NULL,NULL,NULL);
		res11= decode_frame(raw->subfrm2         ,&eph,NULL,NULL,NULL,NULL);

		res2 = decode_frame(raw->subfrm[sat-1]+30,&eph,NULL,NULL,NULL,NULL);
		res22= decode_frame(raw->subfrm2      +30,&eph,NULL,NULL,NULL,NULL);

		res3 = decode_frame(raw->subfrm[sat-1]+60,&eph,NULL,NULL,NULL,NULL);
		res33= decode_frame(raw->subfrm2      +60,&eph,NULL,NULL,NULL,NULL);
	}


    if (decode_frame(raw->subfrm[sat-1]   ,&eph,NULL,NULL,NULL,NULL)!=1||
        decode_frame(raw->subfrm[sat-1]+30,&eph,NULL,NULL,NULL,NULL)!=2||
        decode_frame(raw->subfrm[sat-1]+60,&eph,NULL,NULL,NULL,NULL)!=3) 
	{
		return 0;
	}
    
    if (!strstr(raw->opt,"-EPHALL")) {
        if (eph.iode==raw->nav.eph.iode&&
            eph.iodc==raw->nav.eph.iodc) return 0; /* unchanged */
    }
    eph.sat=sat;
    raw->nav.eph=eph;
    raw->ephsat=sat;
    return 2;
}


/* decode gps and qzss navigation data ---------------------------------------*/
static int decode_nav(raw_t *raw, int sat, int off)
{
    unsigned int words[10];
    int i,id;
    unsigned char *p=raw->buff+6+off;
    
    if (raw->len<48+off) {
        trace(2,"ubx rawsfrbx length error: sat=%d len=%d\n",sat,raw->len);
        return -1;
    }
    for (i=0;i<10;i++,p+=4) words[i]=U4(p)>>6; /* 24 bits without parity */
    
    id=(words[1]>>2)&7;
    if (id<1||5<id) {
        trace(2,"ubx rawsfrbx subfrm id error: sat=%2d\n",sat);
        return -1;
    }
    for (i=0;i<10;i++) {
        setbitu(raw->subfrm[sat-1]+(id-1)*30,i*24,24,words[i]);
		setbitu(raw->subfrm2+(id-1)*30,i*24,24,words[i]);
    }
    if (id==3) return decode_ephem(sat,raw);

    return 0;
}
/* decode galileo navigation data --------------------------------------------*/
static int decode_enav(raw_t *raw, int sat, int off)
{
    eph_t eph={0};
    unsigned char *p=raw->buff+6+off,buff[32],crc_buff[26]={0};
    int i,j,k,part1,page1,part2,page2,type;
    
    if (raw->len<44+off) {
        trace(2,"ubx rawsfrbx length error: sat=%d len=%d\n",sat,raw->len);
        return -1;
    }
    for (i=k=0;i<8;i++,p+=4) for (j=0;j<4;j++) {
        buff[k++]=p[3-j];
    }
    part1=getbitu(buff   ,0,1);
    page1=getbitu(buff   ,1,1);
    part2=getbitu(buff+16,0,1);
    page2=getbitu(buff+16,1,1);
    
    /* skip alert page */
    if (page1==1||page2==1) return 0;
    
    /* test even-odd parts */
    if (part1!=0||part2!=1) {
        trace(2,"ubx rawsfrbx gal page even/odd error: sat=%2d\n",sat);
        return -1;
    }
    /* test crc (4(pad) + 114 + 82 bits) */
    for (i=0,j=  4;i<15;i++,j+=8) setbitu(crc_buff,j,8,getbitu(buff   ,i*8,8));
    for (i=0,j=118;i<11;i++,j+=8) setbitu(crc_buff,j,8,getbitu(buff+16,i*8,8));
    if (rtk_crc24q(crc_buff,25)!=getbitu(buff+16,82,24)) {
        trace(2,"ubx rawsfrbx gal page crc error: sat=%2d\n",sat);
        return -1;
    }
    type=getbitu(buff,2,6); /* word type */
    
    /* skip word except for ephemeris, iono, utc parameters */
    if (type>6) return 0;
    
    /* clear word 0-6 flags */
    if (type==2) raw->subfrm[sat-1][112]=0;
    
    /* save page data (112 + 16 bits) to frame buffer */
    k=type*16;
    for (i=0,j=2;i<14;i++,j+=8) raw->subfrm[sat-1][k++]=getbitu(buff   ,j,8);
    for (i=0,j=2;i< 2;i++,j+=8) raw->subfrm[sat-1][k++]=getbitu(buff+16,j,8);
    
    /* test word 0-6 flags */
    raw->subfrm[sat-1][112]|=(1<<type);
    if (raw->subfrm[sat-1][112]!=0x7F) return 0;
    
    /* decode galileo inav ephemeris */
    if (!decode_gal_inav(raw->subfrm[sat-1],&eph)) {
        return 0;
    }
    /* test svid consistency */
    if (eph.sat!=sat) {
        trace(2,"ubx rawsfrbx gal svid error: sat=%2d %2d\n",sat,eph.sat);
        return -1;
    }
    if (!strstr(raw->opt,"-EPHALL")) {
        if (eph.iode==raw->nav.eph.iode&& /* unchanged */
            timediff(eph.toe,raw->nav.eph.toe)==0.0&&
            timediff(eph.toc,raw->nav.eph.toc)==0.0) return 0;
    }
    eph.sat=sat;
    raw->nav.eph=eph;
    raw->ephsat=sat;
    return 2;
}
/* decode beidou navigation data ---------------------------------------------*/
static int decode_cnav(raw_t *raw, int sat, int off)



{
    eph_t eph={0};
    unsigned int words[10];
    int i,id,pgn,prn;
    unsigned char *p=raw->buff+6+off;
    
    if (raw->len<48+off) {
        trace(2,"ubx rawsfrbx length error: sat=%d len=%d\n",sat,raw->len);
        return -1;
    }
    for (i=0;i<10;i++,p+=4) words[i]=U4(p)&0x3FFFFFFF; /* 30 bits */
    
    satsys(sat,&prn);
    id=(words[0]>>12)&0x07; /* subframe id (3bit) */
    if (id<1||5<id) {
        trace(2,"ubx rawsfrbx subfrm id error: sat=%2d\n",sat);
        return -1;
    }
    if (prn>5) { /* IGSO/MEO */
        
        for (i=0;i<10;i++) {
            setbitu(raw->subfrm[sat-1]+(id-1)*38,i*30,30,words[i]);
        }
        if (id!=3) return 0;
        
        /* decode beidou D1 ephemeris */
        if (!decode_bds_d1(raw->subfrm[sat-1],&eph)) return 0;
    }
    else { /* GEO */
        if (id!=1) return 0;
        
        /* subframe 1 */
        pgn=(words[1]>>14)&0x0F; /* page number (4bit) */
        if (pgn<1||10<pgn) {
            trace(2,"ubx rawsfrbx page number error: sat=%2d\n",sat);
            return -1;
        }
        for (i=0;i<10;i++) {
            setbitu(raw->subfrm[sat-1]+(pgn-1)*38,i*30,30,words[i]);
        }
        if (pgn!=10) return 0;
        
        /* decode beidou D2 ephemeris */
        if (!decode_bds_d2(raw->subfrm[sat-1],&eph)) return 0;
    }
    if (!strstr(raw->opt,"-EPHALL")) {
        if (timediff(eph.toe,raw->nav.eph.toe)==0.0&&
            eph.iode==raw->nav.eph.iode&&
            eph.iodc==raw->nav.eph.iodc) return 0; /* unchanged */
    }

    eph.sat=sat;
    raw->nav.eph=eph;
    raw->ephsat=sat;
    return 2;
}
/* decode glonass navigation data --------------------------------------------*/
static int decode_gnav(raw_t *raw, int sat, int off, int frq)
{
    geph_t geph={0};
    int i,j,k,m,prn;
    unsigned char *p=raw->buff+6+off,buff[64],*fid;
    
    satsys(sat,&prn);
    
    if (raw->len<24+off) {
        trace(2,"ubx rawsfrbx gnav length error: len=%d\n",raw->len);
        return -1;
    }
    for (i=k=0;i<4;i++,p+=4) for (j=0;j<4;j++) {
        buff[k++]=p[3-j];
    }
    /* test hamming of glonass string */
    if (!test_glostr(buff)) {
        trace(2,"ubx rawsfrbx glo string hamming error: sat=%2d\n",sat);
        return -1;
    }
    m=getbitu(buff,1,4);
    if (m<1||15<m) {
        trace(2,"ubx rawsfrbx glo string no error: sat=%2d\n",sat);
        return -1;
    }
    /* flush frame buffer if frame-id changed */
    fid=raw->subfrm[sat-1]+150;
    if (fid[0]!=buff[12]||fid[1]!=buff[13]) {
        for (i=0;i<4;i++) memset(raw->subfrm[sat-1]+i*10,0,10);
        memcpy(fid,buff+12,2); /* save frame-id */
    }
    memcpy(raw->subfrm[sat-1]+(m-1)*10,buff,10);
    
    if (m!=4) return 0;
    
    /* decode glonass ephemeris strings */
    geph.tof=raw->time;
    if (!decode_glostr(raw->subfrm[sat-1],&geph)||geph.sat!=sat) return 0;
    geph.frq=frq-7;
    
    if (!strstr(raw->opt,"-EPHALL")) {
        if (geph.iode==raw->nav.geph.iode) return 0; /* unchanged */
    }
    raw->nav.geph=geph;
    raw->ephsat=sat;
    return 2;
}

/* decode ubx-rxm-sfrbx: raw subframe data (ref [3]) -------------------------*/
static int decode_rxmsfrbx(raw_t *raw)
{
    int prn,sat,sys;
    unsigned char *p=raw->buff+6;
    
    trace(4,"decode_rxmsfrbx: len=%d\n",raw->len);
    

    if (!(sys=ubx_sys(U1(p)))) {
        trace(2,"ubx rxmsfrbx sys id error: sys=%d\n",U1(p));
        return -1;
    }
    prn=U1(p+1)+(sys==SYS_QZS?192:0);
    if (!(sat=satno(sys,prn))) {
        trace(2,"ubx rxmsfrbx sat number error: sys=%d prn=%d\n",sys,prn);
        return -1;
    }
    switch (sys) {
        case SYS_GPS: return decode_nav (raw,sat,8);
        case SYS_QZS: return decode_nav (raw,sat,8);
        case SYS_GAL: return decode_enav(raw,sat,8);
        case SYS_CMP: return decode_cnav(raw,sat,8);
        case SYS_GLO: return decode_gnav(raw,sat,8,U1(p+3));

    }
    return 0;
}

/* decode ublox raw message --------------------------------------------------*/
static int decode_ubx(raw_t *raw)
{
    int type=(U1(raw->buff+2)<<8)+U1(raw->buff+3);
    
    trace(3,"decode_ubx: type=%04x len=%d\n",type,raw->len);
    
    /* checksum */
    if (!checksum(raw->buff,raw->len)) {
        trace(2,"ubx checksum error: type=%04x len=%d\n",type,raw->len);
        return -1;
    }
    switch (type) {
        case ID_RXMRAWX : 
			return decode_rxmrawx (raw);
        case ID_RXMSFRBX: 
			return decode_rxmsfrbx(raw);

    }

    return 0;
}
/* sync code -----------------------------------------------------------------*/
static int sync_ubx(unsigned char *buff, unsigned char data)
{
    buff[0]=buff[1]; buff[1]=data;
    return buff[0]==UBXSYNC1&&buff[1]==UBXSYNC2;
}
/* input ublox raw message from stream -----------------------------------------
* fetch next ublox raw data and input a mesasge from stream
* args   : raw_t *raw   IO     receiver raw data control struct
*          unsigned char data I stream data (1 byte)
* return : status (-1: error message, 0: no message, 1: input observation data,
*                  2: input ephemeris, 3: input sbas message,
*                  9: input ion/utc parameter)
*
* notes  : to specify input options, set raw->opt to the following option
*          strings separated by spaces.
*
*          -EPHALL    : input all ephemerides
*          -INVCP     : invert polarity of carrier-phase
*          -TADJ=tint : adjust time tags to multiples of tint (sec)
*          -STD_SLIP=std: slip by std-dev of carrier phase under std
*
*          The supported messages are as follows.
*
*          UBX-RXM-RAW  : raw measurement data
*          UBX-RXM-RAWX : multi-gnss measurement data
*          UBX-RXM-SFRB : subframe buffer
*          UBX-RXM-SFRBX: subframe buffer extension
*
*          UBX-TRK-MEAS and UBX-TRK-SFRBX are based on NEO-M8N (F/W 2.01).
*          UBX-TRK-D5 is based on NEO-7N (F/W 1.00). They are not formally
*          documented and not supported by u-blox.
*          Users can use these messages by their own risk.
*-----------------------------------------------------------------------------*/
extern int input_ubx(raw_t *raw, unsigned char data)
{
    trace(5,"input_ubx: data=%02x\n",data);
    
    /* synchronize frame */
    if (raw->nbyte==0) {
        if (!sync_ubx(raw->buff,data)) return 0;
        raw->nbyte=2;
        return 0;
    }
    raw->buff[raw->nbyte++]=data;
    
    if (raw->nbyte==6) {
        if ((raw->len=U2(raw->buff+4)+8)>MAXRAWLEN) {
            trace(2,"ubx length error: len=%d\n",raw->len);
            raw->nbyte=0;
            return -1;
        }
    }
    if (raw->nbyte<6||raw->nbyte<raw->len) return 0;
    raw->nbyte=0;
    printf("-------\n");
    /* decode ublox raw message */
    return decode_ubx(raw);
}

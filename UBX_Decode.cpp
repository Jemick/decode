// UBX_Decode.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include "rtklib.h"

typedef struct {        /* receiver raw data control type */
	gtime_t time;       /* message time */
	//    gtime_t tobs[MAXSAT][NFREQ+NEXOBS]; /* observation data time */
	//    sta_t sta;          /* station parameters */
	int ephsat;         /* sat number of update ephemeris (0:no satellite) */
	//    sbsmsg_t sbsmsg;    /* SBAS message */
	char msgtype[256];  /* last message type */
	unsigned char subfrm[MAXSAT][380];  /* subframe buffer */
	//    lexmsg_t lexmsg;    /* LEX message */
	//    double lockt[MAXSAT][NFREQ+NEXOBS]; /* lock time (s) */
	//    double icpp[MAXSAT],off[MAXSAT],icpc; /* carrier params for ss2 */
	//    double prCA[MAXSAT],dpCA[MAXSAT]; /* L1/CA pseudrange/doppler for javad */
	//    unsigned char halfc[MAXSAT][NFREQ+NEXOBS]; /* half-cycle add flag */
	//    char freqn[MAXOBS]; /* frequency number for javad */
	int nbyte;          /* number of bytes in message buffer */ 
	int len;            /* message length (bytes) */
	int iod;            /* issue of data */
	int tod;            /* time of day (ms) */
	int tbase;          /* time base (0:gpst,1:utc(usno),2:glonass,3:utc(su) */
	int flag;           /* general purpose flag */
	int outtype;        /* output message type */
	unsigned char buff[MAXRAWLEN]; /* message buffer */
	char opt[256];      /* receiver dependent options */
//	half_cyc_t *half_cyc; /* half-cycle correction list */

	int format;         /* receiver stream format */
	void *rcv_data;     /* receiver dependent data */
} raw_t2;


int _tmain(int argc, _TCHAR* argv[])
{
	int iii = MAXSAT;
	if (true)
	{
		FILE* f;
		raw_t raw;
		char data,data2;
		int ii;
		unsigned char* buf = new unsigned char [3602474];

		printf("%05d\n", sizeof(raw_t));

		printf("%02d   %02d\n", sizeof(raw_t) - sizeof(nav_t), MAXSAT);

		init_raw(&raw, STRFMT_UBX);

		f = fopen("F:\\TEMP\\uwb", "rb");

		for (ii = 0; ii < 3541410; ii++)
		{
			fread(&data, 1, 1, f);

			input_ubx(&raw, data);


			printf("%010d\n", ii);
		}

		return -1;
	}
	return 0;
}


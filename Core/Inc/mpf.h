#include "stdbool.h"
#include "socket.h"
#include "dhcp.h"
#include "dns.h"
#include "stdlib.h"
#include <stdarg.h>
#include <stdbool.h>
#include <ctype.h>
#include "string.h"
#include "math.h"
#include "stdio.h"
#include "stdint-gcc.h"

#define MAX_RECV_LEN 512
#define MAX_RECV_LEN_S 8
#define DHCP_SOCKET     0
#define DNS_SOCKET      1
#define HTTP_SOCKET     2
volatile bool ip_assigned = false;
#define SOCK_TCPS        0
#define SOCK_TCPS1       0
#define DATA_BUF_SIZE   512
uint8_t gDATABUF[DATA_BUF_SIZE]={0};
uint8_t hDATABUF[DATA_BUF_SIZE]={0};
uint8_t kDATABUF[DATA_BUF_SIZE]={0};
// Default Network Configuration
wiz_NetInfo gWIZNETINFO = { .mac = {0x00, 0x08, 0xdc,0x11, 0x11, 0x11},
                            .ip = {192, 168, 30, 16},
                            .sn = {255,255,255,0},
                            .gw = {192, 168, 30, 1},
                            .dns = {8,8,8,8},
                            .dhcp = NETINFO_STATIC };

uint8_t 	tmp;
uint16_t 	len=0,len1=0;
uint8_t 	memsize[2][8] = {{2,2,2,2,2,2,2,2},{2,2,2,2,2,2,2,2}};
uint8_t 	DstIP[4]={192,168,30,215}; //215
uint16_t	DstPort=5000;
/* MPF Data */


bool flat=true;
bool flat1=true;
bool flat2=true;
bool flat3=true;
bool flat4=true;
bool flat5=true;
bool flat6=true;
bool flat7=true;
char buff1[20],buff[20],buf1[20],buf2[20],arr1[20],buff1[20],buff2[20],buff3[20],buff4[20];
char aRxBuffer1[20],aRxBuffer_new[20];
char msg_buff[50] = {0};
char msg1_buff[MAX_RECV_LEN_S] = {0};
char msg2_buff[10] = {0};
char msg3_buff[MAX_RECV_LEN] ={'0'};
char msg4_buff[20] = {0};
char *msg = msg_buff;
char *msg1 = msg1_buff;
char *msg3 = msg3_buff;
struct template
{
	int qei;
	int tqv;
	int uart4;
	int usart2;
	int recv;
	int timer;
};
struct template Flag;

struct template2
{
	long size;
	double max;
	double min;
	double z;
	int counter;
	int m;
};
struct template2 Tem;
int current;
int temp_array=0;
char ar[]=":MEASure?\n";
char ar1[]=":MEASure:CURRent?\n";
double arr[300]={0};
int pos =13, pos1 = 20;
char rfid[12], cur[6], cur1[3], cur2[3], cur3[3], cur4[4], cur5[4], cur6[4];
char rfidd[50], curr[20], cur11[20], cur22[20], cur33[20], cur44[20], cur55[20];
int com1[10], com2[10], com3[10], com4[10];
char max1[4]={0},min1[4]={0};
int temp_time2=0, temp_time4=0, temp_time15=0;
char time_spent[5] ={0};
int t1=0,t2=0,t3=0,t4=0;

/* Initial information of MPF Motor*/

typedef struct
{
    uint32_t  id;
    uint8_t  voltageLow;
    uint8_t   voltageHigh;
    uint8_t   powerLevel;
    uint8_t   speedSensorError;
    uint8_t   crankRPMlow;
    uint8_t   crankRPMhigh;
    uint8_t   errorCode;
    uint8_t   highTemperature;
}DATA_302_T;

typedef struct
{
    uint32_t  id;
    uint8_t   torqueLow;
    uint8_t   torqueHigh;
    uint8_t   motorRPMlow;
    uint8_t   motorRPMhigh;
    uint8_t   motorTemperatureLow;
    uint8_t   motorTemperatureHigh;
    uint8_t   cw;
    uint8_t   ccw;
}DATA_402_T;

typedef struct
{
	uint16_t motoCrankRPM;
	double motoTorque;
}NB_IOT_DATA;

typedef struct
{
    DATA_302_T    data302;
    DATA_402_T    data402;
    NB_IOT_DATA     nbiotData;
}MPF_DATA_T;



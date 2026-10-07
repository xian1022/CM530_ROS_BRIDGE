/* Production bridge + SDK: only the physical HAL is simulated. */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "bridge.h"
#include "dynamixel.h"
static char output[4096];
static int ledMoving[2], ledCalls;
static unsigned char packets[128][160], rx[80];
static int lengths[128], packetCount, rxSize, rxUsed, polls;
static int failTxAt, badId, fault, chunk, emptyFirst, failures, sdkTest;
static const int ids[2][4]={{17,3,2,15},{12,1,8,16}};
enum { GOOD, NO_REPLY, WRONG_ID, SHORT_LENGTH, LONG_LENGTH, BAD_CHECKSUM,
       MOTOR_ERROR, OUT_OF_RANGE, TRUNCATED, HAL_ERROR, ZERO_LENGTH };
#define CHECK(c) do { if(!(c)){fprintf(stderr,"FAIL line %d: %s\n",__LINE__,#c);failures++;} } while(0)
void BridgeSetArmLed(int arm,int moving){CHECK(arm>=0 && arm<2);CHECK(moving==0 || moving==1);ledMoving[arm]=moving;ledCalls++;}
void BridgeOutput(const char *s){if(strlen(output)+strlen(s)>=sizeof(output))abort();strcat(output,s);}
int dxl_hal_open(int d,int b){(void)d;(void)b;return 1;}
void dxl_hal_close(void){}
void dxl_hal_clear(void){rxSize=rxUsed=0;}
void dxl_hal_set_timeout(int bytes){(void)bytes;polls=0;}
int dxl_hal_timeout(void){return ++polls>100;}
int dxl_hal_rx(unsigned char *p,int n)
{
    int take;
    if(fault==HAL_ERROR)return -1;
    if(emptyFirst){emptyFirst=0;return 0;}
    take=rxSize-rxUsed;if(take>n)take=n;if(chunk && take>chunk)take=chunk;
    if(take<0 || n<0)abort();
    memcpy(p,rx+rxUsed,take);rxUsed+=take;return take;
}
int dxl_hal_tx(unsigned char *p,int n)
{
    int k,mode,value;unsigned char sum=0;
    if(packetCount>=128 || n>160)abort();
    memcpy(packets[packetCount],p,n);lengths[packetCount++]=n;
    CHECK(n==p[3]+4 && p[0]==255 && p[1]==255);
    for(k=2;k<n;k++)sum+=p[k];CHECK(sum==255);
    if(!sdkTest && p[4]!=INST_READ){
        int arm=p[7]==17?0:1, width=p[5]==30?2:1;
        CHECK(p[2]==254 && p[4]==INST_SYNC_WRITE && (p[5]==24 || p[5]==30));
        CHECK(p[6]==width && n==8+4*(width+1));
        for(k=0;k<4;k++)CHECK(p[7+(width+1)*k]==ids[arm][k]);
    }
    if(packetCount==failTxAt)return 0;
    rxUsed=rxSize=0;
    if(p[4]!=INST_READ)return n;
    CHECK(n==8 && p[5]==36 && p[6]==2);
    mode=p[2]==badId?fault:GOOD;if(mode==NO_REPLY)return n;
    value=400+p[2];if(mode==OUT_OF_RANGE)value=1024;
    rx[0]=255;rx[1]=255;rx[2]=(unsigned char)(p[2]+(mode==WRONG_ID));
    rx[3]=4;rx[4]=mode==MOTOR_ERROR?4:0;
    rx[5]=(unsigned char)value;rx[6]=(unsigned char)(value>>8);rxSize=8;
    if(mode==SHORT_LENGTH){rx[3]=2;rxSize=6;}
    if(mode==LONG_LENGTH)rx[3]=255;
    if(mode==ZERO_LENGTH)rx[3]=0;
    sum=0;for(k=2;k<rxSize-1;k++)sum+=rx[k];rx[rxSize-1]=(unsigned char)~sum;
    if(mode==BAD_CHECKSUM)rx[rxSize-1]^=1;
    if(mode==TRUNCATED)rxSize=5;
    return n;
}
static void feed(const char *s){while(*s)BridgeFeed((unsigned char)*s++);}
static void command(const char *s,const char *want)
{
    output[0]=0;feed(s);feed("\n");
    if(strcmp(output,want)){fprintf(stderr,"FAIL %s: want [%s], got [%s]\n",s,want,output);failures++;}
}
static void reject(const char *s,const char *want)
{int before=packetCount;command(s,want);CHECK(packetCount==before);}
static void syncPacket(int index,int arm,int address,int a,int b,int c,int d)
{
    int j,width=address==30?2:1,values[4];unsigned char *p=packets[index];
    values[0]=a;values[1]=b;values[2]=c;values[3]=d;
    CHECK(p[2]==254 && p[4]==INST_SYNC_WRITE && p[5]==address && p[6]==width);
    CHECK(lengths[index]==8+4*(width+1));
    for(j=0;j<4;j++){
        CHECK(p[7+(width+1)*j]==ids[arm][j]);
        CHECK(p[8+(width+1)*j]+(width==2?256*p[9+(width+1)*j]:0)==values[j]);
    }
}
static void reset(void)
{
    packetCount=rxSize=rxUsed=polls=0;failTxAt=badId=-1;
    ledMoving[0]=ledMoving[1]=-1;ledCalls=0;
    fault=GOOD;chunk=emptyFirst=sdkTest=0;output[0]=0;dxl_initialize(0,1);BridgeInit();
}
static void boot(void)
{
    reset();CHECK(!strcmp(output,"READY,5\r\n"));CHECK(packetCount==2);
    syncPacket(0,0,24,0,0,0,0);syncPacket(1,1,24,0,0,0,0);
    CHECK(ledMoving[0]==0 && ledMoving[1]==0 && ledCalls==2);
}
static void test_boot_torque(void)
{
    int i;
    boot();command("VERSION","VERSION,5\r\n");command("PING","PONG\r\n");
    reject("STOP,arm1","ERR,BAD_CMD,arm1\r\n");
    reject("TORQUE,arm1,1","ERR,NO_TARGET,arm1\r\n");
    command("AX,arm1,0,511,512,1023","OK,AX,arm1\r\n");
    CHECK(packetCount==3);syncPacket(2,0,30,0,511,512,1023);
    command("TORQUE,arm1,1","OK,TORQUE,arm1,1\r\n");syncPacket(3,0,24,1,1,1,1);
    reject("TORQUE,arm2,1","ERR,NO_TARGET,arm2\r\n");
    command("AX,arm2,512,512,512,512","OK,AX,arm2\r\n");syncPacket(4,1,30,512,512,512,512);
    command("TORQUE,arm1,0","OK,TORQUE,arm1,0\r\n");syncPacket(5,0,24,0,0,0,0);
    reject("TORQUE,arm1,1","ERR,NO_TARGET,arm1\r\n");
    command("TORQUE,arm2,1","OK,TORQUE,arm2,1\r\n");
    failTxAt=packetCount+1;command("AX,arm1,512,512,512,512","ERR,DXL_TX,arm1\r\n");
    reject("TORQUE,arm1,1","ERR,NO_TARGET,arm1\r\n");
    command("AX,arm1,512,512,512,512","OK,AX,arm1\r\n");
    failTxAt=packetCount+1;command("TORQUE,arm1,0","ERR,DXL_TX,arm1\r\n");
    reject("TORQUE,arm1,1","ERR,NO_TARGET,arm1\r\n");
    reject("TORQUE,arm1,2","ERR,BAD_ARG,arm1\r\n");
    reject("HOLD,arm1,0","ERR,BAD_ARG,arm1\r\n");
    for(i=1;i<=2;i++){
        reset();packetCount=0;output[0]=0;failTxAt=i;BridgeInit();
        CHECK(!strcmp(output,"ERR,INIT_FAILED\r\n"));CHECK(packetCount==2);
        reject("AX,arm1,512","ERR,INIT_FAILED,arm1\r\n");
        reject("AX,arm2,512,512,512,512","ERR,INIT_FAILED,arm2\r\n");
        reject("TORQUE,arm1,0","ERR,INIT_FAILED,arm1\r\n");
        command("PING","PONG\r\n");command("VERSION","VERSION,5\r\n");
    }
}
/* Removed commands must never reach the bus, nor alter either target. */
static void test_direct(void)
{
    int i;
    const char *removed[]={"BEGIN,arm1,7,4,2",
        "PT,arm1,0,300,512,512,512,512","END,arm1,7","STOP,arm1"};
    boot();command("AX,arm1,500,501,502,503","OK,AX,arm1\r\n");
    command("AX,arm2,600,601,602,603","OK,AX,arm2\r\n");
    syncPacket(2,0,30,500,501,502,503);syncPacket(3,1,30,600,601,602,603);
    for(i=0;i<4;i++)reject(removed[i],"ERR,BAD_CMD,arm1\r\n");
    reject("AX,arm1,512","ERR,BAD_ARG,arm1\r\n");
    reject("AX,arm1,512,512,512,1024","ERR,RANGE,arm1\r\n");
    reject("TORQUE,arm1,2","ERR,BAD_ARG,arm1\r\n");
    command("TORQUE,arm1,1","OK,TORQUE,arm1,1\r\n");
    failTxAt=packetCount+1;
    command("AX,arm1,510,511,512,513","ERR,DXL_TX,arm1\r\n");
    reject("TORQUE,arm1,1","ERR,NO_TARGET,arm1\r\n");
    failTxAt=packetCount+1;command("TORQUE,arm2,1","ERR,DXL_TX,arm2\r\n");
    command("TORQUE,arm2,1","OK,TORQUE,arm2,1\r\n");
    command("AX,arm1,510,511,512,513","OK,AX,arm1\r\n");
    command("TORQUE,arm1,1","OK,TORQUE,arm1,1\r\n");
    /* Every new goal, including an identical goal, is sent directly. */
    i=packetCount;command("AX,arm1,510,511,512,513","OK,AX,arm1\r\n");
    CHECK(packetCount==i+1);syncPacket(i,0,30,510,511,512,513);
    BridgeInit();reject("TORQUE,arm1,1","ERR,NO_TARGET,arm1\r\n");
    reject("TORQUE,arm2,1","ERR,NO_TARGET,arm2\r\n");
}
/* Keep regression coverage of the unchanged generic SDK receiver. These are
 * Direct SDK tests supplement the READ/HOLD command-level tests below. */
static void test_sdk(void)
{
    int mode,value;
    for(mode=NO_REPLY;mode<=ZERO_LENGTH;mode++){
        boot();sdkTest=1;badId=17;fault=mode;
        value=dxl_read_word(17,36);(void)value;
        if(mode==NO_REPLY)CHECK(dxl_get_result()==COMM_RXTIMEOUT);
        else if(mode==MOTOR_ERROR){CHECK(dxl_get_result()==COMM_RXSUCCESS);CHECK(dxl_get_rxpacket_error(4));}
        else if(mode==OUT_OF_RANGE){CHECK(dxl_get_result()==COMM_RXSUCCESS);CHECK(value==1024);}
        else if(mode==HAL_ERROR)CHECK(dxl_get_result()==COMM_RXFAIL);
        else CHECK(dxl_get_result()==COMM_RXCORRUPT);
    }
    boot();sdkTest=1;chunk=1;emptyFirst=1;
    CHECK(dxl_read_word(17,36)==417);CHECK(dxl_get_result()==COMM_RXSUCCESS);
    failTxAt=packetCount+1;dxl_read_word(17,36);CHECK(dxl_get_result()==COMM_TXFAIL);
}
static void test_led(void)
{
    int calls,i;
    const char *bad[]={"LED,arm1","LED,arm1,0","LED,arm1,STOP","LED,arm1,",
        "LED,arm1,MOVING,extra","LED,arm1,MOVINGjunk"};
    boot();reject("LED,arm1,MOVING","OK,LED,arm1,MOVING\r\n");
    CHECK(ledMoving[0]==1 && ledMoving[1]==0);
    reject(" led , ARM2 , moving ","OK,LED,arm2,MOVING\r\n");
    CHECK(ledMoving[0]==1 && ledMoving[1]==1);
    reject("LED,arm1,STOPPED","OK,LED,arm1,STOPPED\r\n");
    CHECK(ledMoving[0]==0 && ledMoving[1]==1);
    reject("LED,arm2,stopped","OK,LED,arm2,STOPPED\r\n");
    reject("LED,arm2,STOPPED","OK,LED,arm2,STOPPED\r\n");
    CHECK(ledMoving[0]==0 && ledMoving[1]==0);
    calls=ledCalls;
    for(i=0;i<6;i++)reject(bad[i],"ERR,BAD_ARG,arm1\r\n");
    reject("LED,arm3,MOVING","ERR,BAD_ARG\r\n");
    reject("LED","ERR,BAD_ARG\r\n");
    CHECK(ledCalls==calls);
    reject("TORQUE,arm1,1","ERR,NO_TARGET,arm1\r\n");
    command("AX,arm1,500,501,502,503","OK,AX,arm1\r\n");
    reject("LED,arm1,MOVING","OK,LED,arm1,MOVING\r\n");calls=ledCalls;
    command("TORQUE,arm1,1","OK,TORQUE,arm1,1\r\n");
    command("TORQUE,arm1,0","OK,TORQUE,arm1,0\r\n");
    failTxAt=packetCount+1;command("AX,arm1,500,501,502,503","ERR,DXL_TX,arm1\r\n");
    reject("TORQUE,arm1,1","ERR,NO_TARGET,arm1\r\n");
    command("PING","PONG\r\n");CHECK(ledCalls==calls && ledMoving[0]==1);
    for(i=1;i<=2;i++){
        reset();packetCount=0;output[0]=0;failTxAt=i;BridgeInit();
        CHECK(!strcmp(output,"ERR,INIT_FAILED\r\n"));
        CHECK(ledMoving[0]==0 && ledMoving[1]==0);
        reject("LED,arm2,MOVING","OK,LED,arm2,MOVING\r\n");
        CHECK(ledMoving[1]==1);
        reject("AX,arm2,512,512,512,512","ERR,INIT_FAILED,arm2\r\n");
        reject("TORQUE,arm2,0","ERR,INIT_FAILED,arm2\r\n");
        CHECK(ledMoving[1]==1);
    }
}
static void test_input(void)
{
    int i;char longLine[180];
    static const char *bad[]={"AX,A,512,512,512,512","AX,B,512,512,512,512",
        "AX,arm3,512,512,512,512","AX,arm1extra,512,512,512,512","AX,512","AX","TORQUE",
        "AX,arm1,51.2,512,512,512","AX,arm1,,512,512,512","AX,arm1,2147483648,512,512,512",
        "AX,arm1,-2147483649,512,512,512","AX,arm1,4294967808,512,512,512",
        "AX,arm1,512,512,512,512,extra","AX,arm1,1,2,3,4,5,6,7"};
    boot();for(i=0;i<(int)(sizeof(bad)/sizeof(bad[0]));i++){
        int before=packetCount;output[0]=0;feed(bad[i]);feed("\n");
        CHECK(!strncmp(output,"ERR,BAD_ARG",11));CHECK(before==packetCount);
    }
    reject("AX,arm1,-1,512,512,512","ERR,RANGE,arm1\r\n");reject("AX,arm2,1024,512,512,512","ERR,RANGE,arm2\r\n");
    reject("PING,arm1","ERR,BAD_ARG,arm1\r\n");reject("VERSION,arm1","ERR,BAD_ARG,arm1\r\n");
    command(" ax , ArM2 , +512 ,0,1023,500 ","OK,AX,arm2\r\n");
    command("\xEF\xBB\xBF" "AX\xEF\xBC\x8C" "arm1,512,512,512,512","OK,AX,arm1\r\n");
    memset(longLine,'x',150);strcpy(longLine+150,"AX,arm2,999");reject(longLine,"ERR,OVERFLOW\r\n");
    command("PING\r\n","PONG\r\n");i=packetCount;output[0]=0;
    feed("AX,arm2,5");BridgeAbortLine();feed("12\nPING\n");
    CHECK(!strcmp(output,"ERR,OVERFLOW\r\nPONG\r\n"));CHECK(packetCount==i);
    output[0]=0;feed("AX,arm1,512,512,512,512");BridgeFeed(0);feed("\n");
    CHECK(!strcmp(output,"ERR,BAD_ARG,arm1\r\n"));CHECK(packetCount==i);
    for(i=0;i<3;i++){output[0]=0;feed("AX,arm2,");BridgeFeed(0xEF);if(i)BridgeFeed(0xBC);feed("\n");CHECK(!strcmp(output,"ERR,BAD_ARG,arm2\r\n"));}
}

static void test_v17(void)
{
    int arm,j,mode,before;
    const char *codes[]={"", "DXL_TIMEOUT", "DXL_CORRUPT", "DXL_CORRUPT", "DXL_CORRUPT",
        "DXL_CORRUPT", "DXL_MOTOR", "DXL_RANGE", "DXL_CORRUPT", "DXL_RX", "DXL_CORRUPT"};
    char cmd[80],want[120];
    for(arm=0;arm<2;arm++){
        boot();before=packetCount;
        sprintf(cmd,"GET_HOME,arm%d",arm+1);sprintf(want,"HOME,arm%d,512,512,512,512\r\n",arm+1);
        reject(cmd,want);
        sprintf(cmd,"TORQUE,arm%d,1",arm+1);sprintf(want,"ERR,NO_TARGET,arm%d\r\n",arm+1);reject(cmd,want);
        sprintf(cmd,"HOME,arm%d",arm+1);sprintf(want,"OK,HOME,arm%d\r\n",arm+1);command(cmd,want);
        CHECK(packetCount==before+1);syncPacket(before,arm,30,512,512,512,512);
        sprintf(cmd,"TORQUE,arm%d,1",arm+1);sprintf(want,"OK,TORQUE,arm%d,1\r\n",arm+1);command(cmd,want);
        before=packetCount;
        sprintf(cmd,"READ,arm%d",arm+1);sprintf(want,"POS,arm%d,%d,%d,%d,%d\r\n",arm+1,400+ids[arm][0],400+ids[arm][1],400+ids[arm][2],400+ids[arm][3]);command(cmd,want);
        CHECK(packetCount==before+4);
        for(j=0;j<4;j++)CHECK(packets[before+j][2]==ids[arm][j]);
        before=packetCount;
        sprintf(cmd,"HOLD,arm%d",arm+1);sprintf(want,"OK,HOLD,arm%d\r\n",arm+1);command(cmd,want);
        CHECK(packetCount==before+5);syncPacket(before+4,arm,30,400+ids[arm][0],400+ids[arm][1],400+ids[arm][2],400+ids[arm][3]);
    }
    for(mode=NO_REPLY;mode<=ZERO_LENGTH;mode++)for(j=0;j<4;j++){
        boot();command("HOME,arm1","OK,HOME,arm1\r\n");command("HOME,arm2","OK,HOME,arm2\r\n");
        badId=ids[0][j];fault=mode;before=packetCount;
        /* HAL_ERROR affects all reads in the fake. */
        sprintf(want,"ERR,%s,arm1,%d%s\r\n",codes[mode],mode==HAL_ERROR?17:badId,mode==MOTOR_ERROR?",4":"");
        command("HOLD,arm1",want);CHECK(packetCount==before+(mode==HAL_ERROR?1:j+1));
        reject("TORQUE,arm1,1","ERR,NO_TARGET,arm1\r\n");
        command("TORQUE,arm2,1","OK,TORQUE,arm2,1\r\n");
    }
    boot();command("HOME,arm1","OK,HOME,arm1\r\n");badId=17;fault=NO_REPLY;
    command("READ,arm1","ERR,DXL_TIMEOUT,arm1,17\r\n");
    command("TORQUE,arm1,1","OK,TORQUE,arm1,1\r\n"); /* read does not invalidate */
    command("READ,arm2","POS,arm2,412,401,408,416\r\n");
    fault=GOOD;failTxAt=packetCount+1;command("READ,arm1","ERR,DXL_TX,arm1,17\r\n");
    failTxAt=packetCount+5;command("HOLD,arm1","ERR,DXL_TX,arm1\r\n");
    reject("TORQUE,arm1,1","ERR,NO_TARGET,arm1\r\n");
    failTxAt=packetCount+1;command("HOME,arm1","ERR,DXL_TX,arm1\r\n");
    reject("TORQUE,arm1,1","ERR,NO_TARGET,arm1\r\n");
    reject("READ,arm1,0","ERR,BAD_ARG,arm1\r\n");reject("HOME","ERR,BAD_ARG\r\n");
    reject("GET_HOME,ALL","ERR,BAD_ARG\r\n");reject("HOLD,arm2,1","ERR,BAD_ARG,arm2\r\n");
    boot();packetCount=0;failTxAt=1;BridgeInit();
    reject("GET_HOME,arm1","HOME,arm1,512,512,512,512\r\n");
    reject("HOME,arm1","ERR,INIT_FAILED,arm1\r\n");reject("READ,arm1","ERR,INIT_FAILED,arm1\r\n");reject("HOLD,arm1","ERR,INIT_FAILED,arm1\r\n");
}
int main(int argc,char **argv)
{
    if(argc<2 || !strcmp(argv[1],"boot"))test_boot_torque();
    if(argc<2 || !strcmp(argv[1],"direct"))test_direct();
    if(argc<2 || !strcmp(argv[1],"sdk"))test_sdk();
    if(argc<2 || !strcmp(argv[1],"led"))test_led();
    if(argc<2 || !strcmp(argv[1],"input"))test_input();
    if(argc<2 || !strcmp(argv[1],"v17"))test_v17();
    if(failures){fprintf(stderr,"FAIL: %d checks\n",failures);return 1;}
    puts("PASS: protocol 5 boot/torque, direct goals, LED isolation, removed commands, HOME/READ/HOLD, SDK and parser recovery");return 0;
}

/* Runs production bridge.c + dynamixel.c with only the physical HAL mocked. */
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "bridge.h"
#include "dynamixel.h"

static char output[2048];
static unsigned char packet[256];
static int syncCount, failNextSync, torqueCount[256];

void BridgeOutput(const char *text)
{
    assert(strlen(output) + strlen(text) < sizeof(output));
    strcat(output, text);
}

int dxl_hal_open(int device, int baud) { (void)device; (void)baud; return 1; }
void dxl_hal_close(void) {}
void dxl_hal_clear(void) {}
void dxl_hal_set_timeout(int bytes) { (void)bytes; }
int dxl_hal_timeout(void) { return 1; }
int dxl_hal_rx(unsigned char *data, int count) { (void)data; (void)count; return 0; }
int dxl_hal_tx(unsigned char *data, int count)
{
    int i;
    unsigned char sum = 0;
    assert(data[0] == 255 && data[1] == 255 && count == data[3] + 4);
    for (i = 2; i < count; i++) sum += data[i];
    assert(sum == 255);
    if (data[4] == INST_SYNC_WRITE) {
        assert(count == 20 && data[2] == 254 && data[5] == 30 && data[6] == 2);
        memcpy(packet, data, count);
        syncCount++;
        if (failNextSync) { failNextSync = 0; return 0; }
    } else {
        assert(data[4] == INST_WRITE && data[5] == 24 && data[6] == 1);
        torqueCount[data[2]]++;
    }
    return count;
}

static void feed(const char *text)
{
    while (*text) BridgeFeed((unsigned char)*text++);
}

static void command(const char *text, const char *expected)
{
    output[0] = 0;
    feed(text); feed("\n");
    if (strcmp(output, expected)) {
        fprintf(stderr, "%s: expected [%s], got [%s]\n", text, expected, output);
        assert(0);
    }
}

static void target(int arm, int a, int b, int c, int d)
{
    const int ids[2][4] = {{17,3,2,15}, {12,1,8,16}};
    int positions[4], j;
    positions[0] = a; positions[1] = b; positions[2] = c; positions[3] = d;
    for (j = 0; j < 4; j++) {
        assert(packet[7 + 3*j] == ids[arm][j]);
        assert(packet[8 + 3*j] + 256*packet[9 + 3*j] == positions[j]);
    }
}

static void reject(const char *text, const char *expected)
{
    int before = syncCount;
    command(text, expected);
    assert(before == syncCount);
}

int main(void)
{
    int before, i;
    char longLine[200];
    dxl_initialize(0, 1);
    BridgeInit();
    command("PING", "PONG\r\n");
    command("AX,A,0,511,512,1023", "OK,AX,A\r\n"); target(0,0,511,512,1023);
    assert(torqueCount[17] == 1 && torqueCount[12] == 0);
    command("AX,B,100,200,300,400", "OK,AX,B\r\n"); target(1,100,200,300,400);
    command("STOP,A", "OK,STOP,A\r\n"); target(0,0,511,512,1023);
    command("STOP,B", "OK,STOP,B\r\n"); target(1,100,200,300,400);
    command("HOME,A", "OK,HOME,A\r\n"); target(0,512,512,512,512);
    command("STOP,B", "OK,STOP,B\r\n"); target(1,100,200,300,400);
    command("HOME,B", "OK,HOME,B\r\n"); target(1,512,512,512,512);
    assert(torqueCount[17] == 1 && torqueCount[12] == 1);

    command("BEGIN,A,7,4,2", "OK,BEGIN,A,7\r\n");
    command("BEGIN,B,7,4,1", "OK,BEGIN,B,7\r\n");
    command("PT,A,0,300,501,502,503,504", "OK,PT,A,0\r\n"); target(0,501,502,503,504);
    reject("END,A,7", "ERR,BAD_TRAJ,A\r\n");
    reject("END,B,8", "ERR,BAD_TRAJ,B\r\n");
    failNextSync = 1;
    command("PT,B,0,300,600,601,602,603", "ERR,DXL_TX,B\r\n");
    reject("END,B,7", "ERR,BAD_TRAJ,B\r\n");
    command("PT,B,0,300,610,611,612,613", "OK,PT,B,0\r\n"); target(1,610,611,612,613);
    command("STOP,A", "OK,STOP,A\r\n"); target(0,501,502,503,504);
    reject("PT,A,1,300,500,500,500,500", "ERR,BAD_TRAJ,A\r\n");
    command("END,B,7", "OK,END,B,7\r\n");
    reject("PT,B,1,300,500,500,500,500", "ERR,BAD_TRAJ,B\r\n");
    failNextSync = 1;
    command("AX,B,999", "ERR,DXL_TX,B\r\n");
    command("STOP,B", "OK,STOP,B\r\n"); target(1,610,611,612,613);

    reject("AX,512", "ERR,BAD_ARG\r\n");
    reject("HOME", "ERR,BAD_ARG\r\n");
    reject("STOP", "ERR,BAD_ARG\r\n");
    reject("BEGIN,1,4,2", "ERR,BAD_ARG\r\n");
    reject("PT,0,300,512,512,512,512", "ERR,BAD_ARG\r\n");
    reject("END,1", "ERR,BAD_ARG\r\n");
    reject("AX,C,512", "ERR,BAD_ARG\r\n");
    reject("AX,A,-1", "ERR,RANGE,A\r\n");
    reject("AX,B,1024", "ERR,RANGE,B\r\n");
    reject("AX,A,2147483648", "ERR,BAD_ARG,A\r\n");
    reject("AX,A,-2147483649", "ERR,BAD_ARG,A\r\n");
    reject("AX,A,4294967808", "ERR,BAD_ARG,A\r\n");
    reject("AX,A,51.2", "ERR,BAD_ARG,A\r\n");
    reject("AX,A,512,512,512,512,extra", "ERR,BAD_ARG,A\r\n");
    reject("AX,A,", "ERR,BAD_ARG,A\r\n");
    reject("PING,B", "ERR,BAD_ARG,B\r\n");
    reject("HELLO,B", "ERR,BAD_CMD,B\r\n");
    command("BEGIN,B,9,4,1", "OK,BEGIN,B,9\r\n");
    reject("PT,B,0,300,512,512,512,512,extra", "ERR,BAD_ARG,B\r\n");
    reject("PT,B,0,-1,512,512,512,512", "ERR,BAD_TRAJ,B\r\n");
    reject("BEGIN,B,9,8,1", "ERR,BAD_ARG,B\r\n");
    reject("BEGIN,B,9,4,0", "ERR,BAD_ARG,B\r\n");
    command("PT,B,0,0,512,512,512,512", "OK,PT,B,0\r\n");
    reject("PT,B,1,0,512,512,512,512", "ERR,BAD_TRAJ,B\r\n");
    command("END,B,9", "OK,END,B,9\r\n");
    command(" ax , b , +512 ", "OK,AX,B\r\n");
    command("\xEF\xBB\xBF" "AX\xEF\xBC\x8C" "B,512", "OK,AX,B\r\n");

    before = syncCount;
    memset(longLine, 'x', sizeof(longLine));
    longLine[150] = 0;
    strcat(longLine, "AX,B,999");
    command(longLine, "ERR,OVERFLOW\r\n");
    assert(before == syncCount);
    command("PING\r\n", "PONG\r\n");
    output[0] = 0;
    feed("AX,A,5"); BridgeAbortLine(); feed("12AX,B,999\nPING\n");
    assert(!strcmp(output, "ERR,OVERFLOW\r\nPONG\r\n") && before == syncCount);
    output[0] = 0;
    feed("AX,B,512"); BridgeFeed(0); feed(",999\n");
    assert(!strcmp(output, "ERR,BAD_ARG,B\r\n") && before == syncCount);
    /* Short/partial UTF-8 endings must not read past the buffer. */
    for (i = 0; i < 3; i++) {
        output[0] = 0;
        feed("AX,A,"); BridgeFeed(0xEF);
        if (i) BridgeFeed(0xBC);
        feed("\n");
        assert(!strcmp(output, "ERR,BAD_ARG,A\r\n"));
    }
    command("BEGIN,A,-2147483648,4,1", "OK,BEGIN,A,-2147483648\r\n");
    command("PT,A,2147483647,2147483647,512,512,512,512", "OK,PT,A,2147483647\r\n");
    command("END,A,-2147483648", "OK,END,A,-2147483648\r\n");
    BridgeInit();
    command("STOP,B", "OK,STOP,B\r\n"); target(1,512,512,512,512);
    puts("PASS: production C parser + SDK packet mapping, isolated state, failures, malformed input and overflow recovery");
    return 0;
}

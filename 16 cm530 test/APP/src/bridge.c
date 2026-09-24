/* CM530 v16: explicit A/B routing; target ACK, no arrival feedback. */
#include <limits.h>
#include "bridge.h"
#include "dynamixel.h"

typedef struct {
    unsigned char torqueAttempted[BRIDGE_JOINTS];
    unsigned short last[BRIDGE_JOINTS];
    int active, trajId, expected, received;
} ArmState;

static const unsigned char jointIds[BRIDGE_ARMS][BRIDGE_JOINTS] = {
    {17, 3, 2, 15}, {12, 1, 8, 16}
};
static ArmState arms[BRIDGE_ARMS];
static char line[BRIDGE_LINE_SIZE];
static int used, discard;

static int equal(const char *a, const char *b)
{
    while (*a && *a == *b) { a++; b++; }
    return *a == *b;
}

static void upper(char *s)
{
    for (; *s; s++)
        if (*s >= 'a' && *s <= 'z') *s -= 'a' - 'A';
}

static void number(int n)
{
    char buf[12];
    unsigned int magnitude;
    int i = 11;
    buf[i] = 0;
    magnitude = n < 0 ? 0U - (unsigned int)n : (unsigned int)n;
    do { buf[--i] = (char)('0' + magnitude % 10); magnitude /= 10; } while (magnitude);
    if (n < 0) buf[--i] = '-';
    BridgeOutput(buf + i);
}

static void armTag(int arm)
{
    if (arm >= 0) BridgeOutput(arm == 0 ? ",A" : ",B");
}

static void error(const char *code, int arm)
{
    BridgeOutput("ERR,"); BridgeOutput(code); armTag(arm); BridgeOutput("\r\n");
}

static void ok(const char *cmd, int arm, int withValue, int value)
{
    BridgeOutput("OK,"); BridgeOutput(cmd); armTag(arm);
    if (withValue) { BridgeOutput(","); number(value); }
    BridgeOutput("\r\n");
}

/* Signed 32-bit decimal, checked BEFORE multiplication. */
static int integer(const char *s, int *out)
{
    unsigned int value = 0, digit, limit = INT_MAX;
    int negative = 0;
    if (*s == '-' || *s == '+') { negative = *s == '-'; s++; }
    if (!*s) return 0;
    if (negative) limit++;
    for (; *s; s++) {
        if (*s < '0' || *s > '9') return 0;
        digit = (unsigned int)(*s - '0');
        if (value > (limit - digit) / 10U) return 0;
        value = value * 10U + digit;
    }
    *out = negative ? (value == (unsigned int)INT_MAX + 1U ? INT_MIN : -(int)value) : (int)value;
    return 1;
}

/* Preserve unsupported characters: a float/noisy token must fail, not become
 * an apparently valid integer. Only BOM/full-width commas are normalized. */
static void normalize(char *s)
{
    char *dst = s;
    if ((unsigned char)s[0] == 0xEF && (unsigned char)s[1] == 0xBB &&
        (unsigned char)s[2] == 0xBF) s += 3;
    while (*s) {
        if ((unsigned char)s[0] == 0xEF && (unsigned char)s[1] == 0xBC &&
            (unsigned char)s[2] == 0x8C) { *dst++ = ','; s += 3; }
        else *dst++ = *s++;
    }
    *dst = 0;
}

/* Return -1 on extra tokens instead of silently truncating argv. */
static int split(char *s, char **argv, int max)
{
    int count = 0;
    char *start = s, *end;
    for (;;) {
        if (count == max) return -1;
        while (*s && *s != ',') s++;
        end = s;
        while (start < end && (*start == ' ' || *start == '\t')) start++;
        while (end > start && (end[-1] == ' ' || end[-1] == '\t')) end--;
        argv[count++] = start;
        if (!*s) { *end = 0; return count; }
        *s++ = 0; *end = 0; start = s;
    }
}

static void enableTorque(int arm)
{
    int j;
    for (j = 0; j < BRIDGE_JOINTS; j++) {
        if (arms[arm].torqueAttempted[j]) continue;
        dxl_write_byte(jointIds[arm][j], 24, 1);
        /* v15 best-effort policy: an absent status packet is not a motion
         * failure. This records an attempt, not confirmed torque state. */
        arms[arm].torqueAttempted[j] = 1;
    }
}

void BridgeEnableTorque(void)
{
    int arm;
    for (arm = 0; arm < BRIDGE_ARMS; arm++) enableTorque(arm);
}

static int apply(int arm, const unsigned short *pos)
{
    int j, result;
    enableTorque(arm);
    dxl_set_txpacket_id(BROADCAST_ID);
    dxl_set_txpacket_instruction(INST_SYNC_WRITE);
    dxl_set_txpacket_parameter(0, 30);
    dxl_set_txpacket_parameter(1, 2);
    for (j = 0; j < BRIDGE_JOINTS; j++) {
        dxl_set_txpacket_parameter(2 + 3*j, jointIds[arm][j]);
        dxl_set_txpacket_parameter(3 + 3*j, dxl_get_lowbyte(pos[j]));
        dxl_set_txpacket_parameter(4 + 3*j, dxl_get_highbyte(pos[j]));
    }
    dxl_set_txpacket_length(3 * BRIDGE_JOINTS + 4);
    dxl_txrx_packet();
    result = dxl_get_result();
    if (result != COMM_TXSUCCESS && result != COMM_RXSUCCESS) {
        error("DXL_TX", arm); return 0;
    }
    for (j = 0; j < BRIDGE_JOINTS; j++) arms[arm].last[j] = pos[j];
    return 1;
}

static void process(void)
{
    char *argv[8];
    int argc, arm = -1, i, value, id = 0, count, joints, seq = 0, dt;
    unsigned short pos[BRIDGE_JOINTS];
    ArmState *state;
    normalize(line);
    argc = split(line, argv, 8);
    upper(argv[0]);
    if (argc == 1 && !*argv[0]) return;
    if (argc != 1) {
        upper(argv[1]);
        if (equal(argv[1], "A")) arm = 0;
        else if (equal(argv[1], "B")) arm = 1;
    }
    if (argc < 0) { error("BAD_ARG", arm); return; }
    if (equal(argv[0], "PING")) {
        if (argc != 1) error("BAD_ARG", arm);
        else BridgeOutput("PONG\r\n");
        return;
    }
    if (!equal(argv[0], "AX") && !equal(argv[0], "HOME") &&
        !equal(argv[0], "STOP") && !equal(argv[0], "BEGIN") &&
        !equal(argv[0], "PT") && !equal(argv[0], "END")) {
        error("BAD_CMD", arm); return;
    }
    if (arm < 0) { error("BAD_ARG", -1); return; }
    state = &arms[arm];
    if (equal(argv[0], "BEGIN")) {
        if (argc != 5 || !integer(argv[2], &id) || !integer(argv[3], &joints) ||
            !integer(argv[4], &count) || joints != BRIDGE_JOINTS || count <= 0) {
            error("BAD_ARG", arm); return;
        }
        state->active = 1; state->trajId = id; state->expected = count; state->received = 0;
        ok("BEGIN", arm, 1, id); return;
    }
    if (equal(argv[0], "END")) {
        if (argc != 3 || !integer(argv[2], &id)) { error("BAD_ARG", arm); return; }
        if (!state->active || id != state->trajId || state->received != state->expected) {
            error("BAD_TRAJ", arm); return;
        }
        state->active = 0; ok("END", arm, 1, id); return;
    }
    if (equal(argv[0], "HOME") || equal(argv[0], "STOP")) {
        if (argc != 2) { error("BAD_ARG", arm); return; }
        if (equal(argv[0], "STOP")) state->active = 0;
        for (i = 0; i < BRIDGE_JOINTS; i++)
            pos[i] = equal(argv[0], "HOME") ? 512 : state->last[i];
        if (apply(arm, pos)) ok(argv[0], arm, 0, 0);
        return;
    }
    if (equal(argv[0], "PT")) {
        if (argc != 8 || !integer(argv[2], &seq) || !integer(argv[3], &dt)) {
            error("BAD_ARG", arm); return;
        }
        if (!state->active || state->received >= state->expected || dt < 0) {
            error("BAD_TRAJ", arm); return;
        }
        /* v15: seq is echoed; dt is reserved; no arrival wait/interpolation. */
    } else if (argc != 3 && argc != 6) { error("BAD_ARG", arm); return; }
    for (i = 0; i < BRIDGE_JOINTS; i++) {
        int offset = equal(argv[0], "PT") ? 4 + i : (argc == 3 ? 2 : 2 + i);
        if (!integer(argv[offset], &value)) { error("BAD_ARG", arm); return; }
        if (value < 0 || value > 1023) { error("RANGE", arm); return; }
        pos[i] = (unsigned short)value;
    }
    if (!apply(arm, pos)) return;
    if (equal(argv[0], "PT")) { state->received++; ok("PT", arm, 1, seq); }
    else ok("AX", arm, 0, 0);
}

void BridgeInit(void)
{
    int arm, j;
    used = discard = 0;
    for (arm = 0; arm < BRIDGE_ARMS; arm++) {
        arms[arm].active = arms[arm].trajId = arms[arm].expected = arms[arm].received = 0;
        for (j = 0; j < BRIDGE_JOINTS; j++) {
            arms[arm].torqueAttempted[j] = 0;
            arms[arm].last[j] = 512;
        }
    }
}

void BridgeAbortLine(void)
{
    used = 0;
    if (!discard) error("OVERFLOW", -1);
    discard = 1;
}

void BridgeFeed(unsigned char ch)
{
    if (ch == '\r' || ch == '\n') {
        if (!discard && used) { line[used] = 0; process(); }
        used = discard = 0;
        return;
    }
    if (discard) return;
    if (ch == '\b' || ch == 127) { if (used) used--; return; }
    /* Embedded NUL must not hide trailing tokens. */
    if (ch == 0) ch = '?';
    if (used == BRIDGE_LINE_SIZE - 1) { BridgeAbortLine(); return; }
    line[used++] = (char)ch;
}

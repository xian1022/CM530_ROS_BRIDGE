/* CM530 v17 / serial protocol 5. ACK is processing success, never arrival. */
#include <limits.h>
#include "bridge.h"
#include "arm_config.h"

typedef struct {
    int targetValid;
} ArmState;

static ArmState arms[BRIDGE_ARMS];
static char line[BRIDGE_LINE_SIZE];
static int used, discard, initialized;

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
    if (arm >= 0) BridgeOutput(arm == 0 ? ",arm1" : ",arm2");
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

static int apply(int arm, const unsigned short *pos)
{
    /* A failed/partial new write must not authorize an older target. */
    arms[arm].targetValid = 0;
    if (!Ax12WriteGoals(arm, pos)) {
        error("DXL_TX", arm); return 0;
    }
    arms[arm].targetValid = 1;
    return 1;
}

static void positionsReply(const char *tag, int arm, const unsigned short *pos)
{
    int j;
    BridgeOutput(tag); armTag(arm);
    for (j = 0; j < BRIDGE_JOINTS; j++) { BridgeOutput(","); number(pos[j]); }
    BridgeOutput("\r\n");
}

static int readPositions(int arm, unsigned short *pos)
{
    Ax12ReadError detail;
    if (Ax12ReadPositions(arm, pos, &detail)) return 1;
    BridgeOutput("ERR,"); BridgeOutput(detail.code); armTag(arm);
    BridgeOutput(","); number(detail.id);
    if (detail.motorError) { BridgeOutput(","); number(detail.motorError); }
    BridgeOutput("\r\n");
    return 0;
}

static void process(void)
{
    char *argv[8];
    int argc, arm = -1, i, value;
    unsigned short pos[BRIDGE_JOINTS];
    ArmState *state;
    normalize(line);
    argc = split(line, argv, 8);
    upper(argv[0]);
    if (argc == 1 && !*argv[0]) return;
    if (argc != 1) {
        upper(argv[1]);
        if (equal(argv[1], "ARM1")) arm = 0;
        else if (equal(argv[1], "ARM2")) arm = 1;
    }
    if (argc < 0) { error("BAD_ARG", arm); return; }
    if (equal(argv[0], "PING") || equal(argv[0], "VERSION")) {
        if (argc != 1) error("BAD_ARG", arm);
        else BridgeOutput(equal(argv[0], "PING") ? "PONG\r\n" : "VERSION,5\r\n");
        return;
    }
    if (equal(argv[0], "LED")) {
        if (arm < 0 || argc != 3) { error("BAD_ARG", arm); return; }
        upper(argv[2]);
        if (!equal(argv[2], "MOVING") && !equal(argv[2], "STOPPED")) {
            error("BAD_ARG", arm); return;
        }
        /* Display only: allowed even when motor initialization has failed. */
        BridgeSetArmLed(arm, equal(argv[2], "MOVING"));
        BridgeOutput("OK,LED"); armTag(arm);
        BridgeOutput(","); BridgeOutput(argv[2]); BridgeOutput("\r\n");
        return;
    }
    if (equal(argv[0], "GET_HOME")) {
        if (arm < 0 || argc != 2) { error("BAD_ARG", arm); return; }
        positionsReply("HOME", arm, arm == 0 ? home_A : home_C);
        return;
    }
    if (!equal(argv[0], "AX") && !equal(argv[0], "TORQUE") &&
        !equal(argv[0], "HOME") && !equal(argv[0], "READ") && !equal(argv[0], "HOLD")) {
        error("BAD_CMD", arm); return;
    }
    if (arm < 0) { error("BAD_ARG", -1); return; }
    if (!initialized) { error("INIT_FAILED", arm); return; }
    state = &arms[arm];
    if (equal(argv[0], "HOME") || equal(argv[0], "READ") || equal(argv[0], "HOLD")) {
        if (argc != 2) { error("BAD_ARG", arm); return; }
        if (equal(argv[0], "HOME")) {
            const unsigned short *home = arm == 0 ? home_A : home_C;
            for (i = 0; i < BRIDGE_JOINTS; i++)
                if (home[i] > 1023) { error("RANGE", arm); return; }
            if (apply(arm, home)) ok("HOME", arm, 0, 0);
            return;
        }
        /* A failed HOLD must not leave an old goal eligible for re-enable. */
        if (equal(argv[0], "HOLD")) state->targetValid = 0;
        if (!readPositions(arm, pos)) return;
        if (equal(argv[0], "READ")) positionsReply("POS", arm, pos);
        else if (apply(arm, pos)) ok("HOLD", arm, 0, 0);
        return;
    }
    if (equal(argv[0], "TORQUE")) {
        if (argc != 3 || !integer(argv[2], &value) || (value != 0 && value != 1)) {
            error("BAD_ARG", arm); return;
        }
        if (!value) state->targetValid = 0;
        else if (!state->targetValid) { error("NO_TARGET", arm); return; }
        if (!Ax12WriteTorque(arm, value)) { error("DXL_TX", arm); return; }
        ok("TORQUE", arm, 1, value); return;
    }
    if (argc != 6) { error("BAD_ARG", arm); return; }
    for (i = 0; i < BRIDGE_JOINTS; i++) {
        int offset = 2 + i;
        if (!integer(argv[offset], &value)) { error("BAD_ARG", arm); return; }
        if (value < 0 || value > 1023) { error("RANGE", arm); return; }
        pos[i] = (unsigned short)value;
    }
    if (!apply(arm, pos)) return;
    ok("AX", arm, 0, 0);
}

void BridgeInit(void)
{
    int arm;
    used = discard = 0;
    initialized = 1;
    for (arm = 0; arm < BRIDGE_ARMS; arm++) BridgeSetArmLed(arm, 0);
    for (arm = 0; arm < BRIDGE_ARMS; arm++) {
        arms[arm].targetValid = 0;
        /* Always try both arms, even if the first transmission fails. */
        if (!Ax12WriteTorque(arm, 0)) initialized = 0;
    }
    BridgeOutput(initialized ? "READY,5\r\n" : "ERR,INIT_FAILED\r\n");
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

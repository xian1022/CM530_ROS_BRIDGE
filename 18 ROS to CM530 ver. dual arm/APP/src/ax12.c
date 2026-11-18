#include "ax12.h"
#include "dynamixel.h"

static const unsigned char jointIds[AX12_ARMS][AX12_JOINTS] = {
    {17, 3, 2, 15}, {12, 1, 8, 16}
};

/* width 1: one byte per motor; 4: two consecutive words per motor (Goal
 * Position 30-31, then Moving Speed 32-33). values holds width/2 words per
 * joint, or one value for width 1. */
static int sendSync(int arm, int address, int width, const unsigned short *values)
{
    int j, k, offset, result, words = width == 4 ? 2 : 1;
    unsigned short value;
    dxl_set_txpacket_id(BROADCAST_ID);
    dxl_set_txpacket_instruction(INST_SYNC_WRITE);
    dxl_set_txpacket_parameter(0, address);
    dxl_set_txpacket_parameter(1, width);
    for (j = 0; j < AX12_JOINTS; j++) {
        offset = 2 + (width + 1) * j;
        dxl_set_txpacket_parameter(offset, jointIds[arm][j]);
        for (k = 0; k < words; k++) {
            value = values[j * words + k];
            dxl_set_txpacket_parameter(offset + 1 + 2 * k, dxl_get_lowbyte(value));
            if (width >= 2)
                dxl_set_txpacket_parameter(offset + 2 + 2 * k, dxl_get_highbyte(value));
        }
    }
    dxl_set_txpacket_length((width + 1) * AX12_JOINTS + 4);
    dxl_txrx_packet();
    result = dxl_get_result();
    return result == COMM_TXSUCCESS || result == COMM_RXSUCCESS;
}

/* One SYNC_WRITE sets goal and speed together, so a motor never starts a new
 * goal at a stale speed. */
int Ax12WriteGoals(int arm, const unsigned short *positions, const unsigned short *speeds)
{
    unsigned short values[2 * AX12_JOINTS];
    int j;
    for (j = 0; j < AX12_JOINTS; j++) {
        values[2 * j] = positions[j];
        values[2 * j + 1] = speeds[j];
    }
    return sendSync(arm, 30, 4, values);
}

int Ax12WriteTorque(int arm, int enabled)
{
    unsigned short values[AX12_JOINTS];
    int j;
    for (j = 0; j < AX12_JOINTS; j++) values[j] = (unsigned short)enabled;
    return sendSync(arm, 24, 1, values);
}

/* Sequential unicast reads share the official SDK bus. Never use a returned
 * word until transport and the motor's status error byte have been checked. */
int Ax12ReadPositions(int arm, unsigned short *positions, Ax12ReadError *error)
{
    unsigned short sample[AX12_JOINTS];
    int j, bit, value, result, flags;
    for (j = 0; j < AX12_JOINTS; j++) {
        error->id = jointIds[arm][j];
        error->motorError = 0;
        value = dxl_read_word(error->id, 36);
        result = dxl_get_result();
        if (result != COMM_RXSUCCESS) {
            switch (result) {
            case COMM_TXFAIL: case COMM_TXERROR: error->code = "DXL_TX"; break;
            case COMM_RXTIMEOUT: error->code = "DXL_TIMEOUT"; break;
            case COMM_RXCORRUPT: error->code = "DXL_CORRUPT"; break;
            default: error->code = "DXL_RX"; break;
            }
            return 0;
        }
        flags = 0;
        for (bit = 1; bit <= 128; bit <<= 1)
            if (dxl_get_rxpacket_error(bit)) flags |= bit;
        if (flags) {
            error->code = "DXL_MOTOR"; error->motorError = flags; return 0;
        }
        if (value < 0 || value > 1023) { error->code = "DXL_RANGE"; return 0; }
        sample[j] = (unsigned short)value;
    }
    for (j = 0; j < AX12_JOINTS; j++) positions[j] = sample[j];
    return 1;
}

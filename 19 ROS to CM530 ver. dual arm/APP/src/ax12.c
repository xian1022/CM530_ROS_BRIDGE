#include "ax12.h"
#include "dynamixel.h"

static const unsigned char jointIds[AX12_ARMS][AX12_JOINTS] = {
    {17, 3, 2, 15}, {12, 1, 8, 16}
};

/* width 1: one byte per motor; 2: one word per motor (Punch 48-49); 4: two
 * consecutive words per motor (Goal Position 30-31, then Moving Speed 32-33,
 * or Compliance Margin 26-27, then Slope 28-29). values holds width/2 words per
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

int Ax12WriteCompliance(int arm, const unsigned short *margins,
                        const unsigned short *slopes, const unsigned short *punches)
{
    unsigned short values[2 * AX12_JOINTS];
    int j;
    /* Width 4 from 26: CW margin, CCW margin, CW slope, CCW slope. */
    for (j = 0; j < AX12_JOINTS; j++) {
        values[2 * j] = (unsigned short)(margins[j] | (margins[j] << 8));
        values[2 * j + 1] = (unsigned short)(slopes[j] | (slopes[j] << 8));
    }
    if (!sendSync(arm, 26, 4, values)) return 0;
    return sendSync(arm, 48, 2, punches);
}

/* One unicast READ of length bytes from address. Never use the data until
 * transport, the status error byte and the reply length have been checked. */
static int readBlock(int id, int address, int length, unsigned char *data,
                     Ax12ReadError *error)
{
    int i, bit, result, flags;
    error->id = id;
    error->motorError = 0;
    dxl_set_txpacket_id(id);
    dxl_set_txpacket_instruction(INST_READ);
    dxl_set_txpacket_parameter(0, address);
    dxl_set_txpacket_parameter(1, length);
    dxl_set_txpacket_length(4);
    dxl_txrx_packet();
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
    /* The receiver already refuses a READ reply of the wrong length. */
    if (dxl_get_rxpacket_length() != length + 2) { error->code = "DXL_CORRUPT"; return 0; }
    for (i = 0; i < length; i++) data[i] = (unsigned char)dxl_get_rxpacket_parameter(i);
    return 1;
}

static unsigned short word(const unsigned char *data)
{
    return (unsigned short)(data[0] | (data[1] << 8));
}

/* Sequential unicast reads share the official SDK bus. */
int Ax12ReadPositions(int arm, unsigned short *positions, unsigned short *loads,
                      Ax12ReadError *error)
{
    unsigned short sample[AX12_JOINTS], load[AX12_JOINTS];
    unsigned char data[6];
    int j;
    for (j = 0; j < AX12_JOINTS; j++) {
        if (!readBlock(jointIds[arm][j], 36, 6, data, error)) return 0;
        sample[j] = word(data);
        load[j] = word(data + 4);
        if (sample[j] > 1023 || load[j] > 2047) { error->code = "DXL_RANGE"; return 0; }
    }
    for (j = 0; j < AX12_JOINTS; j++) { positions[j] = sample[j]; loads[j] = load[j]; }
    return 1;
}

int Ax12ReadCompliance(int arm, unsigned short *margins, unsigned short *slopes,
                       unsigned short *punches, Ax12ReadError *error)
{
    unsigned short margin[AX12_JOINTS], slope[AX12_JOINTS], punch[AX12_JOINTS];
    unsigned char data[24];
    int j;
    for (j = 0; j < AX12_JOINTS; j++) {
        if (!readBlock(jointIds[arm][j], 26, 24, data, error)) return 0;
        if (data[0] != data[1] || data[2] != data[3]) { error->code = "DXL_ASYM"; return 0; }
        margin[j] = data[0];
        slope[j] = data[2];
        punch[j] = word(data + 22);
        if (punch[j] > 1023) { error->code = "DXL_RANGE"; return 0; }
    }
    for (j = 0; j < AX12_JOINTS; j++) {
        margins[j] = margin[j]; slopes[j] = slope[j]; punches[j] = punch[j];
    }
    return 1;
}

int Ax12ReadDiag(int arm, unsigned short *voltages, unsigned short *temperatures,
                 Ax12ReadError *error)
{
    unsigned short voltage[AX12_JOINTS], temperature[AX12_JOINTS];
    unsigned char data[2];
    int j;
    for (j = 0; j < AX12_JOINTS; j++) {
        if (!readBlock(jointIds[arm][j], 42, 2, data, error)) return 0;
        voltage[j] = data[0];
        temperature[j] = data[1];
    }
    for (j = 0; j < AX12_JOINTS; j++) { voltages[j] = voltage[j]; temperatures[j] = temperature[j]; }
    return 1;
}

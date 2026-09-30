#include "ax12.h"
#include "dynamixel.h"

static const unsigned char jointIds[AX12_ARMS][AX12_JOINTS] = {
    {17, 3, 2, 15}, {12, 1, 8, 16}
};

static int sendSync(int arm, int address, int width, const unsigned short *values)
{
    int j, offset, result;
    dxl_set_txpacket_id(BROADCAST_ID);
    dxl_set_txpacket_instruction(INST_SYNC_WRITE);
    dxl_set_txpacket_parameter(0, address);
    dxl_set_txpacket_parameter(1, width);
    for (j = 0; j < AX12_JOINTS; j++) {
        offset = 2 + (width + 1) * j;
        dxl_set_txpacket_parameter(offset, jointIds[arm][j]);
        dxl_set_txpacket_parameter(offset + 1, dxl_get_lowbyte(values[j]));
        if (width == 2) dxl_set_txpacket_parameter(offset + 2, dxl_get_highbyte(values[j]));
    }
    dxl_set_txpacket_length((width + 1) * AX12_JOINTS + 4);
    dxl_txrx_packet();
    result = dxl_get_result();
    return result == COMM_TXSUCCESS || result == COMM_RXSUCCESS;
}

int Ax12WriteGoals(int arm, const unsigned short *positions)
{ return sendSync(arm, 30, 2, positions); }

int Ax12WriteTorque(int arm, int enabled)
{
    unsigned short values[AX12_JOINTS];
    int j;
    for (j = 0; j < AX12_JOINTS; j++) values[j] = (unsigned short)enabled;
    return sendSync(arm, 24, 1, values);
}

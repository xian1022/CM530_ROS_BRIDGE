#include "dxl_hal.h"
#include "dynamixel.h"

#define ID					(2)
#define LENGTH				(3)
#define INSTRUCTION			(4)
#define ERRBIT				(4)
#define PARAMETER			(5)
#define DEFAULT_BAUDNUMBER	(1)

unsigned char gbInstructionPacket[MAXNUM_TXPARAM+10] = {0};
unsigned char gbStatusPacket[MAXNUM_RXPARAM+10] = {0};
unsigned char gbRxPacketLength = 0;
unsigned char gbRxGetLength = 0;
int gbCommStatus = COMM_RXSUCCESS;
int giBusUsing = 0;
static int gbRxSawData;


int dxl_initialize( int devIndex, int baudnum )
{
	int baudrate;
	baudrate = 2000000 / (baudnum + 1);
	
	if( dxl_hal_open(devIndex, baudrate) == 0 )
		return 0;

	gbCommStatus = COMM_RXSUCCESS;
	giBusUsing = 0;



	return 1;
}

void dxl_terminate()
{
	dxl_hal_close();
}

void dxl_tx_packet()
{
	unsigned char i;
	unsigned char TxNumByte, RealTxNumByte;
	unsigned char checksum = 0;

	if( giBusUsing == 1 )
		return;
	


	giBusUsing = 1;

	if( gbInstructionPacket[LENGTH] < 2 || gbInstructionPacket[LENGTH] > (MAXNUM_TXPARAM+2) )
	{
		gbCommStatus = COMM_TXERROR;
		giBusUsing = 0;
		return;
	}

	if( gbInstructionPacket[INSTRUCTION] != INST_PING
		&& gbInstructionPacket[INSTRUCTION] != INST_READ
		&& gbInstructionPacket[INSTRUCTION] != INST_WRITE
		&& gbInstructionPacket[INSTRUCTION] != INST_REG_WRITE
		&& gbInstructionPacket[INSTRUCTION] != INST_ACTION
		&& gbInstructionPacket[INSTRUCTION] != INST_RESET
		&& gbInstructionPacket[INSTRUCTION] != INST_SYNC_WRITE )
	{
		gbCommStatus = COMM_TXERROR;
		giBusUsing = 0;
		return;
	}
	
	gbInstructionPacket[0] = 0xff;
	gbInstructionPacket[1] = 0xff;
	for( i=0; i<(gbInstructionPacket[LENGTH]+1); i++ )
		checksum += gbInstructionPacket[i+2];
	gbInstructionPacket[gbInstructionPacket[LENGTH]+3] = ~checksum;
	
    /* Discard stale UART bytes before a new reply-bearing transaction. */
    if (gbInstructionPacket[ID] != BROADCAST_ID) dxl_hal_clear();
    gbRxGetLength = gbRxPacketLength = 0;
    gbRxSawData = 0;

	TxNumByte = gbInstructionPacket[LENGTH] + 4;
	RealTxNumByte = dxl_hal_tx( (unsigned char*)gbInstructionPacket, TxNumByte );

	if( TxNumByte != RealTxNumByte )
	{

		gbCommStatus = COMM_TXFAIL;
		giBusUsing = 0;
		return;
	}

	if( gbInstructionPacket[INSTRUCTION] == INST_READ )
		dxl_hal_set_timeout( gbInstructionPacket[PARAMETER+1] + 6 );
	else
		dxl_hal_set_timeout( 6 );

	gbCommStatus = COMM_TXSUCCESS;
}

/* Bounded incremental receiver. No untrusted length is used as an array
 * index until it has been checked. The transaction deadline never restarts. */
void dxl_rx_packet()
{
    int got, i;
    unsigned char ch, checksum = 0;
    if (!giBusUsing) return;
    if (gbInstructionPacket[ID] == BROADCAST_ID) {
        gbCommStatus = COMM_RXSUCCESS; giBusUsing = 0; return;
    }
    if (dxl_hal_timeout()) {
        gbCommStatus = gbRxSawData ? COMM_RXCORRUPT : COMM_RXTIMEOUT;
        giBusUsing = 0; return;
    }
    got = dxl_hal_rx(&ch, 1);
    if (got < 0 || got > 1) {
        gbCommStatus = COMM_RXFAIL; giBusUsing = 0; return;
    }
    gbCommStatus = COMM_RXWAITING;
    if (!got) return;
    gbRxSawData = 1;
    if (gbRxGetLength < 2) {
        if (ch == 0xff) gbStatusPacket[gbRxGetLength++] = ch;
        else gbRxGetLength = 0;
        return;
    }
    /* An extra FF may precede the packet's actual ID. */
    if (gbRxGetLength == 2 && ch == 0xff) return;
    gbStatusPacket[gbRxGetLength++] = ch;
    if (gbRxGetLength == 4) {
        if (gbStatusPacket[ID] != gbInstructionPacket[ID] ||
            gbStatusPacket[LENGTH] < 2 || gbStatusPacket[LENGTH] > MAXNUM_RXPARAM + 2 ||
            (gbInstructionPacket[INSTRUCTION] == INST_READ &&
             gbStatusPacket[LENGTH] != gbInstructionPacket[PARAMETER+1] + 2)) {
            gbCommStatus = COMM_RXCORRUPT; giBusUsing = 0; return;
        }
        gbRxPacketLength = gbStatusPacket[LENGTH] + 4;
    }
    if (gbRxGetLength < 4 || gbRxGetLength < gbRxPacketLength) return;
    for (i = 2; i < gbRxPacketLength; i++) checksum += gbStatusPacket[i];
    gbCommStatus = checksum == 255 ? COMM_RXSUCCESS : COMM_RXCORRUPT;
    giBusUsing = 0;
}

void dxl_txrx_packet()
{

	dxl_tx_packet();

	if( gbCommStatus != COMM_TXSUCCESS )
		return;	

	do{

		dxl_rx_packet();

	}while( gbCommStatus == COMM_RXWAITING );	


}

int dxl_get_result()
{
	return gbCommStatus;
}

void dxl_set_txpacket_id( int id )
{
	gbInstructionPacket[ID] = (unsigned char)id;
}

void dxl_set_txpacket_instruction( int instruction )
{
	gbInstructionPacket[INSTRUCTION] = (unsigned char)instruction;
}

void dxl_set_txpacket_parameter( int index, int value )
{
	gbInstructionPacket[PARAMETER+index] = (unsigned char)value;
}

void dxl_set_txpacket_length( int length )
{
	gbInstructionPacket[LENGTH] = (unsigned char)length;
}

int dxl_get_rxpacket_error( int errbit )
{
	if( gbStatusPacket[ERRBIT] & (unsigned char)errbit )
		return 1;

	return 0;
}

int dxl_get_rxpacket_length()
{
	return (int)gbStatusPacket[LENGTH];
}

int dxl_get_rxpacket_parameter( int index )
{
	return (int)gbStatusPacket[PARAMETER+index];
}

int dxl_makeword( int lowbyte, int highbyte )
{
	unsigned short word;

	word = highbyte;
	word = word << 8;
	word = word + lowbyte;
	return (int)word;
}

int dxl_get_lowbyte( int word )
{
	unsigned short temp;

	temp = word & 0xff;
	return (int)temp;
}

int dxl_get_highbyte( int word )
{
	unsigned short temp;

	temp = word & 0xff00;
	temp = temp >> 8;
	return (int)temp;
}

void dxl_ping( int id )
{
	while(giBusUsing);

	gbInstructionPacket[ID] = (unsigned char)id;
	gbInstructionPacket[INSTRUCTION] = INST_PING;
	gbInstructionPacket[LENGTH] = 2;
	
	dxl_txrx_packet();
}

int dxl_read_byte( int id, int address )
{
	while(giBusUsing);

	gbInstructionPacket[ID] = (unsigned char)id;
	gbInstructionPacket[INSTRUCTION] = INST_READ;
	gbInstructionPacket[PARAMETER] = (unsigned char)address;
	gbInstructionPacket[PARAMETER+1] = 1;
	gbInstructionPacket[LENGTH] = 4;
	
	dxl_txrx_packet();

	return (int)gbStatusPacket[PARAMETER];
}

void dxl_write_byte( int id, int address, int value )
{
	while(giBusUsing);

	gbInstructionPacket[ID] = (unsigned char)id;
	gbInstructionPacket[INSTRUCTION] = INST_WRITE;
	gbInstructionPacket[PARAMETER] = (unsigned char)address;
	gbInstructionPacket[PARAMETER+1] = (unsigned char)value;
	gbInstructionPacket[LENGTH] = 4;
	
	dxl_txrx_packet();
}

int dxl_read_word( int id, int address )
{
	while(giBusUsing);

	gbInstructionPacket[ID] = (unsigned char)id;
	gbInstructionPacket[INSTRUCTION] = INST_READ;
	gbInstructionPacket[PARAMETER] = (unsigned char)address;
	gbInstructionPacket[PARAMETER+1] = 2;
	gbInstructionPacket[LENGTH] = 4;
	



	dxl_txrx_packet();

	return dxl_makeword((int)gbStatusPacket[PARAMETER], (int)gbStatusPacket[PARAMETER+1]);
}

void dxl_write_word( int id, int address, int value )
{
	while(giBusUsing);

	gbInstructionPacket[ID] = (unsigned char)id;
	gbInstructionPacket[INSTRUCTION] = INST_WRITE;
	gbInstructionPacket[PARAMETER] = (unsigned char)address;
	gbInstructionPacket[PARAMETER+1] = (unsigned char)dxl_get_lowbyte(value);
	gbInstructionPacket[PARAMETER+2] = (unsigned char)dxl_get_highbyte(value);
	gbInstructionPacket[LENGTH] = 5;
	
	dxl_txrx_packet();
}

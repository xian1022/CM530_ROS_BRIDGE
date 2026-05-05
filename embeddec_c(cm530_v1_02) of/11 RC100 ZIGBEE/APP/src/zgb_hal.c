// Zigbee SDK platform dependent source
#include "zgb_hal.h"


int zgb_hal_open( int devIndex, float baudrate )
{
	// Opening device
	// devIndex: Device index
	// baudrate: Real baudrate (ex> 115200, 57600, 38400...)
	// Return: 0(Failed), 1(Succeed)
	EnableZigbee();

	return 1;
}

void zgb_hal_close()
{
	// Closing device
	DisableZigbee();
}

int zgb_hal_tx( unsigned char *pPacket, int numPacket )
{
	// Transmiting date
	// *pPacket: data array pointer
	// numPacket: number of data array
	// Return: number of data transmitted. -1 is error.
	unsigned char i;
	for(i=0 ; i<numPacket; i++  )
		TxDByte_Zigbee(pPacket[i]);

	return numPacket;
}

int zgb_hal_rx( unsigned char *pPacket, int numPacket )
{
	// Recieving date
	// *pPacket: data array pointer
	// numPacket: number of data array
	// Return: number of data recieved. -1 is error.
	unsigned char i;
	for( i=0 ; i<numPacket ; i++ )
	{
		if (CheckNewArrive())
			pPacket[i] =  RxDByte_Zigbee();

		else
			return i;
	}

	return numPacket;
}

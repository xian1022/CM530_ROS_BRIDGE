// Dynamixel SDK platform dependent source
#include "dxl_hal.h"

int dxl_hal_open( int devIndex, int baudrate )
{
	// Opening device
	// devIndex: Device index
	// baudrate: Real baudrate (ex> 115200, 57600, 38400...)
	// Return: 0(Failed), 1(Succeed)

	USART1_Configuration(baudrate);
	return 1;
}

void dxl_hal_close()
{
	// Closing device

	/* Disable the USART1 */
	DisableUSART1();
}

void dxl_hal_clear(void)
{
	// Clear communication buffer

	ClearBuffer256();
}

int dxl_hal_tx( unsigned char *pPacket, int numPacket )
{
	// Transmiting date
	// *pPacket: data array pointer
	// numPacket: number of data array
	// Return: number of data transmitted. -1 is error.

	unsigned char i;
	for(i=0 ; i<numPacket; i++  )
		TxDByte_DXL(pPacket[i]);

	return numPacket;
}

int dxl_hal_rx( unsigned char *pPacket, int numPacket )
{
	// Receiving data
	// *pPacket: data array pointer
	// numPacket: number of data array
	// Return: number of data received. -1 is error.

	unsigned char i;
	for( i=0 ; i<numPacket ; i++ )
	{
		if (CheckNewArrive())
			pPacket[i] =  RxDByte_DXL();
		else
			return i;
	}
	return numPacket;
}

void dxl_hal_set_timeout( int NumRcvByte )
{
	// Start stop watch
	// NumRcvByte: number of recieving data(to calculate maximum waiting time)

	//exceed range of int...
	StartDiscount(NumRcvByte*100);
}

int dxl_hal_timeout(void)
{
	// Check timeout
	// Return: 0 is false, 1 is true(timeout occurred)

	return CheckTimeOut();
}

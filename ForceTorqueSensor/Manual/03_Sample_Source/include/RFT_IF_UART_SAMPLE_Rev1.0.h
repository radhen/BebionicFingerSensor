/*
	Rev1.0, 2016.10.05
		- Initial Creation
*/

#include "afxwin.h"
//#include <windows.h>

#include <vector>
using namespace std;

#ifndef _RT_RFT_UART_IF_H_
#define _RT_RFT_UART_IF_H_

#include "RFT_IF_PACKET_Rev1.0.h" // RFT F/T SENSOR DATA FIELD & UART PACKET HANDLING SOURCE

/* // Declared in winbase.h; windows.h
CBR_110
CBR_300
CBR_600
CBR_1200
CBR_2400
CBR_4800
CBR_9600
CBR_14400
CBR_19200
CBR_38400
CBR_57600
CBR_115200
CBR_128000
CBR_256000

EVENPARITY Even
MARKPARITY Mark
NOPARITY No parity
ODDPARITY Odd
SPACEPARIT Space

ONESTOPBIT 1 stop bit
ONE5STOPBITS 1.5 stop bits
TWOSTOPBITS 2 stop bits

*/

// Flow control flags
#define FC_DTRDSR       (0x01)
#define FC_RTSCTS       (0x02)
/* REMOVE Software Flow Control
#define FC_XONXOFF      (0x04)
*/
#define FC_NONE         (0x00)

// ascii definitions
#define ASCII_BEL       (0x07)
#define ASCII_BS        (0x08)
#define ASCII_LF        (0x0A)
#define ASCII_CR        (0x0D)

// for callback.... 
typedef void(*RFT_CAN_IF_CALLBACK) (void *); // for GUI data such as graph rendering...

#define RFT_UART_TX_BUFF_SIZE (1024)
#define RFT_UART_RX_BUFF_SIZE (1024*4)

class CRT_RFT_UART
{
	typedef struct _tagMyBuffer{
		BYTE *pBuf;
		DWORD size;
	}MYBUFFER;

public:
	CRT_RFT_UART(void);
	~CRT_RFT_UART(void);

	// EXTERNAL INTERFACE FUNCTIONS
	BOOL	openPort(BYTE bPort = 1, DWORD rate = 19200, BYTE byteSize = 8, BYTE stop = ONESTOPBIT, BYTE parity = NOPARITY, BYTE bFlowCtrl = FC_NONE, bool callback_enable = false );
	BOOL	closePort(void);
	BOOL  isConnected(void);

	BOOL write(LPVOID lpBuffer, DWORD dwBytesToWrite, LPDWORD lpdwBytesWritten);

	void	readWorker(void);         // read worker.

	////////////////////////////////////////////////////////////////////////////////////////////
	// FOR RFT INTERFACE
	// for callback function
	bool m_bIsEnabled_Callback;
	RFT_CAN_IF_CALLBACK m_pCallbackFunc;
	void *m_pCallbackParam;
	void setCallback(RFT_CAN_IF_CALLBACK pCallbackFunc, void *callbackParam);

	CRT_RFT_IF_PACKET m_RFT_IF_PACKET;	// data field & uart packet handling class
	
	bool rqst_ProductName(void);		// read product name
	bool rqst_SerialNumber(void);		// read serial number
	bool rqst_Firmwareverion(void);		// read firmware version
	bool rqst_CommSpeed(void);			// read baud-rate
	bool rqst_FT_Filter_Type(void);		// read filter type
	bool rqst_FT(void);					// read force/torque (once)
	bool rqst_FT_Continuous(void);		// start force/torque continuous output 
	bool rqst_FT_Stop(void);			// stop force/torque continuous output
	bool rqst_FT_Cont_Interval(void);   // read force/torque output frq.
	bool rqst_FT_OverloadCnt(void);		// read overload count

	bool set_Comm_Speed(int comm_speed);						// set baud-rate
	bool set_FT_Filter_Type(int filter_type, int sub_type);		// set filter
	bool set_FT_Cont_Interval(int interval);					// set force/torque output frq.
	bool set_FT_Bias(int is_on);								// set bias

	int m_nCurrMode;					// current operation mode or command type
	bool m_bIsRcvd_Response_Pkt;		// receive flag for response packet of current command

	void RFT_Data_Handler(void);		// receive packet handler

	void rcvd_ProduectName(void);		// check function for response of read product name command
	void rcvd_SerialNumber(void);		// check function for response of read serial number command
	void rcvd_Firmwareversion(void);	// check function for response of read firmware version command
	void rcvd_CommSpeed(void);			// check function for response of read baud-rate command
	void rcvd_FT_Filter_Type(void);		// check function for response of read filter type command
	void rcvd_FT(void);					// check function for response of read force/torque command
	void rcvd_FT_Cont_Interval(void);	// check function for response of read force/torque output frq. command
	void rcvd_FT_Overload_Count(void);	// check function for response of read overload count command

	void rcvd_Response_Set_Comm_Speed(void);		// check function for response of set baud-rate command
	void rcvd_Response_Set_FT_Filter_Type(void);	// check function for response of set filter type command
	void rcvd_Response_Set_FT_Cont_Interval(void);	// check function for response of set force/torque output frq. command

protected:
	// 
	void initialize_variables(void);

	HANDLE		m_hCommDev;				// handle of COM Port
	BOOL		  m_bConnected;		    // Open/Closed indicating flag
	OVERLAPPED	m_osRead, m_osWrite;	// Port file Overlapped structure

	int m_nRcvdBufferIdx;
	unsigned char m_rcvdBuff[RFT_UART_RX_BUFF_SIZE];
	unsigned char m_txBuff[RFT_UART_TX_BUFF_SIZE];

	// INTERNAL USE ONLY
	BOOL createComPort(BYTE bPort);
	BOOL setupConnection(DWORD dwBaudrate, BYTE bByteSize, BYTE bStopBits, BYTE bParity, BYTE bFlowCtrl);
	BOOL createDataEvent(void);

private:

	HANDLE m_hCommReadThread;         // Handle of Thread
	DWORD  m_idCommReadThread;        // ID of Thread
	static void commReadThread(CRT_RFT_UART *pThis);

};

#endif//_RT_RFT_UART_IF_H_
//////////////////////////////////////////////////////////////////////////////
// END OF FILE

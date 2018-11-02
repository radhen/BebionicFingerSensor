/*
	Rev1.0, 2016.10.05
	- Initial Creation
*/


//#include "stdafx.h"
#include <stdio.h>
#include "RFT_IF_UART_SAMPLE_Rev1.0.h"

#include "RT_Console_Rev0.1.h"

CRT_RFT_UART::CRT_RFT_UART(void)
{
	initialize_variables();
}

CRT_RFT_UART::~CRT_RFT_UART(void)
{
	closePort();
}


BOOL CRT_RFT_UART::openPort(BYTE bPort, DWORD rate, BYTE byteSize, BYTE stop, BYTE parity, BYTE bFlowCtrl, bool callback_enable)
{
	m_bIsEnabled_Callback = callback_enable;

	if (m_bConnected)
		return TRUE;

	if (!createComPort(bPort))
		return FALSE;

	m_bConnected = setupConnection(rate, byteSize, stop, parity, bFlowCtrl);
	if (!m_bConnected)
	{
		CloseHandle(m_hCommDev);
		return m_bConnected;
	}

	if (!createDataEvent())
	{
		CloseHandle(m_hCommDev);
		m_bConnected = FALSE;
		return FALSE;
	}

	m_hCommReadThread = ::CreateThread(NULL, 0, (LPTHREAD_START_ROUTINE)commReadThread, (LPVOID)this, 0, &m_idCommReadThread);
	if (m_hCommReadThread)
	{
		::SetPriorityClass(m_hCommReadThread, REALTIME_PRIORITY_CLASS);
		::SetThreadPriority(m_hCommReadThread, THREAD_PRIORITY_TIME_CRITICAL);
		::EscapeCommFunction(m_hCommDev, SETDTR);
	}
	else
	{
		m_bConnected = false;
		CloseHandle(m_hCommDev);
	}

	return m_bConnected;
}

BOOL CRT_RFT_UART::createComPort(BYTE bPort)
{
	COMMTIMEOUTS  CommTimeOuts;
	DWORD error;
	CString szPort;
	szPort.Format("\\\\.\\\\COM%d", (int)bPort);

	m_hCommDev = CreateFileA((LPCSTR)szPort.GetBuffer(),
		GENERIC_READ | GENERIC_WRITE,
		0,
		0,
		OPEN_EXISTING,
		FILE_ATTRIBUTE_NORMAL | FILE_FLAG_OVERLAPPED,
		0);
	error = GetLastError();

	if (m_hCommDev == INVALID_HANDLE_VALUE)
		return (FALSE);
	else
	{
		SetCommMask(m_hCommDev, EV_RXCHAR);
		SetupComm(m_hCommDev, 4096, 4096);
		PurgeComm(m_hCommDev, PURGE_TXABORT | PURGE_RXABORT | PURGE_TXCLEAR | PURGE_RXCLEAR);
		/* MSDN
			Members
			ReadIntervalTimeout
			Maximum time allowed to elapse between the arrival of two bytes on the communications line, in milliseconds. During a ReadFile operation, the time period begins when the first byte is received. If the interval between the arrival of any two bytes exceeds this amount, the ReadFile operation is completed and any buffered data is returned. A value of zero indicates that interval time-outs are not used.
			A value of MAXDWORD, combined with zero values for both the ReadTotalTimeoutConstant and ReadTotalTimeoutMultiplier members, specifies that the read operation is to return immediately with the bytes that have already been received, even if no bytes have been received.

			ReadTotalTimeoutMultiplier
			Multiplier used to calculate the total time-out period for read operations, in milliseconds. For each read operation, this value is multiplied by the requested number of bytes to be read.
			ReadTotalTimeoutConstant
			Constant used to calculate the total time-out period for read operations, in milliseconds. For each read operation, this value is added to the product of the ReadTotalTimeoutMultiplier member and the requested number of bytes.
			A value of zero for both the ReadTotalTimeoutMultiplier and ReadTotalTimeoutConstant members indicates that total time-outs are not used for read operations.

			WriteTotalTimeoutMultiplier
			Multiplier used to calculate the total time-out period for write operations, in milliseconds. For each write operation, this value is multiplied by the number of bytes to be written.

			WriteTotalTimeoutConstant
			Constant used to calculate the total time-out period for write operations, in milliseconds. For each write operation, this value is added to the product of the WriteTotalTimeoutMultiplier member and the number of bytes to be written.
			A value of zero for both the WriteTotalTimeoutMultiplier and WriteTotalTimeoutConstant members indicates that total time-outs are not used for write operations.

			Remarks
			If an application sets ReadIntervalTimeout and ReadTotalTimeoutMultiplier to MAXDWORD and sets ReadTotalTimeoutConstant to a value greater than zero and less than MAXDWORD, one of the following occurs when the ReadFile function is called:
			If there are any bytes in the input buffer, ReadFile returns immediately with the bytes in the buffer.
			If there are no bytes in the input buffer, ReadFile waits until a byte arrives and then returns immediately.
			If no bytes arrive within the time specified by ReadTotalTimeoutConstant, ReadFile times out.
			*/
		CommTimeOuts.ReadIntervalTimeout = MAXDWORD; 
		CommTimeOuts.ReadTotalTimeoutMultiplier = MAXDWORD; 
		CommTimeOuts.ReadTotalTimeoutConstant = 1;			

		CommTimeOuts.WriteTotalTimeoutMultiplier = 0;
		CommTimeOuts.WriteTotalTimeoutConstant = 10;
		SetCommTimeouts(m_hCommDev, &CommTimeOuts);
	}
	return TRUE;
}

BOOL CRT_RFT_UART::setupConnection(DWORD dwBaudrate, BYTE bByteSize, BYTE bStopBits, BYTE bParity, BYTE bFlowCtrl)
{
	BOOL	fRetVal;
	DCB		dcb;

	dcb.DCBlength = sizeof(DCB);
	GetCommState(m_hCommDev, &dcb);   

	dcb.BaudRate = dwBaudrate;	   
	dcb.ByteSize = bByteSize;	   
	dcb.Parity = bParity;		   
	dcb.StopBits = bStopBits;	   

	// setup hardware flow control
	BOOL bSet = ((bFlowCtrl & FC_DTRDSR) != 0);
	dcb.fOutxDsrFlow = bSet;
	if (bSet)
		dcb.fDtrControl = DTR_CONTROL_HANDSHAKE;
	else
		dcb.fDtrControl = DTR_CONTROL_DISABLE; //DTR_CONTROL_ENABLE; 

	bSet = ((bFlowCtrl & FC_RTSCTS) != 0);
	dcb.fOutxCtsFlow = bSet;
	if (bSet)
		dcb.fRtsControl = RTS_CONTROL_HANDSHAKE;
	else
		dcb.fRtsControl = DTR_CONTROL_DISABLE;//RTS_CONTROL_ENABLE; 

	// setup software flow control
	bSet = 0; // Don't use software flow control 
	dcb.fInX = dcb.fOutX = bSet;
	dcb.XonChar = 0xff;
	dcb.XoffChar = 0xff;
	dcb.XonLim = 100;
	dcb.XoffLim = 100;

	// other various settings
	dcb.fBinary = TRUE;
	dcb.fParity = TRUE;

	fRetVal = SetCommState(m_hCommDev, &dcb);	
	// for debugging.
	/*
		if (!fRetVal)
		{
		char buf[100];
		char cParity;
		char cStopBits[5];

		switch (bParity)
		{
		case NOPARITY     : cParity = 'n'; break;
		case ODDPARITY    : cParity = 'o'; break;
		case EVENPARITY   : cParity = 'e'; break;
		case MARKPARITY   : cParity = 'm'; break;
		case SPACEPARITY  : cParity = 's'; break;
		default : cParity = 'n'; break;
		}

		switch (bStopBits)
		{
		case ONESTOPBIT   : sprintf(cStopBits, "1"); break;
		case ONE5STOPBITS : sprintf(cStopBits, "1.5"); break;
		case TWOSTOPBITS  : sprintf(cStopBits, "2"); break;
		default : sprintf(cStopBits, "1"); break;
		}

		sprintf(buf, "%ld,%c,%d,%s", dwBaudrate, cParity, bByteSize, cStopBits);

		fRetVal = BuildCommDCB( (LPCWSTR)buf, &dcb);
		}
		*/
	return (fRetVal);
}

BOOL CRT_RFT_UART::createDataEvent(void)
{
	m_osWrite.Offset = 0;
	m_osWrite.OffsetHigh = 0;
	m_osRead.Offset = 0;
	m_osRead.OffsetHigh = 0;

	m_osRead.hEvent = CreateEvent(NULL, TRUE, FALSE, NULL);
	if (m_osRead.hEvent == NULL)
	{
		CloseHandle(m_osRead.hEvent);
		return FALSE;
	}

	m_osWrite.hEvent = CreateEvent(NULL, TRUE, FALSE, NULL);
	if (m_osWrite.hEvent == NULL)
	{
		CloseHandle(m_osWrite.hEvent);
		return FALSE;
	}
	return TRUE;
}

BOOL CRT_RFT_UART::closePort(void)
{
	if (!m_bConnected)
		return TRUE;

	if (m_bConnected)
	{
		// set connected flag to FALSE
		m_bConnected = FALSE;
		// disable event notification and wait for thread to halt

		SetCommMask(m_hCommDev, 0);
		EscapeCommFunction(m_hCommDev, CLRDTR);
		PurgeComm(m_hCommDev, PURGE_TXABORT | PURGE_RXABORT | PURGE_TXCLEAR | PURGE_RXCLEAR);
		CloseHandle(m_hCommDev);
		m_hCommDev = NULL;
	}

	CloseHandle(m_osRead.hEvent);
	CloseHandle(m_osWrite.hEvent);

	return (TRUE);
}

void CRT_RFT_UART::setCallback(RFT_CAN_IF_CALLBACK pCallbackFunc, void *callbackParam)
{
	m_pCallbackFunc = pCallbackFunc;
	m_pCallbackParam = callbackParam;

	CONSOLE_S("SETTING.... RFT UART I/F CALL BACK FUNCTION\n");
}

BOOL CRT_RFT_UART::isConnected(void)
{
	return (m_bConnected);
}

void CRT_RFT_UART::commReadThread(CRT_RFT_UART *pThis)
{
	pThis->readWorker();
	//::ExitThread( 0 );
}

void CRT_RFT_UART::readWorker(void)        // read worker.
{
	CONSOLE_S("RFT UART IF READ THREAD START...\n");

	COMSTAT    ComStat;
	DWORD      dwErrorFlags;
	DWORD      dwReceived;
	DWORD      dwReturned;

	DWORD       dwEvtMask = 0;
	OVERLAPPED  os;

	memset(&os, 0, sizeof(OVERLAPPED));

	os.hEvent = CreateEvent(NULL, TRUE, FALSE, NULL);
	if (os.hEvent == NULL)
	{
		closePort();
		return;
	}

	if (!SetCommMask(m_hCommDev, EV_RXCHAR))
	{
		closePort();
		return;
	}

	m_nRcvdBufferIdx = 0;

	while (m_bConnected)
	{
		// only try to read number of bytes in queue 
		WaitCommEvent(m_hCommDev, &dwEvtMask, NULL);

		if ((dwEvtMask & EV_RXCHAR) == EV_RXCHAR)
		{
			ClearCommError(m_hCommDev, &dwErrorFlags, &ComStat);
			dwReceived = ComStat.cbInQue;

			if (dwReceived <= 0)
			{
				continue;
			}

			dwReturned = 0;

			while (dwReceived > 0)
			{
				if (ReadFile(m_hCommDev, &m_rcvdBuff[m_nRcvdBufferIdx], dwReceived, &dwReturned, &m_osRead))
				{
					m_nRcvdBufferIdx += dwReturned;
					dwReceived -= dwReturned;
				}
				else
				{
					break;
				}
			}

			if (m_nCurrMode == CMD_NONE)
			{
				m_nRcvdBufferIdx = 0;
				CONSOLE_S("CMD NONE: Serial data received\n");
				continue;
			}

			// wait packet.... 
			if (m_nRcvdBufferIdx < UART_RESPONSE_PACKET_SIZE)
			{
				//CONSOLE_S("RECEIVED: %d\n", m_nRcvdBufferIdx);
				continue;
			}

			// find SOP
			int found_idx = -1;
			for (int i = 0; i < m_nRcvdBufferIdx; i++)
			{
				if (m_rcvdBuff[i] == SOP)
				{
					found_idx = i;
					break;
				}
			}

			if (found_idx == -1)
			{
				m_nRcvdBufferIdx = 0;
				CONSOLE_S("SOP NOT FOUNDED\n");
				continue;
			}

			// if the index of SOP is not first(0), shift received data....
			if (found_idx != 0) 
			{
				for (int i = 0; i < (m_nRcvdBufferIdx - found_idx); i++)
				{
					m_rcvdBuff[i] = m_rcvdBuff[i + found_idx];
				}
				CONSOLE_S("SHIFT\n");
				m_nRcvdBufferIdx = m_nRcvdBufferIdx - found_idx;
			}

			// packet handling....
			if (m_nRcvdBufferIdx >= UART_RESPONSE_PACKET_SIZE )
				RFT_Data_Handler();				
		}
	}

	CONSOLE_S("RFT UART IF READ THREAD FINISHED...\n");
}

void CRT_RFT_UART::initialize_variables(void)
{
	m_hCommDev = NULL;				      // handle of COM Port
	m_bConnected = FALSE;		        // Open/Closed indicating flag

	//InitializeCriticalSection( &m_commHandleCrit );


	/////////////////////////////////////////////////////////////////////////
	m_pCallbackFunc = NULL;
	m_pCallbackParam = NULL;
	m_bIsEnabled_Callback = false;

	m_nCurrMode = CMD_NONE;
	m_bIsRcvd_Response_Pkt = false;

	m_nRcvdBufferIdx = 0;
	
	memset(m_rcvdBuff, 0, sizeof(unsigned char)*RFT_UART_RX_BUFF_SIZE);
	memset(m_txBuff, 0, sizeof(unsigned char)*RFT_UART_TX_BUFF_SIZE);
}

void CRT_RFT_UART::RFT_Data_Handler( void )
{
	switch (m_nCurrMode)
	{
	case CMD_GET_PRODUCT_NAME:
		rcvd_ProduectName();
		break;

	case CMD_GET_SERIAL_NUMBER:
		rcvd_SerialNumber();
		break;

	case CMD_GET_FIRMWARE_VER:
		rcvd_Firmwareversion();
		break;

	case CMD_SET_COMM_BAUDRATE:
		rcvd_Response_Set_Comm_Speed();
		break;

	case CMD_GET_COMM_BAUDRATE:
		rcvd_CommSpeed();
		break;

	case CMD_SET_FT_FILTER:
		rcvd_Response_Set_FT_Filter_Type();
		break;

	case CMD_GET_FT_FILTER:
		rcvd_FT_Filter_Type();
		break;

	case CMD_FT_ONCE:
		rcvd_FT();
		break;

	case CMD_FT_CONT:
		rcvd_FT();
		break;

	case CMD_FT_CONT_STOP:
		break;

	case CMD_SET_CONT_OUT_FRQ:
		rcvd_Response_Set_FT_Cont_Interval();
		break;

	case CMD_GET_CONT_OUT_FRQ:
		rcvd_FT_Cont_Interval();
		break;

	case CMD_GET_OVERLOAD_COUNT:
		rcvd_FT_Overload_Count();
		break;

	default: 
		break;
	}
}

BOOL CRT_RFT_UART::write(LPVOID lpBuffer, DWORD dwBytesToWrite, LPDWORD lpdwBytesWritten)
{
	DWORD     dwBytesWritten = 0;
	DWORD     dwTotalWritten = 0;
	DWORD     dwError, dwErrorFlags;
	COMSTAT    ComStat;
	dwTotalWritten = 0;
	BOOL result = TRUE;
	// WRITE
	//EnterCriticalSection( &m_commHandleCrit );
	while ((dwBytesToWrite - dwTotalWritten) > 0)
	{
		if (WriteFile(m_hCommDev, ((char*)lpBuffer) + dwTotalWritten, dwBytesToWrite - dwTotalWritten, &dwBytesWritten, &m_osWrite))
		{
			dwTotalWritten += dwBytesWritten;
		}
		else
		{
			if (GetLastError() == ERROR_IO_PENDING)
			{
				while (!GetOverlappedResult(m_hCommDev, &m_osWrite, &dwBytesWritten, TRUE))
				{
					dwError = GetLastError();
					if (dwError != ERROR_IO_INCOMPLETE)
					{
						ClearCommError(m_hCommDev, &dwErrorFlags, &ComStat);
						result = FALSE;
						break;
					}
				}
				dwTotalWritten += dwBytesWritten;
			}
			else
			{
				dwBytesWritten = 0;
				ClearCommError(m_hCommDev, &dwErrorFlags, &ComStat);
				result = FALSE;
				break;
			}
		}
	}
	//LeaveCriticalSection( &m_commHandleCrit );
	*lpdwBytesWritten = dwTotalWritten;

	return result;
}




bool CRT_RFT_UART::rqst_ProductName( void )
{
	if (m_nRcvdBufferIdx)
	{
		CONSOLE_S("BUFF IDX CLEAR - NEW CMD SENDING\n");
		m_nRcvdBufferIdx = 0;
	}

	int txPktSize = UART_COMMAND_PACKET_SIZE;

	m_RFT_IF_PACKET.UPG_read_product_name(m_txBuff);

	m_bIsRcvd_Response_Pkt = false;
	m_nCurrMode = m_txBuff[1];

	DWORD written;
	bool result = (bool)write(m_txBuff, txPktSize, &written);
	//CONSOLE_S("written: %d\n", written);
	return result;
}

bool CRT_RFT_UART::rqst_SerialNumber(void)
{
	if (m_nRcvdBufferIdx)
	{
		CONSOLE_S("BUFF IDX CLEAR - NEW CMD SENDING\n");
		m_nRcvdBufferIdx = 0;
	}

	int txPktSize = UART_COMMAND_PACKET_SIZE;

	m_RFT_IF_PACKET.UPG_read_serial_name(m_txBuff);

	m_bIsRcvd_Response_Pkt = false;
	m_nCurrMode = m_txBuff[1];

	DWORD written;
	bool result = (bool)write(m_txBuff, txPktSize, &written);
	//CONSOLE_S("written: %d\n", written);
	return result;
}


bool CRT_RFT_UART::rqst_Firmwareverion(void)
{
	if (m_nRcvdBufferIdx)
	{
		CONSOLE_S("BUFF IDX CLEAR - NEW CMD SENDING\n");
		m_nRcvdBufferIdx = 0;
	}

	int txPktSize = UART_COMMAND_PACKET_SIZE;

	m_RFT_IF_PACKET.UPG_read_firmware_version(m_txBuff);

	m_bIsRcvd_Response_Pkt = false;
	m_nCurrMode = m_txBuff[1];

	DWORD written;
	bool result = (bool)write(m_txBuff, txPktSize, &written);
	//CONSOLE_S("written: %d\n", written);
	return result;
}

bool CRT_RFT_UART::rqst_CommSpeed(void)
{
	if (m_nRcvdBufferIdx)
	{
		CONSOLE_S("BUFF IDX CLEAR - NEW CMD SENDING\n");
		m_nRcvdBufferIdx = 0;
	}

	int txPktSize = UART_COMMAND_PACKET_SIZE;

	m_RFT_IF_PACKET.UPG_read_comm_baudrate(m_txBuff);

	m_bIsRcvd_Response_Pkt = false;
	m_nCurrMode = m_txBuff[1];

	DWORD written;
	bool result = (bool)write(m_txBuff, txPktSize, &written);
	//CONSOLE_S("written: %d\n", written);
	return result;
}

bool CRT_RFT_UART::rqst_FT_Filter_Type(void)
{
	if (m_nRcvdBufferIdx)
	{
		CONSOLE_S("BUFF IDX CLEAR - NEW CMD SENDING\n");
		m_nRcvdBufferIdx = 0;
	}

	int txPktSize = UART_COMMAND_PACKET_SIZE;

	m_RFT_IF_PACKET.UPG_read_filter_type(m_txBuff);

	m_bIsRcvd_Response_Pkt = false;
	m_nCurrMode = m_txBuff[1];

	DWORD written;
	bool result = (bool)write(m_txBuff, txPktSize, &written);
	//CONSOLE_S("written: %d\n", written);
	return result;
}

bool CRT_RFT_UART::rqst_FT(void)
{
	if (m_nRcvdBufferIdx)
	{
		CONSOLE_S("BUFF IDX CLEAR - NEW CMD SENDING\n");
		m_nRcvdBufferIdx = 0;
	}

	int txPktSize = UART_COMMAND_PACKET_SIZE;

	m_RFT_IF_PACKET.UPG_read_force_once(m_txBuff);

	m_bIsRcvd_Response_Pkt = false;
	m_nCurrMode = m_txBuff[1];

	DWORD written;
	bool result = (bool)write(m_txBuff, txPktSize, &written);
	//CONSOLE_S("written: %d\n", written);
	return result;
}

bool CRT_RFT_UART::rqst_FT_Continuous(void)
{
	if (m_nRcvdBufferIdx)
	{
		CONSOLE_S("BUFF IDX CLEAR - NEW CMD SENDING\n");
		m_nRcvdBufferIdx = 0;
	}

	int txPktSize = UART_COMMAND_PACKET_SIZE;

	m_RFT_IF_PACKET.UPG_read_force(m_txBuff);

	m_bIsRcvd_Response_Pkt = false;
	m_nCurrMode = m_txBuff[1];

	DWORD written;
	bool result = (bool)write(m_txBuff, txPktSize, &written);
	//CONSOLE_S("written: %d\n", written);
	return result;
}

bool CRT_RFT_UART::rqst_FT_Stop(void)
{
	if (m_nRcvdBufferIdx)
	{
		CONSOLE_S("BUFF IDX CLEAR - NEW CMD SENDING\n");
		m_nRcvdBufferIdx = 0;
	}

	int txPktSize = UART_COMMAND_PACKET_SIZE;

	m_RFT_IF_PACKET.UPG_set_stop_force_out(m_txBuff);

	m_bIsRcvd_Response_Pkt = true; // FT output stop command don't have response packet.
	m_nCurrMode = CMD_NONE;

	DWORD written;
	bool result = (bool)write(m_txBuff, txPktSize, &written);
	//CONSOLE_S("written: %d\n", written);
	return result;
}

bool CRT_RFT_UART::rqst_FT_Cont_Interval(void)
{
	if (m_nRcvdBufferIdx)
	{
		CONSOLE_S("BUFF IDX CLEAR - NEW CMD SENDING\n");
		m_nRcvdBufferIdx = 0;
	}

	int txPktSize = UART_COMMAND_PACKET_SIZE;

	m_RFT_IF_PACKET.UPG_read_output_frq(m_txBuff);

	m_bIsRcvd_Response_Pkt = false;
	m_nCurrMode = m_txBuff[1];

	DWORD written;
	bool result = (bool)write(m_txBuff, txPktSize, &written);
	//CONSOLE_S("written: %d\n", written);
	return result;
}

bool CRT_RFT_UART::rqst_FT_OverloadCnt(void)
{
	if (m_nRcvdBufferIdx)
	{
		CONSOLE_S("BUFF IDX CLEAR - NEW CMD SENDING\n");
		m_nRcvdBufferIdx = 0;
	}

	int txPktSize = UART_COMMAND_PACKET_SIZE;

	m_RFT_IF_PACKET.UPG_read_overload_count(m_txBuff);

	m_bIsRcvd_Response_Pkt = false;
	m_nCurrMode = m_txBuff[1];

	DWORD written;
	bool result = (bool)write(m_txBuff, txPktSize, &written);
	//CONSOLE_S("written: %d\n", written);
	return result;
}

bool CRT_RFT_UART::set_Comm_Speed(int comm_speed)
{
	if (m_nRcvdBufferIdx)
	{
		CONSOLE_S("BUFF IDX CLEAR - NEW CMD SENDING\n");
		m_nRcvdBufferIdx = 0;
	}

	int txPktSize = UART_COMMAND_PACKET_SIZE;

	m_RFT_IF_PACKET.UPG_set_comm_baudrate(m_txBuff, comm_speed);

	m_bIsRcvd_Response_Pkt = false;
	m_nCurrMode = m_txBuff[1];

	DWORD written;
	bool result = (bool)write(m_txBuff, txPktSize, &written);
	//CONSOLE_S("written: %d\n", written);
	return result;
}

bool CRT_RFT_UART::set_FT_Filter_Type(int filter_type, int sub_type)
{
	if (m_nRcvdBufferIdx)
	{
		CONSOLE_S("BUFF IDX CLEAR - NEW CMD SENDING\n");
		m_nRcvdBufferIdx = 0;
	}

	int txPktSize = UART_COMMAND_PACKET_SIZE;

	m_RFT_IF_PACKET.UPG_set_filter_type(m_txBuff, filter_type, sub_type);


	m_bIsRcvd_Response_Pkt = false;
	m_nCurrMode = m_txBuff[1];

	DWORD written;
	bool result = (bool)write(m_txBuff, txPktSize, &written);
	//CONSOLE_S("written: %d\n", written);
	return result;
}


bool CRT_RFT_UART::set_FT_Cont_Interval(int interval)
{
	if (m_nRcvdBufferIdx)
	{
		CONSOLE_S("BUFF IDX CLEAR - NEW CMD SENDING\n");
		m_nRcvdBufferIdx = 0;
	}

	int txPktSize = UART_COMMAND_PACKET_SIZE;

	m_RFT_IF_PACKET.UPG_set_output_frq(m_txBuff, interval);

	m_bIsRcvd_Response_Pkt = false;
	m_nCurrMode = m_txBuff[1];

	DWORD written;
	bool result = (bool)write(m_txBuff, txPktSize, &written);
	//CONSOLE_S("written: %d\n", written);
	return result;
}

bool CRT_RFT_UART::set_FT_Bias(int is_on)
{
	/* Bias setting command don't have response packet
	if (m_nRcvdBufferIdx)
	{
		CONSOLE_S("BUFF IDX CLEAR - NEW CMD SENDING\n");
		m_nRcvdBufferIdx = 0; 
	}
	*/

	int txPktSize = UART_COMMAND_PACKET_SIZE;

	m_RFT_IF_PACKET.UPG_set_bias(m_txBuff, is_on);

	/* Bias setting command don't have response packet
	m_bIsRcvd_Response_Pkt = false;
	m_nCurrMode = m_txBuff[1];
	*/

	DWORD written;
	bool result = (bool)write(m_txBuff, txPktSize, &written);

	//CONSOLE_S("written: %d\n", written);
	return result;
}


void CRT_RFT_UART::rcvd_ProduectName(void)
{
	if (m_nRcvdBufferIdx >= UART_RESPONSE_PACKET_SIZE)
	{
		if (m_RFT_IF_PACKET.UART_packet_processing(m_rcvdBuff, CMD_GET_PRODUCT_NAME))
		{
			m_bIsRcvd_Response_Pkt = true;
			m_nRcvdBufferIdx = 0;
		}
	}
}

void CRT_RFT_UART::rcvd_SerialNumber(void)
{
	if (m_nRcvdBufferIdx >= UART_RESPONSE_PACKET_SIZE)
	{
		if (m_RFT_IF_PACKET.UART_packet_processing(m_rcvdBuff, CMD_GET_SERIAL_NUMBER))
		{
			m_bIsRcvd_Response_Pkt = true;
			m_nRcvdBufferIdx = 0;
		}
	}
}

void CRT_RFT_UART::rcvd_Firmwareversion(void)
{
	if (m_nRcvdBufferIdx >= UART_RESPONSE_PACKET_SIZE)
	{
		if (m_RFT_IF_PACKET.UART_packet_processing(m_rcvdBuff, CMD_GET_FIRMWARE_VER))
		{
			m_bIsRcvd_Response_Pkt = true;
			m_nRcvdBufferIdx = 0;
		}
	}
}

void CRT_RFT_UART::rcvd_CommSpeed(void)
{
	if (m_nRcvdBufferIdx >= UART_RESPONSE_PACKET_SIZE)
	{
		if (m_RFT_IF_PACKET.UART_packet_processing(m_rcvdBuff, CMD_GET_COMM_BAUDRATE))
		{
			m_bIsRcvd_Response_Pkt = true;
			m_nRcvdBufferIdx = 0;
		}
	}
}

void CRT_RFT_UART::rcvd_FT_Filter_Type(void)
{
	if (m_nRcvdBufferIdx >= UART_RESPONSE_PACKET_SIZE)
	{
		if (m_RFT_IF_PACKET.UART_packet_processing(m_rcvdBuff, CMD_GET_FT_FILTER))
		{
			m_bIsRcvd_Response_Pkt = true;
			m_nRcvdBufferIdx = 0;
		}
	}
}

void CRT_RFT_UART::rcvd_FT(void)
{
	int pkt_size = UART_RESPONSE_PACKET_SIZE;
	int num_of_rcvd_pkt = m_nRcvdBufferIdx / pkt_size;
	int num_of_next_pkt_data = m_nRcvdBufferIdx % pkt_size;
	int transfer_data_size;
	int source_start_idx;

	// if the number of received packet is more than 1, 
	// move the last packet data to first location of buffer to interprete the last received data.
	if (num_of_rcvd_pkt > 1) 
	{
		transfer_data_size = pkt_size + num_of_next_pkt_data;
		source_start_idx = (num_of_rcvd_pkt - 1) * pkt_size;

		for (int i = 0; i < transfer_data_size; i++)
			m_rcvdBuff[i] = m_rcvdBuff[source_start_idx + i];

		//CONSOLE_S("DATA TRANSFER - BEFORE\n");
	}

	if (m_nRcvdBufferIdx >= pkt_size)
	{
		if (m_RFT_IF_PACKET.UART_packet_processing(m_rcvdBuff, m_nCurrMode))
		{
			m_bIsRcvd_Response_Pkt = true;

			// 
			if (m_bIsEnabled_Callback)
				m_pCallbackFunc(m_pCallbackParam);

			// if received data size bigger than packet size,
			// shift that to first location of buffer.
			if (num_of_next_pkt_data)
			{
				transfer_data_size = num_of_next_pkt_data;
				source_start_idx = pkt_size;
				for (int i = 0; i < transfer_data_size; i++)
					m_rcvdBuff[i] = m_rcvdBuff[source_start_idx + i];

				//CONSOLE_S("DATA TRANSFER - AFTER\n");

				m_nRcvdBufferIdx = num_of_next_pkt_data;
			}
			else
			{
				m_nRcvdBufferIdx = 0; 
			}
		}
	}
}



void CRT_RFT_UART::rcvd_FT_Cont_Interval(void)
{
	if (m_nRcvdBufferIdx >= UART_RESPONSE_PACKET_SIZE)
	{
		if (m_RFT_IF_PACKET.UART_packet_processing(m_rcvdBuff, CMD_GET_CONT_OUT_FRQ))
		{
			m_bIsRcvd_Response_Pkt = true;
			m_nRcvdBufferIdx = 0;
		}
	}
}

void CRT_RFT_UART::rcvd_Response_Set_Comm_Speed(void)
{
	if (m_nRcvdBufferIdx >= UART_RESPONSE_PACKET_SIZE)
	{
		if (m_RFT_IF_PACKET.UART_packet_processing(m_rcvdBuff, CMD_SET_COMM_BAUDRATE))
		{
			m_bIsRcvd_Response_Pkt = true;
			m_nRcvdBufferIdx = 0;
		}
	}
}

void CRT_RFT_UART::rcvd_Response_Set_FT_Filter_Type(void)
{
	if (m_nRcvdBufferIdx >= UART_RESPONSE_PACKET_SIZE)
	{
		if (m_RFT_IF_PACKET.UART_packet_processing(m_rcvdBuff, CMD_SET_FT_FILTER))
		{
			m_bIsRcvd_Response_Pkt = true;
			m_nRcvdBufferIdx = 0;
		}
	}
}

void CRT_RFT_UART::rcvd_FT_Overload_Count(void)
{
	if (m_nRcvdBufferIdx >= UART_RESPONSE_PACKET_SIZE)
	{
		if (m_RFT_IF_PACKET.UART_packet_processing(m_rcvdBuff, CMD_GET_OVERLOAD_COUNT))
		{
			m_bIsRcvd_Response_Pkt = true;
			m_nRcvdBufferIdx = 0;
		}
	}
}

void CRT_RFT_UART::rcvd_Response_Set_FT_Cont_Interval(void)
{
	if (m_nRcvdBufferIdx >= UART_RESPONSE_PACKET_SIZE)
	{
		if (m_RFT_IF_PACKET.UART_packet_processing(m_rcvdBuff, CMD_SET_CONT_OUT_FRQ)) // 
		{
			m_bIsRcvd_Response_Pkt = true;
			m_nRcvdBufferIdx = 0;
		}
	}
}


//////////////////////////////////////////////////////////////////////////////
// END OF FILE

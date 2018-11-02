//#include "stdafx.h"

/*
  RT_Console.cpp

  edited by Bong oh Kim

  2003. 01. 14
	
	modified by bokim, 2013. 01. 23
	- reference : DebugConsole_win32.h, DebugConsole_win32.cpp
*/

/*
	Rev0.1 error ������ io �Լ��鿡 ���� xxxx_s(..) �Լ��� ��ü
	��� �� ������ ���ֱ� ���� �ҽ��� #define _CRT_SECURE_NO_WARNINGS ���־ �ȴ�.

*/

#include "RT_Console_Rev0.1.h"
#include <stdio.h>
#include <stdlib.h>

//#include <stdiostr.h> // removed for Visual Studio 2005 Enviroment


// CONSTRUCTOR And DESTRUCTOR
CRT_Console::CRT_Console()
{
  m_hIn = NULL;
  m_hOut = NULL;

	create();
}

CRT_Console::~CRT_Console()
{
  if( m_hIn != NULL || m_hOut != NULL )
    destroy();
}

/*
  name       : create()
  description: console window�� �����ϰ� stdio�� file��
               open�Ͽ� stdio �Լ��� ����� �� �ְ� �Ѵ�.
  note       :
  return     : ���������� console window�� �����Ǹ� ture
*/
BOOL CRT_Console::create()
{
  if( AllocConsole() == FALSE )
    return FALSE;

  m_hIn = GetStdHandle(STD_INPUT_HANDLE);
  m_hOut = GetStdHandle(STD_OUTPUT_HANDLE);

  if( m_hIn == m_hOut )
    return FALSE;

  // console window�� stdio function�� ����Ͽ� ����ϱ� ���ؼ���
  // �⺻������ �ҷ����� �Ѵ�.
  //freopen( (const char*)"CONOUT$", (const char*)"wt", (FILE*)stdout );
  //_wfreopen((const wchar_t*)"CONOUT$", (const wchar_t*)"wt", (FILE*)stdout);
  //freopen_s( "CONOUT$", "wt", stdout, stderr );
  freopen_s((FILE**)stdout, (const char*)"CONOUT$", "wt", stderr); // Rev0.1

  /* memory leak !! 
  // c++�� iostream�� ����ϱ� ���ؼ� �Ʒ��� ���� ������ �ϸ�Ǵµ�
  // memory leak�� �߻��Ѵ�.
  //cout = new stdiobuf(stdout);
  //if( cout == NULL )
  //  FALSE;
  */

  return TRUE;
}

/*
  name       : destroy()
  description: ������ console window�� ���ش�.
  note       :
  return     : �Ϸ�Ǹ� TRUE
*/
BOOL CRT_Console::destroy()
{
  if( m_hIn != NULL || m_hOut != NULL )
  {
    //delete cout;  access violation ??
    fclose( stdout );
    FreeConsole();
  }

  return TRUE;
}


void CRT_Console::PRINT_D(char* _formatString, ...)
{
	char     sVBuffer[1024], sPrintBuffer[1024];
	SYSTEMTIME	localTime;
	DWORD       dwWrite;
	va_list     args;

	SetConsoleTextAttribute( m_hOut, CONSOLE_WHITE_D );

	::GetLocalTime(&localTime);
	va_start(args, _formatString);
	//vsprintf( sVBuffer, _formatString, args );
	vsprintf_s(sVBuffer, _formatString, args);
	va_end(args);
	
	//sprintf( sPrintBuffer, "[%02d:%02d:%02d:%03d] ", localTime.wHour, localTime.wMinute, localTime.wSecond, localTime.wMilliseconds);
	sprintf_s(sPrintBuffer, "[%02d:%02d:%02d:%03d] ", localTime.wHour, localTime.wMinute, localTime.wSecond, localTime.wMilliseconds);
	WriteConsole(m_hOut, sPrintBuffer, (DWORD)strlen(sPrintBuffer), &dwWrite, NULL);
	WriteConsole(m_hOut, sVBuffer, (DWORD)strlen(sVBuffer), &dwWrite, NULL);
}

void CRT_Console::PRINT_D(char TextColor, char* _formatString, ...)
{
	char     sVBuffer[1024], sPrintBuffer[1024];
	SYSTEMTIME	localTime;
	DWORD       dwWrite;
	va_list     args;

	SetConsoleTextAttribute( m_hOut, (TextColor & 0x0F) );

	::GetLocalTime(&localTime);
	va_start(args, _formatString);
	//vsprintf( sVBuffer, _formatString, args );
	vsprintf_s(sVBuffer, _formatString, args);
	va_end(args);

	//sprintf( sPrintBuffer, "[%02d:%02d:%02d:%03d] ", localTime.wHour, localTime.wMinute, localTime.wSecond, localTime.wMilliseconds);
	sprintf_s(sPrintBuffer, "[%02d:%02d:%02d:%03d] ", localTime.wHour, localTime.wMinute, localTime.wSecond, localTime.wMilliseconds);
	WriteConsole(m_hOut, sPrintBuffer, (DWORD)strlen(sPrintBuffer), &dwWrite, NULL);
	WriteConsole(m_hOut, sVBuffer, (DWORD)strlen(sVBuffer), &dwWrite, NULL);

	SetConsoleTextAttribute( m_hOut, CONSOLE_WHITE_D );
}

void CRT_Console::PRINT_D(char TextColor, char BackColor, char* _formatString, ...)
{
	char     sVBuffer[1024], sPrintBuffer[1024];
	SYSTEMTIME	localTime;
	DWORD       dwWrite;
	va_list     args;

	SetConsoleTextAttribute( m_hOut, (TextColor | ((BackColor<<4) & 0xF0)) );

	::GetLocalTime(&localTime);
	va_start(args, _formatString);
	//vsprintf( sVBuffer, _formatString, args );
	vsprintf_s(sVBuffer, _formatString, args);
	va_end(args);

	//sprintf( sPrintBuffer, "[%02d:%02d:%02d:%03d] ", localTime.wHour, localTime.wMinute, localTime.wSecond, localTime.wMilliseconds);
	sprintf_s(sPrintBuffer, "[%02d:%02d:%02d:%03d] ", localTime.wHour, localTime.wMinute, localTime.wSecond, localTime.wMilliseconds);
	WriteConsole(m_hOut, sPrintBuffer, (DWORD)strlen(sPrintBuffer), &dwWrite, NULL);
	WriteConsole(m_hOut, sVBuffer, (DWORD)strlen(sVBuffer), &dwWrite, NULL);

	SetConsoleTextAttribute( m_hOut, CONSOLE_WHITE_D );
}

void CRT_Console::PRINT(char* _formatString, ...)
{
    char     sVBuffer[1024];
    DWORD       dwWrite;
    va_list     args;

	SetConsoleTextAttribute( m_hOut, CONSOLE_WHITE_D );

    va_start(args, _formatString);
    //vsprintf( sVBuffer, _formatString, args );
	vsprintf_s(sVBuffer, _formatString, args);
    va_end(args);

    WriteConsole(m_hOut, sVBuffer, (DWORD)strlen(sVBuffer), &dwWrite, NULL);
}

void CRT_Console::PRINT(char TextColor, char* _formatString, ...)
{
	char     sVBuffer[1024];
	DWORD       dwWrite;
	va_list     args;

	SetConsoleTextAttribute( m_hOut, (TextColor & 0x0F) );

	va_start(args, _formatString);
	//vsprintf( sVBuffer, _formatString, args );
	vsprintf_s(sVBuffer, _formatString, args);
	va_end(args);

	WriteConsole(m_hOut, sVBuffer, (DWORD)strlen(sVBuffer), &dwWrite, NULL);

	SetConsoleTextAttribute( m_hOut, CONSOLE_WHITE_D );
}

void CRT_Console::PRINT(char TextColor, char BackColor, char* _formatString, ...)
{
	char     sVBuffer[1024];
	DWORD       dwWrite;
	va_list     args;

	SetConsoleTextAttribute( m_hOut, (TextColor | ((BackColor<<4) & 0xF0)) );

	va_start(args, _formatString);
	//vsprintf( sVBuffer, _formatString, args );
	vsprintf_s(sVBuffer, _formatString, args);
	va_end(args);

	WriteConsole(m_hOut, sVBuffer, (DWORD)strlen(sVBuffer), &dwWrite, NULL);

	SetConsoleTextAttribute( m_hOut, CONSOLE_WHITE_D );
}

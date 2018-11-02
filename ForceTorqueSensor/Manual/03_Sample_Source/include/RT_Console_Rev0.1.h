/*
  RT_Console.h

  for debugging using console window and standard io stream
  
  edited by Bong oh Kim

  ver 1.00
  2003. 01. 14

  CDebugger class를 upgrade 한 것이라고 보면 된다.
  stdio 처럼 그냥 바로 printf나 cout 사용하기

  cout을 사용하기 위해서는 
  cout = new stdiobuf( stdout );
  를 수행 해야 하나 dynamic하게 잡은 memory의 leak이 발생하게 됨
  이를 해결 하기 위해서는 ? 아직 모름

  현재 구현된 기능은 console 창에 stdio 함수인 printf를 사용할
  수 있게함.

	modified by bokim, 2013. 01. 23
	- reference : DebugConsole_win32.h, DebugConsole_win32.cpp
*/

#ifndef _BOKIM_DEBUGGING_CONSOLE_H_
#define _BOKIM_DEBUGGING_CONSOLE_H_

//#include "windows.h"
#include "afxwin.h" // windows.h 파일을 include 하지 말고 afxwin.h 파일을 include 해라...

#define CONSOLE_BLACK			(FOREGROUND_INTENSITY)
#define CONSOLE_BLUE			(FOREGROUND_BLUE|FOREGROUND_INTENSITY)
#define CONSOLE_GREEN			(FOREGROUND_GREEN|FOREGROUND_INTENSITY)
#define CONSOLE_RED				(FOREGROUND_RED|FOREGROUND_INTENSITY)
#define CONSOLE_CYAN			(FOREGROUND_BLUE|FOREGROUND_GREEN|FOREGROUND_INTENSITY)
#define CONSOLE_YELLOW		(FOREGROUND_GREEN|FOREGROUND_RED|FOREGROUND_INTENSITY)
#define CONSOLE_MAGENTA		(FOREGROUND_BLUE|FOREGROUND_RED|FOREGROUND_INTENSITY)
#define CONSOLE_WHITE			(FOREGROUND_BLUE|FOREGROUND_GREEN|FOREGROUND_RED|FOREGROUND_INTENSITY)

#define CONSOLE_BLACK_D		0x00
#define CONSOLE_BLUE_D		(FOREGROUND_BLUE)
#define CONSOLE_GREEN_D		(FOREGROUND_GREEN)
#define CONSOLE_RED_D			(FOREGROUND_RED)
#define CONSOLE_CYAN_D		(FOREGROUND_BLUE|FOREGROUND_GREEN)
#define CONSOLE_YELLOW_D	(FOREGROUND_GREEN|FOREGROUND_RED)
#define CONSOLE_MAGENTA_D	(FOREGROUND_BLUE|FOREGROUND_RED)
#define CONSOLE_WHITE_D		(FOREGROUND_BLUE|FOREGROUND_GREEN|FOREGROUND_RED)

#define CONSOLE_S CRT_Console::getConsole().PRINT
#define CONSOLE_D CRT_Console::getConsole().PRINT_D

class CRT_Console
{
  // member variables
protected:
  HANDLE m_hIn;
  HANDLE m_hOut;

  // member functions


  // interface
public:
	BOOL destroy();
	BOOL create();

	void PRINT_D(char* _formatString, ...);
	void PRINT_D(char TextColor, char* _formatString, ...);
	void PRINT_D(char TextColor, char BackColor, char* _formatString, ...);
	void PRINT(char* _formatString, ...);
	void PRINT(char TextColor, char* _formatString, ...);
	void PRINT(char TextColor, char BackColor, char* _formatString, ...);

	static CRT_Console &getConsole( void )
	{
		static CRT_Console gt;
		return gt;
	}
 
  // constructor and destructor
  CRT_Console();
  ~CRT_Console();

};



#endif//_BOKIM_DEBUGGING_CONSOLE_H_


// RFT_IF_CAN.h : PROJECT_NAME ���� ���α׷��� ���� �� ��� �����Դϴ�.
//

#pragma once

#ifndef __AFXWIN_H__
	#error "PCH�� ���� �� ������ �����ϱ� ���� 'stdafx.h'�� �����մϴ�."
#endif

#include "resource.h"		// �� ��ȣ�Դϴ�.


// CRFT_IF_CANApp:
// �� Ŭ������ ������ ���ؼ��� RFT_IF_CAN.cpp�� �����Ͻʽÿ�.
//

class CRFT_IF_CANApp : public CWinApp
{
public:
	CRFT_IF_CANApp();

// �������Դϴ�.
public:
	virtual BOOL InitInstance();

// �����Դϴ�.

	DECLARE_MESSAGE_MAP()
};

extern CRFT_IF_CANApp theApp;
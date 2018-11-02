
// RFT_IF_CANDlg.h : ��� ����
//

#pragma once

#include "afxwin.h"

////////////////////////////////////////////////////////////////////////////////
#include "RFT_IF_UART_SAMPLE_Rev1.0.h"


// for graph
#include "ChartCtrl.h"
#include "ChartLineSerie.h"
#include <vector>
using namespace std;


// CRFT_IF_CANDlg ��ȭ ����
class CRFT_IF_CANDlg : public CDialogEx
{
// �����Դϴ�.
public:
	CRFT_IF_CANDlg(CWnd* pParent = NULL);	// ǥ�� �������Դϴ�.

// ��ȭ ���� �������Դϴ�.
	enum { IDD = IDD_RFT_IF_CAN_DIALOG };

	protected:
	virtual void DoDataExchange(CDataExchange* pDX);	// DDX/DDV �����Դϴ�.

public:

	//////////////////////////////////////////////////////////////////////////
	// FOR GRAPH
	CChartCtrl m_ChartCtrl_Force;
	CChartCtrl m_ChartCtrl_Torque;
	CChartLineSerie *m_pGraph_F[RFT_NUM_OF_FORCE];

	vector<double> m_vGraphDatas_F[RFT_NUM_OF_FORCE];

	//////////////////////////////////////////////////////////////////////////
	// FOR RFT
	CRT_RFT_UART m_RFT_IF;

	static void callback_RFT_Data_Receive(void *callbackParam);

	//
	//////////////////////////////////////////////////////////////////////////

// �����Դϴ�.
protected:
	HICON m_hIcon;

	// ������ �޽��� �� �Լ�
	virtual BOOL OnInitDialog();
	afx_msg void OnSysCommand(UINT nID, LPARAM lParam);
	afx_msg void OnPaint();
	afx_msg HCURSOR OnQueryDragIcon();
	DECLARE_MESSAGE_MAP()
public:
	virtual BOOL PreTranslateMessage(MSG* pMsg);
	afx_msg void OnDestroy();
	afx_msg void OnTimer(UINT_PTR nIDEvent);
	CComboBox m_combo_baudrate;
	afx_msg void OnBnClickedCheckInterfaceOpen();
	CString m_strRxData;
	afx_msg void OnBnClickedCheckBias();
	afx_msg void OnBnClickedCheckFtOutCont();
	int m_nCOM_PortNumber;
	afx_msg void OnBnClickedReadOverloadCount();
	CComboBox m_combo_Filter_Cutoff_Frq;
	afx_msg void OnBnClickedButtonFilterSetting();
};

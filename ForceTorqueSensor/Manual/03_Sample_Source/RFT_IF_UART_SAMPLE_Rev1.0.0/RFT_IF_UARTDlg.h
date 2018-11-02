
// RFT_IF_CANDlg.h : 헤더 파일
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


// CRFT_IF_CANDlg 대화 상자
class CRFT_IF_CANDlg : public CDialogEx
{
// 생성입니다.
public:
	CRFT_IF_CANDlg(CWnd* pParent = NULL);	// 표준 생성자입니다.

// 대화 상자 데이터입니다.
	enum { IDD = IDD_RFT_IF_CAN_DIALOG };

	protected:
	virtual void DoDataExchange(CDataExchange* pDX);	// DDX/DDV 지원입니다.

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

// 구현입니다.
protected:
	HICON m_hIcon;

	// 생성된 메시지 맵 함수
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

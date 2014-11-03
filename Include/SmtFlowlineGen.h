// NCSFFlowlineGen.h: interface for the CFlowlineGen class.
//
//////////////////////////////////////////////////////////////////////
#ifndef __SMART_FLOWLINE_GEN_H__
#define __SMART_FLOWLINE_GEN_H__

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#define NCSF_TOL_01 0.01 
#define NCSF_TOL_M4 1.0e-4

//��¼������˹�ϵ�ͼӹ�����
class C3DSurfAttribute
{
public:
	int				m_nSameNormal;		//1��ʸͬ��,-1��ʸ����
	int				m_nSense;			//1��ʾ����ȡ����,-1��ʾ����
	int				m_nCutDir;			//0Ϊu��,1Ϊv��
	CGeoTrmSurf*	m_pSurf;			//��Ӧ�����ָ��
	CGeoCurve		*m_pPCurvStart, *m_pPCurvEnd;//�������ָ��
	CTrmFin			*m_pBoundStart, *m_pBoundEnd;//�������Boundָ��
	double			m_dUStep, m_dWStep;
	double			m_dUOverStep, m_dWOverStep;
	BOOL			m_bTakeOutFromHead;
	BOOL			m_bPathReverse;
	BOOL			m_bSpiral	  ;		//����Ƿ�Ϊ����
	CSmtCPathLib	m_cSmtCPathLib;
	// add by yul	
	BOOL            m_bReverseSurf ;     // �����Ƿ���
public:
	C3DSurfAttribute(CGeoTrmSurf* pSurf, CTrmFin* pBnd1, CTrmFin* pBnd2, int nCurDir )
	{
		m_pSurf=pSurf;
//		m_pPCurvStart=pBnd1->m_pEdge->m_pCurve;
//		m_pPCurvEnd=pBnd2->m_pEdge->m_pCurve;

		// By Yul
		m_pPCurvStart=pBnd1->m_pPCurve;
		m_pPCurvEnd=pBnd2->m_pPCurve;

		m_pBoundStart=pBnd1;
		m_pBoundEnd=pBnd2;
		m_nSense=nCurDir;
		m_nSameNormal=1;
		m_nCutDir=0;
		m_dUOverStep=m_dWOverStep=0.02;
		m_dUStep=m_dWStep=0.02;
		m_bSpiral = FALSE ;
	};
	~C3DSurfAttribute(){};
};

typedef CTypedPtrArray<CPtrArray, C3DSurfAttribute*> C3DSurfAttArray;

// Add by Yul
typedef struct _pt_seg PtSeg ;
struct _pt_seg 
{
	PNT3D pnt ; // �������
	int   num ; // ������
} ;
typedef CTypedPtrList<CPtrList, PtSeg*> PTList ;

class CSurfLoop 
{
public:
	CTrmLoop* m_pLoop ;
	CGeoTrmSurf* m_pSurf ;
	
public:
	CSurfLoop( CGeoTrmSurf* pSurf, CTrmLoop* pLoop )
	{
		m_pLoop = pLoop ;
		m_pSurf = pSurf ;
	}
	~CSurfLoop() 
	{} ;
} ;

//��¼��߹�ϵ���Ա���ݹ�������������ӹ�ϵ
class CAdjEdge : public CTrmEdge
{
public:
	CAdjEdge(CGeoCurve* pCurve, CGeoTrmSurf* pSurf1, CGeoTrmSurf* pSurf2, CTrmFin* pBnd1, CTrmFin* pBnd2, int nSameDir)
	{
		m_pCurve=pCurve;
		m_pRefer[0]=pSurf1, m_pRefer[1]=pSurf2;
		m_pFins[0]=pBnd1,  m_pFins[1]=pBnd2;
		pBnd1->m_nSense=pBnd2->m_nSense=nSameDir;	//����Ӧ��ʹ�����������������������Ƿ�ͬ��
	};
	~CAdjEdge() {};
public:
	void SwapBounds()
	{
		CGeoTrmSurf* pSurf=m_pRefer[0]; m_pRefer[0]=m_pRefer[1]; m_pRefer[1]=pSurf;
		CTrmFin* pBnd=m_pFins[0]; m_pFins[0]=m_pFins[1]; m_pFins[1]=pBnd;
	};
};

typedef CTypedPtrArray<CPtrArray, CAdjEdge*> CAdjEdgeArray;
typedef CArray<int, int&> CIntArray;
//���ڼ�¼���߼ӹ�����
class CUWLineParam
{
public:
	int					m_nCutCross  ;	//0Ϊu��(����), 1Ϊv��(����)
	int					m_nOutSide   ;
	C3DSurfAttArray		m_cSurfAttArr;
	CAdjEdgeArray		m_cAdjEdgeArr;
	C3DSurfArray		m_Surfs		 ;	// ���漯�ϣ�������ʱ����

	int				    m_nStartAt   ;  // ·�����𵶵��λ��
	BOOL                m_IsClosed   ;  
	BOOL				m_IsAllQuad  ;
	BOOL                m_bSmooth    ;  // ���Ƿ�⻬

public:
	CUWLineParam() {
		m_nStartAt = 0;
		m_nCutCross = 0;
		m_nOutSide = 1;
	}
	CUWLineParam(int nCutDir, int nOffsetDir, int nStartAt = 0 )
	{
		if( nCutDir==0 ) m_nCutCross=0;
		else m_nCutCross=1;
		if( nOffsetDir==0 )	m_nOutSide=-1;
		else m_nOutSide = 1;
		m_nStartAt = nStartAt ;
	};
	void operator =(const CUWLineParam &UWLPara) {
		m_nStartAt = UWLPara.m_nStartAt;
		m_nCutCross = UWLPara.m_nCutCross;
		m_nOutSide = UWLPara.m_nOutSide;
	}
	void MoveTo(CUWLineParam &UWLPara){
		UWLPara.m_nCutCross = m_nCutCross;
		UWLPara.m_nOutSide = m_nOutSide;
		UWLPara.m_nStartAt = m_nStartAt;
		UWLPara.m_IsClosed = m_IsClosed;
		UWLPara.m_IsAllQuad = m_IsAllQuad;
		UWLPara.m_bSmooth = m_bSmooth;
        UWLPara.m_cSurfAttArr.Append(m_cSurfAttArr);
		m_cSurfAttArr.RemoveAll();
		UWLPara.m_cAdjEdgeArr.Append(m_cAdjEdgeArr);
		m_cAdjEdgeArr.RemoveAll();
		UWLPara.m_Surfs.Append(m_Surfs);
		m_Surfs.RemoveAll();
	}
	void Set(int nCutDir, int nOffsetDir, int nStartAt = 0){
		if( nCutDir==0 ) m_nCutCross=0;
		else m_nCutCross=1;
		if( nOffsetDir==0 )	m_nOutSide=-1;
		else m_nOutSide = 1;
		m_nStartAt = nStartAt ;
	}
	int FindSurf(CGeoSurf *pSurf){
		for (int i=0; i<m_Surfs.GetSize(); i++)
			if (m_Surfs.GetAt(i) == pSurf)
				return i;
		return -1;
	}
	~CUWLineParam()
	{
		INT_PTR i, n=m_cSurfAttArr.GetSize();
		for(i=0; i<n; i++) 
			delete m_cSurfAttArr[i];
		m_cSurfAttArr.RemoveAll();

		n=m_cAdjEdgeArr.GetSize();
		for(i=0; i<n; i++)
		{
			m_cAdjEdgeArr[i]->m_pCurve=NULL;
			delete m_cAdjEdgeArr[i];
		}
		m_cAdjEdgeArr.RemoveAll();
	};
};

typedef CArray<CUWLineParam, CUWLineParam&> CUWLParaArray;

class CFlowlineGen;
class CNc5DAxisCtrl;
//////////////////////////////////////////////////////////////////////////
// �߳����ݶ���
// ������������5��ԭʼ�ӹ�·�����߳�����
typedef struct CreateOne5AxSurfPathThreadData  
{
	int nAtCore;
	int nCoreNum;
	LPVOID* NewPath;
	int nUW;
	double *uwStep;
	CSmtCheckMdl *DriveMdl;
	RFRAME *cRFrame;
	CGeoTrmSurf *pSurf;
	CNc5DAxisCtrl *pAxisCtrl;
	CSmartLoop *AllLoop;
	CSmtCPathLib* AllPath;
	JDNC_PRGDEF PrgDef;	
	BOOL bZigZag ;
	CFlowlineGen *pFlowlineGen;
}CO5SPT_DATA;

// ������������3��ԭʼ�ӹ�·�����߳�����
typedef struct CreateOneSurfPathThreadData  
{
	int nAtCore;
	int nCoreNum;
	double dArcTol;
	BOOL bOffDir;
	CSmartTool* pMiller;
	RFRAME *cRFrame;
	CGeoTrmSurf* pSurf;
	JDNC_PRGDEF PrgDef;
	CSmtCPathLib *PathLib;
	CFlowlineGen *pFlowlineGen;
}COSPT_DATA;

// �����������߳�����
typedef struct CheckAllPathThreadData  
{
	int nAtCore;
	int nCoreNum;
	LPVOID* NewPath;
	double dCur;
	BOOL bCheck;			// ��ʱ���ӵĲ���
	CSmtCheckMdl *pDriveMdl;
	JDNC_TOL *pTol;
	JDNC_PRGDEF PrgDef;
	CSmtCPathLib *AllPath;
	CFlowlineGen *pFlowlineGen;
}CAPT_DATA;

//////////////////////////////////////////////////////////////////////////
// ���������ߵ�·�����̺߳���
// ������������5��ԭʼ�ӹ�·��
static UINT MathCAM_CreateOne5AxSurfPathSubProc(LPVOID lpParam);
// ������������3��ԭʼ�ӹ�·��
static UINT MathCAM_CreateOneSurfPathSubProc(LPVOID lpParam);
// ���������
static UINT MathCAM_CheckAllPathBySelfSubProc(LPVOID lpParam);
// �����������
static UINT MathCAM_CheckAllPathBySurfSubProc(LPVOID lpParam);
// ������ qqs new 2013.11.07
static UINT MathCAM_CheckAllPathByCheckMdlSubProc(LPVOID lpParam);
//////////////////////////////////////////////////////////////////////////

typedef CTypedPtrArray<CPtrArray, CUWLineParam*> CUWLineArr;
typedef CTypedPtrArray<CPtrArray, CSurfLoop*> CSurfLoopArr;

class CIsoSGuideTPath ;
class CNc5DAxisCtrl   ;

#define CUTPNT_INSERT_TMP 100 //���ڱ�ǲ���Ľڵ� qqs 2013.11.06
class CFlowlineGen  
{
public:
	CFlowlineGen( JDNC_SETUP &cSetup, JDNC_FEED &cFeed,
				  JDNC_STOCKEX &cStock, JDNC_TOOLEX &cTool, 
				  JDNC_SPEED &cSpeedDef, CSmartTool *pTool, 
				  int nThreadNum, JDNC_UWLINE &m_cParam) ;
	virtual ~CFlowlineGen();
	
	friend UINT MathCAM_CreateOne5AxSurfPathSubProc(LPVOID lpParam);
	friend UINT MathCAM_CreateOneSurfPathSubProc(LPVOID lpParam);
	// ����·������
	void ModifyPathDirection(CSmtCPathList &PathList);
protected://���������ߵ��ĺ���
	void ChangePCurve(C3DSurfAttribute* pSurfAtt, bool bRestore); //���ڱ�
	void ChangePCurve(C3DSurfAttArray& c3DSurfAttArr, bool bRestore); //���ڱ�

	//ͨ������Ĺ���,�����������,�ҳ����ӵ����˹�ϵ
	int	 IsSameBound(CTrmFin* Bnd1, CTrmFin* Bnd2);
	void SearchAdjacentEdge(CGeoTrmSurf* pSurf1, CGeoTrmSurf* pSurf2, CAdjEdgeArray& cAdjEdgeArr);
	CTrmFin* GetSubtense(CTrmFin* pBndSrc);//�ҶԱ�
	C3DSurfAttribute* CreateSurfAtt(CGeoTrmSurf* pSurf, CTrmFin* pBnd1, CTrmFin* pBnd2, int nCurDir);
	int OrderAllSurf(C3DSurfArray& c3DSurfArr, CAdjEdgeArray& cAdjEdgeArr, C3DSurfAttArray& c3DSurfAttArr);

	
	//����dArcTol����Ҫ��,�öԷֲ��ķ������������ϵ���ӳ�䵽������
	void ImagePntToSurf( CSmartTool *pMiller, RFRAME& cRFrame, CGeoTrmSurf* pSurf, 
						 CSmtCutPath* pPath, double* dParam, BOOL bOffDir, PNT3D dPoint[2] );
	void BisectInsert(  CSmartTool *pMiller, RFRAME& cRFrame, CGeoTrmSurf* pSurf, 
						CSmtCutPath* pCutPath, CSmtCutPointEx* pPnt, BOOL bOffDir,
						PNT3D pt1, PNT3D pt2, double* param1, double* param2,
						double dArcTol, int &nCnt );
	void BisectInsertPnt( CSmartTool *pMiller, RFRAME& cRFrame, CGeoTrmSurf* pSurf, BOOL bOffDir, 
						  CSmtCutPath* pCutPath, CSmtCutPointEx *pStart, CSmtCutPointEx *pEnd );
	BOOL CutPathToSurf(CSmartTool *pMiller, RFRAME& cRFrame, CSmtCutPath* pCutPath, CGeoTrmSurf* pSurf, BOOL bOffDir, double dArcTol);
	void RenewPathByStepAndSurf ( CSmtCutPath *&pPath, double dStep, CGeoTrmSurf *pSurf, RFRAME &lf, BOOL bFlag = FALSE, BOOL b5Axis = FALSE ) ;
	BOOL CalcToolPos( CGeoTrmSurf *pSurf, CSmartTool *pTool, JDNC_TOL &cTol, PNT3D Tip, PNT3D Nor, PNT3D Axis, RFRAME &lf, double &du, double &dw ) ;
	//����ZBQ��CreateLinearPathֻ�ܲ����Ⱦ�����·��,�����߼ӹ��������ϵ�·����಻��,�ʱ�д��CreateLinear������������·��
	int  GenParamSerialByStep( CGeoPLine3d* pPLine, double dStep, double* dParam, int& nNum );
	void GenParamSerialByStep( CSmartTool *pMill, RFRAME &lf, CGeoTrmSurf* pSurf, double dStep, int nCutDir, 
							   double*& dParam, int& nNum, BOOL bOffDir, double dArcTol, CSmtCPathLib &AllPath );
	void CreateLinear( double* dParam, int nNum, int nCutDir, BOOL bZigZag, CSmartLoop* pLoop, CSmtCPathLib& cSmtCPathLib );
	// �������߷��򣬵õ�����Ǹ�·������u/v = 0��0.5��1��ѡ��
	CSmtCutPath *GetMaxLenPath(CSmartTool *pMill, RFRAME &lf, CGeoTrmSurf* pSurf, 
		CSmartLoop* pLoop, int nCutDir, BOOL bOffDir, double dArcTol);
	//���������,����pnt1(ĳ����Ϊ��ʱ������pnt2)��nCutCross��cSmtCPathLib��·���ҵ���������ȡ����,
	//������ͨ��cSmtCPathLib���ĩ��Ϊpnt1��pnt2��ֵ,�Ա�������һ�����.
	//nNearestPnt[0]��0x0001��ʾ·������0x0002��ʾ��cSmtCPathLib��ͷ����ȡ
	int FindNearestPnt( PNT3D pnt1, PNT3D pnt2, CSmtCPathLib& cSmtCPathLib, int nCutCross );
	//����FindNearestPnt��������·������ȡ����
	void SetTakeOutOrder(C3DSurfAttArray& c3DSurfAttArr, int nCutCross);

	//��������ӹ�·�������
	void AdjustStartAt( int nCutDir, int nStartAt, CSmtCPathLib& cSmtCutPathLib );
	// ����ͬ�кŵ�·��
	void ReverseSameLineNoPath(CSmtCPathList &PathList);
	// ����·����˳��
	void ReverseLinePathOrder(CSmtCPathList &PathList);
	// ����·�����к�
	void SetLineNo(CSmtCPathList &PathList, int nLineNo);
	
	// �պ��������������
	void AdjustSpiralPath( CSmtCPathLib &cSmCutPathLib ) ;
	// ��·����Ĳ�Ÿ�Ϊԭֵ0
	void ModifyLayerNo(CSmtCPathLib& PathLib);
	//���վ����·������
	int	 CreateOneSurfPath(CSmartTool* pMiller, RFRAME& cRFrame, CGeoTrmSurf* pSurf, JDNC_UWLINE& cParam,
		 			       int nCutDir, double dArcTol, CSmtCPathLib& cParamPathLib, JDNC_PRGDEF &PrgDef, BOOL &bSpiral);
	// ���������ߵ�·����������
	BOOL CreateOneSurfPathSubProc(double dArcTol, BOOL bOffDir, CSmartTool* pMiller, RFRAME& cRFrame, 
		CGeoTrmSurf* pSurf, CSmtCPathLib& cParamPathLib, JDNC_PRGDEF &PrgDef, int nAtCore, int nCoreNum);
	// ����������������ϲ������·����
	BOOL CombinePathPoint(CSmtCPathLib& PathLib);
	//��������������ߵ�·����nStartLayerNoΪ��ʼ��·����ţ��������һ��Ĳ�ż�1
	int  CreateEachSurfPath(CSmartTool* pMiller, RFRAME& cRFrame, C3DSurfArray& AllSurf, JDNC_UWLINE& cParam, 
						    double dArcTol, CSmtCPathLib& cSmtCPathLib, int &nStartLayerNo, JDNC_PRGDEF &PrgDef, BOOL *arrbSpiral );
	// ����·���Ĳ�ź��кţ�����·��������·������
	void ModifyPathAndCpy(CSmtCPathLib& OldPathLib, int &nStartLayerNo, CSmtCPathLib& NewPathLib, BOOL bModifyLineNo = TRUE);
	void OrderAllPath(C3DSurfAttArray& c3DSurfAttArr, int nCutCross, 
			BOOL bZigZag, CSmtCPathLib& cSmtCPathLib ); //Ϊ��������߼ӹ���·������
	// ����水��Զ����������
	void SortAllSurfByDist( C3DSurfArray &AllSurf, double dTol ) ;	
	// �õ�ƫ�Ʒ���
	BOOL GetOffSetDir(CGeoTrmSurf* pSurf, RFRAME& cRFrame, JDNC_UWLINE& cParam);

    // ���º������������ڼ�����������ĥ������·�� qqs 2013.05.20
	//////////////////////////////////////////////////////////////////////////
public:
	BOOL            m_bGrandMdfy;   // ��¼�Ƿ�����ĥ�������ӹ�

	void CreateLinearNew( double* dParam, int nNum, int nCutDir, CSmartLoop* pLoop, CSmtCPathLib& cSmtCPathLib , RFRAME& cRFrame, 
		                  double dArcTol, CGeoTrmSurf* pSurf, JDNC_UWLINE& cParam, BOOL b5Axis = FALSE ); // ��������·������ĥ�������任
	void GrindParamPathGen( CSmtCutPath* pNew, JDNC_UWLINE& cParam, RFRAME& cRFrame,  CGeoTrmSurf* pSurf, 
		                    int nCurDir, double dArcTol, PNT4D dHead, PNT4D dTail, BOOL b5Axis = FALSE );	
	BOOL ModifyGrindParamPath( CSmtCutPath* pParamPath, CSmtCutPath* pCutPath, int nType, DOUBLE depthInc, int  SinSegs, int nSect, int nCutDir );
	double GetMdfyHeit(CSmartTool *pMill, JDNC_UWLINE cParam, RFRAME &lf, CGeoTrmSurf* pSurf, int nCutDir, BOOL bOffDir, double dArcTol);
	void InsertNewCutPoint(CSmtCutPath * CutPath ,int  nPoint);
	void CreateSinWave(CSmtCutPath * CutPath,DOUBLE depthInc,int nSect, int uwDir);//�����Ҳ���
	void CreateTriWave(CSmtCutPath * CutPath,DOUBLE depthInc,int nSect, int uwDir);//�����β���
	void CreateTrapWave(CSmtCutPath * CutPath,DOUBLE depthInc,int nSect, int uwDir);//���β���
	BOOL IsSet3DRCompMask(); // �ж��Ƿ����˰뾶ĥ�𲹳�
	
	void AdjustSpiralCurve ( CSmartCurve &AllLine ); //���ڼ��㹹��������������
	//////////////////////////////////////////////////////////////////////////

// ���º���ΪYul��ʹ��
public:
	CSmtCPathLib	m_cSmtCPathLib;
	CSmartTool   *  m_pMiller     ;
	double			m_dStep       ; // ����
	int             m_nNum        ; // ΪU���ߵ�ʱ���߶���
	double          m_dZMove      ; // ����̧��
	JDNC_SETUP      m_cSetupDef   ;
	JDNC_FEED		m_cFeedDef	  ;
	JDNC_PRGDEF		m_cPrgDef     ; // �������
	JDNC_5DCTRL		m_c5DCtrlDef  ; // �������
	JDNC_STOCKEX	m_cStockDef	  ;
	JDNC_TOOLEX		m_cToolDef	  ;
	JDNC_SPEED		m_cSpeedDef   ;
	int *m_pErrorType;
	int				m_nCalcThreadNum;
	// û�и���ģ��
	BOOL CreateUWLinePath( CSmtCPathLib& AllPath, C3DSurfArray& AllSurf	,
		                   RFRAME& cRFrame, JDNC_FUWLINE& cParam,
						   JDNC_PRGDEF &PrgDef, double dCur, int *pErrorType, BOOL *arrbSpiral = NULL) ;
	// ���ɱ߽�
	void CreateStitchBnd( C3DSurfArray &AllSurf,
						  C3DSurfArray &tmpSurf,
                          CSurfLoopArr &cLoopArr ) ;
	// ��������
	void CopyAllSurf ( C3DSurfArray &AllSurf, C3DSurfArray &tmpSurf ) ;
	// ɾ������
	void DestroyAllSurf( C3DSurfArray &AllSurf, CUWLParaArray& ParaArr ) ;
	// ɾ���߽�
	void DestroyBnd( C3DSurfArray &AllSurf, CSurfLoopArr &cLoopArr ) ;
	// �õ���Ч����
	void GetValidSurf( C3DSurfArray& AllSurf ) ;
	// ��������ı߽�
	CTrmLoop* CopySurfLoop( CGeoTrmSurf* pSurf ) ;
	// ���´������棬�ñ߽綼Ϊ��ʱ�룬�ع�����߽磬�޳�������
	void ResetAllSurf( C3DSurfArray& AllSurf ) ;
	// �ж����������Ƿ���ڹ⻬���ӻ�λ������������ϣ��������õ�ParaArr�У������������õ�otherSurf��
	BOOL AllSurfAdjacentConnect( C3DSurfArray& AllSurf,
								 C3DSurfArray& otherSurf,
								 CUWLParaArray& ParaArr, 
								 BOOL bSmooth) ;
	// �����Ϊ�ı���ͷ��ı���
	BOOL SortSurf( C3DSurfArray& AllSurf,
				   C3DSurfArray& QuadSurf ) ;
	BOOL IsQuadSurf( CTrmLoop* pLoop ) ;

	// ���������˳�򣬲��ж������Ƿ�������߼ӹ�������
	BOOL  AdjustAllSurfType( C3DSurfArray& AllSurf,
							 CUWLParaArray& ParaArr ) ;
	// ���ݼӹ������������߲���
	void SetUWLParaData(CUWLParaArray &ParaArr, JDNC_FUWLINE& UWLineCut);	
	// ���������߲��������е����ݴ�һ��������һ��
	BOOL MoveUWLArrayData(CUWLParaArray &ParaFrm, CUWLParaArray &ParaTo);
	// �ж��Ƿ���ڹ⻬�����棬������ڣ�������Ĳ�������ParaArr��
	BOOL JudgeSmoothSurf(C3DSurfArray& AllSurf, CUWLParaArray& ParaArr);
	// �ж���������Ļ��Ƿ����ı���
	BOOL IsAllSurfQuad(C3DSurfArray& AllSurf);
	// �ж��Ƿ����λ�����������棬������ڣ�������Ĳ�������ParaArr��
	BOOL JudgeConnectSurf(C3DSurfArray& AllSurf, CUWLParaArray& ParaArr);
	// ����������������·��
	BOOL CreateAdjSurfUWPath(CSmtCPathLib &AllPath, RFRAME &cRFrame, JDNC_FUWLINE &UWLineCut, 
		CUWLineParam &cUWLine, int &nStartLayerNo, JDNC_PRGDEF & PrgDef);
	// ���������е������Ƶ��µ���
	void AddAdjEdgeToArray( CAdjEdgeArray& OldEdgeArr, 
							CAdjEdgeArray& NewEdgeArr ) ;
	// ���������е������Ƶ��µ���
	void AddAttSurfToArray( C3DSurfAttArray& OldSurfArr, 
							C3DSurfAttArray& NewSurfArr ) ;
	void DeleteAdjEdgeArr ( CAdjEdgeArray& cAdjEdgeArr ) ;
	void DeleteAdjSurfArr ( C3DSurfAttArray& cSurfArr ) ;
	// ��������ڱ�
	void SearchAllSurfBound( C3DSurfArray& AllSurfArr, C3DSurfArray& LeftSurfArr, CUWLParaArray& ParaArr, BOOL bSmooth ) ;
	//�л�tempSurfArr����һ��pSurf�����û�У���bFlag��ΪFALSE  qqs 2013.01.31
	void ResetFirstSurf(C3DSurfArray& tempSurfArr, CGeoTrmSurf** pSurf, BOOL& bFlag);
	// ��������������ϵ����飬����ҵ������е�һ���棬�����������ӵ���Ӧ��λ�ã��������µ�λ�����������
	int SearchAndSetSurfArray(CUWLParaArray& ParaArr, CGeoSurf *pSurf1, CGeoSurf *pSurf2);
	// �����ڱ�
	BOOL SearchAdjacentBound( CGeoTrmSurf* pSurf1, 
							  CGeoTrmSurf* pSurf2,
							  CAdjEdgeArray& cAdjEdgeArr,
							  BOOL bSmooth ) ;

	// �ж�����߽��Ƿ��غϡ��߽�pFin1,pFin2,dTolΪ����,bSameDirΪ����
	BOOL JudgeTwoBoundOverlap( CTrmFin* pFin1, CTrmFin* pFin2, 
							   double dTol, BOOL& bSameDir ) ;
	// �ж�����ı߽��Ƿ�⻬����
	BOOL JudgeTwoBoundSmooth ( CGeoTrmSurf* pSurf1, CTrmFin* pFin1,	
							   CGeoTrmSurf* pSurf2,	CTrmFin* pFin2,
							   double dTol, BOOL& bSameDir, BOOL bSmooth ) ;
	// �õ������е㴦�������ϵķ�ʸ
	void GetCurveMidNormal ( CGeoTrmSurf* pSurf, CTrmFin* pFin, VEC3D normal ) ;

	BOOL IsEligibleVec( VEC3D u1, VEC3D u2, double Tol ) ;
	// ������������ı߽���
	void RebuildSurfBnd( CGeoTrmSurf* pSurf ) ;
	BOOL IsGeoSurfClosed( CGeoTrmSurf *pSurf, int &nUW ) ;
	void DiscreteCurve( CGeoCurve* pCurve, PTList& ptList, BOOL bFst ) ;
	void GetLineTangent( CGeoCurve* pCurve, double u, PNT3D pos, VEC3D du ) ;
	void CreateSurfEdge( CGeoTrmSurf* pSurf, CTrmLoop* pLoop ) ;
	void AddPTListPt( PNT3D pt, PTList& ptList ) ;
	CGeoPLine3d* CreatePLine( PTList& ptList ) ;
	CTrmFin* DeleteBound( CTrmFin* pOldBnd, CTrmLoop* pLoop ) ;
	void AddBound( CTrmFin* pOldBnd, CTrmFin* pNewBnd, CTrmLoop* pLoop ) ;
	void ConnectHeadTailCurve( CTrmLoop* pLoop ) ;
	void DeleteTrmLoop( CTrmLoop* pLoop ) ;

	// ����·�������
	void AdjustPathStart( CUWLineParam& cUWLine ) ;
	// �ж��������Ƿ���β����
	BOOL IsSurfArrayColsed( C3DSurfAttArray& cSurfArr ) ;

	// �õ�����ļӹ���ʸ
	void GetSurfNormal( C3DSurfAttArray& cSurfArr, 
					    RFRAME& cRFrame, 
						int nOffsetDir ) ;
	// �������淨ʸ�ķ��򣬵�������߽��ߵķ���Ͳ�����ķ���
	void AdjustAllSurfDir( C3DSurfAttArray& cSurfArr, 
						   RFRAME& cRFrame, 
						   int nOffsetDir ) ;
	// �ж��������߲����Ƿ����
	BOOL IsUWLineParamValid(const CUWLineParam &cUWLine);
	void DeleteSurfEdge ( CGeoTrmSurf* pSurf ) ;
	// ����ı߽練��
	void ReverseSurfEdge( CGeoTrmSurf* pSurf ) ;
	// 0,��ʾ��һ��,1,��ʾ�м�,2��ʾ���һ��
	void ReverseAttSurf( C3DSurfAttribute* pSurfAtt,
						 BOOL bReverse, int nType ) ;
	// �õ����湲����ĳ��ķ�ʸ
	void GetAttSurfNormal( C3DSurfAttribute* pSurfAtt,
						   BOOL bStart,  VEC3D normal ) ;
	CGeoCurve* GetPrevCurve( C3DSurfAttribute* pSurfAtt,
							 CGeoCurve* pCurve ) ;
	CGeoCurve* GetNextCurve( C3DSurfAttribute* pSurfAtt,
							 CGeoCurve* pCurve ) ;
	// �õ����湲���ϵĵ�ķ���
	void GetAttSurfSameLinePntNor( C3DSurfAttribute* pSurfAtt,  BOOL bStart,  VEC3D nor ) ;

	// ����·����˳��
	void ReverseLineOrder( C3DSurfAttribute* pSurfAtt ) ;
	void ReverseLineOrder( CSmtCPathList& pathList ) ;

	// �������ϵ�·��ӳ�䵽�ռ���ĺ���
	BOOL TransfPathToSurf( C3DSurfAttribute* pSurf, CSmartTool *pMiller, 
						   RFRAME& cRFrame, CSmtCutPath* pCutPath ) ;
	
	void ImagePntToSurf( CGeoTrmSurf* pSurf, CSmtCutPath* pPath, CSmartTool* pMiller, 
						 RFRAME& cRFrame, PNT2D param, PNT3D dPoint[2] ) ;

	void BisectInsert  ( CGeoTrmSurf* pSurf, CSmartTool* pMiller,
						 CSmtCutPath* pPath, RFRAME& cRFrame,
						 CSmtCutPointEx* dStart, PNT2D start,
						 CSmtCutPointEx* dEnd  , PNT2D end  ) ;
	void BisectInsertPnt( CGeoTrmSurf* pSurf, CSmartTool* pMiller,
						  CSmtCutPath* pPath, RFRAME& cRFrame,
						  CSmtCutPointEx* dStart, CSmtCutPointEx* dEnd ) ;

	void GetPointNormal( CGeoTrmSurf* pSurf, CSmtCutPath* pPath, CSmartTool* pMiller,
						 PNT2D param, VEC3D dNormal, PNT3D dPoint ) ;
	// �õ�ǰ��ķ�ʸ
	void GetNextNormal ( CGeoTrmSurf* pSurf, CSmtCutPath* pPath,
						 PNT2D param, VEC3D dNormal, PNT3D dPoint ) ;
	// �õ����ҵķ�ʸ
	void GetSideNormal ( CGeoTrmSurf* pSurf, CSmtCutPath* pPath,
						 PNT2D param, VEC3D dNormal, PNT3D dPoint ) ;

	CSmtCutPath* millface_CurveToCPath( CSmartCurve&  Curve ,JDNC_TOL& Tol ) ;

	int GetPoint( CGeoPLine3d *pLine, double tmp, double dLength, PNT3D pos ) ;
	int GetPoint( CGeoLine *pLine, double tmp, double dLength, PNT3D pos ) ;

	void AddPathPointEx( CSmtCutPath *pPath, PNT4D pnt ) ;
	void AddPathPointEx( CSmtCutPath *pPath, float pnt[4] ) ;
// ��һ�ּ��㷽�������¹�����������߷���
public:

	// ��һ�ַ���
	BOOL CreateUWLineByReConstruct( CSmtCPathLib& cSmtCPathLib,
							        CUWLineParam& cUWLine,
									JDNC_FUWLINE& UWLineCut,
							        RFRAME& cRFrame, 
							        JDNC_SETUP& Setup, 
									int &nStartLayerNo,			// ��ʼ��·�����
									JDNC_PRGDEF &PrgDef ) ;
	// ���������uw����
	void GetSurfStep1( C3DSurfAttArray& c3DSurfAttArr, 
				       double dOverStep, 
				       int nCutUW  ) ;

	// ������������·��
	int  GenSurfPathLib1( CUWLineParam& cUWLine,
						  RFRAME& cRFrame ,
						  JDNC_SETUP& Setup,
						  JDNC_FUWLINE& UWLineCut,			
						  JDNC_PRGDEF &PrgDef ) ;

	// ���ɲ������·��
	BOOL GenUParamPath( C3DSurfAttribute* pSurfAtt,
					    CSmartLoop* pLoop,
						CSmtCPathList& pathList,
						PNT4D* dStart, PNT4D* dEnd, BOOL bZigZag ) ;

	void GenUParamPath( C3DSurfAttribute* pSurfAtt,
						CSmartLoop* pLoop,
						CSmtCPathList& pathList,
						PNT4D* dStart , BOOL bZigZag ) ;
	// bHead:�ǲ������ǰ����ߺ���, bReverse:�Ƿ�Ӧ�ý��ڶ����߷���
	void GenRestUPath( C3DSurfAttribute* pSurfAtt,
					   CSmartLoop* pLoop,
					   CSmtCPathList& pathList,
					   BOOL bHead, BOOL bReverse, BOOL bZigZag ) ;

	void GetLinePoint( C3DSurfAttribute* pSurfAtt, 
					   PNT2D start, PNT2D end, 
					   int n, BOOL bHead) ;

	void GetParamPTList( C3DSurfAttribute* pSurfAtt1,
						 C3DSurfAttribute* pSurfAtt2,
						 PNT4D* dArray, BOOL bHead ) ;

	// ����W�����·��
	void GenWParamPath ( C3DSurfAttribute* pSurfAtt,	// <I>:��������
						 CSmtCPathList& pathList,		// <O>:���·��
						 double dOverStep	) ;			// <I>:������
	// �õ����ߵ�i����(�ȷ�)
	void GetPointFromCurve( CGeoTrmSurf* pSurf, 
							CGeoCurve* pCur, 
							CGeoCurve* pPara,
							PNT4D* dArray, int num ) ;

	// ·������
	void ConnectAllUPath1( CUWLineParam& cUWLine, 
						  BOOL bZigZag, 
						  CSmtCPathLib& cSmtCPathLib ) ;

	void ConnectAllWPath1( CUWLineParam& cUWLine, 
						  BOOL bZigZag, 
						  CSmtCPathLib& cSmtCPathLib ) ;


public:

	// �õ�����һ���ߵĳ���,0ΪStart,1Ϊend ;
	double GetSurfBndLength( C3DSurfAttribute* pSurfAtt, 
							 int nFlag ) ;
	
	void ConnectPathByLayer02(	C3DSurfAttArray& c3DSurfAttArr, 
								CSmtCPathLib& cSmtCPathLib,
								int nLayer ) ;
	
	// ����w����������������֮����ͬ��ŵ�·��
	void ConnectNearSurfPath( C3DSurfAttribute* pSurf1,
							  C3DSurfAttribute* pSurf2 ) ;
	void ConnectNearSurfPath( C3DSurfAttribute* pSurf1,
							  C3DSurfAttribute* pSurf2,
							  PNT4D* dArray ) ;
	void TrimCloseSurfPath( C3DSurfAttribute* pSurf,
							  PNT4D* dArray ) ;
	// ����·��
	void ConnectNearPath( CSmtCutPath *pPrev, CSmtCutPath *pCurr, FPNT4D pnt[3] ) ;
	// ��������Բ��
	BOOL CreateArcCenter( PNT3D p[3], PNT3D cen ) ;
	BOOL GetPathDirect( CSmtCutPath* pPrev, CSmtCutPath* pCurr, VEC3D v[2] ) ;
	// �õ�·������ĩ����
	BOOL GetSmtPathDir( CSmtCutPath *pPath, VEC3D v, BOOL bFlag ) ;
	// ֱ������
	void ConnectPathDirect( CSmtCutPath* pPrev, CSmtCutPath* pCurr ) ;
	// ��������
	void ConnectPathExtend( CSmtCutPath* pPrev, CSmtCutPath* pCurr, PNT3D p ) ;
	// �ü�����
	void ConnectPathTrim ( CSmtCutPath* pPrev, CSmtCutPath* pCurr, PNT3D p[3], PNT3D intpt ) ;
	// �ü�����·��
	void TrimSmtCutPath( CSmtCutPath* pPath, double dLength, BOOL bFlag ) ;

// �����ּ��㷽����Ҫ�ĺ���(�������ڣ����Ƕ����ı���)
public:

	// �ڶ��ַ���
	BOOL CreateUWLineByWDirection( CSmtCPathLib& cSmtCPathLib,
							       CUWLineParam& cUWLine,
								   JDNC_FUWLINE& UWLineCut,
							       RFRAME& cRFrame, 
							       JDNC_SETUP& Setup, 
								   int &nStartLayerNo,			// ��ʼ��·�����
								   JDNC_PRGDEF &PrgDef ) ;
	// ������������·��,�����W����
	int  GenSurfPathLib3( CUWLineParam& cUWLine,
						  RFRAME& cRFrame ,
						  JDNC_SETUP& Setup,
						  JDNC_FUWLINE& UWLineCut,
						  JDNC_PRGDEF &PrgDef	) ;

	// ����W�����·��
	void GenWParamPath3 ( C3DSurfAttribute* pSurfAtt,
						  CSmtCPathList& pathList,
						  CSmartLoop* pLoop,
						  RFRAME& cRFrame, 
						  JDNC_UWLINE& cParam,
						  double dOverStep, INT_PTR n ) ;

	int  GetWParamOverStep3( C3DSurfAttribute* pSurfAtt, RFRAME& cRFrame, CSmartLoop* pLoop,
							 JDNC_UWLINE& cParam, double dOverStep, int nDir ) ;
	// ����W��·��
	void ConnectAllWPath3( CUWLineParam& cUWLine, 
						  BOOL bZigZag, 
						  CSmtCPathLib& cSmtCPathLib ) ;
	// ����·��˳��
	void AdjustWLineOrder( C3DSurfAttribute* pSurfAtt,
						   CSmtCPathList& pathLib ) ;

	void KeepSameLineDir ( CSmtCPathList& pathLib, 
						   PNT3D pt, INT_PTR n ) ;
//	void AdjustWPathOrder( C3DSurfAttribute* pSurfAtt,
//						   , int i ) ;
///////////////////////////��·�����и���//////////////////////////////////////////////
public:
	// ������и�����
	BOOL CheckAllPathBySelf( CSmtCheckMdl& DriveMd, CSmtCPathLib& AllPath, JDNC_TOL& Tol, 
						     JDNC_PRGDEF& PrgDef, double dCur, int *pErrorType) ;
	// ���������������
	BOOL CheckAllPathBySelfSubProc( CSmtCheckMdl& DriveMdl, CSmtCPathLib& AllPath, JDNC_TOL& Tol, JDNC_PRGDEF& PrgDef,
		 double dCur, int nAtCore, int nCoreNum, LPVOID* NewPath) ;
	void CheckPathBySelf( CSmtCheckMdl& DriveMdl, CSmtCutPath* pPath, JDNC_TOL& Tol ) ;
	void VerifyCutPathEx( CSmtCheckMdl *DriveMdl, CSmtCutPath *pPath, BOOL bCheck, double dArcTol ) ;
	void InsertPathPntEx( CSmtCheckMdl *DriveMdl, CSmtCutPath *pPath, BOOL bCheck, 
						  CSmtCutPointEx *pStart, CSmtCutPointEx *pEnd, double dArcTol ) ;
	// ����и�����,Ҫ���и�����
	BOOL CheckAllPathBySurf( CSmtCheckMdl& CheckMdl, CSmtCPathLib& AllPath, JDNC_TOL& Tol, 
							 BOOL bCheck, JDNC_PRGDEF& PrgDef, double dCur, int *pErrorType) ;
	// ��������������
	BOOL CheckAllPathBySurfSubProc( CSmtCheckMdl& DriveMdl, CSmtCPathLib& AllPath, 
		JDNC_TOL& Tol, JDNC_PRGDEF& PrgDef, double dCur, BOOL bCheck, int nAtCore, int nCoreNum, LPVOID* NewPath) ;
	CSmtCutPath* TrimPathBySurf( CSmtCheckMdl& CheckMdl, CSmtCutPath* pPath, BOOL bCheck, JDNC_TOL& Tol ) ; 
	// �ж�����ĵ��Ƿ�Ϊ�ڲ��㣨���Ǹ���㣩
	BOOL JudgeIverPt(CSmtCutPointEx *pPoint);
	// �ö��ַ�����·��������ٽ�㣬�����뵽·����
	CSmtCutPointEx *InsertBreakPnt(CSmtCheckMdl& CheckMdl, BOOL bCheck, CSmtCutPath* pCutPath, 
		CSmtCutPointEx *pStartPt, CSmtCutPointEx *pEndPt);
	BOOL GetLineIntPt( CSmtCutPointEx *Start, CSmtCutPointEx * Next, PNT3D intpt[2] ) ;
	double GetLineParam2D ( PNT2D start, PNT2D end, PNT2D mid ) ;
	double GetLineParam3D( PNT3D start, PNT3D end, PNT3D mid ) ;
	double GetCutPointDist( CSmtCutPoint* Start, CSmtCutPoint* End ) ;
	CSmtCutPath* BreakAndDelPnt( CSmtCutPath* pPath) ;
	// ��·�����и߶Ȳü�	
	void	InsertCPointEx( CSmtCutPath *pPath, double dStep ) ;

	//////////////////////////////////////////////////////////////////////////
	/// ���º���Ϊ�޸ĺ��·�������麯�� qqs 2013.11.06
	//////////////////////////////////////////////////////////////////////////
	// ������������
	BOOL CheckAllPathByCheckMdlSubProc( CSmtCheckMdl& DriveMdl, CSmtCPathLib& AllPath,
		                                JDNC_TOL& Tol, JDNC_PRGDEF& PrgDef, double dCur, 
										BOOL bCheck, int nAtCore, int nCoreNum, LPVOID* NewPath) ;
	BOOL CheckAllPathByCheckMdl( CSmtCheckMdl& CheckMdl, // ���ģ��
								 CSmtCPathLib& AllPath,  // �����·��
								 JDNC_TOL& Tol,          // ��龫��
								 BOOL bCheck,            // TRUEʱ��ʾ��ѡ�������顱
								 BOOL bInCheckBySelf,    // TRUEʱ��ʾ�ӹ�������飬FALSEʱ��ʾ�����������
								 JDNC_PRGDEF& PrgDef,    // ����������
								 double dCur,            // ����������ֵ
								 int *pErrorType) ;      // ��������

	// �ж�·���Ƿ�Ϊ��Ч��
 	BOOL IsAllInValidPnt( CSmtCutPath* pPath ) ;	
	// ���·����ɾ����Ч���֮ǰ�Ĳ����
	CSmtCutPath* BreakAndDelPntNew( CSmtCutPath* pPath) ;
	// �ö��ַ�����·��������ٽ�㣬���뵽·����,����·���ڵ���б�ǣ�����Ҫɾ���ĵ���ΪBreak
	void InsertAndLabelCPoint( CSmtCutPath *pPath, double dStep, CSmtCheckMdl *DriveMdl, BOOL bCheck) ;
	// �����ж�·�����Ƿ�Ϊ���е�
	BOOL IsPntOverCut(CSmtCutPointEx pCutPnt, CSmtCheckMdl *DriveMdl, BOOL bCheck, double tol = 0.002);
	// ���ڼ�����е�·�����в��ֵ��ٽ��
	int  CalInsertPnt(CSmtCutPointEx* pPathHead, CSmtCutPointEx* pPathNext, CSmtCutPointEx*& pInsert,CSmtCheckMdl* DriveMdl, BOOL bCheck);
//////////////////////////////////////////////////////////////////////////
// ����������������·��
	BOOL Create5AxUWLinePath( CSmtCheckMdl *DriveMdl, CSmtCPathLib& AllPath	, 
							  C3DSurfArray& AllSurf	, RFRAME& cRFrame		, 
						      JDNC_FUWLINE& cParam	, JDNC_5DCTRL &c5DCtrl	,
							  C5XGraph &allGraph, JDNC_PRGDEF &PrgDef,  
							  CSmartLoop *AllLoop, double dCur, int *pErrorType) ;

	// ���ɵ������������������·��
	BOOL CreateOne5AxSurfPath( CSmtCheckMdl *DriveMdl, CNc5DAxisCtrl &axisCtrl, RFRAME& cRFrame, 
							   CGeoTrmSurf* pSurf, JDNC_UWLINE& cParam, CSmartLoop *AllLoop, 
							   CSmtCPathLib& PathLib, JDNC_PRGDEF &PrgDef ) ;
	// ���ɵ������������������·����������
	BOOL CreateOne5AxSurfPathSubProc(int nUW, double uwStep[2], CSmtCheckMdl *DriveMdl, BOOL bZigZag,
									 RFRAME& cRFrame, CGeoTrmSurf* pSurf, CNc5DAxisCtrl &axisCtrl, 
									 CSmartLoop *AllLoop, CSmtCPathLib& PathLib, JDNC_PRGDEF &PrgDef, 
									 int nAtCore, int nCoreNum, LPVOID* NewPath);
	// ����·�����
	BOOL Calc5AxCutStep( CSmtCheckMdl *DriveMdl, CNc5DAxisCtrl* pAxisCtrl,  
						 CGeoTrmSurf* pSurf, CSmartLoop *AllLoop, RFRAME &lf, double dStep, double uwStep[2], 
						 int nCutDir, double*& dParam, int& nNum, JDNC_PRGDEF &ProgDef ) ;
	// �Եõ���·����������
	void LinkAll5AxisPath( CSmtCheckMdl *DriveMdl, CSmtCPathLib &AllPath, CNc5DAxisCtrl &axisCtrl, BOOL bZigZag ) ;

	// ����uw��������
	void CalcSurfUWCut( CGeoTrmSurf* pSurf, CSmartLoop* pLoop, JDNC_UWLINE &cParam, double uwStep[2], 
						int &nUW, BOOL &bZigZag, BOOL &bSpiral ) ;

	// �õ�����·��
	CSmtCutPath * CreateSGuideTPath( CSmtCheckMdl &DriveMdl,/*�ӹ�ģ��*/
									 CNc5DAxisCtrl* pAxisCtrl, 
									 CSurface *GeoSurf,		/*��������*/
									 CSmartLoop *AllLoop,	/*������*/
									 RFRAME &lf		,		/*�ӹ��桡*/
									 DOUBLE UWLine[2][2],	/*������  */
									 DOUBLE UWStep[2],		/*��������*/
									 int MoveDir,			/*��������*/
									 JDNC_TOL &Tol,			/*�ӹ�����*/
									 BOOL bParam,			/*��ǲ�����*/
									 JDNC_PRGDEF &ProgDef  ) ;

	// �õ�����·����ͬʱ���Դ�����ĥ�������任�Ĳ�����·��
	CSmtCutPath * CreateSGuideTPathNew( CSmtCutPath* temp, CSmtCheckMdl &DriveMdl,/*�ӹ�ģ��*/
	                                    CNc5DAxisCtrl* pAxisCtrl, 
	                                    CSurface *GeoSurf,		/*��������*/
	                                    CSmartLoop *AllLoop,	/*������*/
	                                    RFRAME &lf		,		/*�ӹ��桡*/
	                                    DOUBLE UWStep[2],		/*��������*/
	                                    int MoveDir,			/*��������*/
	                                    JDNC_TOL &Tol,			/*�ӹ�����*/
	                                    BOOL bParam,			/*��ǲ�����*/
	                                    JDNC_PRGDEF &ProgDef  ) ;
	// ����·����
	BOOL Gen5AxisPoint( CSurface *GeoSurf, CNc5DAxisCtrl* pAxisCtrl, 
						double wuAt[2], RFRAME *NcMtx, double AngRot[2], 
						CSmtCutPointEx &CutPoint, BOOL bParam , BOOL bToolPos = FALSE ) ;
	// �ж��Ƿ����
	BOOL Def5AxisPoint( CSmtCheckMdl &DriveMdl, CSmtCutPointEx &CutPoint, JDNC_TOL &Tol, CSmartLoop *AllLoop ) ;
	// �������·����֮�������·��
	BOOL ConnectPathLib(CSmtCPathLib &PathLib1, CSmtCPathLib &PathLib2, CSmtCheckMdl &DriveMdl, CNc5DAxisCtrl &AxisCtrl);

////////////////////////////////////���õ��������·��///////////////////////////////////
public:

	BOOL CreateOne5AxToolPosPath(	CSmtCheckMdl *DriveMdl, CNc5DAxisCtrl &axisCtrl, RFRAME& cRFrame, 
									CGeoTrmSurf* pSurf, JDNC_UWLINE& cParam, CSmartLoop *AllLoop, 
									CSmtCPathLib& PathLib, JDNC_PRGDEF &PrgDef) ;
	// ��õ�ֱ����ĩ��
	void CalcOriginalPath(  CGeoTrmSurf* pSurf, JDNC_UWLINE& cParam, RFRAME& cRFrame, CSmtCPathLib &AllPath ) ;

	// ����ƽ��������Ľ���·��
	BOOL CalcPlaneSurfIntPath( CSmtCheckMdl *DriveMdl, CNc5DAxisCtrl &axisCtrl, CGeoTrmSurf* pSurf,  
								RFRAME& cRFrame, JDNC_UWLINE& cParam, CSmartLoop *AllLoop, 
								CSmtCPathLib &AllPath , JDNC_PRGDEF &PrgDef ) ;
	// ���㽻�߲������Ч����
	CSSICurve * CalcPlaneSurfInt( CGeoTrmSurf* pSurf, PNT3D pivot, VEC3D normal ) ;
	CSSICurve * CalcSurfSurfInt( CGeoTrmSurf* pSurf, CSurface *pSurface ) ;
	// ������ת��·��
	BOOL Calc5axPathFromIntCurve( CSmtCheckMdl *DriveMdl, CNc5DAxisCtrl &axisCtrl,  CGeoTrmSurf* pSurf, 
									RFRAME& cRFrame,  CSSICurve *pIntCurve, JDNC_UWLINE& cParam, 
									CSmartLoop *AllLoop, CSmtCPathLib &AllPath, JDNC_PRGDEF &PrgDef ) ;
	// �Եõ���·����������
	void ConnectAllPathLib( CSmtCPathLib &AllPath ) ;

	// ���·������
	void GetPathPntAndNor( CSmtCutPath *pPath, PNT3D pnt[2], VEC3D vec[2] ) ;
	// ����ֱ����
	CRulSur * CalcRulSurfByLine( PNT3D start, PNT3D end, BOX3D &box ) ;
	// �Եõ���·����������
	void SortAllPath( CSmtCPathLib &AllPath ) ;
	// �����������ߵ��غ�λ��,�������ڼ�ֵ�㸽�������
	void AdjustIntCurve( CSSICurve *pIntCurve, PNT3D pt[4], PNT3D uv[4] ) ; 

	// �õ����е�Ͳ����е�֮��ĵ㣬��ú���Ľ��
	void AdjustFrontAngle( CSmtCheckMdl &DriveMdl, CSmtCutPointEx &pPnt, CNc5DAxisCtrl &axisCtrl ) ;
};



#endif // __SMART_FLOWLINE_GEN_H__

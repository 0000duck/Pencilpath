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

//记录面的拓扑关系和加工属性
class C3DSurfAttribute
{
public:
	int				m_nSameNormal;		//1法矢同向,-1法矢反向
	int				m_nSense;			//1表示参数取正向,-1表示反向
	int				m_nCutDir;			//0为u向,1为v向
	CGeoTrmSurf*	m_pSurf;			//对应曲面的指针
	CGeoCurve		*m_pPCurvStart, *m_pPCurvEnd;//参数域的指针
	CTrmFin			*m_pBoundStart, *m_pBoundEnd;//参数域的Bound指针
	double			m_dUStep, m_dWStep;
	double			m_dUOverStep, m_dWOverStep;
	BOOL			m_bTakeOutFromHead;
	BOOL			m_bPathReverse;
	BOOL			m_bSpiral	  ;		//标记是否为螺旋
	CSmtCPathLib	m_cSmtCPathLib;
	// add by yul	
	BOOL            m_bReverseSurf ;     // 曲面是否反向
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
	PNT3D pnt ; // 点的坐标
	int   num ; // 点的序号
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

//记录面边关系，以便根据共边搜索面的连接关系
class CAdjEdge : public CTrmEdge
{
public:
	CAdjEdge(CGeoCurve* pCurve, CGeoTrmSurf* pSurf1, CGeoTrmSurf* pSurf2, CTrmFin* pBnd1, CTrmFin* pBnd2, int nSameDir)
	{
		m_pCurve=pCurve;
		m_pRefer[0]=pSurf1, m_pRefer[1]=pSurf2;
		m_pFins[0]=pBnd1,  m_pFins[1]=pBnd2;
		pBnd1->m_nSense=pBnd2->m_nSense=nSameDir;	//或许不应当使用这两个变量保存两曲线是否同向
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
//用于记录流线加工的类
class CUWLineParam
{
public:
	int					m_nCutCross  ;	//0为u向(横切), 1为v向(纵切)
	int					m_nOutSide   ;
	C3DSurfAttArray		m_cSurfAttArr;
	CAdjEdgeArray		m_cAdjEdgeArr;
	C3DSurfArray		m_Surfs		 ;	// 邻面集合，用于临时计算

	int				    m_nStartAt   ;  // 路径的起刀点的位置
	BOOL                m_IsClosed   ;  
	BOOL				m_IsAllQuad  ;
	BOOL                m_bSmooth    ;  // 面是否光滑

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
// 线程数据定义
// 计算曲面流线5轴原始加工路径的线程数据
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

// 计算曲面流线3轴原始加工路径的线程数据
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

// 自身干涉检查的线程数据
typedef struct CheckAllPathThreadData  
{
	int nAtCore;
	int nCoreNum;
	LPVOID* NewPath;
	double dCur;
	BOOL bCheck;			// 临时增加的参数
	CSmtCheckMdl *pDriveMdl;
	JDNC_TOL *pTol;
	JDNC_PRGDEF PrgDef;
	CSmtCPathLib *AllPath;
	CFlowlineGen *pFlowlineGen;
}CAPT_DATA;

//////////////////////////////////////////////////////////////////////////
// 曲面流线走刀路径的线程函数
// 计算曲面流线5轴原始加工路径
static UINT MathCAM_CreateOne5AxSurfPathSubProc(LPVOID lpParam);
// 计算曲面流线3轴原始加工路径
static UINT MathCAM_CreateOneSurfPathSubProc(LPVOID lpParam);
// 自身干涉检查
static UINT MathCAM_CheckAllPathBySelfSubProc(LPVOID lpParam);
// 干涉面干涉检查
static UINT MathCAM_CheckAllPathBySurfSubProc(LPVOID lpParam);
// 干涉检查 qqs new 2013.11.07
static UINT MathCAM_CheckAllPathByCheckMdlSubProc(LPVOID lpParam);
//////////////////////////////////////////////////////////////////////////

typedef CTypedPtrArray<CPtrArray, CUWLineParam*> CUWLineArr;
typedef CTypedPtrArray<CPtrArray, CSurfLoop*> CSurfLoopArr;

class CIsoSGuideTPath ;
class CNc5DAxisCtrl   ;

#define CUTPNT_INSERT_TMP 100 //用于标记插入的节点 qqs 2013.11.06
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
	// 调整路径方向
	void ModifyPathDirection(CSmtCPathList &PathList);
protected://关于流线走刀的函数
	void ChangePCurve(C3DSurfAttribute* pSurfAtt, bool bRestore); //找邻边
	void ChangePCurve(C3DSurfAttArray& c3DSurfAttArr, bool bRestore); //找邻边

	//通过找面的共边,对面进行排序,找出连接的拓扑关系
	int	 IsSameBound(CTrmFin* Bnd1, CTrmFin* Bnd2);
	void SearchAdjacentEdge(CGeoTrmSurf* pSurf1, CGeoTrmSurf* pSurf2, CAdjEdgeArray& cAdjEdgeArr);
	CTrmFin* GetSubtense(CTrmFin* pBndSrc);//找对边
	C3DSurfAttribute* CreateSurfAtt(CGeoTrmSurf* pSurf, CTrmFin* pBnd1, CTrmFin* pBnd2, int nCurDir);
	int OrderAllSurf(C3DSurfArray& c3DSurfArr, CAdjEdgeArray& cAdjEdgeArr, C3DSurfAttArray& c3DSurfAttArr);

	
	//根据dArcTol精度要求,用对分插点的方法将参数域上的线映射到曲面上
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
	//由于ZBQ的CreateLinearPath只能产生等距行切路径,而流线加工参数域上的路径间距不等,故编写了CreateLinear函数生成行切路径
	int  GenParamSerialByStep( CGeoPLine3d* pPLine, double dStep, double* dParam, int& nNum );
	void GenParamSerialByStep( CSmartTool *pMill, RFRAME &lf, CGeoTrmSurf* pSurf, double dStep, int nCutDir, 
							   double*& dParam, int& nNum, BOOL bOffDir, double dArcTol, CSmtCPathLib &AllPath );
	void CreateLinear( double* dParam, int nNum, int nCutDir, BOOL bZigZag, CSmartLoop* pLoop, CSmtCPathLib& cSmtCPathLib );
	// 根据流线方向，得到最长的那根路径（从u/v = 0，0.5，1中选择）
	CSmtCutPath *GetMaxLenPath(CSmartTool *pMill, RFRAME &lf, CGeoTrmSurf* pSurf, 
		CSmartLoop* pLoop, int nCutDir, BOOL bOffDir, double dArcTol);
	//查找最近点,根据pnt1(某边退为点时可能用pnt2)和nCutCross在cSmtCPathLib中路径找到最近点的提取方法,
	//并根据通过cSmtCPathLib后的末点为pnt1和pnt2赋值,以便搜索下一个面的.
	//nNearestPnt[0]的0x0001表示路径反向，0x0002表示从cSmtCPathLib的头部提取
	int FindNearestPnt( PNT3D pnt1, PNT3D pnt2, CSmtCPathLib& cSmtCPathLib, int nCutCross );
	//调用FindNearestPnt函数设置路径的提取方法
	void SetTakeOutOrder(C3DSurfAttArray& c3DSurfAttArr, int nCutCross);

	//调整曲面加工路径的起点
	void AdjustStartAt( int nCutDir, int nStartAt, CSmtCPathLib& cSmtCutPathLib );
	// 反向同行号的路径
	void ReverseSameLineNoPath(CSmtCPathList &PathList);
	// 调换路径的顺序
	void ReverseLinePathOrder(CSmtCPathList &PathList);
	// 设置路径组行号
	void SetLineNo(CSmtCPathList &PathList, int nLineNo);
	
	// 闭合曲面沿曲面进刀
	void AdjustSpiralPath( CSmtCPathLib &cSmCutPathLib ) ;
	// 把路径组的层号改为原值0
	void ModifyLayerNo(CSmtCPathLib& PathLib);
	//按照距离对路径排序
	int	 CreateOneSurfPath(CSmartTool* pMiller, RFRAME& cRFrame, CGeoTrmSurf* pSurf, JDNC_UWLINE& cParam,
		 			       int nCutDir, double dArcTol, CSmtCPathLib& cParamPathLib, JDNC_PRGDEF &PrgDef, BOOL &bSpiral);
	// 生成流线走刀路径的主函数
	BOOL CreateOneSurfPathSubProc(double dArcTol, BOOL bOffDir, CSmartTool* pMiller, RFRAME& cRFrame, 
		CGeoTrmSurf* pSurf, CSmtCPathLib& cParamPathLib, JDNC_PRGDEF &PrgDef, int nAtCore, int nCoreNum);
	// 如果是螺旋连刀，合并相近的路径点
	BOOL CombinePathPoint(CSmtCPathLib& PathLib);
	//逐个面生成流线走刀路径，nStartLayerNo为起始的路径层号，传出最后一面的层号加1
	int  CreateEachSurfPath(CSmartTool* pMiller, RFRAME& cRFrame, C3DSurfArray& AllSurf, JDNC_UWLINE& cParam, 
						    double dArcTol, CSmtCPathLib& cSmtCPathLib, int &nStartLayerNo, JDNC_PRGDEF &PrgDef, BOOL *arrbSpiral );
	// 设置路径的层号和行号，并把路径导入新路径组中
	void ModifyPathAndCpy(CSmtCPathLib& OldPathLib, int &nStartLayerNo, CSmtCPathLib& NewPathLib, BOOL bModifyLineNo = TRUE);
	void OrderAllPath(C3DSurfAttArray& c3DSurfAttArr, int nCutCross, 
			BOOL bZigZag, CSmtCPathLib& cSmtCPathLib ); //为多个面流线加工的路径排序
	// 多个面按照远近距离排序
	void SortAllSurfByDist( C3DSurfArray &AllSurf, double dTol ) ;	
	// 得到偏移方向
	BOOL GetOffSetDir(CGeoTrmSurf* pSurf, RFRAME& cRFrame, JDNC_UWLINE& cParam);

    // 以下函数及变量用于计算曲面流线磨削调整路径 qqs 2013.05.20
	//////////////////////////////////////////////////////////////////////////
public:
	BOOL            m_bGrandMdfy;   // 记录是否开启了磨削调整加工

	void CreateLinearNew( double* dParam, int nNum, int nCutDir, CSmartLoop* pLoop, CSmtCPathLib& cSmtCPathLib , RFRAME& cRFrame, 
		                  double dArcTol, CGeoTrmSurf* pSurf, JDNC_UWLINE& cParam, BOOL b5Axis = FALSE ); // 将参数域路径进行磨削调整变换
	void GrindParamPathGen( CSmtCutPath* pNew, JDNC_UWLINE& cParam, RFRAME& cRFrame,  CGeoTrmSurf* pSurf, 
		                    int nCurDir, double dArcTol, PNT4D dHead, PNT4D dTail, BOOL b5Axis = FALSE );	
	BOOL ModifyGrindParamPath( CSmtCutPath* pParamPath, CSmtCutPath* pCutPath, int nType, DOUBLE depthInc, int  SinSegs, int nSect, int nCutDir );
	double GetMdfyHeit(CSmartTool *pMill, JDNC_UWLINE cParam, RFRAME &lf, CGeoTrmSurf* pSurf, int nCutDir, BOOL bOffDir, double dArcTol);
	void InsertNewCutPoint(CSmtCutPath * CutPath ,int  nPoint);
	void CreateSinWave(CSmtCutPath * CutPath,DOUBLE depthInc,int nSect, int uwDir);//类正弦波动
	void CreateTriWave(CSmtCutPath * CutPath,DOUBLE depthInc,int nSect, int uwDir);//三角形波动
	void CreateTrapWave(CSmtCutPath * CutPath,DOUBLE depthInc,int nSect, int uwDir);//梯形波动
	BOOL IsSet3DRCompMask(); // 判断是否开启了半径磨损补偿
	
	void AdjustSpiralCurve ( CSmartCurve &AllLine ); //用于计算构造面封闭螺旋连刀
	//////////////////////////////////////////////////////////////////////////

// 以下函数为Yul所使用
public:
	CSmtCPathLib	m_cSmtCPathLib;
	CSmartTool   *  m_pMiller     ;
	double			m_dStep       ; // 步长
	int             m_nNum        ; // 为U向走刀时的线段数
	double          m_dZMove      ; // 表面抬高
	JDNC_SETUP      m_cSetupDef   ;
	JDNC_FEED		m_cFeedDef	  ;
	JDNC_PRGDEF		m_cPrgDef     ; // 计算进度
	JDNC_5DCTRL		m_c5DCtrlDef  ; // 刀轴控制
	JDNC_STOCKEX	m_cStockDef	  ;
	JDNC_TOOLEX		m_cToolDef	  ;
	JDNC_SPEED		m_cSpeedDef   ;
	int *m_pErrorType;
	int				m_nCalcThreadNum;
	// 没有干涉模型
	BOOL CreateUWLinePath( CSmtCPathLib& AllPath, C3DSurfArray& AllSurf	,
		                   RFRAME& cRFrame, JDNC_FUWLINE& cParam,
						   JDNC_PRGDEF &PrgDef, double dCur, int *pErrorType, BOOL *arrbSpiral = NULL) ;
	// 生成边界
	void CreateStitchBnd( C3DSurfArray &AllSurf,
						  C3DSurfArray &tmpSurf,
                          CSurfLoopArr &cLoopArr ) ;
	// 拷贝曲面
	void CopyAllSurf ( C3DSurfArray &AllSurf, C3DSurfArray &tmpSurf ) ;
	// 删除曲面
	void DestroyAllSurf( C3DSurfArray &AllSurf, CUWLParaArray& ParaArr ) ;
	// 删除边界
	void DestroyBnd( C3DSurfArray &AllSurf, CSurfLoopArr &cLoopArr ) ;
	// 得到有效曲面
	void GetValidSurf( C3DSurfArray& AllSurf ) ;
	// 拷贝曲面的边界
	CTrmLoop* CopySurfLoop( CGeoTrmSurf* pSurf ) ;
	// 重新处理曲面，让边界都为逆时针，重构曲面边界，剔除网格面
	void ResetAllSurf( C3DSurfArray& AllSurf ) ;
	// 判断所有曲面是否存在光滑连接或位置连续曲面组合，存在是置到ParaArr中，把其他曲面置到otherSurf中
	BOOL AllSurfAdjacentConnect( C3DSurfArray& AllSurf,
								 C3DSurfArray& otherSurf,
								 CUWLParaArray& ParaArr, 
								 BOOL bSmooth) ;
	// 将面分为四边面和非四边面
	BOOL SortSurf( C3DSurfArray& AllSurf,
				   C3DSurfArray& QuadSurf ) ;
	BOOL IsQuadSurf( CTrmLoop* pLoop ) ;

	// 调整曲面的顺序，并判断曲面是否符合流线加工的条件
	BOOL  AdjustAllSurfType( C3DSurfArray& AllSurf,
							 CUWLParaArray& ParaArr ) ;
	// 根据加工参数设置流线参数
	void SetUWLParaData(CUWLParaArray &ParaArr, JDNC_FUWLINE& UWLineCut);	
	// 把曲面流线参数数组中的数据从一个移向另一个
	BOOL MoveUWLArrayData(CUWLParaArray &ParaFrm, CUWLParaArray &ParaTo);
	// 判断是否存在光滑的曲面，如果存在，把曲面的参数存入ParaArr中
	BOOL JudgeSmoothSurf(C3DSurfArray& AllSurf, CUWLParaArray& ParaArr);
	// 判断所有曲面的环是否都是四边形
	BOOL IsAllSurfQuad(C3DSurfArray& AllSurf);
	// 判断是否存在位置连续的曲面，如果存在，把曲面的参数存入ParaArr中
	BOOL JudgeConnectSurf(C3DSurfArray& AllSurf, CUWLParaArray& ParaArr);
	// 创建相邻曲面流线路径
	BOOL CreateAdjSurfUWPath(CSmtCPathLib &AllPath, RFRAME &cRFrame, JDNC_FUWLINE &UWLineCut, 
		CUWLineParam &cUWLine, int &nStartLayerNo, JDNC_PRGDEF & PrgDef);
	// 将旧数组中的数据移到新的中
	void AddAdjEdgeToArray( CAdjEdgeArray& OldEdgeArr, 
							CAdjEdgeArray& NewEdgeArr ) ;
	// 将旧数组中的数据移到新的中
	void AddAttSurfToArray( C3DSurfAttArray& OldSurfArr, 
							C3DSurfAttArray& NewSurfArr ) ;
	void DeleteAdjEdgeArr ( CAdjEdgeArray& cAdjEdgeArr ) ;
	void DeleteAdjSurfArr ( C3DSurfAttArray& cSurfArr ) ;
	// 搜索面的邻边
	void SearchAllSurfBound( C3DSurfArray& AllSurfArr, C3DSurfArray& LeftSurfArr, CUWLParaArray& ParaArr, BOOL bSmooth ) ;
	//切换tempSurfArr中下一个pSurf，如果没有，则将bFlag设为FALSE  qqs 2013.01.31
	void ResetFirstSurf(C3DSurfArray& tempSurfArr, CGeoTrmSurf** pSurf, BOOL& bFlag);
	// 搜索包含临面组合的数组，如果找到邻面中的一个面，则把两邻面添加到相应的位置，否则，在新的位置添加两邻面
	int SearchAndSetSurfArray(CUWLParaArray& ParaArr, CGeoSurf *pSurf1, CGeoSurf *pSurf2);
	// 搜索邻边
	BOOL SearchAdjacentBound( CGeoTrmSurf* pSurf1, 
							  CGeoTrmSurf* pSurf2,
							  CAdjEdgeArray& cAdjEdgeArr,
							  BOOL bSmooth ) ;

	// 判断曲面边界是否重合。边界pFin1,pFin2,dTol为精度,bSameDir为方向
	BOOL JudgeTwoBoundOverlap( CTrmFin* pFin1, CTrmFin* pFin2, 
							   double dTol, BOOL& bSameDir ) ;
	// 判断曲面的边界是否光滑连接
	BOOL JudgeTwoBoundSmooth ( CGeoTrmSurf* pSurf1, CTrmFin* pFin1,	
							   CGeoTrmSurf* pSurf2,	CTrmFin* pFin2,
							   double dTol, BOOL& bSameDir, BOOL bSmooth ) ;
	// 得到曲线中点处在曲面上的法矢
	void GetCurveMidNormal ( CGeoTrmSurf* pSurf, CTrmFin* pFin, VEC3D normal ) ;

	BOOL IsEligibleVec( VEC3D u1, VEC3D u2, double Tol ) ;
	// 重新生成曲面的边界线
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

	// 调整路径的起点
	void AdjustPathStart( CUWLineParam& cUWLine ) ;
	// 判断曲面组是否首尾相连
	BOOL IsSurfArrayColsed( C3DSurfAttArray& cSurfArr ) ;

	// 得到曲面的加工法矢
	void GetSurfNormal( C3DSurfAttArray& cSurfArr, 
					    RFRAME& cRFrame, 
						int nOffsetDir ) ;
	// 按照曲面法矢的方向，调整曲面边界线的方向和参数域的方向
	void AdjustAllSurfDir( C3DSurfAttArray& cSurfArr, 
						   RFRAME& cRFrame, 
						   int nOffsetDir ) ;
	// 判断曲面流线参数是否合理
	BOOL IsUWLineParamValid(const CUWLineParam &cUWLine);
	void DeleteSurfEdge ( CGeoTrmSurf* pSurf ) ;
	// 曲面的边界反向
	void ReverseSurfEdge( CGeoTrmSurf* pSurf ) ;
	// 0,表示第一个,1,表示中间,2表示最后一个
	void ReverseAttSurf( C3DSurfAttribute* pSurfAtt,
						 BOOL bReverse, int nType ) ;
	// 得到曲面共边上某点的法矢
	void GetAttSurfNormal( C3DSurfAttribute* pSurfAtt,
						   BOOL bStart,  VEC3D normal ) ;
	CGeoCurve* GetPrevCurve( C3DSurfAttribute* pSurfAtt,
							 CGeoCurve* pCurve ) ;
	CGeoCurve* GetNextCurve( C3DSurfAttribute* pSurfAtt,
							 CGeoCurve* pCurve ) ;
	// 得到曲面共边上的点的方向
	void GetAttSurfSameLinePntNor( C3DSurfAttribute* pSurfAtt,  BOOL bStart,  VEC3D nor ) ;

	// 调整路径的顺序
	void ReverseLineOrder( C3DSurfAttribute* pSurfAtt ) ;
	void ReverseLineOrder( CSmtCPathList& pathList ) ;

	// 参数域上的路径映射到空间域的函数
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
	// 得到前后的法矢
	void GetNextNormal ( CGeoTrmSurf* pSurf, CSmtCutPath* pPath,
						 PNT2D param, VEC3D dNormal, PNT3D dPoint ) ;
	// 得到左右的法矢
	void GetSideNormal ( CGeoTrmSurf* pSurf, CSmtCutPath* pPath,
						 PNT2D param, VEC3D dNormal, PNT3D dPoint ) ;

	CSmtCutPath* millface_CurveToCPath( CSmartCurve&  Curve ,JDNC_TOL& Tol ) ;

	int GetPoint( CGeoPLine3d *pLine, double tmp, double dLength, PNT3D pos ) ;
	int GetPoint( CGeoLine *pLine, double tmp, double dLength, PNT3D pos ) ;

	void AddPathPointEx( CSmtCutPath *pPath, PNT4D pnt ) ;
	void AddPathPointEx( CSmtCutPath *pPath, float pnt[4] ) ;
// 第一种计算方法（重新构造曲面的流线方向）
public:

	// 第一种方法
	BOOL CreateUWLineByReConstruct( CSmtCPathLib& cSmtCPathLib,
							        CUWLineParam& cUWLine,
									JDNC_FUWLINE& UWLineCut,
							        RFRAME& cRFrame, 
							        JDNC_SETUP& Setup, 
									int &nStartLayerNo,			// 起始的路径层号
									JDNC_PRGDEF &PrgDef ) ;
	// 计算曲面的uw方向
	void GetSurfStep1( C3DSurfAttArray& c3DSurfAttArr, 
				       double dOverStep, 
				       int nCutUW  ) ;

	// 计算曲面流线路径
	int  GenSurfPathLib1( CUWLineParam& cUWLine,
						  RFRAME& cRFrame ,
						  JDNC_SETUP& Setup,
						  JDNC_FUWLINE& UWLineCut,			
						  JDNC_PRGDEF &PrgDef ) ;

	// 生成参数域的路径
	BOOL GenUParamPath( C3DSurfAttribute* pSurfAtt,
					    CSmartLoop* pLoop,
						CSmtCPathList& pathList,
						PNT4D* dStart, PNT4D* dEnd, BOOL bZigZag ) ;

	void GenUParamPath( C3DSurfAttribute* pSurfAtt,
						CSmartLoop* pLoop,
						CSmtCPathList& pathList,
						PNT4D* dStart , BOOL bZigZag ) ;
	// bHead:是参数域的前面或者后面, bReverse:是否应该将第二条边反向
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

	// 生成W向参数路径
	void GenWParamPath ( C3DSurfAttribute* pSurfAtt,	// <I>:输入曲面
						 CSmtCPathList& pathList,		// <O>:输出路径
						 double dOverStep	) ;			// <I>:输入间距
	// 得到曲线第i个点(等分)
	void GetPointFromCurve( CGeoTrmSurf* pSurf, 
							CGeoCurve* pCur, 
							CGeoCurve* pPara,
							PNT4D* dArray, int num ) ;

	// 路径连刀
	void ConnectAllUPath1( CUWLineParam& cUWLine, 
						  BOOL bZigZag, 
						  CSmtCPathLib& cSmtCPathLib ) ;

	void ConnectAllWPath1( CUWLineParam& cUWLine, 
						  BOOL bZigZag, 
						  CSmtCPathLib& cSmtCPathLib ) ;


public:

	// 得到曲面一条边的长度,0为Start,1为end ;
	double GetSurfBndLength( C3DSurfAttribute* pSurfAtt, 
							 int nFlag ) ;
	
	void ConnectPathByLayer02(	C3DSurfAttArray& c3DSurfAttArr, 
								CSmtCPathLib& cSmtCPathLib,
								int nLayer ) ;
	
	// 用于w向连刀，连接两面之间相同层号的路径
	void ConnectNearSurfPath( C3DSurfAttribute* pSurf1,
							  C3DSurfAttribute* pSurf2 ) ;
	void ConnectNearSurfPath( C3DSurfAttribute* pSurf1,
							  C3DSurfAttribute* pSurf2,
							  PNT4D* dArray ) ;
	void TrimCloseSurfPath( C3DSurfAttribute* pSurf,
							  PNT4D* dArray ) ;
	// 连接路径
	void ConnectNearPath( CSmtCutPath *pPrev, CSmtCutPath *pCurr, FPNT4D pnt[3] ) ;
	// 三点生成圆心
	BOOL CreateArcCenter( PNT3D p[3], PNT3D cen ) ;
	BOOL GetPathDirect( CSmtCutPath* pPrev, CSmtCutPath* pCurr, VEC3D v[2] ) ;
	// 得到路径的首末方向
	BOOL GetSmtPathDir( CSmtCutPath *pPath, VEC3D v, BOOL bFlag ) ;
	// 直接连接
	void ConnectPathDirect( CSmtCutPath* pPrev, CSmtCutPath* pCurr ) ;
	// 延伸连接
	void ConnectPathExtend( CSmtCutPath* pPrev, CSmtCutPath* pCurr, PNT3D p ) ;
	// 裁减连接
	void ConnectPathTrim ( CSmtCutPath* pPrev, CSmtCutPath* pCurr, PNT3D p[3], PNT3D intpt ) ;
	// 裁减单个路径
	void TrimSmtCutPath( CSmtCutPath* pPath, double dLength, BOOL bFlag ) ;

// 第三种计算方法需要的函数(曲面相邻，并非都是四边形)
public:

	// 第二种方法
	BOOL CreateUWLineByWDirection( CSmtCPathLib& cSmtCPathLib,
							       CUWLineParam& cUWLine,
								   JDNC_FUWLINE& UWLineCut,
							       RFRAME& cRFrame, 
							       JDNC_SETUP& Setup, 
								   int &nStartLayerNo,			// 起始的路径层号
								   JDNC_PRGDEF &PrgDef ) ;
	// 计算曲面流线路径,曲面的W方向
	int  GenSurfPathLib3( CUWLineParam& cUWLine,
						  RFRAME& cRFrame ,
						  JDNC_SETUP& Setup,
						  JDNC_FUWLINE& UWLineCut,
						  JDNC_PRGDEF &PrgDef	) ;

	// 生成W向参数路径
	void GenWParamPath3 ( C3DSurfAttribute* pSurfAtt,
						  CSmtCPathList& pathList,
						  CSmartLoop* pLoop,
						  RFRAME& cRFrame, 
						  JDNC_UWLINE& cParam,
						  double dOverStep, INT_PTR n ) ;

	int  GetWParamOverStep3( C3DSurfAttribute* pSurfAtt, RFRAME& cRFrame, CSmartLoop* pLoop,
							 JDNC_UWLINE& cParam, double dOverStep, int nDir ) ;
	// 连接W向路径
	void ConnectAllWPath3( CUWLineParam& cUWLine, 
						  BOOL bZigZag, 
						  CSmtCPathLib& cSmtCPathLib ) ;
	// 调整路经顺序
	void AdjustWLineOrder( C3DSurfAttribute* pSurfAtt,
						   CSmtCPathList& pathLib ) ;

	void KeepSameLineDir ( CSmtCPathList& pathLib, 
						   PNT3D pt, INT_PTR n ) ;
//	void AdjustWPathOrder( C3DSurfAttribute* pSurfAtt,
//						   , int i ) ;
///////////////////////////对路径进行干涉//////////////////////////////////////////////
public:
	// 自身进行干涉检查
	BOOL CheckAllPathBySelf( CSmtCheckMdl& DriveMd, CSmtCPathLib& AllPath, JDNC_TOL& Tol, 
						     JDNC_PRGDEF& PrgDef, double dCur, int *pErrorType) ;
	// 自身干涉检查主函数
	BOOL CheckAllPathBySelfSubProc( CSmtCheckMdl& DriveMdl, CSmtCPathLib& AllPath, JDNC_TOL& Tol, JDNC_PRGDEF& PrgDef,
		 double dCur, int nAtCore, int nCoreNum, LPVOID* NewPath) ;
	void CheckPathBySelf( CSmtCheckMdl& DriveMdl, CSmtCutPath* pPath, JDNC_TOL& Tol ) ;
	void VerifyCutPathEx( CSmtCheckMdl *DriveMdl, CSmtCutPath *pPath, BOOL bCheck, double dArcTol ) ;
	void InsertPathPntEx( CSmtCheckMdl *DriveMdl, CSmtCutPath *pPath, BOOL bCheck, 
						  CSmtCutPointEx *pStart, CSmtCutPointEx *pEnd, double dArcTol ) ;
	// 如果有干涉面,要进行干涉检查
	BOOL CheckAllPathBySurf( CSmtCheckMdl& CheckMdl, CSmtCPathLib& AllPath, JDNC_TOL& Tol, 
							 BOOL bCheck, JDNC_PRGDEF& PrgDef, double dCur, int *pErrorType) ;
	// 干涉面检查主函数
	BOOL CheckAllPathBySurfSubProc( CSmtCheckMdl& DriveMdl, CSmtCPathLib& AllPath, 
		JDNC_TOL& Tol, JDNC_PRGDEF& PrgDef, double dCur, BOOL bCheck, int nAtCore, int nCoreNum, LPVOID* NewPath) ;
	CSmtCutPath* TrimPathBySurf( CSmtCheckMdl& CheckMdl, CSmtCutPath* pPath, BOOL bCheck, JDNC_TOL& Tol ) ; 
	// 判断输入的点是否为内部点（即非干涉点）
	BOOL JudgeIverPt(CSmtCutPointEx *pPoint);
	// 用二分法查找路径干涉的临界点，并插入到路径中
	CSmtCutPointEx *InsertBreakPnt(CSmtCheckMdl& CheckMdl, BOOL bCheck, CSmtCutPath* pCutPath, 
		CSmtCutPointEx *pStartPt, CSmtCutPointEx *pEndPt);
	BOOL GetLineIntPt( CSmtCutPointEx *Start, CSmtCutPointEx * Next, PNT3D intpt[2] ) ;
	double GetLineParam2D ( PNT2D start, PNT2D end, PNT2D mid ) ;
	double GetLineParam3D( PNT3D start, PNT3D end, PNT3D mid ) ;
	double GetCutPointDist( CSmtCutPoint* Start, CSmtCutPoint* End ) ;
	CSmtCutPath* BreakAndDelPnt( CSmtCutPath* pPath) ;
	// 对路径进行高度裁减	
	void	InsertCPointEx( CSmtCutPath *pPath, double dStep ) ;

	//////////////////////////////////////////////////////////////////////////
	/// 以下函数为修改后的路径干涉检查函数 qqs 2013.11.06
	//////////////////////////////////////////////////////////////////////////
	// 干涉检查主函数
	BOOL CheckAllPathByCheckMdlSubProc( CSmtCheckMdl& DriveMdl, CSmtCPathLib& AllPath,
		                                JDNC_TOL& Tol, JDNC_PRGDEF& PrgDef, double dCur, 
										BOOL bCheck, int nAtCore, int nCoreNum, LPVOID* NewPath) ;
	BOOL CheckAllPathByCheckMdl( CSmtCheckMdl& CheckMdl, // 检查模型
								 CSmtCPathLib& AllPath,  // 待检查路径
								 JDNC_TOL& Tol,          // 检查精度
								 BOOL bCheck,            // TRUE时表示勾选“干涉检查”
								 BOOL bInCheckBySelf,    // TRUE时表示加工面干涉检查，FALSE时表示保护面干涉检查
								 JDNC_PRGDEF& PrgDef,    // 进度条设置
								 double dCur,            // 进度条进度值
								 int *pErrorType) ;      // 错误类型

	// 判断路径是否都为无效点
 	BOOL IsAllInValidPnt( CSmtCutPath* pPath ) ;	
	// 打断路径并删除无效点和之前的插入点
	CSmtCutPath* BreakAndDelPntNew( CSmtCutPath* pPath) ;
	// 用二分法查找路径干涉的临界点，插入到路径中,并对路径节点进行标记，将需要删除的点标记为Break
	void InsertAndLabelCPoint( CSmtCutPath *pPath, double dStep, CSmtCheckMdl *DriveMdl, BOOL bCheck) ;
	// 用于判断路径点是否为过切点
	BOOL IsPntOverCut(CSmtCutPointEx pCutPnt, CSmtCheckMdl *DriveMdl, BOOL bCheck, double tol = 0.002);
	// 用于计算过切的路径过切部分的临界点
	int  CalInsertPnt(CSmtCutPointEx* pPathHead, CSmtCutPointEx* pPathNext, CSmtCutPointEx*& pInsert,CSmtCheckMdl* DriveMdl, BOOL bCheck);
//////////////////////////////////////////////////////////////////////////
// 生成五轴曲面流线路径
	BOOL Create5AxUWLinePath( CSmtCheckMdl *DriveMdl, CSmtCPathLib& AllPath	, 
							  C3DSurfArray& AllSurf	, RFRAME& cRFrame		, 
						      JDNC_FUWLINE& cParam	, JDNC_5DCTRL &c5DCtrl	,
							  C5XGraph &allGraph, JDNC_PRGDEF &PrgDef,  
							  CSmartLoop *AllLoop, double dCur, int *pErrorType) ;

	// 生成单张面的五轴曲面流线路径
	BOOL CreateOne5AxSurfPath( CSmtCheckMdl *DriveMdl, CNc5DAxisCtrl &axisCtrl, RFRAME& cRFrame, 
							   CGeoTrmSurf* pSurf, JDNC_UWLINE& cParam, CSmartLoop *AllLoop, 
							   CSmtCPathLib& PathLib, JDNC_PRGDEF &PrgDef ) ;
	// 生成单张面的五轴曲面流线路径的主函数
	BOOL CreateOne5AxSurfPathSubProc(int nUW, double uwStep[2], CSmtCheckMdl *DriveMdl, BOOL bZigZag,
									 RFRAME& cRFrame, CGeoTrmSurf* pSurf, CNc5DAxisCtrl &axisCtrl, 
									 CSmartLoop *AllLoop, CSmtCPathLib& PathLib, JDNC_PRGDEF &PrgDef, 
									 int nAtCore, int nCoreNum, LPVOID* NewPath);
	// 计算路径间距
	BOOL Calc5AxCutStep( CSmtCheckMdl *DriveMdl, CNc5DAxisCtrl* pAxisCtrl,  
						 CGeoTrmSurf* pSurf, CSmartLoop *AllLoop, RFRAME &lf, double dStep, double uwStep[2], 
						 int nCutDir, double*& dParam, int& nNum, JDNC_PRGDEF &ProgDef ) ;
	// 对得到的路径进行连接
	void LinkAll5AxisPath( CSmtCheckMdl *DriveMdl, CSmtCPathLib &AllPath, CNc5DAxisCtrl &axisCtrl, BOOL bZigZag ) ;

	// 计算uw参数方向
	void CalcSurfUWCut( CGeoTrmSurf* pSurf, CSmartLoop* pLoop, JDNC_UWLINE &cParam, double uwStep[2], 
						int &nUW, BOOL &bZigZag, BOOL &bSpiral ) ;

	// 得到五轴路径
	CSmtCutPath * CreateSGuideTPath( CSmtCheckMdl &DriveMdl,/*加工模型*/
									 CNc5DAxisCtrl* pAxisCtrl, 
									 CSurface *GeoSurf,		/*几何曲面*/
									 CSmartLoop *AllLoop,	/*轮廓线*/
									 RFRAME &lf		,		/*加工面　*/
									 DOUBLE UWLine[2][2],	/*参数线  */
									 DOUBLE UWStep[2],		/*参数步长*/
									 int MoveDir,			/*参数方向*/
									 JDNC_TOL &Tol,			/*加工精度*/
									 BOOL bParam,			/*标记参数域*/
									 JDNC_PRGDEF &ProgDef  ) ;

	// 得到五轴路径，同时可以处理经过磨削调整变换的参数与路径
	CSmtCutPath * CreateSGuideTPathNew( CSmtCutPath* temp, CSmtCheckMdl &DriveMdl,/*加工模型*/
	                                    CNc5DAxisCtrl* pAxisCtrl, 
	                                    CSurface *GeoSurf,		/*几何曲面*/
	                                    CSmartLoop *AllLoop,	/*轮廓线*/
	                                    RFRAME &lf		,		/*加工面　*/
	                                    DOUBLE UWStep[2],		/*参数步长*/
	                                    int MoveDir,			/*参数方向*/
	                                    JDNC_TOL &Tol,			/*加工精度*/
	                                    BOOL bParam,			/*标记参数域*/
	                                    JDNC_PRGDEF &ProgDef  ) ;
	// 计算路径点
	BOOL Gen5AxisPoint( CSurface *GeoSurf, CNc5DAxisCtrl* pAxisCtrl, 
						double wuAt[2], RFRAME *NcMtx, double AngRot[2], 
						CSmtCutPointEx &CutPoint, BOOL bParam , BOOL bToolPos = FALSE ) ;
	// 判断是否过切
	BOOL Def5AxisPoint( CSmtCheckMdl &DriveMdl, CSmtCutPointEx &CutPoint, JDNC_TOL &Tol, CSmartLoop *AllLoop ) ;
	// 添加两个路径组之间的连刀路径
	BOOL ConnectPathLib(CSmtCPathLib &PathLib1, CSmtCPathLib &PathLib2, CSmtCheckMdl &DriveMdl, CNc5DAxisCtrl &AxisCtrl);

////////////////////////////////////利用刀触点计算路径///////////////////////////////////
public:

	BOOL CreateOne5AxToolPosPath(	CSmtCheckMdl *DriveMdl, CNc5DAxisCtrl &axisCtrl, RFRAME& cRFrame, 
									CGeoTrmSurf* pSurf, JDNC_UWLINE& cParam, CSmartLoop *AllLoop, 
									CSmtCPathLib& PathLib, JDNC_PRGDEF &PrgDef) ;
	// 获得的直线起末点
	void CalcOriginalPath(  CGeoTrmSurf* pSurf, JDNC_UWLINE& cParam, RFRAME& cRFrame, CSmtCPathLib &AllPath ) ;

	// 计算平面与曲面的交线路径
	BOOL CalcPlaneSurfIntPath( CSmtCheckMdl *DriveMdl, CNc5DAxisCtrl &axisCtrl, CGeoTrmSurf* pSurf,  
								RFRAME& cRFrame, JDNC_UWLINE& cParam, CSmartLoop *AllLoop, 
								CSmtCPathLib &AllPath , JDNC_PRGDEF &PrgDef ) ;
	// 计算交线并获得有效曲线
	CSSICurve * CalcPlaneSurfInt( CGeoTrmSurf* pSurf, PNT3D pivot, VEC3D normal ) ;
	CSSICurve * CalcSurfSurfInt( CGeoTrmSurf* pSurf, CSurface *pSurface ) ;
	// 将交线转成路径
	BOOL Calc5axPathFromIntCurve( CSmtCheckMdl *DriveMdl, CNc5DAxisCtrl &axisCtrl,  CGeoTrmSurf* pSurf, 
									RFRAME& cRFrame,  CSSICurve *pIntCurve, JDNC_UWLINE& cParam, 
									CSmartLoop *AllLoop, CSmtCPathLib &AllPath, JDNC_PRGDEF &PrgDef ) ;
	// 对得到的路径进行连接
	void ConnectAllPathLib( CSmtCPathLib &AllPath ) ;

	// 获得路径参数
	void GetPathPntAndNor( CSmtCutPath *pPath, PNT3D pnt[2], VEC3D vec[2] ) ;
	// 生成直纹面
	CRulSur * CalcRulSurfByLine( PNT3D start, PNT3D end, BOX3D &box ) ;
	// 对得到的路径进行排序
	void SortAllPath( CSmtCPathLib &AllPath ) ;
	// 调整多条交线的重合位置,处理交点在极值点附近的情况
	void AdjustIntCurve( CSSICurve *pIntCurve, PNT3D pt[4], PNT3D uv[4] ) ; 

	// 得到过切点和不过切点之间的点，求得合理的结果
	void AdjustFrontAngle( CSmtCheckMdl &DriveMdl, CSmtCutPointEx &pPnt, CNc5DAxisCtrl &axisCtrl ) ;
};



#endif // __SMART_FLOWLINE_GEN_H__

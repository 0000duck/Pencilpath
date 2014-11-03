#include "StdAfx.H"
#include "SmartNC.H"
#include "SmtPathGen.h"
#include "SmtPathGen2D.H"
#include "SmtPathGen3D.H"
#include "SmartPathEx.H"
#include "NcBndFilleting.h"
#include "NcHeightMdl.H"
#include "mathcam.h"
#include "GP_Math.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

//////////////////////////////////////////////////
CSmartSRemainGen::CSmartSRemainGen()
{
	m_pLoopArr1 = NULL ;
	m_pAllLoop = NULL ;
	m_pLastLoopArr = NULL ;
	m_pLastTool    = NULL ;
	m_pDriveLoop   = NULL ;
	m_pRoughLoop   = NULL ;
	m_pRemainLoop  = NULL ;
	m_bFlag		   = NULL ;
	m_dZ           = NULL ;
	m_pLPathArr    = NULL ;
	m_pRemainMdl  = NULL ; 
	m_pPlunge	   = NULL ;
	m_pCurveLib    = NULL ;
	m_bFirst	   = TRUE ;
	m_pRoughTool   = NULL ;
	m_dStepZ	   = 0.   ;
	m_dBotHeight   = 0.	  ;
	m_dTopHeight   = 0.   ;
	m_bAllLoop	   = FALSE ;
}
CSmartSRemainGen::~CSmartSRemainGen()
{
	if( m_pAllLoop ) Mini_DeleteContours( m_pAllLoop ) ;
	if( m_pLastTool ) delete m_pLastTool ;
	m_pLastTool = NULL ;;
	ClearAllLoop() ;
	ClearAllCurveLib() ;
	if( m_pRemainMdl ) delete m_pRemainMdl ;
	m_pRemainMdl = NULL ;
	if( m_pRoughTool ) delete m_pRoughTool ;
	m_pRoughTool = NULL ;
}
void CSmartSRemainGen::InitTmpProg ()
{
	m_cTmpPrgDef.m_dLimitAt = 0. ;
	m_cTmpPrgDef.m_dStepAt  = 0 ;
	m_cTmpPrgDef.m_dIncStep = 1. ;
	m_cTmpPrgDef.m_pBrkFunc = NULL ;
	m_cTmpPrgDef.m_pPrgFunc = NULL ;
	m_cTmpPrgDef.m_pPosFunc = NULL ;
}
void CSmartSRemainGen::ClearAllLoop ( )
{
	// 清除没有加工的环 
	ClearAllLoopArr( m_pLastLoopArr, m_nCount ) ;
	ClearAllLoopHead( m_pDriveLoop, m_nCount ) ;
	ClearAllLoopHead( m_pRemainLoop, m_nCount ) ;
	ClearAllLPathArr( m_pLPathArr, m_nCount ) ;
	ClearAllLoopHead( m_pRoughLoop, m_nCount ) ;
	ClearAllLoopArr( m_pLoopArr1, m_nCount ) ;
	ClearAllLoopHead( m_pPlunge, m_nCount ) ;
	ClearAllPlaneLoop( &m_cPlaneList ) ;
	if( m_dZ ) 	delete[] m_dZ ;
	if( m_bFlag ) delete[] m_bFlag ;
	m_pLastLoopArr = NULL ;
	m_pDriveLoop   = NULL ;
	m_pLPathArr    = NULL ;
	m_pRoughLoop   = NULL ;
	m_pLoopArr1    = NULL ;
	m_pPlunge      = NULL ;
	m_dZ		   = NULL ;
	m_bFlag		   = NULL ;
}
void CSmartSRemainGen::ClearAllCurveLib ()
{
	if( m_pCurveLib )
	{
		for( INT_PTR i = 0 ; i < m_nCount ; i++ )
		{
			m_pCurveLib[i].DeleteAllCurves () ;
		}
		delete[] m_pCurveLib ;
	}
	m_pCurveLib = NULL ;
}
BOOL CSmartSRemainGen::CalcTopAndBot ( BOX3D &box )
{
	m_dTopHeight = GetTopHeight() ;
	m_dBotHeight = GetBottomHeight() ;
//	double dZMove = GetDriveSurfZMove() ;
	double dTop = box.max[2] + 4.0e-4 ;
	double dBot = box.min[2] - 4.0e-4 ;
//	if( dZMove > 0 )	dTop += dZMove ;
//	else				dBot += dZMove ;

	if( m_cShapeDef.m_bShapeFlag & NCDEF_SHAPE_AUTODEPTH )
	{
		// 最大高度由毛坯高度限定
		m_dTopHeight = dTop ;
		m_dBotHeight = dBot ;
	}

	if( m_dTopHeight > dTop ) m_dTopHeight = dTop ;
	if( m_dBotHeight < dBot ) m_dBotHeight = dBot ;
	
	// STEP 2 : 分层
	GetECutLayers() ;
	return TRUE ;
}
int CSmartSRemainGen::GetECutLayers ()
{
	double top = m_dTopHeight ;
	double bot = m_dBotHeight ;
	// 避免加工到平面,表面余量+0.001
	double dCutDepth = top - bot ;
	double dStepZ = 0. ;
		
	// STEP 1 : 估算最大的分层dStepZ
	if (m_cFeedDef.m_cLayerDef.m_nLayerType == NCDEF_LAYER_DEF) // 自定义
	{
		JDNC_LAYER& Layer = m_cFeedDef.m_cLayerDef;
		AssignSelfDefineLayerDepth(Layer.m_bLayerFlag, top, bot, m_dZ, m_nCount, TRUE);
		m_dStepZ = dCutDepth / m_nCount ;
		goto EndLine ;
	}
	else if( m_cFeedDef.m_cLayerDef.m_nLayerType == 0 ) // 关闭
	{
		m_nCount = 1 ;
	}
	else if( m_cFeedDef.m_cLayerDef.m_nLayerType == 1 ) // 限定层数
	{
		m_nCount = 1 ;
		if( m_cFeedDef.m_cLayerDef.m_nLayerCount < 1 )
			m_nCount = 1 ;
		else
			m_nCount = m_cFeedDef.m_cLayerDef.m_nLayerCount ;
	}
	else
	{
		dStepZ = m_cFeedDef.m_cLayerDef.m_dSideDInc ;
		if( m_cFeedDef.m_cLayerDef.m_bLayerFlag & NCDEF_LAYER_KEEPZ )
		{
			m_nCount = int( ceil( dCutDepth / dStepZ ) + 2 ) ;
			m_dZ = new double[m_nCount] ;
			m_nCount = 0 ;
			while( top > bot )
			{
				top -= dStepZ ;
				m_dZ[m_nCount] = top ;
				m_nCount++ ;
			}
			if( m_dZ[m_nCount-1] - bot > m_cSetupDef.m_cTolDef.m_dArcTol )
			{
				m_dZ[m_nCount] = bot ;
				m_nCount++ ;
			}
			else
			{
				m_dZ[m_nCount - 1] = bot ;
			}
			m_dStepZ = dStepZ ;
			goto EndLine ;
		}
		else
		{
			m_nCount = ( int ) ceil( dCutDepth / dStepZ - 1.0e-4 ) ;
			if( m_nCount < 1 )	m_nCount = 1 ;
		}
	}
	m_dStepZ = dCutDepth / m_nCount ;
	// 均匀分层,也可以按照cimatron从高到低分层
	m_dZ = new double[m_nCount] ;
	for( INT_PTR i = 0 ; i < m_nCount ; i++ )
	{
		m_dZ[i] = top - ( i+1 ) * m_dStepZ ;
	}
EndLine:
	return 1 ;
}
BOOL CSmartSRemainGen::PrepareRemain ( CSmartGraphic& Graph )
{
	InitTmpProg() ;
	if( !UpdateTool3D(Graph) )
    {
        return 0 ;
    }
	RepairBlindRadius( m_cFeedDef.m_cPlungeDef ) ;
	if( m_cParam.m_cRemain.m_nRemainType != NCDEF_REMAIN_LASTOPER )
	{
		m_cParam.m_cRemain.m_cLastShape.m_cDepth     = m_cShapeDef.m_cDepth ;
		m_cParam.m_cRemain.m_cLastShape.m_cTaper     = m_cShapeDef.m_cTaper ;
		m_cParam.m_cRemain.m_cLastLayer.m_nLayerType = NCDEF_LAYER_DEPTH ;
	}
    if( m_cParam.m_cRemain.m_nRemainType == NCDEF_REMAIN_LASTDIAM )
    {
	    double dOffDist = 0.5 * ( m_cParam.m_cRemain.m_dLastDiam - m_cToolDef.m_dTopDiam) ; 
		if( dOffDist < 0.01  )
        {
            m_nErrorType = JDERROR_GENPATH_NOAREA ;
            return FALSE ;
        }
        m_pLastTool = m_pTool->OffsetBottom( dOffDist ) ;
    }
    else
    {
	    // 构建上把刀具的加工模型 刀具的stock是用这次图形的还是上次图形的????
	    double stepstock = m_cParam.m_cRemain.m_cLastShape.m_cStock0.m_dDepthStock ;
	    m_pLastTool = CreateSmartTool( m_cParam.m_cRemain.m_cLastTool, stepstock, 0. ) ;
    }
	
	return TRUE ;
}
BOOL CSmartSRemainGen::BuildStockMdl( CSmartGraphic &Graph, CSmtCheckMdl &RoughMdl, BOX3D &box )
{

	////////////////////处理毛坯情况////////////////
	// 首先生产加工面模型
	BOOL bRet = TRUE, bSetRough = FALSE ;
	int nOldType = -1 ;
	if( m_cParam.m_cRemain.m_nRemainType != NCDEF_REMAIN_LASTOPER && !Graph.HasRoughSurf () )
	{
		nOldType = Graph.GetRoughType () ;
		Graph.SetRoughType ( ncStockSurfRemain ) ;
		bSetRough = TRUE ;
	}
	if( !Graph.BuildStockModel ( m_cSetupDef, RoughMdl, box ) )
	{
		bRet = FALSE ;
	}
	int nRoughType = Graph.GetRoughType () ;
	double dStock = Graph.GetRoughStock () ;
	if( nRoughType == ncStockShapeSurf && fabs( dStock ) > 0.0001 )
	{
		m_pRoughTool = CreateSmartTool( m_cToolDef, dStock, 0 ) ;
	}
	else
	{
		m_pRoughTool = CreateSmartTool( m_cToolDef, 0., 0 ) ;
	}
	if( bSetRough )
	{
		Graph.SetRoughType ( nOldType ) ;
	}
	return bRet ;
}
void CSmartSRemainGen::BuildAllDriveLoop ( CSmtLoopArr *pLoopArr, double dMdlTop, int nType, 
										   BOX3D &dMaxBox, JDNC_PRGDEF &ProgDef, double dCur )
{
	UNUSED_ALWAYS( dMdlTop ) ;
	int i = 0, nTotal = m_nCount, nRemain = 0 ;
	if( nType > 1 ) nTotal *= 2 ;
	ProgDef.m_dLimitAt =  nTotal / dCur ;
	ProgDef.m_dStepAt  = 0.0 ;
	ProgDef.m_dIncStep = 1.0  ;
	CSmartLoop *pBigLoop = NULL ;
	// 首先得到加工环的最大包围环
	pBigLoop = CreateBigLoopAndBox( m_pDriveLoop, m_pAllLoop, dMaxBox, m_nCount, m_pTool->m_fRadius * 2 ) ;
	m_bFirst = TRUE ;
	for( i = 0 ; i < m_nCount ; i++ ) 
	{
		// 显示进度条
		ProgDef.m_dStepAt += ProgDef.m_dIncStep ;
		while( ProgDef.m_pPrgFunc && ProgDef.m_dStepAt >= ProgDef.m_dLimitAt )
		{
			ProgDef.m_pPrgFunc(1) ;
			ProgDef.m_dStepAt -= ProgDef.m_dLimitAt  ;
		}
		if( ! LinearOneLayer( m_pDriveLoop, pBigLoop, &pLoopArr[i], i, FALSE ) )
		{
			break ;
		}
	}
	nRemain += m_nCount - i ;
	Mini_DeleteContours( pBigLoop ) ;
	// 如果有毛坯,得到最大边界环
	
	///////////////////毛坯面当前刀具的环////////////////////
	if( nType > 1 )
	{
		pBigLoop = CreateBigLoopAndBox( m_pRoughLoop, m_pAllLoop, dMaxBox, m_nCount, m_pTool->m_fRadius * 2.5 ) ;
		m_bFirst = TRUE ;
		m_pLoopArr1 = new CSmtLoopArr[m_nCount] ;
		for( i = 0 ; i < m_nCount ; i ++ ) 
		{
			// 显示进度条
			ProgDef.m_dStepAt += ProgDef.m_dIncStep ;
			while( ProgDef.m_pPrgFunc && ProgDef.m_dStepAt >= ProgDef.m_dLimitAt )
			{
				ProgDef.m_pPrgFunc(1) ;
				ProgDef.m_dStepAt -= ProgDef.m_dLimitAt  ;
			}
			if( ! LinearOneLayer( m_pRoughLoop, pBigLoop, &m_pLoopArr1[i], i, TRUE ) )
			{
				break ;
			}
		}
		nRemain += m_nCount - i ;
		Mini_DeleteContours( pBigLoop ) ;
	}
	// 补充进度条
	for( i = 0 ; i < nRemain ; i++ )
	{
		ProgDef.m_dStepAt += ProgDef.m_dIncStep ;
		while( ProgDef.m_pPrgFunc && ProgDef.m_dStepAt >= ProgDef.m_dLimitAt )
		{
			ProgDef.m_pPrgFunc(1) ;
			ProgDef.m_dStepAt -= ProgDef.m_dLimitAt  ;
		}
	}
}


int CSmartSRemainGen::LinearOneLayer( CSmartLoop **pDriveLoop,	//加工环组
									  CSmartLoop* pBigLoop,     //边界环
									  CSmtLoopArr* LoopArr,		//路径数组
				                      int  Layer,				//分层序号  		                
									  BOOL bRoughCast )
{
	// 得到等高线 ,并得到环
    CSmartLoop looplib, *BndHead = NULL , *BndTail = NULL ; 
    BndHead = Mini_CopyContours( pDriveLoop[Layer] ) ;
	BndHead = BndTail = MathCAM_ExtractAllLoop( BndHead ) ;
	if( !BndHead ) 
	{
		if( !bRoughCast )
		{//如果高度大于曲面的高度，则用边界来加工
			if( m_bFirst )
			{
				BndHead = Mini_CopyContours( m_pAllLoop ) ;//->CopyContour () ;
			}
		}
		else
		{// 毛坯面有可能低于加工面
			DOUBLE dRadRgh[2] = { 0.005, 0.005 } ;
			JDNC_COR  CorDef = GetCorDef() ;
			CorDef.m_nCorType = NCDEF_CORNER_ARCUSER ;
			BndHead = Mini_OffsetContourList( m_pAllLoop , NCDEF_OFFSET_OUTER, dRadRgh, CorDef ) ;
		}
	}
	else
	{
		while( BndTail && BndTail->next )	BndTail = BndTail->next ;
		CSmartLoop *pAdd = Mini_CopyContours( pBigLoop ) ;
		BndHead = Mini_AddContours( BndHead, pAdd ) ;
		BndHead = looplib.BuildContour ( BndHead ) ;
		m_bFirst = FALSE ;
	}

	if( !BndHead ) return FALSE ;
	// 环的排序
	BndHead = looplib.ResortContour ( BndHead, m_cSetupDef.m_cOrderDef.m_nSortType, FALSE ) ;

	AddLoopToLoopArr( *LoopArr, BndHead, Layer, 0. ) ;
	return 1 ;
}

CSmartLoop* CSmartSRemainGen::GetOneLayerLoop ( CSmtLoopArr& LoopArr, BOOL bRough, BOOL bRemove )
{
	CSmartLoop *pHead = NULL, *pLoop = NULL ;
	for( INT_PTR i = 0 ; i < LoopArr.GetSize () ; i++ )
	{
		pLoop = LoopArr.GetAt(i) ;
		if( !pLoop ) continue ;
		pLoop->next = pLoop->prev = NULL ;
		pHead = Mini_AddContours( pHead, pLoop ) ;
	}
	if( bRemove )
        LoopArr.RemoveAll () ;
	if( bRough )
	{
		DOUBLE dRadRgh[2] ;
		dRadRgh[0] = dRadRgh[1] = 0.005 ;

		JDNC_COR  CorDef = GetCorDef() ;
		CorDef.m_nCorType = NCDEF_CORNER_ARCUSER ;
		CSmartLoop* pTmpHead = Mini_OffsetContourList( pHead  , 
													   NCDEF_OFFSET_OUTER , 
												       dRadRgh         , 
												       CorDef ) ;
		if( pTmpHead )
		{
			Mini_DeleteContours( pHead ) ;
			pHead = pTmpHead ;
		}
		// 为毛坯环加上Blank标记,方便下刀
		for( pLoop = pHead ; pLoop ; pLoop = pLoop->next )
		{
			pLoop->SetBoundInfo ( FALSE ) ;
		}
		SetContourBlank( pHead ) ;
	}
	
	return pHead ;
}

BOOL CSmartSRemainGen::SubtractLoop ( CSmtCheckMdl *CheckMdl, CSmtLoopArr* pTmpArr,
									 JDNC_PRGDEF &ProgDef, double dCur )
{
	CSmartLoop LoopLib, *pDriveHead = NULL, *pLastHead = NULL ;

	ProgDef.m_dLimitAt =  m_nCount / dCur ;
	ProgDef.m_dStepAt  = 0.0 ;
	ProgDef.m_dIncStep = 1.0  ;

	// 将两把刀具的环进行相减,将得到的结果放到当前的数组里面
	double dDepth = 0. ;
	for( int i = 0 ; i < m_nCount ; i++ )
	{
		pDriveHead = GetOneLayerLoop( pTmpArr[i], FALSE, TRUE ) ;
		dDepth = m_dZ[i] ;
		pLastHead = GetOneLayerLoop( m_pLastLoopArr[i], FALSE, FALSE ) ;
		CSmartLoop* pLeft = NULL ;
		// 当当前的深度比上把刀的最底层还深
		if( dDepth < m_dZ[m_nCount-1] )
		{
			pLeft = pDriveHead ;
			AddLoopToLPArr( CheckMdl, m_pLPathArr[i], pLeft, i, TRUE ) ;
		}
		else
		{
			if( pLastHead )
			{
				GetRemainPath( CheckMdl, m_pLPathArr[i], pDriveHead, pLastHead, i ) ;
			}
			else //if( !m_pStockPSSL )
			{// 上把刀具的环不存在,删除当前环Mini_DeleteContours( pDriveHead ) ;
			 // 05.3.21加工不到底,如果中间有缺层的?
				pLeft = pDriveHead ;
				AddLoopToLPArr( CheckMdl, m_pLPathArr[i], pLeft, i, TRUE ) ;
			}
		}
		// 测试
/*		if( MathCAM_HasSameValue( m_dZ[i], 2, NCDEF_TESTDATA_REMAIN_LOOP ) )
		{
			TestLoopCurve( &m_pLPathArr[i], m_dZ[i] ) ;
		}*/
		// 显示进度条
		if( ProgDef.m_pBrkFunc && ProgDef.m_pBrkFunc () )
			return FALSE ;
		ProgDef.m_dStepAt += ProgDef.m_dIncStep ;
		while( ProgDef.m_pPrgFunc && ProgDef.m_dStepAt >= ProgDef.m_dLimitAt )
		{
			ProgDef.m_pPrgFunc(1) ;
			ProgDef.m_dStepAt -= ProgDef.m_dLimitAt  ;
		}
	}
	return TRUE ;
}

int CSmartSRemainGen::GeneratePathEx( CPathGroup& NewPath     ,  /*刀具路径*/
						              CSmartGraphic& Graph )  /*雕刻图形*/
{  
	// step 1 : 初始化普通参数:生成刀具\边界环
	if( !PrepareRemain( Graph ) ) return 0 ;	
	if( !m_pTool ) return 0 ;
	double dArea = GetToolBotArea( m_pTool ) ;
	m_dMaxLen = 0.03 ;
	if( dArea < 0.1 ) 
	{
		m_dMaxLen = dArea * 0.25 ;
	}

	// step 2 : 生成毛坯模型 获取边界环
	BOX3D sbox ;
	mathClnBox3D( &sbox ) ;
	CSmtCheckMdl RoughMdl ;
	if( !BuildStockMdl( Graph, RoughMdl, sbox ) )
	{
		m_nErrorType = JDERROR_GENPATH_NOSTOCK ;
		return 0 ;
	}
	double dCur[7] = { 28, 40, 0, 2, 2, 18, 10 }, dBot = sbox.min[2], dTop = sbox.max[2] ;
	// 得到边界
	m_pAllLoop = CreateAreaLoop( Graph, sbox, m_pRoughTool->m_fRadius * 1.5, m_bAllLoop ) ;	
	if( !m_pAllLoop ) 
	{
		m_nErrorType = JDERROR_GENPATH_NOAREA ;
		return FALSE ;
	}
	sbox.min[2] = dBot, sbox.max[2] = dTop ;
	// step 3 : 生成加工模型
	INT_PTR k = 0 ;
	CSmtCheckMdl DriveMdl, CheckMdl ;
	BOX3D box, dMaxBox ;
	double dMax = max( m_pLastTool->m_fRadius * 2, m_pTool->m_fRadius * 2 ) + 1 ;
	MathCam_GetLoopBox( m_pAllLoop, &box ) ;
	mathExpandBox3D( dMax, &box ) ;
	mathDefBox3D( box.min, box.max, &dMaxBox ) ;
	for( k = 0 ; k < 2 ; k++ )
	{
		DriveMdl.m_dFltBox[0][k] = box.min[k] ;
		DriveMdl.m_dFltBox[1][k] = box.max[k] ; 
	}
	if( !BuildDriveAndCheckMdl ( Graph, DriveMdl, CheckMdl ) )
	{
		m_nErrorType = JDERROR_GENPATH_NOAREA ;
		return 0 ;
	}
	// 使用毛坯限定加工范围
	if( !CalcTopAndBot( sbox ) )	return 0 ;
	RoughMdl.m_fBottom =  (TFLOAT)m_dBotHeight ;
	DriveMdl.m_fBottom = CheckMdl.m_fBottom = TFLOAT( m_dBotHeight ) ;
	FPNT3D minPt, maxPt ;
	DriveMdl.CalcFacetBox ( minPt, maxPt ) ;
	double dMdlTop = maxPt[2] ;
/////////////////////////////以下为新的进度条////////////////////////////////
	ProcessNewFunc( m_cPrgDef, IDS_PATHGEN_STEP_REMAINPATH ) ;
/*
	// STEP 4 : 生成所有的等高环包括毛坯面
	CNcZLevelGenMdl heightMdl ;
	if( !CreateAllDriveLoop( &DriveMdl, heightMdl, m_cPrgDef, dCur[0]  ) )
	{
		m_nErrorType = JDERROR_GENPATH_NOAREA ;
		return 0 ;
	}
	// 搜索平面优化平面环 
	if( m_cParam.m_bSRemainFlag & NCDEF_SREMAIN_CUTPLANE )
    {
		CreateAllPlaneBnd( DriveMdl, heightMdl, m_pAllLoop, m_cPlaneList, m_cPrgDef, dCur[1] ) ;	
    }
	heightMdl.ClearAll () ;	
*/
	if( !CreateRemainAllDriveAndPlaneLoop( &DriveMdl, m_cPrgDef, dCur[0] ) )
	{
		m_nErrorType = JDERROR_GENPATH_NOAREA ;
		return 0 ;
	}
	if( !m_pDriveLoop || !m_dZ ) return 0 ;

	// step 5 : 生成残料模型,然后删除残料模型
	int nType = -1 ; 
	if( !CreateAllRemainLoop ( Graph, DriveMdl, RoughMdl, nType, m_cPrgDef, dCur, NewPath ) )
    {
		return 0 ;
    }
	// step 6 : 生成毛坯环
	if( nType > 1 )
	{
	//	if( !CreateAllRoughLoop( Graph, &RoughMdl, m_cPrgDef, dCur[2] ) )
		if( !CreateRemainAllRoughAndPlaneLoop( Graph, &RoughMdl, m_cPrgDef, dCur[2] ) )
		{
			m_nErrorType = JDERROR_GENPATH_NOSTOCK ;
			return 0  ;
		}
	}
	RoughMdl.ClearAllCheck () ;

	// STEP 7 : 得到加工区域环
	CSmtLoopArr *pTmpArr = new CSmtLoopArr[ m_nCount ] ;

	BuildAllDriveLoop( pTmpArr, dMdlTop, nType, dMaxBox, m_cPrgDef, dCur[3] ) ;
/*		
	TestLoopsArr( pTmpArr, m_dZ, m_nCount, NewPath ) ;
	AddLoopToGroup( m_pAllLoop, 0, NewPath ) ;
*/	
	
	// STEP 8 : 合并残料模型的环和平面环
	TrimPlaneHeadByCombineLoop( m_cPlaneList, dMaxBox, nType ) ;
	CombineRemainLoop( dMaxBox, nType, m_cPrgDef, dCur[4] )  ;

	ClearAllLoopHead( m_pRoughLoop, m_nCount ) ;
//	TestAllPlaneLoop( &m_cPlaneList, NewPath, 1 ) ; // 加工环
//	return 1 ;
	// 申请与加工环有关的空间
	m_pLPathArr = new CSmtLPathArr[m_nCount] ;
	m_pCurveLib = new CSmartCurveLib[m_nCount] ;
	m_pPlunge = new CSmartLoop*[m_nCount] ;
	for( INT_PTR i= 0 ; i < m_nCount ; i++ )
		m_pPlunge[i] = NULL ;
	// 计算上把刀具环和残补区域	
	BOOL bRet = SubtractLoop( &DriveMdl, pTmpArr, m_cPrgDef, dCur[5] ) ;
	ClearAllLoopArr( pTmpArr, m_nCount ) ;
	ClearAllLoopHead( m_pRemainLoop, m_nCount ) ;
	if( !bRet ) return 0 ;
	// 删除不需要的变量
	ClearAllLoopArr( m_pLastLoopArr, m_nCount ) ;
	ClearAllLoopArr( m_pLoopArr1, m_nCount ) ;
	
		
/*	TestLoopsArr( m_pLastLoopArr, m_dZ, m_nCount, NewPath ) ;
	for( i = 0 ; i < m_nCount ; i++ )
	{
		AddLoopToGroup( m_pLPathArr[i], m_dZ[i], NewPath ) ;
		AddLoopToGroup( m_pLPathArr[10], m_dZ[10], NewPath ) ;
		AddCurveLibToGroup( m_pCurveLib[10], m_dZ[10], NewPath ) ;
	}
*/	
	// 如果是残料模型并且选择刀具盲区,则过滤平面刀具盲区环
	if( m_cParam.m_cRemain.m_nRemainType == NCDEF_REMAIN_LASTOPER && 
		m_cFeedDef.m_cPlungeDef.m_bPlungeFlag & NCDEF_PLUNGE_DELBLINDAREA )
	{
		FilterAllPlaneBlindArea( m_cPlaneList ) ;
	}
	if( m_cParam.m_bSRemainFlag & NCDEF_SREMAIN_CUTPLANE )
	{// 为平面环搜索合适的位置
		SearchRationalPosition( m_cPlaneList ) ;
	}
	///////////////////第2段进度条赋值统计所有的环的个数////////////////////
	INT_PTR nTotal = 0 ;
	for( k = 0 ; k < m_nCount ; k++ )
		nTotal +=  m_pLPathArr[k].GetSize () ;
	if( nTotal < 1 )
	{
		m_nErrorType = JDERROR_GENPATH_NOAREA ;
		return 0 ;
	}
	// 进度条
	m_cPrgDef.m_dLimitAt =  nTotal / dCur[6] ;
	m_cPrgDef.m_dStepAt  = 0.0 ;
	m_cPrgDef.m_dIncStep = 1.0  ;
	CSmtCPathLib AllPath ;
	/////////////////////////////////////////////////////////
	if(GetOrderDef().m_nLayerOrd )//==1高度优先  
	{
		for( int i = 0 ; i < m_nCount ; i ++ )
		{
			if( !SortLoopByDepth( AllPath, &DriveMdl, i, m_cPrgDef ) )
			{
				NewPath.ClearAll() ;
				break ;
			}
		}
	}	
	else // == 0区域优先 
	{
		if( !SortLoopByArea ( AllPath,  &DriveMdl, m_nCount, m_cPrgDef ) )
			NewPath.ClearAll() ;
	}
	if( m_cParam.m_bSRemainFlag & NCDEF_SREMAIN_CUTPLANE )
	{
		CreateRemainPlanePath( &DriveMdl, AllPath ) ;
	}
	// 删除下刀边界和环
	ClearAllCurveLib() ;
	ClearAllLoopHead( m_pPlunge, m_nCount ) ;
	ClearAllLoopHead( m_pDriveLoop, m_nCount ) ;
	///////////////////////////////////////////
	CPathCombine *PComb = new CPathCombine( NC_WPROCESS_ROUGH ) ;
//	AllPath.AddToPathCombine ( *PComb ) ;
	AddToPathCombine( AllPath, PComb, m_cSpeedDef.m_dFeedRate ) ;
	NewPath.AddData ( 0, PComb ) ;

	if( ! NewPath.m_pHead )
	{
		m_nErrorType = JDERROR_GENPATH_NOTGENPATH ;
		return 0 ;
	}
	// 添加相对连刀
	AddPlungeAndRelativeH( &DriveMdl, &NewPath, FALSE, m_dZ, m_nCount ) ;

	ClearAllLoop() ;
	return 1 ;
}

int CSmartSRemainGen::SortLoopByArea ( CSmtCPathLib &AllPath, CSmtCheckMdl *DriveMdl, int nLayer, JDNC_PRGDEF &ProgDef ) 
{	
	CSmartLoop *pLoop, *pTmp, *pHead, LoopLib ;
	pHead = pLoop = pTmp = NULL ;
	int i, Layer, nSize = 0, nCnt = 0 ;
	CSmtLoopPath *pLPath = NULL ;
	for( Layer = 0 ; Layer < nLayer ; Layer++ )
	{
		nSize = (int)m_pLPathArr[Layer].GetSize () ;
		for( i = 0 ; i < nSize; i++ )
		{
			pLPath = m_pLPathArr[Layer].GetAt(i)  ;
			for( pLoop = pLPath->m_pLoop ; pLoop ; pLoop = pLoop->next )
			{
                pLoop->m_nDepth = int( Layer ) ;
			}
			if( pLPath->m_pLoop ) 
				pHead = Mini_AddContours( pHead, pLPath->m_pLoop ) ;
		}
	}
	// 将得到的环链进行排序
	pHead = Mini_SortSRemainContours( pHead, 
                                      GetLoopCheckTol(),
                                      m_pTool->m_fRadius * 2.0   ) ;
	BOOL bBreak = FALSE, bPrev = FALSE, bAbsorb = FALSE ;
	BOOL bZigZag = FALSE ;
	PNT3D prev ;
	if( m_cParam.m_bSRemainFlag & NCDEF_SREMAIN_ZIGZAG )
		bZigZag = TRUE ;
	double dDepth = m_dTopHeight - m_dBotHeight ;
	int nMin = 0 ;
	if( pHead ) nMin = pHead->m_nDepth ; // 记录层号
	for( pLoop = pHead ; pLoop ; pLoop = pLoop->next )
	{
		Layer = pLoop->m_nDepth ;
		nSize = (int)m_pLPathArr[Layer].GetSize () ;
		pLPath = NULL ;
		for( i = 0 ; i < nSize ; i++ )
		{
			pLPath = m_pLPathArr[Layer].GetAt ( i ) ;
			if( pLoop == pLPath->m_pLoop )
				break ;
		}
		if( !pLPath || !pLPath->m_pLoop ) continue ;
		nCnt++ ;
		bAbsorb = FALSE ;
		// 将路径加到路径组中
		if( !GetAllPathEndPoint( AllPath, prev ) )
			bPrev = FALSE ;
		else
			bPrev = TRUE ;
		if( nMin == 0 || Layer == 0 ) 
			dDepth = m_dTopHeight ;
		else
		{
			dDepth = max( m_dZ[nMin-1], m_dZ[Layer-1] ) ;
		}
		if( ( Layer - nMin  <= 1 ) && ( Layer - nMin ) >= -1 )
			bAbsorb = TRUE ;
		pLPath->ConnectAllCurve_ArcLink ( DriveMdl, m_pPlunge[Layer], m_pCurveLib[Layer],
								  bZigZag, bPrev, prev, m_cSetupDef, dDepth ) ;
		AddCutPathToAllPath( NULL, AllPath, DriveMdl, m_pTool->m_fRadius * 2 ,
							 pLPath->m_cPathLib, m_cSetupDef.m_cTolDef, bAbsorb ) ;
		// 如果选择过滤平面
		if( m_cParam.m_bSRemainFlag & NCDEF_SREMAIN_CUTPLANE )
		{
			CreatePlanePath( DriveMdl, AllPath, pLPath->m_pLoop, Layer, NCDEF_RPOCKET_CAST ) ;
		}
	//	AddTestPathToAllPath( AllPath, pLPath->m_cPathLib ) ;
		pLPath->m_cPathLib.ClearAllPath () ;
		pLPath->m_pLoop = NULL ;
		nMin = Layer ;
		// 显示进度条
		if( ProgDef.m_pBrkFunc && ProgDef.m_pBrkFunc () )
		{
			bBreak = TRUE ;
			break ;
		}
		ProgDef.m_dStepAt += ProgDef.m_dIncStep ;
		while( ProgDef.m_pPrgFunc && ProgDef.m_dStepAt >= ProgDef.m_dLimitAt )
		{
			ProgDef.m_pPrgFunc(1) ;
			ProgDef.m_dStepAt -= ProgDef.m_dLimitAt  ;
		}
	}
	// 清空所有Loop
	for( Layer = 0 ; Layer < nLayer ; Layer++ )
	{
		nSize = (int)m_pLPathArr[Layer].GetSize () ;
		for( i = 0 ; i < nSize; i++ )
		{
			pLPath = m_pLPathArr[Layer].GetAt(i)  ;
			pLPath->m_pLoop = NULL ;
		}
	}
	if( pHead ) Mini_DeleteContours( pHead ) ;
	if( bBreak ) 
		return 0 ;
	return 1 ;
}

/********************************************************
 * 通过深度来重新组环，生成路径。将同一深度的所有环找出 *
 * 来，生成加工路径，放到路径组当中。依次寻找，直到搜索 *
 * 完所有的环。                                         *
*********************************************************/
int CSmartSRemainGen::SortLoopByDepth( CSmtCPathLib &AllPath, CSmtCheckMdl *DriveMdl, int nLayer, JDNC_PRGDEF &ProgDef ) 
{
	CSmtLoopPath *pLPath ;
	BOOL bZigZag = FALSE, bPrev = TRUE ;
	if( m_cParam.m_bSRemainFlag & NCDEF_SREMAIN_ZIGZAG )
		bZigZag = TRUE ;
	PNT3D prev ;
	double dDepth = m_dTopHeight ;
	if( nLayer > 0 )
		dDepth = m_dZ[nLayer-1] ;
	for( INT_PTR i = 0 ; i < m_pLPathArr[nLayer].GetSize () ; i++ )
	{
		pLPath = m_pLPathArr[nLayer].GetAt ( i ) ;
		// 将路径加到路径组中
		if( !GetAllPathEndPoint( AllPath, prev ) )
			bPrev = FALSE ;
		else
			bPrev = TRUE ;

		pLPath->ConnectAllCurve_ArcLink ( DriveMdl, m_pPlunge[nLayer], m_pCurveLib[nLayer], 
								  bZigZag, bPrev, prev, m_cSetupDef, dDepth ) ;
		AddCutPathToAllPath( NULL, AllPath, DriveMdl, m_pTool->m_fRadius * 2 ,
							 pLPath->m_cPathLib, m_cSetupDef.m_cTolDef, TRUE ) ;
		pLPath->m_cPathLib.ClearAllPath () ;
		Mini_DeleteContours( pLPath->m_pLoop ) ;

		// 显示进度条
		if( ProgDef.m_pBrkFunc && ProgDef.m_pBrkFunc () )
			return FALSE ;
		ProgDef.m_dStepAt += ProgDef.m_dIncStep ;
		while( ProgDef.m_pPrgFunc && ProgDef.m_dStepAt >= ProgDef.m_dLimitAt )
		{
			ProgDef.m_pPrgFunc(1) ;
			ProgDef.m_dStepAt -= ProgDef.m_dLimitAt  ;
		}
	}
	// 添加平坦面路径
	if( m_cParam.m_bSRemainFlag  & NCDEF_SREMAIN_CUTPLANE )
	{
		CreatePlanePath( DriveMdl, AllPath, NULL, nLayer, NCDEF_RPOCKET_DEPTH ) ;
	}
	return 1 ;
}


BOOL CSmartSRemainGen::ConnectLine ( CSmtCheckMdl *DriveMdl, PNT3D start, PNT3D end, JDNC_TOL& cTol )
{
	CSmtCutPath tmpPath( MINI_CONNECT_PATH ) ;
	tmpPath.AddPoint ( start ) ;
	tmpPath.AddPoint ( end ) ;
	tmpPath.InsertCPoint ( cTol.m_dMaxStep ) ;
	CSmtCutPoint *pHead = tmpPath.m_pHead ;
	for( ; pHead ; pHead = pHead->next )
	{
		pHead->m_fPoint[3] = pHead->m_fPoint[2] ;
	}
	tmpPath.VerifyLinePath ( *DriveMdl, cTol, m_cTmpPrgDef ) ;

	BOOL bConnect = TRUE ;
	pHead = tmpPath.m_pHead ;
	for( ; pHead ; pHead = pHead->next )
	{
		if( pHead->m_fPoint[2] > pHead->m_fPoint[3] + 0.001 )
		{
			bConnect = FALSE ;
			break ;
		}
	}
	return bConnect ;
}

BOOL CSmartSRemainGen::CreateRemainAllDriveAndPlaneLoop( CSmtCheckMdl *DriveMdl, JDNC_PRGDEF &ProgDef, double dCur ) 
{
	BOOL bLocalCopy = FALSE ;
	if (IsNeedMultiCopy(*DriveMdl) )
	{
        bLocalCopy = TRUE ;
        DriveMdl->CreateMultiCopy(m_nCalcThreadNum - 1) ;
    }

	CNcZLevelGenMdl heightMdl ;
	ProgDef.m_dTotalMove = dCur * 0.4 ;
    if( ! heightMdl.Create( *DriveMdl, NULL, m_cSetupDef, ProgDef ) )
    {
        return FALSE ;
    }
	CSmartLoop *PlaneLoop = NULL ;
	double dPlaneCur = dCur * 0.2, dLastCur = dCur * 0.4 ;
	BOOL bFillet = FALSE ;
	if( m_cParam.m_cRemain.m_nRemainType == NCDEF_REMAIN_TOOLDEF &&
	    m_cParam.m_cRemain.m_cLastTool.m_nToolType == smtToolBall &&
	    m_cParam.m_bSRemainFlag & NCDEF_SREMAIN_CONTRAST )
    {
		bFillet = TRUE ;
    }
	// 搜索平面优化平面环 
	if( m_cParam.m_bSRemainFlag & NCDEF_SREMAIN_CUTPLANE && ( !bFillet ) )
    {
		// 生成平面加工区域
		CSmartFPlaneGen FPlaneGen ;
		(*(CSmartPathGen*)&FPlaneGen ) = * this ;
		FPlaneGen.m_cMethodDef.m_nMethodType = surfncMethodFPlane ;
		SurfNC_InitPathParam( FPlaneGen.m_cParam ) ;
		FPlaneGen.m_cMethodDef.m_cPlaneCut = FPlaneGen.m_cParam ;
		ProgDef.m_dTotalMove = dPlaneCur ;
		FPlaneGen.CreatePlaneContour ( *DriveMdl, &heightMdl, PlaneLoop, ProgDef ) ;
//		for( CSmartLoop *pHead = PlaneLoop ; pHead ; pHead = pHead->next )
//		{
//			pHead->m_dHeight += 0.005 ;
//		}
    }
	else
	{
		dLastCur = dCur * 0.6 ;
	}
	// 得到平面环的高度
	double *dZ = NULL, *dAlldZ = NULL ;
	int nCnt = 0, nAllCnt = 0, i = 0 ;
	BOOL *bFlag = NULL, bCombine = FALSE, bRet = TRUE ;
	if( PlaneLoop && GetAllBndContHeight( PlaneLoop, dZ, nCnt ) )
	{ 
		// 将dZ和m_dZ合并得到新的层数和高度	
		CombineAllHeight( m_dZ, m_nCount, dZ, nCnt, dAlldZ, nAllCnt, bFlag ) ;
		bCombine = TRUE ;
		if( dZ ) delete[] dZ ;
	}
	else
	{
		dAlldZ = m_dZ ;
		nAllCnt = m_nCount ;
	}
	// 生成等高路径
	ProgDef.m_dTotalMove = dLastCur ;
	CSmartLoop **  AllLoop = new CSmartLoop*[nAllCnt] ;
	for( i = 0 ; i < nAllCnt ; i++ )
    {
		AllLoop[i] = NULL ;
    }
	
	if( !heightMdl.CreateContourLoops ( dAlldZ, AllLoop, int( nAllCnt ), ProgDef ) )
	{//已经并为区域
		ClearAllLoopHead( AllLoop, nAllCnt ) ;
		bRet = FALSE ;
		goto EndLine ;
	}
	
	if( AllLoop && dAlldZ )
	{
		// 得到所有加工域环
		GetRemainAllDriveLoop( AllLoop, dAlldZ, bFlag, nAllCnt ) ;
		// 得到所有平面环
		if( m_cParam.m_bSRemainFlag & NCDEF_SREMAIN_CUTPLANE && PlaneLoop )
			GetRemainAllPlaneLoop( AllLoop, dAlldZ, bFlag, nAllCnt, PlaneLoop ) ;
	}
	
	ClearAllLoopHead( AllLoop, nAllCnt ) ;
	// 光滑区域圆角
	if( (m_cSetupDef.m_cModelTol.m_nMdlFlag & NCDEF_SETUP_SMOOTHCORNER) && 
        m_cSetupDef.m_cTolDef.m_dMinRad > 0.02 )
    {
	    JDNC_COR tmpCor = m_cSetupDef.m_cCorDef ;
	    tmpCor.m_nCorType = NCDEF_CORNER_ARCALL ; 
	    DOUBLE dOffDist[2] = { m_cSetupDef.m_cTolDef.m_dMinRad,m_cSetupDef.m_cTolDef.m_dMinRad} ;
	    for( i = 0 ; i < m_nCount ; i++ )
	    {
            CSmartLoop* pOldBnd = m_pDriveLoop[i] ;
            m_pDriveLoop[i] = NULL ; 
            for( CSmartLoop* pLp = pOldBnd ; pLp ; pLp = pLp->next )
            {
                CSmartLoop* pInnLp = pLp->OffsetContour( NCDEF_OFFSET_OUTER, dOffDist[0], dOffDist[1], tmpCor ) ;
                CSmartLoop* pNewLp = Mini_OffsetContourList( pInnLp, NCDEF_OFFSET_INNER, dOffDist, tmpCor ) ;
                Mini_DeleteContours( pInnLp ) ;
                m_pDriveLoop[i] = Mini_AddContours( m_pDriveLoop[i], pNewLp ) ;
            }
            Mini_DeleteContours( pOldBnd ) ;
        }
	}
EndLine:
	if( bLocalCopy == TRUE ) 
    {
        DriveMdl->DeleteMultiCopy() ;
    }
	if( bCombine )
	{
		if( dAlldZ ) delete[] dAlldZ ;
		if( bFlag  ) delete[] bFlag  ;
	}
	return bRet ;
}
void CSmartSRemainGen::GetRemainAllDriveLoop ( CSmartLoop **AllLoop, double *dZ, BOOL *bFlag, int nCnt )
{
	if( !AllLoop || !dZ ) return ;
	INT_PTR i = 0, j = 0 ;
	m_pDriveLoop = new CSmartLoop *[m_nCount] ;
	for( i = 0 ; i < m_nCount ; i++ )
	{
		m_pDriveLoop[i] = NULL ;
		// 从AllLoop中寻找环
		if( bFlag )
		{
			for( j = 0 ; j < nCnt ; j++ )
			{
				if( !bFlag[j] ) continue ;
				if( m_dZ[i] == dZ[j] )
				{
					m_pDriveLoop[i] = Mini_CopyContours( AllLoop[j] ) ;
					break ;
				}
			}
		}
		else
		{
			m_pDriveLoop[i] = Mini_CopyContours( AllLoop[i] ) ;
		}
	}
}
void MathCAM_SetContoursHeight( CSmartLoop* pNewLp, DOUBLE Height ) ;
void CSmartSRemainGen::GetRemainAllPlaneLoop ( CSmartLoop **AllLoop, double *dZ, BOOL *bFlag, int nCnt, CSmartLoop *PlaneLoop )
{
	if( !AllLoop || !dZ || !bFlag || !PlaneLoop ) return ;
	INT_PTR i = 0 ;
	while( PlaneLoop )
	{
        CSmartLoop *pHead = PlaneLoop ;
        PlaneLoop = PlaneLoop->next ;
		pHead->prev = pHead->next = NULL ;
        if( pHead->m_dHeight <= m_dBotHeight || pHead->m_dHeight >= m_dTopHeight )
        {
            Mini_DeleteContours( pHead ) ;
            continue ;
        }
		// 匹配合适的边界
		CSmartLoop *pTemp = Mini_OperateContours( pHead, m_pAllLoop, 0 ) ;
        if( pTemp == NULL ) 
        {
            Mini_DeleteContours( pHead ) ;
            continue ;
        }
		MathCAM_SetContoursHeight( pTemp, pHead->m_dHeight ) ;
		for( i = 0 ; i < nCnt ; i++ )
		{/*替换原始边界*/
			if( bFlag[i] ) continue ;
			if( fabs( pHead->m_dHeight - dZ[i] ) < 0.0003 )
			{
				Mini_DeleteContours( pHead ) ;
				pHead = Mini_CopyContours( AllLoop[i] ) ;
				break ;
			}
		}
		CPlaneLoop *pPlaneLoop = NULL ;
		while ( pTemp )
		{
			CSmartLoop* pTmpNext = pTemp->next ;
			pTemp->prev = pTemp->next = NULL ;
			if( pTmpNext == NULL )
			{
				pPlaneLoop = new CPlaneLoop( pTemp, pHead ) ;
			} 
			else
			{
				pPlaneLoop = new CPlaneLoop( pTemp, Mini_CopyContours(pHead ) ) ;
			}
			m_cPlaneList.AddTail( pPlaneLoop ) ;
			pTemp = pTmpNext ;
		}
	}
	// 计算删除距离
	double dMaxZ = m_cSetupDef.m_cTolDef.m_dArcTol * 3 ;
	if( fabs( m_cStockDef.m_dDriveOffset[0] ) < 0.0001 )
		dMaxZ = m_cSetupDef.m_cTolDef.m_dArcTol ;
	else
        dMaxZ = min( m_dStepZ * 0.2, dMaxZ ) ;
	
    // 对得到的环进行过滤,删除与分层相近的环
	double dTmpH = 0., dMinZ = 0. ;
	double dTol = m_pTool->m_fRadius * 0.1, dRatio = 0.4 ;
	BOOL bDelete = FALSE ;
	CPlaneLoop *pPlaneHead = NULL ;
	POSITION pos, atpos = NULL ;
	pos = m_cPlaneList.GetHeadPosition () ;
	while( pos )
	{
		atpos = pos ;
		pPlaneHead = m_cPlaneList.GetNext ( pos ) ;
		dMinZ = m_dStepZ ;

		pPlaneHead->m_pPlaneLoop->DefineBox () ;
		// 如果该环的深度和等高的分层很近,并在分层下方,则该环可以去掉
		for( i = -1 ; i < m_nCount ; i++ )
		{
			if( i == -1 )
			{
				dTmpH = m_dTopHeight - pPlaneHead->m_pPlaneLoop->m_dHeight ;
			}
			else
			{
				dTmpH = m_dZ[i] - pPlaneHead->m_pPlaneLoop->m_dHeight ;
			}
			if( dTmpH < dMinZ && dTmpH > 0 )
			{
				dMinZ = dTmpH ;
			}
			if( dMinZ < dMaxZ ) break ;
			// 然后处理极值情况,当两层距离非常之小,是否也应该删除?
			if( fabs( dTmpH ) < 0.0005 )
			{
				dMinZ = dTmpH ;
				break ;
			}
		}
		// 对找到的最近距离进行处理
		if( dMinZ < dMaxZ )
		{ // 当两个距离很近时
			bDelete = TRUE ;
			if( i == m_nCount - 1 && 
				IsBottomPlaneValid( m_pDriveLoop[i], pPlaneHead->m_pPlaneLoop, dTol, dRatio ) )
			{// 防止表面余量为0且底面为平面时dMInZ为-0.0003,删除底面平面,而底面路径被边界减掉
				bDelete = FALSE ;
			}
			if( bDelete )
			{
				m_cPlaneList.RemoveAt ( atpos ) ;
				delete pPlaneHead ;
			}
		}
	}
}
BOOL CSmartSRemainGen::CreateAllDriveLoop ( CSmtCheckMdl *DriveMdl, CNcZLevelGenMdl &heightGen, JDNC_PRGDEF &ProgDef, double dCur ) 
{
	ProgDef.m_dTotalMove = dCur * 0.4 ;
    BOOL bLocalCopy = FALSE ;
	if (IsNeedMultiCopy(*DriveMdl) )
	{
        bLocalCopy = TRUE ;
        DriveMdl->CreateMultiCopy(m_nCalcThreadNum - 1) ;
    }
	if( ! heightGen.Create( *DriveMdl, NULL, m_cSetupDef, ProgDef ) )
	{
		return FALSE ;
	}
	// 利用给定的高度得到每一层的环
	m_pDriveLoop = new CSmartLoop*[m_nCount] ;
    INT_PTR i ;
	for( i = 0 ; i < m_nCount ; i++ )
	{
		m_pDriveLoop[i] = NULL ;
	}
	ProgDef.m_dTotalMove = dCur * 0.6 ;
	if( !heightGen.CreateContourLoops ( m_dZ, m_pDriveLoop, int( m_nCount ), ProgDef ) )
	{//已经并为区域
		return FALSE ;
	}
    if( bLocalCopy == TRUE ) 
    {
        DriveMdl->DeleteMultiCopy() ;
    }
	// 光滑区域圆角进行处理
	if( (m_cSetupDef.m_cModelTol.m_nMdlFlag & NCDEF_SETUP_SMOOTHCORNER) && 
		m_cSetupDef.m_cTolDef.m_dMinRad > 0.02 )
	{
		JDNC_COR tmpCor = m_cSetupDef.m_cCorDef ;
		tmpCor.m_nCorType = NCDEF_CORNER_ARCALL ; 
		DOUBLE dOffDist[2] = { m_cSetupDef.m_cTolDef.m_dMinRad,m_cSetupDef.m_cTolDef.m_dMinRad} ;
	    for( i = 0 ; i < m_nCount ; i++ )
	    {
			CSmartLoop* pOldBnd = m_pDriveLoop[i] ;
			m_pDriveLoop[i] = NULL ; 
			for( CSmartLoop* pLp = pOldBnd ; pLp ; pLp = pLp->next )
			{
				CSmartLoop* pInnLp = pLp->OffsetContour( NCDEF_OFFSET_OUTER, dOffDist[0], dOffDist[1], tmpCor ) ;
				CSmartLoop* pNewLp = Mini_OffsetContourList( pInnLp, NCDEF_OFFSET_INNER, dOffDist, tmpCor ) ;
				Mini_DeleteContours( pInnLp ) ;
				m_pDriveLoop[i] = Mini_AddContours( m_pDriveLoop[i], pNewLp ) ;
			}
			Mini_DeleteContours( pOldBnd ) ;
		}
	}
	return TRUE ;
}
BOOL CSmartSRemainGen::CreateRemainAllRoughAndPlaneLoop ( CSmartGraphic &Graph, CSmtCheckMdl *RoughMdl, JDNC_PRGDEF &ProgDef, double dCur )
{
	int nRoughType = Graph.GetRoughType () ;
	double dStock = Graph.GetRoughStock () ;
	if( nRoughType == ncStockShapeSurf && fabs( dStock ) > 0.0001 )
	{
		RoughMdl->ZMoveModel ( dStock ) ;
	}
	RoughMdl->UpdateCurrTool ( m_pRoughTool ) ;

    BOOL bLocalCopy = FALSE ;
	if (IsNeedMultiCopy(*RoughMdl) )
	{
        bLocalCopy = TRUE ;
        RoughMdl->CreateMultiCopy(m_nCalcThreadNum - 1) ;
    }
	CNcZLevelGenMdl heightMdl ;
	ProgDef.m_dTotalMove = dCur * 0.4 ;
	if( ! heightMdl.Create( *RoughMdl, NULL, m_cSetupDef, ProgDef ) )
	{
		return FALSE ;
	}
	
	// 得到平面环的高度
	double *dZ = NULL, *dAlldZ = NULL ;
	int nCnt = 0, nAllCnt = 0, i = 0 ;
	BOOL *bFlag = NULL, bCombine = FALSE, bRet = TRUE ;
	if( GetAllPlaneHeight( &m_cPlaneList, dZ, nCnt ) )
	{ 
		// 将dZ和m_dZ合并得到新的层数和高度	
		CombineAllHeight( m_dZ, m_nCount, dZ, nCnt, dAlldZ, nAllCnt, bFlag ) ;
		bCombine = TRUE ;
		if( dZ ) delete[] dZ ;
	}
	else
	{
		dAlldZ = m_dZ ;
		nAllCnt = m_nCount ;
	}
	// 生成等高路径
	CSmartLoop loop ;
	ProgDef.m_dTotalMove = dCur *  0.6 ;
	CSmartLoop **  AllLoop = new CSmartLoop*[nAllCnt] ;
	for( i = 0 ; i < m_nCount ; i++ )
	{
		AllLoop[i] = NULL ;
	}

	if( !heightMdl.CreateContourLoops ( dAlldZ, AllLoop, int( nAllCnt ), ProgDef ) )
	{//已经并为区域
		ClearAllLoopHead( AllLoop, nAllCnt ) ;
		bRet = FALSE ;
		goto EndLine ;
	}
	if( AllLoop && dAlldZ )
	{
		// 得到所有加工域环
		GetRemainAllRoughLoop( AllLoop, dAlldZ, bFlag, nAllCnt ) ;
		// 得到所有平面环
		if( m_cParam.m_bSRemainFlag & NCDEF_SREMAIN_CUTPLANE  )
		{
			TrimRemainAllPlaneLoop( AllLoop, dAlldZ, bFlag, nAllCnt, m_cPlaneList ) ;
		}
	}

	ClearAllLoopHead( AllLoop, nAllCnt ) ;
	// 对区域进行处理
	for( INT_PTR i = 0 ; i < m_nCount ; i++ )
	{
		m_pRoughLoop[i] = MathCAM_ExtractAllLoop( m_pRoughLoop[i] ) ;
		m_pRoughLoop[i] = loop.BuildContour ( m_pRoughLoop[i] ) ;
		// 毛坯环全部设为blank
		SetContourBlank( m_pRoughLoop[i] ) ;
	}
EndLine:
    if( bLocalCopy == TRUE )
    {
        RoughMdl->DeleteMultiCopy() ;
    }
	if( bCombine )
	{
		if( dAlldZ ) delete[] dAlldZ ;
		if( bFlag  ) delete[] bFlag  ;
	}
	return bRet ;
}
void CSmartSRemainGen::GetRemainAllRoughLoop( CSmartLoop **AllLoop, double *dZ, BOOL *bFlag, int nCnt )
{
	if( !AllLoop || !dZ ) return ;
	INT_PTR i = 0, j = 0 ;
	m_pRoughLoop = new CSmartLoop*[m_nCount] ;
	for( i = 0 ; i < m_nCount ; i++ )
	{
		m_pRoughLoop[i] = NULL ;
		// 从AllLoop中寻找环
		if( bFlag )
		{
			for( j = 0 ; j < nCnt ; j++ )
			{
				if( !bFlag[j] ) continue ;
				if( m_dZ[i] == dZ[j] )
				{
					m_pRoughLoop[i] = Mini_CopyContours( AllLoop[j] ) ;
					break ;
				}
			}
		}
		else
		{
			m_pRoughLoop[i] = Mini_CopyContours( AllLoop[i] ) ;
		}
	}
}
void CSmartSRemainGen::TrimRemainAllPlaneLoop( CSmartLoop **AllLoop, double *dZ, BOOL *bFlag, int nCnt, CPlaneList &cList )
{
	CPlaneLoop *pHead = NULL ;
	CSmartLoop *pRoughHead = NULL ;
		
	POSITION pos = cList.GetHeadPosition () ;
	while( pos )
	{
		pHead = cList.GetNext ( pos ) ;
		pRoughHead = NULL ;
		// 生成毛坯环
		for( INT_PTR i = 0 ; i < nCnt ; i++ )
		{
			if( bFlag[i] ) continue ;
			if( fabs( pHead->m_dHeight - dZ[i] ) < 0.0003 )
			{
				pRoughHead = Mini_CopyContours( AllLoop[i] ) ;
				break ;
			}
		}
		if( !pRoughHead ) continue ;
		pHead->SetRoughLoop ( pRoughHead ) ;
	}
}
BOOL CSmartSRemainGen::CreateAllRoughLoop ( CSmartGraphic &Graph, CSmtCheckMdl *RoughMdl, JDNC_PRGDEF &ProgDef, double dCur )
{
	int nRoughType = Graph.GetRoughType () ;
	double dStock = Graph.GetRoughStock () ;
	if( nRoughType == ncStockShapeSurf && fabs( dStock ) > 0.0001 )
	{
		RoughMdl->ZMoveModel ( dStock ) ;
	}
	RoughMdl->UpdateCurrTool ( m_pRoughTool ) ;
    BOOL bLocalCopy = FALSE ;
	if (IsNeedMultiCopy(*RoughMdl) )
	{
        bLocalCopy = TRUE ;
        RoughMdl->CreateMultiCopy(m_nCalcThreadNum - 1) ;
    }
	// 生成自适应网格
    CNcZLevelGenMdl    heightGen ;
	ProgDef.m_dTotalMove = dCur * 0.4 ;
    if( ! heightGen.Create( *RoughMdl, NULL, m_cSetupDef, ProgDef ) )
    {
        return FALSE ;
    }
	// 生成等高环
	m_pRoughLoop = new CSmartLoop*[m_nCount] ;
	for( INT_PTR i = 0 ; i < m_nCount ; i++ )
	{
		m_pRoughLoop[i] = NULL ;
	}
	ProgDef.m_dTotalMove = dCur * 0.6 ;
	if( !heightGen.CreateContourLoops ( m_dZ, m_pRoughLoop, int( m_nCount ), ProgDef ) )
	{//已经并为区域
		return FALSE ;
	}
	// 过滤平坦面
	if( m_cPlaneList.GetCount() > 0 )
	{
		CPlaneLoop *PlaneLoop = NULL ;
		POSITION pos = m_cPlaneList.GetHeadPosition () ;
		while( pos )
		{
			PlaneLoop = m_cPlaneList.GetNext ( pos ) ;
			heightGen.CreateContourLoop ( PlaneLoop->m_pRoughLoop, PlaneLoop->m_dHeight ) ;
		}
	}
    if( bLocalCopy == TRUE )
    {
        RoughMdl->DeleteMultiCopy() ;
    }
	return TRUE ;
}

void CSmartSRemainGen::CombineRemainLoop ( BOX3D dMaxBox, int nType, JDNC_PRGDEF &ProgDef, double dCur )
{
	CSmartLoop LoopLib ;
	int i = 0, nTotal = m_nCount * 2 ;
	ProgDef.m_dLimitAt =  nTotal / dCur ;
	ProgDef.m_dStepAt  = 0.0 ;
	ProgDef.m_dIncStep = 1.0  ;
	
	// 得到一个比m_pAllLoop大的环
	double dRadRgh[2] = { 3 * m_cSetupDef.m_cTolDef.m_dArcTol, 3 * m_cSetupDef.m_cTolDef.m_dArcTol } ; 
	CSmartLoop *pLastAllLoop = Mini_OffsetContourList( m_pAllLoop, NCDEF_OFFSET_OUTER, dRadRgh, GetCorDef() ) ;

	// 合并残料区域
    m_pLastLoopArr = new CSmtLoopArr[ m_nCount ] ;
	CSmartLoop *BndHead = NULL, *pTmp = NULL ;
	for( i = 0 ; i < m_nCount ; i++ )
	{
		if( !m_pRemainLoop[i] ) 
		{
			m_pRemainLoop[i] = Mini_CopyContours( pLastAllLoop ) ;
			m_bFlag[i] = TRUE ;
		}
		
		// 显示进度条
		ProgDef.m_dStepAt += ProgDef.m_dIncStep ;
		while( ProgDef.m_pPrgFunc && ProgDef.m_dStepAt >= ProgDef.m_dLimitAt )
		{
			ProgDef.m_pPrgFunc(1) ;
			ProgDef.m_dStepAt -= ProgDef.m_dLimitAt  ;
		}
	}
	
	// 将残补环转成残补区域
	CSmartLoop *pBigLoop = CreateBigLoopAndBox( m_pRemainLoop, m_pAllLoop, dMaxBox, m_nCount, m_pTool->m_fRadius * 3 ) ;
	CSmartLoop *pRoughHead = NULL, *pResult = NULL ;
	for( i = 0 ; i < m_nCount ; i++ )
	{
		if( !m_bFlag[i] )
		{
			pTmp = Mini_CopyContours( pBigLoop ) ;
			BndHead = Mini_CopyContours( m_pRemainLoop[i] ) ;
			BndHead = Mini_AddContours( BndHead, pTmp ) ;
			BndHead = LoopLib.BuildContour ( BndHead ) ;
			// 外轮廓开阔,忽略外边框
			SetContourBlank( BndHead ) ;
		}
		else
		{
			BndHead = Mini_CopyContours( m_pRemainLoop[i] ) ;
		}
		// 上把刀环和毛坯环求并
		if( nType > 1 )
		{
			pRoughHead = GetOneLayerLoop( m_pLoopArr1[i], TRUE, TRUE ) ;
            pResult = Mini_OperateContours( BndHead, pRoughHead, 1 ) ;
			Mini_DeleteContours( pRoughHead ) ;
			Mini_DeleteContours( BndHead ) ;
		}
		else
		{
			pResult = BndHead ;
			if( !m_bFlag[i] )
			{ // 对残补环进行偏移,防止在精加工的残料模型直壁处生成多余路径2010/05/22 (YUL)
				DOUBLE dRadRgh[2] = { 0.003, 0.003 } ;
				JDNC_COR  CorDef = GetCorDef() ;
				CorDef.m_nCorType = NCDEF_CORNER_ARCUSER ;
				CSmartLoop* pTmpHead = Mini_OffsetContourList( pResult, NCDEF_OFFSET_OUTER, dRadRgh, CorDef ) ;
				if( pTmpHead )
				{
					Mini_DeleteContours( pResult ) ;
					pResult = pTmpHead ;
				}
			}
		}
		
		BndHead = pResult ;
		AddLoopToLoopArr( m_pLastLoopArr[i], BndHead, i, 0. ) ;
		
		// 显示进度条
		ProgDef.m_dStepAt += ProgDef.m_dIncStep ;
		while( ProgDef.m_pPrgFunc && ProgDef.m_dStepAt >= ProgDef.m_dLimitAt )
		{
			ProgDef.m_pPrgFunc(1) ;
			ProgDef.m_dStepAt -= ProgDef.m_dLimitAt  ;
		}
	}
	Mini_DeleteContours( pBigLoop ) ;

//	ClearAllLoopHead( m_pRemainLoop, m_nCount ) ;
	// 将毛坯环和残补环求并,留作非优化模式的下刀
	for( i = 0 ; i < m_nCount ; i++ )
	{
		if( nType > 1 )
		{
			pResult = Mini_OperateContours( m_pRemainLoop[i], m_pRoughLoop[i], 1 ) ;
			if( pResult )
			{
				Mini_DeleteContours( m_pRemainLoop[i] ) ;
				m_pRemainLoop[i] = pResult ;
			}
		}
		SetContourBlank( m_pRemainLoop[i] ) ;
	}
	ClearAllLoopHead( m_pRoughLoop, m_nCount ) ;
	delete[] m_bFlag ;
	m_bFlag = NULL ;
	Mini_DeleteContours( pLastAllLoop ) ;
}
BOOL CSmartSRemainGen::CompareToolCorner ( CSmartTool *pCurr, CSmartTool *pLast )
{
	int nType1 = pCurr->GetType () ;
	int nType2 = pLast->GetType () ;
	if( nType1 != smtToolFlat && nType1 != smtToolNose ||
		nType2 != smtToolFlat && nType2 != smtToolNose  )
    {
		return FALSE ;
    }
	double r1 = pCurr->m_fCorner ;
	double r2 = pLast->m_fCorner ;
	// 刀具圆角之差比较小
	if( fabs( r2 - r1 ) > m_cSetupDef.m_cTolDef.m_dArcTol * 2 )
    {
		return FALSE ;
    }
	// 模型的抬高也比较接近的情况下,才采用偏环的方式进行计算。
	if( fabs( m_cStockDef.m_dDriveZMove[0] - m_cParam.m_cRemain.m_cLastShape.m_cStock0.m_dSideStock ) > m_cSetupDef.m_cTolDef.m_dArcTol * 2 )
	{
		return FALSE ;
	}
	return TRUE ;
}
BOOL CSmartSRemainGen::CreateAllRemainLoop ( CSmartGraphic &Graph, CSmtCheckMdl &DriveMdl,  
                                             CSmtCheckMdl &StockMdl, int &nType, 
											 JDNC_PRGDEF &ProgDef, double dCur[7],
											 CPathGroup& /*NewPath*/ )
{
	m_pRemainLoop = new CSmartLoop*[m_nCount] ;
	CSmartLoop LoopLib, *pOffset = NULL, *pLoop = NULL ;
	INT_PTR i = 0 ;
	m_bFlag = new BOOL[m_nCount] ;
	for( i = 0 ; i < m_nCount ; i++ )
	{
		m_pRemainLoop[i] = NULL ;
		m_bFlag[i]	   = FALSE ;
	}
	BOOL bRet = TRUE ;
	POSITION pos, atpos ;
	CPlaneLoop *PlaneLoop = NULL ;
	// 使用路径来生成上次残料模型
  	if( (m_cParam.m_cRemain.m_nRemainType == NCDEF_REMAIN_TOOLDEF ) &&
	    (m_cParam.m_cRemain.m_cLastTool.m_nToolType == smtToolBall) &&
	    (m_cParam.m_bSRemainFlag & NCDEF_SREMAIN_CONTRAST ) )
	{// sbg 光滑圆角,此处的残补加工用于清根之前的残补,所以不用作平台面过滤
		nType = 2 ;
		dCur[2] = dCur[1] * 0.25, dCur[1] = dCur[1] * 0.75 ;
		ProgDef.m_dTotalMove = dCur[1] ;
		double dLastZMove = m_cParam.m_cRemain.m_cLastShape.m_cStock0.m_dSideStock ;
		double dToolOff = m_cParam.m_cRemain.m_cLastShape.m_cStock0.m_dDepthStock ;
        CSmartTool *pMyTool = CreateSmartTool( m_cToolDef, dToolOff,0.0 ) ;
	    if( pMyTool == NULL) 
	    {
            m_nErrorType = JDERROR_GENPATH_ERRDATA  ;
            return 0 ;
	    }
		bRet = NcBnd_GenZLevelContours( DriveMdl, m_cSetupDef, ProgDef, 
			 						   m_pLastTool->m_fRadius, pMyTool, dLastZMove,
									   m_dZ, m_pRemainLoop, int( m_nCount ) ) ;
        delete pMyTool ;
	}
  	else if( ((m_cParam.m_cRemain.m_nRemainType == NCDEF_REMAIN_TOOLDEF ) &&
              (CompareToolCorner( m_pTool, m_pLastTool ) )) ||
             ( m_cParam.m_cRemain.m_nRemainType == NCDEF_REMAIN_LASTDIAM ) )
	{ // 两把刀具的圆角很接近,本来可以计算当前刀具环偏出,但为了整体结构,使用上把刀具环偏移得到
		dCur[2] = dCur[1] * 0.2, dCur[1] = dCur[1] * 0.8 ;
		nType = 2 ;
		// 使用上把刀具生成环
		DriveMdl.UpdateCurrTool ( m_pLastTool ) ;
        BOOL bLocalCopy = FALSE ;
        if( IsNeedMultiCopy( DriveMdl ) )
        {
            bLocalCopy = TRUE ;
            DriveMdl.CreateMultiCopy(m_nCalcThreadNum - 1) ;
        }
		CNcZLevelGenMdl    heightGen ;
		ProgDef.m_dTotalMove = dCur[1] * 0.3 ;
		if( !heightGen.Create ( DriveMdl, NULL, m_cSetupDef, ProgDef ) )
		{
			bRet = FALSE ;
			goto EndLine ;
		}
		ProgDef.m_dTotalMove = dCur[1] * 0.5 ;
		if( !heightGen.CreateContourLoops ( m_dZ, m_pRemainLoop, int( m_nCount ), ProgDef ) )
		{//已经并为区域
			bRet = FALSE ;
			goto EndLine ;
		}
		// 对环进行偏移
		ProgDef.m_dTotalMove = dCur[1] * 0.2 ;
		ProgDef.m_dLimitAt =  m_nCount / ProgDef.m_dTotalMove ;
		ProgDef.m_dStepAt  = 0.0 ;
		ProgDef.m_dIncStep = 1.0  ;
		double dOffset = 0. ;
		CSmartTool *pCurrSafe = CreateSmartTool( m_cToolDef, 0., 0. ) ;
		if( m_cParam.m_cRemain.m_nRemainType == NCDEF_REMAIN_LASTDIAM )
		{
			dOffset = m_cParam.m_cRemain.m_dLastDiam * 0.5 - pCurrSafe->m_fRadius + 3 * m_cSetupDef.m_cTolDef.m_dArcTol ;
		}
		else
		{
			CSmartTool *pLastSafe = CreateSmartTool( m_cParam.m_cRemain.m_cLastTool, 0., 0.) ;
			dOffset = pLastSafe->m_fRadius - pCurrSafe->m_fRadius + 3 * m_cSetupDef.m_cTolDef.m_dArcTol ;
			delete pLastSafe ;
		}
		delete pCurrSafe ;
		double dRadRgh[2] = { dOffset, dOffset } ;
		for( INT_PTR i = 0 ; i < m_nCount ; i++ )
		{
			ProgDef.m_dStepAt += ProgDef.m_dIncStep ;
			while( ProgDef.m_pPrgFunc && ProgDef.m_dStepAt >= ProgDef.m_dLimitAt )
			{
				ProgDef.m_pPrgFunc(1) ;
				ProgDef.m_dStepAt -= ProgDef.m_dLimitAt  ;
			}
			if( !m_pRemainLoop[i] )
			{
				if( m_pDriveLoop[i] )
				{ // 如果当前环存在,使用边界环代替上把刀具环
                    m_pRemainLoop[i] = Mini_CopyContours( m_pAllLoop ) ;
				}
				else
				{
				}
			}
			else
			{
				pOffset = OffsetContourList( m_pRemainLoop[i], NCDEF_OFFSET_INNER, dRadRgh, 0., m_cSetupDef.m_cCorDef ) ;
				if( pOffset )
				{
					Mini_DeleteContours( m_pRemainLoop[i] ) ;
					m_pRemainLoop[i] = pOffset ;
				}
			}
		}
		
		// 过滤平面环
		if( m_cPlaneList.GetCount () > 0 )
		{
			pos = m_cPlaneList.GetHeadPosition () ;
			while( pos )
			{
				PlaneLoop = m_cPlaneList.GetNext ( pos ) ;
				heightGen.CreateContourLoop ( pLoop, PlaneLoop->m_dHeight ) ;
				if( !pLoop ) continue ;
				pOffset = OffsetContourList( pLoop, NCDEF_OFFSET_INNER, dRadRgh, 0., m_cSetupDef.m_cCorDef ) ;
				if( pOffset )
				{
					Mini_DeleteContours( pLoop ) ;
				}
				PlaneLoop->m_pRemainLoop = pOffset ;
			}
		}
        if( bLocalCopy == TRUE )
        {
            DriveMdl.DeleteMultiCopy() ;
        }
		/**/
EndLine:
		DriveMdl.UpdateCurrTool ( m_pTool ) ;
		heightGen.ClearAll () ;
	}
	else
	{ // Mesh算法
		double dTmpCur1 = dCur[1] * 0.8, dTmpCur2 = dCur[1] * 0.2 ;
		bRet = BuildRemainZMap( Graph, &DriveMdl, &StockMdl, ProgDef, dTmpCur1 ) ;
		nType = 1 ;
		ProgDef.m_dTotalMove = dTmpCur2 ;
        if( m_pRemainMdl && bRet )
        {
            m_pRemainMdl->GetContoursAtMutipleZLevel( m_dZ, m_pRemainLoop,(int) m_nCount,ProgDef ) ;
		}
		if( bRet && m_pRemainMdl && m_cPlaneList.GetCount() > 0  )
		{// 过滤平面环,使用残料模型对平面路径进行删除,保证平坦面在残补模型之内
			PNT3D minpt, maxpt ;
			m_pRemainMdl->GetBoundBox ( minpt, maxpt ) ;
			if( maxpt[2] > m_dTopHeight ) maxpt[2] = m_dTopHeight ;
			if( minpt[2] < m_dBotHeight ) minpt[2] = m_dBotHeight ;
			pos = m_cPlaneList.GetHeadPosition () ;
			while( pos )
			{
				atpos = pos ;
				PlaneLoop = m_cPlaneList.GetNext ( pos ) ;
				if( PlaneLoop->m_dHeight > maxpt[2] || PlaneLoop->m_dHeight < m_dBotHeight )
				{
					delete PlaneLoop ;
					m_cPlaneList.RemoveAt ( atpos ) ;
					continue ;
				}
				PlaneLoop->m_pRemainLoop = m_pRemainMdl->GetContoursAtZLevel ( (TFLOAT)PlaneLoop->m_dHeight ) ;
                while( pos )
                {/*复制高度相同的平面环的残料边界*/
                    CPlaneLoop *nextLp = m_cPlaneList.GetAt( pos ) ;
                    if( nextLp->m_dHeight != PlaneLoop->m_dHeight )
                    {
                        break ;
                    }
                    nextLp->m_pRemainLoop = Mini_CopyContours( PlaneLoop->m_pRemainLoop ) ;
                    m_cPlaneList.GetNext( pos );
                }
			}
		}
///////////////////for test////////////////////////
//		TestRemainMdl( m_pRemainMdl, NewPath ) ;
///////////////////test end ///////////////////////
	}
	if( m_pRemainMdl )
    {
        delete m_pRemainMdl ;
	    m_pRemainMdl = NULL ;
    }
	if( !bRet )
	{
		m_nErrorType = JDERROR_GENPATH_NOREMAINMDL ;
	}
	return bRet ;
}
#ifdef _NEW_REMAIN_MDL 
int CSmartSRemainGen::BuildRemainZMap( CSmartGraphic& Graph, CSmtCheckMdl *DriveMdl, 
									   CSmtCheckMdl *StockMdl, JDNC_PRGDEF &ProgDef, double dCur )
{
	// STEP 1 : 初始化Mesh模型,加入毛坯
	int nRoughType = Graph.GetRoughType () ;
	double dStock = Graph.GetRoughStock (), dRemain = 0 ;
	if( nRoughType == ncStockShapeSurf && fabs( dStock ) > 0.0001 )
	{
		dRemain = dStock ;
	}
	m_pRemainMdl = new CSmtRemainMdl() ;
	m_pRemainMdl->EnableMultiThread( m_nCalcThreadNum ) ;
    MathCAM_RegisterMeshMdlFunc( *m_pRemainMdl ) ;
    m_pRemainMdl->m_cSetupDef = m_cSetupDef ;
    m_pRemainMdl->m_cPrgDef   = ProgDef ;
    m_pRemainMdl->m_cSetupDef.m_cTolDef.m_dArcTol = max( 0.005, m_cSetupDef.m_cTolDef.m_dArcTol ) ;
    if( ! m_pRemainMdl->InitRemainMdl( *StockMdl, m_pTool->m_fRadius, dRemain ) )
    {
        return FALSE ;
    }
	double dMinStock = 0. ;
    // STEP 2 :根据刀具路径计算ZMesh表示残料模型
    CSmartTool * pMyTool = NULL ;
	if( m_cParam.m_cRemain.m_nRemainType == NCDEF_REMAIN_LASTOPER )
	{//当前残料模型
		CPtrList		AllEnt ;
		Graph.GetAllRemainPath( m_cSetupDef, AllEnt, &dMinStock ) ;

	//	if( AllEnt.GetCount() == 0 ) 
	//	{
	//		m_nErrorType = JDERROR_GENPATH_NOPATH  ;
	//		return 0 ;
	//	}
		INT_PTR nNumPath = AllEnt.GetCount() ;
		ProgDef.m_dTotalMove = dCur * 0.5 / nNumPath ;

		CJDToolGroup *pCurrToolGroup = MathCAM_GetCurrToolGroup() ;
		while( AllEnt.GetCount())
		{
			CPathGroup* pPathGroup = ( CPathGroup*) AllEnt.RemoveHead() ;
            CJDTool* pNcTool =  NULL ; 
			if( pCurrToolGroup )
			{
				pNcTool = pCurrToolGroup->GetByID( pPathGroup->m_nToolID ) ;
            }
			if( pNcTool == NULL)
			{
				delete pPathGroup ;
				continue ;
			}
			pMyTool = CreateSmartTool( pNcTool->m_cToolParam, m_cSetupDef.m_cModelTol.m_dThickOffset,0.0 ) ;
			pPathGroup->ZMoveGroup ( -m_cSetupDef.m_cModelTol.m_dThickZMove ) ;
			m_pRemainMdl->ModifyByPathGroup( *pPathGroup, 
                                              m_cSetupDef.m_cTolDef, 
                                              pMyTool, ProgDef) ;
			delete pPathGroup ;
			delete pMyTool ;
			if( SurfNC_IsAbort())
			{
				break ;
			}
		}
		while( AllEnt.GetCount())
		{
			CPathGroup* pPathGroup = ( CPathGroup*) AllEnt.RemoveHead() ;
			delete pPathGroup ;
		}
	}
	else if( DriveMdl )
	{//指定上把刀具
		double dStock = m_cParam.m_cRemain.m_cLastShape.m_cStock0.m_dSideStock - GetDriveSurfZMove() ;
        TFLOAT fOldZBtm = DriveMdl->m_fBottom ;
        TFLOAT fZDepth  = TFLOAT( m_pLastTool->GetMaxCutDepth() + 0.1 + fabs(dStock) ) ;
		DriveMdl->ZMoveModel( dStock ) ;
        DriveMdl->SetBottom( fOldZBtm - fZDepth ) ;
		DriveMdl->UpdateCurrTool( m_pLastTool ) ;
	    CSmartTool* pLTool = CreateSmartTool(m_cParam.m_cRemain.m_cLastTool, 0.0, 0. ) ;
		ProgDef.m_dTotalMove = dCur * 0.5 ;
		m_pRemainMdl->ZTrimByLastTMdl( *DriveMdl, pLTool, ProgDef ) ;
		DriveMdl->ZMoveModel( -dStock ) ;
        DriveMdl->SetBottom( fOldZBtm ) ;
		DriveMdl->UpdateCurrTool( m_pTool     ) ;
        delete pLTool ;
	}
	if( ProgDef.m_pBrkFunc && ProgDef.m_pBrkFunc () )
	{
		m_nErrorType = JDERROR_GENPATH_ABORT  ;
		return 0 ;
	}
	// STEP 3 :根据ZMesh残料模型计算ZMesh路径模型
	DOUBLE dOffset = min( 0.02,m_pRemainMdl->m_fNetStep * 0.1f) ;
	pMyTool = CreateSmartTool( m_cToolDef, -dOffset,0.0 ) ;
	if( pMyTool == NULL) 
	{
        m_nErrorType = JDERROR_GENPATH_ERRDATA  ;
        return 0 ;
	}

	ProgDef.m_dTotalMove = dCur * 0.5 ;
	m_pRemainMdl->OffsetToToolZMap( pMyTool, ProgDef ) ;
	m_pRemainMdl->ZMove( - (TFLOAT)dOffset ) ;
	delete pMyTool ;
    if( ProgDef.m_pBrkFunc && ProgDef.m_pBrkFunc () )
    {
        m_nErrorType = JDERROR_GENPATH_ABORT  ;
        return FALSE ;
    }/**/
	return TRUE ;
}
#else 
int CSmartSRemainGen::BuildRemainZMap( CSmartGraphic& Graph, CSmtCheckMdl *DriveMdl, 
									   CSmtCheckMdl *StockMdl, JDNC_PRGDEF &ProgDef, double dCur )
{
	UNUSED_ALWAYS( StockMdl ) ;
	// STEP 1 : 构建残料模型
	FPNT3D fMinPnt, fMaxPnt ;
    if( m_pAllLoop )
    {/*使用轮廓的范围*/
		BOX3D box1 ;
		MathCam_GetLoopBox( m_pAllLoop, &box1 ) ;
        for( int k = 0 ; k < 2 ; k ++ ) 
        {
            fMinPnt[k] = (TFLOAT) box1.min[k] ;
            fMaxPnt[k] = (TFLOAT) box1.max[k] ;
        }
    }
    else
    {/*使用面片的范围*/
	    DriveMdl->CalcFacetBox( fMinPnt, fMaxPnt ) ;
    }
	fMinPnt[2] = fMaxPnt[2] = TFLOAT( m_dTopHeight ) ;
	double dRadius = max( m_pTool->m_fRadius, 0.5 * m_cParam.m_cRemain.m_cLastTool.m_dTopDiam );
	for( int k = 0 ; k < 2 ; k ++ ) 
	{
		fMinPnt[k] -= TFLOAT( dRadius * 1.5 + 0.2f) ;
		fMaxPnt[k] += TFLOAT( dRadius * 1.5 + 0.2f) ;
	}
	m_pRemainMdl = new CSmartBitmap() ;
	m_pRemainMdl->InitBuffer( fMinPnt, fMaxPnt ,0.1f, 0.1f ) ;
    // STEP 2 :根据刀具路径计算ZMap残料模型
    CSmartTool * pMyTool = NULL ;
	if( m_cParam.m_cRemain.m_nRemainType == NCDEF_REMAIN_LASTOPER )
	{
		CPtrList		AllEnt ;
		Graph.GetAllRemainPath( m_cSetupDef, AllEnt ) ;

		if( AllEnt.GetCount() == 0 ) 
		{
			m_nErrorType = JDERROR_GENPATH_NOPATH  ;
			return 0 ;
		}
		DOUBLE dPathLen = 0.0 ;
		POSITION atPos = AllEnt.GetHeadPosition() ;
		while( atPos ) 
		{
			CPathGroup* pPathGroup = ( CPathGroup*) AllEnt.GetNext( atPos) ;
			dPathLen += pPathGroup->GetPathLen() ;
		}
		
		ProgDef.m_dTotalMove = dCur ;
		CJDToolGroup *pCurrToolGroup = MathCAM_GetCurrToolGroup() ;
		while( AllEnt.GetCount())
		{
			CPathGroup* pPathGroup = ( CPathGroup*) AllEnt.RemoveHead() ;
            CJDTool* pNcTool =  NULL ; 
			if( pCurrToolGroup )
			{
				pNcTool = pCurrToolGroup->GetByID( pPathGroup->m_nToolID ) ;
            }
			if( pNcTool == NULL)
			{
				delete pPathGroup ;
				continue ;
			}
			pMyTool = CreateSmartTool( pNcTool->m_cToolParam, 0.0,0.0 ) ;
			m_pRemainMdl->ModifyByPathGroup( *pPathGroup, 
                                              m_cSetupDef.m_cTolDef, 
                                              pMyTool, ProgDef ) ;
			delete pPathGroup ;
			delete pMyTool ;
			if( ProgDef.m_pBrkFunc && ProgDef.m_pBrkFunc () )
			{
				break ;
			}
		}
		while( AllEnt.GetCount())
		{
			CPathGroup* pPathGroup = ( CPathGroup*) AllEnt.RemoveHead() ;
			delete pPathGroup ;
		}
	}
	else
	{
		double dStock = m_cParam.m_cRemain.m_cLastShape.m_cStock.m_dDepthStock - GetDriveSurfZMove() ;
        TFLOAT fOldZBtm = DriveMdl->m_fBottom ;
        TFLOAT fZDepth  = TFLOAT( m_pLastTool->GetMaxCutDepth() + 0.1 + fabs(dStock) ) ;
		DriveMdl->ZMoveModel ( dStock  ) ;
        DriveMdl->SetBottom( fOldZBtm - fZDepth ) ;
		DriveMdl->UpdateCurrTool ( m_pLastTool ) ;
	    CSmartTool* pLTool = CreateSmartTool( m_cParam.m_cRemain.m_cLastTool, 0.0, 0. ) ;
		ProgDef.m_dTotalMove = dCur ;
		m_pRemainMdl->ZTrimByLastTMdl( *DriveMdl, pLTool, PrOgDef ) ;
		DriveMdl->ZMoveModel( -dStock ) ;
        DriveMdl->SetBottom( fOldZBtm ) ;
		DriveMdl->UpdateCurrTool( m_pTool     ) ;
        delete pLTool ;
	}
	if( ProgDef.m_pBrkFunc && ProgDef.m_pBrkFunc () )
	{
		m_nErrorType = JDERROR_GENPATH_ABORT  ;
		return 0 ;
	}
	// STEP 2 :根据ZMap残料模型计算ZMap路径模型
	DOUBLE dOffset = min( m_cToolDef.m_dTopDiam*0.1,m_pRemainMdl->m_fStepXX * 0.5f) ;
	pMyTool = CreateSmartTool( m_cToolDef, -dOffset,0.0 ) ;
	if( ! pMyTool ) 
	{
        m_nErrorType = JDERROR_GENPATH_ERRDATA  ;
        return 0 ;
	}
	m_pRemainMdl->OffsetToToolZMap( pMyTool, ProgDef ) ;
	delete pMyTool ;
    if( ProgDef.m_pBrkFunc && ProgDef.m_pBrkFunc () )
    {
        m_nErrorType = JDERROR_GENPATH_ABORT  ;
        return 0 ;
    }
	return TRUE ;
}
#endif 

void CSmartSRemainGen::GetRemainPath( CSmtCheckMdl *CheckMdl, CSmtLPathArr& LPathArr, CSmartLoop* pCurr, CSmartLoop* pLast, int nLayer )
{
	UNUSED_ALWAYS( CheckMdl ) ;
	if( !pCurr || !pLast ) return ;

	// 将两组轮廓相减,得到加工的区域,用于排序
	CSmartLoop * pTmp = NULL, *pHead = NULL ;
	pTmp = Mini_OperateContours( pCurr, pLast, 2 ) ;
	pHead = Mini_OperateContours( pTmp, m_pAllLoop, 0 ) ;
	Mini_DeleteContours( pTmp ) ;
	if( !pHead ) 
	{
		Mini_DeleteContours( pCurr ) ;
		return ;
	}
	MathCAM_ValidOutMostLoop( pHead ) ;
	// 删除狭长环
	if( m_cParam.m_cRemain.m_nRemainType != NCDEF_REMAIN_LASTOPER )
    {
        DeleteInvalidLoop( pHead, m_cSetupDef.m_cTolDef, m_dMaxLen ) ;
    }
	// 如果是残料模型并且选择刀具盲区,则删除刀具盲区环
	if( m_cParam.m_cRemain.m_nRemainType == NCDEF_REMAIN_LASTOPER && 
		m_cFeedDef.m_cPlungeDef.m_bPlungeFlag & NCDEF_PLUNGE_DELBLINDAREA )
	{
		DeleteBlindLoop( pHead, m_cFeedDef.m_cPlungeDef.m_dBlindRadius ) ;
	}
	AddLoopToLPArr( CheckMdl, LPathArr, pHead, nLayer, FALSE ) ;
	m_pPlunge[nLayer] = Mini_CopyContours( pCurr ) ;
	// 利用环环相减生成路径
	if( m_cParam.m_bSRemainFlag & NCDEF_SREMAIN_OPTIMIZE )
	{
		double dOffSet[2] = { 0., 0. } ;
		int nOffType = NCDEF_OFFSET_INNER ;
		BOOL bRet = FALSE ;
		int nLevel = 0 ;
		CSmartCurveLib CurLib ;

		while( 1 )
		{
			if( !pCurr ) break ;
			CSmartLoop *pOffset = OffsetContourList( pCurr, nOffType, dOffSet, 0., m_cSetupDef.m_cCorDef ) ;
			if( nLevel == 0 && m_cParam.m_bSRemainFlag & NCDEF_SREMAIN_BOUNDEND )
                dOffSet[0] = dOffSet[1] = m_cParam.m_dBoundTol ;
			else
                dOffSet[0] = m_cFeedDef.m_cStepDef.m_dOverStep, dOffSet[1] = m_cFeedDef.m_cStepDef.m_dOverStep ;
			if( !pOffset ) break ;
			bRet = SubtractContour( pOffset, pLast, m_pAllLoop, CurLib, m_dMaxLen ) ;
			Mini_DeleteContours( pCurr ) ;
			pCurr = pOffset ;
			if( bRet )
			{
				// 将曲线放到pLPathArr的环中，并且设置闭合标记
				if( !AddCurveToLPath( LPathArr, CurLib, nLevel ) )
					break ;
				nLevel++ ;
			}
			else
			{
				Mini_DeleteContours( pCurr ) ;
				break ;
			}
		}
	}
//	pTmp  = Mini_OperateContours( pLast, m_pAllLoop, 0 ) ;
	CopyPlungeCurve( pLast, m_pCurveLib[nLayer] ) ;
	BOOL bZigZag = FALSE ;
	if( m_cParam.m_bSRemainFlag & NCDEF_SREMAIN_ZIGZAG )
		bZigZag = TRUE ;
	// 生成环切路径
	JDNC_PLUNGE cPlunge = m_cFeedDef.m_cPlungeDef ;
	if( nLayer == 0 )
		cPlunge.m_dIncStep = m_dTopHeight - m_dZ[nLayer] ;
	else
		cPlunge.m_dIncStep = m_dZ[nLayer-1] - m_dZ[nLayer] ;
	
	INT_PTR i = 0, nSize = LPathArr.GetSize () ;
	CSmtLoopPath *pLPath = NULL ;
	CSmartCurveLib *pLib = NULL ;
	CSmartCurve *pCurve = NULL ;
	// 为防止没有清角路径，对于只有一个闭合环的进行处理
	for( i = 0 ; i < nSize ; i++ )
	{
		 pLPath = LPathArr.GetAt ( i ) ;
		// 如果优化,进行最近连刀
		if( m_cParam.m_bSRemainFlag & NCDEF_SREMAIN_OPTIMIZE && 
			pLPath->m_cLibArr.GetSize () == 1 && !(pLPath->m_bClosedFlag) )
		{
			pLib = pLPath->m_cLibArr.GetAt ( 0 ) ;
			pCurve = pLib->m_pHead ;
			if( pLib->m_nNumCurve == 1 && pCurve->IsClosed () )
			{
				pLPath->m_bClosedFlag = TRUE ;
			}
		}
	}
	for( i = 0 ; i < nSize ; i++ )
	{
		pLPath = LPathArr.GetAt ( i ) ;
		// 如果优化,进行最近连刀
		if( m_cParam.m_bSRemainFlag & NCDEF_SREMAIN_OPTIMIZE && 
			pLPath->m_cLibArr.GetSize () > 0 && !(pLPath->m_bClosedFlag) )
		{
			pLPath->SetPlungeInfo ( cPlunge ) ;
		}
		else 
		{
			GenFollowPath( CheckMdl, pLPath, pLast, nLayer ) ;
		}
	}
	Mini_DeleteContours( pCurr ) ;
//	Mini_DeleteContours( pTmp  ) ;
}

void CSmartSRemainGen::AddLoopToLPArr( CSmtCheckMdl *CheckMdl,CSmtLPathArr& LPathArr, CSmartLoop* pHead, int Layer, BOOL bAddPath )
{
	if( !pHead ) return ;
	CSmtLoopPath *pLPath = NULL ;
	CSmartLoop* pLoop = pHead ;
	CSmartLoop* pN = NULL ;
	PNT2D p ;
	double r = min( m_pTool->m_fRadius, 0.4 ) ;
	while( pLoop  )
	{
		pN = pLoop->next ;
		pLoop->prev = pLoop->next = NULL ;
        pLoop->m_nDepth = int( Layer ) ;
		pLoop->m_pCurve->GetStart ( p ) ;
		pLPath = new CSmtLoopPath( pLoop, m_cFeedDef, m_dZ[Layer], r, m_cFeedDef.m_cStepDef.m_dOverStep, TRUE ) ;
		if( m_cParam.m_bSRemainFlag & NCDEF_SREMAIN_BOUNDEND )
		{
			pLPath->SetBoundInfo( m_cSpeedDef, TFLOAT( m_cParam.m_dFeedRatio * 0.01 ), TRUE, m_cParam.m_dBoundTol ) ;
		}
		else
		{
			pLPath->SetBoundInfo ( m_cSpeedDef, 1.f, FALSE, m_cFeedDef.m_cStepDef.m_dOverStep ) ;
		}
		pLPath->m_pPtLoop = m_pDriveLoop[Layer] ;
		if( bAddPath )
		{
			GenFollowPath( CheckMdl, pLPath, NULL, Layer ) ;
			pLPath->m_bAddPath = TRUE ;
		}
		LPathArr.Add ( pLPath ) ;
		pLoop = pN ;
	}
}

int CSmartSRemainGen::GenFollowPath( CSmtCheckMdl *DriveMdl, CSmtLoopPath *pLPath, CSmartLoop *pLast, int nLayer )
{
	UNUSED_ALWAYS( pLast ) ;
	if( !pLPath || !pLPath->m_pLoop ) return 0 ;

	CPathCombine *PComb = new CPathCombine( NC_WPROCESS_ROUGH ) ;
//	if( pLPath->m_PComb->m_pHead ) return 0 ;
	pLPath->m_pLoop->next = pLPath->m_pLoop->prev = NULL ;
	double delH = 0., dLastH = 0 ;
	if( nLayer == 0 )
	{
		dLastH = m_dTopHeight + m_cSpeedDef.m_dRapidHeight ;
		delH = m_dTopHeight - m_dZ[nLayer] ;
	}
	else
	{
		dLastH = m_dZ[nLayer-1] ;
		delH = m_dZ[nLayer-1] - m_dZ[nLayer] ;
	}

	JDNC_FOLLOW  FollowCut ;
	FollowCut.m_dOverStep  = m_cFeedDef.m_cStepDef.m_dOverStep ;
	FollowCut.m_dOverRatio = m_cFeedDef.m_cStepDef.m_dOverRatio ;
	FollowCut.m_nFollowType= 0 ; 
	FollowCut.m_bFollowFlag = NCDEF_FOLLOW_MINUP ;		// 最少抬刀
	if (pLPath->m_bClosedFlag)
	{// edit by liuxin 2013.7.17, 封闭路径从内向外走刀
		FollowCut.m_bFollowFlag |= NCDEF_FOLLOW_TOOUTER;
	}	
	if( m_cFeedDef.m_cSlotDef.m_nSlotType != NCDEF_TSLOT_CLOSE ) 
		FollowCut.m_bFollowFlag |= NCDEF_FOLLOW_SLOTCUT ;
	
	// 环切清角并光滑圆角
	FollowCut.m_bFollowFlag |= NCDEF_FOLLOW_CORNER ;
	FollowCut.m_bFollowFlag |= NCDEF_FOLLOW_SMOOTH ;
		
	if( m_cFeedDef.m_cSlotDef.m_nSlotType != NCDEF_TSLOT_CLOSE ) 
	{ // 计算开槽路径
		PComb->m_nType = NC_WPROCESS_SLOT  ;
		m_cFeedDef.m_cSlotDef.m_dDepthInc =  delH ;
		DefineSlotDepth( m_cFeedDef.m_cSlotDef ) ;
		CreateECutPath( *PComb  , 
						*pLPath->m_pLoop  , 
						m_cFeedDef.m_cSlotDef , 
						m_cFeedDef.m_cPlungeDef ) ;
	}
	PComb->m_nType = NC_WPROCESS_ROUGH  ;

	// STEP 2 : 环切路径
	// 防止环切时尖角处出现过切，2014.6.30 liuxin
	JDNC_SETUP cSetupDef = m_cSetupDef;
	if (FollowCut.m_bFollowFlag & NCDEF_FOLLOW_SMOOTH)
	{
		cSetupDef.m_cCorDef.m_nCorType = NCDEF_CORNER_ARCUSER;
	}
	CSmartNcFollow FollowGen( m_pTool, FollowCut, m_cFeedDef, /*m_cSetupDef*/cSetupDef ) ;
	// 精修边界
	if( m_cParam.m_bSRemainFlag & NCDEF_SREMAIN_BOUNDEND )
        FollowGen.SetBoundtol ( m_cParam.m_dBoundTol, m_cParam.m_dFeedRatio ) ;
	FollowGen.CreatePocketPath ( *pLPath->m_pLoop, *PComb ) ;

	
	// STEP 3 : 计算下刀路径
	CSmartPocketGen  PocketGen ;
	(*(CSmartPathGen*)&PocketGen ) = * this ;
    PocketGen.m_cSpeedDef.m_dPlungeDist = 0.0;

	JDNC_PLUNGE  Plunge = m_cFeedDef.m_cPlungeDef ;
	DOUBLE dMaxDist[2] = { max( 6.0, 1.5 * FollowCut.m_dOverStep * 1.5 ), 0.3 } ;
	if( Plunge.m_nPlungeType != NCDEF_PLUNGE_CLOSE )
	{
		Plunge.m_dIncStep = delH  + Plunge.m_dTopTol ;
		PComb->m_nType = NC_WPROCESS_PLUNGE ;
        m_bFollowOuter = 0 ;
		int nFlag[2] = { surfncMoveFollow, m_cSetupDef.m_cOrderDef.m_nMillDir } ;
		PocketGen.InsertPlungePath( *PComb, *pLPath->m_pLoop, Plunge, m_pDriveLoop[nLayer], 
									m_pRemainLoop[nLayer], dMaxDist, nFlag ) ;
        m_bFollowOuter = 1 ;
	}
	// 减少抬刀
	if( ( m_cFeedDef.m_cLayerDef.m_bLayerFlag & NCDEF_LAYER_KEEPDOWN ) && nLayer > 0 ) 
	{
		// 对路径进行连接
		CRoughLink cLink( m_dZ[nLayer] ) ;
		cLink.AddLinkPath ( DriveMdl, PComb, m_cSetupDef.m_cTolDef, dLastH ) ;
	}
	TransfPCombineToCPath( *PComb, pLPath->m_cPathLib, m_cSpeedDef.m_dFeedRate, m_dZ[nLayer], m_cSetupDef.m_cTolDef ) ;
	delete PComb ;
	//for test　测试环的正确性
/*	AddSmtLoopInPGroup( BndHead, dDepth,  PGroup ) ;*/
	return  1 ;
}

void CSmartSRemainGen::TestLoopError ( CSmartLoop *pCurr, CSmartLoop *pLast, int nLayer )
{
	CPathCombine *PComb = NULL ;
	// 添加等高环
	PComb = AddLoopToPComb( m_pDriveLoop[nLayer], NC_WPROCESS_IMPORT ) ;
	if( PComb )
	{
		PComb->MoveCombine ( 0., 0., m_dZ[nLayer] ) ;
        MathCAM_AddPComb( PComb ) ;
	}
	// 添加当前环
	if( pCurr )
	{
		PComb = AddLoopToPComb( pCurr, NC_WPROCESS_ROUGH ) ;
		if( PComb )
		{
			MathCAM_AddPComb( PComb ) ;
		}
	}
	// 添加上次环
	if( pLast )
	{
		PComb = AddLoopToPComb( pLast, NC_WPROCESS_ROUGH ) ;
		if( PComb )
		{
			MathCAM_AddPComb( PComb ) ;
		}
	}
}
void CSmartSRemainGen::TestLoopCurve ( CSmtLPathArr *pLPath, double dZ )
{
	CPathCombine *PComb = NULL ;
	INT_PTR i, n = pLPath->GetSize () ;
	CSmtLoopPath *pPath = NULL ;
	for( i = 0 ; i < n ; i++ )
	{
		pPath = pLPath->GetAt (i) ;
		PComb = AddLoopToPComb( pPath->m_pLoop, NC_WPROCESS_ROUGH ) ;
		if( PComb )
		{
			PComb->MoveCombine ( 0., 0., dZ ) ;
			MathCAM_AddPComb( PComb ) ;
		}
	}
}

void CSmartSRemainGen::CopyPlungeCurve ( CSmartLoop *pLast, CSmartCurveLib& BndCurve )
{
	if( !pLast ) return ;
	double r = min( m_pTool->m_fRadius, 0.4 ) ;
	double dRadRgh[2] = { r, r } ;

	JDNC_COR  CorDef = GetCorDef() ;
	CorDef.m_nCorType = NCDEF_CORNER_ARCUSER ;
	CSmartLoop *pCopy = Mini_OffsetContourList( pLast, 
												NCDEF_OFFSET_INNER,
												dRadRgh,
												CorDef ) ;
	CSmartLoop *pLoop = pCopy, *pIsland = NULL ;
	for( ; pLoop ; pLoop = pLoop->next )
	{
		BndCurve.AddTail ( pLoop->m_pCurve->CopyMyself () ) ;
		pIsland = pLoop->GetIsland () ;
		for( ; pIsland ; pIsland = pIsland->next )
		{
			BndCurve.AddTail ( pIsland->m_pCurve->CopyMyself () ) ;
		}
	}
	Mini_DeleteContours( pCopy ) ;
}
CSmartLoop *CSmartSRemainGen::CopyPlungeLoop( CSmartLoop *pLast )
{
	CSmartLoop *pLoop, loop, *pObj = NULL ;
	CSmartLoop *pCopy = Mini_CopyContours( pLast ) ;
	pCopy = MathCAM_ExtractAllLoop( pCopy ) ;

	pLoop = pObj = pCopy ;
	for( ; pLoop ; pLoop = pLoop->next )
	{
		if( fabs( pLoop->m_dArea ) > fabs(pObj->m_dArea) )
		{
			pObj = pLoop ;
		}
	}
	pCopy = Mini_DelLoop( pCopy, pObj ) ;
	pCopy = loop.BuildContour ( pCopy ) ;
	
	return pCopy ;
}
/////////////////////////////////////////////////////////
// for test
// 
/////////////////////////////////////////////////////////
int CSmartSRemainGen::SortLoopByArea ( CPathGroup &NewPath, int nLayer, JDNC_PRGDEF &ProgDef ) 
{	
	CSmartLoop *pLoop, *pTmp, *pHead, LoopLib ;
	pHead = pLoop = pTmp = NULL ;
	INT_PTR i, Layer, nSize = 0 ;
	CSmtLoopPath *pLPath = NULL ;
	for( Layer = 0 ; Layer < nLayer ; Layer++ )
	{
		nSize = m_pLPathArr[Layer].GetSize () ;
		for( i = 0 ; i < nSize; i++ )
		{
			pLPath = m_pLPathArr[Layer].GetAt(i)  ;
			for( pLoop = pLPath->m_pLoop ; pLoop ; pLoop = pLoop->next )
			{
                pLoop->m_nDepth = int( Layer ) ;
			}
			if( pLPath->m_pLoop ) 
				pHead = Mini_AddContours( pHead, pLPath->m_pLoop ) ;
		}
	}
	// 将得到的环链进行排序
	pHead = Mini_SortSRemainContours( pHead, 
                                      GetLoopCheckTol(),
                                      m_pTool->m_fRadius * 2.0 ) ;
	BOOL bBreak = FALSE ;
	for( pLoop = pHead ; pLoop ; pLoop = pLoop->next )
	{
		Layer = pLoop->m_nDepth ;
		
		nSize = m_pLPathArr[Layer].GetSize () ;
		pLPath = NULL ;
		for( i = 0 ; i < nSize ; i++ )
		{
			pLPath = m_pLPathArr[Layer].GetAt ( i ) ;
			if( pLoop == pLPath->m_pLoop )
				break ;
		}
		if( !pLPath || !pLPath->m_pLoop ) continue ;
		// 将路径加到路径组中
		AddLoopToGroup( pLPath->m_pLoop, m_dZ[Layer], NewPath ) ;
		pLPath->m_cPathLib.ClearAllPath () ;
		pLPath->m_pLoop = NULL ;
		// 显示进度条
		if( ProgDef.m_pBrkFunc && ProgDef.m_pBrkFunc () )
		{
			bBreak = TRUE ;
			break ;
		}
		ProgDef.m_dStepAt += ProgDef.m_dIncStep ;
		while( ProgDef.m_pPrgFunc && ProgDef.m_dStepAt >= ProgDef.m_dLimitAt )
		{
			ProgDef.m_pPrgFunc(1) ;
			ProgDef.m_dStepAt -= ProgDef.m_dLimitAt  ;
		}
	}
	// 清空所有Loop
	for( Layer = 0 ; Layer < nLayer ; Layer++ )
	{
		nSize = m_pLPathArr[Layer].GetSize () ;
		for( i = 0 ; i < nSize; i++ )
		{
			pLPath = m_pLPathArr[Layer].GetAt(i)  ;
			pLPath->m_pLoop = NULL ;
		}
	}
	if( pHead ) Mini_DeleteContours( pHead ) ;
	if( bBreak ) 
		return 0 ;
	return 1 ;
}
///////////////////////用于路径连接//////////////////////////////
void CSmartSRemainGen::AddTestPathToAllPath( CSmtCPathLib &AllPath, CSmtCPathLib &cPathLib ) 
{
	INT_PTR nCnt = cPathLib.m_cAllPath.GetCount () ;
	if( nCnt < 1 ) return ;
	CSmtCutPath *pPath = NULL ;
	// 如果NewPath中空,则取数组中第一个添加到newPath中
	if( AllPath.m_cAllPath.GetCount () < 1 )
	{
		pPath = cPathLib.m_cAllPath.RemoveHead () ;
		AllPath.AddToTail ( pPath ) ;
	}
	// 然后依次连接路径
	
	POSITION pos, atpos ;
	pos = cPathLib.m_cAllPath.GetHeadPosition () ;
	while( pos )
	{
		atpos = pos ;
		pPath = cPathLib.m_cAllPath.GetNext ( pos ) ;
		cPathLib.m_cAllPath.RemoveAt ( atpos ) ;
		AllPath.AddToTail ( pPath ) ;
	}
	cPathLib.m_cAllPath.RemoveAll () ;
}
#ifdef _NEW_REMAIN_MDL
int CSmartSRemainGen::DebugRemainMdl( CPathGroup& NewPath     ,  /*刀具路径*/
						              CSmartGraphic& Graph )  /*雕刻图形*/
{
    // STEP 1 : 准备加工环境 
	if( !UpdateTool3D(Graph) ) return 0 ;
	// 构建上把刀具的加工模型 刀具的stock是用这次图形的还是上次图形
	double stepstock = m_cParam.m_cRemain.m_cLastShape.m_cStock0.m_dDepthStock ;
	m_pLastTool = CreateSmartTool( m_cParam.m_cRemain.m_cLastTool, stepstock, 0. ) ;
    BOOL bRetFlag = TRUE ;
    CSmtCheckMdl DriveMdl, CheckMdl ;
	if( m_cParam.m_cRemain.m_nRemainType != NCDEF_REMAIN_LASTOPER )
	{
        BuildDriveAndCheckMdl( Graph, DriveMdl, CheckMdl ) ;
        if( SurfNC_IsAbort() )
        {
            return FALSE ;
        }
    }
	CSmtCheckMdl StockMdl ;
	BOX3D box ;
	if( !BuildStockMdl( Graph, StockMdl, box ) )
	{
		m_nErrorType = JDERROR_GENPATH_NOSTOCK ;
		return FALSE ;
	}

    BuildRemainZMap( Graph , &DriveMdl, &StockMdl, m_cPrgDef, 50 ) ;
    if( m_pRemainMdl && m_pRemainMdl->m_bDumpXYPath )
    {/*调试网格曲线*/
        m_pRemainMdl->DumpMeshToPathGroup( NewPath ) ;
    }
    else  if( m_pRemainMdl )
    {/*调试等高轮廓*/
	    double top = m_dTopHeight ;
	    double bot = m_dBotHeight ;
        double dInc = m_cFeedDef.m_cLayerDef.m_dSideDInc ;
        if( dInc < 0.05 ) dInc = 0.05 ;
        CSmtCPathLib AllPath ;
        CPathCombine * pPComb = new CPathCombine( NC_WPROCESS_ROUGH ) ;
        if( 0 )
        {
            top = -9.264;
            CSmartLoop* pLpHead = m_pRemainMdl->GetContoursAtZLevel( (TFLOAT) top ) ;
            for( CSmartLoop* pCont = pLpHead ; pCont ; pCont = pCont->next )
            {
                pPComb->AddCurve( pCont->m_pCurve, FALSE , (TFLOAT) top ) ;
                for( CSmartLoop* pIsl = pCont->m_pIsland ; pIsl ; pIsl = pIsl->next )
                {
                    pPComb->AddCurve( pIsl->m_pCurve, FALSE , (TFLOAT) top ) ;
                }
            }
            Mini_DeleteContours( pLpHead ) ;
        }
        else for( int i = 0 ; i < 1000 ; i ++ ) 
        {
            if( top <= bot ) break ;
            top -= dInc ;
            m_pRemainMdl->GetBoundCurveAtZValue( (TFLOAT) top, AllPath ) ;
            AllPath.AddToPathCombine( * pPComb   ) ;
            AllPath.ClearAllPath() ;
        }
        if( pPComb->m_pHead == NULL ) 
        {
            delete pPComb ;
        }
        else
        {
            NewPath.AddData ( 0., pPComb ) ;
        }
    }
    if( m_pRemainMdl )
    {
        delete m_pRemainMdl ;
        m_pRemainMdl = NULL  ;
    }
    return bRetFlag ;
}
#endif 

int CSmartSRemainGen::FilterAllPlaneBnd( CSmartLoop *&PlaneHead ) 
{
	if( !PlaneHead ) return FALSE ;
   // 计算删除距离
	double dMaxZ = m_cSetupDef.m_cTolDef.m_dArcTol * 3 ;
	if( fabs( m_cStockDef.m_dDriveOffset[0] ) < 0.0001 )
		dMaxZ = m_cSetupDef.m_cTolDef.m_dArcTol ;
	else
        dMaxZ = min( m_dStepZ * 0.2, dMaxZ ) ;
	
    // 对得到的环进行过滤,删除与分层相近的环
	double dTmpH = 0., dMinZ = 0. ;
	INT_PTR i = 0 ;
	BOOL bDelete = FALSE ;
	double dTol = m_pTool->m_fRadius * 0.1, dRatio = 0.4 ;
	CSmartLoop *pHead = PlaneHead , *pNext = NULL ;
	while( pHead )
	{
		pNext = pHead->next ;

		dMinZ = m_dStepZ ;

		pHead->DefineBox () ;
		// 如果该环的深度和等高的分层很近,并在分层下方,则该环可以去掉
		for( i = 0 ; i < m_nCount ; i++ )
		{
			dTmpH = m_dZ[i] - pHead->m_dHeight ;
			if( dTmpH < dMinZ && dTmpH > 0 )
			{
				dMinZ = dTmpH ;
			}
			if( dMinZ < dMaxZ ) break ;
			// 然后处理极值情况,当两层距离非常之小,是否也应该删除?
			if( fabs( dTmpH ) < 0.0005 )
			{
				dMinZ = dTmpH ;
				break ;
			}
		}
		// 对找到的最近距离进行处理
		if( dMinZ < dMaxZ )
		{ // 当两个距离很近时
			bDelete = TRUE ;
			if( i == m_nCount - 1 && 
				IsBottomPlaneValid( m_pDriveLoop[i], pHead, dTol, dRatio ) )
			{// 防止表面余量为0且底面为平面时dMInZ为-0.0003,删除底面平面,而底面路径被边界减掉
				bDelete = FALSE ;
			}
			if( bDelete )
			{
				if( pHead->prev )
					pHead->prev->next = pHead->next ;
				if( pHead->next )
					pHead->next->prev = pHead->prev ;
				if( pHead == PlaneHead ) 
					PlaneHead = PlaneHead->next ;
				pHead->next = pHead->prev = NULL ;
				Mini_DeleteContours( pHead ) ;
			}
		}
		pHead = pNext ;
	}
//////////////////////////////////////for test/////////////////////////////////
//	pos = cList.GetHeadPosition () ;
//	while( pos )
//	{
//		atpos = pos ;
//		pHead = cList.GetNext ( pos ) ;
///		if( fabs( pHead->m_dHeight + 30.845 ) > 0.1 )
//		{
//			delete pHead ;
//			cList.RemoveAt ( atpos ) ;
//		}
//	}
//////////////////////////////////////for test/////////////////////////////////
	return TRUE ;
}
void CSmartSRemainGen::TrimPlaneHeadByCombineLoop ( CPlaneList &cList, BOX3D &MaxBox, int nType )
{
	CSmartLoop LoopLib, *pBigRemain = NULL, *pBigRough = NULL ;
	// 将残补环转成残补区域
	pBigRemain = CreateBigLoopAndBox( m_pRemainLoop, m_pAllLoop, MaxBox, m_nCount, m_pTool->m_fRadius * 3 ) ;
	if( m_pRoughLoop )
        pBigRough  = CreateBigLoopAndBox( m_pRoughLoop,  m_pAllLoop, MaxBox, m_nCount, m_pTool->m_fRadius * 2.5 ) ;
	CPlaneLoop *PlaneLoop = NULL, *pNext = NULL ;
	CSmartLoop *pRemain = NULL, *pRough = NULL, *pCopy = NULL, *pResult = NULL, *BndHead = NULL ;
	double tmpH = 0.;
	POSITION pos, atpos ;
	pos = cList.GetHeadPosition () ;
	while( pos )
	{
		atpos = pos ;
		pRemain = pRough = BndHead = NULL ;
		PlaneLoop = cList.GetNext ( pos ) ;
		SetContourBlank( PlaneLoop->m_pRemainLoop ) ;
		if( PlaneLoop->m_pRemainLoop )
		{
			pRemain = Mini_CopyContours( PlaneLoop->m_pRemainLoop ) ;
			pCopy   = Mini_CopyContours( pBigRemain ) ;
			pRemain = Mini_AddContours ( pRemain, pCopy ) ;
			pRemain = LoopLib.BuildContour ( pRemain ) ;
		}
		if( PlaneLoop->m_pRoughLoop && nType > 1 )
		{
			pRough = Mini_CopyContours( PlaneLoop->m_pRoughLoop ) ;
			pCopy  = Mini_CopyContours( pBigRough ) ;
			pRough = Mini_AddContours ( pRough, pCopy ) ;
			pRough = LoopLib.BuildContour ( pRough ) ;
		}
		if( pRough && pRemain )
		{
			pResult = Mini_OperateContours( pRough, pRemain, 1 ) ;
			if( !pResult ) pResult = Mini_CopyContours( pRemain ) ;
			Mini_DeleteContours( pRemain ) ;
			Mini_DeleteContours( pRough  ) ;
		}
		else if( pRemain )
		{
			pResult = pRemain ;
		}
		else
		{
			pResult = pRough ;
		}
		// 将得到的环和平面环求减,然后再和边界环求交
		BndHead = Mini_OperateContours( PlaneLoop->m_pPlaneLoop, pResult, 2 ) ;
		Mini_DeleteContours( pResult ) ;
		Mini_DeleteContours( PlaneLoop->m_pPlaneLoop ) ;
		if( BndHead )
		{
			BOOL bFirst = TRUE ;
			tmpH = PlaneLoop->m_dHeight ;
			CSmartLoop * BndTail = NULL ;
			while ( BndHead )
			{
				BndTail = BndHead->next ;
				BndHead->next = BndHead->prev = NULL ;
				BndHead->m_dHeight = tmpH ;
				BndHead->DefineBox() ;
				if ( bFirst )
				{
					PlaneLoop->SetPlaneLoop( BndHead ) ;
					bFirst = FALSE ;
				} 
				else
				{
					pNext = new CPlaneLoop( BndHead, Mini_CopyContours( PlaneLoop->m_pOrgLoop) ) ;
					pNext->SetRoughLoop( Mini_CopyContours( PlaneLoop->m_pRoughLoop ) ) ;
					pNext->SetRemainLoop( Mini_CopyContours( PlaneLoop->m_pRemainLoop) ) ;
					cList.InsertBefore( atpos, pNext ) ;
				}
				BndHead = BndTail ;
			}
		}
		else
		{
			cList.RemoveAt ( atpos ) ;
			delete PlaneLoop ;
		}
	}
	Mini_DeleteContours( pBigRemain ) ;
	Mini_DeleteContours( pBigRough  ) ;
}

void CSmartSRemainGen::FilterAllPlaneBlindArea( CPlaneList &cList )
{
	CPlaneLoop *pPlane = NULL ;
	// 删除狭长的环和面积小于0.03的环
	POSITION pos, atpos = NULL ;
	pos = cList.GetHeadPosition () ;
	while( pos )
	{
		atpos = pos ;
		pPlane = cList.GetNext ( pos ) ;
		if( IsBlindContour( pPlane->m_pPlaneLoop, m_cFeedDef.m_cPlungeDef.m_dBlindRadius ) )
		{
			delete pPlane ;
			cList.RemoveAt ( atpos ) ;
		}
	}
}

void CSmartSRemainGen::SearchRationalPosition ( CPlaneList &cList )
{
	double dTol = m_pTool->m_fRadius * 2 ;
	INT_PTR i, j, nSize ;
	CSmartLoop *pCurrLoop = NULL ;
	CSmtLoopPath *pLPath = NULL ; 
	CPlaneLoop *pPlane = NULL ;
	// 删除狭长的环和面积小于0.03的环
	POSITION pos, atpos ;
	pos = cList.GetHeadPosition () ;
	while( pos )
	{
		atpos = pos ;
		pPlane = cList.GetNext ( pos ) ;
		if( !IsRemainLoopValid( pPlane->m_pPlaneLoop, m_cSetupDef.m_cTolDef, 0. ) ||
			fabs( pPlane->m_pPlaneLoop->m_dArea ) < 0. )
		{
			delete pPlane ;
			cList.RemoveAt ( atpos ) ;
		}
	}
	//第一次查找，找下一层相关区域
	pos = cList.GetHeadPosition () ;
	while( pos )
	{
		pPlane = cList.GetNext ( pos ) ;
		for( i = 0 ; i < m_nCount ; i++ )
		{
			if( m_dZ[i] < pPlane->m_dHeight ) break ;
		}
		if( i == m_nCount ) continue ;
		nSize = m_pLPathArr[i].GetSize () ;
		for( j = 0 ; j < nSize ; j++ )
		{
			pLPath = m_pLPathArr[i].GetAt ( j ) ;		  
			pCurrLoop = pLPath->m_pLoop ;
			if ( Is2ContoursInt( pPlane->m_pPlaneLoop, pCurrLoop, dTol ) )
			{
				pPlane->AddPreLoop( pCurrLoop ) ;
			}
		}
	}
	//第二次查找,找上一层.(处理凹槽底平面)
	pos = cList.GetHeadPosition() ;
	while ( pos )
	{
		pPlane = cList.GetNext( pos ) ;
		if ( pPlane->m_PreLoopArr.GetSize() != 0 ) continue ;
		for ( i = 0; i < m_nCount; i++ )
		{
			if ( m_dZ[i] < pPlane->m_dHeight ) break ;
		}
		if( i == m_nCount || i ==0 ) continue ;
		i -- ;
		if( i < 0 ) continue ;
		nSize = m_pLPathArr[i].GetSize() ;
		for ( j = 0; j < nSize ; j++ )
		{
			pLPath = m_pLPathArr[i].GetAt ( j ) ;
			pCurrLoop = pLPath->m_pLoop ;
			if ( IsInSameArea( pPlane->m_pPlaneLoop, pCurrLoop, dTol ) )
			{
				pPlane->AddPreLoop( pCurrLoop ) ;
			}
		}
	}	
}
void CSmartSRemainGen::CreatePlanePath ( CSmtCheckMdl *DriveMdl, CSmtCPathLib &AllPath, CSmartLoop *CurrLoop, int nLayer, int nType )
{
	if( AllPath.m_cAllPath.GetCount () < 1 ) return ;
	CPlaneLoop *pPlaneLoop = NULL, *pFindPlane = NULL ;
	POSITION pos, atpos, atDel = NULL ;
	double dLastH = 0. ;
	// 首先得到所有的平面环
	CPlaneList tmpList ;
	pos = m_cPlaneList.GetHeadPosition () ;
	while( pos )
	{
		atpos = pos ;
		pPlaneLoop = m_cPlaneList.GetNext ( pos ) ;
		if( IsCutPlaneLoop( pPlaneLoop, CurrLoop, nLayer, nType ) )
		{
			tmpList.AddTail( pPlaneLoop ) ;
			m_cPlaneList.RemoveAt( atpos ) ;
		}
	}
	// 然后加工tmpList的平面环
	PNT3D p ;
	double dDist = 0., dMin = 1.0e6;
	while( 1 )
	{
		dMin = 1.0e6 ;
		pFindPlane = NULL ;
		atDel = NULL ;
		GetAllPathEndPoint( AllPath, p ) ;
		pos = tmpList.GetHeadPosition () ;
		while( pos )
		{
			atpos = pos ;
			pPlaneLoop = tmpList.GetNext ( pos ) ;
			dDist = pPlaneLoop->m_pPlaneLoop->MinDistPoint ( p ) ;
			if( dDist < dMin )
			{
				dMin = dDist ;
				pFindPlane = pPlaneLoop ;
				atDel = atpos ;
			}
		}
		if( !pFindPlane || !atDel ) break ;
		tmpList.RemoveAt ( atDel ) ;
		// 高度过滤
		for( int i = 0 ; i < m_nCount ; i++ )
		{
			if( m_dZ[i] < pFindPlane->m_dHeight )
				break ;
		}
		if( i == 0 ) dLastH = m_dTopHeight ;
		else		 dLastH = m_dZ[i-1] ;
		GenPlaneAreaPath( DriveMdl, AllPath, pFindPlane, dLastH, pFindPlane->m_dHeight ) ;
		delete pFindPlane ;
	}
	tmpList.RemoveAll () ;
}

void CSmartSRemainGen::CreateRemainPlanePath ( CSmtCheckMdl *DriveMdl, CSmtCPathLib &AllPath )
{
	double dTmpH = 0, dMinZ = 1., dLastH = 0. ;
	INT_PTR i = 0, nFind = -1 ;
	POSITION pos, atpos ;
	CPlaneLoop *pPlaneLoop = NULL ;
	pos = m_cPlaneList.GetHeadPosition () ;
	while( pos )
	{
		atpos = pos ;
		pPlaneLoop = m_cPlaneList.GetNext ( pos ) ;
		nFind = -1 ;
		dMinZ = m_dTopHeight - pPlaneLoop->m_dHeight  ;
		// 如果该环的深度和等高的深度很近,则该环可以去掉
		for( i = 0 ; i < m_nCount ; i++ )
		{
			dTmpH = m_dZ[i] - pPlaneLoop->m_dHeight ;
			if( dTmpH < dMinZ && dTmpH > 0 )
			{
				dMinZ = dTmpH ;
				nFind = i ;
			}
		}
		if( nFind == -1 )
		{
			dLastH = m_dTopHeight ;
		}
		else
		{
			// 对得到的环生成路径
			if( m_dZ[nFind] > pPlaneLoop->m_dHeight )
			{
				dLastH = m_dZ[nFind] ;
			}
			else
			{
				if( nFind == 0 ) dLastH = dLastH = m_dTopHeight ;
				else			 dLastH = m_dZ[nFind-1] ;
			}
		}
		// 生成路径
		GenPlaneAreaPath( DriveMdl, AllPath, pPlaneLoop, dLastH, pPlaneLoop->m_dHeight ) ;
		// 删除环
		delete pPlaneLoop ;
		m_cPlaneList.RemoveAt ( atpos ) ;
	}
}
void CSmartSRemainGen::GenPlaneAreaPath ( CSmtCheckMdl *DriveMdl, CSmtCPathLib &AllPath, CPlaneLoop *pPlane, double dLastH, double dDepth )
{
	if( !pPlane || !pPlane->m_pPlaneLoop ) return  ;
	CPathCombine *PComb = new CPathCombine( NC_WPROCESS_ROUGH ) ;
	double delH = dLastH - dDepth ;
	JDNC_FOLLOW  FollowCut ;
	FollowCut.m_dOverStep  = m_cFeedDef.m_cStepDef.m_dOverStep ;
	FollowCut.m_dOverRatio = m_cFeedDef.m_cStepDef.m_dOverRatio ;
	FollowCut.m_nFollowType= 0 ; 
	FollowCut.m_bFollowFlag = NCDEF_FOLLOW_MINUP ;		// 最少抬刀
	if (pPlane->m_pPlaneLoop->m_pCurve->IsClosed())
	{// edit by liuxin 2013.7.17, 封闭路径从内向外走刀
		FollowCut.m_bFollowFlag |= NCDEF_FOLLOW_TOOUTER;
	}
	if( m_cFeedDef.m_cSlotDef.m_nSlotType != NCDEF_TSLOT_CLOSE ) 
		FollowCut.m_bFollowFlag |= NCDEF_FOLLOW_SLOTCUT ;
	
	// 环切清角并光滑圆角
	FollowCut.m_bFollowFlag |= NCDEF_FOLLOW_CORNER ;
	FollowCut.m_bFollowFlag |= NCDEF_FOLLOW_SMOOTH ;
		
	if( m_cFeedDef.m_cSlotDef.m_nSlotType != NCDEF_TSLOT_CLOSE ) 
	{ // 计算开槽路径
		PComb->m_nType = NC_WPROCESS_SLOT  ;
		m_cFeedDef.m_cSlotDef.m_dDepthInc =  delH ;
		DefineSlotDepth( m_cFeedDef.m_cSlotDef ) ;
		CreateECutPath( *PComb  , 
						*pPlane->m_pPlaneLoop , 
						m_cFeedDef.m_cSlotDef , 
						m_cFeedDef.m_cPlungeDef ) ;
	}
	PComb->m_nType = NC_WPROCESS_ROUGH  ;

	// STEP 2 : 环切路径
	// 防止环切时尖角处出现过切，2014.6.30 liuxin
	JDNC_SETUP cSetupDef = m_cSetupDef;
	if (FollowCut.m_bFollowFlag & NCDEF_FOLLOW_SMOOTH)
	{
		cSetupDef.m_cCorDef.m_nCorType = NCDEF_CORNER_ARCUSER;
	}
	CSmartNcFollow FollowGen( m_pTool, FollowCut, m_cFeedDef, /*m_cSetupDef*/cSetupDef ) ;
	// 精修边界
	if( m_cParam.m_bSRemainFlag & NCDEF_SREMAIN_BOUNDEND )
        FollowGen.SetBoundtol ( m_cParam.m_dBoundTol, m_cParam.m_dFeedRatio ) ;
	FollowGen.CreatePocketPath ( *pPlane->m_pPlaneLoop, *PComb ) ;
	
	// STEP 3 : 计算下刀路径
	CSmartPocketGen  PocketGen ;
	(*(CSmartPathGen*)&PocketGen ) = * this ;
    PocketGen.m_cSpeedDef.m_dPlungeDist = 0.0;
	JDNC_PLUNGE  Plunge = m_cFeedDef.m_cPlungeDef ;
	DOUBLE dMaxDist[2] = { max( 6.0, 1.5 * FollowCut.m_dOverStep * 1.5 ), 0.3 } ;
	if( Plunge.m_nPlungeType != NCDEF_PLUNGE_CLOSE )
	{
		Plunge.m_dIncStep = delH  + Plunge.m_dTopTol ;
		PComb->m_nType = NC_WPROCESS_PLUNGE ;
        m_bFollowOuter = 0 ;
		int nFlag[2] = { surfncMoveFollow, m_cSetupDef.m_cOrderDef.m_nMillDir } ;
		PocketGen.InsertPlungePath( *PComb, *pPlane->m_pPlaneLoop, Plunge, pPlane->m_pOrgLoop, pPlane->m_pRemainLoop, dMaxDist, nFlag ) ;
        m_bFollowOuter = 1 ;
	}
	// 减少抬刀
	if( ( m_cFeedDef.m_cLayerDef.m_bLayerFlag & NCDEF_LAYER_KEEPDOWN ) ) 
	{
		// 对路径进行连接
		CRoughLink cLink( dDepth ) ;
		cLink.AddLinkPath ( DriveMdl, PComb, m_cSetupDef.m_cTolDef, dLastH ) ;
	}

	TransfPCombineToCPath( *PComb, AllPath, m_cSpeedDef.m_dFeedRate, dDepth, m_cSetupDef.m_cTolDef ) ;
	delete PComb ;

	//for test　测试环的正确性
/*	AddSmtLoopInPGroup( BndHead, dDepth,  PGroup ) ;*/
}

BOOL CSmartSRemainGen::IsCutPlaneLoop(CPlaneLoop* & PlaneLoop, CSmartLoop * CurrLoop, int nLayer, int nType)
{
	if( !PlaneLoop ) return FALSE ;
	if ( nType == NCDEF_RPOCKET_CAST )
	{	
		if( !CurrLoop ) return FALSE ;
		INT_PTR i, nSize = PlaneLoop->m_PreLoopArr.GetSize() ;
		for( i=0; i<nSize; i++ )
		{
			CSmartLoop * pPreLoop = PlaneLoop->m_PreLoopArr.GetAt(i) ;
			if( pPreLoop == CurrLoop )
			{
				PlaneLoop->m_PreLoopArr.RemoveAt(i) ;
				break ;
			}
		}
		nSize = PlaneLoop->m_PreLoopArr.GetSize() ;
		if( nSize != 0 )  return FALSE ;
		if( PlaneLoop->m_bSearch3 ) return TRUE ;
		/*以下消除安全隐患，详细参见总结*/
		PlaneLoop->m_bSearch3 = TRUE ; //设置第三次查找标记
		for ( i = 0; i < m_nCount; i++ )
		{//找到平面环所在的夹层
			if( m_dZ[i] < PlaneLoop->m_dHeight ) break ;
		}
		if( i == 0 ) return TRUE ;  //过滤平面高度大于表面高度
		if( i == m_nCount ) return FALSE ;
		double dTol = m_pTool->m_fRadius * 2 ;
		for( CurrLoop = CurrLoop->next; CurrLoop; CurrLoop = CurrLoop->next )
		{
			if( CurrLoop->m_nDepth > i ) continue ;
			if ( IsInSameArea( PlaneLoop->m_pPlaneLoop, CurrLoop, dTol ) )
			{
				PlaneLoop->AddPreLoop( CurrLoop ) ;
			}
		}
		if ( PlaneLoop->m_PreLoopArr.GetSize() == 0 )
		{
			return TRUE ;
		} 
		else
		{
			return FALSE ;
		}
	} 
	else if( nType == NCDEF_RPOCKET_DEPTH )
	{
		double  dTop = 0, dBot = m_dZ[nLayer] ; 
		// 高度过滤
		if( nLayer == 0 ) dTop = m_dTopHeight ;
		else			  dTop = m_dZ[nLayer-1] ;
		if( PlaneLoop->m_dHeight > dTop || PlaneLoop->m_dHeight < dBot )
		{
			return FALSE ;
		}
		else
		{
			return TRUE ;
		}
	}
	else 
	{
		return FALSE ;
	}
}
#include "StdAfx.H"
#include "SurfGeo.H"
#include "SmartNC.H"
#include "SmtPathGen.h"
#include "SmtPathGen2D.H"
#include "SmtPathGen3D.H"
#include "SmartPathEx.H"
#include "NcHeightMdl.H"
#include "mathcam.h"
#include "RectNetRoughCounter.h"
#include "GP_Math.h"

#include "ProductDef.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

CSmartRPocketGen::CSmartRPocketGen()
{
	m_pDriveLoop = NULL ;
	m_pRoughLoop = NULL ;
	m_pBtwPlgLayer = NULL ;
	m_pRoughCastLoopArr = NULL ;
	m_dZ         = NULL ;
	m_bLayer	 = NULL ;
	m_pAllLoop   = NULL ;
	m_dMaxZ		 = 1    ;
	m_bFirst	 = TRUE ;
	m_pRoughTool = NULL ;
	m_bAllLoop	 = FALSE ;
	m_pBtwLPathArr = NULL ;
	m_pCurveArr  = NULL ;
	m_pBtwCurveArr = NULL ;
	m_pLPathArr  = NULL ;
	m_pPlunge	 = NULL ;
	m_pCurveLib  = NULL ;
	m_bBndTrim   = FALSE ;
	m_pPlaneLoop = NULL ;
	m_dMinArea	 = 0.   ;

	//guomin
	m_pRectNetSurf = NULL ;
}
CSmartRPocketGen::~CSmartRPocketGen()
{
	ClearAllLoop() ;
}
void ClearAllCurveLib ( CSmartCurveLib *&pCurveLib, int nCount )
{
	if( !pCurveLib ) return ;
	
	for( int i = 0 ; i < nCount ; i++ )
	{
		pCurveLib[i].DeleteAllCurves () ;
	}
	delete[] pCurveLib ;
	
	pCurveLib = NULL ;
}
void CSmartRPocketGen::ClearAllLoop ( )
{
	// 清除层间信息
	ClearBtwLoop() ;
	// 清除高度
	if( m_dZ ) 	delete[] m_dZ ;
	m_dZ = NULL ;
	if( m_bLayer ) delete[] m_bLayer ;
	m_bLayer = NULL ;
	if( m_pRoughTool ) delete m_pRoughTool ;
	m_pRoughTool = NULL ;
	// 清除边界环
	if( m_pAllLoop ) Mini_DeleteContours( m_pAllLoop ) ;
	// 清除没有加工的环
	ClearAllPlaneLoop( &m_cPlaneList ) ;
	ClearAllLoopHead( m_pDriveLoop, m_nCount ) ;
	ClearAllLoopArr( m_pRoughCastLoopArr, m_nCount ) ;
	ClearAllLoopHead( m_pRoughLoop, m_nCount ) ;

	ClearAllLPathArr( m_pLPathArr, m_nCount ) ;
	ClearAllLoopHead( m_pPlunge, m_nCount ) ;
	ClearAllCurveLib( m_pCurveLib, int( m_nCount ) ) ;
	//清除平面环
	if( m_pPlaneLoop ) Mini_DeleteContours( m_pPlaneLoop ) ;
	if ( m_pBtwPlgLayer )
	{
		delete m_pBtwPlgLayer ;
		m_pBtwPlgLayer = NULL ;
	}
}
void CSmartRPocketGen::InitTmpProg ()
{
	m_cTmpPrgDef.m_dLimitAt = 0. ;
	m_cTmpPrgDef.m_dStepAt  = 0 ;
	m_cTmpPrgDef.m_dIncStep = 1. ;
	m_cTmpPrgDef.m_pBrkFunc = NULL ;
	m_cTmpPrgDef.m_pPrgFunc = NULL ;
	m_cTmpPrgDef.m_pPosFunc = NULL ;
}
void CSmartRPocketGen::CalcTopAndBot ( BOX3D &box )
{
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
}
int CSmartRPocketGen::GetECutLayers ( CSmartLoop *& PlaneLoop )
{
	double top = m_dTopHeight ;
	double bot = m_dBotHeight ;
	// 避免加工到平面，表面余量+0.001
	double dCutDepth = top - bot, dStepZ = 0. ;
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
		// 按照层高分层
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
			if( m_nCount < 1 ) 	m_nCount = 1 ;
		}
	}
	m_dStepZ = dCutDepth / m_nCount ;
	// 均匀分层,也可以按照cimatron从高到低分层
	m_dZ = new double[m_nCount] ;
	for( int i = 0 ; i < m_nCount ; i++ )
	{
		m_dZ[i] = top - ( i+1 ) * m_dStepZ ;
	}
EndLine:
	BOOL bBtwRoughPlane = m_cParam.m_cBtwRecut.m_nLayerType != NCDEF_BTWLAYER_CLOSE 
					 && m_cParam.m_cBtwRecut.m_nLayerType == NCDEF_BTWLAYER_ROUGH 
				     && m_cParam.m_bRPocketFlag & NCDEF_RPOCKET_CUTPLANE ;
	if( m_cParam.m_cBtwRecut.m_nLayerType != NCDEF_BTWLAYER_CLOSE )
	{
		if( !bBtwRoughPlane )
			SetCutStepBtwLayer() ;
		else
			SetCutStepBtwLayerRough( PlaneLoop ) ;
	}
	else
	{
		m_bLayer = new BOOL[m_nCount] ;
		for ( int i = 0; i < m_nCount; i++ )
		{
			m_bLayer[i] = TRUE ;
		}
	}

	return 1 ;
}
void CSmartRPocketGen::SetCutStepBtwLayer()
{
	int i = 0, n = 0, j = 0, nLayer = m_nCount ;
	double dVStep = 0, dTop = m_dTopHeight ;
	for( i = 0 ; i < m_nCount ; i++ )
	{
		if( i == 0 )
			dVStep = dTop - m_dZ[0] ;
		else
			dVStep = m_dZ[i-1] - m_dZ[i] ;
		if( dVStep <= m_cParam.m_cBtwRecut.m_dMaxVertStep )
			continue ;
		n = ( int ) ceil ( dVStep / m_cParam.m_cBtwRecut.m_dMaxVertStep ) ;
		nLayer += n - 1 ;
	}
	double *dZ = new double[nLayer + 10] ;
	m_bLayer = new BOOL[nLayer+5] ;
	memset( m_bLayer, 0, (nLayer + 5) * sizeof(BOOL) ) ;
	nLayer = -1 ;
	for( i = 0 ; i < m_nCount ; i++ )
	{
		if( i == 0 )
			dVStep = dTop - m_dZ[0] ;
		else
			dVStep = m_dZ[i-1] - m_dZ[i] ;
		if( dVStep <= m_cParam.m_cBtwRecut.m_dMaxVertStep )
		{
			nLayer++ ;
			dZ[nLayer] = m_dZ[i] ;
			m_bLayer[nLayer] = TRUE ;
		}
		else
		{
			double dCnt = dVStep / m_cParam.m_cBtwRecut.m_dMaxVertStep ;
			double dTol = dCnt - int( dCnt ) ;
			if( dTol < 0.05 ) n = int ( dCnt ) ;
			else n = int( dCnt + 1 ) ;

			double dTmp = dVStep / n ;
			for( j = 1 ; j <= n ; j++ )
			{
				nLayer++ ;
				if( i == 0 )
					dZ[nLayer] = dTop - dTmp * j ;
				else
					dZ[nLayer] = m_dZ[i-1] - dTmp * j ;
			}

/*			dTmp = 0. ;
			n = 1 ;
			while( dVStep > m_cParam.m_cBtwRecut.m_dMaxVertStep )
			{
				nLayer++ ;
				dTmp = n * m_cParam.m_cBtwRecut.m_dMaxVertStep ;
				dVStep -= m_cParam.m_cBtwRecut.m_dMaxVertStep  ;
				if( i == 0 )
					dZ[nLayer] = dTop - dTmp ;
				else
					dZ[nLayer] = m_dZ[i-1] - dTmp ;
				n++ ;
			}
			if( dVStep > 0.001 )
			{
				nLayer++ ;
				dZ[nLayer] = m_dZ[i] ;
			}*/
			m_bLayer[nLayer] = TRUE ;
		}
	}
	// 重新得到分层
	delete[] m_dZ ;
	m_dZ = dZ ;
	m_nCount = nLayer + 1 ;
}
int CSmartRPocketGen::FindPrevLayer ( int nLayer ) 
{
	for( int i = nLayer - 1 ; i >= 0 ; i-- )
	{
		if( m_bLayer[i] )
			return i ;
	}
	return -1 ;
}
int CSmartRPocketGen::FindNextLayer ( int nLayer )
{
	for( int i = nLayer + 1 ; i < m_nCount ; i++ )
	{
		if( m_bLayer[i] )
			return i ;
	}
	return m_nCount - 1 ;
}
BOOL CSmartRPocketGen::BuildStockMdl ( CSmartGraphic &Graph, CSmtCheckMdl &RoughMdl, BOX3D &box )
{
	////////////////////处理毛坯情况////////////////
	// 首先生产加工面模型
	if( !Graph.BuildStockModel ( m_cSetupDef, RoughMdl, box ) )
	{
		return FALSE ;
	}
	int nRoughType = Graph.GetRoughType () ;
	double dStock = Graph.GetRoughStock () ;
	if( nRoughType == ncStockShapeSurf && fabs( dStock ) > 0.0001 )
	{
		m_pRoughTool = CreateSmartTool( m_cToolDef, dStock, 0 ) ;
		RoughMdl.ZMoveModel ( dStock ) ;
	}
	else
	{
		m_pRoughTool = CreateSmartTool( m_cToolDef, 0., 0 ) ;
	}
	RoughMdl.UpdateCurrTool( m_pRoughTool )  ;
	return TRUE ;
}

int CSmartRPocketGen::LinearOneLayer( CSmtCheckMdl *DriveMdl, 
									  CSmartLoop** pLoops,     //粗加工环
									  CSmartLoop *pBigLoop,	   //大环
				                      int  Layer ,         //分层序号
									  BOOL bIsRoughCast )	   //是否是毛坯面的环			                
{
	// 得到等高线 ,并得到环
    CSmartLoop * BndHead = NULL , *BndTail = NULL, looplib ;
	BndHead = BndTail = Mini_CopyContours( pLoops[Layer] ) ;
	DOUBLE dRadRgh[2] = { 0.005, 0.005 } ;
	JDNC_COR  CorDef = GetCorDef() ;
	CorDef.m_nCorType = NCDEF_CORNER_ARCUSER ;
    
    if ( !BndHead && !m_bLayer[Layer] ) return 1 ;

	// 如果有轮廓角度, 则进行偏移,只偏移边界环
	if( !BndHead ) 
	{//如果高度大于曲面的高度，则用边界来加工
		if( !bIsRoughCast )
		{
			if( m_bFirst )
				BndHead = Mini_CopyContours( m_pAllLoop ) ;
			else
			{
				return FALSE ;
			}
		}
		else
		{// 出现加工面高于毛坯面的情况
			BndHead = Mini_OffsetContourList( m_pAllLoop, NCDEF_OFFSET_OUTER, dRadRgh, CorDef ) ;
		}
		MathCAM_ValidOutMostLoop( BndHead ) ;
	}
	else
	{	// 如果是毛坯面，就不对环进行处理了
		while( BndTail && BndTail->next )	BndTail = BndTail->next ;
		CSmartLoop *pAdd = Mini_CopyContours( pBigLoop ) ;
		BndHead = Mini_AddContours( BndHead, pAdd ) ;
		BndHead = looplib.BuildContour ( BndHead ) ;
		m_bFirst = FALSE ;
	}
	if( !BndHead ) return FALSE ;
	// 裁减环的排序
	BndHead = looplib.ResortContour ( BndHead, m_cSetupDef.m_cOrderDef.m_nSortType, FALSE ) ;
	if( bIsRoughCast )
	{
		if( m_cParam.m_bRPocketFlag & NCDEF_RPOCKET_OPENBND )
			SetContourBlank( BndHead ) ;
		AddLoopToLoopArr( m_pRoughCastLoopArr[Layer], BndHead, Layer, FALSE ) ;
	}
	else
	{
		AddLoopToLPArr( DriveMdl, m_pLPathArr[Layer], BndHead, Layer, FALSE ) ; 
	}

	return 1 ;
}

CSmartLoop *CSmartRPocketGen::SubtractCastArea ( CSmartLoop *pCutArea, int nLayer )
{
	// 如果是毛坯模式
	CSmartLoop *pBorder = NULL, *pRoughLoop = NULL, *pResult = NULL ;
	int nSize = (int)m_pRoughCastLoopArr[nLayer].GetSize() ;
	for( int j = 0 ; j < nSize ; j++ )
	{
		pRoughLoop = m_pRoughCastLoopArr[nLayer].GetAt( j ) ;
		pRoughLoop->next = pRoughLoop->prev = NULL ;    
		pBorder = Mini_AddContours( pBorder, pRoughLoop ) ;
	}
	if( !pBorder ) return pCutArea ;
	if( !pCutArea ) return Mini_CopyContours( pBorder ) ;
	DOUBLE dRadRgh[2] = { m_cSetupDef.m_cTolDef.m_dArcTol*2 , m_cSetupDef.m_cTolDef.m_dArcTol*2 } ;
	JDNC_COR  CorDef = GetCorDef() ;
	CorDef.m_nCorType = NCDEF_CORNER_ARCUSER ;
	CSmartLoop* pTmpHead = Mini_OffsetContourList( pBorder        , 
												   NCDEF_OFFSET_OUTER , 
												   dRadRgh         , 
												   CorDef ) ;
	// 为毛坯上加Bland标记
	pBorder  = pTmpHead  ;
	for( CSmartLoop* pLoop = pBorder ; pLoop ; pLoop = pLoop->next )
	{
		pLoop->SetBoundInfo( FALSE ) ;
	}
//	if( m_cParam.m_bRPocketFlag & NCDEF_RPOCKET_OPENBND )
  //      SetContourBlank( pBorder ) ;

	pResult = Mini_OperateContours( pCutArea, pBorder, 2 ) ;
	Mini_DeleteContours( pBorder ) ;
	Mini_DeleteContours( pCutArea ) ;
	return pResult ;
} 


int CSmartRPocketGen::GenAreaPath( CPathGroup &PGroup  ,	// 路径集合
								   CSmtCheckMdl *DriveMdl,	// 加工模型 
								   CSmartLoop *BndHead,		// 区域边界
								   double &dDepth,			// 雕刻深度
								   int nLayer,				// 雕刻层数 
								   BOOL bDelete, 			// 是否删除环
								   BOOL bAdsorb )
{
	if( !BndHead ) return 0 ;
	CSmartPocketGen  PocketGen ;
	(*(CSmartPathGen*)&PocketGen ) = * this ;
	PocketGen.m_cSpeedDef.m_dPlungeDist = 0. ;

	JDNC_POCKET&   Pocket = PocketGen.m_cParam;
    Pocket.m_bPocketFlag = 0 ;
	Pocket.m_nMoveType = m_cParam.m_nMoveType  ;
	if( m_cParam.m_nMoveType == surfncMoveLinear )
	{
		Pocket.m_cLinear = m_cParam.m_cLinear ;
	}
	else if( m_cParam.m_nMoveType == surfncMoveFollow )
	{
		Pocket.m_cFollow = m_cParam.m_cFollow ;
		if( ( m_cParam.m_bRPocketFlag & NCDEF_RPOCKET_BOUNDTOL ) &&
			m_cParam.m_dBoundTol  > 0.002 )
		{
			PocketGen.m_dBoundTol = m_cParam.m_dBoundTol ;
			PocketGen.m_dFeedRatio= m_cParam.m_dFeedRatio; 
		}
	}
	else
	{
		Pocket.m_cSpiral = m_cParam.m_cSpiral ;
	}
    if( m_cParam.m_bRPocketFlag & NCDEF_RPOCKET_OPENBND )
    {
        Pocket.m_bPocketFlag |= NCDEF_POCKET_OPENBND ;
        if( Pocket.m_nMoveType == surfncMoveFollow )
        {
            Pocket.m_cFollow.m_bFollowFlag |= NCDEF_FOLLOW_OPENBND ;
        }
    }
    PocketGen.m_cParam = Pocket ;
    PocketGen.UpdateTool2D( ) ;
	
	/////////////////////////////////////////////////////
	double delH = 0., dLastH = 0 ;
	if( nLayer == 0 )
	{
		delH = m_dTopHeight - m_dZ[nLayer] ;
		dLastH = m_dTopHeight + m_cSpeedDef.m_dPlungeDist ;
	}
	else
	{
        int nLastLayer = nLayer-1 ;
        while( !m_bLayer[nLastLayer] ) nLastLayer -- ;
		if ( nLastLayer >= 0 )
		{
            delH = m_dZ[nLastLayer] - m_dZ[nLayer] ;
			dLastH = m_dZ[nLastLayer] ;
		}
		else
		{
			delH = m_dTopHeight - m_dZ[nLayer] ;
            dLastH = m_dTopHeight +  m_cSpeedDef.m_dPlungeDist ;
		}
	}	
	
	CPathCombine *pTComb = new CPathCombine( NC_WPROCESS_ROUGH ) ;
	CSmartLoop *pRoughLoop = NULL, *pDriveLoop = NULL ;
	pRoughLoop = m_pRoughLoop[nLayer] ;
	pDriveLoop = m_pDriveLoop[nLayer] ;
	BOOL bNewLoop = FALSE ;
	if( !pDriveLoop && !pRoughLoop ) pDriveLoop = m_pAllLoop ;
	if( !( m_cParam.m_bRPocketFlag & NCDEF_RPOCKET_OPENBND ) || !HasBlankSectInContour( BndHead ) )
	{
		pRoughLoop = NULL ;
	}
//	else
//	{
//		if( m_bAllLoop )
//		{
//			// 边界和毛坯的交集
//			pRoughLoop = Mini_OperateContours( m_pAllLoop, pRoughLoop, 0 ) ;
//			bNewLoop = TRUE ;
//		}
//	}
	PocketGen.PocketOneContour( PGroup, *pTComb, *BndHead, Pocket, delH, pDriveLoop, pRoughLoop ) ; 
	if( bNewLoop ) Mini_DeleteContours( pRoughLoop ) ;

	if( pTComb->m_pHead)
	{
		// 如果减少抬刀
		if( nLayer >= 0 )
		{
            OptimizeGroup( pTComb, DriveMdl, dDepth, dLastH ) ;
		}
		// 添加同层之间的连刀路径 qqs 2014.09.10
		AddQuickPathInSameLayer(DriveMdl,pTComb,nLayer);
		if( bAdsorb )
		{// 添加相邻层之间的连刀路径 qqs 2014.09.10
			AddQuickPathBetweenLayer(DriveMdl, PGroup, pTComb, nLayer);
			//AddAdsorbPath( DriveMdl, PGroup, pTComb, dDepth ) ;			
		}
		PGroup.AddData( -dDepth , pTComb, TRUE ) ;
	}
	else 
	{
		delete pTComb  ;
	}
/*	//for test　测试环的正确性
	AddSmtLoopInPGroup( BndHead, dDepth,  PGroup ) ;
*/
	if( bDelete )
	{
        Mini_DeleteContours( BndHead ) ;
	}
	return  1 ;
}

CSmartLoop* CSmartRPocketGen::GetOneLayerLoop ( CSmtLoopArr& LoopArr )
{
	CSmartLoop *pHead = NULL, *pLoop = NULL ;
	for( int i = 0 ; i < LoopArr.GetSize () ; i++ )
	{
		pLoop = LoopArr.GetAt(i) ;
		if( !pLoop ) continue ;
		pLoop->next = pLoop->prev = NULL ;
		pHead = Mini_AddContours( pHead, pLoop ) ;
	}
	return pHead ;
}
CSmartLoop* CSmartRPocketGen::GetOneLayerLoop ( CSmtLPathArr &LPathArr )
{
	CSmartLoop *pHead = NULL ;
	CSmtLoopPath *pLPath = NULL ;
	for( int i = 0 ; i < LPathArr.GetSize () ; i++ )
	{
		pLPath = LPathArr.GetAt(i) ;
		if( !pLPath ) continue ;
		pHead = Mini_AddContours( pHead, pLPath->m_pLoop ) ;
	}
	return pHead ;
}
CSmartLoop *CSmartRPocketGen::CopyOneLayerLoop( CSmtLPathArr &LPathArr ) 
{
	CSmartLoop *pHead = NULL, *pCopy = NULL ;
	CSmtLoopPath *pLPath = NULL ;
	for( int i = 0 ; i < LPathArr.GetSize () ; i++ )
	{
		pLPath = LPathArr.GetAt(i) ;
		if( !pLPath ) continue ;
		pCopy = Mini_CopyContours( pLPath->m_pLoop ) ;
		pHead = Mini_AddContours( pHead, pCopy ) ;
	}
	return pHead ;
}
CSmartLoop* CSmartRPocketGen::CopyOneLayerLoop ( CSmtLoopArr& LoopArr )
{
	CSmartLoop *pHead = NULL, *pLoop = NULL, *pCopy = NULL ;
	for( int i = 0 ; i < LoopArr.GetSize () ; i++ )
	{
		pLoop = LoopArr.GetAt(i) ;
		if( !pLoop ) continue ;
		pLoop->next = pLoop->prev = NULL ;
		pCopy = Mini_CopyContours( pLoop ) ;
		pHead = Mini_AddContours( pHead, pCopy ) ;
	}
	return pHead ;
}

CSmartLoop* CSmartRPocketGen::RemoveOneLayerLoop ( CSmtLoopArr& LoopArr )
{
	CSmartLoop *pHead = NULL, *pLoop = NULL ;
	for( int i = 0 ; i < LoopArr.GetSize () ; i++ )
	{
		pLoop = LoopArr.GetAt(i) ;
		if( !pLoop ) continue ;
		pLoop->next = pLoop->prev = NULL ;
		pHead = Mini_AddContours( pHead, pLoop ) ;
	}
	LoopArr.RemoveAll () ;
	return pHead ;
}

CSmartLoop *CSmartRPocketGen::RemoveOneLayerLoop( CSmtLPathArr &LPathArr )
{
	CSmtLoopPath *pLPath = NULL ;
	CSmartLoop *pHead = NULL, *pLoop = NULL ;
	int i = 0, nSize = (int)LPathArr.GetSize () ;
	for( i = 0 ; i < nSize ; i++ )
	{
		pLPath = LPathArr.GetAt ( i ) ;
		pLoop = pLPath->m_pLoop ;
		pLPath->m_pLoop = NULL ;
		delete pLPath ;
		pHead = Mini_AddContours( pHead, pLoop ) ;
	}
	LPathArr.RemoveAll () ;
	return pHead ;
}

BOOL CSmartRPocketGen::CreateAllDriveAndPlaneLoop( CSmtCheckMdl *DriveMdl, CPathGroup& NewPath, JDNC_PRGDEF &ProgDef, double dCur ) 
{
    UNUSED_ALWAYS(NewPath);
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
	CSmartLoop loop, *PlaneLoop = NULL ;
	double dPlaneCur = dCur * 0.2, dLastCur = dCur * 0.4 ;
	// 搜索平面优化平面环 
	if( m_cParam.m_bRPocketFlag & NCDEF_RPOCKET_CUTPLANE )
    {
		// 生成平面加工区域
		CSmartFPlaneGen FPlaneGen ;
		(*(CSmartPathGen*)&FPlaneGen ) = * this ;
		FPlaneGen.m_cMethodDef.m_nMethodType = surfncMethodFPlane ;
		SurfNC_InitPathParam( FPlaneGen.m_cParam ) ;
		FPlaneGen.m_cMethodDef.m_cPlaneCut = FPlaneGen.m_cParam ;
		ProgDef.m_dTotalMove = dPlaneCur ;
		FPlaneGen.CreatePlaneContour ( *DriveMdl, &heightMdl, PlaneLoop, ProgDef ) ;
		//FilterAllPlaneBnd( PlaneLoop ) ;	
    }
	else
	{
		dLastCur = dCur * 0.6 ;
	}

	// 分层
	GetECutLayers( PlaneLoop ) ;	
//	TestAllPlaneLoop( PlaneLoop, NewPath ) ;
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
		GetAllDriveLoop( AllLoop, dAlldZ, bFlag, nAllCnt ) ;
		// 得到所有平面环
 		if( m_cParam.m_bRPocketFlag & NCDEF_RPOCKET_CUTPLANE && PlaneLoop )
		{
			GetAllPlaneLoop( AllLoop, dAlldZ, bFlag, nAllCnt, PlaneLoop ) ;
		}
	}
	
	ClearAllLoopHead( AllLoop, nAllCnt ) ;
	// 对区域进行处理
	
	if( (m_cSetupDef.m_cModelTol.m_nMdlFlag & NCDEF_SETUP_SMOOTHCORNER) && 
        m_cSetupDef.m_cTolDef.m_dMinRad > 0.02 )
    {
	    JDNC_COR tmpCor = m_cSetupDef.m_cCorDef ;
	    tmpCor.m_nCorType = NCDEF_CORNER_ARCALL ; 
	    DOUBLE dOffDist[2] = { m_cSetupDef.m_cTolDef.m_dMinRad,m_cSetupDef.m_cTolDef.m_dMinRad} ;
	    for( int i = 0 ; i < m_nCount ; i++ )
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
void CSmartRPocketGen::GetAllDriveLoop ( CSmartLoop **AllLoop, double *dZ, BOOL *bFlag, int nCnt )
{
	if( !AllLoop || !dZ ) return ;
	int i = 0, j = 0 ;
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
void CSmartRPocketGen::GetAllPlaneLoop ( CSmartLoop **AllLoop, double *dZ, BOOL *bFlag, int nCnt, CSmartLoop *PlaneLoop )
{
	if( !AllLoop || !dZ || !bFlag || !PlaneLoop ) return ;
	int i = 0 ;
	while( PlaneLoop )
	{
		CSmartLoop * pHead = PlaneLoop ;
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
	m_dMaxZ = m_cSetupDef.m_cTolDef.m_dArcTol * 3 ;
	if( fabs( m_cStockDef.m_dDriveOffset[0] ) < 0.0001 )
		m_dMaxZ = m_cSetupDef.m_cTolDef.m_dArcTol ;
	else
        m_dMaxZ = min( m_dStepZ * 0.2, m_dMaxZ ) ;
	
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
				if(!m_bLayer[i] ) continue ;
				dTmpH = m_dZ[i] - pPlaneHead->m_pPlaneLoop->m_dHeight ;
			}
			if( dTmpH < dMinZ && dTmpH > 0 )
			{
				dMinZ = dTmpH ;
			}
			if( dMinZ < m_dMaxZ ) break ;
			// 然后处理极值情况,当两层距离非常之小,是否也应该删除?
			if( fabs( dTmpH ) < 0.0005 )
			{
				dMinZ = dTmpH ;
				break ;
			}
		}
		// 对找到的最近距离进行处理
		if( dMinZ < m_dMaxZ )
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
BOOL CSmartRPocketGen::CreateAllRoughAndPlaneLoop ( CSmtCheckMdl *RoughMdl, JDNC_PRGDEF &ProgDef, double dCur )
{
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
		GetAllRoughLoop( AllLoop, dAlldZ, bFlag, nAllCnt ) ;
		// 得到所有平面环
		if( m_cParam.m_bRPocketFlag & NCDEF_RPOCKET_CUTPLANE  )
		{
			TrimAllPlaneLoop( AllLoop, dAlldZ, bFlag, nAllCnt, m_cPlaneList ) ;
		}
	}

	ClearAllLoopHead( AllLoop, nAllCnt ) ;
	// 对区域进行处理
	for( int i = 0 ; i < m_nCount ; i++ )
	{
		m_pRoughLoop[i] = MathCAM_ExtractAllLoop( m_pRoughLoop[i] ) ;
		m_pRoughLoop[i] = loop.BuildContour ( m_pRoughLoop[i] ) ;
		// 毛坯环全部设为blank
		if( m_cParam.m_bRPocketFlag & NCDEF_RPOCKET_OPENBND )
		{
            SetContourBlank( m_pRoughLoop[i] ) ;
		}
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
void CSmartRPocketGen::GetAllRoughLoop( CSmartLoop **AllLoop, double *dZ, BOOL *bFlag, int nCnt )
{
	if( !AllLoop || !dZ ) return ;
	int i = 0, j = 0 ;
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
void CSmartRPocketGen::TrimAllPlaneLoop( CSmartLoop **AllLoop, double *dZ, BOOL *bFlag, int nCnt, CPlaneList &cList )
{
	CPlaneLoop *pHead = NULL , *pNext = NULL ;
	CSmartLoop looplib, *pRoughHead = NULL ;
	CSmartLoop *pOffset = NULL, *pLarge = NULL, *pCopy = NULL ;
	BOX3D box1, box2, maxBox ;
	int i = 0 ;
	DOUBLE dRadRgh[2] = { 0.002, 0.002 } ;
	JDNC_COR  CorDef = GetCorDef() ;
	CorDef.m_nCorType = NCDEF_CORNER_ARCUSER ;
	double tol = 2 * m_pTool->m_fRadius, tmpH = 0 ;
	MathCam_GetLoopBox( m_pAllLoop, &box1 ) ;
	POSITION pos, atpos = NULL ;
	pos = cList.GetHeadPosition () ;
	while( pos )
	{
		atpos = pos ;
		pHead = cList.GetNext ( pos ) ;
		pRoughHead = NULL ;
		// 生成毛坯环
		for( i = 0 ; i < nCnt ; i++ )
		{
			if( bFlag[i] ) continue ;
			if( fabs( pHead->m_dHeight - dZ[i] ) < 0.0003 )
			{
				pRoughHead = Mini_CopyContours( AllLoop[i] ) ;
				break ;
			}
		}
		if( !pRoughHead ) continue ;
		if( m_cParam.m_bRPocketFlag & NCDEF_RPOCKET_OPENBND )
            SetContourBlank( pRoughHead) ;
		pCopy = Mini_CopyContours( pRoughHead ) ;
		pHead->SetRoughLoop ( pCopy ) ;

		MathCam_GetLoopBox( pRoughHead, &box2 ) ;
		mathCalBox3DUnion( &box1, &box2, &maxBox ) ;
		mathExpandBox3D( tol, &maxBox ) ;
		pLarge = new CSmartLoop() ;
		pLarge->CreateLoop ( maxBox.min, maxBox.max ) ;
		pRoughHead = Mini_AddContours( pRoughHead, pLarge ) ;
		pRoughHead = looplib.BuildContour ( pRoughHead ) ;
		// 将毛坯环设为Blank
		if( m_cParam.m_bRPocketFlag & NCDEF_RPOCKET_OPENBND )
		{
            SetContourBlank( pRoughHead) ;
		}

		pOffset = Mini_OffsetContourList( pRoughHead, NCDEF_OFFSET_OUTER, dRadRgh, CorDef ) ;
		if( pOffset )
		{
			Mini_DeleteContours( pRoughHead ) ;
			pRoughHead = pOffset ;
		}
	
		// 修剪
		tmpH = pHead->m_dHeight ;
		pOffset = Mini_OperateContours( pHead->m_pPlaneLoop, pRoughHead, 2 ) ;
		Mini_DeleteContours( pHead->m_pPlaneLoop ) ;
		pHead->m_pPlaneLoop = NULL ;
		Mini_DeleteContours( pRoughHead ) ;
		if( pOffset )
		{
			BOOL bFirst = TRUE ;
			CSmartLoop * BndHead = pOffset, * BndTail = NULL ;
			while ( BndHead )
			{
				BndTail = BndHead->next ;
				BndHead->next = BndHead->prev = NULL ;
				BndHead->m_dHeight = tmpH ;
				BndHead->DefineBox() ;
				if (bFirst )
				{
					pHead->SetPlaneLoop( BndHead ) ;
					bFirst = FALSE ;
				} 
				else
				{
					pNext = new CPlaneLoop( BndHead, Mini_CopyContours( pHead->m_pOrgLoop) ) ;
					pNext->SetRoughLoop( Mini_CopyContours( pHead->m_pRoughLoop ) ) ;
					pNext->SetRemainLoop( Mini_CopyContours( pHead->m_pRemainLoop ) ) ;
					cList.InsertBefore( atpos, pNext ) ;
				}
				BndHead = BndTail ;
			}
		}
		else
		{
			cList.RemoveAt ( atpos ) ;
			delete pHead ;
		}
	}
	if( pRoughHead ) Mini_DeleteContours( pRoughHead ) ;
	/**/
}
BOOL CSmartRPocketGen::CreateAllRoughLoop ( CSmtCheckMdl *RoughMdl, CPlaneList &cList, JDNC_PRGDEF &ProgDef, double dCur )
{
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
	for( int i = 0 ; i < m_nCount ; i++ )
    {
		m_pRoughLoop[i] = NULL ;
    }
	ProgDef.m_dTotalMove = dCur * 0.6 ;
	if( !heightGen.CreateContourLoops ( m_dZ, m_pRoughLoop, int( m_nCount ), ProgDef ) )
	{//已经并为区域
		return FALSE ;
	}
	if( cList.GetCount () > 0 )
	{
		TrimPlaneHeadByRough( heightGen, cList ) ;
	}
	CSmartLoop loop ;
	for( int i = 0 ; i < m_nCount ; i++ )
	{
		m_pRoughLoop[i] = MathCAM_ExtractAllLoop( m_pRoughLoop[i] ) ;
		m_pRoughLoop[i] = loop.BuildContour ( m_pRoughLoop[i] ) ;
		// 毛坯环全部设为blank
		SetContourBlank( m_pRoughLoop[i] ) ;
	}
    if( bLocalCopy == TRUE )
    {
        RoughMdl->DeleteMultiCopy() ;
    }
	return TRUE ;
}

BOOL CSmartRPocketGen::BuildAllDriveLoop ( CSmtCheckMdl *DriveMdl, BOX3D &dMaxBox, JDNC_PRGDEF &ProgDef, double dCur )
{
	m_pRoughCastLoopArr = new CSmtLoopArr[m_nCount] ;
	m_bFirst = TRUE ;
	int i = 0, nRemain = 0, nTotal = 0 ;
	if( m_cParam.m_cBtwRecut.m_nLayerType != NCDEF_BTWLAYER_CLOSE )
	{
		for( i = 0 ; i < m_nCount ; i++ )
		{
			if( !m_bLayer[i] ) nTotal++ ;
		}
	}
	ProgDef.m_dLimitAt =  ( m_nCount * 2 + nTotal ) / dCur ;
	ProgDef.m_dStepAt  = 0.0 ;
	ProgDef.m_dIncStep = 1.0  ;
	CSmartLoop *pBigLoop = NULL ;
	// 首先得到加工环的最大包围环
	pBigLoop = CreateBigLoopAndBox( m_pRoughLoop, m_pAllLoop, dMaxBox, m_nCount, m_pTool->m_fRadius * 2.5 ) ;
	for( i = 0 ; i < m_nCount ; i ++ ) 
	{
		if( ! LinearOneLayer( DriveMdl, m_pRoughLoop, pBigLoop, i, TRUE )  )
		{
			break ;
		}
		if( ProgDef.m_pBrkFunc && ProgDef.m_pBrkFunc () )
		{
			Mini_DeleteContours( pBigLoop ) ;
			return FALSE ;
		}
		ProgDef.m_dStepAt += ProgDef.m_dIncStep ;
		while( ProgDef.m_pPrgFunc && ProgDef.m_dStepAt >= ProgDef.m_dLimitAt )
		{
			ProgDef.m_pPrgFunc(1) ;
			ProgDef.m_dStepAt -= ProgDef.m_dLimitAt  ;
		}
	}
	nRemain = m_nCount - i ;
	Mini_DeleteContours( pBigLoop ) ;
	pBigLoop = CreateBigLoopAndBox( m_pDriveLoop, m_pAllLoop, dMaxBox, m_nCount, m_pTool->m_fRadius * 2 ) ;
	m_bFirst = TRUE ;
	for( i = 0 ; i < m_nCount ; i ++ ) 
	{ 
		if( ! LinearOneLayer( DriveMdl, m_pDriveLoop, pBigLoop, i, FALSE )  )
		{
			break ;
		}
		if( ProgDef.m_pBrkFunc && ProgDef.m_pBrkFunc () )
		{
			Mini_DeleteContours( pBigLoop ) ;
			return FALSE ;
		}
		ProgDef.m_dStepAt += ProgDef.m_dIncStep ;
		while( ProgDef.m_pPrgFunc && ProgDef.m_dStepAt >= ProgDef.m_dLimitAt )
		{
			ProgDef.m_pPrgFunc(1) ;
			ProgDef.m_dStepAt -= ProgDef.m_dLimitAt  ;
		}		
	}
	Mini_DeleteContours( pBigLoop ) ;
	nRemain += m_nCount - i ;

	//设置最底层标记
	MarkBottomLayer() ;

	if ( m_cParam.m_cBtwRecut.m_nLayerType == NCDEF_BTWLAYER_ROUGH )
	{
		//记录层间加工下刀层
		m_pBtwPlgLayer = new int[m_nCount] ;
		memset( m_pBtwPlgLayer, 0, (m_nCount) * sizeof(int) ) ;
	}

	// 层间补加工	
	if ( m_cParam.m_cBtwRecut.m_nLayerType == NCDEF_BTWLAYER_ROUGH )
	{//层间粗加工
		CSmartLoop looplib, *pNext = NULL, *pCurr = NULL, *pBtwLoop = NULL ;
		int curLayer = 0 ;
		for( i = 0 ; i < m_nCount ; i++ )
		{
			if( m_bLayer[i] ) continue ;
			nTotal-- ;
			if( ProgDef.m_pBrkFunc && ProgDef.m_pBrkFunc () )
			{
				return FALSE ;
			}
			ProgDef.m_dStepAt += ProgDef.m_dIncStep ;
			while( ProgDef.m_pPrgFunc && ProgDef.m_dStepAt >= ProgDef.m_dLimitAt )
			{
				ProgDef.m_pPrgFunc(1) ;
				ProgDef.m_dStepAt -= ProgDef.m_dLimitAt  ;
			}
			// 得到当前层环
			pCurr = RemoveOneLayerLoop( m_pLPathArr[i] ) ;
			// 得到下层原始环
			if( m_cParam.m_cBtwRecut.m_bBtwLayerFlag & NCDEF_BTWLAYER_BOTUP )
			{ // 从下往上走,采用下层环
				curLayer = i + 1 ;
				if( curLayer > m_nCount - 1 ) curLayer = m_nCount - 1 ;
			}
			else
			{ // 从上往下走,得到下层分层环
				curLayer = FindNextLayer( i ) ;
			}
			
			m_pBtwPlgLayer[i] = curLayer ;

			// 获得下层环的拷贝
			pNext = Mini_CopyContours( m_pDriveLoop[curLayer] ) ;
			if( !pCurr && pNext )
			{
				BOX3D box ;
				MathCam_GetLoopBox( pNext, &box ) ;
				pCurr = new CSmartLoop() ;
				pCurr->CreateLoop ( box.min , box.max ) ;
			}
			// 当前层环pCurr与添加包围盒的pNext相减\与毛坯环相减\与边界环求交
			pBtwLoop = CalcCurrBtwLoop( pCurr, pNext, 0.008, i ) ;
			Mini_DeleteContours( pCurr ) ;
			if( !pBtwLoop ) continue ;
			// 裁减环的排序
			pBtwLoop = looplib.ResortContour ( pBtwLoop, m_cSetupDef.m_cOrderDef.m_nSortType, FALSE ) ;
			AddLoopToLPArr( DriveMdl, m_pLPathArr[i], pBtwLoop, i, FALSE ) ;
			pBtwLoop = pCurr = NULL ;
		}
	}
	else if( m_cParam.m_cBtwRecut.m_nLayerType == NCDEF_BTWLAYER_FINISH )
	{//层间精加工
		CSmartLoop * pLoop = NULL ;
		
		for ( i = 0; i < m_nCount; i++ )
		{
			if ( m_bLayer[i] ) continue ;
			nTotal -- ;
			// 进度条显示
			if( ProgDef.m_pBrkFunc && ProgDef.m_pBrkFunc () )
			{
				return FALSE ;
			}
			ProgDef.m_dStepAt += ProgDef.m_dIncStep ;
			while( ProgDef.m_pPrgFunc && ProgDef.m_dStepAt >= ProgDef.m_dLimitAt )
			{
				ProgDef.m_pPrgFunc(1) ;
				ProgDef.m_dStepAt -= ProgDef.m_dLimitAt  ;
			}              
			// 以前若有环，清除
			ClearLPathArr( m_pLPathArr[i] ) ;
            // 得到层间精加工环
			pLoop = Mini_CopyContours( m_pDriveLoop[i] ) ;
			if( !pLoop ) continue ;
			pLoop = MathCAM_ExtractAllLoop( pLoop ) ;
			AddLoopToLPArr( DriveMdl, m_pLPathArr[i], pLoop, i, FALSE ) ;
		}
	}
	nRemain += nTotal ;
	for( i = 0 ; i < nRemain; i++ )
	{
		ProgDef.m_dStepAt += ProgDef.m_dIncStep ;
		while( ProgDef.m_pPrgFunc && ProgDef.m_dStepAt >= ProgDef.m_dLimitAt )
		{
			ProgDef.m_pPrgFunc(1) ;
			ProgDef.m_dStepAt -= ProgDef.m_dLimitAt  ;
		}
	}
	//防止计算误差引起的进度条少走一步
	if( ProgDef.m_pPrgFunc &&  (ProgDef.m_dStepAt/ProgDef.m_dLimitAt ) >0.1 )
	{
		ProgDef.m_pPrgFunc( 1 ) ;
	}
	return TRUE ;
}
int CSmartRPocketGen::GeneratePathEx( CPathGroup& NewPath     ,  /*刀具路径*/
						              CSmartGraphic& Graph )  /*雕刻图形*/
{  
	InitTmpProg() ;
	if( !UpdateTool3D(Graph) ) return 0 ;
    RepairBlindRadius( m_cFeedDef.m_cPlungeDef ) ;
	if( !m_pTool ) return 0 ;
//	double dArea = GetToolBotArea( m_pTool ) ;
//	m_dMinArea = 0.03 ;
//	if( dArea < 0.1 ) m_dMinArea = dArea * 0.25 ;
	m_dMinArea = 0. ;	// 该变量yul封于2011.06.22,涉及到得主要问题是带有小孔的浮雕使用粗加工
					    // 或者残补走完之后，就直接精加工，使用锥刀，会导致吃刀量突然增加，断刀
	// 生成毛坯模型
	m_pAllLoop = NULL ;
	CSmtCheckMdl RoughMdl ;
	BOX3D sbox ;
	mathClnBox3D( &sbox ) ;

	// 牙齿路径中，创建毛坯占80的进度条
	if( !BuildStockMdl( Graph, RoughMdl, sbox ) )
	{
		m_nErrorType = JDERROR_GENPATH_NOSTOCK ;
		return 0 ;
	}
	
	double dCur[5] = { 40, 20, 10, 10, 20 }, dBot = sbox.min[2], dTop = sbox.max[2] ;
	if (glbf_GetProductID() == PRODUCT_ID_DENTFORM)
	{
		dCur[0] = 30;
		dCur[1] = 10;
		dCur[2] = 10;
		dCur[3] = 10;
		dCur[4] = 20;
	}
    ///////////////处理边界环////////////////////////
	m_pAllLoop = CreateAreaLoop( Graph, sbox, m_pRoughTool->m_fRadius * 1.5, m_bAllLoop ) ;
	sbox.min[2] = dBot, sbox.max[2] = dTop ;
	if( !m_pAllLoop )
	{
		m_nErrorType = JDERROR_GENPATH_NOLOOP ;
		return FALSE ;
	}
	// STEP 1 : 提取曲面图形 , 并声称干涉检查曲面
	CSmtCheckMdl  DriveMdl, CheckMdl ;
	BOX3D box ;
	double dExLen = 2. ;
	int k = 0 ;
	if( m_pSafeTool )
		dExLen += max( m_pTool->m_fRadius * 2, m_pSafeTool->m_fRadius * 2 ) ;
	else
		dExLen += m_pTool->m_fRadius * 2 ;
	MathCam_GetLoopBox( m_pAllLoop, &box ) ;
	mathExpandBox3D( dExLen, &box ) ;
	for( k = 0 ; k < 2 ; k++ )
	{
		DriveMdl.m_dFltBox[0][k] = CheckMdl.m_dFltBox[0][k] = box.min[k] ;
		DriveMdl.m_dFltBox[1][k] = CheckMdl.m_dFltBox[1][k] = box.max[k] ; 
	}

	//guomin仅在木雕环境勾选大浮雕开粗时使用重写的函数
	WORD wProductID = glbf_GetProductID() ;
	if(wProductID == PRODUCT_ID_WOODRELIEF 
		/*&& m_cSetupDef.m_cModelTol.m_nMdlFlag & NCDEF_SETUP_LARGEFASTROUGH*/ )
	{
		if( !BuildDriveAndCheckMdlnew( Graph, DriveMdl, CheckMdl ) )
		{
			m_nErrorType = JDERROR_GENPATH_NOAREA ;
			return 0 ;
		}
	}
	else
	{
		if( !BuildDriveAndCheckMdl( Graph, DriveMdl, CheckMdl ) )
		{
			m_nErrorType = JDERROR_GENPATH_NOAREA ;
			return 0 ;
		}
	}	

	//guomin 判断m_pRectNetSurf点数大于6000W则使用多线程连刀 
	BOOL bLarge = FALSE ;
	if( m_pRectNetSurf )
	{
		if( m_pRectNetSurf->m_nNumVertX*m_pRectNetSurf->m_nNumVertY > 1e+7)
			bLarge = TRUE ;
	}
	
	// 使用毛坯限定加工范围
	CalcTopAndBot( sbox ) ;
	RoughMdl.m_fBottom = TFLOAT( m_dBotHeight ) ;
	if( fabs( m_dTopHeight - m_dBotHeight ) < 0 )
	{
		m_nErrorType = JDERROR_GENPATH_NOTGENPATH ;
		return FALSE ;
	}
	DriveMdl.m_fBottom = CheckMdl.m_fBottom = TFLOAT( m_dBotHeight) ;
	// 为m_bBndTrim赋初值
	if( m_cParam.m_nMoveType == NCDEF_RPOCKET_FOLLOW &&  m_cParam.m_cFollow.m_bFollowFlag & NCDEF_FOLLOW_BNDTRIM )
		m_bBndTrim = TRUE ;
	// STEP 2 : 得到等高环\毛坯环\平面环
	if (glbf_GetProductID() != PRODUCT_ID_DENTFORM)
	{
		ProcessNewFunc( m_cPrgDef, IDS_PATHGEN_STEP_ROUGHPATH ) ;
	}
	
	//guomin 木雕环境且准备了一个矩形网格调新方法生成driveLoop
	if( (wProductID == PRODUCT_ID_WOODRELIEF) 
		/*&& m_cSetupDef.m_cModelTol.m_nMdlFlag & NCDEF_SETUP_LARGEFASTROUGH*/
		&& m_pRectNetSurf )
	{
		if( !CreatDriveLoopForRectNetSurf(&DriveMdl,m_cPrgDef,dCur[0]) )
		{
			m_nErrorType = JDERROR_GENPATH_NOAREA ;
			return 0 ;
		}
	}
	else 
	{
		if( !CreateAllDriveAndPlaneLoop( &DriveMdl, NewPath, m_cPrgDef, dCur[0] ) )
		{
			m_nErrorType = JDERROR_GENPATH_NOAREA ;
			return 0 ;
		}	
	}

	if( m_cPrgDef.m_pPosFunc )
	{
		if (glbf_GetProductID() == PRODUCT_ID_DENTFORM)
		{
			m_cPrgDef.m_pPosFunc( (int)dCur[0]+20, 100 ) ;
		}
		else
		{
			m_cPrgDef.m_pPosFunc( (int)dCur[0], 100 ) ;
		}
	}

	if( !CreateAllRoughAndPlaneLoop( &RoughMdl, m_cPrgDef, dCur[1] ) )
	{
		m_nErrorType = JDERROR_GENPATH_NOSTOCK ;
		return 0  ;
	}
	
	if( m_cPrgDef.m_pPosFunc )
	{
		if (glbf_GetProductID() == PRODUCT_ID_DENTFORM)
		{
			m_cPrgDef.m_pPosFunc( (int)(dCur[0] + dCur[1] + 20), 100 ) ;
		}
		else
		{
			m_cPrgDef.m_pPosFunc( (int)(dCur[0] + dCur[1]), 100 ) ;
		}
		
	}
	RoughMdl.ClearAllCheck () ;
	if( !m_pDriveLoop || !m_dZ ) return 0 ;
	if( !m_pRoughLoop ) return 0 ;
	// STEP 3 : 处理等高环
//  TestLoopArr( m_pDriveLoop, m_dZ, m_nCount, NewPath ) ;
//	AddContourToGroup( m_pAllLoop, 0., NewPath ) ;
 //	return 1 ;
	// 定义用于连刀的变量
	m_pLPathArr = new CSmtLPathArr[m_nCount] ;
	if( m_bBndTrim || m_cParam.m_cBtwRecut.m_nLayerType == NCDEF_BTWLAYER_ROUGH )
	{
		m_pCurveLib = new CSmartCurveLib[m_nCount] ;
		m_pPlunge = new CSmartLoop*[m_nCount] ;
		for( int i= 0 ; i < m_nCount ; i++ )
		{
			m_pPlunge[i] = NULL ;
		}
	}
	if( !BuildAllDriveLoop( &DriveMdl, box, m_cPrgDef, dCur[2] ) )
		goto EndLine ;
//////////////fortest////////////////////////////////////////////////
//	for( int i = 0 ; i < m_nCount ; i++ )
//	{
//		AddLoopToGroup( m_pRoughCastLoopArr[i], m_dZ[i], NewPath ) ;
//	}
//	return 1 ;
//////////////////////////////////////////////////////////////////////
	// 进行路径的修剪
	if( !SubtractAllLoopForOpti( &DriveMdl, NewPath, m_cPrgDef, dCur[3] ) )
		goto EndLine ;
	
	// 设置加工域的起点为Blank_SECT处
	SetAllCutAreaHeadAtBlank() ;

	// 对平面加工环进行处理
	if( m_cParam.m_bRPocketFlag & NCDEF_RPOCKET_CUTPLANE )
	{
		// 设置过滤平面的起点为Blank_SECT处
		SetAllPlaneLoopHeadAtBlank() ;
		// 为平面环搜索合适的位置
		SearchRationalPosition( m_cPlaneList ) ;	
	}
// 	TestAllPlaneLoop( &m_cPlaneList, NewPath, 1 ) ;
//	for( int i = 0 ; i < m_nCount ; i++ )
//	{
// 		AddLoopToGroup ( m_pLPathArr[i], m_dZ[i], NewPath ) ;	
//	}
//	return 1 ;
    // 层间精加工
	if( m_cParam.m_cBtwRecut.m_nLayerType == NCDEF_BTWLAYER_FINISH )
	{// 得到层间加工曲线
	    GetAllBtwCurves() ;
	}
 
	// STEP 4 : 生成路径--第2段进度条赋值
	int nTotal = 0 ;
	if( m_cParam.m_cBtwRecut.m_nLayerType == NCDEF_BTWLAYER_FINISH )
	{// 层间精加工时
		for( k = 0 ; k < m_nCount ; k ++ )
		{
			if( m_bLayer[k] )
			{
				nTotal += (int)m_pLPathArr[k].GetSize () ;
			}
			else
			{
				nTotal += (int)m_pCurveArr[k].GetSize() ;
			}
		}
	} 
	else
	{
		for( k = 0 ; k < m_nCount ; k ++ )
		{	
			nTotal += (int)m_pLPathArr[k].GetSize () ;
		}
	}	
	if( nTotal < 1 ) nTotal = 1 ;

	m_cPrgDef.m_dLimitAt =  nTotal / ( dCur[4] ) ;
	m_cPrgDef.m_dStepAt  = 0.0 ;
	m_cPrgDef.m_dIncStep = 1.0  ;
	if(GetOrderDef().m_nLayerOrd )// ==1高度优先  
	{
		for( int i = 0 ; i < m_nCount ; i ++ )
		{	
			if(	!m_bLayer[i] ) continue ; 
			if( !SortLoopByDepth( NewPath, &DriveMdl,  i, m_cPrgDef ) )
			{
				NewPath.ClearAll() ;
				goto EndLine ;
			}
			if( m_cParam.m_cBtwRecut.m_nLayerType == NCDEF_BTWLAYER_CLOSE )
				continue ;
			if( !SorBtwLoopByDepth( NewPath, &DriveMdl, i, m_cPrgDef ) )
			{
				goto EndLine ;
			}
		}
	}	
	else // == 0区域优先 
	{
		if( !SortLoopByArea ( NewPath, &DriveMdl, m_nCount, m_cPrgDef ) )
		{
			NewPath.ClearAll() ;
			goto EndLine ;
		}
	}
	if( m_cParam.m_bRPocketFlag & NCDEF_RPOCKET_CUTPLANE )
	{
		CreateRemainPlanePath( &DriveMdl, NewPath ) ;
	}
	//防止计算误差引起的进度条少走一步
	if( m_cPrgDef.m_pPrgFunc && ( m_cPrgDef.m_dStepAt/m_cPrgDef.m_dLimitAt ) > 0.1 ) 
	{
		m_cPrgDef.m_pPrgFunc( 1 ) ;
	}
EndLine:
	ClearAllLoopHead( m_pDriveLoop, m_nCount ) ;
	ClearAllLoopHead( m_pRoughLoop, m_nCount ) ;
	ClearAllLoopArr( m_pRoughCastLoopArr, m_nCount ) ;
	ClearAllBtwCurves() ;  //清除层间加工曲线
	
	if( ! NewPath.m_pHead )
	{
		m_nErrorType = JDERROR_GENPATH_NOTGENPATH ;
		return 0 ;
	}

	// 添加慢速和相对高度连接
	//guomin 木雕中m_pRectNetSurf顶点数超过6kw才使用多线程
	if( (wProductID == PRODUCT_ID_WOODRELIEF) && !bLarge )
		AddPlungeAndRelativeHbyMultithread( &DriveMdl, &NewPath, FALSE, m_dZ, m_nCount ) ;
	else
		AddPlungeAndRelativeH(&DriveMdl, &NewPath, FALSE, m_dZ, m_nCount ) ;
	ClearAllLoop() ;
	return 1 ;
}
int  CSmartRPocketGen::SorBtwLoopByDepth( CPathGroup& NewPath, CSmtCheckMdl *DriveMdl, int nLayer, JDNC_PRGDEF &ProgDef )
{
	CurveList CurList ;
	int nPrev = FindPrevLayer( nLayer ), j = 0 ;
	if( m_cParam.m_cBtwRecut.m_bBtwLayerFlag & NCDEF_BTWLAYER_BOTUP )
	{  
		for( j = nLayer - 1 ; j > nPrev ; j-- )
		{
		    if ( m_cParam.m_cBtwRecut.m_nLayerType == NCDEF_BTWLAYER_FINISH )
		    {
		        if ( !SortCurvesByDepth( NewPath, DriveMdl, j, CurList ) )
		        {//记录层间曲线到CurList
		            NewPath.ClearAll() ;
		            return 0 ;		       
		        }
		    } 
		    else if ( m_cParam.m_cBtwRecut.m_nLayerType == NCDEF_BTWLAYER_ROUGH )
		    {
				if( !SortLoopByDepth( NewPath, DriveMdl,  j, ProgDef ) )
				{
					NewPath.ClearAll() ;
					return 0 ;
				}
		    }
		}
	}
	else
	{
		for( j = nPrev + 1 ; j < nLayer ; j++ )
		{
			if ( m_cParam.m_cBtwRecut.m_nLayerType == NCDEF_BTWLAYER_FINISH )
			{
				if ( !SortCurvesByDepth( NewPath, DriveMdl, j, CurList ) )
				{//记录层间曲线到CurList
					NewPath.ClearAll() ;
					return 0 ;		       
				}
			} 
			else if ( m_cParam.m_cBtwRecut.m_nLayerType == NCDEF_BTWLAYER_ROUGH )
			{
				if( !SortLoopByDepth( NewPath, DriveMdl,  j, ProgDef ) )
				{
					NewPath.ClearAll() ;
					return 0 ;
				}
			}
		}
	}
	//生成层间精加工路径
	if ( m_cParam.m_cBtwRecut.m_nLayerType == NCDEF_BTWLAYER_FINISH )
	{
		SetCurveListStart( CurList ) ;
		TransfCurveListToPath( NewPath, DriveMdl, CurList, ProgDef ) ;
	}
	return 1 ;
}
void CSmartRPocketGen::SetAllCutAreaHeadAtBlank ()
{
	int i, j, nSize ;
	CSmtLoopPath *pLPath = NULL ;
	CSmtLPathArr tmpArr ;
	for( i = 0 ; i < m_nCount ; i++ )
	{
		nSize =(int) m_pLPathArr[i].GetSize () ;
		for( j = 0 ; j < nSize ; j++ )
		{
			pLPath = m_pLPathArr[i].GetAt ( j ) ;
			// 删除盲区并指定起点
			if( IsBlindLoop( pLPath->m_pLoop ) )
			{
				delete pLPath ;
			}
			else
			{
				SetAllLoopHeadAtBlank( pLPath->m_pLoop ) ;
				tmpArr.Add ( pLPath ) ;
			}
		}
		m_pLPathArr[i].RemoveAll () ;
		nSize = (int)tmpArr.GetSize () ;
		for( j = 0 ; j < nSize ; j++ )
		{
			pLPath = tmpArr.GetAt ( j ) ;
			m_pLPathArr[i].Add ( pLPath ) ;
		}
		tmpArr.RemoveAll () ;
	}
}

void CSmartRPocketGen::SetAllPlaneLoopHeadAtBlank()
{
    CPlaneLoop * pPlane = NULL ;
    POSITION pos, atpos = NULL;
    
    pos = m_cPlaneList.GetHeadPosition() ;
    while ( pos )
    {
        atpos = pos ;
        pPlane = m_cPlaneList.GetNext( pos ) ;
        if (IsBlindLoop( pPlane->m_pPlaneLoop))
        {
            m_cPlaneList.RemoveAt( atpos ) ;
            delete pPlane ;
            pPlane = NULL ;
        }
        else
        {
			// 将起点设置在Blank标记处
			SetAllLoopHeadAtBlank( pPlane->m_pPlaneLoop ) ;
        }
    }
}
BOOL CSmartRPocketGen::IsBlindLoop ( CSmartLoop *pLoop )
{
	double dMinDiam = 0. ;
	CSmartSect *pSect = NULL ;
	if( m_cFeedDef.m_cPlungeDef.m_bPlungeFlag & NCDEF_PLUNGE_DELBLINDAREA )
    {
		// 消除加工中的盲区
        dMinDiam = m_cFeedDef.m_cPlungeDef.m_dBlindRadius * 2.0 ;
        if( dMinDiam  > 0.05 && 
            pLoop->m_dBox[1][0] - pLoop->m_dBox[0][0] < dMinDiam && 
            pLoop->m_dBox[1][1] - pLoop->m_dBox[0][1] < dMinDiam   )
        {
			// 半径过滤
			pSect = pLoop->GetSectHead () ;
            for( ; pSect ; pSect = pSect->next )
            {
                 if( pSect->m_bEndFlag & NC_BLANK_SECT ) break ;
            }
            if( pSect == NULL )
            {
				// 没有发现毛坯边界过滤成功
                return TRUE ;
            }
        }
    }
	return FALSE ;
}
BOOL CSmartRPocketGen::OptimizeGroup ( CPathCombine *pTComb, CSmtCheckMdl *DriveMdl, double dDepth, double dLastH )
{
	BOOL bMinUp = FALSE ;
	int nMoveType = m_cParam.m_nMoveType ;
	if( nMoveType == NCDEF_RPOCKET_LINEAR )
	{
		if( m_cParam.m_cLinear.m_bLineFlag & NCDEF_LINEAR_MINUP )
			bMinUp = TRUE ;
	}
	else if( nMoveType == NCDEF_RPOCKET_FOLLOW )
	{
		if( m_cParam.m_cFollow.m_bFollowFlag & NCDEF_FOLLOW_MINUP )
			bMinUp = TRUE ;
	}
	else 
	{
		if( m_cParam.m_cSpiral.m_bSpiralFlag & NCDEF_SPIRAL_MINUP )
			bMinUp = TRUE ;
	}
	// 如果减少抬刀,对路径进行连接
	if( bMinUp && DriveMdl )
	{
		CRoughLink cLink( dDepth ) ;
		cLink.AddLinkPath ( DriveMdl, pTComb, m_cSetupDef.m_cTolDef, dLastH ) ;
	}
	return TRUE ;
}

int CSmartRPocketGen::SortLoopByArea ( CPathGroup &NewPath, CSmtCheckMdl *DriveMdl, int nLayer, JDNC_PRGDEF &ProgDef )
{
	CSmartLoop LoopLib, *pAllLoop = NULL, *pHead = NULL ;
	int i, Layer, nSize = 0, LineNum[5000], *pLineNum = NULL, nIndex = 0 ;
	BOOL bNewFlag = FALSE ;
	for ( i = 0; i < m_nCount; i++ )
	{
		if( !m_bLayer[i] ) continue ;
		nSize ++ ;
	}
	if( nSize > 5000 ) 
	{//防止层数大于5000数组溢出
		pLineNum = new int[nSize] ;
		bNewFlag = TRUE ;
	}
	else
	{
		pLineNum = LineNum ;
	}
	// 设置层间加工信息
	if ( m_cParam.m_cBtwRecut.m_nLayerType == NCDEF_BTWLAYER_ROUGH )
	{//粗加工
		SetBtwLoopInfor() ;
	}
	else if ( m_cParam.m_cBtwRecut.m_nLayerType == NCDEF_BTWLAYER_FINISH )
	{//精加工
	    SetBtwCurveInfor() ;
	}
	CSmtLoopPath *pLPath = NULL ;
	//得到所有的粗加工环
	for( Layer = 0 ; Layer < nLayer ; Layer++ )
	{
		if( !m_bLayer[Layer] ) continue ;
		nSize = (int)m_pLPathArr[Layer].GetSize () ;
		for( i = 0 ; i < nSize; i++ )
		{
			pLPath = m_pLPathArr[Layer].GetAt(i)  ;
			pHead = pLPath->m_pLoop ;
			for( ; pHead ; pHead = pHead->next )
			{
				pHead->m_nDepth =  INT (nIndex ) ;
			}
			if( pLPath->m_pLoop )
			{
				pAllLoop = Mini_AddContours( pAllLoop, pLPath->m_pLoop ) ;
			}
		}
		pLineNum[nIndex++] = Layer ;
	}
	// 将得到的环链进行排序
	pAllLoop = Mini_SortSRemainContours( pAllLoop, 
                                      GetLoopCheckTol(),
                                      m_pTool->m_fRadius * 2.0 ) ;
	if( m_cParam.m_cBtwRecut.m_nLayerType != NCDEF_BTWLAYER_CLOSE )
	{// 为加工环重新赋层号
		CSmartLoop * pTmpLoop = pAllLoop ;
		while ( pTmpLoop )
		{
			pTmpLoop->m_nDepth = (INT)pLineNum[pTmpLoop->m_nDepth] ;
			pTmpLoop = pTmpLoop->next ;
		}
	}
	//删除内存
	if( bNewFlag && pLineNum )
	{
		delete[] pLineNum ;
		pLineNum = NULL ;
	}

	double dDepth = m_dTopHeight - m_dBotHeight ;
	BOOL bPrev = FALSE, bAbsorb = FALSE, bZigZag = GetParamZigZag(), bBreak = FALSE ;
	PNT3D prev ;
	// 生成毛坯粗加工路径
	int nMin = 0, nPrev = 0 ;
	if( pHead ) nMin = pHead->m_nDepth ; // 记录层号
	CSmtCPathLib AllPath ;
	CPathCombine *PComb = NULL ;
	for( pHead = pAllLoop ; pHead ; pHead = pHead->next )
	{
		Layer = pHead->m_nDepth ;
	
		bAbsorb = FALSE ;
		
		nSize = (int)m_pLPathArr[Layer].GetSize () ;
		pLPath = NULL ;
		for( i = 0 ; i < nSize ; i++ )
		{
			pLPath = m_pLPathArr[Layer].GetAt ( i ) ;
			if( pHead == pLPath->m_pLoop )
				break ;
		}
		if( !pLPath || !pLPath->m_pLoop ) continue ;
		// 将路径加到路径组中
		if( NewPath.m_pHead )
		{
			NewPath.m_pTail->GetEndPoint ( 1, prev ) ;
			bPrev = TRUE ;
		}
		else
		{
			bPrev = FALSE ;
		}
		nPrev = FindPrevLayer( Layer ) ;
		// 当且仅当同一区域层数差1，才进行层间连刀优化 qqs 2014.09.06
		if( ( Layer - nMin  == 1 ) )
			bAbsorb = TRUE ;
		if( nMin == 0 || Layer == 0 ) 
		{
			dDepth = m_dTopHeight ;
		}
		else
		{
			dDepth = max( m_dZ[nMin-1], m_dZ[Layer-1] ) ;
		}
		if( !m_bBndTrim || pLPath->m_bClosedFlag )
		{
			// 生成封闭环的路径
			GenAreaPath( NewPath, DriveMdl, pHead, m_dZ[Layer], Layer, FALSE, bAbsorb ) ;
		}
		else
		{
			// 生成粗加工路径	
			pLPath->ConnectAllCurve_ArcLink ( DriveMdl, m_pPlunge[Layer], m_pCurveLib[Layer],
									bZigZag, bPrev, prev, m_cSetupDef, dDepth ) ;
			AddCutPathToAllPath( &NewPath, AllPath, DriveMdl, m_pTool->m_fRadius * 2, pLPath->m_cPathLib, m_cSetupDef.m_cTolDef, bAbsorb ) ;
			PComb = new CPathCombine( NC_WPROCESS_ROUGH ) ;
			AllPath.AddToPathCombine ( *PComb ) ;
			NewPath.AddData ( 0, PComb, TRUE ) ;
		}
		// 当前环在本函数后被删除,所以将环置为NULL 
		pLPath->m_pLoop = NULL ;
		// 层间补加工
		if ( m_cParam.m_cBtwRecut.m_nLayerType == NCDEF_BTWLAYER_FINISH )
		{// 精加工
			if ( m_cParam.m_bRPocketFlag & NCDEF_RPOCKET_CUTPLANE )
			{
				CreatePlanePath( DriveMdl, NewPath, pHead, Layer, NCDEF_RPOCKET_CAST ) ;
			}
            SortBtwCurveByArea( NewPath, DriveMdl, pHead, Layer, ProgDef ) ;
		} 
		else if ( m_cParam.m_cBtwRecut.m_nLayerType == NCDEF_BTWLAYER_ROUGH )
		{// 粗加工
			SortBtwLoopByArea( NewPath, DriveMdl, pHead, Layer, ProgDef ) ;
		}

        //过滤平面
		if( m_cParam.m_bRPocketFlag & NCDEF_RPOCKET_CUTPLANE &&
			m_cParam.m_cBtwRecut.m_nLayerType == NCDEF_BTWLAYER_CLOSE )
		{
			CreatePlanePath( DriveMdl, NewPath, pHead, Layer, NCDEF_RPOCKET_CAST ) ;
		}

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
		if( !m_bLayer[Layer] ) continue ;
		nSize = (int)m_pLPathArr[Layer].GetSize () ;
		for( i = 0 ; i < nSize; i++ )
		{
			pLPath = m_pLPathArr[Layer].GetAt(i)  ;
			pLPath->m_pLoop = NULL ;
		}
	}
	Mini_DeleteContours( pAllLoop ) ;
	if( bBreak ) return FALSE ;
	// 层间补加工,为防止遗漏,生成其余路径
	if ( m_cParam.m_cBtwRecut.m_nLayerType == NCDEF_BTWLAYER_FINISH )
	{// 层间精加工
        CreateRemainFBtwPath( DriveMdl, NewPath, nLayer, ProgDef ) ;
	}
	else if ( m_cParam.m_cBtwRecut.m_nLayerType == NCDEF_BTWLAYER_ROUGH )
	{// 层间粗加工
		CreateRemainBtwPath( DriveMdl, NewPath, nLayer, ProgDef ) ;
		ClearBtwLoop() ;
	}
	return 1 ;
}

int CSmartRPocketGen::SortBtwLoopByArea ( CPathGroup &NewPath, CSmtCheckMdl *DriveMdl, 
										  CSmartLoop *BndCurr, int nLayer, JDNC_PRGDEF &ProgDef )
{
	if( nLayer ==0 ) return 1 ;
	int i, j, nSize = 0, nLevCnt = 0 ;
	CBtwLPath * pBtwLPath = NULL ;
	CSmtLPathArr tmpLPathArr ;
	CSmtLoopPath *BndHead = NULL, *pLPath = NULL ;
	for ( i = 1; i < m_nCount ; i ++ )
	{
		if( m_bLayer[i] ) nLevCnt ++ ;
	}
	for ( i = 0; i < nLevCnt; i++ )
	{
		nSize = (int)m_pBtwLPathArr[i].GetSize() ;
		for ( j = nSize-1; j >= 0; j-- )
		{
			pBtwLPath = m_pBtwLPathArr[i].GetAt( j ) ;
			if ( IsCutBtwLoop( BndCurr, pBtwLPath ) )
			{
				BndHead = pBtwLPath->m_pLPath ;
				pBtwLPath->m_pLPath = NULL ;
				if( BndHead ) 
					tmpLPathArr.Add( BndHead ) ;
				m_pBtwLPathArr[i].RemoveAt( j ) ;
				delete pBtwLPath ;
			}
		}
	}

	int Layer = -1 , nDel = -1;
	double dDepth = 0. , dMin = 1.0e6, dDist = 0. ;
	PNT3D point ;
	BOOL bPrev = FALSE, bAbsorb = FALSE, bZigZag ;
	bZigZag = m_cParam.m_cBtwRecut.m_bBtwLayerFlag & NCDEF_BTWLAYER_ZIGZAG ;
	PNT3D prev ;
	CPathCombine *PComb = NULL ;
	CSmtCPathLib AllPath ;
	CSmtLoopPath *LPathHead = NULL, *LPathTail = NULL , *LPathFind = NULL ;
	while( 1 )
	{
		nDel = -1 ;
		dMin = 1.0e6 ;
		LPathFind = NULL ;
		NewPath.m_pTail->m_pTail->GetEndPoint ( 1, point ) ;
		nSize = (int)tmpLPathArr.GetSize() ;
		if( nSize == 0 ) break ;
		for ( i = 0; i < nSize; i++ )
		{
			pLPath = tmpLPathArr.GetAt( i ) ;
			if( m_cParam.m_cBtwRecut.m_bBtwLayerFlag & NCDEF_BTWLAYER_BOTUP )
			{//找到尾环
				while ( pLPath && pLPath->next )
				{
					pLPath = pLPath->next ;
				}
			}
			dDist = pLPath->m_pLoop->MinDistPoint( point ) ;
			if ( dDist < dMin )
			{
				dMin = dDist ;
				LPathFind = pLPath ;
				nDel = i ;
			}
		}
		if( nDel >= 0 ) tmpLPathArr.RemoveAt( nDel ) ;
		LPathHead = LPathTail = LPathFind ;
		while( LPathTail && LPathTail->next ) LPathTail = LPathTail->next ;
		while( LPathHead && LPathHead->prev ) LPathHead = LPathHead->prev ;
		//加工代码
		if( m_cParam.m_cBtwRecut.m_bBtwLayerFlag & NCDEF_BTWLAYER_BOTUP )
		{
			for( pLPath = LPathTail ; pLPath ; pLPath = pLPath->prev )
			{
				Layer = pLPath->m_pLoop->m_nDepth ;
				dDepth = m_dZ[Layer] ;
				bPrev = FALSE, bAbsorb = FALSE ;
				// 生成层间路径
				if( NewPath.m_pHead )
				{
					NewPath.m_pTail->GetEndPoint ( 1, prev ) ;
					bPrev = TRUE ;
				}
				//(1)判断是否是平面层
				int nCurLayer = pLPath->m_pLoop->m_nDepth ;
				BOOL bPlaneFlag = FALSE ;
				int nSize = (int)m_nPlaneLayer.GetSize() ;
				for ( int i = 0; i < nSize; i++ )
				{
					if( nCurLayer == m_nPlaneLayer.GetAt(i) )
					{
						bPlaneFlag = TRUE;
						break ;
					}
				}	
				//(2)判断是否与平面环相交
				if ( bPlaneFlag )
				{
					bPlaneFlag = FALSE ;
					CSmartLoop *pTmpLoop = m_pPlaneLoop ;
					while ( pTmpLoop )
					{
						if ( fabs( pTmpLoop->m_dHeight - pLPath->m_dDepth ) < 1.0e-3
							&& IsInSameArea( pLPath->m_pLoop, pTmpLoop, m_pTool->m_fRadius * 2 ) )
						{
							bPlaneFlag = TRUE ;
							break ;
						}
						pTmpLoop = pTmpLoop->next ;
					}
				}
				//(3)分情况生成层间粗加工路径
				if ( ( !bPlaneFlag || m_bBndTrim ) && !(pLPath->m_bClosedFlag) )
				{	
					pLPath->ConnectAllCurve_ArcLink ( DriveMdl, m_pPlunge[Layer], m_pCurveLib[Layer],
													bZigZag, bPrev, prev, m_cSetupDef, dDepth ) ;
					AddCutPathToAllPath( &NewPath, AllPath, DriveMdl, m_pTool->m_fRadius * 2, pLPath->m_cPathLib, m_cSetupDef.m_cTolDef, bAbsorb ) ;
					PComb = new CPathCombine( NC_WPROCESS_ROUGH ) ;
					AllPath.AddToPathCombine ( *PComb ) ;
					NewPath.AddData ( 0, PComb, TRUE ) ;
				}
				else if( bPlaneFlag )
				{
					CreatePlanePathBtwRough( DriveMdl, NewPath, pLPath ) ;
				}
				else
				{
					CreateFollowPath( DriveMdl, NewPath, pLPath ) ;
				}
				
				// 显示进度条
				if( ProgDef.m_pBrkFunc && ProgDef.m_pBrkFunc () )
				{
					return FALSE ;
				}
				ProgDef.m_dStepAt += ProgDef.m_dIncStep ;
				while( ProgDef.m_pPrgFunc && ProgDef.m_dStepAt >= ProgDef.m_dLimitAt )
				{
					ProgDef.m_pPrgFunc(1) ;
					ProgDef.m_dStepAt -= ProgDef.m_dLimitAt  ;
				}
			}
		}
		else
		{
			for( pLPath = LPathHead ; pLPath ; pLPath = pLPath->next )
			{
				Layer = pLPath->m_pLoop->m_nDepth ;
				dDepth = m_dZ[Layer] ;
				// 生成层间路径
				bPrev = FALSE, bAbsorb = FALSE ;
				// 生成层间路径
				if( NewPath.m_pHead )
				{
					NewPath.m_pTail->GetEndPoint ( 1, prev ) ;
					bPrev = TRUE ;
				}
				//(1)判断是否是平面层
				int nCurLayer = pLPath->m_pLoop->m_nDepth ;
				BOOL bPlaneFlag = FALSE ;
				int nSize = (int)m_nPlaneLayer.GetSize() ;
				for ( int i = 0; i < nSize; i++ )
				{
					if( nCurLayer == m_nPlaneLayer.GetAt(i) )
					{
						bPlaneFlag = TRUE;
						break ;
					}
				}
				//(2)判断是否与平面环相交
				if ( bPlaneFlag )
				{
					bPlaneFlag = FALSE ;
					CSmartLoop *pTmpLoop = m_pPlaneLoop ;
					while ( pTmpLoop )
					{
						if ( fabs( pTmpLoop->m_dHeight - pLPath->m_dDepth ) < 1.0e-3
							&& IsInSameArea( pLPath->m_pLoop, pTmpLoop, m_pTool->m_fRadius * 2 ) )
						{
							bPlaneFlag = TRUE ;
							break ;
						}
						pTmpLoop = pTmpLoop->next ;
					}
				}
				//(3)分情况生成层间粗加工路径
				if ( ( !bPlaneFlag || m_bBndTrim ) && !(pLPath->m_bClosedFlag) )
				{	
					pLPath->ConnectAllCurve_ArcLink ( DriveMdl, m_pPlunge[Layer], m_pCurveLib[Layer],
						                             bZigZag, bPrev, prev, m_cSetupDef, dDepth ) ;
					AddCutPathToAllPath( &NewPath, AllPath, DriveMdl, m_pTool->m_fRadius * 2, pLPath->m_cPathLib, m_cSetupDef.m_cTolDef, bAbsorb ) ;
					PComb = new CPathCombine( NC_WPROCESS_ROUGH ) ;
					AllPath.AddToPathCombine ( *PComb ) ;
					NewPath.AddData ( 0, PComb, TRUE ) ;
				}
				else if( bPlaneFlag )
				{
					CreatePlanePathBtwRough( DriveMdl, NewPath, pLPath ) ;
				}
				else
				{
					CreateFollowPath( DriveMdl, NewPath, pLPath ) ;
				}
				// 显示进度条
				if( ProgDef.m_pBrkFunc && ProgDef.m_pBrkFunc () )
				{
					return FALSE ;
				}
				ProgDef.m_dStepAt += ProgDef.m_dIncStep ;
				while( ProgDef.m_pPrgFunc && ProgDef.m_dStepAt >= ProgDef.m_dLimitAt )
				{
					ProgDef.m_pPrgFunc(1) ;
					ProgDef.m_dStepAt -= ProgDef.m_dLimitAt  ;
				}
			}
		} 
		MathCam_DeleteLPath( LPathHead ) ;
	}
	return 1 ;
}

CSmartLoop *CSmartRPocketGen::CalcCurrBtwLoop ( CSmartLoop *pCurr, CSmartLoop *&pDriveLoop, double dTol, int nLayer )
{
	if( !pCurr || !pDriveLoop ) return NULL ;
	// 得到pCurr和pDriveLoop的最大包围盒
	BOX3D box1, box2, maxbox ;
	MathCam_GetLoopBox( pCurr, &box1 ) ;
	MathCam_GetLoopBox( pDriveLoop, &box2 ) ;
	mathCalBox3DUnion( &box1, &box2, &maxbox ) ;
	
	double tol = 2 * m_pTool->m_fRadius  ;
	mathExpandBox3D( tol, &maxbox ) ;
	CSmartLoop* pLarge = new CSmartLoop() ;
	pLarge->CreateLoop ( maxbox.min , maxbox.max ) ;
	pDriveLoop = Mini_AddContours( pDriveLoop, pLarge ) ;
	
	// 得到下个加工域
	CSmartLoop loopLib, *BndHead = NULL, *pHead = NULL ;
	pDriveLoop = loopLib.BuildContour ( pDriveLoop ) ;
	
	// 在修剪之前将下个环向外偏移,扩大区域,以便于删除在直侧壁处的狭长的环
	double dRadRgh[2] = { dTol, dTol } ;
	CSmartLoop *pOffset = Mini_OffsetContourList( pDriveLoop, NCDEF_OFFSET_OUTER, dRadRgh, GetCorDef() ) ;
	if( pOffset )
	{
		Mini_DeleteContours( pDriveLoop ) ;
		pDriveLoop = pOffset ;
	}
	// 两组环求减
	BndHead = Mini_OperateContours( pCurr, pDriveLoop, 2 ) ;
	Mini_DeleteContours( pDriveLoop ) ;

	// 与边界环求交
	if( m_pAllLoop )
		pHead = Mini_OperateContours( BndHead, m_pAllLoop, 0 ) ;
	else
		pHead = Mini_CopyContours( BndHead ) ;
	Mini_DeleteContours( BndHead ) ;

	// 与毛坯环相减
	if( pHead )
	{
        BndHead = SubtractCastArea( pHead, nLayer ) ;
	}
	if( BndHead )
	{
		dRadRgh[0] = dRadRgh[1] = m_cSetupDef.m_cTolDef.m_dArcTol * 2 ;
		pOffset = Mini_OffsetContourList( BndHead, NCDEF_OFFSET_OUTER, dRadRgh, GetCorDef() ) ;
		if( pOffset )
		{
			Mini_DeleteContours( BndHead ) ;
			BndHead = pOffset ;
		}
	}
	return BndHead ;
}
/********************************************************
 * 通过深度来重新组环，生成路径。将同一深度的所有环找出 *
 * 来，生成加工路径，放到路径组当中。依次寻找，直到搜索 *
 * 完所有的环。                                         *
*********************************************************/
int CSmartRPocketGen::SortLoopByDepth( CPathGroup&  NewPath, CSmtCheckMdl *DriveMdl, int nLayer, JDNC_PRGDEF &ProgDef ) 
{
	CSmtLoopPath *pLPath = NULL, *pFind = NULL ;
	
	double dMin = 0, dDist = 0, dDepth = 0. ;
	BOOL bHavePath = FALSE ;
	PNT3D p, prev ;
	if( NewPath.m_pHead ) bHavePath = TRUE ;
	int i = 0, nFind = 0, nSize = 0, nPrev = 0 ;
	BOOL bPrev = FALSE, bAbsorb = FALSE, bZigZag = GetParamZigZag() ;
	if( !m_bLayer[nLayer] ) bZigZag = m_cParam.m_cBtwRecut.m_bBtwLayerFlag & NCDEF_BTWLAYER_ZIGZAG ;
	CPathCombine *PComb = NULL ;
	CSmtCPathLib AllPath ;
	// 最近点连接
	while( m_pLPathArr[nLayer].GetSize () > 0 )
	{
		if( !bHavePath )
		{
			pFind = m_pLPathArr[nLayer].GetAt ( 0 ) ;
			nFind = 0 ;
			bHavePath = TRUE ;
		}
		else
		{
			NewPath.m_pTail->m_pTail->GetEndPoint ( 1, p ) ;
			nSize = (int)m_pLPathArr[nLayer].GetSize () ;
			pFind = NULL ;
			nFind = -1 ;
			dMin = 1.0e6 ;
			for( i = 0 ; i < nSize ; i++ )
			{
				pLPath = m_pLPathArr[nLayer].GetAt ( i ) ;
				dDist = pLPath->m_pLoop->MinDistPoint( p ) ;
				if( dDist < dMin )
				{
					dMin = dDist ;
					nFind = i ;
					pFind = pLPath ;
				}
			}
		}
		if( !pFind || nFind == -1 ) break ;
		m_pLPathArr[nLayer].RemoveAt( nFind ) ;
		
		if( NewPath.m_pHead )
		{
			NewPath.m_pTail->GetEndPoint ( 1, prev ) ;
			bPrev = TRUE ;

		}
		else
			bPrev = FALSE ;
		nPrev = FindPrevLayer( nLayer ) ;
		if( nLayer == 0 ) 
			dDepth = m_dTopHeight ;
		else
		{
			dDepth = m_dZ[nLayer-1] ;
		}
		
		// 生成路径
		if( m_bLayer[nLayer] && !m_bBndTrim )
		{ // 粗加工。将bAbsorb设为FALSE，暂时不需在该函数内部生成快速定位路径，
		  // 而是在计算路径结束后，通过AddPlungeAndRelativeH函数生成。 qqs 2014.07.03
			GenAreaPath( NewPath, DriveMdl, pFind->m_pLoop, m_dZ[nLayer], nLayer, FALSE, FALSE ) ;
		}
		else
		{
			//(1)判断是否是平面层路径
			BOOL bPlaneFlag = FALSE ;
			int nSize = (int)m_nPlaneLayer.GetSize() ;
			for ( int i = 0; i < nSize; i++ )
			{
				if( nLayer == m_nPlaneLayer.GetAt(i) )
				{
					bPlaneFlag = TRUE;
					break ;
				}
			}
			//(2)判断是否与平面环相交
			if ( bPlaneFlag )
			{
				bPlaneFlag = FALSE ;
				CSmartLoop *pTmpLoop = m_pPlaneLoop ;
				while ( pTmpLoop )
				{
					if ( fabs( pTmpLoop->m_dHeight - pFind->m_dDepth ) < 1.0e-3
						&& IsInSameArea( pFind->m_pLoop, pTmpLoop, m_pTool->m_fRadius * 2 ) )
					{
						bPlaneFlag = TRUE ;
						break ;
					}
					pTmpLoop = pTmpLoop->next ;
				}
			}
			//(3)分情况生成层间粗加工路径
			if ( ( !bPlaneFlag || m_bBndTrim ) && !(pFind->m_bClosedFlag) )
			{
				pFind->ConnectAllCurve_ArcLink ( DriveMdl, m_pPlunge[nLayer], m_pCurveLib[nLayer],
					                           bZigZag, bPrev, prev, m_cSetupDef, dDepth ) ;
				AddCutPathToAllPath( &NewPath, AllPath, DriveMdl, m_pTool->m_fRadius * 2, pFind->m_cPathLib, m_cSetupDef.m_cTolDef, bAbsorb ) ;
				PComb = new CPathCombine( NC_WPROCESS_ROUGH ) ;
				AllPath.AddToPathCombine ( *PComb ) ;
				NewPath.AddData ( 0, PComb, TRUE ) ;
			}
			else if( bPlaneFlag )
			{
				CreatePlanePathBtwRough( DriveMdl, NewPath, pFind ) ;
			}
			else
			{
				CreateFollowPath( DriveMdl, NewPath, pFind ) ;
			}
		}			 
		Mini_DeleteContours( pFind->m_pLoop ) ;
		pFind->m_pLoop = NULL ;
		delete pFind ;
		// 显示进度条
		if( ProgDef.m_pBrkFunc && ProgDef.m_pBrkFunc () )
			return FALSE ;
		ProgDef.m_dStepAt += ProgDef.m_dIncStep ;
		while( ProgDef.m_pPrgFunc && ProgDef.m_dStepAt >= ProgDef.m_dLimitAt )
		{
			ProgDef.m_pPrgFunc(1) ;
			ProgDef.m_dStepAt -= ProgDef.m_dLimitAt  ;
		}

		//guomin 2013/04/02 修正不兜边死机
		if( !NewPath.m_pHead )
		{
			bHavePath = FALSE ;
		}
	}
	m_pLPathArr[nLayer].RemoveAll() ;
	// 添加平坦面路径
	if( m_cParam.m_bRPocketFlag & NCDEF_RPOCKET_CUTPLANE )
	{
		CreatePlanePath( DriveMdl, NewPath, NULL, nLayer, NCDEF_RPOCKET_DEPTH ) ;
	}
	return 1 ;
}
int  CSmartRPocketGen::CreateRemainBtwPath( CSmtCheckMdl *DriveMdl, CPathGroup &NewPath, int nLayer, JDNC_PRGDEF &ProgDef )
{
	CSmartLoop LoopLib, *pAllLoop = NULL, *pHead = NULL ;
	int i, Layer, nSize = 0 ;
	CSmtLoopPath *pLPath = NULL ;
	if ( m_cParam.m_cBtwRecut.m_bBtwLayerFlag & NCDEF_BTWLAYER_BOTUP )
	{
		for( Layer = nLayer -1  ; Layer >= 0 ; Layer-- )
		{
			nSize =(int) m_pLPathArr[Layer].GetSize () ;
			for( i = 0 ; i < nSize; i++ )
			{
				pLPath = m_pLPathArr[Layer].GetAt(i)  ;
				if( pLPath->m_pLoop )
				{
					pLPath->m_pLoop->m_nDepth = int( Layer ) ;
					pAllLoop = Mini_AddContours( pAllLoop, pLPath->m_pLoop ) ;
				}
			}
		}
	} 
	else
	{
		for( Layer = 0 ; Layer < nLayer ; Layer++ )
		{
			nSize = (int)m_pLPathArr[Layer].GetSize () ;
			for( i = 0 ; i < nSize; i++ )
			{
				pLPath = m_pLPathArr[Layer].GetAt(i)  ;
				if( pLPath->m_pLoop )
				{
					pLPath->m_pLoop->m_nDepth = int( Layer ) ;
					pAllLoop = Mini_AddContours( pAllLoop, pLPath->m_pLoop ) ;
				}
			}
		}
	}

	// 将得到的环链进行排序
	pAllLoop = Mini_SortSRemainContours( pAllLoop, 
                                         GetLoopCheckTol(),
                                         m_pTool->m_fRadius * 2.0 ) ;
	double dDepth = 0. ;
	BOOL bPrev = FALSE, bAbsorb = TRUE ; 
	BOOL bZigZag = GetParamZigZag(), bBreak = FALSE ;
	PNT3D prev ;
	CSmtCPathLib AllPath ;
	CPathCombine *PComb = NULL ;
	// 生成毛坯粗加工路径
	for( pHead = pAllLoop ; pHead ; pHead = pHead->next )
	{
		Layer = pHead->m_nDepth ;
		nSize = (int)m_pLPathArr[Layer].GetSize () ;
		for( i = 0 ; i < nSize ; i++ )
		{
			pLPath = m_pLPathArr[Layer].GetAt ( i ) ;
			if( pLPath->m_pLoop == pHead )
			{
				break ;
			}
		}
		if( !pLPath || !pLPath->m_pLoop ) continue ;
		
		dDepth = m_dZ[Layer] ;
		bPrev = FALSE, bAbsorb = TRUE ;
		if( NewPath.m_pHead )
		{
			NewPath.m_pTail->GetEndPoint ( 1, prev ) ;
			bPrev = TRUE ;
		}
		//(1)判断是否是平面层
		int nCurLayer = pLPath->m_pLoop->m_nDepth ;
		BOOL bPlaneFlag = FALSE ;
		int nSize = (int)m_nPlaneLayer.GetSize() ;
		for ( int i = 0; i < nSize; i++ )
		{
			if( nCurLayer == m_nPlaneLayer.GetAt(i) )
			{
				bPlaneFlag = TRUE;
				break ;
			}
		}
		//(2)判断是否与平面环相交
		if ( bPlaneFlag )
		{
			bPlaneFlag = FALSE ;
			CSmartLoop *pTmpLoop = m_pPlaneLoop ;
			while ( pTmpLoop )
			{
				if ( fabs( pTmpLoop->m_dHeight - pLPath->m_dDepth ) < 1.0e-3
					&& IsInSameArea( pLPath->m_pLoop, pTmpLoop, m_pTool->m_fRadius * 2 ) )
				{
					bPlaneFlag = TRUE ;
					break ;
				}
				pTmpLoop = pTmpLoop->next ;
			}
		}
		//(3)分情况生成层间粗加工路径
		if ( ( !bPlaneFlag || m_bBndTrim ) && !(pLPath->m_bClosedFlag) )
		{	
			pLPath->ConnectAllCurve_ArcLink ( DriveMdl, m_pPlunge[Layer], m_pCurveLib[Layer],
				                            bZigZag, bPrev, prev, m_cSetupDef, dDepth ) ;
			AddCutPathToAllPath( &NewPath, AllPath, DriveMdl, m_pTool->m_fRadius * 2, pLPath->m_cPathLib, m_cSetupDef.m_cTolDef, bAbsorb ) ;
			PComb = new CPathCombine( NC_WPROCESS_ROUGH ) ;
			AllPath.AddToPathCombine ( *PComb ) ;
			NewPath.AddData ( 0, PComb, TRUE ) ;
		}
		else if( bPlaneFlag )
		{
			CreatePlanePathBtwRough( DriveMdl, NewPath, pLPath ) ;
		}
		else
		{
			CreateFollowPath( DriveMdl, NewPath, pLPath ) ;
		}
		// 当前环在本函数后被删除,所以将环置为NULL 
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
		nSize = (int)m_pLPathArr[Layer].GetSize () ;
		for( i = 0 ; i < nSize; i++ )
		{
			pLPath = m_pLPathArr[Layer].GetAt(i)  ;
			pLPath->m_pLoop = NULL ;
		}
	}
	Mini_DeleteContours( pAllLoop ) ;
	if( bBreak ) return FALSE ;
	return TRUE ;
}
// 有毛坯的连刀路径
///////////////////////用于路径连接//////////////////////////////
void CSmartRPocketGen::AddPCombToNewPath( CPathGroup &NewPath, CSmtCheckMdl *DriveMdl, CPathCombine *PComb, BOOL bAbsorb )
{
	UNUSED_ALWAYS( DriveMdl ) ;
	UNUSED_ALWAYS( bAbsorb ) ;
	// 如果NewPath中空
	if( !NewPath.m_pHead ) 
	{
		NewPath.AddData ( 0, PComb, TRUE ) ;
		return ;
	}
}

int CSmartRPocketGen::FilterAllPlaneBnd( CSmartLoop *&PlaneHead ) 

{
	if( !PlaneHead ) return FALSE ;

   // 计算删除距离
	m_dMaxZ = m_cSetupDef.m_cTolDef.m_dArcTol * 3 ;
	if( fabs( m_cStockDef.m_dDriveOffset[0] ) < 0.0001 )
		m_dMaxZ = m_cSetupDef.m_cTolDef.m_dArcTol ;
	else
        m_dMaxZ = min( m_dStepZ * 0.2, m_dMaxZ ) ;
	
    // 对得到的环进行过滤,删除与分层相近的环
	double dTmpH = 0., dMinZ = 0. ;
	int i = 0 ;
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
			if( dMinZ < m_dMaxZ ) break ;
			// 然后处理极值情况,当两层距离非常之小,是否也应该删除?
			if( fabs( dTmpH ) < 0.0005 )
			{
				dMinZ = dTmpH ;
				break ;
			}
		}
		// 对找到的最近距离进行处理
		if( dMinZ < m_dMaxZ )
		{ // 当两个距离很近时
			bDelete = TRUE ;
			if( i == m_nCount - 1 && m_pDriveLoop && 
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
	return TRUE ;
}
void CSmartRPocketGen::TrimPlaneHeadByRough ( CNcZLevelGenMdl &heightGen, CPlaneList &cList )
{
	CPlaneLoop *pHead = NULL ;
	CSmartLoop looplib, *pRoughHead = NULL, *pCopy = NULL ;
	CSmartLoop *pOffset = NULL, *pLarge = NULL ;
	BOX3D box1, box2, maxBox ;
	DOUBLE dRadRgh[2] = { 0.002, 0.002 } ;
	JDNC_COR  CorDef = GetCorDef() ;
	CorDef.m_nCorType = NCDEF_CORNER_ARCUSER ;
	double tol = 2 * m_pTool->m_fRadius, tmpH = 0 ;
	MathCam_GetLoopBox( m_pAllLoop, &box1 ) ;
	POSITION pos, atpos = NULL ;
	pos = cList.GetHeadPosition () ;
	while( pos )
	{
		atpos = pos ;
		pHead = cList.GetNext ( pos ) ;
		// 生成毛坯环
		if( !heightGen.CreateContourLoop ( pRoughHead, pHead->m_pPlaneLoop->m_dHeight ) )
		{
			continue ;
		}
		if( !pRoughHead ) continue ;
		pCopy = Mini_CopyContours( pRoughHead ) ;
		pHead->SetRoughLoop ( pCopy ) ;

		MathCam_GetLoopBox( pRoughHead, &box2 ) ;
		mathCalBox3DUnion( &box1, &box2, &maxBox ) ;
		mathExpandBox3D( tol, &maxBox ) ;
		pLarge = new CSmartLoop() ;
		pLarge->CreateLoop ( maxBox.min, maxBox.max ) ;
		pRoughHead = Mini_AddContours( pRoughHead, pLarge ) ;
		pRoughHead = looplib.BuildContour ( pRoughHead ) ;
			
		pOffset = Mini_OffsetContourList( pRoughHead, NCDEF_OFFSET_OUTER, dRadRgh, CorDef ) ;
		if( pOffset )
		{
			Mini_DeleteContours( pRoughHead ) ;
			pRoughHead = pOffset ;
		}
	
		// 修剪
		tmpH = pHead->m_pPlaneLoop->m_dHeight ;
		pOffset = Mini_OperateContours( pHead->m_pPlaneLoop, pRoughHead, 2 ) ;
		Mini_DeleteContours( pHead->m_pPlaneLoop ) ;
		pHead->m_pPlaneLoop = NULL ;
		Mini_DeleteContours( pRoughHead ) ;
		if( pOffset )
		{
			for( pLarge = pOffset ; pLarge ; pLarge = pLarge->next )
			{
				pLarge->m_dHeight = tmpH ;
				pLarge->DefineBox () ;
			}
			pHead->SetPlaneLoop ( pOffset ) ;
		}
		else
		{
			cList.RemoveAt ( atpos ) ;
			delete pHead ;
		}
	}
	if( pRoughHead ) Mini_DeleteContours( pRoughHead ) ;

}
void CSmartRPocketGen::SearchRationalPosition ( CPlaneList &cList )
{
	double dTol = m_pTool->m_fRadius * 2 ;
	int i, j, nSize ;
	CSmartLoop *pCurrLoop = NULL ;
	CSmtLoopPath *pLPath = NULL ; 
	CPlaneLoop *pPlane = NULL ;
	//第一次查找，找下一层相关区域
	POSITION pos = cList.GetHeadPosition () ;
	while( pos )
	{
		pPlane = cList.GetNext ( pos ) ;
		for( i = 0 ; i < m_nCount ; i++ )
		{
			if( m_bLayer[i] && m_dZ[i] < pPlane->m_dHeight ) 
				break ;
		}
		if( i == m_nCount ) continue ;
	    nSize =(int) m_pLPathArr[i].GetSize () ;
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
		for ( i=0; i < m_nCount; i++ )
		{
			if ( m_bLayer[i] && m_dZ[i] < pPlane->m_dHeight )
				break ;
		}
		if( i == m_nCount || i ==0 ) continue ;
		i -- ;
		while( i >= 0 && !m_bLayer[i] ) i -- ;
		if( i < 0 ) continue ;
		nSize = (int)m_pLPathArr[i].GetSize() ;
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
void CSmartRPocketGen::CreatePlanePath ( CSmtCheckMdl *DriveMdl, CPathGroup &NewPath, CSmartLoop *CurrLoop, int nLayer, int nType )
{
	if( !NewPath.m_pHead ) return ;
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
		if ( IsCutPlaneLoop( pPlaneLoop, CurrLoop, nLayer, nType ) )
		{
		    tmpList.AddTail( pPlaneLoop ) ;
		    m_cPlaneList.RemoveAt( atpos ) ;
		}
	}

	// 然后加工tmpList的平面环
	PNT3D p ;
	INT i = 0, preLayer = 0 ;
	double dDist = 0., dMin = 1.0e6;
	while( 1 )
	{
		dMin = 1.0e6 ;
		pFindPlane = NULL ;
		atDel = NULL ;
		NewPath.m_pTail->m_pTail->GetEndPoint ( 1, p ) ;

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
		for( i = 0 ; i < m_nCount ; i++ )
		{
			if( m_dZ[i] < pFindPlane->m_dHeight )
				break ;
		}
		preLayer = i - 1 ;
		if ( m_cParam.m_cBtwRecut.m_nLayerType == NCDEF_BTWLAYER_FINISH )
		{
			while( preLayer >= 0 && !m_bLayer[preLayer] ) preLayer -- ;
		}
		if( preLayer < 0 ) dLastH = m_dTopHeight ;
		else		       dLastH = m_dZ[preLayer] ;
		GenPlaneAreaPath( NewPath, DriveMdl, pFindPlane, dLastH, pFindPlane->m_dHeight ) ;
		delete pFindPlane ;
	}
	tmpList.RemoveAll () ;
}
void CSmartRPocketGen::CreateRemainPlanePath ( CSmtCheckMdl *DriveMdl, CPathGroup &NewPath )
{
	double dLastH = 0. ;
	int i = 0, preLayer = 0 ;
	POSITION pos, atpos ;
	CPlaneLoop *pPlaneLoop = NULL ;
	pos = m_cPlaneList.GetHeadPosition () ;
	while( pos )
	{
		atpos = pos ;
		pPlaneLoop = m_cPlaneList.GetNext ( pos ) ;
		// 高度过滤
		for( i = 0 ; i < m_nCount ; i++ )
		{
			if( m_dZ[i] < pPlaneLoop->m_dHeight )
				break ;
		}
		preLayer = i - 1 ;
		if ( m_cParam.m_cBtwRecut.m_nLayerType == NCDEF_BTWLAYER_FINISH )
		{
			while( preLayer >= 0 && !m_bLayer[preLayer] ) preLayer -- ;
		}
		if( preLayer < 0 ) dLastH = m_dTopHeight ;
		else		       dLastH = m_dZ[preLayer] ;
		// 生成路径
		GenPlaneAreaPath( NewPath, DriveMdl, pPlaneLoop, dLastH, pPlaneLoop->m_dHeight ) ;
		// 删除环
		delete pPlaneLoop ;
		m_cPlaneList.RemoveAt ( atpos ) ;
	}
}
void CSmartRPocketGen::GenPlaneAreaPath ( CPathGroup &PGroup, CSmtCheckMdl *DriveMdl, CPlaneLoop *pPlane, double dLastH, double dDepth )
{
	CSmartPocketGen  PocketGen ;
	(*(CSmartPathGen*)&PocketGen ) = * this ;
    PocketGen.m_cSpeedDef.m_dPlungeDist = 0.0;

	JDNC_POCKET&   Pocket = PocketGen.m_cParam;
    Pocket.m_bPocketFlag = 0 ;
	Pocket.m_nMoveType = m_cParam.m_nMoveType  ;
	if( m_cParam.m_nMoveType == surfncMoveLinear )
    {
		Pocket.m_cLinear = m_cParam.m_cLinear ;
    }
	else if( m_cParam.m_nMoveType == surfncMoveFollow )
    {
		Pocket.m_cFollow = m_cParam.m_cFollow ;
		if( ( m_cParam.m_bRPocketFlag & NCDEF_RPOCKET_BOUNDTOL ) && 
			m_cParam.m_dBoundTol  > 0.002 )
		{
			PocketGen.m_dBoundTol = m_cParam.m_dBoundTol ;
			PocketGen.m_dFeedRatio= m_cParam.m_dFeedRatio; 
		}
    }
	else
    {
		Pocket.m_cSpiral = m_cParam.m_cSpiral ;
    }
    if( m_cParam.m_bRPocketFlag & NCDEF_RPOCKET_OPENBND )
    {
        Pocket.m_bPocketFlag |= NCDEF_POCKET_OPENBND ;
        if( Pocket.m_nMoveType == surfncMoveFollow )
        {
            Pocket.m_cFollow.m_bFollowFlag |= NCDEF_FOLLOW_OPENBND ;
        }
    }
    PocketGen.UpdateTool2D( ) ;
	/////////////////////////////////////////////////////
	double delH = dLastH - dDepth;
	CPathCombine *pTComb = new CPathCombine( NC_WPROCESS_ROUGH ) ;
	
	CSmartLoop *pRoughLoop = NULL, *pDriveLoop = NULL ;
	pRoughLoop = GetPlanePlgLoop( pPlane ) ;
	if ( pRoughLoop == NULL )
	{
		pRoughLoop = Mini_CopyContours ( pPlane->m_pRoughLoop ) ;
	}
	pDriveLoop = pPlane->m_pOrgLoop   ;
	if( !pDriveLoop && !pRoughLoop ) pDriveLoop = m_pAllLoop ;
	if( !( m_cParam.m_bRPocketFlag & NCDEF_RPOCKET_OPENBND ) || !HasBlankSectInContour( pPlane->m_pPlaneLoop ) )
	{
		Mini_DeleteContours( pRoughLoop ) ;
	}
  
	PocketGen.PocketOneContour( PGroup, *pTComb, *pPlane->m_pPlaneLoop, Pocket, 
								delH, pDriveLoop, pRoughLoop ) ; 	
	if( pTComb->m_pHead)
	{ // 如果减少抬刀
		OptimizeGroup( pTComb, DriveMdl, dDepth, dLastH ) ;
		PGroup.AddData( -dDepth , pTComb, TRUE ) ;
	}
	else 
	{
		delete pTComb  ;
	}
	if ( pRoughLoop )
	{
		Mini_DeleteContours( pRoughLoop ) ;
	}
}

BOOL CSmartRPocketGen::IsCutPlaneLoop(CPlaneLoop* PlaneLoop, CSmartLoop * CurrLoop, int nLayer, int nType)
{
	if( !PlaneLoop ) return FALSE ;

	if ( nType == NCDEF_RPOCKET_CAST )
	{
		if( !CurrLoop ) return FALSE ;
		int i, nSize = (int)PlaneLoop->m_PreLoopArr.GetSize() ;
		for( i=0; i<nSize; i++ )
		{
			CSmartLoop * pPreLoop = PlaneLoop->m_PreLoopArr.GetAt(i) ;
			if( pPreLoop == CurrLoop )
			{
				PlaneLoop->m_PreLoopArr.RemoveAt(i) ;
				break ;
			}
		}
		nSize =(int) PlaneLoop->m_PreLoopArr.GetSize() ;
		if( nSize != 0 )  return FALSE ;
		if( PlaneLoop->m_bSearch3 ) return TRUE ;
		/*以下消除安全隐患，详细参见总结*/
		PlaneLoop->m_bSearch3 = TRUE ; //设置第三次查找标记
		for ( i = 0; i < m_nCount; i++ )
		{//找到平面环所在的夹层
			if( m_bLayer[i] && m_dZ[i] < PlaneLoop->m_dHeight )
			    break ;
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
		nSize = (int)PlaneLoop->m_PreLoopArr.GetSize() ;
		if( nSize == 0 ) return TRUE ;
		else             return FALSE ;
	} 
	else if( nType == NCDEF_RPOCKET_DEPTH )
	{
		double  dTop = 0, dBot = m_dZ[nLayer] ; 
		// 高度过滤
		if( nLayer == 0 ) dTop = m_dTopHeight ;
		else			  dTop = m_dZ[nLayer-1] ;

		if ( PlaneLoop->m_dHeight > dTop || PlaneLoop->m_dHeight < dBot )
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
void CSmartRPocketGen::AddAdsorbPath ( CSmtCheckMdl *DriveMdl, CPathGroup &PGroup, CPathCombine *PComb, double dHeight )
{
	if( !PComb || !PGroup.m_pHead ) return ;
	PNT3D start, mid1, mid2, end ;
	PGroup.m_pTail->m_pTail->GetEndPoint ( 1, start ) ;
	PComb->m_pHead->GetEndPoint ( 0, end ) ;
	end[2] += dHeight ;
	nc_VectorCopy( mid1, start, 3 ) ;
	mid1[2] += 0.1 ;
	nc_VectorCopy( mid2, end, 2 ) ;
	mid2[2] = mid1[2] ;
	double dLen = DriveMdl->m_pTool->m_fRadius * 2;
	double dDist = mathDist( start, end ) ;
	if( dDist > dLen || dDist < 0.001 ) return ;
	double dDepth = max( mid1[2], mid2[2] ) + 0.1 ;

	FPNT3D fIntPnt[100] ;
	int i = 0, nCnt = 0 ;
	for( i = 0 ; i < 3 ; i++ )
	{
		fIntPnt[0][i] = TFLOAT( mid1[i] ) ;
		fIntPnt[1][i] = TFLOAT( mid2[i] ) ;
	}

	nCnt = DriveMdl->AdsorbConnectLine ( m_cSetupDef.m_cTolDef, fIntPnt, 100, TRUE ) ;
	for( i = 0 ; i < nCnt ; i++ )	
	{
		if( fIntPnt[i][2] > dDepth )
		{
			return ;
		}
	}
	if ( nCnt >= 2 )
	{
		CPathPLine3D *pLine = new CPathPLine3D() ;
		pLine->m_bFeedType = JDNC_FEEDTYPE_QUICK ;
		pLine->m_nCount = nCnt+1 ;
		pLine->m_pTAPos = new TPNT3D[nCnt+2] ;
		nc_VectorCopy( pLine->m_pTAPos[0], start, 3 ) ;
		for( i = 0 ; i < nCnt ; i++ )
		{
			pLine->m_pTAPos[i+1][0] = fIntPnt[i][0] ;
			pLine->m_pTAPos[i+1][1] = fIntPnt[i][1] ;
			pLine->m_pTAPos[i+1][2] = fIntPnt[i][2] ;
		}
		nc_VectorCopy( pLine->m_pTAPos[nCnt+1], end, 3 ) ;
		PGroup.m_pTail->AddEntity ( pLine ) ;
	}
}

void CSmartRPocketGen::AddQuickPathInSameLayer(CSmtCheckMdl *DriveMdl, CPathCombine *PComb, int nLayer)
{
	if (PComb->m_pHead == NULL) return;

	PNT3D start, end, end1;
	double dist = 0.;

	PComb->MoveCombine(0,0,m_dZ[nLayer]);
	CPathEntity* pHead = PComb->m_pHead ;
	CPathEntity* pNext = NULL;
	pHead->GetEndPoint ( 1, start ) ;
	while( pHead )
	{
		pNext = pHead->next ;
		if( !pNext ) break ;
		pNext->GetEndPoint ( 0, end ) ;
		dist = mathDist( start, end ) ;
		if (dist >= 0.0001) 
		{
			// 首先添加慢速下刀
			if( m_cSpeedDef.m_dPlungeDist > 0.01 )
			{
				mathCpyPnt( end, end1 ) ;
				end[2] += m_cSpeedDef.m_dPlungeDist ;
				CPathLine3D *pLine = new CPathLine3D( end, end1 ) ;
				pLine->m_bFeedType = JDNC_FEEDTYPE_PLUNGE ;
				PComb->InsertAfter ( pLine, pHead ) ;
			}

			CPathPLine3D *pPLine = CreateQuickMovePath(DriveMdl,start,end,nLayer);

			if( pPLine )
				PComb->InsertAfter ( pPLine, pHead ) ;
		}		
		pNext->GetEndPoint ( 1, start ) ;
		pHead = pNext ;
	}
	PComb->MoveCombine(0,0,-m_dZ[nLayer]);
}

void CSmartRPocketGen::AddQuickPathBetweenLayer ( CSmtCheckMdl *DriveMdl, CPathGroup &PGroup, CPathCombine *PComb, int nLayer )
{
	if( !PComb || !PGroup.m_pHead ) return ;
	PComb->MoveCombine(0,0,m_dZ[nLayer]);

	PNT3D start,end, end1 ;
	PGroup.m_pTail->m_pTail->GetEndPoint ( 1, start ) ;
	PComb->m_pHead->GetEndPoint ( 0, end ) ;
	// 首先添加慢速下刀
	if( m_cSpeedDef.m_dPlungeDist > 0.01 )
	{
		mathCpyPnt( end, end1 ) ;
		end[2] += m_cSpeedDef.m_dPlungeDist ;
		CPathLine3D *pLine = new CPathLine3D( end, end1 ) ;
		pLine->m_bFeedType = JDNC_FEEDTYPE_PLUNGE ;
		PComb->InsertAfter ( pLine, NULL ) ;
	}
	// nLayer用来计算当前层和当前层上一层的高度
	CPathPLine3D *pPLine = CreateQuickMovePath(DriveMdl,start,end,nLayer);

	if( pPLine )
		PComb->InsertAfter ( pPLine, NULL ) ;

	PComb->MoveCombine(0,0,-m_dZ[nLayer]);
}

// 根据起末点以及路径所在层号，计算快速定位路径 qqs 2014.09.10
CPathPLine3D * CSmartRPocketGen::CreateQuickMovePath(CSmtCheckMdl*DriveMdl,PNT3D start,PNT3D end,int nLayer)
{
	double dist = mathDist( start, end ) ;
	if (dist < 0.0001) 
		return NULL;

	PNT3D tmpStart,tmpEnd;
	mathCpyPnt( start, tmpStart ) ;
	mathCpyPnt( end  , tmpEnd ) ;

	tmpStart[2] = tmpEnd[2] = m_dZ[nLayer];
	CSmtCutPath  path( MINI_CONNECT_PATH ) ;
	path.AddPoint( tmpStart ) ;
	path.AddPoint( tmpEnd ) ;
	path.InsertCPoint ( m_cSetupDef.m_cTolDef.m_dMaxStep ) ;

	JDNC_PRGDEF cPrgDef ;
	SurfNC_InitPathParam( cPrgDef ) ;
	path.VerifyCutPath ( *DriveMdl, m_cSetupDef.m_cTolDef, cPrgDef ) ;
	path.RemovePoint(path.m_pHead);
	path.RemovePoint(path.m_pTail);
	path.DefineBox() ;

	BOOL bOverCheckMdl = FALSE;
	double h1 = path.m_fBox[1][2];     // h1 为不考虑下刀时的快速连刀路径的包围盒z值
	double h2 = max(start[2], end[2]); // h2 为连刀路径首末点的较高z值
	double dTopH = GetTopHeight() ;

	if (h1-m_dZ[nLayer]>0.002)
	{// 当h1比当前层高度要大时，说明连刀路径中间跨过了加工模型，且h1为保证加工模型不过切的高度
		bOverCheckMdl = TRUE;
	}

	// 求出连刀路径跨过加工模型时，h1所在的层或上一层高度
	if(bOverCheckMdl)
	{
		if( h1 >= (m_dZ[0]+0.002) ) 
		{
			h1 = max( h1 , dTopH   ) ;
		}
		else
		{
			for( int i = 1 ; i < m_nCount ; i++ )
			{
				if (fabs(m_dZ[i]-h1)<=0.002)
					break;						
				else if(h1>m_dZ[i])
				{
					h1 = m_dZ[i-1] ;
					break ;
				}
			}
		}
	}

	double dQuickPathH = max(h1,h2);
	if (m_cSpeedDef.m_nRapidMode == NCDEF_RAPIDMODE_ABSOLUTE)
	{// 相对毛胚高度时
		if (m_pStockBox != NULL)
		{// 使用毛胚时取二者最大值
		    dTopH = max(dTopH, m_pStockBox->max[2]);
		}
		dQuickPathH  = max(dTopH, dQuickPathH);
	}

	if (nLayer == 0)
	{
		dQuickPathH = max( dQuickPathH , dTopH   ) ;
	}
	else if (dQuickPathH < m_dZ[nLayer-1])
	{// dQuickPathH最小值需大于等于当前层上一层的深度
		dQuickPathH = m_dZ[nLayer-1];  
	}
	dQuickPathH += m_cSpeedDef.m_dRapidHeight ;
	return CreateRelativeQuick( start, end, dQuickPathH ) ;
}

int CSmartRPocketGen::FindBwtLoopIndex(int nLayer )
{
	int index = -1 ;
	for (int i=1; i<= nLayer; i++ )
	{
		if( m_bLayer[i] ) index ++ ;
	}
	return index ;
}

// 设置层间粗加工环信息
void CSmartRPocketGen::SetBtwLoopInfor()
{
	//(1) 得到粗加工路径层数
	int Layer, nLevCnt = 0;
	for ( Layer = 1; Layer < m_nCount ; Layer ++ )
	{
		if( m_bLayer[Layer] ) nLevCnt ++ ;
	}
	//(2) 查找"层间加工路径块"
	if( nLevCnt == 0 ) return ;
	CPtrArray pBtwLPathArr ;
	CBtwLPath * pBtwLPath = NULL ;
	CSmtLoopPath *pLPath = NULL, *LPathHead = NULL ;
	m_pBtwLPathArr = new CBtwLPathArr[nLevCnt] ;
	int nPrev = 0, index = 0 ;
	for ( Layer = 1; Layer < m_nCount; Layer ++ )
	{
		if( !m_bLayer[Layer] ) continue ;
		index = FindBwtLoopIndex( Layer ) ;
		nPrev = FindPrevLayer( Layer ) ;
		FindAllAreaLoop( Layer, nPrev, pBtwLPathArr ) ; 
		while ( pBtwLPathArr.GetSize() )
		{
			LPathHead = (CSmtLoopPath*)pBtwLPathArr.GetAt( 0 ); 
			pBtwLPathArr.RemoveAt( 0 ) ;
			pBtwLPath = new CBtwLPath( LPathHead ) ;
			pBtwLPath->n_UnderDepth = int( Layer ) ;
			m_pBtwLPathArr[index].Add( pBtwLPath ) ;
		}
	}
	//(3)为凹槽内"层间加工块"设置下层加工域
	double dTol = m_pTool->m_fRadius * 2 ;
	int nSize, nSize2, i, j ;
	for ( Layer = 0; Layer < nLevCnt; Layer ++ )
	{
		nSize = (int)m_pBtwLPathArr[Layer].GetSize() ;
		for ( i = 0; i < nSize ; i++ )
		{
			pBtwLPath = (m_pBtwLPathArr[Layer]).GetAt(i);
			index = pBtwLPath->n_UnderDepth ;
			nSize2 =(int) m_pLPathArr[index].GetSize() ;
			for ( j = 0 ; j < nSize2; j++ )
			{
				pLPath = m_pLPathArr[index].GetAt( j ) ;
				LPathHead = pBtwLPath->m_pLPath ;
				for( ; LPathHead; LPathHead = LPathHead->next )
				{
					if( Is2ContoursInt( pLPath->m_pLoop, LPathHead->m_pLoop, dTol ) )
					{
						pBtwLPath->AddUnderLoop( pLPath->m_pLoop ) ;
						break ;
					}
				}
			}
		}
	}
	//(4) 为凹槽内"层间加工块"设置上层加工域
	for ( Layer = 0; Layer < nLevCnt; Layer ++ )
	{
		nSize = (int)m_pBtwLPathArr[Layer].GetSize() ;
		for ( i = 0; i < nSize ; i++ )
		{
			pBtwLPath = (m_pBtwLPathArr[Layer]).GetAt(i);
			if( pBtwLPath->m_UnderLoops.GetSize() != 0 ) continue ;
			index = pBtwLPath->n_UnderDepth -1 ;
			while( (index >= 0) && !m_bLayer[index] ) index -- ;
			if( index < 0 ) continue ;
			nSize2 =(int)m_pLPathArr[index].GetSize() ;
			for ( j =0 ; j< nSize2; j++ )
			{
				pLPath = m_pLPathArr[index].GetAt( j ) ;
				LPathHead = pBtwLPath->m_pLPath ;
				for( ; LPathHead; LPathHead = LPathHead->next )
				{
					if ( IsInSameArea( pLPath->m_pLoop, LPathHead->m_pLoop, dTol ) )
					{
						pBtwLPath->AddUnderLoop( pLPath->m_pLoop ) ;
						break ;
					}
				}
			}
		}
	}
}
void CSmartRPocketGen::AddLPathToLPathArr( CSmtLPathArr &LPathArr, CSmtLoopPath *LPathHead ) 
{
	if( !LPathHead ) return ;
	CSmtLoopPath *pLNext = NULL ;
	while( LPathHead )
	{
		pLNext = LPathHead->next ;
		LPathHead->prev = LPathHead->next = NULL ;
		LPathArr.Add ( LPathHead ) ;
		LPathHead = pLNext ;
	}
}
// 判断层间加工环是否加以加工（层间粗加工使用）
BOOL CSmartRPocketGen::IsCutBtwLoop( CSmartLoop * BndCurr, CBtwLPath * &pBtwLPath )
{
	if( !BndCurr || !pBtwLPath ) return FALSE ;
	CSmartLoop * pLoop = NULL ;
	int nSize = (int)pBtwLPath->m_UnderLoops.GetSize() ;
	for ( int i = nSize-1; i >= 0; i-- )
	{
		pLoop = pBtwLPath->m_UnderLoops.GetAt( i ) ;
		if ( pLoop == BndCurr )
		{
			pBtwLPath->m_UnderLoops.RemoveAt( i ) ;
			break ;
		}
	}
	nSize = (int)pBtwLPath->m_UnderLoops.GetSize() ;
	if( nSize != 0 ) return FALSE ;
	if( pBtwLPath->m_bSearch3 ) return TRUE ;
	pBtwLPath->m_bSearch3 = TRUE ;
	double dTol = m_pTool->m_fRadius *2 ;
	for ( pLoop = BndCurr->next; pLoop; pLoop = pLoop->next )
	{
		if( pLoop->m_nDepth > pBtwLPath->n_UnderDepth ) continue ;
		CSmtLoopPath * LPathHead = pBtwLPath->m_pLPath  ;
		for ( ; LPathHead; LPathHead = LPathHead->next )
		{
			if( IsInSameArea( pLoop, LPathHead->m_pLoop, dTol ) )
			{
				pBtwLPath->AddUnderLoop( pLoop ) ;
				break ;
			}
		}
	}
	nSize = (int)pBtwLPath->m_UnderLoops.GetSize() ;
	if( nSize == 0 ) return TRUE ;
	else             return FALSE ;
}

// 清除层间粗加工环
void CSmartRPocketGen::ClearBtwLoop()
{
	if( !m_pBtwLPathArr ) return ;
	CBtwLPath * pBLPath = NULL ;
	int i = 0, j = 0, nCnt = 0 , nSize = 0 ;
	for ( i = 1; i < m_nCount; i++ )
	{
		if( m_bLayer[i] ) nCnt ++ ;
	}
	for ( i = 0; i < nCnt; i++ )
	{
		nSize =(int) m_pBtwLPathArr[i].GetSize();
		for ( j = nSize-1; j >= 0; j-- )
		{
			pBLPath = m_pBtwLPathArr[i].GetAt( j ) ;
			m_pBtwLPathArr[i].RemoveAt( j ) ;
			MathCam_DeleteLPath( pBLPath->m_pLPath ) ;
			pBLPath->m_pLPath = NULL ;
			pBLPath->m_UnderLoops.RemoveAll() ;
			delete pBLPath ;
		}
	}
	delete [] m_pBtwLPathArr ;
	m_pBtwLPathArr = NULL ;
}

void CSmartRPocketGen::SortLibPathByArea( CSmtCPathLib & AllPath ) 
{
	CSmtCPathLib TmpLib, TmpLib2;
	CSmtCutPath * pPath = NULL, *pPath2 = NULL  ;
	double dDist = 0., dDist2 = 0. ;
	int	nInc = ( m_cParam.m_cBtwRecut.m_bBtwLayerFlag & NCDEF_BTWLAYER_BOTUP)  ? 1 : (-1) ;
	POSITION pos = NULL, atpos = NULL, delPos = NULL ;
	while ( AllPath.GetNumPath() != 0 )
	{
		pos = AllPath.m_cAllPath.GetHeadPosition() ;
		pPath = AllPath.m_cAllPath.GetAt( pos ) ;
		TmpLib.AddToTail( pPath ) ;
		AllPath.m_cAllPath.RemoveAt( pos ) ;
		pos = AllPath.m_cAllPath.GetHeadPosition() ;
		while ( pos )
		{	    			
			atpos = pos ;
			dDist = 1.0e10 ;
			pPath = TmpLib.m_cAllPath.GetTail() ;
			pPath2 = AllPath.m_cAllPath.GetNext( pos ) ;
			while( pPath->m_nLayerNo == pPath2->m_nLayerNo + nInc )
			{
				dDist2 = mathFDist( pPath->m_pHead->m_fPoint, pPath2->m_pHead->m_fPoint ) ;
				if ( dDist2 < dDist)
				{
					dDist = dDist2 ;
					delPos = atpos ;
				}
				if( !pos ) break ;
				atpos = pos ;
				pPath2 = AllPath.m_cAllPath.GetNext( pos ) ;
			}

			if ( delPos )
			{
				if( pPath->m_nLayerNo != pPath2->m_nLayerNo + nInc ) 
					pos = atpos ;
				pPath = AllPath.m_cAllPath.GetAt( delPos ) ;
				AllPath.m_cAllPath.RemoveAt( delPos ) ;
				delPos = NULL ;
				TmpLib.AddToTail( pPath ) ;
			}
		}

		// 往复走刀
		if( m_cParam.m_cBtwRecut.m_bBtwLayerFlag & NCDEF_BTWLAYER_ZIGZAG )
		{
			POSITION TmpPos = TmpLib.m_cAllPath.GetHeadPosition() ;
			BOOL bReverse = FALSE ;
			while ( TmpPos )
			{
				bReverse = !bReverse ;
				pPath = TmpLib.m_cAllPath.GetNext( TmpPos ) ;
				if ( bReverse && !pPath->IsClosed() )
				{
					pPath->ReverseDirect() ;
				}
			}
		}
		TmpLib2.AppendCPathLib( TmpLib ) ;
		TmpLib.ClearAllPath() ;
	}
	AllPath.ClearAllPath() ;
	pos = TmpLib2.m_cAllPath.GetHeadPosition() ;
	while ( pos )
	{ 
		atpos = pos ;
		pPath = TmpLib.m_cAllPath.GetNext( pos ) ;
		TmpLib2.m_cAllPath.RemoveAt( atpos ) ;
		AllPath.AddToTail( pPath ) ;
	}
}

void CSmartRPocketGen::AddLeadPath(CSmtCheckMdl &DriveMdl, BOOL bCheck, CSmtCPathLib &AllPath)
{
	POSITION pos, atpos ;
	CSmtCutPath *pPath = NULL, *pNext = NULL, *pLeadIn = NULL, *pLeadOut = NULL, *pConnect = NULL ;
	double dDist = 0, dDepth = 0. ;
	double dLen = m_cFeedDef.m_cConnectDef.m_dMaxDist ;
	FPNT3D start, end, fIntpt[100] ;
	int i = 0, nCnt = 0 ;
	BOOL bConnect = TRUE;
	JDNC_TOL cTol = m_cSetupDef.m_cTolDef ;
	JDNC_CONNECT3D cConnect = m_cFeedDef.m_cConnectDef ;
	pos = AllPath.m_cAllPath.GetHeadPosition () ;
	if( !pos ) return ;
	pPath = AllPath.m_cAllPath.GetNext ( pos ) ;
	CalcCutPathLeadIn( DriveMdl, cTol, cConnect, pPath, pLeadIn, bCheck , surfncMethodRPocket) ;
	if( pLeadIn ) AllPath.m_cAllPath.AddHead ( pLeadIn ) ;

	while( pos )
	{
		atpos = pos ;
		pNext = AllPath.m_cAllPath.GetNext ( pos ) ;
		dDist = mathFDist( pPath->m_pTail->m_fPoint, pNext->m_pHead->m_fPoint ) ;
		if( dDist > 0.002  )
		{
			pLeadIn = pLeadOut = NULL ;
			CalcCutPathLeadOut( DriveMdl, cTol, cConnect, pPath, pLeadOut, bCheck, surfncMethodRPocket ) ;
			CalcCutPathLeadIn ( DriveMdl, cTol, cConnect, pNext, pLeadIn , bCheck, surfncMethodRPocket ) ;
			// 计算连刀路径起末点
			if( pLeadOut ) mathFCpyPnt( pLeadOut->m_pTail->m_fPoint, start ) ;
			else		   mathFCpyPnt( pPath->m_pTail->m_fPoint, start ) ;

			if( pLeadIn )  mathFCpyPnt( pLeadIn->m_pHead->m_fPoint, end   ) ;
			else		   mathFCpyPnt( pNext->m_pHead->m_fPoint, end ) ;

			pConnect = NULL ;
			if( dDist < dLen )
			{
				// 生成连刀线
				mathFCpyPnt( start, fIntpt[0] ) ;
				mathFCpyPnt( end  , fIntpt[1] ) ;
				dDepth = max( start[2], end[2] ) + 0.1 ;
				if( bCheck )
				{
					nCnt = DriveMdl.AdsorbConnectLine ( m_cSetupDef.m_cTolDef, fIntpt, 100, TRUE ) ;
				}
				else
				{
					nCnt = 2 ;
				}
				bConnect = TRUE ;
				for( i = 0 ; i < nCnt ; i++ )
				{
					if( fIntpt[i][2] > dDepth )
					{
						bConnect = FALSE ;
						break ;
					}
				}
				if( nCnt >= 2 && bConnect )
				{
					pConnect = new CSmtCutPath( MINI_CONNECT_PATH ) ;
					pConnect->m_bFeedType = JDNC_FEEDTYPE_CONNECT ;
					for( i = 0 ; i < nCnt ; i++ )
						pConnect->AddPoint ( fIntpt[i] ) ;
					pConnect->DefineBox () ;
				}
			}
			// 添加切入切出和连刀
			if( pLeadOut )
				AllPath.m_cAllPath.InsertBefore ( atpos, pLeadOut ) ;
			if( pConnect )
				AllPath.m_cAllPath.InsertBefore ( atpos, pConnect ) ;
			if( pLeadIn  ) 
				AllPath.m_cAllPath.InsertBefore ( atpos, pLeadIn  ) ;
		}
		pPath = pNext ;
	}
	// 添加最后一根线的切出路径
	if( pPath )
	{
		pLeadOut = NULL ;
		CalcCutPathLeadOut( DriveMdl, cTol, cConnect, pPath, pLeadOut, bCheck, surfncMethodRPocket ) ;
		if( pLeadOut ) 
			AllPath.m_cAllPath.AddTail ( pLeadOut ) ;
	}
}

// 得到所有的层间精加工曲线
void CSmartRPocketGen::GetAllBtwCurves()
{
    if( m_nCount < 2 ) return ;
    
    int i = 0, j = 0, nSize = 0 , nLevelNo = -1 ;
    CSmartLoop * pLoop = NULL , * pContour1 = NULL, * pContour2 = m_pAllLoop ;
    CSmartCurve * pHead = NULL, * pNext = NULL ;
	CSmtLoopPath *pLPath = NULL ;
    CSmartCurveLib CurLib ;
    m_pCurveArr = new CSmtCurveArr[m_nCount] ;
    
    for ( i = 0; i < m_nCount; i++ )
    {
        if( m_bLayer[i] ) continue ;
        pContour1 = m_pRoughLoop[i] ;
        nSize = (int)m_pLPathArr[i].GetSize() ;
        for ( j = nSize-1; j >= 0; j-- )
        {
            pLPath = m_pLPathArr[i].GetAt( j ) ;
			pLoop = pLPath->m_pLoop ;
			nLevelNo = pLoop->m_nDepth ;
			pLPath->m_pLoop = NULL ;
			delete pLPath ;
            m_pLPathArr[i].RemoveAt( j ) ;
            if( TrimLoopByTwoContours( pLoop, pContour1, pContour2, CurLib ) )
            {
                pHead = CurLib.m_pHead ;
                while ( pHead )
                {
                    pNext = pHead->next ;
                    CurLib.RemoveCurve( pHead ) ;
                    pHead->next = NULL ;
					pHead->m_nLevelNo = int(nLevelNo) ;
                    m_pCurveArr[i].Add( pHead ) ;
                    pHead = pNext ;
                }
            }
			Mini_DeleteContours( pLoop ) ;
        }
    }
}

// 将曲线转化成路径
CSmtCutPath * CSmartRPocketGen::TransfCurveToPath(CSmartCurve * pCurve , JDNC_TOL &cTol, double dZ, BOOL IsDel )
{
	CSmartSect * pSect = pCurve->GetHead () ;
	if( ! pSect ) return NULL ; 
	PNT4D  dPoint ;
	CSmtCutPath * pPath = new CSmtCutPath() ;
	pPath->m_nLayerNo = pCurve->m_nLevelNo ;
	dPoint[2] = dZ , dPoint[3] = 0.0 ;
	pSect->GetPoint( 0.0, dPoint ) ;
	pPath->AddPoint( dPoint ) ;
	for(  ; pSect ; pSect = pSect->next )
	{
		if( pSect->IsSectLine() )
		{
			pSect->GetPoint( 1.0, dPoint ) ;
			pPath->AddPoint( dPoint ) ;
		}
		else if( pSect->IsSectArc() ) 
		{
			CSmartArc * pArc = ( CSmartArc*) pSect ;
			PNT2D Buff[500] ;
			int nCnt = pArc->Discrete( cTol.m_dArcTol, Buff, 499 ) ;
			for( int i = 1 ; i <= nCnt ; i ++ ) 
			{
				dPoint[0] = Buff[i][0], dPoint[1] = Buff[i][1] ;
				pPath->AddPoint( dPoint ) ;
			}
		}
	}
	if( IsDel ) delete pCurve ;
	return pPath  ; 
}

/********************************************************
* 通过深度来重新组合层间精加工曲线，生成路径。将同一深度的所有曲线找出 *
* 来，生成加工路径，放到路径组当中。依次寻找，直到搜索 *
* 完所有的同层曲线。                                         *
*********************************************************/
BOOL CSmartRPocketGen::SortCurvesByDepth( CPathGroup& NewPath, CSmtCheckMdl *DriveMdl, int nLayer, CurveList & CurList )
{
	CSmartCurve *pCurve = NULL, *pFind = NULL ;
	CSmtCPathLib cLib ;
	double dMin = 0, dDist2 = 0 ;
	BOOL bHavePath = FALSE ;
	PNT3D p ;
	PNT2D start ;
	if( NewPath.m_pHead ) bHavePath = TRUE ;
	int i = 0, nFind = 0, nSize = 0 ;

	// 最近点连接
	while( m_pCurveArr[nLayer].GetSize () > 0 )
	{
		if( !bHavePath )
		{
			pFind = m_pCurveArr[nLayer].GetAt ( 0 ) ;
			nFind = 0 ;
			bHavePath = TRUE ;
		}
		else
		{
			NewPath.m_pTail->m_pTail->GetEndPoint ( 1, p ) ;
			nSize =(int) m_pCurveArr[nLayer].GetSize () ;
			pFind = NULL ;
			nFind = -1 ;
			dMin = 1.0e6 ;
			for( i = 0 ; i < nSize ; i++ )
			{
				pCurve = m_pCurveArr[nLayer].GetAt ( i ) ;
				pCurve->m_pHead->GetStart( start ) ;
				dDist2 = nc_DistanceSquare( p, start, 2 ) ;
				if( dDist2 < dMin )
				{
					dMin = dDist2 ;
					nFind = i ;
					pFind = pCurve ;
				}
			}
		}
		if( !pFind || nFind == -1 ) break ;
		m_pCurveArr[nLayer].RemoveAt( nFind ) ;
		CurList.AddTail( pFind ) ;
	}

	// 添加平坦面路径
	if( m_cParam.m_bRPocketFlag & NCDEF_RPOCKET_CUTPLANE )
	{
		CreatePlanePath( DriveMdl, NewPath, NULL, nLayer, NCDEF_RPOCKET_DEPTH ) ;
	}
	
	return TRUE ;
}

/********************************************************
* 通过区域优先来重新组合层间精加工曲线。将同一区域的所   *
* 有曲线找出来，放入层间曲线类里面中。依次寻找，直到搜索 *
* 完所有的同层曲线。                                     *
*********************************************************/
void CSmartRPocketGen::SetBtwCurveInfor()
{
	int Layer = 0,  nSize = 0, nSize2 = 0, i = 0, j = 0, k = 0,  nPrev = 0, index = 0 ;
	CurveList   pCurveList ;
	CBtwCurve * pBtwCurve = NULL ;
	CSmartCurve * pCurve = NULL ;
    CSmartLoop * pLoop = NULL ;
	CSmtLoopPath *pLPath = NULL ;
	double dDist = 0. ;
	POSITION pos = NULL ;

	// Step0 , 得到粗加工路径层数
	int nLevCnt = 0;
	for ( Layer = 1; Layer < m_nCount ; Layer ++ )
	{
		if( m_bLayer[Layer] ) nLevCnt ++ ;
	}

	// Step1, 按区域查找所有的“层间精加工块”
	if( nLevCnt == 0 ) return ;
	m_pBtwCurveArr = new CBtwCurveArr[nLevCnt] ;
	for ( Layer = 1; Layer < m_nCount; Layer ++ )
	{
		if( !m_bLayer[Layer] ) continue ;
		nPrev = FindPrevLayer( Layer ) ;
		index = FindBwtLoopIndex( Layer ) ;
        if( m_cParam.m_cBtwRecut.m_bBtwLayerFlag & NCDEF_BTWLAYER_BOTUP )
		{//从下往上加工时，从下往上查找
			for ( i = Layer-1; i > nPrev ; i-- )
			{
				while( m_pCurveArr[i].GetSize() )
				{
					pCurveList.RemoveAll() ; // 清空链表
					nSize =(int) m_pCurveArr[i].GetSize() ;
					pCurve = m_pCurveArr[i].GetAt( nSize-1 ) ;
					m_pCurveArr[i].RemoveAt( nSize-1 ) ;
					pCurveList.AddTail( pCurve ) ;
					FindAllLayerCurve( i, nPrev+1, pCurveList ) ;
					SetCurveListStart( pCurveList ) ;
					pBtwCurve = new CBtwCurve ;
					pBtwCurve->m_CurveList.AddTail( &pCurveList ) ;
					pBtwCurve->m_nUnderDepth = Layer ;
					m_pBtwCurveArr[index].Add( pBtwCurve ) ;
				}
			}
		}
		else
		{//从上往下加工时，从上往下查找
			for ( i = nPrev+1; i < Layer; i++ )
			{
				while( m_pCurveArr[i].GetSize() )
				{
					pCurveList.RemoveAll() ;
					nSize = (int)m_pCurveArr[i].GetSize() ;
					pCurve = m_pCurveArr[i].GetAt( nSize-1 ) ;
					m_pCurveArr[i].RemoveAt( nSize-1) ;
					pCurveList.AddTail( pCurve ) ;
					FindAllLayerCurve( Layer-1, i, pCurveList ) ;
					SetCurveListStart( pCurveList ) ;
					pBtwCurve = new CBtwCurve ;
					pBtwCurve->m_CurveList.AddTail( &pCurveList ) ;
					pBtwCurve->m_nUnderDepth = Layer ;
					m_pBtwCurveArr[index].Add( pBtwCurve ) ;
				}
			}
		}
	}
	// Step2, 为所有的层间精加工块设置与其相关联的粗加工区域
	BOOL bAdd = FALSE ;
	CSmartSect * pSect = NULL ;
	PNT2D point ;
	CSmartLoop * pIsland = NULL ;
	for ( Layer = 0; Layer < nLevCnt; Layer ++ )
	{//为"层间加工块"设置相关的粗加工域
		nSize = (int)m_pBtwCurveArr[Layer].GetSize() ;
		for ( i = 0; i < nSize ; i++ )
		{
			pBtwCurve = (m_pBtwCurveArr[Layer]).GetAt(i);
			if( pBtwCurve->m_CurveList.GetCount() == 0 ) continue ;
			for ( k = 0; k <= pBtwCurve->m_nUnderDepth; k++ )
			{
				nSize2 = (int)m_pLPathArr[k].GetSize() ;
				for ( j =0 ; j< nSize2; j++ )
				{
					pLPath = m_pLPathArr[k].GetAt( j ) ;
					pLoop = pLPath->m_pLoop ;
					bAdd = FALSE ;
					// step1 依次判断曲线链表是否和环距离小于刀具直径
					pos = pBtwCurve->m_CurveList.GetHeadPosition() ;
					while( pos )
					{
						pCurve = pBtwCurve->m_CurveList.GetNext( pos ) ;
						dDist = MinDistBetweenLoopAndCurve( pLoop, pCurve ) ;
						if( dDist < m_pTool->m_fRadius * 2 )
						{
							bAdd = TRUE ;
							break ;
						}
					}
					if ( bAdd )
					{
						pBtwCurve->AddUnderLoop( pLoop ) ;
                        continue ;
					}
					// step2 若距离大于刀具直径，再判断是否落在环组成的区域内
                    pCurve = pBtwCurve->m_CurveList.GetHead() ;
					pSect = pCurve->m_pHead ;
					pSect->GetPoint( 0.5, point ) ;
                    if ( pLoop->IsPointIn( point ) )
                    {// 链表在环内
						bAdd = TRUE ;
						pIsland = pLoop->m_pIsland ;
						for( ; pIsland; pIsland = pIsland->next )
						{// 判断链表是否在某个孤岛内
							if ( pIsland->IsPointIn( point ) )
							{
								bAdd = FALSE ;
								break ;
							}
						}
						if ( bAdd )
						{
							pBtwCurve->AddUnderLoop( pLoop ) ;
						}
                    }// if
				}// for j
			}// for k			
		}//for i
	}//for layer 
}

// 查找两粗加工层之间在同一区域内的曲线，并放入曲线链表pCurveList里面
void CSmartRPocketGen::FindAllLayerCurve(int Layer,int Pre, CurveList & pCurveList )
{
	if( pCurveList.GetSize() == 0 ) return ;
    
	double dDist = 0. ;
	POSITION atPos = NULL ;
    int i = 0, j = 0 , nSize = 0 ;
	CSmartCurve *pCurve = NULL , *pCurve2 = NULL;
	if( m_cParam.m_cBtwRecut.m_bBtwLayerFlag & NCDEF_BTWLAYER_BOTUP )
	{//从下往上加工时，从下往上搜索
		for ( i = Layer; i >= Pre; i-- )
		{
			nSize = (int)m_pCurveArr[i].GetSize() ;
			for ( j = nSize-1; j >= 0; j -- )
			{
				pCurve = m_pCurveArr[i].GetAt( j ) ;
				atPos = pCurveList.GetTailPosition() ;
				while( atPos )
				{
					pCurve2 = pCurveList.GetPrev( atPos ) ;
					dDist = MinDistBetweenCurve( pCurve, pCurve2 ) ;
					if ( dDist < m_pTool->m_fRadius * 2 )
					{
						pCurveList.AddTail( pCurve ) ;
						m_pCurveArr[i].RemoveAt( j ) ;
						break ;
					}
				}
			}
		}
	}
	else
	{// 从上往下加工时,从上往下搜索
		for ( i = Pre; i <= Layer; i++ )
		{
			nSize = (int)m_pCurveArr[i].GetSize() ;
			for ( j = nSize - 1 ; j >= 0; j-- )
			{
				pCurve = m_pCurveArr[i].GetAt(j ) ;
				atPos = pCurveList.GetTailPosition() ;
				while( atPos)
				{
					pCurve2 = pCurveList.GetPrev( atPos ) ;
					dDist = MinDistBetweenCurve( pCurve, pCurve2 ) ;
					if ( dDist < m_pTool->m_fRadius * 2 )
					{
						pCurveList.AddTail( pCurve ) ;
						m_pCurveArr[i].RemoveAt( j ) ;
						break ;
					}
				}
			}
		}
	}
}

/********************************************************
* 找出所有与粗加工域BndCurr相关的层间精加工曲线，*
* 放到链表数组当中。依次寻找，直到搜索完所有的层 *
* 间精加工曲线，最后生成路径。                                 *
*********************************************************/
int CSmartRPocketGen::SortBtwCurveByArea( CPathGroup &NewPath, CSmtCheckMdl *DriveMdl, CSmartLoop *BndCurr, int nLayer, JDNC_PRGDEF &ProgDef )
{
	if( nLayer ==0 ) return 1 ;
	int i, j, nSize = 0, nLevCnt = 0 , index = -1 ;
	CBtwCurve * pBtwCurve = NULL ;
    CurveList  pCurveList[5000] ;
	CSmtCPathLib AllPath ;

	for ( i = 1; i < m_nCount ; i ++ )
	{// 计算粗加工环的层数
		if( m_bLayer[i] ) nLevCnt ++ ;
	}
	for ( i = 0; i < nLevCnt; i++ )
	{// 得到当前可加工的曲线链表
		nSize = (int)m_pBtwCurveArr[i].GetSize() ;
		for ( j = nSize-1; j >= 0; j-- )
		{
			pBtwCurve = m_pBtwCurveArr[i].GetAt( j ) ;
			if ( IsCutBtwCurve( BndCurr, pBtwCurve ) )
			{
				pCurveList[++index].AddTail( &(pBtwCurve->m_CurveList) ) ;
				m_pBtwCurveArr[i].RemoveAt( j ) ;
				delete pBtwCurve ;
			}
		}
	}
	for ( i = 0; i <= index ; i++ )
	{// 把曲线转化成路径
		TransfCurveListToPath( NewPath, DriveMdl, pCurveList[i], ProgDef ) ;
	}
	return 1 ;
}

// 判断当前层间曲线是否可以加工（层间精加工使用）
BOOL CSmartRPocketGen::IsCutBtwCurve( CSmartLoop * BndCurr, CBtwCurve * pBtwCurve )
{
	if( !BndCurr || !pBtwCurve ) return FALSE ;
	int nSize = 0 , i=0  ;
	CSmartLoop * pLoop = NULL ;
	nSize = (int)pBtwCurve->m_UnderLoops.GetSize() ;
	for ( i = nSize-1; i >= 0; i-- )
	{
		pLoop = pBtwCurve->m_UnderLoops.GetAt( i ) ;
		if ( pLoop == BndCurr )
		{
			pBtwCurve->m_UnderLoops.RemoveAt( i ) ;
			break ;
		}
	}
    //防止没有找到相关粗加工域的层间路径加工
	if( nSize == 0 ) return FALSE ; 

	nSize = (int)pBtwCurve->m_UnderLoops.GetSize() ;
    if ( nSize == 0 )
    {
		return TRUE ;
    } 
    else
    {
		return FALSE ;
    }
}

// 生成剩余的层间精加工路径
int CSmartRPocketGen::CreateRemainFBtwPath( CSmtCheckMdl *DriveMdl, CPathGroup &NewPath, int nLayer, JDNC_PRGDEF &ProgDef )
{
    int i = 0, j = 0, nLevCnt = 0 , nSize = 0 ;
	CBtwCurve * pBtwCurve = NULL ;
	CurveList pCurveList ;

	for ( i = 1; i < nLayer ; i ++ )
	{// 计算粗加工环的层数
		if( m_bLayer[i] ) nLevCnt ++ ;
	}
    
	for ( i = 0; i < nLevCnt; i++ )
	{
        nSize = (int)m_pBtwCurveArr[i].GetSize() ;
		for ( j = nSize-1; j >= 0; j-- )
		{
			pBtwCurve = m_pBtwCurveArr[i].GetAt( j ) ;
			m_pBtwCurveArr[i].RemoveAt( j ) ;
            pCurveList.AddTail( &(pBtwCurve->m_CurveList) ) ;
			TransfCurveListToPath( NewPath, DriveMdl, pCurveList, ProgDef ) ;
			delete pBtwCurve ;
		}
		m_pBtwCurveArr[i].RemoveAll() ;
	}
	delete [] m_pBtwCurveArr ;
	m_pBtwCurveArr = NULL ;

	return 1 ;
}

// 把曲线链表里的曲线转化成路径，包含连刀
BOOL CSmartRPocketGen::TransfCurveListToPath( CPathGroup &NewPath, CSmtCheckMdl *DriveMdl, CurveList & pCurveList , JDNC_PRGDEF &ProgDef )
{
    if( pCurveList.GetCount() == 0 ) return FALSE ;

	JDNC_TOL tol = m_cSetupDef.m_cTolDef ;
	CSmartCurve * pCurve = NULL ;
	CSmtCPathLib AllPath ;
	CSmtCutPath * pPath = NULL ;
	POSITION atPos = NULL , toPos = NULL ;
	atPos = pCurveList.GetHeadPosition() ;
	while( atPos ) 
	{
		toPos = atPos ;
		pCurve = pCurveList.GetNext( atPos ) ;
		pCurveList.RemoveAt( toPos ) ;
		pPath = TransfCurveToPath( pCurve, tol, m_dZ[pCurve->m_nLevelNo], TRUE ) ;
		if( !pPath ) continue ;
		AllPath.AddToTail( pPath ) ;

		// 进度显示
		if( ProgDef.m_pBrkFunc && ProgDef.m_pBrkFunc () )
		{
			return FALSE ;
		}
		ProgDef.m_dStepAt += ProgDef.m_dIncStep ;
		while( ProgDef.m_pPrgFunc && ProgDef.m_dStepAt >= ProgDef.m_dLimitAt )
		{
			ProgDef.m_pPrgFunc(1) ;
			ProgDef.m_dStepAt -= ProgDef.m_dLimitAt  ;
		}
	}
	// 区域排序
	if( GetOrderDef().m_nLayerOrd == 0 ) 
	{
		SortLibPathByArea( AllPath ) ;
	}
	//添加连刀路径
	AddLeadPath( *DriveMdl, TRUE, AllPath ) ;
	if( AllPath.GetNumPath() == 0 ) return FALSE;
	CPathCombine * pComb = new CPathCombine( NC_WPROCESS_ROUGH ) ;
	AllPath.AddToPathCombine( * pComb ) ;
	NewPath.AddData( 0.0, pComb ) ;
	return TRUE ;
}

// 清除所有的层间精加工曲线
void CSmartRPocketGen::ClearAllBtwCurves()
{
	if( !m_pCurveArr ) return ;
	int i = 0, j = 0, nSize = 0 ;
	CSmartCurve * pCurve = NULL ;
	for ( i = 0; i < m_nCount; i++ )
	{
		nSize = (int)m_pCurveArr[i].GetSize() ;
		for ( j = nSize - 1; j >= 0; j-- )
		{
			pCurve = m_pCurveArr[i].GetAt( j ) ;
			m_pCurveArr[i].RemoveAt( j ) ;
			delete pCurve ;
		}
		m_pCurveArr[i].RemoveAll() ;
	}
    delete [] m_pCurveArr ;
	m_pCurveArr = NULL ;
}

CSmartLoop * CSmartRPocketGen::CalcAllCutArea ( CSmartLoop *AreaLoop,		// <I> 加工区域
												CSmartLoop *AllLoop,		// <I> 原始边界线
												CSmartLoop *RoughLoop,	 	// <I> 毛坯区域)
												BOOL		bDeleRough	)	// <I> 删除毛坯
{
	CSmartLoop *BndHead = NULL ;
	if( m_cParam.m_bRPocketFlag & NCDEF_RPOCKET_OPENBND && m_bAllLoop && RoughLoop )
	{
		double dRadius[2] = { m_pRoughTool->m_fRadius + 0.05, m_pRoughTool->m_fRadius + 0.05 } ;
		// 毛坯环多偏一点，因为毛坯区域已经被build过了，所以需要向外偏移
		CSmartLoop *pOffset = Mini_OffsetContourList( RoughLoop, NCDEF_OFFSET_OUTER, dRadius, m_cSetupDef.m_cCorDef ) ;
		if( bDeleRough )
		{
			Mini_DeleteContours( RoughLoop ) ;
		}
		CSmartLoop *AllBnd = Mini_CopyContours( AllLoop ) ;
	
		CSmartCurveLib CurLib ;
		// STEP 0 : 轮廓初始化, 轮廓求交
		CSmartLoop *pLoopA = pOffset, *pLoopB = AllBnd ;
		for( ; pLoopA ; pLoopA = pLoopA->next )
		{
			pLoopA->InitBuffer() ;
		}
		for( ; pLoopB ; pLoopB = pLoopB->next )
		{
			pLoopB->InitBuffer() ;
		}
		for( pLoopA = pOffset ; pLoopA ; pLoopA = pLoopA->next )
		{
			for( pLoopB = AllBnd ; pLoopB ; pLoopB = pLoopB->next )
			{
				pLoopA->ContourContourInt( pLoopB );
			}
		}
		// STEP 1 轮廓B打断
		CSmartCurve *pCurveHead = NULL, *pCurveTemp = NULL ;
		PNT2D dMidPnt, start, end ;
		int bIntFlag = 0 ;
		for( pLoopB = AllBnd ; pLoopB ; pLoopB = pLoopB->next )
		{
			pCurveHead = pLoopB->ContourBreak();
			for(  ; pCurveHead  ; pCurveHead = pCurveTemp )
			{
				pCurveTemp = pCurveHead->next ;
				pCurveHead->next = NULL ;
				pCurveHead->GetMidPoint( dMidPnt ) ;
				pCurveHead->GetStart ( start ) ;
				pCurveHead->GetEnd ( end ) ;
				bIntFlag = Mini_IsPointOnContours( pOffset, dMidPnt ) ;
				if( bIntFlag < 1 )   //(不落在毛坯区域里 )    
				{ 
				}
				else 
				{
					pCurveHead->SetBlank( TRUE ) ;
				}
				CurLib.AddCurves ( pCurveHead ) ;
			} 
		}
//		// STEP 3 : 连接曲线
		// 删除原始的AllBnd
		Mini_DeleteContours( AllBnd ) ;
		AllBnd = FormAllCurveToContour( CurLib ) ;

////////////////////////////////////////////////////////////////////
		// test 查看AllBnd的段以及首末店
/*		if( AllBnd )
		{
			for( pLoopA = AllBnd ; pLoopA ; pLoopA = pLoopA->next )
			{
				pCurveHead = pLoopA->m_pCurve ;
				for( ; pCurveHead ; pCurveHead = pCurveHead->next)
				{
					CSmartSect *pHead = pCurveHead->m_pHead ;
					for( ; pHead; pHead = pHead->next )
					{
						pHead->GetStart( start ) ;
						pHead->GetEnd  ( end   ) ;
						if( pHead->m_bEndFlag & NC_BLANK_SECT )
						{
							bIntFlag = 1 ;
						}
					}
				}
			}
		}*/
/////////////////////////////////////////////////////////////////////
		BndHead = Mini_OperateContours( AreaLoop, AllBnd, 0 ) ;
		////////////////////////////////////////////////////////////////////
		// test 查看AllBnd的段以及首末店
/*		if( AreaLoop )
		{
			for( pLoopA = AreaLoop ; pLoopA ; pLoopA = pLoopA->next )
			{
				pCurveHead = pLoopA->m_pCurve ;
				for( ; pCurveHead ; pCurveHead = pCurveHead->next)
				{
					CSmartSect *pHead = pCurveHead->m_pHead ;
					for( ; pHead; pHead = pHead->next )
					{
						pHead->GetStart( start ) ;
						pHead->GetEnd  ( end   ) ;
						if( pHead->m_bEndFlag & NC_BLANK_SECT )
						{
							bIntFlag = 1 ;
						}
					}
				}
			}
		}*/
/////////////////////////////////////////////////////////////////////
		// 删除偏移环
		Mini_DeleteContours( pOffset ) ;
		Mini_DeleteContours( AllBnd ) ;
	}
	else
	{
		BndHead = Mini_OperateContours( AreaLoop, AllLoop, 0 ) ;
		//边界等距裁剪死机问题修改 liuyan、qqs、xiejunxian 2013.03.18
		if (bDeleRough)
			Mini_DeleteContours( RoughLoop ) ;
	}
	return BndHead ;
}

////////////////////////////////////////////////////////////////////
// 毛坯边界修剪方式生成路径
/////////////////////////////////////////////////////////////////////
BOOL CSmartRPocketGen::SubtractAllLoopNormal ( CSmtCheckMdl *CheckMdl, JDNC_PRGDEF &ProgDef, double dCur )
{
	UNUSED_ALWAYS( CheckMdl ) ;
	CSmartLoop *pDriveHead = NULL, *BndHead = NULL ;
	ProgDef.m_dLimitAt =  m_nCount / dCur ;
	ProgDef.m_dStepAt  = 0.0 ;
	ProgDef.m_dIncStep = 1.0  ;
	// 将两把刀具的环进行相减,将得到的结果放到当前的数组里面
	for( int i = 0 ; i < m_nCount ; i++ )
	{
		// 显示进度条
		if( ProgDef.m_pBrkFunc && ProgDef.m_pBrkFunc () )
			return FALSE ;
		ProgDef.m_dStepAt += ProgDef.m_dIncStep ;
		while( ProgDef.m_pPrgFunc && ProgDef.m_dStepAt >= ProgDef.m_dLimitAt )
		{
			ProgDef.m_pPrgFunc(1) ;
			ProgDef.m_dStepAt -= ProgDef.m_dLimitAt  ;
		}

		pDriveHead = RemoveOneLayerLoop( m_pLPathArr[i] ) ;
		
		// 如果是当前刀的环,并且有毛坯模型,并且非层间环用当前刀的环减去毛坯环
		BndHead = SubtractCastArea( pDriveHead, i ) ;
		if( !BndHead ) continue ;
		if( m_pAllLoop )
		{
			pDriveHead = Mini_OperateContours( BndHead, m_pAllLoop, 0 ) ;
			Mini_DeleteContours( BndHead ) ;
			BndHead = pDriveHead ;
		}
		AddLoopToLPArr( CheckMdl, m_pLPathArr[i], BndHead, i, FALSE ) ;
	}
	return TRUE ;
}
BOOL CSmartRPocketGen::SubtractAllLoopForOpti ( CSmtCheckMdl *CheckMdl, CPathGroup &NewPath, JDNC_PRGDEF &ProgDef, double dCur )
{
	UNUSED_ALWAYS(NewPath);
	CSmartLoop LoopLib, *pDriveHead = NULL, *pLastHead = NULL ;
	CSmartLoop *pResult = NULL, *BndHead = NULL ;
	ProgDef.m_dLimitAt =  m_nCount / dCur ;
	ProgDef.m_dStepAt  = 0.0 ;
	ProgDef.m_dIncStep = 1.0  ;

	CSmartLoop * pHaveLandLoop = NULL, *pNoLandLoop = NULL ;
	BOOL bRet = FALSE, bContinue = FALSE ;
	int i = 0, curLayer = 0 , preLayer = 0 ;
	int nType = NCDEF_OFFSET_OUTER ; 
	// 将两把刀具的环进行相减,将得到的结果放到当前的数组里面
	for( i = 0 ; i < m_nCount ; i++ )
	{
		// 显示进度条
		if( ProgDef.m_pBrkFunc && ProgDef.m_pBrkFunc () )
			return FALSE ;
		ProgDef.m_dStepAt += ProgDef.m_dIncStep ;
		while( ProgDef.m_pPrgFunc && ProgDef.m_dStepAt >= ProgDef.m_dLimitAt )
		{
			ProgDef.m_pPrgFunc(1) ;
			ProgDef.m_dStepAt -= ProgDef.m_dLimitAt  ;
		}
		if( !m_bLayer[i] && NCDEF_BTWLAYER_FINISH == m_cParam.m_cBtwRecut.m_nLayerType )
		{
			continue ;
		}
		
		if( m_bLayer[i] )
		{
			pDriveHead = RemoveOneLayerLoop( m_pLPathArr[i] ) ;
			if( m_bBndTrim )
			{
				m_pPlunge[i] = Mini_CopyContours( pDriveHead ) ;
				pLastHead  = RemoveOneLayerLoop( m_pRoughCastLoopArr[i] ) ;
				// 残补环和粗加工环相减
				pResult = Mini_OperateContours( pDriveHead, pLastHead, 2 ) ;
				if( !pResult )
				{
					Mini_DeleteContours( pDriveHead ) ;
					Mini_DeleteContours( pLastHead ) ;
					continue ;
				}
				// 结果和边界环求交
//				BndHead = Mini_OperateContours( pResult, m_pAllLoop, 0 ) ;
				BndHead = CalcAllCutArea( pResult, m_pAllLoop, pLastHead, FALSE ) ;
				Mini_DeleteContours( pResult ) ;
				if( !BndHead ) 
				{
					Mini_DeleteContours( pDriveHead ) ;
					Mini_DeleteContours( pLastHead ) ;
					continue ;
				}
				AddLoopToLPArr( CheckMdl, m_pLPathArr[i], BndHead, i , FALSE ) ;
				bContinue = TRUE ;
				nType = NCDEF_OFFSET_OUTER ;
				Mini_DeleteContours( pDriveHead ) ;
				pDriveHead = Mini_CopyContours( m_pDriveLoop[i] ) ;
				pDriveHead = MathCAM_ExtractAllLoop( pDriveHead ) ;
				pDriveHead = LoopLib.BuildContour ( pDriveHead ) ;
			}
			else
			{
				// 如果是当前刀的环,并且有毛坯模型,并且非层间环用当前刀的环减去毛坯环
				BndHead = SubtractCastArea( pDriveHead, i ) ;
/////////////////////////////////////////////////////////////////////////////////////
				// for test 
//				AddContourToGroup( BndHead, m_dZ[i], NewPath ) ;
//////////////////////////////////////////////////////////////////////////////////////
				if( !BndHead ) 	continue ;
				if( m_pAllLoop && m_pDriveLoop[i] )
				{
				//	pDriveHead = Mini_OperateContours( BndHead, m_pAllLoop, 0 ) ;
					pDriveHead = CalcAllCutArea( BndHead, m_pAllLoop, CopyOneLayerLoop( m_pRoughCastLoopArr[i] ), TRUE ) ;
					Mini_DeleteContours( BndHead ) ;
					BndHead = pDriveHead ;
				}
				AddLoopToLPArr( CheckMdl, m_pLPathArr[i], BndHead, i, FALSE ) ;
				bContinue = FALSE ;
			}
		}
		else
		{	
			pDriveHead = CopyOneLayerLoop( m_pLPathArr[i] ) ;
			m_pPlunge[i] = Mini_CopyContours( pDriveHead ) ;
			pLastHead = pDriveHead ;
			
			pDriveHead = Mini_CopyContours( m_pDriveLoop[i] ) ;
			pDriveHead = MathCAM_ExtractAllLoop( pDriveHead ) ;
			pDriveHead = LoopLib.BuildContour ( pDriveHead ) ;
			
			SetContourBlank( pLastHead ) ;
			bContinue = TRUE ;
			nType = NCDEF_OFFSET_OUTER ;
			//把和等高环没有重合部分的环单独拿出来放入pNoLandLoop
			CSmartLoop * pLoop = Mini_CopyContours ( pLastHead ) , * pNext = NULL ;
			while ( pLoop )
			{
				pNext = pLoop->next ;
				pLoop->next = pLoop->prev = NULL ;
				if ( IsRelatedLoop( pLoop, pDriveHead, 0.1 ) )
				{
					pHaveLandLoop = Mini_AddContours( pHaveLandLoop, pLoop ) ;
				}
				else
				{
					pNoLandLoop = Mini_AddContours( pNoLandLoop, pLoop ) ;
				}
				pLoop = pNext ;
			}
			pHaveLandLoop = LoopLib.BuildContour ( pHaveLandLoop ) ;
			pNoLandLoop = LoopLib.BuildContour ( pNoLandLoop ) ;
		}
		if( !bContinue ) continue ;
		// 依次偏移pDriveHead,将得到的环同pOffset相减,将得到的曲线添加到m_pLPathArr[i]中
		double dOffSet[2] = { 0., 0. } ;
		int nLevel = 0 ;
		CSmartCurveLib CurLib ;
		BOOL bCreateAreaPath = FALSE;
		if (pDriveHead == NULL && pLastHead != NULL && m_bBndTrim)
		{// 当只有毛坯环，没有等高环时并且开启边界等距修剪时，直接使用毛坯环生成环切路径 qqs 2014.05.04
			bCreateAreaPath = TRUE;
			goto EndLine;
		}
		while( 1 )
		{
			if( !pDriveHead ) break ;
			pResult = OffsetContourList( pDriveHead, nType , dOffSet, m_dMinArea, m_cSetupDef.m_cCorDef ) ;
			if( nLevel == 0 && m_cParam.m_bRPocketFlag & NCDEF_RPOCKET_BOUNDTOL )
				dOffSet[0] = dOffSet[1] = m_cParam.m_dBoundTol ;
			else
			{
				if( m_bLayer[i] )	// 基于裁减的环切
					dOffSet[0] = dOffSet[1] = m_cFeedDef.m_cStepDef.m_dOverStep ;
				else				// 层间补加工
					dOffSet[0] = dOffSet[1] = m_cParam.m_cBtwRecut.m_dMaxSideStep ;
			}
			if( !pResult ) break ;
			if( m_bLayer[i] )
			{
				bRet = SubtractContour( pResult, pLastHead, m_pAllLoop, CurLib, 0.02, FALSE ) ;
			}
			else
			{
				bRet = SubtractContour( pResult, pHaveLandLoop, m_pAllLoop, CurLib, 0.02, TRUE ) ;
			}
			Mini_DeleteContours( pDriveHead ) ;
			if ( 1 == nLevel && !m_bBndTrim)
			{//设置路径封闭标记
				SetPathClosedFlag( m_pLPathArr[i], CurLib ) ;
			}
			else if( 0 == nLevel && !bRet && m_bBndTrim && m_bAllLoop )
			{
				// 第一次就没有合理的区域产生，应该是加工域和m_pAllLoop没有相交的环导致，需要多次偏移才能得到
				// 不如直接生成路径。2017.07.22
				bCreateAreaPath = TRUE ;
				break ;
			}
			pDriveHead = pResult ;
			if( bRet )
			{// 边界等距修剪时，需去掉路径闭合标记，添加闭合标记会使闭
			 //	合部分的路径按照非边界等距修剪方式生成。 qqs 2014.04.25			 
				if( ! AddCurveToLPath( m_pLPathArr[i], CurLib, nLevel, !m_bBndTrim))
				{
					CurLib.DeleteAllCurves () ;
					break ;
				}
				nLevel++ ;
			}
			else
			{
				break ;
			}
		}
		if( pDriveHead ) Mini_DeleteContours( pDriveHead ) ;
		//对特殊环进行处理,生成丢失的层间路径(2010.6.11)
		if( !m_bLayer[i] )
		{
			//(1) 拷贝没有生成路径的环
			pDriveHead = Mini_CopyContours( pNoLandLoop ) ;
			//(2) 对得到的环向里偏移,并进行修剪
			nType = NCDEF_OFFSET_INNER ;
			dOffSet[0] = dOffSet[1] = 0.0 ;
			CCurveLibArr CuvLibArr ;
			while( 1 )
			{
				if( !pDriveHead ) break ;
				pResult = OffsetContourList( pDriveHead, nType , dOffSet, m_dMinArea, m_cSetupDef.m_cCorDef ) ;
				dOffSet[0] = dOffSet[1] = m_cParam.m_cBtwRecut.m_dMaxSideStep ;
				if( !pResult ) break ;
				CSmartCurveLib *pLib = new CSmartCurveLib() ;
				bRet = SubtractContour( pResult, pNoLandLoop, m_pAllLoop, *pLib, 0.02, TRUE ) ;
				Mini_DeleteContours( pDriveHead ) ;
				pDriveHead = pResult ;
				if( bRet )
					CuvLibArr.Add( pLib ) ; //添加到数组内
				else
					break ;
			}
			//(3) 把得到的曲线加到路径组内
			CSmartCurve * pCurve = NULL ;
			int nLibCnt = (int)CuvLibArr.GetSize() ;
			for ( int j = nLibCnt-1; j >= 0 ; j -- )
			{
				CSmartCurveLib *pLib = CuvLibArr.GetAt( j ) ;
				if( pLib ) pCurve = pLib->m_pHead ;
				if ( nLibCnt-2 == j )
				{//设置封闭路径标记
					SetPathClosedFlag( m_pLPathArr[i], *pLib ) ;
				}
				while ( pCurve )
				{
					pCurve->Reverse() ;
					pCurve = pCurve->next ;
				}
				if( !AddCurveToLPath( m_pLPathArr[i],  *pLib, nLibCnt-1-j) )
				{
					pLib->DeleteAllCurves () ;
				}
				CuvLibArr.RemoveAt( j ) ;
				if( pLib ) delete pLib ;
			}
			//(4)删除内存
			if( pDriveHead ) Mini_DeleteContours( pDriveHead ) ;
			if( pHaveLandLoop ) Mini_DeleteContours( pHaveLandLoop ) ;
			if( pNoLandLoop ) Mini_DeleteContours( pNoLandLoop) ;
		}
		
		if( !m_bLayer[i] )
		{
			curLayer = i + 1 ;
			if ( !( m_cParam.m_cBtwRecut.m_bBtwLayerFlag & NCDEF_BTWLAYER_BOTUP) )
			{
				while( !m_bLayer[curLayer] ) curLayer ++ ;
			}
			CopyPlungeCurve( m_pDriveLoop[curLayer], m_pCurveLib[i] ) ;
		}
		else
		{
			CopyPlungeCurve( m_pRoughLoop[i], m_pCurveLib[i] ) ;
		}
EndLine:		
		Mini_DeleteContours( pLastHead ) ;
		// 生成环切路径
		JDNC_PLUNGE cPlunge = m_cFeedDef.m_cPlungeDef ;
		preLayer = i - 1 ;
		if ( m_bLayer[i] || m_cParam.m_cBtwRecut.m_bBtwLayerFlag & NCDEF_BTWLAYER_BOTUP )
		{
			while ( preLayer >=0 && !m_bLayer[preLayer] )
				preLayer -- ;
		}
		if( preLayer < 0 )
			cPlunge.m_dIncStep = m_dTopHeight - m_dZ[i] ;
		else
			cPlunge.m_dIncStep = m_dZ[preLayer] - m_dZ[i] ;

		int k = 0,  nSize = (int)m_pLPathArr[i].GetSize () ;
		for( k = 0 ; k < nSize ; k++ )
		{
			CSmtLoopPath *pLPath = m_pLPathArr[i].GetAt ( k ) ;
			// 如果优化,进行最近连刀
			if( pLPath->m_cLibArr.GetSize () > 0 )
				pLPath->SetPlungeInfo ( cPlunge ) ;
			else 
			{
				if( bCreateAreaPath || !GetDriveCurvePath( pLPath, int( i ) ) )
					CreateAreaPath( CheckMdl, pLPath, int( i ) ) ;
			}
		}
	}
	//防止计算误差引起的进度条少走一步
	if( ProgDef.m_pPrgFunc && ( ProgDef.m_dStepAt/ProgDef.m_dLimitAt ) > 0.1 )
	{
		ProgDef.m_pPrgFunc( 1 ) ;
	}
	return TRUE ;
}
//设置路径封闭标记
BOOL CSmartRPocketGen::SetPathClosedFlag(CSmtLPathArr& LPathArr, CSmartCurveLib& CurLib )
{
	int i, nSize =(int) LPathArr.GetSize () ;
	CSmtLoopPath *pLPath = NULL ;
	CSmartCurve *pHead , *pNext = NULL ;
	PNT2D mid ;
	for( i = 0 ; i < nSize ; i ++ )
	{
		pLPath = LPathArr.GetAt ( i ) ;
		pHead = CurLib.m_pHead ;
		while( pHead )
		{
			pNext = pHead->next ;
			pHead->GetMidPoint ( mid ) ;
			if( IsPointOnContours ( pLPath->m_pLoop, mid ) )
			{
				if ( pHead->IsClosed() )
				{
					pLPath->m_bClosedFlag = TRUE ;
					break ;
				}
			}
			pHead = pNext ;
		}
	}
	return TRUE ;
}
void CSmartRPocketGen::CreateAreaPath( CSmtCheckMdl *DriveMdl, CSmtLoopPath *pLPath, int nLayer )
{
	if( !pLPath->m_pLoop ) return ;
	CSmartPocketGen  PocketGen ;
	(*(CSmartPathGen*)&PocketGen ) = * this ;
	PocketGen.m_cSpeedDef.m_dPlungeDist = 0. ;

	JDNC_POCKET&   Pocket = PocketGen.m_cParam;
    Pocket.m_bPocketFlag = 0 ;
	Pocket.m_nMoveType = m_cParam.m_nMoveType  ;
	if( m_cParam.m_nMoveType == surfncMoveLinear )
		Pocket.m_cLinear = m_cParam.m_cLinear ;
	else if( m_cParam.m_nMoveType == surfncMoveFollow )
	{
		Pocket.m_cFollow = m_cParam.m_cFollow ;
		if( ( m_cParam.m_bRPocketFlag & NCDEF_RPOCKET_BOUNDTOL ) &&
			m_cParam.m_dBoundTol  > 0.002 )
		{
			PocketGen.m_dBoundTol = m_cParam.m_dBoundTol ;
			PocketGen.m_dFeedRatio= m_cParam.m_dFeedRatio; 
		}
	}
	else
		Pocket.m_cSpiral = m_cParam.m_cSpiral ;
    if( m_cParam.m_bRPocketFlag & NCDEF_RPOCKET_OPENBND )
    {
        Pocket.m_bPocketFlag |= NCDEF_POCKET_OPENBND ;
        if( Pocket.m_nMoveType == surfncMoveFollow )
        {
            Pocket.m_cFollow.m_bFollowFlag |= NCDEF_FOLLOW_OPENBND ;
        }
    }
    PocketGen.m_cParam = Pocket ;
    PocketGen.UpdateTool2D( ) ;
	
	/////////////////////////////////////////////////////
	double delH = 0., dLastH = 0 ;
	if( nLayer == 0 )
	{
		delH = m_dTopHeight - m_dZ[nLayer] ;
		dLastH = m_dTopHeight + m_cSpeedDef.m_dPlungeDist ;
	}
	else
	{
		delH = m_dZ[nLayer-1] - m_dZ[nLayer] ;
		dLastH = m_dZ[nLayer-1] ;
	}
	
	CPathCombine *pTComb = new CPathCombine( NC_WPROCESS_ROUGH ) ;
	CSmartLoop *pRoughLoop = NULL, *pDriveLoop = NULL ;
	pRoughLoop = m_pRoughLoop[nLayer] ;
	pDriveLoop = m_pDriveLoop[nLayer] ;
	BOOL bNewLoop = FALSE ;
	if( !pDriveLoop && !pRoughLoop ) pDriveLoop = m_pAllLoop ;
	if( !( m_cParam.m_bRPocketFlag & NCDEF_RPOCKET_OPENBND ) )
	{
		pRoughLoop = NULL ;
	}
	CPathGroup PGroup( NC_PATHTYPE_RSURFACE ) ;
	PocketGen.PocketOneContour( PGroup, *pTComb, *pLPath->m_pLoop, Pocket, delH, pDriveLoop, pRoughLoop ) ; 
	if( bNewLoop ) Mini_DeleteContours( pRoughLoop ) ;

	if( pTComb->m_pHead)
	{
		// 如果减少抬刀
		if( nLayer >= 0 )
            OptimizeGroup( pTComb, DriveMdl, m_dZ[nLayer], dLastH ) ;

		TransfPCombineToCPath( *pTComb, pLPath->m_cPathLib, m_cSpeedDef.m_dFeedRate, m_dZ[nLayer], m_cSetupDef.m_cTolDef ) ;
		delete pTComb ;
	}
	else 
	{
		delete pTComb  ;
	}	
}
//对"封闭形"加工域,用马蹄形路径代替修剪模式路径
void CSmartRPocketGen::CreateFollowPath(CSmtCheckMdl *DriveMdl, CPathGroup &NewPath, CSmtLoopPath *&pLPath )
{
	if ( !pLPath || !pLPath->m_pLoop )
	{//加工域为空,返回
		return ;
	}
	CSmartPocketGen  PocketGen ;
	(*(CSmartPathGen*)&PocketGen ) = * this ;
	PocketGen.m_cSpeedDef.m_dPlungeDist = 0.0;

	int nLayer = pLPath->m_pLoop->m_nDepth, nPrev = nLayer ;
	double dDepth = pLPath->m_dDepth , dLastH = 0.0, delH = 0.0;
	JDNC_FOLLOW  FollowCut ;
	SurfNC_InitPathParam( FollowCut ) ;
	if ( m_bLayer[nLayer] )
	{
		FollowCut.m_dOverStep  = m_cFeedDef.m_cStepDef.m_dOverStep ;
		FollowCut.m_dOverRatio = m_cFeedDef.m_cStepDef.m_dOverRatio ;
	} 
	else
	{
		FollowCut.m_dOverStep  = m_cParam.m_cBtwRecut.m_dMaxSideStep ;
		PocketGen.m_cFeedDef.m_cStepDef.m_dOverStep = m_cParam.m_cBtwRecut.m_dMaxSideStep ;
	}
	FollowCut.m_nFollowType= 0 ; 
	FollowCut.m_bFollowFlag = NCDEF_FOLLOW_MINUP ;		// 最少抬刀
	if( m_cFeedDef.m_cSlotDef.m_nSlotType != NCDEF_TSLOT_CLOSE ) 
		FollowCut.m_bFollowFlag |= NCDEF_FOLLOW_SLOTCUT ;
	// 环切清角并光滑圆角
	FollowCut.m_bFollowFlag |= NCDEF_FOLLOW_CORNER ;
	FollowCut.m_bFollowFlag |= NCDEF_FOLLOW_SMOOTH ;

	JDNC_POCKET&   Pocket = PocketGen.m_cParam;
	Pocket.m_nMoveType = surfncMoveFollow  ;
	Pocket.m_cFollow = FollowCut ;
	PocketGen.UpdateTool2D( ) ;

	if ( m_bLayer[nLayer] )
	{
		nPrev = nLayer - 1 ;
		while ( nPrev >=0 && !m_bLayer[nPrev] ) nPrev -- ;
	} 
	else
	{
		if ( m_cParam.m_cBtwRecut.m_bBtwLayerFlag & NCDEF_BTWLAYER_BOTUP )
		{
			while( nPrev >= 0 && !m_bLayer[nPrev] ) nPrev -- ;		
		}
		else
		{
			nPrev -- ;
		}
	}
	if( nPrev >=0 ) dLastH = m_dZ[nPrev] ;
	else                dLastH = m_dTopHeight ;
	delH = dLastH - dDepth;
	CPathCombine *pTComb = new CPathCombine( NC_WPROCESS_ROUGH ) ;
	CSmartLoop *pRoughLoop = NULL, *pDriveLoop = NULL ;
	pDriveLoop = pLPath->m_pPtLoop ;
	if ( m_pBtwPlgLayer )
	{
		pRoughLoop =  GetBtwPlgLoop( pLPath->m_pLoop ) ;
	} 
	if( pRoughLoop == NULL )
	{
		pRoughLoop = Mini_CopyContours ( m_pRoughLoop[nLayer] );
	}
	if( !pDriveLoop && !pRoughLoop ) pDriveLoop = m_pAllLoop ;
	PocketGen.PocketOneContour( NewPath, *pTComb, *pLPath->m_pLoop, Pocket, 
		delH, pDriveLoop, pRoughLoop ) ; 	
	if( pTComb->m_pHead)
	{ // 如果减少抬刀
		OptimizeGroup( pTComb, DriveMdl, dDepth, dLastH ) ;
		NewPath.AddData( -dDepth , pTComb, TRUE ) ;
	}
	else 
	{
		delete pTComb  ;
	}
	if ( pRoughLoop )
	{//删除内存
		Mini_DeleteContours( pRoughLoop ) ;
	}
}

void CSmartRPocketGen::AddSectToPath( CSmartSect *pSect, CSmtCutPath *pPath, int nLayer )
{
	if( !pSect || !pPath ) return ;
	PNT2D start = { 0., 0. }, end = { 0., 0. } ;
	PNT2D Buff[1000] ;
	int i = 0, nCnt = 0 ;
	PNT3D tmp = { 0., 0., m_dZ[nLayer] } ;
	double dArc = min( m_cSetupDef.m_cTolDef.m_dArcTol, 0.005 ) ;
	if( pSect->IsSectArc () )
	{
		CSmartArc * pArc = ( CSmartArc*) pSect ;
		nCnt = pArc->Discrete ( dArc, Buff, 999 ) ;
		if( pPath->m_nNumPnt < 1 )
		{
			tmp[0] = Buff[0][0], tmp[1] = Buff[0][1] ;
			pPath->AddPoint ( tmp ) ;
		}
		for( i = 1 ; i <= nCnt ; i++ )
		{
			tmp[0] = Buff[i][0], tmp[1] = Buff[i][1] ;
			pPath->AddPoint ( tmp ) ;
		}
	}
	else
	{
		pSect->GetStart ( start ) ;
		pSect->GetEnd   ( end   ) ;
		if( pPath->m_nNumPnt < 1 )
		{
			tmp[0] = start[0], tmp[1] = start[1] ;
			pPath->AddPoint ( tmp ) ;
		}
		tmp[0] = end[0], tmp[1] = end[1] ;
		pPath->AddPoint ( tmp ) ;
	}
}
BOOL CSmartRPocketGen::GetDriveCurvePath( CSmtLoopPath *pLPath, int nLayer )
{
	if( !pLPath->m_pLoop ) return FALSE ;
	CSmartLoop *BndHead = pLPath->m_pLoop ;
	// 首先判断该区域经过偏移之后是否还有区域
	double dOffset[2] = { m_cFeedDef.m_cStepDef.m_dOverStep, m_cFeedDef.m_cStepDef.m_dOverStep } ;

	CSmartLoop *OffsetCont = Mini_OffsetContourList( BndHead, NCDEF_OFFSET_INNER, dOffset, m_cSetupDef.m_cCorDef ) ;
	if( OffsetCont )
	{
		// 经过偏移，有路径生成，则环切生成路径
		Mini_DeleteContours( OffsetCont ) ;
		return FALSE ;
	}
	// 环
	CSmtCutPath *pPath = new CSmtCutPath() ;
	CSmartSect *pHead = BndHead->GetSectHead (), *pNext = NULL ;
	// for test //
	PNT2D start, end ;
	int nCnt = 0 ;
	for( ; pHead ; pHead = pHead->next )
	{
		pHead->GetStart ( start ) ;
		pHead->GetEnd ( end ) ;
		if( pHead->m_bEndFlag & NC_BLANK_SECT )
			nCnt++ ;
	}
	pHead = BndHead->GetSectHead () ;
	while( pHead )
	{
		pNext = pHead->next ;
		if( ! ( pHead->m_bEndFlag & NC_BLANK_SECT ) )
		{
			AddSectToPath( pHead, pPath, nLayer ) ;
		}
		else
		{
			if( pPath->m_nNumPnt > 1 )
			{
				pLPath->m_cPathLib.AddToTail ( pPath ) ;
				pPath = new CSmtCutPath() ;

			}
		}
		pHead = pNext ;
	}
	if( pPath->m_nNumPnt > 1 )
	{
		pLPath->m_cPathLib.AddToTail ( pPath ) ;
		pPath = new CSmtCutPath() ;
	}
	// 岛
	CSmartLoop *pIsland = BndHead->GetIsland () ;
	for( ; pIsland ; pIsland = pIsland->next )
	{
		pHead = pIsland->GetSectHead() ;
		while( pHead )
		{
			pNext = pHead->next ;
			if( ! ( pHead->m_bEndFlag & NC_BLANK_SECT ) )
			{
				AddSectToPath( pHead, pPath, nLayer ) ;	
			}
			else
			{
				if( pPath->m_nNumPnt > 1 )
				{
					pLPath->m_cPathLib.AddToTail ( pPath ) ;
					pPath = new CSmtCutPath() ;
				}
			}
			pHead = pNext ;
		}
		if( pPath->m_nNumPnt > 1 )
		{
			pLPath->m_cPathLib.AddToTail ( pPath ) ;
			pPath = new CSmtCutPath() ;
		}
	}
	if( pPath->m_nNumPnt < 2 )
		delete pPath ;
	if( pLPath->m_cPathLib.GetNumPath () > 0 )
	{
		pLPath->m_bAddPath = TRUE ;
		return TRUE ;
	}
	return FALSE ;
}
void CSmartRPocketGen::CopyPlungeCurve ( CSmartLoop *pLast, CSmartCurveLib& BndCurve )
{
	if( !pLast ) return ;
	double r = min( m_pTool->m_fRadius, 0.4 ) ;
	double dRadRgh[2] = { r, r } ;
	JDNC_COR  CorDef = GetCorDef() ;
	CorDef.m_nCorType = NCDEF_CORNER_ARCUSER ;
	CSmartLoop *pCopy = Mini_OffsetContourList( pLast, 
												NCDEF_OFFSET_OUTER,
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
void CSmartRPocketGen::AddLoopToLPArr( CSmtCheckMdl *CheckMdl,CSmtLPathArr& LPathArr, CSmartLoop* pHead, int Layer, BOOL bAddPath )
{
	if( !pHead ) return ;
	CSmtLoopPath *pLPath = NULL ;
	CSmartLoop* pLoop = pHead ;
	CSmartLoop* pN = NULL ;
	PNT2D p ;
	double r = min( m_pTool->m_fRadius, 0.4 ) ;
	double dStep = m_cFeedDef.m_cStepDef.m_dOverStep ;
	if( !m_bLayer[Layer] ) dStep = m_cParam.m_cBtwRecut.m_dMaxSideStep ;
	while( pLoop  )
	{
		pN = pLoop->next ;
		pLoop->prev = pLoop->next = NULL ;
        pLoop->m_nDepth = int( Layer ) ;
		pLoop->m_pCurve->GetStart ( p ) ;
		pLPath = new CSmtLoopPath( pLoop, m_cFeedDef, m_dZ[Layer], r, dStep, FALSE ) ;
		
		if( m_cParam.m_bRPocketFlag  & NCDEF_RPOCKET_BOUNDTOL )
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
			CreateAreaPath( CheckMdl, pLPath, int( Layer ) ) ;
			pLPath->m_bAddPath = TRUE ;
		}
		LPathArr.Add ( pLPath ) ;
		pLoop = pN ;
	}
}

BOOL CSmartRPocketGen::GetParamZigZag() 
{
	BOOL bZigZag = FALSE ;
	if( m_cParam.m_nMoveType == surfncMoveLinear )
	{
		if( m_cParam.m_cLinear.m_bLineFlag & NCDEF_LINEAR_ZIGZAG )
			bZigZag = TRUE ;
	}
	else if( m_cParam.m_nMoveType == surfncMoveFollow )
	{
		if( m_cParam.m_cFollow.m_bFollowFlag & NCDEF_FOLLOW_ZIGZAG )
			bZigZag = TRUE ;
	}
	else if( m_cParam.m_nMoveType == surfncMoveSpiral )
	{
		if( m_cParam.m_cSpiral.m_bSpiralFlag & NCDEF_SPIRAL_ZIGZAG )
			bZigZag = TRUE ;
	}
	return bZigZag ;
}
//当选择过滤平面和层间粗加工路径时的分层方式
void CSmartRPocketGen::SetCutStepBtwLayerRough( CSmartLoop * &PlaneLoop )
{
	//STEP 1:得到平面环的高度数组
	double *dPlane_Z = NULL ;
	int nPlaneZCnt = 0 ;
	GetAllBndContHeight( PlaneLoop, dPlane_Z, nPlaneZCnt ) ;
	FilterPlaneZHeight( dPlane_Z, nPlaneZCnt ) ;
	//STEP 2:得到总层数
	int nLayer = m_nCount , nSt = 0, nEnd = 0 ;
	double dVSt = 0.0, dVEnd = 0.0 ;
	double dVStep = 0.0, dTop = m_dTopHeight ;
	for ( int i = 0; i < m_nCount; i++ )
	{
		if( i == 0 ) dVSt = dTop ;
		else           dVSt = m_dZ[i-1] ;
		dVEnd = m_dZ[i] ;
		nSt = nEnd = -1 ;
		int j = 0 ;
		while ( j < nPlaneZCnt && dPlane_Z[j] > dVSt )	j ++ ;
		nSt = j ;
		while( j < nPlaneZCnt && dPlane_Z[j] > dVEnd ) j++ ;
		nEnd = j-1 ;
		for ( j = nSt; j <= nEnd; j++ )
		{
			if( j == nSt ) dVStep = dVSt - dPlane_Z[j] ;
			else               dVStep = dPlane_Z[j-1] - dPlane_Z[j] ;
			int n = (int)ceil( dVStep / m_cParam.m_cBtwRecut.m_dMaxVertStep ) ;
			nLayer += n ;
		}
		if( nEnd >= nSt ) dVStep = dPlane_Z[nEnd] - dVEnd ;
		else                  dVStep = dVSt - dVEnd ;
		int n =  (int)ceil( dVStep / m_cParam.m_cBtwRecut.m_dMaxVertStep ) ;
		nLayer += n-1 ;
	}
	// STEP 3:为各层赋高度值,并设置层间标记
	double *dZ = new double[nLayer] ;
	double dSt, dEnd ;
	m_bLayer = new BOOL[nLayer] ;
	memset( m_bLayer, 0, (nLayer) * sizeof(BOOL) ) ;
	nLayer = -1 ;
	for ( int i = 0 ; i < m_nCount; i++ )
	{
		if( i == 0 ) dVSt = dTop ;
		else           dVSt = m_dZ[i-1] ;
		dVEnd = m_dZ[i] ;
		nSt = nEnd = -1 ;
		int j = 0 ;
		while ( j < nPlaneZCnt && dPlane_Z[j] > dVSt )	j ++ ;
		nSt = j ;
		while( j < nPlaneZCnt && dPlane_Z[j] > dVEnd ) j++ ;
		nEnd = j-1 ;
		for ( j = nSt; j <= nEnd; j++ )
		{
			if( j == nSt ) dSt = dVSt ;
			else              dSt = dPlane_Z[j-1] ;
			dEnd = dPlane_Z[j] ;
			dVStep = dSt - dEnd ;
			int n = (int)ceil( dVStep / m_cParam.m_cBtwRecut.m_dMaxVertStep ) ;
			for ( int k = 1; k <= n; k ++ )
			{
				nLayer ++ ;
				dZ[nLayer] = dSt - k*dVStep/n ;
				if( k == n ) m_nPlaneLayer.Add( (int)nLayer ) ;
			}
		}
		if( nEnd >= nSt ) dVSt = dPlane_Z[nEnd] ;
		dVStep = dVSt - dVEnd ;
		int n =  (int)ceil( dVStep / m_cParam.m_cBtwRecut.m_dMaxVertStep ) ;
		for ( int k = 1; k <= n; k ++ )
		{
			nLayer ++ ;
			dZ[nLayer] = dVSt - k*dVStep/n ;
		}
		m_bLayer[nLayer] = TRUE ;
	}
	//STEP 4:替换原有分层
	delete[] m_dZ; 
	m_dZ = dZ ;
	m_nCount = nLayer +1 ;
	//STEP 5:删除内存
	if( dPlane_Z ) delete [] dPlane_Z ;
	m_pPlaneLoop = PlaneLoop ;
	PlaneLoop = NULL ;
}

//生成层间粗加工路径的平面层路径
void CSmartRPocketGen::CreatePlanePathBtwRough(CSmtCheckMdl *DriveMdl, CPathGroup &NewPath, CSmtLoopPath *&pLPath )
{
	CSmartPocketGen  PocketGen ;
	(*(CSmartPathGen*)&PocketGen ) = * this ;
	PocketGen.m_cSpeedDef.m_dPlungeDist = 0.0;

	JDNC_POCKET&   Pocket = PocketGen.m_cParam;
	Pocket.m_bPocketFlag = 0 ;
	Pocket.m_nMoveType = m_cParam.m_nMoveType  ;
	if( m_cParam.m_nMoveType == surfncMoveLinear )
	{
		Pocket.m_cLinear = m_cParam.m_cLinear ;
		Pocket.m_cLinear.m_dOverStep = m_cParam.m_cBtwRecut.m_dMaxSideStep ;
	}
	else if( m_cParam.m_nMoveType == surfncMoveFollow )
	{
		Pocket.m_cFollow = m_cParam.m_cFollow ;
		if( ( m_cParam.m_bRPocketFlag & NCDEF_RPOCKET_BOUNDTOL ) && 
			m_cParam.m_dBoundTol  > 0.002 )
		{
			PocketGen.m_dBoundTol = m_cParam.m_dBoundTol ;
			PocketGen.m_dFeedRatio= m_cParam.m_dFeedRatio; 
		}
		Pocket.m_cFollow.m_dOverStep = m_cParam.m_cBtwRecut.m_dMaxSideStep ;
		PocketGen.m_cFeedDef.m_cStepDef.m_dOverStep = m_cParam.m_cBtwRecut.m_dMaxSideStep ;
	}
	else
	{
		Pocket.m_cSpiral = m_cParam.m_cSpiral ;
		Pocket.m_cSpiral.m_dOverStep = m_cParam.m_cBtwRecut.m_dMaxSideStep ;
	}
	if( m_cParam.m_bRPocketFlag & NCDEF_RPOCKET_OPENBND )
	{
		Pocket.m_bPocketFlag |= NCDEF_POCKET_OPENBND ;
		if( Pocket.m_nMoveType == surfncMoveFollow )
		{
			Pocket.m_cFollow.m_bFollowFlag |= NCDEF_FOLLOW_OPENBND ;
		}
	}
	PocketGen.UpdateTool2D( ) ;
	
	int nLayer = pLPath->m_pLoop->m_nDepth, nPrev = nLayer ;
	double dDepth = pLPath->m_dDepth , dLastH = 0.0, delH = 0.0;

	if ( m_cParam.m_cBtwRecut.m_bBtwLayerFlag & NCDEF_BTWLAYER_BOTUP )
	{
		while( nPrev >= 0 && !m_bLayer[nPrev] ) nPrev -- ;		
	}
	else
	{
		 nPrev -- ;
	}
	if( nPrev >=0 )dLastH = m_dZ[nPrev] ;
	else                dLastH = m_dTopHeight ;
	delH = dLastH - dDepth;

	CPathCombine *pTComb = new CPathCombine( NC_WPROCESS_ROUGH ) ;
	CSmartLoop *pRoughLoop = NULL, *pDriveLoop = NULL ;
	pDriveLoop = pLPath->m_pPtLoop ;
	if ( m_pBtwPlgLayer  )
	{
		pRoughLoop =  GetBtwPlgLoop(  pLPath->m_pLoop ) ;
	} 
	if( pRoughLoop == NULL )
	{
		pRoughLoop = Mini_CopyContours ( m_pRoughLoop[nLayer] );
	}
	if( !pDriveLoop && !pRoughLoop ) pDriveLoop = m_pAllLoop ;
	PocketGen.PocketOneContour( NewPath, *pTComb, *pLPath->m_pLoop, Pocket, 
												delH, pDriveLoop, pRoughLoop ) ; 	
	if( pTComb->m_pHead)
	{ // 如果减少抬刀
		OptimizeGroup( pTComb, DriveMdl, dDepth, dLastH ) ;
		NewPath.AddData( -dDepth , pTComb, TRUE ) ;
	}
	else 
	{
		delete pTComb  ;
	}
	if ( pRoughLoop )
	{
		Mini_DeleteContours( pRoughLoop ) ;
	}
}

//过滤平面环的高度,去掉与主分层很近的高度
BOOL CSmartRPocketGen::FilterPlaneZHeight(double *& dPlane_Z, int & nPlaneZCnt )
{
	if( !dPlane_Z || nPlaneZCnt == 0) return FALSE ;
	//STEP 0:计算删除距离
	m_dMaxZ = m_cSetupDef.m_cTolDef.m_dArcTol * 3 ;
	if( fabs( m_cStockDef.m_dDriveOffset[0] ) < 0.0001 )
		m_dMaxZ = m_cSetupDef.m_cTolDef.m_dArcTol ;
	else
		m_dMaxZ = min( m_dStepZ * 0.2, m_dMaxZ ) ;
	//STEP 1:找到要删除的平面层高度值
	double dDet = 0.0 , dSt = 0.0, dEnd = 0.0 ;
	int nSt, nEnd, i = 0, j = 0 ;
	BOOL *bFlag = new BOOL[nPlaneZCnt] ;
	if( !bFlag ) return FALSE ;
	memset( bFlag, 0, nPlaneZCnt * sizeof(BOOL) ) ;
	for ( i = -1; i < m_nCount-1; i++ )
	{
		if( i == -1 ) dSt = m_dTopHeight ;
		else          dSt = m_dZ[i] ;
		dEnd = m_dZ[i+1] ;
		nSt = nEnd = -1 ;
		while ( j < nPlaneZCnt && dPlane_Z[j] > dSt ) j++ ;
		nSt = j ;
		while( j < nPlaneZCnt && dPlane_Z[j] > dEnd ) j++ ;
		nEnd = j-1 ;
		for ( j = nSt; j <= nEnd; j++ )
		{
			dDet = dSt - dPlane_Z[j] ;
			if( dDet < m_dMaxZ ) bFlag[j] = TRUE ;
		}
	}
	//STEP 2:删除掉不要的平面分层
	for ( i = 0, j = 0 ; i < nPlaneZCnt ; i++ )
	{
		if( bFlag[i] == FALSE )
		{
			if( j != i )
			{
				dPlane_Z[j] = dPlane_Z[i] ;
			}
			j++;
		}
	}
	nPlaneZCnt = j ;
	//STEP 3:删除内存
	if( bFlag ) delete[] bFlag ;

	return TRUE ;
}

//标记粗加工的最底层,即从下往上裁减环第一个不为空的层(2010.6.19).
BOOL CSmartRPocketGen::MarkBottomLayer()
{
	if ( m_cParam.m_cBtwRecut.m_nLayerType != NCDEF_BTWLAYER_ROUGH ) 
		return TRUE ;

	CSmartLoop *pResult = NULL, *BndHead = NULL, *pLoop=NULL ;
	int i = -1 ;
	for(  i = m_nCount-1 ; i >= 0 ;  )
	{
		//(1)得到该层的裁减环
		for( int j = 0 ; j < m_pLPathArr[i].GetSize () ; j++ )
		{
			pLoop = Mini_CopyContours( m_pLPathArr[i].GetAt(j)->m_pLoop );
			if( !pLoop ) continue ;
			BndHead = Mini_AddContours( BndHead, pLoop ) ;
		}
		//(2)减去毛坯区域
		BndHead = SubtractCastArea( BndHead, i ) ;
		//(3)与边界环求交
		if( BndHead && m_pAllLoop && m_pDriveLoop[i] )
		{
			pResult = Mini_OperateContours( BndHead, m_pAllLoop, 0 ) ;
			Mini_DeleteContours( BndHead ) ;
			BndHead = pResult ;
		}
		//(4)设置标记,删除内存
		if( !BndHead ) 
		{
			m_bLayer[i] = TRUE ;
			i-- ;
		}
		else 
		{
			Mini_DeleteContours( BndHead ) ;
			break ;
		}
	}
	if( i >= 0 ) m_bLayer[i] = TRUE ;
	return TRUE ;
}

//按区域查找两主层之间的所有层间路径
void CSmartRPocketGen::FindAllAreaLoop(int nCurr, int nPrev, CPtrArray & pBtwLPathArr )
{
	CSmtLoopPath * LPathHead = NULL, *pLPath = NULL;
	CPtrArray pAddArr1, pAddArr2 ;
	int i, j, k, m, nSize, nSize2 ;
	double dTol = m_pTool->m_fRadius * 2 ;
	for ( i = nCurr-1; i > nPrev; )
	{
		if( m_pLPathArr[i].GetSize() == 0 )
		{
			i -- ;
			continue ;
		}
		LPathHead = m_pLPathArr[i].GetAt( 0 ) ;
		pAddArr1.Add( LPathHead ) ;
		m_pLPathArr[i].RemoveAt( 0 ) ;
		while ( 1 )
		{
			for ( j = nCurr - 1 ; j > nPrev ; j-- )
			{
				nSize = (int)m_pLPathArr[j].GetSize(); 
				for ( k = nSize-1; k >= 0; k-- )
				{
					pLPath = m_pLPathArr[j].GetAt( k ) ;
					nSize2 =(int) pAddArr1.GetSize() ;
					for ( m = 0; m < nSize2; m++ )
					{
						CSmtLoopPath * pTmpLPath = (CSmtLoopPath *)pAddArr1.GetAt( m ) ;
						if ( IsInSameArea( pLPath->m_pLoop, pTmpLPath->m_pLoop, dTol ) )
						{
							InsertLPath( pLPath, LPathHead ) ;
							pAddArr2.Add( pLPath ) ;
							m_pLPathArr[j].RemoveAt( k) ;
							break ;
						}
					}
				}
			}
			pAddArr1.RemoveAll() ;
			if( pAddArr2.GetSize() == 0 ) break ;
			while( pAddArr2.GetSize() )
			{
				pLPath = (CSmtLoopPath*)pAddArr2.GetAt( 0 ) ;
				pAddArr1.Add( pLPath ) ;
				pAddArr2.RemoveAt( 0 ) ;
			}
		}
		// 将LPathHead反向
		ReverseAllLPath( LPathHead ) ;
		// 将环排序
		LPathHead = SortLPathByRemainLoop( LPathHead, GetLoopCheckTol() + 0.1 , dTol ) ;
		pBtwLPathArr.Add( LPathHead ) ;
	}
}

//重新设置曲线的起始位置
void CSmartRPocketGen::SetCurveListStart(CurveList & CurList )
{
	CSmartCurve * pCurve = NULL, *pPrev = NULL ;
	PNT2D p , nearpt;
	CSmartSect * pNearSect = NULL ;

	POSITION pos = CurList.GetHeadPosition() ;
	while ( pos )
	{
		if ( pos == CurList.GetHeadPosition() )
		{
			pCurve = CurList.GetNext( pos ) ;
		} 
		else
		{
			pPrev = pCurve ;
			pCurve = CurList.GetNext( pos ) ;
		}
		
		if ( pPrev && pCurve->IsClosed() )
		{
			pNearSect = NULL ;
			pPrev->GetStart( p ) ;
			double dMinDist = 1.0e8, dDist ;
			PNT2D end ;
			CSmartSect *pSect = pCurve->m_pHead ;
			for( ; pSect ; pSect = pSect->next )
			{
				dDist = pSect->MinDistPoint ( p, end ) ;
				if( dDist < dMinDist )
				{
					dMinDist = dDist ;
					pNearSect = pSect ;
					mathCpyPnt2D( end, nearpt ) ;
				}
			}
			if ( pNearSect )
			{
				pCurve->SetStartPoint( pNearSect, nearpt ) ;
			}
		}
	}
}

CSmartLoop * CSmartRPocketGen::GetBtwPlgLoop( CSmartLoop * pCurr )
{
	if( !m_pBtwPlgLayer || !pCurr || !m_pDriveLoop ) return NULL ;

	int nLayer = m_pBtwPlgLayer[pCurr->m_nDepth] ;

	return GetPlungeLoop( nLayer, pCurr ) ;
}

CSmartLoop * CSmartRPocketGen::GetPlanePlgLoop(CPlaneLoop * pPlane )
{
	if( !pPlane ) return NULL ;
	int nLayer = 0 ;
	while ( (nLayer < m_nCount) && (m_dZ[nLayer] >= pPlane->m_dHeight) )
	{
		nLayer ++ ;
	}
	while ( (nLayer < m_nCount) &&  (!m_bLayer[nLayer]) )
	{
		nLayer ++ ;
	}
	if( nLayer >= m_nCount )
	{
		return NULL ;
	}
	CSmartLoop *pPlaneLoop = pPlane->m_pPlaneLoop ;
	DOUBLE dTol = m_pTool->m_fRadius + m_cStockDef.m_dDriveOffset[0] ;

	return GetPlungeLoop( nLayer, pPlaneLoop, dTol ) ;
}

CSmartLoop * CSmartRPocketGen::GetPlungeLoop(int nLayer, CSmartLoop * pCurr, DOUBLE dTol /* = 0.01  */)
{
	if( nLayer < 0 || nLayer >= m_nCount 
		||  NULL == pCurr
		||  NULL == m_pDriveLoop[nLayer] ) 
	{
		return NULL ;
	}
	// STEP 1: 把下层模型边界向外偏移0.005
	DOUBLE dOffset[2] = { 0.005, 0.005 } ;
	CSmartLoop *pOffset = Mini_OffsetContourList( m_pDriveLoop[nLayer], 
												  NCDEF_OFFSET_OUTER, 
												  dOffset, 
												  GetCorDef() ) ;
	// STEP 2: 得到与加工环相交的环作为下刀边界
	CSmtLoopArr plgLpArr ;
	CSmartLoop  *pLoop = NULL , *pTmpLp = NULL, *pTmpLp2 = NULL ;
	pLoop = pOffset;
	pLoop = MathCAM_ExtractAllLoop( pLoop ) ;
	while ( pLoop )
	{
		pTmpLp = pLoop->next ;
		pLoop->next = NULL ;
		pTmpLp2 = pCurr ;
		BOOL bAddFlg = FALSE ;
		while( pTmpLp2 )
		{
			if ( Is2ContoursInt( pTmpLp2, pLoop, dTol ) )
			{
				bAddFlg = TRUE ;
				break ;
			}
			pTmpLp2 = pTmpLp2->next ;
		}
		if ( bAddFlg )
		{
			plgLpArr.Add( pLoop ) ;
		}
		else
		{
			Mini_DeleteContours( pLoop ) ;
		}
		if( pTmpLp ) pTmpLp->prev = NULL ;
		pLoop = pTmpLp ;
	}
	// STEP 3: 如果没有下刀曲线A能包含加工域B，则添加毛坯大环，
	//         使下刀路径从下刀曲线的内部走
	//         说明：A和B的关系只有三种:A包含B;A落在B的孤岛;A和B相邻
	//         对后两种情况需要从A的内部开始下刀.
	BOOL bAddRough = TRUE;
	int i, nSize = 0 ;
	PNT2D mid ;
	pCurr->m_pCurve->GetMidPoint( mid ) ;
	nSize =(int) plgLpArr.GetSize() ;
	if ( 0 == nSize )
	{
		return NULL ;
	}
    for ( i = 0; i < nSize; i ++ )
    {
		pLoop = plgLpArr.GetAt( i ) ;
		if ( pLoop->IsPointIn( mid ) )
		{
			bAddRough = FALSE ;
			break ;
		}
	}
	if ( bAddRough )
	{
		plgLpArr.Add( Mini_CopyContours( m_pRoughLoop[nLayer] ) ) ;
	}
	// STEP 4: 重构环，并添加环的毛坯属性
	pLoop = NULL ;
	nSize = (int)plgLpArr.GetSize() ;
	for ( i = 0; i < nSize; i++ )
	{
		pTmpLp = plgLpArr.GetAt( i ) ;
		pLoop = Mini_AddContours( pLoop, pTmpLp ) ;
	}
	plgLpArr.RemoveAll() ;
	if ( pLoop )
	{
		CSmartLoop loopLib ;
		pLoop = loopLib.BuildContour ( pLoop ) ;
		SetContourBlank( pLoop ) ;
	}
	return pLoop ;
}

//guomin
//以下函数用于大浮雕粗加工路径计算
BOOL CSmartRPocketGen::BuildDriveAndCheckMdlnew ( CSmartGraphic& Graph, 
	CSmtCheckMdl& DriveMdl, 
	CSmtCheckMdl& CheckMdl,
	double RotAngle )
{
	//STEP 1 : 构建加工面模型
	if( m_c5DCtrlDef.m_cToolAxis.m_nAxisType != NCDEF_AXIS_VERT )
	{/*多轴模式, 设置多轴剖分模式*/
		DriveMdl.SetMultiAxisCellMode( TRUE ) ;
		CheckMdl.SetMultiAxisCellMode( TRUE ) ;
	}
	JDNC_SETUP drvSetup = m_cSetupDef ;
	if( m_cSetupDef.m_cModelTol.m_nMdlFlag & NCDEF_SETUP_EXACTMODE )
	{
		drvSetup.m_cTolDef.m_dArcTol  = min( 2.0e-4,drvSetup.m_cTolDef.m_dArcTol ) ;
		drvSetup.m_cTolDef.m_dAngTol  = min( 10.0  ,drvSetup.m_cTolDef.m_dAngTol ) ;
		DriveMdl.SetExactCalcMode(TRUE) ;
	}

	//guomin 增加参数用于保存生成CheckModel过程中构造的矩形网格
	if( ! Graph.BuildCheckModelnew( drvSetup, DriveMdl, m_pRectNetSurf,NCDEF_SURF_DRIVE ) ) 
	{
		return FALSE		;
	}
	AutoAdjustCutDepth	( DriveMdl ) ;

	TFLOAT fZBottom =  (TFLOAT)GetBottomHeight() ;
	if( IsDelBndPoint() ) 
	{
		fZBottom -= (TFLOAT)GetSurfaceTol().m_dArcTol ;
	}
	DriveMdl.ZMoveModel( GetDriveSurfZMove() ) ;
	DriveMdl.m_dTouchTol = max( m_cSetupDef.m_cTolDef.m_dArcTol*3.0, m_cSetupDef.m_cModelTol.m_dTouchTol ) ;
	DriveMdl.m_fBottom =  fZBottom ;
	ResetToolCutDepth( DriveMdl, m_pTool ) ;
	//STEP 2 : 构建保护面模型
	JDNC_SETUP checkSetup = m_cSetupDef ;
	checkSetup.m_cModelTol.m_nMdlFlag &= ~NCDEF_SETUP_NOMOSAIC ;
	checkSetup.m_cModelTol.m_nMdlFlag &= ~NCDEF_SETUP_EXACTMODE ;
	SetCheckFltBoxByDriveMdl( DriveMdl,CheckMdl );
	if( Graph.BuildCheckModel( checkSetup, CheckMdl, NCDEF_SURF_CHECK ) )
	{
		CheckMdl.ZMoveModel( GetCheckSurfZMove() ) ;
		CheckMdl.m_fBottom =  fZBottom ;
		CheckMdl.m_dTouchTol = DriveMdl.m_dTouchTol ;
		ResetToolCutDepth( CheckMdl, m_pSafeTool ) ;
		if( CheckMdl.m_cAllChk.GetSize() > 0 ) 
		{
			DriveMdl.SetCheckMdl( & CheckMdl ) ;
		}
	}
	//STEP 3 : 设置加工面属性
	if( IsDelFlatPoint() )
	{/*删除平面路径点*/
		DriveMdl.m_bCheckFlag |= SMARTNC_CHECKMDL_DELFLATPNT ;
		DriveMdl.ZMovePlaneSurf( m_cSetupDef.m_cTolDef.m_dArcTol );
	}
	if( ( m_cSetupDef.m_cModelTol.m_nMdlFlag & NCDEF_SETUP_TOUCHMODE ) && 
		( IsDelBndPoint() || DriveMdl.m_pCheckMdl ) )
	{/*设置刀触点模式*/
		DriveMdl.SetTouchCalcMode( TRUE ) ;
		if( m_cSetupDef.m_cModelTol.m_nMdlFlag & NCDEF_SETUP_TOUCHSURF )
		{
			DriveMdl.m_bCheckFlag |= SMARTNC_CHECKMDL_TOUCHSURF ;
		}
	}
	if( m_cSetupDef.m_cModelTol.m_nMdlFlag & NCDEF_SETUP_EXACTMODE )
	{/*设置精确模式,必须启动消除马赛克选项*/
		m_cSetupDef.m_cModelTol.m_nMdlFlag |= NCDEF_SETUP_NOMOSAIC ;
	}
	if( IsDelBndPoint() )
	{/*设置删除边界路径点模式*/
		DriveMdl.m_bCheckFlag |= SMARTNC_CHECKMDL_DELBOUNDPNT ;
		DriveMdl.DefineBoundEdge();
	}
	if( fabs( RotAngle ) > 1.0e-4 )
	{/*设置旋转模型模式*/
		DriveMdl.RotateByZAxis( RotAngle ) ;
		CheckMdl.RotateByZAxis( RotAngle ) ;
	}
	DriveMdl.UpdateAllTool( m_pTool, m_pSafeTool ) ;
	return TRUE ;
}

//获取刀具设置
BOOL CSmartRPocketGen::GetToolDef(SRectNetRoughTool &tool)
{
	//类型
	int nOriginType = m_cToolDef.m_nToolType ;
	switch( nOriginType )
	{
	case 0:
		tool.m_nToolType = 1 ;
		break;
	case 1:
		tool.m_nToolType = 0 ;
		break;
	case 2:
		tool.m_nToolType = 5 ;
		break;
	case 3:
		tool.m_nToolType = 2 ;
		break;
	case 4:
		tool.m_nToolType = 3 ;
		break;
	case  5:
		tool.m_nToolType = 4 ;
		break;
	default:
		return FALSE ;
	}	
	tool.m_fDiameter = (float)m_cToolDef.m_dTopDiam ;
	tool.m_fDiameterBase = (float)m_cToolDef.m_dBtmDiam ;
	tool.m_fAngleFull = (float)m_cToolDef.m_dSideAng ;
	tool.m_fRoundRadius = (float)m_cToolDef.m_dCorRadi ;
	return TRUE ;
}

BOOL CSmartRPocketGen::CreatDriveLoopForRectNetSurf(CSmtCheckMdl *DriveMdl, JDNC_PRGDEF &ProgDef, double dCur)
{
	if( !m_pRectNetSurf )
		return FALSE ;	

	//1.准备输入参数,刀具
	SRectNetRoughTool tool ;
	if( !GetToolDef(tool) )//未处理刀具不生成路径环
		return FALSE ;	

	//2.准备输入参数,分层数及各层高度
	ProgDef.m_dTotalMove = dCur * 0.4 ;

	CSmartLoop loop, *PlaneLoop = NULL ;
	double dPlaneCur = dCur * 0.5, dLastCur = dCur * 0.4 ;
	if( m_cParam.m_bRPocketFlag & NCDEF_RPOCKET_CUTPLANE )
	{
		CNcZLevelGenMdl heightMdl ;
		if( ! heightMdl.Create( *DriveMdl, NULL, m_cSetupDef, ProgDef ) )
		{
			return FALSE ;
		}	
		// 生成平面加工区域
		CSmartFPlaneGen FPlaneGen ;
		(*(CSmartPathGen*)&FPlaneGen ) = * this ;
		FPlaneGen.m_cMethodDef.m_nMethodType = surfncMethodFPlane ;
		SurfNC_InitPathParam( FPlaneGen.m_cParam ) ;
		FPlaneGen.m_cMethodDef.m_cPlaneCut = FPlaneGen.m_cParam ;
		ProgDef.m_dTotalMove = dPlaneCur ;
		FPlaneGen.CreatePlaneContour ( *DriveMdl, &heightMdl, PlaneLoop, ProgDef ) ;
	}
	else
	{
		dLastCur = dCur * 0.9 ;
	}
	// 分层
	GetECutLayersForRectSurf( PlaneLoop ) ;
	// 得到平面环的高度
	double *dZ = NULL, *dAlldZ = NULL ;
	int nCnt = 0, nAllCnt = 0 ;
	BOOL *bFlag = NULL, bCombine = FALSE ;
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

	ProgDef.m_dTotalMove =  dLastCur ;
	//3.根据余量重构矩形网格
	float fRemainder = (float)m_cStockDef.m_dDriveZMove[0] ;
	BOX3D box ;
	m_pRectNetSurf->UpdateBox(&box) ;
	float flowz = (float)box.min[2] ;
	//guomin 负余量的情况下转化为正余量的情况进行处理 
	RecreateMesh(m_pRectNetSurf,tool.m_fDiameter,fabs(fRemainder),flowz )  ;
	//4.等距
	//负余量情况下,将曲面Z值反向,再进行正向等距,再将曲面反向回来
	if( fRemainder<0 )
		RectSurfMirror(m_pRectNetSurf) ;

	OffsetRectMesh(m_pRectNetSurf,	fabs(fRemainder)) ;

	if( fRemainder<0 )
		RectSurfMirror(m_pRectNetSurf) ;


	// 3. 曲面按照加工刀具进行偏刀计算
	if( !VirtualCarving(	0 ,				
		m_pRectNetSurf ,
		tool.m_fDiameter ,
		tool.m_fDiameterBase ,	
		tool.m_fAngleFull ,	
		tool.m_fRoundRadius ,   
		tool.m_nToolType,
		ProgDef) )		
	{
		if( m_pRectNetSurf )
		{
			delete m_pRectNetSurf ;
			m_pRectNetSurf = NULL ;
		}
		return FALSE ;
	}

	// 4 扩边
	EnlargeRectMesh( m_pRectNetSurf , 1 ) ; 	

	//进度条
	ProgDef.m_dTotalMove = dCur*0.1 ;
	ProgDef.m_dIncStep = 0.0; 
	ProgDef.m_dLimitAt = 1.0 ;
	ProgDef.m_dStepAt  = 0.0 ;
	if( ProgDef.m_dTotalMove > 0.0 )
	{
		ProgDef.m_dIncStep = ProgDef.m_dTotalMove / DOUBLE(nAllCnt); 
	}
	CSmartLoop **pAllLoop = new CSmartLoop*[nAllCnt] ;
	for( int i=0 ; i<nAllCnt ; i++ )
		pAllLoop[i] = NULL ;
	// 5 根据给定高度，计算该高度的轮廓
	for( int i=0 ; i<nAllCnt ; i++ )
	{
		if( ProgDef.m_pBrkFunc &&  ProgDef.m_pBrkFunc()  )
		{ /* 用户中断 || 进度提示 */
			break; 
		}
		ProgDef.m_dStepAt += ProgDef.m_dIncStep ;
		while( ProgDef.m_pPrgFunc && ProgDef.m_dStepAt >= ProgDef.m_dLimitAt )
		{
			ProgDef.m_pPrgFunc( 1 ) ;
			ProgDef.m_dStepAt -= ProgDef.m_dLimitAt ;
		}

		float fZ = (float)dAlldZ[i] ;
		CToolPathContours contour ;
		CalAllContours(	m_pRectNetSurf ,fZ ,	contour ) ;

		float fTol = (float)m_cSetupDef.m_cTolDef.m_dArcTol ;
		for( int j=0 ; j<contour.nCurve ; j++ )
		{
			//判断下,面积太小的环舍去不要
			float fArea = CalPtsArea(contour.aPts[j],contour.anPtsInCurve[j]) ;
			if( fArea < 2.f )
				continue ;
			CSmartLoop *pLoop = ConvertToSmartLoop(contour.aPts[j],contour.anPtsInCurve[j],fTol) ;
			pAllLoop[i] = Mini_AddContours( pAllLoop[i], pLoop ) ;
		}
		contour.cleardata() ;
	}

	if( pAllLoop && dAlldZ )
	{
		// 得到所有加工域环
		GetAllDriveLoop( pAllLoop, dAlldZ, bFlag, nAllCnt ) ;
		// 得到所有平面环
		if( m_cParam.m_bRPocketFlag & NCDEF_RPOCKET_CUTPLANE && PlaneLoop )
		{
			GetAllPlaneLoop( pAllLoop, dAlldZ, bFlag, nAllCnt, PlaneLoop ) ;
		}
	}
	ClearAllLoopHead( pAllLoop, nAllCnt ) ;
	if( m_pRectNetSurf )
	{
		delete m_pRectNetSurf ;
		m_pRectNetSurf =  NULL ;
	}
	if( bCombine )
	{
		if( dAlldZ ) delete[] dAlldZ ;
		if( bFlag  ) delete[] bFlag  ;
	}
	return TRUE ;
}

int CSmartRPocketGen::GetECutLayersForRectSurf(CSmartLoop *&PlaneLoop)
{
	//根据m_pRectNetSurf重新获取最高点最低点
	if( !m_pRectNetSurf ) 
		return FALSE ;

	BOX3D box ;
	m_pRectNetSurf->UpdateBox(&box) ;

	double top = box.max[2] ;
	double bot = box.min[2]+m_cStockDef.m_dDriveZMove[0]/*+0.2*/+0.0001 ;

	// 避免加工到平面，表面余量+0.001
	double dCutDepth = top - bot, dStepZ = 0. ;
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
		// 按照层高分层
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
			if( m_nCount < 1 ) 	m_nCount = 1 ;
		}
	}
	m_dStepZ = dCutDepth / m_nCount ;
	// 均匀分层,也可以按照cimatron从高到低分层
	m_dZ = new double[m_nCount] ;
	for( int i = 0 ; i < m_nCount ; i++ )
	{
		m_dZ[i] = top - ( i+1 ) * m_dStepZ ;
	}
EndLine:
	BOOL bBtwRoughPlane = m_cParam.m_cBtwRecut.m_nLayerType != NCDEF_BTWLAYER_CLOSE 
		&& m_cParam.m_cBtwRecut.m_nLayerType == NCDEF_BTWLAYER_ROUGH 
		&& m_cParam.m_bRPocketFlag & NCDEF_RPOCKET_CUTPLANE ;
	if( m_cParam.m_cBtwRecut.m_nLayerType != NCDEF_BTWLAYER_CLOSE )
	{
		if( !bBtwRoughPlane )
			SetCutStepBtwLayer() ;
		else
			SetCutStepBtwLayerRough( PlaneLoop ) ;
	}
	else
	{
		m_bLayer = new BOOL[m_nCount] ;
		for ( int i = 0; i < m_nCount; i++ )
		{
			m_bLayer[i] = TRUE ;
		}
	}

	return 1 ;
}
#include "StdAfx.H"
#include "SmartNC.H"
#include "SurfGeo.H"
#include "SmartBound.H"
#include "SmartVoro.H"
#include "SmtPathGen.H"
#include "SmartPathEx.h"
#include "SmtPathGen2D.H"
#include "NcGCode.H"
#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char BASED_CODE THIS_FILE[] = __FILE__;
#endif

/////////////////////////////////////
//////////////////////////
// 使用圆弧逼近清角路径
//
void Wizard_AddCornerPath( CPathCombine& TComb  , 
						   PNT3D TAPos[]        , 
						   int Count            ,
						   DOUBLE MaxDepth      ,
						   JDNC_SETUP& Setup  )
{
	if( Setup.m_cTolDef.m_nFitType == NCDEF_FITTYPE_LINE )
	{
		CPathPLine3D* pPLine3D = new CPathPLine3D( ) ;
		pPLine3D->m_pTAPos = new TPNT3D[ Count + 1 ] ;
		for( int i = 0 ; i <= Count ; i ++ ) 
		{
			pPLine3D->m_pTAPos[i][0] = TAPos[i][0]  ;
			pPLine3D->m_pTAPos[i][1] = TAPos[i][1]  ;
			pPLine3D->m_pTAPos[i][2] = TAPos[i][2]  ;
		}
		pPLine3D->m_nCount = Count ;
		TComb.AddEntity( pPLine3D ) ;
		return  ;
	}
 	CSmartCurve TmpCurve ; 
	DOUBLE dFitTol[2] ; 
    dFitTol[0] = Setup.m_cTolDef.m_dArcTol      ;
    dFitTol[1] = cos( ANGLE_TO_RADIAN( Setup.m_cTolDef.m_dAngTol ) )  ;
	PNT2D  *dPtArr = new PNT2D[Count+5];
	PNT2D  *dMidArr= new PNT2D[Count+5] ;
	int nSamePnt ,nAtPos, nEndPos, k  ;
	nAtPos =  nEndPos = nSamePnt = 0  ; 
	for( int i = 0 ; i <= Count ; i ++ ) 
	{
	    if( fabs( TAPos[i][2] - MaxDepth ) <= 1.0e-6 ) 
		{/* Z坐标相同 */
			nSamePnt ++ ; 
			if( i < Count ) continue  ;
		}
		else if( nSamePnt < 6 )
		{ /*点数太少，不作圆弧逼近*/ 
			nSamePnt = 0 ;
			if( i < Count ) continue  ;
		}
		else 
		{ /*索引回退*/
			 i -- ; 
		}
		if( nSamePnt == 0 ) nEndPos = i ;
		else nEndPos = i - nSamePnt + 1 ;
		if( nEndPos - nAtPos >= 1 )
		{ 
		    CPathPLine3D* pPLine3D = new CPathPLine3D() ;
			pPLine3D->m_nCount = nEndPos - nAtPos ;
			pPLine3D->m_pTAPos = new TPNT3D[ pPLine3D->m_nCount + 1 ] ;
			for( k = 0 ; k <= (int)pPLine3D->m_nCount ; k ++ ) 
			{ 
				pPLine3D->m_pTAPos[k][0] = TAPos[ nAtPos+k][0]  ;
				pPLine3D->m_pTAPos[k][1] = TAPos[ nAtPos+k][1]  ;
				pPLine3D->m_pTAPos[k][2] = TAPos[ nAtPos+k][2]  ;
			}
		    TComb.AddEntity( pPLine3D ) ;
		}
		if( nSamePnt >= 2  ) 
		{
			for( k = 0 ; k <= nSamePnt-1; k ++ ) 
			{
				dPtArr[k][0] = TAPos[ nEndPos + k][0] ;
				dPtArr[k][1] = TAPos[ nEndPos + k][1] ;
				if( k != 0 )
				{
					dMidArr[k-1][0] = (dPtArr[k-1][0] + dPtArr[k][0] ) * 0.5 ;
					dMidArr[k-1][1] = (dPtArr[k-1][1] + dPtArr[k][1] ) * 0.5 ;
				}
			}
			TmpCurve.AddPointArray( dPtArr, nSamePnt-1, dFitTol, dMidArr ) ;
			TComb.AppendCurve2D( TmpCurve, MaxDepth ) ;
			TmpCurve.ClearAll() ; 
		}
		nSamePnt = 0  ;
		nAtPos = nEndPos = i ;
	}
    delete[] dPtArr ;
	delete[] dMidArr ;
}

CSmartDrillGen::CSmartDrillGen()
{
}
CSmartDrillGen::~CSmartDrillGen()
{
}
double MathCAM_MiniDistToContour( CSmartLoop& Contour, DOUBLE Point[2] )
{
    DOUBLE dMinDist = Contour.MinDistPoint( Point ) ;
    for( CSmartLoop* pIsl = Contour.GetIsland() ; pIsl ; pIsl = pIsl->next )
    {
        double dDist = pIsl->MinDistPoint( Point ) ;
        if( dDist < dMinDist ) dMinDist = dDist ;
    }
    return dMinDist ;

}
// 计算位置钻孔路径
BOOL CSmartDrillGen::GeneratePathEx( CPathGroup& NewPath       ,   /*雕刻路径*/ 
		                             CSmartGraphic& Graph)   /*雕刻参数*/
{
	// STEP 0 : 校正雕刻环境
	if( !UpdateTool2D() ) return FALSE ;

	CPtrList  AllEnt ;
	Graph.GetAllEntity( m_cSetupDef, AllEnt, SMARTGRAPH_TYPE_POINT ) ; 
	if( m_cParam.m_cPointDef.m_nPointType != NCDEF_TPOINT_STEP )
	{ // 不是线上取点, 和普通点一样。
	   Graph.GetFeaturePoint( m_cSetupDef, AllEnt, m_cParam.m_cPointDef ) ;
	}
	CPointList  AllPoint ;
	CSmartPoint* pPoint ;
	double dTop = -1.0e10 ;
	POSITION pos = AllEnt.GetHeadPosition() ;
	while( pos ) 
	{
		pPoint = ( CSmartPoint*) AllEnt.GetNext( pos ) ;
		if( !( m_cParam.m_bDrillFlag & NCDEF_FDRILL_REDEPTH ) )
		{ /*不保留孔位高度*/
			pPoint->m_dPoint[2] = 0.0 ;
		}
	    if( !( m_cParam.m_bDrillFlag & NCDEF_FDRILL_FLTSAME ) ||
			  AllPoint.FindPoint( pPoint->m_dPoint ) < 0  )
		{
			AllPoint.InsertAfter( pPoint, AllPoint.GetTail() ) ;  
		}
		else
		{ // 删除重点
			delete pPoint ;
			pPoint = NULL;
		}
		if( pPoint!=NULL && pPoint->m_dPoint[2] > dTop ) 
		{
			dTop = pPoint->m_dPoint[2] ;
		}
	} 
    AllEnt.RemoveAll() ;
    if( GetSortType() != NCDEF_SORTTYPE_SELECT )
    {
        AllPoint.ResortAllPoint(GetSortType(), TRUE, float( m_cToolDef.m_dTopDiam * 0.5f) ) ;
    }
	if( m_cParam.m_cPointDef.m_nPointType == NCDEF_TPOINT_STEP )
	{ // 线上取点 , 不排序
        AllEnt.RemoveAll() ;
	    Graph.GetFeaturePoint( m_cSetupDef, AllEnt, m_cParam.m_cPointDef ) ;
	    pos = AllEnt.GetHeadPosition() ;
	    while( pos ) 
		{ 
			pPoint = ( CSmartPoint*) AllEnt.GetNext( pos ) ;
			if( !( m_cParam.m_bDrillFlag & NCDEF_FDRILL_REDEPTH ) )
			{ /*不保留孔位高度*/
				pPoint->m_dPoint[2] = 0.0 ;
			}
			if( !( m_cParam.m_bDrillFlag & NCDEF_FDRILL_FLTSAME ) ||
				  AllPoint.FindPoint( pPoint->m_dPoint ) < 0  )
			{
				AllPoint.InsertAfter( pPoint, AllPoint.GetTail() ) ;  
			}
			else
			{ // 删除重点
				delete pPoint ;
				pPoint = NULL;
			}
			if( pPoint!=NULL && pPoint->m_dPoint[2] > dTop )
			{
				dTop = pPoint->m_dPoint[2] ;
			}
		} 
	}
	if( AllPoint.GetNum() == 0  )
	{
		m_nErrorType = JDERROR_GENPATH_NOPOINT ;
		return FALSE ;
	}
	JDNC_LAYER * pLayer   = GetLayerDef() ;
	pLayer->m_dTotalDepth = GetCutDepth() - m_cStockDef.m_dDepthStock + 
							m_cParam.m_dRetractDist + m_cParam.m_dTipComp + m_cParam.m_dPassDist ; 
	if( m_cParam.m_nDrillType != NCDEF_DRILLTYPE_G83 && m_cParam.m_nDrillType != NCDEF_DRILLTYPE_G73 &&
		m_cParam.m_nDrillType != NCDEF_DRILLTYPE_G84 && m_cParam.m_nDrillType != NCDEF_DRILLTYPE_G74 )
		pLayer->m_nLayerType = NCDEF_LAYER_CLOSE ;
	DefineLayerDepth( * pLayer) ; 
	// STEP 1 : 计算雕刻路径
    CPathCombine *pTComb = new CPathCombine( NC_WPROCESS_DRILL ) ;
	for( pPoint = AllPoint.GetHead(); pPoint ; pPoint = pPoint->next )
	{
		CPathPoint3D *pDrill  = new CPathPoint3D( pLayer->m_nLayerCount ) ;
		pDrill->m_fPoint[0] = pPoint->m_dPoint[0] ;
		pDrill->m_fPoint[1] = pPoint->m_dPoint[1] ;
		if( m_cParam.m_bDrillFlag & NCDEF_FDRILL_REDEPTH )
		{ /*保留孔位高度*/
			pDrill->m_fPoint[2] = pPoint->m_dPoint[2] ;
		}
		else
		{ 
			pDrill->m_fPoint[2] = 0.0 ;
		}
 		for( int  i = 1 ; i <= m_cFeedDef.m_cLayerDef.m_nLayerCount ; i ++ )
		{
			pDrill->m_fIncAt[i-1][2] =  pLayer->m_dLayerDepth[i] ;
			pDrill->m_fIncAt[i-1][0] = pDrill->m_fPoint[0] ;
			pDrill->m_fIncAt[i-1][1] = pDrill->m_fPoint[1] ;
		}
		MatchingDrillType( pDrill ) ;
		pDrill->m_fMoveUp      = m_cParam.m_dRetractDist; 
		pDrill->m_bRetractTop  = m_cParam.m_nRetractM  ;
		if( m_cParam.m_bDrillFlag & NCDEF_FDRILL_DWELLTIME )
            pDrill->m_fDwellTime = m_cParam.m_dDWellTime ;
		else
			pDrill->m_fDwellTime = 0. ;
		pDrill->m_fIncStep = ( pLayer->m_dTotalDepth ) / m_cFeedDef.m_cLayerDef.m_nLayerCount ;
		pDrill->m_fShiftIJK[0] = m_cParam.m_dShiftIJK[0] ;
		pDrill->m_fShiftIJK[1] = m_cParam.m_dShiftIJK[1] ;
		pDrill->m_fShiftIJK[2] = m_cParam.m_dShiftIJK[2] ;
		if( m_cParam.m_bDrillFlag & NCDEF_FDRILL_SHIFTIJK ) pDrill->m_bIsShift = 1 ;
		else												pDrill->m_bIsShift = 0 ;
		// 如果pTComb前面有钻孔点,需要添加G98/G99/或者优化的Quick
		if( pTComb->m_pTail )
		{
			CPathPLine3D *pQuick = CreateQuickFromDrill( ( CPathPoint3D *)pTComb->m_pTail, pDrill, dTop ) ;
			if( pQuick ) pTComb->AddEntity ( pQuick ) ;
		}
		pTComb->AddEntity( pDrill ) ;
	}
	// 添加回退路径到安全高度或者R平面
	if( pTComb && pTComb->m_pTail && pTComb->m_pTail->GetType () == NC_PATH_POINT3D )
	{
		CPathEntity *pEntity = CreateQuickLine( pTComb ) ;
		if( pEntity )
			pTComb->AddEntity ( pEntity ) ;
	}
	// 将钻孔路径转为直线路径
	if( m_cParam.m_bDrillFlag & NCDEF_FDRILL_PLINETYPE )
	{
		ConvertDrillPath( pTComb ) ;
		pTComb->m_nType = NC_WPROCESS_ROUGH ;
	}
	// STEP 2 : 输出路径
	if( ! pTComb->m_pHead ) 
	{
		delete pTComb ;
	}
	else NewPath.AddData( 0.0, pTComb ) ;
	if( m_cParam.m_bDrillFlag & NCDEF_FDRILL_REDEPTH ) 
	{ // 忽略表面高度
        DOUBLE dTop = -1.0e6 ;
	    for( pPoint = AllPoint.GetHead(); pPoint ; pPoint = pPoint->next )
	    {
            if( pPoint->m_dPoint[2] > dTop ) dTop = pPoint->m_dPoint[2] ;
        }
        m_dTopHeight = dTop;
	}
	else
	{// 校正表面高度
		NewPath.ZMoveGroup( GetTopHeight() ) ;
	}
	
	if( pLayer->m_dLayerDepth ) delete[] pLayer->m_dLayerDepth ;
	pLayer->m_dLayerDepth = NULL ;
	return TRUE ;
}
CPathPLine3D * CSmartDrillGen::CreateQuickFromDrill( CPathPoint3D *pHead, CPathPoint3D *pNext, double dTop )
{
	if( !pHead || !pNext ) return NULL ;
	PNT3D start[2], end[2], tmp ;
	CPathPLine3D *pQuick = NULL ;
	int n = 0 ;
	// 计算pHead的末点坐标
	n = pHead->m_nIncTime - 1 ;
	pHead->GetEndPoint ( 1, start[0] ) ;
	start[0][2] = pHead->m_fPoint[2] + pHead->m_fMoveUp - pHead->m_fIncAt[n][2] ;
	mathCpyPnt( start[0], start[1] ) ;
	if( m_cParam.m_nRetractM == NCDEF_DRILLRETRACT_SAFE )
        start[1][2] = dTop + m_cSpeedDef.m_dSafeHeight ;
	else 
		start[1][2] = pHead->m_fPoint[2] + pHead->m_fMoveUp ;

	// 计算pNext的起点坐标
	pNext->GetEndPoint ( 0, end[0] ) ;
	end[0][2] = pNext->m_fPoint[2] + pNext->m_fMoveUp ;
	mathCpyPnt( end[0], end[1] ) ;
	if( m_cParam.m_nRetractM == NCDEF_DRILLRETRACT_SAFE )
		end[1][2] = dTop + m_cSpeedDef.m_dSafeHeight ;
	else
		end[1][2] = pNext->m_fPoint[2] + pNext->m_fMoveUp ;
	// 生成连刀
	double dDist = fabs( start[1][2] - end[1][2] ) ;

	pQuick = new CPathPLine3D() ;
	pQuick->m_bFeedType = JDNC_FEEDTYPE_QUICK ;
	if( m_cParam.m_nRetractM == NCDEF_DRILLRETRACT_SAFE )
	{
		pQuick->m_nCount = 3 ;
		pQuick->m_pTAPos = new PNT3D[pQuick->m_nCount +1 ] ;
		mathCpyPnt( start[0], pQuick->m_pTAPos[0] ) ;
		mathCpyPnt( start[1], pQuick->m_pTAPos[1] ) ;
		mathCpyPnt( end[1]  , pQuick->m_pTAPos[2] ) ;
		mathCpyPnt( end[0]  , pQuick->m_pTAPos[3] ) ;
	}
	else
	{
		if( dDist > 0.0001 )
		{
			pQuick->m_nCount = 3 ;
			pQuick->m_pTAPos = new PNT3D[pQuick->m_nCount +1 ] ;
			mathCpyPnt( start[0], pQuick->m_pTAPos[0] ) ;
			mathCpyPnt( start[1], pQuick->m_pTAPos[1] ) ;
			if( start[1][2] > end[0][2] )
			{
				mathCpyPnt( end[0], tmp ) ;
				tmp[2] = start[1][2] ;
			}
			else
			{
				mathCpyPnt( start[1], tmp ) ;
				tmp[2] = end[0][2] ;
			}
			mathCpyPnt( tmp     , pQuick->m_pTAPos[2] ) ;
			mathCpyPnt( end[0]  , pQuick->m_pTAPos[3] ) ;
		}
		else
		{
			pQuick->m_nCount = 2 ;
			pQuick->m_pTAPos = new PNT3D[pQuick->m_nCount + 1 ] ;
			mathCpyPnt( start[0], pQuick->m_pTAPos[0] ) ;
			mathCpyPnt( start[1], pQuick->m_pTAPos[1] ) ;
			mathCpyPnt( end[0]  , pQuick->m_pTAPos[2] ) ;
		}
	}
	return pQuick ;
} 
void CSmartDrillGen::MatchingDrillType ( CPathPoint3D *DrillPnt )
{
	if( !DrillPnt ) return ;
	if( m_cParam.m_nDrillType == NCDEF_DRILLTYPE_G81 )
	{
		DrillPnt->m_bDrillType = NC_DRILL_G81 ;
	}
	else if( m_cParam.m_nDrillType == NCDEF_DRILLTYPE_G82 )
	{
		DrillPnt->m_bDrillType = NC_DRILL_G82 ;
	}
	else if( m_cParam.m_nDrillType == NCDEF_DRILLTYPE_G83 )
	{
		DrillPnt->m_bDrillType = NC_DRILL_G83 ;
	}
	else if( m_cParam.m_nDrillType == NCDEF_DRILLTYPE_G73 )
	{
		DrillPnt->m_bDrillType = NC_DRILL_G73 ;
	}
	else if( m_cParam.m_nDrillType == NCDEF_DRILLTYPE_G76 )
	{
		DrillPnt->m_bDrillType = NC_DRILL_G76 ;
	}
	else if( m_cParam.m_nDrillType == NCDEF_DRILLTYPE_G74 )
	{
		DrillPnt->m_bDrillType = NC_DRILL_G74 ;
	}
	else if( m_cParam.m_nDrillType == NCDEF_DRILLTYPE_G84 )
	{
		DrillPnt->m_bDrillType = NC_DRILL_G84 ;
	}
	else if( m_cParam.m_nDrillType == NCDEF_DRILLTYPE_G85 )
	{
		DrillPnt->m_bDrillType = NC_DRILL_G85 ;
	}
	else if( m_cParam.m_nDrillType == NCDEF_DRILLTYPE_G86 )
	{
		DrillPnt->m_bDrillType = NC_DRILL_G86 ;
	}
	else if( m_cParam.m_nDrillType == NCDEF_DRILLTYPE_G87 )
	{
		DrillPnt->m_bDrillType = NC_DRILL_G87 ;
	}
	else if( m_cParam.m_nDrillType == NCDEF_DRILLTYPE_G88 )
	{
		DrillPnt->m_bDrillType = NC_DRILL_G88 ;
	}
	else if( m_cParam.m_nDrillType == NCDEF_DRILLTYPE_G89 )
	{
		DrillPnt->m_bDrillType = NC_DRILL_G89 ;
	}
	else
	{
		DrillPnt->m_bDrillType = m_cParam.m_nDrillType ;
	}
}
void CSmartDrillGen::ConvertDrillPath( CPathCombine *PComb )
{
	if( !PComb ) return ;

	CPathEntity *pHead = PComb->m_pHead, *pNext = NULL ;
	CPathPoint3D *pPoint = NULL ;
	CPathEntity *pEntity = NULL ;
	CSmtCPathLib AllPath ;
	CSmtCutPath *pPath = NULL ;
	POSITION pos, atpos = NULL ;
	while( pHead )
	{
		pNext = pHead->next ;
		if( pHead->GetType () != NC_PATH_POINT3D )
			goto EndLine ;

		pPoint = ( CPathPoint3D *)pHead ;
		//		pEntity = ConvertDrillPathToLine( pPoint ) ;
		//		if( pEntity )
		//		{
		//			PComb->InsertAfter ( pEntity, pPoint ) ;
		//			PComb->RemoveEntity ( pPoint ) ;
		//			delete pPoint ;
		//		}
		ConvertDrillPathToLine( pPoint, AllPath ) ;
		pos = AllPath.m_cAllPath.GetTailPosition () ;
		while( pos )
		{
			atpos = pos ;
			pPath = AllPath.m_cAllPath.GetPrev ( pos ) ;
			pEntity = TransfCutPathToPathLine( pPath ) ;

			if( pEntity )
			{
				PComb->InsertAfter ( pEntity, pPoint ) ;
			}
			delete pPath ;
			AllPath.m_cAllPath.RemoveAt ( atpos ) ;
		}
		PComb->RemoveEntity ( pPoint ) ;
		delete pPoint ;

EndLine:
		pHead = pNext ;
	}
}
CPathEntity *CSmartDrillGen::ConvertDrillPathToLine ( CPathPoint3D *DrillPnt )
{
	if( !DrillPnt ) return NULL ;

	// 先将深度设好
	for( int k = 0 ; k < DrillPnt->m_nIncTime ; k++ )
	{
		DrillPnt->m_fIncAt[k][2] = DrillPnt->m_fPoint[2] + m_cParam.m_dRetractDist- DrillPnt->m_fIncAt[k][2] ;
	}

	// 退刀量和R平面高度
	double dChipDist = m_cParam.m_dChipDist ;
	PNT3D drillPos, tmpPos ;
	mathCpyPnt( DrillPnt->m_fPoint, drillPos ) ;
	drillPos[2] += m_cParam.m_dRetractDist ;

	CSmtCutPath *pPath = new CSmtCutPath() ;
	pPath->m_bFeedType = JDNC_FEEDTYPE_ROUGH ;

	// 进给
	pPath->AddPoint ( drillPos ) ;
	
	for( int i = 0 ; i < DrillPnt->m_nIncTime - 1 ; i++ )
	{
		// 添加进给
		mathCpyPnt( DrillPnt->m_fIncAt[i], tmpPos ) ;
		pPath->AddPoint ( tmpPos ) ;
		// 排屑
		if( m_cParam.m_nDrillType == NCDEF_DRILLTYPE_G83 )
		{ // G83 排屑钻孔,退刀到R平面高度
			mathCpyPnt( drillPos, tmpPos ) ;

			pPath->AddPoint ( tmpPos ) ;
		}
		else if( m_cParam.m_nDrillType == NCDEF_DRILLTYPE_G73 )
		{ // G73 高速排屑钻孔,退刀dChipDist
			tmpPos[2] += dChipDist ;
			pPath->AddPoint ( tmpPos ) ;
		}
		else
		{ // G81/G82普通钻孔,没有退刀
		}
	}

	// 最后一点和退刀
	mathCpyPnt( DrillPnt->m_fIncAt[DrillPnt->m_nIncTime - 1], tmpPos ) ;
	pPath->AddPoint ( tmpPos ) ;

	// 将pPath转为CPathPLine3D
	CPathEntity *pEntity = TransfCutPathToPathLine( pPath ) ;
	delete pPath ;

	return pEntity ;
}

BOOL CSmartDrillGen::ConvertDrillPathToLine( CPathPoint3D *DrillPnt, CSmtCPathLib &AllPath ) 
{
	if( !DrillPnt ) return FALSE ;

	// 先将深度设好
	for( int k = 0 ; k < DrillPnt->m_nIncTime ; k++ )
	{
		DrillPnt->m_fIncAt[k][2] = DrillPnt->m_fPoint[2] + m_cParam.m_dRetractDist- DrillPnt->m_fIncAt[k][2] ;
	}

	// 退刀量和R平面高度
	double dChipDist = m_cParam.m_dChipDist ;
	PNT3D drillPos, tmpPos ;
	mathCpyPnt( DrillPnt->m_fPoint, drillPos ) ;
	drillPos[2] += m_cParam.m_dRetractDist ;

	CSmtCutPath *pPath = new CSmtCutPath() ;
	pPath->m_bFeedType = JDNC_FEEDTYPE_ROUGH ;

	// 进给
	pPath->AddPoint ( drillPos ) ;

	for( int i = 0 ; i < DrillPnt->m_nIncTime - 1 ; i++ )
	{
		// 添加进给
		mathCpyPnt( DrillPnt->m_fIncAt[i], tmpPos ) ;
		pPath->AddPoint ( tmpPos ) ;

		if( m_cParam.m_nDrillType != NCDEF_DRILLTYPE_G83 && m_cParam.m_nDrillType != NCDEF_DRILLTYPE_G73 )
		{ // G81/G82普通钻孔,没有退刀
		}
		else
		{
			AllPath.AddToTail ( pPath ) ;
			pPath = new CSmtCutPath() ;
			pPath->m_bFeedType = JDNC_FEEDTYPE_QUICK ;
			pPath->AddPoint ( tmpPos ) ;
			// 排屑
			if( m_cParam.m_nDrillType == NCDEF_DRILLTYPE_G83 )
			{ // G83 排屑钻孔,退刀到R平面高度
				mathCpyPnt( drillPos, tmpPos ) ;
				pPath->AddPoint ( tmpPos ) ;
				// 慢下距离
				mathCpyPnt( DrillPnt->m_fIncAt[i], tmpPos ) ;
				tmpPos[2] += dChipDist ;
				pPath->AddPoint ( tmpPos ) ;
			}
			else // m_cParam.m_nDrillType == NCDEF_DRILLTYPE_G73 )
			{ // G73 高速排屑钻孔,退刀dChipDist
				tmpPos[2] += dChipDist ;
				pPath->AddPoint ( tmpPos ) ;
			}
			// 将排屑路径放到AllPath
			AllPath.AddToTail ( pPath ) ;

			// 路径
			pPath = new CSmtCutPath() ;
			pPath->m_bFeedType = JDNC_FEEDTYPE_ROUGH ;
			pPath->AddPoint ( tmpPos ) ;
		}
	}

	// 最后一点和退刀
	mathCpyPnt( DrillPnt->m_fIncAt[DrillPnt->m_nIncTime - 1], tmpPos ) ;
	pPath->AddPoint ( tmpPos ) ;

	AllPath.AddToTail ( pPath ) ;
	return TRUE ;
}
CPathEntity *CSmartDrillGen::CreateQuickLine ( CPathCombine *PComb )
{
	if( !PComb || !PComb->m_pTail || PComb->m_pTail->GetType () != NC_PATH_POINT3D ) 
		return NULL ;
	PNT3D tmpPos[2] ;
	CPathPoint3D *DrillPnt = ( CPathPoint3D *)PComb->m_pTail ;
	mathCpyPnt( DrillPnt->m_fPoint, tmpPos[0] ) ;
	tmpPos[0][2] = DrillPnt->m_fPoint[2] + m_cParam.m_dRetractDist- DrillPnt->m_fIncAt[DrillPnt->m_nIncTime - 1][2] ;
	// 退刀
	mathCpyPnt( DrillPnt->m_fPoint, tmpPos[1] ) ;
	if( m_cParam.m_nRetractM == NCDEF_DRILLRETRACT_RTOP )
		tmpPos[1][2] += m_cParam.m_dRetractDist ;
	else
		tmpPos[1][2] += m_cSpeedDef.m_dSafeHeight ;

	CPathPLine3D *pLine = new CPathPLine3D() ;
	pLine->m_nCount = 1 ;
	pLine->m_pTAPos = new TPNT3D[2] ;
	
	for( int i = 0 ; i < 3 ; i++ )
	{
		pLine->m_pTAPos[0][i] = tmpPos[0][i] ;
		pLine->m_pTAPos[1][i] = tmpPos[1][i] ;
	}
	pLine->m_bFeedType = JDNC_FEEDTYPE_QUICK ;
	return pLine ;
}
CPathEntity *CSmartDrillGen::TransfCutPathToPathLine ( CSmtCutPath *pPath )
{
	if( !pPath ) return NULL ;

	CPathPLine3D *pLine = new CPathPLine3D() ;
	pLine->m_nCount = pPath->m_nNumPnt - 1 ;
	pLine->m_bFeedType = pPath->m_bFeedType ;
	pLine->m_pTAPos = new TPNT3D[pPath->m_nNumPnt] ;
	int i = 0, nCnt = 0 ;
	CSmtCutPoint *pHead = pPath->m_pHead ;
	for( ; pHead ; pHead = pHead->next )
	{
		for( i = 0 ; i < 3 ; i++ )
		{
			pLine->m_pTAPos[nCnt][i] = pHead->m_fPoint[i] ;
		}
		nCnt++ ;
	}
	return pLine ;
}
CSmartBoreGen::CSmartBoreGen()
{
}
CSmartBoreGen::~CSmartBoreGen()
{
}

// 计算扩孔路径
BOOL CSmartBoreGen::GeneratePathEx( CPathGroup& NewPath ,   /*雕刻路径*/ 
		                            CSmartGraphic& Graph )  /*雕刻参数*/
{
	// STEP 0 : 校正雕刻环境
	if( !UpdateTool2D() ) return FALSE;

	CPtrList  AllEnt ;
    Graph.GetAllEntity( m_cSetupDef, AllEnt, SMARTGRAPH_TYPE_POINT ) ; 
	Graph.GetFeaturePoint( m_cSetupDef, AllEnt, m_cParam.m_cPointDef ) ;
	if( AllEnt.GetCount() == 0 )
	{
		m_nErrorType = JDERROR_GENPATH_NOPOINT ;
		return FALSE ;
	}
	CPointList  AllPoint ;
	CSmartPoint* pPoint ;
	POSITION pos = AllEnt.GetHeadPosition() ;
	while( pos ) 
	{
		pPoint = ( CSmartPoint*) AllEnt.GetNext( pos ) ;
		pPoint->m_dPoint[2] = 0.0 ;
	    if( !( m_cParam.m_bBoreFlag & NCDEF_BORE_FLTSAME ) ||
			  AllPoint.FindPoint( pPoint->m_dPoint ) < 0  )
		{
			AllPoint.InsertAfter( pPoint, AllPoint.GetTail() ) ;  
		}
		else
		{ // 删除重点
			delete pPoint ;
		}
	} 
    AllEnt.RemoveAll() ;
    if( GetSortType() != NCDEF_SORTTYPE_SELECT )
    {
	    AllPoint.ResortAllPoint( GetSortType(), TRUE , float( m_cToolDef.m_dTopDiam * 0.5f)) ;
    }
	// STEP 2 : 计算圆弧半径、刀具补偿
	DOUBLE dRadius = m_cParam.m_dHoleDiameter / 2.0 - GetSideStock();
	if( m_cShapeDef.m_bAntiFace ) 
	{
		dRadius -= m_pTool->GetRadiusComp( m_cShapeDef.m_cDepth.m_dCutDepth   , 
			                    m_cShapeDef.m_cTaper.m_dBoundAngle) ;
	}
	else
	{
		dRadius -= m_pTool->GetRadiusComp( m_cShapeDef.m_cDepth.m_dCutDepth, 
			                    m_cShapeDef.m_cTaper.m_dBoundAngle) ;
	}
    if( dRadius < 0.02 )
	{
		m_nErrorType = JDERROR_GENPATH_ERRDIAM ;
		return  FALSE ;
	}
	JDNC_LAYER * pLayer   = GetLayerDef() ;
	pLayer->m_dTotalDepth = GetCutDepth()  - m_cStockDef.m_dDepthStock; 
	DefineLayerDepth( * pLayer) ; 
    CPathCombine *pTComb = new CPathCombine( NC_WPROCESS_ROUGH ) ;
	CSmartArc ArcSect ;
	if( GetMillDir() == 1 ) 
	{ // 顺铣 
	   ArcSect.m_aAngle[0] = 0.0 , ArcSect.m_aAngle[1] = MiniPai2 ;
	}
	else
	{
	   ArcSect.m_aAngle[0] = MiniPai2 , ArcSect.m_aAngle[1] = 0.0 ;
	}
	ArcSect.m_dRadius    = dRadius  ;
    int nRCompMask = 0 ;
    if( m_cSetupDef.m_cModelTol.m_nMdlFlag & NCDEF_SETUP_WEARCOMP_INC )
    {
        if( GetMillDir() == 0 ) nRCompMask = JDNC_FGCODE_RCOMPG41 ;
        nRCompMask = JDNC_FGCODE_RCOMPG42 ;
    }
    else if( m_cSetupDef.m_cModelTol.m_nMdlFlag & NCDEF_SETUP_WEARCOMP_DEC )
    {
        if( GetMillDir() == 0 ) nRCompMask = JDNC_FGCODE_RCOMPG42 ;
        nRCompMask = JDNC_FGCODE_RCOMPG41 ;
    }
    JDNC_LEAD  rcompLead = m_cFeedDef.m_cLeadDef ;
    rcompLead.m_dLength = max( 0.05, m_cFeedDef.m_cLeadDef.m_dWearLine) ;
    if( rcompLead.m_nLeadType == 0 ) rcompLead.m_nLeadType = NCDEF_LEAD_SLINE ;
    else   rcompLead.m_nLeadType = NCDEF_LEAD_LINE ;
    DOUBLE rcompZShift[2] = { 0.0, 0.0 } ;
    JDNC_LEAD  leadIn  = m_cFeedDef.m_cLeadDef ;
    JDNC_LEAD  leadOut = m_cFeedDef.m_cLeadDef ;
  	DOUBLE ZShift[2] = {} ;
    CSmartLoop tmpCont ;
    CSmartCurve* pCurve = new CSmartCurve() ;
    DOUBLE dAng[2] = { 0.0, MiniPai2 } ;
    DOUBLE dCen[2] = { 0.0, 0.0 } ;
    DOUBLE dRad  = ArcSect.m_dRadius ;
    pCurve->AddSect( new CSmartArc( dCen, dAng, dRad ) ) ;
    tmpCont.UpdateSect( pCurve ) ;
	// STEP 3 : 计算雕刻路径
    CPathCombine tmpTComb(NC_WPROCESS_ROUGH) ;
	for( pPoint = AllPoint.GetHead() ; pPoint ; pPoint = pPoint->next)
	{
		ArcSect.m_aCenter[0] = pPoint->m_dPoint[0] ;
		ArcSect.m_aCenter[1] = pPoint->m_dPoint[1] ;
        tmpCont.m_pCurve->MoveCurve( pPoint->m_dPoint[0],pPoint->m_dPoint[1] ) ;
        tmpCont.DefineBox() ;
        if( !( m_cParam.m_bBoreFlag & NCDEF_BORE_HELIX ) )
        {/*步进扩孔*/
		    for( int i = 1 ; i <= m_cFeedDef.m_cLayerDef.m_nLayerCount ; i ++ ) 
		    {
			    double dDepth    = pLayer->m_dLayerDepth[i] ; 
				double dDepthInc = dDepth - pLayer->m_dLayerDepth[i-1] ;
				CPathArc3D  *pArc3D  = new CPathArc3D() ;
				pArc3D->Create( -dDepth, ArcSect ) ;
				tmpTComb.AddEntity( pArc3D ) ;
		        AddLeadPathAtEndEx( leadIn, leadOut, tmpTComb, tmpCont, dDepth, ZShift, dDepthInc) ;						
                if( nRCompMask != 0 )
                {
					int bFlag = AddLeadPathAtEndEx( rcompLead, rcompLead, tmpTComb, tmpCont, 0.0, rcompZShift) ;
					AddRCompMask( tmpTComb, nRCompMask ) ;
					if (!bFlag)
					{
						m_bRComb = 2;
					}                   
                }
                pTComb->AppendCombine( tmpTComb ) ;
            }
        }
        else
        {/*螺旋扩孔*/
			double dDepth = 0.;
			double dDepthInc = 0.;
			// 沿轮廓进退刀时关闭进刀，保留退刀 qqs 2013.02.18
			if (leadIn.m_nLeadType == NCDEF_LEAD_BYCONTOUR)
			{
				leadIn.m_nLeadType = NCDEF_LEAD_CLOSE;
			}
		    for( int i = 1 ; i <= m_cFeedDef.m_cLayerDef.m_nLayerCount ; i ++ ) 
		    {
			    dDepth    = pLayer->m_dLayerDepth[i] ; 
			    dDepthInc = dDepth - pLayer->m_dLayerDepth[i-1] ;
				CPathArc3D  *pArc3D  = new CPathArc3D() ;
				pArc3D->Create( -(dDepth - dDepthInc) , ArcSect ) ;
				pArc3D->m_fDepth = -dDepthInc ;
				tmpTComb.AddEntity( pArc3D ) ;
                if( !( m_cParam.m_bBoreFlag & NCDEF_BORE_FINISHEND ) || 
                    i == m_cFeedDef.m_cLayerDef.m_nLayerCount  )
                {
				    CPathArc3D  *pArc3D  = new CPathArc3D() ;
				    pArc3D->Create( -dDepth, ArcSect ) ;
				    tmpTComb.AddEntity( pArc3D ) ;
                }
		    }
		    AddLeadPathAtEndEx( leadIn, leadOut, tmpTComb, tmpCont, pLayer->m_dTotalDepth , ZShift, dDepthInc) ;						
            if( nRCompMask != 0 )
            {
				int bFlag = AddLeadPathAtEndEx( rcompLead, rcompLead, tmpTComb, tmpCont, 0.0, rcompZShift) ;
				AddRCompMask( tmpTComb, nRCompMask ) ;
				if ( !bFlag)
				{
					m_bRComb = 2;
				}                                    
            }
            pTComb->AppendCombine( tmpTComb ) ;
        }
        tmpCont.m_pCurve->MoveCurve( -pPoint->m_dPoint[0],-pPoint->m_dPoint[1] ) ;
	}
    NewPath.AddData( 0.0, pTComb ) ;
	// 校正表面高度
	NewPath.ZMoveGroup( GetTopHeight() ) ;
	if( pLayer->m_dLayerDepth )
    {
        delete[] pLayer->m_dLayerDepth ;
	    pLayer->m_dLayerDepth = NULL ;
    }
	return TRUE ;
}

/*当加工方法为单线切割和轮廓切割且尖角形式为延伸圆弧时调用*/
void ResetCurve( CSmartCurve* pCurve, 
				DOUBLE dRadius, /*延伸圆弧半径,输入参数*/
				int nRComp    /*半径补偿方向*/ )
			
{
	if( pCurve == NULL ) 
		return ;
	if( nRComp == 0 )//关闭补偿
		return ;/**/
	
	if( dRadius < 0.01 ) 
		return ;
	DOUBLE dAng1 = 1. / 6. * PI1;
	DOUBLE dAng2 = 11. / 6. * PI1;

	for( CSmartSect* pSect = pCurve->m_pHead ; pSect ; pSect = pSect->next )
	{
		//int nCurType = pSect->GetType() ;
		DOUBLE dLen = pSect->GetLength() ;
		if( dLen < 0.1 ) 
		{
			if( pSect->next )
				pSect = pSect->next ;
			else 
				return ;
		}
		int nNxtType ;
		if( pSect->next )
		{
			nNxtType = pSect->next->GetType() ;
			dLen = pSect->next->GetLength() ;

			if(dLen > 0.1)
			{
				PNT2D e1, c1, e2, c2 ;
				VEC2D vn1, vt1, vn2, vt2 ;
				VEC3D tmpVt2 ;
				CSmartArc* pArc1 ;
				CSmartLine* pLn1, *pLn2 ;
				pSect->GetPoint( 1., e1 ) ;
				pSect->GetNormal( 1., vn1 ) ;
				pSect->GetTangent(1., vt1 ) ;
				pSect->next->GetStart( e2 ) ;
				pSect->next->GetNormal( 0., vn2 ) ;
				pSect->next->GetTangent( 0., vt2 ) ;

				tmpVt2[0] = vt1[0];
				tmpVt2[1] = vt1[1];
				tmpVt2[2] = 0.0 ;
				DOUBLE dAng = mathGetACWAngle2D( tmpVt2, vt2 ) ;
				if( (dAng > dAng1) && (dAng < dAng2) )/*角度判断条件*/
				{
					if( nRComp == 1 )
					{//轮廓切割dRComp<0
						if( dAng < PI1)
							continue ;
					}
					else if( nRComp == 2 )
					{
						if( dAng > PI1 )
						{//连接方向为逆时针
							continue ;
						}
					}
					
					DOUBLE dA = fabs( (dAng - PI1) / 2.0 )  ;
					DOUBLE dDist = fabs(dRadius / tan(dA) );
					c1[0] = e1[0] + dDist * vt1[0] ;
					c1[1] = e1[1] + dDist * vt1[1] ;
					c2[0] = e2[0] - dDist * vt2[0] ;
					c2[1] = e2[1] - dDist * vt2[1] ;
					pArc1 = new CSmartArc() ;
					pArc1->Define2PTArc( c1, c2, vt1 ) ;
					if( pArc1 )
					{
						pLn1 = new CSmartLine( e1, c1 ) ;
						pCurve->InsertAfter( pLn1, pSect ) ;
						pCurve->InsertAfter( pArc1, pLn1 ) ;
						pLn2 = new CSmartLine( c2, e2 ) ;
						pCurve->InsertAfter( pLn2, pArc1 ) ;
					}
				}
			}
		}

	}
}

CSmartCurveGen::CSmartCurveGen()
{
}
CSmartCurveGen::~CSmartCurveGen()
{
	MathCam_DeleteAllRedepthCur( m_cListCur ) ;
}

static void SmartMill_SortAllComb( CPtrList& AllComb, int SortType  )
{
    CPathCombine* pPComb ;
    PNT3D dStart, dEnd, dMidPnt ;
    memset( dStart, 0 , sizeof( PNT3D )) ;
    memset( dEnd, 0 , sizeof( PNT3D )) ;
   if( SortType == NCDEF_SORTTYPE_XZIGZAG || SortType == NCDEF_SORTTYPE_YZIGZAG ||
       SortType == NCDEF_SORTTYPE_XONEWAY || SortType == NCDEF_SORTTYPE_YONEWAY )
    {
        CPointList sortList ;
        CSmartSPoint * pPoint ;
        while( AllComb.GetCount() ) 
        {
            pPComb = (CPathCombine*)AllComb.RemoveHead() ;
            pPComb->GetEndPoint( 0, dStart ) ;
            pPComb->GetEndPoint( 1, dEnd   ) ;
            nc_GetPointAtLine( dStart, dEnd, 0.5, dMidPnt, 3 ) ;
            pPoint = new CSmartSPoint( dMidPnt, pPComb ) ;
		    sortList.InsertAfter( pPoint, sortList.m_pTail ) ;
        }
        sortList.ResortAllPoint( SortType, TRUE, 0.02f );
        for( pPoint = (CSmartSPoint*)sortList.m_pHead ; pPoint ; pPoint = (CSmartSPoint*)pPoint->next )
        {
            pPComb = (CPathCombine*) pPoint->m_pData ;
            AllComb.AddTail( pPComb ) ;
        }
        return ;
    }
    else if( SortType == NCDEF_SORTTYPE_TOOUTER )
    {/*从里向外, 保留原有次序排序*/
        return ;
    }
	else if( SortType == NCDEF_SORTTYPE_SELECT  )
	{/*按照选择次序排序*/
		return ;
	}
    else if( SortType == NCDEF_SORTTYPE_TOINNER )
    {/*从里向外, 原有次序反向排序*/
        CPtrList oldList ;
        while( AllComb.GetCount() ) 
        {
            pPComb = (CPathCombine*)AllComb.RemoveHead() ;
            oldList.AddTail( pPComb ) ;
        }
        while( oldList.GetCount() ) 
        {
            pPComb = (CPathCombine*)oldList.RemoveHead() ;
            AllComb.AddHead( pPComb ) ;
        }
        return ;
    }
    CPtrList tmpList ;
    while( AllComb.GetCount() ) 
    {
        pPComb = (CPathCombine*)AllComb.RemoveHead() ;
        tmpList.AddTail( pPComb ) ;
    }
    while( tmpList.GetCount() ) 
    {
        CPathCombine * pObj = NULL ; 
        DOUBLE dMinDist = 1.0e12, dDist ;
        POSITION atPos = tmpList.GetHeadPosition() ;
        while( atPos )
        {
			pPComb = ( CPathCombine*) tmpList.GetNext( atPos ) ;
            if( AllComb.GetCount() == 0 ) 
            {
                pObj = pPComb  ; 
                break ;
            }
            if( SortType == NCDEF_SORTTYPE_MINAREA )
            {
                dDist = pPComb->GetPathLen() ;
            }
            else
            {
                pPComb->GetEndPoint( 0, dStart ) ;
                dDist = nc_Distance( dStart, dEnd, 2 ) ;
            }
            if( pObj == NULL || dDist < dMinDist )
            {
                pObj = pPComb , dMinDist = dDist ;
            }
        }
        atPos = tmpList.Find( pObj ) ;
        tmpList.RemoveAt( atPos ) ;
        AllComb.AddTail( pObj ) ;
        pObj->GetEndPoint(1, dEnd ) ;
    }
    return ;
}
static void SmartMill_SortAllCurveByDist( CPtrList &AllCurve, BOOL bAllowRev, BOOL bDepth )
{
	 if( AllCurve.GetCount() <= 1 ) 
    {
        return ;
    }
	POSITION pos, atpos, atFind = NULL ;
	CSmartCurve *pCurve = NULL, *pNext = NULL, *pFind = NULL ;
	double dMin = 1.0e10, dDist[2] = { 0., 0. } ;
	CPtrList sortList ;
	pCurve = ( CSmartCurve *)AllCurve.RemoveHead () ;
	sortList.AddTail ( pCurve ) ;
	PNT3D p, start, end ;
	BOOL bReverse = FALSE ;
	while( AllCurve.GetCount () > 0 )
	{
		pFind = NULL ;
		dMin = 1.0e10 ;
		bReverse = FALSE ;
		pCurve->GetEnd ( p ) ;
		if( bDepth ) p[2] = pCurve->m_pTail->m_dZValue[1] ;
		pos = AllCurve.GetHeadPosition () ;
		while( pos )
		{
			atpos = pos ;
			pNext = ( CSmartCurve *)AllCurve.GetNext ( pos ) ;
			pNext->GetStart ( start ) ;
			pNext->GetEnd ( end ) ;
			if( bDepth ) 
			{
				start[2] = pNext->m_pHead->m_dZValue[0] ;
				end[2]   = pNext->m_pTail->m_dZValue[1] ;
			}

			dDist[0] = mathDist2D( p, start ) ;
			dDist[1] = mathDist2D( p, end ) ;
			if( bDepth )
			{
				dDist[0] = mathDist( p, start ) ;
				dDist[1] = mathDist( p, end   ) ;
			}
			if( bAllowRev )
			{
				if( dDist[0] < dMin || dDist[1] < dMin )
				{
					pFind = pNext ;
					atFind = atpos ;
					dMin = min( dDist[0], dDist[1] ) ;

					if( dDist[0] > dDist[1] ) 	bReverse = TRUE ;
					else						bReverse = FALSE ;
				}
			}
			else
			{
				if( dDist[0] < dMin )
				{
					pFind = pNext ;
					atFind = atpos ;
					dMin = dDist[0] ;
				}
			}
		}
		if( pFind )
		{
			if( bReverse && bAllowRev ) pFind->Reverse () ;
			sortList.AddTail ( pFind ) ;
			AllCurve.RemoveAt ( atFind ) ;
			pCurve = pFind ;
		}
		else
		{
			break ;
		}
	}
	AllCurve.RemoveAll () ;
	AllCurve.AddTail ( &sortList ) ;
	sortList.RemoveAll () ;
}
static void SmartMill_SortAllCurve( CPtrList& AllCurve, int SortType, BOOL bAllowRev, BOOL bDepth  )
{
	if (AllCurve.IsEmpty() || AllCurve.GetCount() < 2)
	{
		return;
	}

    CSmartCurve* pCurve ;
    PNT3D dStart, dEnd, dMidPnt ;
    memset( dStart, 0 , sizeof( PNT3D )) ;
    memset( dEnd, 0 , sizeof( PNT3D )) ;
    if( SortType == NCDEF_SORTTYPE_XZIGZAG || SortType == NCDEF_SORTTYPE_YZIGZAG ||
        SortType == NCDEF_SORTTYPE_XONEWAY || SortType == NCDEF_SORTTYPE_YONEWAY )
    {
        CPointList sortList ;
        CSmartSPoint * pPoint ;
        while( AllCurve.GetCount() ) 
        {
            pCurve = (CSmartCurve*)AllCurve.RemoveHead() ;
            pCurve->GetStart( dStart ) ;
            pCurve->GetEnd( dEnd   ) ;
            nc_GetPointAtLine( dStart, dEnd, 0.5, dMidPnt, 2 ) ;
            pPoint = new CSmartSPoint( dMidPnt, pCurve ) ;
		    sortList.InsertAfter( pPoint, sortList.m_pTail ) ;
        }
        sortList.ResortAllPoint( SortType, TRUE, 0.02f );
        for( pPoint = (CSmartSPoint*)sortList.m_pHead ; pPoint ; pPoint = (CSmartSPoint*)pPoint->next )
        {
            pCurve = (CSmartCurve*) pPoint->m_pData ;
            AllCurve.AddTail( pCurve ) ;
        }
        return ;
    }
	else if( SortType == NCDEF_SORTTYPE_SELECT  )
	{/*按照选择次序排序*/
		return ;
	}
    else if( SortType == NCDEF_SORTTYPE_TOOUTER )
    {/*从里向外, 保留原有次序排序*/
        return ;
    }
    else if( SortType == NCDEF_SORTTYPE_TOINNER )
    {/*从里向外, 原有次序反向排序*/
        CPtrList oldList ;
        while( AllCurve.GetCount() ) 
        {
            pCurve = (CSmartCurve*)AllCurve.RemoveHead() ;
            oldList.AddTail( pCurve ) ;
        }
        while( oldList.GetCount() ) 
        {
            pCurve = (CSmartCurve*)oldList.RemoveHead() ;
            AllCurve.AddHead( pCurve ) ;
        }
        return ;
    }
	else if( SortType == NCDEF_SORTTYPE_MINDIST )
	{
		SmartMill_SortAllCurveByDist( AllCurve, bAllowRev, bDepth ) ;
		return ;
	}
    CPtrList tmpList ;
    while( AllCurve.GetCount() ) 
    {
        pCurve = (CSmartCurve*)AllCurve.RemoveHead() ;
        tmpList.AddTail( pCurve ) ;
    }
    while( tmpList.GetCount() ) 
    {
        CSmartCurve * pObj = NULL ; 
        DOUBLE dMinDist = 1.0e12, dDist ;
        POSITION atPos = tmpList.GetHeadPosition() ;
        while( atPos )
        {
			pCurve = ( CSmartCurve*) tmpList.GetNext( atPos ) ;
            if( AllCurve.GetCount() == 0 ) 
            {
                pObj = pCurve  ; 
                break ;
            }
            if( SortType == NCDEF_SORTTYPE_MINAREA )
            {
                dDist = pCurve->GetLength() ;
            }
            else
            {
                pCurve->GetStart( dStart ) ;
                dDist = nc_Distance( dStart, dEnd, 2 ) ;
            }
            if( pObj == NULL || dDist < dMinDist )
            {
                pObj = pCurve , dMinDist = dDist ;
            }
        }
        atPos = tmpList.Find( pObj ) ;
        tmpList.RemoveAt( atPos ) ;
        AllCurve.AddTail( pObj ) ;
        pObj->GetEnd( dEnd ) ;
    }
    return ;
}

// 路径反向,调用CPathCombine的圆弧反向有错误
void ReversePComb( CPathCombine &PComb )
{
	if( !PComb.m_pHead ) return ;
	CPathEntity *pCurr = PComb.m_pTail , *pTemp = NULL ;
	PComb.m_pHead = PComb.m_pTail = NULL ;
	CPathArc3D *pArc = NULL ;
	PNT3D start, end, restart, reend ;

	while( pCurr )
	{
		pTemp = pCurr->prev ;
		pCurr->prev = pCurr->next = NULL ;

		pCurr->GetEndPoint( 0, start ) ;
		pCurr->GetEndPoint( 1, end   ) ;

		pCurr->ReverseDirect() ;
		if( pCurr->GetType () == NC_PATH_ARC3D )
		{
			pArc = ( CPathArc3D *)pCurr ;
			pArc->m_fCenter[2] += pArc->m_fDepth;
			pArc->m_fDepth = -pArc->m_fDepth ;
		}

		pCurr->GetEndPoint( 0, restart ) ;
		pCurr->GetEndPoint( 1, reend   ) ;

		PComb.AddEntity( pCurr ) ;
		pCurr = pTemp ; 
	}
}

BOOL CSmartCurveGen::GeneratePathEx(CPathGroup& NewPath       ,  /*雕刻路径*/ 
                                    CSmartGraphic& Graph)  /*图形函数*/
{
	
	if( !UpdateTool2D() ) return FALSE ;
    if(  m_cParam.m_nRCompSide != 2 && 
		( m_cSlayerDef.m_nLayerType != NCDEF_SLAYER_CLOSE) &&
		  !( m_cParam.m_bCurveFlag & NCDEF_FCURVE_GRNDMDFY )) 
    {
        return CreateBoundCurve( NewPath, Graph ) ;
    }
	// STEP 0 : 取得局部坐标系
	RFRAME LocFrame ;
	if( !Graph.GetTPlaneFrame( LocFrame ) )
	{
		mathInitRFrame( &LocFrame ) ;
	}

	if (m_c5DCtrlDef.m_cToolAxisApp.m_nFrameFlag != NCDEF_FRAME_DEFAULT)
	{
		mathMultRFrame( &LocFrame, &m_c5DCtrlDef.m_cToolAxisApp.m_dIndexFrame, &LocFrame ) ;
	}
	// STEP 1 : 提取加工图形
	BOOL bDepth = FALSE ;
	if( m_cParam.m_bCurveFlag & NCDEF_FCURVE_REDEPTH )
		bDepth = TRUE ;
	CPtrList   AllCurve , AllComb, tmpList ;
	int bEntType = SMARTGRAPH_TYPE_CURVE ;
	BOOL bAllowRev = FALSE ;
	if( m_cParam.m_nRCompSide == 2 ) bAllowRev = TRUE ;

	if( m_cParam.m_bCurveFlag & NCDEF_FCURVE_REDEPTH ) 
	{
		Graph.GetAllEntity( m_cSetupDef, m_cListCur, SMARTGRAPH_TYPE_CURVE2D ) ;
		bEntType = SMARTGRAPH_TYPE_CURVE3D ;
	}

	if( ( m_cParam.m_bCurveFlag & NCDEF_FCURVE_REDEPTH  ) && 
		m_cParam.m_nRCompSide == 2  )
	{	// 保留曲线高度 &&  关闭半径补偿, 支持空间曲线
		// 2012/07/24,不应该支持空间曲线，螺旋线在投影的时候重叠，
		// 在后期保留高度的时候很难追踪到原始高度
		Graph.GetAllCurveAndVerticalPath( m_cSetupDef, AllCurve, AllComb ) ;
        SmartMill_SortAllComb( AllComb , GetSortType()) ;
		POSITION  atPos = AllComb.GetHeadPosition() ;
		while( atPos )
		{
		    CPathCombine* pSelComb = (CPathCombine*) AllComb.GetNext(atPos) ;
            for( CPathEntity* pEnt = pSelComb->m_pHead ; pEnt ; pEnt = pEnt->next )
            {
                pEnt->m_bFeedType = JDNC_FEEDTYPE_ROUGH ;
                pEnt->m_fFeedScale= 1.0f ;
            }
        }
		SmartMill_SortAllCurve( AllCurve , GetSortType(), bAllowRev, bDepth ) ;
	}
	else 
	{
	    Graph.GetAllEntity( m_cSetupDef, AllCurve, bEntType  ) ;
        SmartMill_SortAllCurve( AllCurve , GetSortType(), bAllowRev, bDepth ) ;
	}
    DOUBLE     dTopHeight = -1.0e6 ;
	if( AllComb.GetCount() == 0 && AllCurve.GetCount() == 0 )
	{ // 校正图形
		m_nErrorType = JDERROR_GENPATH_NOCURVE ;
		return FALSE ;
	}
    if( m_cParam.m_bCurveFlag & NCDEF_FCURVE_REDEPTH ) 
	{
        double dMin[3], dMax[3] ;
        POSITION atPos = AllComb.GetHeadPosition() ;
        while( atPos )
        {
            CPathCombine* pTComb = (CPathCombine*) AllComb.GetNext( atPos ) ;
            if( pTComb->GetBoundBox( dMin , dMax ) )
            {
                if( dTopHeight < dMax[2] ) dTopHeight = dMax[2] ;
            }
        }
		atPos = AllCurve.GetHeadPosition() ;
		while( atPos )
		{
			CSmartCurve* pCurve = ( CSmartCurve*) AllCurve.GetNext( atPos ) ;
            for( CSmartSect* pSect = pCurve->m_pHead ; pSect ; pSect = pSect->next )
            {
                if(  dTopHeight < pSect->m_dZValue[0]) dTopHeight = pSect->m_dZValue[0] ; 
                if(  dTopHeight < pSect->m_dZValue[1]) dTopHeight = pSect->m_dZValue[1] ; 
            }
		}
    }
	
	// STEP 2 : 计算分层深度
	JDNC_LAYER * pLayer   = GetLayerDef() ;
	pLayer->m_dTotalDepth = GetCutDepth()  - m_cStockDef.m_dDepthStock; 
	if(m_cParam.m_bCurveFlag & NCDEF_FCURVE_GRNDMDFY )//磨削调整时关闭分层
	{
		pLayer->m_nLayerType = NCDEF_LAYER_CLOSE;
	}
	DefineLayerDepth( * pLayer) ; 
	if( (m_cParam.m_bCurveFlag & NCDEF_FCURVE_REDEPTH ) &&
		(m_pTool->GetType() == smtToolBall ) && 
		(m_cParam.m_nRCompSide != 2 ) && 
		!(m_cParam.m_bCurveFlag & NCDEF_FCURVE_USERRCOMP) )
	{
	    double dAngle = ANGLE_TO_RADIAN( m_cShapeDef.m_cTaper.m_dBoundAngle  ) ;
		double dZMove = m_pTool->m_fRadius * ( 1.0 - sin( dAngle ) );
	    for( int k = 1 ; k <= pLayer->m_nLayerCount ;  k ++ ) 
		{
			pLayer->m_dLayerDepth[k] += dZMove ;
		}
		pLayer->m_dTotalDepth += dZMove ;
	}
	// STEP 3 : 计算偏移值
	DOUBLE dRComp , dAngle; 
	BOOL bSameOff = FALSE ;
	dAngle = ANGLE_TO_RADIAN( m_cShapeDef.m_cTaper.m_dBoundAngle  ) ;
    if( m_cParam.m_nRCompSide ==  2 )
	{ /* 关闭补偿 */
		dRComp = 0.0  ;
		bSameOff  = TRUE ;
	}
	else if( m_cParam.m_bCurveFlag & NCDEF_FCURVE_USERRCOMP ) 
	{ /*  定义补偿值 */
		dRComp    = m_cParam.m_dRCompDist + GetSideStock(); 
		bSameOff  = TRUE ;
	}
	else if( m_cShapeDef.m_bAntiFace )
	{ /* 底面效果 */
		dRComp    = m_pTool->GetRadiusComp( 0.0, 0.0 ) + GetSideStock(); 
		bSameOff  = TRUE ;
	}
	else if(  m_cFeedDef.m_cLayerDef.m_bLayerFlag & NCDEF_LAYER_COPYMODE )
	{ /*  拷贝分层  */
		dRComp = m_pTool->GetRadiusComp( GetCutDepth(), dAngle) + GetSideStock(); 
		bSameOff  = TRUE ;
	}
	else dRComp = 0.0 ;
	BOOL bAntiFace = m_cShapeDef.m_bAntiFace ;
    if( (m_cParam.m_nRCompSide ==  2) ||
        (m_cParam.m_bCurveFlag & NCDEF_FCURVE_USERRCOMP) ||
		(pLayer->m_nLayerCount <= 1 ) ||
		(pLayer->m_bLayerFlag & NCDEF_LAYER_COPYMODE) )
	{// 关闭补偿 ||定义补偿值|| 不分层 ||  选择拷贝分层
		bAntiFace = FALSE ;
	}
	// STEP 4 : 计算切割路径
	CSmartCurve * pCurve, *pCv, *pCvHead  ; 
	if( m_cParam.m_nRCompSide == 1 )
	{ // 向右补偿
		POSITION pos = AllCurve.GetHeadPosition() ;
		while( pos )
		{
			pCurve = ( CSmartCurve*) AllCurve.GetNext( pos ) ;
			pCurve->Reverse() ;
		}
	}
	
	// STEP 6 : 生成路径
	DOUBLE dDepth , dDepthInc, dAllComp = 0. ;
	BOOL bFst = TRUE ;
	if( GetLayerOrder() == 1 )
	{ // 高度优先
		for( int i = 1 ; i <= pLayer->m_nLayerCount ;  i ++ ) 
		{
			dDepth = pLayer->m_dLayerDepth[i] ; 
			dDepthInc = dDepth - pLayer->m_dLayerDepth[i-1] ;
			if( bSameOff == 0 )
			{
				dRComp = m_pTool->GetRadiusComp( dDepth, dAngle ) + GetSideStock() ;
			}
			/*添加平面曲线路径*/
			POSITION pos = AllCurve.GetHeadPosition() ;
			while( pos )
			{
				dAllComp = 0. ;
				BOOL bRev = FALSE ;
	 			pCurve = ( CSmartCurve*) AllCurve.GetNext( pos ) ;
	            CPathCombine* pTComb = new CPathCombine( NC_WPROCESS_ROUGH ) ;
				if( m_cSetupDef.m_cCorDef.m_nCorType == NCDEF_CORNER_EXTARC && 
					(dRComp >= 1.0e-6  || dRComp <= -1.0e-6)  )//延伸圆弧
				{//先按照直线延长方式偏移曲线，然后再延伸圆弧
					m_cSetupDef.m_cCorDef.m_nCorType = NCDEF_CORNER_EXTUSER ;
					pCvHead = GenerateOffsetCurve(pCurve, dRComp);
					dAllComp = dRComp ; 
					m_cSetupDef.m_cCorDef.m_nCorType = NCDEF_CORNER_EXTARC ;
					if( dRComp < 0 )
					{
						pCvHead->Reverse() ;
						bRev = TRUE ;
					}
				}
				else
				{
					pCvHead = GenerateOffsetCurve(pCurve, dRComp);
					dAllComp = dRComp ;
				}
				if( bAntiFace && pCvHead  )
				{ // 底面雕刻效果
					double dOffComp ;
					dOffComp = m_pTool->GetRadiusComp( pLayer->m_dTotalDepth - dDepth, dAngle ) 
								   - m_pTool->GetRadiusComp( 0.0,0.0 )  ;
					if( dOffComp > 1.0e-3 )
					{
						CSmartCurve* pAtCurve = pCvHead ;
						pCvHead = Mini_OffsetCurveList( pAtCurve, NCDEF_OFFSET_OUTER, dOffComp, GetCorDef() ) ;
						dAllComp += dOffComp ;
						Mini_DeleteCurves( pAtCurve ) ;
					}
				}
				for( pCv = pCvHead ; pCv ; pCv = pCv->next )
				{
					if(m_cSetupDef.m_cCorDef.m_nCorType == NCDEF_CORNER_EXTARC &&
						(dRComp >= 1.0e-6  || dRComp <= -1.0e-6) )
					{
						ResetCurve( pCv, m_cProcessDef.m_dExtRadius, 2) ;
						if( bRev )
						{
							pCv->Reverse() ;
						}
					}
					CurveOneLayer( *pTComb , AllCurve, AllComb, pCvHead, dAllComp, *pCv, m_cParam, 
									LocFrame, dDepth, dDepthInc , bFst, i, 0 );
				}  
			    Mini_DeleteCurves( pCvHead ) ;
				bFst = FALSE ;
				NewPath.AddData( dDepth, pTComb, TRUE ) ;
			}
		}

		for( int i = 1 ; i <= pLayer->m_nLayerCount ;  i ++ ) 
		{
			dDepth = pLayer->m_dLayerDepth[i] ; 
			dDepthInc = dDepth - pLayer->m_dLayerDepth[i-1] ;
			
			/*添加空间曲线路径*/
			POSITION atPos = AllComb.GetHeadPosition() ;
			while( atPos ) 
			{
				CPathCombine* pSelComb = (CPathCombine*) AllComb.GetNext( atPos ) ;
				CPathCombine* pTComb = NULL ;
				if( i == pLayer->m_nLayerCount )
					pTComb = pSelComb ;
				else 
                    pTComb = pSelComb->CopyMyself() ;
			    DefineCurve3DPath( *pTComb, m_cParam, i )  ;
				NewPath.AddData( dDepth, pTComb,TRUE ) ;
			}
		}
		AllComb.RemoveAll() ;
	}
	else 
	{ // 区域优先
        PNT3D  dStart, dEnd ;
	    BOOL   bKeepDown , bAddLine ;
		/* 添加平面曲线  */
		POSITION pos = AllCurve.GetHeadPosition() ;
		while( pos )
		{
			BOOL bRev = FALSE ;
			
			pCurve = ( CSmartCurve*) AllCurve.GetNext( pos ) ;
			BOOL bIsClosed = pCurve->IsClosed() ;
			bKeepDown = (pLayer->m_bLayerFlag & NCDEF_LAYER_KEEPDOWN)? TRUE:FALSE ;
			if( m_cFeedDef.m_cPlungeDef.m_nPlungeType == NCDEF_PLUNGE_HELIX )
			{
				bKeepDown = FALSE ;
			}
		    CPathCombine* pTComb = new CPathCombine( NC_WPROCESS_ROUGH ) ;
            double dLastRComp = dRComp ;
			for( int i = 1 ; i <= pLayer->m_nLayerCount ;  i ++ ) 
			{
				dAllComp = 0. ;
				bAddLine = FALSE ;
				dDepth = pLayer->m_dLayerDepth[i] ; 
				dDepthInc = dDepth - pLayer->m_dLayerDepth[i-1] ;
				if( bSameOff == 0 ) 
				{ 
				    dRComp = m_pTool->GetRadiusComp( dDepth, dAngle ) + GetSideStock() ;
				} 
				if( m_cSetupDef.m_cCorDef.m_nCorType == NCDEF_CORNER_EXTARC &&
					(dRComp >= 1.0e-6  || dRComp <= -1.0e-6) )//延伸圆弧
				{//先按照直线延长方式偏移曲线，然后再延伸圆弧
					m_cSetupDef.m_cCorDef.m_nCorType = NCDEF_CORNER_EXTUSER ;
					pCvHead = GenerateOffsetCurve(pCurve, dRComp);
					dAllComp = dRComp ;
					m_cSetupDef.m_cCorDef.m_nCorType = NCDEF_CORNER_EXTARC ;
					if( dRComp < 0 )
					{
						pCvHead->Reverse() ;
						bRev = TRUE ;
					}
				}
				else
				{
					pCvHead = GenerateOffsetCurve(pCurve, dRComp);
					dAllComp = dRComp ;
				}
				if( bAntiFace && pCvHead  )
				{ // 底面雕刻效果
					double dOffComp ;
					dOffComp = m_pTool->GetRadiusComp( pLayer->m_dTotalDepth - dDepth, dAngle ) 
								   - m_pTool->GetRadiusComp( 0.0,0.0 )  ;
					if( dOffComp > 1.0e-3 )
					{
						CSmartCurve* pAtCurve = pCvHead ;
						pCvHead = Mini_OffsetCurveList( pAtCurve, NCDEF_OFFSET_OUTER, dOffComp, GetCorDef() ) ;
						dAllComp += dOffComp ;
						Mini_DeleteCurves( pAtCurve ) ;
					}
				}
				if( pCvHead && pCvHead->next )
				{ // 出现两条曲线
					bKeepDown = FALSE ;
				}
                int bRevDir = 0 ;
				if( bKeepDown )
				{// 增加连续下刀路径
					if( ( i % 2 == 0 ) && !bIsClosed && 
						!(m_cParam.m_bCurveFlag & NCDEF_FCURVE_TWICECUT) )
					{ // 曲线反向
                        bRevDir = 1 ;
					}
					if( i > 1 )	bAddLine = TRUE ;
				}
			    CPathCombine  tmpComb( NC_WPROCESS_ROUGH ) ;
				for( pCv = pCvHead ; pCv ; pCv = pCv->next )
				{ 
					DOUBLE dTopTol = m_cFeedDef.m_cPlungeDef.m_dTopTol ;
					if( i != 1 && bKeepDown == TRUE )
					{
						m_cFeedDef.m_cPlungeDef.m_dTopTol = 0.0 ;
					}
					if(m_cSetupDef.m_cCorDef.m_nCorType == NCDEF_CORNER_EXTARC &&
						(dRComp >= 1.0e-6  || dRComp <= -1.0e-6) )
					{
						ResetCurve( pCv, m_cProcessDef.m_dExtRadius, 2 ) ;
						if( bRev )
						{
							pCv->Reverse() ;
						}
					}
					CurveOneLayer( tmpComb , AllCurve, AllComb, pCvHead, dAllComp, *pCv, m_cParam,
									LocFrame, dDepth, dDepthInc, bFst, i , bRevDir);
					
					m_cFeedDef.m_cPlungeDef.m_dTopTol = dTopTol;
				}  
				bFst = FALSE ;
				Mini_DeleteCurves( pCvHead ) ;
				if( tmpComb.m_pHead && pTComb->m_pHead )
				{
					if( bAddLine && 
						pTComb->GetEndPoint( 1, dStart ) &&
						tmpComb.GetEndPoint( 0, dEnd   )  )
					{
						dStart[2] += dDepth ;
                        double dDist2D  = nc_Distance( dStart, dEnd , 2 ) ;
                        double dZDist   = (dStart[2] - dEnd[2] ) ;
                        double dOffDist = fabs( dRComp - dLastRComp ) + 0.01;
                        if( dZDist> 2.0e-4 && dDist2D < 2.0e-4 ||
                            dZDist> 2.0e-4 && !bIsClosed && dDist2D< dOffDist )
                        {
							CPathLine3D *pLine = new CPathLine3D( dStart, dEnd ) ;
                            if( m_cFeedDef.m_cPlungeDef.m_nPlungeType == 0 )
                            {
                                pLine->m_bFeedType = JDNC_FEEDTYPE_CONNECT ;
                            }
                            else
                            {
                                pLine->m_bFeedType = JDNC_FEEDTYPE_PLUNGE ;
                            }
							tmpComb.InsertAfter(pLine, NULL) ;
						}
					}
				}
				tmpComb.MoveCombine( 0.0, 0.0, -dDepth ) ;
				pTComb->AppendCombine( tmpComb ) ;
                dLastRComp = dRComp ;
			}
		    NewPath.AddData( 0.0, pTComb , TRUE) ;
		}

		/*添加空间曲线*/
		POSITION  atPos = AllComb.GetHeadPosition() ;
		while( atPos )
		{ 
		    CPathCombine* pSelComb = (CPathCombine*) AllComb.GetNext(atPos) ;
			bKeepDown = pLayer->m_bLayerFlag & NCDEF_LAYER_KEEPDOWN ;
			CPathCombine* pTComb = new CPathCombine(NC_WPROCESS_ROUGH) ;
			CPathCombine* tmpComb = NULL ;
			for( int i = 1 ; i <= pLayer->m_nLayerCount ;  i ++ ) 
			{
				if( i == pLayer->m_nLayerCount ) tmpComb = pSelComb ;
				else  tmpComb = pSelComb->CopyMyself() ;
				DefineCurve3DPath( *tmpComb, m_cParam, i )  ;
				dDepth = pLayer->m_dLayerDepth[i] ; 
				dDepthInc = dDepth - pLayer->m_dLayerDepth[i-1] ;
				bAddLine = FALSE ;
				if( bKeepDown )
				{// 增加连续下刀路径
					if( ( i % 2 == 0 ) && 
						!(m_cParam.m_bCurveFlag & NCDEF_FCURVE_TWICECUT) )
					{ // 曲线反向
//						tmpComb->ReverseDirect() ;
						ReversePComb( *tmpComb ) ;
					}
					if( i > 1 )	bAddLine = TRUE ;
				}
				if( tmpComb->m_pHead && pTComb->m_pHead)
				{
					if( bAddLine && 
						pTComb->GetEndPoint( 1, dStart ) &&
						tmpComb->GetEndPoint( 0, dEnd   )  )
					{
						dStart[2] += dDepth ;
						if( nc_FabsDist( dStart, dEnd , 2 ) < 2.0e-4 && 
							(dStart[2] - dEnd[2] ) > 2.0e-4 )
						{
							CPathLine3D *pLine = new CPathLine3D( dStart, dEnd ) ;
                            if( m_cFeedDef.m_cPlungeDef.m_nPlungeType == 0 )
                            {
                                pLine->m_bFeedType = JDNC_FEEDTYPE_CONNECT ;
                            }
                            else
                            {
                                pLine->m_bFeedType = JDNC_FEEDTYPE_PLUNGE ;
                            }
							tmpComb->InsertAfter(pLine, NULL) ;
						}
					}
				}
				tmpComb->MoveCombine( 0.0, 0.0, -dDepth ) ;
				pTComb->AppendCombine( *tmpComb ) ;
				delete tmpComb  ;
			} 
			NewPath.AddData( 0.0, pTComb, TRUE ) ;
		}
		AllComb.RemoveAll() ;
	}
	MathCAM_ClearCurveList( AllCurve ) ;

	// STEP 4 : 计算加工路径
	if( ! NewPath.m_pHead )
	{
		m_nErrorType = JDERROR_GENPATH_NOTGENPATH ;
		return FALSE ; 
	}
	if( !( m_cParam.m_bCurveFlag & NCDEF_FCURVE_REDEPTH  ) )
	{// 校正表面高度
		NewPath.ZMoveGroup( GetTopHeight() ) ;
	}
	if( pLayer->m_dLayerDepth ) delete[] pLayer->m_dLayerDepth ;
	pLayer->m_dLayerDepth = NULL ;
	//if( m_cParam.m_bCurveFlag & NCDEF_FCURVE_GRNDMDFY )//磨削调整
	//{
	//	//ModifyGrind(NewPath, m_cParam.m_nMdfyType, m_cParam.m_dMdfyHeit, m_cParam.m_dMdfyDist);
	//	ModifyGrindNew(NewPath, m_cParam.m_nMdfyType, m_cParam.m_dMdfyHeit, m_cParam.m_dMdfyDist,m_cParam.m_nMdfySegs);
	//}
	return TRUE ;
}

CSmartCurve* CSmartCurveGen::GenerateOffsetCurve(CSmartCurve *pOrgCurve, double dRComp)
{
	CSmartCurve* pOffsetCurve;
	CSmartCurve *pCurve = pOrgCurve->CopyMyself();
	BOOL bIsClosed = pCurve->IsClosed();
	if (bIsClosed && (dRComp > 1.0e-4))
	{
		pCurve->AddCurve(pCurve->CopyMyself());
		pOffsetCurve = pCurve->OffsetCurveEx( dRComp, m_cSetupDef.m_cCorDef ) ;
		if (pOffsetCurve)
		{			
			BOOL bFind = FALSE;
			CSmartSect* pHead = pOffsetCurve->GetHead();
			// 第一步：删除没有重合线的路径和重合的路径部分，防止生成的闭合轮廓有分支或重复
			while (pHead)
			{
				if (pHead->GetType() == NC_SECT_LINE)
				{
					CSmartLine* pHeadLine = (CSmartLine*) pHead;
					CSmartSect* pNext = pHead->next;
					while (pNext)
					{
						if (pNext->GetType() != NC_SECT_LINE)
						{
							pNext = pNext->next;
							continue;
						}
						CSmartLine* pNextLine = (CSmartLine*) pNext;
						PNT2D ptStart[2], ptEnd[2];
						pHeadLine->GetStart(ptStart[0]);
						pHeadLine->GetEnd(ptEnd[0]);

						pNextLine->GetStart(ptStart[1]);
						pNextLine->GetEnd(ptEnd[1]);
						double dis1 = mathDist2D(ptStart[0], ptStart[1]);
						double dis2 = mathDist2D(ptEnd[0], ptEnd[1]);

						if (dis1 < 1.0e-4 && dis2 < 1.0e-4)
						{// 删除重合的直线路径
							pOffsetCurve->RemoveSect(pNextLine);
							delete pNextLine;
							pNextLine = NULL;
							bFind = TRUE;
							break;
						}	
						pNext = pNext->next;
					}
					if (bFind)
					{
						pHead = pHead->next;
						continue;
					}
					else
					{// 删除没有重合线的直线路径，防止生成的轮廓有分支
						CSmartSect* next = pHead->next;
						pOffsetCurve->RemoveSect(pHead);
						delete pHead;
						pHead = NULL;
						pHead = next;
						continue;
					}
				}
				else if(pHead->GetType() == NC_SECT_ARC) 
				{
					CSmartArc* pHeadArc = (CSmartArc*) pHead;
					CSmartSect* pNext = pHead->next;
					while (pNext)
					{
						if (pNext->GetType() != NC_SECT_ARC)
						{
							pNext = pNext->next;
							continue;
						}
						CSmartArc* pNextArc = (CSmartArc*) pNext;

						if (mathDist2D(pHeadArc->m_aCenter,pNextArc->m_aCenter) < 1.0e-4 &&
							mathDist2D(pHeadArc->m_aAngle, pNextArc->m_aAngle) < 1.0e-4 &&
							abs(pHeadArc->m_dRadius - pNextArc->m_dRadius) < 1.0e-4)
						{// 删除重合的过渡圆弧路径
							pOffsetCurve->RemoveSect(pNextArc);
							delete pNextArc;
							pNextArc = NULL;
							bFind = TRUE;
							break;
						}
						pNext = pNext->next;
					}
					if (bFind)
					{
						pHead = pHead->next;
						continue;
					}
					else
					{  // 删除没有重合线的圆弧路径，防止生成的轮廓有分支
						CSmartSect* next = pHead->next;
						pOffsetCurve->RemoveSect(pHead);
						delete pHead;
						pHead = NULL;
						pHead = next;
						continue;
					}
				}
				break;
			}

			// 第二步：删除长度为0的线
			pHead = pOffsetCurve->m_pHead;
			CSmartCurve* tmp = new CSmartCurve;
			while (pHead) 
			{
				CSmartSect* pNext = pHead->next;
				if (pHead->GetLength()>1.0e-4)
				{
					tmp->AddSect(pHead->CopyMyself());
				}				

				pHead = pNext;
				if (tmp->IsClosed())
				{					
					break;
				}				
			}

			pHead = pOffsetCurve->m_pHead;
			while (pHead)
			{
				CSmartSect* pNext = pHead->next;
				pOffsetCurve->RemoveSect(pHead);
				delete pHead;
				pHead = NULL;
				pHead = pNext;
			}
			delete pOffsetCurve;

			// 第四步：处理结果
			// 判断如果生成的路径为空，则基本确定时因为第一步中将路径全部删除。
			// 路径全被删除，说明pCurve偏移结果没有重合线，所以可以直接使用。
			// 使用pCurve结果要比使用pOrgCurve效果好
			if (tmp->m_nNumSect < 1) 
			{
				delete tmp;
				pOffsetCurve = pCurve->OffsetCurveEx( dRComp, m_cSetupDef.m_cCorDef ) ;
			}
			else
			{
				pOffsetCurve = tmp;
			}
		}						
	}
	else
	{
		pOffsetCurve = pCurve->OffsetCurveEx( dRComp, m_cSetupDef.m_cCorDef ) ;
	}		
	CSmartSect* pHead = pCurve->GetHead();
	while (pHead)
	{
		CSmartSect* pNext = pHead->next;
		pCurve->RemoveSect(pHead);
		delete pHead;
		pHead = NULL;
		pHead = pNext;
	}
	delete pCurve;
	pCurve = NULL;
	return pOffsetCurve;
}
///////////////////////
// 添加修边路径
int CSmartCurveGen::BoundCurveOneLayer( CPathCombine& PComb, /*路径集合*/
                                        CSmartCurve&  Curve, /*切割曲线*/
						                JDNC_CURVE& CurveCut,/*切割参数*/
										RFRAME &LocFrame	,/*局部坐标系*/
                                        DOUBLE     Depth    ,
			                            DOUBLE DepthInc    , /*雕刻深度*/  
						                int  Layer         )  /*层号    */
{
	int& SideTime = m_cSlayerDef.m_nLayerCount;
	if( SideTime <= 0 )
	{
        return FALSE ;
	}
	CPtrList AllCurve, AllComb ;
    BOOL bZigZag = (m_cParam.m_bCurveFlag & NCDEF_FCURVE_ZIGZAG) ? TRUE : FALSE ;
	BOOL bReverse, bOffBreak = FALSE ;
    CPathCombine  tmpTComb( NC_WPROCESS_ROUGH ) ;
	double SideInc = 0.0;
	for( int nTime = SideTime-1 ; nTime >= 0 ; nTime -- ) 
	{
		double dOffset = m_cSlayerDef.m_dLayerDepth[nTime] ;
		if (nTime > 0)
		{
			SideInc = dOffset - m_cSlayerDef.m_dLayerDepth[nTime - 1] ;
		}
		else
		{
			SideInc = max(dOffset, SideInc);
		}
		CSmartCurve* pCvHead = NULL, *pCv ;
        bReverse = ( bZigZag && ( nTime % 2 ) == 1 ) ? TRUE : FALSE ;
		pCvHead = GenerateOffsetCurve(&Curve, dOffset);
        if( pCvHead && pCvHead->next ) bOffBreak = TRUE ;
        else bOffBreak = FALSE ;
        for( pCv = pCvHead ; pCv ; pCv = pCv->next )
        {
            CPathCombine   cvTComb( NC_WPROCESS_ROUGH ) ;
			CurveOneLayer( cvTComb, AllCurve, AllComb, NULL, 0., *pCv, CurveCut, 
							LocFrame, Depth, DepthInc, FALSE, Layer, bReverse, nTime);
			
            if( bZigZag && tmpTComb.m_pTail && cvTComb.m_pHead)
            {
                TPNT3D dStart, dEnd ;
                tmpTComb.m_pTail->GetEndPoint( 1, dStart ) ;
                cvTComb.m_pHead->GetEndPoint( 0, dEnd ) ;
                if( nc_Distance( dStart, dEnd , 2 ) < max( 0.1, SideInc * 5.0) && 
                    nc_Distance( dStart, dEnd , 2 ) > 2.0e-4 && bOffBreak == FALSE )
                {
				    CPathLine3D* pLine3D  = new CPathLine3D( dStart, dEnd ) ;
				    pLine3D->m_bFeedType = JDNC_FEEDTYPE_CONNECT ;	
					
					tmpTComb.AddEntity( pLine3D ) ;
                }
            }
            tmpTComb.AppendCombine( cvTComb ) ;
        }
		Mini_DeleteCurves(pCvHead) ;
	}
    PComb.AppendCombine( tmpTComb ) ;
	return TRUE ;
}
BOOL CSmartCurveGen::CreateBoundCurve(CPathGroup& NewPath ,  /*雕刻路径*/ 
                                      CSmartGraphic& Graph)  /*图形函数*/
{
	// STEP 0 : 获取速度曲线参数
	if( m_cParam.m_bCurveFlag & NCDEF_FCURVE_REDEPTH ) 
	{
		Graph.GetAllEntity( m_cSetupDef, m_cListCur, SMARTGRAPH_TYPE_CURVE2D ) ;
	}
    // STEP 1 : 计算曲线
	CPtrList   AllCurve , tmpList ;
	BOOL bDepth = FALSE ;
	int bEntType = SMARTGRAPH_TYPE_CURVE ;
    if( m_cParam.m_bCurveFlag & NCDEF_FCURVE_REDEPTH ) 
	{
		bEntType = SMARTGRAPH_TYPE_CURVE3D ;
		bDepth = TRUE ;
	}
	Graph.GetAllEntity( m_cSetupDef, AllCurve, bEntType  ) ;
	BOOL bAllowRev = FALSE ;
	if( m_cParam.m_nRCompSide == 2 ) bAllowRev = TRUE ;

    SmartMill_SortAllCurve( AllCurve , GetSortType(), bAllowRev, bDepth ) ;
    DOUBLE     dTopHeight = -1.0e6 ;
	if( AllCurve.GetCount() == 0 )
	{ // 校正图形
		m_nErrorType = JDERROR_GENPATH_NOCURVE ;
		return FALSE ;
	}
    if( m_cParam.m_bCurveFlag & NCDEF_FCURVE_REDEPTH ) 
	{
		POSITION atPos = AllCurve.GetHeadPosition() ;
		while( atPos )
		{
			CSmartCurve* pCurve = ( CSmartCurve*) AllCurve.GetNext( atPos ) ;
            for( CSmartSect* pSect = pCurve->m_pHead ; pSect ; pSect = pSect->next )
            {
                if(  dTopHeight < pSect->m_dZValue[0]) dTopHeight = pSect->m_dZValue[0] ; 
                if(  dTopHeight < pSect->m_dZValue[1]) dTopHeight = pSect->m_dZValue[1] ; 
            }
		}
    }
	// STEP 2 : 计算分层深度
	JDNC_LAYER * pLayer   = GetLayerDef() ;
	pLayer->m_dTotalDepth = GetCutDepth()  - m_cStockDef.m_dDepthStock; 
	DefineLayerDepth( * pLayer) ; 
	DefineSideLayerInc(m_cSlayerDef.m_cSideDef);// 计算侧向分层

	// STEP 3 : 计算偏移值
	DOUBLE dRComp , dAngle; 
	dAngle = ANGLE_TO_RADIAN( m_cShapeDef.m_cTaper.m_dBoundAngle  ) ;
	BOOL bAntiFace = m_cShapeDef.m_bAntiFace ;
    if( (m_cParam.m_nRCompSide ==  2) ||
        (m_cParam.m_bCurveFlag & NCDEF_FCURVE_USERRCOMP) ||
		(pLayer->m_nLayerCount <= 1 ) ||
		(pLayer->m_bLayerFlag & NCDEF_LAYER_COPYMODE) )
	{// 关闭补偿 ||定义补偿值|| 不分层 ||  选择拷贝分层
		bAntiFace = FALSE ;
	}
	// STEP 4 : 计算切割路径
	CSmartCurve * pCurve, *pCv, *pCvHead  ; 
	if( m_cParam.m_nRCompSide == 1 )
	{ // 向右补偿
		POSITION pos = AllCurve.GetHeadPosition() ;
		while( pos )
		{
			pCurve = ( CSmartCurve*) AllCurve.GetNext( pos ) ;
			pCurve->Reverse() ;
		}
	}
	// STEP 5 : 取得局部坐标系
	RFRAME LocFrame ;
	if( !Graph.GetTPlaneFrame( LocFrame ) )
	{
		mathInitRFrame( &LocFrame ) ;
	}

	if (m_c5DCtrlDef.m_cToolAxisApp.m_nFrameFlag != NCDEF_FRAME_DEFAULT)
	{
		mathMultRFrame( &LocFrame, &m_c5DCtrlDef.m_cToolAxisApp.m_dIndexFrame, &LocFrame ) ;
	}
	// STEP 6 : 计算路径
	DOUBLE dDepth , dDepthInc;
	if( GetLayerOrder() == 1 )
	{ // 高度优先
		for( int i = 1 ; i <= pLayer->m_nLayerCount ;  i ++ ) 
		{
			dDepth = pLayer->m_dLayerDepth[i] ; 
			dDepthInc = dDepth - pLayer->m_dLayerDepth[i-1] ;
            if( m_cParam.m_bCurveFlag & NCDEF_FCURVE_USERRCOMP )
            {
                dRComp = m_cParam.m_dRCompDist + GetSideStock() ;
            }
            else
            {
 		        dRComp = m_pTool->GetRadiusComp( dDepth, dAngle ) + GetSideStock() ;
            }
			/*添加平面曲线路径*/
			POSITION pos = AllCurve.GetHeadPosition() ;
			while( pos )
			{
	 			pCurve = ( CSmartCurve*) AllCurve.GetNext( pos ) ;
	            CPathCombine* pTComb = new CPathCombine( NC_WPROCESS_ROUGH ) ;
				pCvHead = GenerateOffsetCurve(pCurve, dRComp);
				if( bAntiFace && pCvHead  )
				{ // 底面雕刻效果
					double dOffComp ;
					dOffComp = m_pTool->GetRadiusComp( pLayer->m_dTotalDepth - dDepth, dAngle ) 
								   - m_pTool->GetRadiusComp( 0.0,0.0 )  ;
					if( dOffComp > 1.0e-3 )
					{
						CSmartCurve* pAtCurve = pCvHead ;
						pCvHead = Mini_OffsetCurveList( pAtCurve, NCDEF_OFFSET_OUTER, dOffComp, GetCorDef() ) ;
						Mini_DeleteCurves( pAtCurve ) ;
					}
				}
				for( pCv = pCvHead ; pCv ; pCv = pCv->next )
				{
                    BoundCurveOneLayer( *pTComb, *pCv, m_cParam, LocFrame, dDepth,dDepthInc, i) ;
				}  
			    Mini_DeleteCurves( pCvHead ) ;
				NewPath.AddData( dDepth, pTComb, TRUE ) ;
			}
		}
	}
	else 
	{ // 区域优先
		POSITION pos = AllCurve.GetHeadPosition() ;
		while( pos )
		{
			pCurve = ( CSmartCurve*) AllCurve.GetNext( pos ) ;
		    CPathCombine* pTComb = new CPathCombine( NC_WPROCESS_ROUGH ) ;
			for( int i = 1 ; i <= pLayer->m_nLayerCount ;  i ++ ) 
			{				
				dDepth = pLayer->m_dLayerDepth[i] ; 
				dDepthInc = dDepth - pLayer->m_dLayerDepth[i-1] ;
                if( m_cParam.m_bCurveFlag & NCDEF_FCURVE_USERRCOMP )
                {
                    dRComp = m_cParam.m_dRCompDist + GetSideStock() ;
                }
                else
                {
			        dRComp = m_pTool->GetRadiusComp( dDepth, dAngle ) + GetSideStock() ;
                }
			    pCvHead =  GenerateOffsetCurve(pCurve, dRComp) ;
				if( bAntiFace && pCvHead  )
				{ // 底面雕刻效果
					double dOffComp ;
					dOffComp = m_pTool->GetRadiusComp( pLayer->m_dTotalDepth - dDepth, dAngle ) 
													 - m_pTool->GetRadiusComp( 0.0,0.0 )  ;
					if( dOffComp > 1.0e-3 )
					{
						CSmartCurve* pAtCurve = pCvHead ;
						pCvHead = Mini_OffsetCurveList( pAtCurve, NCDEF_OFFSET_OUTER, dOffComp, GetCorDef() ) ;
														Mini_DeleteCurves( pAtCurve ) ;
					}
				}
			    CPathCombine  tmpComb( NC_WPROCESS_ROUGH ) ;
				for( pCv = pCvHead ; pCv ; pCv = pCv->next )
				{ 
					DOUBLE dTopTol = m_cFeedDef.m_cPlungeDef.m_dTopTol ;
					if( i != 1 )
					{
						m_cFeedDef.m_cPlungeDef.m_dTopTol = 0.0 ;
					}
                    BoundCurveOneLayer(tmpComb, *pCv, m_cParam, LocFrame, dDepth, dDepthInc, i) ;
										m_cFeedDef.m_cPlungeDef.m_dTopTol = dTopTol;
				}  
				Mini_DeleteCurves( pCvHead ) ;
				tmpComb.MoveCombine( 0.0, 0.0, -dDepth ) ;
				pTComb->AppendCombine( tmpComb ) ;
			}
		    NewPath.AddData( 0.0, pTComb , TRUE) ;
		}
	}
	MathCAM_ClearCurveList( AllCurve ) ;

	// STEP 4 : 计算加工路径
	if( ! NewPath.m_pHead )
	{
		m_nErrorType = JDERROR_GENPATH_NOAREA ;
		return FALSE ; 
	}

	if( !( m_cParam.m_bCurveFlag & NCDEF_FCURVE_REDEPTH ) )
	{// 校正表面高度
		NewPath.ZMoveGroup( GetTopHeight() ) ;
	}
	if( pLayer->m_dLayerDepth ) delete[] pLayer->m_dLayerDepth ;
	pLayer->m_dLayerDepth = NULL ;
	if(m_cSlayerDef.m_dLayerDepth) delete[] m_cSlayerDef.m_dLayerDepth;
	m_cSlayerDef.m_dLayerDepth = NULL;
	return TRUE ;
}
//重订报定义路径高度
void CSmartCurveGen::RedepthPathCombine(CPathCombine* pTComb, CSmartGraphic& Graph, DOUBLE dTop)
{
	if( !pTComb ) return ;

	CPtrList    Curve3D ;
	Graph.GetAllEntity( m_cSetupDef, 
						Curve3D, 
						SMARTGRAPH_TYPE_CURVE3D ) ;
	if( Curve3D.GetCount() == 0 ) 
	{
		return ;
	}
	for( CPathEntity* pEnt = pTComb->m_pHead ; pEnt ; pEnt = pEnt->next )
	{
		RedepthPathEntity( Curve3D, pEnt ) ;
	}
	POSITION pos = Curve3D.GetHeadPosition() ;
	while( pos )
	{
		CSmartCurve* pCurve = (CSmartCurve*) Curve3D.GetNext(pos) ;
		for( CSmartSect* pSect = pCurve->GetHead(); pSect ; pSect = pSect->next )
		{
			if( pSect->m_dZValue[0]  > dTop ) dTop = pSect->m_dZValue[0] ;
			if( pSect->m_dZValue[1]  > dTop ) dTop = pSect->m_dZValue[1] ;
		}
		if( pCurve ) delete pCurve ;
	}
}

#define NCDEF_OFFSET_BTM  0.004 

void MathCAM_SetContoursHeight( CSmartLoop* pNewLp, DOUBLE Height ) ;
CSmartLoop* MathCAM_BuildTrimContours( CSmartLoop* AllCont, DOUBLE ZAt, DOUBLE Angle, JDNC_COR& CorDef)
{
    CSmartLoop* pTrimCont = NULL ; 
    int nGroupID = 1 ;
    for( CSmartLoop* pCont = AllCont ; pCont ; pCont = pCont->next )
    {
        double dOffDist = ( pCont->m_dHeight - ZAt ) * tan( Angle ) + NCDEF_OFFSET_BTM * 2.0 ;
        CSmartLoop* pLpHead = pCont->OffsetContour( NCDEF_OFFSET_OUTER, dOffDist, dOffDist, CorDef ) ;
        for( CSmartLoop * pLp = pLpHead ; pLp ; pLp = pLp->next )
        {
            pLp->m_nGroupId = nGroupID ;
            for( CSmartLoop * pIsl = pLp->m_pIsland ; pIsl ; pIsl = pIsl->next )
            {
                pIsl->m_nGroupId = nGroupID ;
            }
        }
        nGroupID ++ ;
        pTrimCont = Mini_AddContours( pTrimCont , pLpHead ) ;
    }
    CSmartLoop tmpLoop ;
    pTrimCont = tmpLoop.WeldContours( pTrimCont ) ;
    return pTrimCont ;
}
void MathCAM_TrimCurveByContours( CSmartLoop* TrimCont, CSmartCurveLib& CurveLib ) 
{
    CSmartCurve *pCvHead = CurveLib.m_pHead ;
    CurveLib.m_pHead = CurveLib.m_pTail = NULL ; 
    CurveLib.m_nNumCurve = 0 ;
    while( pCvHead )
    {
        CSmartCurve* pCurve = pCvHead  ;
        pCvHead = pCvHead->next ;
        pCurve->InitIntBuffer() ;
        for( CSmartLoop* pLp = TrimCont ; pLp ; pLp = pLp->next )
        {
            pLp->InitBuffer() ;
            pCurve->CurveCurveInt( pLp->m_pCurve ) ;
            for( CSmartLoop* pIsl = pLp->m_pIsland ; pIsl ; pIsl = pIsl->next )
            {
                pCurve->CurveCurveInt( pIsl->m_pCurve ) ;
            }
        }
        CSmartCurve* pNewList = pCurve->BreakCurve() ;
        double dZAt = pCurve->m_dDepth ;
        delete pCurve ;
		while( pNewList )
		{
			pCurve = pNewList, pNewList = pNewList->next ;
			pCurve->next = NULL ; 
            pCurve->m_dDepth = dZAt ;
		    PNT2D dPoint[3] ;
			pCurve->GetPoint( 0.0, dPoint[0] ) ;
			pCurve->GetPoint( 0.5, dPoint[1] ) ;
			pCurve->GetPoint( 1.0, dPoint[2] ) ;
            for( pLp = TrimCont ; pLp ; pLp = pLp->next )
            {
                if( pLp->IsOnContourEx( dPoint[0] ) == 1 ||
                    pLp->IsOnContourEx( dPoint[1] ) == 1 || 
                    pLp->IsOnContourEx( dPoint[2] ) == 1  )
                {
                    break ;
                }
            }
            if( pLp != NULL ) 
            {
			    delete pCurve ;
                continue ;
            }
            CurveLib.AddCurves( pCurve ) ;
		}
	}
}

int CSmartCurveGen::GetCurveRCompMask()
{
    if( m_cParam.m_nRCompSide == 2 ) 
    {
        return 0 ;
    }
    if( m_cSetupDef.m_cModelTol.m_nMdlFlag & NCDEF_SETUP_WEARCOMP_INC )
    {
        if( GetMillDir() == 0 ) return JDNC_FGCODE_RCOMPG41 ;
        return JDNC_FGCODE_RCOMPG42 ;
    }
    else if( m_cSetupDef.m_cModelTol.m_nMdlFlag & NCDEF_SETUP_WEARCOMP_DEC )
    {
        if( GetMillDir() == 0 ) return JDNC_FGCODE_RCOMPG42 ;
        return JDNC_FGCODE_RCOMPG41 ;
    }
    return 0 ;
}

///////////////////////
// 提取曲线路径
int CSmartCurveGen::CurveOneLayer( CPathCombine& PComb  ,  /*路径集合*/
								   CPtrList &AllCurve	,  /*所有曲线*/
								   CPtrList &AllComb		,  /*所有路径*/	
								   CSmartCurve *pCvHead	,  /*偏移曲线*/
								   double		dRComp	,  /*偏移距离*/
						           CSmartCurve&   Curve ,  /*切割曲线*/
								   JDNC_CURVE&  CurveCut,  /*切割参数*/
								   RFRAME	&LocFrame	,  /*局部坐标系*/
                                   DOUBLE Depth         ,
								   DOUBLE DepthInc      ,  /*雕刻深度*/
								   BOOL	bFst				,  /*第一根曲线*/
								   int    Layer         ,  /*层号*/
                                   BOOL   RevDir			,
								   int	  SideLayer)
{
	// 注释掉原因:目前没有办法处理磨损补偿G41和G42的切换问题(G41和G42切换时，中间必须要通过G40)
	//if ((m_cFeedDef.m_cPlungeDef.m_nPlungeType == NCDEF_PLUNGE_CONTOUR ||
	//	m_cFeedDef.m_cPlungeDef.m_nPlungeType == NCDEF_PLUNGE_RAMP))
	//{
	//	// 沿轮廓下刀或折线下刀时，先生成进刀路径，然后再生成下刀路径， 2013.12.9 liuxin
	//	return CurveOneLayerByContourPlunge(PComb, AllCurve, AllComb, pCvHead, 
	//									dRComp, Curve, CurveCut, LocFrame, Depth, DepthInc,
	//									bFst, Layer, RevDir, SideLayer);
	//}
	// STEP 0 : 判断曲线并反向
	if( Curve.m_pHead == NULL  )
	{
		return 0 ;
	}
	Curve.SetFeedType( JDNC_FEEDTYPE_ROUGH ) ;
	if( GetMillDir() == 0 )
	{ // 逆铣走刀
		Curve.Reverse() ;
	}
	// STEP 1 : 保留曲线高度
	CPathCombine tmpBase( NC_WPROCESS_ROUGH ) ;
	tmpBase.AppendCurve2D( Curve , 0.0, TRUE ) ;
	// 保留原有深度和速度
	BOOL bKeep[2] = { FALSE, FALSE } ;
	if(  m_cParam.m_bCurveFlag & NCDEF_FCURVE_REDEPTH  )
	{ 
		bKeep[0] = TRUE ;
	}
	
	if( bKeep[0] )
	{
		RedepthCPathCombine( m_cListCur, &tmpBase, bKeep, LocFrame);
	}
	// STEP 2 : 延伸曲线和路径
	if( (CurveCut.m_bCurveFlag & NCDEF_FCURVE_EXTDIST) && 
		CurveCut.m_dExtDist > 1.0e-2  )
	{
		ExtendCurveAndPComb( tmpBase, Curve, CurveCut ) ;
	}
	Curve.SetAllZValue( 0.0 ) ;
	// STEP 3 : 增加切入切出路径
	JDNC_LEAD  leadIn  = m_cFeedDef.m_cLeadDef ;
	JDNC_LEAD  leadOut = m_cFeedDef.m_cLeadDef ;

	// 由于路径反向会使进刀路径性质变为退刀，退刀路径变为进刀，为了
	// 使生成的进退刀路径满足路径参数设置，此时需判断是否需要将路径
	// 反向，如果需要反向，则需将退刀设置复制到leadIn中。 qqs 2014.03.10
	if (RevDir == TRUE)
	{
		UpdateLeadOutParam(m_cParam.m_cLeadOut, leadIn);
	}
	else
	{
		UpdateLeadOutParam(m_cParam.m_cLeadOut, leadOut);
	}

	int nRCompMask = GetCurveRCompMask() ;
	JDNC_LEAD  rcompLeadIn = m_cFeedDef.m_cLeadDef ;
	rcompLeadIn.m_dLength = max( 0.05, m_cFeedDef.m_cLeadDef.m_dWearLine) ;
	if( rcompLeadIn.m_nLeadType == 0 ) rcompLeadIn.m_nLeadType = NCDEF_LEAD_SLINE ;
	else   rcompLeadIn.m_nLeadType = NCDEF_LEAD_LINE ;
	JDNC_LEAD  rcompLeadOut = rcompLeadIn;
	BOOL bOnlyEndLead = m_cParam.m_bCurveFlag & NCDEF_FCURVE_ENDLEAD ? TRUE : FALSE;
	if (bOnlyEndLead)
	{// 仅首末段增加进退刀
		if (SideLayer != 0 && SideLayer != m_cSlayerDef.m_nLayerCount - 1)
		{
			leadIn.m_nLeadType = leadOut.m_nLeadType = NCDEF_LEAD_CLOSE;
		}
		else if (SideLayer == m_cSlayerDef.m_nLayerCount - 1)
		{			
			if (RevDir == FALSE)
			{
				leadOut.m_nLeadType = NCDEF_LEAD_CLOSE;
			}
			else
			{   // 如果开启仅起末点进退刀，且路径需要反向时，在路径
				// 起点处,反向后退刀路径将会变为进刀路径，所以不能关
				// 闭退刀而需要关闭进刀。  qqs 2014.03.10
				leadIn.m_nLeadType = NCDEF_LEAD_CLOSE;
			}
		}
		else if (SideLayer == 0)
		{			
			if (RevDir == FALSE)
			{
				leadIn.m_nLeadType = NCDEF_LEAD_CLOSE;
			}
			else
			{   // 如果开启仅起末点进退刀，且路径需要反向时，在路径
				// 末点处,反向后进刀路径将会变为退刀路径，所以不能关
				// 闭进刀而需要关闭退刀。  qqs 2014.03.10
				leadOut.m_nLeadType = NCDEF_LEAD_CLOSE;
			}			
		}
	}	

	DOUBLE ZShift[2] = {} ;
	GetLeadCutShift(m_cParam.m_cLeadOut, ZShift);
	CSmartCurve LeadInCv, LeadOutCv ;
	AddLeadSectAtEndExNew( leadIn, leadOut, Curve, LeadInCv, LeadOutCv, NULL , Depth, ZShift, FALSE, DepthInc) ;
	// 添加曲线段
	AddLeadInOutForCurveAndPComb( tmpBase, Curve, &LeadInCv, &LeadOutCv, bKeep, LocFrame ) ;
	// STEP 4 : 路径反向	
	DOUBLE rcompZShift[2] = { 0.0, 0.0 } ;
	if( nRCompMask != 0)
	{
		CSmartCurve LeadCompIn , LeadCompOut ;
		int bFlag = AddLeadSectAtEndExNew( rcompLeadIn, rcompLeadOut, Curve, 
											LeadCompIn, LeadCompOut, 
											NULL, 0.0, rcompZShift, TRUE ) ;
		// 添加曲线段
		AddLeadInOutForCurveAndPComb( tmpBase, Curve, &LeadCompIn, &LeadCompOut ) ;
		if (!bFlag)
		{
			m_bRComb = 2;
		}            
	}
	if( RevDir )
	{
		int bNewMask = 0 ;
		if( nRCompMask & JDNC_FGCODE_RCOMPG41 ) bNewMask = JDNC_FGCODE_RCOMPG42 ;
		else if( nRCompMask & JDNC_FGCODE_RCOMPG42 ) bNewMask = JDNC_FGCODE_RCOMPG41 ;
		nRCompMask = bNewMask ;
		Curve.Reverse() ;
//		tmpBase.ReverseDirect() ;
		ReversePComb( tmpBase ) ;
	}

	AddRCompMask( tmpBase, nRCompMask ) ;
	if( ( CurveCut.m_bCurveFlag & NCDEF_FCURVE_TWICECUT ) || 
		( CurveCut.m_bCurveFlag & NCDEF_FCURVE_TWICEEND )&&
		( Layer == m_cFeedDef.m_cLayerDef.m_nLayerCount  ) )
	{ /*所有层重刻 ||  最后一层重刻*/
		
		int bMask = 0 ;
		if( nRCompMask & JDNC_FGCODE_RCOMPG41 ) bMask = JDNC_FGCODE_RCOMPG42 ;
		else if( nRCompMask & JDNC_FGCODE_RCOMPG42 ) bMask = JDNC_FGCODE_RCOMPG41 ;
		
		// 直接把tmpBase拷贝并反向作为反向重刻路径
		CPathCombine * pTmpRep = tmpBase.CopyMyself() ;
		if( pTmpRep )
		{
//			pTmpRep->ReverseDirect() ;
			ReversePComb( *pTmpRep ) ;
			// 重置补偿码，2014.1.3，liuxin
			for( CPathEntity* pTEnt = pTmpRep->m_pHead ; pTEnt; pTEnt = pTEnt->next )
			{
				pTEnt->m_bMoveFlag &= ~nRCompMask;
			}
			AddRCompMask(*pTmpRep, bMask);
			tmpBase.AppendCombine( *pTmpRep ) ;
			delete pTmpRep ;
		} 
		CSmartCurve * pTmp = Curve.CopyMyself() ;
		if (pTmp)
		{
			pTmp->Reverse() ;
			Curve.AddCurve( pTmp ) ;
		}
	}
	// 添加连接进退刀路径
	BOOL bCnt[2] = { FALSE, FALSE } , bRet= FALSE ;
	if( bFst )
	{
		bRet = ConnectLeadInOutPosForCurve(	 tmpBase, AllCurve, AllComb, pCvHead, dRComp, 
											 m_cParam.m_nRCompSide, m_cParam.m_cLeadOut, bCnt ) ;
	}
	// STEP 5 : 增加下刀路径
	JDNC_PLUNGE Plunge = m_cFeedDef.m_cPlungeDef ;

    //在开启侧向分层和往复走刀的情况下，只有侧向分层第一层需要下刀路径 qqs、liuyan、xiejunxian 2013.03.13
	BOOL bAddPlunge = TRUE;
	if ( m_cSlayerDef.m_nLayerType != NCDEF_SLAYER_CLOSE)
	{
		if (m_cParam.m_bCurveFlag & NCDEF_FCURVE_ZIGZAG)
			bAddPlunge = SideLayer == m_cSlayerDef.m_nLayerCount - 1;
	}
	PNT2D dNor ;
	//同时开启沿轮廓下刀和沿轮廓进刀，关闭沿轮廓下刀  xh
	if ( m_cFeedDef.m_cPlungeDef.m_nPlungeType == NCDEF_PLUNGE_CONTOUR 
		&& m_cFeedDef.m_cLeadDef.m_nLeadType == NCDEF_LEAD_BYCONTOUR )
	{
		Plunge.m_nPlungeType = NCDEF_PLUNGE_CLOSE;
	}

	if( Plunge.m_nPlungeType != NCDEF_PLUNGE_CLOSE && bAddPlunge) 
	{
		Plunge.m_dIncStep =  DepthInc + Plunge.m_dTopTol ; 
		PNT3D  dPoint, dAtPoint ;;
		if( bRet && bCnt[0] )
			tmpBase.GetEndPoint( 0, dPoint ) ;
		else
			Curve.GetPoint(  0.0, dPoint ) ;
		dPoint[2]  = 0.0  ;
		CPathCombine cTPlunge( NC_WPROCESS_PLUNGE ) ;
		if( Plunge.m_nPlungeType == NCDEF_PLUNGE_VERT ||
			Plunge.m_nPlungeType == NCDEF_PLUNGE_HELIX  )
		{// 在固定位置下刀
			memcpy( dAtPoint , dPoint , sizeof( PNT3D ) ) ;
			Curve.m_pHead->GetNormal( 0.0, dNor ) ;
			if( GetMillDir() == 0 )
			{
				dNor[0] = -dNor[0] , dNor[1] = -dNor[1] ; 
			}
			DOUBLE dOffset = Plunge.m_dSideTol ; 
			if( Plunge.m_nPlungeType != NCDEF_PLUNGE_VERT ) 
				dOffset = + Plunge.m_dRadius ;
			dAtPoint[0] = dPoint[0] + dNor[0] * dOffset ;
			dAtPoint[1] = dPoint[1] + dNor[1] * dOffset ;
			AddPlungePathAt( cTPlunge, Plunge, dPoint, dAtPoint ) ;
		}
		else if( Plunge.m_nPlungeType == NCDEF_PLUNGE_RAMP )
		{ /*沿轮廓折线下刀*/
			AddPlungePathRamp( cTPlunge, Plunge , dPoint, Curve, TRUE ) ;
			if( bKeep[0] || bKeep[1] )
			{
				RedepthCPathCombine( m_cListCur, &cTPlunge, bKeep, LocFrame);
			}
		}
		else if( Plunge.m_nPlungeType == NCDEF_PLUNGE_CONTOUR ) 
		{ /*沿轮廓折线*/
			AddPlungePathBy( cTPlunge, Plunge , dPoint, Curve, TRUE ) ;
			if( bKeep[0] || bKeep[1] )
			{
				RedepthCPathCombine( m_cListCur, &cTPlunge, bKeep, LocFrame);
			}
		}
		// 平移下刀路径，防止路径断开, 2013.12.17 liuxin
		{
			tmpBase.GetEndPoint( 0, dPoint ) ;
			cTPlunge.GetEndPoint(1, dAtPoint);
			cTPlunge.MoveCombine(0.0, 0.0, dPoint[2] - dAtPoint[2]);
		}
		PComb.AppendCombine( cTPlunge ) ;
		
	}
	PComb.AppendCombine( tmpBase ) ;
	return 1 ; 
}

int CSmartCurveGen::CurveOneLayerByContourPlunge( CPathCombine& PComb  ,  /*路径集合*/
												CPtrList &AllCurve	,  /*所有曲线*/
												CPtrList &AllComb		,  /*所有路径*/	
												CSmartCurve *pCvHead	,  /*偏移曲线*/
												double		dRComp	,  /*偏移距离*/
												CSmartCurve&   Curve ,  /*切割曲线*/
												JDNC_CURVE&  CurveCut,  /*切割参数*/
												RFRAME	&LocFrame	,  /*局部坐标系*/
												DOUBLE Depth         ,
												DOUBLE DepthInc      ,  /*雕刻深度*/
												BOOL	bFst				,  /*第一根曲线*/
												int    Layer         ,  /*层号*/
												BOOL   RevDir			,
												int	  SideLayer)
{
	// STEP 0 : 判断曲线并反向
	if( Curve.m_pHead == NULL  )
	{
		return 0 ;
	}
	Curve.SetFeedType( JDNC_FEEDTYPE_ROUGH ) ;
	if( GetMillDir() == 0 )
	{ // 逆铣走刀
		Curve.Reverse() ;
	}
	// STEP 1 : 保留曲线高度
	CPathCombine tmpBase( NC_WPROCESS_ROUGH ) ;
	tmpBase.AppendCurve2D( Curve , 0.0, TRUE ) ;
	// 保留原有深度和速度
	BOOL bKeep[2] = { FALSE, FALSE } ;
	if(  m_cParam.m_bCurveFlag & NCDEF_FCURVE_REDEPTH  )
	{ 
		bKeep[0] = TRUE ;
	}
	
	if( bKeep[0] )
	{
		RedepthCPathCombine( m_cListCur, &tmpBase, bKeep, LocFrame);
	}
	// STEP 2 : 延伸曲线和路径
	if( (CurveCut.m_bCurveFlag & NCDEF_FCURVE_EXTDIST) && 
		CurveCut.m_dExtDist > 1.0e-2  )
	{
		ExtendCurveAndPComb( tmpBase, Curve, CurveCut ) ;
	}
	Curve.SetAllZValue( 0.0 ) ;

	CSmartCurve cPlungeCurve;
	BOOL bReversePlungeCurve = FALSE;
	// STEP 3 : 增加下刀路径
	JDNC_PLUNGE Plunge = m_cFeedDef.m_cPlungeDef ;

	//在开启侧向分层和往复走刀的情况下，只有侧向分层第一层需要下刀路径 qqs、liuyan、xiejunxian 2013.03.13
	BOOL bAddPlunge = TRUE;
	if ( m_cSlayerDef.m_nLayerType != NCDEF_SLAYER_CLOSE)
	{
		if (m_cParam.m_bCurveFlag & NCDEF_FCURVE_ZIGZAG)
			bAddPlunge = SideLayer == m_cSlayerDef.m_nLayerCount - 1;
	}
	if(bAddPlunge) 
	{
		Plunge.m_dIncStep =  DepthInc + Plunge.m_dTopTol ; 
		PNT3D  dPoint ;
		Curve.GetPoint(  0.0, dPoint ) ;
		dPoint[2]  = 0.0  ;
		CPathCombine cTPlunge( NC_WPROCESS_PLUNGE ) ;
		if( Plunge.m_nPlungeType == NCDEF_PLUNGE_RAMP )
		{ /*沿轮廓折线下刀*/
			AddPlungePathRamp( cTPlunge, Plunge , dPoint, Curve, TRUE ) ;
		}
		else if( Plunge.m_nPlungeType == NCDEF_PLUNGE_CONTOUR ) 
		{ /*沿轮廓折线*/
			AddPlungePathBy( cTPlunge, Plunge , dPoint, Curve, TRUE ) ;
		}
		CPtrList cPlungeCurList;
		TransfPCombineToSmartCurve(cTPlunge, cPlungeCurList, m_cSetupDef.m_cTolDef);
		if (!cPlungeCurList.IsEmpty())
		{
			CSmartCurve* pPlungeCur = NULL;
			if (!Curve.IsClosed())
			{
				bReversePlungeCurve = TRUE;
			}	
			while (!cPlungeCurList.IsEmpty())
			{
				pPlungeCur = (CSmartCurve*)cPlungeCurList.RemoveHead();
				cPlungeCurve.AddCurve(pPlungeCur);
			}
			if (bReversePlungeCurve)
			{
				cPlungeCurve.Reverse();
			}
		}
		if( bKeep[0] || bKeep[1] )
		{
			RedepthCPathCombine( m_cListCur, &cTPlunge, bKeep, LocFrame);
		}
		cTPlunge.AppendCombine(tmpBase);
		tmpBase.AppendCombine(cTPlunge);
	}
	// STEP 4 : 增加切入切出路径
	JDNC_LEAD  leadIn  = m_cFeedDef.m_cLeadDef ;
	JDNC_LEAD  leadOut = m_cFeedDef.m_cLeadDef ;
	UpdateLeadOutParam(m_cParam.m_cLeadOut, leadOut);
	int nRCompMask = GetCurveRCompMask() ;
	JDNC_LEAD  rcompLeadIn = m_cFeedDef.m_cLeadDef ;
	rcompLeadIn.m_dLength = max( 0.05, m_cFeedDef.m_cLeadDef.m_dWearLine) ;
	if( rcompLeadIn.m_nLeadType == 0 ) rcompLeadIn.m_nLeadType = NCDEF_LEAD_SLINE ;
	else   rcompLeadIn.m_nLeadType = NCDEF_LEAD_LINE ;
	JDNC_LEAD  rcompLeadOut = rcompLeadIn;
	BOOL bOnlyEndLead = m_cParam.m_bCurveFlag & NCDEF_FCURVE_ENDLEAD ? TRUE : FALSE;
	if (bOnlyEndLead)
	{// 仅首末段增加进退刀
		if (SideLayer != 0 && SideLayer != m_cSlayerDef.m_nLayerCount - 1)
		{
			leadIn.m_nLeadType = leadOut.m_nLeadType = NCDEF_LEAD_CLOSE;
		}
		else if (SideLayer == m_cSlayerDef.m_nLayerCount - 1)
		{
			leadOut.m_nLeadType = NCDEF_LEAD_CLOSE;
		}
		else if (SideLayer == 0)
		{
			leadIn.m_nLeadType = NCDEF_LEAD_CLOSE;
		}
	}
	// 生成沿轮廓下刀成功，关闭沿轮廓进刀
	if (leadIn.m_nLeadType == NCDEF_LEAD_BYCONTOUR && cPlungeCurve.m_pHead)
	{
		leadIn.m_nLeadType = NCDEF_LEAD_CLOSE;
	}

	DOUBLE ZShift[2] = {} ;
	GetLeadCutShift(m_cParam.m_cLeadOut, ZShift);
	CSmartCurve LeadInCv, LeadOutCv ;

	BOOL bFlag1 = TRUE, bFlag2 = TRUE;
	bFlag1 = GenLeadSectAtEndEx( leadIn, bReversePlungeCurve, cPlungeCurve, NULL, Depth, ZShift[0], LeadInCv, DepthInc ) ;
	bFlag2 = GenLeadSectAtEndEx( leadOut,TRUE , Curve, NULL, Depth, ZShift[1], LeadOutCv, DepthInc ) ;
	CSmartSect* pSect ;
	if (bReversePlungeCurve)
	{
		LeadInCv.Reverse();
		cPlungeCurve.Reverse();
	}
	for( pSect = LeadInCv.GetHead() ; pSect ; pSect = pSect->next )
	{
		pSect->m_dZValue[0] += cPlungeCurve.m_pHead->m_dZValue[0]  ;
		pSect->m_dZValue[1] += cPlungeCurve.m_pHead->m_dZValue[0]  ;
	}
	for( pSect = LeadOutCv.GetHead() ; pSect ; pSect = pSect->next )
	{
		pSect->m_dZValue[0] += Curve.m_pTail->m_dZValue[1]  ;
		pSect->m_dZValue[1] += Curve.m_pTail->m_dZValue[1]  ;
	}
	Curve.InsertAfter(cPlungeCurve, NULL);
	//cPlungeCurve.ClearAll();
	// 添加曲线段
	AddLeadInOutForCurveAndPComb( tmpBase, Curve, &LeadInCv, &LeadOutCv, bKeep, LocFrame ) ;
	// STEP 5 : 路径反向	
	DOUBLE rcompZShift[2] = { 0.0, 0.0 } ;
	if( nRCompMask != 0)
	{
		CSmartCurve LeadCompIn , LeadCompOut ;
		int bFlag = AddLeadSectAtEndExNew( rcompLeadIn, rcompLeadOut, Curve, 
			LeadCompIn, LeadCompOut, NULL, 0.0, rcompZShift, TRUE) ;
			
		// 添加曲线段
		AddLeadInOutForCurveAndPComb( tmpBase, Curve, &LeadCompIn, &LeadCompOut ) ;
		if (!bFlag)
		{
			m_bRComb = 2;
		}            
	}
	if( RevDir )
	{
		int bNewMask = 0 ;
		if( nRCompMask & JDNC_FGCODE_RCOMPG41 ) bNewMask = JDNC_FGCODE_RCOMPG42 ;
		else if( nRCompMask & JDNC_FGCODE_RCOMPG42 ) bNewMask = JDNC_FGCODE_RCOMPG41 ;
		nRCompMask = bNewMask ;
		Curve.Reverse() ;
//		tmpBase.ReverseDirect() ;
		ReversePComb( tmpBase ) ;
	}

	AddRCompMask( tmpBase, nRCompMask ) ;
	if( ( CurveCut.m_bCurveFlag & NCDEF_FCURVE_TWICECUT ) || 
		( CurveCut.m_bCurveFlag & NCDEF_FCURVE_TWICEEND )&&
		( Layer == m_cFeedDef.m_cLayerDef.m_nLayerCount  ) )
	{ /*所有层重刻 ||  最后一层重刻*/
		
		int bMask = 0 ;
		if( nRCompMask & JDNC_FGCODE_RCOMPG41 ) bMask = JDNC_FGCODE_RCOMPG42 ;
		else if( nRCompMask & JDNC_FGCODE_RCOMPG42 ) bMask = JDNC_FGCODE_RCOMPG41 ;
		
		// 直接使用tmpBase拷贝并反向作为反向重刻路径
		CPathCombine * pTmpRep = tmpBase.CopyMyself() ;
		if( pTmpRep )
		{
//			pTmpRep->ReverseDirect() ;
			ReversePComb( *pTmpRep ) ;
			// 重置补偿码，2014.1.3，liuxin
			for( CPathEntity* pTEnt = pTmpRep->m_pHead ; pTEnt; pTEnt = pTEnt->next )
			{
				pTEnt->m_bMoveFlag &= ~nRCompMask;
			}
			AddRCompMask(*pTmpRep, bMask);
			tmpBase.AppendCombine( *pTmpRep ) ;
			delete pTmpRep ;
		} 
		CSmartCurve * pTmp = Curve.CopyMyself() ;
		if (pTmp)
		{
			pTmp->Reverse() ;
			Curve.AddCurve( pTmp ) ;
		}
	}
	// 添加连接进退刀路径
	BOOL bCnt[2] = { FALSE, FALSE } , bRet= FALSE ;
	if( bFst )
	{
		bRet = ConnectLeadInOutPosForCurve(	 tmpBase, AllCurve, AllComb, pCvHead, dRComp, 
			m_cParam.m_nRCompSide, m_cParam.m_cLeadOut, bCnt ) ;
	}

	PComb.AppendCombine( tmpBase ) ;
	return 1 ; 
}

void CSmartCurveGen::ExtendCurveAndPComb( CPathCombine &PComb	,	/*路径*/ 
											CSmartCurve &Curve	,	/*曲线*/
											JDNC_CURVE&  CurveCut)	/*切割参数*/
{
	if( !PComb.m_pHead || !Curve.m_pHead ) return ;
	// 曲线2D延伸
	CSmartSect *pSect = NULL, *pLine = NULL ;
	PNT2D dStart, dEnd , dTan1, dTan2 ; 

	pSect = Curve.GetHead() ;
	pSect->GetPoint( 0.0, dEnd ) ;
	pSect->GetTangent( 0.0, dTan1 ) ;
	dStart[0]  = dEnd[0] - dTan1[0] * CurveCut.m_dExtDist ;
	dStart[1]  = dEnd[1] - dTan1[1] * CurveCut.m_dExtDist ;
	pLine = new CSmartLine( dStart, dEnd ) ;
	pLine->m_bFeedType = JDNC_FEEDTYPE_ROUGH ;
	Curve.InsertAfter( pLine, NULL ) ;
	pSect = Curve.GetTail() ;
	pSect->GetPoint( 1.0, dStart ) ;
	pSect->GetTangent( 1.0, dTan2 ) ;
	dEnd[0]  = dStart[0] + dTan2[0] * CurveCut.m_dExtDist ;
	dEnd[1]  = dStart[1] + dTan2[1] * CurveCut.m_dExtDist ;
	pLine = new CSmartLine( dStart, dEnd ) ;
	pLine->m_bFeedType = JDNC_FEEDTYPE_ROUGH ;
	Curve.AddSect( pLine ) ;

	// 路径2D延伸
	TPNT3D fStart, fEnd ;
	//VEC3D	fTan1, fTan2 ;
	CPathLine3D *pLine3D = NULL ;
	PComb.m_pHead->GetEndPoint( 0, fEnd ) ;
	// 直接使用曲线的方向进行延伸,防止封闭路径取的切向与曲线取的切向不一致;2014.1.7 liuxin
	//PComb.m_pHead->GetEndTangent( 0, fTan1 ) ; 
	fStart[0] = fEnd[0] - dTan1[0] * CurveCut.m_dExtDist ;
	fStart[1] = fEnd[1] - dTan1[1] * CurveCut.m_dExtDist ;
	fStart[2] = fEnd[2] ;
	pLine3D = new CPathLine3D( fStart, fEnd ) ;
	pLine3D->m_bFeedType = JDNC_FEEDTYPE_ROUGH ;
	PComb.InsertAfter( pLine3D, NULL ) ;

	PComb.m_pTail->GetEndPoint( 1, fStart ) ;
	// 直接使用曲线的方向进行延伸,防止封闭路径取的切向与曲线取的切向不一致;2014.1.7 liuxin
	//PComb.m_pTail->GetEndTangent( 1, fTan2 ) ;
	fEnd[0] = fStart[0] + dTan2[0] * CurveCut.m_dExtDist ;
	fEnd[1] = fStart[1] + dTan2[1] * CurveCut.m_dExtDist ;
	fEnd[2] = fStart[2] ;
	pLine3D = new CPathLine3D( fStart, fEnd ) ;
	pLine3D->m_bFeedType = JDNC_FEEDTYPE_ROUGH ;
	
	PComb.AddEntity( pLine3D ) ;
}
											 
void CSmartCurveGen::AddLeadInOutForCurveAndPComb( CPathCombine &PComb,
													CSmartCurve &Curve,
													CSmartCurve *LeadIn,
													CSmartCurve *LeadOut )
{
	if( !PComb.m_pHead ) return ;
	TPNT3D start, end ;
	
	if( LeadIn->m_pHead )
	{
		LeadIn->SetFeedType ( JDNC_FEEDTYPE_LEAD ) ;
		PComb.m_pHead->GetEndPoint( 0, start ) ;
		CPathEntity *pHead = MathCam_TransfSmtCurveToPathEntity( *LeadIn, start[2] ) ;
		CPathEntity *pTail = pHead ;
		pTail->m_bFeedType = JDNC_FEEDTYPE_LEAD ;
		
		while( pTail && pTail->next)
		{
			pTail = pTail->next ;
			pTail->m_bFeedType = JDNC_FEEDTYPE_LEAD ;			
		}
		PComb.m_pHead->prev = pTail ;
		pTail->next = PComb.m_pHead ;
		PComb.m_pHead = pHead ;
				
		Curve.InsertAfter( *LeadIn, NULL ) ;
	}
	if( LeadOut->m_pHead )
	{
//		CPathEntity* pEnt = NULL;
//		CPathEntity* pTail = PComb.m_pTail;
		LeadOut->SetFeedType( JDNC_FEEDTYPE_LEAD ) ;
		PComb.m_pTail->GetEndPoint( 1, end ) ;
		PComb.AddCurve( LeadOut, FALSE, end[2], JDNC_FEEDTYPE_LEAD ) ;

		Curve.AppendCurve( *LeadOut ) ;
	}  
}

void CSmartCurveGen::AddLeadInOutForCurveAndPComb( CPathCombine &PComb,
												   CSmartCurve &Curve,
												   CSmartCurve *LeadIn,
												   CSmartCurve *LeadOut,
												   BOOL			bKeep[2],
												   RFRAME		&LocFrame)
{
	if( !PComb.m_pHead ) return ;
	TPNT3D start, end ;
	
	if( LeadIn->m_pHead )
	{
		CPathCombine tmpLeadIn(JDNC_FEEDTYPE_LEAD);
		LeadIn->SetFeedType ( JDNC_FEEDTYPE_LEAD ) ;
		PComb.m_pHead->GetEndPoint( 0, start ) ;
		tmpLeadIn.AppendCurve3D(*LeadIn, start[2], JDNC_FEEDTYPE_LEAD);
		if( bKeep[0] && m_cFeedDef.m_cLeadDef.m_nLeadType == NCDEF_LEAD_BYCONTOUR)
		{
			RedepthCPathCombine( m_cListCur, &tmpLeadIn, bKeep, LocFrame);
		}
		
		tmpLeadIn.GetEndPoint(1, end);
		tmpLeadIn.MoveCombine(0.0, 0.0, start[2] - end[2]);
		tmpLeadIn.AppendCombine(PComb);
		PComb.AppendCombine(tmpLeadIn);

		Curve.InsertAfter(*LeadIn, NULL);
	}
	if( LeadOut->m_pHead )
	{
		CPathCombine tmpLeadOut(JDNC_FEEDTYPE_LEAD);
		LeadOut->SetFeedType( JDNC_FEEDTYPE_LEAD ) ;
		PComb.m_pTail->GetEndPoint( 1, end ) ;
		tmpLeadOut.AppendCurve3D(*LeadOut, end[2], JDNC_FEEDTYPE_LEAD);
		if( bKeep[0] && m_cFeedDef.m_cLeadDef.m_nLeadType == NCDEF_LEAD_BYCONTOUR )
		{
			RedepthCPathCombine( m_cListCur, &tmpLeadOut, bKeep, LocFrame);
		}
		
		tmpLeadOut.GetEndPoint(0, start);
		tmpLeadOut.MoveCombine(0.0, 0.0, end[2] - start[2]);
		PComb.AppendCombine(tmpLeadOut);
		Curve.AppendCurve( *LeadOut ) ;
	}
}

BOOL CSmartCurveGen::ConnectLeadInOutPosForCurve (	CPathCombine &PComb	,	// <I> 输入路径
													CPtrList &AllCurve	,	// <I> 原始曲线
													CPtrList &AllComb	,	// <I> 所有路径
													CSmartCurve *pCvHead	,	// <I> 偏移曲线
													double		dRComp	,	// <I> 偏移距离
													int		nRCompSide	, 	// <I> 补偿方向 )	
													JDNC_LEADOUT &cLead	, 	// <I> 退刀参数
													BOOL	bCnt[2]		)		// <O> 成功返回TRUE
{
	UNUSED_ALWAYS(AllComb);
	// 判断数据的有效性
	if( !PComb.m_pHead ) return FALSE ;
	// 如果有侧向分层，退出
	//if( m_cSlayerDef.m_nLayerType != NCDEF_SLAYER_CLOSE ) return FALSE ;
	// 轴向分层多层，退出，2013.9.27，liuxin
	if (m_cFeedDef.m_cLayerDef.m_nLayerCount > 1)
	{
		return FALSE;
	}
	// 如果非指定点下刀，转到LeadOut
	BOOL bLeadIn = FALSE, bLeadStart = TRUE ;
	CSmtCPathLib AllPath, tmpAllPath ;
	CPtrList listSmtCurve;
	CPathLine3D *pLine = NULL ;
	BOOL bSame = cLead.m_bSameLeadIn ;
	//// 将路径转换为AllPath
	//TransfPCombineToCPath( PComb, AllPath, 0., 0., m_cSetupDef.m_cTolDef ) ;

	// 将路径转成CSmartCurve
	TransfPCombineToSmartCurve(PComb, listSmtCurve, m_cSetupDef.m_cTolDef);

	// step 1 : 生成起点连接路径
	if( m_cFeedDef.m_cLeadDef.m_nPositionType != NCDEF_LEADPOS_REFER )
		goto EndLine1 ;
	// 如果进刀方式为沿轮廓\圆弧内切\直线相切，转到LeadOut
	if( m_cFeedDef.m_cLeadDef.m_nLeadType == NCDEF_LEAD_BYCONTOUR ||
		m_cFeedDef.m_cLeadDef.m_nLeadType == NCDEF_LEAD_LINE	||
		m_cFeedDef.m_cLeadDef.m_nLeadType == NCDEF_LEAD_INARC )
		goto EndLine1 ;
	// 如果非直线下刀或者关闭下刀，转到LeadOut
	if( m_cFeedDef.m_cPlungeDef.m_nPlungeType > NCDEF_PLUNGE_VERT )
		goto EndLine1 ;

	if( m_cProcessDef.m_bProcessFlag & NCDEF_LEADIN_CONNECTPOS) 
		bLeadIn = TRUE ;
	// 目前只是用LeadIn来控制二者的连接
	if( !bLeadIn ) 
		goto EndLine1 ;

	PNT3D start, pnt = { m_cFeedDef.m_cLeadDef.m_dLeadPos[0], m_cFeedDef.m_cLeadDef.m_dLeadPos[1], 0.} ;
	PComb.GetEndPoint( 0, start ) ;

	if( m_cFeedDef.m_cLeadDef.m_nLeadType == NCDEF_LEAD_SLINE )
	{ // 如果直线连接则判断距离
		double dist = mathDist2D( pnt, start ) ;
		if( dist < m_cFeedDef.m_cLeadDef.m_dLength )
		{
			bLeadStart = FALSE ;
		}
	}

	// 如果为关闭
	if( nRCompSide == 2 )
	{
		pnt[2] = start[2] ;
		if( mathDist( start, pnt ) > 1.0e-4 && bLeadStart && 
			!AllCurveIntLine( listSmtCurve, start, pnt ) &&
			!SmartCurveIntLine( pCvHead, start, pnt ) )
		{
			pLine = new CPathLine3D( pnt, start ) ;
			pLine->m_bFeedType = JDNC_FEEDTYPE_LEAD ;
			
			PComb.InsertAfter( pLine, NULL ) ;
			bCnt[0] = TRUE ;
		}
	}
	else
	{
		double dDist = MinDistPntAndAllCurve( AllCurve, pnt ) ;

		if( dDist < fabs( dRComp ) )
			goto EndLine1 ;

		if( !AllCurveIntLine( AllCurve, pnt, start ) && bLeadStart && 
			!AllCurveIntLine( listSmtCurve, pnt, start ) && 
			!SmartCurveIntLine( pCvHead, pnt, start )   )
		{
			pnt[2] = start[2] ;
			if( mathDist( start, pnt ) > 1.0e-4 )
			{
				pLine = new CPathLine3D( pnt, start ) ;
				pLine->m_bFeedType = JDNC_FEEDTYPE_LEAD ;
				
				PComb.InsertAfter( pLine, NULL ) ;
				bCnt[0] = TRUE ;
			}
		}
	}

EndLine1:
	// step 2 : 生成末点连接路径
	BOOL bLeadOut = FALSE, bLeadEnd = TRUE ;
	// 如果非指定点下刀，返回
	if( m_cProcessDef.m_nLeadOutPosType != NCDEF_LEADPOS_REFER && !bSame )
		goto EndLine2 ;

	// 如果退刀方式为沿轮廓\圆弧内切\直线相切，转到EndLine2
	if( bSame )
	{
		// 2013.9.27 liuxin
		if( m_cFeedDef.m_cLeadDef.m_nPositionType != NCDEF_LEADPOS_REFER )
			goto EndLine2 ;
		if( m_cFeedDef.m_cLeadDef.m_nLeadType == NCDEF_LEAD_BYCONTOUR ||
			m_cFeedDef.m_cLeadDef.m_nLeadType == NCDEF_LEAD_LINE	||
			m_cFeedDef.m_cLeadDef.m_nLeadType == NCDEF_LEAD_INARC )
			goto EndLine2 ;
	}
	else
	{
		if( cLead.m_nLeadType == NCDEF_LEAD_BYCONTOUR ||
			cLead.m_nLeadType == NCDEF_LEAD_LINE	||
			cLead.m_nLeadType == NCDEF_LEAD_INARC )
			goto EndLine2 ;
	}

	if( m_cProcessDef.m_bProcessFlag & NCDEF_LEADOUT_CONNECTPOS || 
		bSame && m_cProcessDef.m_bProcessFlag & NCDEF_LEADIN_CONNECTPOS ) 
		bLeadOut = TRUE ;
	// 目前只是用LeadIn来控制二者的连接
	if( !bLeadOut ) 
		goto EndLine2 ;

	PNT3D end ;
	if( !bSame ) 
	{
		pnt[0] = m_cProcessDef.m_dLeadOutPos[0] ;
		pnt[1] = m_cProcessDef.m_dLeadOutPos[1] ;
	}

	PComb.GetEndPoint( 1, end ) ;

	if( bSame && m_cFeedDef.m_cLeadDef.m_nLeadType == NCDEF_LEAD_SLINE || 
		cLead.m_nLeadType == NCDEF_LEAD_SLINE )
	{ // 如果直线连接则判断距离
		double dist = mathDist2D( pnt, end ) ;
		if( dist < cLead.m_dLength )
		{
			bLeadEnd = FALSE ;
		}
	}

	// 如果为关闭
	if( nRCompSide == 2 )
	{
		pnt[2] = end[2] ;
		if( mathDist( end  , pnt ) > 1.0e-4 && bLeadEnd &&
			!AllCurveIntLine( listSmtCurve, end, pnt ) &&
			!SmartCurveIntLine( pCvHead, end, pnt ) )
		{
			pLine = new CPathLine3D( end  , pnt ) ;
			pLine->m_bFeedType = JDNC_FEEDTYPE_LEAD ;
			
			PComb.AddEntity ( pLine ) ;
			bCnt[1] = TRUE ;
		}
	}
	else
	{
		double dDist = MinDistPntAndAllCurve( AllCurve, pnt ) ;

		if( dDist < fabs( dRComp ) )
			goto EndLine2 ;
		
		if( !AllCurveIntLine( AllCurve, pnt, end  ) && bLeadOut && 
			!AllCurveIntLine( listSmtCurve, pnt, end ) && 
			!SmartCurveIntLine( pCvHead, pnt, end )		)
		{
			pnt[2] = end[2] ;
			if( mathDist( end  , pnt ) > 1.0e-4 )
			{
				pLine = new CPathLine3D( end  , pnt ) ;
				pLine->m_bFeedType = JDNC_FEEDTYPE_LEAD ;
				
				PComb.AddEntity( pLine ) ;
				bCnt[1] = TRUE ;
			}
		}
	}/**/
EndLine2:
	//AllPath.ClearAllPath () ;
	tmpAllPath.ClearAllPath() ;
	while (!listSmtCurve.IsEmpty())
	{
		CSmartCurve* pCurve = (CSmartCurve*)listSmtCurve.RemoveHead();
		if (pCurve)
		{
			pCurve->ClearAll();
			delete pCurve;
		}
	}
	return TRUE ;
}

int CSmartCurveGen::DefineCurve3DPath( CPathCombine& SelComb,  /*切割路径*/
					                   JDNC_CURVE&  CurveCut,  /*路径    */
							           BOOL   Layer        )   /*层号    */ 
{
	// STEP 1 : 延伸曲线
	CPathEntity *pLine ;
	PNT3D dStart, dEnd, dTan ; 
	if( (CurveCut.m_bCurveFlag & NCDEF_FCURVE_EXTDIST) && 
		CurveCut.m_dExtDist > 1.0e-2  )
	{ 
	    SelComb.m_pHead->GetEndPoint( 0, dEnd ) ;
		SelComb.m_pHead->GetEndTangent( 0, dTan ) ;
		dStart[0]  = dEnd[0] - dTan[0] * CurveCut.m_dExtDist ;
		dStart[1]  = dEnd[1] - dTan[1] * CurveCut.m_dExtDist ;
		dStart[2]  = dEnd[2] - dTan[2] * CurveCut.m_dExtDist ;
		pLine = new CPathLine3D( dStart, dEnd ) ;
		pLine->m_bFeedType = JDNC_FEEDTYPE_ROUGH ;
		SelComb.InsertAfter( pLine , NULL ) ;
		SelComb.m_pTail->GetEndPoint( 1, dStart ) ;
	    SelComb.m_pTail->GetEndTangent( 1, dTan ) ;
		dEnd[0]  = dStart[0] + dTan[0] * CurveCut.m_dExtDist ;
		dEnd[1]  = dStart[1] + dTan[1] * CurveCut.m_dExtDist ;
		dEnd[2]  = dStart[2] + dTan[2] * CurveCut.m_dExtDist ;
		pLine = new CPathLine3D( dStart, dEnd ) ;
		pLine->m_bFeedType = JDNC_FEEDTYPE_ROUGH ;
		SelComb.AddEntity( pLine ) ;
	}
	// STEP 2 : 反向再刻一次
	if( ( CurveCut.m_bCurveFlag & NCDEF_FCURVE_TWICECUT ) || 
		( CurveCut.m_bCurveFlag & NCDEF_FCURVE_TWICEEND )&&
		 ( Layer == m_cFeedDef.m_cLayerDef.m_nLayerCount  ) )
	{ /*所有层重刻 ||  最后一层重刻*/
		CPathCombine * pTmp = SelComb.CopyMyself() ;
		if( pTmp )
		{
//			pTmp->ReverseDirect() ;
			ReversePComb( *pTmp ) ;
		    SelComb.AppendCombine( *pTmp ) ;
			delete pTmp ;
		} 
	}
	// STEP 4 : 路径反向
	if( GetMillDir() == 0 )
	{ // 逆铣走刀
//		SelComb.ReverseDirect() ;
		ReversePComb( SelComb ) ;
	}
	return 1 ; 
}

// 为路径添加进退刀曲线,该函数为轮廓切割、修边中调用
void CSmartContourGen::AddLeadInOutForPComb( CPathCombine &PComb,	CSmartCurve *LeadIn, CSmartCurve *LeadOut) 
{
	if( !PComb.m_pHead ) return ;
	TPNT3D start, end ;
	
	if(LeadIn && LeadIn->m_pHead )
	{
		CPathCombine tmpLeadIn(JDNC_FEEDTYPE_LEAD);
		LeadIn->SetFeedType ( JDNC_FEEDTYPE_LEAD ) ;
		PComb.m_pHead->GetEndPoint( 0, start ) ;
		tmpLeadIn.AppendCurve3D(*LeadIn, start[2], JDNC_FEEDTYPE_LEAD);
		
		tmpLeadIn.AppendCombine(PComb);
		PComb.AppendCombine(tmpLeadIn);
	}
	if(LeadOut && LeadOut->m_pHead )
	{
		CPathCombine tmpLeadOut(JDNC_FEEDTYPE_LEAD);
		LeadOut->SetFeedType( JDNC_FEEDTYPE_LEAD ) ;
		PComb.m_pTail->GetEndPoint( 1, end ) ;
		tmpLeadOut.AppendCurve3D(*LeadOut, end[2], JDNC_FEEDTYPE_LEAD);
		
		PComb.AppendCombine(tmpLeadOut);
	}  
}

// 为路径添加进退刀曲线,该函数为轮廓切割、修边中调用
void CSmartContourGen::AddLeadInOutAndOverlapForPComb( CPathCombine &PComb, 
													   CSmartCurve *Overlap, 
													   CSmartCurve *LeadIn, 
													   CSmartCurve *LeadOut,
													   double Depth, 
													   CPtrList &CurveList, 
													   BOOL bKeep[2], 
													   RFRAME &LocFrame) 
{
	if( !PComb.m_pHead ) return ;
	TPNT3D start, end ;
	if (Overlap && Overlap->m_pHead)
	{// 将重复加工段加入路径中
		CPathCombine tmpOverlap( NC_WPROCESS_ROUGH ) ;
		tmpOverlap.AppendCurve3D(*Overlap, -Depth, TRUE);
		if( bKeep[0] || bKeep[1] )
		{
			RedepthCPathCombine( CurveList, &tmpOverlap, bKeep, LocFrame);
		}
		PComb.AppendCombine(tmpOverlap);
	}
	if(LeadIn && LeadIn->m_pHead )
	{
		CPathCombine tmpLeadIn(JDNC_FEEDTYPE_LEAD);
		LeadIn->SetFeedType ( JDNC_FEEDTYPE_LEAD ) ;
		PComb.m_pHead->GetEndPoint( 0, start ) ;
		tmpLeadIn.AppendCurve3D(*LeadIn, start[2], JDNC_FEEDTYPE_LEAD);
		if( bKeep[0] && m_cFeedDef.m_cLeadDef.m_nLeadType == NCDEF_LEAD_BYCONTOUR )
		{
			RedepthCPathCombine( CurveList, &tmpLeadIn, bKeep, LocFrame);
		}
		
		tmpLeadIn.GetEndPoint(1, end);
		tmpLeadIn.MoveCombine(0.0, 0.0, start[2] - end[2]);
		tmpLeadIn.AppendCombine(PComb);
		PComb.AppendCombine(tmpLeadIn);
	}
	if(LeadOut && LeadOut->m_pHead )
	{
		CPathCombine tmpLeadOut(JDNC_FEEDTYPE_LEAD);
		LeadOut->SetFeedType( JDNC_FEEDTYPE_LEAD ) ;
		PComb.m_pTail->GetEndPoint( 1, end ) ;
		tmpLeadOut.AppendCurve3D(*LeadOut, end[2], JDNC_FEEDTYPE_LEAD);
		if( bKeep[0] && m_cFeedDef.m_cLeadDef.m_nLeadType == NCDEF_LEAD_BYCONTOUR )
		{
			RedepthCPathCombine( CurveList, &tmpLeadOut, bKeep, LocFrame);
		}
		
		tmpLeadOut.GetEndPoint(0, start);
		tmpLeadOut.MoveCombine(0.0, 0.0, end[2] - start[2]);
		PComb.AppendCombine(tmpLeadOut);
	}  
}

CSmartContourGen::CSmartContourGen()
{
}
CSmartContourGen::~CSmartContourGen()
{
	MathCam_DeleteAllRedepthCur( m_cListCur ) ;
}

///////////////////////
// 提取轮廓路径
int CSmartContourGen::LoopOneLayer( CPathCombine& PComb   ,  /*路径集合*/
									CSmartLoop *AllCont  ,  /*所有区域*/
									CSmartLoop&    Contour,  /*切割轮廓*/
									DOUBLE Depth ,
									DOUBLE DepthInc,          /*切割深度*/
									JDNC_LAYER &Layer ,		/*分层*/
									RFRAME &	LocFrame,	/*局部坐标系*/
									BOOL bFst		   ,	/*是否为第一个环*/
									BOOL bLast,			/*是否为最后一个环*/
									int  nRev  )    /*圆弧延伸标记*/
{
	if (m_cFeedDef.m_cPlungeDef.m_nPlungeType == NCDEF_PLUNGE_CONTOUR)
	{
		// 沿轮廓下刀，先生成进刀路径，然后再下刀路径，2013.12.09 liuxin
		return LoopOneLayerByContourPlunge(PComb, AllCont, Contour, Depth, DepthInc, Layer, LocFrame, bFst, bLast, nRev);
	}
	//  STEP 0 : 偏移轮廓, 计算下刀保护边界 
	JDNC_PLUNGE  Plunge = m_cFeedDef.m_cPlungeDef ;
	if( Plunge.m_nPlungeType != NCDEF_PLUNGE_CLOSE )
	{
		Plunge.m_dIncStep =  DepthInc  + Plunge.m_dTopTol ;
	}
	DOUBLE dOverlapLen = m_cParam.m_cLeadOut.m_dOverlap ;
	JDNC_LEAD  leadIn  = m_cFeedDef.m_cLeadDef ;
	JDNC_LEAD  leadOut = m_cFeedDef.m_cLeadDef ;
	UpdateLeadOutParam(m_cParam.m_cLeadOut, leadOut);

	DOUBLE ZShift[2] = {} ;
	GetLeadCutShift(m_cParam.m_cLeadOut, ZShift);
	// STEP 1 : 提取所有轮廓 , 并生成路径
	CSmtLoopArr   AllLoop ;
	Contour.ExtractAllLoop( m_cSetupDef.m_cOrderDef.m_nSortType , AllLoop ) ;
	INT_PTR nLoop = AllLoop.GetSize() ;
	CSmartCurve* pBound  ;
	PNT3D  dPoint, dAtPoint ;
	int nRCompMask = GetContourRCompMask() ;
	JDNC_LEAD  rcompLead = m_cFeedDef.m_cLeadDef ;
	rcompLead.m_dLength = max( 0.05, m_cFeedDef.m_cLeadDef.m_dWearLine) ;
	if( rcompLead.m_nLeadType == 0 ) rcompLead.m_nLeadType = NCDEF_LEAD_SLINE ;
	else rcompLead.m_nLeadType = NCDEF_LEAD_LINE ;
	DOUBLE rcompZShift[2] = { 0.0, 0.0 } ;

	// 判断是否保留曲线高度或者从曲线读取速度
	BOOL bKeep[2] = { FALSE, FALSE } ;
	if(  m_cParam.m_bLoopFlag & NCDEF_FLOOP_REDEPTH  )
	{ 
		bKeep[0] = TRUE ;
	}
	
	for( INT_PTR i = 0 ; i < nLoop ; i ++ ) 
	{
		if( fabs( AllLoop[i]->m_dArea ) > 1.0e10 || 
			(AllLoop[i]->m_bUseFlag & NC_LOOP_OUTER)  || 
			AllLoop[i]->m_pCurve == NULL  )
		{
			continue  ;
		}		
		pBound = AllLoop[i]->m_pCurve->CopyMyself() ;
		pBound->SetAllZValue( 0.0 ) ;
		if( ! pBound ) continue ;		
		if( m_cSetupDef.m_cCorDef.m_nCorType == NCDEF_CORNER_EXTARC && nRev != 0 )
		{//延伸圆弧
			ResetCurve( pBound, m_cProcessDef.m_dExtRadius, nRev ) ;
		}
		pBound->SetFeedType( JDNC_FEEDTYPE_ROUGH ) ;

		// 生成路径并保留曲线高度、得到速度
		// STEP 1 : 保留曲线高度
		CPathCombine tmpBase( NC_WPROCESS_ROUGH ) ;
		if (bLast && m_cParam.m_bLoopFlag & NCDEF_FLOOP_RECUTLAST)
		{// 最后一层重复加工， 2013.12.05,liuxin
			CSmartCurve *pCopyCurve = pBound->CopyMyself();
			for (int k = 0; k < m_cParam.m_nReCutTimes; ++k)
			{							
				pBound->AddCurve(pCopyCurve->CopyMyself());
			}
			tmpBase.AppendCurve3D( *pBound , 0.0, TRUE ) ;
			pCopyCurve->ClearAll();
			delete pCopyCurve;
		}
		else
		{
			tmpBase.AppendCurve3D( *pBound , 0.0, TRUE ) ;
		}
		if( bKeep[0] || bKeep[1] )
		{
			RedepthCPathCombine( m_cListCur, &tmpBase, bKeep, LocFrame);
		}
		if( leadIn.m_nLeadType != NCDEF_LEAD_CLOSE || 
			leadOut.m_nLeadType != NCDEF_LEAD_CLOSE )
		{/* 1: 尽可能搜索到切入切出 */
			CSmartCurve LeadInCv , LeadOutCv, OverlapCv ;
			if( GetMillDir() == 0 ) 
			{
				CreateBoundLeadExNew( leadIn,leadOut,*pBound,OverlapCv, LeadInCv,LeadOutCv, &Contour, dOverlapLen , Depth, ZShift, DepthInc) ;
			}
			else
			{
				CreateBoundLeadExNew( leadOut,leadIn,*pBound,OverlapCv, LeadInCv, LeadOutCv, &Contour, dOverlapLen, Depth, ZShift , DepthInc) ;
			}
			// 将进退刀和重复加工段添加到路径中
			AddLeadInOutAndOverlapForPComb(tmpBase, &OverlapCv, &LeadInCv, &LeadOutCv, 0.0, m_cListCur, bKeep, LocFrame);
			pBound->InsertAfter( LeadInCv, NULL ) ;
			pBound->AppendCurve( LeadOutCv ) ;
		}

		if( GetMillDir() == 0 ) 
		{
			pBound->Reverse() ;
//			tmpBase.ReverseDirect();
			ReversePComb( tmpBase ) ;
		}

		if( nRCompMask != 0 ) 
		{
			CSmartCurve compLeadIn, compLeadOut ;
			int bFlag = AddLeadSectAtEndExNew( rcompLead, rcompLead, *pBound, compLeadIn, compLeadOut, 
				&Contour, 0.0, rcompZShift, TRUE) ;
			// 添加曲线段
			AddLeadInOutForPComb( tmpBase, &compLeadIn, &compLeadOut ) ;
			pBound->InsertAfter(compLeadIn, NULL);
			pBound->AppendCurve(compLeadOut);
			if (!bFlag)
			{
				m_bRComb = 2;
			}                        
		}
		//		CPathCombine tmpBase( NC_WPROCESS_ROUGH ) ;
		//		tmpBase.AppendCurve3D( *pBound , 0.0, TRUE ) ;
		AddRCompMask( tmpBase,  nRCompMask ) ;
		BOOL bCnt = FALSE ;
		if( bFst )
		{
			DOUBLE dComp[2] = { 0., 0. } ;
			GetBoundOffset( Layer.m_dTotalDepth, Depth, dComp ) ;
			bCnt = ConnectLeadInOutPos( tmpBase, AllCont, dComp, m_cParam.m_nRCompSide, m_cFeedDef.m_cLeadDef.m_dLeadPos ) ;
		}

		CPathCombine cTPlunge( NC_WPROCESS_PLUNGE ) ;
		CSmartLoop *pChild = NULL ;
		if( Plunge.m_nPlungeType != NCDEF_PLUNGE_CLOSE )
		{ // 增加下刀方式
			if( bCnt )	tmpBase.GetEndPoint( 0, dPoint ) ;
			else		pBound->GetPoint( 0.0, dPoint ) ;
			dPoint[2]  = 0  ;
			if( Plunge.m_nPlungeType == NCDEF_PLUNGE_VERT ||
				Plunge.m_nPlungeType == NCDEF_PLUNGE_HELIX    || 
				Plunge.m_nPlungeType == NCDEF_PLUNGE_RAMP    )
			{
				DOUBLE dOffset = Plunge.m_dSideTol ; 
				if( Plunge.m_nPlungeType == NCDEF_PLUNGE_HELIX ) 
				{
					double dDist = MathCAM_MiniDistToContour( Contour, dPoint ) ;
					dOffset += max( Plunge.m_dRadius, dDist) ;
				}
				pChild = Contour.OffsetContourEx( dOffset , dOffset, 
					m_cSetupDef.m_cCorDef ) ;
			}
			if( FindPlungePoint( Plunge, Contour, pChild, dPoint, dAtPoint, pBound ) )
			{ // 在固定位置下刀
				AddPlungePathAt( cTPlunge, Plunge, dPoint, dAtPoint,&Contour ) ;
			}
			if( ! cTPlunge.m_pHead )
			{
				AddPlungePathBy( cTPlunge, Plunge , dPoint, *pBound, TRUE ) ;
			}
			// 平移下刀路径，防止路径断开, 2013.12.17 liuxin
			{
				tmpBase.GetEndPoint( 0, dPoint ) ;
				cTPlunge.GetEndPoint(1, dAtPoint);
				cTPlunge.MoveCombine(0.0, 0.0, dPoint[2] - dAtPoint[2]);
			}
			PComb.AppendCombine( cTPlunge ) ;
		}
		Mini_DeleteContours( pChild ) ;

		PComb.AppendCombine( tmpBase ) ;
		delete pBound ;        
	}
	return 1 ; 
}

int CSmartContourGen::LoopOneLayerByContourPlunge( CPathCombine& PComb   ,  /*路径集合*/
												CSmartLoop *AllCont  ,  /*所有区域*/
												CSmartLoop&    Contour,  /*切割轮廓*/
												DOUBLE Depth ,
												DOUBLE DepthInc,          /*切割深度*/
												JDNC_LAYER &Layer ,		/*分层*/
												RFRAME &	LocFrame,	/*局部坐标系*/
												BOOL bFst		   ,	/*是否为第一个环*/
												BOOL bLast,			/*是否为最后一个环*/
												int  nRev  )    /*圆弧延伸标记*/
{
	UNUSED_ALWAYS(bFst);
	UNUSED_ALWAYS(AllCont);
	//  STEP 0 : 偏移轮廓, 计算下刀保护边界 
	JDNC_PLUNGE  Plunge = m_cFeedDef.m_cPlungeDef ;
	Plunge.m_dIncStep =  DepthInc  + Plunge.m_dTopTol ;
	JDNC_LEAD  leadIn  = m_cFeedDef.m_cLeadDef ;
	JDNC_LEAD  leadOut = m_cFeedDef.m_cLeadDef ;
	UpdateLeadOutParam(m_cParam.m_cLeadOut, leadOut);
	// 关闭沿轮廓进刀
	if (leadIn.m_nLeadType == NCDEF_LEAD_BYCONTOUR)
	{
		leadIn.m_nLeadType = NCDEF_LEAD_CLOSE;
	}

	DOUBLE ZShift[2] = {} ;
	GetLeadCutShift(m_cParam.m_cLeadOut, ZShift);
	// STEP 1 : 提取所有轮廓 , 并生成路径
	CSmtLoopArr   AllLoop ;
	Contour.ExtractAllLoop( m_cSetupDef.m_cOrderDef.m_nSortType , AllLoop ) ;
	INT_PTR nLoop = AllLoop.GetSize() ;
	CSmartCurve* pBound  ;
	int nRCompMask = GetContourRCompMask() ;
	JDNC_LEAD  rcompLead = m_cFeedDef.m_cLeadDef ;
	rcompLead.m_dLength = max( 0.05, m_cFeedDef.m_cLeadDef.m_dWearLine) ;
	if( rcompLead.m_nLeadType == 0 ) rcompLead.m_nLeadType = NCDEF_LEAD_SLINE ;
	else rcompLead.m_nLeadType = NCDEF_LEAD_LINE ;
	DOUBLE rcompZShift[2] = { 0.0, 0.0 } ;

	// 判断是否保留曲线高度或者从曲线读取速度
	BOOL bKeep[2] = { FALSE, FALSE } ;
	if(  m_cParam.m_bLoopFlag & NCDEF_FLOOP_REDEPTH  )
	{ 
		bKeep[0] = TRUE ;
	}
	
	for( INT_PTR i = 0 ; i < nLoop ; i ++ ) 
	{
		if( fabs( AllLoop[i]->m_dArea ) > 1.0e10 || 
			(AllLoop[i]->m_bUseFlag & NC_LOOP_OUTER)  || 
			AllLoop[i]->m_pCurve == NULL  )
		{
			continue  ;
		}		
		pBound = AllLoop[i]->m_pCurve->CopyMyself() ;
		pBound->SetAllZValue( 0.0 ) ;
		if( ! pBound ) continue ;		
		if( m_cSetupDef.m_cCorDef.m_nCorType == NCDEF_CORNER_EXTARC && nRev != 0 )
		{//延伸圆弧
			ResetCurve( pBound, m_cProcessDef.m_dExtRadius, nRev ) ;
		}
		pBound->SetFeedType( JDNC_FEEDTYPE_ROUGH ) ;
		if( GetMillDir() == 0 ) 
		{
			pBound->Reverse() ; 
		}

		// 生成路径并保留曲线高度、得到速度
		// STEP 1 : 保留曲线高度
		CPathCombine tmpBase( NC_WPROCESS_ROUGH ) ;
		// 沿轮廓下刀，先生成进刀路径，然后再下刀路径，2013.12.06 liuxin
		DOUBLE dAngle = ANGLE_TO_RADIAN( Plunge.m_dAngle);
		CSmartCurve* pNewBound = NULL  ;
		tmpBase.AddSlantCurve( *pBound ,dAngle,
			Plunge.m_dIncStep,
			- Depth,
			TRUE, &pNewBound ) ;
		if( pNewBound && pNewBound != pBound )
		{
			delete pBound ;							
			pBound = pNewBound  ;						
		} 		
		if (bLast && m_cParam.m_bLoopFlag & NCDEF_FLOOP_RECUTLAST)
		{// 最后一层重复加工， 2013.9.16,liuxin
			for (int k = 0; k < m_cParam.m_nReCutTimes; ++k)
			{
				tmpBase.AddCurve(pBound->CopyMyself(), FALSE, -Layer.m_dLayerDepth[Layer.m_nLayerCount]);
			}
		}
		// 如果保留曲线高度或者读取曲线速度
		if( bKeep[0] || bKeep[1] )
		{
			RedepthCPathCombine( m_cListCur, &tmpBase, bKeep, LocFrame) ;
		}
		AddLeadPathAtEndEx( leadIn, leadOut, tmpBase, Contour, Depth, ZShift, DepthInc ) ;						
		if( nRCompMask != 0 ) 						
		{							
			int bFlag = AddLeadPathAtEndEx( rcompLead, rcompLead, tmpBase, Contour, 0.0, rcompZShift) ;
			AddRCompMask( tmpBase, nRCompMask ) ;
			if (!bFlag)
			{
				m_bRComb = 2;
			}            
		}

		PComb.AppendCombine( tmpBase ) ;
		delete pBound ;        
	}
	return 1 ; 
}

int CSmartContourGen::CopyOneCurve(CPathCombine	& PComb,
	                               CSmartCurve	*&Curve,
	                               JDNC_LAYER	& Layer ,
	                               JDNC_PLUNGE	& Plunge)
{
	DOUBLE dIncStep = 0.0;
	DOUBLE dDepth = 0.0; 
	DOUBLE dPlungeDepth = 0.0;
	DOUBLE dPlungeAngle = ANGLE_TO_RADIAN( Plunge.m_dAngle);
	double dMaxZIncAngle = atan2( Plunge.m_dMaxZInc, Curve->GetLength()) ;
	DOUBLE dAngle = dPlungeAngle;
	CPathCombine  LyComb( NC_WPROCESS_ROUGH )  ;
	CSmartCurve* pNewCurve = NULL;
	if (m_cParam.m_bLoopFlag & NCDEF_FLOOP_MOVEUP)
	{// 从下向上切割
		Curve->Reverse();
		for( int k = 1 ; k <= Layer.m_nLayerCount ; k ++ )
		{
			Plunge.m_dIncStep = Layer.m_dLayerDepth[ k ] - Layer.m_dLayerDepth[ k-1 ] ;
			if(  k == Layer.m_nLayerCount ) 
			{
				Plunge.m_dIncStep += Plunge.m_dTopTol ; 
			} 
			/////////////////////////////////////////////////////////////////
			// edit by liuxin, 2013.7.23;
			dAngle = dPlungeAngle;
			if (Layer.m_nLayerCount == 1 && Plunge.m_bPlungeFlag & NCDEF_PLUNGE_ROUNDPLUNGE)
			{	
				// 保证下刀位置和抬刀位置一致
				dPlungeDepth = tan(dAngle) * Curve->GetLength();
				dDepth = min(dPlungeDepth,Plunge.m_dMaxZInc);
				dDepth = min(Plunge.m_dIncStep, dDepth);
				int nCnt = int(Plunge.m_dIncStep/dDepth + 0.5);
				if (nCnt < 1)
				{
					nCnt = 1;
				}
				dDepth = Plunge.m_dIncStep/nCnt;
				dAngle = atan2( dDepth, Curve->GetLength()) ;
			}
			else if (Layer.m_nLayerType == NCDEF_LAYER_GRAD || Layer.m_nLayerType == NCDEF_LAYER_DEF && Layer.m_bLayerFlag & NCDEF_LAYER_DEFGRAD)
			{// 渐变分层时，下刀深度等于分层深度，保证生成深度均匀变化的螺旋路径，2013.10.17 liuxin
				dAngle = atan2( Plunge.m_dIncStep, Curve->GetLength()) ;
			}
			else
			{
				// 保证每层深度不超过最大下刀深度
				if( Plunge.m_dIncStep > Plunge.m_dMaxZInc )
				{
					if( dAngle > dMaxZIncAngle )
						dAngle = dMaxZIncAngle ;
				}
			}
			///////////////////////////////////////////////////////////////////
			pNewCurve = NULL  ;
			BOOL bOverLap = (k == Layer.m_nLayerCount) ? TRUE : FALSE ;
			dIncStep = Plunge.m_dIncStep;
			dDepth = - Layer.m_dLayerDepth[k];
			if (k == Layer.m_nLayerCount)
			{
				dDepth -= Plunge.m_dTopTol;
			}
			LyComb.AddSlantCurve( *Curve ,dAngle , 
				dIncStep, 
				dDepth, 
				bOverLap ,
				&pNewCurve ) ;
			if( pNewCurve && pNewCurve != Curve )
			{
				delete Curve ;							
				Curve = pNewCurve  ;						
			} 		
		}
//		CPathEntity* pPrevEnt = NULL;
//		CPathEntity* pHeadEnt = LyComb.m_pHead;
		CPathEntity* pTailEnt = LyComb.m_pTail;
		CPathCombine  RevLyComb( NC_WPROCESS_ROUGH );
		TPNT3D ptStart, ptEnd;
		DOUBLE dDis = 0.0;
		// 删除最后一层切削路径
		pTailEnt = LyComb.m_pTail;
		while (pTailEnt && pTailEnt->m_bFeedType == JDNC_FEEDTYPE_ROUGH)
		{
			LyComb.RemoveEntity(pTailEnt);
			delete pTailEnt;
			pTailEnt = LyComb.m_pTail;
		}
		// 反向路径
		ReversePComb( LyComb ) ;

		// 添加最后一层路径
		LyComb.GetEndPoint(1, ptEnd);
		CSmartSect* pObjSect = Curve->MinDistSect(ptEnd, ptStart, dDis);
		Curve->SetStartPoint(pObjSect, ptStart);
		Curve->Reverse();
		LyComb.AddCurve(Curve->CopyMyself(), TRUE, -Layer.m_dLayerDepth[0]);
		// 添加重复加工路径						
		if (m_cParam.m_bLoopFlag & NCDEF_FLOOP_RECUTLAST)
		{
			for (int k = 0; k < m_cParam.m_nReCutTimes; ++k)
			{
				LyComb.AddCurve(Curve->CopyMyself(), TRUE, -Layer.m_dLayerDepth[0]);
			}
		}
	}
	else
	{// 从上向下切割
		for( int k = 1 ; k <= Layer.m_nLayerCount ; k ++ )
		{
			Plunge.m_dIncStep = Layer.m_dLayerDepth[ k ] - Layer.m_dLayerDepth[ k-1 ] ;
			if(  k == 1 ) 
			{
				Plunge.m_dIncStep += Plunge.m_dTopTol ; 
			} 
			/////////////////////////////////////////////////////////////////
			// edit by liuxin, 2013.7.23;
			dAngle = dPlungeAngle;
			if (Layer.m_nLayerCount == 1 && Plunge.m_bPlungeFlag & NCDEF_PLUNGE_ROUNDPLUNGE)
			{	
				// 保证下刀位置和抬刀位置一致
				dPlungeDepth = tan(dAngle) * Curve->GetLength();
				dDepth = min(dPlungeDepth,Plunge.m_dMaxZInc);
				dDepth = min(Plunge.m_dIncStep, dDepth);
				int nCnt = int(Plunge.m_dIncStep/dDepth + 0.5);
				if (nCnt < 1)
				{
					nCnt = 1;
				}
				dDepth = Plunge.m_dIncStep/nCnt;
				dAngle = atan2( dDepth, Curve->GetLength()) ;
			}
			else if (Layer.m_nLayerType == NCDEF_LAYER_GRAD || Layer.m_nLayerType == NCDEF_LAYER_DEF && Layer.m_bLayerFlag & NCDEF_LAYER_DEFGRAD)
			{// 渐变分层时，下刀深度等于分层深度，保证生成深度均匀变化的螺旋路径，2013.10.17 liuxin
				dAngle = atan2( Plunge.m_dIncStep, Curve->GetLength()) ;
			}
			else
			{
				// 保证每层深度不超过最大下刀深度
				if( Plunge.m_dIncStep > Plunge.m_dMaxZInc )
				{
					if( dAngle > dMaxZIncAngle )
						dAngle = dMaxZIncAngle ;
				}
			}
			///////////////////////////////////////////////////////////////////
			pNewCurve = NULL  ;
			BOOL bOverLap = (k == Layer.m_nLayerCount) ? TRUE : FALSE ;
			dIncStep = Plunge.m_dIncStep;
			dDepth = - Layer.m_dLayerDepth[k];
			LyComb.AddSlantCurve( *Curve ,dAngle , 
				dIncStep, 
				dDepth, 
				bOverLap ,
				&pNewCurve ) ;
			if( pNewCurve && pNewCurve != Curve )
			{
				delete Curve ;							
				Curve = pNewCurve  ;						
			} 		
		}
		if (m_cParam.m_bLoopFlag & NCDEF_FLOOP_RECUTLAST)
		{// 最后一层重复加工， 2013.9.16,liuxin
			for (int k = 0; k < m_cParam.m_nReCutTimes; ++k)
			{
				LyComb.AddCurve(Curve->CopyMyself(), TRUE, -Layer.m_dLayerDepth[Layer.m_nLayerCount]);
			}				
		}
	}

	PComb.AppendCombine(LyComb);

	return TRUE;
}

int CSmartContourGen::CopyOneContour( CPathCombine& PComb,    /*数据*/
									  CSmartLoop *AllCont,	/*所有区域*/
							          CSmartLoop& Contour,    /*轮廓*/
							          JDNC_LAYER&  Layer ,    /*参数*/
									  RFRAME &	LocFrame,	/*局部坐标系*/
							          JDNC_PLUNGE& Plunge,   /*下刀*/
									  BOOL bFst		   ,	/*是否为第一个环*/
									  int nRev )    /*圆弧延伸标记*/
{
    DOUBLE Depth = /*0.0*/Layer.m_dTotalDepth;
	DOUBLE DepthInc = Depth/Layer.m_nLayerCount;
	CSmtLoopArr  AllLoop ;
	Contour.ExtractAllLoop( m_cSetupDef.m_cOrderDef.m_nSortType, AllLoop ) ;
	INT_PTR n = AllLoop.GetSize() ; 
	int nRCompMask = GetContourRCompMask() ;
	JDNC_LEAD  rcompLead = m_cFeedDef.m_cLeadDef ;
	rcompLead.m_dLength = max( 0.05, m_cFeedDef.m_cLeadDef.m_dWearLine) ;
	if( rcompLead.m_nLeadType == 0 ) rcompLead.m_nLeadType = NCDEF_LEAD_SLINE ;
	else   rcompLead.m_nLeadType = NCDEF_LEAD_LINE ;
    DOUBLE rcompZShift[2] = { 0.0, 0.0 } ;
	JDNC_LEAD  leadIn  = m_cFeedDef.m_cLeadDef ;
	JDNC_LEAD  leadOut = m_cFeedDef.m_cLeadDef ;
	UpdateLeadOutParam(m_cParam.m_cLeadOut, leadOut);
	// 沿轮廓下刀时关闭沿轮廓进刀
	if (leadIn.m_nLeadType == NCDEF_LEAD_BYCONTOUR)
	{
		leadIn.m_nLeadType = NCDEF_LEAD_CLOSE;
	}

	// 判断是否保留曲线高度或者从曲线读取速度
	BOOL bKeep[2] = { FALSE, FALSE } ;
	if(  m_cParam.m_bLoopFlag & NCDEF_FLOOP_REDEPTH  )
	{ 
		bKeep[0] = TRUE ;
	}
	
	DOUBLE ZShift[2] = {} ;
	GetLeadCutShift(m_cParam.m_cLeadOut, ZShift);
	for( INT_PTR i = 0 ; i < n ; i ++ )
	{
		if( fabs( AllLoop[i]->m_dArea ) > 1.0e10 || 
			(AllLoop[i]->m_bUseFlag & NC_LOOP_OUTER)  || 
			AllLoop[i]->m_pCurve == NULL  )
		{
			continue  ;
		}
		CSmartCurve* pCurve = AllLoop[i]->m_pCurve->CopyMyself()   ;
		if( GetMillDir() == 0 ) pCurve->Reverse() ;
		DOUBLE dAngle = ANGLE_TO_RADIAN( Plunge.m_dAngle);
		if( Layer.m_bLayerFlag & NCDEF_LAYER_KEEPDOWN )
		{// 分层不抬刀
			CPathCombine  LyComb( NC_WPROCESS_ROUGH )  ;
			if( m_cSetupDef.m_cCorDef.m_nCorType == NCDEF_CORNER_EXTARC && nRev != 0 )
			{//延伸圆弧
				ResetCurve( pCurve, m_cProcessDef.m_dExtRadius, nRev ) ;
			}
			CopyOneCurve(LyComb, pCurve, Layer, Plunge);	
			// 如果保留曲线高度或者读取曲线速度
			if( bKeep[0] || bKeep[1] )
			{
				RedepthCPathCombine( m_cListCur, &LyComb, bKeep, LocFrame) ;
			}
			//添加重复加工路径,必须在退刀之前 20140901 xh
			if (m_cMethodDef.m_nMethodType == surfncMethodLoop)
			{
				CPathCombine overLappedPath(NC_WPROCESS_ROUGH);
				double dOverlapLen = m_cMethodDef.m_cLoopCut.m_cLeadOut.m_dOverlap ;//重复加工长度  20140827 xh
				//添加重复加工路径
				GenOverLappedPath(LyComb, overLappedPath, dOverlapLen);
				LyComb.AppendCombine( overLappedPath ) ;
			}
			AddLeadPathAtEndEx( leadIn, leadOut, LyComb, Contour, Depth, ZShift, DepthInc ) ;
			if( nRCompMask != 0 ) 						
			{							
				int bFlag = AddLeadPathAtEndEx( rcompLead, rcompLead, LyComb, Contour, 0.0, rcompZShift) ;
				AddRCompMask( LyComb, nRCompMask ) ;
				if (!bFlag)
				{
					m_bRComb = 2;
				}            
			}
			// 连接进退刀点
			if( bFst )
			{
				DOUBLE dComp[2] = { 0., 0. } ;
				BOOL bCnt[2] = { FALSE, FALSE } ;
				GetBoundOffset( Layer.m_dTotalDepth, Layer.m_dLayerDepth[Layer.m_nLayerCount], dComp ) ;
//				ConnectLeadInOutPos( LyComb, AllCont, dComp, m_cParam.m_nRCompSide, m_cFeedDef.m_cLeadDef.m_dLeadPos ) ;
				ConnectLeadInOutPos( LyComb, AllCont, dComp, m_cParam.m_nRCompSide, m_cParam.m_cLeadOut, bCnt ) ;
			}
			PComb.AppendCombine( LyComb ) ;											
		}
		else
		{
			CPathCombine  LyComb( NC_WPROCESS_ROUGH )  ;
			CSmartCurve* pNewCurve = NULL  ;
			if( m_cSetupDef.m_cCorDef.m_nCorType == NCDEF_CORNER_EXTARC && nRev != 0)
			{//延伸圆弧
				ResetCurve( pCurve, m_cProcessDef.m_dExtRadius, nRev ) ;
			}
			for( int k = 1 ; k <= Layer.m_nLayerCount ; k ++ )
			{
				Plunge.m_dIncStep = Layer.m_dLayerDepth[ k ] - Layer.m_dLayerDepth[ k-1 ] ;
				if(  k == 1 ) Plunge.m_dIncStep += Plunge.m_dTopTol ; 
				LyComb.AddSlantCurve( *pCurve ,dAngle,
									Plunge.m_dIncStep,
									- Layer.m_dLayerDepth[k],
									TRUE, &pNewCurve ) ;
				if( pNewCurve && pNewCurve != pCurve )
				{
					delete pCurve ;							
					pCurve = pNewCurve  ;						
				} 		
			}
			if (m_cParam.m_bLoopFlag & NCDEF_FLOOP_RECUTLAST)
			{// 最后一层重复加工， 2013.9.16,liuxin
				for (int k = 0; k < m_cParam.m_nReCutTimes; ++k)
				{
					LyComb.AddCurve(pCurve->CopyMyself(), FALSE, -Layer.m_dLayerDepth[Layer.m_nLayerCount]);
				}
			}

			// 如果保留曲线高度或者读取曲线速度
			if( bKeep[0] || bKeep[1] )
			{
				RedepthCPathCombine( m_cListCur, &LyComb, bKeep, LocFrame) ;
			}
			// 沿轮廓下刀时，只有最后一层有退刀路径，所以退刀路径的末点高度应为倒数第二层路径的高度既Depth - DepthInc qqs 2014.02.17
			AddLeadPathAtEndEx( leadIn, leadOut, LyComb, Contour, Depth, ZShift, DepthInc ) ; 						
			if( nRCompMask != 0 ) 						
			{							
				int bFlag = AddLeadPathAtEndEx( rcompLead, rcompLead, LyComb, Contour, 0.0, rcompZShift) ;
				AddRCompMask( LyComb, nRCompMask ) ;
				if (!bFlag)
				{
					m_bRComb = 2;
				}            
			}
			// 连接进退刀点
			if( bFst )
			{
				DOUBLE dComp[2] = { 0., 0. } ;
				BOOL bCnt[2] = { FALSE, FALSE } ;
				GetBoundOffset( Layer.m_dTotalDepth, Layer.m_dLayerDepth[Layer.m_nLayerCount], dComp ) ;
//				ConnectLeadInOutPos( LyComb, AllCont, dComp, m_cParam.m_nRCompSide, m_cFeedDef.m_cLeadDef.m_dLeadPos ) ;
				ConnectLeadInOutPos( LyComb, AllCont, dComp, m_cParam.m_nRCompSide, m_cParam.m_cLeadOut, bCnt ) ;
			}
			PComb.AppendCombine( LyComb ) ;										
		}
		delete pCurve ;
	}
	return TRUE ;
}

BOOL CSmartContourGen::IsSameOffset()
{
	DOUBLE dAngle[2]; 
	dAngle[0] = ANGLE_TO_RADIAN( m_cShapeDef.m_cTaper.m_dBoundAngle  ) ;
	dAngle[1] = ANGLE_TO_RADIAN( m_cShapeDef.m_cTaper.m_dIslandAngle ) ;
    if( m_cParam.m_nRCompSide ==  2 )
	{ /* 关闭补偿 */
		return TRUE ;
	}
	else if(m_cParam.m_bLoopFlag & NCDEF_FLOOP_USERRCOMP ) 
	{ /* 定义补偿值 */
		return TRUE ;
	}
	else if( m_cShapeDef.m_bAntiFace && dAngle[0] > 0.001 )
	{/* 底面效果 */
		if( m_cFeedDef.m_cLayerDef.m_bLayerFlag & NCDEF_LAYER_COPYMODE )
		{
			return TRUE ;
		}
	}
	else if( m_cFeedDef.m_cLayerDef.m_bLayerFlag & NCDEF_LAYER_COPYMODE )
	{ /*  拷贝分层  */
		return TRUE ;
	}
	else if( dAngle[0] < 0.001 && m_cToolDef.m_nToolType == surfncToolFlat )
	{
		return TRUE ;
	}
	else if (m_cFeedDef.m_cLayerDef.m_nLayerCount == 1)
	{/*只有一层路径*/
		return TRUE;
	}
	return FALSE ;
}

CSmartLoop* CSmartContourGen::CreateLoopBound( CSmartLoop& Contour, 
											   DOUBLE TotalDepth  ,
											   DOUBLE Depth,
											   int* nRev ) 
{
	CSmartLoop *ContHead = NULL ; 
	DOUBLE dRComp[2] , dAngle[2]; 
	dAngle[0] = ANGLE_TO_RADIAN( m_cShapeDef.m_cTaper.m_dBoundAngle  ) ;
	dAngle[1] = ANGLE_TO_RADIAN( m_cShapeDef.m_cTaper.m_dIslandAngle ) ;
    if( m_cParam.m_nRCompSide ==  2 )
	{ /* 关闭补偿 */
		dRComp[0] = dRComp[1] = 0.0 ;
	}
	else if(m_cParam.m_bLoopFlag & NCDEF_FLOOP_USERRCOMP ) 
	{ /* 定义补偿值 */
		dRComp[0] = dRComp[1] = m_cParam.m_dRCompDist; 
	}
	else if( m_cShapeDef.m_bAntiFace && dAngle[0] > 0.001 )
	{/* 底面效果 */
		if( m_cFeedDef.m_cLayerDef.m_bLayerFlag & NCDEF_LAYER_COPYMODE )
		{
		}
		else
		{
			double dMyDepth = (TotalDepth - Depth ) * tan( dAngle[0] ) ;
			if( dMyDepth > 1.0e-3 )
			{
				dRComp[0] = dRComp[1] = dMyDepth ;
				if( m_cSetupDef.m_cCorDef.m_nCorType == NCDEF_CORNER_EXTARC )
				{
					 m_cSetupDef.m_cCorDef.m_nCorType = NCDEF_CORNER_EXTUSER ;
					 ContHead = Contour.OffsetContour( NCDEF_OFFSET_OUTER,dRComp[0], dRComp[1],GetCorDef() ) ;
					 m_cSetupDef.m_cCorDef.m_nCorType = NCDEF_CORNER_EXTARC ;
				}
				else
				{
					ContHead = Contour.OffsetContour( NCDEF_OFFSET_OUTER,dRComp[0], dRComp[1],GetCorDef() ) ;
				}
				
			}
		}
		dRComp[0] = m_pTool->GetRadiusComp( 0.0, 0.0) ; 
        dRComp[1] = m_pTool->GetRadiusComp( 0.0, 0.0) ; 
	}
	else if( m_cFeedDef.m_cLayerDef.m_bLayerFlag & NCDEF_LAYER_COPYMODE  || m_cFeedDef.m_cLayerDef.m_nLayerCount == 1)
	{ /*  拷贝分层  */
		dRComp[0] = m_pTool->GetRadiusComp( TotalDepth, dAngle[0]) ; 
	    dRComp[1] = m_pTool->GetRadiusComp( TotalDepth, dAngle[1]) ; 
	}
	else 
	{
		dRComp[0] = m_pTool->GetRadiusComp( Depth, dAngle[0]) ; 
		dRComp[1] = m_pTool->GetRadiusComp( Depth, dAngle[1]) ; 
	}
	dRComp[0] += GetSideStock() ;
	dRComp[1] += GetSideStock() ;
	*nRev = ( dRComp[0] < 0 ) ? 1 : 2 ;
	if( ContHead && fabs(dRComp[0]) > 1.0e-3 )
	{
		CSmartLoop* tmpLoop = ContHead; 
		if( dRComp[0] > 0.0 )
		{
			if( m_cSetupDef.m_cCorDef.m_nCorType == NCDEF_CORNER_EXTARC && fabs( dRComp[0]) > 1.0e-4 )
			{
				m_cSetupDef.m_cCorDef.m_nCorType = NCDEF_CORNER_EXTUSER ;
				ContHead = Mini_OffsetContourList( tmpLoop ,NCDEF_OFFSET_INNER, dRComp,GetCorDef() ) ;
				m_cSetupDef.m_cCorDef.m_nCorType = NCDEF_CORNER_EXTARC ;
			}
			else
			{
				ContHead = Mini_OffsetContourList( tmpLoop ,NCDEF_OFFSET_INNER, dRComp,GetCorDef() ) ;
			}

		}
		else
		{
			dRComp[0] =- dRComp[0] , dRComp[1] = -dRComp[1] ;
			if( m_cSetupDef.m_cCorDef.m_nCorType == NCDEF_CORNER_EXTARC && fabs( dRComp[0]) > 1.0e-4 )
			{
				m_cSetupDef.m_cCorDef.m_nCorType = NCDEF_CORNER_EXTUSER ;
				ContHead = Mini_OffsetContourList( tmpLoop ,NCDEF_OFFSET_OUTER, dRComp,GetCorDef() ) ;
				m_cSetupDef.m_cCorDef.m_nCorType = NCDEF_CORNER_EXTARC ;
			}
			else
			{
				ContHead = Mini_OffsetContourList( tmpLoop ,NCDEF_OFFSET_OUTER, dRComp,GetCorDef() ) ;
			}
			//ContHead = Mini_OffsetContourList( tmpLoop ,NCDEF_OFFSET_OUTER, dRComp,GetCorDef() ) ;
		}
		Mini_DeleteContours( tmpLoop ) ;
	}
	else
	{
		if( m_cSetupDef.m_cCorDef.m_nCorType == NCDEF_CORNER_EXTARC && fabs( dRComp[0]) > 1.0e-4 )
		{
			m_cSetupDef.m_cCorDef.m_nCorType = NCDEF_CORNER_EXTUSER ;
			ContHead = Contour.OffsetContourEx( dRComp[0], dRComp[1],GetCorDef() ) ;
			m_cSetupDef.m_cCorDef.m_nCorType = NCDEF_CORNER_EXTARC ;
		}
		else
		{
			ContHead = Contour.OffsetContourEx( dRComp[0], dRComp[1],GetCorDef() ) ;
		}
	}
	if( m_cParam.m_bLoopFlag & NCDEF_FLOOP_AUTOSTART )
	{ /*优化路径起点*/
		BOOL bAt = 0 ;
		CSmartSect* pSect, *pObj ;
		PNT2D dSeed, dPnt ;
		for( CSmartLoop * pLoop = ContHead ; pLoop ; pLoop = pLoop->next )
		{
			for( CSmartLoop * pLp = pLoop ; pLp ; )
			{
				CSmartCurve * pCurve = pLp->GetCurve() ;
				if( pCurve == NULL ) 
				{
					if( pLp == pLoop ) pLp = pLoop->GetIsland() ;
				    else pLp = pLp->next ;
					continue ;
				}
				bAt  = FALSE ;
				if( GetMillDir() == 1 ) bAt = !bAt ;
				if( pLp == pLoop ) bAt = !bAt  ;
                if( m_cParam.m_nRCompSide == 0 ) bAt = !bAt ;
				pObj = pCurve->GetHead() ; 
				pObj->GetPoint( 0.0, dSeed ) ;
				for( pSect = pObj->next ; pSect ; pSect = pSect->next )
				{
					pSect->GetPoint( 0.0, dPnt ) ;
					if( bAt == 0  )
					{/*右下角*/
						if( (dPnt[1] - dSeed[1] ) < 0.01 && 
						    (dPnt[0] - dSeed[0] ) > -0.01 )
						{ 
							pObj = pSect ;
							memcpy( dSeed, dPnt, sizeof( PNT2D ) ) ;
						} 
					}
				    else 
					{/*左上角*/
						if( (dSeed[1] - dPnt[1]) < 0.01 && 
						    (dSeed[0] - dPnt[0]) > -0.01    )
						{  
							pObj = pSect ;
							memcpy( dSeed, dPnt, sizeof( PNT2D ) ) ;
						}  
					}
				}
				if( pObj != pCurve->GetHead() )
				{
					pCurve->SetStartSect( pObj ) ;
				}
				if( pLp == pLoop ) pLp = pLoop->GetIsland() ;
				else pLp = pLp->next ;
			}
		}
	}
	ContHead = Contour.ResortContour( ContHead, m_cSetupDef.m_cOrderDef.m_nSortType, 0 ) ;
	return ContHead ;
}
int CSmartContourGen::GetContourRCompMask()
{
    if( m_cParam.m_nRCompSide == 2 ) 
    {
        return 0 ;
    }
    if( m_cSetupDef.m_cModelTol.m_nMdlFlag & NCDEF_SETUP_WEARCOMP_INC )
    {
        if( GetMillDir() == 0 ) return JDNC_FGCODE_RCOMPG41 ;
        return JDNC_FGCODE_RCOMPG42 ;
    }
    else if( m_cSetupDef.m_cModelTol.m_nMdlFlag & NCDEF_SETUP_WEARCOMP_DEC )
    {
        if( GetMillDir() == 0 ) return JDNC_FGCODE_RCOMPG42 ;
        return JDNC_FGCODE_RCOMPG41 ;
    }
    return 0 ;
}

// 计算一个环的切割路径
int CSmartContourGen::ContourOneLoop( CPathCombine& PComb,    /*加工数据*/
									  CSmartLoop *AllCont,    /*区域数组*/
				                      CSmartLoop& Loop   ,    /*轮廓曲线*/
						              DOUBLE  Depth      ,    /*深度位置*/
						              DOUBLE  DepthInc,       /*吃刀深度*/
									  JDNC_LAYER &Layer ,		/*分层*/
									  RFRAME &	LocFrame,	/*局部坐标系*/
									  BOOL bFst		   ,	/*是否为第一个环*/
									  BOOL	bLast,			/*是否为最后一层*/
									  int nRev,              /*延伸圆弧标记*/
									  int nLp  )            /*是否为该加工轮廓第一个环 qqs 2013.03.26*/ 
{
	if (m_cFeedDef.m_cPlungeDef.m_nPlungeType == NCDEF_PLUNGE_CONTOUR)
	{
		return ContourOneLoopByContourPlunge(PComb, AllCont, Loop, Depth, DepthInc, 
											Layer, LocFrame, bFst, bLast, nRev, nLp);
	}
	//  STEP 1 : 计算进刀路径
	CPathCombine  tmpComb( NC_WPROCESS_ROUGH ) ;
	CSmartLoop * pContour = &Loop ;
	if( Loop.m_pParent ) pContour = Loop.m_pParent ;
	CSmartCurve * pBound = Loop.m_pCurve->CopyMyself() ;
    pBound->SetAllZValue( 0.0 ) ;
	DOUBLE dOverlapLen = m_cParam.m_cLeadOut.m_dOverlap ;
	JDNC_LEAD  leadIn  = m_cFeedDef.m_cLeadDef ;
	JDNC_LEAD  leadOut = m_cFeedDef.m_cLeadDef ;
	UpdateLeadOutParam(m_cParam.m_cLeadOut, leadOut);

	DOUBLE ZShift[2] = {} ;
	GetLeadCutShift(m_cParam.m_cLeadOut, ZShift);
	int nRCompMask = GetContourRCompMask() ;
	JDNC_LEAD  rcompLead = m_cFeedDef.m_cLeadDef ;
	rcompLead.m_dLength = max( 0.05, m_cFeedDef.m_cLeadDef.m_dWearLine) ;
	if( rcompLead.m_nLeadType == 0 ) rcompLead.m_nLeadType = NCDEF_LEAD_SLINE ;
	else   rcompLead.m_nLeadType = NCDEF_LEAD_LINE ;
    DOUBLE rcompZShift[2] = { 0.0, 0.0 } ;

	if( m_cSetupDef.m_cCorDef.m_nCorType == NCDEF_CORNER_EXTARC && nRev != 0)
	{//延伸圆弧
		ResetCurve( pBound, m_cProcessDef.m_dExtRadius, nRev ) ;	
	}
	pBound->SetFeedType( JDNC_FEEDTYPE_ROUGH ) ;
	// 判断是否保留曲线高度或者从曲线读取速度
	BOOL bKeep[2] = { FALSE, FALSE } ;
	if(  m_cParam.m_bLoopFlag & NCDEF_FLOOP_REDEPTH  )
	{ 
		bKeep[0] = TRUE ;
	}
	// 生成路径并保留曲线高度、得到速度
	CPathCombine tmpBase( NC_WPROCESS_ROUGH ) ;
	if (bLast && m_cParam.m_bLoopFlag & NCDEF_FLOOP_RECUTLAST)
	{// 最后一层重复加工， 2013.12.05,liuxin
		CSmartCurve *pCopyCurve = pBound->CopyMyself();
		for (int k = 0; k < m_cParam.m_nReCutTimes; ++k)
		{							
			pBound->AddCurve(pCopyCurve->CopyMyself());
		}
//		tmpBase.AppendCurve3D( *pBound , -Depth, TRUE ) ;
		pCopyCurve->ClearAll();
		delete pCopyCurve;
	}
	// 封掉原因：因圆弧进退刀有可能会引起加工曲线起始点的变化，
	// 所以需要在生成完进退刀路径之后才将曲线转化为路径，否则路径起点会不正确 qqs 2014.02.26
//	else
//	{
//		tmpBase.AppendCurve3D( *pBound , -Depth, TRUE ) ;
//	}
				
//	if( bKeep[0] || bKeep[1] )
//	{
//		RedepthCPathCombine( m_cListCur, &tmpBase, bKeep, LocFrame);
//	}

	if( leadIn.m_nLeadType != NCDEF_LEAD_CLOSE || 
		leadOut.m_nLeadType != NCDEF_LEAD_CLOSE )
	{/* 1: 尽可能搜索到切入切出 */
		CSmartCurve  LeadInCv , LeadOutCv, OverlapCv ; 
		if( GetMillDir() == 0 ) 
		{
			CreateBoundLeadExNew(leadOut,leadIn,*pBound,OverlapCv,LeadInCv,LeadOutCv,pContour, dOverlapLen, Depth, ZShift, DepthInc ) ;
		}
		else
		{
			CreateBoundLeadExNew(leadIn,leadOut,*pBound,OverlapCv,LeadInCv,LeadOutCv,pContour, dOverlapLen, Depth, ZShift,DepthInc ) ;
		}
		// 圆弧进退刀有可能会引起加工曲线起始点的变化，所以需要在生成完进退刀路径之后才将曲线转化为路径，否则路径起点会不正确 qqs 2014.02.26
		tmpBase.AppendCurve3D( *pBound , -Depth, TRUE ) ;
		if( bKeep[0] || bKeep[1] )
		{
			RedepthCPathCombine( m_cListCur, &tmpBase, bKeep, LocFrame);
		}
		// 将进退刀和重复加工段添加到路径中
		AddLeadInOutAndOverlapForPComb(tmpBase, &OverlapCv, &LeadInCv, &LeadOutCv, Depth, m_cListCur, bKeep, LocFrame);
		pBound->InsertAfter( LeadInCv, NULL ) ;
		pBound->AppendCurve( LeadOutCv ) ;
	}
	else
	{
		tmpBase.AppendCurve3D( *pBound , -Depth, TRUE ) ;
		//考虑重复加工长度 20141010  xh
		CSmartCurve overLapped;
		double dOverlapLen = m_cMethodDef.m_cLoopCut.m_cLeadOut.m_dOverlap ;
		if ( abs(dOverlapLen) > 0.0001)
		{
			PNT3D EndPoint ;
			tmpBase.GetEndPoint( 1, EndPoint );
			GenOverLappedPath( *pBound, overLapped, dOverlapLen);
			tmpBase.AppendCurve3D( overLapped, EndPoint[2], NC_WPROCESS_ROUGH );
			pBound->AddCurve( overLapped.CopyMyself() );
		}
		if( bKeep[0] || bKeep[1] )
		{
			RedepthCPathCombine( m_cListCur, &tmpBase, bKeep, LocFrame);
		}
	}

	if( GetMillDir() == 0 ) 
	{
		pBound->Reverse() ;
//		tmpBase.ReverseDirect();
		ReversePComb( tmpBase ) ;
	}
	
	if( nRCompMask != 0 ) 
	{
		CSmartCurve compLeadIn, compLeadOut ;
		int bFlag = AddLeadSectAtEndExNew( rcompLead, rcompLead, *pBound, compLeadIn, compLeadOut,
											pContour, 0.0, rcompZShift, TRUE ) ;

		AddLeadInOutForPComb( tmpBase, &compLeadIn, &compLeadOut ) ;
		pBound->InsertAfter(compLeadIn, NULL);
		pBound->AppendCurve(compLeadOut);
		if (!bFlag)
		{
			m_bRComb = 2;
		}            
	}
//	CPathCombine tmpBase( NC_WPROCESS_ROUGH ) ;
//	tmpBase.AppendCurve3D( *pBound , -Depth, TRUE ) ;
	AddRCompMask( tmpBase, nRCompMask ) ;
	// 添加连接仅提到功能
	BOOL bCnt[2] = { FALSE, FALSE } ;
	if( bFst )
	{
		DOUBLE dComp[2] = { 0., 0. } ;
		GetBoundOffset( Layer.m_dTotalDepth, Depth, dComp ) ;
	//	bCnt = ConnectLeadInOutPos( tmpBase, AllCont, dComp, m_cParam.m_nRCompSide, m_cFeedDef.m_cLeadDef.m_dLeadPos ) ;
		ConnectLeadInOutPos( tmpBase, AllCont, dComp, m_cParam.m_nRCompSide, m_cParam.m_cLeadOut, bCnt ) ;
	}
	// STEP 2 :  计算下刀路径
	JDNC_PLUNGE  Plunge = m_cFeedDef.m_cPlungeDef ;	
	if( Plunge.m_nPlungeType != NCDEF_PLUNGE_CLOSE )	
	{	
		PNT3D dPoint , dAtPoint;		
		if( bCnt[0] )	tmpBase.GetEndPoint( 0, dPoint ) ;
		else		pBound->GetPoint( 0.0, dPoint ) ;
		dPoint[2]  = 0  ;
		CSmartLoop *pChild = NULL ; 
		Plunge.m_dIncStep =  DepthInc  + Plunge.m_dTopTol ;
		if( Plunge.m_nPlungeType == NCDEF_PLUNGE_VERT ||
			Plunge.m_nPlungeType == NCDEF_PLUNGE_HELIX || 
			Plunge.m_nPlungeType == NCDEF_PLUNGE_RAMP   )
		{
			DOUBLE dOffset = Plunge.m_dSideTol ; 
			if( Plunge.m_nPlungeType == NCDEF_PLUNGE_HELIX ) 			
			{				
				//double dDist = MathCAM_MiniDistToContour( *pContour, dPoint ) ;				
				//dOffset += max( Plunge.m_dRadius, dDist) ;
				dOffset += Plunge.m_dRadius ;
			}
			pChild = pContour->OffsetContourEx( dOffset , dOffset, 
				m_cSetupDef.m_cCorDef ) ;
		}
		CPathCombine cTPlunge( NC_WPROCESS_PLUNGE ) ;
		int nHexReff = FindPlungePoint( Plunge, *pContour, pChild, dPoint, dAtPoint, pBound ) ;
		if( nHexReff > 0 )		
		{ // 在固定位置下刀
			AddPlungePathAt( cTPlunge, Plunge, dPoint, dAtPoint, pContour, nHexReff ) ;		
		}		
		if( ! cTPlunge.m_pHead )		
		{			
			AddPlungePathBy( cTPlunge, Plunge , dPoint, *pBound, TRUE ) ;		
		}		
		//cTPlunge.MoveCombine( 0.0, 0.0, -Depth ) ;
		// 平移下刀路径，防止路径断开, 2013.12.17 liuxin
		{
			tmpBase.GetEndPoint( 0, dPoint ) ;
			cTPlunge.GetEndPoint(1, dAtPoint);
			cTPlunge.MoveCombine(0.0, 0.0, dPoint[2] - dAtPoint[2]);
		}
		tmpComb.AppendCombine( cTPlunge ) ;
		Mini_DeleteContours( pChild ) ;	
	}	
	delete pBound ;
	tmpComb.AppendCombine( tmpBase ) ;	
	//liuyan、xiejunxian、qqs 2013.03.26
	//通过比较当前环与上一个环的面积，取面积较大的环作为判断是否生成连刀路径的环
	if( !nLp ) 
		m_cLastLoop = pContour; 
	else
	{
		if (pContour->m_dArea - m_cLastLoop->m_dArea > 0.001)
			m_cLastLoop = pContour;
		else
		{
			CSmartLoop* tmpLoop;
			tmpLoop = m_cLastLoop;
			m_cLastLoop = pContour;
			pContour = tmpLoop;
		}
	}
	
	if( PComb.m_pHead && 
		( m_cFeedDef.m_cLayerDef.m_bLayerFlag & NCDEF_LAYER_KEEPDOWN ) )	
	{
		TPNT3D  dStart, dEnd, dMid ;
		if( PComb.GetEndPoint( 1, dStart ) && 
			tmpComb.GetEndPoint( 0, dEnd )   )
		{
			BOOL bFindErr = FALSE ;
			double dAngle = 0., dMinLen = 0. ;
			//liuyan、xiejunxian、qqs 2013.03.26修改
			//修改减少抬刀功能失效问题
			dAngle = ANGLE_TO_RADIAN( m_cShapeDef.m_cTaper.m_dBoundAngle  ) ;
			dMinLen = m_pTool->GetRadiusComp(fabs(DepthInc), dAngle) -
				      m_pTool->GetRadiusComp(0.0, 0.0) ;
			BOOL bFlag = TRUE;//用于判断起点和末点投影是否重合
			if( nc_Distance( dStart, dEnd, 2) > 0.001) 
			{
				bFlag = FALSE;
				nc_GetPointAtLine( dStart, dEnd, 0.5, dMid, 2 ) ;
				if( fabs( dMinLen - nc_Distance( dStart, dEnd, 2) ) < 0.001 )
				{
				}
				else if( m_cParam.m_nRCompSide == 2 ) 
				{
					bFindErr = TRUE ;
				}
				else if( pContour->IntLineContour( dStart, dEnd ) )
				{
					bFindErr = TRUE ;
				}
				else if( !pContour->IsPtOnContour( dMid ) )
				{ // 自动向内偏移
					bFindErr = TRUE ;
				}
			}

			if( dMinLen < m_pTool->m_fRadius * 4 )
				dMinLen = m_pTool->m_fRadius * 4 ;
			// 选择多个轮廓进行加工的时候，防止距离过长，进行连接2012/11/05
			if( bFindErr == FALSE && nc_Distance( dStart, dEnd, 3 ) > 2.0e-4 &&
				dMinLen - nc_Distance( dStart, dEnd, 3 ) > -2.0e-4 )
			{
				TPNT3D dTemp;
				if ( bFlag ) // 起末点投影重合
				{
					if (fabs(dStart[2] - dEnd[2]) > 0.001) //Z值不同时，直接连接
					{
						CPathLine3D* pLine3D  = new CPathLine3D( dStart, dEnd ) ;
						pLine3D->m_bFeedType = JDNC_FEEDTYPE_CONNECT ;
						
						tmpComb.InsertAfter( pLine3D, NULL ) ;
					}
				}
				else // 起末点投影不重合
				{
					if (fabs(dStart[2] - dEnd[2]) < 0.001) //Z值相同时，直接连接
					{
						CPathLine3D* pLine3D  = new CPathLine3D( dStart, dEnd ) ;
						pLine3D->m_bFeedType = JDNC_FEEDTYPE_CONNECT ;
						
						tmpComb.InsertAfter( pLine3D, NULL ) ;
					}
					else // Z值不同时，3D折线连
					{
						if (dStart[2] > dEnd[2])
						{
							dTemp[0] = dEnd[0];
							dTemp[1] = dEnd[1];
							dTemp[2] = dStart[2];					
						}
						else if (dStart[2] < dEnd[2])
						{
							dTemp[0] = dStart[0];
							dTemp[1] = dStart[1];
							dTemp[2] = dEnd[2];
						}

						CPathPLine3D *pLine3D = new CPathPLine3D;
						pLine3D->m_nCount = 2;
						pLine3D->m_pTAPos = new TPNT3D[3];
						mathCpyPnt(dStart,pLine3D->m_pTAPos[0]);
						mathCpyPnt(dTemp,pLine3D->m_pTAPos[1]);
						mathCpyPnt(dEnd,pLine3D->m_pTAPos[2]);
						pLine3D->m_bFeedType = JDNC_FEEDTYPE_CONNECT ;
						
						tmpComb.InsertAfter( pLine3D, NULL ) ;
					}
				}
			}
		}
	}
	PComb.AppendCombine( tmpComb ) ;	
	return TRUE ; 
}

int CSmartContourGen::ContourOneLoopByContourPlunge( CPathCombine& PComb,    /*加工数据*/
													CSmartLoop *AllCont,    /*区域数组*/
													CSmartLoop& Loop   ,    /*轮廓曲线*/
													DOUBLE  Depth      ,    /*深度位置*/
													DOUBLE  DepthInc,       /*吃刀深度*/
													JDNC_LAYER &Layer ,		/*分层*/
													RFRAME &	LocFrame,	/*局部坐标系*/
													BOOL bFst		   ,	/*是否为第一个环*/
													BOOL	bLast,			/*是否为最后一层*/
													int nRev,              /*延伸圆弧标记*/
													int nLp  )            /*是否为该加工轮廓第一个环 qqs 2013.03.26*/ 
{
	CPathCombine  tmpComb( NC_WPROCESS_ROUGH ) ;
	CPathCombine tmpBase( NC_WPROCESS_ROUGH ) ;
	CSmartLoop * pContour = &Loop ;
	if( Loop.m_pParent ) pContour = Loop.m_pParent ;
	CSmartCurve * pBound = Loop.m_pCurve->CopyMyself() ;
	pBound->SetAllZValue( 0.0 ) ;
	JDNC_LEAD  leadIn  = m_cFeedDef.m_cLeadDef ;
	JDNC_LEAD  leadOut = m_cFeedDef.m_cLeadDef ;
	UpdateLeadOutParam(m_cParam.m_cLeadOut, leadOut);

	DOUBLE ZShift[2] = {} ;
	GetLeadCutShift(m_cParam.m_cLeadOut, ZShift);
	int nRCompMask = GetContourRCompMask() ;
	JDNC_LEAD  rcompLead = m_cFeedDef.m_cLeadDef ;
	rcompLead.m_dLength = max( 0.05, m_cFeedDef.m_cLeadDef.m_dWearLine) ;
	if( rcompLead.m_nLeadType == 0 ) rcompLead.m_nLeadType = NCDEF_LEAD_SLINE ;
	else   rcompLead.m_nLeadType = NCDEF_LEAD_LINE ;
	DOUBLE rcompZShift[2] = { 0.0, 0.0 } ;

	if( m_cSetupDef.m_cCorDef.m_nCorType == NCDEF_CORNER_EXTARC && nRev != 0)
	{//延伸圆弧
		ResetCurve( pBound, m_cProcessDef.m_dExtRadius, nRev ) ;	
	}
	if( GetMillDir() == 0 ) 
	{
		pBound->Reverse() ; 
	}
	pBound->SetFeedType( JDNC_FEEDTYPE_ROUGH ) ;
	// 判断是否保留曲线高度或者从曲线读取速度
	BOOL bKeep[2] = { FALSE, FALSE } ;
	if(  m_cParam.m_bLoopFlag & NCDEF_FLOOP_REDEPTH  )
	{ 
		bKeep[0] = TRUE ;
	}

	// STEP 1 :  计算下刀路径
	JDNC_PLUNGE  Plunge = m_cFeedDef.m_cPlungeDef ;	
	{
		PNT3D dPoint;		
		pBound->GetPoint( 0.0, dPoint ) ;
		dPoint[2]  = 0  ;
		Plunge.m_dIncStep =  DepthInc  + Plunge.m_dTopTol ;
		CPathCombine cTPlunge( NC_WPROCESS_PLUNGE ) ;
		AddPlungePathBy( cTPlunge, Plunge , dPoint, *pBound, TRUE ) ;			
		cTPlunge.MoveCombine( 0.0, 0.0, -Depth ) ;
		// 生成沿轮廓下刀成功，关闭沿轮廓进刀
		if (leadIn.m_nLeadType == NCDEF_LEAD_BYCONTOUR && cTPlunge.m_pHead)
		{
			leadIn.m_nLeadType = NCDEF_LEAD_CLOSE;
		}
		tmpBase.AppendCombine( cTPlunge ) ;
	}

	//  STEP 2 : 计算进刀路径
	// 生成路径并保留曲线高度、得到速度
	if (bLast && m_cParam.m_bLoopFlag & NCDEF_FLOOP_RECUTLAST)
	{// 最后一层重复加工， 2013.12.05,liuxin
		CSmartCurve *pCopyCurve = pBound->CopyMyself();
		for (int k = 0; k < m_cParam.m_nReCutTimes; ++k)
		{							
			pBound->AddCurve(pCopyCurve->CopyMyself());
		}
		tmpBase.AppendCurve3D( *pBound , -Depth, TRUE ) ;
		pCopyCurve->ClearAll();
		delete pCopyCurve;
	}
	else
	{
		tmpBase.AppendCurve3D( *pBound , -Depth, TRUE ) ;
	}

	if( bKeep[0] || bKeep[1] )
	{
		RedepthCPathCombine( m_cListCur, &tmpBase, bKeep, LocFrame);
	}

	AddLeadPathAtEndEx( leadIn, leadOut, tmpBase, *pContour, Depth, ZShift, DepthInc ) ;						
	if( nRCompMask != 0 ) 						
	{							
		int bFlag = AddLeadPathAtEndEx( rcompLead, rcompLead, tmpBase, *pContour, 0.0, rcompZShift) ;
		AddRCompMask( tmpBase, nRCompMask ) ;
		if (!bFlag)
		{
			m_bRComb = 2;
		}            
	}
	// 添加连接仅提到功能
	BOOL bCnt[2] = { FALSE, FALSE } ;
	if( bFst )
	{
		DOUBLE dComp[2] = { 0., 0. } ;
		GetBoundOffset( Layer.m_dTotalDepth, Depth, dComp ) ;
		//	bCnt = ConnectLeadInOutPos( tmpBase, AllCont, dComp, m_cParam.m_nRCompSide, m_cFeedDef.m_cLeadDef.m_dLeadPos ) ;
		ConnectLeadInOutPos( tmpBase, AllCont, dComp, m_cParam.m_nRCompSide, m_cParam.m_cLeadOut, bCnt ) ;
	}
	
	delete pBound ;
	tmpComb.AppendCombine( tmpBase ) ;	
	//liuyan、xiejunxian、qqs 2013.03.26
	//通过比较当前环与上一个环的面积，取面积较大的环作为判断是否生成连刀路径的环
	if( !nLp ) 
		m_cLastLoop = pContour; 
	else
	{
		if (pContour->m_dArea - m_cLastLoop->m_dArea > 0.001)
			m_cLastLoop = pContour;
		else
		{
			CSmartLoop* tmpLoop;
			tmpLoop = m_cLastLoop;
			m_cLastLoop = pContour;
			pContour = tmpLoop;
		}
	}

	if( PComb.m_pHead && 
		( m_cFeedDef.m_cLayerDef.m_bLayerFlag & NCDEF_LAYER_KEEPDOWN ) )	
	{
		TPNT3D  dStart, dEnd, dMid ;
		if( PComb.GetEndPoint( 1, dStart ) && 
			tmpComb.GetEndPoint( 0, dEnd )   )
		{
			BOOL bFindErr = FALSE ;
			double dAngle = 0., dMinLen = 0. ;
			//liuyan、xiejunxian、qqs 2013.03.26修改
			//修改减少抬刀功能失效问题
			dAngle = ANGLE_TO_RADIAN( m_cShapeDef.m_cTaper.m_dBoundAngle  ) ;
			dMinLen = m_pTool->GetRadiusComp(fabs(DepthInc), dAngle) -
				m_pTool->GetRadiusComp(0.0, 0.0) ;
			BOOL bFlag = TRUE;//用于判断起点和末点投影是否重合
			if( nc_Distance( dStart, dEnd, 2) > 0.001) 
			{
				bFlag = FALSE;
				nc_GetPointAtLine( dStart, dEnd, 0.5, dMid, 2 ) ;
				if( fabs( dMinLen - nc_Distance( dStart, dEnd, 2) ) < 0.001 )
				{
				}
				else if( m_cParam.m_nRCompSide == 2 ) 
				{
					bFindErr = TRUE ;
				}
				else if( pContour->IntLineContour( dStart, dEnd ) )
				{
					bFindErr = TRUE ;
				}
				else if( !pContour->IsPtOnContour( dMid ) )
				{ // 自动向内偏移
					bFindErr = TRUE ;
				}
			}

			if( dMinLen < m_pTool->m_fRadius * 4 )
				dMinLen = m_pTool->m_fRadius * 4 ;
			// 选择多个轮廓进行加工的时候，防止距离过长，进行连接2012/11/05
			if( bFindErr == FALSE && nc_Distance( dStart, dEnd, 3 ) > 2.0e-4 &&
				dMinLen - nc_Distance( dStart, dEnd, 3 ) > -2.0e-4 )
			{
				TPNT3D dTemp;
				if ( bFlag ) // 起末点投影重合
				{
					if (fabs(dStart[2] - dEnd[2]) > 0.001) //Z值不同时，直接连接
					{
						CPathLine3D* pLine3D  = new CPathLine3D( dStart, dEnd ) ;
						pLine3D->m_bFeedType = JDNC_FEEDTYPE_CONNECT ;
						
						tmpComb.InsertAfter( pLine3D, NULL ) ;
					}
				}
				else // 起末点投影不重合
				{
					if (fabs(dStart[2] - dEnd[2]) < 0.001) //Z值相同时，直接连接
					{
						CPathLine3D* pLine3D  = new CPathLine3D( dStart, dEnd ) ;
						pLine3D->m_bFeedType = JDNC_FEEDTYPE_CONNECT ;
						
						tmpComb.InsertAfter( pLine3D, NULL ) ;
					}
					else // Z值不同时，3D折线连
					{
						if (dStart[2] > dEnd[2])
						{
							dTemp[0] = dEnd[0];
							dTemp[1] = dEnd[1];
							dTemp[2] = dStart[2];					
						}
						else if (dStart[2] < dEnd[2])
						{
							dTemp[0] = dStart[0];
							dTemp[1] = dStart[1];
							dTemp[2] = dEnd[2];
						}

						CPathPLine3D *pLine3D = new CPathPLine3D;
						pLine3D->m_nCount = 2;
						pLine3D->m_pTAPos = new TPNT3D[3];
						mathCpyPnt(dStart,pLine3D->m_pTAPos[0]);
						mathCpyPnt(dTemp,pLine3D->m_pTAPos[1]);
						mathCpyPnt(dEnd,pLine3D->m_pTAPos[2]);
						pLine3D->m_bFeedType = JDNC_FEEDTYPE_CONNECT ;
						
						tmpComb.InsertAfter( pLine3D, NULL ) ;
					}
				}
			}
		}
	}
	PComb.AppendCombine( tmpComb ) ;	
	return TRUE ; 
}

int  CSmartContourGen::AppendCombineEx( CPathCombine& Base, CPathCombine& To , CSmartLoop* Contour ) 
{
	if( To.m_pHead == NULL ) return 1;
	if( Base.m_pHead == NULL )
	{
		Base.AppendCombine( To ) ;
		return 1 ; 
	}
	PNT3D dStart , dEnd, dMid ;
	Base.GetEndPoint( 1, dStart ) ;
	To.GetEndPoint( 0, dEnd ) ;
	BOOL bValid = TRUE ;
	if( nc_FabsDist( dStart , dEnd , 3 ) < 1.0e-3 )
	{
		bValid = FALSE ;
	}
	if( Contour )
	{
		if( Contour->IntLineContour( dStart, dEnd )  )
		{
			bValid = FALSE ;
		}
	}
	if( bValid )
	{
		CPathLine3D* pLine3D = NULL ; 
		if( dStart[2] - dEnd[2] > 1.0e-3 )
		{
			dMid[0] = dEnd[0], dMid[1] = dEnd[1] , dMid[2] = dStart[2] ;
			pLine3D = new CPathLine3D(dStart, dMid) ;
			pLine3D->m_bFeedType = JDNC_FEEDTYPE_CONNECT ;
			Base.AddEntity( pLine3D ) ;
			pLine3D = new CPathLine3D(dMid , dEnd) ;
			pLine3D->m_bFeedType = JDNC_FEEDTYPE_PLUNGE ;
			Base.AddEntity( pLine3D ) ;
		}
		else
		{
			pLine3D = new CPathLine3D(dStart, dEnd) ;
			pLine3D->m_bFeedType = JDNC_FEEDTYPE_CONNECT ;
			Base.AddEntity( pLine3D ) ;
		}
	}
	Base.AppendCombine( To ) ;
	return 1 ;
}

void MathCAM_SetContoursStart( CSmartLoop* AllCont, JDNC_LEAD& LeadDef,
                              CSmartGraphic& Graph, JDNC_SETUP& Setup )
{
    if( LeadDef.m_nPositionType == NCDEF_LEADPOS_START )
    {
        return  ;
    }
    CPtrList  AllPoint ;
    if( LeadDef.m_nPositionType == NCDEF_LEADPOS_AUTO  )
    {
        Graph.GetAllEntity( Setup, AllPoint, SMARTGRAPH_TYPE_POINT ) ;
        if( AllPoint.GetCount() == 0 )
        {
            return ;
        }
    }
    CSmtLoopArr LoopArr ;
    for( CSmartLoop* pLoop = AllCont ; pLoop ; pLoop = pLoop->next )
    {
        pLoop->ExtractAllLoop( NCDEF_SORTTYPE_TOINNER , LoopArr ) ; 
    }
    INT_PTR nSize = LoopArr.GetSize() ;
    PNT2D dSeedPnt , dStart ; 
    for( INT_PTR i = 0 ; i < nSize ; i ++ ) 
    {
        CSmartSect* pObjSect = NULL ;
        DOUBLE dDist , dMinDist = 1.0e12;
        pLoop = LoopArr.GetAt( i ) ;
        if( pLoop->m_pCurve == NULL )
        {
            continue ;
        }
        if( LeadDef.m_nPositionType == NCDEF_LEADPOS_AUTO  )
        {
            POSITION atPos = AllPoint.GetHeadPosition() ;
            while( atPos ) 
            {
                CSmartPoint* pPoint = (CSmartPoint*) AllPoint.GetNext(atPos ) ;
                CSmartSect* pSect = pLoop->m_pCurve->MinDistSect( pPoint->m_dPoint, dSeedPnt, dDist ) ;
                if( ! pObjSect || dDist < dMinDist )
                {
                    pObjSect = pSect , dMinDist = dDist ;
                    nc_VectorCopy( dStart, dSeedPnt,2 ) ;
                }
            }
        }
		else if( LeadDef.m_nPositionType == NCDEF_LEADPOS_REFER )
		{
			dSeedPnt[0] =  LeadDef.m_dLeadPos[0] ;
			dSeedPnt[1] =  LeadDef.m_dLeadPos[1] ;
			pObjSect = pLoop->m_pCurve->MinDistSect( dSeedPnt, dStart, dDist ) ;
		}
        else 
        {
            if( LeadDef.m_nPositionType == NCDEF_LEADPOS_LEFTDOWN )
            {
                dSeedPnt[0] =  pLoop->m_dBox[0][0] ;
                dSeedPnt[1] =  pLoop->m_dBox[0][1] ;
            }
            else if( LeadDef.m_nPositionType == NCDEF_LEADPOS_LEFTUP ) 
            {
                dSeedPnt[0] =  pLoop->m_dBox[0][0] ;
                dSeedPnt[1] =  pLoop->m_dBox[1][1] ;
            }
            else if( LeadDef.m_nPositionType == NCDEF_LEADPOS_RIGHTDOWN )
            {
                dSeedPnt[0] =  pLoop->m_dBox[1][0] ;
                dSeedPnt[1] =  pLoop->m_dBox[0][1] ;
            }
            else
            {
                dSeedPnt[0] =  pLoop->m_dBox[1][0] ;
                dSeedPnt[1] =  pLoop->m_dBox[1][1] ;
            }
            pObjSect = pLoop->m_pCurve->MinDistSect( dSeedPnt, dStart, dDist ) ;
        }
        pLoop->m_pCurve->SetStartPoint( pObjSect, dStart ) ;
    }
    while( AllPoint.GetCount() ) 
    {
        CSmartPoint* pPoint = (CSmartPoint*)AllPoint.RemoveHead() ;
        delete pPoint ;
    }
}

void SurfNC_InitPathParam( JDNC_BOUND&  Bound  ) ;
BOOL CSmartContourGen::BoundContour( CPathGroup& NewPath,  /*雕刻路径*/ 
                                     CSmartGraphic& Graph)  /*图形函数*/  
{
	// STEP 1 : 设置路径参数
	CSmartBoundGen  BoundGen ;
    *(CSmartPathGen*)&BoundGen = *this  ;
	SurfNC_InitPathParam( BoundGen.m_cParam ) ;
	BoundGen.m_cParam.m_nSTolTime = m_cParam.m_nSTolTime;
	BoundGen.m_cParam.m_dSideTol  = m_cParam.m_dSideTol ;
	if( m_cParam.m_nRCompSide == 0 )
	{// 向外切割
		BoundGen.m_cParam.m_bBoundFlag  |= NCDEF_FBOUND_LOOPOUT ;
	}
	if (m_cParam.m_bLoopFlag & NCDEF_FLOOP_REDEPTH)
	{
		BoundGen.m_cParam.m_bBoundFlag  |= NCDEF_FBOUND_REDEPTH;
	}

//  封掉原因：当用户选择多个加工域时，区域之间的不抬刀路径可能是过切的。（遵照孟工建议修改,jbp）
// 	if (m_cFeedDef.m_cLayerDef.m_bLayerFlag & NCDEF_LAYER_KEEPDOWN)
// 	{
// 		BoundGen.m_cParam.m_bBoundFlag  |= NCDEF_FBOUND_MINUP;
// 	}
	BoundGen.m_cParam.m_bBoundFlag  |= NCDEF_FBOUND_LOOPMODE ;
	BoundGen.m_cParam.m_bBoundFlag  &= ~NCDEF_FBOUND_NOOUTER ;
    if( m_cParam.m_bLoopFlag & NCDEF_FLOOP_USERRCOMP )
    {
		BoundGen.m_cStockDef.m_dSideStock += (m_cParam.m_dRCompDist - 0.1) ;
        BoundGen.m_cToolDef.m_nToolType = surfncToolFlat ;
        BoundGen.m_cToolDef.m_dTopDiam  = 0.2 ;
    }
    BoundGen.m_cParam.m_cLeadOut = m_cParam.m_cLeadOut ;
	BoundGen.m_bLoopCut = TRUE ;
    // STEP 2 : 计算修边路径
    int bRetFlag = BoundGen.GeneratePathEx(NewPath, Graph) ;

	// 注释原因：连接及慢下路径通过CSmartPathGen::AddPlungeAndRelativeH生成，此处不需生成 qqs 2014.07.02
	// step 3 : 添加连接路径
	//AddConnectForNewPath( NewPath, Graph ) ;

	m_nErrorType = BoundGen.m_nErrorType ;	
    return bRetFlag ;
}

BOOL CSmartContourGen::GetBoundOffset ( DOUBLE TotalDepth,		// <I> 总深度
										DOUBLE dDepth,			// <I> 当前深度
										DOUBLE dRComp[2] )		// <I> 偏移补偿 )
{
	dRComp[0] = dRComp[1] = 0. ;
	DOUBLE dAngle[2]; 
	dAngle[0] = ANGLE_TO_RADIAN( m_cShapeDef.m_cTaper.m_dBoundAngle  ) ;
	dAngle[1] = ANGLE_TO_RADIAN( m_cShapeDef.m_cTaper.m_dIslandAngle ) ;
	if( m_cParam.m_nRCompSide ==  2 )
	{ /* 关闭补偿 */
		dRComp[0] = dRComp[1] = 0.0 ;
	}
	else if(m_cParam.m_bLoopFlag & NCDEF_FLOOP_USERRCOMP ) 
	{ /* 定义补偿值 */
		dRComp[0] = dRComp[1] = m_cParam.m_dRCompDist; 
	}
	else if( m_cShapeDef.m_bAntiFace && dAngle[0] > 0.001 )
	{/* 底面效果 */
		dRComp[0] = m_pTool->GetRadiusComp( 0.0, 0.0) ; 
		dRComp[1] = m_pTool->GetRadiusComp( 0.0, 0.0) ; 
	}
	else if( m_cFeedDef.m_cLayerDef.m_bLayerFlag & NCDEF_LAYER_COPYMODE )
	{ /*  拷贝分层  */
		dRComp[0] = m_pTool->GetRadiusComp( TotalDepth, dAngle[0]) ; 
		dRComp[1] = m_pTool->GetRadiusComp( TotalDepth, dAngle[1]) ; 
	}
	else 
	{
		dRComp[0] = m_pTool->GetRadiusComp( dDepth, dAngle[0]) ; 
		dRComp[1] = m_pTool->GetRadiusComp( dDepth, dAngle[1]) ; 
	}
	dRComp[0] += GetSideStock() ;
	dRComp[1] += GetSideStock() ;
	
	return TRUE ;
}

BOOL CSmartContourGen::GeneratePathEx( CPathGroup& NewPath,  /*雕刻路径*/ 
                                       CSmartGraphic& Graph)  /*图形函数*/   
{
	// STEP -1 : 获取保留曲线高度和速度的原始曲线
	if( m_cParam.m_bLoopFlag & NCDEF_FLOOP_REDEPTH ) 
	{
		Graph.GetAllEntity( m_cSetupDef, m_cListCur, SMARTGRAPH_TYPE_CURVE2D ) ;
	}
	// STEP 0 : 侧向分层时
	if (m_cParam.m_nRCompSide != 2 && m_cSlayerDef.m_nLayerType != NCDEF_SLAYER_CLOSE &&
		!( m_cParam.m_bLoopFlag & NCDEF_FLOOP_GRNDMDFY )	)
	{
		return BoundContour(NewPath, Graph);
	}

	// STEP 1 : 校正雕刻环境
	if( !UpdateTool2D() ) return FALSE ;
	BOOL bRedepth = FALSE ;
	if( m_cParam.m_bLoopFlag & NCDEF_FLOOP_REDEPTH )
	{
		bRedepth = TRUE ; 
	}
	CSmartLoop* AllCont = Graph.GetAllContour( m_cSetupDef , bRedepth) ;
	if(  !AllCont ) 
	{ // 校正图形
		m_nErrorType = JDERROR_GENPATH_NOLOOP ;
		return FALSE ;
	}
    MathCAM_SetContoursStart( AllCont, m_cFeedDef.m_cLeadDef,Graph, m_cSetupDef ) ;
	JDNC_LAYER * pLayer   = GetLayerDef() ;
	pLayer->m_dTotalDepth = GetCutDepth()  - m_cStockDef.m_dDepthStock; 
	if(m_cParam.m_bLoopFlag & NCDEF_FLOOP_GRNDMDFY )//磨削调整时关闭分层
	{
		pLayer->m_nLayerType = NCDEF_LAYER_CLOSE ;
	}
	DefineLayerDepth( * pLayer) ; 
	if( (m_cParam.m_bLoopFlag & NCDEF_FLOOP_REDEPTH ) &&
		 m_pTool->GetType() == smtToolBall &&
 		(m_cParam.m_nRCompSide != 2 ) && 
		!(m_cParam.m_bLoopFlag & NCDEF_FLOOP_USERRCOMP) )
	{
	    double dAngle = ANGLE_TO_RADIAN( m_cShapeDef.m_cTaper.m_dBoundAngle  ) ;
		double dZMove = m_pTool->m_fRadius * ( 1.0 - sin( dAngle ) );
	    for( int k = 1 ; k <= pLayer->m_nLayerCount ;  k ++ ) 
		{
			pLayer->m_dLayerDepth[k] += dZMove ;
		}
		pLayer->m_dTotalDepth += dZMove ;
	}
	CSmartLoop LoopTmp, *pLoop, *pContour;
	if( m_cParam.m_nRCompSide == 0 )
	{ // 向外切割
	    PNT3D  dBox[2] ; 
		nc_CleanBox3D( dBox ) ;
		for( pContour = AllCont ; pContour ; pContour = pContour->next )
		{
			nc_ClipBox3D(  dBox, pContour->m_dBox ) ;
		}
		dBox[0][0] -= 1.0e6, dBox[0][1] -= 1.0e6 ;
		dBox[1][0] += 1.0e6, dBox[1][1] += 1.0e6 ;
		pContour = new CSmartLoop() ;
        pContour->CreateLoop( dBox[0] , dBox[1] ) ;
        if( pContour->m_pCurve ) 
        {
            pContour->m_pCurve->SetSelectID( -1 ) ;
        }
		AllCont = Mini_AddContours( AllCont, pContour ) ;
		for( pContour = AllCont ; pContour ; pContour = pContour->next )
		{
			if( pContour->m_pIsland == NULL ) continue ;
			pLoop = pContour->m_pIsland ;
			pContour->m_pIsland = NULL  ;
			AllCont = Mini_AddContours( AllCont, pLoop ) ;
			while( pLoop ) 
			{
				pLoop->m_pParent = NULL  ; 
				pLoop = pLoop->next ;
			}
		}
		AllCont = LoopTmp.BuildContour( AllCont ) ;
	    AllCont = LoopTmp.ResortContour( AllCont, m_cSetupDef.m_cOrderDef.m_nSortType , FALSE) ;
	}
	// STEP 5 : 取得局部坐标系
	RFRAME LocFrame ;
	if( !Graph.GetTPlaneFrame( LocFrame ) )
	{
		mathInitRFrame( &LocFrame ) ;
	}

	if (m_c5DCtrlDef.m_cToolAxisApp.m_nFrameFlag != NCDEF_FRAME_DEFAULT)
	{
		mathMultRFrame( &LocFrame, &m_c5DCtrlDef.m_cToolAxisApp.m_dIndexFrame, &LocFrame ) ;
	}
	// STEP 1 : 计算偏移值， 优化计算速度
	DOUBLE dDepth , dDepthInc ;
	CSmartLoop* pLpHead , *pChildHead = NULL ;
	// STEP 2 : 分类计算刀具路径
	BOOL bFst = TRUE, bLast = FALSE;
	CPathCombine* pTComb = NULL ; 
	if( GetLayerOrder() == 1 && m_cFeedDef.m_cLayerDef.m_nLayerCount != 1)// 只有一层路径时按照区域优先方式计算， 2013.8.21，liuxin
	{ // 高度优先
		for( int i = 1 ; i <= pLayer->m_nLayerCount ;  i ++ ) 
		{
			dDepth = pLayer->m_dLayerDepth[i] ; 
			dDepthInc = dDepth - pLayer->m_dLayerDepth[i-1] ;
			for( pContour = AllCont ; pContour ; pContour = pContour->next )
			{
				CPathCombine* pTComb = new CPathCombine( NC_WPROCESS_ROUGH ) ;
				int nR = 0 ;
				pLpHead = CreateLoopBound( *pContour, pLayer->m_dTotalDepth, dDepth, &nR ) ;

				for( pLoop = pLpHead ; pLoop ; pLoop = pLoop->next )
				{ 
					if (i == pLayer->m_nLayerCount)
					{// 最后一层重复加工， 2013.9.16,liuxin
						bLast = TRUE;
					}
					LoopOneLayer( *pTComb , AllCont, *pLoop, dDepth, dDepthInc, 
									*pLayer, LocFrame, bFst, bLast, nR ) ;
					bFst = FALSE ;
				}  
				Mini_DeleteContours( pLpHead ) ;
				if( pTComb->m_pHead )	
				{
					if (m_cFeedDef.m_cPlungeDef.m_nPlungeType == NCDEF_PLUNGE_CONTOUR)
						NewPath.AddData(0.0, pTComb ) ;
					else
						NewPath.AddData(dDepth, pTComb ) ;
				}
				else
				{
					delete pTComb ;
				}
			}
		}
	}
	else 
	{ // 区域优先 
        if( IsSameOffset() && 
			(m_cFeedDef.m_cPlungeDef.m_nPlungeType == NCDEF_PLUNGE_CONTOUR) )
		{//偏移值相同 && 沿轮廓下刀 , 使用拷贝切割  
			dDepth = 0.0 ;
            CPathCombine* pTComb = new CPathCombine( NC_WPROCESS_ROUGH ) ;
			for( pContour = AllCont ; pContour ; pContour = pContour->next )
			{
				int nR = 0 ;
				pLpHead = CreateLoopBound( *pContour, pLayer->m_dTotalDepth, dDepth, &nR ) ;
				for( pLoop = pLpHead ; pLoop ; pLoop = pLoop->next )
				{ 
		            CopyOneContour( *pTComb ,  AllCont , *pLoop , *pLayer , LocFrame,  
					                  m_cFeedDef.m_cPlungeDef, bFst, nR ) ;
					bFst = FALSE ;
				}  
				Mini_DeleteContours( pLpHead ) ;
			}

			if( pTComb->m_pHead )	NewPath.AddData( 0.0, pTComb ) ;
			else delete pTComb ;
		}
		else
		{
			CSmtLoopArr LoopArr ;
			int nR = 0; 
			for( pContour = AllCont ; pContour ; pContour = pContour->next )
			{
				for( int i = 1 ; i <= pLayer->m_nLayerCount ;  i ++ ) 
				{
					dDepth = pLayer->m_dLayerDepth[i] ; 
					pLpHead = CreateLoopBound( *pContour, pLayer->m_dTotalDepth, dDepth, &nR ) ;
					for( pLoop = pLpHead ; pLoop ; pLoop = pLoop->next )
					{
						pLoop->m_nDepth = i , pLoop->m_pParent = NULL ;
						CSmartLoop *pIsland ; 
						for( pIsland = pLoop->GetIsland() ; pIsland ; pIsland = pIsland->next )
						{
							pIsland->m_nDepth = i ;
						}
						CSmtLoopArr tmpArr ;
						pLoop->ExtractAllLoop( GetSortType() , tmpArr ) ;
						INT_PTR nCount = tmpArr.GetSize() ;
						for( INT_PTR k = 0 ; k < nCount ; k ++ ) 
						{
							pIsland = tmpArr.GetAt( k ) ;
							if( fabs( pIsland->m_dArea ) < 1.0e10 && 
								(!(pIsland->m_bUseFlag & NC_LOOP_OUTER)) && 
								pIsland->m_pCurve != NULL  )
							{ /*无效轮廓曲线*/
								LoopArr.Add( pIsland ) ;
							}
						}
					}
					pChildHead = Mini_AddContours( pChildHead, pLpHead ) ;
				}
			}
			BOOL bMoveUp = FALSE ;
		    if( pLayer->m_nLayerCount > 1 )
			{
				double dRadius = m_pTool->m_fRadius ;
				if( m_pTool->GetType() == surfncToolAFlat )
				{
					dRadius = m_pTool->m_fBottom ;
				}
				SortLoopArray( LoopArr, dRadius ) ;
			} 
			if( m_cParam.m_bLoopFlag & NCDEF_FLOOP_MOVEUP  ) 
			{
				bMoveUp = TRUE ;
			}
			INT_PTR nCount = LoopArr.GetSize() ; 
			CSmtLoopArr   curList ;
			int nLastDepth = 0 ;
			for( INT_PTR nAt = 0 ; nAt < nCount ;  nAt ++ ) 
			{
				pLoop = LoopArr.GetAt( nAt ) ;
				if( ( pLoop->m_nDepth == nLastDepth + 1 ) || 
					curList.GetSize() == 0 )
				{
					if( bMoveUp )  curList.InsertAt( 0 , pLoop ) ;
					else curList.Add( pLoop ) ;
					if( nAt <  nCount - 1  )
					{
						nLastDepth = pLoop->m_nDepth ;
					    continue ;
					} 
				}
				else 
				{
					nAt -- ;
					nLastDepth = (int)nAt;
				}
				if( pTComb ) NewPath.AddData( 0.0, pTComb ) ;
				pTComb = new CPathCombine( NC_WPROCESS_ROUGH ) ;
				for( int nLp = 0 ; nLp < curList.GetSize() ; nLp ++ ) 
				{
					// xiejunxian、qqs、liuyan 2013.03.26
					//开启减少抬刀后，将除第一层下刀外的下刀表面预留置为0
					double dTopTol = m_cFeedDef.m_cPlungeDef.m_dTopTol;
					if( nLp != 0 && (m_cFeedDef.m_cLayerDef.m_bLayerFlag & NCDEF_LAYER_KEEPDOWN ) )
					{
						m_cFeedDef.m_cPlungeDef.m_dTopTol = 0.0 ;
					}
				    pLoop = curList.GetAt( nLp ) ;
					if( nLp != 0  &&  pLoop->m_pCurve )
					{ /*重设轮廓起点, 减少定位路径*/
						PNT2D dSeed[2];
						DOUBLE dDist ;
						CSmartLoop* pLp = curList.GetAt( nLp-1 ) ;
						pLp->m_pCurve->GetPoint( 1.0, dSeed[0] ) ;
						CSmartSect* pSect = pLoop->m_pCurve->FindNearSect( dSeed[0],dSeed[1], dDist ) ;
						if( pSect && pSect != pLoop->GetSectHead() )
						{
							pLoop->m_pCurve->SetStartSect( pSect ) ;
						}
					}
					dDepth = pLayer->m_dLayerDepth[pLoop->m_nDepth] ; 
				    dDepthInc = dDepth - pLayer->m_dLayerDepth[pLoop->m_nDepth-1] ;
					if (nLp == nLastDepth)
					{
						bLast = TRUE;
					}
					else
					{
						bLast = FALSE;
					}
				    ContourOneLoop( *pTComb , AllCont, *pLoop, dDepth, dDepthInc, *pLayer, LocFrame, bFst, bLast, nR , nLp);
					m_cFeedDef.m_cPlungeDef.m_dTopTol = dTopTol;
					bFst = FALSE ;
				}
				nLastDepth = 0 ;
				curList.RemoveAll() ;
			}
			if( pTComb ) NewPath.AddData( 0.0, pTComb ) ;
		}
	}
	Mini_DeleteContours( AllCont ) ;
	// STEP 4 : 计算加工路径
	if( ! NewPath.m_pHead )
	{
		m_nErrorType = JDERROR_GENPATH_NOAREA ;
		return FALSE ; 
	}

	if( ! ( m_cParam.m_bLoopFlag & NCDEF_FLOOP_REDEPTH ) )
	{// 校正表面高度
		NewPath.ZMoveGroup( GetTopHeight() ) ;
	}
	if( pLayer->m_dLayerDepth ) delete[] pLayer->m_dLayerDepth ;
	pLayer->m_dLayerDepth = NULL ;
	Mini_DeleteContours( pChildHead ) ;
	//if( m_cParam.m_bLoopFlag & NCDEF_FLOOP_GRNDMDFY )//磨削调整
	//{
	//	//ModifyGrind(NewPath, m_cParam.m_nMdfyType, m_cParam.m_dMdfyHeit, m_cParam.m_dMdfyDist);
	//	ModifyGrindNew(NewPath, m_cParam.m_nMdfyType, m_cParam.m_dMdfyHeit, m_cParam.m_dMdfyDist,m_cParam.m_nMdfySegs);
	//}
	return TRUE ;
}

int CSmartContourGen:: AddConnectForNewPath(	CPathGroup &NewPath,		// 路径
												CSmartGraphic &Graph )		// 图形
{
	if( !NewPath.m_pHead ) return 0 ;

	// 计算最高点
	double dTop = GetTopHeight() + m_cSpeedDef.m_dRapidHeight ;
	int nEntType = SMARTGRAPH_TYPE_CONTOUR ;

	if( m_cParam.m_bLoopFlag & NCDEF_FLOOP_REDEPTH )
	{
		nEntType = SMARTGRAPH_TYPE_CONTOUR3D ;
	}
	DOUBLE box[2][3] ;
	// 保留曲线高度有可能加工域比路径高，取最高点
	if( Graph.GetEntBox( box, nEntType ) )
	{
		if (dTop < (box[1][2] + m_cSpeedDef.m_dRapidHeight))
			dTop = box[1][2] + m_cSpeedDef.m_dRapidHeight ;
	}
	PNT3D start, end ;
	double dTol = 1.0e-6, dTol2D = min(0.0002, m_cSetupDef.m_cTolDef.m_dArcTol), dDist = 0., dDist2D = 0. ;
	CPathCombine *PComb = NewPath.m_pHead ;
	CPathEntity *pHead = NULL, *pNext = NULL ;
	CPathPLine3D *pLine = NULL ;
	for( ; PComb ; PComb = PComb->next )
	{
		pHead = PComb->m_pHead ;
		pHead->GetEndPoint ( 1, start ) ;
		while( pHead )
		{
			pNext = pHead->next ;
			if( !pNext ) break ;
			pNext->GetEndPoint ( 0, end   ) ;

			dDist = mathDist( start, end ), dDist2D = mathDist2D( start, end ) ;
			if( dDist > dTol )
			{
				// 平面距离小于精度的直接连接，2013.9.30，liuxin
				if( /*fabs( start[2] - end[2] )*/dDist2D < dTol2D )
				{
					// 竖直线，两个点,同一平面，直接连接
					pLine = new CPathPLine3D () ;
					pLine->m_nCount = 1 ;
					pLine->m_pTAPos = new TPNT3D[2] ;
					mathCpyPnt( start, pLine->m_pTAPos[0] ) ;
					mathCpyPnt( end  , pLine->m_pTAPos[1] ) ;
					//pLine->m_pTAPos[0][2] = pLine->m_pTAPos[1][2] = dTop ; // 防止路径突然跳起
				}
				else
				{
					// 非竖直线四个点
					pLine = new CPathPLine3D() ;
					pLine->m_nCount = 3 ;
					pLine->m_pTAPos = new TPNT3D[4] ;
					mathCpyPnt( start, pLine->m_pTAPos[0] ) ;
					mathCpyPnt( start, pLine->m_pTAPos[1] ) ;
					mathCpyPnt( end  , pLine->m_pTAPos[2] ) ;
					mathCpyPnt( end  , pLine->m_pTAPos[3] ) ;
					pLine->m_pTAPos[1][2] = pLine->m_pTAPos[2][2] = dTop ;
				}

				if( pLine )
				{
					pLine->m_bFeedType = JDNC_FEEDTYPE_QUICK ;
					PComb->InsertAfter ( pLine, pHead ) ;
				}
			}
			pHead = pNext ;
			pHead->GetEndPoint( 1, start ) ;
		}
	}

	return 1 ;
}

CSmartRemainGen::CSmartRemainGen()
{
}
CSmartRemainGen::~CSmartRemainGen()
{
}

BOOL MathCAM_DefineRemainTool( JDNC_REMAIN& Remain , JDNC_TOOLEX& AtTool )
{
	if( Remain.m_nRemainType != 2  ||
		Remain.m_dLastDiam < 0.01  ) return FALSE ;
	double dDiameter = Remain.m_dLastDiam  ; 
	double dScale     ;
	SurfNC_CopyToolInforFromNew(Remain.m_cLastTool, AtTool);
	switch( Remain.m_cLastTool.m_nToolType )
	{
		case surfncToolFlat :
		case surfncToolBall :
		case surfncToolDrill:
			Remain.m_cLastTool.m_dTopDiam = dDiameter ;
			break ;
		case surfncToolAFlat :
		case surfncToolAFlat2:
		case surfncToolANose :
		case surfncToolNose  :
			dScale = dDiameter /Remain.m_cLastTool.m_dBtmDiam ;
			Remain.m_cLastTool.m_dBtmDiam *= dScale ;
			Remain.m_cLastTool.m_dTopDiam *= dScale ;
			break ;
		case surfncToolABall :
			dScale = dDiameter / Remain.m_cLastTool.m_dCorRadi ;
			Remain.m_cLastTool.m_dCorRadi *= dScale ;
			Remain.m_cLastTool.m_dTopDiam *= dScale ;
			break ;
	}
	return TRUE ;
}

/////////////////////////////
// 计算一层的残料补加工路径
//
int CSmartRemainGen::RemainOneLayer(  CPathCombine& PComb   ,  /*路径集合*/
						              CSmartLoop&    Contour,  /*边界轮廓*/
									  JDNC_REMAIN& /*RemainCut*/,  /*加工参数*/
			                          DOUBLE DepthInc      )   /*雕刻深度*/  
{
    // STEP 1 : 分析盲区
    if( m_cFeedDef.m_cPlungeDef.m_bPlungeFlag & NCDEF_PLUNGE_DELBLINDAREA )
    {// 消除加工中的盲区
        double dMinDiam = m_cFeedDef.m_cPlungeDef.m_dBlindRadius * 2.0 ;
        if( dMinDiam  > 0.05 && 
            Contour.m_dBox[1][0] - Contour.m_dBox[0][0] < dMinDiam && 
            Contour.m_dBox[1][1] - Contour.m_dBox[0][1] < dMinDiam   )
        {/*半径过滤*/
            for( CSmartSect* pSect = Contour.GetSectHead() ; pSect ; pSect = pSect->next )
            {
                 if( pSect->m_bEndFlag & NC_BLANK_SECT ) break ;
            }
            if( pSect == NULL )
            {/*没有发现毛坯边界, 过滤成功*/
                return 1 ;
            }
        }
    }
    if( m_cFeedDef.m_cPlungeDef.m_nPlungeType != NCDEF_PLUNGE_CLOSE )
	{
		m_cFeedDef.m_cPlungeDef.m_dIncStep = DepthInc + m_cFeedDef.m_cPlungeDef.m_dTopTol ;
    }
    // STEP 2 : 计算开槽路径
    BOOL bEnSlot = FALSE ;
    CPathCombine  TComb( NC_WPROCESS_ROUGH ) ;
	if( m_cFeedDef.m_cSlotDef.m_nSlotType != NCDEF_TSLOT_CLOSE ) 
	{ // 计算开槽路径
        TComb.m_nType = NC_WPROCESS_SLOT ;
		m_cFeedDef.m_cSlotDef.m_dDepthInc =  DepthInc ;
		DefineSlotDepth( m_cFeedDef.m_cSlotDef ) ;
		CreateECutPath( TComb    , 
			            Contour   , 
		                m_cFeedDef.m_cSlotDef , 
					    m_cFeedDef.m_cPlungeDef ) ;
        bEnSlot = TRUE ;
	}
    // STEP 3 : 计算环切路径
	JDNC_FOLLOW  FollowCut ;
	SurfNC_InitPathParam( FollowCut ) ;
	FollowCut.m_dOverStep  = m_cFeedDef.m_cStepDef.m_dOverStep ;
	FollowCut.m_dOverRatio = 50.0f ;
	FollowCut.m_bFollowFlag= NCDEF_FOLLOW_SMOOTH | NCDEF_FOLLOW_CORNER;
	if( bEnSlot )
    {
        FollowCut.m_bFollowFlag |= NCDEF_FOLLOW_SLOTCUT ;
    }
	else
    {
        FollowCut.m_bFollowFlag &= ~NCDEF_FOLLOW_SLOTCUT ;
        FollowCut.m_bFollowFlag |= NCDEF_FOLLOW_TOOUTER ;
    }
	// 防止环切时尖角处出现过切，2014.6.30 liuxin
	JDNC_SETUP cSetupDef = m_cSetupDef;
	if (FollowCut.m_bFollowFlag & NCDEF_FOLLOW_SMOOTH)
	{
		cSetupDef.m_cCorDef.m_nCorType = NCDEF_CORNER_ARCUSER;
	}
    CSmartNcFollow FollowGen( m_pTool, FollowCut, m_cFeedDef, /*m_cSetupDef */ cSetupDef ) ;
    FollowGen.CreatePocketPath( Contour,TComb ) ;
    // STEP 4 : 增加下刀路径
    DOUBLE dMaxDist[2] = { max(m_pTool->m_fRadius*2.0, FollowCut.m_dOverStep * 1.5 ),0.2} ;
	JDNC_PLUNGE  Plunge = m_cFeedDef.m_cPlungeDef ;
    if( (m_cFeedDef.m_cSlotDef.m_nSlotType == NCDEF_TSLOT_CLOSE) && 
	    ( Plunge.m_nPlungeType != NCDEF_PLUNGE_CLOSE) )
	{
		TComb.m_nType = NC_WPROCESS_PLUNGE ;
        if( !(FollowCut.m_bFollowFlag & NCDEF_FOLLOW_TOOUTER) )
        {
            m_bFollowOuter = 0 ;
        }
		int nFlag[2] = { surfncMoveFollow, m_cSetupDef.m_cOrderDef.m_nMillDir } ;
		InsertPlungePath( TComb, Contour, Plunge, NULL , NULL,dMaxDist, nFlag ) ;
        m_bFollowOuter = 1 ;
	}
	PComb.AppendCombine( TComb ) ;
	return TRUE  ;
}
CSmartLoop* CSmartRemainGen::GetRemainContour( CSmartLoop& Contour  , 
											 JDNC_REMAIN& RemainCut ,
											 DOUBLE Depth           , 
											 DOUBLE /*DepthInc*/        )
{
	// STEP 1 : 偏移原始曲线
	CSmartLoop* pLpHead = OffsetPathBound( Contour, Depth, 0.0, FALSE ) ;
	if( ! pLpHead ) return NULL  ;
	for( CSmartLoop* pLoop = pLpHead ; pLoop ; pLoop = pLoop->next )
	{
		pLoop->SetBoundInfo( TRUE ) ;
	}
    DOUBLE dRadRgh[2] , dMyDepth = Depth ;
    for( int i = 1 ; i <= RemainCut.m_cLastLayer.m_nLayerCount ; i ++ ) 
	{
		dMyDepth = RemainCut.m_cLastLayer.m_dLayerDepth[i] ;
		if( Depth <= dMyDepth ) break ;
	}
	if( dMyDepth - Depth < -0.003 ) return pLpHead ; 
	// STEP 2 :  偏移原始曲线
	CSmartTool * pLastTool = CreateSmartTool(RemainCut.m_cLastTool, 0.0, 0. )  ; 
	if( ! pLastTool ) return pLpHead ; 
	CSmartLoop *pCutHead = OffsetPathBound( Contour, dMyDepth, pLastTool, 
		                                    &RemainCut.m_cLastLayer,
											RemainCut.m_cLastShape.m_cStock0.m_dSideStock ) ;
	DOUBLE dOffDist = dMyDepth - Depth ;
	if( dOffDist < 0.0 ) dOffDist = 0.0 ;
    dRadRgh[0] = dRadRgh[1] = pLastTool->GetRadiusComp( dOffDist, 0.0 ) ;
	// STEP 3 : 计算剪切曲线
	if( dRadRgh[0] > 1.0e-3 && pCutHead)
	{ /*获得加工边界*/
		JDNC_COR  CorDef = GetCorDef() ;
		CorDef.m_nCorType = NCDEF_CORNER_ARCALL ;
		CSmartLoop* pTmpHead = Mini_OffsetContourList( pCutHead        , 
													   NCDEF_OFFSET_OUTER , 
													   dRadRgh         , 
													   CorDef ) ;
		Mini_DeleteContours( pCutHead ) ;
		pCutHead  = pTmpHead  ;
	}
    dRadRgh[0] = dRadRgh[1] = m_pTool->GetRadiusComp( 0.0, 0.0 ) -0.001;
	if( dRadRgh[0] > 0.0 && pCutHead) 
	{
		CSmartLoop* pTmpHead = Mini_OffsetContourList( pCutHead        , 
													   NCDEF_OFFSET_INNER , 
													   dRadRgh         , 
													   m_cSetupDef.m_cCorDef ) ;
		Mini_DeleteContours( pCutHead ) ;
		pCutHead  = pTmpHead  ;
	}
	for( pLoop = pLpHead ; pLoop ; pLoop = pLoop->next )
	{
		pLoop->SetContourSectId( 100 ) ;
	}
	for( pLoop = pCutHead ; pLoop ; pLoop = pLoop->next )
	{
		pLoop->SetContourSectId( -100 ) ;
//		该处设置空段后,会在该处寻找下刀点.但是在函数中没有将PartBnd也就是本函数中的pLpHead传进去
//		所以,得到的下刀线是错误的,就算传进去,也不见得会得到合适的,这和形状有关系.
/*		for( CSmartSect* pSect = pLoop->GetSectHead() ; pSect ; pSect = pSect->next )
		{
			pSect->m_bEndFlag |= NC_BLANK_SECT ;
		}
		for( CSmartLoop* pIsl = pLoop->m_pIsland ; pIsl ; pIsl = pIsl->next )
		{
			for( pSect = pIsl->GetSectHead() ; pSect ; pSect = pSect->next )
			{
				pSect->m_bEndFlag |= NC_BLANK_SECT ;
			}
		}*/
	}
	// STEP 4 : 修剪轮廓
	if( pCutHead )
	{
         pLpHead = Contour.SubtractContours( pLpHead, pCutHead ) ;  
		 for( pLoop = pLpHead ; pLoop ; pLoop = pLoop->next )
		 {
			 CSmartLoop* pIsland = pLoop->GetIsland() ;
			 for(  ; pIsland ; pIsland = pIsland->next )
			 {
				if( pIsland->IsSameLoopSectId( -100 ) )
				{
					pIsland->m_bUseFlag |= NC_LOOP_REMAIN ;
				}
			 }
		 }
	}
	delete pLastTool ;
	return pLpHead ; 
}


BOOL CSmartRemainGen::GeneratePathEx( CPathGroup& NewPath ,  /*雕刻路径*/ 
                                      CSmartGraphic& Graph)  /*图形函数*/   
{
	// STEP 0 : 校正环境
	if( !UpdateTool2D() ) return FALSE ;
	CSmartLoop  *AllCont = Graph.GetAllContour( m_cSetupDef )  ;
	AllCont = RedefineBoundLoops( AllCont ) ;
	if( ! AllCont )
	{ // 校正图形
		m_nErrorType = JDERROR_GENPATH_NOLOOP ;
		return FALSE ;
	}

	BOOL bRet = CreatPath ( AllCont, NewPath ) ;

	Mini_DeleteContours( AllCont ) ;
	return bRet ;
}

BOOL CSmartRemainGen::CreatPath ( CSmartLoop* AllCont,
								  CPathGroup& NewPath )
{
	if ( !AllCont )
		return FALSE ;

	MathCAM_DefineRemainTool( m_cParam, m_cToolDef ) ;
	if( m_cParam.m_nRemainType )
	{
		m_cParam.m_cLastShape.m_cDepth = m_cShapeDef.m_cDepth ;
		m_cParam.m_cLastShape.m_cTaper = m_cShapeDef.m_cTaper ;
		m_cParam.m_cLastLayer.m_nLayerType = NCDEF_LAYER_DEPTH ;
	}
	
	JDNC_LAYER * pLayer   = GetLayerDef() ;
	pLayer->m_dTotalDepth = GetCutDepth()  - m_cStockDef.m_dDepthStock; 
	DefineLayerDepth( * pLayer) ; 
	// STEP 2 : 计算残料补加工路径
	CSmartLoop * pContour, *pLoop, *pLpHead  ; 
	m_cParam.m_cLastLayer.m_dTotalDepth = m_cParam.m_cLastShape.m_cDepth.m_dCutDepth - 
		                                  m_cParam.m_cLastShape.m_cStock0.m_dDepthStock ;
	DefineLayerDepth( m_cParam.m_cLastLayer ) ; 
    RepairBlindRadius( m_cFeedDef.m_cPlungeDef ) ;
	DOUBLE dDepth, dDepthInc ;
	if( GetLayerOrder() == 1 )
	{ // 高度优先
		for(  int i = 1 ;  i <= pLayer->m_nLayerCount ; i ++ ) 
		{
			dDepth  =  pLayer->m_dLayerDepth[ i ] ;
			dDepthInc = dDepth - pLayer->m_dLayerDepth[ i-1 ] ;
			CPathCombine * pTComb = new CPathCombine( NC_WPROCESS_ROUGH ) ;
			for( pContour = AllCont ; pContour ; pContour = pContour->next)
			{ 
				pLpHead = GetRemainContour( *pContour, m_cParam, dDepth, dDepthInc) ;
				for( pLoop = pLpHead ; pLoop ; pLoop = pLoop->next )
				{
					RemainOneLayer( *pTComb, *pLoop, m_cParam, dDepthInc ) ;
				}  
				Mini_DeleteContours( pLpHead ) ;
			}
			if( pTComb->m_pHead ) NewPath.AddData( dDepth, pTComb ) ;
			else delete pTComb ;
		}
	}
	else 
	{ // 区域优先
		CSmartLoop LoopTmp ;
		for( pContour = AllCont ; pContour ; pContour = pContour->next)
		{ 
		    CPathCombine * pTComb = new CPathCombine( NC_WPROCESS_ROUGH ) ;
			pLpHead = NULL ; 
			for(  int i = 1 ;  i <= pLayer->m_nLayerCount ; i ++ ) 
			{
				dDepth    =  pLayer->m_dLayerDepth[ i ] ;
				dDepthInc  = dDepth - pLayer->m_dLayerDepth[ i-1 ] ;
				CSmartLoop* pChild =  GetRemainContour( *pContour, m_cParam, dDepth, dDepthInc) ;
				for( pLoop = pChild ; pLoop ; pLoop = pLoop->next )
				{
					pLoop->m_nDepth = i ; 
					CSmartLoop* pIsland = pLoop->GetIsland()  ;
					for(  ; pIsland ; pIsland = pIsland->next )
					{
						pIsland->m_nDepth = i ;
					}
				}
				pChild = LoopTmp.ResortContour( pChild,NCDEF_SORTTYPE_MINDIST,FALSE ) ;  
				pLpHead = Mini_AddContours( pLpHead, pChild ) ;
			}
			if( pLayer->m_nLayerCount > 1 )
			{
				pLpHead = Mini_SortLayerContours( pLpHead , m_pTool->GetRadiusComp( 0.0, 0.0 ) ) ;
			}
			for( pLoop = pLpHead ; pLoop ; pLoop = pLoop->next )
			{
				CPathCombine tmpComb( NC_WPROCESS_ROUGH ) ;
				dDepth    =  pLayer->m_dLayerDepth[ pLoop->m_nDepth ] ;
				dDepthInc  = dDepth - pLayer->m_dLayerDepth[ pLoop->m_nDepth-1 ] ;
				RemainOneLayer( tmpComb, *pLoop, m_cParam, dDepthInc ) ;
				tmpComb.MoveCombine( 0.0, 0.0, -dDepth ) ;
				pTComb->AppendCombine( tmpComb ) ;
			}
			Mini_DeleteContours( pLpHead ) ;
			if( pTComb->m_pHead ) NewPath.AddData( 0.0, pTComb ) ;
			else delete pTComb ;
		}  
	}
	
	if( pLayer->m_dLayerDepth ) 
	{
		delete[] pLayer->m_dLayerDepth ;
		pLayer->m_dLayerDepth = NULL ;
	}
	if ( m_cParam.m_cLastLayer.m_dLayerDepth )
	{
		delete[] m_cParam.m_cLastLayer.m_dLayerDepth ;
		m_cParam.m_cLastLayer.m_dLayerDepth = NULL ;
	}

	// STEP 3 : 转换路径
	if( ! NewPath.m_pHead )
	{
		m_nErrorType = JDERROR_GENPATH_NOAREA ;
		return FALSE ; 
	}
	// 校正表面高度
	NewPath.ZMoveGroup( GetTopHeight() ) ;
	
	return TRUE ;
}

BOOL ArtSurf_AllCutContour( CPathCombine& TComb, CSmartLoop& Contour, double RComp, JDNC_SETUP& Setup )
{
	CSmtVoronoiMap    Voronoi ;
	TPNT3D    Buffer[4005] ;
	int      nPoint = 0 ;
	CPathPLine3D *pPLine ; 
	Voronoi.m_bCenAll = FALSE ;
	Voronoi.SetTolAndCorner( Setup.m_cTolDef, Setup.m_cCorDef ) ;
    Voronoi.UpdateContour( Contour, RComp ) ;
	Voronoi.DefineBound(  VORONOI_CORSIDE ) ; 
	CCLineCurve * CurveHead, *pCurve ;
	CurveHead = Voronoi.DefineCLineCurve() ;
	for( pCurve = CurveHead ; pCurve ; pCurve = pCurve->next )
	{
		if( pCurve->m_bIsCenter != TRUE  ) continue ;
		CCLinePoint *  pPoint ;
		for( pPoint = pCurve->m_pHead ; pPoint ; pPoint = pPoint->next )
		{
			pPoint->m_fPoint[2] = 0.0 ;
		}
		pCurve->RefineCurve ( 0.002f ) ;
		for( pPoint = pCurve->m_pHead  ; pPoint ; pPoint = pPoint->next )
		{
			nc_VectorCopy( Buffer[ nPoint ], pPoint->m_fPoint, 3 ) ;
	        if( nPoint == 0 ) nPoint ++ ;
	        else if ( ( pPoint == pCurve->m_pHead && 
				nc_FabsDist( Buffer[nPoint], Buffer[nPoint-1],3) > 0.002 ) || 
				nPoint >= 4000 )
			{ 
				pPLine = new CPathPLine3D() ;
                pPLine->Copy( Buffer , nPoint - 1 ) ;
		        TComb.AddEntity( pPLine ) ;
				if( pPoint == pCurve->m_pHead && nPoint < 4000 )
				{
					nc_VectorCopy(Buffer[0], Buffer[nPoint], 3 ) ;
		            nPoint = 1 ;
				}
				else
				{
					nc_VectorCopy(Buffer[0], Buffer[nPoint-1], 3) ; 
					nc_VectorCopy(Buffer[1], Buffer[nPoint], 3 ) ;
		            nPoint = 2 ;
				}
			} 
			else nPoint ++ ;
		}
	}
	if( nPoint > 1 )
	{
       pPLine = new CPathPLine3D() ;
       pPLine->Copy( Buffer , nPoint - 1 ) ;
       TComb.AddEntity( pPLine ) ;
	}
	while( CurveHead )
	{
		pCurve = CurveHead->next ;
		delete CurveHead ;
		CurveHead = pCurve  ;
	}
	return TRUE ; 
}
CSmartBoundGen::CSmartBoundGen()
{
	m_bLoopCut = FALSE ;
}
CSmartBoundGen::~CSmartBoundGen()
{
	MathCam_DeleteAllRedepthCur( m_cListCur ) ;
}
int CSmartBoundGen::GetBoundRCompMask()
{
    if( m_cSetupDef.m_cModelTol.m_nMdlFlag & NCDEF_SETUP_WEARCOMP_INC )
    {
        if( GetMillDir() == 0 ) return JDNC_FGCODE_RCOMPG41 ;
        return JDNC_FGCODE_RCOMPG42 ;
    }
    else if( m_cSetupDef.m_cModelTol.m_nMdlFlag & NCDEF_SETUP_WEARCOMP_DEC )
    {
        if( GetMillDir() == 0 ) return JDNC_FGCODE_RCOMPG42 ;
        return JDNC_FGCODE_RCOMPG41 ;
    }
    return 0 ;
}

///////////////////////
// 提取轮廓路径
int CSmartBoundGen::BoundOneLayer(  CPathCombine& PComb   ,  /*路径集合*/
						            CSmartLoop&    Contour,  /*边界轮廓*/
								    JDNC_BOUND&  BoundCut ,  /*加工参数*/
									RFRAME &	LocFrame  ,	 /*局部坐标系*/
                                    DOUBLE       Depth    ,  /*深度位置*/
						            DOUBLE       DepthInc ,   /*雕刻深度*/
									int nR )     /*圆弧延伸*/
{
	if (m_cFeedDef.m_cPlungeDef.m_nPlungeType == NCDEF_PLUNGE_CONTOUR)
	{
		// 沿轮廓下刀，先生成进刀路径，然后再下刀路径，2013.12.06 liuxin
		return BoundOneLayerByContourPlunge(PComb, Contour, BoundCut, LocFrame, 0., DepthInc, nR);
	}
	//  STEP 0 : 偏移轮廓, 计算下刀保护边界 
	JDNC_PLUNGE  Plunge = m_cFeedDef.m_cPlungeDef ;
	if( Plunge.m_nPlungeType != NCDEF_PLUNGE_CLOSE )
	{
		Plunge.m_dIncStep = DepthInc + Plunge.m_dTopTol ;
	}
	// STEP 1 : 提取所有轮廓 , 并生成路径
	CSmtLoopArr   AllLoop ;
	Contour.ExtractAllLoop( m_cSetupDef.m_cOrderDef.m_nSortType , AllLoop ) ;
	INT_PTR nLoop = AllLoop.GetSize() ;
	CSmartSect *pSect  ; 
	CSmartCurve * pBound, LeadInCv, LeadOutCv, OverlapCv ;
    FLOAT fRateScale = 1.0f ;
    if( BoundCut.m_bBoundFlag & NCDEF_FBOUND_FEEDRATIO ) 
    {
        fRateScale = (float)( BoundCut.m_dFeedRatio * 0.01 ) ;
    }
    int nRCompMask = GetBoundRCompMask() ;
    JDNC_LEAD  rcompLead = m_cFeedDef.m_cLeadDef ;
    rcompLead.m_dLength = max( 0.05, m_cFeedDef.m_cLeadDef.m_dWearLine) ;
    if( rcompLead.m_nLeadType == 0 ) rcompLead.m_nLeadType = NCDEF_LEAD_SLINE ;
    else   rcompLead.m_nLeadType = NCDEF_LEAD_LINE ;
    DOUBLE rcompZShift[2] = { 0.0, 0.0 } ;
	DOUBLE dOverlapLen = m_cParam.m_cLeadOut.m_dOverlap ;
	JDNC_LEAD  leadIn  = m_cFeedDef.m_cLeadDef ;
	JDNC_LEAD  leadOut = m_cFeedDef.m_cLeadDef ;
	if( m_bLoopCut )
		UpdateLeadOutParam( m_cParam.m_cLeadOut, leadOut ) ;
	else
	{
		if( m_cParam.m_cLeadOut.m_bSameLeadIn == FALSE )
		{
			leadOut.m_nLeadType = m_cParam.m_cLeadOut.m_nLeadType ;
			leadOut.m_dLength   = m_cParam.m_cLeadOut.m_dLength   ;
			leadOut.m_dAngle    = m_cParam.m_cLeadOut.m_dAngle    ;
			leadOut.m_dRadius   = m_cParam.m_cLeadOut.m_dRadius   ;
		}
	}
	// 保留曲线高度参数
	BOOL bKeep[2] = { FALSE, FALSE } ;
	if( m_cParam.m_bBoundFlag & NCDEF_FBOUND_REDEPTH )
	{
		bKeep[0] = TRUE ;
	}
	
	DOUBLE ZShift[2] = {} ;
	GetLeadCutShift(m_cParam.m_cLeadOut, ZShift);
	for( INT_PTR i = 0 ; i < nLoop ; i ++ ) 
	{
		if( (AllLoop[i]->m_bUseFlag & NC_LOOP_OUTER) || 
			AllLoop[i]->m_pCurve   == NULL  )
		{
			continue  ;
		}
	    pBound = AllLoop[i]->m_pCurve->CopyMyself() ;
        pBound->SetAllZValue( 0.0 ) ;
		if( ! pBound ) continue ;
		if( m_cSetupDef.m_cCorDef.m_nCorType == NCDEF_CORNER_EXTARC && nR != 0 )
		{//轮廓切割&&延伸圆弧&&侧向分层
			ResetCurve( pBound, m_cProcessDef.m_dExtRadius, nR ) ;
		}
		for( pSect = pBound->GetHead() ; pSect ; pSect = pSect->next )
		{
            if( (pSect->m_bEndFlag & NC_BOUND_SECT) && fRateScale != 1.0f )
            {
                pSect->m_bFeedType = JDNC_FEEDTYPE_SLOT  ;
            }
            else
            {
			    pSect->m_bFeedType = JDNC_FEEDTYPE_ROUGH ;
            }
		}

		// 生成路径,并保留曲线高度
		CPathCombine tmpBase( NC_WPROCESS_ROUGH ) ;
		// 封掉原因：因圆弧进退刀有可能会引起加工曲线起始点的变化，
		// 所以需要在生成完进退刀路径之后才将曲线转化为路径，否则路径起点会不正确 qqs 2014.02.26
		//tmpBase.AppendCurve3D( *pBound , 0.0, TRUE ) ;
		//if( bKeep[0] || bKeep[1])
		//{
		//	RedepthCPathCombine( m_cListCur, &tmpBase, bKeep, LocFrame ) ;
		//}
		if( leadIn.m_nLeadType != NCDEF_LEAD_CLOSE || 
			leadOut.m_nLeadType != NCDEF_LEAD_CLOSE )
		{/* 1: 尽可能搜索到切入切出 */
			if( GetMillDir() == 0 ) 
			{
				CreateBoundLeadExNew( leadOut,leadIn,*pBound,OverlapCv, LeadInCv,LeadOutCv,&Contour, dOverlapLen, Depth, ZShift, DepthInc) ;
			}
			else
			{
				CreateBoundLeadExNew( leadIn,leadOut,*pBound,OverlapCv, LeadInCv,LeadOutCv,&Contour, dOverlapLen, Depth, ZShift, DepthInc) ;
			}
			// 因圆弧进退刀有可能会引起加工曲线起始点的变化，所以需要在生成完进退刀路径之后才将曲线转化为路径，否则路径起点会不正确 qqs 2014.02.26
			tmpBase.AppendCurve3D( *pBound , 0.0, TRUE ) ;
			if( bKeep[0] )
			{
				RedepthCPathCombine( m_cListCur, &tmpBase, bKeep, LocFrame ) ;
			}
			// 将进退刀和重复加工段添加到路径中
			AddLeadInOutAndOverlapForPComb(tmpBase, &OverlapCv, &LeadInCv, &LeadOutCv, 0.0, m_cListCur, bKeep, LocFrame);
			pBound->InsertAfter( LeadInCv, NULL ) ;
			pBound->AppendCurve( LeadOutCv ) ;
		} 
		else
		{
			tmpBase.AppendCurve3D( *pBound , 0.0, TRUE ) ;
			if( bKeep[0] )
			{
				RedepthCPathCombine( m_cListCur, &tmpBase, bKeep, LocFrame ) ;
			}
		}
		if( GetMillDir() == 0 ) 
		{
			pBound->Reverse() ;
			ReversePComb( tmpBase ) ;
		}

		if( nRCompMask != 0 ) 
		{
			CSmartCurve compLeadInCv, compLeadOutCv ;
			int bFlag = AddLeadSectAtEndExNew( rcompLead, rcompLead, *pBound, compLeadInCv, 
				compLeadOutCv, &Contour, 0.0, rcompZShift, TRUE ) ;
		
			// 将曲线添加到路径中
			AddLeadInOutForPComb( tmpBase, &compLeadInCv, &compLeadOutCv ) ;
			pBound->InsertAfter(compLeadInCv, NULL);
			pBound->AppendCurve(compLeadOutCv);
			if (!bFlag)
			{
				m_bRComb = 2;
			}            
		}
		//        CPathCombine tmpBase( NC_WPROCESS_ROUGH ) ;
		//	    tmpBase.AppendCurve3D( *pBound , 0.0, TRUE ) ;
		for( CPathEntity* pEnt = tmpBase.m_pHead ; pEnt ; pEnt = pEnt->next )
		{
			if( pEnt->m_bFeedType == JDNC_FEEDTYPE_SLOT )
			{
				pEnt->m_bFeedType = JDNC_FEEDTYPE_ROUGH ;
				pEnt->m_fFeedScale = fRateScale ;
			}
		}
		AddRCompMask( tmpBase, nRCompMask ) ;
		if( Plunge.m_nPlungeType != NCDEF_PLUNGE_CLOSE )
		{ // 增加下刀方式
			CSmartLoop *pChild = NULL ; 
			PNT3D  dPoint, dAtPoint ;
			pBound->GetPoint( 0.0, dPoint ) ;
			dPoint[2]  = 0  ;
			if( Plunge.m_nPlungeType == NCDEF_PLUNGE_VERT  ||
				Plunge.m_nPlungeType == NCDEF_PLUNGE_HELIX   || 
				Plunge.m_nPlungeType == NCDEF_PLUNGE_RAMP    )
			{
				DOUBLE dOffset = Plunge.m_dSideTol ; 
				if( Plunge.m_nPlungeType == NCDEF_PLUNGE_HELIX ) 
				{
					double dDist = MathCAM_MiniDistToContour( Contour, dPoint ) ;
					dOffset += max( Plunge.m_dRadius, dDist) ;
				}
				pChild = Contour.OffsetContourEx( dOffset , dOffset, 
					m_cSetupDef.m_cCorDef ) ;
			}
			CPathCombine cTPlunge( NC_WPROCESS_PLUNGE ) ;
			if( FindPlungePoint(Plunge,  Contour, pChild, dPoint, dAtPoint, pBound ) )
			{ // 在固定位置下刀
				AddPlungePathAt( cTPlunge, Plunge, dPoint, dAtPoint,&Contour ) ;
			}
			if( ! cTPlunge.m_pHead )
			{
				AddPlungePathBy( cTPlunge, Plunge , dPoint, *pBound, TRUE ) ;
			}
			// 平移下刀路径，防止路径断开, 2013.12.17 liuxin
			{
				tmpBase.GetEndPoint( 0, dPoint ) ;
				cTPlunge.GetEndPoint(1, dAtPoint);
				cTPlunge.MoveCombine(0.0, 0.0, dPoint[2] - dAtPoint[2]);
			}
			PComb.AppendCombine( cTPlunge ) ;
			Mini_DeleteContours( pChild ) ;
		}

		if( BoundCut.m_bBoundFlag & NCDEF_FBOUND_MINUP ) 
		{  // 减少抬刀
			PNT3D  dStart, dEnd, dMid ;
			double dMaxLen = m_pTool->m_fBottom * 4 ;
			pBound->GetPoint( 0.0, dEnd ) ;
			dStart[2] = dEnd[2] = 0.0 ;
			if( PComb.GetEndPoint( 1, dStart )  && 
				fabs( dStart[2] ) < 1.0e-3 ) 
			{
				nc_GetPointAtLine( dStart, dEnd, 0.5, dMid, 2 ) ;
				if( nc_Distance( dStart, dEnd, 2 ) > 1.0e-3 && 
					nc_Distance( dStart, dEnd, 2 ) < dMaxLen && 
					Contour.IsPtOnContour( dMid ) && 
					!Contour.IntLineContour( dStart, dEnd ) ) 
				{
					CPathLine3D *pLine3D = new CPathLine3D( dStart, dEnd ) ;
					pLine3D->m_bFeedType = JDNC_FEEDTYPE_CONNECT ;
					PComb.AddEntity( pLine3D ) ;
				}
			}
		}
        delete pBound ;
        PComb.AppendCombine( tmpBase ) ;
	}
	return 1 ; 
}

int CSmartBoundGen::BoundOneLayerByContourPlunge(  CPathCombine& PComb   ,  /*路径集合*/
													CSmartLoop&    Contour,  /*边界轮廓*/
													JDNC_BOUND&  BoundCut ,  /*加工参数*/
													RFRAME &	LocFrame  ,	 /*局部坐标系*/
													DOUBLE       Depth    ,  /*深度位置*/
													DOUBLE       DepthInc ,   /*雕刻深度*/
													int nR )     /*圆弧延伸*/
{
	UNUSED_ALWAYS(Depth); 
	//  STEP 0 : 偏移轮廓, 计算下刀保护边界 
	JDNC_PLUNGE  Plunge = m_cFeedDef.m_cPlungeDef ;
	if( Plunge.m_nPlungeType != NCDEF_PLUNGE_CLOSE )
	{
		Plunge.m_dIncStep = DepthInc + Plunge.m_dTopTol ;
	}
	// STEP 1 : 提取所有轮廓 , 并生成路径
	CSmtLoopArr   AllLoop ;
	Contour.ExtractAllLoop( m_cSetupDef.m_cOrderDef.m_nSortType , AllLoop ) ;
	INT_PTR nLoop = AllLoop.GetSize() ;
	CSmartSect *pSect  ; 
	CSmartCurve * pBound, LeadInCv, LeadOutCv ;
	FLOAT fRateScale = 1.0f ;
	if( BoundCut.m_bBoundFlag & NCDEF_FBOUND_FEEDRATIO ) 
	{
		fRateScale = (float)( BoundCut.m_dFeedRatio * 0.01 ) ;
	}
	int nRCompMask = GetBoundRCompMask() ;
	JDNC_LEAD  rcompLead = m_cFeedDef.m_cLeadDef ;
	rcompLead.m_dLength = max( 0.05, m_cFeedDef.m_cLeadDef.m_dWearLine) ;
	if( rcompLead.m_nLeadType == 0 ) rcompLead.m_nLeadType = NCDEF_LEAD_SLINE ;
	else   rcompLead.m_nLeadType = NCDEF_LEAD_LINE ;
	DOUBLE rcompZShift[2] = { 0.0, 0.0 } ;
	JDNC_LEAD  leadIn  = m_cFeedDef.m_cLeadDef ;
	JDNC_LEAD  leadOut = m_cFeedDef.m_cLeadDef ;
	if( m_bLoopCut )
		UpdateLeadOutParam( m_cParam.m_cLeadOut, leadOut ) ;
	else
	{
		if( m_cParam.m_cLeadOut.m_bSameLeadIn == FALSE )
		{
			leadOut.m_nLeadType = m_cParam.m_cLeadOut.m_nLeadType ;
			leadOut.m_dLength   = m_cParam.m_cLeadOut.m_dLength   ;
			leadOut.m_dAngle    = m_cParam.m_cLeadOut.m_dAngle    ;
			leadOut.m_dRadius   = m_cParam.m_cLeadOut.m_dRadius   ;
		}
	}
	// 关闭沿轮廓进刀
	if (leadIn.m_nLeadType == NCDEF_LEAD_BYCONTOUR)
	{
		leadIn.m_nLeadType = NCDEF_LEAD_CLOSE;
	}
	// 保留曲线高度参数
	BOOL bKeep[2] = { FALSE, FALSE } ;
	if( m_cParam.m_bBoundFlag & NCDEF_FBOUND_REDEPTH )
	{
		bKeep[0] = TRUE ;
	}
	
	DOUBLE ZShift[2] = {} ;
	GetLeadCutShift(m_cParam.m_cLeadOut, ZShift);
	for( INT_PTR i = 0 ; i < nLoop ; i ++ ) 
	{
		if( (AllLoop[i]->m_bUseFlag & NC_LOOP_OUTER) || 
			AllLoop[i]->m_pCurve   == NULL  )
		{
			continue  ;
		}
		pBound = AllLoop[i]->m_pCurve->CopyMyself() ;
		pBound->SetAllZValue( 0.0 ) ;
		if( ! pBound ) continue ;
		if( m_cSetupDef.m_cCorDef.m_nCorType == NCDEF_CORNER_EXTARC && nR != 0 )
		{//轮廓切割&&延伸圆弧&&侧向分层
			ResetCurve( pBound, m_cProcessDef.m_dExtRadius, nR ) ;
		}
		for( pSect = pBound->GetHead() ; pSect ; pSect = pSect->next )
		{
			if( (pSect->m_bEndFlag & NC_BOUND_SECT) && fRateScale != 1.0f )
			{
				pSect->m_bFeedType = JDNC_FEEDTYPE_SLOT  ;
			}
			else
			{
				pSect->m_bFeedType = JDNC_FEEDTYPE_ROUGH ;
			}
		}
		if( GetMillDir() == 0 ) 
		{
			pBound->Reverse() ; 
		}

		// 生成路径,并保留曲线高度
		CPathCombine tmpBase( NC_WPROCESS_ROUGH ) ;
		// 沿轮廓下刀，先生成进刀路径，然后再下刀路径，2013.12.06 liuxin
		// 将-Depth改为0，是因为用到该沿轮廓下刀函数计算路径的地方，计算结束后，都会将路劲再次按加工深度进行平移，
		// 如果此处按-Depth进行路径生成，结束后再次平移，会导致路径深度为实际设置的深度的两倍的错误。 qqs 2014.02.07
		DOUBLE dAngle = ANGLE_TO_RADIAN( Plunge.m_dAngle);
		CSmartCurve* pNewBound = NULL  ;
		tmpBase.AddSlantCurve( *pBound ,dAngle,
								Plunge.m_dIncStep,
								0. , //- Depth
								TRUE, &pNewBound ) ;
		if( pNewBound && pNewBound != pBound )
		{
			delete pBound ;							
			pBound = pNewBound  ;						
		} 		
		// 如果保留曲线高度或者读取曲线速度
		if( bKeep[0] || bKeep[1] )
		{
			RedepthCPathCombine( m_cListCur, &tmpBase, bKeep, LocFrame) ;
		}
		AddLeadPathAtEndEx( leadIn, leadOut, tmpBase, Contour, 0., ZShift, DepthInc) ;//	Depth					
		if( nRCompMask != 0 ) 						
		{							
			int bFlag = AddLeadPathAtEndEx( rcompLead, rcompLead, tmpBase, Contour, 0.0, rcompZShift) ;
			AddRCompMask( tmpBase, nRCompMask ) ;
			if (!bFlag)
			{
				m_bRComb = 2;
			}            
		}

		if( BoundCut.m_bBoundFlag & NCDEF_FBOUND_MINUP ) 
		{  // 减少抬刀
			PNT3D  dStart, dEnd, dMid ;
			pBound->GetPoint( 0.0, dEnd ) ;
			dStart[2] = dEnd[2] = 0.0 ;
			if( PComb.GetEndPoint( 1, dStart )  && 
				fabs( dStart[2] ) < 1.0e-3 ) 
			{
				nc_GetPointAtLine( dStart, dEnd, 0.5, dMid, 2 ) ;
				if( nc_Distance( dStart, dEnd, 2 ) > 1.0e-3 && 
					Contour.IsPtOnContour( dMid ) && 
					!Contour.IntLineContour( dStart, dEnd ) ) 
				{
					CPathLine3D *pLine3D = new CPathLine3D( dStart, dEnd ) ;
					pLine3D->m_bFeedType = JDNC_FEEDTYPE_CONNECT ;
					PComb.AddEntity( pLine3D ) ;
				}
			}
		}
		delete pBound ;
		PComb.AppendCombine( tmpBase ) ;
	}
	return 1 ; 
}

int CSmartBoundGen::AddLeadSectAtSpiralEnd( JDNC_LEAD& LeadInDef , 
									        JDNC_LEAD& LeadOutDef, 
									        CSmartCurve& Curve   , 
									        CSmartLoop *BndCont[2],
                                            DOUBLE Depth, 
                                            DOUBLE ZShift[2] ) 
{
	// 该函数未使用
	if( ! Curve.m_pHead ) return 0 ;
    CSmartCurve leadInCv, leadOutCv ;
    BOOL bFlag1 = TRUE, bFlag2 = TRUE;
	bFlag1 = GenLeadSectAtEndEx( LeadInDef, FALSE, Curve, BndCont[0], Depth, ZShift[0], leadInCv ) ;
    bFlag2 = GenLeadSectAtEndEx( LeadOutDef,TRUE , Curve, BndCont[1], Depth, ZShift[1], leadOutCv ) ;
    CSmartSect* pSect ;
    for( pSect = leadInCv.GetHead() ; pSect ; pSect = pSect->next )
    {
        pSect->m_dZValue[0] += Curve.m_pHead->m_dZValue[0]  ;
        pSect->m_dZValue[1] += Curve.m_pHead->m_dZValue[0]  ;
    }
    for( pSect = leadOutCv.GetHead() ; pSect ; pSect = pSect->next )
    {
        pSect->m_dZValue[0] += Curve.m_pTail->m_dZValue[1]  ;
        pSect->m_dZValue[1] += Curve.m_pTail->m_dZValue[1]  ;
    }
    Curve.InsertAfter( leadInCv, NULL ) ;
    Curve.AppendCurve( leadOutCv ) ;
	return bFlag1&&bFlag2 ;
}
int CSmartBoundGen::AddLeadPCombAtSpiralEnd( JDNC_LEAD& LeadInDef , 
									        JDNC_LEAD& LeadOutDef, 
											CPathCombine &PComb	,
									        CSmartCurve& Curve   , 
									        CSmartLoop *BndCont[2],
                                            DOUBLE Depth, 
                                            DOUBLE ZShift[2],
											BOOL bKeep[2],
											RFRAME &LocFrame,
											DOUBLE dDepthInc) 
{
	if( ! Curve.m_pHead ) return 0 ;
    CSmartCurve leadInCv, leadOutCv ;
    BOOL bFlag1 = TRUE, bFlag2 = TRUE;
	bFlag1 = GenLeadSectAtEndEx( LeadInDef, FALSE, Curve, BndCont[0], Depth, ZShift[0], leadInCv, dDepthInc ) ;
    bFlag2 = GenLeadSectAtEndEx( LeadOutDef,TRUE , Curve, BndCont[1], Depth, ZShift[1], leadOutCv, dDepthInc ) ;
    CSmartSect* pSect ;
    for( pSect = leadInCv.GetHead() ; pSect ; pSect = pSect->next )
    {
        pSect->m_dZValue[0] += Curve.m_pHead->m_dZValue[0]  ;
        pSect->m_dZValue[1] += Curve.m_pHead->m_dZValue[0]  ;
    }
    for( pSect = leadOutCv.GetHead() ; pSect ; pSect = pSect->next )
    {
        pSect->m_dZValue[0] += Curve.m_pTail->m_dZValue[1]  ;
        pSect->m_dZValue[1] += Curve.m_pTail->m_dZValue[1]  ;
    }

	// 将曲线添加到PComb当中
	TPNT3D start, end ;
	
	if( PComb.m_pHead )
	{
		if( leadInCv.m_pHead )
		{
			CPathCombine tmpLeadIn(JDNC_FEEDTYPE_LEAD);
			leadInCv.SetFeedType ( JDNC_FEEDTYPE_LEAD ) ;
			PComb.m_pHead->GetEndPoint( 0, start ) ;
			tmpLeadIn.AppendCurve3D(leadInCv, start[2], JDNC_FEEDTYPE_LEAD);
			if( bKeep[0] && m_cFeedDef.m_cLeadDef.m_nLeadType == NCDEF_LEAD_BYCONTOUR )
			{
				RedepthCPathCombine( m_cListCur, &tmpLeadIn, bKeep, LocFrame);
			}
			
			tmpLeadIn.GetEndPoint(1, end);
			tmpLeadIn.MoveCombine(0.0, 0.0, start[2] - end[2]);
			tmpLeadIn.AppendCombine(PComb);
			PComb.AppendCombine(tmpLeadIn);
		}
		if( leadOutCv.m_pHead )
		{
			CPathCombine tmpLeadOut(JDNC_FEEDTYPE_LEAD);
			leadOutCv.SetFeedType( JDNC_FEEDTYPE_LEAD ) ;
			PComb.m_pTail->GetEndPoint( 1, end ) ;
			tmpLeadOut.AppendCurve3D(leadOutCv, end[2], JDNC_FEEDTYPE_LEAD);
			if( bKeep[0] && m_cFeedDef.m_cLeadDef.m_nLeadType == NCDEF_LEAD_BYCONTOUR )
			{
				RedepthCPathCombine( m_cListCur, &tmpLeadOut, bKeep, LocFrame);
			}
			
			tmpLeadOut.GetEndPoint(0, start);
			tmpLeadOut.MoveCombine(0.0, 0.0, end[2] - start[2]);
			PComb.AppendCombine(tmpLeadOut);
		}
	}
	Curve.InsertAfter( leadInCv, NULL ) ;
	Curve.AppendCurve( leadOutCv ) ;

	return bFlag1&&bFlag2 ;
}

static DOUBLE MathCAM_MinDistToContour( CSmartLoop& Contour, PNT2D Point )
{
    double dMinDist = Contour.MinDistPoint( Point ) ;
    for( CSmartLoop * pIsl = Contour.GetIsland() ; pIsl; pIsl = pIsl->next )
    {
        double dDist = pIsl->MinDistPoint( Point ) ;
        if( dDist < dMinDist ) dMinDist = dDist ;
    }
    return dMinDist; 
}

// 添加光滑的沿轮廓下刀路径
int CSmartBoundGen::AddContourPlungePath(CPathCombine	& PComb, 
										 CSmartCurve	& Curve,
										 JDNC_PLUNGE	& Plunge,
										 PNT3D			ToPoint,
										 CSmartCurve	& PlungeCrv,
										 CSmartCurve *   pBoundOrg )
{
	// STEP 0 : 获取轮廓曲线
	if( ! Curve.m_pHead ) return 0 ;
	CSmartLoop* pBndCount = NULL;
	CSmartCurve* pBound = NULL;	
	if( Curve.IsClosed() )
	{
		pBound = Curve.CopyMyself();
	}
	else
	{
		PNT2D dPoint[2];
		CSmartCurve tmpCur;
		CSmartSect* pSect = NULL, *pPrevSect = NULL;
		Curve.GetEnd(dPoint[1]);
		for (pSect = Curve.m_pTail; pSect; pSect = pSect->prev)
		{
			pPrevSect = pSect->prev;
			if (pSect && pPrevSect)
			{
				pPrevSect->GetStart(dPoint[0]);
				if (mathDist2D(dPoint[0], dPoint[1]) < MIN_DIS)
				{
					tmpCur.InsertAfter(pSect->CopyMyself(), NULL);
					tmpCur.InsertAfter(pPrevSect->CopyMyself(), NULL);
					break;
				}
				else
				{
					tmpCur.InsertAfter(pSect->CopyMyself(), NULL);
				}
			}
		}
		if (tmpCur.m_pHead && tmpCur.IsClosed())
		{
			CSmartLoop* pLoop = tmpCur.FormLoop();
			if (pLoop)
			{
				Curve.GetStart(dPoint[0]);
				Curve.GetEnd(dPoint[1]);
				double dDist = max( 0.0, MathCAM_MinDistToContour( *pLoop, dPoint[0] )) ;
				pBndCount = pLoop->OffsetContourEx(dDist, dDist, m_cSetupDef.m_cCorDef);
				if (pBndCount)
				{
					pSect = pBndCount->m_pCurve->MinDistSect(dPoint[0], dPoint[1], dDist);
					if (pSect && dDist < MIN_DIS && pSect->m_nSectId >= 0 && pSect->m_nSectId < pBndCount->m_pCurve->m_nNumSect)
					{
						pBndCount->m_pCurve->SetStartPoint(pSect, dPoint[1]);
						pBound = pBndCount->m_pCurve->CopyMyself();
					}
					Mini_DeleteContours(pBndCount);
				}
				Mini_DeleteContours(pLoop);
			}
		}	
	}
	if (!pBound)
	{
		return FALSE;
	}

	if( pBound->GetLength() < 0.02 )
	{
		pBound->ClearAll() ;
		delete pBound;
		return FALSE;
	}

	// STEP 1 : 计算长度
	CSmartCurve   PlgCurve ;
	DOUBLE dCurveLen = pBound->GetLength() ;
	if( dCurveLen < 0.02 ) return FALSE ;
	DOUBLE dTan = tan( ANGLE_TO_RADIAN( Plunge.m_dAngle)) ;
	if( Plunge.m_dIncStep > Plunge.m_dMaxZInc )
	{
		double dMaxTan = Plunge.m_dMaxZInc / dCurveLen ;
		if( dTan > dMaxTan ) dTan = dMaxTan ;
	}
	DOUBLE dSlantLen = Plunge.m_dIncStep / dTan;
	int nLoop = (int)( dSlantLen / dCurveLen ) ;
	if( nLoop > 30 && dCurveLen < 1.0 ) nLoop  =  30  ;/*短线段保护*/
	// STEP 2 : 路径分段
	DOUBLE dLeft = dSlantLen - dCurveLen * nLoop ;
	CSmartCurve *pFirst = NULL, *pSecond = NULL  ;
	CSmartCurve TmpCurve ;
	{ // 封闭曲线
		// 根据实际加工曲线的起点，修改下刀路径环的起点，并根据新起点位置对路径进行打断 qqs 2014.02.21
		if( pBoundOrg )
		{
			PNT2D ptStart, ptObj;
			DOUBLE dDis = 0. ;
			pBoundOrg->GetStart( ptStart ) ;
			CSmartSect* pObjSect = pBound->MinDistSect(ptStart, ptObj, dDis);
			pBound->SetStartPoint(pObjSect, ptObj);
		}
		pBound->BreakCurveAt(  dCurveLen - dLeft , pSecond, pFirst ) ;
		
		if( pFirst ) PlgCurve.AddCurve( pFirst->CopyMyself() ) ;
		for( int i = 0 ; i < nLoop ; i ++  )
		{
			if( pSecond ) PlgCurve.AddCurve( pSecond->CopyMyself() ) ;
			if( pFirst  ) PlgCurve.AddCurve( pFirst->CopyMyself() ) ;
		}
		if( pFirst ) delete pFirst ;
		if( pSecond ) delete pSecond ;
	}		
	PComb.AddCurveEx( PlgCurve, Plunge.m_dIncStep, Plunge.m_dIncStep ) ;
	PlungeCrv.AppendCurve(PlgCurve);
	PNT3D dAtPoint ;
	if( PlgCurve.m_pHead &&  PComb.GetEndPoint( 1, dAtPoint ) && 
		nc_Distance( dAtPoint,ToPoint, 3 )  > 2.0e-4 )
	{
		CPathLine3D* pLine3D = new CPathLine3D( dAtPoint , ToPoint ) ;
		pLine3D->m_bFeedType = JDNC_FEEDTYPE_CONNECT ;
		PComb.AddEntity( pLine3D ) ;
	}

	pBound->ClearAll();
	delete pBound;
	return TRUE ;
}

int CSmartBoundGen::BoundOneLayerEx(  CPathCombine& PComb   ,  /*路径集合*/
						              CSmartLoop&    Contour,  /*边界轮廓*/
                                      CSmtLoopArr&   AllLoop,  /*所有边界*/
								      JDNC_BOUND&  BoundCut ,  /*加工参数*/
									  RFRAME	&	LocFrame,	/*局部坐标系*/
									  BOOL		   bSmooth	,  /*是否光滑*/
                                      DOUBLE       Depth    ,
						              DOUBLE       DepthInc ,  /*雕刻深度*/  
									  DOUBLE	   dSideInc)   /*偏移量  */  
{
	if (m_cFeedDef.m_cPlungeDef.m_nPlungeType == NCDEF_PLUNGE_CONTOUR)
	{
		return BoundOneLayerExByContourPlunge(PComb, Contour, AllLoop, BoundCut,
											LocFrame, bSmooth, Depth, DepthInc, dSideInc);
	}
	//  STEP 0 : 偏移轮廓, 计算下刀保护边界 
	JDNC_PLUNGE  Plunge = m_cFeedDef.m_cPlungeDef ;
	if( Plunge.m_nPlungeType != NCDEF_PLUNGE_CLOSE )
	{
		Plunge.m_dIncStep = DepthInc + Plunge.m_dTopTol ;
	}
	JDNC_LEAD  leadIn  = m_cFeedDef.m_cLeadDef ;
	JDNC_LEAD  leadOut = m_cFeedDef.m_cLeadDef ;
	if( m_bLoopCut )
		UpdateLeadOutParam( m_cParam.m_cLeadOut, leadOut ) ;
	else
	{
		if( m_cParam.m_cLeadOut.m_bSameLeadIn == FALSE )
		{
			leadOut.m_nLeadType = m_cParam.m_cLeadOut.m_nLeadType ;
			leadOut.m_dLength   = m_cParam.m_cLeadOut.m_dLength   ;
			leadOut.m_dAngle    = m_cParam.m_cLeadOut.m_dAngle    ;
			leadOut.m_dRadius   = m_cParam.m_cLeadOut.m_dRadius   ;
		}
	}
	DOUBLE ZShift[2] = {} ;
	GetLeadCutShift(m_cParam.m_cLeadOut, ZShift);
	// STEP 1 : 提取所有轮廓 , 并生成路径
    DOUBLE dMaxLen = dSideInc * 30 ;
	CSmartSect *pSect  ; 
	CSmartCurve * pBound ;
    int nRCompMask = GetBoundRCompMask() ;
    JDNC_LEAD  rcompLead = m_cFeedDef.m_cLeadDef ;
    rcompLead.m_dLength = max( 0.05, m_cFeedDef.m_cLeadDef.m_dWearLine) ;
    FLOAT fRateScale = 1.0f ;
    if( BoundCut.m_bBoundFlag & NCDEF_FBOUND_FEEDRATIO ) 
    {
        fRateScale = (float)( BoundCut.m_dFeedRatio * 0.01 ) ;
    }
    if( rcompLead.m_nLeadType == 0 ) rcompLead.m_nLeadType = NCDEF_LEAD_SLINE ;
    else   rcompLead.m_nLeadType = NCDEF_LEAD_LINE ;
    DOUBLE rcompZShift[2] = { 0.0, 0.0 } ;
    RemoveNousedLoop( AllLoop ) ;

	// 判断是否保留曲线高度或者从曲线读取速度
	BOOL bKeep[2] = { FALSE, FALSE } ;
	if(  m_cParam.m_bBoundFlag & NCDEF_FBOUND_REDEPTH  )
	{ 
		bKeep[0] = TRUE ;
	}
	
    while( AllLoop.GetSize() > 0 )
    {
        CSmtLoopArr  tmpArr ;
        GetSubLoopGroup( AllLoop, tmpArr ) ;
		if( bSmooth )
			pBound = SmoothConnectLoops( tmpArr, dMaxLen, Contour ) ;
		else
			pBound = ConnectLoopsByLine( tmpArr, dMaxLen, Contour ) ;
        if( ! pBound ) break ;
        pBound->SetAllZValue( 0.0 ) ;
		for( pSect = pBound->GetHead() ; pSect ; pSect = pSect->next )
		{
            if( (pSect->m_bEndFlag & NC_BOUND_SECT) && fRateScale != 1.0f )
            {
                pSect->m_bFeedType = JDNC_FEEDTYPE_SLOT  ;
            }
            else
            {
			    pSect->m_bFeedType = JDNC_FEEDTYPE_ROUGH ;
            }
		}

		// 生成路径
		CPathCombine tmpBase( NC_WPROCESS_ROUGH ) ;
		tmpBase.AppendCurve3D( *pBound , 0.0, TRUE ) ;

		// 添加重复加工路径 20140901 xh
		CSmartCurve overLapped ;
		if (m_cMethodDef.m_nMethodType == surfncMethodLoop)
		{
			double dOverlapLen = m_cMethodDef.m_cLoopCut.m_cLeadOut.m_dOverlap ;//重复加工长度  20140827 xh
			if ( abs(dOverlapLen) > 0.0001)
			{
				PNT3D EndPoint ;
				tmpBase.GetEndPoint( 1, EndPoint );
				GenOverLappedPath( *pBound, overLapped, dOverlapLen);
				tmpBase.AppendCurve3D( overLapped, EndPoint[2], NC_WPROCESS_ROUGH );
				pBound->AddCurve( overLapped.CopyMyself() );
			}
		}
		if( bKeep[0] || bKeep[1])
		{
			RedepthCPathCombine( m_cListCur, &tmpBase, bKeep, LocFrame ) ;
		}
		// 添加进退刀
		PNT2D dPoint[2] ;
		DOUBLE dDist[2] ;
		CSmartLoop* bndCont[2] = { NULL, NULL } ;
		pBound->GetStart( dPoint[0] ) ;
		pBound->GetEnd( dPoint[1] ) ;
		dDist[0] = max( 0.0,MathCAM_MinDistToContour( Contour,dPoint[0] )-0.002) ;
		dDist[1] = max( 0.0,MathCAM_MinDistToContour( Contour,dPoint[1] )-0.002) ;
		bndCont[0] = Contour.OffsetContourEx( dDist[0], dDist[0], m_cSetupDef.m_cCorDef ) ;
		bndCont[1] = Contour.OffsetContourEx( dDist[1], dDist[1], m_cSetupDef.m_cCorDef ) ;
		if( leadIn.m_nLeadType != NCDEF_LEAD_CLOSE || 
			leadOut.m_nLeadType != NCDEF_LEAD_CLOSE )
		{/* 1: 尽可能搜索到切入切出 */
			//        AddLeadSectAtSpiralEnd( leadIn, leadOut, *pBound , bndCont, Depth, ZShift ) ;
			AddLeadPCombAtSpiralEnd( leadIn, leadOut, tmpBase, *pBound, bndCont, Depth, ZShift, bKeep, LocFrame, DepthInc) ;
		} 
		if( nRCompMask != 0 ) 
		{
			//            int bFlag = AddLeadSectAtSpiralEnd( rcompLead, rcompLead, *pBound , bndCont, 0.0, rcompZShift) ;
			int bFlag = AddLeadPCombAtSpiralEnd( rcompLead, rcompLead, tmpBase, *pBound, bndCont, 0.0, rcompZShift, bKeep, LocFrame ) ;
			if (!bFlag)
			{
				m_bRComb = 2;
			}               
		}
		Mini_DeleteContours( bndCont[0] ) ;
		Mini_DeleteContours( bndCont[1] ) ;

		for( CPathEntity* pEnt = tmpBase.m_pHead ; pEnt ; pEnt = pEnt->next )
		{
			if( pEnt->m_bFeedType == JDNC_FEEDTYPE_SLOT )
			{
				pEnt->m_bFeedType = JDNC_FEEDTYPE_ROUGH ;
				pEnt->m_fFeedScale = fRateScale ;
			}
		}
		AddRCompMask( tmpBase, nRCompMask ) ;
		if( Plunge.m_nPlungeType != NCDEF_PLUNGE_CLOSE )
		{ // 增加下刀方式
			CSmartLoop *pChild = NULL ; 
			PNT3D  dPoint, dAtPoint ;
			pBound->GetPoint( 0.0, dPoint ) ;
			dPoint[2]  = 0  ;
			if( Plunge.m_nPlungeType == NCDEF_PLUNGE_VERT  ||
				Plunge.m_nPlungeType == NCDEF_PLUNGE_HELIX   || 
				Plunge.m_nPlungeType == NCDEF_PLUNGE_RAMP    )
			{
				DOUBLE dOffset = Plunge.m_dSideTol ; 
				if( Plunge.m_nPlungeType == NCDEF_PLUNGE_HELIX ) 
				{
					double dDist = MathCAM_MiniDistToContour( Contour, dPoint ) ;
					dOffset += max( Plunge.m_dRadius, dDist) ;
				}
				pChild = Contour.OffsetContourEx( dOffset , dOffset, 
					m_cSetupDef.m_cCorDef ) ;
			}
			CPathCombine cTPlunge( NC_WPROCESS_PLUNGE ) ;
			if( FindPlungePoint(Plunge,  Contour, pChild, dPoint, dAtPoint, pBound ) )
			{ // 在固定位置下刀
				AddPlungePathAt( cTPlunge, Plunge, dPoint, dAtPoint,&Contour ) ;
			}
			if( ! cTPlunge.m_pHead )
			{ // 沿轮廓下刀
				// 进退刀和沿轮廓下刀同时设置时，会出错，因此恢复到2011.12.22蔡锐龙修改前状态，2013.9.30，liuxin
				AddPlungePathBy( cTPlunge, Plunge , dPoint, *pBound, TRUE ) ;
				//ASSERT(tmpArr[0]->m_pCurve != NULL);
				//CSmartCurve * pStartCV = tmpArr[0]->m_pCurve->CopyMyself() ;
				//if( GetMillDir() == NCDEF_MILLDIR_CONVENTIONAL  )
				//	pStartCV->Reverse() ;
				//pStartCV->GetPoint( 0.0, dAtPoint ) ;
				//if (nc_Distance( dAtPoint,dPoint, 3 )  > 2.0e-4)
				//{// 处理连接路径
				//	double dEnd[2], t;
				//	CSmartSect* pSect = pStartCV->MinDistSect( dPoint, dEnd, t ) ;
				//	if (t < 2.0e-4)
				//	{// 在轮廓上，调整起始点即可
				//		pStartCV->SetStartPoint( pSect , dEnd ) ; 
				//	}
				//	else
				//	{// 在轮廓外，需添加过渡路径
				//		CPathCombine  PLeadComb(NC_WPROCESS_PLUNGE);
				//		AddSmoothLeadPathAtPos(dPoint, *pStartCV, 0.0, PLeadComb);
				//		PComb.AppendCombine( PLeadComb ) ;
				//	}
				//}
				//AddPlungePathBy( cTPlunge, Plunge , dPoint, *pStartCV, TRUE ) ;
				//delete pStartCV;
			}
			// 平移下刀路径，防止路径断开, 2013.12.17 liuxin
			{
				tmpBase.GetEndPoint( 0, dPoint ) ;
				cTPlunge.GetEndPoint(1, dAtPoint);
				cTPlunge.MoveCombine(0.0, 0.0, dPoint[2] - dAtPoint[2]);
			}
			PComb.AppendCombine( cTPlunge ) ;
			Mini_DeleteContours( pChild ) ;
		}

        delete pBound ;
        PComb.AppendCombine( tmpBase ) ;
	}
	return 1 ; 
}

int CSmartBoundGen::BoundOneLayerExByContourPlunge(  CPathCombine& PComb   ,  /*路径集合*/
													CSmartLoop&    Contour,  /*边界轮廓*/
													CSmtLoopArr&   AllLoop,  /*所有边界*/
													JDNC_BOUND&  BoundCut ,  /*加工参数*/
													RFRAME	&	LocFrame,	/*局部坐标系*/
													BOOL		   bSmooth	,  /*是否光滑*/
													DOUBLE       Depth    ,
													DOUBLE       DepthInc ,  /*雕刻深度*/  
													DOUBLE	   dSideInc)   /*偏移量  */  
{
	//  STEP 0 : 偏移轮廓, 计算下刀保护边界 
	JDNC_PLUNGE  Plunge = m_cFeedDef.m_cPlungeDef ;
	if( Plunge.m_nPlungeType != NCDEF_PLUNGE_CLOSE )
	{
		Plunge.m_dIncStep = DepthInc + Plunge.m_dTopTol ;
	}
	JDNC_LEAD  leadIn  = m_cFeedDef.m_cLeadDef ;
	JDNC_LEAD  leadOut = m_cFeedDef.m_cLeadDef ;
	if( m_bLoopCut )
		UpdateLeadOutParam( m_cParam.m_cLeadOut, leadOut ) ;
	else
	{
		if( m_cParam.m_cLeadOut.m_bSameLeadIn == FALSE )
		{
			leadOut.m_nLeadType = m_cParam.m_cLeadOut.m_nLeadType ;
			leadOut.m_dLength   = m_cParam.m_cLeadOut.m_dLength   ;
			leadOut.m_dAngle    = m_cParam.m_cLeadOut.m_dAngle    ;
			leadOut.m_dRadius   = m_cParam.m_cLeadOut.m_dRadius   ;
		}
	}
	DOUBLE ZShift[2] = {} ;
	GetLeadCutShift(m_cParam.m_cLeadOut, ZShift);
	// STEP 1 : 提取所有轮廓 , 并生成路径
	DOUBLE dMaxLen = dSideInc * 30 ;
	CSmartSect *pSect  ; 
	CSmartCurve * pBound ;
	int nRCompMask = GetBoundRCompMask() ;
	JDNC_LEAD  rcompLead = m_cFeedDef.m_cLeadDef ;
	rcompLead.m_dLength = max( 0.05, m_cFeedDef.m_cLeadDef.m_dWearLine) ;
	FLOAT fRateScale = 1.0f ;
	if( BoundCut.m_bBoundFlag & NCDEF_FBOUND_FEEDRATIO ) 
	{
		fRateScale = (float)( BoundCut.m_dFeedRatio * 0.01 ) ;
	}
	if( rcompLead.m_nLeadType == 0 ) rcompLead.m_nLeadType = NCDEF_LEAD_SLINE ;
	else   rcompLead.m_nLeadType = NCDEF_LEAD_LINE ;
	DOUBLE rcompZShift[2] = { 0.0, 0.0 } ;
	RemoveNousedLoop( AllLoop ) ;

	// 判断是否保留曲线高度或者从曲线读取速度
	BOOL bKeep[2] = { FALSE, FALSE } ;
	if(  m_cParam.m_bBoundFlag & NCDEF_FBOUND_REDEPTH  )
	{ 
		bKeep[0] = TRUE ;
	}
	
	while( AllLoop.GetSize() > 0 )
	{
		CSmtLoopArr  tmpArr ;
		GetSubLoopGroup( AllLoop, tmpArr ) ;
		if( bSmooth )
			pBound = SmoothConnectLoops( tmpArr, dMaxLen, Contour ) ;
		else
			pBound = ConnectLoopsByLine( tmpArr, dMaxLen, Contour ) ;
		if( ! pBound ) break ;
		pBound->SetAllZValue( 0.0 ) ;
		for( pSect = pBound->GetHead() ; pSect ; pSect = pSect->next )
		{
			if( (pSect->m_bEndFlag & NC_BOUND_SECT) && fRateScale != 1.0f )
			{
				pSect->m_bFeedType = JDNC_FEEDTYPE_SLOT  ;
			}
			else
			{
				pSect->m_bFeedType = JDNC_FEEDTYPE_ROUGH ;
			}
		}

		// 生成路径
		CPathCombine tmpBase( NC_WPROCESS_ROUGH ) ;
		// 沿轮廓下刀，先生成进刀路径，然后再下刀路径，2013.12.06 liuxin
		// 增加下刀方式
		CSmartCurve cPlungeCur;
		PNT3D  dPlungePoint;
		pBound->GetPoint( 0.0, dPlungePoint ) ;
		dPlungePoint[2]  = 0  ;
		CPathCombine cTPlunge( NC_WPROCESS_PLUNGE ) ;
		//AddPlungePathBy( cTPlunge, Plunge , dPlungePoint, *pBound, TRUE ) ;
		// 取第一个环（既偏移方向最外侧环，用于生成下刀路径 qqs 2014.02.20
		CSmartCurve* pPlugBound = tmpArr.GetAt(0)->m_pCurve->CopyMyself();
		AddContourPlungePath(cTPlunge, *pPlugBound, Plunge, dPlungePoint, cPlungeCur, pBound);
		delete pPlugBound ;
		pPlugBound = NULL;

		// 生成沿轮廓下刀成功，关闭沿轮廓进刀
		if (leadIn.m_nLeadType == NCDEF_LEAD_BYCONTOUR && cTPlunge.m_pHead)
		{
			leadIn.m_nLeadType = NCDEF_LEAD_CLOSE;
		}
		tmpBase.AppendCombine(cTPlunge);
		tmpBase.AppendCurve3D( *pBound , 0.0, TRUE ) ;
		// 将下刀添加到曲线中
		cPlungeCur.AppendCurve(*pBound);
		pBound->AppendCurve(cPlungeCur);

		// 添加重复加工路径 20140901 xh
		CSmartCurve overLapped ;
		if (m_cMethodDef.m_nMethodType == surfncMethodLoop)
		{
			double dOverlapLen = m_cMethodDef.m_cLoopCut.m_cLeadOut.m_dOverlap ;//重复加工长度  20140827 xh
			if ( abs(dOverlapLen) > 0.0001)
			{
				PNT3D EndPoint ;
				tmpBase.GetEndPoint( 1, EndPoint );
				GenOverLappedPath( *pBound, overLapped, dOverlapLen);
				tmpBase.AppendCurve3D( overLapped, EndPoint[2], NC_WPROCESS_ROUGH );
				pBound->AddCurve( overLapped.CopyMyself() );
			}
		}

		if( bKeep[0] || bKeep[1])
		{
			RedepthCPathCombine( m_cListCur, &tmpBase, bKeep, LocFrame ) ;
		}
		// 添加进退刀
		PNT2D dPoint[2] ;
		DOUBLE dDist[2] ;
		CSmartLoop* bndCont[2] = { NULL, NULL } ;
		pBound->GetEnd( dPoint[0] ) ;
		pBound->GetEnd( dPoint[1] ) ;
		dDist[0] = max( 0.0,MathCAM_MinDistToContour( Contour,dPoint[0] )-0.002) ;
		dDist[1] = max( 0.0,MathCAM_MinDistToContour( Contour,dPoint[1] )-0.002) ;
		bndCont[0] = Contour.OffsetContourEx( dDist[0], dDist[0], m_cSetupDef.m_cCorDef ) ;
		bndCont[1] = Contour.OffsetContourEx( dDist[1], dDist[1], m_cSetupDef.m_cCorDef ) ;
		if( leadIn.m_nLeadType != NCDEF_LEAD_CLOSE || 
			leadOut.m_nLeadType != NCDEF_LEAD_CLOSE )
		{/* 1: 尽可能搜索到切入切出 */
			//        AddLeadSectAtSpiralEnd( leadIn, leadOut, *pBound , bndCont, Depth, ZShift ) ;
			AddLeadPCombAtSpiralEnd( leadIn, leadOut, tmpBase, *pBound,
									bndCont, Depth, ZShift, bKeep, LocFrame, DepthInc) ;
			//AddLeadPathAtEndEx( leadIn, leadOut, tmpBase, Contour, Depth, ZShift ) ;
		} 
		if( nRCompMask != 0 ) 
		{
			//int bFlag = AddLeadSectAtSpiralEnd( rcompLead, rcompLead, *pBound , bndCont, 0.0, rcompZShift) ;
			int bFlag = AddLeadPCombAtSpiralEnd( rcompLead, rcompLead, tmpBase, *pBound,
												bndCont, 0.0, rcompZShift, bKeep, LocFrame ) ;
			if (!bFlag)
			{
				m_bRComb = 2;
			}               
		}
		Mini_DeleteContours( bndCont[0] ) ;
		Mini_DeleteContours( bndCont[1] ) ;

		for( CPathEntity* pEnt = tmpBase.m_pHead ; pEnt ; pEnt = pEnt->next )
		{
			if( pEnt->m_bFeedType == JDNC_FEEDTYPE_SLOT )
			{
				pEnt->m_bFeedType = JDNC_FEEDTYPE_ROUGH ;
				pEnt->m_fFeedScale = fRateScale ;
			}
		}
		AddRCompMask( tmpBase, nRCompMask ) ;

		delete pBound ;
		PComb.AppendCombine( tmpBase ) ;
	}
	return 1 ; 
}

int CSmartBoundGen::AddSmoothLeadPathAtPos(PNT2D OutPos,
										   CSmartCurve& atCurve,
										   DOUBLE atParam,
										   CPathCombine& LeadPath)
{
	double dEnd[2], dDist, dAtPos = 0.0, d, t;
	CSmartSect* pSect, *pAtSect = atCurve.MinDistSect( OutPos, dEnd, dDist ) ;

	for( pSect = atCurve.m_pHead ; pSect; pSect = pSect->next )
	{
		d = pSect->GetLength() ;
		if( pSect == pAtSect )
		{
			t = pSect->GetParam( dEnd ) ;
			dAtPos += ( t * d ) ;
			break ;
		}
		else
		{
			dAtPos += d ;
		}
	}
	d = atParam * atCurve.GetLength(); 
	if (fabs(d - dAtPos) < 1.0e-3)
	{// 直接连刀
		PNT3D start, end;
		nc_VectorCopy(start, OutPos, 2);
		atCurve.GetPoint(atParam, end);
		start[2] = end[2] = 0.0;
		CPathLine3D  *pLine3D = new CPathLine3D(start, end);
		pLine3D->m_bFeedType = JDNC_FEEDTYPE_CONNECT ;
		LeadPath.AddEntity(pLine3D);
		return 1;
	}

	CSmartCurve * beginPart, *endPart;
	atCurve.BreakCurveAt( dAtPos, beginPart, endPart) ;
	if (beginPart != NULL)
	{
		delete beginPart;
		beginPart = NULL;
	}
	if (endPart == NULL)
	{		
		return 0;
	}
	CSmartCurve *outPart = endPart->OffsetCurve(dDist, m_cSetupDef.m_cCorDef);
	if (outPart == NULL)
	{
		delete endPart;
		return 0;
	}
	PNT2D dStart, dNewPt[2] ;
	outPart->DiscreteCurve( 0.002 ) ;
	outPart->m_pHead->GetStart( dNewPt[0] ) ;
	CSmartCurve cntCurve ;	
	d = outPart->GetLength() ;
	dAtPos = 0.0 ;
	for( pSect = outPart->m_pHead ; pSect;pSect = pSect->next )
	{
		if( pSect->next == NULL ) 
		{
			endPart->m_pTail->GetEnd( dNewPt[1] ) ;
		}
		else 
		{
			dAtPos += pSect->GetLength() ;
			pSect->GetEnd( dStart ) ;
			endPart->MinDistSect( dStart, dEnd , dDist ) ;
			nc_GetPointAtLine( dStart, dEnd , dAtPos / d , dNewPt[1], 2 ) ;
		}
		cntCurve.AddSect( new CSmartLine( dNewPt[0], dNewPt[1]) ) ;
		nc_VectorCopy( dNewPt[0], dNewPt[1], 2 ) ;
	}
	LeadPath.AddCurve( &cntCurve, FALSE ) ;
	cntCurve.ClearAll () ;

	delete outPart;
	delete endPart;	
	return 1;
}

// 为路径添加进退刀曲线,该函数为轮廓切割、修边中调用
void CSmartBoundGen::AddLeadInOutForPComb( CPathCombine &PComb,	CSmartCurve *LeadIn, CSmartCurve *LeadOut ) 
{
	if( !PComb.m_pHead ) return ;
	TPNT3D start, end ;
	
	if(LeadIn && LeadIn->m_pHead )
	{
		CPathCombine tmpLeadIn(JDNC_FEEDTYPE_LEAD);
		LeadIn->SetFeedType ( JDNC_FEEDTYPE_LEAD ) ;
		PComb.m_pHead->GetEndPoint( 0, start ) ;
		tmpLeadIn.AppendCurve3D(*LeadIn, start[2], JDNC_FEEDTYPE_LEAD);
		
		tmpLeadIn.AppendCombine(PComb);
		PComb.AppendCombine(tmpLeadIn);
	}
	if(LeadOut && LeadOut->m_pHead )
	{
		CPathCombine tmpLeadOut(JDNC_FEEDTYPE_LEAD);
		LeadOut->SetFeedType( JDNC_FEEDTYPE_LEAD ) ;
		PComb.m_pTail->GetEndPoint( 1, end ) ;
		tmpLeadOut.AppendCurve3D(*LeadOut, end[2], JDNC_FEEDTYPE_LEAD);
		
		PComb.AppendCombine(tmpLeadOut);
	}  
}


// 为路径添加进退刀曲线,该函数为轮廓切割、修边中调用
void CSmartBoundGen::AddLeadInOutAndOverlapForPComb( CPathCombine &PComb, 
													 CSmartCurve *Overlap,
													 CSmartCurve *LeadIn,
													 CSmartCurve *LeadOut,
													 double Depth, 
													 CPtrList &CurveList,
													 BOOL bKeep[2], 
													 RFRAME &LocFrame) 
{
	if( !PComb.m_pHead ) return ;
	TPNT3D start, end ;
	
	if (Overlap && Overlap->m_pHead)
	{// 将重复加工段加入路径中
		CPathCombine tmpOverlap( NC_WPROCESS_ROUGH ) ;
		//tmpOverlap.AppendCurve3D(*Overlap, -Depth, TRUE);
		tmpOverlap.AppendCurve2D(*Overlap, -Depth, TRUE);
		if( bKeep[0] || bKeep[1] )
		{
			RedepthCPathCombine( CurveList, &tmpOverlap, bKeep, LocFrame);
		}
		PComb.AppendCombine(tmpOverlap);
	}
	if(LeadIn && LeadIn->m_pHead )
	{
		CPathCombine tmpLeadIn(JDNC_FEEDTYPE_LEAD);
		LeadIn->SetFeedType ( JDNC_FEEDTYPE_LEAD ) ;
		PComb.m_pHead->GetEndPoint( 0, start ) ;
		tmpLeadIn.AppendCurve3D(*LeadIn, start[2], JDNC_FEEDTYPE_LEAD);
		if( bKeep[0] && m_cFeedDef.m_cLeadDef.m_nLeadType == NCDEF_LEAD_BYCONTOUR )
		{
			RedepthCPathCombine( CurveList, &tmpLeadIn, bKeep, LocFrame);
		}
		
		tmpLeadIn.GetEndPoint(1, end);
		tmpLeadIn.MoveCombine(0.0, 0.0, start[2] - end[2]);
		tmpLeadIn.AppendCombine(PComb);
		PComb.AppendCombine(tmpLeadIn);
	}
	if(LeadOut && LeadOut->m_pHead )
	{
		CPathCombine tmpLeadOut(JDNC_FEEDTYPE_LEAD);
		LeadOut->SetFeedType( JDNC_FEEDTYPE_LEAD ) ;
		PComb.m_pTail->GetEndPoint( 1, end ) ;
		tmpLeadOut.AppendCurve3D(*LeadOut, end[2], JDNC_FEEDTYPE_LEAD);
		if( bKeep[0] && m_cFeedDef.m_cLeadDef.m_nLeadType == NCDEF_LEAD_BYCONTOUR )
		{
			RedepthCPathCombine( CurveList, &tmpLeadOut, bKeep, LocFrame);
		}

		tmpLeadOut.GetEndPoint(0, start);
		tmpLeadOut.MoveCombine(0.0, 0.0, end[2] - start[2]);
		PComb.AppendCombine(tmpLeadOut);
	}  
}

//////////////////////////////
// 模糊修边
int CSmartBoundGen::BoundAllCutPath( CPathCombine& TComb , 
									 CSmartLoop& Contour , 
									 RFRAME &LocFrame	,
									 DOUBLE Depth         )
{
	DOUBLE dRComp, dAngle ;
	dAngle = ANGLE_TO_RADIAN( m_cShapeDef.m_cTaper.m_dBoundAngle  ) ;
	dRComp= GetSideStock() ;
	CSmartLoop* pChild = Contour.OffsetContourEx( dRComp, dRComp,GetCorDef() ) ; 
	if( ! pChild ) return  FALSE ;
	if( m_cShapeDef.m_bAntiFace )
	{
		JDNC_COR CorDef = GetCorDef() ;
		CorDef.m_nCorType = NCDEF_CORNER_ARCALL ;
		DOUBLE dOffDist[2] ;
		dOffDist[0] = dOffDist[1] = Depth * tan( dAngle ) ;
		if( dRComp > 1.0e-3 )
		{
			CSmartLoop* tmpChild = pChild ;
			pChild = Mini_OffsetContourList( tmpChild, NCDEF_OFFSET_OUTER, dOffDist, CorDef ) ;
			Mini_DeleteContours( tmpChild ) ;
		}
	}
	dRComp= m_pTool->GetRadiusComp( Depth, dAngle)  ;

	BOOL bKeep[2] = { FALSE, FALSE } ;
	if( m_cParam.m_bBoundFlag & NCDEF_FBOUND_REDEPTH )
	{
		bKeep[0] = TRUE ;
	}
	
	for( CSmartLoop* pLoop = pChild ; pLoop ; pLoop = pLoop->next )
	{
		CPathCombine tmpComb( NC_WPROCESS_ROUGH ) ;
		ArtSurf_AllCutContour( tmpComb, *pLoop, 
							 dRComp,  m_cSetupDef ) ;
		// 保留曲线高度
		if( bKeep[0] || bKeep[1])
		{
			RedepthCPathCombine( m_cListCur, &tmpComb, bKeep, LocFrame);
		}
	 	TComb.AppendCombine( tmpComb ) ;
	}
	Mini_DeleteContours( pChild ) ;
    return TRUE ;
}
void CSmartBoundGen::ConnectAllPath( CPathCombine& TComb, CSmartLoop* AllBnd, DOUBLE MaxDist )
{
	TPNT3D dStart , dEnd, dMid ; 
	for( CPathEntity* pEnt = TComb.m_pHead ; pEnt && pEnt->next; pEnt = pEnt->next)
	{
		if( ! pEnt->GetEndPoint( 1, dStart ) || 
			! pEnt->next->GetEndPoint( 0, dEnd   )  )
		{
			continue ;
		}
		double dDist = nc_Distance( dStart, dEnd , 2 ) ;
		if( dDist < 0.01 || dDist > MaxDist ||
            fabs( dStart[2] - dEnd[2]) < 0.01 )
        {
            continue ;
        }
		CSmartLine tmpLine( dStart, dEnd ) ;
        tmpLine.GetPoint( 0.5 , dMid ) ;
		for( CSmartLoop* pCont = AllBnd ; pCont ; pCont = pCont->next )
		{
            if( pCont->IsPtOnContour( dMid ) )
            {
                break ;
            }
        }
        if( pCont == NULL ) continue ;
		for( pCont = AllBnd ; pCont ; pCont = pCont->next )
		{
			if( pCont->IntSectContour( & tmpLine) )
			{
				break ;
			}
		}
		if( pCont ) continue ;
		CPathLine3D *pCnt3D = new CPathLine3D( dStart , dEnd ) ;
		pCnt3D->m_bFeedType = JDNC_FEEDTYPE_CONNECT ;
		
		TComb.InsertAfter( pCnt3D, pEnt ) ;
		pEnt = pCnt3D ;
	}
}

BOOL CSmartBoundGen::GeneratePathEx( CPathGroup& NewPath ,   /*雕刻路径*/ 
		                             CSmartGraphic& Graph)   /*图形函数*/
{
	// STEP 0 : 校正雕刻环境
	if( !UpdateTool2D() ) return FALSE ;

	// STEP -1 : 获取保留曲线高度和速度的原始曲线
	if( m_cParam.m_bBoundFlag & NCDEF_FBOUND_REDEPTH ) 
	{
		Graph.GetAllEntity( m_cSetupDef, m_cListCur, SMARTGRAPH_TYPE_CURVE2D ) ;
	}

	CSmartLoop  *AllCont = Graph.GetAllContour( m_cSetupDef, m_cParam.m_bBoundFlag & NCDEF_FBOUND_REDEPTH ? TRUE : FALSE) ;
	AllCont = RedefineBoundLoops( AllCont ) ;
	if( ! AllCont )
	{ // 校正图形
		m_nErrorType = JDERROR_GENPATH_NOLOOP ;
		return FALSE ;
	}
    MathCAM_SetContoursStart( AllCont, m_cFeedDef.m_cLeadDef,Graph, m_cSetupDef ) ;

	BOOL bRet = CreatPath ( AllCont, NewPath, Graph ) ;

	Mini_DeleteContours( AllCont ) ;

	return bRet ;
}

BOOL CSmartBoundGen::CreatPath ( CSmartLoop *&AllCont, /*由于内部可能改变指针造成内存泄露，因此修改为指针引用 RlCai 2011-11-30*/
								 CPathGroup &NewPath,
								 CSmartGraphic& Graph )
{
	if ( !AllCont )
		return FALSE ;

	CSmartLoop * pContour, *pLoop, *pLpHead  ; 
	
	if( m_cParam.m_bBoundFlag & NCDEF_FBOUND_LOOPOUT )
	{// 轮廓切割向外偏移
	    PNT3D  dBox[2] ; 
		nc_CleanBox3D( dBox ) ;
		for( pContour = AllCont ; pContour ; pContour = pContour->next )
		{
			nc_ClipBox3D(  dBox, pContour->m_dBox ) ;
		}
		dBox[0][0] -= 1.0e6, dBox[0][1] -= 1.0e6 ;
		dBox[1][0] += 1.0e6, dBox[1][1] += 1.0e6 ;
		pContour = new CSmartLoop() ;
        pContour->CreateLoop( dBox[0], dBox[1] ) ;
		pContour->m_bUseFlag  |= NC_LOOP_OUTER ;
		pContour->m_pCurve->SetBlank( TRUE ) ;
		AllCont = Mini_AddContours( AllCont, pContour ) ;
		CSmartLoop LoopTmp;
		AllCont = LoopTmp.BuildContour( AllCont ) ;
	    AllCont = LoopTmp.ResortContour( AllCont, m_cSetupDef.m_cOrderDef.m_nSortType , FALSE) ;
	}
	if( m_cParam.m_bBoundFlag & NCDEF_FBOUND_NOOUTER )
	{ // 忽略外边框
		for( pLoop = AllCont ; pLoop ; pLoop = pLoop->next )
		{
			if( pLoop->m_pCurve && MathCAM_IsOutMostLoop( AllCont, pLoop ) )
			{
				pLoop->m_bUseFlag  |= NC_LOOP_OUTER ;
				pLoop->m_pCurve->SetBlank( TRUE ) ;
			}
		}
	}
	JDNC_LAYER * pLayer   = GetLayerDef() ;
	pLayer->m_dTotalDepth = GetCutDepth()  - m_cStockDef.m_dDepthStock; 
	DefineLayerDepth( * pLayer) ; 
	DOUBLE dSideInc = 0.0;
	int&  nSideTime = m_cSlayerDef.m_nLayerCount;
	if (m_cParam.m_bBoundFlag & NCDEF_FBOUND_LOOPMODE)
	{
		DefineSideLayerInc(m_cSlayerDef.m_cSideDef);// 计算侧向分层
		if (nSideTime > 1)
		{
			dSideInc = m_cSlayerDef.m_dLayerDepth[nSideTime-1]-m_cSlayerDef.m_dLayerDepth[nSideTime-2];
		}
		else
		{
			dSideInc = m_cSlayerDef.m_dLayerDepth[0];
		}
	}
	else
	{
		nSideTime = max( 1, m_cParam.m_nSTolTime) ;
		m_cSlayerDef.m_dLayerDepth = new double[nSideTime+1];
		dSideInc = max( 0.0, m_cParam.m_dSideTol)  / nSideTime ;
		for( int i = 0; i <= nSideTime ; i ++ ) 
		{
			m_cSlayerDef.m_dLayerDepth[i] = i * dSideInc ;
		}
	}
	double dFinishTol = 0.0, dSideIEx = dSideInc;
    if( m_cParam.m_bBoundFlag & NCDEF_FBOUND_FINISHTOL && nSideTime > 1 )
    {
        dFinishTol = max( 0.0, m_cParam.m_dFinishTol ) ;
        dSideIEx   = max( 0.0, m_cParam.m_dSideTol - dFinishTol) / (nSideTime-1) ;
    }
	DOUBLE dDepth , dDepthInc ;
	// STEP 2 : 计算区域修边路径
	CSmartLoop* tmpBndCont = NULL ; 
    CPathCombine * pBTComb = new CPathCombine( NC_WPROCESS_ROUGH ) ;
	DOUBLE  dOffset = 0.0 ;
    BOOL bSmoothCnt = FALSE ;
    if( nSideTime > 1 && 
        !(m_cParam.m_bBoundFlag & NCDEF_FBOUND_ALLCUT) &&
        (m_cSetupDef.m_cModelTol.m_nMdlFlag & NCDEF_SETUP_SMOOTHCONNECT) )
    {// 光滑连刀路径
        bSmoothCnt = TRUE ;
    }

	// STEP 5 : 取得局部坐标系,并且保留曲线高度得到原始曲线信息
	RFRAME LocFrame ;
	if( !Graph.GetTPlaneFrame( LocFrame ) )
	{
		mathInitRFrame( &LocFrame ) ;
	}

	if (m_c5DCtrlDef.m_cToolAxisApp.m_nFrameFlag != NCDEF_FRAME_DEFAULT)
	{
		mathMultRFrame( &LocFrame, &m_c5DCtrlDef.m_cToolAxisApp.m_dIndexFrame, &LocFrame ) ;
	}
	if( m_cParam.m_bBoundFlag & NCDEF_FBOUND_REDEPTH )
	{
		Graph.GetAllEntity( m_cSetupDef, m_cListCur, SMARTGRAPH_TYPE_CURVE2D ) ;
	}

	if( GetLayerOrder() == 1 )
	{ // 高度优先
		for(  int i = 1 ;  i <= pLayer->m_nLayerCount ; i ++ ) 
		{
			dDepth = pLayer->m_dLayerDepth[i] ; 
			dDepthInc = dDepth - pLayer->m_dLayerDepth[i-1] ;
			// 轮廓切割时，支持侧向分层间的连刀，区域修边不支持侧向分层间的连刀，
			// 保证高度优先和区域优先在同一高度的路径相同，2013.9.30，liuxin
			if (m_bLoopCut || bSmoothCnt)
			{
				for( pContour = AllCont ; pContour ; pContour = pContour->next)
				{ 
					CSmartLoop* subLpHead = OffsetPathBound( *pContour, dDepth, 0.0, TRUE ) ; 
					MathCAM_ValidOutMostLoop( subLpHead ) ;
					for( CSmartLoop* pSubCont = subLpHead ; pSubCont ; pSubCont = pSubCont->next )
					{
						CPathCombine tmpTComb( NC_WPROCESS_ROUGH ) ;
						CSmtLoopArr loopArr ;
						int nTime = nSideTime;
						if (!bSmoothCnt)
						{
							--nTime;
						}
						for( ; nTime >= 0 ; nTime -- ) 
						{
							dOffset = m_cSlayerDef.m_dLayerDepth[nTime] ;
							if( m_cParam.m_bBoundFlag & NCDEF_FBOUND_FINISHBTM && pLayer->m_nLayerCount > 1 )// 2013.9.30,liuxin
							{/*最后一层精修*/
								if( i != pLayer->m_nLayerCount && m_cParam.m_bBoundFlag & NCDEF_FBOUND_FINISHTOL ) 
								{ // 如果未开启定义精修量，偏移量上不增加精修量 qqs 2014.03.04
									dOffset += m_cParam.m_dFinishTol ;
								}
							}
							else if( dFinishTol > 0.002 )
							{/*最后一层精修*/
								if( nTime == 0 ) dOffset = 0.0 ;
								else dOffset = dFinishTol + (nTime-1) * dSideIEx ;
							}
							pLpHead = pSubCont->OffsetContourEx( dOffset,dOffset,GetCorDef() ) ; 
							pLpHead = pSubCont->ResortContour( pLpHead , m_cSetupDef.m_cOrderDef.m_nSortType , FALSE ) ;
							MathCAM_ValidOutMostLoop( pLpHead ) ;
							for( pLoop = pLpHead ; pLoop ; pLoop = pLoop->next )
							{
								pLoop->SetContourDepth( nSideTime - nTime ) ;
								if( nTime == 0 ) pLoop->SetBoundInfo( TRUE ) ;
								else pLoop->SetBoundInfo( FALSE ) ;							
								pLoop->ExtractAllLoop( m_cSetupDef.m_cOrderDef.m_nSortType, loopArr ) ;
							}
							tmpBndCont = Mini_AddContours( tmpBndCont, pLpHead ) ;
						}
						BoundOneLayerEx( tmpTComb, *pSubCont, loopArr, m_cParam, LocFrame, bSmoothCnt, dDepth, dDepthInc, dSideInc) ;					
						if( tmpTComb.m_pHead ) 
						{
							tmpTComb.MoveCombine( 0.0, 0.0, -dDepth ) ;
							pBTComb->AppendCombine( tmpTComb ) ;
						}
					}
					Mini_DeleteContours( subLpHead ) ;
				}
			}
			else
			{
				for( pContour = AllCont ; pContour ; pContour = pContour->next)
				{ 
				    CPathCombine tmpTComb( NC_WPROCESS_ROUGH ) ;
					for( int nTime = nSideTime-1 ; nTime >= 0 ; nTime -- ) 
					{
						if( (m_cParam.m_bBoundFlag & NCDEF_FBOUND_ALLCUT) && nTime == 0)
						{// 模糊修边
							BoundAllCutPath( tmpTComb, *pContour, LocFrame, dDepth ) ; 
						} 
						else 
						{ // 普通修边
							dOffset = m_cSlayerDef.m_dLayerDepth[nTime] ;
				            if( m_cParam.m_bBoundFlag & NCDEF_FBOUND_ALLCUT )
				            {// 模糊修边
				            }
							else if( m_cParam.m_bBoundFlag & NCDEF_FBOUND_FINISHBTM && pLayer->m_nLayerCount > 1 )// 2013.9.30,liuxin
				            {/*最后一层精修*/
				                if( i != pLayer->m_nLayerCount && m_cParam.m_bBoundFlag & NCDEF_FBOUND_FINISHTOL ) 
								{ // 如果未开启定义精修量，偏移量上不增加精修量 qqs 2014.03.04
									dOffset += m_cParam.m_dFinishTol ;
								}
				            }
				            else if( dFinishTol > 0.002 )
				            {/*最后一层精修*/
				                if( nTime == 0 ) dOffset = 0.0 ;
				                else dOffset = dFinishTol + (nTime-1) * dSideIEx ;
				            }
							int nR = 0 ;
							pLpHead = OffsetPathBoundEx( *pContour, dDepth , dOffset, TRUE, &nR ) ; 
							MathCAM_ValidOutMostLoop( pLpHead ) ;
						    for( pLoop = pLpHead ; pLoop ; pLoop = pLoop->next )
							{
								if( nTime == 0 ) pLoop->SetBoundInfo( TRUE ) ;
								else pLoop->SetBoundInfo( FALSE ) ;
								BoundOneLayer( tmpTComb, *pLoop, m_cParam, LocFrame, dDepth, dDepthInc, nR );
							}    
							tmpBndCont = Mini_AddContours( tmpBndCont, pLpHead ) ;
						}
					}
					if( tmpTComb.m_pHead )
					{
						tmpTComb.MoveCombine( 0.0, 0.0, -dDepth );
						pBTComb->AppendCombine( tmpTComb ) ;
					}
				}
			}
		}
	}
    else 
	{ // 区域优先
		for( pContour = AllCont ; pContour ; pContour = pContour->next)
		{ 
			for(  int i = 1 ;  i <= m_cFeedDef.m_cLayerDef.m_nLayerCount ; i ++ ) 
			{
				dDepth = pLayer->m_dLayerDepth[i] ; 
				dDepthInc = dDepth - pLayer->m_dLayerDepth[i-1] ;
				if( m_bLoopCut || bSmoothCnt)
				{
					CSmartLoop* subLpHead = OffsetPathBound( *pContour, dDepth, 0.0, TRUE ) ; 
					MathCAM_ValidOutMostLoop( subLpHead ) ;
					for( CSmartLoop* pSubCont = subLpHead ; pSubCont ; pSubCont = pSubCont->next )
					{
						CPathCombine tmpTComb( NC_WPROCESS_ROUGH ) ;
						CSmtLoopArr loopArr ;
						int nTime = nSideTime;
						if (!bSmoothCnt)
						{
							--nTime;
						}
						for( ; nTime >= 0 ; nTime -- ) 
						{
							dOffset = m_cSlayerDef.m_dLayerDepth[nTime] ;
							if( m_cParam.m_bBoundFlag & NCDEF_FBOUND_FINISHBTM && pLayer->m_nLayerCount > 1 )// 2013.9.30,liuxin
							{/*最后一层精修*/
								if( i != pLayer->m_nLayerCount && m_cParam.m_bBoundFlag & NCDEF_FBOUND_FINISHTOL ) 
								{ // 如果未开启定义精修量，偏移量上不增加精修量 qqs 2014.03.04
									dOffset += m_cParam.m_dFinishTol ;
								}
							}
							else if( dFinishTol > 0.002 )
							{/*最后一层精修*/
								if( nTime == 0 ) dOffset = 0.0 ;
								else dOffset = dFinishTol + (nTime-1) * dSideIEx ;
							}
							pLpHead = pSubCont->OffsetContourEx( dOffset,dOffset,GetCorDef() ) ; 
							pLpHead = pSubCont->ResortContour( pLpHead , m_cSetupDef.m_cOrderDef.m_nSortType , FALSE ) ;
							MathCAM_ValidOutMostLoop( pLpHead ) ;
							for( pLoop = pLpHead ; pLoop ; pLoop = pLoop->next )
							{
								pLoop->SetContourDepth( nSideTime - nTime ) ;
								if( nTime == 0 ) pLoop->SetBoundInfo( TRUE ) ;
								else pLoop->SetBoundInfo( FALSE ) ;							
								pLoop->ExtractAllLoop( m_cSetupDef.m_cOrderDef.m_nSortType, loopArr ) ;
							}
							tmpBndCont = Mini_AddContours( tmpBndCont, pLpHead ) ;
						}
						BoundOneLayerEx( tmpTComb, *pSubCont, loopArr, m_cParam, LocFrame, bSmoothCnt, dDepth, dDepthInc, dSideInc) ;
						if( tmpTComb.m_pHead ) 
						{
							tmpTComb.MoveCombine( 0.0, 0.0, -dDepth ) ;
							pBTComb->AppendCombine( tmpTComb ) ;
						}
					}
					Mini_DeleteContours( subLpHead ) ;
				}
				else
				{
					CPathCombine tmpTComb( NC_WPROCESS_ROUGH ) ;
					for( int nTime = nSideTime-1 ; nTime >= 0 ; nTime -- ) 
					{
						if( (m_cParam.m_bBoundFlag & NCDEF_FBOUND_ALLCUT) && nTime == 0 )
						{ //模糊雕刻
						   BoundAllCutPath( tmpTComb, *pContour, LocFrame, dDepth ) ; 
						} 
						else
						{//普通修边 
							dOffset = m_cSlayerDef.m_dLayerDepth[nTime] ;
							if( m_cParam.m_bBoundFlag & NCDEF_FBOUND_ALLCUT )
							{// 模糊修边
							}
							else if( m_cParam.m_bBoundFlag & NCDEF_FBOUND_FINISHBTM && pLayer->m_nLayerCount > 1)// 2013.9.30,liuxin
							{//最后一层精修
								if( i != pLayer->m_nLayerCount && m_cParam.m_bBoundFlag & NCDEF_FBOUND_FINISHTOL ) 
								{ // 如果未开启定义精修量，偏移量上不增加精修量 qqs 2014.03.04
									dOffset += m_cParam.m_dFinishTol ;
								}
							}
							else if( dFinishTol > 0.002 )
							{//最后一层精修
								if( nTime == 0 ) dOffset = 0.0 ;
								else dOffset = dFinishTol + (nTime-1) * dSideIEx ;
							}
							int nR = 0 ;
							pLpHead = OffsetPathBoundEx( *pContour, dDepth, dOffset, TRUE, &nR ) ; 
							MathCAM_ValidOutMostLoop( pLpHead ) ;
							for( pLoop = pLpHead ; pLoop ; pLoop = pLoop->next )
							{
								if( nTime == 0 ) pLoop->SetBoundInfo( TRUE ) ;
								else pLoop->SetBoundInfo( FALSE ) ;
								BoundOneLayer( tmpTComb, *pLoop, m_cParam, LocFrame, dDepth, dDepthInc, nR );
							}
							tmpBndCont = Mini_AddContours( tmpBndCont, pLpHead ) ;
						}
					}
					if( tmpTComb.m_pHead ) 
					{
						tmpTComb.MoveCombine( 0.0, 0.0, -dDepth ) ;
						pBTComb->AppendCombine( tmpTComb ) ;
					}
				}
			}
		}
	}
	if( m_cFeedDef.m_cLayerDef.m_nLayerCount && tmpBndCont &&
		( m_cParam.m_bBoundFlag & NCDEF_LAYER_KEEPDOWN ) )
	{
		ConnectAllPath( *pBTComb, tmpBndCont, max( 5.0*m_cToolDef.m_dTopDiam, 10.0) ) ; 
	}
    Mini_DeleteContours( tmpBndCont ) ;
	
	if( pBTComb->m_pHead ) NewPath.AddData(  0.0, pBTComb ) ;
	else delete pBTComb ;

	if( pLayer->m_dLayerDepth ) 
	{
		delete[] pLayer->m_dLayerDepth ;
		pLayer->m_dLayerDepth = NULL ;
	}
	if( m_cSlayerDef.m_dLayerDepth ) 
	{
		delete[] m_cSlayerDef.m_dLayerDepth;
		m_cSlayerDef.m_dLayerDepth = NULL;
	}

	// STEP 3 : 转换路径
	if( ! NewPath.m_pHead )
	{
		m_nErrorType = JDERROR_GENPATH_NOAREA ;
		return FALSE ; 
	}

	if( ! ( m_cParam.m_bBoundFlag & NCDEF_FBOUND_REDEPTH ) )
	{// 校正表面高度
		NewPath.ZMoveGroup( GetTopHeight() ) ;
	}
	
	return TRUE ;
}


/////////////////////////////////////////////
CSmartExpendGen::CSmartExpendGen()
{
}
CSmartExpendGen::~CSmartExpendGen()
{
}
BOOL CSmartExpendGen::GeneratePathEx( CPathGroup& NewPath ,  /*雕刻路径*/ 
                                      CSmartGraphic& Graph)  /*图形函数*/
{
	if( !UpdateTool2D() ) return FALSE ;

 	
	// STEP 1 : 提取加工图形
	JDNC_LAYER * pLayer   = GetLayerDef() ;
	pLayer->m_dTotalDepth = GetCutDepth()  -m_cStockDef.m_dDriveZMove[0]; 
	DefineLayerDepth( * pLayer) ; 
	DOUBLE dAngle = ANGLE_TO_RADIAN( m_cShapeDef.m_cTaper.m_dBoundAngle ) ;
	if( dAngle < m_pTool->m_fAngle  ) dAngle = m_pTool->m_fAngle ;
	if( dAngle < ANGLE_TO_RADIAN( 3 ) )
	{
		dAngle = ANGLE_TO_RADIAN( 3 ) ;
	}
    DOUBLE fTanAngle = tan( dAngle ) ;
	CPtrList   AllCurve  ;
	BOOL bDepth = FALSE ;
	int bEntType = SMARTGRAPH_TYPE_CURVE ;
    if( m_cParam.m_bCornerFlag & NCDEF_CORNER_REDEPTH ) 
	{
		bEntType = SMARTGRAPH_TYPE_CURVE3D ;
		bDepth = TRUE ;
	}
	Graph.GetAllEntity( m_cSetupDef, AllCurve, bEntType  ) ;

	BOOL bAllowRev = FALSE ;
	SmartMill_SortAllCurve( AllCurve , GetSortType(), bAllowRev, bDepth ) ;
	if( AllCurve.GetCount() == 0  )
	{ // 校正图形
		m_nErrorType = JDERROR_GENPATH_NOCURVE ;
		return FALSE ;
	}
	// STEP 2 : 偏移加工区域
    double  dTopHeight = m_dTopHeight ;
    int nGroupID = 1 ;
    CSmartLoop tmpLoop ;
    CSmartLoop*  AllCont = NULL, *pLp ; 
    CSmartCurve* pCurve ;
	double dBtm = m_pTool->GetRadiusComp(0.0,0.0);
    double  dTMaxBtm = dBtm ;
    if( m_pTool->GetType() == smtToolABall || 
        m_pTool->GetType() == smtToolANose || 
        m_pTool->GetType() == smtToolNose  || 
        m_pTool->GetType() == smtToolBall   )
	{// 校正圆角刀
        DOUBLE dCorRad = m_pTool->m_fCorner ;
        if( m_pTool->GetType() == smtToolBall ) 
        {
            dCorRad = m_pTool->m_fRadius ;
        }
	    double dChord  = dCorRad * ( 1.0 - sin( dAngle ) ) ;
        dTMaxBtm += ( dCorRad * cos( dAngle ) - dChord * fTanAngle) ;
	}
    if( m_cParam.m_bCornerFlag & NCDEF_CORNER_REDEPTH )
    {
        double dZAt , dBtmRad ;
        dBtmRad = dTMaxBtm + GetSideStock() - NCDEF_OFFSET_BTM;
        double dZMove = 0.0 ;
        if( dBtmRad > 0.0 ) dZMove = dBtmRad / tan( dAngle ) ;
        POSITION atPos = AllCurve.GetHeadPosition() ;
        while( atPos )
        {
            pCurve =  (CSmartCurve*)AllCurve.GetNext( atPos ) ;
            pCurve->m_dDepth = pCurve->m_pHead->m_dZValue[0] + dZMove - dTopHeight ;
            dZAt = min( 0.0, pCurve->m_dDepth );
            pCurve->SetAllZValue( dZAt ) ;
        }
        while( AllCurve.GetCount() )
        {
            POSITION atPos = AllCurve.GetHeadPosition() ;
            POSITION atObj = NULL ;
            CSmartCurve* pCvObj = NULL ;
            while( atPos )
            {/*查找最高的曲线*/
                pCurve =  (CSmartCurve*)AllCurve.GetAt( atPos ) ;
                if( pCvObj == NULL || 
                    pCurve->m_pHead->m_dZValue[0] > pCvObj->m_pHead->m_dZValue[0] )
                {
                    pCvObj = pCurve ;
                    atObj = atPos ;
                }
                AllCurve.GetNext( atPos ) ;
            }
            AllCurve.RemoveAt( atObj ) ;
            CSmartCurveLib tmpCLib ;
            tmpCLib.AddTail( pCvObj ) ;
            dZAt = pCvObj->m_pHead->m_dZValue[0] ;
            atPos = AllCurve.GetHeadPosition() ;
            while( atPos )
            {/*寻找高度相同的曲线*/
                atObj = atPos ;
                pCurve =  (CSmartCurve*)AllCurve.GetNext( atPos ) ;
                if( fabs(pCurve->m_pHead->m_dZValue[0] - dZAt ) < 0.001 )
                {
                    AllCurve.RemoveAt( atObj ) ;
                    tmpCLib.AddTail( pCurve ) ;
                }
            }
            if( AllCont != NULL ) 
            {
                CSmartLoop* pTrimCont = MathCAM_BuildTrimContours( AllCont, dZAt, dAngle, m_cSetupDef.m_cCorDef ) ;
                MathCAM_TrimCurveByContours( pTrimCont, tmpCLib ) ;
                Mini_DeleteContours( pTrimCont ) ;
            }
            CSmartLoop* pTmpHead = NULL ;
            nGroupID = 1;
            for( pCurve = tmpCLib.m_pHead ; pCurve ; pCurve = pCurve->next )
            {
                dBtmRad = max( 0.0, pCurve->m_dDepth ) * tan( dAngle ) + NCDEF_OFFSET_BTM ; 
                CSmartLoop* pLpHead =  pCurve->ExpendCurveArcEnd( dBtmRad, m_cSetupDef.m_cCorDef ) ;
                for( pLp = pLpHead ; pLp ; pLp = pLp->next )
                {
                    pLp->m_nGroupId = nGroupID ;
                    for( CSmartLoop* pIsl = pLp->m_pIsland ; pIsl ; pIsl = pIsl->next )
                    {
                        pIsl->m_nGroupId = nGroupID ;
                    }
                }
                nGroupID ++ ;
                pTmpHead = Mini_AddContours( pTmpHead, pLpHead ) ;
            }
            pTmpHead = tmpLoop.WeldContours( pTmpHead ) ;
            MathCAM_SetContoursHeight( pTmpHead, dZAt ) ;
            AllCont = Mini_AddContours( AllCont, pTmpHead ) ;
        }
    }
    else 
    {/*不保留曲线高度*/
        DOUBLE dBtmRad = dTMaxBtm + GetSideStock();
        if( dBtmRad < NCDEF_OFFSET_BTM )  dBtmRad = NCDEF_OFFSET_BTM;
        while( AllCurve.GetCount() ) 
        {
            CSmartCurve* pCurve =  (CSmartCurve*)AllCurve.RemoveHead() ;
            CSmartLoop* pLpHead =  pCurve->ExpendCurveArcEnd( dBtmRad, m_cSetupDef.m_cCorDef ) ;
            for( pLp = pLpHead ; pLp ; pLp = pLp->next )
            {
                pLp->m_nGroupId = nGroupID ;
                for( CSmartLoop* pIsl = pLp->m_pIsland ; pIsl ; pIsl = pIsl->next )
                {
                    pIsl->m_nGroupId = nGroupID ;
                }
            }
            AllCont = Mini_AddContours( AllCont, pLpHead ) ;
            nGroupID ++ ;
            delete pCurve ;
        }
        AllCont = tmpLoop.WeldContours( AllCont ) ;
        MathCAM_SetContoursHeight( AllCont, 0.0 ) ;
    }
    if( AllCont == NULL ) 
    {
        return FALSE ;
    }
	// STEP 3 : 构建清角模型
	DOUBLE fZScale   = 1.0 / fTanAngle ;
	DOUBLE dFrom , dTo ;
    for( CSmartLoop* pLp = AllCont ; pLp ; pLp = pLp->next )
    {
        double dZAt =  pLp->m_dHeight  ;
        pLp->m_dHeight  = dZAt * fTanAngle ;
        for( CSmartLoop* pIsl = pLp->m_pIsland ; pIsl ; pIsl = pIsl->next )
        {
            pIsl->m_dHeight  = dZAt * fTanAngle ;
        }
    }
    CSmtVoronoiMap  tmpVMap  ;
    tmpVMap.m_bCenAll = FALSE  ; 
    tmpVMap.SetTolAndCorner( m_cSetupDef.m_cTolDef, m_cSetupDef.m_cCorDef ) ;
    tmpVMap.UpdateCurveLoop( AllCont , pLayer->m_dTotalDepth * tan( dAngle) ) ;
    tmpVMap.DefineBound( VORONOI_CORSIDE ) ; 
//	Mini_DeleteContours( AllCont ) ; // 放到后面删除，用于减少抬刀 2014.3.5 liuxin
	// STEP 4 : 计算分层路径
	CVoronoiBound *pBndHead , *pBndTail , *pBnd, *pCorBnd ;
    pBndHead = pBndTail = NULL ; 
	for(  int i = 1 ;  i <= pLayer->m_nLayerCount ; i ++ ) 
	{
		if( pLayer->m_bLayerFlag & NCDEF_LAYER_COPYMODE )
		{ /*Z向平移清角*/
			dFrom = ( pLayer->m_dTotalDepth - pLayer->m_dLayerDepth[i]) * fTanAngle ;
			dTo   = ( pLayer->m_dLayerDepth[i] ) * fTanAngle + dFrom;
		}
		else
		{
			dFrom  = pLayer->m_dLayerDepth[i-1] * fTanAngle ;
			dTo    = pLayer->m_dLayerDepth[i] * fTanAngle;
		}
		tmpVMap.DefineCornerCurve( dTo + tmpVMap.m_dZeroHeight, -1.0 ) ; 
		pCorBnd = tmpVMap.GetCornerBound( dFrom, dTo + 0.1, -fZScale, FALSE ) ;
		if( ! pBndHead ) pBndHead = pBndTail = pCorBnd  ;
		else pBndTail->next = pCorBnd ;
		while( pBndTail && pBndTail->next ) pBndTail = pBndTail->next ;
        for( pBnd = pCorBnd ; pBnd ; pBnd = pBnd->next )
		{  
			pBnd->m_nLayer = i ;
			if( !( pLayer->m_bLayerFlag & NCDEF_LAYER_COPYMODE) )
			{
				pBnd->MoveBound( 0.0, 0.0, -pLayer->m_dLayerDepth[i-1]  ) ;
			}
			if( (GetMillDir() == 0 )  )
			{  /*逆铣走刀 */
		        pBnd->Reverse() ;
			} 
		}
	}
    tmpVMap.ClearFacet() ;
	CPathCombine * pTComb = new CPathCombine( NC_WPROCESS_ROUGH ) ;
	double dTol = pLayer->m_dLayerDepth[1] * fTanAngle + NCDEF_OFFSET_BTM;
	if( m_pTool->GetType() == smtToolABall || 
		m_pTool->GetType() == smtToolANose || 
		m_pTool->GetType() == smtToolNose  || 
		m_pTool->GetType() == smtToolBall   )
	{// 校正圆角刀
		dTol = dTMaxBtm - dBtm + dTol;
	}
    pBndHead = Mini_SortVoronoiBound( pBndHead, /*dTMaxBtm*/ dTol ) ;
	while( pBndHead )
	{
		pBnd = pBndHead ; 
		pBndHead = pBndHead->next   ; 
		Wizard_AddCornerPath( *pTComb        , 
			        pBnd->m_dPoint , 
			        pBnd->m_nCount , 
					-pLayer->m_dLayerDepth[pBnd->m_nLayer], 
					m_cSetupDef ) ; 
		delete pBnd ;
	}
	if( m_cFeedDef.m_cPlungeDef.m_nPlungeType != 0 )
	{// 在下刀位置重设起点edit by liuxin 2013/8/1
		ReDefinePathStartPntAtPlungePos(pTComb, m_cFeedDef.m_cPlungeDef);
	}
	if (pLayer->m_bLayerFlag & NCDEF_LAYER_KEEPDOWN)
	{// 加入上下层间的连刀路径 edit by liuxin 2013/8/1
		AddLinkPathBetweenLayers(pTComb, AllCont, dAngle);
	}
	Mini_DeleteContours( AllCont ) ;
    if( m_cFeedDef.m_cPlungeDef.m_nPlungeType != 0 )
    {/*增加垂直下刀*/
        JDNC_PLUNGE tmpPlunge = m_cFeedDef.m_cPlungeDef ;
        tmpPlunge.m_nPlungeType = NCDEF_PLUNGE_VERT ;
        tmpPlunge.m_dIncStep = pLayer->m_dSideDInc  + tmpPlunge.m_dTopTol ;
		//if( tmpPlunge.m_nPositionType == NCDEF_PLUNGEPOS_REFER )
		//	CreateVertPlungeAtPos( *pTComb, tmpPlunge ) ;
		//else
			InsertVertPlunge( *pTComb, tmpPlunge ) ;
    }
	NewPath.AddData( 0.0, pTComb, TRUE ) ;
	if( pLayer->m_dLayerDepth ) delete[] pLayer->m_dLayerDepth ;
	pLayer->m_dLayerDepth = NULL ;
    NewPath.ZMoveGroup( dTopHeight ) ;
	return TRUE ;
}
//
CSmartCornerGen::CSmartCornerGen()
{
}
CSmartCornerGen::~CSmartCornerGen()
{
}


/////////////////////////////
// 计算一层清角加工路径
int CSmartCornerGen::CornerOneLayer(CPathCombine& PComb     ,  /*路径集合*/
		                            CSmtVoronoiMap&  Voronoi,  /*清角模型*/
									JDNC_CORNER&  CornerCut ,  /*加工参数*/
									int  Layer              ,  /*层号    */  
									DOUBLE        dSideAngle)  /*侧面角度 */  
{
	// STEP 1 : 计算清角路径
	DOUBLE dDepth = m_cFeedDef.m_cLayerDef.m_dLayerDepth[ Layer ] ;
	DOUBLE dDepthInc = dDepth -  m_cFeedDef.m_cLayerDef.m_dLayerDepth[ Layer-1 ];
	CPathCombine  tmpComb(0)  ;
	// STEP 2 : 设置反向标志
	DOUBLE fTanAngle = tan( dSideAngle ) ;
	DOUBLE fZScale   = 1.0 / fTanAngle ;
	DOUBLE dFrom , dTo , dSideInc = 0.0 ;
	DOUBLE dRemainDist = -1.0 ;
	int nSideTime = CornerCut.m_nSTolTime ;
	if(nSideTime < 1 )	nSideTime = 1 ;
	if( nSideTime > 1 ) dSideInc = CornerCut.m_dSideTol / nSideTime ;
	if( dSideInc < 0.0 ) dSideInc = 0.0 ;
	if( CornerCut.m_nCornerType == NCDEF_CORNER_REMAIN )
	{  /* 残料清角 */
		dRemainDist = CornerCut.m_dRemainDist ;
		if( dRemainDist < 0.0 ) dRemainDist = 0.0 ; 
	}
    for( int nTime = nSideTime-1 ;  nTime >= 0 ; nTime -- ) 
	{
		if( m_cFeedDef.m_cLayerDef.m_bLayerFlag & NCDEF_LAYER_COPYMODE )
		{ /*Z向平移清角*/
			int nAt = m_cFeedDef.m_cLayerDef.m_nLayerCount - Layer ;
			dFrom = m_cFeedDef.m_cLayerDef.m_dLayerDepth[ nAt] * fTanAngle + dSideInc * nTime ;
			dTo   = dFrom + dDepth*fTanAngle  ;
			Voronoi.DefineCornerCurve( dTo + Voronoi.m_dZeroHeight, -1.0 ) ; 
		} 
		else
		{ /*常规清角*/
			dFrom  = (dDepth-dDepthInc) * fTanAngle + nTime * dSideInc;
			dTo    =  dDepth*fTanAngle + nTime * dSideInc  ;
			Voronoi.DefineCornerCurve( dTo + Voronoi.m_dZeroHeight, dRemainDist ) ; 
		}
	    CVoronoiBound* pCorBnd = Voronoi.GetCornerBound( dFrom, dTo + 0.1, -fZScale, FALSE ) ;
	    while( pCorBnd )
		{ 
			CVoronoiBound* pBnd = pCorBnd ; 
		    pCorBnd = pCorBnd->next   ; 
			Wizard_AddCornerPath( tmpComb        , 
				                  pBnd->m_dPoint , 
				                  pBnd->m_nCount , 
								  dFrom - dTo    , 
								  m_cSetupDef ) ; 
			delete pBnd ;
		}
	}
	// STEP 5 : 添加路径
	if( !( m_cFeedDef.m_cLayerDef.m_bLayerFlag & NCDEF_LAYER_COPYMODE) )
	{
		tmpComb.MoveCombine( 0.0, 0.0, - dDepth + dDepthInc ) ;
	}
	if( (GetMillDir() == 0 ) &&  
	   ( !(CornerCut.m_bCornerFlag & NCDEF_CORNER_NODOWN ) ) )
	{  /*逆铣走刀 && 禁止向向下*/
//		tmpComb.ReverseDirect() ;
		ReversePComb( tmpComb ) ;
	}
    PComb.AppendCombine( tmpComb ) ;
	return TRUE ;
}
// 计算清角加工路径
int CSmartCornerGen::CreateCornerPath( CPathCombine&  TComb , /* 路径组  */
						               CSmartLoop&   Contour, /* 轮廓组  */
								       JDNC_CORNER&  Corner , /* 清角参数*/  
									   DOUBLE        Angle  ) /* 角度    */  
{
	CVoronoiFacet *pFacet ;
	CVoronoiBound *pBound ; 
	double dScale , dDepthBtm ;
	DOUBLE dSideAngle ;
    CPathPLine3D *pCurve ;
    PNT3D    fBuffer[4000] ;
    UINT      nCount = 0   ;
	CSmtVoronoiMap    Voronoi ;
    Voronoi.m_bCenAll   = FALSE  ; 
	Voronoi.SetTolAndCorner( GetCurveTol(), GetCorDef() ) ;
	dSideAngle = Angle ;
	dScale = tan( dSideAngle ) ; 
	Voronoi.m_bCenAll = FALSE ;
    Voronoi.UpdateContour( Contour, Corner.m_dCutDepth * dScale ) ;
	Voronoi.DefineBound( VORONOI_UNDEFINE ) ; 
    dDepthBtm = Corner.m_dCutDepth * dScale ; 
	for( pFacet = Voronoi.GetFacetHead() ; pFacet ; pFacet = pFacet->next )
	{
		if( (Contour.m_bUseFlag & NC_LOOP_OUTER ) && 
			Corner.m_bCornerFlag & NCDEF_CORNER_NOBOUND && 
			pFacet->m_nLoop == Contour.m_nDepth ) 
		{
		}
		pBound  = pFacet->ExtractBound(VORONOI_BOUND_CORNER, Voronoi.m_dCosBnd ) ;
		if( !pBound )
		{
			continue ; 
		}
		for( int i = 0 ; i <= pBound->m_nCount ; i ++  )
		{
			memcpy( fBuffer[nCount], pBound->m_dPoint[i], sizeof(double[2]) ) ;
			fBuffer[nCount][2] = -pBound->m_dPoint[i][2]/dScale ;
			if( ( nCount && ( Corner.m_bCornerFlag & NCDEF_CORNER_NODOWN ) && fBuffer[nCount][2] >  fBuffer[nCount-1][2] ) || 
				( nCount && ( Corner.m_bCornerFlag & NCDEF_CORNER_CORONLY) && fBuffer[nCount][2] == fBuffer[nCount-1][2] && 
				  pBound->m_dPoint[i][2] == dDepthBtm   )  )
			{ // 禁止向下走刀 || 只加工清角
				if( nCount > 1 )
				{
					pCurve = new CPathPLine3D() ;
					pCurve->Copy( fBuffer, nCount-1 ) ;
					TComb.AddEntity( pCurve )  ; 
				}
				memcpy( fBuffer[0], fBuffer[nCount], sizeof(PNT3D) ) ;
				nCount = 0 ;
			}
			if( nCount == 0 ) 	 nCount ++ ;
			else if( i == 0  && nc_Distance( fBuffer[ nCount-1], fBuffer[ nCount ],3 ) > 0.001 )
			{ // 新线段
				 pCurve = new CPathPLine3D() ;
				 pCurve->Copy( fBuffer, nCount-1 ) ;
				 TComb.AddEntity( pCurve )  ; 
			    memcpy( fBuffer[0], fBuffer[nCount], sizeof(PNT3D) ) ;
				 nCount = 1 ;
			}
 			else if( nCount == 3999 )
			{ 
				 pCurve = new CPathPLine3D() ;
				 pCurve->Copy( fBuffer, nCount ) ;
				 TComb.AddEntity( pCurve )  ; 
			     memcpy( fBuffer[0], fBuffer[nCount], sizeof(PNT3D) ) ;
				 nCount = 1 ;
			}
            else if( nc_Distance( fBuffer[ nCount-1], fBuffer[ nCount], 3 ) < 0.001 ) 
				 continue ;
			else nCount ++ ;
		}
		delete pBound ; 
	}
    if( nCount >= 2 )
	{
		 pCurve = new CPathPLine3D() ;
		 pCurve->Copy( fBuffer, nCount-1 ) ;
		 TComb.AddEntity( pCurve )  ; 
	}
    CPathEntity * pEntity = TComb.m_pHead ;
    while( pEntity )
	{
		if( pEntity->GetType() == NC_PATH_PLINE3D )
		{ 
		  pCurve = ( CPathPLine3D *) pEntity ;
//		  pCurve->RefineCornerPath( 0.001, dSideAngle ) ;
		} 
	    pEntity = pEntity->next ;
	} 
	return TRUE ; 
}

// 计算单个区域的清角路径
BOOL  CSmartCornerGen::CFuzzyOneLayer( CPathCombine& TComp,  /* 增加路径 */
								       CSmartLoop& Contour,  /* 轮廓曲线 */ 
									   JDNC_LAYER* pLayer ,  /* 分层     */ 
								       int    AtLayer     ,  /* 层号     */
									   DOUBLE Angle,         /* 侧面角度 */
									   DOUBLE SideTol  )     /* 侧面进给 */ 
{
	// STEP 0 : 计算清角路径
	DOUBLE dDepth , dDepthInc, dRComp ;
    dDepth    = pLayer->m_dLayerDepth[AtLayer-1];   // 和其他的平面雕刻的深度不同（其他的为pLayer->m_dLayerDepth[i]）
	dDepthInc = pLayer->m_dLayerDepth[AtLayer] - dDepth ;
	dRComp = m_pTool->GetRadiusComp( dDepth, Angle ) + SideTol;
	CSmartLoop *pChild ,*pLoop ;
	CSmartSurfList SurfList ;
	CPathCombine  *tmpComb  ;
	double dDComp = dRComp/tan(Angle) ;
	SurfList.m_cSetup = m_cSetupDef ;
	if( m_cParam.m_nCornerType == NCDEF_CORNER_ALLCUT && AtLayer == 1 )
	{
		SurfList.InitSurfList( &Contour, Angle, dDepthInc + dDComp) ;
		DOUBLE dMinDepth = m_cParam.m_dMinDepth ;
		if( dMinDepth > dDepthInc ) dMinDepth = dDepthInc ;
        tmpComb = SurfList.GenClearPath( m_cParam.m_bCornerFlag, dDComp, TRUE, dMinDepth ) ;
	    TComp.AppendCombine( tmpComb ) ;
	}
	else 
	{
		pChild = Contour.OffsetContourEx(dRComp, dRComp, GetCorDef() ) ;
		pChild = Contour.ResortContour( pChild, NCDEF_SORTTYPE_MINDIST , FALSE ) ;
		if( Contour.m_bUseFlag & NC_LOOP_OUTER )
		{
			MathCAM_ValidOutMostLoop( pChild ) ;
		}
 	    for( pLoop = pChild ; pLoop ; pLoop = pLoop->next )
		{
			tmpComb = NULL ;
		    CSmartSect * pSect = pChild->GetSectHead() ;
		    if( !pChild->GetIsland() && pSect && 
				! pSect->next && pSect->IsSectArc()   )
			{ 
				PNT3D   dCenter, dStart ;
				CSmartArc* pArc = ( CSmartArc*) pSect ;
				dCenter[0] = pArc->m_aCenter[0] ;
				dCenter[1] = pArc->m_aCenter[1] ;
				dCenter[2] = -pArc->m_dRadius / tan( Angle)  ;
				pArc->GetPoint( 0.0, dStart ) ;
				dStart[2] = 0.0 ;
				if( fabs(dCenter[2]) < dDepthInc )
				{
		            tmpComb = new CPathCombine( NC_WPROCESS_ROUGH ) ;
				    CPathLine3D *pLine = new CPathLine3D( dStart, dCenter ) ;
				    tmpComb->AddEntity(pLine ) ;
				}
			} 
			else
			{
				SurfList.InitSurfList( pLoop, Angle,dDepthInc ) ;
                tmpComb = SurfList.GenClearPath( m_cParam.m_bCornerFlag, 0.0, FALSE, 0.0 ) ;
			}
			if( ! tmpComb ) continue ;
	        TComp.AppendCombine( tmpComb ) ;
		}
		Mini_DeleteContours( pChild ) ;
	}
	TComp.MoveCombine( 0.0, 0.0, -dDepth ) ;
	return TRUE ;
}

// 模糊清角
BOOL CSmartCornerGen::FuzzyCornerPath( CPathGroup& NewPath   , 
									   CSmartLoop* AllCont   ,
									   JDNC_LAYER* pLayer    ,
									   DOUBLE      Angle )
{
	CSmartLoop* pContour ;
	int  i, nBoundTime = 1  ;
	DOUBLE dSideInc = 0.0 ;
	if( m_cParam.m_nSTolTime > 1 && 
		m_cParam.m_nCornerType == NCDEF_CORNER_BOUND )
	{
	   dSideInc = m_cParam.m_dSideTol / m_cParam.m_nSTolTime ;
	   nBoundTime = m_cParam.m_nSTolTime ;
	}
	for( int nTime = nBoundTime-1 ; nTime >= 0 ; nTime -- )
	{
		if( GetLayerOrder() == 1 && AllCont && AllCont->next)
		{ // 高度优先
			for( i = 1 ; i <= pLayer->m_nLayerCount; i ++  )
			{
				for( pContour = AllCont ; pContour ; pContour = pContour->next )
				{ // 按照轮廓优先逐层清角
					CPathCombine * pTComb = new CPathCombine( NC_WPROCESS_ROUGH ) ;
					CFuzzyOneLayer( *pTComb, *pContour, pLayer, i, Angle, dSideInc*nTime ) ;
					if( pTComb->m_pHead ) NewPath.AddData( 0.0, pTComb ) ;
					else delete pTComb ;
				}   
			} 
		} 
		else
		{ // 区域优先
			for( pContour = AllCont ; pContour ; pContour = pContour->next )
			{ // 按照轮廓优先逐层清角
				CPathCombine tmpPComb(NC_WPROCESS_ROUGH);
				for( i = 1 ; i <= pLayer->m_nLayerCount; i ++  )
				{
					CPathCombine * pTComb = new CPathCombine( NC_WPROCESS_ROUGH ) ;
					CFuzzyOneLayer( *pTComb, *pContour, pLayer, i, Angle, dSideInc*nTime) ;
					if (pLayer->m_bLayerFlag & NCDEF_LAYER_KEEPDOWN)
					{// 加入上下层间的连刀路径 edit by liuxin 2013/8/1
						AddLinkPathBetweenLayers(pTComb, AllCont, Angle);
					}
					if( pTComb->m_pHead )
						tmpPComb.AppendCombine(pTComb) ;
					else 
						delete pTComb ;
				}
				if (tmpPComb.m_pHead)
				{
					CPathCombine * pTComb = new CPathCombine( NC_WPROCESS_ROUGH ) ;
					pTComb->AppendCombine(tmpPComb);
					NewPath.AddData(0.0, pTComb);
				}
			}
		}
	}
	return TRUE ;
}
// 残料清角
BOOL CSmartCornerGen::CBoundOneLayer( CPathCombine& TComb, 
									 CSmtVoronoiMap& Voronoi,
									 JDNC_LAYER*  pLayer, 
									 int i,
									 DOUBLE Angle )
{
	DOUBLE fTanAngle = tan( Angle ) ;
	DOUBLE fZScale   = 1.0 / fTanAngle ;
	DOUBLE dFrom , dTo ;
	if( pLayer->m_bLayerFlag & NCDEF_LAYER_COPYMODE )
	{ /*Z向平移清角*/
		dFrom = ( pLayer->m_dTotalDepth - pLayer->m_dLayerDepth[i]) * fTanAngle ;
		dTo   = ( pLayer->m_dLayerDepth[i] ) * fTanAngle + dFrom;
	}
	else
	{
		dFrom  = pLayer->m_dLayerDepth[i-1] * fTanAngle ;
		dTo    = pLayer->m_dLayerDepth[i] * fTanAngle;
	}
	// STEP 2 : 设置反向标志
	Voronoi.DefineCornerCurve( dTo + Voronoi.m_dZeroHeight, -1.0 ) ; 
    CVoronoiBound* pCorBnd = Voronoi.GetCornerBound( dFrom, dTo + 0.1, -fZScale, FALSE ) ;
    while( pCorBnd )
	{ 
	    CVoronoiBound* pBnd = pCorBnd ; 
		pCorBnd = pCorBnd->next   ; 
		Wizard_AddCornerPath( TComb        , 
			                  pBnd->m_dPoint , 
			                  pBnd->m_nCount , 
							  dFrom - dTo , 
							  m_cSetupDef ) ; 
		delete pBnd ;
	}
	if( !( pLayer->m_bLayerFlag & NCDEF_LAYER_COPYMODE) )
	{
		TComb.MoveCombine( 0.0, 0.0, - pLayer->m_dLayerDepth[i-1] ) ;
	}
	if( (GetMillDir() == 0 ) &&  
	   ( !(m_cParam.m_bCornerFlag & NCDEF_CORNER_NODOWN ) ) )
	{  /*逆铣走刀 && 禁止向向下*/
//		TComb.ReverseDirect() ;
		ReversePComb( TComb ) ;
	}
	return TRUE ;
}
// 修边清角
BOOL CSmartCornerGen::BoundCornerPath( CPathGroup& NewPath, 
									   CSmartLoop* AllCont,
									   JDNC_LAYER* pLayer,
									   DOUBLE      Angle    )
{
    // STEP 1 : 计算偏移值
    int bOffType = NCDEF_OFFSET_INNER ;
    DOUBLE dOffDist[2] ={ 0.0, 0.0 } ;
    dOffDist[0] = dOffDist[1] = GetSideStock() + m_pTool->GetRadiusComp( 0.0, 0.0 );
    if( m_cShapeDef.m_bAntiFace ) 
    {
	    double dTotalDepth = GetCutDepth()  ;
	    double dRComp = tan( Angle ) * dTotalDepth;
        dOffDist[0] -= dRComp , dOffDist[1] -= dRComp ;
    }
    if( dOffDist[0] < 0.0 )
    {
        bOffType = NCDEF_OFFSET_OUTER ;
        dOffDist[0] = dOffDist[1] = - dOffDist[0] ;
    }
    CSmartLoop* pLpHead = NULL , *pContour; 
	int nCount = 0 , nID = 1 ;
    for( pContour = AllCont ; pContour ; pContour = pContour->next )
    {
		CSmartLoop* pLpNew = pContour->OffsetContour( bOffType, dOffDist[0],dOffDist[1], GetCorDef() ) ;
		pLpHead = Mini_AddContours( pLpHead, pLpNew ) ;
		for( ; pLpNew ; pLpNew = pLpNew->next ) 
        {
            pLpNew->m_nGroupId = nID ;
            nCount ++ ;
        }
		nID ++ ;
    }
    if( pLpHead == NULL ) return FALSE ;
    CSmartLoop tmpLoop ; 
    pLpHead = tmpLoop.ResortContour( pLpHead, m_cSetupDef.m_cOrderDef.m_nSortType, FALSE ) ;
    MathCAM_ValidOutMostLoop( pLpHead ) ;
	// STEP 2 : 计算清角深度
	DOUBLE dSideInc = 0.0 , dRComp , dDepth ;
	if( m_cParam.m_nSTolTime > 1 )
	{
	   dSideInc = m_cParam.m_dSideTol / m_cParam.m_nSTolTime ;
	}
	dRComp = (m_cParam.m_nSTolTime-1) * dSideInc ;
	dDepth = pLayer->m_dTotalDepth * tan( Angle) + dRComp ;
	// STEP 3 : 构建清角模型
	CSmtVoronoiMap * pVoronoiList = new CSmtVoronoiMap[ nCount + 1 ] ;
	nCount =  0 ;
	for( pContour = pLpHead ; pContour ; pContour = pContour->next )
	{
       pVoronoiList[nCount].m_bCenAll = FALSE  ; 
       pVoronoiList[nCount].SetTolAndCorner( m_cSetupDef.m_cTolDef, m_cSetupDef.m_cCorDef ) ;
	   pVoronoiList[nCount].UpdateContour( *pContour, dDepth )  ;
	   pVoronoiList[nCount].m_dZeroHeight = 0.0 ;
	   pVoronoiList[nCount].DefineBound( VORONOI_CORSIDE ) ; 
	   nCount ++ ; 
	}
	// STEP 3 : 逐次计算清角路径
	CVoronoiBound *pBndHead , *pBndTail , *pBnd, *pCorBnd ;
	DOUBLE fTanAngle = tan( Angle ) ;
	DOUBLE fZScale   = 1.0 / fTanAngle ;
	DOUBLE dFrom , dTo ;
	for( int nBoundTime = m_cParam.m_nSTolTime-1 ; nBoundTime >= 0 ; nBoundTime -- )
	{
		dRComp = nBoundTime * dSideInc ;
	    for( int nLoop = 0 ; nLoop < nCount ; nLoop ++ ) 
		{
			pVoronoiList[nLoop].m_dZeroHeight = dRComp;
		}
	    if( GetLayerOrder() == 1 && nCount > 1)
		{ // 高度优先
			for(  int i = 1 ;  i <= pLayer->m_nLayerCount ; i ++ ) 
			{
				for( nLoop = 0 ; nLoop < nCount ; nLoop ++ ) 
				{
					CPathCombine * pTComb = new CPathCombine( NC_WPROCESS_ROUGH ) ;
					CBoundOneLayer( *pTComb, pVoronoiList[nLoop],pLayer, i, Angle ) ;
					if( m_cFeedDef.m_cPlungeDef.m_nPlungeType != 0 )
					{// 在下刀位置重设起点edit by liuxin 2013/8/1
						ReDefinePathStartPntAtPlungePos(pTComb, m_cFeedDef.m_cPlungeDef);
					}
					if( pTComb->m_pHead ) NewPath.AddData( 0.0, pTComb ) ;
					else delete pTComb ;
				} 
			}
		}
	    else 
		{ // 区域优先
		    for( int nLoop = 0 ; nLoop < nCount ; nLoop ++ ) 
			{
				pBndHead = pBndTail = NULL ;  
				for(  int i = 1 ;  i <= pLayer->m_nLayerCount ; i ++ ) 
				{
					if( pLayer->m_bLayerFlag & NCDEF_LAYER_COPYMODE )
					{ /*Z向平移清角*/
						dFrom = ( pLayer->m_dTotalDepth - pLayer->m_dLayerDepth[i]) * fTanAngle ;
						dTo   = ( pLayer->m_dLayerDepth[i] ) * fTanAngle + dFrom;
					}
					else
					{
						dFrom  = pLayer->m_dLayerDepth[i-1] * fTanAngle ;
						dTo    = pLayer->m_dLayerDepth[i] * fTanAngle;
					}
					pVoronoiList[nLoop].DefineCornerCurve( dTo + pVoronoiList[nLoop].m_dZeroHeight, -1.0 ) ; 
					pCorBnd = pVoronoiList[nLoop].GetCornerBound( dFrom, dTo + 0.1, -fZScale, FALSE ) ;
					if( ! pBndHead ) pBndHead = pBndTail = pCorBnd  ;
					else pBndTail->next = pCorBnd ;
					while( pBndTail && pBndTail->next ) pBndTail = pBndTail->next ;
                    for( pBnd = pCorBnd ; pBnd ; pBnd = pBnd->next )
					{  
						pBnd->m_nLayer = i ;
						if( !( pLayer->m_bLayerFlag & NCDEF_LAYER_COPYMODE) )
						{
							pBnd->MoveBound( 0.0, 0.0, -pLayer->m_dLayerDepth[i-1]  ) ;
						}
						if( (GetMillDir() == 0 )  )
						{  /*逆铣走刀 */
		                    pBnd->Reverse() ;
						} 
					}
				}
			    CPathCombine * pTComb = new CPathCombine( NC_WPROCESS_ROUGH ) ;
				// 2014.1.6 liuxin
				double dTol = pLayer->m_dLayerDepth[1] * fTanAngle + NCDEF_OFFSET_BTM;
				if( m_pTool->GetType() == smtToolABall || 
					m_pTool->GetType() == smtToolANose || 
					m_pTool->GetType() == smtToolNose  || 
					m_pTool->GetType() == smtToolBall   )
				{// 校正圆角刀
					DOUBLE dCorRad = m_pTool->m_fCorner ;
					if( m_pTool->GetType() == smtToolBall ) 
					{
						dCorRad = m_pTool->m_fRadius ;
					}
					double dChord  = dCorRad * ( 1.0 - sin( Angle ) ) ;
					dTol += ( dCorRad * cos( Angle ) - dChord * fTanAngle) ;
				}
	            pBndHead = Mini_SortVoronoiBound( pBndHead, /*m_pTool->GetRadiusComp( 0.0, 0.0 )*/dTol ) ;
	            while( pBndHead )
				{
					pBnd = pBndHead ; 
		            pBndHead = pBndHead->next   ; 
		            Wizard_AddCornerPath( *pTComb        , 
			                  pBnd->m_dPoint , 
			                  pBnd->m_nCount , 
							  -pLayer->m_dLayerDepth[pBnd->m_nLayer], 
							  m_cSetupDef ) ; 
		            delete pBnd ;
				}
				if( m_cFeedDef.m_cPlungeDef.m_nPlungeType != 0 )
				{// 在下刀位置重设起点edit by liuxin 2013/8/1
					ReDefinePathStartPntAtPlungePos(pTComb, m_cFeedDef.m_cPlungeDef);
				}
				if (pLayer->m_bLayerFlag & NCDEF_LAYER_KEEPDOWN)
				{// 加入上下层间的连刀路径 edit by liuxin 2013/8/1
					AddLinkPathBetweenLayers(pTComb, AllCont, Angle);
				}
                if( pTComb->m_pHead ) NewPath.AddData( 0.0, pTComb ) ;
                else delete pTComb ;
			}
		}
	}
    delete[] pVoronoiList ;
    Mini_DeleteContours( pLpHead ) ;
	return TRUE ;
}
// 残料清角
BOOL CSmartCornerGen::CRemainOneLayer( CPathCombine& TComb, 
									   CSmtVoronoiMap& Voronoi,
									   JDNC_LAYER*  pLayer, 
									   int i,
									   DOUBLE Angle )
{
	DOUBLE fTanAngle = tan( Angle ) ;
	DOUBLE fZScale   = 1.0 / fTanAngle ;
	DOUBLE dFrom , dTo ;
	if( pLayer->m_bLayerFlag & NCDEF_LAYER_COPYMODE )
	{ /*Z向平移清角*/
		dFrom = ( pLayer->m_dTotalDepth - pLayer->m_dLayerDepth[i]) * fTanAngle ;
		dTo   = ( pLayer->m_dLayerDepth[i] ) * fTanAngle + dFrom;
	}
	else
	{
		dFrom  = pLayer->m_dLayerDepth[i-1] * fTanAngle ;
		dTo    = pLayer->m_dLayerDepth[i] * fTanAngle;
	}
	// STEP 2 : 设置反向标志
	Voronoi.DefineCornerCurve( dTo + Voronoi.m_dZeroHeight, m_cParam.m_dRemainDist ) ; 
	BOOL bNoDown = m_cParam.m_bCornerFlag & NCDEF_CORNER_NODOWN ? TRUE : FALSE ;
    CVoronoiBound* pCorBnd = Voronoi.GetCornerBound( dFrom, dTo + 0.1, -fZScale, bNoDown ) ;
    while( pCorBnd )
	{ 
	    CVoronoiBound* pBnd = pCorBnd ; 
		pCorBnd = pCorBnd->next   ; 
		Wizard_AddCornerPath( TComb        , 
			                  pBnd->m_dPoint , 
			                  pBnd->m_nCount , 
							  dFrom - dTo , 
							  m_cSetupDef ) ; 
		delete pBnd ;
	}
	if( !( pLayer->m_bLayerFlag & NCDEF_LAYER_COPYMODE) )
	{
		TComb.MoveCombine( 0.0, 0.0, - pLayer->m_dLayerDepth[i-1] ) ;
	}
	if( (GetMillDir() == 0 ) &&  
	   ( !(m_cParam.m_bCornerFlag & NCDEF_CORNER_NODOWN ) ) )
	{  /*逆铣走刀 && 禁止向向下*/
//		TComb.ReverseDirect() ;
		ReversePComb( TComb ) ;
	}
	return TRUE ;
}
// 残料清角
BOOL CSmartCornerGen::CRemainAllLayer( CPathCombine& TComb, 
									   CSmtVoronoiMap& Voronoi,
									   JDNC_LAYER*  pLayer, 
									   DOUBLE Angle )
{
	CVoronoiBound *pBndHead , *pBndTail , *pBnd, *pCorBnd ;
	DOUBLE fTanAngle = tan( Angle ) ;
	DOUBLE fZScale   = 1.0 / fTanAngle ;
	DOUBLE dFrom , dTo ;
	BOOL bNoDown = m_cParam.m_bCornerFlag & NCDEF_CORNER_NODOWN ? TRUE : FALSE ;
	pBndHead = pBndTail = NULL ; 
	for( int i = 1 ; i <= pLayer->m_nLayerCount ; i ++ ) 
	{
		if( pLayer->m_bLayerFlag & NCDEF_LAYER_COPYMODE )
		{ /*Z向平移清角*/
		   dFrom = ( pLayer->m_dTotalDepth - pLayer->m_dLayerDepth[i]) * fTanAngle ;
		   dTo   = ( pLayer->m_dLayerDepth[i] ) * fTanAngle + dFrom;
		} 
	    else
		{
			dFrom  = pLayer->m_dLayerDepth[i-1] * fTanAngle ;
		    dTo    = pLayer->m_dLayerDepth[i] * fTanAngle;
		} 
	    // STEP 2 : 设置反向标志
	    Voronoi.DefineCornerCurve( dTo + Voronoi.m_dZeroHeight, m_cParam.m_dRemainDist ) ; 
        pCorBnd = Voronoi.GetCornerBound( dFrom, dTo+0.1, -fZScale, bNoDown ) ;
		if( ! pBndHead ) pBndHead = pBndTail = pCorBnd  ;
		else pBndTail->next = pCorBnd ;
		while( pBndTail && pBndTail->next ) pBndTail = pBndTail->next ;
	    for( pBnd = pCorBnd ;  pBnd ; pBnd = pBnd->next )
		{
			pBnd->m_nLayer = i ;
			if( !( pLayer->m_bLayerFlag & NCDEF_LAYER_COPYMODE) )
			{
				pBnd->MoveBound( 0.0, 0.0, -pLayer->m_dLayerDepth[i-1]  ) ;
			}
			if( (GetMillDir() == 0 ) &&  
			   ( !(m_cParam.m_bCornerFlag & NCDEF_CORNER_NODOWN ) ) )
			{  /*逆铣走刀 && 禁止向向下*/
				pBnd->Reverse() ;
			}
		}
	}
	//double dTol = m_pTool->GetRadiusComp( 0.0, 0.0 ) ;
	// 2014.1.6 liuxin
	double dTol = pLayer->m_dLayerDepth[1] * fTanAngle + NCDEF_OFFSET_BTM;
	if( m_pTool->GetType() == smtToolABall || 
		m_pTool->GetType() == smtToolANose || 
		m_pTool->GetType() == smtToolNose  || 
		m_pTool->GetType() == smtToolBall   )
	{// 校正圆角刀
		DOUBLE dCorRad = m_pTool->m_fCorner ;
		if( m_pTool->GetType() == smtToolBall ) 
		{
			dCorRad = m_pTool->m_fRadius ;
		}
		double dChord  = dCorRad * ( 1.0 - sin( Angle ) ) ;
		dTol += ( dCorRad * cos( Angle ) - dChord * fTanAngle) ;
	}
	pBndHead = Mini_SortVoronoiBound( pBndHead, dTol ) ;
	while( pBndHead )
	{
	    pBnd = pBndHead ; 
		pBndHead = pBndHead->next   ; 
		Wizard_AddCornerPath( TComb        , 
			                  pBnd->m_dPoint , 
			                  pBnd->m_nCount , 
							  -pLayer->m_dLayerDepth[pBnd->m_nLayer], 
							  m_cSetupDef ) ; 
		delete pBnd ;
	}

	return TRUE ;
}
// 残料清角
BOOL CSmartCornerGen::RemainCornerPath( CPathGroup& NewPath   , 
									    CSmartLoop* AllCont   ,
										JDNC_LAYER* pLayer    ,
										DOUBLE      Angle )
{
	DOUBLE dRComp = m_pTool->GetRadiusComp( 0.0, 0.0 ) ;
	DOUBLE dDepth = pLayer->m_dTotalDepth * tan( Angle) + dRComp ;
	m_cParam.m_dRemainDist = m_cParam.m_dLastDiam/2.0f - (float)dRComp ;
	if( m_cParam.m_dRemainDist  < 0.0 ) m_cParam.m_dRemainDist = 0.0 ; 
		dDepth += m_cParam.m_dRemainDist ;
	// STEP 2 : 构建清角模型
	CSmartLoop* pContour ;
	int nCount = 0 ;
	for( pContour = AllCont ; pContour ; pContour = pContour->next )
			nCount ++ ;
	CSmtVoronoiMap * pVoronoiList = new CSmtVoronoiMap[ nCount + 1 ] ;
	nCount =  0 ;
	for( pContour = AllCont ; pContour ; pContour = pContour->next )
	{
       pVoronoiList[nCount].m_bCenAll = FALSE  ; 
	   pVoronoiList[nCount].SetTolAndCorner( GetCurveTol(),GetCorDef() ) ;
	   pVoronoiList[nCount].UpdateContour( *pContour, dDepth )  ;
	   pVoronoiList[nCount].m_dZeroHeight = dRComp ;
	   pVoronoiList[nCount].DefineBound( VORONOI_CORSIDE ) ; 
	   nCount ++ ; 
	}
	// STEP 3 : 逐次计算清角路径
	if( GetLayerOrder() == 1 )
	{ // 高度优先
		for(  int i = 1 ;  i <= pLayer->m_nLayerCount ; i ++ ) 
		{
			for( int nLoop = 0 ; nLoop < nCount ; nLoop ++ ) 
			{
				CPathCombine * pTComb = new CPathCombine( NC_WPROCESS_ROUGH ) ;
				CRemainOneLayer( *pTComb, pVoronoiList[nLoop],pLayer, i,Angle ) ;
				if( pTComb->m_pHead ) NewPath.AddData(0.0, pTComb ) ;
				else delete pTComb ;
			} 
		}
	}
	else 
	{ // 区域优先
		for( int nLoop = 0 ; nLoop < nCount ; nLoop ++ ) 
		{
			CPathCombine * pTComb = new CPathCombine( NC_WPROCESS_ROUGH ) ;
			CRemainAllLayer( *pTComb, pVoronoiList[nLoop], pLayer, Angle ) ;
			if( pTComb->m_pHead ) NewPath.AddData( 0.0, pTComb ) ;
			else delete pTComb ;
		}
	}
    delete[] pVoronoiList ;
	return TRUE ;
}
// 计算清角路径
BOOL CSmartCornerGen::GeneratePathEx( CPathGroup& NewPath  ,   /*雕刻路径*/ 
		                              CSmartGraphic& Graph)   /*图形函数*/
{
	// STEP 1 : 校正雕刻环境
	if( !UpdateTool2D() ) return FALSE ;

    // STEP 2 : 获得轮廓边界 
	int nOldFit = m_cSetupDef.m_cTolDef.m_nFitType ;
    m_cSetupDef.m_cTolDef.m_nFitType = NCDEF_FITTYPE_LINE ; // 直线轮廓
	CSmartLoop* AllCont = Graph.GetAllContour( m_cSetupDef ) ;
	AllCont = RedefineBoundLoops( AllCont ) ;
    m_cSetupDef.m_cTolDef.m_nFitType = nOldFit ; 
	if(  !AllCont ) 
	{/* 校正图形 */
		m_nErrorType = JDERROR_GENPATH_NOLOOP ;
		return FALSE ;
	}
	BOOL bRet = CreatPath ( AllCont, NewPath ) ;
		
	Mini_DeleteContours(  AllCont ) ;
	return bRet ;
}

BOOL CSmartCornerGen::CreatPath ( CSmartLoop* AllCont,
								  CPathGroup& NewPath )
{
	if ( !AllCont )
		return FALSE ;

	if( m_cParam.m_bCornerFlag & NCDEF_CORNER_NOBOUND )
	{/* 标记外边框轮廓 */
		CSmartLoop* pLoop ;
		for( pLoop = AllCont ; pLoop ; pLoop = pLoop->next )
		{
			if( pLoop->m_pCurve && MathCAM_IsOutMostLoop( AllCont, pLoop ) )
			{
				pLoop->m_bUseFlag  |= NC_LOOP_OUTER ;
				pLoop->m_pCurve->SetBlank( TRUE ) ;
			}
		}
	}
    // STEP 3 : 计算分层方式
	DOUBLE dAngle = ANGLE_TO_RADIAN( m_cShapeDef.m_cTaper.m_dBoundAngle ) ;
	if( dAngle < m_pTool->m_fAngle  ) dAngle = m_pTool->m_fAngle ;
	if( dAngle < ANGLE_TO_RADIAN( 3 ) )
	{
		LPSTR lpBuffer = glbf_GetPromptString() ;
		glbf_LoadSysString ( IDS_BOUNDCORNER_TOOLANGLEERROR, lpBuffer, MAX_STR_LENG ) ;
		AfxMessageBox( lpBuffer );
		dAngle = ANGLE_TO_RADIAN( 3 ) ;
	}
	JDNC_LAYER * pLayer   = GetLayerDef() ;
	pLayer->m_dTotalDepth = GetCutDepth()  - m_cStockDef.m_dDepthStock; 
	DefineLayerDepth( * pLayer) ; 
	if( m_cParam.m_nCornerType == NCDEF_CORNER_BOUND )
	{ /* 清角修边  */
		BoundCornerPath( NewPath, AllCont, pLayer, dAngle ) ;
	}
    else
    {
		CSmartLoop *AllLoop = NULL ;
        int bOffType = NCDEF_OFFSET_INNER ;
        DOUBLE dOffDist[2] = { 0.0, 0.0 }, bNewLoop = FALSE ;
        dOffDist[0] = dOffDist[1] = GetSideStock() ;
        if( m_cShapeDef.m_bAntiFace ) 
	    {
	        double dRComp = tan( dAngle ) * GetCutDepth() ;
            dOffDist[0] -= dRComp, dOffDist[1] -= dRComp  ;
        }
        if( fabs( dOffDist[0]) > 0.0002f )
        {
            if( dOffDist[0] < 0.0 )
            {
                bOffType = NCDEF_OFFSET_OUTER ;
                dOffDist[0] = dOffDist[1] = - dOffDist[0] ;
            }
	        int nID = 1 ;
	        CSmartLoop* pLpHead = NULL , *pContour, *pLpNew ; 
            for( pContour = AllCont ; pContour ; pContour = pContour->next )
            {
		        pLpNew = pContour->OffsetContour( bOffType, dOffDist[0],dOffDist[1], GetCorDef() ) ;
		        pLpHead = Mini_AddContours( pLpHead, pLpNew ) ;
		        for( ; pLpNew ; pLpNew = pLpNew->next ) pLpNew->m_nGroupId = nID ;
		        nID ++ ;
            }
            AllLoop = pLpHead ;
			bNewLoop = TRUE ;
        }
		else
		{
			AllLoop = AllCont ;
		}
		MathCAM_ValidOutMostLoop( AllLoop ) ;
	    if( m_cParam.m_nCornerType == NCDEF_CORNER_ALLCUT )
	    { /* 模糊清角*/
		    FuzzyCornerPath( NewPath, AllLoop, pLayer, dAngle ) ;
	    }
	    else 
	    { /* 清角补加工*/
		    RemainCornerPath( NewPath, AllLoop, pLayer, dAngle ) ;
	    }
		if( AllLoop && bNewLoop )
            Mini_DeleteContours( AllLoop ) ;
    }
	
	// STEP 3 : 转换路径
	if( ! NewPath.m_pHead )
	{
		m_nErrorType = JDERROR_GENPATH_NOAREA ;
		if( pLayer->m_dLayerDepth ) 
		{
			delete[] pLayer->m_dLayerDepth ;
			pLayer->m_dLayerDepth = NULL ;
		}
		return FALSE ; 
	}
	if( m_cFeedDef.m_cPlungeDef.m_nPlungeType != 0 )
    {/*增加垂直下刀*/
        JDNC_PLUNGE tmpPlunge = m_cFeedDef.m_cPlungeDef ;
        tmpPlunge.m_nPlungeType = NCDEF_PLUNGE_VERT ;
        tmpPlunge.m_dIncStep = pLayer->m_dSideDInc  + tmpPlunge.m_dTopTol ;
        for( CPathCombine *pTComb = NewPath.m_pHead ; pTComb ; pTComb = pTComb->next )
        {
			//if( tmpPlunge.m_nPositionType == NCDEF_PLUNGEPOS_REFER)
			//	CreateVertPlungeAtPos( *pTComb, tmpPlunge ) ;
			//else
				InsertVertPlunge( *pTComb, tmpPlunge ) ;
        }
    }
	// 校正表面高度
	NewPath.ZMoveGroup( GetTopHeight() ) ;
	if( pLayer->m_dLayerDepth ) 
	{
		delete[] pLayer->m_dLayerDepth ;
		pLayer->m_dLayerDepth = NULL ;
	}

	return TRUE ;
}

CSmartCurveSlotGen::CSmartCurveSlotGen()
{
}
CSmartCurveSlotGen::~CSmartCurveSlotGen()
{
}

BOOL CSmartCurveSlotGen::GeneratePathEx(CPathGroup& NewPath       ,  /*雕刻路径*/ 
                                        CSmartGraphic& Graph)  /*图形函数*/
{
	// STEP 1 : 提取加工图形
	if( !UpdateTool2D() ) return FALSE ;
	CPtrList   AllCurve ;
	CPtrList   AllComb ;
	int bEntType = SMARTGRAPH_TYPE_CURVE ;
    Graph.GetAllEntity( m_cSetupDef, AllCurve, bEntType  ) ;
	if( AllCurve.GetCount() == 0 )
	{ // 校正图形
		m_nErrorType = JDERROR_GENPATH_NOCURVE ;
		return FALSE ;
	}
    JDNC_SPIRAL spiralDef ;
    SurfNC_InitPathParam( spiralDef ) ;
    CSmartNcSpiral ncSpiral( m_pTool, spiralDef, m_cFeedDef, m_cSetupDef ) ;
	JDNC_LAYER * pLayer   = GetLayerDef() ;
	pLayer->m_dTotalDepth = GetCutDepth()  -m_cStockDef.m_dDepthStock; 
	DefineLayerDepth( * pLayer) ; 
	// STEP 2 : 计算切割路径
    CPathCombine* pPComb = new CPathCombine( NC_WPROCESS_ROUGH ) ;
	CSmartCurve * pCurve; 
	POSITION pos = AllCurve.GetHeadPosition() ;
	while( pos )
	{
		pCurve = ( CSmartCurve*) AllCurve.GetNext( pos ) ;
		for( int i = 1 ; i <= pLayer->m_nLayerCount ;  i ++ ) 
		{
			double dDepth = pLayer->m_dLayerDepth[i] ; 
            CPathCombine  *pTmpComb= new CPathCombine( NC_WPROCESS_ROUGH ) ;
            ncSpiral.CreateCurveSlotPath( *pCurve, m_cParam, *pTmpComb ) ;
            pTmpComb->MoveCombine( 0.0, 0.0, -dDepth ) ;
            pPComb->AppendCombine( pTmpComb ) ;
        }
    }
	MathCAM_ClearCurveList( AllCurve ) ;
    pPComb->MoveCombine( 0.0, 0.0, GetTopHeight() ) ;
    NewPath.AddData( 0.0, pPComb, TRUE ) ;
	if( pLayer->m_dLayerDepth ) delete[] pLayer->m_dLayerDepth ;
	pLayer->m_dLayerDepth = NULL ;
	if( ! NewPath.m_pHead )
	{
		m_nErrorType = JDERROR_GENPATH_NOAREA ;
		return FALSE ; 
	}
	return TRUE ;
}

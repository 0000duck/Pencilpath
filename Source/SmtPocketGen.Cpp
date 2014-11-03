#include "StdAfx.H"
#include "SmartNC.H"
#include "SurfGeo.H"
#include "SmartBound.H"
#include "SmartVoro.H"
#include "SmtPathGen.H"
#include "SmtPathGen2D.H"
#include "SmartPathEx.h"
#include "NcGCode.H"
#include "SmartPocket.H"
#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char BASED_CODE THIS_FILE[] = __FILE__;
#endif

CSmartPocketGen::CSmartPocketGen()
{
    m_dBoundTol = -1.0 ;
    m_dFeedRatio= 100.0; 

}
CSmartPocketGen::~CSmartPocketGen()
{
}
CSmartSect* MathCAM_CreateNewSect( CSmartLoop& Contour,
								   CSmartSect* SectA ,
			                    CSmartSect* SectB ,
								   DOUBLE  OverStep  ,
								   DOUBLE  Radius    ) 
{
	// STEP 0 : 计算尖角
   if( SectA->m_bEndFlag & NC_BOUND_SECT || 
		SectB->m_bEndFlag & NC_BOUND_SECT   )
	{ // 边界轮廓
		return NULL ; 
	}
	VEC2D  dTanA , dTanB ;
	SectA->GetTangent( 1.0, dTanA ) ;
	SectB->GetTangent( 0.0, dTanB ) ;
	dTanB[0] = - dTanB[0], dTanB[1] = -dTanB[1] ;
	DOUBLE dAngle ;
	dAngle = dTanB[0] * dTanA[1] - dTanB[1] * dTanA[0] ;
	if( dAngle < 0.0 ) return NULL ; 
	dAngle = mathGetAngle2DUnit( dTanB, dTanA ) ;
	if( dAngle < 0.1 || dAngle > PI1 - 0.1 )
	{ // 角度不合理
		return NULL ;
	}
	// STEP 1 : 计算增加小圆半径
	DOUBLE dR , dLen , dSinA ;
    DOUBLE dOverLap = 2.0 * Radius - OverStep  ;
	dSinA = sin( dAngle/ 2.0 ) ;
	if( (OverStep - Radius)/ dSinA - Radius < 0.02 )
	{ // 不需生成清角路径
		return NULL ; 
	}
	dLen = ( OverStep - Radius )  / dSinA - dOverLap ;
	dR = dLen / ( 1 + 1.0 / dSinA )  ;
	if( dR < 0.05 ) return NULL  ; 
	if( dR > OverStep/2.0 )
	{
		if( SectA->parent && (SectA->parent->m_bEndFlag & NC_BOUND_SECT ) ||
            SectB->parent && (SectB->parent->m_bEndFlag & NC_BOUND_SECT )  )
		{
			dR = OverStep/2.0 * 0.9 ; 
		}
	}
    // STEP 2 : 增加圆弧过渡段
	PNT2D dStart, dEnd , dMidPnt, dCenter, dIntPt[10];
	PNT2D dAt[3] ; 
	DOUBLE t1, t2 , dMinDist = 1.0e12 , dDist ;
	for( int nTryTime = 0 ; nTryTime <= 5 ; nTryTime ++ )
	{
		if( nTryTime != 0 ) dR /= 1.2 ;
		CSmartSect* pTmpA = SectA->Offset( - dR ) ;
		CSmartSect* pTmpB = SectB->Offset( - dR ) ;
		SectA->GetPoint( 1.0, dAt[0] ) ; 
		int nCount = 0, i = 0  ;
		dMinDist = 1.0e12 ;
		if( ! pTmpA || ! pTmpB || 
			! Mini_SectSectInter( *pTmpA, *pTmpB , dIntPt, nCount) )
		{
		}
		else for( i = 0 ; i < nCount ; i ++ ) 
		{
			t1 = pTmpA->GetParam( dIntPt[i] ) ;
			t2 = pTmpB->GetParam( dIntPt[i] ) ;
			if( t1 > 1.0 && t2 < 0.0 ) 
			{
				SectA->GetPoint( t1, dAt[1] ) ;
				SectB->GetPoint( t2, dAt[2] ) ;
				dDist = mathDist2D( dAt[0], dAt[1] ) + mathDist2D( dAt[0], dAt[2] ) ;
				if( dDist < dMinDist )
				{
					memcpy( dStart, dAt[1], sizeof( PNT2D ) ) ;
					memcpy( dEnd  , dAt[2], sizeof( PNT2D ) ) ;
					memcpy( dCenter,dIntPt[i], sizeof( PNT2D ) ) ;
					dMinDist = dDist ; 
				}
			}
		}
		if( pTmpA ) delete pTmpA ;
		if( pTmpB ) delete pTmpB ; 
		if( dMinDist > 1.0e3 )	continue ; 
		CSmartArc * pArc = new CSmartArc(dCenter, dStart, dEnd ) ;
		pArc->GetPoint( 0.5, dMidPnt ) ;
		pTmpA = SectA->CopyMyself() ;
		pTmpB = SectB->CopyMyself() ;
		pTmpA->TrimEnd( 1, dStart, TRUE ) ;
		pTmpB->TrimEnd( 0, dEnd  , TRUE ) ;
		if(	Contour.IntSectContour( pArc ) || 
			!Contour.IsPtOnContour( dMidPnt ) ||
			Contour.IntSectContour( pTmpA )||
			Contour.IntSectContour( pTmpB ) )
		{
			delete pArc ;
			pArc = NULL ; 
		}
		delete pTmpA ;
		delete pTmpB ;
		if( pArc )
		{
			SectA->TrimEnd( 1, dStart, TRUE ) ;
			SectB->TrimEnd( 0, dEnd  , TRUE ) ;
			return pArc ; 
		}
	}
   return NULL ; 
}
// 生成不光滑的清角路径 
BOOL CSmartPocketGen::CreateFCornerPath( CPathCombine& TComb ,   /* 刀具路径*/
						                 CSmartLoop&  Contour,   /* 原始轮廓*/
	 								     JDNC_FOLLOW& Follow )   /* 走刀方式*/
{
	CSmartLoop *pChild , *pNewCont; 
	CSmtVoronoiMap    Voronoi ;
	CSmartCurve * pCurve, *pNext ;
	Voronoi.m_bCenAll = TRUE ;
	Voronoi.SetTolAndCorner( GetCurveTol(), GetCorDef() ) ;
    Voronoi.UpdateContour( Contour, -1.0 ) ;
	Voronoi.DefineBound( VORONOI_ALLSIDE ) ; 
	int nLayer = (int)ceil(Voronoi.m_dMaxDepth / Follow.m_dOverStep) ;
	double dHeight  ;
	PNT3D  dStart, dEnd ;
	dStart[2] = dEnd[2] = 0.0 ; 
	pNewCont = Contour.CopyContour() ;
	pNewCont->m_nDepth  = 1 ;
	pNewCont->m_pChild  = pNewCont->m_pIsland ;
	pNewCont->m_pIsland = NULL ; 
	pChild = pNewCont->GetChild() ;
	while( pChild ) 
	{
		pChild->m_nDepth =  1 ;
		pChild->ReverseLoop() ; 
		pChild = pChild->next ;
	}
	dHeight = 0.0 ;
	for( int k = 1 ; k <= nLayer ; k ++ ) 
	{
		dHeight += Follow.m_dOverStep ;
		pCurve = Voronoi.GetIsoHeightCurve( dHeight, Follow.m_dOverStep, Follow.m_dOverRatio) ;
		while( pCurve )
		{
			pNext = pCurve->next  ;
			pCurve->next = NULL ; 
			if( ! pCurve->IsClosed() )
			{
				pCurve->GetPoint( 1.0, dStart ) ;
				pCurve->GetPoint( 0.0, dEnd ) ;
				CSmartLine *pLine =  new CSmartLine( dStart, dEnd )  ;
				pCurve->AddSect( pLine ) ;
			}
			pChild = new CSmartLoop() ;
			pChild->UpdateSect( pCurve ) ;
			pCurve->GetPoint( 0.5, dEnd ) ;
			if( pChild->m_dArea < 0.0 )
			{
				pChild->ReverseLoop() ;
			}
			pChild->m_nDepth = k + 1 ;
			if( Contour.IsContourInt( pChild ) || 
				!Contour.IsPtOnContour( dEnd )  )
			{
				Mini_DeleteCurves( pNext ) ;
				if( pNewCont ) delete pNewCont ;
				delete pChild ;
				return FALSE ;
			}
			pNewCont->InsertNewLoop( pChild ) ;
			pCurve = pNext ;
		}
	}
	Contour.m_pCurve->GetPoint(0.0, dStart ) ;
	if( Follow.m_bFollowFlag & NCDEF_FOLLOW_MINUP )
	{ // 最少抬刀
   	    if( GetMillDir() == 0  )
		{ // 逆铣方式 
		    pNewCont->ReverseContour( FALSE ) ;
		} 
		pNewCont->ConnectAllChild() ; 
		pNewCont->ExtractAllChild( TComb, 0 , &Contour) ;
		if( !( Follow.m_bFollowFlag & NCDEF_FOLLOW_TOOUTER ) )
		{ // 从内向外环切
			TComb.ReverseDirect() ;
		}
	}
	else
	{ // 缺省优化
   	    if( GetMillDir() == 0 )
		{ // 逆铣方式 
		    pNewCont->ReverseContour( FALSE ) ;
		} 
		pNewCont->ExtractAllCurve( TComb, dStart, dEnd, &Contour) ;
		if( ( Follow.m_bFollowFlag & NCDEF_FOLLOW_TOOUTER ) )
		{ // 从内向外环切
			TComb.ReverseDirect() ;
		}
	}
	delete pNewCont ; 
	return TRUE ; 
}
// 生成光滑的清角路径
int  CSmartPocketGen::CreateFCornerPathSmooth( CPathCombine& TComb , /*加工路径*/
									     CSmartLoop& Contour , /*加工轮廓*/
	 								     JDNC_FOLLOW& Follow ) /*走刀方式*/
{
	// STEP 1 : 初始化轮廓
	CSmartLoop* lpStack = NULL ,*pChild; 
	Contour.FollowContour( NCDEF_OFFSET_INNER, 
		                   Follow.m_dOverStep ,
		                   GetCorDef(), -1, 
						   Follow.m_dOverRatio ) ;

	// STEP 2 : 搜索所有轮廓 , 增加过渡圆弧
    CSmartCurve*  pCurve ;
	PNT3D dStart, dEnd, dMidPt ;
	DOUBLE dDist, dRadius  ;
	dRadius = Follow.m_dOverStep / ( 1.0 - 0.01 * Follow.m_dOverRatio ) / 2.0 ;
	dStart[2] = dEnd[2] =  0.0 ; 
    CPathCombine    tmpTComb( NC_WPROCESS_ROUGH ) ;
	int nDepth = Contour.m_nDepth ;
	for( lpStack = & Contour ; lpStack ; lpStack = lpStack->link )
	{
		pChild = lpStack ;
		pCurve = pChild->m_pCurve->CopyMyself() ;
		CSmartSect* pSect, *pNext, *pNew  ;
		if( pChild->m_nDepth == 1  )
		{ 
			pCurve->GetPoint( 1.0, dStart ) ;
		}
		else
		{ /*调整起始边界*/
			pSect = pCurve->FindNearSect( dStart, dEnd, dDist ) ;
			pCurve->SetStartSect( pSect ) ;
			pSect = pCurve->GetHead() ;
			while( pSect && pSect->next )
			{
				pNext = pSect->next ;
				pNew  = MathCAM_CreateNewSect( Contour,
					                        pSect ,
					                        pNext ,
											   Follow.m_dOverStep,
											   dRadius ) ;
				if( pNew )
				{
					pCurve->InsertAfter( pNew, pSect ) ;
				}
				pSect = pNext ;
			}
		}
		if( pChild->m_nDepth >= 1 || pChild->m_nDepth == nDepth + 1 )
		{
			pCurve->GetPoint( 0.0, dEnd ) ;
			nc_GetPointAtLine( dStart, dEnd, 0.5, dMidPt, 2 ) ;
			if( Contour.IntLineContour( dStart, dEnd ) == FALSE && 
				Contour.IsPtOnContour( dMidPt ) == 1 && 
				pChild->IntLineContour( dStart, dEnd ) == FALSE ) 
			{
				for( CSmartLoop* pLeftLp = pChild->link ; pLeftLp ; pLeftLp = pLeftLp->next )
				{
					if( pLeftLp->m_nDepth == nDepth || pLeftLp->m_nDepth == nDepth-1 || pLeftLp->m_nDepth == nDepth+1 )
					{
						if( pLeftLp->IntLineContour( dStart, dEnd ) )
						{
							break ;
						}
					}
				}
				if( pLeftLp == NULL && nc_Distance( dStart, dEnd, 3) > 2.0e-4) 
				{
                    CPathLine3D* pLine = new CPathLine3D( dStart, dEnd ) ;
				    tmpTComb.AddEntity( pLine ) ;
				}
			}
			mathCpyPnt( dEnd , dStart ) ;
		}
        if( GetMillDir() == 1 )
        {
            pCurve->Reverse() ;
        }
        tmpTComb.AddCurve( pCurve, TRUE , 0.0 ) ;
		nDepth = pChild->m_nDepth ;
	}
    if( Follow.m_bFollowFlag & NCDEF_FOLLOW_TOOUTER )
    {
        tmpTComb.ReverseDirect() ;
    }
    TComb.AppendCombine( tmpTComb ) ;
    // STEP 4 : 提取孤岛
	for( pChild = Contour.GetIsland() ; pChild ; pChild = pChild->next )
	{
        pCurve = pChild->m_pCurve->CopyMyself() ;
        if( GetMillDir() == 0 ) pCurve->Reverse() ;
		TComb.AddCurve( pCurve, TRUE , 0.0 ) ;
	}
	return 1 ;
}
// 计算带才槽的路径
int CSmartPocketGen::PocketOneLayerEx( CPathCombine&  PathComb ,  /*路径组  */
				                      CSmartLoop&    Contour,    /*轮廓    */
									   JDNC_POCKET&   Param,      /*参数    */
								       DOUBLE DepthInc )          /*深度    */
{ 
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
	// STEP 1 : 底面雕刻效果
	CSmartLoop  *pLpHead, *pLoop ; 
	DOUBLE dRComp[2], dAngle[2] ;
	dAngle[0] = ANGLE_TO_RADIAN( m_cShapeDef.m_cTaper.m_dBoundAngle  ) ;
	dAngle[1] = ANGLE_TO_RADIAN( m_cShapeDef.m_cTaper.m_dIslandAngle ) ;
    //STEP 2 : 计算开槽路径 
	ASSERT( m_cFeedDef.m_cSlotDef.m_nSlotType != 0 ) ;
	JDNC_SLOT& SlotCut = m_cFeedDef.m_cSlotDef ;
	SlotCut.m_dDepthInc = DepthInc ;
	DefineSlotDepth( SlotCut, & m_cToolDef ) ;
	JDNC_SLOT  TmpSlot = SlotCut ;
	TmpSlot.m_nSlotCount = 1 ;
	TmpSlot.m_nSlotType  = NCDEF_TSLOT_COUNT ;
	TmpSlot.m_dSlotDepth = NULL  ;
	for(  int i = 1 ;  i <= SlotCut.m_nSlotCount ; i ++  )
	{
		double dAtDepth = SlotCut.m_dDepthInc - SlotCut.m_dSlotDepth[i] ; 
		dRComp[0] = dAtDepth * tan( dAngle[0] ) ;
		dRComp[1] = dAtDepth * tan( dAngle[1] ) ;
		pLpHead = Contour.OffsetContourEx( dRComp[0], dRComp[1], GetCorDef() ) ;
		MathCAM_ValidOutMostLoop( pLpHead ) ;
	    TmpSlot.m_nSlotCount = i ;
		TmpSlot.m_dDepthInc  = SlotCut.m_dSlotDepth[i] ;
	    DefineSlotDepth( TmpSlot ) ;
		CPathCombine   SlotComb(NC_WPROCESS_SLOT) ;
		for( pLoop = pLpHead ; pLoop ; pLoop = pLoop->next )
		{
			CreateECutPath( SlotComb , *pLoop, TmpSlot, m_cFeedDef.m_cPlungeDef ) ;
		}
		SlotComb.MoveCombine( 0.0, 0.0, DepthInc-SlotCut.m_dSlotDepth[i] ) ;
		PathComb.AppendCombine( SlotComb ) ;
		Mini_DeleteContours( pLpHead ) ;
	}
	if( TmpSlot.m_dSlotDepth ) delete[] TmpSlot.m_dSlotDepth ;
	TmpSlot.m_dSlotDepth = NULL ; 
	CPathCombine   TComb(NC_WPROCESS_ROUGH) ;
	if( m_cParam.m_nMoveType == surfncMoveLinear ) 
	{ // 行切
		JDNC_LINEAR Linear = Param.m_cLinear ;
		Linear.m_bLineFlag |= NCDEF_LINEAR_SLOTCUT ;
		CreateLinearPath( TComb       ,  /*保存路径*/
						  Contour      ,  /*轮廓曲线*/
						  Linear); /*行切参数*/ 
	}
	else if( m_cParam.m_nMoveType == surfncMoveFollow ) 
	{ // 环切
		JDNC_FOLLOW Follow = Param.m_cFollow ;
		Follow.m_bFollowFlag |= NCDEF_FOLLOW_SLOTCUT ;
		CreateFollowPath( TComb       ,  /*保存路径*/
						  Contour     ,  /*轮廓曲线*/
						  Follow);  /*环切参数*/ 
	}
	else if( m_cParam.m_nMoveType == surfncMoveSpiral ) 
	{ // 环绕
		JDNC_SPIRAL Spiral = Param.m_cSpiral ;
		Spiral.m_bSpiralFlag |= NCDEF_SPIRAL_SLOTCUT ;
		CreateSpiralPath( TComb     ,  /*保存路径*/
						  Contour   ,  /*轮廓曲线*/
						  Spiral   );  /*行切参数*/ 
	}
	PathComb.AppendCombine( TComb ) ;
	return TRUE  ;
}
int CSmartPocketGen::PocketOneLayer( CPathCombine&  PComb    ,  /*路径组  */
				                     CSmartLoop&   Contour   ,  /*轮廓    */
								     JDNC_POCKET& PocketCut  ,  /*区域加工*/
								     double dDepthInc        ,  /*深度进给*/
									 CSmartLoop*  PartBnd,      /*模型边界,非切区域*/
									 CSmartLoop*  BlankBnd)     /*毛坯边界,搜索下刀引入点*/
{ 
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
		m_cFeedDef.m_cPlungeDef.m_dIncStep = dDepthInc + m_cFeedDef.m_cPlungeDef.m_dTopTol ;
    }
    CPathCombine  TComb( NC_WPROCESS_ROUGH ) ;
	BOOL bEnSlot = FALSE ;
	if( m_cFeedDef.m_cSlotDef.m_nSlotType != NCDEF_TSLOT_CLOSE ) 
	{ // 计算开槽路径
        TComb.m_nType = NC_WPROCESS_SLOT  ;
		m_cFeedDef.m_cSlotDef.m_dDepthInc =  dDepthInc ;
		DefineSlotDepth( m_cFeedDef.m_cSlotDef,& m_cToolDef ) ;
		CreateECutPath( TComb    , 
			            Contour   , 
		                m_cFeedDef.m_cSlotDef , 
					    m_cFeedDef.m_cPlungeDef ) ;
		bEnSlot = TRUE ;
	}
    DOUBLE dMaxDist[2] = {max( 6.0, 2.0*m_cToolDef.m_dTopDiam),  0.2 } ;
    TComb.m_nType = NC_WPROCESS_ROUGH  ;
	if( PocketCut.m_nMoveType == surfncMoveLinear ) 
	{ // 行切
		if( bEnSlot ) PocketCut.m_cLinear.m_bLineFlag |= NCDEF_LINEAR_SLOTCUT ;
		else PocketCut.m_cLinear.m_bLineFlag &= ~NCDEF_LINEAR_SLOTCUT ;
		CreateLinearPath( TComb       ,  /*保存路径*/
					      Contour     ,  /*轮廓曲线*/
	                      PocketCut.m_cLinear); /*行切参数*/ 
		dMaxDist[0] = max( dMaxDist[0], PocketCut.m_cLinear.m_dOverStep * 1.5) ;
        dMaxDist[1] = PocketCut.m_cLinear.m_dOverStep ;
	}
	else if( PocketCut.m_nMoveType == surfncMoveFollow ) 
	{ // 环切
		JDNC_FOLLOW&  FollowCut = PocketCut.m_cFollow  ;
		if( bEnSlot ) FollowCut.m_bFollowFlag |= NCDEF_FOLLOW_SLOTCUT ;
		else FollowCut.m_bFollowFlag &= ~NCDEF_FOLLOW_SLOTCUT ;
		// 防止环切时尖角处出现过切，2014.6.30 liuxin
		JDNC_SETUP cSetupDef = m_cSetupDef;
		if (FollowCut.m_bFollowFlag & NCDEF_FOLLOW_SMOOTH)
		{
			cSetupDef.m_cCorDef.m_nCorType = NCDEF_CORNER_ARCUSER;
		}
        CSmartNcFollow FollowGen( m_pTool, FollowCut, m_cFeedDef, /*m_cSetupDef*/cSetupDef ) ;
        if( m_cFeedDef.m_cSlotDef.m_nSlotType == NCDEF_TSLOT_CLOSE )
        {/*开槽加工加工不支持修边*/
            FollowGen.SetBoundtol( m_dBoundTol ,m_dFeedRatio );
        }
        FollowGen.CreatePocketPath( Contour,TComb ) ;
		dMaxDist[0] = max( dMaxDist[0], PocketCut.m_cFollow.m_dOverStep * 1.5) ;
        dMaxDist[1] = PocketCut.m_cFollow.m_dOverStep ;
	}
	else if( PocketCut.m_nMoveType == surfncMoveSpiral ) 
	{ // 环绕
        if( bEnSlot || ( PocketCut.m_bPocketFlag & NCDEF_POCKET_OPENBND) )
        {
            PocketCut.m_cSpiral.m_bSpiralFlag |= NCDEF_SPIRAL_SLOTCUT ;
        }
        DOUBLE dOldZInc = m_cFeedDef.m_cPlungeDef.m_dIncStep ;
        m_cFeedDef.m_cPlungeDef.m_dIncStep = dDepthInc + m_cFeedDef.m_cPlungeDef.m_dTopTol ;
		CreateSpiralPath( TComb     ,  /*保存路径*/
						  Contour   ,  /*轮廓曲线*/
		                  PocketCut.m_cSpiral);  /*行切参数*/ 
		dMaxDist[0] = max( dMaxDist[0], PocketCut.m_cSpiral.m_dOverStep * 1.5) ;
		dMaxDist[1] = PocketCut.m_cSpiral.m_dOverStep ;
        m_cFeedDef.m_cPlungeDef.m_dIncStep = dOldZInc ;
	}
	// STEP 3 : 计算下刀路径
	JDNC_PLUNGE  Plunge = m_cFeedDef.m_cPlungeDef ;
    if( Plunge.m_nPlungeType != NCDEF_PLUNGE_CLOSE )
	{
		TComb.m_nType = NC_WPROCESS_PLUNGE ;
		Plunge.m_dIncStep = dDepthInc + Plunge.m_dTopTol ;
        if( BlankBnd == NULL ) 
        {
            dMaxDist[1] += (0.5 * m_cToolDef.m_dTopDiam) ;
        }
        if( PocketCut.m_nMoveType == surfncMoveFollow && 
            !(PocketCut.m_cFollow.m_bFollowFlag & NCDEF_FOLLOW_TOOUTER) ||
            PocketCut.m_nMoveType == surfncMoveSpiral && 
            !( PocketCut.m_cSpiral.m_bSpiralFlag & NCDEF_SPIRAL_TOOUTER) )
        {
            m_bFollowOuter = 0 ;
        }
		int nFlag[2] = { PocketCut.m_nMoveType, m_cSetupDef.m_cOrderDef.m_nMillDir } ;
		InsertPlungePath( TComb, Contour, Plunge, PartBnd, BlankBnd, dMaxDist, nFlag ) ;
        m_bFollowOuter = 1 ;
	}
	PComb.AppendCombine( TComb ) ;
	return TRUE  ;
}
int CSmartPocketGen::PocketOneContour( CPathGroup& PGroup,
                                       CPathCombine&  PComb    ,  /*路径组  */
				                       CSmartLoop&   Contour   ,  /*轮廓    */
								       JDNC_POCKET& PocketCut  ,  /*区域加工*/
								       double dDepthInc        ,  /*深度进给*/
									   CSmartLoop*  PartBnd,      /*模型边界,非切区域*/
									   CSmartLoop*  BlankBnd)     /*毛坯边界,搜索下刀引入点*/
{ 
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
		m_cFeedDef.m_cPlungeDef.m_dIncStep = dDepthInc + m_cFeedDef.m_cPlungeDef.m_dTopTol ;
    }
    CPathCombine  TComb( NC_WPROCESS_ROUGH ) ;
	BOOL bEnSlot = FALSE ;
	if( m_cFeedDef.m_cSlotDef.m_nSlotType != NCDEF_TSLOT_CLOSE ) 
	{ // 计算开槽路径
        TComb.m_nType = NC_WPROCESS_SLOT  ;
		m_cFeedDef.m_cSlotDef.m_dDepthInc =  dDepthInc ;
		DefineSlotDepth( m_cFeedDef.m_cSlotDef,& m_cToolDef ) ;
		CreateECutPath( TComb    , 
			            Contour   , 
		                m_cFeedDef.m_cSlotDef , 
					    m_cFeedDef.m_cPlungeDef ) ;
		bEnSlot = TRUE ;
	}
    DOUBLE dMaxDist[2] = {max( 6.0, 2.0*m_cToolDef.m_dTopDiam),  0.2 } ;
    TComb.m_nType = NC_WPROCESS_ROUGH  ;
	if( PocketCut.m_nMoveType == surfncMoveLinear ) 
	{ // 行切
		if( bEnSlot ) PocketCut.m_cLinear.m_bLineFlag |= NCDEF_LINEAR_SLOTCUT ;
		else PocketCut.m_cLinear.m_bLineFlag &= ~NCDEF_LINEAR_SLOTCUT ;
		CreateLinearPath( TComb       ,  /*保存路径*/
					      Contour     ,  /*轮廓曲线*/
	                      PocketCut.m_cLinear); /*行切参数*/ 
		dMaxDist[0] = max( dMaxDist[0], PocketCut.m_cLinear.m_dOverStep * 1.5) ;
        dMaxDist[1] = PocketCut.m_cLinear.m_dOverStep ;
	}
	else if( PocketCut.m_nMoveType == surfncMoveFollow ) 
	{ // 环切
		JDNC_FOLLOW&  FollowCut = PocketCut.m_cFollow  ;
		if( bEnSlot ) FollowCut.m_bFollowFlag |= NCDEF_FOLLOW_SLOTCUT ;
		else FollowCut.m_bFollowFlag &= ~NCDEF_FOLLOW_SLOTCUT ;
		// 防止环切时尖角处出现过切，2014.6.30 liuxin
		JDNC_SETUP cSetupDef = m_cSetupDef;
		if (FollowCut.m_bFollowFlag & NCDEF_FOLLOW_SMOOTH)
		{
			cSetupDef.m_cCorDef.m_nCorType = NCDEF_CORNER_ARCUSER;
		}
        CSmartNcFollow FollowGen( m_pTool, FollowCut, m_cFeedDef, /*m_cSetupDef*/ cSetupDef ) ;
        if( PGroup.m_pTail )
        {
            TPNT3D dStartAt ;
            if( PGroup.m_pTail->GetEndPoint(1, dStartAt ) )
            {
				if( PocketCut.m_bPocketFlag & NCDEF_POCKET_OPENBND )
				{
					SetContourHeadAtBlank( &Contour, m_pTool->m_fRadius * 2, dStartAt ) ;
				}
				FollowGen.SetStartAt( dStartAt ) ;
            }
        }
        if( m_cFeedDef.m_cSlotDef.m_nSlotType == NCDEF_TSLOT_CLOSE )
        {/*开槽加工加工不支持修边*/
            FollowGen.SetBoundtol( m_dBoundTol ,m_dFeedRatio );
        }
        FollowGen.CreatePocketPath( Contour,TComb ) ;
		dMaxDist[0] = max( dMaxDist[0], PocketCut.m_cFollow.m_dOverStep * 1.5) ;
        dMaxDist[1] = PocketCut.m_cFollow.m_dOverStep ;
	}
	else if( PocketCut.m_nMoveType == surfncMoveSpiral ) 
	{ // 环绕
        if( bEnSlot || ( PocketCut.m_bPocketFlag & NCDEF_POCKET_OPENBND) )
        {
            PocketCut.m_cSpiral.m_bSpiralFlag |= NCDEF_SPIRAL_SLOTCUT ;
        }
		CreateSpiralPath( TComb     ,  /*保存路径*/
						  Contour   ,  /*轮廓曲线*/
		                  PocketCut.m_cSpiral);  /*行切参数*/ 
		dMaxDist[0] = max( dMaxDist[0], PocketCut.m_cSpiral.m_dOverStep * 1.5) ;
		dMaxDist[1] = PocketCut.m_cSpiral.m_dOverStep ;
	}
	// STEP 3 : 计算下刀路径
	JDNC_PLUNGE  Plunge = m_cFeedDef.m_cPlungeDef ;
    if( Plunge.m_nPlungeType != NCDEF_PLUNGE_CLOSE )
	{
		TComb.m_nType = NC_WPROCESS_PLUNGE ;
		Plunge.m_dIncStep = dDepthInc + Plunge.m_dTopTol ;
        if( BlankBnd == NULL ) 
        {
            dMaxDist[1] += (0.5 * m_cToolDef.m_dTopDiam) ;
        }
        if( PocketCut.m_nMoveType == surfncMoveFollow && 
            !(PocketCut.m_cFollow.m_bFollowFlag & NCDEF_FOLLOW_TOOUTER) ||
            PocketCut.m_nMoveType == surfncMoveSpiral && 
            !( PocketCut.m_cSpiral.m_bSpiralFlag & NCDEF_SPIRAL_TOOUTER) )
        {
            m_bFollowOuter = 0 ;
        }
		int nFlag[2] = { PocketCut.m_nMoveType, m_cSetupDef.m_cOrderDef.m_nMillDir } ;
		InsertPlungePath( TComb, Contour, Plunge, PartBnd, BlankBnd, dMaxDist, nFlag ) ;
        m_bFollowOuter = 1 ;
	}
	PComb.AppendCombine( TComb ) ;
	return TRUE  ;
}

void SurfNC_InitPathParam( JDNC_CORNER&  Corner  ) ;
void SurfNC_InitPathParam( JDNC_BOUND&  Bound  ) ;

BOOL CSmartPocketGen::FinishPocketBound( CPathGroup& NewPath ,   /*雕刻路径*/ 
                                         CSmartLoop* AllCont ,	 /*雕刻图形*/ 
										 RFRAME		&LocFrame )   
{
    JDNC_CORNER  CornerCut ;
    JDNC_BOUND   BoundCut ;
    CSmartCornerGen CornerGen ;
    CSmartBoundGen  BoundGen ;
    BOOL bCornerMode = FALSE ;
    DOUBLE dSideAngle = 10.0 ;
	DOUBLE dDepth  = GetCutDepth() - m_cStockDef.m_dDepthStock;
    DOUBLE dDepthInc = GetLayerDef()->m_dTotalDepth / GetLayerDef()->m_nLayerCount;
    if( m_cParam.m_bPocketFlag & NCDEF_POCKET_CORNERMODE )
    {
	    dSideAngle = ANGLE_TO_RADIAN( m_cShapeDef.m_cTaper.m_dBoundAngle ) ;
	    if( dSideAngle < m_pTool->m_fAngle  ) dSideAngle = m_pTool->m_fAngle ;
        if( m_cToolDef.m_nToolType == surfncToolAFlat && 
            dSideAngle > ANGLE_TO_RADIAN( 3 )  &&
            dDepth > 0.03 )
        {
            bCornerMode = TRUE ;
        }
    }
    if( bCornerMode )
    {
        SurfNC_InitPathParam( CornerCut ) ;
        CornerCut.m_dCutDepth = dDepth ;
        if( m_cParam.m_bPocketFlag & NCDEF_POCKET_OPENBND )
        {
            CornerCut.m_bCornerFlag  |= NCDEF_CORNER_NOBOUND ;
        }
    	CornerGen.m_cSetupDef = m_cSetupDef ;
		CornerGen.m_cShapeDef = m_cShapeDef ;
		CornerGen.m_cToolDef  = m_cToolDef  ;
		CornerGen.m_cSpeedDef = m_cSpeedDef ;
        CornerGen.m_cParam    = CornerCut    ;
		CornerGen.UpdateTool2D() ;
    }
    else
    {
        SurfNC_InitPathParam( BoundCut ) ;
        if( m_cParam.m_bPocketFlag & NCDEF_POCKET_OPENBND )
        {
            BoundCut.m_bBoundFlag  |= NCDEF_FBOUND_NOOUTER ;
        }
    	BoundGen.m_cSetupDef = m_cSetupDef ;
		BoundGen.m_cShapeDef = m_cShapeDef ;
		BoundGen.m_cToolDef  = m_cToolDef  ;
		BoundGen.m_cSpeedDef = m_cSpeedDef ;
        BoundGen.m_cParam    = BoundCut    ;
        BoundGen.m_cFeedDef.m_cLeadDef  = m_cFeedDef.m_cLeadDef     ;
        BoundGen.m_cFeedDef.m_cPlungeDef  = m_cFeedDef.m_cPlungeDef ;
        BoundGen.m_cFeedDef.m_cPlungeDef.m_nPlungeType = NCDEF_PLUNGE_CLOSE ;
		BoundGen.UpdateTool2D() ;
    }
    int bOffType = NCDEF_OFFSET_INNER ;
    DOUBLE dOffDist[2] ={ 0.0, 0.0 } ;
    dOffDist[0] = dOffDist[1] = GetSideStock() + m_pTool->GetRadiusComp( 0.0, 0.0 );
    if( dOffDist[0] < 0.0 )
    {
        bOffType = NCDEF_OFFSET_OUTER ;
        dOffDist[0] = dOffDist[1] = - dOffDist[0] ;
    }
    CPathCombine* pTComb = new CPathCombine( NC_WPROCESS_ROUGH ) ;
    for( CSmartLoop* pContour = AllCont ; pContour ; pContour = pContour->next )
    {
        if( bCornerMode )
        {
		    CSmartLoop* pLpHead = pContour->OffsetContour( bOffType, dOffDist[0],dOffDist[1], GetCorDef() ) ;
   	        MathCAM_ValidOutMostLoop( pLpHead ) ;
            for( CSmartLoop* pLoop = pLpHead ; pLoop ; pLoop = pLoop->next )
            {
                CornerGen.CreateCornerPath(*pTComb, *pLoop,CornerCut,dSideAngle); 
            }
            Mini_DeleteContours( pLpHead ) ;
        }
        else
        {
		    CSmartLoop* pLpHead = OffsetPathBound( *pContour, dDepth , 0.0, TRUE ) ; 
   	        MathCAM_ValidOutMostLoop( pLpHead ) ;
            for( CSmartLoop* pLoop = pLpHead ; pLoop ; pLoop = pLoop->next )
            {
         	    BoundGen.BoundOneLayer( *pTComb, *pLoop, BoundCut, LocFrame, dDepth, dDepthInc, 0 ) ;
            }
            Mini_DeleteContours( pLpHead ) ;
        }
    }
    for( CPathEntity* pTEnt = pTComb->m_pHead ; pTEnt ; pTEnt = pTEnt->next )
    {
        pTEnt->m_fFeedScale = (float)( m_cParam.m_dFeedRatio * 0.01) ; 
    }
    if( bCornerMode ) dDepth = 0.0 ;
    NewPath.AddData( dDepth , pTComb, TRUE ) ;
    return TRUE ;
}

BOOL CSmartPocketGen::GeneratePathEx( CPathGroup& NewPath ,   /*雕刻路径*/ 
		                              CSmartGraphic& Graph)   /*图形函数*/
{
	// STEP 0 : 校正雕刻环境
	if( !UpdateTool2D()) return FALSE ;
	CSmartLoop  *BndCont = Graph.GetAllContour( m_cSetupDef ) ;
	BndCont = RedefineBoundLoops( BndCont ) ;
	if( ! BndCont  )
	{ // 校正图形
		m_nErrorType = JDERROR_GENPATH_NOLOOP ;
		return FALSE ;
	}

	BOOL bRet = CreatPath ( BndCont, NewPath, Graph ) ;
	
	Mini_DeleteContours( BndCont ) ; 

	return bRet ;
}

BOOL CSmartPocketGen::CreatPath ( CSmartLoop	*BndCont,
								  CPathGroup	&NewPath,
								  CSmartGraphic& Graph )
{
	BOOL  bIsOpenBnd = m_cParam.m_bPocketFlag & NCDEF_POCKET_OPENBND  ? TRUE : FALSE ;
	CSmartLoop * pContour, *pLoop, *pLpHead  ; 

	if( bIsOpenBnd == TRUE  )
	{// 外轮廓开阔, 忽略外边框
		for( pLoop = BndCont ; pLoop ; pLoop = pLoop->next )
		{
			if( pLoop->m_pCurve && MathCAM_IsOutMostLoop( BndCont, pLoop, FALSE ) )
			{
				pLoop->m_bUseFlag|= NC_LOOP_OUTER ;
				pLoop->m_pCurve->SetBlank( TRUE ) ;
			}
		}
	}
    CSmartLoop* AllCont = BndCont ; 
    if( (m_cParam.m_bPocketFlag & NCDEF_POCKET_BOUNDEND) && 
        m_cParam.m_dBoundTol > 0.001 )
    {/*区域修边*/
        DOUBLE dRComp[2] = {m_cParam.m_dBoundTol, m_cParam.m_dBoundTol} ;
        AllCont  = Mini_OffsetContourList( BndCont, NCDEF_OFFSET_INNER, dRComp, GetCorDef() ) ; 
	    if( bIsOpenBnd == TRUE  )
	    {// 外轮廓开阔, 忽略外边框
		    for( pLoop = AllCont ; pLoop ; pLoop = pLoop->next )
		    {
			    if( pLoop->m_pCurve && MathCAM_IsOutMostLoop( AllCont, pLoop, FALSE ) )
			    {
				    pLoop->m_bUseFlag|= NC_LOOP_OUTER ;
				    pLoop->m_pCurve->SetBlank( TRUE ) ;
			    }
		    }
	    }
    }
	
	JDNC_LAYER * pLayer   = GetLayerDef() ;
	pLayer->m_dTotalDepth = GetCutDepth() - m_cStockDef.m_dDepthStock; 
	DefineLayerDepth( * pLayer) ; 
    RepairBlindRadius( m_cFeedDef.m_cPlungeDef ) ;
	// STEP 1 : 计算区域路径
	BOOL bAllBound = FALSE ;
	CSmartBoundGen BoundGen ;
	if( m_cParam.m_nMoveType == surfncMoveLinear )
	{
		if( m_cParam.m_cLinear.m_bLineFlag & NCDEF_LINEAR_SIDETOL && 
			m_cParam.m_cLinear.m_bLineFlag & NCDEF_LINEAR_ALLCUT   )
		{
			bAllBound = TRUE ;
			m_cParam.m_cLinear.m_bLineFlag &= ~NCDEF_LINEAR_SIDETOL ;
		}
	}
	else if( m_cParam.m_nMoveType == surfncMoveFollow )
	{
		if( m_cParam.m_bPocketFlag & NCDEF_RPOCKET_OPENBND  )
		{
			m_cParam.m_cFollow.m_bFollowFlag |= NCDEF_FOLLOW_OPENBND ;
		}
	}
	if( bAllBound )
	{
		BoundGen.m_cSetupDef = m_cSetupDef ;
		BoundGen.m_cShapeDef = m_cShapeDef ;
		BoundGen.m_cToolDef  = m_cToolDef  ;
		BoundGen.m_cSpeedDef = m_cSpeedDef ;
		BoundGen.UpdateTool2D() ;
	}
	// STEP 2 : 取得局部坐标系
	RFRAME LocFrame ;
	if( !Graph.GetTPlaneFrame( LocFrame ) )
	{
		mathInitRFrame( &LocFrame ) ;
	}

	if (m_c5DCtrlDef.m_cToolAxisApp.m_nFrameFlag != NCDEF_FRAME_DEFAULT)
	{
		mathMultRFrame( &LocFrame, &m_c5DCtrlDef.m_cToolAxisApp.m_dIndexFrame, &LocFrame ) ;
	}
	
	DOUBLE dDepth, dDepthInc ;
	if( GetLayerOrder() == 1  && ( bAllBound == NULL) )
	{ // 高度优先
		for(  int i = 1 ;  i <= pLayer->m_nLayerCount ; i ++ ) 
		{
			dDepth  =  pLayer->m_dLayerDepth[ i ] ;
			dDepthInc = dDepth - pLayer->m_dLayerDepth[ i-1 ] ;
			for(  pContour = AllCont ; pContour ; pContour = pContour->next )
			{ 
			    CPathCombine * pTComb = new CPathCombine( NC_WPROCESS_ROUGH ) ;
				if( m_cParam.m_bPocketFlag & NCDEF_POCKET_NORCOMP )
				{
					pLpHead = pContour->CopyContour() ;
				}
				else
				{
					pLpHead = OffsetPathBound( *pContour, dDepth, 0.0, TRUE ) ;
				}
				MathCAM_ValidOutMostLoop( pLpHead ) ;
				for( pLoop = pLpHead ; pLoop ; pLoop = pLoop->next )
				{
					PocketOneLayer( *pTComb, *pLoop,m_cParam, dDepthInc,AllCont ) ;
				}
				Mini_DeleteContours( pLpHead ) ;
				if( bAllBound )
				{ 
					BoundGen.BoundAllCutPath( *pTComb, *pContour, LocFrame, dDepth ) ;
				}
				NewPath.AddData( dDepth, pTComb, TRUE ) ;
			}
		}
	}
	else 
	{ // 区域优先 , 轮廓需要排序
		for( pContour = AllCont ; pContour ; pContour = pContour->next )
		{ 
			if( bAllBound )
			{ /*模糊兜边*/
				for(  int i = 1 ;  i <= pLayer->m_nLayerCount ; i ++ ) 
				{
					dDepth  =  pLayer->m_dLayerDepth[ i ] ;
					dDepthInc = dDepth - pLayer->m_dLayerDepth[ i-1 ] ;
				    CPathCombine* pTComb = new CPathCombine( NC_WPROCESS_ROUGH ) ;
					if( m_cParam.m_bPocketFlag & NCDEF_POCKET_NORCOMP )
					{
						pLpHead = pContour->CopyContour() ;
					}
					else
					{
						pLpHead = OffsetPathBound( *pContour, dDepth, 0.0, TRUE ) ;
					}
					MathCAM_ValidOutMostLoop( pLpHead ) ;
					for( pLoop = pLpHead ; pLoop ; pLoop = pLoop->next )
					{
						PocketOneLayer( *pTComb, *pLoop,m_cParam, dDepthInc,AllCont ) ;
					}
					Mini_DeleteContours( pLpHead ) ;
					if( bAllBound )
					{ 
						BoundGen.BoundAllCutPath( *pTComb, *pContour, LocFrame, dDepth ) ;
					}
					NewPath.AddData( dDepth, pTComb, TRUE ) ;
				}
			}
			else
			{
				CPathCombine* pTComb = new CPathCombine( NC_WPROCESS_ROUGH ) ;
				pLpHead = NULL  ;
				for(  int i = 1 ;  i <= pLayer->m_nLayerCount ; i ++ ) 
				{
					dDepth  =  pLayer->m_dLayerDepth[ i ] ;
					dDepthInc = dDepth - pLayer->m_dLayerDepth[ i-1 ] ;
					CSmartLoop* pChild = NULL ; 
					if( m_cParam.m_bPocketFlag & NCDEF_POCKET_NORCOMP )
					{
						pChild = pContour->CopyContour() ;
					}
					else
					{
						pChild = OffsetPathBound( *pContour, dDepth, 0.0, TRUE ) ;
					}
					if( ! pChild ) continue ;
					MathCAM_ValidOutMostLoop( pChild ) ;
					for( pLoop = pChild ; pLoop ; pLoop = pLoop->next )
					{
						pLoop->m_nDepth = i ; 
						CSmartLoop* pIsland = pLoop->GetIsland()  ;
						for(  ; pIsland ; pIsland = pIsland->next )
						{
							pIsland->m_nDepth = i ;
						}
					}
					pLpHead = Mini_AddContours( pLpHead, pChild ) ;
				}
				if( pLayer->m_nLayerCount > 1 ) 
				{/*按照高度优先排序*/
					pLpHead = Mini_SortLayerContours( pLpHead , m_pTool->GetRadiusComp( 0.0, 0.0 ) );
				}
				//PNT3D start, end;
				for( pLoop = pLpHead ; pLoop ; pLoop = pLoop->next )
				{
					CPathCombine tmpComb( NC_WPROCESS_ROUGH ) ;
					dDepth  =  pLayer->m_dLayerDepth[ pLoop->m_nDepth ] ;
					dDepthInc = dDepth - pLayer->m_dLayerDepth[ pLoop->m_nDepth-1 ] ;
					PocketOneLayer( tmpComb, *pLoop,m_cParam, dDepthInc, AllCont ) ;
					tmpComb.MoveCombine( 0.0, 0.0, -dDepth ) ;
					// 2013/03/12
					// 防止生成上下两层路径之间连刀时，连刀路径直接在上一层高度划过
					// 因此这里不再生成上下层间的连接路径
					// 添加上下两层的连接路径
					//AddCnctPathBtwLayers(tmpComb, pTComb, dDepth, dDepthInc, 
					//	                 pLoop->prev, start, end ) ;
					pTComb->AppendCombine( tmpComb ) ;	
				}
				Mini_DeleteContours( pLpHead ) ;
				NewPath.AddData( 0.0, pTComb, TRUE ) ;
			}
		}
	}
	if( AllCont != BndCont ) Mini_DeleteContours( AllCont ) ; 
    AllCont = NULL ; 
    if( m_cParam.m_bPocketFlag & NCDEF_POCKET_BOUNDEND )
    {/*区域修边*/
        FinishPocketBound( NewPath, BndCont, LocFrame ) ;
    }
	
	if( bAllBound == TRUE )
	{
		m_cParam.m_cLinear.m_bLineFlag |= ~NCDEF_LINEAR_SIDETOL ;
	}

	if( pLayer->m_dLayerDepth ) 
	{
		delete[] pLayer->m_dLayerDepth ;
		pLayer->m_dLayerDepth = NULL ;
	}
	// STEP 2 : 转换路径
	if( ! NewPath.m_pHead )
	{
		m_nErrorType = JDERROR_GENPATH_NOAREA ;
		return FALSE ; 
	}
	// 校正表面高度
	NewPath.ZMoveGroup( GetTopHeight() ) ;
	
	return TRUE ;
}

//生成二维刀具扫掠环
CSmartLoop * CSmartPocketGen::CreateToolLoop( PNT3D start, PNT3D end, 
											  double dRComp , CSmartLoop* pLoop )
{
	PNT2D start_cpy, end_cpy ;
	nc_VectorCopy( start_cpy, start, 2 ) ;
	nc_VectorCopy( end_cpy, end, 2 ) ;
	if ( m_cParam.m_bPocketFlag & NCDEF_POCKET_OPENBND 
		&& m_cFeedDef.m_cPlungeDef.m_nPlungeType != NCDEF_PLUNGE_CLOSE )
	{/*外轮廓开阔&打开下刀*/
		if( !pLoop ) return NULL ;
		PNT2D PntArr[200] ;
		int nCont = pLoop->GetLineIntContour( start, end, PntArr, 200 ) ;
		int nNum = 0 ;
		PNT2D IntPArr[200] ;
		for ( int i = 0; i < nCont; i++ )
		{
			if( PntArr[i][0] >= start[0] && PntArr[i][0] <= end[0]
			|| PntArr[i][0] <= start[0] && PntArr[i][0] >= end[0] )
			{
				nc_VectorCopy( IntPArr[nNum++], PntArr[i], 2 ) ;
			}
		}
		if ( 1 == nNum )
		{ 
			if( nc_DistanceSquare(start_cpy, IntPArr[0], 2) > 1.0e-2)
			{
			    nc_VectorCopy( end_cpy, IntPArr[0], 2 ) ;
			}
		}
		else if( 2 == nNum )
		{
			if ( nc_DistanceSquare( end_cpy, IntPArr[0], 2)
				> nc_DistanceSquare( end_cpy, IntPArr[1], 2 ) )
			{
				nc_VectorCopy( end_cpy, IntPArr[1], 2 ) ;
			} 
			else
			{
				nc_VectorCopy( end_cpy, IntPArr[0], 2 ) ;
			}
		}
		else if( nNum > 2 )
		{
			return NULL ;
		}
	}
	VEC3D dir = {0}, dZ = { 0, 0, 1 } ;
	nc_VectorMinus( start_cpy, end_cpy, dir, 2 ) ;
	nc_Normalize( dir, 2 )  ;
	VEC3D v_dir ; //直线法向
	nc_VProduct( dir, dZ, v_dir ) ;
	v_dir[2] = 0 ;
	nc_Normalize( v_dir, 2 ) ;
	CSmartCurve* pCurve = new CSmartCurve();
	PNT2D st[2], ed[2] ;
	for ( int i = 0; i < 2 ; i++ )
	{
		for ( int j = 0; j < 2; j++ )
		{
			st[j][i] = start_cpy[i] + dRComp *(1-2*j) * v_dir[i] ;
			ed[j][i] = end_cpy[i] + dRComp *(1-2*j) * v_dir[i] ;
		}
	}
	//构造第一个直线段
	CSmartLine * pLine = new CSmartLine( st[0], ed[0] ) ;
	pCurve->AddSect( pLine );
	//构造第二个直线段
	pLine = new CSmartLine( ed[0], ed[1] ) ;
	pCurve->AddSect( pLine ) ;
	//构造第三个直线段
	pLine = new CSmartLine( ed[1], st[1] ) ;
	pCurve->AddSect( pLine ) ;
	//构造第四个直线段
	pLine = new CSmartLine( st[1], st[0] ) ;
	pCurve->AddSect( pLine ) ;

	CSmartLoop* pTLoop = new CSmartLoop();
	pCurve->LinearCurve(2.0e-4, 0.17);
	pTLoop->UpdateSect(pCurve);
	if( pTLoop->Area() < 0 )
	{
		pTLoop->ReverseLoop() ;
	}
	return pTLoop ;
}
//判断加工环和刀具环是否相交
BOOL CSmartPocketGen::IsContourIntTool( CSmartLoop* pContour, CSmartLoop* pTLoop )
{
	if( !pContour || !pTLoop ) return TRUE ;
	//(1) 判断是否相交
	if( pContour->IsContourInt( pTLoop ) == TRUE )
	{
		return TRUE ;
	}
	//(2) 判断是否包含孤岛
	CSmartLoop* pIsLand = pContour->GetIsland() ;
	for ( ; pIsLand; pIsLand = pIsLand->next )
	{//孤岛是否在刀具环内
		PNT2D mid ;
		pIsLand->GetCurve()->GetMidPoint( mid ) ;
		if ( pTLoop->IsPointIn( mid ) > 0 )
		{
			return TRUE ;
		}
	}
	//(3) 判断pTLoop是否在pContour内
	PNT2D mid;
	pTLoop->GetCurve()->GetMidPoint( mid ) ;
	if ( pContour->IsPointIn( mid ) <= 0 )
	{
		return TRUE ;
	}
	return FALSE ;
}
//添加分层连接路径
BOOL CSmartPocketGen::AddCnctPathBtwLayers(CPathCombine& tmpComb, CPathCombine* pTComb,
										   double dDepth, double dDepthInc, 
										   CSmartLoop* pPreLoop, PNT3D start, PNT3D end )
{
	JDNC_LAYER* pLayer = GetLayerDef() ;
	if(!(pLayer->m_bLayerFlag & NCDEF_LAYER_KEEPDOWN)
		|| tmpComb.m_pHead == NULL )
	{
		return FALSE ;
	}
	if ( pTComb->m_pHead != NULL && pPreLoop != NULL )
	{
		tmpComb.GetEndPoint(0, end ) ;
	}
	if( pTComb->m_pHead != NULL && pPreLoop != NULL
		&& start[2] > end[2] )
	{
		double dRComp = m_pTool->GetRadiusComp( dDepth-dDepthInc, 0.0 ) ;
		CSmartLoop *pChkLoop = pPreLoop->OffsetContour(NCDEF_OFFSET_OUTER,
														dRComp, 
														dRComp,
														GetCorDef()) ;
		CSmartLoop *pTLoop = CreateToolLoop( start, end, dRComp, pPreLoop ) ;
		if ( IsContourIntTool( pChkLoop, pTLoop ) == FALSE )
		{
			PNT3D mid;
			nc_VectorCopy( mid, end, 2 ) ;
			mid[2] = start[2] ;
			CPathPLine3D* pPLine3D = new CPathPLine3D() ;
			pPLine3D->m_bFeedType = JDNC_FEEDTYPE_ROUGH ;
			pPLine3D->m_pTAPos = new TPNT3D[3] ;
			pPLine3D->m_nCount = 2 ;
			nc_VectorCopy(pPLine3D->m_pTAPos[0], start, 3 ) ;
			nc_VectorCopy(pPLine3D->m_pTAPos[1], mid, 3 ); 
			nc_VectorCopy(pPLine3D->m_pTAPos[2], end, 3 ) ;
			pTComb->AddEntity( pPLine3D ) ;
		}
		delete pTLoop ;
		delete pChkLoop ;
	}
	tmpComb.GetEndPoint(1, start ) ;
	return TRUE ;
}
#include "StdAfx.H"
#include "Global.h"
#include "SurfNC.h"			//为使用SurfNC_GetJDCalSetup()而添加
#include "SurfGeo.H"
#include "SmartNC.H"
#include "SysPrompt.H"
#include "SmtPathGen2D.H"
#include "SmtPathGen3D.H"
#include "NcHeightMdl.H"
#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

#define NCTOL_TRIMBND_ZMOVE 0.01
#define NCTOL_FILLET_PLANE  0.03
/////////////////////////////////////////
//CSmartFPlaneGen : 成组平面雕刻
//
CSmartFPlaneGen::CSmartFPlaneGen()
{
}


CSmartFPlaneGen::~CSmartFPlaneGen()
{
}
void MathCAM_SetContoursHeight( CSmartLoop* pNewLp, DOUBLE Height )
{
    for( CSmartLoop* pLp = pNewLp ; pLp ; pLp = pLp->next )
    {
        pLp->m_dHeight = Height ;
        for( CSmartLoop* pIsl = pLp->m_pIsland ; pIsl ; pIsl = pIsl->next )
        {
            pIsl->m_dHeight = Height ;
        }
    }
}
void MathCAM_ZMoveContours( CSmartLoop* BndCont, DOUBLE ZMove )
{
    for( CSmartLoop* pCont = BndCont ; pCont ; pCont = pCont->next )
    {
        pCont->m_dHeight += ZMove ;
        for( CSmartLoop* pIsl = pCont->GetIsland() ;pIsl ; pIsl = pIsl->next )
        {
            pIsl->m_dHeight += ZMove ;
        }
    }
}

extern void MathCAM_ResetPathDepth( CSmtCPathLib& AllPath, int Flag );

CSmartTool* MathCAM_FilletFPlaneTool( CSmartTool& OrgTool, 
                                      BOOL IsFPlane, 
                                      DOUBLE& SlopeAngle )
{
    if( OrgTool.GetType() == smtToolBall )
    {/*校正最小角度*/
        if( OrgTool.m_fRadius > 0.03 )
        {
            double dMaxSlope = asin((OrgTool.m_fRadius-0.02)/OrgTool.m_fRadius) ;
            if( SlopeAngle > dMaxSlope ) SlopeAngle = dMaxSlope ;
        }
        return NULL ; 
    }
    if( OrgTool.GetType() != smtToolFlat  
       && OrgTool.GetType() != smtToolNose )
    {/*类型不满足*/
        return NULL ; 
    }
    DOUBLE dRadius = OrgTool.m_fRadius ;
    DOUBLE dCorner = max( 0.1, 0.02 / ( 1.0 - sin( SlopeAngle ) ) ) ;
    if( IsFPlane ) dCorner = NCTOL_FILLET_PLANE ;
    dCorner = min( dCorner, 0.8 * dRadius ) ;
    if( dCorner > 0.03 )
    {
        double dMaxSlope = asin((dCorner-0.02)/dCorner) ;
        if( SlopeAngle > dMaxSlope ) SlopeAngle = dMaxSlope ;
    }
    if( OrgTool.GetType() == smtToolNose && 
        dCorner < OrgTool.m_fCorner-0.001 )
    {/*不需要导圆角*/
        return NULL ;
    }
    CSmartTool* pNoseTool = new CSmtToolNose() ;
    pNoseTool->SetParam( dRadius, dRadius - dCorner,0.0, dCorner ) ; 
    pNoseTool->RefineParam() ;
    return pNoseTool ;
}
BOOL CSmartFPlaneGen::CreateBoundArea( CSmtCheckMdl& DriveMdl, 
                                       CSmartLoop* AllCont   ,
                                       CSmartLoop*& BndCont  , 
                                       JDNC_PRGDEF& PrgDef   )
{
    // STEP 1 : 计算成组平面的角度
	if( DriveMdl.m_pTool == NULL ) return FALSE ;
    BOOL bFPlane= ( m_cParam.m_bPlaneFlag & NCDEF_PLANE_FLATSURF)? FALSE : TRUE ;
    double dSlopeAngle = 0.725 ;
    if( bFPlane == TRUE )
    {
		DriveMdl.SetOnlyFlatCalcMode( TRUE ) ; 
    }
    else 
    {
		DriveMdl.SetOnlyFlatCalcMode( FALSE );
        dSlopeAngle = m_cParam.m_dSlopeAngle ;
	    if(dSlopeAngle > 45.0 ) dSlopeAngle -= 0.225 ;
        else dSlopeAngle += 0.225 ;
    }
    dSlopeAngle = ANGLE_TO_RADIAN( dSlopeAngle ) ;
    // STEP 2 : 修正平底刀具及尺寸,提高边界质量和稳定性
    JDNC_TOL oldTol = m_cSetupDef.m_cTolDef ;
    m_cSetupDef.m_cTolDef.m_dArcTol = min( 0.002, m_cSetupDef.m_cTolDef.m_dArcTol ) ;
    DOUBLE  dArcTol = min( 0.005, m_cSetupDef.m_cTolDef.m_dArcTol );   
    CSmartTool*  pOrgDriveTool = NULL, *pNoseTool = NULL ; 
    pNoseTool = MathCAM_FilletFPlaneTool( *DriveMdl.m_pTool, 
                                          bFPlane ,
                                          dSlopeAngle ) ;
    if( pNoseTool )
    {/*平底刀或小圆角牛鼻刀需要修正圆角半径 */
        pOrgDriveTool = DriveMdl.m_pTool ;
        DriveMdl.UpdateCurrTool( pNoseTool ) ;
    }
    // STEP 3 : 构建网格模型, 提取可加工区域的边界线
    CSmtMeshMdl   MeshMdl;
    MeshMdl.m_bMeshFlag |= SMARTNC_MESHMDL_NODEBUGDLG ;
    if( dSlopeAngle > 0.0 && bFPlane == FALSE )
    {/*平坦面加工模式*/
        MeshMdl.m_nMeshMode = SMARTNC_MESHMDL_MIXTURE  ;
    }
    double dTotalMove = PrgDef.m_dTotalMove ;
    m_cPrgDef.m_dTotalMove = 0.5 * dTotalMove ;
    int bRet = BuildMeshMdl(MeshMdl, DriveMdl, AllCont, TRUE ) ;
    if( bRet != FALSE ) 
    { /*转换边界曲线*/
        PrgDef.m_dTotalMove = 0.5 * dTotalMove ;
	    if( bFPlane == TRUE )
	    {
            BndCont = MeshMdl.GenZPlaneContours(2.0f*dArcTol, PrgDef );
	    }
        else 
        {
	        double		dOffDist= max( 0.01, 2.0f * dArcTol );
            CSmtCPathLib  bndCLib ;
    	    MeshMdl.ExtractFlatAreaBound( (TFLOAT) RADIAN_TO_ANGLE(dSlopeAngle), bndCLib, PrgDef ) ;
            if(  RADIAN_TO_ANGLE(dSlopeAngle) < 15.0 )
            {
                JDNC_TOL tmpTol = m_cSetupDef.m_cTolDef ;
                tmpTol.m_dArcTol = max( tmpTol.m_dArcTol, 0.01 ) ;
                POSITION atPos = bndCLib.m_cAllPath.GetHeadPosition() ;
                while( atPos )
                {
                    CSmtCutPath* pTPath = (CSmtCutPath*)bndCLib.m_cAllPath.GetNext(atPos) ;
                    pTPath->FairBoundPath( DriveMdl, m_cSetupDef.m_cTolDef ) ;
                }
            }
		    BndCont = bndCLib.ConvertToContours(FALSE, m_cSetupDef.m_cCorDef, dOffDist );
        }
    }
	if(bFPlane==TRUE) 
    {//去掉平坦面加工标志
		DriveMdl.SetOnlyFlatCalcMode( FALSE ) ;
    }
    // STEP 7 : 还原原始刀具
    if( pNoseTool )
    {
        DriveMdl.UpdateCurrTool( pOrgDriveTool ) ;
        delete pNoseTool ;
    }
    DOUBLE dRadSize = DriveMdl.m_pTool->GetRadiusComp( 0.0,0.0 ) ;
    CSmartLoop* pLpNext = NULL ; 
	for(CSmartLoop*	pLoop = BndCont; pLoop; pLoop=pLpNext)
    {
        pLpNext = pLoop->next ;
        if( bFPlane == FALSE && RADIAN_TO_ANGLE(dSlopeAngle) < 15.0 &&
           ( pLoop->m_dArea < 0.2 || pLoop->m_pCurve->GetLength() < 1.0 ))
        {
            BndCont = Mini_RemoveLoop( BndCont, pLoop ) ;
            delete pLoop ;
            continue ;
        }
        CSmartLoop* pIsl , *pIslNext = NULL ; 
        for( pIsl = pLoop->m_pIsland ; pIsl ; pIsl = pIslNext )
        {
            pIslNext = pIsl->next ;
            if( (pIsl->m_dBox[1][0] - pIsl->m_dBox[0][0] < dRadSize) && 
                (pIsl->m_dBox[1][1] - pIsl->m_dBox[0][1] < dRadSize)  )
            {
                pLoop->m_pIsland = Mini_RemoveLoop( pLoop->m_pIsland, pIsl ) ;
                delete pIsl ;
            }
        }
    }
    m_cSetupDef.m_cTolDef = oldTol ;
    if( SurfNC_IsAbort() ) return FALSE ;
    return bRet ;
}


BOOL CSmartFPlaneGen::GenProjPocketPath( CSmtCheckMdl& /*DriveMdl*/, 
                                         CSmartLoop* BndCont   ,
                                         CPathGroup& PGroup    )
{	//绝对平面才是粗加工,添加下刀
	CSmartPocketGen  PocketGen ; 
	(*(CSmartPathGen*)&PocketGen ) = * this ;
	
	//构建参数
	JDNC_POCKET		cPocketParam;
	cPocketParam.m_nMoveType=m_cParam.m_nMoveType;
	cPocketParam.m_bPocketFlag=0;
	if(m_cParam.m_nMoveType==0)
		cPocketParam.m_cLinear=m_cParam.m_cLinear;
	else if(m_cParam.m_nMoveType==1)
    {
		cPocketParam.m_cFollow=m_cParam.m_cFollow;
        if( (m_cParam.m_bPlaneFlag & NCDEF_PLANE_BOUNDTOL )  && 
            m_cParam.m_dBoundTol > 0.01 )
        {
            PocketGen.m_dBoundTol = m_cParam.m_dBoundTol ;
            PocketGen.m_dFeedRatio= m_cParam.m_dFeedRatio; 
        }
    }
	else if(m_cParam.m_nMoveType==2)
		cPocketParam.m_cSpiral=m_cParam.m_cSpiral;
	PocketGen.m_cSpeedDef.m_dPlungeDist = 0 ;
    PocketGen.UpdateTool2D( ) ;

    DOUBLE dDepthInc = 0.0, dZAt = 0.0, dZBase = 0.0 ;
    int nLayerCount = 1 ;
    if( m_cFeedDef.m_cLayerDef.m_nLayerType == NCDEF_LAYER_COUNT && 
        m_cFeedDef.m_cLayerDef.m_nLayerCount > 0 )
    {
        nLayerCount = m_cFeedDef.m_cLayerDef.m_nLayerCount ;
        dDepthInc   = m_cFeedDef.m_cLayerDef.m_dSideDInc   ;
        dZBase   = nLayerCount * dDepthInc ;
    }
	for(CSmartLoop* pLoop=BndCont; pLoop; pLoop=pLoop->next)
    {
        CPathCombine	*pNewPComb = new CPathCombine(NC_WPROCESS_ROUGH);
        double dBtmAt = pLoop->m_dHeight ;
        for( CSmartLoop* pIsl = pLoop->GetIsland() ; pIsl ; pIsl = pIsl->next )
        {
            if( pIsl->m_dHeight  > dBtmAt ) dBtmAt = pIsl->m_dHeight ;
        }
        dZAt = dZBase ;
        for( int nLayer = 1 ; nLayer <= nLayerCount ; nLayer ++ ) 
        {
	        CPathCombine	tmpCb(NC_WPROCESS_ROUGH);
		    PocketGen.PocketOneLayer(tmpCb, *pLoop, cPocketParam, dDepthInc, NULL);
            dZAt -= dDepthInc ;
            tmpCb.MoveCombine( 0.0, 0.0, dZAt  ) ;
            pNewPComb->AppendCombine( tmpCb ) ;
        }
		// 去掉第一刀的慢速下刀路径，改在后面整体添加所有的慢速下刀路径
        //if( pNewPComb->m_pHead ) 
        //{
        //    AddHeadPlungePath( pNewPComb ) ;
        //}
        PGroup.AddData( -dBtmAt, pNewPComb , TRUE ) ;
    }
	return TRUE;
}


// 跟踪精确的平面加工边界
BOOL CSmartFPlaneGen::TracingPlaneAreaBound( CSmtCheckMdl& DriveMdl, 
                                            CSmartLoop*& BndCont  )
{
    JDNC_PRGDEF tmpPrg = m_cPrgDef ;
    tmpPrg.m_pPosFunc = NULL; 
    tmpPrg.m_pPrgFunc = NULL; 
    tmpPrg.m_pBrkFunc = NULL;
    CSmtLoopArr allLp ;
	for(CSmartLoop*	pLoop = BndCont; pLoop; pLoop=pLoop->next)
    {
        pLoop->ExtractAllLoop( 0, allLp ) ;
    }
	DriveMdl.SetOnlyFlatCalcMode( TRUE ) ;
    INT_PTR nSize = allLp.GetSize() ;
    TFLOAT fBtmAt = DriveMdl.m_fBottom ;
    for( INT_PTR k = 0 ; k < nSize ; k ++ ) 
    {/*跌代跟踪，避免出现平底刀计算误差*/
        pLoop = allLp.GetAt( k ) ;
        CSmtCutPath tmpPath ;
        tmpPath.AddCurve( *pLoop->m_pCurve, m_cSetupDef.m_cTolDef ) ;
        TFLOAT fZValue = (TFLOAT)pLoop->m_dHeight ;
        DriveMdl.m_fBottom = (TFLOAT)max( fBtmAt, pLoop->m_dHeight) ;
        tmpPath.VerifyCutPath( DriveMdl, m_cSetupDef.m_cTolDef, tmpPrg ) ;
        tmpPath.DefineBox() ;
        BOOL bFindPlane = FALSE ;
        for( CSmtCutPoint* pTPnt = tmpPath.m_pHead ; pTPnt ; pTPnt = pTPnt->next )
        {
            if( pTPnt->m_bType > 0 )
            {
                bFindPlane = TRUE ;
                break ;
            }
        }
        if( tmpPath.m_fBox[1][2] - tmpPath.m_fBox[0][2] < 4.0e-4 && bFindPlane )
        {/*满足平面要求*/
            continue ;
        }
        CSmartLoop *pOffInn = NULL ; 
        pOffInn = pLoop->OffsetLoopEx( 0.03, m_cSetupDef.m_cCorDef ) ;
        for( int nOffTry = 1 ; nOffTry <= 3 && !bFindPlane; nOffTry ++ ) 
        {/*重新寻找ZValue*/
            if( nOffTry != 1)
            {
                Mini_DeleteContours( pOffInn ) ;
                pOffInn = pLoop->OffsetLoopEx( 0.03*nOffTry, m_cSetupDef.m_cCorDef ) ;
            }
            if( pOffInn == NULL ) break ;
            CSmtCutPath innPath ;
            for( CSmartLoop* pIsl = pOffInn ; pIsl ; pIsl = pIsl->next )
            {
                innPath.AddCurve( *pIsl->m_pCurve, m_cSetupDef.m_cTolDef ) ;
                innPath.VerifyCutPath( DriveMdl, m_cSetupDef.m_cTolDef, tmpPrg ) ;
                innPath.DefineBox() ;
                for( pTPnt = innPath.m_pHead ; pTPnt ; pTPnt = pTPnt->next )
                {
                    if( pTPnt->m_bType > 0 )
                    {
                        if( pTPnt->m_fPoint[2] < fZValue )
                        {
                            fZValue = pTPnt->m_fPoint[2] ;
                        }
                        bFindPlane = TRUE ;
                    }
                }
                innPath.ClearAllPoint() ;
            }
            if( bFindPlane ) break ;
        }
        fZValue += 3.0e-4f ;
        for( pTPnt = tmpPath.m_pHead; pTPnt && pOffInn; pTPnt = pTPnt->next )
        {
            FPNT3D fTPoint, fStart, fEnd, fDir, fBox3D[2];
            if( pTPnt->m_fPoint[2] < fZValue )
            {
                continue ;
            }
            nc_VectorCopy( fStart, pTPnt->m_fPoint, 3 ) ;
            nc_InitBox3D( fStart, fStart, fBox3D ) ;
            nc_ExpandBox3D( fBox3D, 0.2f, TRUE ) ;
            DriveMdl.LabelCheckByBox( fBox3D ) ;
            DOUBLE dMinDist = 1000.0 , dSeed[2], dAt[2] ;
            nc_FloatToDouble( dSeed, fStart, 2 ) ;
            for( CSmartLoop* pIsl = pOffInn ; pIsl ; pIsl = pIsl->next )
            {
                double dDist = 0.0 ;
                pIsl->m_pCurve->MinDistSect( dSeed, dAt, dDist ) ;
                if( dDist < dMinDist )
                {
                    fDir[0] = (TFLOAT)(dSeed[0] - dAt[0]) ;
                    fDir[1] = (TFLOAT)(dSeed[1] - dAt[1]) ;
                    nc_Normalize( fDir, 2 ) ;
                    dMinDist = dDist ;
                }
            }
            fDir[2] = 0.0f ;
            TFLOAT fZ[3] = { 0.0f, 0.0f, 1.0f } ;
            TFLOAT fBase[3] = { 0.0f, 0.0f, 0.0f } ;
            TFLOAT fOld[3] ; 
            nc_VectorCopy( fOld, fDir, 3 ) ;
            for( int nRotAt = 0 ; nRotAt <= 5 ; nRotAt ++ )
            {
                TFLOAT fLen = 0.002f ;
                for( int nTry = 0 ; nTry <= 15 ; nTry ++ ) 
                {
                    fEnd[0] = fStart[0] - fLen * fDir[0];
                    fEnd[1] = fStart[1] - fLen * fDir[1];
                    fEnd[2] = fZValue - 0.1f ;
                    DriveMdl.DefineHeight( fEnd ) ;
                    if( fEnd[2] < fZValue ) break ;
                    if( nTry < 5 ) fLen += 0.005f ;
                    else fLen += 0.015f ;
                    if( fLen > 0.2f ) break ;
                }
                if( fEnd[2] < fZValue )
                {
                    break ;
                }
                nc_VectorCopy( fDir, fOld, 3 ) ;
                if( nRotAt == 0 ) nc_RotatePoint( fBase, fZ, fDir, (TFLOAT)ANGLE_TO_RADIAN( -10.0 ) ) ;
                else if( nRotAt == 1 ) nc_RotatePoint( fBase, fZ, fDir, (TFLOAT)ANGLE_TO_RADIAN( 10.0 ) ) ;
                else if( nRotAt == 2 ) nc_RotatePoint( fBase, fZ, fDir, (TFLOAT)ANGLE_TO_RADIAN( -25.0 ) ) ;
                else if( nRotAt == 3 ) nc_RotatePoint( fBase, fZ, fDir, (TFLOAT)ANGLE_TO_RADIAN( 25.0 ) ) ;
            }
            if( fEnd[2] > fZValue )
            {
                continue ;
            }
            if( DriveMdl.CalcTPointAtZ( fStart, fEnd,   fZValue, fTPoint ) )
            {
                nc_VectorCopy( pTPnt->m_fPoint, fTPoint, 3 ) ;
            }
        }
        nc_VectorCopy( tmpPath.m_pHead->m_fPoint, tmpPath.m_pTail->m_fPoint, 3 ) ;
        tmpPath.DelPointOnLine( 2.0e-5 ) ;
        Mini_DeleteContours( pOffInn ) ;
        pLoop->m_pCurve->ClearAll() ;
        for( pTPnt = tmpPath.m_pHead; pTPnt && pTPnt->next; pTPnt = pTPnt->next )
        {
            PNT3D dStart, dEnd ;
            nc_FloatToDouble( dStart, pTPnt->m_fPoint, 3 ) ;
            nc_FloatToDouble( dEnd, pTPnt->next->m_fPoint, 3 ) ;
            pLoop->m_pCurve->AddSect( new CSmartLine( dStart, dEnd ) );
        }
        pLoop->RemoveAutoIntSect() ; 
        pLoop->DefineBox() ;
        tmpPath.ClearAllPoint() ;
    }
    DriveMdl.m_fBottom = fBtmAt ;
	DriveMdl.SetOnlyFlatCalcMode( FALSE ) ;
    return TRUE ;
}
CSmartLoop* MathCAM_CreateEdgeLoop( PNT2D dStart, PNT2D dEnd, DOUBLE Offset )
{
    if( nc_Distance( dStart, dEnd, 2 ) < 2.0e-6 )
    {
        return NULL ; 
    }
    double dMinAng  = atan2( dStart[1] - dEnd[1], dStart[0] - dEnd[0] ) ;
    double dAng[2] ; 
    dAng[0] = dMinAng - MiniPai1_2 , dAng[1] = dMinAng + MiniPai1_2 ;
    CSmartArc* pArc1 = new CSmartArc( dStart, dAng, Offset) ;
    dAng[0] = dMinAng + MiniPai1_2 , dAng[1] = MiniPai2 + dMinAng - MiniPai1_2 ;
    CSmartArc* pArc2 = new CSmartArc( dEnd  , dAng, Offset) ;
    PNT2D dPnt[2] ;
    pArc1->GetEnd( dPnt[0] ) , pArc2->GetStart( dPnt[1] ) ;
    CSmartLine *pLine1 = new CSmartLine( dPnt[0], dPnt[1] ) ;
    pArc2->GetEnd( dPnt[0] ) , pArc1->GetStart( dPnt[1] ) ;
    CSmartLine *pLine2 = new CSmartLine( dPnt[0], dPnt[1] ) ;
    CSmartCurve* pCurve = new CSmartCurve() ;
    pCurve->AddSect( pArc1 ) ;
    pCurve->AddSect( pLine1 ) ;
    pCurve->AddSect( pArc2 ) ;
    pCurve->AddSect( pLine2 ) ;
    CSmartLoop* pLoop = new CSmartLoop() ;
    pLoop->UpdateSect( pCurve ) ;
    return pLoop ;
}
CSmartLoop* MathCAM_CreateBoundLoop_New( CSmartCurve& AllEdge , 
                                         DOUBLE Offset    )
{
    CSmartLoop* pLoop , *pTmpHead = NULL ; 
    PNT2D dStart, dEnd ;
    int nID = 1 ; 
    for( CSmartSect* pEdge = AllEdge.m_pHead ; pEdge ; pEdge = pEdge->next )
    {
        pEdge->GetStart( dStart ) ;
        pEdge->GetEnd( dEnd ) ;
        pLoop = MathCAM_CreateEdgeLoop( dStart,dEnd,Offset );
        pLoop->m_nGroupId = nID ; 
        pTmpHead = Mini_AddContours( pTmpHead, pLoop ) ;
        nID ++ ; 
    }
    CSmartLoop tmpLoop ;
    pTmpHead = tmpLoop.WeldContours( pTmpHead ) ;
    return pTmpHead ;
}
void MathCAM_FittingByArc( CSmartCurve& Curve , DOUBLE FitTol[2]) 
{
    int		nCnt = 1;
    for( CSmartSect* pSect = Curve.m_pHead ; pSect ; pSect = pSect->next )
    {
        nCnt ++ ;
        if( pSect->IsSectArc() ) return ;
    }
    PNT2D*	dPnt	=new PNT2D[nCnt];
	PNT2D*	dMidPnt =new PNT2D[nCnt];
    Curve.GetStart( dPnt[0] ) ;
	nCnt=1;
    for( pSect = Curve.m_pHead ; pSect ; pSect = pSect->next )
    {
        pSect->GetEnd( dPnt[nCnt] ) ;
		dMidPnt[nCnt-1][0]=0.5*(dPnt[nCnt-1][0]+dPnt[nCnt][0]);
		dMidPnt[nCnt-1][1]=0.5*(dPnt[nCnt-1][1]+dPnt[nCnt][1]);
		nCnt++;
	}
    Curve.ClearAll() ;
    Curve.AddPointArray( dPnt , nCnt-1 , FitTol, dMidPnt);
	Curve.LinearCurve(1.0e-4);
	delete[] dPnt;
	delete[] dMidPnt;
}

CSmartLoop* MathCAM_CreateBoundLoop( CSmartCurve& AllEdge , 
                                    DOUBLE Offset    )
{
    DOUBLE dFitTol[2] = { 0.005, cos( 0.17 ) };
    CSmartCurveLib  surfCLib ;
    AllEdge.ConnectToCurveLib( surfCLib, 1.0e-3 ) ;
    JDNC_COR tmpCor ;
    SurfNC_InitPathParam( tmpCor ) ;
    tmpCor.m_nCorType = NCDEF_CORNER_ARCALL ;
    CSmartLoop* pTmpHead = NULL ; 
    int nID = 1 ;
    for( CSmartCurve* pCv = surfCLib.m_pHead ; pCv ; pCv = pCv->next )
    {
        pCv->ConnectAllSect( FALSE ) ;
        MathCAM_FittingByArc( *pCv, dFitTol ) ;
        CSmartLoop* pBndLp = pCv->ExpendCurveArcEnd( Offset, tmpCor ) ;
        for( CSmartLoop* pLp = pBndLp ; pLp ; pLp = pLp->next )
        {
            pLp->m_nGroupId = nID ;
        }
        pTmpHead = Mini_AddContours( pTmpHead, pBndLp ) ;
        nID ++ ;
     }
     CSmartLoop tmpLoop ;
     pTmpHead = tmpLoop.WeldContours( pTmpHead ) ;
     return pTmpHead ;
}
void MathCAM_SetLimitContour( CSmartLoop& Cont )
{
    for( CSmartSect* pSect = Cont.GetSectHead() ; pSect ; pSect = pSect->next )
    {
        pSect->m_bEndFlag &= ~NC_BOUND_SECT  ;
        pSect->m_bEndFlag |=  NC_LIMIT_SECT  ;
    }
    for( CSmartLoop* pIsl = Cont.m_pIsland ; pIsl ; pIsl = pIsl->next )
    {
        for( pSect = pIsl->GetSectHead() ; pSect ; pSect = pSect->next )
        {
            pSect->m_bEndFlag &= ~NC_BOUND_SECT  ;
            pSect->m_bEndFlag |=  NC_LIMIT_SECT  ;
        }
    }
}

// 修剪无效平面加工边界
CSmartLoop* CSmartFPlaneGen::TrimingInvalidPlaneArea( CSmtCheckMdl& DriveMdl, 
                                                      DOUBLE BtmRad ,
                                                      CSmartLoop* BndCont  )
{
    if( BtmRad < 0.05 ) return BndCont ;
    JDNC_PRGDEF tmpPrg = m_cPrgDef ;
    tmpPrg.m_pPosFunc = NULL; 
    tmpPrg.m_pPrgFunc = NULL; 
    tmpPrg.m_pBrkFunc = NULL;
    CSmtLoopArr allLp ;
    BndCont->ExtractAllLoop( 0, allLp ) ;
    INT_PTR nNumLp = allLp.GetSize() ;
    DOUBLE delLen = 0.0;
    CSmartCurve   remSect, delSect ;
    TFLOAT fBtmZAt  = DriveMdl.m_fBottom ;
    TFLOAT fZAt = (TFLOAT)(BndCont->m_dHeight + NCTOL_TRIMBND_ZMOVE) ;
    for( INT_PTR k = 0 ; k < nNumLp ; k ++ ) 
    {
        CSmartLoop* pLp = allLp.GetAt( k ) ;
        CSmtCutPath tmpPath ;
        tmpPath.AddCurve( *pLp->m_pCurve, m_cSetupDef.m_cTolDef ) ;
        DriveMdl.m_fBottom = fZAt - 0.02f ;
        tmpPath.VerifyCutPath( DriveMdl, m_cSetupDef.m_cTolDef, tmpPrg ) ;
        for( CSmtCutPoint* pTPnt = tmpPath.m_pHead ; pTPnt ; pTPnt = pTPnt->next )
        {
            if( pTPnt->m_fPoint[2] > fZAt - 0.0004f )
            {
                pTPnt->m_fPoint[2] = fZAt + 0.02f ;
            }
            else
            {
                pTPnt->m_fPoint[2] = fZAt - 0.02f ;
            }
        }
        tmpPath.DelPointOnLine( 0.008 ) ;
        for( CSmtCutPoint* pTPnt = tmpPath.m_pHead ; pTPnt && pTPnt->next; pTPnt = pTPnt->next )
        {
            DOUBLE dStart[3], dEnd[3] ;
            nc_FloatToDouble( dStart, pTPnt->m_fPoint, 3 ) ;
            nc_FloatToDouble( dEnd, pTPnt->next->m_fPoint, 3 ) ;
            CSmartLine * pLine = new CSmartLine( dStart, dEnd ) ;
            if( dStart[2] > fZAt || dEnd[2] > fZAt  )
            {
                remSect.AddSect( pLine ) ;
            }
            else
            {
                delSect.AddSect( pLine ) ;
                delLen += pLine->GetLength() ;
            }
        }
    }
    DriveMdl.m_fBottom = fBtmZAt ;
    if( delSect.m_pHead == NULL || delLen < 2.0 * BtmRad )
    {/*全部为保留区域*/
        return  BndCont ;
    }
    else if( remSect.m_pHead == NULL )
    {
        MathCAM_SetLimitContour( *BndCont ) ;
        CSmartLoop* pNewLp = BndCont->OffsetContourEx( BtmRad, BtmRad,m_cSetupDef.m_cCorDef ) ;
        if( pNewLp == NULL ) return BndCont ;
        MathCAM_SetContoursHeight( pNewLp, BndCont->m_dHeight ) ;
        delete BndCont ;
        return pNewLp ;
    }
    CSmartLoop* delLp = MathCAM_CreateBoundLoop( delSect, BtmRad ) ;
    if( delLp == NULL ) 
    {
        return BndCont ;
    }
    CSmartLoop* remLp = MathCAM_CreateBoundLoop( remSect, 0.26*BtmRad) ;
    if( remLp == NULL )
    {
        Mini_DeleteContours( delLp ) ;
        return remLp ;
    }
    CSmartLoop* lpCut = Mini_OperateContours( delLp, remLp, 2 );
    Mini_DeleteContours( delLp ) ;
    Mini_DeleteContours( remLp ) ;
    if( lpCut == NULL ) 
    {
        return BndCont ;
    }
    for( CSmartLoop* pLp = lpCut; pLp ; pLp = pLp->next )
    {
        MathCAM_SetLimitContour( *pLp ) ;
    }
    remLp = Mini_OperateContours( BndCont, lpCut, 2 );
    Mini_DeleteContours( lpCut ) ;
    if( remLp == NULL ) 
    {
        return BndCont ;
    }
    MathCAM_SetContoursHeight( remLp, BndCont->m_dHeight ) ;
    delete BndCont ;
    return remLp ;
}
int CSmartFPlaneGen::GeneratePathEx( CPathGroup&    NewPath    ,  /*雕刻路径*/
				                     CSmartGraphic& Graph     )  /*雕刻图形*/
{// 生成刀具路径 
    // STEP 1 : 构造加工曲面
	if( !UpdateTool3D(Graph) )
    {
        return FALSE ;
    }
	CSmartLoop*		AllLoop=Graph.GetAllContour(m_cSetupDef);
    if( AllLoop ) OffsetBorder( AllLoop );
	CSmtCheckMdl  DriveMdl, CheckMdl ;
    if( ! BuildDriveAndCheckMdl( Graph, DriveMdl, CheckMdl ) )
	{
        Mini_DeleteContours( AllLoop ) ;
        m_nErrorType = JDERROR_GENPATH_NOSURFACE  ;
        return FALSE ;
	}
	if( m_nCalcThreadNum > 1 )
	{
		DriveMdl.CreateMultiCopy( m_nCalcThreadNum-1 ) ;
	}
    DOUBLE fZShiftDist = m_cSetupDef.m_cTolDef.m_dArcTol ;
	if( !(m_cParam.m_bPlaneFlag & NCDEF_PLANE_FLATSURF) )
    {/*平面路径*/
        if( DriveMdl.ZMovePlaneSurf( fZShiftDist ) == FALSE )
        {
            Mini_DeleteContours( AllLoop ) ;
            m_nErrorType = JDERROR_GENPATH_NOPATH  ;
            return TRUE ;
        }
        m_dTopHeight += fZShiftDist ;
        m_dCutDepth  += fZShiftDist ;
    }
    // STEP 2 : 构造加工边界生成路径
	ProcessNewFunc( m_cPrgDef, IDS_PATHGEN_STEP_FPLANEPATH ) ;

    CSmartLoop*  BndCont = NULL  ; 
    BOOL bRetFlag = FALSE ; 
    m_cPrgDef.m_dTotalMove = 100.0 ;
	if( !(m_cParam.m_bPlaneFlag & NCDEF_PLANE_FLATSURF) && 
        (m_cSetupDef.m_cModelTol.m_nMdlFlag & NCDEF_SETUP_DELBNDPNT ) )
    {/*平面路径*/
        bRetFlag = CreateBoundArea( DriveMdl, NULL, BndCont, m_cPrgDef );
    }
    else
    {
        bRetFlag = CreateBoundArea( DriveMdl, AllLoop, BndCont, m_cPrgDef );
        Mini_DeleteContours( AllLoop ) ;
    }
    if( bRetFlag == FALSE )
    {
        Mini_DeleteContours( AllLoop ) ;
        m_nErrorType = JDERROR_GENPATH_ABORT  ;
        return FALSE ;
    }
	double dOverRatio =  m_cFeedDef.m_cStepDef.m_dOverRatio ;
    if( m_cParam.m_nMoveType == surfncMoveFollow )
	{
		m_cParam.m_cFollow.m_dOverRatio = dOverRatio ;
	}
	else if( m_cParam.m_nMoveType == surfncMoveLinear )
	{
		m_cParam.m_cLinear.m_dOverRatio = dOverRatio ;
	}
	else if( m_cParam.m_nMoveType == surfncMoveSpiral )
	{
		m_cParam.m_cSpiral.m_dOverRatio = dOverRatio ;
	}
    BOOL bRet = FALSE ;
	CSmartLoop tmpLoop ;
	if( !(m_cParam.m_bPlaneFlag & NCDEF_PLANE_FLATSURF) )
    {/*平面路径*/
    //STEP 1 : 跟踪精确边界
        TracingPlaneAreaBound( DriveMdl,  BndCont) ;
        if(m_cSetupDef.m_cModelTol.m_nMdlFlag & NCDEF_SETUP_DELBNDPNT )
        {
            ReomvePlaneAreaOnEdge( DriveMdl,  BndCont) ;
            TrimingPlaneAreaByContours( BndCont, AllLoop ) ;
        }
	    BndCont  = tmpLoop.ResortContour( BndCont,NCDEF_SORTTYPE_MINDIST, FALSE ) ; 
        bRet = CreatePlaneCutPath( DriveMdl, BndCont, NewPath ) ;
    }
    else 
    {
	    BndCont  = tmpLoop.ResortContour( BndCont,NCDEF_SORTTYPE_MINDIST, FALSE ) ; 
        bRet = CreateFlatAreaPath( DriveMdl, BndCont, NewPath ) ;
    }
    Mini_DeleteContours( BndCont ) ;
    Mini_DeleteContours( AllLoop ) ;
	if( bRet && !(m_cParam.m_bPlaneFlag & NCDEF_PLANE_FLATSURF) )
    {/*平面路径*/
        NewPath.ZMoveGroup( - fZShiftDist ) ;
    }
	
	if( m_c5DCtrlDef.m_cToolAxis.m_nAxisType == NCDEF_AXIS_VERT )
	{
		DriveMdl.UpdateCurrTool(m_pTool);
		AddPlungeAndRelativeH( &DriveMdl, &NewPath, TRUE ) ;
	}

    return bRet ;
}
BOOL CSmartFPlaneGen::CreatePlaneCutPath( CSmtCheckMdl& DriveMdl, 
                                          CSmartLoop*  BndCont  ,
                                          CPathGroup&  NewPath  )
{
    //STEP 2:  缩小刀具投影避免干涉
    CSmartTool* pOldTool = DriveMdl.m_pTool ;
    CSmartTool* pNewTool = NULL  ;
    DOUBLE dOffTol = max( 0.01, 2.0 * m_cSetupDef.m_cTolDef.m_dArcTol) ;
    if( pOldTool->GetType() == smtToolFlat && pOldTool->m_fRadius > 2.0 * dOffTol )
    {
	    pNewTool=new CSmtToolFlat(); 
        pNewTool->SetParam( pOldTool->m_fRadius-dOffTol,	//顶半径
					            0.0 , 0.0, 0.0 ); //圆角半径
    }
    else if( pOldTool->GetType() == smtToolNose && pOldTool->m_fBottom > 2.0 * dOffTol )
    {
	    pNewTool=new CSmtToolNose(); 
        pNewTool->SetParam( pOldTool->m_fRadius-dOffTol,	//顶半径
            pOldTool->m_fBottom-dOffTol,0.0 , pOldTool->m_fCorner); //圆角半径
    }
    if( pNewTool )
    {
        DriveMdl.UpdateCurrTool( pNewTool ) ;
    }
    //STEP 3:  缩小刀具投影避免干涉
	int bRet = GenProjPocketPath( DriveMdl, BndCont, NewPath);
    if( pNewTool ) 
    {
        DriveMdl.m_pTool = NULL ; 
        delete pNewTool ;
    }
    return bRet ;
}
// 计算平坦面加工路径
BOOL CSmartFPlaneGen::CreateFlatAreaPath( CSmtCheckMdl& DriveMdl, 
                                          CSmartLoop*  BndCont  ,
                                          CPathGroup&  NewPath  )
{/*平坦面路径*/
	CSmartPocketGen  PocketGen ; 
	(*(CSmartPathGen*)&PocketGen ) = * this ;
	
	//构建参数
	JDNC_POCKET		cPocketParam;
    SurfNC_InitPathParam( cPocketParam ) ;
	cPocketParam.m_nMoveType=m_cParam.m_nMoveType;
	cPocketParam.m_bPocketFlag=0;
	if(m_cParam.m_nMoveType==surfncMoveLinear) cPocketParam.m_cLinear=m_cParam.m_cLinear;
	else if(m_cParam.m_nMoveType==surfncMoveFollow)	cPocketParam.m_cFollow=m_cParam.m_cFollow;
	else if(m_cParam.m_nMoveType==surfncMoveSpiral)	cPocketParam.m_cSpiral=m_cParam.m_cSpiral;
	PocketGen.m_cSpeedDef.m_dPlungeDist = 0 ;
    PocketGen.m_cFeedDef.m_cPlungeDef.m_nPlungeType = NCDEF_PLUNGE_CLOSE ;
    PocketGen.UpdateTool2D( ) ;
	CPathCombine	tmpPComb(NC_WPROCESS_ROUGH);
	for(CSmartLoop*	pLoop=BndCont; pLoop; pLoop=pLoop->next)
    {
	    CPathCombine	tmpCb(NC_WPROCESS_ROUGH);
		PocketGen.PocketOneLayer(tmpPComb, *pLoop, cPocketParam, 0.0, NULL);
    }
	CSmtCPathLib	AllPath  ;
	AllPath.CreateCutPath( tmpPComb, m_cSetupDef.m_cTolDef ) ;
	SurfNC_SetNewStepEx(IDS_PATHGEN_STEP_ORGPATH);
	AllPath.VerifyCutPath( DriveMdl, m_cSetupDef.m_cTolDef, m_cPrgDef ) ;
	if( SurfNC_IsAbort() )  return FALSE ;
	AllPath.DelPointOnLine(2.0e-4) ;
    AddLeadPathAndAddInPGroup(DriveMdl, AllPath, NewPath);
	return TRUE;
}
// 删除边界位置的加工区域
BOOL CSmartFPlaneGen::ReomvePlaneAreaOnEdge( CSmtCheckMdl& DriveMdl, 
                                             CSmartLoop*& BndCont  )
{
    DOUBLE dBtmRad = DriveMdl.m_pTool->GetRadiusComp( 0.0, 0.0 ) ;
    if( dBtmRad < 0.06 )
    {
        return TRUE ;
    }
    CSmartTool*  pOrgDriveTool = DriveMdl.m_pTool ; 
    CSmtToolFlat  flatTool ;
    DOUBLE dOffBtm = min( dBtmRad * 0.5 , 0.06 );
    flatTool.SetParam( dBtmRad - dOffBtm ,0.0 , 0.0, 0.0 ); //圆角半径
	flatTool.m_nToolID = DriveMdl.m_pTool->m_nToolID;
	strcpy( flatTool.m_sToolName, DriveMdl.m_pTool->m_sToolName ) ;
	flatTool.RefineParam();
    DriveMdl.UpdateCurrTool( &flatTool ) ;
    DriveMdl.ZMovePlaneSurf( NCTOL_TRIMBND_ZMOVE );
    CSmartLoop* pNewBnd = NULL, *pCont ;
    while( BndCont )
    {
        pCont = BndCont ;
        BndCont = BndCont->next ;
        pCont->next = pCont->prev = NULL ; 
        CSmartLoop* pLpHead = TrimingInvalidPlaneArea( DriveMdl, dBtmRad,  pCont ) ;
        pNewBnd = Mini_AddContours( pNewBnd , pLpHead ) ;
    }
    DriveMdl.UpdateCurrTool( pOrgDriveTool ) ;
    DriveMdl.ZMovePlaneSurf( -NCTOL_TRIMBND_ZMOVE );
    BndCont = pNewBnd ;
    return TRUE ;
}
void CSmartFPlaneGen::TrimingPlaneAreaByContours( CSmartLoop*& BndCont, CSmartLoop* AllLoop )
{
    if( AllLoop == NULL ) return ;
    CSmartLoop* pLpHead = NULL ; 
    while( BndCont )
    {
        CSmartLoop* pLpNext = BndCont->next ;
        BndCont->prev = BndCont->next = NULL ; 
        CSmartLoop* pTmpLp = Mini_OperateContours( BndCont, AllLoop , 0 ) ;
        MathCAM_SetContoursHeight( pTmpLp, BndCont->m_dHeight ) ;
        pLpHead = Mini_AddContours( pLpHead, pTmpLp ) ;
        Mini_DeleteContours( BndCont ) ;
        BndCont = pLpNext ;
    }
    BndCont = pLpHead ;
}
void CSmartFPlaneGen::TrimAllPlaneAreaByContours( CNcZLevelGenMdl* MeshSurf, 
												  CSmartLoop*& BndCont, 
												  CSmartLoop* AllLoop,
												  CPlaneList &cList )
{
    if( AllLoop == NULL ) return ;
	CSmartLoop *OrgBnd = NULL ;
    while( BndCont )
    {
        CSmartLoop* pLpNext = BndCont->next ;
        BndCont->prev = BndCont->next = NULL ; 
        CSmartLoop* pTmpLp = Mini_OperateContours( BndCont, AllLoop , 0 ) ;
        MathCAM_SetContoursHeight( pTmpLp, BndCont->m_dHeight ) ;
		if( pTmpLp )
		{
			MeshSurf->CreateContourLoop ( OrgBnd, BndCont->m_dHeight ) ;
			if( OrgBnd ) 
			{
				Mini_DeleteContours( BndCont ) ;
				BndCont = OrgBnd ;
			}
			CPlaneLoop *pPlaneLoop = new CPlaneLoop( pTmpLp, BndCont ) ;
			cList.AddTail ( pPlaneLoop ) ;
		}
		else
		{
			Mini_DeleteContours( BndCont ) ;
		}
        BndCont = pLpNext ;
    }
}
CSmartLoop* MathCAM_ExtractAllLoop( CSmartLoop* ContHead ) ; // 将环中的岛取出和环构成链表

BOOL CSmartFPlaneGen::SmoothPlaneAreaBound( CNcZLevelGenMdl* MeshSurf  ,
                                            CSmartLoop*&  BndCont   )
{/*边界倒圆角*/
    if( !(m_cSetupDef.m_cModelTol.m_nMdlFlag & NCDEF_SETUP_SMOOTHCORNER) ||
        m_cSetupDef.m_cTolDef.m_dMinRad <= 0.02 ||
        !MeshSurf || BndCont == NULL )
    {/*不需要光滑*/
        return TRUE ;
    }
	// 临时进度条
	JDNC_PRGDEF	PrgDef ;
	PrgDef.m_dLimitAt = 100 / ( 100 * 1.00  ) ;
	PrgDef.m_dStepAt  = 0 ;
	PrgDef.m_dIncStep = 1. ;
	PrgDef.m_pNewFunc = NULL ;
	PrgDef.m_pPosFunc = NULL ; 
	PrgDef.m_pBrkFunc = NULL ;
	PrgDef.m_pPrgFunc = NULL ;
    CSameZValueSet valueSet ;
	for(CSmartLoop*	pLoop = BndCont; pLoop; pLoop=pLoop->next)
    {
        valueSet.AddValue( TFLOAT( pLoop->m_dHeight) );
    }
    DOUBLE dZShiftTol = m_cSetupDef.m_cTolDef.m_dArcTol;
    int nNumZt = valueSet.m_nCount ;
    double *dZ = new double[ nNumZt + 1 ] ;
    CSmartLoop ** loopAt = new CSmartLoop*[nNumZt+1] ;
	for( int k= 0 ; k < nNumZt ; k++ )
    {
		loopAt[k] = NULL ;
        dZ[k] = valueSet.m_dValue[nNumZt-1-k] + dZShiftTol ;
    }
	if( !MeshSurf->CreateContourLoops ( dZ, loopAt, nNumZt, PrgDef ) )
	{
        delete[] dZ ;
        delete[] loopAt ;
        Mini_DeleteContours( BndCont ) ;
		return FALSE ;
	}
	for( k= 0 ; k < nNumZt ; k++ )
    {
        dZ[k] -= dZShiftTol ;
    }
    CSmartLoop  tmpLoop ;
    JDNC_COR tmpCor = m_cSetupDef.m_cCorDef ;
    tmpCor.m_nCorType = NCDEF_CORNER_ARCALL ; 
    DOUBLE dOffOut[2] = { m_cSetupDef.m_cTolDef.m_dMinRad+0.01,m_cSetupDef.m_cTolDef.m_dMinRad+0.01} ;
    DOUBLE dOffInn[2] = { m_cSetupDef.m_cTolDef.m_dMinRad,m_cSetupDef.m_cTolDef.m_dMinRad} ;
    for( k = 0 ; k < nNumZt ; k ++ ) 
    {
        loopAt[k] = MathCAM_ExtractAllLoop( loopAt[k] ) ;
		loopAt[k] = tmpLoop.BuildContour ( loopAt[k] ) ;
        CSmartLoop* pOldBnd = loopAt[k] ;
        loopAt[k] = NULL ; 
        int nGroupID = 1 ;
        for( CSmartLoop* pLp = pOldBnd ; pLp ; pLp = pLp->next )
        {
            CSmartLoop* pInnLp = pLp->OffsetContour( NCDEF_OFFSET_OUTER, dOffOut[0], dOffOut[1], tmpCor ) ;
            CSmartLoop* pNewLp = Mini_OffsetContourList( pInnLp, NCDEF_OFFSET_INNER, dOffInn, tmpCor ) ;
            Mini_DeleteContours( pInnLp ) ;
            for( CSmartLoop* pAt = pNewLp ; pAt ; pAt = pAt->next )
            {
                pAt->m_nGroupId = nGroupID ;
            }
            loopAt[k] = Mini_AddContours( loopAt[k], pNewLp ) ;
            nGroupID ++ ;
        }
        loopAt[k] = tmpLoop.WeldContours( loopAt[k] ) ;
        Mini_DeleteContours( pOldBnd ) ;
    }
    CSmartLoop* pOldBnd = BndCont ;
    BndCont = NULL ; 
    while( pOldBnd )
    {
        CSmartLoop* pNextLp = pOldBnd->next ;
        pOldBnd->next = pOldBnd->prev = NULL ;
        int nObj = -1 ;
        for( k = 0 ; k < nNumZt ; k ++ ) 
        {
            if( fabs( dZ[k] - pOldBnd->m_dHeight ) < 0.0002 )
            {
                nObj = k ; 
                break ;
            }
        }
        if( nObj == -1  || loopAt[nObj] == NULL )
        {
            BndCont = Mini_AddContours( BndCont, pOldBnd ) ;
        }
        else
        {
            CSmartLoop* pNewLp = Mini_OperateContours( pOldBnd, loopAt[nObj], 2) ;
            MathCAM_SetContoursHeight( pNewLp, pOldBnd->m_dHeight ) ;
            BndCont = Mini_AddContours( BndCont, pNewLp ) ;
            Mini_DeleteContours( pOldBnd ) ;
        }
        pOldBnd = pNextLp ; 
    }
	for( k= 0 ; k < nNumZt ; k++ )
    {
		Mini_DeleteContours( loopAt[k] );
    }
    delete[] loopAt ;
    delete[] dZ     ;
    return TRUE ;
}
int CSmartFPlaneGen::CreatePlaneContours( CSmtCheckMdl& DriveMdl  ,
										  CSmartLoop *AllLoop     ,
                                          CNcZLevelGenMdl* MeshSurf,
                                          CPlaneList &cList    ,
                                          JDNC_PRGDEF& PrgDef ) 
{// 生成平面加工区域
    CSmartLoop * BndCont = NULL ;
    if( DriveMdl.m_pTool == NULL )
    {
        return FALSE ;
    }
    BOOL bFindMCopy = FALSE ;
    if( DriveMdl.GetMultiCopy() != NULL ) 
    {/*本函数肯能破坏多线程模型*/
        bFindMCopy = TRUE ;
    }
    DOUBLE fZShiftDist = m_cSetupDef.m_cTolDef.m_dArcTol ;
    if( DriveMdl.ZMovePlaneSurf( fZShiftDist ) )
    {/*没有发现平面*/
        return TRUE ;
    }
    JDNC_DEPTH bakDepth = m_cShapeDef.m_cDepth ;
    m_dTopHeight += fZShiftDist ;
    m_dCutDepth  += fZShiftDist ;
    // STEP 2 : 构造加工边界
    if( ! CreateBoundArea( DriveMdl, NULL, BndCont,PrgDef ) )
    {
        Mini_DeleteContours( BndCont ) ;
        m_cShapeDef.m_cDepth = bakDepth ;
        m_nErrorType = JDERROR_GENPATH_ABORT  ;
        return FALSE ;
    }
    //STEP 3 : 跟踪精确边界
    TracingPlaneAreaBound( DriveMdl,  BndCont) ;
    if( m_cSetupDef.m_cTolDef.m_nFitType != NCDEF_FITTYPE_LINE )
    {
        CSmtLoopArr allLp ;
	    for(CSmartLoop*	pLoop = BndCont; pLoop; pLoop=pLoop->next)
        {
            pLoop->ExtractAllLoop( 0, allLp ) ;
        }
        DOUBLE dFitTol[2] = { m_cSetupDef.m_cTolDef.m_dArcTol , 
                             cos( ANGLE_TO_RADIAN( m_cSetupDef.m_cTolDef.m_dAngTol) ) };
        INT_PTR nSize = allLp.GetSize() ;
        for( INT_PTR k = 0 ; k < nSize ; k ++ ) 
        {
            pLoop = allLp.GetAt( k ) ;
            MathCAM_FittingByArc( *pLoop->m_pCurve, dFitTol ) ;
            pLoop->DefineBox() ;
        }
    }
    //STEP 4 : 光滑边界
    if( (m_cSetupDef.m_cModelTol.m_nMdlFlag & NCDEF_SETUP_SMOOTHCORNER) &&
        m_cSetupDef.m_cTolDef.m_dMinRad > 0.02 &&
        MeshSurf && BndCont )
    {/*需要光滑*/
        DriveMdl.ZMovePlaneSurf( -fZShiftDist );
        MathCAM_ZMoveContours( BndCont, - fZShiftDist ) ;
        if( SmoothPlaneAreaBound( MeshSurf,  BndCont) == FALSE )
        {
            Mini_DeleteContours( BndCont ) ;
            m_nErrorType = JDERROR_GENPATH_ABORT  ;
            return FALSE ;
        }
        DriveMdl.ZMovePlaneSurf( +fZShiftDist );
        MathCAM_ZMoveContours( BndCont, +fZShiftDist ) ;
    }
    //STEP 4 : 过滤无效边界
    ReomvePlaneAreaOnEdge( DriveMdl,  BndCont) ;
    //STEP 5 : 边界修剪边界
    TrimAllPlaneAreaByContours( MeshSurf, BndCont, AllLoop, cList ) ;
	//STEP 6 : 平移环
	POSITION pos = cList.GetHeadPosition() ;
	while( pos )
	{
		CPlaneLoop *pPlaneLoop = cList.GetNext( pos ) ;
        MathCAM_ZMoveContours( pPlaneLoop->m_pPlaneLoop , - fZShiftDist ) ;
        MathCAM_ZMoveContours( pPlaneLoop->m_pOrgLoop , - fZShiftDist ) ;
	}
    DriveMdl.ZMovePlaneSurf( -fZShiftDist );
    if( bFindMCopy == TRUE && DriveMdl.GetMultiCopy() == NULL )
    {/*恢复多线程计算*/
        DriveMdl.CreateMultiCopy(m_nCalcThreadNum - 1) ;
    }
    m_cShapeDef.m_cDepth = bakDepth ;
    return TRUE ;
}

int CSmartFPlaneGen::CreatePlaneContour ( CSmtCheckMdl& DriveMdl  ,
                                          CNcZLevelGenMdl* MeshSurf,
                                          CSmartLoop *&BndCont, 
                                          JDNC_PRGDEF& PrgDef ) 
{// 生成平面加工区域
    BndCont = NULL ;
    if( DriveMdl.m_pTool == NULL )
    {
        return FALSE ;
    }
    BOOL bFindMCopy = FALSE ;
    if( DriveMdl.GetMultiCopy() != NULL ) 
    {/*本函数肯能破坏多线程模型*/
        bFindMCopy = TRUE ;
    }
    DOUBLE fZShiftDist = m_cSetupDef.m_cTolDef.m_dArcTol ;
    if( DriveMdl.ZMovePlaneSurf( fZShiftDist ) == FALSE )
    {/*没有发现平面*/
        return TRUE ;
    }
    m_dTopHeight += fZShiftDist ;
    m_dCutDepth  += fZShiftDist ;
    // STEP 2 : 构造加工边界
    if( ! CreateBoundArea( DriveMdl, NULL, BndCont,PrgDef ) )
    {
        Mini_DeleteContours( BndCont ) ;
        m_nErrorType = JDERROR_GENPATH_ABORT  ;
        return FALSE ;
    }
    //STEP 3 : 跟踪精确边界
    TracingPlaneAreaBound( DriveMdl,  BndCont) ;
    if( m_cSetupDef.m_cTolDef.m_nFitType != NCDEF_FITTYPE_LINE )
    {
        CSmtLoopArr allLp ;
	    for(CSmartLoop*	pLoop = BndCont; pLoop; pLoop=pLoop->next)
        {
            pLoop->ExtractAllLoop( 0, allLp ) ;
        }
        DOUBLE dFitTol[2] = { m_cSetupDef.m_cTolDef.m_dArcTol , 
                             cos( ANGLE_TO_RADIAN( m_cSetupDef.m_cTolDef.m_dAngTol) ) };
        INT_PTR nSize = allLp.GetSize() ;
        for( INT_PTR k = 0 ; k < nSize ; k ++ ) 
        {
            pLoop = allLp.GetAt( k ) ;
            MathCAM_FittingByArc( *pLoop->m_pCurve, dFitTol ) ;
            pLoop->DefineBox() ;
        }
    }
    //STEP 4 : 光滑边界
    if( (m_cSetupDef.m_cModelTol.m_nMdlFlag & NCDEF_SETUP_SMOOTHCORNER) &&
        m_cSetupDef.m_cTolDef.m_dMinRad > 0.02 &&
        MeshSurf && BndCont )
    {/*需要光滑*/
        DriveMdl.ZMovePlaneSurf( -fZShiftDist );
        MathCAM_ZMoveContours( BndCont, -fZShiftDist ) ; 
        if( SmoothPlaneAreaBound( MeshSurf,  BndCont) == FALSE )
        {
            Mini_DeleteContours( BndCont ) ;
            m_nErrorType = JDERROR_GENPATH_ABORT  ;
            return FALSE ;
        }
        DriveMdl.ZMovePlaneSurf( fZShiftDist );
        MathCAM_ZMoveContours( BndCont, fZShiftDist ) ; 
    }
    //STEP 5 : 过滤无效边界
    ReomvePlaneAreaOnEdge( DriveMdl,  BndCont) ;
    MathCAM_ZMoveContours( BndCont, -fZShiftDist ) ; 
    DriveMdl.ZMovePlaneSurf( -fZShiftDist );
    if( bFindMCopy == TRUE && DriveMdl.GetMultiCopy() == NULL )
    {/*恢复多线程计算*/
        DriveMdl.CreateMultiCopy(m_nCalcThreadNum - 1) ;
    }
    return TRUE ;
}
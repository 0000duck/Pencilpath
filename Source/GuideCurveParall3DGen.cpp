//GuideCurveParall3DGen.cpp
#include "stdafx.h"
#include "SmtAutoFinishGen.h"
#include "NcBndAutoFinish.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char BASED_CODE THIS_FILE[] = __FILE__;
#endif

extern void MathCAM_RotateByZAxis( CSmtCPathLib& AllPath, DOUBLE Angle );
BOOL MathCAM_CreateGuideCurve( CSmartGraphic& Graph  , 
                               JDNC_SETUP&    Setup  ,
                               CSmtCheckMdl& DriveMdl,
                               CSmtCPathLib& GuideLib,
                               JDNC_PRGDEF&  PrgDef  )
{
    JDNC_PRGDEF tmpPrg = PrgDef ;
    tmpPrg.m_pPosFunc = NULL ; 
    tmpPrg.m_pPrgFunc = NULL ; 
    CPtrList AllList ;
    Graph.GetAllEntity( Setup, AllList, SMARTGRAPH_TYPE_GUIDECURVE );
    if( AllList.GetCount() == 0 ) return FALSE ;
    while( AllList.GetCount() ) 
    {
        CSmartCurve* pCurve = (CSmartCurve*) AllList.RemoveHead() ;
        CSmtCutPath* pNewPath = new CSmtCutPath() ;
        pNewPath->AddCurve( *pCurve, Setup.m_cTolDef ) ;
        delete pCurve ;
        for( CSmtCutPoint* pTPoint = pNewPath->m_pHead; pTPoint; pTPoint = pTPoint->next )
        {
            pTPoint->m_fPoint[2] = DriveMdl.m_fBottom ;
        }
        pNewPath->InsertCPoint(  Setup.m_cTolDef.m_dMaxStep ) ; 
		if( fabs( DriveMdl.m_dZAxisRotate ) > 0 )
			pNewPath->RotatePath ( DriveMdl.m_dZAxisRotate ) ;
        pNewPath->VerifyCutPath( DriveMdl, Setup.m_cTolDef, tmpPrg ) ;
        GuideLib.AddToTail( pNewPath ) ;
    }
    if( GuideLib.m_cAllPath.GetCount() == 0 )
    {
        return FALSE ;
    }
    return TRUE ;
}

static BOOL MathCAM_DepartParallelStepTLib( CSmtCPathLib& GuideTLib, 
                                            CSmtCPathLib& StepTLib)
{
    POSITION atTPos = GuideTLib.m_cAllPath.GetHeadPosition() ;
    while( atTPos )
    {
        CSmtCutPath* pTPath = GuideTLib.m_cAllPath.GetNext( atTPos ) ;
        CSmtCutPoint* pTPnt, *pTNew ;
        for( pTPnt = pTPath->m_pHead ; pTPnt ; pTPnt = pTPnt->next )
        {
            pTPnt->m_bType = SMART_CUTPNT_TRIA ;
            if( pTPnt->next && pTPnt->m_fPoint[1] == pTPnt->next->m_fPoint[1] )
            {
                if( pTPnt->prev == NULL || pTPnt->m_fPoint[1] == pTPnt->prev->m_fPoint[1]  )
                {
                    pTPnt->m_bType = 0 ;
                }
            }
            if( pTPnt->prev && pTPnt->m_fPoint[1] == pTPnt->prev->m_fPoint[1] )
            {
                if( pTPnt->next == NULL || pTPnt->m_fPoint[1] == pTPnt->next->m_fPoint[1]  )
                {
                    pTPnt->m_bType = 0 ;
                }
            }
        }
        for( pTPnt = pTPath->m_pHead ; pTPnt; pTPnt = pTPnt->next )
        {
            if( pTPnt->prev == NULL ) continue ;
            if( pTPnt->m_bType && pTPnt->prev->m_bType && 
                pTPnt->m_fPoint[1] == pTPnt->prev->m_fPoint[1] )
            {
                pTNew = pTPnt->prev->GenMidPoint( pTPnt, 0.5f ) ;
                pTNew->m_bType = 0 ;
                pTPath->InsertAfter( pTNew, pTPnt->prev  ) ;
            }
        }
        for( pTPnt = pTPath->m_pHead ; pTPnt ; pTPnt = pTPnt->next )
        {
            if( pTPnt->prev == NULL || pTPnt->next == NULL ) 
            {
                continue ;
            }
            if( pTPnt->prev->m_bType == 0 || pTPnt->m_bType == 0 || pTPnt->next->m_bType == 0 )
            {
                continue ;
            }
            if( pTPnt->prev->m_fPoint[1] > pTPnt->m_fPoint[1] && 
                pTPnt->next->m_fPoint[1] > pTPnt->m_fPoint[1] || 
                pTPnt->prev->m_fPoint[1] < pTPnt->m_fPoint[1] && 
                pTPnt->next->m_fPoint[1] < pTPnt->m_fPoint[1]  )
            {
                pTNew = pTPnt->CopyMyself() ;
                pTPath->InsertAfter( pTNew, pTPnt->prev ) ;
                pTNew = pTPnt->CopyMyself() ;
                pTNew->m_bType = 0 ;
                pTPath->InsertBefore( pTNew, pTPnt ) ;
            }
        }
        CSmtCutPath* pNewPath = pTPath->BreakAtNullPoint( FALSE ) ;
        while( pNewPath )
        {
            CSmtCutPath* pTNext = pNewPath->next ;
            pNewPath->next = pNewPath->prev = NULL ; 
            pNewPath->DefineBox() ;
            if( pNewPath->m_pHead->m_fPoint[1] > pNewPath->m_pTail->m_fPoint[1] )
            {
                pNewPath->ReverseDirect() ;
            }
            StepTLib.AddToTail( pNewPath ) ;
            pNewPath = pTNext ;
        }
    }
    return TRUE ;
}


BOOL GetMachAreaAndAllocICS3D(CSmtCheckMdl& DriveMdl, CSmartLoop* AreaCont   , 
							  CSmtCPathLib& StepTLib, double OverStep,
							  JDNC_SETUP& m_cSetup,double * &aY,int &nY)
{
	FPNT3D MinPt, MaxPt;
	DriveMdl.CalcFacetBox(MinPt, MaxPt);
	double dMachArea[2] = { MinPt[1], MaxPt[1] };
	if(AreaCont)
	{
		dMachArea[0]=MAX_DBL, dMachArea[1]=-MAX_DBL;
		for ( CSmartLoop* pLoop = AreaCont ; pLoop ; pLoop = pLoop->next )
		{
			if( pLoop->m_dBox[0][1]<dMachArea[0] ) dMachArea[0]=pLoop->m_dBox[0][1];
			if( pLoop->m_dBox[1][1]>dMachArea[1] ) dMachArea[1]=pLoop->m_dBox[1][1];
		}
	}
	dMachArea[0]+=3*m_cSetup.m_cTolDef.m_dArcTol;
	dMachArea[1]-=3*m_cSetup.m_cTolDef.m_dArcTol;


    // STEP 1 : 准备加工数据, 计算第一层
    CSmtCutPath tmpstep ;
    TFLOAT fYAt[3] = { 0.0f, (TFLOAT) dMachArea[0], 0.0f } ;
    tmpstep.AddPoint( fYAt ) ;
    // STEP 2 : 修建计算后续的层
    TFLOAT fMaxStep = (TFLOAT) OverStep , fYNext ;
    CSmtCutPoint* pTPnt , *pTNext ;
    while( fYAt[1] < dMachArea[1] )
    {
        fYNext = fYAt[1] + fMaxStep ;
        POSITION atTPos = StepTLib.m_cAllPath.GetHeadPosition() ;
        while( atTPos )
        {
            FPNT3D fIntAt ;
            CSmtCutPath* pTPath = StepTLib.m_cAllPath.GetNext(atTPos ) ;
            if( pTPath->m_pHead->m_fPoint[1] >= fYNext || 
                pTPath->m_pTail->m_fPoint[1] <= fYAt[1] )
            {/*低于下一层高度 || 高于当前高度*/
                continue ;
            }
            if( pTPath->m_nNumPnt < 2 ) continue ;
            pTPnt = pTPath->m_pHead ;
            DOUBLE t = 1.0 , fLenAt = 0.0 , fLen;
            if( pTPath->m_pHead->m_fPoint[1] > fYAt[1] )
            {
                fLenAt = pTPath->m_pHead->m_fPoint[1] - fYAt[1];
            }
            else
            {
                while( pTPnt && pTPnt->next )
                {
                    if( pTPnt->m_fPoint[1] <= fYAt[1] && 
                        pTPnt->next->m_fPoint[1] >= fYAt[1] )
                    {
                        break ;
                    }
                    pTPnt = pTPnt->next ;
                }
                pTNext = pTPnt->next ;
                if( pTNext == NULL ) continue ;
                t = 1.0f ;
                if( fabs( pTPnt->m_fPoint[1] - pTNext->m_fPoint[1]) > 2.0e-4 )
                {
                    t = ( fYAt[1] - pTPnt->m_fPoint[1] ) / ( pTNext->m_fPoint[1] - pTPnt->m_fPoint[1] ) ;
                }
                fLen = nc_Distance( pTPnt->m_fPoint, pTNext->m_fPoint, 3 ) ;
                fLenAt = - t * fLen ;
            }
            for( ; pTPnt && pTPnt->next ; pTPnt = pTPnt->next )
            {
                pTNext = pTPnt->next ;
                fLen = nc_Distance( pTPnt->m_fPoint, pTNext->m_fPoint, 3 ) ;
                if( fLen + fLenAt >= fMaxStep )
                {
                    t = ( fMaxStep - fLenAt) / fLen ;
                    nc_GetPointAtLine( pTPnt->m_fPoint, pTNext->m_fPoint, (TFLOAT)t, fIntAt , 3 ) ;
                    if( fIntAt[1] < fYNext && fIntAt[1] > fYAt[1] ) 
                    {
                        fYNext = fIntAt[1] ;
                    }
                    break ;
                }
                fLenAt += fLen ;
            }
        }
        TFLOAT fYInc  =  fYNext - fYAt[1] ;
        if( fYInc < 0.003f ) fYInc = 0.003f ;
        if( fYInc > fMaxStep ) fYInc = fMaxStep ;
        fYAt[1] += fYInc ;
        if( fYAt[1] > dMachArea[1] ) break ;
        tmpstep.AddPoint( fYAt ) ;
    }
    // STEP 3 : 计算分层参数
    int nCnt = 0 ; 
    for( pTPnt = tmpstep.m_pHead ; pTPnt ; pTPnt = pTPnt->next )
    {
        nCnt ++ ;
    }
	nY = nCnt;
    aY = new double[ nCnt] ;
    pTPnt = tmpstep.m_pHead ;
    for( int i = 0 ; i < nCnt ; i ++ ) 
	{
        aY[i] = pTPnt->m_fPoint[1] ;
        pTPnt = pTPnt->next ;
    }
	return TRUE;
}
void CSmartAutoFinishGen::SingleThreadCPPEx(CSmtCheckMdl& DriveMdl, CSmartLoop *AreaCont, 
										    CSmtCPathLib &AllPath,
										    double * aY,int nY, JDNC_PRGDEF &PrgDef)
{
	CSmtCPathLib tmpTLib;
	CSmartLoop* pLoop;
	int nGroupNo = 0 , nLineNo = 0 ;
	int k;
	PNT2D dBox2D[2] ;
	DOUBLE y = 0.0 ;
	//DOUBLE yStep = m_cFeedDef.m_cStepDef.m_dOverStep  ;
	PNT4D dStart, dEnd  ;
	dStart[2] = dEnd[2] = 0.0 ;
	dStart[3] = dEnd[3] = 0.0 ;
	PNT2D dPtArr[1000];
	for( pLoop = AreaCont ; pLoop ; pLoop = pLoop->next, nGroupNo ++  )
	{
		nc_VectorCopy ( dBox2D[0], pLoop->m_dBox[0], 2 ) ;
		nc_VectorCopy ( dBox2D[1], pLoop->m_dBox[1], 2 ) ;
		CSmtCPathLib curTLib ;
		//for( y = dBox2D[0][1], nLineNo = 0 ; y < dBox2D[1][1]; y += yStep, nLineNo ++ ) 
		for(k=0, nLineNo = 0; k<nY; k++,nLineNo ++)
		{
			y = aY[k];
			if( y < pLoop->m_dBox[0][1] ) continue ;
			if( y > pLoop->m_dBox[1][1] ) break    ;
			//////////////////////////////
			UINT nCount = pLoop->GetYLineIntContour( y , dPtArr, 1000 ) ;
			if( nCount == 0 || nCount % 2 ) continue ;
			for( UINT i = 0 ; i < nCount ; i += 2 )
			{
				dStart[0] = dPtArr[i  ][0], dStart[1] = dPtArr[i ][1] ;
				dEnd[0]   = dPtArr[i+1][0], dEnd[1]   = dPtArr[i+1][1] ;
				if( dEnd[0] <= dBox2D[0][0] || dStart[0] >= dBox2D[1][0] )
				{
					continue ;
				}
				if( dStart[0]  < dBox2D[0][0] ) dStart[0] = dBox2D[0][0] ;
				if( dEnd[0]    > dBox2D[1][0] ) dEnd[0] = dBox2D[1][0]  ;
				CSmtCutPath* pNewPath = new CSmtCutPath() ;
				pNewPath->AddPoint( dStart ) ;
				pNewPath->AddPoint( dEnd   ) ;
				pNewPath->m_nLineNo  = nLineNo ;
				pNewPath->m_nLayerNo = nGroupNo ;
				curTLib.AddToTail( pNewPath ) ;
			}
		}
		tmpTLib.AppendCPathLib ( curTLib ) ;
	}


	JDNC_PRGDEF tmpPrg = PrgDef ;
	tmpPrg.m_pPosFunc = NULL ; 
	tmpPrg.m_pPrgFunc = NULL ; 
	INT_PTR nNumPath = tmpTLib.m_cAllPath.GetCount();
	double dTotalMove = PrgDef.m_dTotalMove ;
	if( m_cFeedDef.m_cConnectDef.m_bConnect3DFlag & NCDEF_FCONNECT3D_ZIGZAG  )
	{
		PrgDef.m_dTotalMove = dTotalMove * 0.7 ;
	}
	else
	{
		if( m_c5DCtrlDef.m_cToolAxis.m_nAxisType )
			PrgDef.m_dTotalMove = dTotalMove * 0.7 ;
		else
			PrgDef.m_dTotalMove = dTotalMove * 1.0;
	}
	PrgDef.m_dIncStep = PrgDef.m_dStepAt = 0.0 ;
	PrgDef.m_dLimitAt = 1.0 ;
	if( nNumPath )
	{
		PrgDef.m_dIncStep = PrgDef.m_dTotalMove / nNumPath ;
	}

	/*CPPT_DATA ThreadData(&tmpTLib, &AllPath, &DriveMdl, &m_cSetupDef, &PrgDef, &tmpPrg,
		(TFLOAT)GetBottomHeight(), (TFLOAT)GetTopHeight());
	MathCAM_CreateParallelPathSubProc(&ThreadData);*/
    double  dLineTol = max( 2.0e-4, (0.1 * m_cSetupDef.m_cTolDef.m_dArcTol) ) ;
    if( m_cSetupDef.m_cModelTol.m_nMdlFlag & NCDEF_SETUP_NOMOSAIC )
    {
        dLineTol = 2.0e-5;
    }
	double fBtmHeight = GetBottomHeight();
	double fTopHeight = GetTopHeight();
	while( tmpTLib.m_cAllPath.GetCount() > 0)
	{
		CSmtCutPath* pBasePath = tmpTLib.m_cAllPath.RemoveHead() ;
		pBasePath->InsertCPoint( m_cSetupDef.m_cTolDef.m_dMaxStep ) ;	// 插点（非随机）
		pBasePath->VerifyLinePath(DriveMdl , m_cSetupDef.m_cTolDef, tmpPrg) ;
		pBasePath->DefineEdgePoint( DriveMdl, TRUE );
		pBasePath->LabelNullPointByZValue(TFLOAT(fBtmHeight-0.002), TFLOAT(fTopHeight));
        CSmtCutPath*	pNewPath=pBasePath->BreakAtNullPoint(FALSE,m_cFeedDef.m_cConnectDef.m_dDelShort);
		delete pBasePath;
		while(pNewPath)
		{
			pBasePath = pNewPath;
			pNewPath  = pNewPath->next;
			pBasePath->next=pBasePath->prev=NULL;
			pBasePath->DelPointOnLine( dLineTol ) ;
			AllPath.AddToTail(pBasePath);
		}
		if( PrgDef.m_pBrkFunc && PrgDef.m_pBrkFunc() )
		{
			break ;
		}
		PrgDef.m_dStepAt += PrgDef.m_dIncStep ;
		while( PrgDef.m_pPrgFunc && PrgDef.m_dStepAt >= PrgDef.m_dLimitAt )
		{
			PrgDef.m_pPrgFunc(1) ;
			PrgDef.m_dStepAt -= PrgDef.m_dLimitAt  ;
		}
	}
}
int CSmartAutoFinishGen::CreateGuideParallel3DPath( CSmtCheckMdl& DriveMdl , 
                                                DOUBLE  LineAngle      ,
                                                CSmartLoop* AllCont    , 
                                                CSmtCPathLib& FlatPath  ,
                                                JDNC_PRGDEF&  PrgDef   )
{
	if(!m_pSmtGraph)
		return FALSE;

	CSmtCPathLib GuideLib ;
	//提取导动线
	if(!MathCAM_CreateGuideCurve(*m_pSmtGraph,m_cSetupDef,DriveMdl,GuideLib,PrgDef))
	{
		return CreateParallelPath(DriveMdl,LineAngle, AllCont,FlatPath,PrgDef);
	}

	double dTotalMove = PrgDef.m_dTotalMove ;
	BOOL bRot = FALSE ;
	double	dAngle = ANGLE_TO_RADIAN(LineAngle);
	if( fabs(dAngle)>1.0e-4 )
    {
		bRot = TRUE ;
	    for(CSmartLoop* pLoop = AllCont ; pLoop ; pLoop = pLoop->next )
	    {
            pLoop->RotateContour( - dAngle ) ;
        }
        DriveMdl.RotateByZAxis( - dAngle, TRUE ) ;
        POSITION atTPos = GuideLib.m_cAllPath.GetHeadPosition() ;
        while( atTPos ) 
        {
            CSmtCutPath* pTPath = GuideLib.m_cAllPath.GetNext( atTPos ) ;
            pTPath->RotatePath( -dAngle ) ;
        }
    }
	
	CSmtCPathLib AllPath,stepTLib ;
	MathCAM_DepartParallelStepTLib( GuideLib, stepTLib ) ;

	double OverStep = m_cFeedDef.m_cStepDef.m_dOverStep;
	double * aY = NULL ;
	int nY = 0;
	if(!GetMachAreaAndAllocICS3D(DriveMdl,AllCont, stepTLib, OverStep,m_cSetupDef, aY, nY) ||
		NULL==aY || nY<1)
		return FALSE;

	SingleThreadCPPEx(DriveMdl,AllCont,AllPath, aY, nY,PrgDef);
	if(aY)
	{
		delete [] aY;
		aY = NULL;
	}

	if( PrgDef.m_pBrkFunc && PrgDef.m_pBrkFunc() )
	{
        return FALSE ;
    }

    BOOL bRet = FALSE ;
	
    SmartNC_RegCreateConnect3DPath( MathCAM_GenZDirLeadPathEx );
    PrgDef.m_dTotalMove = dTotalMove * 0.3 ;
    if( m_cFeedDef.m_cConnectDef.m_bConnect3DFlag & NCDEF_FCONNECT3D_ZIGZAG )
    { /*路径连接*/
        DOUBLE dMaxDist = m_cFeedDef.m_cConnectDef.m_dMaxDist  ;
        if( m_cFeedDef.m_cConnectDef.m_bConnect3DFlag & NCDEF_FCONNECT3D_BYPASS )
        {
			bRet = NcBnd_ConnectPath_ByPass( AllPath, DriveMdl, 
											PrgDef, m_cSetupDef.m_cTolDef, 
											m_cFeedDef.m_cConnectDef, dMaxDist ) ;
        }
		else
        {
			bRet = AllPath.ConnectPathByLineNo( DriveMdl, dMaxDist,m_cSetupDef.m_cTolDef, PrgDef, TRUE, NULL,&(m_cFeedDef.m_cConnectDef)) ;
        }
    }
    else 
    {
        if( GetMillDir() == 1)
        { /*从外向内*/
                AllPath.ReverseDirect() ;
        }
        AllPath.CreateLeadPath( DriveMdl, m_cSetupDef.m_cTolDef, m_cFeedDef.m_cConnectDef) ;
	    bRet = TRUE ;
    }
    SmartNC_RegCreateConnect3DPath( NULL );

	if(fabs( dAngle ) > 1.0e-4 && bRet )
    { 
        MathCAM_RotateByZAxis( AllPath, dAngle ) ;
        for(CSmartLoop* pLoop = AllCont ; pLoop ; pLoop = pLoop->next  )
        {
			pLoop->RotateContour ( dAngle ) ;
		}
        DriveMdl.RotateByZAxis( dAngle , TRUE ) ;
    }
    if( bRet )
    {
        FlatPath.AppendCPathLib( AllPath ) ;
    }
	return TRUE;
}
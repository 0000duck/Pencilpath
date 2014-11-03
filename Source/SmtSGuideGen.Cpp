#include "StdAfx.H"
#include "SurfGeo.H"
#include "SurfNC.H"
#include "SmartNC.H"
#include "SmtPathGen.H"
#include "SmtPathGen3D.H"
#include "mathcam.h"
#include "Nc3DStepAndSpiral.h"
//////////////////////
// 清根加工
CSmartSGuideGen::CSmartSGuideGen()          
{
}
CSmartSGuideGen::~CSmartSGuideGen() 
{
}
// 生成曲线吸附投影路径
int CSmartSGuideGen::CreateAdsorbPath( CSmtCPathLib& AllCurve3D, 
                                        CSmtCheckMdl& DriveMdl, 
                                        CSmtCPathLib& AllPath   ,
                                        CSmartLoop*   AllCont   )  
{
	SurfNC_SetNewStepEx(IDS_PATHGEN_STEP_ORGPATH ) ;
	JDNC_TOL  tmpTol = GetSurfaceTol() ;
	JDNC_PRGDEF tmpPrg = m_cPrgDef ;
	tmpPrg.m_pPosFunc = NULL ; 
	tmpPrg.m_pPrgFunc = NULL ; 
	CSmartTool* pOrgTool = DriveMdl.m_pTool ;
	if( ! pOrgTool ) return FALSE ;
	DOUBLE fDist;
	FPNT3D fPoint, fSurfPos, fSurfNor, fOffset ;
	int nLineAt = 0, nLineTotal = AllCurve3D.GetNumPath() ;
	CSmtCutPath* pTPath ;
    CSmartTool* pOffTool = NULL ; 
    CSmtCPathLib tmpCLib ;
    JDNC_LAYER tmpLayer = m_cFeedDef.m_cLayerDef ;
    if( tmpLayer.m_nLayerCount < 1 || 
        tmpLayer.m_nLayerType != NCDEF_LAYER_COUNT )
    {
        tmpLayer.m_nLayerCount = 1   ;
        tmpLayer.m_dSideDInc   = 0.0 ;
    }
    JDNC_CONNECT3D  cLead3D = m_cFeedDef.m_cConnectDef ;
    cLead3D.m_bConnect3DFlag |=  NCDEF_FCONNECT3D_EXTMODE ;
    SmartNC_RegCreateConnect3DPath( MathCAM_GenNormalLeadPathEx );
    nLineTotal *= tmpLayer.m_nLayerCount ;
    DOUBLE fZMove ;
    for( int nLayer = 1 ; nLayer <= tmpLayer.m_nLayerCount ; nLayer ++  )
    {
        fZMove = ( tmpLayer.m_nLayerCount - nLayer) * tmpLayer.m_dSideDInc - m_cParam.m_dSlotDepth  ;
        pOffTool = NULL ; 
        if( fabs(fZMove) > 0.005 ) pOffTool = pOrgTool->Offset( fZMove ) ;
        if( pOffTool )
        {
            DriveMdl.UpdateCurrTool( pOffTool ) ;
        }
        else
        {
           DriveMdl.UpdateCurrTool( pOrgTool ) ;
        }
        POSITION atPos = AllCurve3D.m_cAllPath.GetHeadPosition() ;
		while( atPos ) 
		{
			pTPath = AllCurve3D.m_cAllPath.GetNext( atPos ) ;
			pTPath = pTPath->CopyMyself() ;
			if( m_cPrgDef.m_pPosFunc )
			{
				m_cPrgDef.m_pPosFunc( nLineAt, nLineTotal ) ;
				nLineAt ++ ;
			}
			if( m_cPrgDef.m_pBrkFunc && m_cPrgDef.m_pBrkFunc() ) 
			{
				break ;
			}
			for( CSmtCutPoint* pTPoint = pTPath->m_pHead ;pTPoint ; pTPoint = pTPoint->next )
			{
				pTPoint->m_fPoint[2] += float( DriveMdl.m_dMoveDist[2]) ;
				nc_VectorCopy( fSurfPos, pTPoint->m_fPoint, 3 ) ;
				fSurfNor[0] = fSurfNor[1] = 0.0f ;
				fSurfNor[2] = 1.0f ;
				fDist = DriveMdl.AdsorbToSurface( pTPoint->m_fPoint, fSurfPos, fSurfNor, 1000.0f ) ;
				if( fDist < 1000.0 )
				{
					DriveMdl.m_pTool->GetFSafeVectorEx( fSurfNor, fOffset ) ;
					for( int k = 0 ; k < 3; k ++ ) fPoint[k] = fSurfPos[k] + fOffset[k] ;
					nc_VectorCopy( pTPoint->m_fPoint, fPoint , 3 ) ;
				}
				else
				{
					ASSERT( 0 ) ;
				}
				pTPoint->m_fPoint[3] = fPoint[2] ;
			}
			pTPath->VerifyCutPath( DriveMdl, tmpTol, tmpPrg ) ;
			pTPath->DefineEdgePoint( DriveMdl, TRUE ) ;
			for( CSmtCutPoint* pTPoint = pTPath->m_pHead ;pTPoint ; pTPoint = pTPoint->next )
			{
				if( pTPoint->m_fPoint[2] > pTPoint->m_fPoint[3] + 0.05f )
				{
					pTPoint->m_bType = 0 ; 
				}
			}

			CSmtCutPath* pNewTPath = pTPath->BreakAtNullPoint( FALSE ) ;
			delete pTPath ;
			while( pNewTPath ) 
			{
				pTPath = pNewTPath->next ;
				pNewTPath->prev = pNewTPath->prev = NULL ; 
				pNewTPath->DelPointOnLine( 2.0e-4 ) ;
				pNewTPath->TransPath( 0.0f, 0.0f, (float)fZMove ) ;
				tmpCLib.AddToTail( pNewTPath ) ;
				pNewTPath = pTPath ;
			}/**/
		}
		if( AllCont )
		{
			CSmtCPathLib tmpcb ;
			tmpCLib.TrimCutPathByContours( tmpcb, AllCont ) ;
			tmpCLib.ClearAllPath() ;
			while( tmpcb.m_cAllPath.GetCount() )
			{
				pTPath = tmpcb.m_cAllPath.RemoveHead() ;
				tmpCLib.AddToTail( pTPath) ;
			}
		}
		tmpCLib.CreateLeadPath( DriveMdl, tmpTol, cLead3D ) ;
		if( pOffTool ) 
		{
			DriveMdl.m_pTool = NULL  ;
			delete pOffTool ;
		}
		tmpCLib.DefineBox() ;
		PNT3D dLastPnt ;
		nc_VectorCopy( dLastPnt, tmpCLib.m_fBox[0], 3 ) ;
		if( AllPath.m_cAllPath.GetCount() ) 
		{
			pTPath = AllPath.m_cAllPath.GetTail() ;
			nc_FloatToDouble( dLastPnt, pTPath->m_pTail->m_fPoint, 3 ) ;
		}
		tmpCLib.ResortAllPath( dLastPnt, TRUE ) ;
		while( tmpCLib.m_cAllPath.GetCount() ) 
		{
			pTPath = tmpCLib.m_cAllPath.RemoveHead() ;
			AllPath.AddToTail( pTPath ) ;
		} /**/
    }
	if( m_cPrgDef.m_pBrkFunc && m_cPrgDef.m_pBrkFunc() ) 
	{
	    m_nErrorType = JDERROR_GENPATH_ABORT  ;
		return FALSE  ;
	}
    if( DriveMdl.m_pTool == NULL ) 
    {
        DriveMdl.UpdateCurrTool( pOrgTool ) ;
    }
	return TRUE ;
}

int CSmartSGuideGen::CreateZProjectPath( CSmartCurveLib& AllCurve, 
                                         CSmtCheckMdl& DriveMdl  , 
						                 CSmtCPathLib& AllPath   ,
                                         CSmartLoop*   AllCont   ) 
{
    // STEP 1 : 计算投影路径
    SurfNC_SetNewStepEx(IDS_PATHGEN_STEP_ORGPATH ) ;
 	JDNC_TOL  tmpTol = GetSurfaceTol() ;
    JDNC_PRGDEF tmpPrg = m_cPrgDef ;
    tmpPrg.m_pPosFunc = NULL ; 
    tmpPrg.m_pPrgFunc = NULL ; 
    CSmtCutPath tmpPath, *pTPath ;
    JDNC_LAYER  tmpLayer = m_cFeedDef.m_cLayerDef  ;
    if( m_cParam.m_dSlotDepth > 0.0002 )
    {
        if( tmpLayer.m_nLayerType != NCDEF_LAYER_COUNT ||
            tmpLayer.m_nLayerCount < 1  )
        {
            tmpLayer.m_nLayerType = NCDEF_LAYER_COUNT ;
            tmpLayer.m_nLayerCount = 1 ;
            tmpLayer.m_dSideDInc = 0.0 ;
        }
    }
    int nLineAt = 0 , nLineTotal = AllCurve.m_nNumCurve ;
    if( tmpLayer.m_bLayerFlag & NCDEF_LAYER_NORMAL )
    {/*偏移分层*/
        nLineTotal += tmpLayer.m_nLayerCount * AllCurve.m_nNumCurve ;
    }
    for( CSmartCurve* pCurve = AllCurve.m_pHead ; pCurve ;pCurve = pCurve->next) 
    {
        tmpPath.ClearAllPoint() ;
        tmpPath.AddCurve( *pCurve , tmpTol ) ;
        tmpPath.InsertCPoint( tmpTol.m_dMaxStep ) ;
        tmpPath.VerifyCutPath( DriveMdl, tmpTol, tmpPrg ) ;
        tmpPath.DefineEdgePoint( DriveMdl, TRUE ) ;
        CSmtCutPath* pNewTPath = tmpPath.BreakAtNullPoint( FALSE ) ;
        while( pNewTPath )
        {
            pTPath = pNewTPath->next ;
            pNewTPath->prev = pNewTPath->next = NULL ; 
            pNewTPath->DelPointOnLine( 2.0e-4 ) ;
            pNewTPath->m_nLineNo = nLineAt ;
            AllPath.AddToTail( pNewTPath ) ;
            pNewTPath = pTPath ;
        }
        if( m_cPrgDef.m_pBrkFunc && m_cPrgDef.m_pBrkFunc() )
        {
	        m_nErrorType = JDERROR_GENPATH_ABORT  ;
            return FALSE ;
        }
        if( m_cPrgDef.m_pPosFunc )
        {
            m_cPrgDef.m_pPosFunc( nLineAt, nLineTotal ) ;
        }
        nLineAt ++ ;
    }
    if( AllCont )
    {
        CSmtCPathLib tmpCLib ;
        AllPath.TrimCutPathByContours( tmpCLib, AllCont ) ;
        AllPath.ClearAllPath() ;
        while( tmpCLib.m_cAllPath.GetCount() )
        {
           pTPath = tmpCLib.m_cAllPath.RemoveHead() ;
           AllPath.AddToTail( pTPath) ;
        }
    }
    JDNC_CONNECT3D  cLead3D = m_cFeedDef.m_cConnectDef ;
    cLead3D.m_bConnect3DFlag |=  NCDEF_FCONNECT3D_EXTMODE ;
    SmartNC_RegCreateConnect3DPath( MathCAM_GenNormalLeadPathEx );
    // STEP　2 : 路径分层
    AllPath.DefineBox() ;
    AllPath.ResortAllPath( AllPath.m_fBox[0] , TRUE ) ;
    if( tmpLayer.m_nLayerType != NCDEF_LAYER_COUNT ||
        tmpLayer.m_nLayerCount < 1   )
    {
        AllPath.CreateLeadPath( DriveMdl, tmpTol, cLead3D ) ;
        return TRUE ;
    }
    CSmtCPathLib tmpLib ;
    while( AllPath.m_cAllPath.GetCount() )
    {
        pTPath = AllPath.m_cAllPath.RemoveHead() ;
        tmpLib.AddToTail( pTPath ) ;
    }
    // STEP 2 : 计算分层路径
    CSmtCutPath* pNewTPath ;
    CSmtCPathList  *allLayer = new CSmtCPathList[ tmpLayer.m_nLayerCount ] ;
    TFLOAT  fZMove = 0.0;
    CSmartTool * pTool = NULL , *pSafeTool = NULL ;
    pTool = DriveMdl.m_pTool ;
    if( DriveMdl.m_pCheckMdl )  pSafeTool = DriveMdl.m_pCheckMdl->m_pTool ;
    if( tmpLayer.m_bLayerFlag & NCDEF_LAYER_NORMAL )
    {/*拷贝分层*/
        AllPath.CreateLeadPath( DriveMdl, tmpTol, cLead3D ) ;
    }
    POSITION atPos = tmpLib.m_cAllPath.GetHeadPosition() ;
    while( atPos ) 
    {
        pTPath = tmpLib.m_cAllPath.GetNext( atPos ) ;
        for( int nLayer = 1 ; nLayer <= tmpLayer.m_nLayerCount; nLayer ++ ) 
        {
            pNewTPath = pTPath->CopyMyself() ;
            fZMove =  TFLOAT( (tmpLayer.m_nLayerCount - nLayer) * tmpLayer.m_dSideDInc - m_cParam.m_dSlotDepth)  ;
            if( !( tmpLayer.m_bLayerFlag & NCDEF_LAYER_NORMAL ) )
            {/*拷贝分层*/
                pNewTPath->TransPath( 0.0f, 0.0f, fZMove ) ;
            }
            else
            {/*法向偏移*/
                pNewTPath->ZOffsetPath( fZMove, tmpTol, tmpPrg ) ;
                if( m_cPrgDef.m_pPosFunc )
                {
                    m_cPrgDef.m_pPosFunc( nLineAt, nLineTotal ) ;
                }
                nLineAt ++ ;
            }
            allLayer[nLayer-1].AddTail( pNewTPath ) ;
        }
    }
    // STEP 3 : 添加路径
    while( allLayer[0].GetSize() > 0 )
    {
        CSmtCutPath* pLastPath = NULL ; 
        for( int nLayer = 1 ; nLayer <= tmpLayer.m_nLayerCount; nLayer ++ ) 
        {
            pTPath = allLayer[nLayer-1].RemoveHead() ;
            if( tmpLayer.m_bLayerFlag & NCDEF_LAYER_KEEPDOWN && pLastPath )
            {/*分层不抬刀*/
                if( (nLayer % 2 ) == 0 &&( pTPath->IsClosed() == FALSE))
                {
                    pTPath->ReverseDirect() ;
                }
                CSmtCutPath* pPlunge = new CSmtCutPath() ;
                pPlunge->m_bFeedType = JDNC_FEEDTYPE_PLUNGE ;
                pPlunge->AddPoint( pLastPath->m_pTail->m_fPoint ) ;
                pPlunge->AddPoint( pTPath->m_pHead->m_fPoint ) ;
                AllPath.AddToTail( pPlunge ) ;
            }
            AllPath.AddToTail( pTPath ) ;
            pLastPath = pTPath ;
        }
    }
    if( allLayer ) delete[] allLayer ;
    return TRUE ;
}
int SmartNC_GetPointOnCurve( CSmartCurve& Curve, int NumPnt , CSmtCutPath& tmpPath)
{
    if( NumPnt < 2 ) NumPnt = 2 ;
    tmpPath.ClearAllPoint() ;
    PNT3D dPoint ;
    dPoint[2] = 0.0 ;
    double dAtDist = 0.0, u  , dLen , dDist; 
    dLen = Curve.GetLength() ;
    dDist = dLen / (NumPnt-1) ;
    Curve.GetPoint( 0.0, dPoint ) ;
    tmpPath.AddPoint( dPoint   ) ;
    int nPoint = 1 ;
    for( CSmartSect* pSect = Curve.GetHead() ; pSect ; pSect = pSect->next )
    {
        dLen = pSect->GetLength() ;
        if( dLen<1.0e-7 ) continue; 
        dAtDist += dLen ;
        while( dAtDist >= dDist )
        {
           u = 1.0 - ( dAtDist - dDist ) / dLen ;
           pSect->GetPoint( u , dPoint ) ;
           tmpPath.AddPoint( dPoint ) ;
           nPoint ++ ;
           if( nPoint == NumPnt -1 ) break ;
           dAtDist -= dDist ;  
        }
    }
    Curve.GetPoint( 1.0, dPoint) ;
    tmpPath.AddPoint( dPoint ) ;
    return 1 ;
}
// 生成单轨扫描路径
int CSmartSGuideGen::CreateOneSweepPath( CSmartCurveLib& AllCurve, 
                                         CSmtCheckMdl& DriveMdl  ,
						                 CSmtCPathLib& AllPath   ,
                                         CSmartLoop*   AllCont   ) 
{
    // STEP 1 : 获得原始曲线
    SurfNC_SetNewStepEx(IDS_PATHGEN_STEP_ORGPATH ) ;
    CSmtCutPath  railPath, *pTPath ;
    JDNC_TOL tmpTol = GetSurfaceTol() ;
    JDNC_PRGDEF  tmpPrg = m_cPrgDef;
    tmpPrg.m_pPosFunc = NULL  ;
    tmpPrg.m_pPrgFunc = NULL  ;
	CSmtCPathLib tmpCLib ;
	int nLayer = 1 ;
    DOUBLE fOverStep = m_cFeedDef.m_cStepDef.m_dOverStep ;
	DOUBLE fNetStep =  fOverStep ;
    BOOL bByRail = FALSE ;
	if( m_cParam.m_bSGuideFlag & NCDEF_FSGUIDE_BYRAIL )
	{
		fNetStep = min( tmpTol.m_dMaxStep, fOverStep * 3.0 ) ;
		if( fNetStep < 0.1 ) fNetStep = 0.1 ;
        bByRail = TRUE ;
    }
    int nTotalCv = 0 ;
    for( CSmartCurve* pCurve = AllCurve.m_pHead ; pCurve ;pCurve = pCurve->next ) 
    {
        nTotalCv ++ ;
    }
    if( nTotalCv == 0 ) nTotalCv = 1 ;
    for( pCurve = AllCurve.m_pHead ; pCurve ;pCurve = pCurve->next, nLayer ++ ) 
    {
	    railPath.ClearAllPoint() ;
	    railPath.AddCurve( *pCurve , tmpTol ) ;
        if( bByRail )  m_cPrgDef.m_dTotalMove = 35.0 / nTotalCv ;
        else m_cPrgDef.m_dTotalMove = 70.0 / nTotalCv ;
        tmpCLib.CreateOneRailSection( DriveMdl, tmpTol , fNetStep ,m_cParam.m_dExtDist, 
                                      m_cParam.m_nOffsetMode, railPath, m_cPrgDef , bByRail) ;
        if( bByRail )
        {
		    CSmtCPathLib atLib; 
            m_cPrgDef.m_dTotalMove = 35.0 / nTotalCv ;
		    tmpCLib.ExtractAlongRailPath( DriveMdl, tmpTol,fOverStep, atLib, m_cPrgDef ) ;
            tmpCLib.ClearAllPath() ;
            while( atLib.m_cAllPath.GetCount() )
            {
                pTPath = atLib.m_cAllPath.RemoveHead() ;
                tmpCLib.AddToTail( pTPath ) ;
            }
        }
		while( tmpCLib.m_cAllPath.GetCount() ) 
		{
			pTPath = tmpCLib.m_cAllPath.RemoveHead() ;
			pTPath->m_nLayerNo = nLayer ;
            pTPath->DelPointOnLine( 2.0e-4f ) ;
			AllPath.AddToTail( pTPath ) ;
		}
	}
    if( AllCont )
    {
        tmpCLib.ClearAllPath() ;
        AllPath.TrimCutPathByContours( tmpCLib, AllCont ) ;
        AllPath.ClearAllPath() ;
        while( tmpCLib.m_cAllPath.GetCount() )
        {
           pTPath = tmpCLib.m_cAllPath.RemoveHead() ;
           AllPath.AddToTail( pTPath) ;
        }
    }
    // STEP　2 : 路径连接
    m_cPrgDef.m_dTotalMove = 30.0 ;
    BOOL bZigzag = ( m_cParam.m_bSGuideFlag & NCDEF_FSGUIDE_ZIGZAG ) ? TRUE : FALSE ;
    JDNC_CONNECT3D  tmpCnt3D = m_cFeedDef.m_cConnectDef ;
    if( bZigzag ) 
    {
        tmpCnt3D.m_bConnect3DFlag |= NCDEF_FCONNECT3D_ZIGZAG ;
    }
    else
    {
        tmpCnt3D.m_bConnect3DFlag &= ~NCDEF_FCONNECT3D_ZIGZAG ;
    }
    tmpCnt3D.m_dSafeDist = m_cFeedDef.m_cStepDef.m_dOverStep ;
    if( m_cSetupDef.m_cOrderDef.m_nLayerOrd == 1 && 
        tmpCnt3D.m_dSafeDist < m_pTool->m_fRadius * 2.0 )
    { //高度优先
        tmpCnt3D.m_dSafeDist = m_pTool->m_fRadius * 2.0 ;
    }
    if ( m_cSetupDef.m_cModelTol.m_nMdlFlag & NCDEF_SETUP_SMOOTHCORNER )
	{/*光滑尖角*/
		AllPath.SmoothAllPath ( DriveMdl, m_cSetupDef, m_cPrgDef ) ;
	}
	if (m_cFeedDef.m_cConnectDef.m_bConnect3DFlag & NCDEF_FCONNECT3D_SPIRAL ) 
	{/*螺旋连接*/
		NcBound_ConnectAllPath( DriveMdl, m_cSetupDef, m_cFeedDef,
								m_cFeedDef.m_cStepDef.m_dOverStep,AllPath, 
                                m_cPrgDef,tmpCnt3D.m_dMaxDist,float(25.0*m_cFeedDef.m_cStepDef.m_dOverStep),
								bZigzag ) ;
    }
    else 
    {//根据行号连接多段刀具路径
        AllPath.ConnectPathByLineNo3D( DriveMdl, m_cSetupDef.m_cTolDef,tmpCnt3D , m_cPrgDef);
    }
	if( m_cPrgDef.m_pBrkFunc && m_cPrgDef.m_pBrkFunc() )
   {
	    m_nErrorType = JDERROR_GENPATH_ABORT  ;
        return FALSE ;
   }
    return TRUE ;
}
// 生成双轨扫描路径
int CSmartSGuideGen::CreateTwoSweepPath( CSmartCurveLib& AllCurve,
                                         CSmtCheckMdl& DriveMdl  ,
                                         CSmtCPathLib& AllPath   ,
                                         CSmartLoop*   AllCont   ) 
{
    // STEP 1 : 获得原始曲线
	if( AllCurve.m_nNumCurve != 2 )
    {
	    MathCAM_ReportErrorGenPath( IDS_ERROR_GENPATH_TWORAIL, MB_ICONSTOP ) ;
        m_nErrorType=JDERROR_GENPATH_NOPATH;
        return 0 ;
    }
    SurfNC_SetNewStepEx(IDS_PATHGEN_STEP_ORGPATH ) ;
	JDNC_TOL tmpTol = GetSurfaceTol() ;
	JDNC_PRGDEF tmpPrg = m_cPrgDef ;
	tmpPrg.m_pPosFunc = NULL ;
	tmpPrg.m_pPrgFunc = NULL ; 
    CSmtCutPath  railPath1, railPath2 ;
	if( AllCurve.m_nNumCurve > 1 && 
        m_cParam.m_bSGuideFlag & NCDEF_FSGUIDE_AUTODIR )
    {
        PNT2D dStart, dEnd ;
        if( AllCurve.m_pHead->IsClosed() && 
            AllCurve.m_pTail->IsClosed()    )
        {/*封闭曲线调整起点和方向*/
            double dDist ;
            AllCurve.m_pHead->GetPoint( 0.0, dStart ) ;
            CSmartSect* pSect = AllCurve.m_pTail->MinDistSect( dStart, dEnd, dDist ) ;
            AllCurve.m_pTail->SetStartPoint( pSect, dEnd ) ;
            double dArea1 = AllCurve.m_pHead->CalcArea() ;
            double dArea2 = AllCurve.m_pTail->CalcArea() ;
            if( dArea1 * dArea2 < 0.0 ) 
            {
                AllCurve.m_pTail->Reverse() ;
            }
        }
        else
        {/*开口曲线调整起点和方向*/
			// edit by liuxin, 2013.8.1
			DOUBLE dDis1 = 0.0, dDis2 = 0.0;
			PNT2D ptArr1[12], ptArr2[12];
			for ( int i = 0; i <= 10; ++i )
			{
				AllCurve.m_pHead->GetPoint ( 0.1 * i, ptArr1[i] ) ;
				AllCurve.m_pTail->GetPoint ( 0.1 * i, ptArr2[i] ) ;
			}
			for ( int i = 0; i <= 10; ++i )
			{
				dDis1 = dDis1 + mathDist2D( ptArr1[i], ptArr2[i] ) ;
				dDis2 = dDis2 + mathDist2D( ptArr1[i], ptArr2[10 - i] ) ;
			}
			if (dDis2 < dDis1)
			{
				AllCurve.m_pTail->Reverse() ;
			}
        }
    }
	railPath1.AddCurve( *AllCurve.m_pHead, tmpTol ) ;
	railPath2.AddCurve( *AllCurve.m_pTail, tmpTol ) ;
	CSmtCPathLib  AllSect ;
	DOUBLE fOverStep = m_cFeedDef.m_cStepDef.m_dOverStep ;
    if( m_cPrgDef.m_pBrkFunc && m_cPrgDef.m_pBrkFunc() )
    {
	    m_nErrorType = JDERROR_GENPATH_ABORT  ;
	    return  FALSE ;
    }
    BOOL bByRail = FALSE ;
    DOUBLE fNetStep = fOverStep ;
	if( m_cMethodDef.m_cSGuideCut.m_bSGuideFlag & NCDEF_FSGUIDE_BYRAIL )
	{
        fNetStep = min( tmpTol.m_dMaxStep, fOverStep * 3.0 ) ;
        if( fNetStep < 0.10 ) fNetStep = 0.10 ;
        bByRail = TRUE ;
    }
    if( bByRail ) m_cPrgDef.m_dTotalMove = 35.0 ;
    else m_cPrgDef.m_dTotalMove = 70.0 ;
    AllSect.CreateTwoRailSection( DriveMdl, tmpTol, fNetStep , railPath1, railPath2, m_cPrgDef, bByRail ) ;
    if( m_cPrgDef.m_pBrkFunc && m_cPrgDef.m_pBrkFunc() )
	{
		m_nErrorType = JDERROR_GENPATH_ABORT  ;
		return  FALSE ;
	}
    if( bByRail )
    {
        m_cPrgDef.m_dTotalMove = 35.0 ;
	    AllSect.ExtractAlongRailPath( DriveMdl, tmpTol, fOverStep , AllPath, m_cPrgDef ) ;
		if( m_cPrgDef.m_pBrkFunc && m_cPrgDef.m_pBrkFunc() )
		{
			m_nErrorType = JDERROR_GENPATH_ABORT  ;
			return  FALSE ;
		}
	}
    else
    {
	 	while( AllSect.m_cAllPath.GetCount() ) 
		{
			CSmtCutPath* pTPath = AllSect.m_cAllPath.RemoveHead() ;
			AllPath.AddToTail( pTPath ) ;
		}
	}
    AllPath.DelPointOnLine( 2.0e-4 ) ;
    if( AllCont )
    {
        CSmtCPathLib tmpCLib ;
        AllPath.TrimCutPathByContours( tmpCLib, AllCont ) ;
        AllPath.ClearAllPath() ;
        while( tmpCLib.m_cAllPath.GetCount() )
        {
           CSmtCutPath* pTPath = tmpCLib.m_cAllPath.RemoveHead() ;
           AllPath.AddToTail( pTPath) ;
        }
    }
    // STEP　2 : 曲线投影
    BOOL bZigzag = ( m_cParam.m_bSGuideFlag & NCDEF_FSGUIDE_ZIGZAG ) ? TRUE : FALSE ;
    JDNC_CONNECT3D  tmpCnt3D = m_cFeedDef.m_cConnectDef ;
    if( bZigzag ) 
    {
        tmpCnt3D.m_bConnect3DFlag |= NCDEF_FCONNECT3D_ZIGZAG ;
    }
    else
    {
        tmpCnt3D.m_bConnect3DFlag &= ~NCDEF_FCONNECT3D_ZIGZAG ;
    }
    tmpCnt3D.m_dSafeDist = m_cFeedDef.m_cStepDef.m_dOverStep ;
    if( m_cSetupDef.m_cOrderDef.m_nLayerOrd == 1 && 
        tmpCnt3D.m_dSafeDist < m_pTool->m_fRadius * 2.0  )
    { //高度优先
        tmpCnt3D.m_dSafeDist = m_pTool->m_fRadius * 2.0 ;
    }
    if ( m_cSetupDef.m_cModelTol.m_nMdlFlag & NCDEF_SETUP_SMOOTHCORNER )
	{/*光滑尖角*/
		AllPath.SmoothAllPath ( DriveMdl, m_cSetupDef, m_cPrgDef ) ;
	}
    m_cPrgDef.m_dTotalMove = 30.0 ;
	if (m_cFeedDef.m_cConnectDef.m_bConnect3DFlag & NCDEF_FCONNECT3D_SPIRAL ) 
	{/*螺旋连接*/
		NcBound_ConnectAllPath( DriveMdl, m_cSetupDef, m_cFeedDef,
								m_cFeedDef.m_cStepDef.m_dOverStep,AllPath, 
								m_cPrgDef,tmpCnt3D.m_dMaxDist,float(25.0*m_cFeedDef.m_cStepDef.m_dOverStep),
								bZigzag ) ;
    }
    else 
    {//根据行号连接多段刀具路径
        AllPath.ConnectPathByLineNo3D( DriveMdl,m_cSetupDef.m_cTolDef, tmpCnt3D, m_cPrgDef);
    }
	if( m_cPrgDef.m_pBrkFunc && m_cPrgDef.m_pBrkFunc() )
   {
	    m_nErrorType = JDERROR_GENPATH_ABORT  ;
        return FALSE ;
   }
    return TRUE ;
}

int CSmartSGuideGen::GeneratePathEx( CPathGroup&   NewPath,  /*雕刻路径*/
						              CSmartGraphic& Graph )  /*加工图形*/ 
{
	// STEP 1 : 转换刀具
    if( !UpdateTool3D(Graph) ) return 0 ;
	CSmtCPathLib Curve3D ;
	CSmartCurveLib AllCurve ;
	CPtrList  AllList ;
	if( ! Graph.GetAllEntity( m_cSetupDef, AllList, SMARTGRAPH_TYPE_GUIDECURVE ) )
    {
		m_nErrorType=JDERROR_GENPATH_NOCURVE;
		return 0 ;
    }
	if( m_cParam.m_nSGuideType == NCDEF_SGUIDETYPE_NPROJECT)
    {
		// 干掉AllList，重新获得
		while( AllList.GetCount() ) 
		{
			CSmartCurve* pCurve = (CSmartCurve*) AllList.RemoveHead() ;
			delete pCurve ;
		}
		if( !Graph.GetGuideCurve ( m_cSetupDef, AllList ) )
		{
			m_nErrorType=JDERROR_GENPATH_NOCURVE;
			return 0 ;
		}
		// 将曲线吸附到曲面上，得到带高度的路径
        JDNC_TOL tmpTol = GetSurfaceTol() ;
		while( AllList.GetCount() ) 
        {
			CSmartCurve* pCurve = (CSmartCurve*) AllList.RemoveHead();
			if( ! pCurve ) continue ;
//			CSmtCutPath* pCutPath = new CSmtCutPath() ;
//			pCutPath->AddCurve( *pCurve, tmpTol ) ;
			CSmtCutPath *pCutPath = GetSmtCutPath(  *pCurve, tmpTol ) ;
			if( pCutPath ) 
				Curve3D.AddToTail( pCutPath ) ;
			delete pCurve ;
		}

	}
	else
	{
		while( AllList.GetCount() ) 
		{
			CSmartCurve* pCurve = (CSmartCurve*) AllList.RemoveHead() ;
			AllCurve.AddTail( pCurve ) ;
		}
		if( AllCurve.m_nNumCurve != 2 &&  m_cParam.m_nSGuideType == NCDEF_SGUIDETYPE_TWOSWEEP  )
		{
			MathCAM_ReportErrorGenPath( IDS_ERROR_GENPATH_TWORAIL, MB_ICONSTOP ) ;
			m_nErrorType=JDERROR_GENPATH_NOPATH;
			return 0 ;
		}
	}
	// STEP 2 : 构建检查模型
	CSmartLoop*		AllCont=Graph.GetAllContour(m_cSetupDef);
    OffsetBorder(AllCont);
	SurfNC_SetNewStepEx(IDS_PATHGEN_STEP_CHECKMDL ) ;
	CSmtCheckMdl  DriveMdl, CheckMdl ;
	if( !BuildDriveAndCheckMdl( Graph, DriveMdl, CheckMdl ) )
	{
		m_nErrorType = JDERROR_GENPATH_NOSURFACE  ;
		return 0 ; 
	}
    BOOL bRet = FALSE ;
	// STEP 3 : 计算原始路径
	CSmtCPathLib AllPath    ;  /*选择路径*/
    SmartNC_RegCreateConnect3DPath( MathCAM_GenNormalLeadPathEx );
    if( m_cParam.m_nSGuideType == NCDEF_SGUIDETYPE_NPROJECT )
    {
       bRet = CreateAdsorbPath( Curve3D, DriveMdl, AllPath, AllCont ) ;
    }
    else if( m_cParam.m_nSGuideType == NCDEF_SGUIDETYPE_ZPROJECT )
	{
        bRet = CreateZProjectPath( AllCurve, DriveMdl, AllPath, AllCont ) ;
    }
    else if( m_cParam.m_nSGuideType == NCDEF_SGUIDETYPE_ONESWEEP )
    {
        bRet = CreateOneSweepPath( AllCurve, DriveMdl, AllPath, AllCont ) ;
    }
    else if( m_cParam.m_nSGuideType == NCDEF_SGUIDETYPE_TWOSWEEP )
    {
        bRet = CreateTwoSweepPath( AllCurve, DriveMdl, AllPath, AllCont ) ;
    }
    Mini_DeleteContours( AllCont ) ;
    SmartNC_RegCreateConnect3DPath( NULL );
    if( bRet == FALSE )
    {
        return FALSE ;
    }
    // STEP 4 : 转换路径
    if( m_cParam.m_nSGuideType == NCDEF_SGUIDETYPE_ZPROJECT )
    {
        CPathCombine * pPComb = new CPathCombine( NC_WPROCESS_ROUGH ) ;
        AllPath.AddToPathCombine( * pPComb   ) ;
        NewPath.AddData ( 0., pPComb ) ;
        AddPlungeAndRelativeH( &DriveMdl, &NewPath, TRUE ) ;
    }
    else
    {
        bRet = AddLeadPathAndAddInPGroup(DriveMdl, AllPath, NewPath);
    }
  	return bRet ;
}

// 从SmartCurve转换到CSmtCutPath，保留高度
CSmtCutPath * CSmartSGuideGen::GetSmtCutPath( CSmartCurve& Curve, JDNC_TOL& Tol )
{
	CSmartSect *pSect  = Curve.GetHead() ;
	if( ! pSect ) return NULL ;
	DOUBLE point[3] ;
	
	CSmtCutPath *pPath = new CSmtCutPath() ;
	pSect->GetStart ( point ) ;
	point[2] = pSect->m_dZValue[0] ;
	pPath->AddPoint ( point ) ;

	for( ; pSect ; pSect = pSect->next )
	{
		if( pSect->IsSectLine () )
		{
			pSect->GetEnd ( point ) ;
			point[2] = pSect->m_dZValue[1] ;
			pPath->AddPoint ( point ) ;
		}
		else if( pSect->IsSectArc () )
		{
			PNT2D dBuff[5000] ;
			CSmartArc *pArc = ( CSmartArc *)pSect ;
			int nCnt = pArc->Discrete ( Tol.m_dArcTol, dBuff, 5000 ) ;
			for( int i = 0 ; i < nCnt ; i++ )
			{
				point[0] = dBuff[i][0] ;
				point[1] = dBuff[i][1] ;
				point[2] = pArc->m_dZValue[0] ;
				pPath->AddPoint ( point ) ;
			}
		}
		else
		{
			ASSERT( 0 ) ;
		}
	}
	pPath->DefineBox () ;
	return pPath ;
}
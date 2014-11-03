#include "StdAfx.H"
#include "SmtPathGen.H"
#include "SmtPathGen2D.H"
#include "SmtPathGen3D.H"
#include "SmtFlowlineGen.h"
#include "SmartWrap.H"
#include "mathcam.h"
#include "Section.h"
#include "NcGCode.H"
#include "Nc5DToolAxis.h"
#include "Nc5DGenSideOffPath.h"
#include "SurfEtt.H"
#include "PathEtt.H"
#include "SurfTrim.h"
#include "CrvProjToSurf.H"
#include "CrvPullToSurf.h"
#include "SurfDraft.h"
#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

void MathCAM_ResetPathDepth( CSmtCPathLib& AllPath, int Flag )
{
	CSmtCutPath* pPath ;
	CSmtCutPoint* pPoint ;
    POSITION  pos = AllPath.m_cAllPath.GetHeadPosition() ;
	while( pos ) 
	{
		pPath = AllPath.m_cAllPath.GetNext( pos ) ;
		if( ! pPath ) continue ;
		for( pPoint = pPath->m_pHead ; pPoint ; pPoint = pPoint->next)
		{
			if( Flag == 0  ) pPoint->m_fPoint[3] = pPoint->m_fPoint[2] ;
			else  pPoint->m_fPoint[2] += pPoint->m_fPoint[3] ;
		}
	}
}


////////////////////////////
// 投影加工
CSmartProjectGen::CSmartProjectGen()          
{
}
CSmartProjectGen::~CSmartProjectGen() 
{
}
int CSmartProjectGen::GeneratePathEx( CPathGroup&   NewPath ,  /*雕刻路径*/
					                  CSmartGraphic& Graph  )  /*雕刻图形*/
{
	// STEP 1 : 转换刀具
	DOUBLE    dSurfStock = GetDriveSurfOffset() ;
	DOUBLE    dCheckStock = GetCheckSurfOffset() ;
	JDNC_TOOLEX tmpTool = m_cToolDef ;
	if( m_cParam.m_bProjectFlag & NCDEF_FPROJECT_RCLOSE )
	{
		tmpTool.m_nToolType = surfncToolFlat ;
		tmpTool.m_dTopDiam  = 0.005 ;
		dSurfStock   = dCheckStock = 0.0 ;
	}
	m_pTool = CreateSmartTool( tmpTool,  dSurfStock, 0.0 ) ;
	if( ! m_pTool ) 
	{
		m_nErrorType = JDERROR_GENPATH_NOTOOL ;
		return 0 ;
	}

	m_pSafeTool = CreateSmartTool( tmpTool, dCheckStock, 0.) ;
	// STEP 2 : 准备投影数据
	CPtrList   AllEnt ;
	Graph.GetAllEntity( m_cSetupDef, AllEnt, SMARTGRAPH_TYPE_PATHGROUP ) ;
	if( AllEnt.GetCount() == 0 ) 
	{
		m_nErrorType = JDERROR_GENPATH_NOPATH  ;
		return 0 ;
	}
	// STEP 3 : 准备曲面
	CSmartTool*  pMyTool = NULL, *pMySafeTool = NULL  ;
	CJDToolGroup *pCurrToolGroup = MathCAM_GetCurrToolGroup() ;
	SurfNC_SetNewStepEx(IDS_PATHGEN_STEP_CHECKMDL ) ;
	CSmtCheckMdl  DriveMdl, CheckMdl ;
	DOUBLE dBox[2][3], dSideTol = m_pTool->m_fRadius ;
	Graph.GetEntBox( dBox, SMARTGRAPH_TYPE_PATHGROUP ) ;
	if( ( m_cParam.m_bProjectFlag & NCDEF_FPROJECT_DELETE ) &&
		!(m_cParam.m_bProjectFlag & NCDEF_FPROJECT_RCLOSE  )  )
	{ // 删除原始路径, (实际就是投影变换)
		POSITION atPos = AllEnt.GetHeadPosition() ;
		while( atPos && pCurrToolGroup )
		{
		    CPathGroup* pPathGroup = ( CPathGroup*) AllEnt.GetNext( atPos ) ;
			CJDTool*  pNcTool = pCurrToolGroup->GetByID( pPathGroup->m_nToolID ) ;
			if( pNcTool && pNcTool->m_cToolParam.m_dTopDiam/2 > dSideTol )
			{
				dSideTol = pNcTool->m_cToolParam.m_dTopDiam/2 ;
			}
		}
	}
	DriveMdl.m_dFltBox[0][0] = dBox[0][0] - dSideTol ;
	DriveMdl.m_dFltBox[0][1] = dBox[0][1] - dSideTol ;
	DriveMdl.m_dFltBox[1][0] = dBox[1][0] + dSideTol ;
	DriveMdl.m_dFltBox[1][1] = dBox[1][1] + dSideTol ;
	if( ! Graph.BuildCheckModel( m_cSetupDef, DriveMdl,NCDEF_SURF_DRIVE ) )
	{
		m_nErrorType = JDERROR_GENPATH_NOSURFACE  ;
	 	return 0 ; 
	}
  	DriveMdl.ZMoveModel( dSurfStock ) ;
 	DriveMdl.UpdateCurrTool( m_pTool )  ;
	DriveMdl.m_fBottom = (float)GetBottomHeight()  ;
	DriveMdl.m_bCheckMode = SMART_MODEL_PROJECT ;
	memcpy( CheckMdl.m_dFltBox, DriveMdl.m_dFltBox, sizeof( double[2][3])) ;
    Graph.BuildCheckModel( m_cSetupDef, CheckMdl, NCDEF_SURF_CHECK ) ;
	CheckMdl.ZMoveModel( dCheckStock ) ;
	CheckMdl.m_fBottom =  (float)GetBottomHeight() ;
    CheckMdl.UpdateCurrTool( m_pSafeTool )  ;
	if( CheckMdl.m_cAllChk.GetSize() > 0 ) 
	{
		DriveMdl.SetCheckMdl( & CheckMdl ) ;
	}
	SurfNC_SetCurPos( 8, 10 ) ;
    // STEP 4 :干涉检查, 生成无干涉的刀具路径
	CSmtCPathLib AllPath ;
	BOOL  bReDepth = m_cParam.m_bProjectFlag & NCDEF_FPROJECT_REMAIN ? TRUE : FALSE ;
	char strName[200], strFmt[100] ;
	glbf_LoadSysString( IDS_PATHGEN_PROJECT_INDEX, strFmt, 100 ) ;
	INT_PTR nEntCount = AllEnt.GetCount() , nAtIndex = 1 ;
	POSITION atPos = AllEnt.GetHeadPosition() ;
	while( atPos ) 
	{
	    sprintf( strName,strFmt , nAtIndex, nEntCount) ;
		SurfNC_SetNewStep( strName ) ;
		
		nAtIndex ++ ;
		CPathGroup* pPathGroup = ( CPathGroup*) AllEnt.GetNext( atPos ) ;
		MathCAM_AddPathGroup( AllPath, * pPathGroup , m_cSetupDef.m_cTolDef , bReDepth ) ;
		if( bReDepth ) MathCAM_ResetPathDepth( AllPath, 0 ) ;
		if( ( m_cParam.m_bProjectFlag & NCDEF_FPROJECT_DELETE ) &&
			!(m_cParam.m_bProjectFlag & NCDEF_FPROJECT_RCLOSE  )  )
		{ // 删除原始路径, (实际就是投影变换)
			CJDTool* pNcTool = NULL ; 
			if( pCurrToolGroup ) pNcTool = pCurrToolGroup->GetByID( pPathGroup->m_nToolID ) ;
			if( pNcTool && pNcTool->GetToolID() != DriveMdl.m_pTool->m_nToolID ) 
			{
				if( pMyTool ) delete pMyTool ; 
				if( pMySafeTool ) delete pMySafeTool ;
				pMyTool = CreateSmartTool( pNcTool->m_cToolParam, dSurfStock, 0.0 ) ;
			    if( pMyTool ) 
                {
                    DriveMdl.UpdateCurrTool( pMyTool )  ;
                }
				pMySafeTool = CreateSmartTool( pNcTool->m_cToolParam, dCheckStock, 0. ) ;
                if( pMySafeTool )
                {
                    CheckMdl.UpdateCurrTool( pMySafeTool ) ;
                }
			}
		}
		INT_PTR nPnt = AllPath.GetNumPoint() ;
		if( nPnt < 1 )  nPnt = 1      ;
		m_cPrgDef.m_dLimitAt  =  nPnt / 100.0   ;   // 前进一次的计算量
		m_cPrgDef.m_dStepAt   =  0.0            ;   // 当前计算进度
		m_cPrgDef.m_dIncStep  =  1.0            ;   // 每次计算的前进量
		if( ! AllPath.VerifyCutPath( DriveMdl, GetSurfaceTol(), m_cPrgDef ) )
		{
			break ;
		}
        CPathCombine * pPComb = new CPathCombine( NC_WPROCESS_IMPORT ) ;
		pPComb->m_pEntFrom  = pPathGroup->m_pEntFrom  ;
		if( bReDepth ) MathCAM_ResetPathDepth( AllPath, 1 ) ;
	    AllPath.DelPointOnLine(2.0e-4) ; // 删除同一直线上的点
		AllPath.AddToPathCombine( * pPComb   ) ;
	 	AllPath.ClearAllPath() ;
        CPathGroup tmpPGroup(NC_PATHTYPE_PSURFACE) ;
	    tmpPGroup.AddData( 0.0 , pPComb ) ;
	    AddPlungeAndRelativeH( &DriveMdl, &tmpPGroup ) ;
        while( tmpPGroup.m_pHead )
        {
            pPComb = tmpPGroup.m_pHead ;
            tmpPGroup.RemoveCombine( pPComb ) ;
            NewPath.AddData( 0.0, pPComb ) ;
        }
	}
    if( pMyTool ) delete pMyTool ; 
	if( pMySafeTool ) delete pMySafeTool ; 
	pMyTool = NULL ;
	atPos = AllEnt.GetHeadPosition() ;
	while( atPos ) 
	{
		CPathGroup* pPathGroup = ( CPathGroup*) AllEnt.GetNext( atPos ) ;
		delete pPathGroup ;
	}
	AllEnt.RemoveAll() ;
    if( m_cPrgDef.m_pBrkFunc && m_cPrgDef.m_pBrkFunc() )
	{
	    m_nErrorType = JDERROR_GENPATH_ABORT  ;
		return 0 ; 
	}
	return 1 ;
}
// 分层粗加工方式
int CSmartProjectGen::LayerRoughPath( CPathGroup&   PGroup  , /*路径组　*/
								     CSmtCPathLib& AllPath , /*所有路径*/
								     CSmtCheckMdl& DriveMdl, /*检查模型*/
									 JDNC_PROJECT& /*PrjCut*/  ) /*投影参数*/
{
	CSmtCPathLib  PathNew ;
	if( GetDownStep() < 0.002 ) return 0 ;
	DOUBLE dZValue   = GetTopHeight() - GetDownStep() ;
	DOUBLE dMaxDepth = GetBottomHeight() ;
	while( dZValue > dMaxDepth )
	{
		AllPath.TrimByZValue( PathNew , dZValue ) ;
		PathNew.ConnectAllPath( DriveMdl, 5.0 ) ;
		CPathCombine * pPComb = new CPathCombine( NC_WPROCESS_ROUGH ) ;
		PathNew.AddToPathCombine( * pPComb ) ;
		PathNew.ClearAllPath() ;
		if( pPComb->m_pHead )	PGroup.AddData( 0.0, pPComb ) ;
		else delete pPComb ;
		dZValue -= GetDownStep() ;
	}
	if( AllPath.GetNumPoint() ) 
	{
        CPathCombine * pPComb = new CPathCombine( NC_WPROCESS_ROUGH ) ;
		PathNew.AddToPathCombine( * pPComb ) ;
		PathNew.ClearAllPath() ;
	    PGroup.AddData( 0.0, pPComb ) ;
	}
	return 1 ; 
}
// 分层精加工方式
int CSmartProjectGen::LayerFinishPath( CPathGroup&   PGroup  , /*路径组　*/
								      CSmtCPathLib& AllPath , /*所有路径*/
								      CSmtCheckMdl& /*DriveMdl*/, /*检查模型*/
									  JDNC_PROJECT& PrjCut  ) /*投影参数*/
{
	double dTotalDepth = PrjCut.m_dProjectDepth ;
	double dDownStep   = GetDownStep() ;
	if( dDownStep < 1.0e-6 ) dTotalDepth ; 
	int nCount =  int( dTotalDepth / dDownStep) ;
	if( nCount > 1000 ) 
	{
		nCount = 1000 ;
		dDownStep  = dTotalDepth / nCount ;
	}
	if( ( dTotalDepth - nCount * dDownStep) > 1.0e-4 )
	{
		nCount ++ ;
	}
	double dDepth  = 0.0 ; 
	for( int i = 0 ; i < nCount ; i ++ ) 
	{
		if( i == nCount-1 ) dDepth = dTotalDepth ;
		else dDepth += dDownStep ;
        CPathCombine * pPComb = new CPathCombine( NC_WPROCESS_ROUGH ) ;
		AllPath.AddToPathCombine( * pPComb ) ;
		if( PrjCut.m_bProjectFlag & NCDEF_FPROJECT_SURFACE )
		{ // 加工曲面
		    pPComb->MoveCombine( 0.0, 0.0, dTotalDepth - dDepth ) ;
		}
		else 
		{ // 投影雕刻曲面
		    pPComb->MoveCombine( 0.0, 0.0, - dDepth ) ;
		}
		PGroup.AddData( 0.0, pPComb ) ;
	}
	return 1 ;
}
////////////////////////////
// 包裹加工
CSmartWrapGen::CSmartWrapGen()          
{
}
CSmartWrapGen::~CSmartWrapGen() 
{
}
int CSmartWrapGen::GeneratePathEx( CPathGroup&   NewPath ,  /*雕刻路径*/
					               CSmartGraphic& Graph  )  /*雕刻图形*/
{
	// STEP 1 : 转换刀具
	JDNC_TOOLEX tmpTool = m_cToolDef ;
	tmpTool.m_nToolType = surfncToolFlat ;
	tmpTool.m_dTopDiam  = 0.01 ;
	m_pTool = CreateSmartTool( tmpTool, 0.0, 0.) ;
	if( ! m_pTool ) 
	{
		m_nErrorType = JDERROR_GENPATH_NOTOOL ;
		return 0 ;
	}
	// STEP 2 : 准备投影数据
	CPtrList   AllEnt ;
	Graph.GetAllEntity( m_cSetupDef, AllEnt, SMARTGRAPH_TYPE_PATHGROUP ) ;
	if( AllEnt.GetCount() == 0 ) 
	{
		m_nErrorType = JDERROR_GENPATH_NOPATH  ;
		return 0 ;
	}
	// STEP 3 : 准备曲面
	SurfNC_SetNewStepEx(IDS_PATHGEN_STEP_CHECKMDL ) ;
	CSmtCheckMdl  DriveMdl ;
	PNT3D dMinPt, dMaxPt, dCenter ;
	dCenter[0] = m_cParam.m_dWrapCenter[0] ;
	dCenter[1] = m_cParam.m_dWrapCenter[1] ;
	dCenter[2] = 0.0 ; 
	DOUBLE dAngle = 0.0 ;
	dAngle = ANGLE_TO_RADIAN( m_cParam.m_dWrapAngle ) ;
	if( ! Graph.BuildCheckModel( m_cSetupDef, DriveMdl ) )
	{
		m_nErrorType = JDERROR_GENPATH_NOSURFACE  ;
	 	return 0 ; 
	}
	DriveMdl.MoveModel( -dCenter[0], -dCenter[1], 0.0 ) ;
	if( fabs( dAngle ) > 1.0e-3 )
	{
		DriveMdl.RotateByZAxis( -dAngle ) ;
	}
    DriveMdl.UpdateCurrTool( m_pTool )  ;
	DriveMdl.m_fBottom = (float)GetBottomHeight()  ;
	DriveMdl.DefineDriveSurfBox( dMinPt, dMaxPt ) ;
	// STEP 4 : 构建包裹模型
	CSmartWraper  Wraper ;
	double dStep = m_cSetupDef.m_cTolDef.m_dMaxStep ;
	double dAt[2] ;
	dAt[0] = dAt[1] = 0.0 ;
	Wraper.InitBuffer( dMinPt, dMaxPt, dStep , dStep, dAt ) ;
	Wraper.CheckCloud( DriveMdl,  m_cPrgDef ) ;
	Wraper.DefineUWParam() ;
	Wraper.DefineNormal() ;
    // STEP 4 :干涉检查, 生成无干涉的刀具路径
	CSmtCPathLib AllPath ;
	BOOL  bReDepth = m_cParam.m_bWrapFlag & NCDEF_FWRAP_REMAIN ? TRUE : FALSE ;
	POSITION atPos = AllEnt.GetHeadPosition() ;
	while( atPos ) 
	{
		CPathGroup* pPathGroup = ( CPathGroup*) AllEnt.GetNext( atPos ) ;
		pPathGroup->MoveGroup( -dCenter[0], -dCenter[1], 0.0 ) ; 
		if( fabs(dAngle) > 1.0e-3 ) pPathGroup->RotateGroup( -dAngle ) ;
		MathCAM_AddPathGroup( AllPath, * pPathGroup , m_cSetupDef.m_cTolDef, bReDepth) ;
		if( ! Wraper.WrapCutPath( AllPath, m_cSetupDef.m_cTolDef, m_cPrgDef ) )
		{ // 用户中断
			break ;
		}
	    AllPath.DelPointOnLine(2.0e-4) ; // 删除同一直线上的点
        CPathCombine * pPComb = new CPathCombine( NC_WPROCESS_ROUGH ) ;
		pPComb->m_pEntFrom  = pPathGroup->m_pEntFrom  ;
		AllPath.AddToPathCombine( * pPComb   ) ;
		if( fabs( dAngle ) > 1.0e-3 ) pPComb->RotateCombine( dAngle ) ;
		pPComb->MoveCombine( dCenter[0], dCenter[1], 0.0 ) ;
	    NewPath.AddData( 0.0, pPComb ) ;
	 	AllPath.ClearAllPath() ;
	}
	atPos = AllEnt.GetHeadPosition() ;
	while( atPos ) 
	{
		CPathGroup* pPathGroup = ( CPathGroup*) AllEnt.GetNext( atPos ) ;
		delete pPathGroup ;
	}
	AllEnt.RemoveAll() ;
    if( m_cPrgDef.m_pBrkFunc && m_cPrgDef.m_pBrkFunc() )
	{
	    m_nErrorType = JDERROR_GENPATH_ABORT  ;
		return 0 ; 
	}
  	NewPath.ZMoveGroup( GetDriveSurfZMove() ) ;
	return 1 ;
}
//////////////////////////////////////////////////////////////////////////////////////////
//CSmartRotate3DGen 
CSmartRotate3DGen::CSmartRotate3DGen()
{
}
CSmartRotate3DGen::~CSmartRotate3DGen()
{
}
int CSmartRotate3DGen::LayerAllPath( CPathGroup&   PGroup  , /*路径组　*/
							  	     CSmtCPathLib& AllPath , /*所有路径*/
								     CSmtCheckMdl& DriveMdl, /*检查模型*/
								     JDNC_PRGDEF&  ProgDef ) /*进度计算*/ 
{
    SurfNC_SetNewStepEx(IDS_PATHGEN_STEP_LEVEL ) ;
	CSmtCPathLib  PathNew ;
	JDNC_LAYER *pLayer = GetLayerDef() ;
	DOUBLE dZValue   = 0.0 ;
	JDNC_PRGDEF tmpPrgDef = ProgDef ;
	tmpPrgDef.m_pPrgFunc  = NULL ; 
	tmpPrgDef.m_pBrkFunc  = NULL ; 
    tmpPrgDef.m_pPosFunc  = NULL ;
	for( int nLayer = 1 ; nLayer <= pLayer->m_nLayerCount ; nLayer ++ ) 
	{
        if( ProgDef.m_pPrgFunc  )
		{
			ProgDef.m_pPosFunc( nLayer,  pLayer->m_nLayerCount ) ;
		}
		if( AllPath.GetNumPath() == 0 ) break   ;
		dZValue = GetTopHeight() - pLayer->m_dLayerDepth[nLayer] ;
		AllPath.TrimByZValue( PathNew , dZValue ) ;
		POSITION atPos = PathNew.m_cAllPath.GetHeadPosition() ;
		while( atPos )
		{
			CSmtCutPath* pPath = (CSmtCutPath*) PathNew.m_cAllPath.GetNext( atPos ) ;
			pPath->m_nLayerNo = nLayer ;
		}
		if( m_cParam.m_bRotate3DFlag & NCDEF_FROTATE3D_ZIGZAG )
		{
            DOUBLE dMaxDist = m_cFeedDef.m_cConnectDef.m_dMaxDist ;
			PathNew.ConnectPathByLineNo( DriveMdl, dMaxDist,m_cSetupDef.m_cTolDef, tmpPrgDef , TRUE) ;
		}
        CPathCombine * pPComb = new CPathCombine( NC_WPROCESS_ROUGH ) ;
		PathNew.AddToPathCombine( * pPComb ) ;
		PathNew.ClearAllPath() ;
		if( pPComb->m_pHead )	PGroup.AddData( 0.0, pPComb ) ;
		else delete pPComb ; 
		/* 用户中断 || 进度提示 */
        if( ProgDef.m_pBrkFunc &&  ProgDef.m_pBrkFunc()  )
		{ 
			return 0 ;
		}
	}
	if( AllPath.GetNumPoint() ) 
	{
        CPathCombine * pPComb = new CPathCombine( NC_WPROCESS_ROUGH ) ;
		PathNew.AddToPathCombine( * pPComb ) ;
		PathNew.ClearAllPath() ;
	    PGroup.AddData( 0.0, pPComb ) ;
	}
	return 1 ; 
}
void  MathCAM_CalcPointAngle( CSmtCutPoint& CutPoint )
{
    CutPoint.m_fPoint[3] = atan2( CutPoint.m_fPoint[1], CutPoint.m_fPoint[2]) ;
    if( CutPoint.m_fPoint[3] < 0.0 ) CutPoint.m_fPoint[3] += float(MiniPai2) ;
}
void tmp_YMoveLoop( CSmartLoop& Loop, double YMove )
{
    Loop.m_pCurve->MoveCurve( 0.0, YMove ) ;
    Loop.m_dBox[0][1] += YMove ;
    Loop.m_dBox[1][1] += YMove ;
}
BOOL CSmartRotate3DGen::ExtractFairedBoundPath( CSmartGraphic& Graph ,
                                           CSmtCPathLib& BndPath )
{
    CPtrList   AllComb ;
    Graph.GetAllEntity( m_cSetupDef, AllComb, SMARTGRAPH_TYPE_PATHCOMB ) ;
    if( AllComb.GetCount() == 0 ) return NULL ; 
    // STEP 2 : 空间曲线转换成路径段
    JDNC_TOL tmpTol = GetSurfaceTol() ;
    while( AllComb.GetCount() ) 
	{
       CPathCombine* pPathComb = (CPathCombine*) AllComb.RemoveHead();
       CSmtCutPath * pTPath = NULL ; 
       CSmtCutPoint *pTPoint = NULL ;
       TPNT3D dStart, dEnd ;
       for( CPathEntity* pEntity = pPathComb->m_pHead; pEntity ; pEntity = pEntity->next )
		{ 
			int nType = pEntity->GetType() ;
			pEntity->GetEndPoint( 0 , dStart ) ;
			if( ! pTPath || mathDist( dStart, dEnd ) > 0.01)
			{
		       pTPath  = new CSmtCutPath() ;
			   pTPath->AddPoint( dStart ) ;
			   BndPath.AddToTail( pTPath ) ;
			}
			pEntity->GetEndPoint( 1 , dEnd ) ;
		    if( nType == NC_PATH_PLINE3D )
			{ // 3D 折线
			    CPathPLine3D *pPLine = ( CPathPLine3D*) pEntity ;
			    for( int i = 1 ; i <= pPLine->m_nCount  ; i ++ )
				{
			        pTPoint = new CSmtCutPoint( pPLine->m_pTAPos[i] ) ;
				    pTPath->AddTail( pTPoint ) ;
				} 
			}
		    else if( nType == NC_PATH_LINE3D )
			{ 
				CPathLine3D * pLine3D = ( CPathLine3D*) pEntity ;
		        pTPoint = new CSmtCutPoint( pLine3D->m_fEnd ) ;
			    pTPath->AddTail( pTPoint ) ;
			}
		    else if( nType == NC_PATH_ARC3D )
			{
				TPNT3D dBuff[500] ;
				CPathArc3D * pArc3D = ( CPathArc3D*) pEntity ;
				int n = pArc3D->Discrete( tmpTol.m_dArcTol,ANGLE_TO_RADIAN( tmpTol.m_dAngTol), dBuff, 500 ) ;
			    for( int i = 1 ; i <= n ; i ++ ) 
				{ 
		            pTPoint = new CSmtCutPoint( dBuff[i] ) ;
				    pTPath->AddTail( pTPoint ) ;
				} 
			} 
		    else pTPath = NULL ;
		}
        delete pPathComb ;
    }
    // STEP 3 : 路径段连接,过滤不封闭的曲线
    BndPath.ConnectAllPath( 0.1, 0.0f  ) ;
    return TRUE ;
}
// 计算旋转雕刻的边界轮廓
CSmartLoop*  CSmartRotate3DGen::ExtractFairedContours(CSmartGraphic& Graph) /*雕刻图形*/
{
    // STEP 1 : 提取空间曲线
    CSmtCPathLib BndPath ;
    ExtractFairedBoundPath( Graph, BndPath ) ;
    POSITION atPos = BndPath.m_cAllPath.GetHeadPosition() ;
    while( atPos )
    {
        POSITION atNext = atPos ; 
        CSmtCutPath* pTPath = BndPath.m_cAllPath.GetNext( atNext ) ;
        if( pTPath->IsClosed( 0.1f ) == FALSE )
        {
            BndPath.m_cAllPath.RemoveAt( atPos );
            delete pTPath ;
        }
        else if( nc_Distance( pTPath->m_pHead->m_fPoint, pTPath->m_pTail->m_fPoint, 3 ) < 0.01 )
        {
            nc_VectorCopy( pTPath->m_pTail->m_fPoint, pTPath->m_pHead->m_fPoint, 3 ) ;
        }
        else
        {
            pTPath->AddPoint( pTPath->m_pHead->m_fPoint ) ;
        }
        atPos = atNext ;
    }
    if( BndPath.GetNumPath() == 0 ) return NULL ;
    // STEP 4 : 路径段转换成直线段
    CSmartLoop *AllLoop = NULL ; 
    DOUBLE dRadius = GetTopHeight() ;
    atPos = BndPath.m_cAllPath.GetHeadPosition() ;
    while( atPos )
    {
        CSmtCutPath* pTPath = BndPath.m_cAllPath.GetNext( atPos ) ;
        for( CSmtCutPoint* pTPoint = pTPath->m_pHead; pTPoint ; pTPoint = pTPoint->next )
        {/*计算角度*/
           MathCAM_CalcPointAngle( *pTPoint ) ;
           if( pTPoint->prev == NULL ) continue ;
           while( pTPoint->m_fPoint[3] - pTPoint->prev->m_fPoint[3] > MiniPai )
           {
               pTPoint->m_fPoint[3] -= float(MiniPai2) ;
           }
           while( pTPoint->prev->m_fPoint[3] - pTPoint->m_fPoint[3] > MiniPai )
           {
               pTPoint->m_fPoint[3] += float(MiniPai2) ;
           }
        }
        if( fabs(pTPath->m_pHead->m_fPoint[3] - pTPath->m_pTail->m_fPoint[3]) > 0.01 ) 
        {
            continue ;
        }
        PNT2D dStart, dEnd ;
        CSmartCurve* pCurve = new CSmartCurve() ;
        for( pTPoint = pTPath->m_pHead; pTPoint && pTPoint->next; pTPoint = pTPoint->next )
        {/*计算角度*/
           dStart[0] = pTPoint->m_fPoint[0] ;
           dStart[1] = dRadius * pTPoint->m_fPoint[3] + m_cParam.m_dYOffset;
           dEnd[0] = pTPoint->next->m_fPoint[0] ;
           dEnd[1] = dRadius * pTPoint->next->m_fPoint[3] + m_cParam.m_dYOffset;
           CSmartLine* pLine = new CSmartLine( dStart, dEnd ) ;
           pCurve->AddSect( pLine ) ;
        }
        CSmartLoop* pLoop = new CSmartLoop() ;
        pLoop->UpdateSect( pCurve ) ;
        if( pLoop->m_dArea < 0.0 ) pLoop->ReverseLoop() ;
        AllLoop = Mini_AddContours( AllLoop, pLoop ) ;
    }
    // STEP 6 : 构建加工区域
    DOUBLE dYMin = 0.0, dYMax = 1.0, dSpanDist = dRadius * MiniPai2;
    if( m_cParam.m_dAngle[1] > 0.0 ) 
    {
   	    dYMin = ANGLE_TO_RADIAN( m_cParam.m_dAngle[0] ) * dRadius ;
	    dYMax = ANGLE_TO_RADIAN( m_cParam.m_dAngle[1] + m_cParam.m_dAngle[0]) * dRadius;
    }
    else
    {
   	    dYMax = ANGLE_TO_RADIAN( m_cParam.m_dAngle[0] ) * dRadius ;
	    dYMin = ANGLE_TO_RADIAN( m_cParam.m_dAngle[1] + m_cParam.m_dAngle[0]) * dRadius;
    }

    CSmartLoop tmpLoop ;
    AllLoop = tmpLoop.ResortContour( AllLoop, NCDEF_SORTTYPE_MAXAREA, FALSE ) ;
    CSmartLoop* AllCont = NULL, *pContour, *pLoop ; 
    while( AllLoop )
    {
        pContour = AllLoop ; 
        AllLoop = Mini_RemoveLoop( AllLoop, pContour ) ;
        while( pContour->m_dBox[1][1] < dYMin ) 
        {
            tmp_YMoveLoop( *pContour, dSpanDist ) ;
        }
        while( pContour->m_dBox[0][1] > dYMax) 
        {
            tmp_YMoveLoop( *pContour, -dSpanDist ) ;
        }
        CSmartLoop* pLpNext = NULL ; 
        for( pLoop = AllLoop ; pLoop ; pLoop = pLpNext )
        {
            pLpNext = pLoop->next ;
            PNT2D dMidPnt ;
            pLoop->m_pCurve->GetPoint( 0.5, dMidPnt ) ; 
            double dOldY = dMidPnt[1] ;
            while( pContour->m_dBox[1][1] < dMidPnt[1] ) dMidPnt[1] -= dSpanDist ;
            while( pContour->m_dBox[0][1] > dMidPnt[1] ) dMidPnt[1] += dSpanDist ;
            if( pContour->IsOnContourEx( dMidPnt, 1.0e-4 ) == 1 )
            {
                tmp_YMoveLoop( *pLoop, dMidPnt[1] - dOldY ) ;
                AllLoop = Mini_RemoveLoop( AllLoop, pLoop ) ;
                pContour->AddIsland( pLoop ) ;
            }
        }
        AllCont = Mini_AddContours( AllCont, pContour ) ;
    }
    OffsetBorder(AllCont);
    return AllCont ;
}

int CSmartRotate3DGen::LineIntRotateContours( CSmartLoop* AllCont, 
                                         PNT2D Start, PNT2D End ,
                                         PNT2D Buff[], 
                                         int Size )
{
    if( AllCont == NULL )
    {
        nc_VectorCopy( Buff[0], Start, 2 ) ;
        nc_VectorCopy( Buff[1], End, 2 ) ;
        return 2 ;
    }
    PNT2D  dMidPnt[3] , dPoint[5] ;
    CSmartLoop* pLNext = NULL ; 
    CSmartLine tmpLine( Start, End ) ;
    for( CSmartLoop* pContour = AllCont; pContour ; pContour = pContour->next  )
    for( CSmartLoop* pLoop = pContour ;  pLoop ; pLoop =pLNext) 
    {// 计算交线段
        if( pLoop == pContour ) pLNext = pContour->GetIsland() ;
        else pLNext = pLNext->next ;
        if( tmpLine.m_dBox[0][0] > pLoop->m_dBox[1][0] || 
            tmpLine.m_dBox[0][1] > pLoop->m_dBox[1][1] ||
           tmpLine.m_dBox[1][0] < pLoop->m_dBox[0][0] ||
           tmpLine.m_dBox[1][1] < pLoop->m_dBox[0][1]  )
        {
           continue ;
        }
        for( CSmartSect *pSect = pLoop->GetSectHead() ; pSect ; pSect = pSect->next )
        {
            if( !nc_BoxBoxInt( tmpLine.m_dBox,pSect->m_dBox,NC_BOXINT_ERROR ))
            {
                continue ;
            }
            int nIntPt = Mini_SectSectInt( *pSect, tmpLine, dPoint) ;
			 if( nIntPt == 0 ) continue ;
	        for( int i = 0 ; i < nIntPt ; i ++ ) 
		    {
			    if( nc_FabsDist( tmpLine.m_aPoint[0], dPoint[i], 2 ) < 1.0e-4 || 
			        nc_FabsDist( tmpLine.m_aPoint[1], dPoint[i], 2 ) < 1.0e-4   ) 
               continue ;
               tmpLine.AddIntPt( dPoint[i] ) ;
		    }
        }
	}
    tmpLine.AddIntPt( Start ) ;
    tmpLine.AddIntPt( End   ) ;
    int nNumPnt = 0, bFlag ; 
    CSmartPoint * pIntPnt = tmpLine.m_pInterPt->m_pHead ;
    for(  ; pIntPnt && pIntPnt->next ; pIntPnt = pIntPnt->next ) 
    {
        nc_VectorCopy( dPoint[0], pIntPnt->m_dPoint, 2 ) ;
        nc_VectorCopy( dPoint[1], pIntPnt->next->m_dPoint, 2 ) ;
        nc_GetPointAtLine( dPoint[0], dPoint[1], 0.5, dMidPnt[0], 2 ) ;
        nc_GetPointAtLine( dPoint[0], dPoint[1], 0.25, dMidPnt[1], 2 ) ;
        nc_GetPointAtLine( dPoint[0], dPoint[1], 0.75, dMidPnt[2], 2 ) ;
        bFlag = 0 ;
        for( CSmartLoop* pCont = AllCont; pCont ; pCont = pCont->next )
        {
            bFlag = pCont->IsOnContourEx( dMidPnt[0], 1.0e-4 ) ;
            if( bFlag == 2 ) bFlag = pCont->IsOnContourEx( dMidPnt[1], 1.0e-4 ) ;
            if( bFlag == 2 ) bFlag = pCont->IsOnContourEx( dMidPnt[2], 1.0e-4 ) ;
            if( bFlag == 1 ) break ;
        }
        if( bFlag == 1 && nNumPnt < Size - 1) 
        {
            nc_VectorCopy( Buff[nNumPnt], dPoint[0], 2 ) ;
            nc_VectorCopy( Buff[nNumPnt+1], dPoint[1], 2 ) ;
            nNumPnt += 2 ;
        }
    }
    return nNumPnt ;
}
int CSmartRotate3DGen::CreateRotatePath( CPathGroup& NewPath   , 
									     CSmtCheckMdl& DriveMdl,
                                     CSmartLoop* AllCont   )
{
	// STEP 0 : 计算角度范围
    PNT3D    dStart, dEnd ;
    PNT2D dBuff[100] ;
	DOUBLE dAngle , dRotInc, dRange, dRadius ;
	int nCount ;
	dRadius = GetTopHeight() ;
	if( dRadius < 0.01 )
	{
		return 0 ;
	}
	dRotInc  = m_cParam.m_dOverStep / dRadius ;
	if( dRotInc < 1.0e-4 ) dRotInc = 0.0001 ;
	dAngle = ANGLE_TO_RADIAN( m_cParam.m_dAngle[0] ) ;
	dRange = ANGLE_TO_RADIAN( m_cParam.m_dAngle[1] ) ;
    if( AllCont )
    {
        double dMinAng = 1.0e12, dMaxAng = -1.0e12, dAng;
        for( CSmartLoop* pLoop = AllCont ; pLoop ; pLoop = pLoop->next )
        {
            dAng = pLoop->m_dBox[0][1] / dRadius ;
            if( dMinAng > dAng ) dMinAng = dAng ;
            dAng = pLoop->m_dBox[1][1] / dRadius ;
            if( dMaxAng < dAng ) dMaxAng = dAng ;
        }
        dMinAng += (0.1 * dRotInc ), dMaxAng -= (0.1 * dRotInc ) ;
        if( dRange > 0.0 ) 
        {
            dAngle= max( dAngle,  dMinAng) ;
            dAng  = min( dRange + dAngle, dMaxAng ) ;
            dRange = dAng - dAngle ;
            if( dRange <= 0.0 )
            {
                m_nErrorType = JDERROR_GENPATH_NOAREA ;
                return FALSE ;
            }
        }
        else
        {
            dAngle= min( dAngle,  dMaxAng) ;
            dAng  = max( dRange + dAngle, dMinAng ) ;
            dRange = dAng - dAngle ;
            if( dRange >= 0.0 )
            {
                m_nErrorType = JDERROR_GENPATH_NOAREA ;
                return FALSE ;
            }
        }
    }
	nCount = (int)(fabs( dRange / dRotInc) ) ;
	if( nCount <= 0 ) return 0 ;
	dRotInc = dRange / nCount ;
	dStart[0] = m_cParam.m_dWidth[0]  ;
	dEnd[0]   = m_cParam.m_dWidth[1]  ;
    dStart[1] = dEnd[1] = m_cParam.m_dYOffset ;
    dStart[2] = dEnd[2] = DriveMdl.m_fBottom ;
    // STEP 2 :干涉检查, 生成无干涉的刀具路径
	SurfNC_SetNewStepEx(IDS_PATHGEN_STEP_ORGPATH ) ;
    JDNC_PRGDEF tmpPrg = m_cPrgDef ;
    tmpPrg.m_pPosFunc = NULL  ;
    tmpPrg.m_pPrgFunc = NULL  ; 
    m_cPrgDef.m_dTotalMove = 70.0 ;
    m_cPrgDef.m_dIncStep = m_cPrgDef.m_dStepAt = 0.0 ;
    m_cPrgDef.m_dLimitAt = 1.0 ;
    if( nCount )
    {
        m_cPrgDef.m_dIncStep = m_cPrgDef.m_dTotalMove / nCount ;
    }
	CSmtCPathLib  AllPath ;
    CSmtCheckMdl *pChkMdl = DriveMdl.GetCheckMdl();
    DriveMdl.RotateByXAxis( dAngle) ;
    if( pChkMdl ) pChkMdl->RotateByXAxis( dAngle) ;
	for( int i = 0 ; i < nCount ; i ++ ) 
	{	
		if( m_cPrgDef.m_pBrkFunc && m_cPrgDef.m_pBrkFunc() )
		{
		    m_nErrorType = JDERROR_GENPATH_ABORT  ;
			return FALSE ;
		}
        m_cPrgDef.m_dStepAt += m_cPrgDef.m_dIncStep ;
        while( m_cPrgDef.m_pPrgFunc && m_cPrgDef.m_dStepAt >= m_cPrgDef.m_dLimitAt )
        {
            m_cPrgDef.m_pPrgFunc(1) ;
	        m_cPrgDef.m_dStepAt -= m_cPrgDef.m_dLimitAt  ;
        }
        dStart[0] = min( m_cParam.m_dWidth[0],m_cParam.m_dWidth[1]) ;
		dEnd[0]   = max( m_cParam.m_dWidth[0],m_cParam.m_dWidth[1]) ;
        dStart[1] = dEnd[1] = dRadius * dAngle ;
		if( !( m_cParam.m_bRotate3DFlag & NCDEF_FROTATE3D_ZIGZAG))
		{/*开口线单向走刀*/
			BOOL bRevDir = FALSE ;
			if( GetMillDir() == 1 ) bRevDir = !bRevDir ; /*顺铣*/
			if( dRotInc < 0.0 ) bRevDir = !bRevDir ; /*反向旋转*/
			if( bRevDir )nc_VectorSwap( dStart, dEnd, 2 ) ;
		}
        DOUBLE dYAt = dStart[1] + m_cParam.m_dYOffset ;
        int nNumPnt = LineIntRotateContours( AllCont, dStart, dEnd , dBuff, 100 ) ;
        if( nNumPnt >= 2  )
        {
           DriveMdl.ZMoveModel( GetDriveSurfZMove() ) ;
           if( pChkMdl ) pChkMdl->ZMoveModel( GetCheckSurfZMove() );
           for( int nAt = 0 ; nAt < nNumPnt ; nAt += 2 ) 
           {
               dStart[0] = dBuff[nAt][0], dEnd[0] = dBuff[nAt+1][0] ;
               dStart[1] = dEnd[1] = m_cParam.m_dYOffset ;
               CSmtCutPath *pNewPath = DriveMdl.CreateLinePath( dStart, dEnd, GetSurfaceTol(), tmpPrg) ;
               while( pNewPath )
               {
                  CSmtCutPath* pTNext = pNewPath->next ;
                  pNewPath->next = pNewPath->prev = NULL ;
                  pNewPath->DelPointOnLine(2.0e-4) ;
                  pNewPath->m_nLineNo = i ;
                  pNewPath->m_nLayerNo = 0 ; 
                  for( CSmtCutPoint* pPnt = pNewPath->m_pHead ; pPnt ; pPnt = pPnt->next )
                  {
                      pPnt->m_fPoint[1] = float( dYAt ) ; 
                  }
                  AllPath.AddToTail( pNewPath ) ;
                  pNewPath = pTNext ; 
               }
           }
           DriveMdl.ZMoveModel( -GetDriveSurfZMove() ) ;
           if( pChkMdl ) pChkMdl->ZMoveModel( -GetCheckSurfZMove() );
        }
		DriveMdl.RotateByXAxis( dRotInc ) ;
       if( pChkMdl ) pChkMdl->RotateByXAxis( dRotInc ) ;
		dAngle += dRotInc ;
	}
	if( fabs(DriveMdl.m_dRotAngle) > 0.0 )
	{
		DriveMdl.RotateByXAxis( -DriveMdl.m_dRotAngle ) ;
        if( pChkMdl ) pChkMdl->RotateByXAxis( -pChkMdl->m_dRotAngle ) ; 
	}
	// STEP 3 : 转换路径生成路径数据 
    DriveMdl.m_bCheckMode = SMART_MODEL_ROTATE ;
    DriveMdl.m_dRotRadius = GetTopHeight() ;
    DriveMdl.UpdateRotateTool( m_pTool ) ;
    DriveMdl.m_dMoveDist[2] = GetDriveSurfZMove() ;
    if( pChkMdl ) 
    {
        pChkMdl->m_bCheckMode = SMART_MODEL_ROTATE ;
        pChkMdl->m_dRotRadius = GetTopHeight() ;
        pChkMdl->UpdateRotateTool( m_pSafeTool ) ;
        pChkMdl->m_dMoveDist[2] = GetCheckSurfZMove() ;
    }
	/*禁止圆弧路径*/
    m_cPrgDef.m_dTotalMove = 30.0 ;
	m_cSetupDef.m_cTolDef.m_nFitType = NCDEF_FITTYPE_LINE ; 
	JDNC_LAYER * pLayer   = GetLayerDef() ;
	pLayer->m_dTotalDepth = GetCutDepth() ; 
	DefineLayerDepth( * pLayer) ; 
	if( pLayer->m_nLayerCount > 1  )
	{ /*分层粗加工*/ 
		LayerAllPath( NewPath, AllPath , DriveMdl, m_cPrgDef) ;
	}
	else
	{
		if( m_cParam.m_bRotate3DFlag & NCDEF_FROTATE3D_ZIGZAG)
		{
            DOUBLE dMaxDist = m_cFeedDef.m_cConnectDef.m_dMaxDist ;
		    AllPath.ConnectPathByLineNo( DriveMdl, dMaxDist,m_cSetupDef.m_cTolDef, m_cPrgDef, TRUE ) ;
		}
        CPathCombine* pPComb =new CPathCombine( NC_WPROCESS_ROUGH ) ;
        AllPath.AddToPathCombine( *pPComb ) ;
        NewPath.AddData( 0.0, pPComb ) ;
	}
	if( pLayer->m_dLayerDepth ) delete[] pLayer->m_dLayerDepth ;
	pLayer->m_dLayerDepth = NULL ;
	return 1 ;
}
int CSmartRotate3DGen::CreateRotatePathEx( CPathGroup& NewPath   , 
									       CSmtCheckMdl& DriveMdl,
                                       CSmartLoop* AllCont   )
{
	// STEP 0 : 计算角度范围
	PNT3D    dStart, dEnd ;
    PNT2D dBuff[100] , dPtArr[10] ;
	DOUBLE dAngle , dPathAngle = 0.0, dRange ;
	PNT2D dMinPt, dMaxPt ;
    CSmtCPathLib  tmpCLib ;
	int nCount, nLineNo = 0, nRet = 0   ;
    DOUBLE dRadius = GetTopHeight() ;
	dAngle = ANGLE_TO_RADIAN( m_cParam.m_dAngle[0] ) ;
	dRange = ANGLE_TO_RADIAN( m_cParam.m_dAngle[1] ) ;
	dPathAngle = ANGLE_TO_RADIAN( m_cParam.m_dPathAngle ) ;
	dMinPt[0] = min( m_cParam.m_dWidth[0], m_cParam.m_dWidth[1])  ;
	dMaxPt[0] = max( m_cParam.m_dWidth[0], m_cParam.m_dWidth[1])  ;
	dMinPt[1] = min( dRadius * dAngle,  dRadius * ( dAngle + dRange ));
	dMaxPt[1] = max( dRadius * dAngle,  dRadius * ( dAngle + dRange ));
    CSmartLoop  tmpLoop ;
    tmpLoop.CreateLoop( dMinPt, dMaxPt ) ;
    tmpLoop.RotateContour( -dPathAngle ) ;
    dStart[2] = dEnd[2] = DriveMdl.m_fBottom ;
	// STEP 2 : 计算平面路径
	CSmtCPathLib  AllPath ;
	double yStep = 1.0 , yAt = 0.0 ; 
	if( dRange > 0.0 ) 
	{
		yAt = tmpLoop.m_dBox[0][1] + 0.01 ;
  	    yStep = m_cParam.m_dOverStep ;
	}
	else
	{
        yAt = tmpLoop.m_dBox[1][1] - 0.01 ;
  	    yStep  = -m_cParam.m_dOverStep ;
	}
	CSmtCutPath* pNewPath ;
	for( yAt ;  ; yAt += yStep, nLineNo ++ ) 
	{
		if( yStep > 0.0 && yAt > tmpLoop.m_dBox[1][1] || 
			yStep < 0.0 && yAt < tmpLoop.m_dBox[0][1]   )
		{
			break ;
		}
		nCount = tmpLoop.GetYLineIntContour( yAt , dPtArr, 10 ) ;
		if( nCount != 2 ) continue ;
		CSmartLine tmpLine( dPtArr[0] , dPtArr[1] ) ;
		if( tmpLine.GetLength() < 0.05 ) continue   ;
		tmpLine.RotateSect( cos( dPathAngle) , sin( dPathAngle ) ) ;
		nc_VectorCopy( dStart , tmpLine.m_aPoint[0], 2 ) ;
		nc_VectorCopy( dEnd   , tmpLine.m_aPoint[1], 2 ) ;
		if( !( m_cParam.m_bRotate3DFlag & NCDEF_FROTATE3D_ZIGZAG) )
		{/*开口线单向走刀*/
			BOOL bRevDir = FALSE ;
			if( GetMillDir() == 1 ) bRevDir = !bRevDir ; /*顺铣*/
			if( yStep < 0.0 ) bRevDir = !bRevDir ; /*反向旋转*/
			if( bRevDir )nc_VectorSwap( dStart, dEnd, 2 ) ;
		}
        int nNumPnt = LineIntRotateContours( AllCont, dStart, dEnd , dBuff , 100 ) ;
       for( int nAt = 0 ; nAt < nNumPnt ; nAt += 2 )
       {
           dStart[0] = dBuff[nAt][0] ;
           dStart[1] = dBuff[nAt][1] + m_cParam.m_dYOffset ;
           dEnd[0]  = dBuff[nAt+1][0] ;
           dEnd[1]   = dBuff[nAt+1][1] + m_cParam.m_dYOffset ;
		    pNewPath = new CSmtCutPath() ;
		    pNewPath->AddPoint( dStart ) ;
		    pNewPath->AddPoint( dEnd   ) ;
		    pNewPath->m_nLineNo = nLineNo ;
           pNewPath->m_nLayerNo = 0 ;
		    pNewPath->DefineBox() ;
		    pNewPath->InsertCPoint( GetSurfaceTol().m_dMaxStep ) ;
		    tmpCLib.AddToTail( pNewPath ) ;
       }
	} 
	// STEP 3 : 计算旋转投影路径
	SurfNC_SetNewStepEx(IDS_PATHGEN_STEP_ORGPATH ) ;
    JDNC_PRGDEF tmpPrg = m_cPrgDef ;
    int nLineTotal = tmpCLib.GetNumPath() , nLineAt = 0 ;
    tmpPrg.m_pPosFunc = NULL  ;
    tmpPrg.m_pPrgFunc = NULL  ; 
    m_cPrgDef.m_dTotalMove = 70.0 ;
    m_cPrgDef.m_dIncStep = m_cPrgDef.m_dStepAt = 0.0 ;
    m_cPrgDef.m_dLimitAt = 1.0 ;
    if( nLineTotal )
    {
        m_cPrgDef.m_dIncStep = m_cPrgDef.m_dTotalMove / nLineTotal ;
    }
	DriveMdl.m_bCheckMode = SMART_MODEL_ROTATE ;
	POSITION pos = tmpCLib.m_cAllPath.GetHeadPosition() ;
	while( pos )
	{
		if( m_cPrgDef.m_pBrkFunc && m_cPrgDef.m_pBrkFunc() )
		{
		    m_nErrorType = JDERROR_GENPATH_ABORT  ;
			return FALSE  ;
		}
        m_cPrgDef.m_dStepAt += m_cPrgDef.m_dIncStep ;
        while( m_cPrgDef.m_pPrgFunc && m_cPrgDef.m_dStepAt >= m_cPrgDef.m_dLimitAt )
        {
            m_cPrgDef.m_pPrgFunc(1) ;
	        m_cPrgDef.m_dStepAt -= m_cPrgDef.m_dLimitAt  ;
        }
		CSmtCutPath* pBasePath = ( CSmtCutPath*) tmpCLib.m_cAllPath.GetNext( pos ) ;
		nRet = pBasePath->VerifyRotatePath(DriveMdl,
		                               GetSurfaceTol(),
		                               tmpPrg ) ;
		if( nRet == 0 )  break ;
        pNewPath = pBasePath->BreakAtNullPoint(FALSE) ;
        while( pNewPath )
        {
           CSmtCutPath* pTNext = pNewPath->next ;
           pNewPath->prev = pNewPath->next = NULL  ;
		   pNewPath->DelPointOnLine(2.0e-4) ;
		   pNewPath->DefineBox() ;
           AllPath.AddToTail( pNewPath ) ;
           pNewPath = pTNext ;
       }
        nLineAt ++ ; 
	}
	if( m_cPrgDef.m_pBrkFunc && m_cPrgDef.m_pBrkFunc() )
	{
		m_nErrorType = JDERROR_GENPATH_ABORT  ;
       return FALSE ;
	}
	if( ! nRet ) return 0;
	/*禁止圆弧路径*/
    m_cPrgDef.m_dTotalMove = 30.0 ;
	m_cSetupDef.m_cTolDef.m_nFitType = NCDEF_FITTYPE_LINE ; 
	JDNC_LAYER * pLayer   = GetLayerDef() ;
	pLayer->m_dTotalDepth = GetCutDepth() ; 
	DefineLayerDepth( * pLayer) ; 
	if( pLayer->m_nLayerCount > 1  )
	{ /*分层粗加工*/ 
		LayerAllPath( NewPath, AllPath , DriveMdl, m_cPrgDef ) ;
	}
	else
	{
        if( m_cParam.m_bRotate3DFlag & NCDEF_FROTATE3D_ZIGZAG )
		{
            DOUBLE dMaxDist = m_cFeedDef.m_cConnectDef.m_dMaxDist ;
            AllPath.ConnectPathByLineNo( DriveMdl, dMaxDist, m_cSetupDef.m_cTolDef,m_cPrgDef, TRUE ) ;
		}
		CPathCombine* pPComb =new CPathCombine( NC_WPROCESS_ROUGH ) ;
	    AllPath.AddToPathCombine( *pPComb ) ;
	    NewPath.AddData( 0.0, pPComb ) ;
	}
	if( pLayer->m_dLayerDepth ) delete[] pLayer->m_dLayerDepth ;
	pLayer->m_dLayerDepth = NULL ;
	return 1 ;
}
int CSmartRotate3DGen::CreateRotatePlunge( CSmtCutPath& TPath  ,
                                            CSmtCutPath& Plunge,
                                            DOUBLE DepthInc )
{
    Plunge.m_bFeedType = JDNC_FEEDTYPE_PLUNGE ;
    if( m_cFeedDef.m_cPlungeDef.m_nPlungeType == NCDEF_PLUNGE_VERT )
    {
        Plunge.AddPoint( TPath.m_pHead->m_fPoint ) ;
        Plunge.AddPoint( TPath.m_pHead->m_fPoint ) ;
        Plunge.m_pHead->m_fPoint[2] += float(DepthInc) ;
    }
    else if( m_cFeedDef.m_cPlungeDef.m_nPlungeType == NCDEF_PLUNGE_RAMP )
    {
        CSmtCutPath tmpPath ;
        CSmtCutPoint* pTPoint, *pTNew ;
        DOUBLE dLenAt = 0.0, dLen ;
        for( pTPoint = TPath.m_pHead; pTPoint; pTPoint = pTPoint->next )
        {
            tmpPath.AddPoint( pTPoint->m_fPoint ) ;
            if( ! pTPoint->next  ) break ;
            dLen = nc_Distance( pTPoint->next->m_fPoint, pTPoint->m_fPoint, 2 ) ;
            if( dLenAt + dLen < m_cFeedDef.m_cPlungeDef.m_dRadius )
            {
                dLenAt += dLen ;
            }
            else
            {
                double u = ( m_cFeedDef.m_cPlungeDef.m_dRadius - dLenAt) / dLen ;
                pTNew  = pTPoint->GenMidPoint( pTPoint->next, float( u ) ) ;
                tmpPath.AddTail( pTNew ) ;
                dLenAt = m_cFeedDef.m_cPlungeDef.m_dRadius ;
                break ;
            }
        }
        DOUBLE dTan   = tan( ANGLE_TO_RADIAN( m_cFeedDef.m_cPlungeDef.m_dAngle ) ) ;
        dLen = 2.0 * dLenAt ;
	    double dInc = dLen * dTan ; 
        int nLoop = (int)ceil( DepthInc / dInc ) ;
        if( nLoop <= 0 ) nLoop = 1 ; 
        else if( nLoop > 30 ) nLoop = 30 ;
        pTNew = new CSmtCutPoint( tmpPath.m_pHead->m_fPoint ) ;
        pTNew->m_fPoint[3] = 0.0 ;
        Plunge.AddTail( pTNew ) ;
        for( int k = 0 ; k < nLoop ; k ++ ) 
        {
            for( int nTime = 0 ; nTime < 2 ; nTime ++ ) 
            {
                for( pTPoint = tmpPath.m_pHead->next ; pTPoint ; pTPoint = pTPoint->next)
                {
                    dLen = nc_Distance( pTPoint->prev->m_fPoint, pTPoint->m_fPoint, 2 ) ;
                    pTNew = new CSmtCutPoint( pTPoint->m_fPoint ) ;
                    pTNew->m_fPoint[3] = float(dLen) + Plunge.m_pTail->m_fPoint[3] ;
                    Plunge.AddTail( pTNew ) ;
                }
                tmpPath.ReverseDirect() ;
            }
        }
        dLenAt = Plunge.m_pTail->m_fPoint[3] ;
        for( pTPoint = Plunge.m_pHead; pTPoint; pTPoint = pTPoint->next )
        {/*添加高度*/
            pTPoint->m_fPoint[2] +=  float( DepthInc * ( 1.0 - pTPoint->m_fPoint[3] / dLenAt));
        }
        nc_VectorCopy( Plunge.m_pTail->m_fPoint,TPath.m_pHead->m_fPoint, 3 ) ;
    }
    Plunge.DefineBox() ;
    if( Plunge.m_nNumPnt < 2 ) return 0 ;
    return 1 ;
}

int CSmartRotate3DGen::AddOneBoundPath( CPathCombine& PComb ,  
                                       CSmtCutPath& TPath   ,
                                       DOUBLE ZMove  )
{
	CPathCombine  tmpTComb( NC_WPROCESS_ROUGH ) ;
    if( m_cFeedDef.m_cLayerDef.m_nLayerType == NCDEF_LAYER_COUNT )
    {
        CSmtCutPath  plungePath ;
        DOUBLE dDepthInc = m_cFeedDef.m_cPlungeDef.m_dTopTol + m_cFeedDef.m_cLayerDef.m_dSideDInc; ;
        if( CreateRotatePlunge( TPath, plungePath, dDepthInc ) )
        {
            plungePath.AddToPathCombine( tmpTComb ) ;
        }
    }
    TPath.AddToPathCombine( tmpTComb ) ;
    tmpTComb.MoveCombine( 0.0, 0.0, ZMove ) ;
    PComb.AppendCombine( tmpTComb ) ;
    return 1 ;
}

int CSmartRotate3DGen::CreateBoundPath( CPathGroup& NewPath   , 
									   CSmtCheckMdl& DriveMdl,
                                   CSmtCPathLib& BndPath   )
{
    if( GetMillDir() == 0 ) 
    {/*逆铣反向*/
        BndPath.ReverseDirect() ;
    }
    DOUBLE dRadius = GetTopHeight() ;
    POSITION atPos = BndPath.m_cAllPath.GetHeadPosition() ;
    while( atPos )
    {
        CSmtCutPath* pTPath = BndPath.m_cAllPath.GetNext( atPos ) ;
        for( CSmtCutPoint* pTPoint = pTPath->m_pHead; pTPoint ; pTPoint = pTPoint->next )
        {/*计算角度*/
           MathCAM_CalcPointAngle( *pTPoint ) ;
           if( pTPoint->prev == NULL ) continue ;
           while( pTPoint->m_fPoint[3] - pTPoint->prev->m_fPoint[3] > MiniPai )
           {
               pTPoint->m_fPoint[3] -= float(MiniPai2) ;
           }
           while( pTPoint->prev->m_fPoint[3] - pTPoint->m_fPoint[3] > MiniPai   )
           {
               pTPoint->m_fPoint[3] += float(MiniPai2) ;
           }
        }
        for( pTPoint = pTPath->m_pHead; pTPoint ; pTPoint = pTPoint->next )
        {/*计算角度*/
            pTPoint->m_fPoint[1] = float(dRadius * pTPoint->m_fPoint[3] + m_cParam.m_dYOffset);
            pTPoint->m_fPoint[2] = (float)GetBottomHeight() ;
        }
    }
    CSmtCPathLib AllPath ;
	SurfNC_SetNewStepEx(IDS_PATHGEN_STEP_ORGPATH ) ;
    JDNC_TOL tmpTol = GetSurfaceTol() ;
    JDNC_PRGDEF tmpPrg = m_cPrgDef ;
    int nLineTotal = BndPath.GetNumPath() ;
    tmpPrg.m_pPosFunc = NULL  ;
    tmpPrg.m_pPrgFunc = NULL  ;
    m_cPrgDef.m_dTotalMove = 100.0 ;
    m_cPrgDef.m_dIncStep = m_cPrgDef.m_dStepAt = 0.0 ;
    m_cPrgDef.m_dLimitAt = 1.0 ;
    if( nLineTotal )
    {
        m_cPrgDef.m_dIncStep = m_cPrgDef.m_dTotalMove / nLineTotal ;
    }
    int nRet = 1 ;
	POSITION pos = BndPath.m_cAllPath.GetHeadPosition() ;
	while( pos )
	{
		if( m_cPrgDef.m_pBrkFunc && m_cPrgDef.m_pBrkFunc() )
		{
		    m_nErrorType = JDERROR_GENPATH_ABORT  ;
			return FALSE  ;
		}
        m_cPrgDef.m_dStepAt += m_cPrgDef.m_dIncStep ;
        while( m_cPrgDef.m_pPrgFunc && m_cPrgDef.m_dStepAt >= m_cPrgDef.m_dLimitAt )
        {
            m_cPrgDef.m_pPrgFunc(1) ;
	        m_cPrgDef.m_dStepAt -= m_cPrgDef.m_dLimitAt  ;
        }
		CSmtCutPath* pBasePath = ( CSmtCutPath*) BndPath.m_cAllPath.GetNext( pos ) ;
        pBasePath->InsertCPoint( tmpTol.m_dMaxStep ) ;
		nRet = pBasePath->VerifyRotatePath(DriveMdl,
		                               GetSurfaceTol(),
		                               tmpPrg ) ;
		if( nRet == 0 )  break ;
        CSmtCutPath* pNewPath = pBasePath->BreakAtNullPoint(FALSE) ;
        while( pNewPath )
        {
            CSmtCutPath* pTNext = pNewPath->next ;
            pNewPath->prev = pNewPath->next = NULL  ;
            pNewPath->DelPointOnLine(2.0e-4) ;
            pNewPath->DefineBox() ;
            AllPath.AddToTail( pNewPath ) ;
            pNewPath = pTNext ;
        }
	}
	if( m_cPrgDef.m_pBrkFunc && m_cPrgDef.m_pBrkFunc() )
	{
		m_nErrorType = JDERROR_GENPATH_ABORT  ;
       return FALSE ;
	}
	if( ! nRet ) return 0;
	m_cSetupDef.m_cTolDef.m_nFitType = NCDEF_FITTYPE_LINE ; /*禁止圆弧路径*/
    CPathCombine* pPComb = new CPathCombine( NC_WPROCESS_ROUGH ) ;
    int nLayerCount = 1 ;
    double dDepthInc = 0.0 ;
    if( m_cFeedDef.m_cLayerDef.m_nLayerType == NCDEF_LAYER_COUNT )
    {
        nLayerCount = m_cFeedDef.m_cLayerDef.m_nLayerCount ;
        dDepthInc = m_cFeedDef.m_cLayerDef.m_dSideDInc     ;
    }
    if( m_cSetupDef.m_cOrderDef.m_nLayerOrd == 1 && nLayerCount>1 )
    {/*高度优先*/
        for( int nLayer = nLayerCount-1  ; nLayer >= 0 ; nLayer --)
        {
            double dZMove = dDepthInc * nLayer ;
            POSITION atPos = AllPath.m_cAllPath.GetHeadPosition() ;
            while( atPos )
            {
                CSmtCutPath* pTPath = AllPath.m_cAllPath.GetNext( atPos ) ;
                AddOneBoundPath( *pPComb, *pTPath, dZMove ) ;            
            }
        }
    }
    else
    {/*区域优先*/
        POSITION atPos = AllPath.m_cAllPath.GetHeadPosition() ;
        while( atPos )
        {
            CSmtCutPath* pTPath = AllPath.m_cAllPath.GetNext( atPos ) ;
            BOOL bIsClosed = pTPath->IsClosed() ;
            if( ( m_cParam.m_bRotate3DFlag & NCDEF_FROTATE3D_ZIGZAG) &&
                nLayerCount % 2 == 0 && (! bIsClosed) )
            {
                pTPath->ReverseDirect() ;
            }
            DOUBLE dTopTol = m_cFeedDef.m_cPlungeDef.m_dTopTol ;
            for( int nLayer = nLayerCount-1  ; nLayer >= 0 ; nLayer --)
            {
                double dZMove = dDepthInc * nLayer ;
                if( ( m_cParam.m_bRotate3DFlag & NCDEF_FROTATE3D_ZIGZAG)&&
                    ( m_cFeedDef.m_cLayerDef.m_bLayerFlag & NCDEF_LAYER_KEEPDOWN ) && 
                    ( nLayer != nLayerCount-1 ) )
                {/*分层不抬刀*/
                    m_cFeedDef.m_cPlungeDef.m_dTopTol = 0.0 ;
                }
                AddOneBoundPath( *pPComb, *pTPath, dZMove ) ;  
                if( ( m_cParam.m_bRotate3DFlag & NCDEF_FROTATE3D_ZIGZAG)&& 
                   (! bIsClosed) )
                {
                    pTPath->ReverseDirect() ;
                }
            }
            m_cFeedDef.m_cPlungeDef.m_dTopTol = dTopTol ;
        }
    }
    NewPath.AddData( 0.0, pPComb ) ;
	return 1 ;
}

int CSmartRotate3DGen::GeneratePathEx( CPathGroup& NewPath ,
								       CSmartGraphic&  Graph )
{ // 生成刀具路径 
	// STEP 1 : 转换刀具
	if( !UpdateTool3D( Graph) ) return 0 ;
    if( !( m_cParam.m_bRotate3DFlag & NCDEF_FROTATE3D_YOFFSET ) )
    {/*设置Y向偏移*/
        m_cParam.m_dYOffset = 0.0 ;
    }
    CSmtCPathLib BoundPath ;
    CSmartLoop*   AllCont = NULL ; 
    if( m_cParam.m_bRotate3DFlag & NCDEF_FROTATE3D_BOUND )
    { /*只加工边界线*/
         ExtractFairedBoundPath( Graph, BoundPath); 
         if( BoundPath.GetNumPath() == 0 )
         {
			m_nErrorType=JDERROR_GENPATH_NOCURVE;
           return FALSE ;
         }
    }
    else
    {
         AllCont = ExtractFairedContours( Graph ) ;
    }
	// STEP 2 : 提取曲面图形 , 并生成干涉检查曲面
	SurfNC_SetNewStepEx(IDS_PATHGEN_STEP_CHECKMDL ) ;
	SurfNC_SetCurPos( 1, 10 ) ;
	CSmtCheckMdl  DriveMdl, CheckMdl ;
    if( ( m_cParam.m_bRotate3DFlag & NCDEF_FROTATE3D_BOUND ) || 
        fabs( m_cParam.m_dPathAngle ) >= 1.0e-3 )
    { /*只加工边界线 || 斜线加工*/
        DriveMdl.m_bCheckMode = SMART_MODEL_ROTATE ;
        CheckMdl.m_bCheckMode = SMART_MODEL_ROTATE ;
    }
	if( ! Graph.BuildCheckModel( m_cSetupDef, DriveMdl, NCDEF_SURF_DRIVE )  ) 
	{ 
		m_nErrorType = JDERROR_GENPATH_NOSURFACE ;
        Mini_DeleteContours( AllCont ) ;
		return 0 ;
	}
    Graph.BuildCheckModel( m_cSetupDef, CheckMdl, NCDEF_SURF_CHECK ) ;
	BOX3D  MaxBox ;
	mathClnBox3D( & MaxBox ) ;
	DriveMdl.DefineDriveSurfBox( MaxBox.min, MaxBox.max ) ;
	MaxBox.min[2] = GetBottomHeight()  ;
	MaxBox.max[2] = GetTopHeight() ;
	DriveMdl.m_dRotAngle  = 0.0 ;
	DriveMdl.m_fBottom = (float)GetBottomHeight()  ;
    if( CheckMdl.m_cAllChk.GetCount() > 0 )
    {
        CheckMdl.m_dRotAngle = 0.0 ;
        CheckMdl.m_fBottom = (float)GetBottomHeight() ;
        DriveMdl.SetCheckMdl( & CheckMdl ) ;
    }
	// STEp 3 : 计算旋转路径
    if( !( m_cParam.m_bRotate3DFlag & NCDEF_FROTATE3D_WIDTH ) )
	{ /*没有定义加工宽度*/
		m_cParam.m_dWidth[0] = MaxBox.min[0] ;
		m_cParam.m_dWidth[1] = MaxBox.max[0]  ;
		m_cParam.m_dAngle[0] = 0.0 ;
		m_cParam.m_dAngle[1] = 360 ;
	}
	NewPath.m_nToolID = m_pTool->m_nToolID ;
    DriveMdl.m_dMoveDist[1] = m_cParam.m_dYOffset ;
    DriveMdl.m_dMoveDist[2] = GetDriveSurfZMove() ;
    CheckMdl.m_dMoveDist[1] = m_cParam.m_dYOffset ;
    CheckMdl.m_dMoveDist[2] = GetCheckSurfZMove() ;
	DriveMdl.m_dRotRadius = GetTopHeight() ;
	CheckMdl.m_dRotRadius = GetTopHeight() ;
    DriveMdl.m_bCheckMode = SMART_MODEL_ROTATE ;
    CheckMdl.m_bCheckMode = SMART_MODEL_ROTATE ;
    DriveMdl.UpdateRotateTool( m_pTool ) ;
    CheckMdl.UpdateRotateTool( m_pSafeTool ) ;

    int nRet = 1 ;
    if( m_cParam.m_bRotate3DFlag & NCDEF_FROTATE3D_BOUND )
    {/*只加工边界线*/
        nRet = CreateBoundPath( NewPath, DriveMdl, BoundPath ) ;
    }
    else if( fabs( m_cParam.m_dPathAngle ) < 1.0e-3 )
    {/*水平加工*/
        DriveMdl.m_bCheckMode = SMART_MODEL_PROJECT ;
        CheckMdl.m_bCheckMode = SMART_MODEL_PROJECT ;
        nRet = CreateRotatePath( NewPath, DriveMdl, AllCont ) ;
    }
    else
    {/*斜线加工*/
        nRet = CreateRotatePathEx( NewPath, DriveMdl, AllCont ) ;
    }
    Mini_DeleteContours( AllCont ) ;
    if( nRet == 0  )	return 0 ; 
    return 1 ;
}
////////////////////////////////////////////////////////////////////////////////////////
//特征槽加工
static  CALLBACK_FACETTOCHECK _callback_convertToCheck = NULL; 
static  CALLBACK_CURVE3DTO2D  _callback_curve3DTo2D = NULL;

void MathCAM_RegConvertToCheckEx( CALLBACK_FACETTOCHECK  func_convert) 								
{
	_callback_convertToCheck = func_convert; 
}
void  MathCAM_RegCurve3DToCurve2D(CALLBACK_CURVE3DTO2D  func_cur3dto2d) 
{
	_callback_curve3DTo2D = func_cur3dto2d ;
}

CSmartCheck* MathCAM_ConvertToCheckEx(FACET& Facet, BOOL ToMix, DOUBLE MaxCos,
									  DOUBLE FltBox[2][3]) 								
{
	if (_callback_convertToCheck == NULL)
	{
		return NULL; 
	}
	return _callback_convertToCheck(Facet, ToMix, MaxCos, FltBox);
}
CSmartCurve*  MathCAM_Curve3DToCurve2D(CGeoCurve* Curve,  
									   JDNC_TOL& Tol   ,
									   RFRAME* NcMtx   ,
									   BOOL Redepth) 
{
	if (_callback_curve3DTo2D == NULL)
	{
		return NULL; 
	}
	return _callback_curve3DTo2D(Curve, Tol, NcMtx, Redepth);
}
CSmartFeatureSlotGen::CSmartFeatureSlotGen()
{
	m_cTmpPrgDef.m_dLimitAt = 0. ;
	m_cTmpPrgDef.m_dStepAt  = 0 ;
	m_cTmpPrgDef.m_dIncStep = 1. ;
	m_cTmpPrgDef.m_pBrkFunc = NULL ;
	m_cTmpPrgDef.m_pPrgFunc = NULL ;
	m_cTmpPrgDef.m_pPosFunc = NULL ;
}
CSmartFeatureSlotGen::~CSmartFeatureSlotGen()
{
}
//修剪最底层路径的起末点
void TrimPathEnd(CSmtCutPath*& pOrgPath, DOUBLE dHeight)
{
	if( !pOrgPath || pOrgPath->m_nNumPnt < 2 )
		return;
	CSmtCutPath* pNewPath = new CSmtCutPath();

	INT_PTR iSt = 0, iEd = 0, i=0 ;
	for(CSmtCutPoint *pHead = pOrgPath->m_pHead; pHead; pHead = pHead->next)
	{
		if(fabs(pHead->m_fPoint[2] - dHeight) < 0.001)
			break;
		++iSt ;
	}
	for(CSmtCutPoint *pTail = pOrgPath->m_pTail; pTail; pTail = pTail->prev)
	{
		if(fabs(pTail->m_fPoint[2] - dHeight) < 0.001)
			break;
		++iEd;
	}
	for(CSmtCutPoint* pCutPnt = pOrgPath->m_pHead; pCutPnt ; pCutPnt = pCutPnt->next)
	{
		i++ ;
		if( (i > iSt)&& (i <= (pOrgPath->m_nNumPnt-iEd)))
		{
			CSmtCutPoint* pNew = pCutPnt->CopyMyself();
			pNewPath->AddTail(pNew);
		}
	}
	pOrgPath->ClearAllPoint();
	delete pOrgPath ;
	pOrgPath = pNewPath ;
}
CSmtCutPointEx *MathCAM_GetMiddlePntEx ( CSmtCutPointEx *pStart, CSmtCutPointEx *pEnd, double u )
{
	if( !pStart || !pEnd ) return NULL ;
	CSmtCutPointEx *pMidd = new CSmtCutPointEx() ;
	pMidd->m_bType = 0 ;
	for( int i = 0 ; i < 3 ; i++ )
	{
		pMidd->m_fPoint[i] = pStart->m_fPoint[i] +TFLOAT( u * ( pEnd->m_fPoint[i] - pStart->m_fPoint[i] ) ) ;
		pMidd->m_fSurfNor[i] = pStart->m_fSurfNor[i] +TFLOAT( u * ( pEnd->m_fSurfNor[i] - pStart->m_fSurfNor[i] ) ) ;
		pMidd->m_fSurfPos[i] = pStart->m_fSurfPos[i] +TFLOAT( u * ( pEnd->m_fSurfPos[i] - pStart->m_fSurfPos[i] ) ) ;
	}
	return pMidd ;
}
BOOL MathCAM_LineIntLine( CSmtCutPath *pPath, CSmtCutPointEx *pStart, CSmtCutPointEx *pEnd, double dZValue )
{
	FPNT3D start, end ;
	mathFCpyPnt( pStart->m_fPoint, start ) ;
	mathFCpyPnt( pEnd->m_fPoint  , end   ) ;

	double t = end[2] - start[2] ;
	if( fabs( t ) < 1.0e-6 )
	{
		pStart->m_bType = SMART_CUTPNT_BREAK ;
	}
	else
	{
		CSmtCutPointEx *pInsert = NULL ;
		t = ( dZValue - start[2] ) / t ;
		if( t < 1.0e-6 )
			pStart->m_bType = SMART_CUTPNT_BREAK ;
		else if( t > 1 - 1.0e-6 )
			pEnd->m_bType = SMART_CUTPNT_BREAK ;
		else
		{
			pInsert = MathCAM_GetMiddlePntEx( pStart, pEnd, t ) ;
			pInsert->m_bType = SMART_CUTPNT_BREAK ;
			pPath->InsertBefore ( pInsert, pEnd ) ;
		}
	}
	return TRUE ;
}

BOOL MathCAM_TrimPathByZValue ( CSmtCutPath *pPath, double dZValue )
{
	if( pPath->m_nNumPnt < 1 ) return FALSE ;
	CSmtCutPointEx *pStart = NULL, *pEnd = NULL ;
	pStart = (CSmtCutPointEx *)pPath->m_pHead ;
	BOOL bInt = FALSE ;
	for( pEnd = (CSmtCutPointEx *)pStart->next ; pEnd ; pEnd = (CSmtCutPointEx *)pEnd->next )
	{
		if( pStart->m_fPoint[2] > dZValue && pEnd->m_fPoint[2] > dZValue ||
			pStart->m_fPoint[2] < dZValue && pEnd->m_fPoint[2] < dZValue )
		{
		}
		else
		{
			if( MathCAM_LineIntLine( pPath, pStart, pEnd, dZValue ) )
				bInt = TRUE ;
		}
		pStart = pEnd ;
	}
	return bInt ;
}
void ClearPntStr(CStrpt* pStrpt)
{
	if( !pStrpt)
		return ;
	if(pStrpt->m_ps)
	{
		delete[] pStrpt->m_ps ;
		pStrpt->m_ps = NULL ;
	}
	if( pStrpt->m_ts )
	{
		delete[] pStrpt->m_ts ;
		pStrpt->m_ts = NULL ;
	}
	pStrpt->Clear();
	delete pStrpt ;
	pStrpt = NULL ;
}
void ClearSurfList(CGeoTrmSurfList& ResultSide)
{
	POSITION pos1 = ResultSide.GetHeadPosition();
	POSITION atPos = NULL ;
	while(pos1)
	{
		atPos = pos1 ;
		CGeoTrmSurf* pSurf = ResultSide.GetNext(pos1);
		if( pSurf )
			delete pSurf;
		ResultSide.RemoveAt(atPos);
		pSurf = NULL ;

	}
	ResultSide.RemoveAll();
}
// 平面曲线转换成刀具路径 
CSmtCutPath* CSmartFeatureSlotGen::TransCurveToCPath( CSmartCurve&  Curve ,JDNC_TOL& Tol ) 
{
	CSmartSect * pSect = Curve.GetHead () ;
	if( ! pSect ) return NULL ; 
	PNT4D  dPoint ;
	CSmtCutPath * pPath = new CSmtCutPath() ;
	dPoint[2] = Curve.m_dDepth , dPoint[3] = 0.0 ;
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
			int nCnt = pArc->Discrete( Tol.m_dArcTol, Buff, 499 ) ;
			for( int i = 1 ; i <= nCnt ; i ++ ) 
			{
				dPoint[0] = Buff[i][0], dPoint[1] = Buff[i][1] ;
				pPath->AddPoint( dPoint ) ;
			}
		}
	}
	return pPath  ; 
}
//投影路径转化为3D折线
CGeoPLine3d* CSmartFeatureSlotGen::ConvertCPathToCur(CSmtCutPath* pCutPath)
{
	if( pCutPath == NULL || pCutPath->NumPoint() < 2 )
	{
		ASSERT(0);
		return NULL ;
	}
	int nCnt = 0 ;
	for(CSmtCutPoint* pCutPnt = pCutPath->m_pHead ; pCutPnt != NULL; pCutPnt = pCutPnt->next)
	{
		++nCnt;
	}
	if ( nCnt < 2 || nCnt != pCutPath->NumPoint() )
	{
		ASSERT(0) ;
		return NULL ;
	}
	CGeoPLine3d* pGeoPLine3D = new CGeoPLine3d(nCnt-1);
	PNT3D pt ;
	nCnt = 0 ;
	for(CSmtCutPoint* pCutPnt = pCutPath->m_pHead; pCutPnt != NULL; pCutPnt = pCutPnt->next)
	{
		nc_FloatToDouble(pt, pCutPnt->m_fPoint, 3) ;
		nc_VectorCopy(pGeoPLine3D->m_dPoint[nCnt], pt, 3);
		++nCnt;
	}
	return pGeoPLine3D;
}
//连接每层路径
void CSmartFeatureSlotGen::ConnectLayerPath(SlotPathList& layerSlotPathList, CSmtCheckMdl* pDriveMdl)
{
	if(layerSlotPathList.IsEmpty())
		return;

	SlotPathList tmpSlotPathList;
	POSITION pos = layerSlotPathList.GetHeadPosition();
	SlotPath* pSlotPath = NULL;
	SlotPath* pNxtSlotPath = NULL; 
	while(pos)
	{
		pSlotPath = layerSlotPathList.GetNext(pos);
		tmpSlotPathList.AddTail(pSlotPath);
		if(pos)
		{
			pNxtSlotPath = layerSlotPathList.GetAt(pos);
			if(pNxtSlotPath != NULL)
			{
				if(pSlotPath->pCutPath && pNxtSlotPath->pCutPath)
				{
					//断点处添加沿曲面连刀路径
					CSmtCutPath* pConPath = new CSmtCutPath ;
					pConPath->m_bFeedType = JDNC_FEEDTYPE_CONNECT ;
					CSmtCutPoint* pCutPnt1 = pSlotPath->pCutPath->m_pTail->CopyMyself();
					CSmtCutPoint* pCutPnt2 = pNxtSlotPath->pCutPath->m_pHead->CopyMyself();
					pConPath->AddTail(pCutPnt1);
					pConPath->AddTail(pCutPnt2);
					pConPath->InsertCPoint(GetSurfaceTol().m_dMaxStep);
					for(CSmtCutPoint* pCutPnt = pConPath->m_pHead ; pCutPnt; pCutPnt = pCutPnt->next)
					{
						pDriveMdl->DefineHeight(*pCutPnt);
					}
					pConPath->AddHead(pSlotPath->pCutPath->m_pTail->CopyMyself());
					pConPath->AddTail(pNxtSlotPath->pCutPath->m_pHead->CopyMyself());
					if(pConPath != NULL)
					{
						pConPath->DelPointOnLine();
						if(pConPath->m_nNumPnt < 2)
						{
							delete pConPath;
							continue;
						}
						SlotPath* pConSlotPath = new SlotPath;
						pConPath->m_nLayerNo = pSlotPath->pCutPath->m_nLayerNo ;
						pConSlotPath->pCutPath = pConPath;
						pConSlotPath->bEClose = TRUE ;
						pConSlotPath->bSClose = TRUE ;
						tmpSlotPathList.AddTail(pConSlotPath);
					}
				}
			}
		}
	}
	layerSlotPathList.RemoveAll();
	layerSlotPathList.AddTail(&tmpSlotPathList);
	tmpSlotPathList.RemoveAll();
}
//裁剪每层路径
void CSmartFeatureSlotGen::TrimOneLayer(PNT3D start, PNT3D end,   /*起末点*/
	                                    CGeoCurve* pTopCurve,     /*顶部曲线*/
										CGeoCurve* pBtmCurve,     /*底部曲线*/
										CSmtCheckMdl* pDriveMdl,  /*检查模型*/
										DOUBLE dTopWidth,        /*槽顶宽*/
										DOUBLE dBtmWidth,        /*槽底宽*/
										int nLayer,             /*当前层数*/   
										SlotPathList& layerSlotPathList)
{
	if( pTopCurve == NULL || pBtmCurve == NULL )
		return  ;
	if( pDriveMdl == NULL )
		return  ;
	
	DOUBLE dArcTol = GetCurveTol().m_dArcTol;
	CGeoInter geoInter;
	CCurIntpt intpt1, intpt2;
	PNT3D slotPntSt, slotPnted ;
	VEC2D vec ;
	nc_VectorCopy(slotPntSt, start, 3);
	nc_VectorCopy(slotPnted, end, 3) ;
	vec[0] = slotPnted[0] - slotPntSt[0];   
	vec[1] = slotPnted[1] - slotPntSt[1];
	nc_Normalize(vec, 2);

	//与顶部曲线求交裁剪每层路径
	if(geoInter.CalcLineCurveAllIntpt(slotPntSt, slotPnted, pTopCurve, MIN_DIS, &intpt1, &intpt2))
	{
		//更新刀具
		JDNC_TOOLEX tmpTool ;
		tmpTool.m_nToolType = surfncToolFlat;
		tmpTool.m_dTopDiam = dTopWidth;  
		CSmartTool* pTool = CreateSmartTool(tmpTool, 0., 0.);
		pDriveMdl->UpdateCurrTool(pTool);
		intpt1.SortByParam();

		CSmtCutPath* pTmpPath = new CSmtCutPath;
		pTmpPath->AddPoint(slotPntSt);
		pTmpPath->AddPoint(slotPnted);
		pTmpPath->VerifyCutPath(*pDriveMdl, GetSurfaceTol(), m_cTmpPrgDef);

		if( start[2] - pTmpPath->m_pHead->m_fPoint[2] > dArcTol )
		{
			slotPntSt[0] = intpt1.m_pHead->pos[0] ;
			slotPntSt[1] = intpt1.m_pHead->pos[1] ;
		}
		if( end[2] - pTmpPath->m_pTail->m_fPoint[2]  > dArcTol )
		{
			slotPnted[0] = intpt1.m_pTail->pos[0] ;
			slotPnted[1] = intpt1.m_pTail->pos[1] ;
		}

		intpt1.Destory() ; 
		intpt2.Destory() ; 
		pTmpPath->ClearAllPoint();
		delete pTmpPath;
		pTmpPath = NULL ;
		if(pTool)
			delete pTool;
		pTool = NULL ;
	}
	
	//与底部曲线求交裁剪每层路径
	if(geoInter.CalcLineCurveAllIntpt(slotPntSt, slotPnted, pBtmCurve, MIN_DIS, &intpt1, &intpt2))
	{
		//更新刀具
		JDNC_TOOLEX tmpTool ;
		tmpTool.m_nToolType = m_pTool->GetType() ;// surfncToolFlat;
		tmpTool.m_dTopDiam = dBtmWidth; 
		tmpTool.m_dBtmDiam = dBtmWidth;
		CSmartTool* pTool = CreateSmartTool(tmpTool, 0., 0.);
		if( !pTool )
			pTool = m_pTool->CopyMyself();
		pDriveMdl->UpdateCurrTool(pTool);
		ASSERT(intpt1.m_pHead);
		intpt1.SortByParam();
		//分段判断每段路径
		tagCCIntpt *ipt1,*ipt2;	
		PNT3D midPt ;
		FPNT3D fMidPt ;
		int j = 0 ; //交点个数
		//首段
		ipt1 = intpt1.GetHead();
		midPt[2] = slotPntSt[2] ;
		if(nc_Distance(slotPntSt, ipt1->pos, 2) > dArcTol)//起点跟第一个交点不重合
		{
			midPt[0] = (slotPntSt[0] + ipt1->pos[0]) / 2.0 ;
			midPt[1] = (slotPntSt[1] + ipt1->pos[1]) / 2.0 ;
			nc_DoubleToFloat(fMidPt, midPt, 3);
			pDriveMdl->DefineHeight(fMidPt);
			if(fMidPt[2] - midPt[2] > dArcTol)
			{
			}
			else
			{
				j++ ;
				SlotPath* pTmpSlotPath = new SlotPath;
				CSmtCutPath* pCutPath = new CSmtCutPath;
				pCutPath->m_nLayerNo = nLayer ;
				pCutPath->AddPoint(slotPntSt);
				pCutPath->AddPoint(ipt1->pos);
				pTmpSlotPath->pCutPath = pCutPath ; 
				pTmpSlotPath->bSClose = FALSE ;
				pTmpSlotPath->bEClose = TRUE ;
				layerSlotPathList.AddTail(pTmpSlotPath);
			}
		}
		//中间段
		INT_PTR iCnt = intpt1.m_nCount -1 ;
		for(int i = 0; i < iCnt; i++)
		{
			ipt1 = intpt1.GetPoint(i);
			ipt2 = intpt1.GetPoint(i+1);
			midPt[0] = (ipt1->pos[0] + ipt2->pos[0])/2.0 ;
			midPt[1] = (ipt1->pos[1] + ipt2->pos[1])/2.0 ;
			nc_DoubleToFloat(fMidPt, midPt, 3);
			pDriveMdl->DefineHeight(fMidPt);
			if(fMidPt[2] - midPt[2] > dArcTol)
			{
			}
			else
			{
				j++;
				SlotPath* pTmpSlotPath = new SlotPath;
				CSmtCutPath* pCutPath = new CSmtCutPath;
				pCutPath->m_nLayerNo = nLayer ;
				pCutPath->AddPoint(ipt1->pos);
				pCutPath->AddPoint(ipt2->pos);
				pTmpSlotPath->pCutPath = pCutPath ; 
				pTmpSlotPath->bSClose = TRUE ;
				pTmpSlotPath->bEClose = TRUE ;
				layerSlotPathList.AddTail(pTmpSlotPath);
			}
		}
		//末段
		ipt2 = intpt1.GetTail();
		if( nc_Distance(slotPnted, ipt2->pos, 2 ) > dArcTol )//末点与最后一个交点不重合
		{
			midPt[0] = (slotPnted[0] + ipt2->pos[0])/ 2.0 ;
			midPt[1] = (slotPnted[1] + ipt2->pos[1])/ 2.0 ;
			nc_DoubleToFloat(fMidPt, midPt, 3);
			nc_DoubleToFloat(fMidPt, midPt, 3);
			pDriveMdl->DefineHeight(fMidPt);
			if(fMidPt[2] - midPt[2] > dArcTol)
			{
			}
			else
			{
				j++ ;
				SlotPath* pTmpSlotPath = new SlotPath;
				CSmtCutPath* pCutPath = new CSmtCutPath;
				pCutPath->m_nLayerNo = nLayer ;
				pCutPath->AddPoint(ipt2->pos);
				pCutPath->AddPoint(slotPnted);
				pTmpSlotPath->pCutPath = pCutPath ; 
				pTmpSlotPath->bSClose = TRUE ;
				pTmpSlotPath->bEClose = FALSE ;
				layerSlotPathList.AddTail(pTmpSlotPath);
			}
		}
		intpt1.Destory() ; 
		intpt2.Destory() ; 

		if( j > 1 && m_cSetupDef.m_cOrderDef.m_nLayerOrd == NCDEF_LAYERORD_HEIGHT)//高度优先时连接每层断开路径
		{
			ConnectLayerPath(layerSlotPathList, pDriveMdl);
		}
		if(pTool)
		{
			delete pTool ;
		}
		pTool = NULL ;
		
	}
	else 
	{	
		SlotPath* pTmpSlotPath = new SlotPath;
		CSmtCutPath* pCutPath = new CSmtCutPath;
		pCutPath->m_nLayerNo = nLayer ;
		pCutPath->AddPoint(slotPntSt);
		pCutPath->AddPoint(slotPnted);
		pTmpSlotPath->bEClose = FALSE;
		pTmpSlotPath->bSClose = FALSE;
		pTmpSlotPath->pCutPath = pCutPath ; 
		layerSlotPathList.AddTail(pTmpSlotPath);
	}
}
//开口端点处延伸路径
void CSmartFeatureSlotGen::ExtendLayerEnd(SlotPathList& slotPathList, CSmtCheckMdl* pDriveMdl,BOOL bIsLine)
{
	if( pDriveMdl == NULL )
		return ;
	INT_PTR nCnt = slotPathList.GetCount();
	if( nCnt < 1)
		return;

	DOUBLE dRa[2];
	dRa[0] = GetCurveTol().m_dArcTol;
	dRa[1] = m_cParam.m_dExtDist ;
	TFLOAT fRa[2] ; //fRa[0] = fArcTol , fRa[1] = fRa;
	nc_DoubleToFloat( fRa, dRa, 2 );
	SlotPath* pSlotPath = NULL ;
	POSITION atPos = NULL ;
	CSmtCutPoint* pCutPnt = NULL ;
	FPNT3D fPnt, fPntExt;
	TFLOAT fVec[2] ;
	POSITION pos = slotPathList.GetHeadPosition();
	while(pos)
	{
		pSlotPath = slotPathList.GetNext(pos);
		if(pSlotPath && pSlotPath->pCutPath)
		{		
			if( bIsLine==TRUE )
			{
				nc_VectorMinus( pSlotPath->pCutPath->m_pHead->m_fPoint,pSlotPath->pCutPath->m_pTail->m_fPoint,fVec, 2 );
			}
			else
			{
				pSlotPath->pCutPath->GetLeadTangent(0, fPnt);
				nc_VectorCopy(fVec, fPnt, 2);
				fVec[0] =- fVec[0];
				fVec[1] =- fVec[1];
			}
			nc_Normalize(fVec, 2);
			if( pSlotPath->bSClose == FALSE )
			{
				nc_VectorCopy( fPnt, pSlotPath->pCutPath->m_pHead->m_fPoint, 3);
				nc_VectorCopy( fPntExt, pSlotPath->pCutPath->m_pHead->m_fPoint, 3); 
				fPntExt[0] -= fVec[0] * fRa[1] ; 
				fPntExt[1] -= fVec[1] * fRa[1] ;
				CSmtCutPath* pExtPath = new CSmtCutPath();
				if(pExtPath)
				{
					pExtPath->AddPoint(fPnt);
					pExtPath->AddPoint(fPntExt);
					pExtPath->InsertCPoint(GetSurfaceTol().m_dMaxStep);
					for(pCutPnt = pExtPath->m_pHead; pCutPnt; pCutPnt = pCutPnt->next)
					{
						pDriveMdl->DefineHeight(pCutPnt->m_fPoint);
						if(pCutPnt->m_fPoint[2]  - pSlotPath->pCutPath->m_pHead->m_fPoint[2] > fRa[0]) 
						{
							break;
						}
						else
						{
							pSlotPath->pCutPath->InsertBefore( pCutPnt->CopyMyself(),pSlotPath->pCutPath->m_pHead);
						}
					}

					pExtPath->ClearAllPoint();
					delete pExtPath ;
				}
				pExtPath = NULL ;
			}
			if( pSlotPath->bEClose == FALSE )
			{
				if(bIsLine == FALSE)
				{
					pSlotPath->pCutPath->GetLeadTangent(1, fPnt);
					nc_VectorCopy(fVec, fPnt, 2);
				}
				nc_VectorCopy( fPnt, pSlotPath->pCutPath->m_pTail->m_fPoint, 3);
				nc_VectorCopy(fPntExt, pSlotPath->pCutPath->m_pTail->m_fPoint, 3 );
				fPntExt[0] += fVec[0] * fRa[1] ; 
				fPntExt[1] += fVec[1] * fRa[1] ;
				CSmtCutPath* pExtPath = new CSmtCutPath();
				if(pExtPath)
				{
					pExtPath->AddPoint(fPnt);
					pExtPath->AddPoint(fPntExt);
					pExtPath->InsertCPoint(GetSurfaceTol().m_dMaxStep);
					for(pCutPnt = pExtPath->m_pHead; pCutPnt; pCutPnt = pCutPnt->next)
					{
						pDriveMdl->DefineHeight(pCutPnt->m_fPoint);
						if(pCutPnt->m_fPoint[2] - pSlotPath->pCutPath->m_pTail->m_fPoint[2] > fRa[0]) 
						{
							break;
						}
						else
						{
							pSlotPath->pCutPath->AddTail(pCutPnt->CopyMyself());
						}
					}
					pExtPath->ClearAllPoint();
					delete pExtPath ;
				}
				pExtPath = NULL ;
			}
			pSlotPath->pCutPath->DelPointOnLine();
		}
	}
	pos = slotPathList.GetHeadPosition();
	while(pos)
	{
		atPos = pos;
		pSlotPath = slotPathList.GetNext(pos);
		//删除短路径
		if( pSlotPath->pCutPath != NULL && pSlotPath->bEClose == TRUE && pSlotPath->bSClose == TRUE )
		{
			if(nc_FabsDist(pSlotPath->pCutPath->m_pHead->m_fPoint,pSlotPath->pCutPath->m_pTail->m_fPoint, 2) < m_pTool->m_fRadius*2)
			{
				pSlotPath->pCutPath->ClearAllPoint();
				delete pSlotPath->pCutPath;
				pSlotPath->pCutPath = NULL;	
				delete pSlotPath;
				pSlotPath = NULL;
				slotPathList.RemoveAt(atPos);
			}
		}
	}
}
BOOL CSmartFeatureSlotGen::AddPathListToPComb(SlotPathList& slotPathList, 
											  int /*nIncNum*/ ,
											  CGeoCurve* pBtmCurve,
											  JDNC_PRGDEF& cPrgDef,
											  double       dCur,
											  CPathCombine& PComb)
{
	INT_PTR nCnt = slotPathList.GetCount();
	if(nCnt < 1)
		return FALSE;
	
	cPrgDef.m_dLimitAt =  nCnt / dCur ;
	cPrgDef.m_dStepAt  = 0.0 ;
	cPrgDef.m_dIncStep = 1.0  ;
	int nPlungeType = m_cFeedDef.m_cPlungeDef.m_nPlungeType ;
	BOOL bStart = TRUE ;
	BOOL bEnd = TRUE ;
	BOOL bRev = FALSE ;
	SlotPath* pSlotPath = NULL; 
	SlotPath* pSlotPathNext = NULL;
	CSmtCutPath* pCutPath = NULL ;
	PNT3D pt, p0, p1; 
	INT_PTR k = 0 ;
	TFLOAT fInc = 0.0f;
	POSITION pos = slotPathList.GetHeadPosition();
	while(pos)
	{
		//////////////计算进度条///////
		if( cPrgDef.m_pBrkFunc && cPrgDef.m_pBrkFunc () )
		{
			return FALSE;
		}
		cPrgDef.m_dStepAt += cPrgDef.m_dIncStep ;
		while( cPrgDef.m_pPrgFunc && cPrgDef.m_dStepAt >= cPrgDef.m_dLimitAt )
		{
			cPrgDef.m_pPrgFunc(1) ;
			cPrgDef.m_dStepAt -= cPrgDef.m_dLimitAt  ;
		}
		//////////////////////////////
		k++ ;
		pSlotPath = (SlotPath*)slotPathList.GetNext(pos);
		if( k == 1 && pSlotPath->bSClose == TRUE )
			bStart= FALSE ;
		if( k == 1 && pSlotPath->bEClose == TRUE)
			bEnd = FALSE ;
		if( pos)
		{
			pSlotPathNext = (SlotPath*)slotPathList.GetAt(pos);
		}
		//根据首层开闭口情况添加首层下刀。
		if( k==1 )
		{
			nc_FloatToDouble(p0, pSlotPath->pCutPath->m_pHead->m_fPoint, 3);
			nc_FloatToDouble(p1, pSlotPath->pCutPath->m_pTail->m_fPoint, 3);
			if(pSlotPathNext != NULL)
				fInc = pSlotPath->pCutPath->m_pHead->m_fPoint[2] - pSlotPathNext->pCutPath->m_pHead->m_fPoint[2];
			if( (bStart == FALSE && bEnd == FALSE) || nPlungeType == NCDEF_PLUNGE_RAMP )
			{
				nc_VectorCopy(pt, p0, 3);
				pt[2] = p0[2] + m_cFeedDef.m_cPlungeDef.m_dTopTol + fInc ;
				AddRampPlungeEx(PComb, p0, p1, pt, NULL, TRUE);
				bStart = TRUE ;
			}
			else if( bStart == TRUE)
			{
				nc_VectorCopy(pt, p0, 3);
				pt[2] = p0[2] + m_cFeedDef.m_cPlungeDef.m_dTopTol + fInc ;
				CSmtCutPath* pPlg = new CSmtCutPath;
				pPlg->m_bFeedType = JDNC_FEEDTYPE_PLUNGE;
				pPlg->AddPoint(pt);
				pPlg->AddPoint(p0);
				pPlg->AddToPathCombine(PComb);
				pPlg->ClearAllPoint();
				delete pPlg;
				pPlg = NULL ;
			}
			else
			{
				nc_VectorCopy(pt, p1, 3);
				pt[2] = p1[2] + m_cFeedDef.m_cPlungeDef.m_dTopTol + fInc ;
				CSmtCutPath* pPlg = new CSmtCutPath;
				pPlg->m_bFeedType = JDNC_FEEDTYPE_PLUNGE;
				pPlg->AddPoint(pt);
				pPlg->AddPoint(p1);
				pPlg->AddToPathCombine(PComb);
				pPlg->ClearAllPoint();
				delete pPlg;
				pPlg = NULL ;
			}
		}
		if(m_cParam.m_bZIGZAG==TRUE)
		{
			if(bStart == TRUE)
			{
				if( k%2 )
				{
					bRev = FALSE;
				}
				else
				{
					bRev = TRUE ;
				}	
			}
			else
			{
				if( k%2 )
				{
					bRev = TRUE ;
				}
				else
				{
					bRev = FALSE ;
				}
			}
			//添加上层路径
			if( pSlotPath->pCutPath != NULL )
			{
				pCutPath = pSlotPath->pCutPath->CopyMyself();
				if(bRev)
				{
					pCutPath->ReverseDirect();
				}
				pCutPath->AddToPathCombine(PComb);
				//添加层间下刀
				if(pSlotPathNext)
				{
					PlungeBtwnLayer(pSlotPath, pSlotPathNext, pBtmCurve,bRev, PComb);		
				}
				pCutPath->ClearAllPoint();
				delete pCutPath ;
			}
			pSlotPathNext = NULL ;
		}
		else
		{
			pCutPath = pSlotPath->pCutPath->CopyMyself();
			if(bStart == FALSE)
			{
				pCutPath->ReverseDirect();
			}
			if(k > 1)
			{
				if( (bStart == FALSE && pSlotPath->bEClose==TRUE) ||
				    (bStart == TRUE  && pSlotPath->bSClose==TRUE) ||
				    nPlungeType == NCDEF_PLUNGE_RAMP )
				{
					nc_FloatToDouble(p0, pCutPath->m_pHead->m_fPoint, 3);
					nc_FloatToDouble(p1, pCutPath->m_pTail->m_fPoint, 3);
					nc_VectorCopy(pt, p0, 3);
					pt[2] = p0[2] + fInc ;
					AddRampPlungeEx(PComb, p0, p1, pt, NULL, TRUE);
				}
				else
				{
					nc_FloatToDouble(p0, pCutPath->m_pHead->m_fPoint, 3);
					nc_VectorCopy(pt, p0, 3);
					pt[2] = p0[2] + fInc ;
					CSmtCutPath* pPlg = new CSmtCutPath;
					pPlg->m_bFeedType = JDNC_FEEDTYPE_PLUNGE;
					pPlg->AddPoint(pt);
					pPlg->AddPoint(p0);
					pPlg->AddToPathCombine(PComb);
					pPlg->ClearAllPoint();
					delete pPlg;
					pPlg = NULL ;
				}
			}
			pCutPath->AddToPathCombine(PComb);
			pCutPath->ClearAllPoint();
			delete pCutPath ;
		}
	}
	slotPathList.RemoveAll() ;
	return TRUE ;
}
void CSmartFeatureSlotGen::AreaSortPathList( SlotPathList& allSlotPathList, 
											int nLayerCnt, 
											SlotPathList& areaSortList )
{
	if(allSlotPathList.IsEmpty())
		return ;
	SlotPathList layerListk;
	SlotPath* pSlotPath ;
	POSITION pos, atPos ;
	SlotPathList* pTmpPathGrp = new SlotPathList[nLayerCnt];
	int k = 1 ;
	//把原始路径按层分组
	for( k = 1; k <= nLayerCnt; k++ )
	{
		layerListk.RemoveAll();
		SearchOneLayerList( allSlotPathList, k, layerListk );
		pTmpPathGrp[k-1].AddTail(&layerListk);
	}

	for( k = 1; k <= nLayerCnt; k++ )
	{
		while(pTmpPathGrp[k-1].GetCount())
		{
			pos = pTmpPathGrp[k-1].GetHeadPosition();
			while(pos)
			{
				atPos = pos;
				pSlotPath = pTmpPathGrp[k-1].GetNext(pos);
				if( pSlotPath && pSlotPath->pCutPath )
				{
					// 从这一层中删除该条路径
					pTmpPathGrp[k-1].RemoveAt(atPos);
					GenAreaSlotPath( pTmpPathGrp, pSlotPath, nLayerCnt, areaSortList);
				}
				break ;
			}
		}
	}
	if(pTmpPathGrp)
	{
		delete[] pTmpPathGrp;
	}
	pTmpPathGrp = NULL ;
}
int CSmartFeatureSlotGen::GenAreaSlotPath( SlotPathList* pSlotPathGrp,
										  SlotPath* preSlotPath,  
										  int nLayerCnt ,
										  SlotPathList& areaSortList )
{
	if( preSlotPath == NULL )
		return 0 ;
	if( preSlotPath->pCutPath == NULL )
		return 0 ;
	/*if( m_cPrgDef.m_pBrkFunc && m_cPrgDef.m_pBrkFunc () )
	{
		return 2 ;
	}*/
	if(preSlotPath->pCutPath->m_nLayerNo >= nLayerCnt)
	{
		areaSortList.AddTail(preSlotPath);
		return 0;
	}
	// 从下一层中找出与preSlotPath在同一区域中的路径
	SlotPathList CurPathList;
	GetAreaPathInNextLayer( pSlotPathGrp, preSlotPath, CurPathList ) ;
	//加工上一层路径preSlotPath
	areaSortList.AddTail(preSlotPath);
	if(CurPathList.GetCount() < 1)
		return 0 ;
	/*if( m_cPrgDef.m_pBrkFunc && m_cPrgDef.m_pBrkFunc () )
	{
		return 2 ;
	}*/
	SlotPath* pSlotPath = NULL ;
	POSITION pos, atPos ;
	while(CurPathList.GetCount())
	{
		pos = CurPathList.GetHeadPosition();
		while(pos)
		{
			atPos = pos ;
			pSlotPath = CurPathList.GetNext(pos);
			//从当前层删除
			CurPathList.RemoveAt(atPos);
			GenAreaSlotPath ( pSlotPathGrp, pSlotPath, nLayerCnt, areaSortList) ;
			break ;
		}
	}
	return 1 ;
}

void CSmartFeatureSlotGen::GetAreaPathInNextLayer(SlotPathList* pSlotPathGrp, 
												  SlotPath* preSlotPath, 
												  SlotPathList& CurPathList ) 
{
	if(pSlotPathGrp== NULL)
		return ;
	if( preSlotPath == NULL || preSlotPath->pCutPath == NULL )
		return ;     
	DOUBLE dArcTol = GetCurveTol().m_dArcTol ;
	CSmtCutPath* pPrePath = preSlotPath->pCutPath ;
	INT_PTR k = pPrePath->m_nLayerNo ;
	POSITION pos = pSlotPathGrp[k].GetHeadPosition();
	POSITION atPos = NULL ;
	SlotPath* pSlotPath = NULL ;
	DOUBLE dPreMin0, dPreMin1, dPreMax0, dPreMax1 ;
	dPreMin0 = min(pPrePath->m_pHead->m_fPoint[0], pPrePath->m_pTail->m_fPoint[0]);
	dPreMin1 = min(pPrePath->m_pHead->m_fPoint[1], pPrePath->m_pTail->m_fPoint[1]);
	dPreMax0 = max(pPrePath->m_pHead->m_fPoint[0], pPrePath->m_pTail->m_fPoint[0]);
	dPreMax1 = max(pPrePath->m_pHead->m_fPoint[1], pPrePath->m_pTail->m_fPoint[1]);
	pPrePath->DefineBox();
	BOX2D box;
	box.max[0] = pPrePath->m_fBox[1][0]; //dPreMax0;
	box.max[1] = pPrePath->m_fBox[1][1]; //dPreMax1;
	box.min[0] = pPrePath->m_fBox[0][0]; //dPreMin0;
	box.min[1] = pPrePath->m_fBox[0][1]; //dPreMin1;
	mathExpandBox2D(GetCurveTol().m_dArcTol, &box);
	PNT2D pnt0, pnt1;
	while(pos)
	{
		atPos = pos ;
		pSlotPath = pSlotPathGrp[k].GetNext(pos);
		if(pSlotPath->pCutPath)
		{
			CSmtCutPath* pCurPath = pSlotPath->pCutPath ;
			pCurPath->DefineBox();
			pnt0[0] = pCurPath->m_fBox[0][0] ; //min(pCurPath->m_pHead->m_fPoint[0], pCurPath->m_pTail->m_fPoint[0]); 
			pnt0[1] = pCurPath->m_fBox[0][1] ; //min(pCurPath->m_pHead->m_fPoint[1], pCurPath->m_pTail->m_fPoint[1]); 
			pnt1[0] = pCurPath->m_fBox[1][0] ; //max(pCurPath->m_pHead->m_fPoint[0], pCurPath->m_pTail->m_fPoint[0]); 
			pnt1[1] = pCurPath->m_fBox[1][1] ; //max(pCurPath->m_pHead->m_fPoint[1], pCurPath->m_pTail->m_fPoint[1]); 
			if(mathChkBox2DPnt(&box, pnt0, dArcTol) || mathChkBox2DPnt(&box, pnt1, dArcTol))
			{
				pSlotPathGrp[k].RemoveAt(atPos);
				CurPathList.AddTail(pSlotPath);
			}
		}
	}
}

BOOL CSmartFeatureSlotGen::AddPathListToPCombEx(SlotPathList& allSlotPathList, 
												int nLayerCnt,
												CGeoCurve* pBtmCurve, 
												JDNC_PRGDEF& cPrgDef,
												double       dCur,
												CPathCombine& PComb)
{
	if(allSlotPathList.IsEmpty())
		return FALSE;
	SlotPath* pHead = NULL ;
	SlotPath* pTail = NULL ;
	BOOL bStart = TRUE ;
	BOOL bEnd = TRUE ;
	BOOL bRev = FALSE ;
	TFLOAT fInc = 5.0f ;//下刀深度
	int nPlungeType = m_cFeedDef.m_cPlungeDef.m_nPlungeType ;
	PNT3D pt, p0, p1; 
	SlotPathList layerList1, layerList2 ;//存放相邻两层路径
	cPrgDef.m_dLimitAt =  nLayerCnt / dCur ;
	cPrgDef.m_dStepAt  = 0.0 ;
	cPrgDef.m_dIncStep = 1.0  ;

	for(int k = 1; k < nLayerCnt; k++ )
	{
		//////////////计算进度条///////
		if( cPrgDef.m_pBrkFunc && cPrgDef.m_pBrkFunc () )
		{
			return FALSE;
		}
		cPrgDef.m_dStepAt += cPrgDef.m_dIncStep ;
		while( cPrgDef.m_pPrgFunc && cPrgDef.m_dStepAt >= cPrgDef.m_dLimitAt )
		{
			cPrgDef.m_pPrgFunc(1) ;
			cPrgDef.m_dStepAt -= cPrgDef.m_dLimitAt  ;
		}
		//////////////////////////////
		//找到相邻两层路径
		SearchOneLayerList( allSlotPathList, k, layerList1 );
		SearchOneLayerList( allSlotPathList, k+1, layerList2 );
		if(k==1)//首层下刀
		{
			pHead = layerList1.GetHead();
			pTail = layerList1.GetTail();
			SlotPath* pTmpPath = layerList2.GetHead();
			if( pTmpPath && pTmpPath->pCutPath)
			{
				fInc = pHead->pCutPath->m_pHead->m_fPoint[2] - pTmpPath->pCutPath->m_pHead->m_fPoint[2] ;
			}
			if(pHead->bSClose == TRUE)
				bStart = FALSE;
			if(pTail->bEClose == TRUE)
				bEnd = FALSE ;	
			nc_FloatToDouble(p0, pHead->pCutPath->m_pHead->m_fPoint, 3);
			nc_FloatToDouble(p1, pTail->pCutPath->m_pTail->m_fPoint, 3);
			if((bStart == FALSE && bEnd == FALSE) || nPlungeType == NCDEF_PLUNGE_RAMP)//两端闭口或折线下刀
			{
				nc_VectorCopy(pt, p0, 3);
				pt[2] = p0[2] + m_cFeedDef.m_cPlungeDef.m_dTopTol + fInc ;
				AddRampPlungeEx(PComb, p0, p1, pt, NULL, TRUE);
				bStart = TRUE ;
			}
			else if( bStart == TRUE)
			{
				nc_VectorCopy(pt, p0, 3);
				pt[2] = p0[2] + m_cFeedDef.m_cPlungeDef.m_dTopTol + fInc ;
				CSmtCutPath* pPlg = new CSmtCutPath;
				pPlg->m_bFeedType = JDNC_FEEDTYPE_PLUNGE;
				pPlg->AddPoint(pt);
				pPlg->AddPoint(p0);
				pPlg->AddToPathCombine(PComb);
				pPlg->ClearAllPoint();
				delete pPlg;
				pPlg = NULL ;
			}
			else
			{
				nc_VectorCopy(pt, p1, 3);
				pt[2] = p1[2] + m_cFeedDef.m_cPlungeDef.m_dTopTol + fInc ;
				CSmtCutPath* pPlg = new CSmtCutPath;
				pPlg->m_bFeedType = JDNC_FEEDTYPE_PLUNGE;
				pPlg->AddPoint(pt);
				pPlg->AddPoint(p1);
				pPlg->AddToPathCombine(PComb);
				pPlg->ClearAllPoint();
				delete pPlg;
				pPlg = NULL ;
			}
		}
		if(m_cParam.m_bZIGZAG == TRUE)
		{
			if(bStart == TRUE)
			{
				if( k%2 )
				{
					bRev = FALSE;
				}
				else
				{
					bRev = TRUE ;
				}	
			}
			else
			{
				if( k%2 )
				{
					bRev = TRUE ;
				}
				else
				{
					bRev = FALSE ;
				}
			}
			//添加上层路径
			AddLayerPathToPComb(layerList1, PComb, bRev);
			//添加层间下刀
			PlungeBtwnLayerEx(layerList1, layerList2, pBtmCurve, bRev, PComb);	
			if( k == nLayerCnt - 1 )
			{
				AddLayerPathToPComb(layerList2, PComb, !bRev) ;
			}
		}
		else
		{
			pHead = layerList1.GetHead();
			pTail = layerList1.GetTail();
			nc_FloatToDouble(p0, pHead->pCutPath->m_pHead->m_fPoint, 3);
			nc_FloatToDouble(p1, pTail->pCutPath->m_pTail->m_fPoint, 3);
			if(bStart == TRUE)
			{
				if(k>1)
				{
					nc_VectorCopy(pt, p0, 3);
					pt[2] = p0[2] + fInc ;
					if(pHead->bSClose == TRUE)
						AddRampPlungeEx(PComb, p0, p1, pt, pBtmCurve, TRUE);
				}
				AddLayerPathToPComb(layerList1, PComb, FALSE);
				if( k == nLayerCnt - 1 )
				{
					AddLayerPathToPComb(layerList2, PComb, FALSE) ;
				}
			}
			else
			{
				if(k>1)
				{
					nc_VectorCopy(pt, p1, 3);
					pt[2] = p1[2] + fInc ;
					if(pTail->bEClose == TRUE)
						AddRampPlungeEx(PComb, p1, p0, pt, pBtmCurve, TRUE);
				}
				AddLayerPathToPComb(layerList1, PComb, TRUE);
				if( k == nLayerCnt - 1 )
				{
					AddLayerPathToPComb(layerList2, PComb, TRUE) ;
				}
			}
		}
		layerList1.RemoveAll();
		layerList2.RemoveAll();
	}
	return TRUE ;
}
void CSmartFeatureSlotGen::AddLayerPathToPComb(SlotPathList& layerList, CPathCombine& PComb, BOOL bRev)
{
	if(layerList.IsEmpty())
		return;
	POSITION pos = layerList.GetHeadPosition();
	SlotPath* pSlotPath = NULL ;
	CSmtCutPath* pCutPath = NULL ;
	if( !bRev)
	{
		while(pos)
		{
			pSlotPath = layerList.GetNext(pos);
			if(pSlotPath && pSlotPath->pCutPath)
			{
				pSlotPath->pCutPath->AddToPathCombine(PComb);	
			}
		}
	}
	else
	{
		pos = layerList.GetTailPosition();
		while(pos)
		{
			pSlotPath = layerList.GetPrev(pos);
			if(pSlotPath && pSlotPath->pCutPath)
			{
				pCutPath = pSlotPath->pCutPath->CopyMyself();
				pCutPath->ReverseDirect();
				pCutPath->AddToPathCombine(PComb);
				pCutPath->ClearAllPoint();
				delete pCutPath ;
				pCutPath = NULL ;		
			}
		}
	}

}
void CSmartFeatureSlotGen::SearchOneLayerList(SlotPathList& allPathList, int nLayer, SlotPathList& curLayerList)
{
	if(allPathList.IsEmpty())
		return;
	SlotPath* pSlotPath = NULL;
	POSITION pos = allPathList.GetHeadPosition() ;
	while(pos)
	{
		pSlotPath = allPathList.GetNext(pos);
		if(pSlotPath && pSlotPath->pCutPath)
		{
			if(pSlotPath->pCutPath->m_nLayerNo == nLayer)
				curLayerList.AddTail(pSlotPath);
			else if(pSlotPath->pCutPath->m_nLayerNo > nLayer)
				break ;
		}
	}
}
void ClearTmpData(CSmtCutPath* pTopPath, CSmtCutPath* pBtmPath, CSmartTool* pTool)
{
	if(pTopPath)
	{
		pTopPath->ClearAllPoint();
		delete pTopPath;
	}
	if(pBtmPath)
	{
		pBtmPath->ClearAllPoint();
		delete pBtmPath ;
	}
	if(pTool)
		delete pTool;
	pTool = NULL ;
	pBtmPath = NULL ;
	pTopPath = NULL ;
}
BOOL CSmartFeatureSlotGen::OneSlotPath( CPathCombine& PComb,     /*单个槽路径*/
									   CSmtCheckMdl* pDriveMdl, /*检查模型*/
									   CSmartCurve* pMidCur,   /*槽壁两边界中位线*/
									   BOOL   bIsLine,        /*中位线直线标记*/
									   DOUBLE dTopWidth,       /*槽顶宽*/
									   DOUBLE dBtmWidth,     /*槽底宽*/
									   JDNC_PRGDEF& cPrgDef) 
{
	if( pDriveMdl == NULL ||  pMidCur == NULL)
		return FALSE;

	int k = 0 ;
	FPNT3D minp, maxp ;
	pDriveMdl->CalcFacetBox ( minp, maxp ) ;
	pDriveMdl->m_fBottom = (TFLOAT)minp[2]  ;
	pDriveMdl->m_bCheckMode = SMART_MODEL_PROJECT ;
	//Step1:得到顶部路径
	CSmtCutPath* pTopPath =  TransCurveToCPath(*pMidCur, m_cSetupDef.m_cTolDef);
	CSmtCutPath* pBtmPath = pTopPath->CopyMyself();
	for(CSmtCutPoint* pCutPnt = pBtmPath->m_pHead; pCutPnt; pCutPnt = pCutPnt->next)
	{
		pCutPnt->m_fPoint[2] = pDriveMdl->m_fBottom ;
	}
	//1.1创建刀具
	JDNC_TOOLEX tmpTool ;
	tmpTool.m_nToolType = surfncToolFlat;
	tmpTool.m_dTopDiam = dTopWidth;  
	CSmartTool* pTool = CreateSmartTool(tmpTool, 0., 0.);
	pDriveMdl->UpdateCurrTool(pTool);
	//1.2投影路径
	pTopPath->InsertCPoint(GetSurfaceTol().m_dMaxStep);
	pTopPath->VerifyCutPath(*pDriveMdl, GetSurfaceTol(), m_cTmpPrgDef);
	//Step2:得到底部路径
	//2.1创建刀具
	if( pTool )
		delete pTool;
	tmpTool.m_dTopDiam = dBtmWidth; 
	tmpTool.m_dBtmDiam = dBtmWidth;
	tmpTool.m_nToolType = m_pTool->GetType();
	pTool = CreateSmartTool(tmpTool, 0., 0.);
	if(!pTool)
		pTool = m_pTool->CopyMyself() ;
	pDriveMdl->UpdateCurrTool(pTool);
	//2.2投影路径
	pBtmPath->InsertCPoint(GetSurfaceTol().m_dMaxStep);
	pBtmPath->VerifyCutPath(*pDriveMdl, GetSurfaceTol(), m_cTmpPrgDef);
	pTopPath->DelPointOnLine();
	pBtmPath->DelPointOnLine();
	//修剪下掉路径
	MathCAM_TrimPathByZValue(pBtmPath, pDriveMdl->m_fBottom);
	if( !pBtmPath || pBtmPath->NumPoint() < 2 )
	{
		ClearTmpData(pTopPath, NULL, pTool) ;
		return FALSE ;
	}
	
	//step3:布Z向分层路径
	BOX3D topBox, btmBox ;
	//3.1路径转为曲线
	CGeoCurve* pTopCurve = ConvertCPathToCur(pTopPath);
	CGeoCurve* pBtmCurve = ConvertCPathToCur(pBtmPath);
	pTopCurve->UpdateBox(&topBox);
	pBtmCurve->UpdateBox(&btmBox);

	DOUBLE dDepthInc = m_cFeedDef.m_cLayerDef.m_dSideDInc ;
	int nIncNum =(int)(ceil((topBox.max[2]- btmBox.min[2])/dDepthInc));
	dDepthInc = (topBox.max[2]-btmBox.min[2])/nIncNum ; //重算分层深度;
	//3.2裁剪分层路径
	SlotPathList  allSlotPathList ;
	PNT3D start1, end1 ;//投影线的最大范围
	pTopCurve->GetEndPoint(start1, end1);
	//定义进度条
	double dCur[2] = { 80, 20 } ;
	cPrgDef.m_dLimitAt =  nIncNum / dCur[0] ;
	cPrgDef.m_dStepAt  = 0.0 ;
	cPrgDef.m_dIncStep = 1.0  ;
	BOOL bBreak = FALSE ;
	for( k = 1 ; k <= nIncNum; k++ )
	{
		//////////////计算进度条///////
		if( cPrgDef.m_pBrkFunc && cPrgDef.m_pBrkFunc () )
		{
			bBreak = TRUE;
			break;
		}
		cPrgDef.m_dStepAt += cPrgDef.m_dIncStep ;
		while( cPrgDef.m_pPrgFunc && cPrgDef.m_dStepAt >= cPrgDef.m_dLimitAt )
		{
			cPrgDef.m_pPrgFunc(1) ;
			cPrgDef.m_dStepAt -= cPrgDef.m_dLimitAt  ;
		}
		//////////////////////////////
		start1[2] = topBox.max[2] - dDepthInc * k ;
		end1[2] =  topBox.max[2] - dDepthInc * k ;
		if(k == nIncNum)
		{
			start1[2] = start1[2] + 0.001 ;
			end1[2] = end1[2] + 0.001 ;
		}
		SlotPathList  layerSlotPathList ;
		TrimOneLayer(start1, end1, pTopCurve, pBtmCurve,pDriveMdl, dTopWidth, dBtmWidth, k, layerSlotPathList);
		allSlotPathList.AddTail(&layerSlotPathList);
		layerSlotPathList.RemoveAll();
	}
	if(bBreak)//用户中断
	{
		ClearTmpData(pTopPath, pBtmPath, pTool) ;
		ClearSlotPathList(allSlotPathList);
		if(pTopCurve)
		{
			pTopCurve->Destroy();
			delete pTopCurve;
		}
		if( pBtmCurve )
		{
			pBtmCurve->Destroy();
			delete pBtmCurve;
		}
		pTopCurve =NULL ;
		pBtmCurve = NULL ;
		return FALSE;
	}
	if(pTopCurve)
	{
		pTopCurve->Destroy();
		delete pTopCurve;
	}
	pTopCurve =NULL ;

	//3.3添加最后一层路径
/*	INT_PTR nCnt = allSlotPathList.GetCount();
	if( nCnt > 1)
	{
		SlotPath* pPreLastPath = (SlotPath*)allSlotPathList.GetTail();//得到最后一层路径的开闭口标记
		CSmtCutPath* pCutPath = pBtmPath->CopyMyself();
		if( pCutPath != NULL )
		{
			pCutPath->InsertCPoint(GetSurfaceTol().m_dMaxStep);
			TrimPathEnd(pCutPath, btmBox.min[2]);
			if(pCutPath->m_nNumPnt >=2 )
			{
				pCutPath->DelPointOnLine();
				//保证底层路径跟上层路径同向
				VEC2D v1, v2;
				v1[0] = pPreLastPath->pCutPath->m_pTail->m_fPoint[0] - pPreLastPath->pCutPath->m_pHead->m_fPoint[0];
				v1[1] = pPreLastPath->pCutPath->m_pTail->m_fPoint[1] - pPreLastPath->pCutPath->m_pHead->m_fPoint[1];
				v2[0] = pCutPath->m_pTail->m_fPoint[0] - pCutPath->m_pHead->m_fPoint[0];
				v2[1] = pCutPath->m_pTail->m_fPoint[1] - pCutPath->m_pHead->m_fPoint[1];
				if(mathGetAngle2D(v1, v2, MIN_DIS) > PI1_2)
					pCutPath->ReverseDirect();
				//
				SlotPath* pLastSlotPath = new SlotPath ;
				pLastSlotPath->pCutPath = pCutPath ;
				pLastSlotPath->pCutPath->m_nLayerNo = nIncNum ;
				pLastSlotPath->bEClose = pPreLastPath->bEClose ;
				pLastSlotPath->bSClose = pPreLastPath->bSClose ;
				allSlotPathList.AddTail(pLastSlotPath);
			}
			else
			{
				pCutPath->ClearAllPoint();
				delete pCutPath ;
				pCutPath = NULL ;
			}
		}
	}*/

	pDriveMdl->UpdateCurrTool(pTool);
	//3.4开口端点处延伸路径
	if(m_cParam.m_bExtEnd == TRUE)
	{
		ExtendLayerEnd(allSlotPathList, pDriveMdl, bIsLine);
	}
	//step4:allSlotPathList添加到PathCombine中
	if(m_cSetupDef.m_cOrderDef.m_nLayerOrd == NCDEF_LAYERORD_HEIGHT)
	{
		if(allSlotPathList.GetCount() > 0 && !AddPathListToPCombEx(allSlotPathList, nIncNum, pBtmCurve, cPrgDef, dCur[1], PComb))
		{
			ClearSlotPathList(allSlotPathList);
			ClearTmpData(NULL, pBtmPath, NULL);
			if( pBtmCurve )
			{
				pBtmCurve->Destroy();
				delete pBtmCurve;
			}
			pBtmCurve = NULL ;	
			return FALSE ;
		}
	}
	else if(m_cSetupDef.m_cOrderDef.m_nLayerOrd == NCDEF_LAYERORD_REGION)
	{//区域排序
		SlotPathList slotPathList;
		AreaSortPathList(allSlotPathList, nIncNum, slotPathList);
		if(slotPathList.GetCount() > 0 && !AddPathListToPComb(slotPathList, nIncNum, pBtmCurve, cPrgDef, dCur[1], PComb))
		{
			ClearSlotPathList(allSlotPathList);
			ClearTmpData(NULL, pBtmPath, NULL);
			if( pBtmCurve )
			{
				pBtmCurve->Destroy();
				delete pBtmCurve;
			}
			pBtmCurve = NULL ;	
			return FALSE ;
		}
	}
	else
	{
		ASSERT(0) ;
	}
	if(m_cParam.m_nFinishPath == 1)
	{
		pBtmPath->AddToPathCombine(PComb);
	}
	
	//清除临时数据
	ClearSlotPathList(allSlotPathList);
	ClearTmpData(pTopPath, pBtmPath, pTool);
	if( pBtmCurve )
	{
		pBtmCurve->Destroy();
		delete pBtmCurve;
	}
	pBtmCurve = NULL ;	
	pBtmPath = NULL ;
	return TRUE;
}
void DeleteSSICurve( CSSICurve*&  intCur )
{
	while( intCur) 
	{
		CSSICurve* pCur = intCur->m_next ;
		delete intCur ;
		intCur = pCur ;
	}
}

void CSmartFeatureSlotGen::SortSideList(CEntList& inList) //侧壁面组排序
{
	if(inList.IsEmpty())
		return ;
	CEntity3D* pEnt = NULL ;
	CEttSurface* pEntSurf = NULL;
	CGeoTrmSurf* pTrmSurf = NULL ;
	CPointList sortList ;
	CSmartSPoint* pPoint ;
	PNT3D pnt ;
	CEntList tmpList ;
	POSITION pos = inList.GetHeadPosition();
	POSITION atPos = NULL ;
	while(pos)
	{
		atPos = pos ;
		pEnt = inList.GetNext(pos);
		if( pEnt->GetType() != ETTYPE_SURFACE)
			continue;
		pEntSurf = (CEttSurface*)pEnt;
		pTrmSurf = pEntSurf->GetTrmSurf(); 
		pTrmSurf->GetPoint(0.5, 0.5, pnt);
		pPoint = new CSmartSPoint(pnt, pEntSurf);
		sortList.InsertAfter(pPoint, sortList.m_pTail);
		inList.RemoveAt(atPos);
	}
	inList.RemoveAll();
	DOUBLE dX, dY;
	dX = fabs( sortList.m_pTail->m_dPoint[0] - sortList.m_pHead->m_dPoint[0]);
	dY = fabs( sortList.m_pTail->m_dPoint[1] - sortList.m_pHead->m_dPoint[1]);
	if( dX > dY )
	{
		sortList.ResortAllPoint( NCDEF_SORTTYPE_XONEWAY, TRUE, 0.02f );
	}
	else
	{
		sortList.ResortAllPoint( NCDEF_SORTTYPE_YONEWAY, TRUE, 0.02f );
	}
	for( pPoint = (CSmartSPoint*)sortList.m_pHead ; pPoint ; pPoint = (CSmartSPoint*)pPoint->next )
	{
		pEntSurf = (CEttSurface*) pPoint->m_pData ;
		tmpList.AddTail(pEntSurf);
	}
	inList.AddTail(&tmpList);
	tmpList.RemoveAll();
}

BOOL CSmartFeatureSlotGen::CreateDraftSurf(CGeoCurve* pMidCur , VEC3D draftDir, DOUBLE dDraftWidth, CGeoTrmSurfList& draftSurList)
{
	if(pMidCur == NULL)
		return FALSE ;
	CGeoCurveList midCurList;
	if(pMidCur->GetType() == OBJ3D_CURVE_POLYLINE )
	{
		CGeoPLine3d* pPLine = (CGeoPLine3d*)pMidCur ;
		for(int i = 0 ; i < pPLine->m_num ; i++)
		{
			CGeoLine* pLine = new CGeoLine(pPLine->m_dPoint[i], pPLine->m_dPoint[i+1]);
			if(pLine)
			{
				midCurList.AddTail(pLine);
			}
		}
	}
	else
	{
		midCurList.AddTail(pMidCur);
	}

	CGeoCurveList CurveListTop, CurveListBtm;
	//构造拉伸面参数
	SURFDRAFT_PARAM  es ;
	es.m_dHeight = es.m_dHeight2 = dDraftWidth ;
	es.m_dAngle = 0.0 ;
	nc_VectorCopy(es.m_dirAxis, draftDir, 3);
	es.m_bFlipDir = FALSE ;
	es.m_bFlipAng = FALSE ;
	es.m_bBidir = TRUE ;
	es.m_bCapTop = es.m_bCapDown = FALSE ;
	es.m_bStitch = TRUE ;
	es.m_nCornerType = 0 ;
	es.m_bGrpSurf = TRUE;
	CSurfDraft creator ;
	creator.SurfDraft_Dir(&midCurList, es, &draftSurList, &CurveListTop, &CurveListBtm) ;
	DeltGeoCurveList( CurveListTop ) ;
	DeltGeoCurveList( CurveListBtm ) ;
	if(pMidCur->GetType() == OBJ3D_CURVE_POLYLINE )
	{
		DeltGeoCurveList( midCurList ) ;
	}
	if(draftSurList.IsEmpty())
		return FALSE;
	return TRUE ;
}
void CSmartFeatureSlotGen::SelectValidSurf(CGeoTrmSurfList& btmTrmSurfList, VEC3D vec, CGeoTrmSurfList& validSurfList)
{
	if(btmTrmSurfList.IsEmpty())
		return;
	UNUSED_ALWAYS(vec);
	//DOUBLE dAngTol = GetCurveTol().m_dAngTol;
	//PNT3D pnt;
	//VEC3D norVec;
	CGeoTrmSurf* pTrmSurf = NULL ;
	POSITION pos = btmTrmSurfList.GetHeadPosition();
	while(pos)
	{
		pTrmSurf = btmTrmSurfList.GetNext(pos);
		if(pTrmSurf)
		{
			/*pTrmSurf->GetNormal(0.5, 0.5, pnt, norVec);
			DOUBLE dAng = RADIAN_TO_ANGLE( mathGetAngle(norVec, vec, MIN_LEN) );
			if(dAng < dAngTol || fabs(dAng - 180.0) < dAngTol)
				continue;*/
			validSurfList.AddTail(pTrmSurf);
		}
	}
}
//找到侧壁组面最高点
void CSmartFeatureSlotGen::SideListTopZ(CEntList& SideList, DOUBLE& dTopZ)
{
	if(SideList.IsEmpty())
		return ;
	BOX3D box ;
	CEttSurface* pEntSurf = NULL;
	CGeoSurf* pGeoSurf = NULL;
	POSITION pos = SideList.GetHeadPosition();
	while(pos)
	{
		pEntSurf = (CEttSurface*)SideList.GetNext(pos);
		if( pEntSurf->GetType() != ETTYPE_SURFACE)
			continue;
		pGeoSurf = pEntSurf->GetSurf() ;
		pGeoSurf->UpdateBox(&box);
		if(box.max[2] > dTopZ)
			dTopZ = box.max[2] ;
	}
}
//找到等高路径中同层的位于不同侧壁的对应路径
SlotPath* CSmartFeatureSlotGen::FindPathInList(SlotPathList& allSlotPathList, int nIndex)
{
	if(allSlotPathList.IsEmpty())
		return NULL ;
	SlotPath* pSlotPath = NULL ;
	int j = 0 ;
	POSITION pos = allSlotPathList.GetHeadPosition();
	while(pos)
	{
		pSlotPath = allSlotPathList.GetNext(pos);
		j++;
		if(j == nIndex && pSlotPath)
		{
			return pSlotPath ;
		}
		pSlotPath = NULL ;
	}
	return pSlotPath ;
}
//连接同层对应路径
CSmtCutPath* CSmartFeatureSlotGen::ConnectTwoCPath(CSmtCutPath* pSidePath1, CSmtCutPath* pSidePath2)
{
	if(pSidePath1 == NULL && pSidePath2 == NULL)
		return NULL ;
	CSmtCutPath* pCutPath = new CSmtCutPath;
	pCutPath->m_bFeedType = JDNC_FEEDTYPE_ROUGH;
	for(CSmtCutPoint* pCPnt = pSidePath1->m_pHead; pCPnt; pCPnt = pCPnt->next)
	{
		if(pCPnt)
			pCutPath->AddTail(pCPnt->CopyMyself());
	}
	CSmtCutPoint* pTailPnt1 = pSidePath1->m_pTail->CopyMyself();
	pCutPath->AddTail(pTailPnt1);
	CSmtCutPoint* pTailPnt2 = pSidePath2->m_pTail->CopyMyself();
	pCutPath->AddTail(pTailPnt2);
	pSidePath2->ReverseDirect();
	for(CSmtCutPoint* pCPnt1 = pSidePath2->m_pHead; pCPnt1; pCPnt1 = pCPnt1->next )
	{
		if(pCPnt1)
			pCutPath->AddTail(pCPnt1->CopyMyself());
	}
	CSmtCutPoint* pHeadPnt2 = pSidePath2->m_pTail->CopyMyself();
	pCutPath->AddTail(pHeadPnt2);
	CSmtCutPoint* pHeadPnt1 = pSidePath1->m_pHead->CopyMyself();
	pCutPath->AddTail(pHeadPnt1);
	if(pCutPath->m_nNumPnt < 2)
	{
		pCutPath->ClearAllPoint();
		delete pCutPath;
		pCutPath = NULL ;
	}
	return pCutPath ;
}
//当前层路径添加到等高路径中
void AddCurLayerPath(SlotPathList& oneLayerPath, SlotPathList& sameHhtList)
{
	if(oneLayerPath.IsEmpty())
		return;
	SlotPath* pSlotPath = NULL ;
	POSITION pos = oneLayerPath.GetHeadPosition() ;
	while(pos)
	{
		pSlotPath = oneLayerPath.GetNext(pos);
		if(pSlotPath)
		{
			SlotPath* pNewHgtPath = new SlotPath;
			pNewHgtPath->bEClose = pSlotPath->bEClose ;
			pNewHgtPath->bSClose = pSlotPath->bSClose ;
			pNewHgtPath->pCutPath = pSlotPath->pCutPath->CopyMyself();
			pNewHgtPath->pCutPath->m_nLayerNo = pSlotPath->pCutPath->m_nLayerNo;
			sameHhtList.AddTail(pNewHgtPath);
		}
	}
}
void CSmartFeatureSlotGen::ConnectLayerSidePath(SlotPathList& allSlotPathList, int nLayerCnt, SlotPathList& sameHhtList)
{
	if(allSlotPathList.IsEmpty())
		return ;
	SlotPathList curLayerList;
	INT_PTR nCurLayerCnt = 0 ;
	POSITION pos = NULL ;
	SlotPath* pSlotPath = NULL, *pPairSlotPath = NULL;
	int j = 0 ;
	FPNT3D fVec1, fVec2 ;
	VEC3D vec1, vec2 ;
	BOOL bRev = FALSE ;
	BOOL bStart = TRUE;
	for(int i = 1 ; i <= nLayerCnt ; i++)
	{
		curLayerList.RemoveAll();
		SearchOneLayerList(allSlotPathList, i, curLayerList);
		if(curLayerList.IsEmpty())
			continue ;
		if( curLayerList.GetCount() < 2 )
		{
			AddCurLayerPath(curLayerList, sameHhtList);
			continue ;
		}
		nCurLayerCnt = curLayerList.GetCount() / 2;
		pos = curLayerList.GetHeadPosition();
		j = 0 ;
		while(pos)
		{
			pSlotPath = curLayerList.GetNext(pos);
			j++ ;
			if(j > nCurLayerCnt)
				break;
			if(pSlotPath)
			{
				pPairSlotPath = FindPathInList(curLayerList, int(j+nCurLayerCnt));
				if(pPairSlotPath)
				{
					bRev = FALSE ;
					//保证同层路径中找到的对应路径位于槽的不同侧壁
					nc_VectorMinus(pSlotPath->pCutPath->m_pHead->m_fPoint,pSlotPath->pCutPath->m_pTail->m_fPoint, fVec1, 3 );
					if( nc_Distance(pSlotPath->pCutPath->m_pHead->m_fPoint,pPairSlotPath->pCutPath->m_pHead->m_fPoint, 2) <
						nc_Distance(pSlotPath->pCutPath->m_pHead->m_fPoint,pPairSlotPath->pCutPath->m_pTail->m_fPoint, 2))
					{
						nc_VectorMinus(pSlotPath->pCutPath->m_pHead->m_fPoint,pPairSlotPath->pCutPath->m_pHead->m_fPoint, fVec2, 3 );
					}
					else
					{
						nc_VectorMinus(pSlotPath->pCutPath->m_pHead->m_fPoint,pPairSlotPath->pCutPath->m_pTail->m_fPoint, fVec2, 3 );
						bRev = TRUE ;
					}
					for(int k = 0 ; k < 3 ; k++)
					{
						vec1[k] = fVec1[k];
						vec2[k] = fVec2[k]; 
					}
					DOUBLE dAng = mathGetAngle(vec1, vec2, MIN_LEN);
					//DOUBLE dAngTol = ANGLE_TO_RADIAN(2.0);
					if(dAng < 0.01 || fabs(dAng - PI1) < 0.01)
					{
						AddCurLayerPath(curLayerList, sameHhtList);
						break;
					}
					if(fabs(pSlotPath->pCutPath->m_pHead->m_fPoint[2] - pPairSlotPath->pCutPath->m_pHead->m_fPoint[2]) > 0.001)
					{
						AddCurLayerPath(curLayerList, sameHhtList);
						break ;
					}
					//
					if(bRev)
					{
						pPairSlotPath->pCutPath->ReverseDirect();
					}
					if( i==1 )
					{
						if(pSlotPath->bSClose == TRUE && pSlotPath->bEClose == FALSE)
							bStart = FALSE ;
					}
					CSmtCutPath* pHgtCutPath = NULL ;
					if(bStart == TRUE)
					{
						pHgtCutPath = ConnectTwoCPath(pSlotPath->pCutPath, pPairSlotPath->pCutPath);
					}
					else
					{
						pSlotPath->pCutPath->ReverseDirect();
						pPairSlotPath->pCutPath->ReverseDirect();
						pHgtCutPath = ConnectTwoCPath( pSlotPath->pCutPath, pPairSlotPath->pCutPath );
					}				 
					if(pHgtCutPath)
					{
						SlotPath* pNewHgtPath = new SlotPath;
						pNewHgtPath->bSClose = pSlotPath->bSClose ;
						pNewHgtPath->bEClose = pSlotPath->bEClose ;
						pHgtCutPath->m_nLayerNo = i ;
						pNewHgtPath->pCutPath = pHgtCutPath ;
						sameHhtList.AddTail(pNewHgtPath);
					}	
				}
				else
				{
					AddCurLayerPath(curLayerList, sameHhtList);
					break ;
				}
			}

		}
	}
}
void AdjustSideList(CEntList& list1, CEntList& list2)
{
	if(list1.IsEmpty() || list2.IsEmpty())
		return ;
	if(list1.GetCount() < 2 || list2.GetCount() < 2)
		return;
	CEntity3D *pEnt = NULL ;
	CEttSurface *pSurfHead = NULL, *pSurfTail = NULL;

	PNT3D pntH, pntT ;
	VEC3D vec1, vec2 ;
	CEntList tmpList ;
	POSITION atPos = NULL ;
	//找到list1头元素
	POSITION pos = list1.GetHeadPosition();
	while(pos)
	{
		pEnt = list1.GetNext(pos);
		if( pEnt->GetType() != ETTYPE_SURFACE )
			continue ;
		pSurfHead = (CEttSurface*)pEnt;
		if(pSurfHead)
			break ;
	}
	//找到list1尾元素
	pos = list1.GetTailPosition();
	while(pos)
	{
		pEnt = list1.GetPrev(pos);
		if(pEnt->GetType() != ETTYPE_SURFACE)
			continue;
		pSurfTail = (CEttSurface*)pEnt;
		if(pSurfTail)
			break ;
	}
	pSurfHead->GetTrmSurf()->GetPoint(0.2, 0.5, pntH);
	pSurfTail->GetTrmSurf()->GetPoint(0.8, 0.5, pntT);
	nc_VectorMinus(pntH, pntT, vec1, 3);
	pSurfHead = pSurfTail = NULL ;
	//找到list2头元素
	pos = list2.GetHeadPosition();
	while(pos)
	{
		pEnt = list2.GetNext(pos);
		if(pEnt->GetType() != ETTYPE_SURFACE)
			continue;
		pSurfHead = (CEttSurface*)pEnt;
		if(pSurfHead)
			break ;
	}
	//找到list2尾元素
	pos = list2.GetTailPosition();
	while(pos)
	{
		pEnt = list2.GetPrev(pos);
		if(pEnt->GetType() != ETTYPE_SURFACE)
			continue;
		pSurfTail = (CEttSurface*)pEnt ;
		if(pSurfTail)
			break ;
	}
	pSurfHead->GetTrmSurf()->GetPoint(0.2, 0.5, pntH);
	pSurfTail->GetTrmSurf()->GetPoint(0.8, 0.5, pntT);
	nc_VectorMinus(pntH, pntT, vec2, 3);
	pSurfHead = pSurfTail = NULL ;
	DOUBLE dAng = RADIAN_TO_ANGLE(  mathGetAngle(vec1, vec2, MIN_LEN) );
	if(dAng > 90.0)//两组侧壁排序方向相反时，对list2反向排序
	{
		pos = list2.GetTailPosition();
		while(pos)
		{
			atPos = pos ;
			pEnt = list2.GetPrev(pos);
			tmpList.AddTail(pEnt);
			list2.RemoveAt(atPos);
		}
		list2.RemoveAll();
		pos = tmpList.GetHeadPosition();
		while(pos)
		{
			atPos = pos ;
			pEnt = tmpList.GetNext(pos);
			list2.AddTail(pEnt);
			tmpList.RemoveAt(atPos);
		}
		tmpList.RemoveAll();
	}
}
void AddPlungeToPath(CSmtCPathLib& pathLib, DOUBLE dDepth)
{
	if(pathLib.m_cAllPath.IsEmpty())
		return ;
	CSmtCutPath* pCutPath = pathLib.m_cAllPath.GetHead();
	if(pCutPath)
	{
		CSmtCutPoint* pCPnt1 = pCutPath->m_pHead->CopyMyself();
		CSmtCutPoint* pCPnt0 = pCPnt1->CopyMyself();
		pCPnt0->m_fPoint[2] += TFLOAT( dDepth ) ;
		CSmtCutPath* pPlunge = new CSmtCutPath;
		pPlunge->m_bFeedType = JDNC_FEEDTYPE_PLUNGE;
		pPlunge->AddTail(pCPnt0);
		pPlunge->AddTail(pCPnt1);
		pathLib.m_cAllPath.AddHead(pPlunge);
	}
}
BOOL CSmartFeatureSlotGen::OneDraftSlotPathEx( CPathCombine& PComb,  /*单个槽路径*/
												CSmtCheckMdl* pDriveMdl,   /*检查模型*/
												CSmartCurve* pMidCur,  /*槽壁两边界中位线*/
												CEntList&  SideList1,    /*槽侧壁1*/
												CEntList&  SideList2,    /*槽侧壁2*/
												CEttSurface* pSideSurf1,  /*侧壁1虚拟面*/
												CEttSurface* pSideSurf2,  /*侧壁2虚拟面*/
												DOUBLE dMaxWidth,
												JDNC_PRGDEF& cPrgDef )
{
	if( SideList1.IsEmpty() && SideList2.IsEmpty() )
		return FALSE;
	if(pDriveMdl == NULL || pMidCur == NULL)
		return FALSE;
	DOUBLE dArcTol = GetCurveTol().m_dArcTol;
	FPNT3D minp, maxp ;
	pDriveMdl->CalcFacetBox ( minp, maxp ) ;
	pDriveMdl->m_fBottom = (TFLOAT)minp[2]  ;
	pDriveMdl->m_bCheckMode = SMART_MODEL_PROJECT ;
	//找到侧壁面最高点
	DOUBLE dMax1 = pDriveMdl->m_fBottom ;
	SideListTopZ(SideList1, dMax1) ;
	DOUBLE dMax2 = pDriveMdl->m_fBottom ;
	SideListTopZ(SideList2, dMax2);
	DOUBLE dMax = max( dMax1, dMax2 ) ;
	//1:创建求交截面
	CGeoTrmSurfList draftSurList ;
	VEC3D vec1, norVec, movVec;
	PNT2D st2d, ed2d ;
	PNT3D st, ed ;
	pMidCur->GetStart(st2d);
	pMidCur->GetEnd(ed2d);
	st[0] = st2d[0];
	st[1] = st2d[1];
	ed[0] = ed2d[0];
	ed[1] = ed2d[1];
	st[2] = ed[2] = dMax ;
	nc_VectorMinus(st, ed, vec1, 3);
	nc_Normalize(vec1, 3);
	pMidCur->GetNormal(0., norVec);
	nc_Normalize(norVec, 2);
	norVec[2] = 0. ;
	CGeoLine* pLine3d = new CGeoLine(st, ed);
	if( !CreateDraftSurf(pLine3d, norVec, dMaxWidth, draftSurList))
	{
		if(pLine3d)
		{
			pLine3d->Destroy();
			delete pLine3d;
			pLine3d = NULL ;
		}
		return FALSE ;
	}
	if(pLine3d)
	{
		pLine3d->Destroy();
		delete pLine3d;
		pLine3d = NULL ;
	}
	//2:得到底部投影线,计算分层深度
	CSmtCutPath* pBtmPath = TransCurveToCPath(*pMidCur, m_cSetupDef.m_cTolDef);
	JDNC_TOOLEX tmpTool ;
	tmpTool.m_nToolType = m_pTool->GetType(); //surfncToolFlat;
	tmpTool.m_nToolID = m_pTool->m_nToolID ;
	tmpTool.m_dTopDiam = m_pTool->m_fRadius * 2 - 0.001 ;  
	tmpTool.m_dBtmDiam = m_pTool->m_fRadius * 2 - 0.001 ;
	CSmartTool* pTool = CreateSmartTool(tmpTool, 0., 0.);
	if(pTool == NULL)
		pTool = m_pTool->CopyMyself() ;
	pDriveMdl->UpdateCurrTool(pTool);
	pBtmPath->InsertCPoint(GetSurfaceTol().m_dMaxStep);
	pBtmPath->VerifyCutPath(*pDriveMdl, GetSurfaceTol(), m_cTmpPrgDef);
	pBtmPath->DelPointOnLine();
	pBtmPath->DefineBox();
	//底部投影线构造拉伸裁剪面
	CGeoTrmSurfList btmTrmSurfList, validSurfList;
	CGeoPLine3d* pBtmCur = ConvertCPathToCur(pBtmPath);
	CreateDraftSurf(pBtmCur, norVec, dMaxWidth, btmTrmSurfList);
	nc_VProduct(vec1, norVec, movVec); //中位线法向和切向叉乘得到平移方向
	SelectValidSurf(btmTrmSurfList, movVec, validSurfList);//剔除不合理的底部裁剪面
	DOUBLE dDepthInc = m_cFeedDef.m_cLayerDef.m_dSideDInc ;
	INT_PTR nIncNum = (INT_PTR)ceil((dMax - pBtmPath->m_fBox[0][2]) / dDepthInc);
	dDepthInc = ( dMax - pBtmPath->m_fBox[0][2] ) / nIncNum ; //重算分层深度;
	//3:z向平移裁剪面得到等高分层路径
	CGeoTrmSurf* pSideTrmSurf = (CGeoTrmSurf*)draftSurList.GetHead();
	if(movVec[2] > 0.0)
	{
		movVec[2] = -movVec[2];
	}
	VEC3D revVec;//当路径过切时向上抬刀向量
	revVec[0] = -movVec[0] ;
	revVec[1] = -movVec[1] ;
	revVec[2] = -movVec[2] ;
	movVec[0] *= dDepthInc ;
	movVec[1] *= dDepthInc ;
	movVec[2] *= dDepthInc ;
	if(SideList1.GetCount() > 2)//侧壁面数目大于2时需要排序
	{
		SortSideList(SideList1);
	}
	if(SideList2.GetCount() > 2)
	{
		SortSideList(SideList2);
	}
	if(SideList1.GetCount() > 1 && SideList2.GetCount() > 1) //使两组面排序方向一致
	{
		AdjustSideList(SideList1, SideList2);
	}
	SlotPathList allSlotPathList ;
	CSmtCPathLib layerCPathList;
	BOOL bBreak = FALSE , bPrgBrk = FALSE;
	DOUBLE dRevDepth = 0. ;
	double dCur[2] = {80., 20.};
	cPrgDef.m_dLimitAt = nIncNum / dCur[0];
	cPrgDef.m_dStepAt = 0.0 ;
	cPrgDef.m_dIncStep = 1.0 ;
	for(int i = 1; i < nIncNum ; i++ )
	{
		//////////////计算进度条///////
		if( cPrgDef.m_pBrkFunc && cPrgDef.m_pBrkFunc () )
		{
			bPrgBrk = TRUE;
			break;
		}
		cPrgDef.m_dStepAt += cPrgDef.m_dIncStep ;
		while( cPrgDef.m_pPrgFunc && cPrgDef.m_dStepAt >= cPrgDef.m_dLimitAt )
		{
			cPrgDef.m_pPrgFunc(1) ;
			cPrgDef.m_dStepAt -= cPrgDef.m_dLimitAt  ;
		}
		//////////////////////////////
		pSideTrmSurf->Move(movVec);
		CSmtCutPath* pSideLayerPath1 = DraftOneLayerPath(SideList1, pSideSurf1, pSideTrmSurf);
		if(pSideLayerPath1)
		{
			//当加工深度-底面高度小于刀具半径时，开始判断路径过切
			if( dMax - i*dDepthInc - pBtmPath->m_fBox[0][2] < m_pTool->m_fRadius)
			{
				if(pSideLayerPath1->m_pHead->m_fPoint[2] < pBtmPath->m_fBox[0][2])
				{//当前层已过切时抬高至不过切高度并结束循环
					dRevDepth = pBtmPath->m_fBox[0][2] - pSideLayerPath1->m_pHead->m_fPoint[2];
					revVec[0] *= dRevDepth ;
					revVec[1] *= dRevDepth ;
					revVec[2] *= dRevDepth ;
					pSideTrmSurf->Move(revVec);
					pSideLayerPath1->ClearAllPoint();
					delete pSideLayerPath1;
					pSideLayerPath1 = NULL ;
					CSmtCutPath* pLastLayerPath1 = DraftOneLayerPath(SideList1, pSideSurf1, pSideTrmSurf);
					if(pLastLayerPath1)
					{
						pLastLayerPath1->m_nLayerNo = i ;
						layerCPathList.AddToTail(pLastLayerPath1);
					}
					bBreak = TRUE ;
				}
				else
				{
					pSideLayerPath1->m_nLayerNo = i ;
					layerCPathList.AddToTail(pSideLayerPath1);
				}
			}
			else
			{
				pSideLayerPath1->m_nLayerNo = i ;
				layerCPathList.AddToTail(pSideLayerPath1);
			}
		}
		CSmtCutPath* pSideLayerPath2 = DraftOneLayerPath(SideList2, pSideSurf2, pSideTrmSurf);
		if(pSideLayerPath2)
		{
			if(bBreak)
			{
				pSideLayerPath2->ClearAllPoint();
				delete pSideLayerPath2;
				pSideLayerPath2= NULL ;
				CSmtCutPath* pLastLayerPath2 = DraftOneLayerPath(SideList2, pSideSurf2, pSideTrmSurf);
				if(pLastLayerPath2)
				{
					pLastLayerPath2->m_nLayerNo = i ;
					layerCPathList.AddToTail(pLastLayerPath2);
				}
			}
			else
			{
				pSideLayerPath2->m_nLayerNo = i ;
				layerCPathList.AddToTail(pSideLayerPath2);
			}
		}
		if(bBreak)
			break ;
	}
	if(bPrgBrk)//用户中断
	{
		DeleteSurfList(draftSurList);
		DeleteSurfList(btmTrmSurfList);
		ClearCPathLib(layerCPathList);
		ClearTmpData(NULL, pBtmPath, pTool);
		if(pBtmCur)
		{
			pBtmCur->Destroy();
			delete pBtmCur;
		}
		pBtmCur = NULL ;
		return FALSE ;
	}
	if(i==nIncNum && m_cParam.m_bExtEnd == TRUE)//最后一层路径:前面所有层路径均未过切
	{
		CGeoTrmSurf* pCopyTrmSurf = (CGeoTrmSurf*)pSideTrmSurf->CopyMyself();
		VEC3D copyVec;
		copyVec[0] = movVec[0];
		copyVec[1] = movVec[1];
		copyVec[2] = movVec[2];
		CSmtCutPath* pBtmLayerPath1 = DraftLastLayerPath(SideList1, pSideSurf1, pSideTrmSurf, pDriveMdl, pBtmPath->m_fBox[0][2], movVec);
		if(pBtmLayerPath1 != NULL)
		{
			pBtmLayerPath1->m_nLayerNo = i ;
			layerCPathList.AddToTail(pBtmLayerPath1);
		}
		CSmtCutPath* pBtmLayerPath2 = DraftLastLayerPath(SideList2, pSideSurf2, pCopyTrmSurf, pDriveMdl, pBtmPath->m_fBox[0][2], copyVec);
		if(pBtmLayerPath2 != NULL)
		{
			pBtmLayerPath2->m_nLayerNo = i ;
			layerCPathList.AddToTail(pBtmLayerPath2);
		}
		pCopyTrmSurf->Destroy();
		delete pCopyTrmSurf;
		pCopyTrmSurf = NULL ;
	}
	if( layerCPathList.m_cAllPath.GetCount() > 0 &&
		!TrimDraftAllLayer(layerCPathList, validSurfList, pDriveMdl, allSlotPathList, cPrgDef, dCur[1]) )
	{
		DeleteSurfList(draftSurList);
		DeleteSurfList(btmTrmSurfList);
		ClearCPathLib(layerCPathList);
		ClearTmpData(NULL, pBtmPath, pTool);
		ClearSlotPathList(allSlotPathList);
		validSurfList.RemoveAll();
		if(pBtmCur)
		{
			pBtmCur->Destroy();
			delete pBtmCur;
		}
		pBtmCur = NULL ;
		return FALSE ;
	}
	validSurfList.RemoveAll();
	//4.延伸路径端点
	if(m_cParam.m_bExtEnd == TRUE)
	{
		ExtendLayerEnd(allSlotPathList, pDriveMdl, TRUE);
	}
	//5.连接两侧壁同层路径
	SlotPathList sameHhtList ;
	ConnectLayerSidePath(allSlotPathList, (int)nIncNum, sameHhtList);
	//6:路径添加到PComb
	SlotPathList areaSortList;
	AreaSortPathList(sameHhtList, (int)nIncNum, areaSortList);
	AddDraftPathListToPCombEx(areaSortList, PComb);
	areaSortList.RemoveAll();
	//7.未加工到的区域路径:底层光刀
	//pBtmPath->AddToPathCombine(PComb);
	if(m_cParam.m_nFinishPath)
	{
		CEntList sideSurf1, sideSurf2 ;
		CSmtCPathLib sideCPath1, sideCPath2 ;
		sideSurf1.AddTail(pSideSurf1) ;
		sideSurf2.AddTail(pSideSurf2) ;
		POSITION pos = btmTrmSurfList.GetHeadPosition();
		while(pos)
		{
			CGeoTrmSurf* pBtmSurf = btmTrmSurfList.GetNext(pos);
			if(pBtmSurf)
			{
				CSmtCutPath* pBtmCutPath1 = DraftOneLayerPath(sideSurf1, pSideSurf1, pBtmSurf);
				if(pBtmCutPath1)
				{
					pBtmCutPath1->InsertCPoint(GetSurfaceTol().m_dMaxStep);
					for(CSmtCutPoint* pCPnt = pBtmCutPath1->m_pHead ; pCPnt ; pCPnt = pCPnt->next)
					{
						pDriveMdl->DefineHeight(*pCPnt);
					}
					pBtmCutPath1->DelPointOnLine();
					sideCPath1.AddToTail(pBtmCutPath1);
				}
				CSmtCutPath* pBtmCutPath2 = DraftOneLayerPath(sideSurf2, pSideSurf2, pBtmSurf);
				if(pBtmCutPath2)
				{
					pBtmCutPath2->InsertCPoint(GetSurfaceTol().m_dMaxStep);
					for(CSmtCutPoint* pCPnt = pBtmCutPath2->m_pHead ; pCPnt ; pCPnt = pCPnt->next)
					{
						pDriveMdl->DefineHeight(*pCPnt);
					}
					pBtmCutPath2->DelPointOnLine();
					sideCPath2.AddToTail(pBtmCutPath2);
				}
			}
		}
		sideCPath1.ConnectAllPath(*pDriveMdl, dArcTol);
		//添加下刀
		AddPlungeToPath(sideCPath1, dDepthInc);
		sideCPath1.AddToPathCombine(PComb);
		sideCPath2.ConnectAllPath(*pDriveMdl, dArcTol);
		AddPlungeToPath(sideCPath2, dDepthInc);
		sideCPath2.AddToPathCombine(PComb);
		ClearCPathLib(sideCPath1);
		ClearCPathLib(sideCPath2);
	}
	pBtmPath->AddToPathCombine(PComb);
	DeleteSurfList(draftSurList);
	DeleteSurfList(btmTrmSurfList);

	ClearSlotPathList(allSlotPathList);
	ClearSlotPathList(sameHhtList);
	ClearCPathLib(layerCPathList);
	
	if(pBtmPath)
	{
		pBtmPath->ClearAllPoint();
		delete pBtmPath;
	}
	if(pBtmCur)
	{
		pBtmCur->Destroy();
		delete pBtmCur;
	}
	pBtmCur = NULL ;
	pBtmPath = NULL ;
	if(pTool)
		delete pTool;
	pTool = NULL ;
	PComb.DelOverlapPoint();
	return TRUE ;
}

BOOL CSmartFeatureSlotGen::OneDraftSlotPath( CPathCombine& PComb,       /*单个槽路径*/
											CSmtCheckMdl* pDriveMdl,    /*检查模型*/
											CSmartCurve* pMidCur,   /*槽壁两边界中位线*/
											CEntList&  SideList,  /*槽侧壁*/
											CEttSurface* pSideSurf, /*侧壁虚拟面*/
											DOUBLE dMaxWidth,
											JDNC_PRGDEF& cPrgDef ) 
{
	if( SideList.IsEmpty() )
		return FALSE;
	if(pDriveMdl == NULL || pMidCur == NULL)
		return FALSE;
	FPNT3D minp, maxp ;
	pDriveMdl->CalcFacetBox ( minp, maxp ) ;
	pDriveMdl->m_fBottom = (TFLOAT)minp[2]  ;
	pDriveMdl->m_bCheckMode = SMART_MODEL_PROJECT ;
	//找到侧壁面最高点
	DOUBLE dMax = pDriveMdl->m_fBottom;
	SideListTopZ(SideList, dMax);
	//1:创建求交截面
	CGeoTrmSurfList draftSurList ;
	VEC3D vec1, norVec, movVec;
	PNT2D st2d, ed2d ;
	PNT3D st, ed ;
	pMidCur->GetStart(st2d);
	pMidCur->GetEnd(ed2d);
	st[0] = st2d[0];
	st[1] = st2d[1];
	ed[0] = ed2d[0];
	ed[1] = ed2d[1];
	st[2] = ed[2] = dMax ;
	nc_VectorMinus(st, ed, vec1, 3);
	nc_Normalize(vec1, 3);
	pMidCur->GetNormal(0., norVec);
	nc_Normalize(norVec, 2);
	norVec[2] = 0. ;
	CGeoLine* pLine3d = new CGeoLine(st, ed);
	if( !CreateDraftSurf(pLine3d, norVec, dMaxWidth, draftSurList))
	{
		if(pLine3d)
		{
			pLine3d->Destroy();
			delete pLine3d;
			pLine3d = NULL ;
		}
		return FALSE ;
	}
	if(pLine3d)
	{
		pLine3d->Destroy();
		delete pLine3d;
		pLine3d = NULL ;
	}
	//2:得到底部投影线,计算分层深度
	CSmtCutPath* pBtmPath = TransCurveToCPath(*pMidCur, m_cSetupDef.m_cTolDef);
	JDNC_TOOLEX tmpTool ;
	tmpTool.m_nToolType = m_pTool->GetType(); //surfncToolFlat;
	tmpTool.m_nToolID = m_pTool->m_nToolID ;
	tmpTool.m_dTopDiam = m_pTool->m_fRadius * 2 - 0.001 ;  
	tmpTool.m_dBtmDiam = m_pTool->m_fRadius * 2 - 0.001 ;
	CSmartTool* pTool = CreateSmartTool(tmpTool, 0., 0.);
	if(pTool == NULL)
		pTool = m_pTool->CopyMyself() ;
	pDriveMdl->UpdateCurrTool(pTool);
	pBtmPath->InsertCPoint(GetSurfaceTol().m_dMaxStep);
	pBtmPath->VerifyCutPath(*pDriveMdl, GetSurfaceTol(), m_cTmpPrgDef);
	pBtmPath->DelPointOnLine();
	pBtmPath->DefineBox();
	//底部投影线构造拉伸裁剪面
	CGeoTrmSurfList btmTrmSurfList, validSurfList;
	CGeoPLine3d* pBtmCur = ConvertCPathToCur(pBtmPath);
	CreateDraftSurf(pBtmCur, norVec, dMaxWidth, btmTrmSurfList);
	nc_VProduct(vec1, norVec, movVec); //中位线法向和切向叉乘得到平移方向
	SelectValidSurf(btmTrmSurfList, movVec, validSurfList);//剔除不合理的底部裁剪面
	DOUBLE dDepthInc = m_cFeedDef.m_cLayerDef.m_dSideDInc ;
	INT_PTR nIncNum =(int)(ceil((dMax - pBtmPath->m_fBox[0][2]) / dDepthInc));
	dDepthInc = ( dMax - pBtmPath->m_fBox[0][2] ) / nIncNum ; //重算分层深度;
	//3:z向平移裁剪面得到等高分层路径
	CGeoTrmSurf* pSideTrmSurf = (CGeoTrmSurf*)draftSurList.GetHead();
	if(movVec[2] > 0.0)
	{
		movVec[2] = -movVec[2];
	}
	VEC3D revVec;//当路径过切时向上抬刀向量
	revVec[0] = -movVec[0] ;
	revVec[1] = -movVec[1] ;
	revVec[2] = -movVec[2] ;
	movVec[0] *= dDepthInc ;
	movVec[1] *= dDepthInc ;
	movVec[2] *= dDepthInc ;
	if(SideList.GetCount() > 2)//侧壁面数目大于2时需要排序
	{
		SortSideList(SideList);
	}
	SlotPathList allSlotPathList ;
	CSmtCPathLib layerCPathList;
	DOUBLE dRevDepth = 0. ;
	
	BOOL bPrgBrk = FALSE ;
	double dCur[2] = { 40, 10 } ;
	cPrgDef.m_dLimitAt = nIncNum / dCur[0] ;
	cPrgDef.m_dStepAt = 0.0 ;
	cPrgDef.m_dIncStep = 1.0 ;

	for(int i = 1; i < nIncNum ; i++ )
	{
		//////////////计算进度条///////
		if( cPrgDef.m_pBrkFunc && cPrgDef.m_pBrkFunc () )
		{
			bPrgBrk = TRUE;
			break;
		}
		cPrgDef.m_dStepAt += cPrgDef.m_dIncStep ;
		while( cPrgDef.m_pPrgFunc && cPrgDef.m_dStepAt >= cPrgDef.m_dLimitAt )
		{
			cPrgDef.m_pPrgFunc(1) ;
			cPrgDef.m_dStepAt -= cPrgDef.m_dLimitAt  ;
		}
		//////////////////////////////
		pSideTrmSurf->Move(movVec);
		CSmtCutPath* pOneLayerPath = DraftOneLayerPath(SideList, pSideSurf, pSideTrmSurf);
		if(pOneLayerPath)
		{
			//当加工深度-底面高度小于刀具半径时，开始判断路径过切
			if( dMax - i* dDepthInc - pBtmPath->m_fBox[0][2] < m_pTool->m_fRadius)
			{
				if(pOneLayerPath->m_pHead->m_fPoint[2] < pBtmPath->m_fBox[0][2])
				{//当前层已过切时抬高至不过切高度并结束循环
					dRevDepth = pBtmPath->m_fBox[0][2] - pOneLayerPath->m_pHead->m_fPoint[2] ; 
					revVec[0] *= dRevDepth ;
					revVec[1] *= dRevDepth ;
					revVec[2] *= dRevDepth ;
					pOneLayerPath->ClearAllPoint();
					delete pOneLayerPath;
					pOneLayerPath = NULL ;
					pSideTrmSurf->Move(revVec);
					CSmtCutPath* pLastLayerPath = DraftOneLayerPath(SideList, pSideSurf, pSideTrmSurf);
					if(pLastLayerPath)
					{
						pLastLayerPath->m_nLayerNo = i ;
						layerCPathList.AddToTail(pLastLayerPath);
					}
					break ;
				}
			}
			pOneLayerPath->m_nLayerNo = i ;
			layerCPathList.AddToTail(pOneLayerPath);
		}
	}
	if(bPrgBrk)
	{
		DeleteSurfList(draftSurList);
		DeleteSurfList(btmTrmSurfList);
		ClearCPathLib(layerCPathList);
		ClearTmpData(NULL, pBtmPath, pTool);
		if(pBtmCur)
		{
			pBtmCur->Destroy();
			delete pBtmCur;
		}
		pBtmCur = NULL ;
		return FALSE ;
	}
	if(i==nIncNum && m_cParam.m_bExtEnd == TRUE)//最后一层路径:前面所有层路径均未过切
	{
		CSmtCutPath* pBtmLayerPath = DraftLastLayerPath(SideList, pSideSurf, pSideTrmSurf, pDriveMdl, pBtmPath->m_fBox[0][2], movVec);
		if(pBtmLayerPath != NULL)
		{
			pBtmLayerPath->m_nLayerNo = i ;
			layerCPathList.AddToTail(pBtmLayerPath);
		}
	}
	if( layerCPathList.m_cAllPath.GetCount() > 0 && 
		!TrimDraftAllLayer(layerCPathList, validSurfList, pDriveMdl, allSlotPathList, cPrgDef, dCur[1]) )
	{
		DeleteSurfList(draftSurList);
		DeleteSurfList(btmTrmSurfList);
		ClearCPathLib(layerCPathList);
		ClearSlotPathList(allSlotPathList);
		ClearTmpData(NULL, pBtmPath, pTool);
		validSurfList.RemoveAll();
		if(pBtmCur)
		{
			pBtmCur->Destroy();
			delete pBtmCur;
		}
		pBtmCur = NULL ;
		return FALSE ;
	}
	
	validSurfList.RemoveAll();
	//4.开口处延伸路径
	if(m_cParam.m_bExtEnd == TRUE)
	{
		ExtendLayerEnd(allSlotPathList, pDriveMdl, TRUE);
	}
	//5.路径添加到PComb
	SlotPathList areaSortList;
	AreaSortPathList(allSlotPathList, (int)nIncNum, areaSortList);
	AddDraftPathListToPComb(areaSortList, PComb);

	areaSortList.RemoveAll();
	//6.未加工到的区域路径
	//pBtmPath->AddToPathCombine(PComb);
	CEntList sideSurf ;
	CSmtCPathLib sideCPath ;
	sideSurf.AddTail(pSideSurf) ;
	POSITION pos = btmTrmSurfList.GetHeadPosition();
	while(pos)
	{
		CGeoTrmSurf* pBtmSurf = btmTrmSurfList.GetNext(pos);
		if(pBtmSurf)
		{
			CSmtCutPath* pBtmCutPath = DraftOneLayerPath(sideSurf, pSideSurf, pBtmSurf);
			if(pBtmCutPath)
			{
				pBtmCutPath->InsertCPoint(GetSurfaceTol().m_dMaxStep);
				for(CSmtCutPoint* pCPnt = pBtmCutPath->m_pHead ; pCPnt ; pCPnt = pCPnt->next)
				{
					pDriveMdl->DefineHeight(*pCPnt);
				}
				pBtmCutPath->DelPointOnLine();
				sideCPath.AddToTail(pBtmCutPath);
			}
		}
	}
	sideCPath.ConnectAllPath(*pDriveMdl, GetCurveTol().m_dArcTol);
	AddPlungeToPath(sideCPath, dDepthInc);
	sideCPath.AddToPathCombine(PComb);
	DeleteSurfList(draftSurList);
	DeleteSurfList(btmTrmSurfList);
	ClearSlotPathList(allSlotPathList);
	ClearCPathLib(layerCPathList);
	ClearCPathLib(sideCPath);

	if(pBtmPath)
	{
		pBtmPath->ClearAllPoint();
		delete pBtmPath;
	}
	if(pBtmCur)
	{
		pBtmCur->Destroy();
		delete pBtmCur;
	}
	pBtmCur = NULL ;
	pBtmPath = NULL ;
	if(pTool)
		delete pTool;
	pTool = NULL ;
	return TRUE ;
}
CSmtCutPath* CSmartFeatureSlotGen::DraftLastLayerPath(CEntList& SideList, 
													  CEttSurface* pSideSurf, 
													  CGeoTrmSurf* pIntSurf, 
													  CSmtCheckMdl* pDirveMdl, 
													  DOUBLE dBtm,
													  VEC3D movVec)
{
	if( SideList.IsEmpty() )
		return NULL ;
	if(pDirveMdl == NULL || pIntSurf == NULL)
		return NULL ;
	DOUBLE dArcTol = GetCurveTol().m_dArcTol ;
	VEC3D revVec;
	revVec[0] = -movVec[0] ;
	revVec[1] = -movVec[1] ;
	revVec[2] = -movVec[2] ;
	nc_Normalize(revVec, 3);
	while(1)
	{
		movVec[0] *= 0.5 ;
		movVec[1] *= 0.5 ;
		movVec[2] *= 0.5 ;
		pIntSurf->Move(movVec);
		CSmtCutPath* pTmpLayerPath = DraftOneLayerPath(SideList, pSideSurf, pIntSurf);
		if(pTmpLayerPath)
		{
			if(pTmpLayerPath->m_pHead->m_fPoint[2] < dBtm)
			{//当前高度已过切时直接抬高至不过切处并退出
				DOUBLE dRevDepth = dBtm - pTmpLayerPath->m_pHead->m_fPoint[2];
				revVec[0] *= dRevDepth ;
				revVec[1] *= dRevDepth ;
				revVec[2] *= dRevDepth ;
				pIntSurf->Move(revVec);
				pTmpLayerPath->ClearAllPoint();
				delete pTmpLayerPath;
				pTmpLayerPath = NULL ;
				CSmtCutPath* pBtmLayerPath = DraftOneLayerPath(SideList, pSideSurf, pIntSurf);
				if(pBtmLayerPath)
				{
					return pBtmLayerPath ;
				}
			}
			else if(pTmpLayerPath->m_pHead->m_fPoint[2] - dBtm < dArcTol)
			{//当前高度已逼近至底面高度时直接生成路径并退出
				return pTmpLayerPath ;
			}
			else 
			{
				pTmpLayerPath->ClearAllPoint();
				delete pTmpLayerPath;
				pTmpLayerPath = NULL ;
			}
		}
		else
		{
			return NULL;
		}
	}
}
CSmtCutPath* CSmartFeatureSlotGen::DraftOneLayerPath(CEntList& SideList, CEttSurface* pVirtualSideSurf, CGeoTrmSurf* pIntSurf)
{
	if( SideList.IsEmpty())
		return NULL ;
	if( pIntSurf == NULL )
		return NULL ;
	CEttPModelEx* pPModel= SurfNC_GetCurPathModel() ;
	if( ! pPModel )
		return FALSE;
	CJDTPlaneLib* pPLib = pPModel->GetTPlaneLib() ;
	if( ! pPLib )
		return FALSE;
	CJDTPlane * pCurrTPlane = pPLib->GetActiveTPlane();
	if( ! pCurrTPlane )
		return FALSE;
	BOOL bRev = FALSE ;
	if(SideList.GetCount() > 1)
		bRev = TRUE ;
	RFRAME tpFrm ;
	SurfNC_GetTPlaneFrame(pCurrTPlane, tpFrm, pPModel);

	PNT3D pnt ;
	VEC3D toolAxis, vNor,  offset;
	nc_VectorCopy(toolAxis, tpFrm.Z, 3);
	VEC3D listVec, intVec ;
	memset(listVec, 0, sizeof(VEC3D));
	memset(intVec, 0, sizeof(VEC3D));
	if(bRev)//得到排序后向量
	{
		PNT3D headPnt, tailPnt ;
		CEttSurface* pHeadSurf = (CEttSurface*)SideList.GetHead();
		CEttSurface* pTailSurf = (CEttSurface*)SideList.GetTail();
		CGeoTrmSurf* pSurf1 = pHeadSurf->GetTrmSurf();
		CGeoTrmSurf* pSurf2 = pTailSurf->GetTrmSurf();
		pSurf1->GetPoint(0.2, 0.5, headPnt);
		pSurf2->GetPoint(0.5, 0.5, tailPnt);
		nc_VectorMinus(headPnt, tailPnt, listVec, 3);
		nc_Normalize(listVec, 3);
	}
	//得到虚拟侧壁面法向,保证槽的两组侧壁法失方向为相对
	VEC3D vSNor ;
	if(pVirtualSideSurf && pVirtualSideSurf->GetTrmSurf())
	{
		pVirtualSideSurf->GetTrmSurf()->GetNormal(0.5, 0.5, pnt, vSNor);
	}

	CSmtCutPath* pCutPath = new CSmtCutPath;
	CEttSurface* pEntSurf = NULL;
	CGeoTrmSurf* pSideSurf = NULL ;
	CSSICurve* pIntCurve = NULL, *pCv ;
	CGeoInter ssInter ;
	POSITION pos = SideList.GetHeadPosition();
	while(pos)
	{
		pEntSurf = (CEttSurface*)SideList.GetNext(pos);
		if( pEntSurf->GetType() != ETTYPE_SURFACE)
			continue;
		pSideSurf = pEntSurf->GetTrmSurf();
		ssInter.TwoSurfaceInt(pSideSurf->m_pSurface, pIntSurf->m_pSurface, pIntCurve);
		if( pIntCurve == NULL ) 
			continue ;
		for( pCv = pIntCurve ; pCv ; pCv = pCv->m_next )
		{
			if( pCv->m_headssipt == NULL )
			{
				continue ;
			}
			nc_VectorMinus( pCv->m_headssipt->p, pCv->m_tailssipt->p, intVec, 3);
			nc_Normalize(intVec, 3);
			//调整交线方向
			if( bRev && nc_OProduct( listVec, intVec, 3 ) < 0.0 ) 
			{
				pCv->ReverseIntCur() ;
			}
			for(tagSSIntpt *pIntAt = pCv->m_headssipt; pIntAt; pIntAt = pIntAt->next)
			{
				DOUBLE uv[2];
				uv[0] = pIntAt->uv[0];
				uv[1] = pIntAt->uv[1];
				if( !pSideSurf->IsInParaLoopByCrsLine(uv, 0.01))//判断交点是否位于侧壁面上
				{
					continue ;
				}
				pSideSurf->GetNormal(pIntAt->uv[0], pIntAt->uv[1], pnt, vNor);
				if( nc_OProduct(vSNor, vNor, 3) < 1.0e-8 )//点积为负
				{
					vNor[0] = -vNor[0] ;
					vNor[1] = -vNor[1] ;
					vNor[2] = -vNor[2] ;
				}
				m_pTool->Get5AxToolOffset(toolAxis, vNor, offset); //得到刀触点转化为刀尖点的平移向量
				pIntAt->p[0] += offset[0];
				pIntAt->p[1] += offset[1];
				pIntAt->p[2] += offset[2];
				pCutPath->AddPoint(pIntAt->p);
			}
		}
		DeleteSSICurve(pIntCurve);
	}
	pCutPath->DelPointOnLine();
	if(pCutPath->m_nNumPnt < 2)
	{
		pCutPath->ClearAllPoint();
		delete pCutPath ;
		pCutPath = NULL ;
	}
	return pCutPath ;
}
void SortDraftOneLayer(CSmtCPathLib& OrgPathList)
{
	if(OrgPathList.m_cAllPath.IsEmpty())
		return ;
	POSITION pos = OrgPathList.m_cAllPath.GetHeadPosition();
	BOOL bFirst = TRUE ;
	CSmtCutPath* pLayerCPath = NULL;
	FPNT3D firstVec, layerVec ;
	while(pos)
	{
		pLayerCPath = OrgPathList.m_cAllPath.GetNext(pos);
		if(pLayerCPath == NULL)
			continue ;
		if(bFirst)
		{
			nc_VectorMinus(pLayerCPath->m_pHead->m_fPoint, pLayerCPath->m_pTail->m_fPoint,firstVec, 3);
			bFirst = FALSE ;
		}
		else
		{
			nc_VectorMinus(pLayerCPath->m_pHead->m_fPoint, pLayerCPath->m_pTail->m_fPoint, layerVec, 3);
			if(nc_OProduct(firstVec, layerVec, 3) < 0.0)
			{
				pLayerCPath->ReverseDirect();
			}
		}
	}
}
//裁剪等高分层路径
BOOL CSmartFeatureSlotGen::TrimDraftAllLayer( CSmtCPathLib& cutPathList, /*原始等高截线路径*/
											 CGeoTrmSurfList& btmSurfList,    /*底部拉伸裁剪面*/
											 CSmtCheckMdl* pDriveMdl,  /*检查模型*/	
											 SlotPathList& allSlotPathList,
											 JDNC_PRGDEF& cPrgDef,
											 double dCur)
{
	if(cutPathList.m_cAllPath.IsEmpty())
		return FALSE ;
	//无底部裁剪面时，直接将原始路径生成槽路径
	CSmtCutPath* pLayerCPath = NULL;
	POSITION pos = NULL;
	if(btmSurfList.IsEmpty())
	{
		pos = cutPathList.m_cAllPath.GetHeadPosition();
		while(pos)
		{
			pLayerCPath = cutPathList.m_cAllPath.GetNext(pos);
			if(pLayerCPath)
			{
				SlotPath* pTmpSlotPath = new SlotPath;
				pTmpSlotPath->pCutPath = pLayerCPath->CopyMyself();
				pTmpSlotPath->bEClose = FALSE ;
				pTmpSlotPath->bSClose = FALSE ;
				allSlotPathList.AddTail(pTmpSlotPath);
			}
		}
		return TRUE ;
	}
	//保证每层路径同向
	SortDraftOneLayer(cutPathList);
	//离散底部裁剪面
	DOUBLE dArcTol = GetCurveTol().m_dArcTol ;
	DOUBLE dAngTol = GetCurveTol().m_dAngTol ;
	SurfFacetList allFacet ; 
	CGeoTrmSurf* pTrmSurf = NULL ;
	POSITION posSurf = btmSurfList.GetHeadPosition();
	while(posSurf)
	{
		pTrmSurf = btmSurfList.GetNext(posSurf);
		CFacet* pFacet = pTrmSurf->DiscreteToFacet(dArcTol, dAngTol);
		SurfAndFacet* snode = new SurfAndFacet;
		snode->pTrmSurf = pTrmSurf;
		snode->pFacet = pFacet;
		allFacet.AddTail(snode);
	}
	//裁剪每层路径
	CGeoPLine3d* pLayerPLine = NULL;
	CCurIntpt intpt1;
	PNT3D slotPntSt, slotPnted;
	int nLayerNo = 0;
	CSmartTool* pCutTool = pDriveMdl->m_pTool ;
	JDNC_TOOLEX tmpTool ;
	tmpTool.m_nToolType = surfncToolFlat;
	tmpTool.m_dTopDiam = pCutTool->m_fRadius * 2 - 0.02;  
	CSmartTool* pCheckTool = CreateSmartTool(tmpTool, 0., 0.);
	pDriveMdl->UpdateCurrTool(pCheckTool);//可能会由于模型精度问题导致误判所以更新刀具
	//定义进度条
	int nCnt = (int)cutPathList.m_cAllPath.GetCount();
	cPrgDef.m_dLimitAt = nCnt / dCur ;
	cPrgDef.m_dStepAt = 0.0 ;
	cPrgDef.m_dIncStep = 1.0 ;
	pos =  cutPathList.m_cAllPath.GetHeadPosition();
	while(pos)
	{
		//////////////计算进度条///////
		if( cPrgDef.m_pBrkFunc && cPrgDef.m_pBrkFunc () )
		{
			return FALSE ;
		}
		cPrgDef.m_dStepAt += cPrgDef.m_dIncStep ;
		while( cPrgDef.m_pPrgFunc && cPrgDef.m_dStepAt >= cPrgDef.m_dLimitAt )
		{
			cPrgDef.m_pPrgFunc(1) ;
			cPrgDef.m_dStepAt -= cPrgDef.m_dLimitAt  ;
		}
		pLayerCPath = cutPathList.m_cAllPath.GetNext(pos);
		if(pLayerCPath)
		{
			nLayerNo = pLayerCPath->m_nLayerNo ;
			pLayerPLine = ConvertCPathToCur(pLayerCPath);
			if(pLayerPLine == NULL)
				continue;
			pLayerPLine->GetEndPoint(slotPntSt, slotPnted);
			if(GetPLineAllSurfIntPnt(pLayerPLine, allFacet, intpt1))
			{
				intpt1.SortByParam();
				//分段判断每段路径
				tagCCIntpt *ipt1,*ipt2;	
				PNT3D midPt ;
				FPNT3D fMidPt ;
				int j = 0 ; //交点个数
				//首段
				ipt1 = intpt1.GetHead();
				midPt[2] = slotPntSt[2] ;
				if(nc_Distance(slotPntSt, ipt1->pos, 2) > dArcTol)//起点跟第一个交点不重合
				{
					midPt[0] = (slotPntSt[0] + ipt1->pos[0]) / 2.0 ;
					midPt[1] = (slotPntSt[1] + ipt1->pos[1]) / 2.0 ;
					nc_DoubleToFloat(fMidPt, midPt, 3);
					pDriveMdl->DefineHeight(fMidPt);
					if(fMidPt[2] - midPt[2] > dArcTol)
					{
					}
					else
					{
						j++ ;
						CGeoCurve* pSubLine = pLayerPLine->GetSubCopy(0., ipt1->u);
						if(pSubLine)
						{
							CSmtCutPath* pCutPath = TransCur3DToCPath(pSubLine);
							if(pCutPath)
							{
								SlotPath* pTmpSlotPath = new SlotPath;
								pCutPath->m_nLayerNo = nLayerNo ;
								pTmpSlotPath->pCutPath = pCutPath ; 
								pTmpSlotPath->bSClose = FALSE ;
								pTmpSlotPath->bEClose = TRUE ;
								allSlotPathList.AddTail(pTmpSlotPath);
							}
							pSubLine->Destroy();
							delete pSubLine;
							pSubLine = NULL;
						}
					}
				}
				//中间段
				INT_PTR iCnt = intpt1.m_nCount -1 ;
				for(int i = 0; i < iCnt; i++)
				{
					ipt1 = intpt1.GetPoint(i);
					ipt2 = intpt1.GetPoint(i+1);
					midPt[0] = (ipt1->pos[0] + ipt2->pos[0])/2.0 ;
					midPt[1] = (ipt1->pos[1] + ipt2->pos[1])/2.0 ;
					nc_DoubleToFloat(fMidPt, midPt, 3);
					pDriveMdl->DefineHeight(fMidPt);
					if(fMidPt[2] - midPt[2] > dArcTol)
					{
					}
					else
					{
						j++;
						CGeoCurve* pSubLine = pLayerPLine->GetSubCopy( ipt1->u, ipt2->u );
						if(pSubLine)
						{
							CSmtCutPath* pCutPath = TransCur3DToCPath(pSubLine);
							if(pCutPath)
							{
								SlotPath* pTmpSlotPath = new SlotPath;
								pCutPath->m_nLayerNo = nLayerNo ;
								pTmpSlotPath->pCutPath = pCutPath ; 
								pTmpSlotPath->bSClose = TRUE ;
								pTmpSlotPath->bEClose = TRUE ;
								allSlotPathList.AddTail(pTmpSlotPath);
							}
							pSubLine->Destroy();
							delete pSubLine;
							pSubLine = NULL;
						}
					}
				}
				//末段
				ipt2 = intpt1.GetTail();
				if( nc_Distance(slotPnted, ipt2->pos, 2 ) > dArcTol )//末点与最后一个交点不重合
				{
					midPt[0] = (slotPnted[0] + ipt2->pos[0])/ 2.0 ;
					midPt[1] = (slotPnted[1] + ipt2->pos[1])/ 2.0 ;
					nc_DoubleToFloat(fMidPt, midPt, 3);
					pDriveMdl->DefineHeight(fMidPt);
					if(fMidPt[2] - midPt[2] > dArcTol)
					{
					}
					else
					{
						j++ ;
						CGeoCurve* pSubLine = pLayerPLine->GetSubCopy( ipt2->u, 1. );
						if(pSubLine)
						{
							CSmtCutPath* pCutPath = TransCur3DToCPath(pSubLine);
							if(pCutPath)
							{
								SlotPath* pTmpSlotPath = new SlotPath;
								pCutPath->m_nLayerNo = nLayerNo ;
								pTmpSlotPath->pCutPath = pCutPath ; 
								pTmpSlotPath->bSClose = TRUE ;
								pTmpSlotPath->bEClose = FALSE ;
								allSlotPathList.AddTail(pTmpSlotPath);
							}
							pSubLine->Destroy();
							delete pSubLine;
						}
						pSubLine = NULL;
					}
				}
				intpt1.Destory();
			}
			else
			{
				SlotPath* pTmpSlotPath = new SlotPath;
				pTmpSlotPath->pCutPath = pLayerCPath->CopyMyself() ; 
				pTmpSlotPath->bSClose = FALSE ;
				pTmpSlotPath->bEClose = FALSE ;
				allSlotPathList.AddTail(pTmpSlotPath);
			}
			pLayerPLine->Destroy();
			delete pLayerPLine;
			pLayerPLine = NULL ;
		}
	}
	ClearFacetList(allFacet);
	pDriveMdl->UpdateCurrTool(pCutTool);
	if(pCheckTool)
		delete pCheckTool ;
	pCheckTool = NULL ;
	return TRUE ;

}
//3D折线与底部裁剪面的所有交点
BOOL CSmartFeatureSlotGen::GetPLineAllSurfIntPnt(CGeoPLine3d* pLine3d, SurfFacetList& allFacet, CCurIntpt& allPntList)
{
	if(pLine3d == NULL)
		return FALSE ;
	if(allFacet.IsEmpty())
		return FALSE ;
	CCurIntpt  intList ; 
	SurfAndFacet* pSNode = NULL ;
	POSITION pos = allFacet.GetHeadPosition();
	while(pos)
	{
		pSNode = allFacet.GetNext(pos);
		if(pSNode)
		{
			if(GetPLineSurfIntPnt(pLine3d, pSNode, intList) && intList.GetCount() > 0)
			{
				allPntList.AddTail(&intList);
			}
		}
	}
	if(allPntList.GetCount() < 1)
		return FALSE ;
	return TRUE ;
}

int		LineTrmSurfInt(PNT3D org,VEC3D dir,CGeoTrmSurf *pTrmSurf,CFacet *pFacet,CCurIntpt& linPntList,CCurIntpt& surfPntList);

//3D折线与裁剪面的所有交点
BOOL CSmartFeatureSlotGen::GetPLineSurfIntPnt(CGeoPLine3d* pLine3d,SurfAndFacet* pSNode, CCurIntpt& linPntList)
{
	if(pLine3d == NULL || pSNode == NULL)
		return FALSE;
	if(pSNode->pFacet == NULL || pSNode->pTrmSurf == NULL)
		return FALSE;
	//包围盒初判
	BOX3D lnBox, surfBox;
	pSNode->pTrmSurf->UpdateBox(&surfBox);
	pLine3d->UpdateBox(&lnBox);
	if(IDINT != mathChkBox3DInt(&lnBox, &surfBox, 0.01))
		return FALSE ;
	VEC3D vec;
	CCurIntpt  ptIntA, ptIntB;
	DOUBLE dLen = 0.;
	DOUBLE dSubLen = 0.;
	double  t = 0, dist = 0;  
	for(int i= 0 ; i < pLine3d->m_num; i++)
	{
		CGeoLine* pSect = new CGeoLine(pLine3d->m_dPoint[i], pLine3d->m_dPoint[i+1]);
		if(pSect)
		{
			pSect->UpdateBox(&lnBox);
			if( IDINT == mathChkBox3DInt(&lnBox, &surfBox, 0.01))
			{
				dLen += nc_Distance(pLine3d->m_dPoint[i], pLine3d->m_dPoint[i+1], 3);
				nc_VectorMinus(pLine3d->m_dPoint[i], pLine3d->m_dPoint[i+1], vec, 3);
				LineTrmSurfInt(pLine3d->m_dPoint[i],vec,pSNode->pTrmSurf,pSNode->pFacet,ptIntA,ptIntB);
				ptIntA.SortByParam();
				for(int j = 0; j < ptIntA.GetCount(); j++)
				{
					tagCCIntpt* pIntPt = ptIntA.GetPoint(j);
					dSubLen = dLen - nc_Distance(pIntPt->pos, pLine3d->m_dPoint[i+1],3);
					if(1 == pLine3d->GetLengthParam(0., dSubLen, t, dist) )
					{
						tagCCIntpt* pOutPnt = new tagCCIntpt(pLine3d, t, pIntPt->pos);
						linPntList.AddTail(pOutPnt);
					}
				}
				ptIntA.Destory();
				ptIntB.Destory();	
			}
			pSect->Destroy();
			delete pSect;
			pSect = NULL ;		
		}
	}
	if(linPntList.IsEmpty())
		return FALSE;
	return TRUE ;
}
CSmtCutPath* CSmartFeatureSlotGen::TransCur3DToCPath(CGeoCurve* pCur3D)
{
	if(pCur3D == NULL)
		return NULL ;
	DOUBLE dArcTol = GetCurveTol().m_dArcTol ;
	DOUBLE dAngTol = GetCurveTol().m_dAngTol ;
	CSmtCutPath* pCutPath = new CSmtCutPath();
	INT_PTR nType = pCur3D->GetType();
	if( nType == OBJ3D_CURVE_BSPLINE)
	{
		CStrpt* pStrPnt = pCur3D->DiscreteToStrpt(dArcTol, dAngTol*PI1/180);
		int nCount = pStrPnt->m_np ;
		for(int j=0; j < nCount; j++)
		{//连接每段中的离散点
			pCutPath->AddPoint(pStrPnt->m_ps[j]);
		}
		ClearPntStr(pStrPnt);
	}
	else 
	{
		PNT3D p1, p2 ;
		pCur3D->GetEndPoint(p1, p2);
		pCutPath->AddPoint(p1);
		pCutPath->AddPoint(p2);						
	}
	pCutPath->DelPointOnLine();
	if(pCutPath->m_nNumPnt < 2)
	{
		delete pCutPath;
		pCutPath = NULL ;
	}

	return pCutPath ;
}	
//等高截槽路径添加到PathCombine中，同时添加层间下刀(高度优先,下层路径无需反向)
void CSmartFeatureSlotGen::AddDraftPathListToPCombEx(SlotPathList& slotPathList, CPathCombine& PComb)
{
	if(slotPathList.IsEmpty())
		return ;
	BOOL bStart = TRUE ;
	BOOL bEnd = TRUE ;
	SlotPath* pSlotPath = NULL; 
	SlotPath* pSlotPathNext = NULL;
	PNT3D pt, p0, p1; 
	FPNT3D fPt0, fPt1;
	INT_PTR k = 0 ;
	TFLOAT fInc = 5.0f;
	int nPlungeType = m_cFeedDef.m_cPlungeDef.m_nPlungeType ;
	POSITION pos = slotPathList.GetHeadPosition();
	while(pos)
	{
		k++ ;
		pSlotPath = (SlotPath*)slotPathList.GetNext(pos);
		if( k == 1 && pSlotPath->bSClose == TRUE )
			bStart= FALSE ;
		if( k == 1 && pSlotPath->bEClose == TRUE)
			bEnd = FALSE ;
		if( pos)
		{
			pSlotPathNext = (SlotPath*)slotPathList.GetAt(pos);
		}
		//根据首层开闭口情况添加首层下刀。
		if( k==1 )
		{
			nc_FloatToDouble(p0, pSlotPath->pCutPath->m_pHead->m_fPoint, 3);
			nc_FloatToDouble(p1, pSlotPath->pCutPath->m_pTail->m_fPoint, 3);
			if(pSlotPathNext )
			{
				fInc = pSlotPath->pCutPath->m_pHead->m_fPoint[2] - pSlotPathNext->pCutPath->m_pHead->m_fPoint[2];
			}
			if((bStart == FALSE && bEnd == FALSE)|| nPlungeType == NCDEF_PLUNGE_RAMP)
			{
				nc_VectorCopy(pt, p0, 3);
				pt[2] = p0[2] + m_cFeedDef.m_cPlungeDef.m_dTopTol + fInc ;
				pSlotPath->pCutPath->NormalizeLen();
				CSmtCutPoint* pCutPnt = pSlotPath->pCutPath->GetPointAt(0.2f, fPt1);
				nc_FloatToDouble(p1, fPt1, 3);
				AddRampPlungeEx(PComb, p0, p1, pt, NULL, TRUE);
				pCutPnt = NULL ;
			}
			else if( bStart == TRUE)
			{
				nc_VectorCopy(pt, p0, 3);
				pt[2] = p0[2] + m_cFeedDef.m_cPlungeDef.m_dTopTol + fInc ;
				CSmtCutPath* pPlg = new CSmtCutPath;
				pPlg->m_bFeedType = JDNC_FEEDTYPE_PLUNGE;
				pPlg->AddPoint(pt);
				pPlg->AddPoint(p0);
				pPlg->AddToPathCombine(PComb);
				pPlg->ClearAllPoint();
				delete pPlg;
				pPlg = NULL ;
			}
			else
			{
				nc_VectorCopy(pt, p1, 3);
				pt[2] = p1[2] + m_cFeedDef.m_cPlungeDef.m_dTopTol + fInc ;
				CSmtCutPath* pPlg = new CSmtCutPath;
				pPlg->m_bFeedType = JDNC_FEEDTYPE_PLUNGE;
				pPlg->AddPoint(pt);
				pPlg->AddPoint(p1);
				pPlg->AddToPathCombine(PComb);
				pPlg->ClearAllPoint();
				delete pPlg;
				pPlg = NULL ;
			}
		}
		if(m_cParam.m_bZIGZAG == TRUE)
		{
			//添加上层路径
			if( pSlotPath->pCutPath != NULL )
			{
				pSlotPath->pCutPath->AddToPathCombine(PComb);
				//添加层间下刀
				if(pSlotPathNext )
				{
					if(pSlotPathNext->pCutPath->m_nLayerNo > pSlotPath->pCutPath->m_nLayerNo)
						DraftPlungeBtwnLayer(pSlotPath, pSlotPathNext, FALSE, PComb);
					else
					{
						CSmtCutPath* pPlunge = new CSmtCutPath;
						pPlunge->m_bFeedType = JDNC_FEEDTYPE_PLUNGE ;
						nc_VectorCopy(fPt0, pSlotPathNext->pCutPath->m_pHead->m_fPoint, 3);
						nc_VectorCopy(fPt1, fPt0, 3) ;
						fPt0[2] += fInc ;
						pPlunge->AddPoint(fPt0);
						pPlunge->AddPoint(fPt1);
						pPlunge->AddToPathCombine(PComb);
						pPlunge->ClearAllPoint();
						delete pPlunge ;
						pPlunge = NULL ;
					}

				}
			}
		}
		else
		{
			if(k>1)
			{
				CSmtCutPath* pPlunge = new CSmtCutPath;
				pPlunge->m_bFeedType = JDNC_FEEDTYPE_PLUNGE ;
				nc_VectorCopy(fPt0, pSlotPath->pCutPath->m_pHead->m_fPoint, 3);
				nc_VectorCopy(fPt1, fPt0, 3) ;
				fPt0[2] += fInc ;
				pPlunge->AddPoint(fPt0);
				pPlunge->AddPoint(fPt1);
				pPlunge->AddToPathCombine(PComb);
				pPlunge->ClearAllPoint();
				delete pPlunge ;
				pPlunge = NULL ;
			}
			pSlotPath->pCutPath->AddToPathCombine(PComb);
		}
		pSlotPathNext = NULL ;
	}
}

//等高截槽路径添加到PathCombine中，同时添加层间下刀
void CSmartFeatureSlotGen::AddDraftPathListToPComb(SlotPathList& slotPathList, CPathCombine& PComb)
{
	if(slotPathList.IsEmpty())
		return ;
	BOOL bStart = TRUE ;
	BOOL bEnd = TRUE ;
	BOOL bRev = FALSE ;
	SlotPath* pSlotPath = NULL; 
	SlotPath* pSlotPathNext = NULL;
	CSmtCutPath* pCutPath = NULL ;
	PNT3D pt, p0, p1; 
	FPNT3D fPt0, fPt1;
	INT_PTR k = 0 ;
	TFLOAT fInc = 0.0f;
	int nPlungeType = m_cFeedDef.m_cPlungeDef.m_nPlungeType ;
	POSITION pos = slotPathList.GetHeadPosition();
	while(pos)
	{
		k++ ;
		pSlotPath = (SlotPath*)slotPathList.GetNext(pos);
		if( k == 1 && pSlotPath->bSClose == TRUE )
			bStart= FALSE ;
		if( k == 1 && pSlotPath->bEClose == TRUE)
			bEnd = FALSE ;
		if( pos)
		{
			pSlotPathNext = (SlotPath*)slotPathList.GetAt(pos);
		}
		//根据首层开闭口情况添加首层下刀。
		if( k==1 )
		{
			if( pSlotPathNext == NULL )
				return ;
			nc_FloatToDouble(p0, pSlotPath->pCutPath->m_pHead->m_fPoint, 3);
			nc_FloatToDouble(p1, pSlotPath->pCutPath->m_pTail->m_fPoint, 3);
			fInc = pSlotPath->pCutPath->m_pHead->m_fPoint[2] - pSlotPathNext->pCutPath->m_pHead->m_fPoint[2];
			if((bStart == FALSE && bEnd == FALSE)|| nPlungeType == NCDEF_PLUNGE_RAMP)
			{
				nc_VectorCopy(pt, p0, 3);
				pt[2] = p0[2] + m_cFeedDef.m_cPlungeDef.m_dTopTol + fInc ;
				pSlotPath->pCutPath->NormalizeLen();
				CSmtCutPoint* pCutPnt = pSlotPath->pCutPath->GetPointAt(0.2f, fPt1);
				nc_FloatToDouble(p1, fPt1, 3);
				AddRampPlungeEx(PComb, p0, p1, pt, NULL, TRUE);
				bStart = TRUE ;
				pCutPnt = NULL ;
			}
			else if( bStart == TRUE)
			{
				nc_VectorCopy(pt, p0, 3);
				pt[2] = p0[2] + m_cFeedDef.m_cPlungeDef.m_dTopTol + fInc ;
				CSmtCutPath* pPlg = new CSmtCutPath;
				pPlg->m_bFeedType = JDNC_FEEDTYPE_PLUNGE;
				pPlg->AddPoint(pt);
				pPlg->AddPoint(p0);
				pPlg->AddToPathCombine(PComb);
				pPlg->ClearAllPoint();
				delete pPlg;
				pPlg = NULL ;
			}
			else
			{
				nc_VectorCopy(pt, p1, 3);
				pt[2] = p1[2] + m_cFeedDef.m_cPlungeDef.m_dTopTol + fInc ;
				CSmtCutPath* pPlg = new CSmtCutPath;
				pPlg->m_bFeedType = JDNC_FEEDTYPE_PLUNGE;
				pPlg->AddPoint(pt);
				pPlg->AddPoint(p1);
				pPlg->AddToPathCombine(PComb);
				pPlg->ClearAllPoint();
				delete pPlg;
				pPlg = NULL ;
			}
		}
		if(m_cParam.m_bZIGZAG == TRUE)
		{
			if(bStart == TRUE)
			{
				if( k%2 )
				{
					bRev = FALSE;
				}
				else
				{
					bRev = TRUE ;
				}	
			}
			else
			{
				if( k%2 )
				{
					bRev = TRUE ;
				}
				else
				{
					bRev = FALSE ;
				}
			}
			//添加上层路径
			if( pSlotPath->pCutPath != NULL )
			{
				pCutPath = pSlotPath->pCutPath->CopyMyself();
				if(bRev)
				{
					pCutPath->ReverseDirect();
				}
				pCutPath->AddToPathCombine(PComb);
				//添加层间下刀
				if(pSlotPathNext )
				{
					if(pSlotPathNext->pCutPath->m_nLayerNo > pCutPath->m_nLayerNo)
						DraftPlungeBtwnLayer(pSlotPath, pSlotPathNext, bRev, PComb);
					else
					{
						CSmtCutPath* pPlunge = new CSmtCutPath;
						pPlunge->m_bFeedType = JDNC_FEEDTYPE_PLUNGE ;
						nc_VectorCopy(fPt0, pSlotPathNext->pCutPath->m_pHead->m_fPoint, 3);
						nc_VectorCopy(fPt1, fPt0, 3) ;
						fPt0[2] += fInc ;
						pPlunge->AddPoint(fPt0);
						pPlunge->AddPoint(fPt1);
						pPlunge->AddToPathCombine(PComb);
						pPlunge->ClearAllPoint();
						delete pPlunge ;
						pPlunge = NULL ;
					}

				}
				pCutPath->ClearAllPoint();
				delete pCutPath ;
			}
			pSlotPathNext = NULL ;
		}
		else
		{
			pCutPath = pSlotPath->pCutPath->CopyMyself();
			if(bStart == FALSE)
			{
				pCutPath->ReverseDirect();
			}
			if(k>1)
			{
				CSmtCutPath* pPlunge = new CSmtCutPath;
				pPlunge->m_bFeedType = JDNC_FEEDTYPE_PLUNGE ;
				nc_VectorCopy(fPt0, pCutPath->m_pHead->m_fPoint, 3);
				nc_VectorCopy(fPt1, fPt0, 3) ;
				fPt0[2] += fInc ;
				pPlunge->AddPoint(fPt0);
				pPlunge->AddPoint(fPt1);
				pPlunge->AddToPathCombine(PComb);
				pPlunge->ClearAllPoint();
				delete pPlunge ;
				pPlunge = NULL ;
			}
			pCutPath->AddToPathCombine(PComb);
			pCutPath->ClearAllPoint();
			delete pCutPath ;
			pCutPath = NULL ;
		}
	}
}
//生成两条分层路径间的下刀路径,输入两层CutPath,输出生成的下刀路径PathComb
void  CSmartFeatureSlotGen::DraftPlungeBtwnLayer( SlotPath* pPrePath, 
												  SlotPath* pNxtPath, 
												  BOOL bRev,           /*下层路径反向标记*/
												  CPathCombine& pComb)
{
	if( pPrePath == NULL || pNxtPath == NULL)
		return ;

	FPNT3D fPnt1, fPnt2;
	if(bRev)
	{
		nc_VectorCopy(fPnt1, pPrePath->pCutPath->m_pHead->m_fPoint, 3);
		nc_VectorCopy(fPnt2, pNxtPath->pCutPath->m_pHead->m_fPoint, 3);
	}
	else
	{
		nc_VectorCopy(fPnt1, pPrePath->pCutPath->m_pTail->m_fPoint, 3);
		nc_VectorCopy(fPnt2, pNxtPath->pCutPath->m_pTail->m_fPoint, 3);
	}
	CSmtCutPath* pPlunge = new CSmtCutPath;
	pPlunge->m_bFeedType = JDNC_FEEDTYPE_PLUNGE ;
	DOUBLE dTol = tan(PI1/36.0);
	DOUBLE dDepthInc = m_cFeedDef.m_cLayerDef.m_dSideDInc;
	dTol *= dDepthInc ;
	if(nc_Distance(fPnt1, fPnt2, 2) > dTol )//GetSurfaceTol().m_dArcTol /*m_pTool->m_fRadius*/
	{//距离较大时可能是台阶面不能直接连接上下层
		FPNT3D fConPnt;
		nc_VectorCopy(fConPnt, fPnt2, 3);
		fConPnt[2] = fPnt1[2] ;
		pPlunge->AddPoint(fPnt1);
		pPlunge->AddPoint(fConPnt);
		pPlunge->AddPoint(fPnt2);
	}
	else
	{
		pPlunge->AddPoint(fPnt1);
		pPlunge->AddPoint(fPnt2);
	}

	pPlunge->AddToPathCombine(pComb);
	pPlunge->ClearAllPoint();
	delete pPlunge ;
	pPlunge = NULL ;
}


//是否有非工件面加入检查模型
BOOL CSmartFeatureSlotGen::IsReuildCheckMdl(CEttFeatureSlot* pEntSlot, CSmtCheckMdl* pMdl, RFRAME& frm )
{
	BOOL bReturn = FALSE;
	if( pEntSlot == NULL || pMdl == NULL )
		return bReturn;
	//把槽特征的侧壁面添加到检查模型中
	if(pEntSlot->m_SideList0.GetCount() > 1 )
	{
		if( pEntSlot->m_pSideSurf0)	
		{
			CGeoSurf* pSurf = pEntSlot->m_pSideSurf0->GetSurf();
			AddSurfToCheckMdl(pMdl, pSurf, frm);
		}
		bReturn = TRUE ;
	}
	if(pEntSlot->m_SideList1.GetCount() > 1 )
	{
	
		if(pEntSlot->m_pSideSurf1)
		{
			CGeoSurf* pSurf = pEntSlot->m_pSideSurf1->GetSurf();
			AddSurfToCheckMdl(pMdl, pSurf, frm);
		}
		bReturn = TRUE ;
	}
	//
	//如果槽由两组线定义，则用两线分别构造拉伸面并添加到检查模型中
	if( pEntSlot->m_pSideSurf0 == NULL && pEntSlot->m_pSideSurf1 == NULL &&
		pEntSlot->m_pSideCurve0 && pEntSlot->m_pSideCurve1 )
	{
		CGeoCurveList   sideCurList ;
		CGeoTrmSurfList draftSurList ;
		CGeoCurveList CurveListTop, CurveListBtm;
		//构造拉伸面参数
		SURFDRAFT_PARAM  es ;
		es.m_dHeight = pEntSlot->m_dWidthMax ;
		es.m_dAngle = 0.0 ;
		es.m_dirAxis[2] = -frm.Z[2] ;
		es.m_bFlipDir = FALSE ;
		es.m_bFlipAng = FALSE ;
		es.m_bBidir = FALSE ;
		es.m_bCapTop = es.m_bCapDown = FALSE ;
		es.m_bStitch = FALSE ;
		es.m_nCornerType = 0 ;
		es.m_bGrpSurf = FALSE ;
		sideCurList.AddTail(pEntSlot->m_pSideCurve0->GetCurve());
		sideCurList.AddTail(pEntSlot->m_pSideCurve1->GetCurve());
		CSurfDraft creator ;
		creator.SurfDraft_Dir(&sideCurList, es, &draftSurList, &CurveListTop, &CurveListBtm) ;
		DeltGeoCurveList( CurveListTop ) ;
		DeltGeoCurveList( CurveListBtm ) ;
		sideCurList.RemoveAll();
		if(draftSurList.GetHead() != NULL)
		{
			AddSurfToCheckMdl(pMdl, draftSurList.GetHead(), frm);
		}
		if(draftSurList.GetTail() != NULL)
		{
			AddSurfToCheckMdl(pMdl, draftSurList.GetTail(), frm);
		}
		bReturn = TRUE ;
		ClearSurfList(draftSurList);
	}
	return bReturn ;
}

void CSmartFeatureSlotGen::AddSurfToCheckMdl(CSmtCheckMdl* pMdl, CGeoSurf* pSurf, RFRAME& frm )
{
	if(pMdl == NULL || pSurf == NULL)
		return;
	CSmtChkArray tmpArr ;
	CNcDiscParam	cDiscParam;
	cDiscParam.m_dArcTol = m_cSetupDef.m_cTolDef.m_dArcTol;
	cDiscParam.m_dAngTol = ANGLE_TO_RADIAN( m_cSetupDef.m_cTolDef.m_dAngTol ) ;
	cDiscParam.m_bPerturbVert=false;
	DOUBLE dMaxCos = cos( ANGLE_TO_RADIAN( 1.0 ) ) ;
	POSITION atPos = NULL ;
	DOUBLE    FltBox[2][3];
	CPtrList		allFacet ; 
	for( INT_PTR i = 0 ; i < 3 ; i++ )
	{
		FltBox[0][i] = pSurf->m_dBox.min[i];
		FltBox[1][i] = pSurf->m_dBox.max[i];
	}
	MathGeo_DiscToFacet( pSurf, cDiscParam, allFacet ) ;
	atPos = allFacet.GetHeadPosition() ;
	while( atPos )
	{
		CFacet* pFacet  = ( CFacet*) allFacet.GetNext( atPos ) ;
		if( pFacet == NULL ) continue ;
		if( pSurf->GetType() == OBJ3D_SURFACE_GEO )
		{
			pFacet->surface = pSurf ; 
		}
		pFacet->TransWorld( &frm ) ;
		CSmartCheck* pCheck = MathCAM_ConvertToCheckEx( *pFacet, TRUE, dMaxCos, FltBox ) ;
		delete pFacet ;
		if( pCheck ) 
		{
			pCheck->m_bCheckFlag = 200 ;
			pCheck->DefineBox() ;
			pMdl->AddCheck(pCheck);
		}
	}
	allFacet.RemoveAll() ;
}
void SortAllSlotPComb( CPathCombineList& AllComb, int SortType  )
{
	if(AllComb.IsEmpty())
		return ;
	if(AllComb.GetCount() < 2)
		return ;
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
			pPComb = AllComb.RemoveHead() ;
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
	}
	else if( SortType == NCDEF_SORTTYPE_MINDIST )
	{
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
				pPComb->GetEndPoint( 0, dStart ) ;
				dDist = nc_Distance( dStart, dEnd, 2 ) ;
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
	}
}
int CSmartFeatureSlotGen::GeneratePathEx( CPathGroup&   NewPath ,  /*雕刻路径*/
										 CSmartGraphic& Graph  )  /*雕刻图形*/
{
	if(!UpdateTool2D())
		return 0 ;
	CEttPModelEx* pPModel= SurfNC_GetCurPathModel() ;
	if( ! pPModel )
		return 0;
	CJDTPlaneLib* pPLib = pPModel->GetTPlaneLib() ;
	if( ! pPLib )
		return 0;
	CJDTPlane * pCurrTPlane = pPLib->GetActiveTPlane();
	if( ! pCurrTPlane )
		return 0;
	//先搜索全部路径判断是否需要重建检查模型
	BOOL bPartChange = FALSE ;
	CPtrArray	AllPath;
	pPModel->EnumAllPComb( AllPath, pCurrTPlane->GetPlaneID());
	INT_PTR n = AllPath.GetSize();
	for (INT_PTR j=0; j<n; j++)
	{
		CEttPCombEx * pPComb = (CEttPCombEx *)AllPath[j];
		if (pPComb == NULL) 
			continue;

		CJDParameter* pMethod = pPComb->GetParameter(); 
		if (pMethod == NULL) continue;
		if( pMethod->GetMethodType() == surfncMethodFeatureSlot && 
			pMethod->m_cMethodDef.m_cFeatureSlotCut.m_bRebuildMdlFlag == 1)
		{
			pMethod->m_cMethodDef.m_cFeatureSlotCut.m_bRebuildMdlFlag = 0 ;
			bPartChange = TRUE ;
		}
	}
	RFRAME tpFrm ;
	SurfNC_GetTPlaneFrame(pCurrTPlane, tpFrm, pPModel);
	//Step1:构建检查模型
	CSmtCheckMdl* pDriveMdl = pCurrTPlane->m_cVerifyMdl.GetCheckModel();
	if(bPartChange)//工件形状信息改变
	{
		if(pDriveMdl)
		{
			pCurrTPlane->m_cVerifyMdl.DeleteCheckModel();
		}
		pDriveMdl = NULL ;
	}
	if( pDriveMdl == NULL )
	{
		pDriveMdl = new CSmtCheckMdl;  
		if( !Graph.BuildCheckModel(m_cSetupDef, *pDriveMdl, NCDEF_SURF_CHECK) )
		{
			delete pDriveMdl;
			m_nErrorType = JDERROR_GENPATH_NOSURFACE  ;
			return 0 ; 
		}
		pCurrTPlane->m_cVerifyMdl.SetCheckModel(pDriveMdl);		
	}	
	
	if( pDriveMdl == NULL )
		return 0 ;
	INT_PTR i = 0 ;
	i = pDriveMdl->m_cAllChk.GetCount();//test
	//Step2:得到槽特征生成路径
	CPtrList allSlot ;
	Graph.GetAllEntity(m_cSetupDef, allSlot, SMARTGRAPH_TYPE_FEATURESLOT);
	if( allSlot.GetSize() < 1 )
	{
		m_nErrorType = JDERROR_GENPATH_NOAREA;
		return 0 ;
	}
	BOOL bReMdl = FALSE ; 
	CEntity3D* pEnt = NULL;
	CEttFeatureSlot* pEntSlot = NULL ;
	CGeoCurve* pMidCur3D = NULL ;
	CSmartCurve* pMidCur2D = NULL ;
	INT_PTR nType = 0 ;
	CPathCombineList PathCombList;
	POSITION posSlot = allSlot.GetHeadPosition();
	while(posSlot)
	{
		ProcessNewFunc( m_cPrgDef, IDS_PATHGEN_STEP_FEATURESLOTPATH ) ;	//定义进度条
		pEnt = (CEntity3D*)allSlot.GetNext(posSlot);
		nType = pEnt->GetType();
		if( nType != ETTYPE_FEATURESLOT )
			continue;
		pEntSlot = (CEttFeatureSlot*)pEnt;
		if(pEntSlot->m_pMidCurve == NULL)
			continue ;
		DOUBLE dTopWidth = pEntSlot->m_dWidthMax+0.01 ;
		int nTempWidth = int(dTopWidth * 100);
		dTopWidth = nTempWidth * 0.01 ; 
		if( m_pTool->m_fRadius*2 > dTopWidth )//刀具半径过大
		{
			m_nErrorType = JDERROR_FEATURESLOT_TOOLERROR ;
			return 0;
		}
		bReMdl = FALSE ;
		bReMdl = IsReuildCheckMdl(pEntSlot, pDriveMdl, tpFrm);
		//生成每个槽的路径
		pMidCur3D = pEntSlot->m_pMidCurve->GetCurve();
		// 空间曲线转换成平面曲线,并变换到加工平面
		pMidCur2D = MathCAM_Curve3DToCurve2D( pMidCur3D, m_cSetupDef.m_cTolDef, &tpFrm, FALSE ) ;
		CPathCombine * pPComb = new CPathCombine( NC_WPROCESS_ROUGH ) ;
		pPComb->m_pEntFrom  = NewPath.m_pEntFrom  ;
		//如果槽由两组线定义，则用单线划槽
		if( pEntSlot->m_pSideSurf0 == NULL && pEntSlot->m_pSideSurf1 == NULL &&
			pEntSlot->m_pSideCurve0 && pEntSlot->m_pSideCurve1 )
		{
			OneSlotPath(*pPComb, pDriveMdl, pMidCur2D, TRUE, pEntSlot->m_dWidthMax+0.01, m_pTool->m_fRadius*2-0.001, m_cPrgDef);
		}
		else
		{
			if(m_cParam.m_nMethodType == NCDEF_FEATURESLOT_CURVE)
			{
				OneSlotPath(*pPComb, pDriveMdl, pMidCur2D, TRUE, pEntSlot->m_dWidthMax+0.01, m_pTool->m_fRadius*2-0.001, m_cPrgDef);
			}
			else if(m_cParam.m_nMethodType == NCDEF_FEATURESLOT_ZHEIGHT)
			{
				if( pEntSlot->m_dWidthMax - m_pTool->m_fRadius*2 < 0.01)//刀具半径合适
				{//单线划槽
					OneSlotPath(*pPComb, pDriveMdl, pMidCur2D, TRUE, pEntSlot->m_dWidthMax+0.01, m_pTool->m_fRadius*2-0.001, m_cPrgDef);
				}
				else
				{//等高截线
					if(pEntSlot->m_dWidthMin > m_pTool->m_fRadius * 6.0 )//槽宽间距较大
					{
						if(m_cSetupDef.m_cOrderDef.m_nLayerOrd == NCDEF_LAYERORD_REGION)
						{
							OneDraftSlotPath(*pPComb, pDriveMdl, pMidCur2D, pEntSlot->m_SideList0,
								pEntSlot->m_pSideSurf0, pEntSlot->m_dWidthMax, m_cPrgDef);
							OneDraftSlotPath(*pPComb, pDriveMdl, pMidCur2D, pEntSlot->m_SideList1, 
								pEntSlot->m_pSideSurf1, pEntSlot->m_dWidthMax,m_cPrgDef);
						}
						else if(m_cSetupDef.m_cOrderDef.m_nLayerOrd == NCDEF_LAYERORD_HEIGHT)
						{
							OneDraftSlotPathEx( *pPComb, pDriveMdl, pMidCur2D, 
								pEntSlot->m_SideList0, pEntSlot->m_SideList1, 
								pEntSlot->m_pSideSurf0,
								pEntSlot->m_pSideSurf1,
								pEntSlot->m_dWidthMax,
								m_cPrgDef);
						}
					}
					else
					{
						OneDraftSlotPathEx( *pPComb, pDriveMdl, pMidCur2D, 
							pEntSlot->m_SideList0, pEntSlot->m_SideList1, 
							pEntSlot->m_pSideSurf0,
							pEntSlot->m_pSideSurf1,
							pEntSlot->m_dWidthMax,
							m_cPrgDef);
					}
				}
			}				
		}
		if( pMidCur2D )
			delete pMidCur2D ;
		pMidCur2D = NULL ;
		PathCombList.AddTail(pPComb);
		//NewPath.AddData(0., pPComb, TRUE);
		//删掉checkmdl中的临时信息
		if(bReMdl == TRUE)
		{
			for(i = 0 ; i < pDriveMdl->m_cAllChk.GetSize(); /* i++*/)
			{
				CSmartCheck* pCheck = pDriveMdl->m_cAllChk.GetAt(i);
				if( pCheck && pCheck->m_bCheckFlag == 200 )
				{
					pDriveMdl->DeleteCheck(pCheck);
				}
				else
				{
					i++;
				}
			}
		}
	}

	if(SurfNC_IsAbort())
	{
		while(PathCombList.GetCount())
		{
			CPathCombine* pComb = PathCombList.RemoveHead();
			if(pComb)
				delete pComb;
			pComb = NULL ;
		}
		m_nErrorType = JDERROR_GENPATH_ABORT  ;
		return 0 ;
	}
	
	if(PathCombList.GetCount() > 1)//路径排序
	{	
		SortAllSlotPComb( PathCombList, GetSortType()) ;
	}
	while(PathCombList.GetCount())
	{
		CPathCombine* pComb = PathCombList.RemoveHead();
		NewPath.AddData(0., pComb, TRUE);
	}

	allSlot.RemoveAll();
	return 1 ;
}

void CSmartFeatureSlotGen::PlungeBtwnLayer(SlotPath* pPrePath, SlotPath* pNxtPath, 
										   CGeoCurve* pBtmCur,BOOL bRev, CPathCombine& pPComb)

{
	if( pPrePath == NULL || pNxtPath == NULL )
		return;
	if( pPrePath->pCutPath == NULL || pNxtPath->pCutPath == NULL )
		return ;
	if( pBtmCur == NULL )
		return;
	if(pNxtPath->pCutPath->m_nLayerNo <= pPrePath->pCutPath->m_nLayerNo)
		return;
	CEttPModelEx* pPModel= SurfNC_GetCurPathModel() ;
	if( ! pPModel )
		return ;

	CJDTPlaneLib* pPLib = pPModel->GetTPlaneLib() ;
	if( ! pPLib )
		return ;
	CJDTPlane * pCurrTPlane = pPLib->GetActiveTPlane();
	if( ! pCurrTPlane )
		return ;
	CGeoInter geoInter;
	PNT3D st, ed;
	CCurIntpt intpt1, intpt2; 
	int nPlungeType = m_cFeedDef.m_cPlungeDef.m_nPlungeType;
	RFRAME tpFrm ;
	SurfNC_GetTPlaneFrame(pCurrTPlane, tpFrm, pPModel);

	BOOL bVertical = FALSE ;
	PNT3D htPnt;//折线下刀线段的高度
	PNT3D p0, p1 ;
	if( !bRev )
	{
		nc_FloatToDouble(htPnt, pPrePath->pCutPath->m_pTail->m_fPoint, 3);
		if( pNxtPath->bEClose == TRUE || nPlungeType == NCDEF_PLUNGE_RAMP )
		{
			nc_FloatToDouble(p1, pNxtPath->pCutPath->m_pHead->m_fPoint, 3);
			nc_FloatToDouble(p0, pNxtPath->pCutPath->m_pTail->m_fPoint, 3);
			if( nc_Distance(pPrePath->pCutPath->m_pTail->m_fPoint, pNxtPath->pCutPath->m_pTail->m_fPoint, 2) < NC_PATH_ERROR)
			{//如果封闭端面为直壁		
				bVertical = TRUE ;
			}
			else if(nc_Distance(pPrePath->pCutPath->m_pTail->m_fPoint, pNxtPath->pCutPath->m_pTail->m_fPoint, 2) > m_pTool->m_fRadius)
			{//如果上下两层路径为台阶
				CSmtCutPoint* pConPnt1 = pPrePath->pCutPath->m_pTail->CopyMyself();
				CSmtCutPoint* pConPnt2 = pNxtPath->pCutPath->m_pTail->CopyMyself();
				pConPnt2->m_fPoint[2] = pConPnt1->m_fPoint[2] ;
				nc_FloatToDouble(st, pConPnt1->m_fPoint, 3);
				nc_FloatToDouble(ed, pConPnt2->m_fPoint, 3);
				if( !geoInter.CalcLineCurveAllIntpt(st, ed, pBtmCur,MIN_DIS, &intpt1, &intpt2) || intpt1.GetCount() < 2)
				{
					CSmtCutPath* pConnect = new CSmtCutPath();
					pConnect->m_bFeedType = JDNC_FEEDTYPE_CONNECT;
					pConnect->AddTail(pConPnt1);
					pConnect->AddTail(pConPnt2);
					pConnect->AddToPathCombine(pPComb);
					pConnect->ClearAllPoint();
					delete pConnect;
					pConnect = NULL ;
				}
				else
				{
					if(pConPnt1 != NULL) delete pConPnt1;
					if(pConPnt2 != NULL) delete pConPnt2;
				}
				pConPnt1 = NULL ;
				pConPnt2 = NULL ;
				intpt1.Destory();
				intpt2.Destory();
				htPnt[0] = p0[0] ;
				htPnt[1] = p0[1] ;
				bVertical = TRUE ;
			}
			AddRampPlungeEx(pPComb, p0, p1, htPnt, pBtmCur, bVertical);
		}
		else// if( pNxtPath->bEClose == FALSE )
		{
			CSmtCutPath* pPlunge = new CSmtCutPath();
			pPlunge->m_bFeedType = JDNC_FEEDTYPE_PLUNGE;
			CSmtCutPoint* pPlgPnt1 = pPrePath->pCutPath->m_pTail->CopyMyself();
			CSmtCutPoint* pPlgPnt2 = pNxtPath->pCutPath->m_pTail->CopyMyself();
			pPlunge->AddTail(pPlgPnt1);
			pPlunge->AddTail(pPlgPnt2);
			if(nc_Distance(pPlgPnt1->m_fPoint, pPlgPnt2->m_fPoint, 2) >0.01)
			{
				CSmtCutPoint* pNewPnt = pPlgPnt2->CopyMyself();
				pNewPnt->m_fPoint[2] = pPlgPnt1->m_fPoint[2];
				pPlunge->InsertBefore(pNewPnt, pPlgPnt2);
			}
			pPlunge->AddToPathCombine(pPComb);
			pPlunge->ClearAllPoint();
			delete pPlunge;
			pPlunge = NULL ;
		}
	}
	else
	{
		nc_FloatToDouble(htPnt, pPrePath->pCutPath->m_pHead->m_fPoint, 3);
		if( pNxtPath->bSClose == TRUE || nPlungeType == NCDEF_PLUNGE_RAMP )
		{
			nc_FloatToDouble(p0, pNxtPath->pCutPath->m_pHead->m_fPoint, 3);
			nc_FloatToDouble(p1, pNxtPath->pCutPath->m_pTail->m_fPoint, 3);
			if(nc_Distance(pPrePath->pCutPath->m_pHead->m_fPoint, pNxtPath->pCutPath->m_pHead->m_fPoint, 2) < NC_PATH_ERROR)
			{
				bVertical = TRUE ;
			}
			else if(nc_Distance(pPrePath->pCutPath->m_pHead->m_fPoint, pNxtPath->pCutPath->m_pHead->m_fPoint, 2) > m_pTool->m_fRadius)
			{
				CSmtCutPoint* pConPnt1 = pPrePath->pCutPath->m_pHead->CopyMyself();
				CSmtCutPoint* pConPnt2 = pNxtPath->pCutPath->m_pHead->CopyMyself();
				pConPnt2->m_fPoint[2] = pConPnt1->m_fPoint[2] ;
				nc_FloatToDouble(st, pConPnt1->m_fPoint, 3);
				nc_FloatToDouble(ed, pConPnt2->m_fPoint, 3);
				if( !geoInter.CalcLineCurveAllIntpt(st, ed, pBtmCur,MIN_DIS, &intpt1, &intpt2) || intpt1.GetCount() < 2)
				{
					CSmtCutPath* pConnect = new CSmtCutPath();
					pConnect->m_bFeedType = JDNC_FEEDTYPE_CONNECT;
					pConnect->AddTail(pConPnt1);
					pConnect->AddTail(pConPnt2);
					pConnect->AddToPathCombine(pPComb);
					pConnect->ClearAllPoint();
					delete pConnect;
					pConnect = NULL ;
				}
				else
				{
					if(pConPnt1 != NULL) delete pConPnt1;
					if(pConPnt2 != NULL) delete pConPnt2;
				}
				pConPnt1 = NULL ;
				pConPnt2 = NULL ;
				intpt1.Destory();
				intpt2.Destory();
				htPnt[0] = p0[0] ;
				htPnt[1] = p0[1] ;
				bVertical = TRUE ;
			}
			AddRampPlungeEx(pPComb, p0, p1, htPnt, pBtmCur, bVertical);
		}
		else //if( pNxtPath->bSClose == FALSE)
		{
			CSmtCutPath* pPlunge = new CSmtCutPath();
			pPlunge->m_bFeedType = JDNC_FEEDTYPE_PLUNGE;
			CSmtCutPoint* pPlgPnt1 = pPrePath->pCutPath->m_pHead->CopyMyself();
			pPlunge->AddTail(pPlgPnt1);
			CSmtCutPoint* pPlgPnt2 = pNxtPath->pCutPath->m_pHead->CopyMyself();
			pPlunge->AddTail(pPlgPnt2);
			if(nc_Distance(pPlgPnt1->m_fPoint, pPlgPnt2->m_fPoint, 2) > 0.01)
			{
				CSmtCutPoint* pNewPnt = pPlgPnt2->CopyMyself();
				pNewPnt->m_fPoint[2] = pPlgPnt1->m_fPoint[2];
				pPlunge->InsertBefore(pNewPnt, pPlgPnt2);
			}
			pPlunge->AddToPathCombine(pPComb);
			pPlunge->ClearAllPoint();
			delete pPlunge;
			pPlunge = NULL ;
		}
	}
}
void CSmartFeatureSlotGen::PlungeBtwnLayerEx( SlotPathList& pLayerList1, SlotPathList& pLayerList2, 
											 CGeoCurve* pBtmCur, BOOL bRev, CPathCombine& pPComb )

{
	if( pLayerList1.IsEmpty() || pLayerList2.IsEmpty() )
		return ;
	if( pBtmCur == NULL )
		return ;
	CEttPModelEx* pPModel= SurfNC_GetCurPathModel() ;
	if( ! pPModel )
		return ;

	CJDTPlaneLib* pPLib = pPModel->GetTPlaneLib() ;
	if( ! pPLib )
		return ;
	CJDTPlane * pCurrTPlane = pPLib->GetActiveTPlane();
	if( ! pCurrTPlane )
		return ;
	SlotPath* pHead1 = pLayerList1.GetHead();
	SlotPath* pTail1 = pLayerList1.GetTail();
	SlotPath* pHead2 = pLayerList2.GetHead();
	SlotPath* pTail2 = pLayerList2.GetTail();
	if( !pHead1 || !pHead2 || !pTail1 || !pTail2 )
		return ;
	int nPlungeType = m_cFeedDef.m_cPlungeDef.m_nPlungeType ;
	BOOL bVertical = FALSE ;
	PNT3D htPnt;//折线下刀线段的高度
	PNT3D p0, p1 ;
	RFRAME tpFrm ;
	SurfNC_GetTPlaneFrame(pCurrTPlane, tpFrm, pPModel);

	if( !bRev )
	{
		nc_FloatToDouble(htPnt, pTail1->pCutPath->m_pTail->m_fPoint, 3);
		if( pTail2->bEClose == TRUE || nPlungeType == NCDEF_PLUNGE_RAMP)
		{
			nc_FloatToDouble(p1, pHead2->pCutPath->m_pHead->m_fPoint, 3);
			nc_FloatToDouble(p0, pTail2->pCutPath->m_pTail->m_fPoint, 3);
			if( nc_Distance(htPnt, p0, 2) < NC_PATH_ERROR)
			{//如果封闭端面为直壁		
				bVertical = TRUE ;
			}
			AddRampPlungeEx(pPComb, p0, p1, htPnt, pBtmCur, bVertical);
		}
		else //if( pTail2->bEClose == FALSE )
		{
			CSmtCutPath* pPlunge = new CSmtCutPath();
			pPlunge->m_bFeedType = JDNC_FEEDTYPE_PLUNGE;
			CSmtCutPoint* pPlgPnt1 = pTail1->pCutPath->m_pTail->CopyMyself();
			CSmtCutPoint* pPlgPnt2 = pTail2->pCutPath->m_pTail->CopyMyself();
			pPlunge->AddTail(pPlgPnt1);
			pPlunge->AddTail(pPlgPnt2);
			if(nc_Distance(pPlgPnt1->m_fPoint, pPlgPnt2->m_fPoint, 2) > 0.01)
			{
				CSmtCutPoint* pNewPnt = pPlgPnt2->CopyMyself();
				pNewPnt->m_fPoint[2] = pPlgPnt1->m_fPoint[2];
				pPlunge->InsertBefore(pNewPnt, pPlgPnt2);
			}
			pPlunge->AddToPathCombine(pPComb);
			pPlunge->ClearAllPoint();
			delete pPlunge;
			pPlunge = NULL ;
		}
	}
	else
	{
		nc_FloatToDouble(htPnt, pHead1->pCutPath->m_pHead->m_fPoint, 3);
		if( pHead2->bSClose == TRUE || nPlungeType == NCDEF_PLUNGE_RAMP )
		{
			nc_FloatToDouble(p0, pHead2->pCutPath->m_pHead->m_fPoint, 3);
			nc_FloatToDouble(p1, pTail2->pCutPath->m_pTail->m_fPoint, 3);
			if(nc_Distance(htPnt, p0, 2) < NC_PATH_ERROR)
			{
				bVertical = TRUE ;
			}
			AddRampPlungeEx(pPComb, p0, p1, htPnt, pBtmCur, bVertical);
		}
		else// if( pHead2->bSClose == FALSE)
		{
			CSmtCutPath* pPlunge = new CSmtCutPath();
			pPlunge->m_bFeedType = JDNC_FEEDTYPE_PLUNGE;
			CSmtCutPoint* pPlgPnt1 = pHead1->pCutPath->m_pHead->CopyMyself();
			pPlunge->AddTail(pPlgPnt1);
			CSmtCutPoint* pPlgPnt2 = pHead2->pCutPath->m_pHead->CopyMyself();
			pPlunge->AddTail(pPlgPnt2);
			if(nc_Distance(pPlgPnt1->m_fPoint, pPlgPnt2->m_fPoint, 2) >m_pTool->m_fRadius)
			{
				CSmtCutPoint* pNewPnt = pPlgPnt2->CopyMyself();
				pNewPnt->m_fPoint[2] = pPlgPnt1->m_fPoint[2];
				pPlunge->InsertBefore(pNewPnt, pPlgPnt2);
			}
			pPlunge->AddToPathCombine(pPComb);
			pPlunge->ClearAllPoint();
			delete pPlunge;
			pPlunge = NULL ;
		}
	}
}
int CSmartFeatureSlotGen::AddRampPlungeEx(CPathCombine& PComb, PNT3D pt0, PNT3D pt1, PNT3D pt,
										  CGeoCurve* pBtmCur,  BOOL bVertical )
{
	VEC2D vec;
	vec[0] = pt1[0] - pt0[0];
	vec[1] = pt1[1] - pt0[1];
	nc_Normalize(vec, 2);
	DOUBLE dCurveLen = m_cFeedDef.m_cPlungeDef.m_dRadius ;
	if( /*(nc_Distance(pt0, pt1, 3) > 0.1) && */(dCurveLen> nc_Distance(pt0, pt1, 3)) )
	{
		dCurveLen = nc_Distance(pt0, pt1, 3);
	}
	DOUBLE dTan = tan( ANGLE_TO_RADIAN( m_cFeedDef.m_cPlungeDef.m_dAngle)) ;
	DOUBLE dMaxTan = m_cFeedDef.m_cPlungeDef.m_dMaxZInc / (dCurveLen * 2.0);
	if( dTan > dMaxTan ) dTan = dMaxTan ;

	DOUBLE dDepthInc = dCurveLen * dTan * 2.0;//分层折线深度
	int nLoopCnt = (int)(ceil((pt[2] - pt0[2])/dDepthInc));
	if( nLoopCnt > 1 )
	{
		dDepthInc = (pt[2] - pt0[2]) / nLoopCnt ; //根据层数重算分层深度
	}

	if(bVertical == TRUE)
	{//生成折线
		dDepthInc = dDepthInc * 0.5 ;
		CPathPLine3D * pLine3D = new CPathPLine3D() ;
		pLine3D->m_bFeedType = JDNC_FEEDTYPE_PLUNGE ;
		pLine3D->m_pTAPos = new TPNT3D[ nLoopCnt * 2 + 1 ] ;
		pLine3D->m_nCount = nLoopCnt * 2  ;
		nc_VectorCopy(pLine3D->m_pTAPos[0], pt, 3);
		nc_VectorCopy(pLine3D->m_pTAPos[nLoopCnt * 2], pt0, 3);
		for(int j = 1; j < pLine3D->m_nCount ; j++ )
		{
			if(j % 2)
			{
				pLine3D->m_pTAPos[j][0] = pt[0] + vec[0] * dCurveLen;
				pLine3D->m_pTAPos[j][1] = pt[1] + vec[1] * dCurveLen;
				pLine3D->m_pTAPos[j][2] = pt[2] - dDepthInc*(j-0.5); 
			}
			else
			{
				pLine3D->m_pTAPos[j][0] = pt[0] ;
				pLine3D->m_pTAPos[j][1] = pt[1] ;
				pLine3D->m_pTAPos[j][2] = pt[2] - dDepthInc * j; 
			}
		}
		PComb.AddEntity( pLine3D ) ;
	}
	else if(bVertical == FALSE)
	{
		if(pBtmCur == NULL)
			return 0 ;
		CSmtCutPath* pOrgPath = new CSmtCutPath();
		pOrgPath->m_bFeedType = JDNC_FEEDTYPE_PLUNGE ;
		PNT3D p0, p1; //分层路径起末点

		CGeoInter geoInter;
		CCurIntpt intpt1, intpt2;
		p0[0] = pt[0];
		p0[1] = pt[1];
		p1[0] = pt0[0];
		p1[1] = pt0[1];

		pOrgPath->AddPoint(pt);
		for( int i = 1 ; i < nLoopCnt ; i++ )
		{
			p0[2] = p1[2] = pt[2] - dDepthInc * i;
			if(!geoInter.CalcLineCurveAllIntpt(p0, p1, pBtmCur, MIN_DIS, &intpt1, &intpt2))
			{
				break;
			}
			else
			{
				pOrgPath->AddPoint(intpt1.m_pHead->pos);
				intpt1.Destory();
				intpt2.Destory();
			}
		}
		pOrgPath->AddPoint(pt0);
		//在每层路径交点处插入折线点
		CSmtCutPath* pPlungePath = new CSmtCutPath();
		pPlungePath->m_bFeedType = JDNC_FEEDTYPE_PLUNGE ;
		if(pOrgPath->m_nNumPnt < 3)
		{
			pPlungePath->AddPoint(pOrgPath->m_pHead->m_fPoint);
			pPlungePath->AddPoint(pOrgPath->m_pTail->m_fPoint);
		}
		else
		{
			for(CSmtCutPoint* pCPnt = pOrgPath->m_pHead ; pCPnt ; pCPnt = pCPnt->next )
			{
				if(pCPnt)
				{
					CSmtCutPoint* pCopyPnt = pCPnt->CopyMyself();
					pPlungePath->AddTail(pCopyPnt);
					if(pCPnt->next)
					{
						PNT3D cutPt0, cutPt1, midPt; 
						nc_FloatToDouble(cutPt0,pCPnt->m_fPoint, 3);
						nc_FloatToDouble(cutPt1,pCPnt->next->m_fPoint, 3);
						midPt[0] = cutPt0[0] + vec[0]*dCurveLen;
						midPt[1] = cutPt0[1] + vec[1]*dCurveLen;
						midPt[2] = (cutPt0[2] + cutPt1[2]) * 0.5 ;
						pPlungePath->AddPoint(midPt);
						CSmtCutPoint* pNxtPnt = pCPnt->next->CopyMyself();
						pPlungePath->AddTail(pNxtPnt);
					}
				}
			}
		}
	
		pPlungePath->DelPointOverlap(1.0e-4);
		pPlungePath->AddToPathCombine(PComb);
		pPlungePath->ClearAllPoint();
		delete pPlungePath ;
		pPlungePath = NULL ;
		if(pOrgPath)
		{
			pOrgPath->ClearAllPoint();
			delete pOrgPath;
		}

		pOrgPath = NULL ;
	}
	return 1 ;
}

void CSmartFeatureSlotGen::ClearSlotPathList(SlotPathList& slotPathList)
{
	INT_PTR nCnt = slotPathList.GetCount();
	if(nCnt < 1)
		return ;
	POSITION pos = slotPathList.GetHeadPosition() ;
	POSITION atPos = NULL ;
	while(pos )
	{
		atPos = pos ;
		SlotPath* pSlotPath = slotPathList.GetNext ( pos ) ;
		if( pSlotPath)
		{
			if( pSlotPath->pCutPath)
			{
				pSlotPath->pCutPath->ClearAllPoint();
				delete pSlotPath->pCutPath;
				pSlotPath->pCutPath = NULL ;
			}
			delete pSlotPath ;
		}
		pSlotPath = NULL ;
		slotPathList.RemoveAt ( atPos ) ;
	}
	slotPathList.RemoveAll () ;
}

void  CSmartFeatureSlotGen::ClearCPathLib(CSmtCPathLib& AllPath)
{
	if( AllPath.m_cAllPath.GetSize() < 1 )
		return ;
	POSITION pos = AllPath.m_cAllPath.GetHeadPosition();
	POSITION atPos = NULL ;
	while(pos)
	{
		atPos = pos ;
		CSmtCutPath* pCutPath = AllPath.m_cAllPath.GetNext(pos);
		if( pCutPath)
		{
			pCutPath->ClearAllPoint();
			delete pCutPath;
		}
		pCutPath = NULL ;
		AllPath.m_cAllPath.RemoveAt(atPos);
	}
	AllPath.m_cAllPath.RemoveAll();
}
void CSmartFeatureSlotGen::ClearFacetList(SurfFacetList& facetList)
{
	if(facetList.IsEmpty())
		return;
	SurfAndFacet* pSNode = NULL ;
	POSITION pos = facetList.GetHeadPosition();
	POSITION atPos = NULL ;
	while(pos)
	{
		atPos = pos ;
		pSNode = (SurfAndFacet*)facetList.GetNext(pos);
		if(pSNode)
		{
			if(pSNode->pFacet)
			{
				pSNode->pFacet->Destory();
				delete pSNode->pFacet;
			}	
			pSNode->pFacet = NULL;
			delete pSNode;
		}
		pSNode = NULL ;
		facetList.RemoveAt(atPos);
	}
	facetList.RemoveAll();
}
void CSmartFeatureSlotGen::DeleteSurfList(CGeoTrmSurfList& ResultSide)
{
	POSITION pos1 = ResultSide.GetHeadPosition();
	POSITION atPos = NULL ;
	while(pos1)
	{
		atPos = pos1 ;
		CGeoTrmSurf* pSurf = ResultSide.GetNext(pos1);
		if( pSurf )
			delete pSurf;
		ResultSide.RemoveAt(atPos);
		pSurf = NULL ;

	}
	ResultSide.RemoveAll();
}
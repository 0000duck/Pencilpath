#include "StdAfx.H"
#include "SmtPathGen.H"
#include "SmtPathGen3D.H"
#include "mathcam.h"

float	g_fExtLen=0.0; //NCSF_TOL_01;
float	g_fUpwardScale=0.0; //NCSF_TOL_01;
//////////////////////
// 插铣加工
CSmartPlungeMillGen::CSmartPlungeMillGen()          
{
}

CSmartPlungeMillGen::~CSmartPlungeMillGen() 
{
}



int CSmartPlungeMillGen::GeneratePathEx( CPathGroup&   NewPath,  /*雕刻路径*/
						                CSmartGraphic&  Graph )  /*加工图形*/ 
{ // 生成刀具路径 
	// STEP 1 : 转换刀具
	if( ! UpdateTool3D( Graph ) )
    {
		return FALSE ;
    }
    // STEP 2 : 构建虚拟加工模型
	CSmtCheckMdl  DriveMdl, CheckMdl ;
	if( !BuildDriveAndCheckMdl( Graph, DriveMdl, CheckMdl ) )
	{
		m_nErrorType = JDERROR_GENPATH_NOSURFACE  ;
		return 0 ; 
	}
    // STEP 3 : 获得加工路径
	CPtrList   AllEnt ;
	Graph.GetAllEntity( m_cSetupDef, AllEnt, SMARTGRAPH_TYPE_PATHGROUP ) ;
	if( AllEnt.GetCount() == 0 ) 
	{
		m_nErrorType = JDERROR_GENPATH_NOPATH  ;
		return 0 ;
	}
	CSmtCPathLib AllPath ;
	POSITION atPos = AllEnt.GetHeadPosition() ;
	while( atPos ) 
	{
		CPathGroup* pPathGroup = ( CPathGroup*) AllEnt.GetNext( atPos ) ;
		MathCAM_AddPathGroup( AllPath, * pPathGroup , m_cSetupDef.m_cTolDef , FALSE ) ;
        delete pPathGroup ;
    }
    AllEnt.RemoveAll() ;

    // STEP 4 :干涉检查, 生成无干涉的刀具路径
	SurfNC_SetNewStepEx(IDS_PATHGEN_STEP_ORGPATH ) ;
	int nPnt = AllPath.GetNumPoint() ;
	if( nPnt < 1 )  nPnt = 1      ;
	m_cPrgDef.m_dLimitAt  =  nPnt / 100.0   ;   // 前进一次的计算量
	m_cPrgDef.m_dStepAt   =  0.0            ;   // 当前计算进度
	m_cPrgDef.m_dIncStep  =  1.0            ;   // 每次计算的前进量
	if( ! AllPath.VerifyCutPath( DriveMdl, GetSurfaceTol(), m_cPrgDef ) )
	{
		return FALSE ;
	}
    CPathCombine * pPComb = new CPathCombine( NC_WPROCESS_IMPORT ) ;
    AllPath.DelPointOnLine() ; // 删除同一直线上的点
	AllPath.AddToPathCombine( * pPComb   ) ;
 	AllPath.ClearAllPath() ;
	NewPath.AddData( 0.0 , pPComb ) ;
    if( m_cPrgDef.m_pBrkFunc && m_cPrgDef.m_pBrkFunc() )
	{
	    m_nErrorType = JDERROR_GENPATH_ABORT  ;
		return 0 ; 
	}
    return TRUE ;
}

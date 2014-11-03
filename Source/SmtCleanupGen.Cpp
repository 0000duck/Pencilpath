#include "StdAfx.H"
#include "global.h"
#include "SurfNC.H"
#include "SurfGeo.H"
#include "SmartNC.H"
#include "SmtPathGen.H"
#include "PencilMdl.h"
#include "PencilLink.h"
#include "SmtPathGen2D.H"
#include "SmtPathGen3D.H"
#include "SmtFlowlineGen.h"
#include "mathcam.h"
#include "Nc3DStepAndSpiral.h"
#include "NcBndProg.h"
#include "NcHeightMdl.H"
#include "SysPrompt.h"
#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

//////////////////////
// 清根加工
CSmartCleanupGen::CSmartCleanupGen()          
{
}
CSmartCleanupGen::~CSmartCleanupGen() 
{
}

int CSmartCleanupGen::GenNetPath(  CPathGroup&   NewPath,
									CSmartGraphic& Graph ,  /*雕刻图形*/
									CSmtCheckMdl& DriveMdl    ,  /*加工模型*/
									CSmartLoop*&  AllLoop)
{
	// STEP 1 : 提取曲面边界
	double  MaxBox[2][3] ;
	CPtrArray AllSurf ;
	RFRAME dNcMtx ;
	if( !Graph.GetTPlaneFrame ( dNcMtx ) )
	{
		mathInitRFrame( &dNcMtx ) ;
	}
	if( m_c5DCtrlDef.m_cToolAxisApp.m_nFrameFlag )
	{
		mathMultRFrame( &dNcMtx, &m_c5DCtrlDef.m_cToolAxisApp.m_dIndexFrame, &dNcMtx ) ;
	}
	Graph.GetEntBox ( MaxBox, SMARTGRAPH_TYPE_GEOSURF ) ;
	MathCAM_TrimBox3DByContours( AllLoop, MaxBox ) ;
	double tol = m_pTool->m_fRadius * 2 + 3.2 ;
	for( int k = 0 ; k < 2 ; k++ )
	{
		MaxBox[0][k] = MaxBox[0][k] - tol ;
		MaxBox[1][k] = MaxBox[1][k] + tol ;
	}
	double dStock = GetDriveSurfZMove() ;
	if( dStock < 0 )
		MaxBox[0][2] += dStock ;
	else if( dStock > 0 )
		MaxBox[1][2] += dStock ;
	if( MaxBox[0][2] < GetBottomHeight() )
		MaxBox[0][2] = GetBottomHeight() ;
	if( MaxBox[1][2] > GetTopHeight() )
		MaxBox[1][2] = GetTopHeight() ;

	// STEP 2 : 清根路径
	//CPencilMdl cPenMdl( &DriveMdl, m_pTool, m_pSafeTool, LastTool, LastSafe, dNcMtx ) ;
	//cPenMdl.SetParam ( m_cParam, m_cSetupDef, m_cFeedDef, m_cStockDef, MaxBox ) ;
	CSmtCPathLib AllPath ;
	BOOL bRet = TRUE;
	int XCount = (MaxBox[1][0] - MaxBox[0][0])/0.3 + 0.5;
	int YCount = (MaxBox[1][1] - MaxBox[0][1])/0.3 + 0.5;
	double XStep = (MaxBox[1][0] - MaxBox[0][0])/(XCount);
	double YStep = (MaxBox[1][1] - MaxBox[0][1])/(YCount);
	CSmtCutPath* aXDirPath = new CSmtCutPath[XCount ] ;
	CSmtCutPath* aYDirPath = new CSmtCutPath[YCount] ;
	FPNT4D ptst, ptend;
	CSmtCutPoint* pXPoint , *pYPoint ;
	PNT3D dPoint ;
	dPoint[0] = dPoint[1] = dPoint[2] = 0.0 ;
	for( int i = 0 ; i < XCount ; i ++ ) 
	{
		for( int j = 0 ; j < YCount ; j ++ )
		{
			dPoint[0] = MaxBox[0][0] + j * 0.3 ;
			dPoint[1] = MaxBox[0][1] + i * 0.3 ;
			pXPoint = new CSmtCutPoint( dPoint ) ;
			pYPoint = new CSmtCutPoint( dPoint ) ;
			aXDirPath[i].AddTail( pXPoint ) ;
			aYDirPath[j].AddTail( pYPoint ) ;
		}
	}
	CPathCombine* pPathComb = new CPathCombine( NC_WPROCESS_ROUGH ) ;
	CPathCombine tmpComb( NC_WPROCESS_ROUGH ) ;
	CSmtCutPath* pXDirPencilPntPath = new CSmtCutPath;
	CSmtCutPath* pYDirPencilPntPath = new CSmtCutPath;
	for (int i=0; i<XCount; i++)
	{
		aXDirPath[i].VerifyCutPath(DriveMdl,m_cSetupDef.m_cTolDef,m_cPrgDef);
		//FindPencilPntOnXYDir(&aXDirPath[i],pXDirPencilPntPath);

		aXDirPath[i].AddToPathCombine(tmpComb);
		pPathComb->AppendCombine(tmpComb);
	}
	for (int i=0; i<YCount; i++)
	{
		aYDirPath[i].VerifyCutPath(DriveMdl,m_cSetupDef.m_cTolDef,m_cPrgDef);
		//FindPencilPntOnXYDir(&aYDirPath[i],pYDirPencilPntPath);
		aYDirPath[i].AddToPathCombine(tmpComb);
		pPathComb->AppendCombine(tmpComb);
	}
	//pXDirPencilPntPath->AddToPathCombine(tmpComb);
	//pYDirPencilPntPath->AddToPathCombine(tmpComb);
	//pPathComb->AppendCombine(tmpComb);
	NewPath.AddData(0.,pPathComb) ;

	if( m_c5DCtrlDef.m_cToolAxis.m_nAxisType == NCDEF_AXIS_VERT )
	{
		AddPlungeAndRelativeH( &DriveMdl, &NewPath,TRUE ) ;
	}
	return bRet ;
}

int CSmartCleanupGen::GenPencilPath( CPathGroup&   NewPath,
								  CSmartGraphic& Graph ,  /*雕刻图形*/
								  CSmtCheckMdl& DriveMdl    ,  /*加工模型*/
								  CSmartLoop*&  AllLoop)
{
	// STEP 1 : 提取曲面边界
	double  MaxBox[2][3] ;
	CPtrArray AllSurf ;
	RFRAME dNcMtx ;
	if( !Graph.GetTPlaneFrame ( dNcMtx ) )
	{
		mathInitRFrame( &dNcMtx ) ;
	}
	if( m_c5DCtrlDef.m_cToolAxisApp.m_nFrameFlag )
	{
		mathMultRFrame( &dNcMtx, &m_c5DCtrlDef.m_cToolAxisApp.m_dIndexFrame, &dNcMtx ) ;
	}
	Graph.GetEntBox ( MaxBox, SMARTGRAPH_TYPE_GEOSURF ) ;
	MathCAM_TrimBox3DByContours( AllLoop, MaxBox ) ;
	double tol = m_pTool->m_fRadius * 2 + 3.2 ;
	for( int k = 0 ; k < 2 ; k++ )
	{
		MaxBox[0][k] = MaxBox[0][k] - tol ;
		MaxBox[1][k] = MaxBox[1][k] + tol ;
	}
	double dStock = GetDriveSurfZMove() ;
	if( dStock < 0 )
		MaxBox[0][2] += dStock ;
	else if( dStock > 0 )
		MaxBox[1][2] += dStock ;
	if( MaxBox[0][2] < GetBottomHeight() )
		MaxBox[0][2] = GetBottomHeight() ;
	if( MaxBox[1][2] > GetTopHeight() )
		MaxBox[1][2] = GetTopHeight() ;

	// STEP 2 : 清根路径
	CSmtCPathLib AllPath ;
	BOOL bRet = TRUE;
	int XCount = (MaxBox[1][0] - MaxBox[0][0])/0.3 + 0.5;
	int YCount = (MaxBox[1][1] - MaxBox[0][1])/0.3 + 0.5;
	double XStep = (MaxBox[1][0] - MaxBox[0][0])/(XCount);
	double YStep = (MaxBox[1][1] - MaxBox[0][1])/(YCount);
	CSmtCutPath* aXDirPath = new CSmtCutPath[XCount ] ;
	CSmtCutPath* aYDirPath = new CSmtCutPath[YCount] ;
	FPNT4D ptst, ptend;
	CSmtCutPoint* pXPoint , *pYPoint ;
	PNT3D dPoint ;
	dPoint[0] = dPoint[1] = dPoint[2] = 0.0 ;
	for( int i = 0 ; i < XCount ; i ++ ) 
	{
		for( int j = 0 ; j < YCount ; j ++ )
		{
			dPoint[0] = MaxBox[0][0] + j * 0.3 ;
			dPoint[1] = MaxBox[0][1] + i * 0.3 ;
			pXPoint = new CSmtCutPoint( dPoint ) ;
			pYPoint = new CSmtCutPoint( dPoint ) ;
			aXDirPath[i].AddTail( pXPoint ) ;
			aYDirPath[j].AddTail( pYPoint ) ;
		}
	}
	CPathCombine* pPathComb = new CPathCombine( NC_WPROCESS_ROUGH ) ;
	CPathCombine tmpComb( NC_WPROCESS_ROUGH ) ;
	CSmtCutPath* pXDirPencilPntPath = new CSmtCutPath;
	CSmtCutPath* pYDirPencilPntPath = new CSmtCutPath;
	for (int i=0; i<XCount; i++)
	{
		aXDirPath[i].VerifyCutPath(DriveMdl,m_cSetupDef.m_cTolDef,m_cPrgDef);
		FindPencilPntOnXYDir(&aXDirPath[i],pXDirPencilPntPath);
	}
	for (int i=0; i<YCount; i++)
	{
		aYDirPath[i].VerifyCutPath(DriveMdl,m_cSetupDef.m_cTolDef,m_cPrgDef);
		FindPencilPntOnXYDir(&aYDirPath[i],pYDirPencilPntPath);
	}
	pXDirPencilPntPath->AddToPathCombine(tmpComb);
	pYDirPencilPntPath->AddToPathCombine(tmpComb);
	pPathComb->AppendCombine(tmpComb);
	NewPath.AddData(0.,pPathComb) ;

	if( m_c5DCtrlDef.m_cToolAxis.m_nAxisType == NCDEF_AXIS_VERT )
	{
		AddPlungeAndRelativeH( &DriveMdl, &NewPath,TRUE ) ;
	}
	return bRet ;
}
double limit = 0.2;
void CSmartCleanupGen::FindPencilPntOnXYDir(CSmtCutPath* pDirPath,CSmtCutPath* pDirPencilPntPath)
{
	CSmtCutPoint* pHead = pDirPath->m_pHead;
	while (pHead && pHead->next && pHead->next->next)
	{
		VEC3D nor1 = {pHead->next->m_fPoint[0]-pHead->m_fPoint[0],pHead->next->m_fPoint[1]-pHead->m_fPoint[1],pHead->next->m_fPoint[2]-pHead->m_fPoint[2]};
		VEC3D nor2 = {pHead->next->next->m_fPoint[0]-pHead->next->m_fPoint[0],pHead->next->next->m_fPoint[1]-pHead->next->m_fPoint[1],pHead->next->next->m_fPoint[2]-pHead->next->m_fPoint[2]};
	    // 求三点夹角
		double ab = nor1[0]*nor2[0] + nor1[1]*nor2[1] + nor1[2]*nor2[2] ;
		double a = sqrt(nor1[0]*nor1[0] + nor1[1]*nor1[1] + nor1[2]*nor1[2] );
		double b = sqrt(nor2[0]*nor2[0] + nor2[1]*nor2[1] + nor2[2]*nor2[2] );
		double CosA = ab/(a*b);
		double Angle = acos(CosA);
		if (Angle > limit)
		{
			pDirPencilPntPath->AddTail(pHead->next->CopyMyself());
		}
		pHead = pHead->next;
	}
}

int CSmartCleanupGen::GenAutoPath( CPathGroup&   NewPath,
                               CSmartGraphic& Graph ,  /*雕刻图形*/
                               CSmtCheckMdl& DriveMdl    ,  /*加工模型*/
                               CSmartLoop*&  AllLoop,       /*边界轮廓*/
							   CSmartTool *LastTool, 
							   CSmartTool *LastSafe )
{
	// STEP 1 : 提取曲面边界
	double  MaxBox[2][3] ;
	CPtrArray AllSurf ;
	RFRAME dNcMtx ;
	if( !Graph.GetTPlaneFrame ( dNcMtx ) )
    {
        mathInitRFrame( &dNcMtx ) ;
    }
	if( m_c5DCtrlDef.m_cToolAxisApp.m_nFrameFlag )
	{
		mathMultRFrame( &dNcMtx, &m_c5DCtrlDef.m_cToolAxisApp.m_dIndexFrame, &dNcMtx ) ;
	}
	Graph.GetEntBox ( MaxBox, SMARTGRAPH_TYPE_GEOSURF ) ;
    MathCAM_TrimBox3DByContours( AllLoop, MaxBox ) ;
	double tol = m_pTool->m_fRadius * 2 + 3.2 ;
	for( int k = 0 ; k < 2 ; k++ )
    {
		MaxBox[0][k] = MaxBox[0][k] - tol ;
		MaxBox[1][k] = MaxBox[1][k] + tol ;
    }
	double dStock = GetDriveSurfZMove() ;
	if( dStock < 0 )
		MaxBox[0][2] += dStock ;
	else if( dStock > 0 )
		MaxBox[1][2] += dStock ;
	if( MaxBox[0][2] < GetBottomHeight() )
		MaxBox[0][2] = GetBottomHeight() ;
	if( MaxBox[1][2] > GetTopHeight() )
		MaxBox[1][2] = GetTopHeight() ;

	// STEP 2 : 清根路径
	CPencilMdl cPenMdl( &DriveMdl, m_pTool, m_pSafeTool, LastTool, LastSafe, dNcMtx ) ;
	cPenMdl.SetParam ( m_cParam, m_cSetupDef, m_cFeedDef, m_cStockDef, MaxBox ) ;
	CSmtCPathLib AllPath ;
	BOOL bRet = cPenMdl.CreateCleanupPath ( AllLoop, dStock, AllPath, NewPath, m_cPrgDef ) ;
	if( !bRet ) 
    {
        m_nErrorType = JDERROR_GENPATH_NOTGENPATH ;
        return FALSE ;
    }
	// 三轴转五轴路径
	if( m_c5DCtrlDef.m_cToolAxis.m_nAxisType != NCDEF_AXIS_VERT )
	{
		if (!TransAllPathFrom3AxisTo5Axis( Graph, DriveMdl, AllPath, NewPath ))
		{
            AllPath.ClearAllPath();
			return FALSE;
		}
	}
	// 将得到的路径添加到路径组中
	CPathCombine *PComb = new CPathCombine( NC_WPROCESS_ROUGH ) ;
	AllPath.AddToPathCombine ( *PComb ) ;
	NewPath.AddData ( 0., PComb ) ;
	if( m_c5DCtrlDef.m_cToolAxis.m_nAxisType == NCDEF_AXIS_VERT )
    {
        AddPlungeAndRelativeH( &DriveMdl, &NewPath,TRUE ) ;
    }
	return bRet ;
}
void MathCAM_CalcContourBox( CSmartLoop* AllCont, PNT2D Box2D[2] );
int bFlag = 0;

int CSmartCleanupGen::GeneratePathEx( CPathGroup&   NewPath,  /*雕刻路径*/
						              CSmartGraphic& Graph )  /*加工图形*/ 
{
	// STEP 0 : 转换刀具
	if( !UpdateTool3D( Graph) ) return 0 ;
	if (!IsAxisCtrlValid(Graph))	// 刀轴控制模型判断
		return FALSE;

	BOOL bCheck = Graph.HasCheckSurf () ;
	if( m_cParam.m_nCleanupType == NCDEF_CLEANUP_INTCUR && !bCheck )
	{
        m_nErrorType = JDERROR_GENPATH_NOCHECKSURF ;
		return 0 ;
	}
	// STEP 1 : 生成上把刀具,添加扰动值,而且避免过小
	double dLastStock = m_cParam.m_dLastStock ;
	double dTol  = 0.09543 ;
	if( dTol < m_cSetupDef.m_cTolDef.m_dArcTol * 3 )
		dTol = m_cSetupDef.m_cTolDef.m_dArcTol * 3 ;

	if( dLastStock > -MIN_LEN && dLastStock < dTol )
	{
		dLastStock = dTol ;
	}
	else if( dLastStock < 0. && fabs( dLastStock ) < dTol )
	{
		dLastStock = -dTol ;
	}
	BOOL bMulti = FALSE ;
	if( m_cParam.m_nCleanupType == NCDEF_CLEANUP_MPENCIL && m_cParam.m_nPencilType == NCDEF_PENCIL_LASTTOOL )
		bMulti = TRUE ;
	// 对于混合清根和缝合清根,判断刀具圆角和平底部分
	CSmartTool *pLastTool = NULL, *pLastSafe = NULL ;
	if( m_cParam.m_nCleanupType == NCDEF_CLEANUP_LINEAR  ||
		m_cParam.m_nCleanupType == NCDEF_CLEANUP_FOLLOW  ||
		m_cParam.m_nCleanupType == NCDEF_CLEANUP_HYBRID  ||
        m_cParam.m_nCleanupType == NCDEF_CLEANUP_MIXTURE ||
		m_cParam.m_nCleanupType == NCDEF_CLEANUP_STITCH  || bMulti )
	{
		if( m_cParam.m_cLastTool.m_dTopDiam  * 0.5 + m_cParam.m_dLastStock <= m_pTool->m_fRadius )
		{
			m_nErrorType = JDERROR_GENPATH_CLEANUPTOOL ;
			return FALSE ;
		}
		pLastTool = CreateSmartTool( m_cParam.m_cLastTool , dLastStock + GetDriveSurfOffset(), 0. ) ;
		if( !pLastTool ) 
		{
			m_nErrorType = JDERROR_GENPATH_CLEANUPTOOL ;
			return FALSE ;
		}
		pLastSafe = CreateSmartTool( m_cParam.m_cLastTool , dLastStock + GetCheckSurfOffset(), 0. ) ;
		if( !pLastSafe ) 
		{
			delete pLastTool ;
			m_nErrorType = JDERROR_GENPATH_CLEANUPTOOL ;
			return FALSE ;
		}
		if( pLastTool->m_fRadius < m_pTool->m_fRadius )
		{
			delete pLastTool, delete pLastSafe ;
			m_nErrorType = JDERROR_GENPATH_CLEANUPTOOL ;
			return FALSE ;
		}
		if( !JudgeToolValid( pLastTool, pLastSafe, dLastStock, bMulti ) )
		{
			m_nErrorType = JDERROR_GENPATH_NOTGENPATH ;
			return FALSE ;
		}
	}
	
	// STEP 2 : 生成检查模型
	CSmartLoop*		AllLoop=Graph.GetAllContour(m_cSetupDef);
    OffsetBorder(AllLoop);
	CSmtCheckMdl  DriveMdl  , CheckMdl;
    if( AllLoop )
    {
        PNT2D	dBox2D[2]={{MAX_DBL, MAX_DBL}, {-MAX_DBL, -MAX_DBL}};
        MathCAM_CalcContourBox( AllLoop, dBox2D ) ;
	    double	dExtLen = m_pTool->m_fRadius;
        if( m_pSafeTool && m_pSafeTool->m_fRadius > dExtLen ) dExtLen = m_pSafeTool->m_fRadius ;
        if(   pLastTool &&   pLastTool->m_fRadius > dExtLen ) dExtLen = pLastTool->m_fRadius ;
        if(   pLastSafe &&   pLastSafe->m_fRadius > dExtLen ) dExtLen = pLastSafe->m_fRadius ;
        dExtLen += ( 2.0+2*GetLeadArcLen()) ;
	    for( int k = 0 ; k < 2; k++)
	    {
            CheckMdl.m_dFltBox[0][k] = DriveMdl.m_dFltBox[0][k]=dBox2D[0][k]-dExtLen;
		    CheckMdl.m_dFltBox[1][k] = DriveMdl.m_dFltBox[1][k]=dBox2D[1][k]+dExtLen;
	    }
	    CheckMdl.m_dFltBox[0][2] = DriveMdl.m_dFltBox[0][2]=-MAX_DBL; 
	    CheckMdl.m_dFltBox[1][2] = DriveMdl.m_dFltBox[1][2]= MAX_DBL;  
    }
	SurfNC_SetNewStepEx(IDS_PATHGEN_STEP_CHECKMDL ) ;
    if( ! BuildDriveAndCheckMdl( Graph, DriveMdl, CheckMdl ) )
	{
		Mini_DeleteContours( AllLoop ) ;
		m_nErrorType = JDERROR_GENPATH_NOSURFACE  ;
		delete pLastTool, delete pLastSafe ;
		return FALSE ;
	}
    int bRet = FALSE ;
    TFLOAT fOldThickness = SmartNC_GetLeftoverThickness() ;
    SmartNC_SetLeftoverThickness( (FLOAT)max( m_cParam.m_dThickness, m_cSetupDef.m_cTolDef.m_dArcTol * 3.0) ) ;

    SmartNC_RegCreateConnect3DPath( MathCAM_GenNormalLeadPathEx );

	if(m_cParam.m_nCleanupType == NCDEF_CLEANUP_LINEAR )
	{//平行截线
		bRet = GenLinearLvPath( NewPath,Graph, DriveMdl, AllLoop, pLastTool, pLastSafe );
	}
	else if(m_cParam.m_nCleanupType == NCDEF_CLEANUP_FOLLOW )
	{//环绕等距
		bRet = Gen3DStepLvPath(NewPath,Graph, DriveMdl, AllLoop, pLastTool, pLastSafe );
	}
	else if(m_cParam.m_nCleanupType == NCDEF_CLEANUP_MIXTURE )
	{//角度分区
		bRet = GenMixtureLvPath(NewPath,Graph, DriveMdl, AllLoop, pLastTool, pLastSafe );
	}
	else if( m_cParam.m_nCleanupType == NCDEF_CLEANUP_MPENCIL )
	{//多笔清根路径
		bRet = GenMutiplePencilPath(NewPath,Graph, DriveMdl, AllLoop );
	}
	else if( m_cParam.m_nCleanupType == NCDEF_CLEANUP_HYBRID || 
		     m_cParam.m_nCleanupType == NCDEF_CLEANUP_INTCUR ||
/*		     m_cParam.m_nCleanupType == NCDEF_CLEANUP_PENCIL ||*/
		     m_cParam.m_nCleanupType == NCDEF_CLEANUP_STITCH )
	{
		bRet = GenAutoPath( NewPath, Graph, DriveMdl, AllLoop, pLastTool, pLastSafe ) ;
	}
	else if(m_cParam.m_nCleanupType == NCDEF_CLEANUP_PENCIL )
	{
		if (bFlag == 0)
		{
			bRet = GenPencilPath( NewPath, Graph, DriveMdl, AllLoop) ;
		}
		else if (bFlag == 1)
		{
			bRet = GenNetPath( NewPath, Graph, DriveMdl, AllLoop);
		}
		else
		{
			bRet = GenAutoPath( NewPath, Graph, DriveMdl, AllLoop, pLastTool, pLastSafe ) ;
		}

	}
    SmartNC_RegCreateConnect3DPath( NULL );
	
	delete pLastTool ;
    delete pLastSafe ;
    Mini_DeleteContours( AllLoop ) ;
    SmartNC_SetLeftoverThickness( fOldThickness ) ;
	return bRet  ;
}

BOOL  CSmartCleanupGen::IsToolValid()
{
	//不能选用带锥度的刀
	if(m_cParam.m_cLastTool.m_nToolType == smtToolAFlat || m_cParam.m_cLastTool.m_nToolType == surfncToolABall ||
		m_cParam.m_cLastTool.m_nToolType == surfncToolANose || m_cToolDef.m_nToolType == smtToolAFlat ||
		m_cToolDef.m_nToolType == surfncToolABall ||m_cToolDef.m_nToolType == surfncToolANose )
		return FALSE;

	//上把刀不能选用平底刀
	if(m_cParam.m_cLastTool.m_nToolType == smtToolFlat)
		return FALSE;

	//如果加工余量不大于0，当前刀也不能选用平底刀
	if(GetDriveSurfOffset()<=0.0 && m_cToolDef.m_nToolType == smtToolFlat)
    {
		return FALSE;
    }

	//两把刀的平坦部分尺寸应相等。如果上把刀为球刀，则当前刀也必须为球刀，
	//如果上把刀为牛鼻刀，则当前刀必须为牛鼻刀或平底刀，且两把刀的平坦部分的宽度（即刀具直径 － 圆角半径 * 2 ）应相等；
	if(m_cToolDef.m_nToolType == smtToolBall && m_cParam.m_cLastTool.m_nToolType!=smtToolBall)
		return FALSE;
	if(m_cParam.m_cLastTool.m_nToolType == smtToolNose)
	{
		if(m_cToolDef.m_nToolType!=smtToolNose || m_cToolDef.m_nToolType!=smtToolFlat
			|| fabs((m_cParam.m_cLastTool.m_dTopDiam-2.0*m_cParam.m_cLastTool.m_dCorRadi) 
			- (m_cToolDef.m_dTopDiam-2.0*m_cToolDef.m_dCorRadi))>0.0001)
			return FALSE;
	}

	//上把刀半径小于清根刀具直径，返回
	if(m_cParam.m_cLastTool.m_dTopDiam < m_cToolDef.m_dTopDiam)
		return FALSE;
	
	return TRUE;
}
BOOL CSmartCleanupGen::JudgeToolValid ( CSmartTool *&pLastTool, CSmartTool *&pLastSafe, double dLastStock, BOOL bMulti )
{
	double dLastCorner = GetToolCorner( pLastTool ) ;
	double dLastFlat   = GetToolFlat  ( pLastTool ) ;
	double dTol = m_cSetupDef.m_cTolDef.m_dArcTol * 2.5 ;
	double dCurrCorner = GetToolCorner( m_pTool ) ;
	double dCurrFlat = GetToolFlat( m_pTool ) ;
	double dRadius = pLastTool->m_fRadius - m_pTool->m_fRadius ;
	if( dLastCorner < 0.01 || dLastCorner < dCurrCorner || fabs( dLastFlat - dCurrFlat ) > dTol )
	{
		delete pLastTool, delete pLastSafe ;
		LPTSTR strError = glbf_GetPromptString() ;
		glbf_LoadSysString( IDS_ERROR_GENPATH_CLEANERROR, strError, 200 ) ;
		if( bMulti || SurfNC_IsBatchGenPathMode()) 
		{
			MathCAM_ReportErrorGenPath( strError, MB_ICONSTOP ) ;
			return FALSE ;
		}
		else
		{
			if( AfxMessageBox( strError, MB_YESNO ) == IDNO )
			{
				return FALSE ;
			}
		}
		JDNC_TOOL cLastTool = m_cParam.m_cLastTool ;
		// 有时候加上表面余量，m_pTool的类型和m_cTool.m_nToolType不一样
		int nToolType = m_pTool->GetType () ;
		if( nToolType == surfncToolFlat || nToolType == surfncToolNose )
			cLastTool.m_nToolType = surfncToolNose ;
		else if( nToolType == surfncToolAFlat || nToolType == surfncToolANose )
			cLastTool.m_nToolType = surfncToolANose ;
		else
            cLastTool.m_nToolType = m_cToolDef.m_nToolType ;
		if( dRadius > 0 )
		{
			cLastTool.m_dCorRadi = cLastTool.m_dTopDiam * 0.5 - dCurrFlat ;
		}
		else
		{
			cLastTool.m_dCorRadi += ( fabs( dRadius - 0.1 ) ) ;
		}

		pLastTool = CreateSmartTool( cLastTool , dLastStock + GetDriveSurfOffset(), 0. ) ;
		pLastSafe = CreateSmartTool( cLastTool , dLastStock + GetCheckSurfOffset(), 0. ) ;
	}
	return TRUE ;
}
//生成多笔清根路径
int CSmartCleanupGen::GenMutiplePencilPath(CPathGroup&    NewPath ,  /*雕刻路径*/
                                           CSmartGraphic&  Graph  ,  /*雕刻图形*/
                                           CSmtCheckMdl& DriveMdl ,  /*加工模型*/
                                           CSmartLoop*&  AllCont  )  /*边界轮廓*/
{
    SurfNC_SetNewStepEx( IDS_PATHGEN_STEP_ORGPATH ); 
	//STEP 1 : 构建刀位点网格模型
	CSmtMeshMdl   MeshMdl ;
    MeshMdl.m_nMeshMode = SMARTNC_MESHMDL_MUTIPENCIL ;
    m_cPrgDef.m_dTotalMove = 15.0 ;
    CSmartLoop * bndBox = NULL, boxLoop ;
    if( AllCont )
    {
        PNT2D dBox[2] ;
        nc_VectorCopy( dBox[0], AllCont->m_dBox[0], 2 ) ;
        nc_VectorCopy( dBox[1], AllCont->m_dBox[1], 2 ) ;
        for( CSmartLoop* pCont = AllCont->next ; pCont ; pCont = pCont->next )
        {
            for( int k = 0 ; k< 2 ; k ++) 
            {
                dBox[0][k] = min( pCont->m_dBox[0][k], dBox[0][k] ) ;
                dBox[1][k] = max( pCont->m_dBox[1][k], dBox[1][k] ) ;
            }
        }
        dBox[0][0] -= 2.0 * DriveMdl.m_pTool->m_fRadius ;
        dBox[0][1] -= 2.0 * DriveMdl.m_pTool->m_fRadius ;
        dBox[1][0] += 2.0 * DriveMdl.m_pTool->m_fRadius ;
        dBox[1][1] += 2.0 * DriveMdl.m_pTool->m_fRadius ;
        boxLoop.CreateLoop( dBox[0], dBox[1] ) ;
        bndBox = &boxLoop ;
    }
    if( BuildMeshMdl( MeshMdl, DriveMdl, bndBox, TRUE ) == FALSE )
    {
        m_nErrorType = JDERROR_GENPATH_ABORT ;
        return FALSE ;
    }
    DriveMdl.CreateAccChkBox() ;
	//　STEP 3 : 计算加工路径
    CSmtCPathLib  AllPath, AutoBnd ;
#ifdef _DEBUG
    if( MeshMdl.m_bDumpXYPath )
    {
        MeshMdl.DumpMeshToPathGroup( NewPath ) ;
        return TRUE ;
    }
#endif 
/*	
	if( !MathCAM_GenPencilBound( MeshMdl, AllLoop, AllPath ) )
	{
		return 0 ;
	}
	CPathCombine *PComb = new CPathCombine( NC_WPROCESS_ROUGH ) ;
	AllPath.AddToPathCombine ( *PComb ) ;
	NewPath.AddData ( 0., PComb ) ;
*/
    m_cPrgDef.m_dTotalMove = 15.0 ;
    if( !MeshMdl.GenMeshSelfBound( AutoBnd, m_cPrgDef ) )
    {
        m_nErrorType = JDERROR_GENPATH_ABORT ;
        return FALSE;
    }
    DOUBLE dOffDist = min( m_cSetupDef.m_cTolDef.m_dArcTol * 3.0, 0.5 * m_cFeedDef.m_cStepDef.m_dOverStep )  ; 
    CSmartLoop* bndTrim = AutoBnd.ConvertToContours( FALSE, m_cSetupDef.m_cCorDef, dOffDist ) ;
    if( AllCont )
    {
        CSmartLoop* tmpCont = Mini_OperateContours( bndTrim, AllCont, 0 ) ;
        Mini_DeleteContours( bndTrim ) ;
        if( tmpCont )
        {
            bndTrim = tmpCont ;
        }
        else 
        {
            bndTrim = Mini_CopyContours( AllCont ) ;
        }
    }
    TFLOAT fZBtm = MeshMdl.GetValidMeshBottom() - TFLOAT(2.0*MeshMdl.m_fNetStep);
    TFLOAT fZOld = DriveMdl.SetBottom( max( fZBtm, DriveMdl.m_fBottom) ) ;
    m_cPrgDef.m_dTotalMove = 50.0 ;
    if( ! MeshMdl.CreateMutiplePencilPath( m_cParam ,  //清根参数
                                           bndTrim ,
                                           m_cFeedDef.m_cStepDef.m_dOverStep,
                                           AllPath, //返回路径
                                           NewPath,
                                           m_cPrgDef ))//调试路径
    {
        Mini_DeleteContours( bndTrim ) ;
        if( SurfNC_IsAbort() ) m_nErrorType = JDERROR_GENPATH_ABORT ;
        else  m_nErrorType = JDERROR_GENPATH_NOAREA ;
        return FALSE ;
    }
    Mini_DeleteContours( bndTrim ) ;
	//STEP 4 :路径连接
    m_cPrgDef.m_dTotalMove = 20.0 ;
    BOOL bZigzag = ( m_cParam.m_bClearupFlag & NCDEF_FCLEANUP_ZIGZAG ) ? TRUE : FALSE ;
    JDNC_CONNECT3D tmpCnt3D = m_cFeedDef.m_cConnectDef ;
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
	if (m_cFeedDef.m_cConnectDef.m_bConnect3DFlag & NCDEF_FCONNECT3D_SPIRAL ) 
	{/*螺旋连接*/
		NcBound_ConnectAllPath( DriveMdl, m_cSetupDef, m_cFeedDef,
			m_cFeedDef.m_cStepDef.m_dOverStep,AllPath, 
			m_cPrgDef,tmpCnt3D.m_dMaxDist,float(25.0*m_cFeedDef.m_cStepDef.m_dOverStep),
			bZigzag ) ;
	}
	else 
	{//根据行号连接多段刀具路径
		AllPath.ConnectPathByLineNo3D( DriveMdl,m_cSetupDef.m_cTolDef, tmpCnt3D, m_cPrgDef );
	}
    if( SurfNC_IsAbort() )
    {
		m_nErrorType = JDERROR_GENPATH_ABORT ;
		return FALSE;
    }
	 // 转化路径
	AddPlungeLeadPath( DriveMdl, m_cFeedDef.m_cConnectDef ,AllPath );	
    if( SurfNC_IsAbort() )
    {
       m_nErrorType = JDERROR_GENPATH_ABORT  ;
       return FALSE ;
    }
    DriveMdl.DeleteAccChkBox() ;
    DriveMdl.SetBottom( fZOld ) ;
	// 三轴转五轴路径
	if( m_c5DCtrlDef.m_cToolAxis.m_nAxisType != NCDEF_AXIS_VERT )
	{
		if (!TransAllPathFrom3AxisTo5Axis( Graph, DriveMdl, AllPath, NewPath ))
		{
			AllPath.ClearAllPath();
			return FALSE;
		}
	}
    CPathCombine * pPComb = new CPathCombine(NC_WPROCESS_ROUGH) ;
    AllPath.AddToPathCombine( *pPComb ) ;
    NewPath.AddData( 0.0, pPComb  ) ;
	if( m_c5DCtrlDef.m_cToolAxis.m_nAxisType == NCDEF_AXIS_VERT )
    {
        AddPlungeAndRelativeH( &DriveMdl, &NewPath,TRUE ) ;
    }
    if( SurfNC_IsAbort() )
    {
		m_nErrorType = JDERROR_GENPATH_ABORT ;
		return FALSE;
    }
	return TRUE ;
}

// 生成刀具路径 
int CSmartCleanupGen::GenLinearLvPath(CPathGroup&   NewPath  ,  /*雕刻路径*/
                                      CSmartGraphic& Graph   ,  /*雕刻图形*/
                                      CSmtCheckMdl& DriveMdl ,  /*加工模型*/
                                      CSmartLoop*&  AllLoop  ,  /*边界轮廓*/
 							          CSmartTool *LastTool, 
								      CSmartTool *LastSafe )
{
	//STEP 1 : 生成检查模型
    SurfNC_SetNewStepEx( IDS_PATHGEN_STEP_ORGPATH ) ;
	DOUBLE dRotateAngle = ANGLE_TO_RADIAN( m_cParam.m_dLineAngle ) ;
	if( fabs( dRotateAngle ) > 0.0001 )
	{ 
        DriveMdl.RotateByZAxis( -dRotateAngle , TRUE ) ;
        for( CSmartLoop* pLp = AllLoop ; pLp ; pLp = pLp->next )
        {
            pLp->RotateContour( - dRotateAngle ) ;
        }
	}
	CSmartLoop*	AllCont = AllLoop, tmpLoop ;
	if( AllCont == NULL ) 
	{
		FPNT3D fMinPt, fMaxPt ;
		PNT2D  dBox[2] ;
		DriveMdl.CalcFacetBox(fMinPt, fMaxPt ) ;
       for( int k = 0 ; k < 2 ; k ++ ) 
       {
           dBox[0][k] = fMinPt[k] - m_pTool->m_fRadius ;
           dBox[1][k] = fMaxPt[k] + m_pTool->m_fRadius ;
       }
       tmpLoop.CreateLoop( dBox[0], dBox[1] );
       AllCont = & tmpLoop ;
	}
	//STEP 2 : 计算初始的平行截线路径
	CSmtCPathLib  tmpLvPath ;
	if( !GenerateLinePathEx( AllCont, tmpLvPath) )
	{
		m_nErrorType = JDERROR_GENPATH_ABORT ;
		return FALSE;
	}
	//STEP 3 : 计算残料加工路径
    m_cPrgDef.m_dTotalMove = 70.0 ;
	if(!CreateLinearCleanupPath( DriveMdl,tmpLvPath, LastTool, LastSafe, m_cPrgDef ))
	{
		m_nErrorType = JDERROR_GENPATH_ABORT ;
		return FALSE;
	}
	//STEP 4 :路径连接
    m_cPrgDef.m_dTotalMove = 30.0 ;
    JDNC_CONNECT3D  cLead3D = m_cFeedDef.m_cConnectDef ;
    cLead3D.m_bConnect3DFlag |=  NCDEF_FCONNECT3D_EXTMODE ;
    SmartNC_RegCreateConnect3DPath( MathCAM_GenZDirLeadPathEx );
    DOUBLE dMaxDist = m_cFeedDef.m_cConnectDef.m_dMaxDist ;
    if( ! tmpLvPath.ConnectPathByLineNo( DriveMdl, dMaxDist,m_cSetupDef.m_cTolDef, m_cPrgDef ,TRUE, NULL, &cLead3D ) )
    {//根据行号连接多段刀具路径
		m_nErrorType = JDERROR_GENPATH_ABORT ;
		return FALSE;
    }
	// 三轴转五轴路径
	if( m_c5DCtrlDef.m_cToolAxis.m_nAxisType != NCDEF_AXIS_VERT )
	{
		if (!TransAllPathFrom3AxisTo5Axis( Graph, DriveMdl, tmpLvPath, NewPath ))
		{
			tmpLvPath.ClearAllPath();
			return FALSE;
		}
	}
	CPathCombine* pPComb = new CPathCombine( NC_WPROCESS_ROUGH ) ;
	tmpLvPath.AddToPathCombine( *pPComb ) ;
	if( fabs( dRotateAngle ) > 0.0001 )
	{
	   pPComb->RotateCombine( dRotateAngle ) ;
       DriveMdl.RotateByZAxis( dRotateAngle , TRUE ) ;
       for( CSmartLoop* pLp = AllLoop ; pLp ; pLp = pLp->next )
       {
            pLp->RotateContour( dRotateAngle ) ;
       }
	}
	NewPath.AddData( 0.0, pPComb ); //最下面一层
	if( m_c5DCtrlDef.m_cToolAxis.m_nAxisType == NCDEF_AXIS_VERT )
    {
        AddPlungeAndRelativeH( &DriveMdl, &NewPath, TRUE ) ;
    }
	return TRUE ;
}

void MathCam_LayerAllSurf( double dTop, double dBot, JDNC_TOL &cTol, JDNC_FEED & cFeedDef, 
						   double *&dSurfZ, int nSurf, double *&dZ, int nCnt, 
						   BOOL bInit, BOOL bFlat, CHtIntStrSetLib2D &IntCLib,
						   double *&dAlldZ, BOOL *&bFlag, int &nCount );

int CSmartCleanupGen::CreateHeightPath( CSmtCheckMdl& DriveMdl , /*检查模型  */ 
                                        CSmtCheckMdl& LastTMdl , /*上把模型  */ 
                                        CSmartLoop*  AllCont   , /*边界裁剪  */
                                        CSmtCPathLib& AllPath  , /*加工路径  */
                                        JDNC_PRGDEF&  PrgDef )
{
	NcBnd_SetProgress ( m_cPrgDef.m_pNewFunc, m_cPrgDef.m_pPosFunc, m_cPrgDef.m_pBrkFunc ) ; 
    BOOL bOldMode = DriveMdl.SetTouchCalcMode( FALSE ) ;
	// 需要多线程的拷贝，则拷贝一份DriveMdl
	//STEP 1 : 构建刀位点网格模型
	CNcZLevelGenMdl   HeightMdl ;
    HeightMdl.m_nValidMode = NCZLEVEL_VALID_TOOLPOS ;
    HeightMdl.SetSlopeAngle( m_cParam.m_dSlopeAngle ) ;
    HeightMdl.SetLastTMdl( &LastTMdl ) ;
    double dTotalMove = PrgDef.m_dTotalMove ;
    PrgDef.m_dTotalMove = 0.2 * dTotalMove ;
    if ( !HeightMdl.Create ( DriveMdl, AllCont, m_cSetupDef , PrgDef) )
    {
        return FALSE ;
    }
	//STEP 2 : 计算等高路径
    CHtIntStrSetLib2D IntCLib ;
	double dTop = GetTopHeight() ;
	double dBot = GetBottomHeight() ;
    IntCLib.CreateLayers( dTop, dBot, m_cFeedDef.m_cStepDef.m_dZIncStep, HeightMdl.m_cSameZSet ) ;
    PrgDef.m_dTotalMove = 0.5 * dTotalMove ;
    if( ! HeightMdl.GenIntCurveSetLib( IntCLib, PrgDef ) )
    {
        return FALSE ;
    }
	IntCLib.TrimByContours ( AllCont, AllPath, NCDEF_HEIGHT_ALL ) ;
	// DriveMdl的数据在本函数拷贝，则在本函数删除
    if( SurfNC_IsAbort() )
    {
		m_nErrorType = JDERROR_GENPATH_ABORT ;
		return FALSE;
    }
	//STEP 4 :路径连接
	JDNC_CONNECT3D  tmpCnt3D = m_cFeedDef.m_cConnectDef ;
    if( m_cParam.m_bClearupFlag & NCDEF_FCLEANUP_ZIGZAG )
	{
		tmpCnt3D.m_bConnect3DFlag |= NCDEF_FCONNECT3D_ZIGZAG ;
	}
	else
	{
		tmpCnt3D.m_bConnect3DFlag &= ~NCDEF_FCONNECT3D_ZIGZAG ;
	}
    tmpCnt3D.m_dSafeDist = m_cFeedDef.m_cStepDef.m_dOverStep ;
    if( m_cSetupDef.m_cOrderDef.m_nLayerOrd == 1 )
    {/*高度优先*/
        tmpCnt3D.m_dSafeDist = 2.0 * min( m_pTool->m_fRadius, fabs(LastTMdl.m_pTool->m_fRadius - m_pTool->m_fRadius) ) ;
    }

	SmartNC_RegCreateConnect3DPath( MathCAM_GenHeightLeadPathEx );
    PrgDef.m_dTotalMove = 0.3 * dTotalMove ;

    CSmtConnectPathMdl  cntMdl ;
	cntMdl.m_dOverStep =  m_cFeedDef.m_cStepDef.m_dOverStep;
	cntMdl.ConnectHeightPath(AllPath, DriveMdl, m_cSetupDef, tmpCnt3D, PrgDef) ;
	
	SmartNC_RegCreateConnect3DPath( NULL );
	
	if( SurfNC_IsAbort() )
	{
		m_nErrorType = JDERROR_GENPATH_ABORT ;
		return FALSE;
	}
	NcBnd_SetProgress ( NULL, NULL, NULL ) ; 
    DriveMdl.SetTouchCalcMode( bOldMode ) ;
	return TRUE ;
}
// 计算平坦面加工路径
int CSmartCleanupGen::CreateFlatAreaPath( CSmtCheckMdl& DriveMdl, 
                                          CSmtCPathLib& AutoBnd ,
                                          CSmtCPathLib& AllPath ,
                                          JDNC_PRGDEF&  PrgDef  )
{/*平坦面路径*/
    DOUBLE dSideTol = max( 0.01, m_cSetupDef.m_cTolDef.m_dArcTol*2 ) ;
    CSmartLoop* BndCont = AutoBnd.ConvertToContours( FALSE, m_cSetupDef.m_cCorDef, dSideTol ) ;
    if( ! BndCont ) 
    {
        return TRUE ;
    }
    CSmartLoop tmpLp ;
    BndCont = tmpLp.ResortContour( BndCont, NCDEF_SORTTYPE_MINDIST, FALSE ) ;
	//构建参数
	JDNC_FOLLOW followDef;
    SurfNC_InitPathParam( followDef ) ;
    followDef.m_bFollowFlag = NCDEF_FOLLOW_SMOOTH | NCDEF_FOLLOW_CORNER | NCDEF_FOLLOW_ZIGZAG ;
    JDNC_FEED tmpFeed = m_cFeedDef ;
    tmpFeed.m_cPlungeDef.m_nPlungeType = NCDEF_PLUNGE_CLOSE ;
	// 防止环切时尖角处出现过切，2014.6.30 liuxin
	JDNC_SETUP cSetupDef = m_cSetupDef;
	if (followDef.m_bFollowFlag & NCDEF_FOLLOW_SMOOTH)
	{
		cSetupDef.m_cCorDef.m_nCorType = NCDEF_CORNER_ARCUSER;
	}	
    CSmartNcFollow FollowGen(m_pTool , followDef, tmpFeed, /*m_cSetupDef*/cSetupDef);
	CPathCombine	tmpPComb(NC_WPROCESS_ROUGH);
	for(CSmartLoop*	pLoop=BndCont; pLoop; pLoop=pLoop->next)
    {
		FollowGen.CreatePocketPath( *pLoop,  tmpPComb );
    }
    Mini_DeleteContours( BndCont ) ;
	AllPath.CreateCutPath( tmpPComb, m_cSetupDef.m_cTolDef ) ;
	AllPath.VerifyCutPath( DriveMdl, m_cSetupDef.m_cTolDef, PrgDef ) ;
	if( SurfNC_IsAbort() )  return FALSE ;
	AllPath.DelPointOnLine(2.0e-4) ;
	return TRUE;
}
// 计算平坦面加工路径
int CSmartCleanupGen::CreateFollow3DStep( CSmtCheckMdl& DriveMdl, 
                                          CSmtMeshMdl&  MeshMdl ,
                                          CSmtCPathLib& AutoBnd ,
                                          CSmtCPathLib& AllPath ,
                                          CPathGroup&   NewPath ,
                                          JDNC_PRGDEF&  PrgDef  )
{
    DOUBLE dTotalMove = PrgDef.m_dTotalMove ;
    PrgDef.m_dTotalMove = dTotalMove * 0.7 ;
    JDNC_FOLLOW  tmpFollow ;
    SurfNC_InitPathParam( tmpFollow ) ;
    tmpFollow.m_nFollowType = NCDEF_FOLLOW_MIDDLE ;
    tmpFollow.m_dOverStep = m_cFeedDef.m_cStepDef.m_dOverStep ;
    tmpFollow.m_dOverRatio = 50.0 ;
    JDNC_STARTPNT startPnt ;
    SurfNC_InitPathParam( startPnt ) ;
    CSmtCPathLib  guideCLib ;
    if( !MeshMdl.CreateFollow3DPath( tmpFollow, startPnt, guideCLib,
                                    AutoBnd, AllPath, NewPath, PrgDef ) )
    {
        m_nErrorType = JDERROR_GENPATH_ABORT  ;
	    return FALSE ;
    }
    PrgDef.m_dTotalMove = dTotalMove * 0.3 ;
    BOOL bZigzag = ( m_cParam.m_bClearupFlag & NCDEF_FCLEANUP_ZIGZAG ) ? TRUE : FALSE ;
    JDNC_CONNECT3D tmpCnt3D = m_cFeedDef.m_cConnectDef ;
    if( bZigzag ) tmpCnt3D.m_bConnect3DFlag |= NCDEF_FCONNECT3D_ZIGZAG ;
    else tmpCnt3D.m_bConnect3DFlag &= ~NCDEF_FCONNECT3D_ZIGZAG ;
    tmpCnt3D.m_dSafeDist = m_cFeedDef.m_cStepDef.m_dOverStep ;
    if( m_cSetupDef.m_cOrderDef.m_nLayerOrd == 1 && 
        tmpCnt3D.m_dSafeDist < m_pTool->m_fRadius * 2.0 )
    { /*高度优先*/
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
		                        PrgDef,tmpCnt3D.m_dMaxDist,float(25.0*m_cFeedDef.m_cStepDef.m_dOverStep),
							    bZigzag ) ;
    }
    else
    {
        AllPath.ConnectPathByLineNo3D( DriveMdl,m_cSetupDef.m_cTolDef,tmpCnt3D, PrgDef);
    }
    if( SurfNC_IsAbort() )
    {
        m_nErrorType = JDERROR_GENPATH_ABORT  ;
	    return FALSE ;
    }
    PrgDef.m_dTotalMove = dTotalMove ;
    return TRUE ;
}

int CSmartCleanupGen::Gen3DStepLvPath(CPathGroup&    NewPath    ,  /*雕刻路径*/
                                      CSmartGraphic& Graph	  ,  /*雕刻图形*/
                                      CSmtCheckMdl& DriveMdl    ,  /*加工模型*/
                                      CSmartLoop*&  AllLoop,       /*边界轮廓*/
								      CSmartTool *LastTool, 
								      CSmartTool *LastSafe )
{
    SurfNC_SetNewStepEx( IDS_PATHGEN_STEP_ORGPATH ) ;
    // STEP 1 : 构建上把刀具模型
    CSmtCheckMdl  LastTMdl, LastCheck;
    SmartNC_CopyCheckMdl( LastTMdl, DriveMdl ) ;
    LastTMdl.UpdateCurrTool( LastTool ) ;
    CSmtCheckMdl *pCheckMdl = DriveMdl.GetCheckMdl() ;
    if( pCheckMdl && pCheckMdl->m_cAllChk.GetSize() > 0 ) 
    {
        SmartNC_CopyCheckMdl( LastCheck, *pCheckMdl ) ;
        LastCheck.UpdateCurrTool( LastSafe ) ;
        LastTMdl.SetCheckMdl( &LastCheck ) ;
    }
	BOOL bLocalCopy = FALSE ;	
	if( IsNeedMultiCopy(DriveMdl))
	{
		bLocalCopy = TRUE ;
		DriveMdl.CreateMultiCopy(m_nCalcThreadNum - 1) ;
	}	
    if( DriveMdl.GetMultiCopy() )
    {
        LastTMdl.CreateMultiCopy(m_nCalcThreadNum - 1) ;
    }
    // STEP 2 : 计算环绕等距网格
    m_cPrgDef.m_dTotalMove = 10.0 ;
    CSmtMeshMdl MeshMdl ;
    MeshMdl.m_nMeshMode = SMARTNC_MESHMDL_LEFTOVER ; 
    if( BuildMeshMdl( MeshMdl ,DriveMdl , AllLoop, TRUE ) == FALSE )
    {
        return FALSE ;
    }
#ifdef _DEBUG
    if( MeshMdl.m_bDumpXYPath )
    {
        MeshMdl.DumpMeshToPathGroup( NewPath ) ;
        return TRUE  ;
    }
#endif 
    m_cPrgDef.m_dTotalMove = 20.0 ;
    CSmtCPathLib  AllPath , AutoBnd ;// 修边路经
    if( ! MeshMdl.LabelPointFlagByLastTMdl( LastTMdl, m_cPrgDef ) )
    {
        m_nErrorType = JDERROR_GENPATH_ABORT ;
        return FALSE ;
    }
    if( ! MeshMdl.GenMeshSelfBound( AutoBnd, m_cPrgDef ) )
    {
        m_nErrorType = JDERROR_GENPATH_ABORT ;
        return FALSE;
    }
    MeshMdl.m_bMeshFlag &= ~SMARTNC_MESHMDL_LABELBYLASTMDL ;
    MeshMdl.m_pLastTMdl = NULL  ;
    LastTMdl.ClearAllCheck() ;
    LastCheck.ClearAllCheck() ;
    if( AutoBnd.GetNumPath() == 0 )
    {
        m_nErrorType = JDERROR_GENPATH_NOAREA ;
        return FALSE;
    }
    // STEP 3 : 计算加工路径
    m_cPrgDef.m_dTotalMove = 70.0 ;
    if( !CreateFollow3DStep( DriveMdl, MeshMdl, AutoBnd, AllPath, NewPath, m_cPrgDef ) )
    {
        m_nErrorType = JDERROR_GENPATH_ABORT ;
        return FALSE;
    }
	if( bLocalCopy == TRUE )	
	{
        DriveMdl.DeleteMultiCopy() ;
	}	
	//STEP 4 :根据行号连接多段刀具路径
	 // 转化路径
	AddPlungeLeadPath( DriveMdl, m_cFeedDef.m_cConnectDef ,AllPath );	
    if( SurfNC_IsAbort() )
    {
       m_nErrorType = JDERROR_GENPATH_ABORT  ;
       return FALSE ;
    }
	// STEP 5 : 三轴转五轴路径
	if( m_c5DCtrlDef.m_cToolAxis.m_nAxisType != NCDEF_AXIS_VERT )
	{
		if(!TransAllPathFrom3AxisTo5Axis( Graph, DriveMdl, AllPath, NewPath ))
		{
			AllPath.ClearAllPath();
			return FALSE;
		}
	}
	// STEP 6 : 转换成加工路径
    CPathCombine * pPComb = new CPathCombine(NC_WPROCESS_ROUGH) ;
    AllPath.AddToPathCombine( *pPComb ) ;
    NewPath.AddData( 0.0, pPComb  ) ;
	if( m_c5DCtrlDef.m_cToolAxis.m_nAxisType == NCDEF_AXIS_VERT )
    {
		AddPlungeAndRelativeH( &DriveMdl, &NewPath, TRUE ) ;
    }
    if( SurfNC_IsAbort() )
    {
		m_nErrorType = JDERROR_GENPATH_ABORT ;
		return FALSE;
    }
	return TRUE ;
}
int CSmartCleanupGen::GenMixtureLvPath(CPathGroup&    NewPath    ,  /*雕刻路径*/
                                       CSmartGraphic& Graph	  ,  /*雕刻图形*/
                                       CSmtCheckMdl& DriveMdl    ,  /*加工模型*/
                                       CSmartLoop*&  AllLoop,       /*边界轮廓*/
								       CSmartTool *LastTool, 
								       CSmartTool *LastSafe )
{
    SurfNC_SetNewStepEx( IDS_PATHGEN_STEP_ORGPATH ) ;
    // STEP 1 : 构建上把刀具模型
    CSmtCheckMdl  LastTMdl, LastCheck;
    SmartNC_CopyCheckMdl( LastTMdl, DriveMdl ) ;
    LastTMdl.UpdateCurrTool( LastTool ) ;
    CSmtCheckMdl *pCheckMdl = DriveMdl.GetCheckMdl() ;
    if( pCheckMdl && pCheckMdl->m_cAllChk.GetSize() > 0 ) 
    {
        SmartNC_CopyCheckMdl( LastCheck, *pCheckMdl ) ;
        LastCheck.UpdateCurrTool( LastSafe ) ;
        LastTMdl.SetCheckMdl( &LastCheck ) ;
    }
	BOOL bLocalCopy = FALSE ;	
	if( IsNeedMultiCopy(DriveMdl))
	{
		bLocalCopy = TRUE ;
		DriveMdl.CreateMultiCopy(m_nCalcThreadNum - 1) ;
	}	
    if( DriveMdl.GetMultiCopy() )
    {
        LastTMdl.CreateMultiCopy(m_nCalcThreadNum - 1) ;
    }
    // STEP 2 : 计算陡峭区域路径
    CSmtCPathLib  steepTLib, flatTLib ;
    DOUBLE dCoef = (m_cParam.m_nRegionType  == NCEEF_FCLEANUP_VERTFLAT ) ? 0.5 : 1.0;
    if( m_cParam.m_nRegionType  != NCDEF_FCLEANUP_DELVERT )
    {
        m_cPrgDef.m_dTotalMove = 100.0 * dCoef ;
        if( ! CreateHeightPath( DriveMdl, LastTMdl, AllLoop, steepTLib, m_cPrgDef ) )
        {
            return FALSE ;
        }
    }
    // STEP 3 : 计算平坦区域路径
    if( m_cParam.m_nRegionType  != NCDEF_FCLEANUP_DELFLAT )
    {
        // STEP 3A : 构建网格模型
        m_cPrgDef.m_dTotalMove = 10.0 * dCoef;
        CSmtMeshMdl MeshMdl ;
        MeshMdl.m_nMeshMode = SMARTNC_MESHMDL_LEFTOVER ; 
        if( BuildMeshMdl( MeshMdl ,DriveMdl , AllLoop, TRUE ) == FALSE )
        {
            return FALSE ;
        }
    #ifdef _DEBUG
        if( MeshMdl.m_bDumpXYPath )
        {
            MeshMdl.DumpMeshToPathGroup( NewPath ) ;
            return TRUE  ;
        }
    #endif 
        // STEP 3B : 计算轮廓边界
        m_cPrgDef.m_dTotalMove = 20.0 * dCoef;
        CSmtCPathLib  AutoBnd ;// 修边路经
        MeshMdl.m_bMeshFlag |= SMARTNC_MESHMDL_LABELBYLASTMDL ;
        MeshMdl.m_pLastTMdl = & LastTMdl;
        if( ! MeshMdl.ExtractFlatAreaBound( m_cParam.m_dSlopeAngle, 
                                            AutoBnd, m_cPrgDef ) ) 
        {
            m_nErrorType = JDERROR_GENPATH_ABORT ;
            return FALSE;
        }
        MeshMdl.m_bMeshFlag &= ~SMARTNC_MESHMDL_LABELBYLASTMDL ;
        MeshMdl.m_pLastTMdl = NULL  ;
        // STEP 3C : 计算加工路径
        if( AutoBnd.GetNumPath() )
        {
            m_cPrgDef.m_dTotalMove = 70.0 * dCoef;
            if( ! CreateFlatAreaPath( DriveMdl, AutoBnd, flatTLib, m_cPrgDef ) )
            {
                m_nErrorType = JDERROR_GENPATH_ABORT ;
                return FALSE;
            }
        }
    }
	if( bLocalCopy == TRUE )	
	{
        DriveMdl.DeleteMultiCopy() ;
	}	
    LastTMdl.ClearAllCheck() ;
    LastCheck.ClearAllCheck() ;
	//STEP 4 :根据行号连接多段刀具路径
    CSmtCPathLib AllPath ;
    AllPath.AppendCPathLib( steepTLib ) ;
    AllPath.AppendCPathLib( flatTLib ) ;
    if( AllPath.GetNumPath() == 0 )
    {
       m_nErrorType = JDERROR_GENPATH_NOAREA  ;
       return FALSE ;
    }
	 // 转化路径
	AddPlungeLeadPath( DriveMdl, m_cFeedDef.m_cConnectDef ,AllPath );	
    if( SurfNC_IsAbort() )
    {
       m_nErrorType = JDERROR_GENPATH_ABORT  ;
       return FALSE ;
    }
	// STEP 5 : 三轴转五轴路径
	if( m_c5DCtrlDef.m_cToolAxis.m_nAxisType != NCDEF_AXIS_VERT )
	{
		if(!TransAllPathFrom3AxisTo5Axis( Graph, DriveMdl, AllPath, NewPath ))
		{
			AllPath.ClearAllPath();
			return FALSE;
		}
	}
	// STEP 6 : 转换加工路径
    CPathCombine * pPComb = new CPathCombine(NC_WPROCESS_ROUGH) ;
    AllPath.AddToPathCombine( *pPComb ) ;
    NewPath.AddData( 0.0, pPComb  ) ;
	if( m_c5DCtrlDef.m_cToolAxis.m_nAxisType == NCDEF_AXIS_VERT )
    {
		AddPlungeAndRelativeH( &DriveMdl, &NewPath, TRUE ) ;
    }
    if( SurfNC_IsAbort() )
    {
		m_nErrorType = JDERROR_GENPATH_ABORT ;
		return FALSE;
    }
	return TRUE ;
}
///////////////////////////////
// 计算带曲面法向的平行截线路径
int CSmartCleanupGen::GenerateLinePathEx( CSmartLoop*  AllCont  ,
 									     CSmtCPathLib&    LvPath )
{
	if( AllCont == NULL )
	{
		ASSERT( 0 ) ;
		return 0 ;
	}
	CSmtCPathLib  AllPath  ;
	CSmtCutPath* pNewPath; 
	PNT2D dMinPt, dMaxPt , dPtArr[1000];
	PNT4D dStart, dEnd  ;
	dStart[2] = dEnd[2] = 0.0 ;
	dStart[3] = dEnd[3] = 0.0 ;
	int nLineNo , nGroup = 0, nLineCnt = 0  ; 
	for( CSmartLoop* pLoop = AllCont; pLoop ; pLoop = pLoop->next )
	{
		if( pLoop == AllCont )
		{
			nc_VectorCopy( dMinPt , pLoop->m_dBox[0], 2 ) ;
			nc_VectorCopy( dMaxPt , pLoop->m_dBox[1], 2 ) ;
		}
		else
		{
			for( int k = 0 ; k < 2 ;k ++ ) 
			{
				if( pLoop->m_dBox[0][k] < dMinPt[k] ) dMinPt[k] = pLoop->m_dBox[0][k] ;
				if( pLoop->m_dBox[1][k] > dMaxPt[k] ) dMaxPt[k] = pLoop->m_dBox[1][k] ;
			}
		}
		nGroup++ ;
	}
	DOUBLE yStep = m_cFeedDef.m_cStepDef.m_dOverStep, y  ;
    int  nTotalLine = (int )ceil( ( dMaxPt[1] - dMinPt[1] ) / yStep ) ;
	nTotalLine *= nGroup ;
	for( pLoop = AllCont, nGroup = 0 ; pLoop ; pLoop = pLoop->next, nGroup++ )
	{
		for( y = dMinPt[1], nLineNo = 0 ; y < dMaxPt[1] ; y += yStep, nLineNo ++ ) 
		{
			nLineCnt ++ ; 
			if( y < pLoop->m_dBox[0][1] ) continue ;
			if( y > pLoop->m_dBox[1][1] ) break ;
			//////////////////////////////
			UINT nCount = pLoop->GetYLineIntContour( y , dPtArr, 1000 ) ;
			if( nCount == 0 || nCount % 2 ) continue ;
			for( UINT i = 0 ; i < nCount ; i += 2 )
			{
				dStart[0] = dPtArr[i  ][0], dStart[1] = dPtArr[i ][1] ;
				dEnd[0]   = dPtArr[i+1][0], dEnd[1]   = dPtArr[i+1][1] ;
				if( dEnd[0] <= dMinPt[0] || dStart[0] >= dMaxPt[0] )
				{
					continue ;
				}
				if( dStart[0]  < dMinPt[0] ) dStart[0] = dMinPt[0] ;
				if( dEnd[0]    > dMaxPt[0] ) dEnd[0] = dMaxPt[0]  ;
				pNewPath = new CSmtCutPath() ;
				pNewPath->AddPoint( dStart ) ;
				pNewPath->AddPoint( dEnd   ) ;
				pNewPath->m_nLineNo  = nLineNo ;
				pNewPath->m_nLayerNo = nGroup ; 
				LvPath.AddToTail( pNewPath ) ;
			}
	    }
	}
	return 1 ;
}
struct LINEAR_DATA
{
    CSmtCPathList* m_cAllPath;
    CSmtCheckMdl* m_pDriveMdl;
    CSmtCheckMdl* m_pLastTMdl;
    JDNC_TOL      m_cTolDef  ;
    JDNC_PRGDEF   m_cPrgDef  ;
	int           m_nAtCore;	// 所用cpu编号, 从零开始
	int			  m_nCoreNum;	// cpu个数
    FLOAT         m_fZMin    ;
    FLOAT         m_fZMax    ;
};
static UINT MathCAM_CreateLinearCleanupProc( LPVOID lpParam )
{
	LINEAR_DATA& calData = *( LINEAR_DATA*) lpParam ;
    JDNC_TOL& Tol = calData.m_cTolDef ;
    CSmtCheckMdl& DriveMdl = *calData.m_pDriveMdl ;
    CSmtCheckMdl& LastTMdl = *calData.m_pLastTMdl ;
    JDNC_PRGDEF& PrgDef = calData.m_cPrgDef ;
    JDNC_PRGDEF  tmpPrg = PrgDef ;
    tmpPrg.m_pPosFunc = NULL ; 
    tmpPrg.m_pPrgFunc = NULL ; 
    POSITION atTPos =  calData.m_cAllPath->GetHeadPosition() ;
    for( int nCnt = 0 ; atTPos ; nCnt ++)
    {
        CSmtCutPath* pTPath = calData.m_cAllPath->GetNext( atTPos ) ;
        if( nCnt % calData.m_nCoreNum != calData.m_nAtCore )
        {            
			MathCAM_MovePrgStep(PrgDef);
            continue ;
        }
        pTPath->InsertCPoint(Tol.m_dMaxStep ) ;
        pTPath->VerifyLinePath(DriveMdl, Tol, tmpPrg ) ;
        pTPath->DefineEdgePoint(DriveMdl, TRUE ) ;
        pTPath->LabelNullPointByZValue( calData.m_fZMin , calData.m_fZMax ) ;
        pTPath->LabelNullPointByLastTMdl( DriveMdl, LastTMdl , TRUE ) ;
        if( PrgDef.m_pBrkFunc && PrgDef.m_pBrkFunc()  )
        { /* 用户中断 || 进度提示 */
	        return FALSE ;
        }
        MathCAM_MovePrgStep(PrgDef);
    }
	return TRUE ;
}


int CSmartCleanupGen::CreateLinearCleanupPath( CSmtCheckMdl& DriveMdl, 
										       CSmtCPathLib&  LvPath,
										       CSmartTool *pLastTool, 
										       CSmartTool *pLastSafe,
                                               JDNC_PRGDEF& PrgDef  )
{
	CSmtCheckMdl LastTMdl , LastCheck;
	SmartNC_CopyCheckMdl( LastTMdl, DriveMdl ) ;
	LastTMdl.UpdateCurrTool( pLastTool ) ;
    CSmtCheckMdl* pCheckMdl = DriveMdl.GetCheckMdl() ;
    if( pCheckMdl && pCheckMdl->m_cAllChk.GetSize() > 0 ) 
    {
	    SmartNC_CopyCheckMdl( LastCheck, *pCheckMdl ) ;
	    LastCheck.UpdateCurrTool( pLastSafe ) ;
	    LastTMdl.SetCheckMdl( & LastCheck ) ;
    }
	JDNC_TOL tmpTol = GetSurfaceTol() ;
    PrgDef.m_dIncStep = PrgDef.m_dStepAt = 0.0 ;
    PrgDef.m_dLimitAt = 1.0 ;
    if( PrgDef.m_dTotalMove > 0.0 && LvPath.m_cAllPath.GetCount()) 
    {
        PrgDef.m_dIncStep = PrgDef.m_dTotalMove / LvPath.m_cAllPath.GetCount() ;
    }
	/* 根据加工的深度范围限定有效点, 高度范围内部的点的属性不变,范围外部点的属性为 0　*/
	float fZMax = (float) GetTopHeight()  ;
	float fZMin = (float) GetBottomHeight() ;
	BOOL bLocalCopy = FALSE ;	
	INT_PTR nPathCount = LvPath.m_cAllPath.GetCount();
	if( nPathCount >=2 && IsNeedMultiCopy(DriveMdl))
	{
		bLocalCopy = TRUE ;
		DriveMdl.CreateMultiCopy(m_nCalcThreadNum - 1) ;
	}	
    if( nPathCount >=2 && DriveMdl.GetMultiCopy() )
    {
        LastTMdl.CreateMultiCopy(m_nCalcThreadNum - 1) ;
    }	
    if (nPathCount >=2 && DriveMdl.GetMultiCopy() && LastTMdl.GetMultiCopy() )
    {/*双线程计算*/
		LINEAR_DATA ThreadData[NC_CFG_CPU_MAXNUM];
		LPVOID lpParam[NC_CFG_CPU_MAXNUM] = {NULL};
		int i=0, k=1;
		int nThreadNum = min((int)nPathCount, m_nCalcThreadNum);
		for (i=0; i<nThreadNum; i++)
		{
			ThreadData[i].m_pDriveMdl = i == 0 ? &DriveMdl : DriveMdl.GetMultiCopy(i-k); 
			ThreadData[i].m_pLastTMdl = i == 0 ? &LastTMdl : LastTMdl.GetMultiCopy(i-k); 
			if (ThreadData[i].m_pDriveMdl == NULL ||
				ThreadData[i].m_pLastTMdl == NULL)
			{
				--i, --k, --nThreadNum;
				continue;
			}
            ThreadData[i].m_cAllPath = &LvPath.m_cAllPath ;			
			ThreadData[i].m_cPrgDef  = PrgDef ;
			if (i > 0)
			{
				ThreadData[i].m_cPrgDef.m_pPrgFunc = NULL ; 
				ThreadData[i].m_cPrgDef.m_pPosFunc = NULL ; 
			}			
            ThreadData[i].m_cTolDef  = tmpTol ;
            ThreadData[i].m_fZMax    = fZMax  ;
            ThreadData[i].m_fZMin    = fZMin  ;
			ThreadData[i].m_nAtCore   = i; 
			lpParam[i] = &ThreadData[i];
		}
		for (i=0; i<nThreadNum; i++)
		{
			ThreadData[i].m_nCoreNum = nThreadNum;
		}
		// 调用多线程函数
		MathCAM_ThreadMainFunc(MathCAM_CreateLinearCleanupProc, lpParam, nThreadNum) ;
    }
    else
    {/*单线程计算*/
        JDNC_PRGDEF  tmpPrg = m_cPrgDef ;
        tmpPrg.m_pPosFunc = NULL ; 
        tmpPrg.m_pPrgFunc = NULL ; 
	    POSITION atPos = LvPath.m_cAllPath.GetHeadPosition() ;
	    while( atPos )
	    {
		    CSmtCutPath* pTPath = LvPath.m_cAllPath.GetNext( atPos ) ;
            pTPath->InsertCPoint( tmpTol.m_dMaxStep ) ;
            pTPath->VerifyLinePath( DriveMdl, tmpTol, tmpPrg ) ;
            pTPath->DefineEdgePoint(DriveMdl, TRUE ) ;
            pTPath->LabelNullPointByZValue( fZMin , fZMax ) ;
            pTPath->LabelNullPointByLastTMdl( DriveMdl, LastTMdl , TRUE ) ;
            if( PrgDef.m_pBrkFunc && PrgDef.m_pBrkFunc() ) 
            {
                break ;
            }
			MathCAM_MovePrgStep(PrgDef);            
	    }
    }
	if( bLocalCopy == TRUE )
    {
        DriveMdl.DeleteMultiCopy() ;
    }
	if( SurfNC_IsAbort())	
    {
	    return FALSE  ;
    }
	CSmtCPathLib tmpTLib ;
    while( LvPath.m_cAllPath.GetCount() )
    {
        CSmtCutPath* pTPath = LvPath.m_cAllPath.RemoveHead() ;
		CSmtCutPath * pPathHead = pTPath->BreakAtNullPoint( FALSE ) ;
        delete pTPath ;
		while( pPathHead ) 
		{
			pTPath = pPathHead ;
            pPathHead = pPathHead->next ;
            pTPath->next = pTPath->prev = NULL ; 
			if( pTPath->GetLength() < 0.2f ) delete pTPath ;
			else tmpTLib.AddToTail( pTPath ) ;
		}
    }
    LvPath.ClearAllPath() ;
    LvPath.AppendCPathLib( tmpTLib ) ;
	return TRUE ;
}

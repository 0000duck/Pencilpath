#include "StdAfx.H"
#include "global.h"
#include "SurfNC.H"
#include "SurfGeo.H"
#include "SmartNC.H"
#include "SmtPathGen.H"
#include "SmtAutoFinishGen.H"
#include "SmartPathEx.h"
#include "SysPrompt.H"
#include "Nc3DStepAndSpiral.h"
#include "NcHeightMdl.H"
#include "NcBndFilleting.H"
#include "NcBndFillCorner.H"
#include "NcBndAutoFinish.h"
#include "NcBndLeftOver.h"
#include "NcBndFillLayer.H"
#include "NcBndProg.h"
#include "Nc5DToolAxis.h"
#include "MathCam.h"
#include "entity.h"
#include "surfett.h"
#include "PathEtt.h"			//为使用SurfNC_GetCurJobSetup()而添加
#include "PathExtend.h"
#include "FHeightKeepSharpEdge.h"
#include "PathSplFit.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

// 多线程运算线程函数
static UINT MathCAM_CreateParallelPathSubProc(LPVOID lpParam)
{
	if (lpParam == NULL)
    {
		return 0;
    }
	CPPT_DATA& cData = *(CPPT_DATA *)lpParam;	
    if( cData.pFinishGen == NULL || 
        cData.TmpPath == NULL    || 
		cData.AllPath == NULL    || 
        cData.DriveMdl == NULL   ) 
    {
        return 0 ;
    }
	return cData.pFinishGen->CreateParallelPathSubProc(cData.nAtCore, cData.nCoreNum, *cData.TmpPath, 
                cData.AllPath, *cData.DriveMdl, cData.bRandMode, cData.PrgDef);
}



void MathCAM_CalcContourBox( CSmartLoop* AllCont, PNT2D Box2D[2] )
{
    if( ! AllCont ) return ;
    nc_VectorCopy( Box2D[0], AllCont->m_dBox[0], 2 ) ;
    nc_VectorCopy( Box2D[1], AllCont->m_dBox[1], 2 ) ;
    for( CSmartLoop* pLoop = AllCont->next ; pLoop ; pLoop = pLoop->next )
    {
        for( int k = 0 ; k< 2 ; k ++) 
        {
            Box2D[0][k] = min( pLoop->m_dBox[0][k], Box2D[0][k] ) ;
            Box2D[1][k] = max( pLoop->m_dBox[1][k], Box2D[1][k] ) ;
        }
    }
}
static int MathCAM_DefineNormalAt( CSmtCheckMdl&   CheckMdl, 
                            JDNC_TOL&       Tol ,
                            TFLOAT  StartAt[3] , 
                            TFLOAT  Normal[3] )
{/*计算位置点附近的近似方向*/
    TFLOAT fSideTol = max( 0.02f, (TFLOAT)(2.0 * Tol.m_dArcTol )) ;
    FPNT3D fCross[4], fVec[2]  ;
    FPNT3D fBox[2] ;
    nc_InitBox3D( StartAt, StartAt, fBox  ) ;
    fBox[0][2] = fBox[1][2] = CheckMdl.m_fBottom ;
    nc_ExpandBox3D( fBox, fSideTol, FALSE ) ;
    CheckMdl.LabelCheckByBox( fBox ) ;
    for( int k = 0 ; k < 4 ; k ++ ) 
    {
        nc_VectorCopy( fCross[k], StartAt , 3 ) ;
        fCross[k][2] = CheckMdl.m_fBottom ;
        if( k == 0 ) fCross[k][0] -= fSideTol ;
        else if( k == 1 ) fCross[k][0] += fSideTol;
        else if( k == 2 ) fCross[k][1] -= fSideTol ;
        else fCross[k][1] += fSideTol ;
        CheckMdl.DefineHeight( fCross[k] ) ;
    }
    fVec[0][0] = fSideTol * 2.0f, fVec[0][1] = 0.0f ;
    fVec[0][2] = (fCross[1][2] - fCross[0][2] ) ;
    fVec[1][0] = 0.0f, fVec[1][1] = fSideTol * 2.0f ;
    fVec[1][2] = (fCross[3][2] - fCross[2][2] ) ;
    nc_VProduct( fVec[0], fVec[1], Normal ) ;
	if( !nc_Normalize( Normal, 3 ) )
    {
        Normal[0] = Normal[1] = Normal[2] ;
    }
    return 1 ;
}
// 计算位置点的进刀路径
static int MathCAM_GenLeadPathEx(CSmtCheckMdl&   CheckMdl  , 
                                 JDNC_TOL& Tol, JDNC_CONNECT3D& ConnectDef, 
			                     TFLOAT  StartAt[3],  TFLOAT  TanDir[3]  , 
                                 TFLOAT  NorDir[3], CSmtCutPath& CutPath, BOOL bHeight )
{
    // STEP 1 : 建立局部坐标系, dXDir圆弧切线,dYDir指向圆心的法线,dZDir圆弧法向
    DOUBLE dYDir[3], dZDir[3], dXDir[3] ;
    nc_FloatToDouble( dXDir, TanDir, 3 ) ;
    nc_FloatToDouble( dYDir, NorDir, 3 ) ;
    if( ! nc_Normalize( dXDir, 3 ) ||
        ! nc_Normalize( dYDir, 3 )   )
    {
        return 0 ;
    }
    nc_VProduct( dXDir, dYDir, dZDir ) ;
    nc_VProduct( dZDir, dXDir, dYDir) ;
	if( fabs( dYDir[2] ) < 1.0e-4 ) dYDir[2] = 0. ;
    if( ! nc_Normalize( dYDir, 3 ) )
    {
        dZDir[0] = 0.0f, dZDir[1] = 1.0f, dZDir[2] = 0.0f ;
        nc_VProduct( dZDir, dXDir, dYDir ) ;
        if( ! nc_Normalize( dYDir, 3 ) )
        {
            return 0 ;
        }
    }
    if( dYDir[2] < 0.0 ) nc_VectorReverse( dYDir, 3 ) ;
    nc_VProduct( dXDir, dYDir,dZDir ) ;
    if( ! nc_Normalize( dZDir, 3 ) ) 
    {
        return 0 ;
    }
    // STEP 2 : 构建延伸段
    JDNC_PRGDEF tmpPrg ;
    SurfNC_InitPathParam( tmpPrg ) ;
    DOUBLE fTPoint[3], fBase[3] = { 0.0f, 0.0f, 0.0f} ;
    FPNT3D fLineEnd[2] ;
    BOOL bExtLine = FALSE ;
    if( ConnectDef.m_dLead3DLine > 0.002 )
    {
        VEC3D fOldXDir, fOldZDir ;
        nc_VectorCopy( fOldXDir, dXDir, 3 ) ;
        nc_VectorCopy( fOldZDir, dYDir, 3 ) ;
        CSmtCutPath linePath(MINI_CONNECT_PATH) ;
        for( int nTry = 0 ; nTry <= 4 ; nTry ++ ) 
        {
            linePath.AddPoint( StartAt ) ;
            for( int k = 0 ;k < 3 ; k ++ )
            {
                fTPoint[k] = StartAt[k] + dXDir[k] * TFLOAT(ConnectDef.m_dLead3DLine) ;
            }
            linePath.AddPoint( fTPoint ) ;
            linePath.InsertCPoint3D ( 0.2 ) ;
            if( linePath.VerifyLeadPath( CheckMdl, Tol, tmpPrg, 0.002 ) == TRUE )
            {
                break ;
            }
            linePath.ClearAllPoint() ;
            nc_RotatePoint( fBase, dZDir, dXDir, ANGLE_TO_RADIAN(0.5) ) ;
            nc_RotatePoint( fBase, dZDir, dYDir, ANGLE_TO_RADIAN(0.5) ) ;
        }
        if( linePath.m_pTail )
        {
            nc_VectorCopy( fLineEnd[0], StartAt , 3 ) ;
            nc_VectorCopy( fLineEnd[1], linePath.m_pTail->m_fPoint , 3 ) ;
            bExtLine = TRUE ;
        }
        else
        {
            nc_VectorCopy( dXDir, fOldXDir, 3 ) ;
            nc_VectorCopy( dYDir, fOldZDir, 3 ) ;
        }
    }
    // STEP 3 : 构建平面圆弧点串
    PNT2D dPoint[51] ;
    DOUBLE dCenter[2] = { 0.0 , ConnectDef.m_dLead3DRadius } ;
    DOUBLE dAngle [2] = { MiniPai2 * 0.75, MiniPai2 * 0.75 + ANGLE_TO_RADIAN( ConnectDef.m_dLead3DAngle )} ;
    CSmartArc tmpArc( dCenter, dAngle, ConnectDef.m_dLead3DRadius ) ;
    int nCnt = tmpArc.DiscreteEx( Tol.m_dArcTol, ANGLE_TO_RADIAN( Tol.m_dAngTol), dPoint, 50 );
    dPoint[0][0] = dPoint[0][1] = 0.0;
    FPNT3D fEndAt ;
    nc_VectorCopy( fEndAt, StartAt, 3 ) ;
    if( bExtLine == TRUE )
    {
        nc_VectorCopy( fEndAt, fLineEnd[1], 3 ) ;
    }
    // STEP 3 : 尝试是否存在过切
    CutPath.SetCutMode( MINI_CONNECT_PATH ) ;
    for( int nTry = 0 ; nTry <= 4 ; nTry ++ ) 
    {
        for( int k = 0 ; k <= nCnt ; k ++ ) 
        {
            fTPoint[0] = fEndAt[0] + (dXDir[0] * dPoint[k][0] + dYDir[0] * dPoint[k][1]) ;
            fTPoint[1] = fEndAt[1] + (dXDir[1] * dPoint[k][0] + dYDir[1] * dPoint[k][1]) ;
            fTPoint[2] = fEndAt[2] + (dXDir[2] * dPoint[k][0] + dYDir[2] * dPoint[k][1]) ;
            CutPath.AddPoint( fTPoint ) ;
        }
        if( CutPath.VerifyLeadPath( CheckMdl, Tol, tmpPrg, 0.005 ) == TRUE )
        {
            break ;
        }
        CutPath.ClearAllPoint() ;
        nc_RotatePoint( fBase, dZDir, dXDir, ANGLE_TO_RADIAN(1.5) ) ;
        nc_RotatePoint( fBase, dZDir, dYDir, ANGLE_TO_RADIAN(1.5) ) ;
    }
	// STEP 4 : 对在平坦面上的点进行检查,只检查后面点
	if( bHeight && CutPath.m_pTail )
	{
		FPNT3D fTemp, fBox[2] ;
		mathFCpyPnt( CutPath.m_pTail->m_fPoint, fTemp ) ;
		mathFDefBox3D( fTemp, fTemp, fBox, 0.1 ) ;
        fBox[1][2] = 1.0e6f ;
		CheckMdl.LabelCheckByBox( fBox ) ;
		fTemp[2] -= 0.1f ;
		CheckMdl.DefineHeight( fTemp ) ;
		double fMove = min( 0.1, 0.1 - ( CutPath.m_pTail->m_fPoint[2] - fTemp[2] )) ;
		if( fMove > 0.001 )
		{
            CutPath.NormalizeLen() ;
            for( CSmtCutPoint* pTPnt = CutPath.m_pHead->next ; pTPnt ; pTPnt = pTPnt->next )
			{
				pTPnt->m_fPoint[2] += TFLOAT( pTPnt->m_fPoint[3] * fMove ) ;
			}
		}
	}
    // STEP 5 : 合并路径，修正起点
    if( bExtLine == TRUE ) 
    {
        if( CutPath.m_pHead == NULL ) 
        {
            CutPath.AddPoint( fLineEnd[0] ) ;
            CutPath.AddPoint( fLineEnd[1] ) ;
        }
        else
        {
            CSmtCutPoint* pTNew = new CSmtCutPoint( StartAt ) ; 
            CutPath.InsertAfter( pTNew, NULL ) ;
        }
    }
    if( CutPath.m_pHead == NULL ) return 0 ;
    nc_VectorCopy( CutPath.m_pHead->m_fPoint, StartAt, 3 ) ;
    CutPath.DefineBox() ;
    return 1 ;
}

int MathCAM_GenZDirLeadPathEx(CSmtCheckMdl&   CheckMdl, 
                              JDNC_TOL&       Tol ,
                              JDNC_CONNECT3D& ConnectDef, 
						      TFLOAT  StartAt[3] , 
                              TFLOAT  TanDir[3]  , 
                              CSmtCutPath& CutPath )
{// 用于平行截线＼径向放射路径
    TFLOAT fNormal[3] = { 0.0f, 0.0f, 1.0f } ;
    return MathCAM_GenLeadPathEx(CheckMdl, Tol, ConnectDef, 
                                 StartAt, TanDir, fNormal, CutPath, FALSE ) ;
}
int MathCAM_GenHeightLeadPathEx(CSmtCheckMdl&   CheckMdl, 
                              JDNC_TOL&       Tol ,
                              JDNC_CONNECT3D& ConnectDef, 
						      TFLOAT  StartAt[3] , 
                              TFLOAT  TanDir[3]  , 
                              CSmtCutPath& CutPath )
{// 用于等高路径
    TFLOAT fNormal[3] ;  
    MathCAM_DefineNormalAt( CheckMdl,Tol,StartAt,fNormal );
    fNormal[2] = 0.0f ;
    if( ! nc_Normalize( fNormal, 3 ))
    {
        fNormal[0] = fNormal[1] = 0.0f ;
        fNormal[2] = 1.0f ;
    }
    return MathCAM_GenLeadPathEx(CheckMdl, Tol, ConnectDef, 
                                 StartAt, TanDir, fNormal, CutPath, TRUE ) ;
}
int MathCAM_GenNormalLeadPathEx(CSmtCheckMdl&   CheckMdl, 
                                JDNC_TOL&       Tol ,
                                JDNC_CONNECT3D& ConnectDef, 
						        TFLOAT  StartAt[3] , 
                                TFLOAT  TanDir[3]  , 
                                CSmtCutPath& CutPath )
{// 用于环绕等距路径
    TFLOAT fNormal[3] ;  
    MathCAM_DefineNormalAt( CheckMdl,Tol,StartAt,fNormal );
    return MathCAM_GenLeadPathEx(CheckMdl, Tol, ConnectDef, 
                                 StartAt, TanDir, fNormal, CutPath, FALSE ) ;
}

CSmartAutoFinishGen::CSmartAutoFinishGen()
{
    m_pSmtGraph  = NULL ; 
    m_pDebugPath = new CPathGroup(0) ;
    m_bTrimByZSize = TRUE ;
}
CSmartAutoFinishGen::~CSmartAutoFinishGen()
{
	if ( m_pDebugPath )
    {
		delete m_pDebugPath ;
        m_pDebugPath = NULL ;
    }
}
void MathCAM_RotateByZAxis( CSmtCPathLib& AllPath, DOUBLE Angle )
{
	DOUBLE fCos , fSin, x , y ;
	fCos = cos( Angle ) ;
	fSin = sin( Angle ) ;
    POSITION atPos = AllPath.m_cAllPath.GetHeadPosition() ;
    while( atPos ) 
    {
        CSmtCutPath* pTPath = AllPath.m_cAllPath.GetNext( atPos ) ;
        for( CSmtCutPoint* pTPoint = pTPath->m_pHead ; pTPoint ; pTPoint = pTPoint->next )
        {
		    x = pTPoint->m_fPoint[0] * fCos - pTPoint->m_fPoint[1] * fSin ;
		    y = pTPoint->m_fPoint[0] * fSin + pTPoint->m_fPoint[1] * fCos ;
		    pTPoint->m_fPoint[0] = (TFLOAT)x  ;
		    pTPoint->m_fPoint[1] = (TFLOAT)y  ;
        }
        pTPath->DefineBox() ;
	}
}



int  CSmartAutoFinishGen::CreateParallelPath( CSmtCheckMdl& DriveMdl , 
                                              DOUBLE  LineAngle      ,
                                              CSmartLoop* AreaCont   , 
                                              CSmtCPathLib& FlatPath ,
                                              JDNC_PRGDEF&  PrgDef   )
{
    if( ! AreaCont ) return FALSE ;
	double dTotalMove = PrgDef.m_dTotalMove ;
    CSmartLoop* pLoop ;
	DOUBLE dRotAng = ANGLE_TO_RADIAN( LineAngle) ; ;
	if(fabs( dRotAng ) > 1.0e-4 )
    { 
        for( pLoop = AreaCont ; pLoop ; pLoop = pLoop->next  )
        {
			pLoop->RotateContour ( -dRotAng ) ;
		}
        DriveMdl.RotateByZAxis( - dRotAng, TRUE ) ;
    }
	// 生成初始的截线路径
	CSmtCPathLib AllPath, OrgPath;	
	GenOrginalPath( AreaCont, OrgPath );
	BOOL bRandPointMode = TRUE ;
	if( bRandPointMode == TRUE )
	{// 随机插点算法 CSmtCutPath::m_bMoveFlag 表示随机数
		srand( 1201);
		POSITION atTPos = OrgPath.m_cAllPath.GetHeadPosition() ;
		while( atTPos )
		{
			CSmtCutPath* pTPath = OrgPath.m_cAllPath.GetNext(atTPos) ;
			pTPath->m_bMoveFlag = rand() ;
		}
	}
	// 需要多线程的拷贝，则拷贝一份DriveMdl
	BOOL bLocalCopy = FALSE ;	
	INT_PTR nPathNum = OrgPath.m_cAllPath.GetCount();
	if (nPathNum >=2 && IsNeedMultiCopy(DriveMdl))
	{
		bLocalCopy = TRUE ;
		DriveMdl.CreateMultiCopy(m_nCalcThreadNum - 1) ;
	}
	CVecPointer NewPath;
	NewPath.resize(nPathNum + 5);
	ZeroMemory(&NewPath[0], sizeof(LPVOID) * (nPathNum + 5));
	// 支持多线程，即拷贝数据不为空
	if (nPathNum >= 2 && DriveMdl.GetMultiCopy() != NULL)
	{
		CPPT_DATA ThreadData[NC_CFG_CPU_MAXNUM];
		LPVOID lpParam[NC_CFG_CPU_MAXNUM] = {NULL};
		JDNC_PRGDEF tmpPrg = PrgDef;
		tmpPrg.m_pPrgFunc = NULL ; 
		tmpPrg.m_pPosFunc = NULL ; 
		int nThreadNum = min(m_nCalcThreadNum, (int)nPathNum);		
		int i=0, k=1;
		for (i=0; i<nThreadNum; i++)
		{
			ThreadData[i].DriveMdl = i == 0 ? &DriveMdl : DriveMdl.GetMultiCopy(i-k);
			if (ThreadData[i].DriveMdl == NULL)
			{
				i--, k--, nThreadNum--;
				continue;
			}
			ThreadData[i].nAtCore = i;
			ThreadData[i].TmpPath = &OrgPath;
			ThreadData[i].AllPath = &NewPath[0];
			ThreadData[i].PrgDef = i==0 ? PrgDef : tmpPrg;
			ThreadData[i].bRandMode = bRandPointMode ;
			ThreadData[i].pFinishGen = this;			
			lpParam[i] = &ThreadData[i];
		}
		for (i=0; i<nThreadNum; i++)
		{
			ThreadData[i].nCoreNum = nThreadNum;
		}
		// 调用多线程函数
		MathCAM_ThreadMainFunc(MathCAM_CreateParallelPathSubProc, lpParam, nThreadNum);
	}
	else
    {// 单线程生成路径
		CreateParallelPathSubProc(0, 1, OrgPath, &NewPath[0], DriveMdl, bRandPointMode , PrgDef);
    }	
	// DriveMdl的数据在本函数拷贝，则在本函数删除
	if( bLocalCopy == TRUE ) 
	{
		DriveMdl.DeleteMultiCopy() ;
	}
	OrgPath.m_cAllPath.RemoveAll();
	if( PrgDef.m_pBrkFunc && PrgDef.m_pBrkFunc() )
	{
		return FALSE ;
	}
	double  dLineTol = max( 2.0e-4, (0.1 * m_cSetupDef.m_cTolDef.m_dArcTol) ) ;
	if( m_cSetupDef.m_cModelTol.m_nMdlFlag & NCDEF_SETUP_NOMOSAIC )
	{
		dLineTol = 2.0e-5;
	}
	Nc5D_AddPathArrayToLib(&NewPath[0], nPathNum, AllPath);
	Nc5D_DelPointOnLine(AllPath, dLineTol);

	/*路径连接*/
	BOOL bRet = FALSE ;
    SmartNC_RegCreateConnect3DPath( MathCAM_GenZDirLeadPathEx );
    PrgDef.m_dTotalMove = dTotalMove * 0.3 ;
    if( m_cFeedDef.m_cConnectDef.m_bConnect3DFlag & NCDEF_FCONNECT3D_ZIGZAG  )
    {
        DOUBLE dMaxDist = m_cFeedDef.m_cConnectDef.m_dMaxDist  ;
        if( m_cFeedDef.m_cConnectDef.m_bConnect3DFlag & NCDEF_FCONNECT3D_BYPASS )
        {
			bRet = NcBnd_ConnectPath_ByPass( AllPath, DriveMdl, 
											PrgDef, m_cSetupDef.m_cTolDef, 
											m_cFeedDef.m_cConnectDef, dMaxDist ) ;
        }
		else
        {
			bRet = AllPath.ConnectPathByLineNo( DriveMdl, dMaxDist,m_cSetupDef.m_cTolDef, 
                                                PrgDef, TRUE, NULL,&(m_cFeedDef.m_cConnectDef)) ;
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
    
	if( fabs( dRotAng ) > 1.0e-4 && bRet )
    { 
        MathCAM_RotateByZAxis( AllPath, dRotAng ) ;
        for( pLoop = AreaCont ; pLoop ; pLoop = pLoop->next  )
        {
			pLoop->RotateContour ( dRotAng ) ;
		}
        DriveMdl.RotateByZAxis( dRotAng, TRUE ) ;
    }
    if( bRet )
    {
        FlatPath.AppendCPathLib( AllPath ) ;
    }
    return bRet ;
}
int  CSmartAutoFinishGen::CreateParallel3DPath( CSmtCheckMdl& DriveMdl , 
                                                DOUBLE  LineAngle      ,
                                                CSmartLoop* AreaCont   , 
                                                CSmtCPathLib& FlatPath ,
                                                JDNC_PRGDEF&  PrgDef   )
{
    if( AreaCont == NULL ) 
    {
        return TRUE ;
    }
    CSmtCPathLib  AllPath ; 
    double dTotalMove = PrgDef.m_dTotalMove ;
    int nGroup = 0 , bRetFlag = TRUE ; 
	for ( CSmartLoop* pLoop = AreaCont ; pLoop ; pLoop = pLoop->next )
	{
        nGroup++ ;
    }
    PrgDef.m_dTotalMove /= nGroup ;
	double dRotAng = ANGLE_TO_RADIAN( LineAngle ) ;
    BOOL bRot = FALSE ;
    if( fabs( dRotAng  ) > 1.0e-4 )
    {
        bRot = TRUE ;
	    for( pLoop = AreaCont ; pLoop ; pLoop = pLoop->next )
	    {
            pLoop->RotateContour( - dRotAng ) ;
        }
        DriveMdl.RotateByZAxis( - dRotAng, TRUE ) ;
    }
    for( pLoop = AreaCont ; pLoop ; pLoop = pLoop->next )
    {
        bRetFlag = CreateOneParallel3DPath( DriveMdl, *pLoop, AllPath, PrgDef ) ;
        if( ! bRetFlag ) break ;
    }
    if( bRot == TRUE )
    {
        MathCAM_RotateByZAxis( AllPath, dRotAng ) ;
	    for( pLoop = AreaCont ; pLoop ; pLoop = pLoop->next )
	    {
            pLoop->RotateContour( dRotAng ) ;
        }
        DriveMdl.RotateByZAxis( dRotAng , TRUE) ;
    }
    PrgDef.m_dTotalMove = dTotalMove ;
    if( bRetFlag )
    {
        FlatPath.AppendCPathLib( AllPath ) ;
    }
    return bRetFlag ;
}
int  CSmartAutoFinishGen::CreateOneParallel3DPath( CSmtCheckMdl& DriveMdl, 
                                                   CSmartLoop&   BndCont , 
                                                   CSmtCPathLib& AllPath ,
                                                   JDNC_PRGDEF&  PrgDef  )
{
    // STEP 1 : 计算路径间距分布曲线
    BOOL bRetFlag = TRUE ;
    CSmartLoop* pTrimCont = BndCont.CopyContour() ;
    double dTotalMove = PrgDef.m_dTotalMove ;
    JDNC_PRGDEF tmpPrg = m_cPrgDef ;
    tmpPrg.m_pPosFunc  = NULL ; 
    tmpPrg.m_pPrgFunc  = NULL ; 
	CSmtCPathLib StepTLib ; /*计算路径间距用的路径组*/
    CSmtCutPath tmpPath ;
    PNT3D dStart, dEnd ;
    DOUBLE stepMax = m_cFeedDef.m_cStepDef.m_dOverStep ;
    DOUBLE moveStep= m_cSetupDef.m_cTolDef.m_dMaxStep  ;
    dStart[2] = dEnd[2] = 0.0 ;
    dStart[1] = BndCont.m_dBox[0][1], dEnd[1] = BndCont.m_dBox[1][1] ;
    PrgDef.m_dTotalMove = dTotalMove * 0.2 ;
    double x = BndCont.m_dBox[0][0] + 0.1 * moveStep ;
    int nNumLine = (int)( (BndCont.m_dBox[1][0] - BndCont.m_dBox[0][0] ) / moveStep) ;
    if( nNumLine < 1 ) nNumLine = 1 ;
    PrgDef.m_dIncStep = PrgDef.m_dTotalMove / nNumLine ;
    PrgDef.m_dStepAt  = 0.0 ;
    PrgDef.m_dLimitAt = 1.0 ;
    for( ; x < BndCont.m_dBox[1][0] - 0.1 * moveStep ; x += moveStep ) 
	{
        if( PrgDef.m_pBrkFunc && PrgDef.m_pBrkFunc() ) 
        {
            bRetFlag = FALSE ;
            break ;
        }
        dStart[0] = dEnd[0] = x ;
        tmpPath.ClearAllPoint() ;
		tmpPath.AddPoint( dStart ) ;
		tmpPath.AddPoint( dEnd   ) ;
		tmpPath.m_nLineNo = 0 ;
		tmpPath.m_nLayerNo = 0 ;
        tmpPath.InsertCPoint( m_cSetupDef.m_cTolDef.m_dMaxStep ) ;
        tmpPath.VerifyLinePath( DriveMdl, m_cSetupDef.m_cTolDef, tmpPrg ) ;
        tmpPath.DefineEdgePoint( DriveMdl, TRUE ) ;
        tmpPath.LabelNullPointByZValue( (TFLOAT)GetBottomHeight(),(TFLOAT)GetTopHeight() ) ;
        tmpPath.LabelNullPointByContour( pTrimCont ) ;
        CSmtCutPath* pNewPath = tmpPath.BreakAtNullPoint( FALSE, 0.02 ) ;
        while( pNewPath ) 
        {
            CSmtCutPath* pTNext = pNewPath->next  ;
            pNewPath->next = pNewPath->prev = NULL ; 
            pNewPath->DefineBox() ;
            StepTLib.AddToTail( pNewPath ) ;
            pNewPath = pTNext ;
        }
        PrgDef.m_dStepAt += PrgDef.m_dIncStep ; 
        while( PrgDef.m_pPrgFunc && PrgDef.m_dStepAt >= PrgDef.m_dLimitAt )
        {
            PrgDef.m_pPrgFunc(1) ;
            PrgDef.m_dStepAt -= PrgDef.m_dLimitAt  ;
        }
	}
    // STEP 2 : 修剪取样曲线
    int nLineNo  = 0;
	CSmtCPathLib tmpTLib, trimTLib ; /*计算路径间距用的路径组*/
    dStart[0] = BndCont.m_dBox[0][0], dEnd[0] = BndCont.m_dBox[1][0] ;
    double y = BndCont.m_dBox[0][1] + 0.1 * stepMax ;
    if( StepTLib.GetNumPath() ) 
    {
        TFLOAT fYMin = (TFLOAT) BndCont.m_dBox[1][1] ;
        POSITION atPos = StepTLib.m_cAllPath.GetHeadPosition() ;
        while( atPos )
        {
            CSmtCutPath* pTPath = StepTLib.m_cAllPath.GetNext(atPos ) ;
            if( pTPath->m_fBox[0][1] < fYMin ) 
            {
                fYMin = pTPath->m_fBox[0][1] ;
            }
        }
        y = fYMin ;
    }
    PrgDef.m_dTotalMove = dTotalMove * 0.5 ;
    PrgDef.m_dIncStep = PrgDef.m_dTotalMove / ( BndCont.m_dBox[1][1] -BndCont.m_dBox[0][1]) ;
    PrgDef.m_dStepAt  = 0.0 ;
    PrgDef.m_dLimitAt = 1.0 ;
    for( ; y < BndCont.m_dBox[1][1]-0.1 * stepMax; nLineNo ++ ) 
	{
        if( PrgDef.m_pBrkFunc && PrgDef.m_pBrkFunc() ) 
        {
            bRetFlag = FALSE ;
            break ;
        }
        dStart[1] = dEnd[1] = y ;
        tmpPath.ClearAllPoint() ;
		tmpPath.AddPoint( dStart ) ;
		tmpPath.AddPoint( dEnd   ) ;
		tmpPath.m_nLineNo  = nLineNo ;
		tmpPath.m_nLayerNo = 1 ;
        tmpPath.InsertCPoint( m_cSetupDef.m_cTolDef.m_dMaxStep ) ;
        tmpPath.VerifyCutPath( DriveMdl, m_cSetupDef.m_cTolDef, tmpPrg ) ;
        tmpPath.DefineBox() ;
        double yNext = y + stepMax; 
        trimTLib.AddToTail( & tmpPath ) ;
        StepTLib.TrimYDirTLibByXDirTLib( trimTLib , stepMax, y, yNext ) ;
        trimTLib.m_cAllPath.RemoveAll() ;
        double yStep = yNext - y ;
        if( yStep < 0.1 * stepMax ) yStep = 0.1 * stepMax ;
        else if( yStep > stepMax ) yStep = stepMax ;
        if( yStep < 0.002 ) yStep = 0.002 ;
        y += yStep ;
        tmpPath.DefineEdgePoint( DriveMdl, TRUE ) ;
        tmpPath.LabelNullPointByZValue( (TFLOAT)GetBottomHeight(),(TFLOAT)GetTopHeight() ) ;
        tmpPath.LabelNullPointByContour( pTrimCont ) ;
        CSmtCutPath* pNewPath = tmpPath.BreakAtNullPoint( FALSE,m_cFeedDef.m_cConnectDef.m_dDelShort ) ;
        while( pNewPath ) 
        {
            CSmtCutPath* pTNext = pNewPath->next  ;
            pNewPath->next = pNewPath->prev = NULL ; 
            tmpTLib.AddToTail( pNewPath ) ;
            pNewPath = pTNext ;
        }
        PrgDef.m_dStepAt +=  yStep * PrgDef.m_dIncStep ; 
        while( PrgDef.m_pPrgFunc && PrgDef.m_dStepAt >= PrgDef.m_dLimitAt )
        {
            PrgDef.m_pPrgFunc(1) ;
            PrgDef.m_dStepAt -= PrgDef.m_dLimitAt  ;
        }
	}
	
	// STEP 3 : 路径连接\生成进退刀
    SmartNC_RegCreateConnect3DPath( MathCAM_GenZDirLeadPathEx );
    PrgDef.m_dTotalMove = dTotalMove * 0.3 ;
    if( bRetFlag && 
        (m_cFeedDef.m_cConnectDef.m_bConnect3DFlag & NCDEF_FCONNECT3D_ZIGZAG ))
    {
        DOUBLE dMaxDist = m_cFeedDef.m_cConnectDef.m_dMaxDist  ;
        if( m_cFeedDef.m_cConnectDef.m_bConnect3DFlag & NCDEF_FCONNECT3D_BYPASS )
        {
			bRetFlag = NcBnd_ConnectPath_ByPass( tmpTLib, DriveMdl, 
											PrgDef, m_cSetupDef.m_cTolDef, 
											m_cFeedDef.m_cConnectDef, dMaxDist ) ;
        }
		else
        {
			bRetFlag = tmpTLib.ConnectPathByLineNo( DriveMdl, dMaxDist,m_cSetupDef.m_cTolDef,
                                                    PrgDef, TRUE, NULL,&(m_cFeedDef.m_cConnectDef)) ;
        }
    }
    else if( bRetFlag ) 
    {
		if( GetMillDir() == 1)
		{ /*从外向内*/
			tmpTLib.ReverseDirect() ;
		}
		tmpTLib.CreateLeadPath( DriveMdl, m_cSetupDef.m_cTolDef, m_cFeedDef.m_cConnectDef) ;
		bRetFlag = TRUE ;
    }
    SmartNC_RegCreateConnect3DPath( NULL );

    if( bRetFlag )
    {
        AllPath.AppendCPathLib( tmpTLib ) ;
    }
    delete pTrimCont ;
    PrgDef.m_dTotalMove = dTotalMove ;
    return bRetFlag  ;
}


void CSmartAutoFinishGen::CreateGuideCurve( CSmartGraphic& Graph, 
                                            CSmtCheckMdl& DriveMdl, 
                                            CSmartLoop*  /*AllCont*/, 
                                            CSmtCPathLib& GuideLib )
{
    JDNC_PRGDEF tmpPrg = m_cPrgDef ;
    tmpPrg.m_pPosFunc = NULL ; 
    tmpPrg.m_pPrgFunc = NULL ; 
    CPtrList AllList ;
    Graph.GetAllEntity( m_cSetupDef, AllList, SMARTGRAPH_TYPE_GUIDECURVE );
    while( AllList.GetCount() ) 
    {
        CSmartCurve* pCurve = (CSmartCurve*) AllList.RemoveHead() ;
        CSmtCutPath* pNewPath = new CSmtCutPath() ;
        pNewPath->AddCurve( *pCurve, m_cSetupDef.m_cTolDef ) ;
        delete pCurve ;
        for( CSmtCutPoint* pTPoint = pNewPath->m_pHead; pTPoint; pTPoint = pTPoint->next )
        {
            pTPoint->m_fPoint[2] = DriveMdl.m_fBottom ;
        }
        pNewPath->InsertCPoint( m_cSetupDef.m_cTolDef.m_dMaxStep ) ; 
        pNewPath->VerifyCutPath( DriveMdl, m_cSetupDef.m_cTolDef, tmpPrg ) ;
        GuideLib.AddToTail( pNewPath ) ;
    }
}

int CSmartAutoFinishGen::CreateFollowPath( CSmtCheckMdl& DriveMdl , /*网格模型  */ 
                                           JDNC_FOLLOW&  FollowDef, 
                                            CSmartLoop*  AreaCont , /*边界裁剪  */
                                            CSmtCPathLib& AllPath , /*加工路径  */
                                            JDNC_PRGDEF&  PrgDef )
{
    CSmtCPathLib   guideCLib ;
    FollowDef.m_dOverStep = m_cFeedDef.m_cStepDef.m_dOverStep ;
    FollowDef.m_nFollowType = NCDEF_FOLLOW_MIDDLE ;
	// STEP 3 : 计算路径
    CSmtCutPath* pTPath;
    /* 平面路径间距 */
    CSmartLoop* AllLoop = Mini_CopyContours( AreaCont) ;
 	CSmartLoop cTmpLoop ;
	AllLoop =cTmpLoop.ResortContour( AllLoop, GetSortType(), FALSE);
    CSmartCurveLib AllCurve ;
    CSmartCurve * pCurve ;
	CSmartLoop *pContour ;
    CSmtCPathLib tmpTLib ;
    int nLayerNo = 0 ;
	for( pContour = AllLoop ; pContour ; pContour = pContour->next )
	{
        pContour->CreateFollowCurve( FollowDef, m_cSetupDef,
                                        m_cMethodDef.m_cSFinishCut.m_cStartPnt, AllCurve ) ;
        if( AllCurve.m_pTail == NULL ) continue ;
        nLayerNo ++ ;
        for( pCurve = AllCurve.m_pHead ; pCurve ; pCurve = pCurve->next )
        {
            pTPath = new CSmtCutPath() ;
            pTPath->AddCurve( *pCurve , m_cSetupDef.m_cTolDef ) ;
            pTPath->m_nLineNo = pCurve->m_nLevelNo ;
            pTPath->m_nLayerNo = nLayerNo ;
            AllPath.AddToTail( pTPath ) ;
        }
        AllCurve.DeleteAllCurves() ;
	}
    Mini_DeleteContours( AllLoop ) ;
    // STEP 4 :干涉检查, 生成无干涉的刀具路径
    double dTotalMove = PrgDef.m_dTotalMove ;
    PrgDef.m_dTotalMove = 0.7 * dTotalMove ;
    if( !AllPath.VerifyCutPath( DriveMdl, GetSurfaceTol(), PrgDef ) )
	{
		m_nErrorType = JDERROR_GENPATH_ABORT  ;
		return FALSE ;
	}
    AllPath.DelPointOnLine() ;
	
	//STEP 4 :路径连接
    JDNC_CONNECT3D  tmpCnt3D = m_cFeedDef.m_cConnectDef ;
    tmpCnt3D.m_dSafeDist = FollowDef.m_dOverStep ;
    SmartNC_RegCreateConnect3DPath( MathCAM_GenNormalLeadPathEx );
    PrgDef.m_dTotalMove = 0.3 * dTotalMove ;
	if ( m_cSetupDef.m_cModelTol.m_nMdlFlag & NCDEF_SETUP_SMOOTHCORNER )
	{/*光滑尖角*/
		AllPath.SmoothAllPath ( DriveMdl, m_cSetupDef,m_cPrgDef ) ;
	}
	if (m_cFeedDef.m_cConnectDef.m_bConnect3DFlag & NCDEF_FCONNECT3D_SPIRAL ) 
	{/*螺旋连接*/
		NcBound_ConnectAllPath( DriveMdl, m_cSetupDef, m_cFeedDef,
			                    FollowDef.m_dOverStep,AllPath, 
			                    PrgDef,tmpCnt3D.m_dMaxDist,TFLOAT(25.0*FollowDef.m_dOverStep)) ;
	}
	else
    {
        AllPath.ConnectPathByLineNo3D( DriveMdl,m_cSetupDef.m_cTolDef, tmpCnt3D, PrgDef );//根据行号连接多段刀具路径
    }
    SmartNC_RegCreateConnect3DPath( NULL );
    
    if( SurfNC_IsAbort() ) 
    {
		m_nErrorType = JDERROR_GENPATH_ABORT  ;
		return FALSE ;
    }
	return TRUE ;
}

int CSmartAutoFinishGen::Create3DStepPath( CSmtCheckMdl& DriveMdl , /*网格模型  */ 
                                           JDNC_FOLLOW&  FollowDef, /*环切设置  */
                                           CSmartLoop*  AllCont  , /*边界裁剪  */
                                           CSmtCPathLib& AllPath , /*加工路径  */
                                           JDNC_PRGDEF&  PrgDef )
{
    CSmtCPathLib   guideCLib ;
    if( FollowDef.m_nFollowType == NCDEF_FOLLOW_GUIDE )
    {/*导动线等距环切*/
        CreateGuideCurve( *m_pSmtGraph, DriveMdl, AllCont, guideCLib ) ;
    }
	// STEP 2 : 构建MeshMdl
    double dTotalMove = PrgDef.m_dTotalMove ;
    CSmtMeshMdl   MeshMdl ;
    m_cPrgDef.m_dTotalMove = 0.10 * dTotalMove ;
    if( BuildMeshMdl( MeshMdl, DriveMdl, AllCont,TRUE ) == FALSE )
    {
        return FALSE ;
    }
    CSmtCPathLib  AutoBnd ;// 修边路经
    PrgDef.m_dTotalMove = 0.10 * dTotalMove ;
    MeshMdl.GenMeshSelfBound( AutoBnd, PrgDef ) ;
#ifdef _DEBUG
    if( MeshMdl.m_bDumpXYPath && m_pDebugPath)
    {
        MeshMdl.DumpMeshToPathGroup( *m_pDebugPath ) ;
        CPathCombine* pTComb = new CPathCombine( NC_WPROCESS_ROUGH ) ;
        AutoBnd.AddToPathCombine( *pTComb ) ;
        m_pDebugPath->AddData( 0.0, pTComb ); //最下面一层
        return TRUE ;
    }
#endif 
    if( AutoBnd.GetNumPath() == 0 ) 
    {
 	    m_nErrorType = JDERROR_GENPATH_NOAREA  ;
        return FALSE ;
    }
	// STEP 3 : 计算路径
    int nLineNo = 1 ;
    CSmtCutPath* pTPath;
    MeshMdl.m_pGenMeshGuidePointBound = NcBound_GenMeshGuidePointBound ;
    MeshMdl.m_pGenMeshGuideCurveBound = NcBound_GenMeshGuideCurveBound ;
    PrgDef.m_dTotalMove = 0.50 * dTotalMove ;
    MeshMdl.CreateFollow3DPath( FollowDef, 
                                m_cMethodDef.m_cSFinishCut.m_cStartPnt, 
                                guideCLib ,
                                AutoBnd, AllPath,
                                *m_pDebugPath,PrgDef ) ;
    if( SurfNC_IsAbort() )
    {
		m_nErrorType = JDERROR_GENPATH_ABORT  ;
		return FALSE ;
    }
	//STEP 4 :路径连接
    JDNC_CONNECT3D  tmpCnt3D = m_cFeedDef.m_cConnectDef ;
    tmpCnt3D.m_dSafeDist = FollowDef.m_dOverStep ;
    SmartNC_RegCreateConnect3DPath( MathCAM_GenNormalLeadPathEx );
    PrgDef.m_dTotalMove = 0.30 * dTotalMove ;
	if ( m_cSetupDef.m_cModelTol.m_nMdlFlag & NCDEF_SETUP_SMOOTHCORNER )
	{/*光滑尖角*/
		AllPath.SmoothAllPath ( DriveMdl, m_cSetupDef, m_cPrgDef ) ;
	}
	if (m_cFeedDef.m_cConnectDef.m_bConnect3DFlag & NCDEF_FCONNECT3D_SPIRAL ) 
	{/*螺旋连接*/
		NcBound_ConnectAllPath( DriveMdl, m_cSetupDef, m_cFeedDef,
			                    FollowDef.m_dOverStep,AllPath, 
			                    PrgDef,tmpCnt3D.m_dMaxDist,TFLOAT(25.0*FollowDef.m_dOverStep)) ;
	}
	else
    {
        AllPath.ConnectPathByLineNo3D( DriveMdl,m_cSetupDef.m_cTolDef, tmpCnt3D, PrgDef );//根据行号连接多段刀具路径
    }
    SmartNC_RegCreateConnect3DPath( NULL );
	if( GetMillDir() == 1 )
    { /*设置顺铣方向*/
        AutoBnd.ReverseDirect();
    }
    while( AutoBnd.m_cAllPath.GetCount() > 0 ) 
    {
        pTPath = AutoBnd.m_cAllPath.RemoveHead() ;
        if( ! pTPath ) break ;
        if( pTPath->GetCutMode() == MINI_MILL_PATH )
        {
           pTPath->m_nLineNo = nLineNo ;
           pTPath->m_nLayerNo = 1 ;
           AllPath.AddToTail( pTPath ) ;
        }
        else
        {
            delete pTPath ;
        }
    }

    if( SurfNC_IsAbort() ) 
    {
		m_nErrorType = JDERROR_GENPATH_ABORT  ;
		return FALSE ;
    }
	return TRUE ;
}
int CSmartAutoFinishGen::CreateSEmitPath( CSmtCheckMdl& DriveMdl , /*网格模型  */ 
                                          CSmartLoop*  AllCont  , /*边界裁剪  */
                                          CSmtCPathLib& AllPath  , /*加工路径  */
                                          JDNC_PRGDEF&  PrgDef )
{
	// STEP 1 : 计算中心半径
    DOUBLE dRadius , dRadAng, dAngle ; 
    PNT2D dBox2D[2], dCenter ;
    nc_VectorCopy( dBox2D[0], AllCont->m_dBox[0], 2 ) ;
    nc_VectorCopy( dBox2D[1], AllCont->m_dBox[1], 2 ) ;
    for( CSmartLoop* pLoop = AllCont->next ; pLoop ; pLoop = pLoop->next )
    {
        for( int k = 0 ; k< 2 ; k ++) 
        {
            dBox2D[0][k] = min( pLoop->m_dBox[0][k], dBox2D[0][k] ) ;
            dBox2D[1][k] = max( pLoop->m_dBox[1][k], dBox2D[1][k] ) ;
        }
    }
    dRadius = nc_Distance( dBox2D[0], dBox2D[1], 2 ) * 0.5 ;
    dCenter[0] = 0.5*(dBox2D[0][0]+dBox2D[1][0]) ;
    dCenter[1] = 0.5*(dBox2D[0][1]+dBox2D[1][1]) ;
    FPNT3D fStart, fEnd ;
    fStart[2] = fEnd[2] = DriveMdl.m_fBottom  ;
    fStart[0] = (TFLOAT)dCenter[0] ;
    fStart[1] = (TFLOAT)dCenter[1] ;
    DOUBLE dAngStep = m_cFeedDef.m_cStepDef.m_dOverStep ;
    dAngStep = dAngStep / dRadius ; //* 180 / MiniPai ;
    dRadius += DriveMdl.m_pTool->m_fRadius ;

    int nCount = (int)fabs( MiniPai2 / dAngStep) ;
	if( nCount < 1 ) nCount = 1 ;
	double dAngleInc = MiniPai2 / nCount ;
	dAngle = 0.0 ;
    JDNC_TOL tmpTol  = GetSurfaceTol() ;
    // STEP 2 :计算平面路径
    CSmtCutPath*  pNewPath, *pTNext, tmpPath ;
    /*边界路径点类型*/
    TFLOAT fZMin, fZMax ;
    fZMin = TFLOAT( GetBottomHeight()) ;
    fZMax = TFLOAT( GetTopHeight() ) ;
    dAngle = 0.0 ;
    JDNC_PRGDEF tmpPrg = m_cPrgDef ;
    tmpPrg.m_pBrkFunc = NULL ; 
    tmpPrg.m_pPosFunc = NULL ; 
    tmpPrg.m_pPrgFunc = NULL ;
    double dTotalMove = PrgDef.m_dTotalMove ;
    if( m_cFeedDef.m_cConnectDef.m_bConnect3DFlag & NCDEF_FCONNECT3D_ZIGZAG )
    {
        PrgDef.m_dTotalMove = 0.7 * dTotalMove ;
    }
    PrgDef.m_dIncStep = PrgDef.m_dStepAt = 0.0 ;
    PrgDef.m_dLimitAt = 1.0 ;
    if( nCount )
    {
        PrgDef.m_dIncStep = PrgDef.m_dTotalMove / nCount ;
    }
    for( int i = 0 ; i < nCount ; i ++, dAngle += dAngleInc ) 
    {
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
	    dRadAng = dAngle ; //ANGLE_TO_RADIAN( dAngle ) ;
	    fEnd[0] = fStart[0] + TFLOAT( cos( dRadAng ) * dRadius) ;
	    fEnd[1] = fStart[1] + TFLOAT( sin( dRadAng ) * dRadius) ;
        tmpPath.ClearAllPoint() ;
        tmpPath.AddPoint( fStart ) ;
        tmpPath.AddPoint( fEnd ) ;
        tmpPath.InsertCPoint( tmpTol.m_dMaxStep ) ;
        tmpPath.VerifyLinePath( DriveMdl, tmpTol, tmpPrg ) ;
        tmpPath.DefineEdgePoint( DriveMdl,TRUE ) ;
        tmpPath.LabelNullPointByContour( AllCont ) ;
        tmpPath.LabelNullPointByZValue(fZMin, fZMax) ;
        pNewPath = tmpPath.BreakAtNullPoint( FALSE,m_cFeedDef.m_cConnectDef.m_dDelShort ) ;
        for(  ; pNewPath ; pNewPath = pTNext  )
        {
           pTNext = pNewPath->next ;
           pNewPath->prev = pNewPath->next = NULL ; 
           pNewPath->m_nLineNo = i + 1;
           pNewPath->m_nLayerNo = 0 ;
           AllPath.AddToTail( pNewPath ) ;
        }
    }
    SmartNC_RegCreateConnect3DPath( MathCAM_GenZDirLeadPathEx );
    PrgDef.m_dTotalMove = 0.3 * dTotalMove ;
	if( m_cFeedDef.m_cConnectDef.m_bConnect3DFlag & NCDEF_FCONNECT3D_ZIGZAG  )
	{ /*路径连接*/
        DOUBLE dMaxDist = m_cFeedDef.m_cConnectDef.m_dMaxDist  ;
        AllPath.ConnectPathByLineNo( DriveMdl, dMaxDist,m_cSetupDef.m_cTolDef,
                                     PrgDef, TRUE, dCenter,&(m_cFeedDef.m_cConnectDef)) ;
	}
    else 
    {
       if( GetMillDir() == 1)
       { /*从外向内*/
           AllPath.ReverseDirect() ;
       }
       AllPath.CreateLeadPath( DriveMdl, m_cSetupDef.m_cTolDef, m_cFeedDef.m_cConnectDef) ;
    }
    SmartNC_RegCreateConnect3DPath( NULL );
	return TRUE  ; 

}
static BOOL MathCAM_CreateHeightStepTLib( CSmartGraphic& Graph  , 
                                          JDNC_SETUP&    Setup  ,
                                          CSmtCheckMdl& DriveMdl,
                                          CSmtCPathLib& GuideLib,
                                          JDNC_PRGDEF&  PrgDef  )
{
    CSmtCutPath tmpPath ;
    JDNC_PRGDEF tmpPrg = PrgDef ;
    tmpPrg.m_pPosFunc = NULL ; 
    tmpPrg.m_pPrgFunc = NULL ; 
    CPtrList AllList ;
    Graph.GetAllEntity( Setup, AllList, SMARTGRAPH_TYPE_GUIDECURVE );
    if( AllList.GetCount() == 0 ) return FALSE ;
    while( AllList.GetCount() ) 
    {
        CSmartCurve* pCurve = (CSmartCurve*) AllList.RemoveHead() ;
        tmpPath.ClearAllPoint() ;
        tmpPath.AddCurve( *pCurve, Setup.m_cTolDef ) ;
        delete pCurve ;
        CSmtCutPoint* pTPnt, *pTNew ;
        for( pTPnt = tmpPath.m_pHead; pTPnt; pTPnt = pTPnt->next )
        {
            pTPnt->m_fPoint[2] = DriveMdl.m_fBottom ;
        }
        tmpPath.InsertCPoint(  Setup.m_cTolDef.m_dMaxStep ) ; 
        tmpPath.VerifyCutPath( DriveMdl, Setup.m_cTolDef, tmpPrg ) ;
        for( pTPnt = tmpPath.m_pHead ; pTPnt ; pTPnt = pTPnt->next )
        {
            pTPnt->m_bType = SMART_CUTPNT_TRIA ;
            if( pTPnt->next && pTPnt->m_fPoint[2] == pTPnt->next->m_fPoint[2] )
            {
                if( pTPnt->prev == NULL || pTPnt->m_fPoint[2] == pTPnt->prev->m_fPoint[2]  )
                {
                    pTPnt->m_bType = 0 ;
                }
            }
            if( pTPnt->prev && pTPnt->m_fPoint[2] == pTPnt->prev->m_fPoint[2] )
            {
                if( pTPnt->next == NULL || pTPnt->m_fPoint[2] == pTPnt->next->m_fPoint[2]  )
                {
                    pTPnt->m_bType = 0 ;
                }
            }
        }
        for( pTPnt = tmpPath.m_pHead ; pTPnt; pTPnt = pTPnt->next )
        {
            if( pTPnt->prev == NULL ) continue ;
            if( pTPnt->m_bType && pTPnt->prev->m_bType && 
                pTPnt->m_fPoint[2] == pTPnt->prev->m_fPoint[2] )
            {
                pTNew = pTPnt->prev->GenMidPoint( pTPnt, 0.5f ) ;
                pTNew->m_bType = 0 ;
                tmpPath.InsertAfter( pTNew, pTPnt->prev  ) ;
            }
        }
        for( pTPnt = tmpPath.m_pHead ; pTPnt ; pTPnt = pTPnt->next )
        {
            if( pTPnt->prev == NULL || pTPnt->next == NULL ) 
            {
                continue ;
            }
            if( pTPnt->prev->m_bType == 0 || pTPnt->m_bType == 0 || pTPnt->next->m_bType == 0 )
            {
                continue ;
            }
            if( pTPnt->prev->m_fPoint[2] > pTPnt->m_fPoint[2] && 
                pTPnt->next->m_fPoint[2] > pTPnt->m_fPoint[2] || 
                pTPnt->prev->m_fPoint[2] < pTPnt->m_fPoint[2] && 
                pTPnt->next->m_fPoint[2] < pTPnt->m_fPoint[2]  )
            {
                pTNew = pTPnt->CopyMyself() ;
                tmpPath.InsertAfter( pTNew, pTPnt->prev ) ;
                pTNew = pTPnt->CopyMyself() ;
                pTNew->m_bType = 0 ;
                tmpPath.InsertBefore( pTNew, pTPnt ) ;
            }
        }
        CSmtCutPath* pNewPath = tmpPath.BreakAtNullPoint( FALSE ) ;
        while( pNewPath )
        {
            CSmtCutPath* pTNext = pNewPath->next ;
            pNewPath->next = pNewPath->prev = NULL ; 
            pNewPath->DefineBox() ;
            if( pNewPath->m_pHead->m_fPoint[2] < pNewPath->m_pTail->m_fPoint[2] )
            {
                pNewPath->ReverseDirect() ;
            }
            GuideLib.AddToTail( pNewPath ) ;
            pNewPath = pTNext ;
        }
    }
    if( GuideLib.m_cAllPath.GetCount() == 0 )
    {
        return FALSE ;
    }
    return TRUE ;
}

BOOL CreateHeightLayers( DOUBLE ZTop, DOUBLE ZBottom, DOUBLE ZStep, double *&dZ, int &nCnt  ) 
{
	// 避免加工到平面，表面余量+0.001
	double dCutDepth = ZTop - ZBottom ;
	nCnt = ( int ) ceil( dCutDepth / ZStep - 1.0e-4 ) ;
	if( nCnt < 1 ) nCnt = 1 ;
	dZ = new double[nCnt] ;
	// 得到分层的位置
    DOUBLE dZAt = ZBottom ; 
    for( int i = nCnt-1 ; i >= 0 ; i -- )
    {
        dZ[i] = dZAt ;
        dZAt += ZStep ;
    }
	return TRUE ;
}
void FilterPlaneHeightByLayer( double *dLayer, int nCnt, double *&dSurfZ, int &nSurf, double dTol )
{
	// 将dSurfZ从高到低排序
	int i = 0, j = 0, n = nSurf / 2 ;
	double dTmpZ = 0 ;
	for( i = 0 ; i < n ; i++ )
	{
		dTmpZ = dSurfZ[i] ;
		dSurfZ[i] = dSurfZ[nSurf-i-1] ;
		dSurfZ[nSurf-i-1] = dTmpZ ;
	}
    // 对dSurfZ进行过滤,删除比dLayer中高dTol的层
	BOOL * bFlag = new BOOL[nSurf], bFind = FALSE ;
	memset( bFlag, 1, nSurf * sizeof(BOOL) ) ;

	for( i = 0 ; i < nSurf ; i++ )
	{
		// 如果该深度和等高的分层很近,并在分层上方方,则该层可以去掉
		for( j = 0 ; j < nCnt ; j++ )
		{
			if( fabs( dLayer[j] - dSurfZ[i] ) < dTol )
			{
				if( dSurfZ[i] > dLayer[j] )
				{ // 将等高层赋值为平面层,平面层优先.
					dLayer[j] = dSurfZ[i] ;
				}
                bFlag[i] = FALSE, bFind = TRUE ;
				break ;
			}
			else if( dLayer[j] < dSurfZ[i] )
				break ;

		}
	}
	if( bFind )
	{
		double *dZ = new double[nSurf] ;
		int nCount = 0 ;
		for( i = 0 ; i < nSurf ; i++ )
		{
			if( bFlag[i] )
			{
				dZ[nCount] = dSurfZ[i] ;
				nCount++ ;
			}
		}
		delete[] dSurfZ ;
		dSurfZ = dZ ;
		nSurf = nCount ;
	}
	delete[] bFlag ;
}

void MachCam_ConnectHeightPath(CSmtCheckMdl& DriveMdl,JDNC_SETUP &SetupDef,
							   JDNC_CONNECT3D & CnnctDef, JDNC_PRGDEF& PrgDef,
							   double OverStep, CSmtCPathLib &AllPath)
{
	CSmtConnectPathMdl  cntMdl ;
	cntMdl.m_dOverStep = OverStep;
	cntMdl.ConnectHeightPath(AllPath, DriveMdl, SetupDef, CnnctDef, PrgDef) ;
}

void MathCam_LayerAllSurf( double dTop, double dBot, JDNC_TOL &cTol, JDNC_FEED & cFeedDef, 
						   double *&dSurfZ, int nSurf, double *&dZ, int nCnt, 
						   BOOL bInit, BOOL bFlat, CHtIntStrSetLib2D &IntCLib,
						   double *&dAlldZ, BOOL *&bFlag, int &nCount )
{
	if( bInit )
        CreateHeightLayers( dTop, dBot, cFeedDef.m_cStepDef.m_dOverStep, dZ, nCnt ) ;
	if( bFlat && dSurfZ )
	{ 
		// 过滤平面分层
		FilterPlaneHeightByLayer( dZ, nCnt, dSurfZ, nSurf, 0.002 ) ;
		//合并分层
		CombineAllHeight( dZ, nCnt, dSurfZ, nSurf, dAlldZ, nCount, bFlag ) ;
		if( dZ ) delete[] dZ ;
		bFlat = TRUE ;
	}
	else
	{
		dAlldZ = dZ ;
		nCount = nCnt ;
		bFlag = new BOOL[nCount] ;
		memset( bFlag, 1, nCount * sizeof( BOOL ) ) ;
	}
	if( nCount > 1 && fabs( dAlldZ[0] - dTop ) < cTol.m_dArcTol * 2. )
	{
		for( int i = 1 ; i < nCount ; i++ )
		{
			dAlldZ[i-1] = dAlldZ[i] ;
			bFlag[i-1]  = bFlag[i]  ;
		}
		nCount -= 1 ;
	}
	IntCLib.CreateLayers( dAlldZ, nCount, bFlag ) ;
	if( bFlag ) delete[] bFlag ;
	if( dAlldZ ) delete[] dAlldZ ;
}

void CSmartAutoFinishGen::ExtendFHeightPath(CSmtCPathLib & AllPath, CSmtCheckMdl& DriveMdl)
{
	if (!(m_cMethodDef.m_cSFinishCut.m_cHeightCut.m_bHeightFlag&NCDEF_HEIGHT_LOCALFEATURE))
	{
		return ;
	}
	//非闭合等高路径向外直线延伸
	CHeightExtendOut cHtExtdOut;
	JDNC_HOLDER cTHolder;
	GetToolHolder(cTHolder);
	double dExtendDist = m_cMethodDef.m_cSFinishCut.m_cHeightCut.m_dExtendOut;
	cHtExtdOut.ExtendOutSide(AllPath, dExtendDist, m_cSetupDef, cTHolder, DriveMdl, 0);

	//路径向上延伸
	if ( IsDelBndPoint() == TRUE )
	{
		dExtendDist = m_cMethodDef.m_cSFinishCut.m_cHeightCut.m_dExtendUp;
		dExtendDist = min( dExtendDist, m_pTool->m_fRadius * 20 ) ;
		BOOL is3DStep = (m_cSetupDef.m_cModelTol.m_nMdlFlag & NCDEF_SETUP_3DSTEP) ;
		ExtendUpHeigtPath( DriveMdl, AllPath, dExtendDist, m_cFeedDef.m_cStepDef.m_dOverStep, is3DStep ) ;
	}
}
int CSmartAutoFinishGen::CreateHeightPath( CSmtCheckMdl& DriveMdl , /*网格模型  */ 
                                           CSmartLoop*  AllCont   , /*边界裁剪  */
									       int 		   ValidMode  , /*计算模式  */
                                           BOOL        IsCutPlane , /*是否增加平面分层  */
                                           DOUBLE      SlopeAngle , /*刀触点模式时的分区角度*/
                                           CSmtCPathLib& AllPath  , /*加工路径  */
                                           JDNC_PRGDEF&  PrgDef )
{
		FILE *fp;
		fp = fopen("C:\\Users\\QQS\\Desktop\\1.txt","w+");
		DWORD st_time, end_time;
		st_time = GetTickCount();
	NcBnd_SetProgress ( m_cPrgDef.m_pNewFunc, m_cPrgDef.m_pPosFunc, m_cPrgDef.m_pBrkFunc ) ; 
    BOOL bOldMode = DriveMdl.SetTouchCalcMode( FALSE ) ;
	double dTop = GetTopHeight() ;
	double dBot = GetBottomHeight() ;
	// 需要多线程的拷贝，则拷贝一份DriveMdl
	BOOL bLocalCopy = FALSE ;	
	if (IsNeedMultiCopy(DriveMdl))
	{
		bLocalCopy = TRUE ;
		DriveMdl.CreateMultiCopy(m_nCalcThreadNum - 1) ;
	}	
	//STEP 1 : 构建刀位点网格模型
	CNcZLevelGenMdl   HeightMdl ;
    HeightMdl.m_nValidMode = ValidMode ;
    HeightMdl.SetSlopeAngle( SlopeAngle ) ;
    double dTotalMove = PrgDef.m_dTotalMove ;
    PrgDef.m_dTotalMove = 0.2 * dTotalMove ;
    if ( !HeightMdl.Create ( DriveMdl, AllCont, m_cSetupDef , PrgDef) )
    {
        return FALSE ;
    }
	int i = 0 ;
	for( i = 0 ; i < HeightMdl.m_cSameZSet.m_nCount ; i ++ )
	{
		if ( fabs ( dBot - HeightMdl.m_cSameZSet.m_dValue[i] ) < m_cSetupDef.m_cTolDef.m_dArcTol 
			&& dBot < dTop )
        {
			dBot += m_cSetupDef.m_cTolDef.m_dArcTol ;
        }
	}
	double dMinStepZ = m_cFeedDef.m_cStepDef.m_dOverStep / 4 ;
	for ( i = 0 ; i < HeightMdl.m_cSameZSet.m_nCount ; i ++ )
	{
		if ( dBot < HeightMdl.m_cSameZSet.m_dValue[i] && 
			 HeightMdl.m_cSameZSet.m_dValue[i] - dBot < dMinStepZ && 
			 HeightMdl.m_cSameZSet.m_dValue[i] < dTop )
		{
			dBot = HeightMdl.m_cSameZSet.m_dValue[i] ;
			break ;
		}
	}
    int nSameCnt = 0 ;
	double *dSurfZ = NULL ;
	BOOL bFlat = FALSE ;
	if( IsCutPlane && HeightMdl.m_cSameZSet.m_nCount > 0 )
	{
        double dZTop = GetTopHeight() + 0.0002;
        double dZBtm = GetBottomHeight() - 0.0002;
		dSurfZ = new double[HeightMdl.m_cSameZSet.m_nCount] ;
        nSameCnt = 0 ;
		for( i = 0 ; i < HeightMdl.m_cSameZSet.m_nCount ; i++ )
        {
            if( HeightMdl.m_cSameZSet.m_dValue[i] < dZTop && 
                HeightMdl.m_cSameZSet.m_dValue[i] > dZBtm    )
            {
			    dSurfZ[nSameCnt] = HeightMdl.m_cSameZSet.m_dValue[i] ;
                nSameCnt ++ ;
            }
        }
		DriveMdl.DeleteInvalidPlane ( dSurfZ, nSameCnt ) ;
		for( i = 0 ; i < nSameCnt ; i++ )
        {
			dSurfZ[i] += 0.0002 ;
        }
		while( nSameCnt > 0 && dSurfZ[0] < dBot )
		{
			for( i = 0 ; i < nSameCnt - 1 ; i++ )
			{// 去掉比加工底面低的层
				dSurfZ[i] = dSurfZ[i+1] ;
			}
			nSameCnt-- ;
		}
		if( nSameCnt > 0 ) bFlat = TRUE ;
	}
	//STEP 2 : 计算等高路径
    CHtIntStrSetLib2D IntCLib ;
    PrgDef.m_dTotalMove = 0.5 * dTotalMove ;


	int		nCnt = 0, nCount = 0 ;
	double *dZ = NULL, *dAlldZ = NULL  ;
	BOOL *bFlag = NULL ;
	CSmtCPathLib GuideLib ;
	if( (m_cMethodDef.m_cSFinishCut.m_bMoveFlag & NCDEF_SFINISH_GUIDESTEP3D ) && 
        MathCAM_CreateHeightStepTLib(*m_pSmtGraph,m_cSetupDef,DriveMdl,GuideLib,PrgDef))
	{//生成依据导动线分布的z间距
		BOOL bRet = HeightMdl.GenZStepsFromGuideCurve( GuideLib, m_cFeedDef.m_cStepDef.m_dOverStep, 
					                                    dTop, dBot,FALSE,dSurfZ,nSameCnt, dZ, nCnt ) ;
		if( !bRet )
		{
			if( dSurfZ ) delete[] dSurfZ ;
			return FALSE ;
		}
		//增加平面分层
		MathCam_LayerAllSurf( dTop, dBot, m_cSetupDef.m_cTolDef, m_cFeedDef, dSurfZ, nSameCnt, dZ, nCnt, 
								FALSE, bFlat, IntCLib, dAlldZ, bFlag, nCount ) ;
		if( !HeightMdl.GenIntCurveSetLib ( IntCLib, PrgDef ) )
		{
			if( dSurfZ ) delete[] dSurfZ ;
			return FALSE ;
		}
		IntCLib.TrimByContours ( AllCont, AllPath, NCDEF_HEIGHT_VERT ) ;
	}
	else if( m_cSetupDef.m_cModelTol.m_nMdlFlag & NCDEF_SETUP_3DSTEP ) 
	{//自适应计算生成生成空间间距
		double dTotalMove = PrgDef.m_dTotalMove ;
		if( bFlat )
		{
			PrgDef.m_dTotalMove = dTotalMove * 0.8 ;
		}

		double dMinZStep = 0.1 * m_cFeedDef.m_cStepDef.m_dOverStep;
		CJDJob* pJDJob = SurfNC_GetCurJobSetup();
		if(pJDJob)
		{//得到系统设置的最小层深
			dMinZStep = max(dMinZStep , pJDJob->m_dDepthDm[0]);
		}
		
		HeightMdl.GenContour3DPath( dTop, dBot,  m_cFeedDef.m_cStepDef.m_dOverStep, 
                                    dMinZStep,AllPath,PrgDef ) ;
		if( bFlat )
		{// 平坦面分层
			PrgDef.m_dTotalMove = dTotalMove * 0.2 ;
			bFlag = new BOOL[nSameCnt] ;
			memset( bFlag, 0, nSameCnt *sizeof( BOOL ) ) ;
			IntCLib.CreateLayers( dSurfZ, nSameCnt, bFlag ) ;
			delete[] bFlag ;
			HeightMdl.GenIntCurveSetLib ( IntCLib, PrgDef ) ;
		}
	}
	else
	{//按照高度间距分层
		MathCam_LayerAllSurf( dTop, dBot, m_cSetupDef.m_cTolDef, m_cFeedDef, dSurfZ, nSameCnt, dZ, nCnt, 
							  TRUE, bFlat, IntCLib, dAlldZ, bFlag, nCount ) ;
        if( ! HeightMdl.GenIntCurveSetLib( IntCLib, PrgDef ) )
        {
			if( dSurfZ ) delete[] dSurfZ ;
            return FALSE ;
        }
		IntCLib.TrimByContours ( AllCont, AllPath, NCDEF_HEIGHT_VERT ) ;
	}
	if( dSurfZ ) delete[] dSurfZ ;
	// DriveMdl的数据在本函数拷贝，则在本函数删除
	if( bLocalCopy == TRUE ) 
	{
		DriveMdl.DeleteMultiCopy() ;
	}
    if( SurfNC_IsAbort() )
    {
		m_nErrorType = JDERROR_GENPATH_ABORT ;
		return FALSE;
    }
	//STEP 3 : 将平面加工路径添加到AllPath中
	if( bFlat )
	{
        int nMaxLineNo  = 0 ;
        POSITION atTPos = AllPath.m_cAllPath.GetHeadPosition() ;
        while( atTPos )
        {
            CSmtCutPath* pTPath = AllPath.m_cAllPath.GetNext( atTPos ) ;
            if( pTPath->m_nLineNo > nMaxLineNo )
            {
                nMaxLineNo = pTPath->m_nLineNo ;
            }
        }
        nMaxLineNo += 2 ;
		PrgDef.m_dTotalMove = 0.05 * dTotalMove ;
		CSmtCPathLib FlatPath ;
		IntCLib.TrimByContours( AllCont, FlatPath, NCDEF_HEIGHT_FLAT ) ;
		IntCLib.TrimByPlaneShape( FlatPath, DriveMdl, m_cSetupDef.m_cTolDef.m_dArcTol ) ;
		TrimAllPathByHeight( FlatPath, dTop + m_cSetupDef.m_cTolDef.m_dArcTol, dBot - m_cSetupDef.m_cTolDef.m_dArcTol ) ;
        while( FlatPath.m_cAllPath.GetSize() )
        {
            CSmtCutPath* pTPath = FlatPath.m_cAllPath.RemoveHead() ;
            pTPath->m_nLineNo += nMaxLineNo ;
            AllPath.AddToTail( pTPath ) ;
        }
    }

	if(m_cMethodDef.m_cSFinishCut.m_nMoveType == surfncMoveHeight)
	{
		ExtendFHeightPath(AllPath, DriveMdl);
	}
	end_time = GetTickCount();

	
	//float time = 4.5;
	fprintf(fp,"前期路径时间：%d\n",end_time-st_time);
	st_time = GetTickCount();
	
	//STEP 4 :路径连接
	JDNC_CONNECT3D  tmpCnt = m_cFeedDef.m_cConnectDef ;
	tmpCnt.m_dSafeDist= GetLoopCheckTol() ;
	SmartNC_RegCreateConnect3DPath( MathCAM_GenHeightLeadPathEx );
	end_time = GetTickCount();
	fprintf(fp,"LeadPath路径时间：%d\n",end_time-st_time);
	if( !bFlat )
        PrgDef.m_dTotalMove = 0.3 * dTotalMove ;
	else
		PrgDef.m_dTotalMove = 0.25 * dTotalMove ;

	//保留锋利边界(会移动路径修改路径起末点，所以放在螺旋连刀前)
	KeepSharpEdges(AllPath, DriveMdl);
	st_time = GetTickCount();
	MachCam_ConnectHeightPath(DriveMdl, m_cSetupDef, tmpCnt, PrgDef, 
							  m_cFeedDef.m_cStepDef.m_dOverStep, AllPath ) ;
	end_time = GetTickCount();
	fprintf(fp,"螺旋连刀路径时间：%d\n",end_time-st_time);
	SmartNC_RegCreateConnect3DPath( NULL );
		
	if( SurfNC_IsAbort() )
	{
		m_nErrorType = JDERROR_GENPATH_ABORT ;
		return FALSE;
	}
	NcBnd_SetProgress ( NULL, NULL, NULL ) ; 
    DriveMdl.SetTouchCalcMode( bOldMode ) ;
	fclose(fp);
	return TRUE ;
}

int CSmartAutoFinishGen::CreateMPencilPath( CSmtCheckMdl& DriveMdl , /*网格模型  */ 
                                            CSmartLoop*  AllCont  , /*边界裁剪  */
                                            CSmtCPathLib& AllPath , /*加工路径  */
                                            JDNC_PRGDEF&  PrgDef )
{
	//STEP 1 : 构建刀位点网格模型
    DOUBLE dTotalMove = PrgDef.m_dTotalMove ;
	CSmtMeshMdl   MeshMdl ;
    MeshMdl.m_nMeshMode = SMARTNC_MESHMDL_MUTIPENCIL ;
    m_cPrgDef.m_dTotalMove = 0.1 * dTotalMove ;
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
    if( BuildMeshMdl( MeshMdl, DriveMdl, bndBox,TRUE ) == FALSE )
    {
       m_nErrorType = JDERROR_GENPATH_ABORT ;
       return FALSE ;
    }
	//　STEP 3 : 计算加工路径
    CSmtCPathLib  AutoBnd ;
#ifdef _DEBUG
    if( MeshMdl.m_bDumpXYPath && m_pDebugPath)
    {
        MeshMdl.DumpMeshToPathGroup( *m_pDebugPath ) ;
        return TRUE ;
    }
#endif 
    PrgDef.m_dTotalMove = 0.1 * dTotalMove ;
    if( !MeshMdl.GenMeshSelfBound( AutoBnd, PrgDef ) )
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
    JDNC_CLEANUP pencilParam ;
    SurfNC_InitPathParam( pencilParam ) ;
    pencilParam.m_nCleanupType = NCDEF_CLEANUP_MPENCIL ;
    pencilParam.m_nPencilType  = NCDEF_PENCIL_UNLIMITED; 
    PrgDef.m_dTotalMove = 0.5 * dTotalMove ;
    if( ! MeshMdl.CreateMutiplePencilPath( pencilParam ,  //清根参数
                                           bndTrim ,
                                           (TFLOAT)m_cFeedDef.m_cStepDef.m_dOverStep,
                                           AllPath, //返回路径
                                           *m_pDebugPath,
                                           PrgDef ))//调试路径
    {
        Mini_DeleteContours( bndTrim ) ;
        if( SurfNC_IsAbort() ) m_nErrorType = JDERROR_GENPATH_ABORT ;
        else  m_nErrorType = JDERROR_GENPATH_NOAREA ;
        return FALSE ;
    }
    Mini_DeleteContours( bndTrim ) ;
	//STEP 4 :路径连接
    JDNC_CONNECT3D  tmpCnt3D = m_cFeedDef.m_cConnectDef ;
    tmpCnt3D.m_dSafeDist = m_cFeedDef.m_cStepDef.m_dOverStep ;
	if( m_cSetupDef.m_cOrderDef.m_nLayerOrd == 1 && 
		tmpCnt3D.m_dSafeDist <m_pTool->m_fRadius * 2.0 )
	{ //高度优先
		tmpCnt3D.m_dSafeDist = m_pTool->m_fRadius * 2.0 ;
	}
    SmartNC_RegCreateConnect3DPath( MathCAM_GenNormalLeadPathEx );
    PrgDef.m_dTotalMove = 0.3 * dTotalMove ;
	if ( m_cSetupDef.m_cModelTol.m_nMdlFlag & NCDEF_SETUP_SMOOTHCORNER )
	{/*光滑尖角*/
		AllPath.SmoothAllPath ( DriveMdl, m_cSetupDef, m_cPrgDef ) ;
	}
	if (m_cFeedDef.m_cConnectDef.m_bConnect3DFlag & NCDEF_FCONNECT3D_SPIRAL ) 
	{/*螺旋连接*/
		NcBound_ConnectAllPath( DriveMdl, m_cSetupDef, m_cFeedDef,
			m_cFeedDef.m_cStepDef.m_dOverStep,AllPath, 
			PrgDef,tmpCnt3D.m_dMaxDist,TFLOAT(25.0*m_cFeedDef.m_cStepDef.m_dOverStep)) ;
	}
	else 
	{//根据行号连接多段刀具路径
		AllPath.ConnectPathByLineNo3D( DriveMdl,m_cSetupDef.m_cTolDef, tmpCnt3D, PrgDef);
	}
    SmartNC_RegCreateConnect3DPath( NULL );
    if( SurfNC_IsAbort() )
    {
		m_nErrorType = JDERROR_GENPATH_ABORT ;
		return FALSE;
    }
    return TRUE ;
}
int CSmartAutoFinishGen::CreateUWLinePath( CSmtCheckMdl& DriveMdl , /*网格模型  */ 
                                           CSmartLoop*  AllCont  , /*边界裁剪  */
                                           CSmtCPathLib& AllPath , /*加工路径  */
                                           JDNC_PRGDEF&  PrgDef )
{
	NcBnd_SetProgress ( m_cPrgDef.m_pNewFunc, m_cPrgDef.m_pPosFunc, m_cPrgDef.m_pBrkFunc ) ; 
	//改变计算精度
	m_cSetupDef.m_cTolDef.m_dArcTol *= 0.5 ;
    DOUBLE dTotalMove = PrgDef.m_dTotalMove ;
	//Buile Mesh Model
	CSmtMeshMdl meshMdl ;
    m_cPrgDef.m_dTotalMove = 0.15 * dTotalMove ;
    if( BuildMeshMdl( meshMdl, DriveMdl, AllCont, TRUE ) == FALSE )
    {
       m_nErrorType = JDERROR_GENPATH_ABORT ;
       return FALSE ;
    }
	CSmtCPathLib boundPath ;
    PrgDef.m_dTotalMove = 0.15 * dTotalMove ;
	meshMdl.GenMeshSelfBound ( boundPath, PrgDef ) ;

	//恢复计算精度
	m_cSetupDef.m_cTolDef.m_dArcTol *= 2 ;
    PrgDef.m_dTotalMove = 0.7 * dTotalMove ;
	NcBnd_GenUWLinePath ( DriveMdl, meshMdl, boundPath, m_cSetupDef, m_cFeedDef, AllPath ) ;
 
	NcBnd_SetProgress ( NULL, NULL, NULL ) ; 
    return TRUE ;
}


void CSmartAutoFinishGen::SetCheckFltBoxByContours( CSmartLoop* AllCont, 
                                                    CSmtCheckMdl& DriveMdl ,
                                                    CSmtCheckMdl& CheckMdl )
{/*根据边界设置过滤的曲面*/
    if( ! AllCont ) 
    {// 没有选择边界曲线
        return ;
    }
    PNT2D	dBox2D[2];
    MathCAM_CalcContourBox( AllCont, dBox2D ) ;
    double	dExtLen = 2.0+m_cFeedDef.m_cConnectDef.m_dLead3DRadius + m_cFeedDef.m_cConnectDef.m_dLead3DLine;
    if( m_pSafeTool )
    {
        dExtLen += max( m_pSafeTool->m_fRadius, m_pTool->m_fRadius );
    }
    else
    {
        dExtLen += m_pTool->m_fRadius ;
    }
	for(int  k = 0 ; k < 2; k++)
	{
        CheckMdl.m_dFltBox[0][k] = DriveMdl.m_dFltBox[0][k]=dBox2D[0][k]-dExtLen;
		CheckMdl.m_dFltBox[1][k] = DriveMdl.m_dFltBox[1][k]=dBox2D[1][k]+dExtLen;
	}
	CheckMdl.m_dFltBox[0][2] = DriveMdl.m_dFltBox[0][2]=-MAX_DBL; 
	CheckMdl.m_dFltBox[1][2] = DriveMdl.m_dFltBox[1][2]=MAX_DBL;  
}
void CSmartAutoFinishGen::SetCheckFltBoxByDriveMdl( CSmtCheckMdl& DriveMdl ,
                                                    CSmtCheckMdl& CheckMdl )
{/*根据边界设置过滤的曲面*/
    FPNT3D  fBox3D[2] ;
    if( ! DriveMdl.CalcFacetBox( fBox3D[0], fBox3D[1] ))
    {
        return ;
    }
    double	dExtLen = 2.0+m_cFeedDef.m_cConnectDef.m_dLead3DRadius + m_cFeedDef.m_cConnectDef.m_dLead3DLine;
    if( m_pSafeTool )
    {
        dExtLen += max( m_pSafeTool->m_fRadius, m_pTool->m_fRadius );
    }
    else
    {
        dExtLen += m_pTool->m_fRadius ;
    }
    fBox3D[0][0] -= (TFLOAT)dExtLen, fBox3D[0][1] -= (TFLOAT)dExtLen;
    fBox3D[1][0] += (TFLOAT)dExtLen, fBox3D[1][1] += (TFLOAT)dExtLen;
    if( CheckMdl.m_dFltBox[0][0] < CheckMdl.m_dFltBox[1][0] && 
        CheckMdl.m_dFltBox[0][1] < CheckMdl.m_dFltBox[1][1]   )
    {/*挡墙包围盒有效,缩小包围盒*/
        for( int k = 0 ; k < 2 ; k ++ ) 
        {
            if( CheckMdl.m_dFltBox[0][k] < fBox3D[0][k]) 
            {
                CheckMdl.m_dFltBox[0][k] = fBox3D[0][k] ;
            }
            if( CheckMdl.m_dFltBox[1][k] > fBox3D[1][k]) 
            {
                CheckMdl.m_dFltBox[1][k] = fBox3D[1][k] ;
            }
            if( CheckMdl.m_dFltBox[1][k] < CheckMdl.m_dFltBox[0][k]+0.1 )
            {
                CheckMdl.m_dFltBox[1][k] = CheckMdl.m_dFltBox[0][k] + 0.1 ;
            }
        }
    }
    else
    {
        nc_FloatToDouble( CheckMdl.m_dFltBox[0], fBox3D[0], 2 ) ;
        nc_FloatToDouble( CheckMdl.m_dFltBox[1], fBox3D[1], 2 ) ;
	    CheckMdl.m_dFltBox[0][2] = -MAX_DBL; 
	    CheckMdl.m_dFltBox[1][2] = MAX_DBL;  
    }
}

CSmartTool* MathCAM_FilletTCorner( DOUBLE& SlopeAngle,  // 分区角度, 5 ~ 85 度
                                   CSmartTool& OrgTool) // 当前刀具
{
    if( SlopeAngle < 0.0 ) return NULL ; 
    if( OrgTool.GetType() == smtToolBall )
    {/*校正最小角度*/
        if( OrgTool.m_fRadius > 0.03 )
        {
            double dMaxSlope = RADIAN_TO_ANGLE ( asin((OrgTool.m_fRadius-0.02)/OrgTool.m_fRadius) ) ;
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
    DOUBLE dCorner = max( min(0.1,dRadius*0.8), 0.02 / ( 1.0 - sin( ANGLE_TO_RADIAN(SlopeAngle) ) ) ) ;
    dCorner = min( dCorner, 0.8 * dRadius ) ;
    if( dCorner > 0.03 )
    {
        double dMaxSlope = RADIAN_TO_ANGLE ( asin((dCorner-0.02)/dCorner) ) ;
        if( SlopeAngle > dMaxSlope ) SlopeAngle = dMaxSlope ;
    }
    if( OrgTool.GetType() == smtToolNose && 
        dCorner < OrgTool.m_fCorner-0.001 )
    {/*不需要倒圆角*/
        return NULL ;
    }
    CSmartTool* pNoseTool = new CSmtToolNose() ;
    pNoseTool->SetParam( dRadius, dRadius - dCorner,0.0, dCorner ) ; 
    pNoseTool->RefineParam() ;
    return pNoseTool ;
}
BOOL CSmartAutoFinishGen::CreateBoundPath( CSmtCheckMdl& DriveMdl ,// 加工模型
                                           CSmartLoop*   AllCont  ,// 区域范围
                                           CSmtCPathLib& BndPath  )// 边界路径
{
    // STEP 1 : 计算网格模型
    m_cPrgDef.m_dTotalMove = 0.0 ;
    CSmtMeshMdl   MeshMdl;
    MeshMdl.m_nMeshMode = SMARTNC_MESHMDL_FINISHBND  ;
    if( !BuildMeshMdl( MeshMdl, DriveMdl, AllCont,m_bTrimByZSize ) ) 
    {
        return FALSE ;
    }
    // STEP 2 : 搜索原始边界
    JDNC_PRGDEF tmpPrgDef = m_cPrgDef ;
    tmpPrgDef.m_dTotalMove= 0.0 ;
    tmpPrgDef.m_pPosFunc = NULL ; 
    tmpPrgDef.m_pPrgFunc = NULL ;
	if( !MeshMdl.GenMeshSelfBound( BndPath, tmpPrgDef ) )
    {
        return FALSE ;
    }
    return TRUE ;
}
BOOL CSmartAutoFinishGen::CreateFlatAreas( CSmtCheckMdl& DriveMdl ,   // 加工模型
                                           CSmartLoop*   AllCont  ,   // 区域范围
                                           DOUBLE SlopeAng,           // 分区角度, 5 ~ 85 度
                                           CSmartLoop*& FlatAreaBnd,  // 平坦区域边界
                                           DOUBLE  UsedBndTol    ,    // 计算修边路径  
                                           JDNC_PRGDEF&  PrgDef ,
										   double dStep,              // 路径间距
										   BOOL bFollow,              // 平坦部分是否环切
										   CSmartLoop*& SteepAreaBnd ,// 陡峭区域边界
										   DOUBLE dOverLapDist)       // 重叠区域宽度
{
    DOUBLE dSideTol = 0.0, SlopeAngle = SlopeAng ;
    if( UsedBndTol >= 0.0 || SlopeAngle == 0.0 ) 
    {
        dSideTol = UsedBndTol;
        if( dSideTol >= 1.0e-3  )
        {/* 校正轮廓*/
            dSideTol = max( dSideTol, m_cSetupDef.m_cTolDef.m_dArcTol * 2.0 ) ; 
        }
        if( SlopeAngle == 0.0 ) SlopeAngle = 0.725 ;
    }
    DOUBLE  dArcTol = min( 0.005, m_cSetupDef.m_cTolDef.m_dArcTol );   
    // STEP 1 : 修正平底刀具及尺寸,提高边界质量和稳定性
    CSmartTool*  pOrgDriveTool = NULL, *pNoseTool = NULL ; 
    pNoseTool = MathCAM_FilletTCorner( SlopeAngle, *(DriveMdl.m_pTool)) ;
    if( pNoseTool )
    {/*平底刀或小圆角牛鼻刀需要修正圆角半径 */
        pOrgDriveTool = DriveMdl.m_pTool ;
        DriveMdl.UpdateCurrTool( pNoseTool ) ;
    }
    // STEP 3 : 构建网格模型, 提取可加工区域的边界线
    DOUBLE dTotalMove = PrgDef.m_dTotalMove ;
    CSmtMeshMdl   MeshMdl;
    if( SlopeAngle >= 0.0  )
    {/*角度分区模式*/
        MeshMdl.m_nMeshMode = SMARTNC_MESHMDL_MIXTURE  ;
    }
    m_cPrgDef.m_dTotalMove = 0.3 * dTotalMove;
	/*删除边界路径点*/
    int bRet = BuildMeshMdl( MeshMdl, DriveMdl, AllCont,m_bTrimByZSize ) ;
    if( bRet )
    {// STEP 4 : 设置平行截线的兜边量
        CSmtCPathLib  BndPath ;
		CSmartLoop tmpLp ;
        PrgDef.m_dTotalMove = 0.7 * dTotalMove;
		/*所有曲面*/
		MeshMdl.GenMeshSelfBound( BndPath, PrgDef ) ;
        if( SlopeAngle <= 0.0 )
        {	        
            DOUBLE dOffset = 0.0 ;
            if( dSideTol  <= 1.0e-6 && !IsDelBndPoint())  dOffset = - 2.0 * dArcTol ;
            else dOffset = max( 2.0f * dArcTol, dSideTol );
            FlatAreaBnd = BndPath.ConvertToContours(FALSE, m_cSetupDef.m_cCorDef,dOffset );
        }
        else
        {//平截线平坦面
			CSmtCPathLib  FlatBndPath ;			
		    MeshMdl.ExtractFlatAreaBound( (TFLOAT)SlopeAngle, FlatBndPath,PrgDef ) ;						
			TFLOAT fZMax  = (TFLOAT) GetTopHeight()  ;
			TFLOAT fZMin  = (TFLOAT) GetBottomHeight() ;
			MeshMdl.LabelPointFlagByZValue( fZMin, fZMax ) ;

			//计算得出包括重叠区域的陡峭环  qqs、xiejunxian 2013.04.25
			CSmartLoop* baseBnd = BndPath.ConvertToContours(FALSE, m_cSetupDef.m_cCorDef, 2.0f * dArcTol );
			CSmartLoop* flatBnd = FlatBndPath.ConvertToContours(FALSE, m_cSetupDef.m_cCorDef,0);
			CSmartLoop* steepBnd = Mini_OperateContours( baseBnd, flatBnd, 2 ) ;// 2 :求减（求差 A-B）
			
			double dOffset[2] = {0.0,0.0};
			dOffset[0] = dOffset[1] =  2.0f * dArcTol ;
			FlatAreaBnd = Mini_OffsetContourList(flatBnd, NCDEF_OFFSET_INNER, dOffset, m_cSetupDef.m_cCorDef);
			Mini_DeleteContours( flatBnd );
			dOffset[0] = dOffset[1] =  max( dOverLapDist + 4.0f * dArcTol, 0.02) ;		
			CSmartLoop* tempLoop = steepBnd;
			steepBnd = Mini_OffsetContourList( steepBnd, NCDEF_OFFSET_OUTER, dOffset, m_cSetupDef.m_cCorDef ) ;
			Mini_DeleteContours( tempLoop );
			SteepAreaBnd = Mini_OperateContours( baseBnd, steepBnd, 0 ) ;// 1 :求交
			Mini_DeleteContours( baseBnd );
			Mini_DeleteContours( steepBnd );
			SteepAreaBnd = tmpLp.ResortContour( SteepAreaBnd, NCDEF_SORTTYPE_MINDIST, FALSE ) ;

			//qqs,liuyan,xiejunxian 2013.03.19
			//防止陡峭壁附近路径翘起
			dSideTol = 2.0f * dArcTol ;
			if( bFollow )
			{
				dSideTol = min( 0.03, 0.8 * dStep ) ;
				Mini_DeleteContours(FlatAreaBnd);
				FlatAreaBnd = FlatBndPath.ConvertToContours(FALSE, m_cSetupDef.m_cCorDef, dSideTol );
			}

        }
		
		/*区域按照最短距离排序*/
        FlatAreaBnd = tmpLp.ResortContour( FlatAreaBnd, NCDEF_SORTTYPE_MINDIST, FALSE ) ;
		
    }
    // STEP 5 : 还原原始刀具
    if( pNoseTool )
    {
        DriveMdl.UpdateCurrTool( pOrgDriveTool ) ;
        delete pNoseTool ;
    }
    return bRet ;
}
BOOL CSmartAutoFinishGen::CreateXParallelAreas( CSmtCheckMdl& DriveMdl,
                                                CSmartLoop* AllCont ,
                                                DOUBLE SlopeAngle,     // 分区角度, 5 ~ 85 度
												DOUBLE dOverLapDist,	 // 重叠区域宽度
                                                DOUBLE LineAngle  ,    // 行切角度
                                                CSmartLoop* AreaBnd[2], //  [0], 所有边界||平坦边界 , [1], 陡峭边界
                                                DOUBLE  UsedBndTol   ,
                                                JDNC_PRGDEF&  PrgDef )
{
    DOUBLE dSideTol = 0.0 ;
    if( UsedBndTol >= 0.0 )
    {
        dSideTol = UsedBndTol;
        if( dSideTol >= 1.0e-3  )
        {/* 校正轮廓*/
            dSideTol = max( dSideTol, m_cSetupDef.m_cTolDef.m_dArcTol * 2.0 ) ; 
        }
    }
    DOUBLE  dArcTol = min( 0.005, m_cSetupDef.m_cTolDef.m_dArcTol );   
    // STEP 1 : 修正平底刀具及尺寸,提高边界质量和稳定性
    CSmartTool*  pOrgDriveTool = NULL, *pNoseTool = NULL ; 
    pNoseTool = MathCAM_FilletTCorner( SlopeAngle,*(DriveMdl.m_pTool)) ;
    if( pNoseTool )
    {/*平底刀或小圆角牛鼻刀需要修正圆角半径 */
        pOrgDriveTool = DriveMdl.m_pTool ;
        DriveMdl.UpdateCurrTool( pNoseTool ) ;
    }
    // STEP 3 : 构建网格模型, 提取可加工区域的边界线
    DOUBLE dTotalMove = PrgDef.m_dTotalMove ;
    CSmtMeshMdl   MeshMdl;
    MeshMdl.m_nMeshMode = SMARTNC_MESHMDL_MIXTURE  ;
 	/*删除边界路径点*/
    DOUBLE dOverlapDist = dOverLapDist ;
    m_cPrgDef.m_dTotalMove = 0.3 * dTotalMove ;
    int bRet = BuildMeshMdl( MeshMdl, DriveMdl, AllCont,TRUE ) ;
    // STEP 4 : 设置平行截线的兜边量
    if( bRet )
    {
        CSmtCPathLib  BndPath ;
        PrgDef.m_dTotalMove = 0.3 * dTotalMove ;
        MeshMdl.GenMeshSelfBound( BndPath, PrgDef ) ;
        PrgDef.m_dTotalMove = 0.4 * dTotalMove ;
	    MeshMdl.SetParallelAreaAngle( (TFLOAT) LineAngle );
	    double		dOffDist=max( dSideTol, 2.0f * dArcTol );
        double   dOffset[2] = {0.0, 0.0};
        CSmtCPathLib tmpFlatBnd ;
        MeshMdl.ExtractFlatAreaBound( (TFLOAT)SlopeAngle, tmpFlatBnd , PrgDef) ;
	    CSmartLoop* pBaseCont = BndPath.ConvertToContours(FALSE, m_cSetupDef.m_cCorDef, fabs( dOffDist) );
	    CSmartLoop*  pFlatCont = tmpFlatBnd.ConvertToContours(FALSE, m_cSetupDef.m_cCorDef, 0.0 );
	    CSmartLoop* pCutCont   = Mini_OperateContours( pBaseCont, pFlatCont, 2 ) ;// 2 :求减（求差 A-B）
        if( dSideTol < 3.0f * dArcTol )
        {
            dOffset[0] = dOffset[1] =  max( dSideTol, 2.0f * dArcTol ) ;
            AreaBnd[0] = Mini_OffsetContourList( pFlatCont, NCDEF_OFFSET_INNER, dOffset, m_cSetupDef.m_cCorDef ) ;
        }
        else
        {
            AreaBnd[0] =  Mini_OperateContours( pBaseCont, pFlatCont, 0 ) ;// 0 :求交（求差 A-B）
        }
        Mini_DeleteContours( pFlatCont ) ;
        dOffset[0] = dOffset[1] =  max( dOverlapDist + 4.0f * dArcTol, 0.02) ;
	    AreaBnd[1] = Mini_OffsetContourList( pCutCont, NCDEF_OFFSET_OUTER, dOffset, m_cSetupDef.m_cCorDef ) ;
	    Mini_DeleteContours( pCutCont);
        pCutCont = AreaBnd[1] ;
        if( dOffDist < 0.0 ) 
        {
	        Mini_DeleteContours( pBaseCont);
	        pBaseCont= BndPath.ConvertToContours(FALSE, m_cSetupDef.m_cCorDef, dOffDist );
        }
	    AreaBnd[1]= Mini_OperateContours( pBaseCont, pCutCont, 0 ) ;// 1 :求交
	    Mini_DeleteContours( pBaseCont);
	    Mini_DeleteContours( pCutCont);
        /*区域按照最短距离排序*/
        CSmartLoop tmpLp ;
        AreaBnd[0] = tmpLp.ResortContour( AreaBnd[0], NCDEF_SORTTYPE_MINDIST, FALSE ) ;
        AreaBnd[1] = tmpLp.ResortContour( AreaBnd[1], NCDEF_SORTTYPE_MINDIST, FALSE ) ;
    }
    // STEP 5 : 还原原始刀具
    if( pNoseTool )
    {
        DriveMdl.UpdateCurrTool( pOrgDriveTool ) ;
        delete pNoseTool ;
    }
    return bRet ;
}

BOOL CSmartAutoFinishGen::CreateMixtureAreas( CSmtCheckMdl& DriveMdl , // 加工模型
                                              CSmartLoop*   AllCont  , // 区域范围
                                              DOUBLE SlopeAng        , // 分区角度, 5 ~ 85 度
											  DOUBLE dOverLapDist    , // 重叠区域宽度
											  BOOL	bFollow			 , // 平坦部分是否环切
											  double	dStep		 , // 路径间距
                                              CSmartLoop* AreaBnd[2] , // [0], 平坦边界 , [1], 陡峭边界
                                              int  AreaType          , // 区域类型
                                              JDNC_PRGDEF&  PrgDef )
{
    DOUBLE  SlopeAngle = SlopeAng ;
    DOUBLE  dArcTol = min( 0.005, m_cSetupDef.m_cTolDef.m_dArcTol );   
    // STEP 1 : 修正平底刀具及尺寸,提高边界质量和稳定性
    CSmartTool*  pOrgDriveTool = NULL, *pNoseTool = NULL ; 
    CSmartTool*	 pOrgCheckTool = NULL , *pLargeTool= NULL;

    pNoseTool = MathCAM_FilletTCorner( SlopeAngle, *(DriveMdl.m_pTool)) ;
    if( pNoseTool )
    {/*平底刀或小圆角牛鼻刀需要修正圆角半径 */
        pOrgDriveTool = DriveMdl.m_pTool ;
        DriveMdl.UpdateCurrTool( pNoseTool ) ;
    }
    // STEP 2 : 修正保护面刀具及尺寸,提高垂直边界质量和稳定性
    if( DriveMdl.m_pCheckMdl )
	{	//扩大保护面的刀具
		pOrgCheckTool=DriveMdl.m_pCheckMdl->m_pTool;
		pLargeTool=pOrgCheckTool->OffsetBottom( max(0.01, 4.0*dArcTol) );
		DriveMdl.UpdateCheckTool(pLargeTool);
	}
    // STEP 3 : 构建网格模型, 提取可加工区域的边界线
    DOUBLE dTotalMove = PrgDef.m_dTotalMove ;
    CSmtMeshMdl   MeshMdl;
    MeshMdl.m_nMeshMode = SMARTNC_MESHMDL_MIXTURE  ;
	/*删除边界路径点*/
    m_cPrgDef.m_dTotalMove = 0.3 * dTotalMove ;
    int bRet = BuildMeshMdl( MeshMdl , DriveMdl, AllCont, FALSE ) ;
    DOUBLE dOverlapDist = dOverLapDist ;
	if( !bRet ) goto EndLine ;
    // STEP 4 : 设置兜边量
    if( bRet && AreaType == NCDEF_FSLOPE_SLOPE )
    {/*提取陡峭区域*/
        CSmtCPathLib tmpBndPath, tmpSteepBnd ;
        PrgDef.m_dTotalMove = 0.3 * dTotalMove ;
        bRet = MeshMdl.GenMeshSelfBound( tmpBndPath, PrgDef ) ;
        if( bRet )
        {
	        CSmartLoop* baseCont = tmpBndPath.ConvertToContours( FALSE, m_cSetupDef.m_cCorDef, - 3.0 * dArcTol );
            DOUBLE dExtDist = max( dOverlapDist + 4.0f * dArcTol, 0.02) ;
            PrgDef.m_dTotalMove = 0.4 * dTotalMove ;
            bRet = MeshMdl.ExtractSteepAreaBound( (TFLOAT)SlopeAngle, tmpSteepBnd, PrgDef ) ;
	        CSmartLoop* steepCont = tmpSteepBnd.ConvertToContours(FALSE, m_cSetupDef.m_cCorDef, -dExtDist );
            AreaBnd[0] = NULL ;
	        AreaBnd[1] = Mini_OperateContours( baseCont, steepCont, 0 ) ;// 1 :求交
            Mini_DeleteContours( baseCont ) ;
            Mini_DeleteContours( steepCont ) ;
        }
    }
    else if( bRet && AreaType == NCDEF_FSLOPE_FLAT )
    {/*提取平坦区域*/
        CSmtCPathLib tmpFlatBnd ;
        PrgDef.m_dTotalMove = 0.7 * dTotalMove ;
	    TFLOAT fZMax  = (TFLOAT) GetTopHeight()  ;
	    TFLOAT fZMin  = (TFLOAT) GetBottomHeight() ;
	    MeshMdl.LabelPointFlagByZValue( fZMin, fZMax ) ;
        bRet = MeshMdl.ExtractFlatAreaBound( (TFLOAT)SlopeAngle, tmpFlatBnd, PrgDef ) ;
		double dSideTol = 2.0f * dArcTol ;
		if( bFollow ) // 防止陡峭壁附近翘起
			dSideTol = min( 0.03, 0.5 * dStep ) ;
	    AreaBnd[0] = tmpFlatBnd.ConvertToContours(FALSE, m_cSetupDef.m_cCorDef, dSideTol );
        AreaBnd[1] = NULL ;
    }
    else
    {
        CSmtCPathLib tmpBndPath, tmpFlatBnd ;
        PrgDef.m_dTotalMove = 0.3 * dTotalMove ;
        bRet = MeshMdl.GenMeshSelfBound( tmpBndPath, PrgDef ) ;
        if( bRet == TRUE )
        {
            /* 根据加工的深度范围限定有效点, 高度范围内部的点的属性不变,范围外部点的属性为 0　*/
	        TFLOAT fZMax  = (TFLOAT) GetTopHeight()  ;
	        TFLOAT fZMin  = (TFLOAT) GetBottomHeight() ;
	        MeshMdl.LabelPointFlagByZValue( fZMin, fZMax ) ;
            double		dOffDist= -3.0 * dArcTol;
            double dOffset[2] = {0.0,0.0};
            PrgDef.m_dTotalMove = 0.4 * dTotalMove ;
            MeshMdl.ExtractFlatAreaBound( (TFLOAT)SlopeAngle, tmpFlatBnd, PrgDef ) ;
	        CSmartLoop* baseCont = tmpBndPath.ConvertToContours(FALSE, m_cSetupDef.m_cCorDef, fabs( dOffDist) );
	        CSmartLoop* flatCont = tmpFlatBnd.ConvertToContours(FALSE, m_cSetupDef.m_cCorDef, 0.0 );
	        CSmartLoop* steepCont= Mini_OperateContours( baseCont, flatCont, 2 ) ;// 2 :求减（求差 A-B）
			dOffset[0] = dOffset[1] =  2.0f * dArcTol ;
            AreaBnd[0] = Mini_OffsetContourList( flatCont, NCDEF_OFFSET_INNER, dOffset, m_cSetupDef.m_cCorDef ) ;
            Mini_DeleteContours( flatCont ) ;
            dOffset[0] = dOffset[1] =  max( dOverlapDist + 4.0f * dArcTol, 0.02) ;
            CSmartLoop *bakCont =  steepCont ;
	        steepCont = Mini_OffsetContourList( steepCont, NCDEF_OFFSET_OUTER, dOffset, m_cSetupDef.m_cCorDef ) ;
	        Mini_DeleteContours( bakCont );
            if( dOffDist < 0.0 ) 
            {
	            Mini_DeleteContours( baseCont);
	            baseCont= tmpBndPath.ConvertToContours(FALSE, m_cSetupDef.m_cCorDef, dOffDist );
            }
	        AreaBnd[1]= Mini_OperateContours( baseCont, steepCont, 0 ) ;// 1 :求交
	        Mini_DeleteContours( baseCont);
	        Mini_DeleteContours( steepCont);
            /*区域按照最短距离排序*/
            CSmartLoop tmpLp ;
           
		   //qqs,liuyan,xiejunxian 2013.03.19
			//防止陡峭壁附近路径翘起				
			double dSideTol = 2.0f * dArcTol ;
			if( bFollow )
			{
				Mini_DeleteContours(AreaBnd[0]);
				dSideTol = min( 0.03, 0.5 * dStep ) ;
				AreaBnd[0] = tmpFlatBnd.ConvertToContours(FALSE, m_cSetupDef.m_cCorDef, dSideTol );
			}
			AreaBnd[0] = tmpLp.ResortContour( AreaBnd[0], NCDEF_SORTTYPE_MINDIST, FALSE ) ;
			AreaBnd[1] = tmpLp.ResortContour( AreaBnd[1], NCDEF_SORTTYPE_MINDIST, FALSE ) ;
        }
    }
EndLine:
    // STEP 5 : 还原原始刀具
    if( pNoseTool )
    {
        DriveMdl.UpdateCurrTool( pOrgDriveTool ) ;
        delete pNoseTool ;
    }
	if(pLargeTool)
	{
		DriveMdl.UpdateCheckTool(pOrgCheckTool);
		delete pLargeTool;
	}
    return bRet ;
}
BOOL CSmartAutoFinishGen::CreateNcAreas ( CSmtCheckMdl	& DriveMdl,
										  CSmartLoop	*& AllCont )
{
	// 构建网格模型, 提取可加工区域的边界线
    CSmtMeshMdl smtMeshMdl ;
    /*删除边界路径点*/
    int bRet = BuildMeshMdl ( smtMeshMdl, DriveMdl, NULL,TRUE ) ;
    if( bRet )
    {
		CSmtCPathLib bndPath ;
        smtMeshMdl.GenMeshSelfBound ( bndPath, m_cPrgDef ) ;
        DOUBLE dOffset = 0 ;
        AllCont = bndPath.ConvertToContours ( FALSE, m_cSetupDef.m_cCorDef, dOffset ) ;
    }
    
	return bRet ;
}
BOOL CSmartAutoFinishGen::BuildSFinishGen( CPathGroup&   /*NewPath */,
                                           CSmartGraphic& Graph    , 
                                           CSmtCheckMdl&  DriveMdl , 
                                           CSmtCheckMdl&  CheckMdl ,
                                           CSmartLoop*&   BoundCont, 
										   double dAngle )
{
	// STEP 1 : 转换刀具
	if( ! UpdateTool3D( Graph ) )
    {
		m_nErrorType = JDERROR_GENPATH_STOCK ;
		return FALSE ;
    }
    m_pSmtGraph = & Graph  ;
	BoundCont = Graph.GetAllContour(m_cSetupDef);
	if( BoundCont ) 
    {
        OffsetBorder( BoundCont ) ;
        CSmartLoop tmpLp ;
        BoundCont = tmpLp.ResortContour( BoundCont, NCDEF_SORTTYPE_MINDIST, FALSE ) ;
		// 边界偏移后为空,加工域不存在,返回FALSE 
		if( !BoundCont ) 	
		{
			m_nErrorType = JDERROR_GENPATH_NOAREA ;
			return FALSE ;
		}
    }
	SetCheckFltBoxByContours( BoundCont, DriveMdl, CheckMdl ) ; 
    if( ! BuildDriveAndCheckMdl( Graph, DriveMdl, CheckMdl, dAngle ) )
    {
        Mini_DeleteContours( BoundCont ) ;
        m_nErrorType = JDERROR_GENPATH_NOSURFACE  ;
        return FALSE ;
    }
	if ( m_cSetupDef.m_cModelTol.m_nMdlFlag & NCDEF_SETUP_FILLET )
	{/*模型倒角*/
		NcBnd_ModifyCheckMdlByFillet ( DriveMdl, m_cSetupDef, m_cPrgDef,
										m_cSetupDef.m_cTolDef.m_dMaxRad ) ;
	}
    // 如果边界存在,旋转边界
	if( BoundCont && fabs( dAngle ) > 1.0e-4 )
	{
		for( CSmartLoop *pLoop = BoundCont ; pLoop ; pLoop = pLoop->next )
		{
			pLoop->RotateContour ( dAngle ) ;
		}
	}
    if( BoundCont == NULL )
    {
		FPNT3D fBox3D[2] ;
		DriveMdl.CalcBox( fBox3D[0], fBox3D[1]) ;
		PNT2D dMinPt, dMaxPt ;
		nc_FloatToDouble( dMinPt, fBox3D[0], 2 ) ;
		nc_FloatToDouble( dMaxPt, fBox3D[1], 2 ) ;
		BoundCont = new CSmartLoop() ;
		BoundCont->CreateLoop(dMinPt, dMaxPt ) ;
    }
	if (IsNeedMultiCopy(DriveMdl))
	{
		DriveMdl.CreateMultiCopy(m_nCalcThreadNum - 1) ;
	}	
    return TRUE ;
}

// 生成初始的截线路径
void CSmartAutoFinishGen::GenOrginalPath(CSmartLoop *AreaCont, CSmtCPathLib &OrgPath)
{
	CSmartLoop* pLoop;
	int nGroupNo = 0 , nLineNo = 0 ;
	PNT2D dBox2D[2] ;
	DOUBLE y = 0.0 ;
	DOUBLE yStep = m_cFeedDef.m_cStepDef.m_dOverStep  ;
	PNT4D dStart, dEnd  ;
	dStart[2] = dEnd[2] = 0.0 ;
	dStart[3] = dEnd[3] = 0.0 ;
	PNT2D dPtArr[1000];
	for( pLoop = AreaCont ; pLoop ; pLoop = pLoop->next, nGroupNo ++  )
	{
		nc_VectorCopy ( dBox2D[0], pLoop->m_dBox[0], 2 ) ;
		nc_VectorCopy ( dBox2D[1], pLoop->m_dBox[1], 2 ) ;
		CSmtCPathLib curTLib ;
		for( y = dBox2D[0][1], nLineNo = 0 ; y < dBox2D[1][1]; y += yStep, nLineNo ++ ) 
		{
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
				pNewPath->m_nLayerNo = nGroupNo;
				curTLib.AddToTail( pNewPath )  ;
			}
		}
		OrgPath.AppendCPathLib ( curTLib ) ;
	}
}

// 平行截线线程函数
BOOL CSmartAutoFinishGen::CreateParallelPathSubProc(int nAtCore, int nCoreNum, CSmtCPathLib &TmpPath, LPVOID* AllPath, 
													 CSmtCheckMdl &DriveMdl, BOOL RandPointMode, JDNC_PRGDEF &PrgDef)
{
	INT_PTR nNumPath = TmpPath.m_cAllPath.GetCount();
	if (nNumPath < 1)
	{
		return FALSE;
	}
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
	PrgDef.m_dIncStep = PrgDef.m_dTotalMove / nNumPath ;
	JDNC_PRGDEF tmpPrg = m_cPrgDef ;
	tmpPrg.m_pPosFunc = NULL ; 
	tmpPrg.m_pPrgFunc = NULL ; 	
	int nIndex = 0;
	TFLOAT fBtmHeight = (TFLOAT)GetBottomHeight(), fTopHeight = (TFLOAT)GetTopHeight();
	POSITION pos = TmpPath.m_cAllPath.GetHeadPosition();
	while( pos )
	{
		if( PrgDef.m_pBrkFunc && PrgDef.m_pBrkFunc() )
		{
			break ;
		}
		MathCAM_MovePrgStep(PrgDef);
		CSmtCutPath* pBasePath = TmpPath.m_cAllPath.GetNext(pos) ;
		if (nIndex++ % nCoreNum != nAtCore || pBasePath == NULL)
		{
			continue;
		}
        if( RandPointMode )
        {
            pBasePath->InsertRandCPoint( m_cSetupDef.m_cTolDef.m_dMaxStep, pBasePath->m_bMoveFlag ) ;
        }
        else
        {
            pBasePath->InsertCPoint( m_cSetupDef.m_cTolDef.m_dMaxStep ) ;
        }
        pBasePath->m_bMoveFlag = 0 ;
		pBasePath->VerifyLinePath( DriveMdl , m_cSetupDef.m_cTolDef, tmpPrg) ;
		pBasePath->DefineEdgePoint( DriveMdl, TRUE );
		pBasePath->LabelNullPointByZValue(fBtmHeight-0.002f, fTopHeight);
        CSmtCutPath* pNewPath=pBasePath->BreakAtNullPoint(FALSE, m_cFeedDef.m_cConnectDef.m_dDelShort);
		delete pBasePath;
		AllPath[nIndex-1] = pNewPath;
	}
	if( PrgDef.m_pBrkFunc && PrgDef.m_pBrkFunc() )
	{// 中断时删除数据		
		Nc5D_DestroyMultiCalcData(AllPath, nIndex, &TmpPath, pos, nCoreNum, nAtCore);		
		return FALSE;
	}
	return TRUE;
}

// 偏移一组轮廓
CSmartLoop*   AutoFinish_OffsetContourList( CSmartLoop* AllCont,  /*轮廓组  */
											DOUBLE OffDist[2]  ,  /*偏移距离*/
											JDNC_COR&  CorDef)    /*尖角形式*/
{
	int nID = 1 ;
	CSmartLoop* pLpHead = NULL , *pContour, *pLpNew ; 
	for( pContour = AllCont ; pContour ; pContour = pContour->next )
	{
		pLpNew = pContour->OffsetContourEx( OffDist[0], OffDist[1], CorDef ) ;
		pLpHead = Mini_AddContours( pLpHead, pLpNew ) ;
		for( ; pLpNew ; pLpNew = pLpNew->next ) pLpNew->m_nGroupId = nID ;
		nID ++ ;
	}
	if( ! pLpHead ) return NULL ;
	if( pLpHead) 
	{
		CSmartLoop LoopTmp ;
		pLpHead = LoopTmp.WeldContours( pLpHead ) ;
	}
	return pLpHead ; 
}

// 生成曲面边界轮廓
CSmartLoop* CSmartAutoFinishGen::GenSurfBorder(CSmartGraphic *pSmtGraph, CSmtCheckMdl *pDriveMdl, double RotAngle )
{
	// 构建网格模型, 提取可加工区域的边界线
	CSmartPathGen& PathGen = *(this) ;
	CSmtMeshMdl MeshMdl;
	BOOL bTrimFirst = TRUE ; /*是否首先启动高度过滤*/ 
	double dTotalMove = m_cPrgDef.m_dTotalMove;
	m_cPrgDef.m_dTotalMove = 0.5 * dTotalMove;
	int bRet = BuildMeshMdl(MeshMdl , *pDriveMdl, NULL, bTrimFirst ) ;
	if( bRet == FALSE )
	{
		return FALSE ;
	}

	CSmtCPathLib tmpCLib ;
	m_cPrgDef.m_dTotalMove = 0.5 * dTotalMove;
	MeshMdl.GenMeshSelfBound( tmpCLib, m_cPrgDef ) ;
	// yul 2013/04/19 该处的偏移值会将岛向内偏移，导致在连刀的时候抬起，原值为-0.01，现在
	// 进行修改为m_cSetupDef.m_cTolDef.m_dArcTol，岛向外偏移，避开直壁，外轮廓向外偏移
	CSmartLoop* BndCont = tmpCLib.ConvertToContours(FALSE, m_cSetupDef.m_cCorDef, 0. );
	// 对得到的轮廓进行偏移，如上面注释所示
	double dist[2] = { m_cSetupDef.m_cTolDef.m_dArcTol, m_cSetupDef.m_cTolDef.m_dArcTol } ;
	CSmartLoop *TmpCont = AutoFinish_OffsetContourList( BndCont, dist, m_cSetupDef.m_cCorDef ) ;
	if( TmpCont )
	{
		Mini_DeleteContours( BndCont ) ;
		BndCont = TmpCont ;
	}
	CSmartLoop*	AllCont = pSmtGraph->GetAllContour(m_cSetupDef);
	if( AllCont ) 
	{
		if( fabs( RotAngle) > 1.0e-4 )
		{
			for( CSmartLoop *pLoop = AllCont ; pLoop ; pLoop = pLoop->next )
			{
				pLoop->RotateContour( RotAngle ) ;
			}
		}
        PathGen.OffsetBorder( AllCont );
	
		CSmartLoop* pContHead = Mini_OperateContours( AllCont, BndCont, 0 ) ;// 0 :求交
		Mini_DeleteContours( BndCont ) ;
		BndCont = pContHead ;
	}
	Mini_DeleteContours( AllCont ) ;
	return BndCont ;
}

// 生成沿边界连刀的路径
int CSmartAutoFinishGen::CreatePathhByBound(CSmartGraphic &Graph, CSmtCheckMdl &DriveMdl, 
											JDNC_LINEAR& FLinear, JDNC_SETUP& Setup, 
											DOUBLE RotAngle, CSmtCPathLib& AllPath)
{
	// STEP 1 : 判断初始数据条件
	CSmartPathGen& PathGen = *(this) ;
	DOUBLE dTotalMove = m_cPrgDef.m_dTotalMove;
	m_cPrgDef.m_dTotalMove = 0.5 * dTotalMove;
	CSmartLoop *AllLoop =  GenSurfBorder( &Graph, &DriveMdl, RotAngle) ;
	
	// STEP 2 : 生成平面行切路径
	CPathCombine  tmpComb( NC_WPROCESS_ROUGH ) ;
	JDNC_LINEAR   tmpLinear = FLinear ;
	tmpLinear.m_dLineAngle = 0.0 ;
	tmpLinear.m_bLineFlag |= NCDEF_LINEAR_AUTODIR ;
	for( CSmartLoop* pLp = AllLoop ; pLp; pLp = pLp->next )
	{
		PathGen.CreateLinearPath( tmpComb, *pLp, tmpLinear ) ;
	}
	// STEP 3 : 投影路径
	AllPath.CreateCutPath( tmpComb, Setup.m_cTolDef) ;
	//ProcessNewFunc( m_cPrgDef, IDS_PATHGEN_STEP_ORGPATH ) ;
	DriveMdl.m_bCheckMode = SMART_MODEL_PROJECT ;
	m_cPrgDef.m_dTotalMove = 0.5 * dTotalMove;
	AllPath.VerifyCutPath( DriveMdl, Setup.m_cTolDef, PathGen.m_cPrgDef ) ;
	Mini_DeleteContours(AllLoop);
	if( SurfNC_IsAbort() ) 
	{
		AllPath.ClearAllPath();
		return FALSE ; 
	}		
	AllPath.DelPointOnLine() ; // 删除同一直线上的点
	return TRUE;
}

void CSmartAutoFinishGen::KeepSharpEdges(CSmtCPathLib & AllPath,CSmtCheckMdl & DriveMdl )
{
	if(m_pSmtGraph==NULL)
	{
		return;
	}
	CPtrList cSharpEdges;
	int nMoveType = m_cMethodDef.m_cSFinishCut.m_nMoveType;
	if(nMoveType == surfncMoveHeight)
	{
		m_pSmtGraph->GetSharpEdges(m_cSetupDef, cSharpEdges);
		CFHeightKeepSharpEdge cFHKpShpEdg;
		cFHKpShpEdg.m_bKeepHoz = 1;
		cFHKpShpEdg.m_dHozEdgeZShift = m_cMethodDef.m_cSFinishCut.m_cHeightCut.m_dHozEdgeZShift; 
		cFHKpShpEdg.KeepSharpEdges(AllPath, cSharpEdges, m_cSetupDef, m_cStockDef, DriveMdl);
		cFHKpShpEdg.m_bKeepHoz = 0;
		cFHKpShpEdg.KeepSharpEdges(AllPath, cSharpEdges, m_cSetupDef, m_cStockDef, DriveMdl);
	}
	else if(nMoveType == surfncMoveLinear)
	{
#if 0
		m_pSmtGraph->GetSharpEdges(m_cSetupDef, cSharpEdges);
		CFLinearKeepSharpEdge cFLnKpShpEdg;
		cFLnKpShpEdg.KeepSharpEdges(AllPath, cSharpEdges, m_cSetupDef, m_cStockDef, DriveMdl);
#endif
	}

	while(cSharpEdges.GetCount())
	{
		CStrpt * pEdge = (CStrpt*)cSharpEdges.RemoveHead();
		if(pEdge)
		{
			delete pEdge;
			pEdge = NULL;
		}
	}
}


//样条拟合路径多线程运算线程函数
static UINT MathCAM_PathSplFitProc(LPVOID lpParam)
{
	if (lpParam == NULL)
    {
		return 0;
    }
	PSPLFIT_DATA& cData = *(PSPLFIT_DATA *)lpParam;	
    if( cData.pPathSplFitGen == NULL || cData.AllPath == NULL ) 
    {
        return 0 ;
    }
	return cData.pPathSplFitGen->SplFitSubProc(cData.nAtCore, cData.nCoreNum,  
		cData.AllPath, cData.dFitTol, cData.PrgDef);
}
BOOL CSmartAutoFinishGen::PathSplFit(CSmtCPathLib & AllPath)
{
	double dFitTol = m_cSetupDef.m_cTolDef.m_dArcTol ;
	int nPathNum = (int)AllPath.m_cAllPath.GetCount(); 
	if(fabs(dFitTol)<0.001 || nPathNum<1)
	{
		return TRUE;
	}

#if 0
	clock_t start ;
	start = clock();
#endif

	double dLen = 0;
	POSITION pPos = AllPath.m_cAllPath.GetHeadPosition();
	while(pPos)
	{
		CSmtCutPath * pCurrPath = AllPath.m_cAllPath.GetNext(pPos);
		if(pCurrPath && pCurrPath->GetCutMode()==MINI_MILL_PATH )
		{
			dLen += pCurrPath->GetLength();
		}
	}

	SurfNC_SetNewStepEx(IDS_SMOOTH_PATH);
	m_cPrgDef.m_dLimitAt = 1.0 ;
	m_cPrgDef.m_dStepAt  = 0.0 ;
	m_cPrgDef.m_dTotalMove = 100;
	m_cPrgDef.m_dIncStep = m_cPrgDef.m_dTotalMove / double(nPathNum); 
	if(dLen>5000 && nPathNum>NC_CFG_CPU_MAXNUM && MathCAM_IsSupportMultiThread())
	{
		PSPLFIT_DATA ThreadData[NC_CFG_CPU_MAXNUM];
		LPVOID lpParam[NC_CFG_CPU_MAXNUM] = {NULL};
		JDNC_PRGDEF tmpPrg = m_cPrgDef;
		tmpPrg.m_pPrgFunc = NULL ; 
		tmpPrg.m_pPosFunc = NULL ; 
		int nThreadNum = m_nCalcThreadNum;
		m_cPrgDef.m_dIncStep *= nThreadNum; 
		//路径分配到各个线程中
		CSmtCPathLib cAllSubPath[NC_CFG_CPU_MAXNUM];
		int n = nPathNum / nThreadNum ;
		int nAtGrp = 0;
		int i = 0;
		pPos = AllPath.m_cAllPath.GetHeadPosition();
		while(pPos)
		{
			CSmtCutPath * pCurrPath = AllPath.m_cAllPath.GetNext(pPos);
			nAtGrp = i / n;
			if(nAtGrp >= nThreadNum-1)	{nAtGrp = nThreadNum-1;}
			cAllSubPath[nAtGrp].m_cAllPath.AddTail(pCurrPath);
			i++;
		}
		CPathSplFit cAllPathSplFit[NC_CFG_CPU_MAXNUM];
		for (i=0; i<nThreadNum; i++)
		{
			ThreadData[i].nAtCore = i;
			ThreadData[i].nCoreNum = nThreadNum;
			ThreadData[i].dFitTol = dFitTol;
			ThreadData[i].AllPath = &cAllSubPath[i];
			ThreadData[i].PrgDef = i==0 ? m_cPrgDef : tmpPrg;
			ThreadData[i].pPathSplFitGen = &cAllPathSplFit[i];			
			lpParam[i] = &ThreadData[i];
		}
		// 调用多线程函数
		MathCAM_ThreadMainFunc(MathCAM_PathSplFitProc, lpParam, nThreadNum);
		for( i=0; i<NC_CFG_CPU_MAXNUM; i++)
		{
			cAllSubPath[i].m_cAllPath.RemoveAll();
		}
		if( m_cPrgDef.m_pBrkFunc && m_cPrgDef.m_pBrkFunc() )
		{
			return FALSE ;
		}
	}
	else
	{//单线程
		CPathSplFit cPathSplFit;
		if(!cPathSplFit.SplFit(AllPath, dFitTol, m_cPrgDef))	
		{ 
			return FALSE ;
		}
	}

#if 0
	clock_t finish;
	finish = clock();
	double duration = (double)(finish - start)/ CLOCKS_PER_SEC;
	CString str;
	str.Format("path smooth time: %6.5f s\n", duration);
	AfxMessageBox(str);
#endif
	return TRUE;
}

static UINT MathCAM_PathSplFitProcEx(LPVOID lpParam)
{
	if (lpParam == NULL)
    {
		return 0;
    }
	PSPLFIT_DATA& cData = *(PSPLFIT_DATA *)lpParam;	
	if( cData.pPathSplFitGen == NULL || cData.AllPath == NULL || cData.AllComb == NULL) 
    {
        return 0 ;
    }
	return cData.pPathSplFitGen->SplFitSubProcEx(cData.nAtCore, cData.nCoreNum,  
		cData.AllPath, cData.dFitTol, cData.AllComb, cData.PrgDef);
}



BOOL CSmartAutoFinishGen::PathSplFit(CSmtCPathLib & AllPath, CPathCombine & PathComb)
{
	double dFitTol = m_cSetupDef.m_cTolDef.m_dArcTol ;
	int nPathNum = (int)AllPath.m_cAllPath.GetCount(); 
	if(fabs(dFitTol)<0.001 || nPathNum<1)
	{
		return TRUE;
	}
	
	double dLen = 0;
	POSITION pPos = AllPath.m_cAllPath.GetHeadPosition();
	while(pPos)
	{
		CSmtCutPath * pCurrPath = AllPath.m_cAllPath.GetNext(pPos);
		if(pCurrPath && pCurrPath->GetCutMode()==MINI_MILL_PATH )
		{
			dLen += pCurrPath->GetLength();
		}
	}

	SurfNC_SetNewStepEx(IDS_SMOOTH_PATH);
	m_cPrgDef.m_dLimitAt = 1.0 ;
	m_cPrgDef.m_dStepAt  = 0.0 ;
	m_cPrgDef.m_dTotalMove = 100;
	m_cPrgDef.m_dIncStep = m_cPrgDef.m_dTotalMove / double(nPathNum); 
	if(dLen>5000 && nPathNum>NC_CFG_CPU_MAXNUM && MathCAM_IsSupportMultiThread())
	{
		PSPLFIT_DATA ThreadData[NC_CFG_CPU_MAXNUM];
		LPVOID lpParam[NC_CFG_CPU_MAXNUM] = {NULL};
		JDNC_PRGDEF tmpPrg = m_cPrgDef;
		tmpPrg.m_pPrgFunc = NULL ; 
		tmpPrg.m_pPosFunc = NULL ; 
		int nThreadNum = m_nCalcThreadNum;
		m_cPrgDef.m_dIncStep *= nThreadNum; 
		//路径分配到各个线程中
		CSmtCPathLib cAllSubPath[NC_CFG_CPU_MAXNUM];
		int n = nPathNum / nThreadNum ;
		int nAtGrp = 0;
		int i = 0;
		pPos = AllPath.m_cAllPath.GetHeadPosition();
		while(pPos)
		{
			CSmtCutPath * pCurrPath = AllPath.m_cAllPath.GetNext(pPos);
			nAtGrp = i / n;
			if(nAtGrp >= nThreadNum-1)	{nAtGrp = nThreadNum-1;}
			cAllSubPath[nAtGrp].m_cAllPath.AddTail(pCurrPath);
			i++;
		}
		CPathSplFit cAllPathSplFit[NC_CFG_CPU_MAXNUM];
		CTypedPtrArray<CPtrArray,CPathCombine*> cAllSubComb;
		for (i=0; i<nThreadNum; i++)
		{
			ThreadData[i].nAtCore = i;
			ThreadData[i].nCoreNum = nThreadNum;
			ThreadData[i].dFitTol = dFitTol;
			ThreadData[i].AllPath = &cAllSubPath[i];
			CPathCombine * pNewComb = new CPathCombine( NC_WPROCESS_ROUGH );
			ThreadData[i].AllComb = pNewComb;
			cAllSubComb.Add(pNewComb);
			ThreadData[i].PrgDef = i==0 ? m_cPrgDef : tmpPrg;
			ThreadData[i].pPathSplFitGen = &cAllPathSplFit[i];			
			lpParam[i] = &ThreadData[i];
		}
		// 调用多线程函数
		MathCAM_ThreadMainFunc(MathCAM_PathSplFitProcEx, lpParam, nThreadNum);
		for( i=0; i<NC_CFG_CPU_MAXNUM; i++)
		{
			cAllSubPath[i].m_cAllPath.RemoveAll();
		}
		if( m_cPrgDef.m_pBrkFunc && m_cPrgDef.m_pBrkFunc() )
		{
			for(int k=0; k<cAllSubComb.GetCount(); k++)	{
				CPathCombine * pAtComb = cAllSubComb.GetAt(k);
				if(pAtComb)	{
					delete pAtComb;	pAtComb = NULL;
				}
			}
			return FALSE ;
		}
		for(int k=0; k<cAllSubComb.GetCount(); k++)
		{
			CPathCombine * pAtComb = cAllSubComb.GetAt(k);
			if(pAtComb) 
				PathComb.AppendCombine(pAtComb);
		}
	}
	else
	{//单线程
		CPathSplFit cPathSplFit;
		if(!cPathSplFit.SplFit(AllPath, dFitTol, PathComb, m_cPrgDef))	
		{ 
			return FALSE ;
		}
	}
	return TRUE;
}
#include "stdafx.h"
#include "SmartNC.h"
#include "SmartMath.h"
#include "Surfnc.h"
#include "mathCam.h"
#include "SmartPathEx.H"
#include "SmtPathGen.h"
#include "PencilLink.h"
#include "PencilInfo.h"
#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char BASED_CODE THIS_FILE[] = __FILE__;
#endif

// 计算位置点的进退刀
static int MathCAM_GenLeadPathEx(CSmtCheckMdl&   CheckMdl, JDNC_TOL& Tol, 
								 JDNC_CONNECT3D& ConnectDef, TFLOAT StartAt[3],  
								 TFLOAT TanDir[3], TFLOAT NorDir[3], 
								 CSmtCutPath& CutPath, BOOL bCheck )
{
	// STEP 1 : 建立局部坐标系
	TFLOAT fZDir[3], fYDir[3], fXDir[3] ;
	nc_VectorCopy( fZDir, NorDir , 3 ) ;
	nc_VProduct( fZDir, TanDir, fYDir ) ;
	nc_VProduct( TanDir, fYDir, fZDir) ;
//	if( fabs( fZDir[2] ) < 0.002 ) fZDir[2] = 0. ;
	if( ! nc_Normalize( fZDir, 3 ) )
	{
		fYDir[0] = 0.0f, fYDir[1] = 1.0f, fYDir[2] = 0.0f ;
		nc_VProduct( fYDir, TanDir, fZDir ) ;
		if( ! nc_Normalize( fZDir, 3 ) )
		{
			return 0 ;
		}
	}

//	if( fZDir[2] < 0.0 ) nc_VectorReverse( fZDir, 3 ) ;
	nc_VProduct( fZDir, TanDir, fYDir ) ;
	nc_VectorCopy( fXDir, TanDir, 3 ) ;
	// STEP 2 : 构建平面圆弧点串
	PNT2D dPoint[51] ;
	DOUBLE dCenter[2] = { 0.0 , ConnectDef.m_dLead3DRadius } ;
	DOUBLE dAngle [2] = { MiniPai2 * 0.75, MiniPai2 * 0.75 + ANGLE_TO_RADIAN( ConnectDef.m_dLead3DAngle )} ;
	CSmartArc tmpArc( dCenter, dAngle, ConnectDef.m_dLead3DRadius ) ;
	int nCnt = tmpArc.DiscreteEx( Tol.m_dArcTol, ANGLE_TO_RADIAN( Tol.m_dAngTol), dPoint, 50 );
	dPoint[0][0] = dPoint[0][1] = 0.0;
	if( ConnectDef.m_dLead3DLine > 0.002 )
	{
		for( int k = nCnt; k >= 0 ; k -- )
		{
			dPoint[k+1][0] = dPoint[k][0] + ConnectDef.m_dLead3DLine ;
			dPoint[k+1][1] = dPoint[k][1] ;
		}
		dPoint[0][0] = ConnectDef.m_dLead3DLine ;
		dPoint[0][1] = 0.0 ;
		nCnt ++ ;
	}
	// STEP 3 : 尝试是否存在过切
	JDNC_PRGDEF tmpPrg ;
	SurfNC_InitPathParam( tmpPrg ) ;
	CutPath.SetCutMode( MINI_CONNECT_PATH ) ;
	TFLOAT fTPoint[3], fBase[3] = { 0.0f, 0.0f, 0.0f} ;
	for( int nTry = 0 ; nTry <= 3 ; nTry ++ ) 
	{
		CutPath.ClearAllPoint() ;
		for( int k = 0 ; k <= nCnt ; k ++ ) 
		{
			fTPoint[0] = StartAt[0] + (TFLOAT)(fXDir[0] * dPoint[k][0] + fZDir[0] * dPoint[k][1]) ;
			fTPoint[1] = StartAt[1] + (TFLOAT)(fXDir[1] * dPoint[k][0] + fZDir[1] * dPoint[k][1]) ;
			fTPoint[2] = StartAt[2] + (TFLOAT)(fXDir[2] * dPoint[k][0] + fZDir[2] * dPoint[k][1]) ;
			CutPath.AddPoint( fTPoint ) ;
		}
		if( !bCheck ) 
		{ // 如果非干涉,直接生成切入切出
			break ;
		}
		else
		{
			if( CutPath.VerifyLeadPath( CheckMdl, Tol, tmpPrg, 0.05 ) ==TRUE )
            {
				break ;
			}
		}
		CutPath.ClearAllPoint() ;
		nc_RotatePoint( fBase, fYDir, fXDir, (TFLOAT)ANGLE_TO_RADIAN(3) ) ;
		nc_RotatePoint( fBase, fYDir, fZDir, (TFLOAT)ANGLE_TO_RADIAN(3) ) ;
	}
	if( CutPath.m_pHead == NULL ) return 0 ;
	// STEP 3 : 尝试是否存在过切
	nc_VectorCopy( CutPath.m_pHead->m_fPoint, StartAt, 3 ) ;
	return 1 ;
}
static int MathCAM_DefineNormalAt( CSmtCheckMdl& CheckMdl, JDNC_TOL& Tol ,
								   TFLOAT StartAt[3], TFLOAT Normal[3] )
{// 计算位置点附近的近似方向
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
void CalcSmtCutPathLead( CSmtCheckMdl &CheckMdl, JDNC_TOL &cTol, JDNC_CONNECT3D &cConnect, 
						 CSmtCutPath *pPath, CSmtCutPath *&LeadIn, CSmtCutPath *&LeadOut, BOOL bCheck )
{
	// 首先得到参数
	int i = 0 ;
	FPNT3D pnt[2], tan[2], nor[2] ;
	VEC3D vec[2] ;
	nor[0][0] = nor[0][1] = nor[1][0] = nor[1][1] = 0, nor[0][2] = nor[1][2] = 1. ;
	mathFCpyPnt( pPath->m_pHead->m_fPoint, pnt[0] ) ;
	mathFCpyPnt( pPath->m_pTail->m_fPoint, pnt[1] ) ;

	MathCAM_DefineNormalAt( CheckMdl, cTol, pnt[0], nor[0] ) ;
	MathCAM_DefineNormalAt( CheckMdl, cTol, pnt[1], nor[1] ) ;
	nor[0][2] = nor[1][2] = 0.f ;
	if( !nc_Normalize( nor[0], 3 ) )
	{
		if( !bCheck )
		{
			CSmtCutPointEx *pHead = ( CSmtCutPointEx *)pPath->m_pHead ;
			mathFCpyPnt( pHead->m_fSurfNor, nor[0] ) ;
			if( !nc_Normalize( nor[0], 3 ) )
			{
				nor[0][0] = nor[0][1] = 0.f, nor[0][2] = 1.f ;
			}
		}
		else
		{
			nor[0][0] = nor[0][1] = 0.f, nor[0][2] = 1.f ;
		}
	}
	if( !nc_Normalize( nor[1], 3 ) )
	{
		if( !bCheck )
		{
			CSmtCutPointEx *pTail = ( CSmtCutPointEx *)pPath->m_pTail ;
			mathFCpyPnt( pTail->m_fSurfNor, nor[1] ) ;
			if( !nc_Normalize( nor[1], 3 ) )
			{
				nor[1][0] = nor[1][1] = 0.f, nor[1][2] = 1.f ;
			}
		}
		else
		{
			nor[1][0] = nor[1][1] = 0.f, nor[1][2] = 1.f ;
		}
	}	
	mathFGetUnitVec( pPath->m_pHead->next->m_fPoint, pPath->m_pHead->m_fPoint, vec[0] ) ;
	mathFGetUnitVec( pPath->m_pTail->prev->m_fPoint, pPath->m_pTail->m_fPoint, vec[1] ) ;
	for( i = 0 ; i < 3 ; i++ )
	{
		tan[0][i] = TFLOAT( vec[0][i] ), tan[1][i] = TFLOAT( vec[1][i] ) ;
	}
	// è然后计算切入切出
	CSmtCutPath tmpLead( MINI_CONNECT_PATH ) ;
	for( i = 0 ; i < 2 ; i++ )
	{
		if( !MathCAM_GenLeadPathEx( CheckMdl, cTol, cConnect, pnt[i], 
									tan[i], nor[i], tmpLead, bCheck ) )
		{
			continue ;
		}
		if( i == 0 ) 
		{
			tmpLead.ReverseDirect () ;
			LeadIn = new CSmtCutPath( MINI_CONNECT_PATH ) ;
			LeadIn->m_bFeedType = JDNC_FEEDTYPE_LEAD ;
			LeadIn->AppendCutPoint ( tmpLead.m_pHead ) ;
		}
		else
		{
			LeadOut = new CSmtCutPath( MINI_CONNECT_PATH ) ;
			LeadOut->m_bFeedType = JDNC_FEEDTYPE_LEAD ;
			LeadOut->AppendCutPoint ( tmpLead.m_pHead ) ;
		}
		tmpLead.m_pHead = tmpLead.m_pTail = NULL ;
		tmpLead.ClearAllPoint () ;
	}
}
void CalcCutPathLeadIn( CSmtCheckMdl &CheckMdl, JDNC_TOL &cTol, JDNC_CONNECT3D &cConnect, 
						CSmtCutPath *pPath, CSmtCutPath *&LeadIn, BOOL bCheck, int nType )
{
	// 首先得到参数
	int i = 0 ;
	FPNT3D pnt, tan, nor = { 0, 0, 1 } ;
	VEC3D vec ;
	mathFCpyPnt( pPath->m_pHead->m_fPoint, pnt ) ;
	if( bCheck )
        MathCAM_DefineNormalAt( CheckMdl, cTol, pnt, nor ) ;
	else
	{
		for( i = 0 ; i < 3 ; i++ )
			nor[i] = ( ( CSmtCutPointEx *)pPath->m_pHead )->m_fSurfNor[i] ; 
	}
	if( nType != surfncMethodSFinish )
        nor[2] = 0.f ;
	if( !nc_Normalize( nor, 3 ) )
	{
		if( !bCheck )
		{
			CSmtCutPointEx *pHead = ( CSmtCutPointEx* )pPath->m_pHead ;
			mathFCpyPnt( pHead->m_fSurfNor, nor ) ;
			nor[2] = 0. ;
			if( !nc_Normalize( nor, 3 ) )
			{
				nor[0] = nor[1] = 0.f, nor[2] = 1.f ;
			}
		}
		else
		{
			nor[0] = nor[1] = 0.f, nor[2] = 1.f ;
		}
	}	
	mathFGetUnitVec( pPath->m_pHead->next->m_fPoint, pPath->m_pHead->m_fPoint, vec ) ;
	for( i = 0 ; i < 3 ; i++ )
	{
		tan[i] = TFLOAT( vec[i] ) ;
	}
	// 然后计算切入切出
	CSmtCutPath tmpLead( MINI_CONNECT_PATH ) ;
	if( !MathCAM_GenLeadPathEx( CheckMdl, cTol, cConnect, 
								pnt, tan, nor, tmpLead, bCheck ) )
	{
		return ;
	}
	tmpLead.ReverseDirect () ;
	LeadIn = new CSmtCutPath( MINI_CONNECT_PATH ) ;
	LeadIn->m_bFeedType = JDNC_FEEDTYPE_LEAD ;
	LeadIn->AppendCutPoint ( tmpLead.m_pHead ) ;
	LeadIn->DefineBox () ;

	tmpLead.m_pHead = tmpLead.m_pTail = NULL ;
	tmpLead.ClearAllPoint () ;
	
}
void CalcCutPathLeadOut( CSmtCheckMdl &CheckMdl, JDNC_TOL &cTol, JDNC_CONNECT3D &cConnect, 
						 CSmtCutPath *pPath, CSmtCutPath *&LeadOut, BOOL bCheck, int nType )
{
	// 首先得到参数
	int i = 0 ;
	FPNT3D pnt, tan, nor = { 0, 0, 1 } ;
	VEC3D vec ;
	mathFCpyPnt( pPath->m_pTail->m_fPoint, pnt ) ;
	if( bCheck )
        MathCAM_DefineNormalAt( CheckMdl, cTol, pnt, nor ) ;
	else
	{
		for( i = 0 ; i < 3 ; i++ )
			nor[i] = ( ( CSmtCutPointEx *)pPath->m_pTail )->m_fSurfNor[i] ; 
	}
	if( nType != surfncMethodSFinish )
        nor[2] = 0.f ;
	if( !nc_Normalize( nor, 3 ) )
	{
		if( !bCheck )
		{
			CSmtCutPointEx *pTail = ( CSmtCutPointEx *)pPath->m_pTail ;
			mathFCpyPnt( pTail->m_fSurfNor, nor ) ;
			nor[2] = 0. ;
			if( !nc_Normalize( nor, 3 ) )
			{
				nor[0] = nor[1] = 0.f, nor[2] = 1.f ;
			}
		}
		else
		{
			nor[0] = nor[1] = 0.f, nor[2] = 1.f ;
		}
	} 	
	mathFGetUnitVec( pPath->m_pTail->prev->m_fPoint, pPath->m_pTail->m_fPoint, vec ) ;
	for( i = 0 ; i < 3 ; i++ )
	{
		tan[i] = TFLOAT( vec[i] ) ;
	}
	// 然后计算切入切出
	CSmtCutPath tmpLead( MINI_CONNECT_PATH ) ;
	if( !MathCAM_GenLeadPathEx( CheckMdl, cTol, cConnect, 
								pnt, tan, nor, tmpLead, bCheck ) )
	{
		return ;
	}
	LeadOut = new CSmtCutPath( MINI_CONNECT_PATH ) ;
	LeadOut->m_bFeedType = JDNC_FEEDTYPE_LEAD ;
	LeadOut->AppendCutPoint ( tmpLead.m_pHead ) ;
	LeadOut->DefineBox () ;

	tmpLead.m_pHead = tmpLead.m_pTail = NULL ;
	tmpLead.ClearAllPoint () ;
}

// 为曲面流线磨损补偿路径添加水平的圆弧进刀路径 qqs 2013.09.23
void CalcCutPath2DLeadIn( CSmtCheckMdl &CheckMdl, JDNC_TOL &cTol, JDNC_CONNECT3D &cConnect, 
					   CSmtCutPath *pPath, CSmtCutPath *&LeadIn, BOOL bCheck )
{
	// 首先得到参数
	int i = 0 ;
	FPNT3D pnt, tan, nor = { 0, 0, 1 } ;
	VEC3D vec ;
	mathFCpyPnt( pPath->m_pHead->m_fPoint, pnt ) ;
	if( bCheck )
		MathCAM_DefineNormalAt( CheckMdl, cTol, pnt, nor ) ;
	else
	{
		for( i = 0 ; i < 3 ; i++ )
			nor[i] = ( ( CSmtCutPointEx *)pPath->m_pHead )->m_fSurfNor[i] ; 
	}

	if( !nc_Normalize( nor, 3 ) )
	{
		if( !bCheck )
		{
			CSmtCutPointEx *pHead = ( CSmtCutPointEx* )pPath->m_pHead ;
			mathFCpyPnt( pHead->m_fSurfNor, nor ) ;
			nor[2] = 0. ;
			if( !nc_Normalize( nor, 3 ) )
			{
				nor[0] = nor[1] = 0.f, nor[2] = 1.f ;
			}
		}
		else
		{
			nor[0] = nor[1] = 0.f, nor[2] = 1.f ;
		}
	}	

	FPNT4D start, end;
	for (int j = 0; j < 4; j++)
	{
		start[j] = pPath->m_pHead->m_fPoint[j];
		end[j] = pPath->m_pHead->next->m_fPoint[j];
	}
	end[2] = start[2];
	mathFGetUnitVec( end, start, vec ) ;
	for( i = 0 ; i < 3 ; i++ )
	{
		tan[i] = TFLOAT( vec[i] ) ;
	}
	// 然后计算切入切出
	CSmtCutPath tmpLead( MINI_CONNECT_PATH ) ;
	if (!(nor[0] < 1.0e-4 && nor[1] < 1.0e-4))
	{
		nor[2] = 0.;
	}
	
	if( !MathCAM_GenLeadPathEx( CheckMdl, cTol, cConnect, 
		pnt, tan, nor, tmpLead, bCheck ) )
	{
		return ;
	}
	tmpLead.ReverseDirect () ;

	LeadIn = new CSmtCutPath( MINI_CONNECT_PATH ) ;
	LeadIn->m_bFeedType = JDNC_FEEDTYPE_LEAD ;
	LeadIn->AppendCutPoint ( tmpLead.m_pHead ) ;
	LeadIn->DefineBox () ;

	tmpLead.m_pHead = tmpLead.m_pTail = NULL ;
	tmpLead.ClearAllPoint () ;

}

// 为曲面流线磨损补偿路径添加水平的圆弧退刀路径 qqs 2013.09.23
void CalcCutPath2DLeadOut( CSmtCheckMdl &CheckMdl, JDNC_TOL &cTol, JDNC_CONNECT3D &cConnect, 
						CSmtCutPath *pPath, CSmtCutPath *&LeadOut, BOOL bCheck )
{
	// 首先得到参数
	int i = 0 ;
	FPNT3D pnt, tan, nor = { 0, 0, 1 } ;
	VEC3D vec ;
	mathFCpyPnt( pPath->m_pTail->m_fPoint, pnt ) ;
	if( bCheck )
		MathCAM_DefineNormalAt( CheckMdl, cTol, pnt, nor ) ;
	else
	{
		for( i = 0 ; i < 3 ; i++ )
			nor[i] = ( ( CSmtCutPointEx *)pPath->m_pTail )->m_fSurfNor[i] ; 
	}

	if( !nc_Normalize( nor, 3 ) )
	{
		if( !bCheck )
		{
			CSmtCutPointEx *pTail = ( CSmtCutPointEx *)pPath->m_pTail ;
			mathFCpyPnt( pTail->m_fSurfNor, nor ) ;
			nor[2] = 0. ;
			if( !nc_Normalize( nor, 3 ) )
			{
				nor[0] = nor[1] = 0.f, nor[2] = 1.f ;
			}
		}
		else
		{
			nor[0] = nor[1] = 0.f, nor[2] = 1.f ;
		}
	} 	
	FPNT4D start, end;
	for (int j = 0; j < 4; j++)
	{
		start[j] = pPath->m_pTail->m_fPoint[j];
		end[j] = pPath->m_pTail->prev->m_fPoint[j];
	}
	end[2] = start[2];
	mathFGetUnitVec( end, start, vec ) ;

	for( i = 0 ; i < 3 ; i++ )
	{
		tan[i] = TFLOAT( vec[i] ) ;
	}

	// 然后计算切入切出
	CSmtCutPath tmpLead( MINI_CONNECT_PATH ) ;

	if (!(nor[0] < 0.01 && nor[1] < 0.01))
	{
		nor[2] = 0.;
	}
	if( !MathCAM_GenLeadPathEx( CheckMdl, cTol, cConnect, 
		pnt, tan, nor, tmpLead, bCheck ) )
	{
		return ;
	}
	
	LeadOut = new CSmtCutPath( MINI_CONNECT_PATH ) ;
	LeadOut->m_bFeedType = JDNC_FEEDTYPE_LEAD ;
	LeadOut->AppendCutPoint ( tmpLead.m_pHead ) ;
	LeadOut->DefineBox () ;

	tmpLead.m_pHead = tmpLead.m_pTail = NULL ;
	tmpLead.ClearAllPoint () ;
}
////////////////////////////////////////////////////////////////////////////////////////
// RoughLink.cpp
CRoughLink::CRoughLink( double dDepth )
{
	m_dDepth = dDepth ;
}
CRoughLink::~CRoughLink()
{
}
void CRoughLink::InitProg ()
{
	m_cPrgDef.m_dLimitAt = 100 / ( 100 * 1.00  ) ;
	m_cPrgDef.m_dStepAt  = 0 ;
	m_cPrgDef.m_dIncStep = 1. ;
	m_cPrgDef.m_pBrkFunc = NULL ;
	m_cPrgDef.m_pPrgFunc = NULL ;
}

BOOL CRoughLink::AddLinkPath ( CSmtCheckMdl* DriveMdl, CPathCombine *pTComb, JDNC_TOL& Tol, double dLastH )
{	
	InitProg() ;																							
	DriveMdl->m_bCheckMode = SMART_MODEL_PROJECT ;
	CPathCombine *pHead = pTComb, *pNext = NULL, *pCurr = NULL ;
	
	pCurr = pHead ;
	// 为第一层添加连刀
	if( pCurr ) 
		AddSameLayerLink( DriveMdl, pHead, Tol, dLastH ) ;
	pHead = pHead->next ;
	while( pHead )
	{
		pNext = pHead->next ;
		// 首先为自身添加连刀路径
		AddSameLayerLink( DriveMdl, pHead, Tol, dLastH ) ;
		// 然后为前后两个添加连刀
		AddNextLayerLink( DriveMdl, pCurr, pHead, Tol, dLastH ) ;

		pCurr = pHead ;
		pHead = pNext ;
	}
	
	return TRUE ;
}
BOOL CRoughLink::AddSameLayerLink ( CSmtCheckMdl* DriveMdl, CPathCombine* TComb, JDNC_TOL& Tol, double dLastH )
{
	if( !TComb || !TComb->m_pHead ) return FALSE ;
	CPathEntity *pHead = TComb->m_pHead ;
	CPathEntity *pNext = NULL, *pNew = NULL ;
	PNT3D start, end ;
	double dist = 0. ;
	double dLen = DriveMdl->m_pTool->m_fRadius * 4 ;
	pHead->GetEndPoint ( 1, start ) ;
	pHead = pHead->next ;
	while( pHead )
	{
		pNext = pHead->next ;
		pHead->GetEndPoint ( 0, end ) ;
		dist = mathDist( start, end ) ;
		if( dist > 0.001 && dist < dLen )
		{
			pNew = AddSafeLink( DriveMdl, start, end, Tol, dLastH ) ;
			if( pNew )  InsertBefore( TComb, pNew, pHead ) ;
		}
		pHead->GetEndPoint( 1, start ) ;
		pHead = pNext ;
	}
	return TRUE ;
}
BOOL CRoughLink::AddNextLayerLink ( CSmtCheckMdl* DriveMdl, CPathCombine *pHead, CPathCombine *pNext, JDNC_TOL& Tol, double dH )
{
	if( !pHead || !pNext ) return FALSE ;
	if( !pHead->m_pTail || !pNext->m_pHead ) return FALSE ;
	// STEP1 : pHead的末点和pNext的起点
	PNT3D start, end ;
	pHead->m_pTail->GetEndPoint ( 1, start ) ;
	pNext->m_pHead->GetEndPoint ( 0, end   ) ;
	CPathEntity *pNew = NULL ;
	double dLen = DriveMdl->m_pTool->m_fRadius * 4 ;
	double dist = mathDist( start, end ) ;
	
	BOOL bRet = FALSE ;
	if( dist > 0.001 && dist < dLen )
	{
		pNew = AddSafeLink( DriveMdl, start, end, Tol, dH ) ;
		if( pNew )
		{
			InsertAfter( pHead, pNew, pHead->m_pTail ) ;
			bRet = TRUE ;
		}
	}
	
	return bRet ;
}

void CRoughLink::InsertBefore ( CPathCombine* TComb, CPathEntity* pNew, CPathEntity* pNext )
{
	if( !TComb || !pNew ) return ;
	CPathEntity *pHead = pNew, *pTail = pNew ;
	while( pTail && pTail->next ) pTail = pTail->next ;
	// 添加到尾部
	if( !pNext )
	{
		while( pHead )
		{
			pTail = pHead->next ;
			TComb->AddEntity ( pHead ) ;
			pHead = pTail ;
		}
	}
	else
	{
		pTail->next = pNext ;
		pHead->prev = pNext->prev ;
		if( pNext->prev )
			pNext->prev->next = pHead ;
		else
			TComb->m_pHead = pHead ;
		pNext->prev = pTail ;
		/*
		pNew->next = pNext ;
		pNew->prev = pNext->prev ;
		if( pNext->prev )
			pNext->prev->next = pNew ;
		else
			TComb->m_pHead = pNew ;
		pNext->prev = pNew ;
		*/
	}
}
void CRoughLink::InsertAfter( CPathCombine* TComb, CPathEntity* pNew, CPathEntity* pPrev )
{
	if( !TComb || !pNew ) return ;
	CPathEntity *pHead = pNew, *pTail = pNew ;
	while( pTail && pTail->next ) pTail = pTail->next ;
	if( !pPrev )
	{
		while( pHead )
		{
			pTail = pHead->next ;
			TComb->AddEntity ( pHead ) ;
			pHead = pTail ;
		}
	}
	else
	{
		pHead->prev = pPrev ;
		pTail->next = pPrev->next ;
		if( pPrev->next )
			pPrev->next->prev = pTail ;
		else
			TComb->m_pTail = pTail ;
		pPrev->next = pHead ;
		/*
		pNew->prev = pPrev ;
		pNew->next = pPrev->next ;
		if( pPrev->next )
			pPrev->next->prev = pNew ;
		else
			TComb->m_pTail = pNew ;
		pPrev->next = pNew ;*/
	}
}

CPathEntity* CRoughLink::AddPathEntity ( CPathEntity* pHead, CPathEntity* pPath )
{
	// 返回
	if( !pPath ) return pHead ;
	if( !pHead ) return pPath ;
	// 如果链表有值,找到最后一个
	CPathEntity* pEntity = pHead ;
	while( pEntity && pEntity->next )
		pEntity = pEntity->next ;
	// 双向链表赋值
	pEntity->next = pPath ;
	pPath->prev = pEntity ;
	return pHead ;
}

CSmartSect* CRoughLink::AddSmartSect ( CSmartSect* pHead, CSmartSect* pSect )
{
	// 返回
	if( !pSect ) return pHead ;
	if( !pHead ) return pSect ;
	// 如果链表有值,找到最后一个
	CSmartSect* pEntity = pHead ;
	while( pEntity && pEntity->next )
		pEntity = pEntity->next ;
	// 双向链表赋值
	pEntity->next = pSect ;
	pSect->prev = pEntity ;
	return pHead ;
}

CPathEntity* CRoughLink::AddSafeLink( CSmtCheckMdl *DriveMdl, PNT3D start, PNT3D end, JDNC_TOL &Tol, double dLastH )
{
	if( !DriveMdl ) return NULL ;

	// STEP 1: 两点之间生成干涉路径,找到最大的高度h
	PNT3D p1, p2 ;
	mathCpyPnt( start, p1 ) ;
	mathCpyPnt( end  , p2 ) ;
	
	p1[2] += m_dDepth ;
	p2[2] += m_dDepth ;

	double dMaxH = max( p1[2], p2[2] ) ;
	dMaxH = max( dMaxH, dLastH ) ;

	CSmtCutPath  tmpPath( MINI_CONNECT_PATH ) ;
    tmpPath.AddPoint( p1 ) ;
    tmpPath.AddPoint( p2 ) ;
    tmpPath.InsertCPoint( Tol.m_dMaxStep ) ;
	tmpPath.VerifyLinePath ( *DriveMdl, Tol, m_cPrgDef ) ;
    tmpPath.DefineBox() ;
    if( tmpPath.m_fBox[1][2] > dMaxH ) return NULL ;
	
	// 计算高度 
	dLastH -= m_dDepth ;
	CPathEntity* pEntity = GetPLineEntity( start, end, dMaxH - m_dDepth ) ;
	return pEntity ;
}

CPathEntity* CRoughLink::GetPLineEntity ( PNT3D start, PNT3D end, double h )
{
	PNT3D p1, p2 ;
	mathCpyPnt( start, p1 ) ;
	mathCpyPnt( end,   p2 ) ;
	p1[2] = p2[2] = h ;
	CPathEntity *pHead = NULL ;
	// 生成四段直线
	CPathPLine3D *pLine = new CPathPLine3D() ;
	pLine->m_bFeedType = JDNC_FEEDTYPE_ROUGH ;
	pLine->m_nCount = 1 ;
	pLine->m_pTAPos = new PNT3D[2] ;
	mathCpyPnt( start, pLine->m_pTAPos[0] ) ;
	mathCpyPnt( p1, pLine->m_pTAPos[1] ) ;
	pHead = AddPathEntity( pHead, pLine ) ;
	
	pLine = new CPathPLine3D() ;
	pLine->m_bFeedType = JDNC_FEEDTYPE_ROUGH ;
	pLine->m_nCount = 1 ;
	pLine->m_pTAPos = new PNT3D[2] ;
	mathCpyPnt( p1, pLine->m_pTAPos[0] ) ;
	mathCpyPnt( p2, pLine->m_pTAPos[1] ) ;
	pHead = AddPathEntity( pHead, pLine ) ;

	pLine = new CPathPLine3D() ;
	pLine->m_bFeedType = JDNC_FEEDTYPE_ROUGH ;
	pLine->m_nCount = 1 ;
	pLine->m_pTAPos = new PNT3D[2] ;
	mathCpyPnt( p2, pLine->m_pTAPos[0] ) ;
	mathCpyPnt( end, pLine->m_pTAPos[1] ) ;
	pHead = AddPathEntity( pHead, pLine ) ;
	
	return pHead ;
}
BOOL CRoughLink::AddPCombToHead ( CPathCombine *pHead, CPathCombine *TComb )
{
	if( !TComb ) return FALSE ;
	if( !pHead ) pHead = TComb ;

	CPathEntity *pCurr = NULL, *pNext = NULL ;
	pCurr = TComb->m_pHead ;
	while( pCurr )
	{
		pNext = pCurr->next ;
		TComb->RemoveEntity ( pCurr ) ;
		pCurr->next = pCurr->prev = NULL ;
		pHead->AddEntity ( pCurr ) ;
		pCurr = pNext ;
	}
	delete TComb ;
	return TRUE ;
}
/////////////////////////////////////////////////////////////////////////////////////
// 用于残料补加工，存储环和其路径
CSmtLoopPath::CSmtLoopPath( CSmartLoop* pLoop, JDNC_FEED &cFeed, double dDepth, 
						    double dRadius, double dStep, BOOL bRemain )
{
	m_pLoop = pLoop ;
	m_dStep = dStep ;
	m_dDepth = dDepth ;
	m_cFeed = cFeed	;
	m_pPlunge = NULL ;
	m_pPtLoop = NULL ;
	m_dRadius = dRadius ;
	m_bAddPath = FALSE  ;
	m_bUsed	= FALSE ;
	m_bRemain = bRemain ;
	next = prev = NULL ;
	m_cPrgDef.m_dLimitAt = 0. ;
	m_cPrgDef.m_dStepAt  = 0. ;
	m_cPrgDef.m_dIncStep = 0. ;
	m_cPrgDef.m_pBrkFunc = NULL ; // 中断函数
	m_cPrgDef.m_pPrgFunc = NULL ; // 前进函数 
	m_cPrgDef.m_pNewFunc = NULL ; // 新启进度
	m_cPrgDef.m_pPosFunc = NULL ; // 进度定位
	m_bClosedFlag = FALSE ;
}
CSmtLoopPath::~CSmtLoopPath()
{
	m_pPtLoop = NULL ;
	next = prev = NULL ;
	RemoveLibArr() ;
	m_cPathLib.ClearAllPath () ;
	if( m_pLoop ) 
		Mini_DeleteContours( m_pLoop ) ;
}
void CSmtLoopPath::RemoveLibArr ()
{
	int  i, nSize = (int)m_cLibArr.GetSize () ;
	for( i = 0 ; i < nSize ; i++ )
	{
		CSmartCurveLib *pLib = m_cLibArr[i] ;
		pLib->DeleteAllCurves () ;
		delete pLib ;
	}
	m_cLibArr.RemoveAll () ;
}
void CSmtLoopPath::SetBoundInfo( JDNC_SPEED &cSpeed, TFLOAT fRatio, BOOL bBoundTol, double dBoundTol )
{
	m_cSpeed = cSpeed ;
	m_fFeedRatio = fRatio ;
	m_bBoundTol = bBoundTol ;
	m_dBoundTol = dBoundTol ;
}
void CSmtLoopPath::SetPlungeInfo ( JDNC_PLUNGE &cPlunge )
{
	m_cPlunge = cPlunge ;
}
void CSmtLoopPath::ReverseAllCurve ()
{
	int  i,  nSize =(int) m_cLibArr.GetSize () ;
	if( nSize < 1 ) return ;
	CSmartCurve *pHead = NULL ;
	CSmartCurveLib *pCurLib = NULL ;
	for( i = 0 ; i < nSize ; i++ )
	{
		pCurLib = m_cLibArr.GetAt ( i ) ;
		pHead = pCurLib->m_pHead ;
		for( ; pHead ; pHead = pHead->next )
		{
			pHead->Reverse () ;
		}
	}	
}
///////////////////////////////////////上面是另外一种连接方法////////////////////////////////////////

BOOL CSmtLoopPath::ConnectLine ( CSmtCheckMdl *CheckMdl, PNT3D start, PNT3D end, double dDepth, JDNC_TOL& cTol )
{
	UNUSED_ALWAYS( dDepth ) ;
	double len = mathDist( start, end ) ; 
	if( len > CheckMdl->m_pTool->m_fRadius * 2 )
		return FALSE ;
	CSmtCutPath tmpPath( MINI_CONNECT_PATH ) ;
	tmpPath.AddPoint ( start ) ;
    tmpPath.AddPoint ( end ) ;
//	pPath->m_pHead->m_fPoint[2] = pPath->m_pTail->m_fPoint[2] = TFLOAT( dDepth ) ;
	tmpPath.InsertCPoint ( cTol.m_dMaxStep ) ;
	CSmtCutPoint *pHead = tmpPath.m_pHead ;
	for( ; pHead ; pHead = pHead->next )
	{
		pHead->m_fPoint[3] = pHead->m_fPoint[2] ;
	}
	tmpPath.VerifyLinePath ( *CheckMdl, cTol, m_cPrgDef ) ;

	BOOL bConnect = TRUE ;
	pHead = tmpPath.m_pHead ;
	for( ; pHead ; pHead = pHead->next )
	{
		if( pHead->m_fPoint[2] > pHead->m_fPoint[3] + 0.001 )
		{
			bConnect = FALSE ;
			break ;
		}
	}
	return bConnect ;
}

void CSmtLoopPath::ConnectAllCurve ( CSmtCheckMdl *CheckMdl, CSmartLoop *pPlunge, CSmartCurveLib& BndCurve, 
									 BOOL bZigZag, JDNC_SETUP &cSetup, JDNC_PLUNGE &cPlunge, double dDepth )
{
	int  i,  nSize = (int)m_cLibArr.GetSize () ;
	if( nSize < 1 ) return ;
	m_cCLib = BndCurve ;
	CSmartCurve *pCurve = NULL ;
	BOOL bAllCurve = TRUE, bFst = FALSE, bAddClose = FALSE ;
	// 从相同的曾数之间进行
	while( 1 )
	{
		bFst = TRUE ;
		bAllCurve = TRUE , bAddClose = FALSE ;
		for( i = nSize - 1 ; i >= 0 ; i-- )
		{
			if( m_cLibArr[i]->m_nNumCurve < 1 )
				continue ;

			if( bFst )
			{
				pCurve = m_cLibArr[i]->m_pHead ;
				m_cLibArr[i]->RemoveCurve ( pCurve ) ;
				AddPlungeCurve( CheckMdl, m_cPathLib, i, pPlunge, pCurve, cSetup, cPlunge, bAddClose, TRUE, bFst, bZigZag ) ;
			}
			else
			{
				FindNearCurve( CheckMdl, m_cPathLib, pPlunge, m_cLibArr[i], 
								i, bZigZag, cSetup, cPlunge, dDepth, bAddClose ) ;
			}
			bFst = FALSE ;
			bAllCurve = FALSE ;
		}
		if( bAllCurve ) break ;
	}
	// 删除曲线
	RemoveLibArr() ;
	m_cCLib.RemoveAll () ;
}

BOOL CSmtLoopPath::ConnectAllCurve ( CSmtCheckMdl *CheckMdl, CSmartLoop *pPlunge, CSmartCurveLib &BndCurve, 
									 BOOL bZigZag, BOOL bPrev, PNT3D prev, JDNC_SETUP &cSetup, double dDepth )
{
	if( m_cPathLib.m_cAllPath.GetCount () > 0 ) 
		return TRUE ;
	int  i,  nSize = (int)m_cLibArr.GetSize () ;
	if( nSize < 1 ) return TRUE ;
	CSmartCurve *pCurve = NULL ;
	m_cCLib = BndCurve ;
	m_pPlunge = pPlunge ;
	BOOL bAllCurve = TRUE, bFst = FALSE ;
    BOOL  bDefPrev = bPrev, bAddClose = FALSE, bLeadOut = FALSE ;
    PNT3D dSeedPnt = { 0.0, 0.0, 0.0 } ;
    if( bDefPrev ) nc_VectorCopy( dSeedPnt, prev , 3 ) ;
	// 从相同的曾数之间进行
	while( 1 )
	{
		bFst = TRUE, bAllCurve = TRUE, bAddClose = FALSE ;
		for( i = nSize - 1 ; i >= 0 ; i-- )
		{
			if( m_cLibArr[i]->m_nNumCurve < 1 )
				continue ;

			if( bFst )
			{
                if( bDefPrev)
				{
					pCurve = FindFstCurve( *m_cLibArr[i], dSeedPnt, bZigZag ) ;
				}
				else
				{
					pCurve = m_cLibArr[i]->m_pHead ;
					m_cLibArr[i]->RemoveCurve ( pCurve ) ;
				}

				// 添加退刀
				if( nSize == 1 && m_cLibArr[i]->m_nNumCurve == 0 && pCurve->IsClosed () )
				{
					bLeadOut = TRUE ;
				}
				else
				{
					bLeadOut = FALSE ;
				}
				// 添加进刀
				AddPlungeCurve( CheckMdl, m_cPathLib, i, m_pPlunge, pCurve, cSetup, m_cPlunge, bAddClose, TRUE, bFst, bZigZag, bLeadOut ) ;
				
			}
			else
			{
				FindNearCurve( CheckMdl, m_cPathLib, m_pPlunge, m_cLibArr[i], 
								i, bZigZag, cSetup, m_cPlunge, dDepth, bAddClose ) ;
			}
			bFst = FALSE ;
			bAllCurve = FALSE ;
            if( m_cPathLib.m_cAllPath.GetCount() )
            {
                CSmtCutPath *pTPath = m_cPathLib.m_cAllPath.GetTail() ;
                nc_FloatToDouble( dSeedPnt, pTPath->m_pTail->m_fPoint, 3 ) ;
                bDefPrev = TRUE ;
            }
		}
		if( bAllCurve ) break ;
	}
	// 删除曲线
	RemoveLibArr() ;
	m_cCLib.RemoveAll () ;
	return TRUE ;
}
CSmartCurve *CSmtLoopPath::FindFstCurve ( CSmartCurveLib &cCurveLib, PNT3D end, BOOL bZigZag )
{
	if( !cCurveLib.m_pHead ) return NULL ;
	CSmartCurve *pHead = cCurveLib.m_pHead, *pObj = NULL ;
	double dMin = MAX_DBL, dDist = 0. ;
	PNT3D p[2] ;
	BOOL bReverse = FALSE ;
	for( ; pHead ; pHead = pHead->next )
	{
		pHead->GetStart ( p[0] ) ;
		pHead->GetEnd   ( p[1] ) ;
		p[0][2] = p[1][2] = m_dDepth ;
		dDist = mathDist( end, p[0] ) ;
		if( dDist < dMin )
		{
			dMin = dDist ;
			pObj = pHead ;
			bReverse = FALSE ;
		}
		dDist = mathDist( end, p[1] ) ;
		if( bZigZag && dDist < dMin )
		{
			dMin = dDist ;
			pObj = pHead ;
			bReverse = TRUE ;
		}
	}
	if( pObj )
	{
		cCurveLib.RemoveCurve ( pObj ) ;
		if( bReverse && !(pObj->IsClosed()) )
			pObj->Reverse () ;
	}
	return pObj ;
}
void CSmtLoopPath::AddCurveToPathLib ( CSmartCurve *pCurve, CSmtCPathLib &cPathLib, BOOL bDel, int  nLevel, JDNC_TOL &cTol )
{
	if( !pCurve ) return ;
	CSmtCutPath *pPath = TransfCurveToPath( pCurve, cTol, bDel ) ;
	if( m_bBoundTol && nLevel == 0 )
	{
		pPath->m_fFeedRate = float( m_cSpeed.m_dFeedRate * m_fFeedRatio ) ;
	}
	cPathLib.AddToTail ( pPath ) ;
}
CSmtCutPath * CSmtLoopPath::TransfCurveToPath(  CSmartCurve * pCurve , JDNC_TOL &cTol, BOOL IsDel ) 
{
	CSmartSect * pSect = pCurve->GetHead () ;
	if( ! pSect ) return NULL ; 
	PNT4D  dPoint ;
	CSmtCutPath * pPath = new CSmtCutPath() ;
	dPoint[2] = m_dDepth , dPoint[3] = 0.0 ;
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

void CSmtLoopPath::FindNearCurve ( CSmtCheckMdl *CheckMdl, CSmtCPathLib &cPathLib, CSmartLoop *pPlunge, CSmartCurveLib* pLib, 
								   int  i, BOOL bZigZag, JDNC_SETUP &cSetup, JDNC_PLUNGE &cPlunge, double dDepth, BOOL &bAddClose )
{
	UNUSED_ALWAYS( dDepth ) ;
	if( pLib->m_nNumCurve < 1 ) return ;
	CSmartCurve *pHead = pLib->m_pHead, *pObj = NULL ;
	CSmtCutPath *pPath  = NULL ;
	// 如果空,添加第一条
	if( cPathLib.m_cAllPath.GetCount () < 1 )
	{
		pLib->RemoveCurve ( pHead ) ;
		pPath = TransfCurveToPath( pHead, cSetup.m_cTolDef , TRUE ) ;
		cPathLib.AddToTail ( pPath ) ;
	}
	PNT3D start, end ;
	BOOL bFind = FALSE ;
	// 取得当前路经(非进退刀\非连刀)最后一点
	POSITION pos,atpos ;
	pos = cPathLib.m_cAllPath.GetTailPosition () ;
	while( pos )
	{
		pPath = cPathLib.m_cAllPath.GetPrev ( pos ) ;
		if( pPath->GetCutMode () == MINI_MILL_PATH )
		{
			GetSmtPathEndPnt( pPath, 1, start ) ;
			bFind = TRUE ;
			break ;
		}
	}
	if( !bFind ) return ;
	// 寻找最近曲线(加上进退刀就不准了)
	PNT3D p1, p2 ;
	PNT2D nearpt ;
	CSmartSect *pFind = NULL ;
	for( pHead = pLib->m_pHead ; pHead ; pHead = pHead->next )
	{
		if( IsNextCurve( pHead, start, i, pFind , nearpt, cSetup.m_cTolDef.m_dArcTol ) )
		{
			pObj = pHead ;
			break ;
		}
	}
	if( !pObj ) return ;
	pLib->RemoveCurve ( pObj ) ;
	BOOL bClose = pObj->IsClosed () ;
	// 得到当前连刀或者进退刀的最后点
	pPath = cPathLib.m_cAllPath.GetTail () ;
	GetSmtPathEndPnt( pPath, 1, start ) ;
	
//	double dLink = m_cFeed.m_cConnectDef.m_dMaxDist  ;
//	if( !m_bRemain ) dLink = CheckMdl->m_pTool->m_fRadius * 4 ;
	double dLink = CheckMdl->m_pTool->m_fRadius * 2 , dLen = 1.0e12 ;
	// 为pObj添加切入切出路径
	CSmtCPathLib tmpLib ;
	if( pObj->IsClosed () )
	{// 如果闭合,判断是否需要添加下刀路径
		if( pFind ) 
		{
			pObj->SetStartPoint ( pFind, nearpt ) ;
		}
		pObj->GetStart ( p1 ) ;
		p1[2] = m_dDepth ;
		BOOL bAdd = FALSE ;
		if( mathDist( start, p1 ) > dLink )
		{
			AddPlungeCurve( CheckMdl, tmpLib, i, pPlunge, pObj, cSetup, cPlunge, bAdd, TRUE, FALSE, bZigZag) ;
			bAddClose = bAdd ;
		}
		else
		{
			AdjustCloseCurveDir( pObj, cSetup ) ;
			AddCurveToPathLib( pObj, tmpLib, TRUE, i, cSetup.m_cTolDef ) ;
		}
		GetSmtCPathLibEndPoint( tmpLib, end, TRUE ) ;
		dLen = mathDist( start, end ) ;
	}
	else
	{
		AddPlungeCurve( CheckMdl, tmpLib, i, pPlunge, pObj, cSetup, cPlunge, bAddClose, TRUE, FALSE, bZigZag ) ;
		if( !bZigZag )
		{
			GetSmtCPathLibEndPoint( tmpLib, end, TRUE ) ;
			dLen = mathDist( start, end ) ;
		}
		else
		{
			GetSmtCPathLibEndPoint( tmpLib, p1, TRUE ) ;
			GetSmtCPathLibEndPoint( tmpLib, p2, FALSE ) ;
			double l1 = mathDist( start, p1 ) ;
			double l2 = mathDist( start, p2 ) ;
			if( l1 > l2 )
			{
				dLen = l2 ;
				mathCpyPnt( p2, end ) ;
				if( !bClose )
					ReverseAllPath( tmpLib ) ;
			}
			else
			{
				dLen = l1 ;
				mathCpyPnt( p1, end ) ;
			}
		}
	}
	if( dLen < dLink )
	{
		pPath = CreateAdsorbLine( CheckMdl, start, end, cSetup.m_cTolDef ) ;
		if( pPath )
		{
			cPathLib.AddToTail ( pPath ) ;
		}
		else
            bAddClose = FALSE ;
	}
	else
		bAddClose = FALSE ;
	pos = tmpLib.m_cAllPath.GetHeadPosition () ;
	while( pos )
	{
		atpos = pos ;
		pPath = tmpLib.m_cAllPath.GetNext ( pos ) ;
		cPathLib.AddToTail ( pPath ) ;
		tmpLib.m_cAllPath.RemoveAt ( atpos ) ;
	}
	tmpLib.m_cAllPath.RemoveAll () ;
}
BOOL CSmtLoopPath::IsNextCurve( CSmartCurve * pCurve, PNT3D p, int  i, CSmartSect *&pNearSect, PNT2D nearpt, double dTol )
{
	// 判断点p 到直线pCurve的距离是否是m_dStep 
	PNT2D start, mid ;
	start[0] = p[0], start[1] = p[1] ;

	// 然后利用点到线的距离进行判断
	double dist = GetPntToCurveDist( start, pCurve, pNearSect, nearpt ) ;
	double dStep = m_dStep ;
	if( m_bBoundTol && i == 0 ) dStep = m_dBoundTol ;

	if( fabs( dist - dStep ) > dTol * 1.5 ) return FALSE ;
	    
	// 还要判断上面一层中的曲线到这条线的距离都比较远
	CSmartCurveLib* pLib = m_cLibArr[i+1] ;
	if( pLib->m_nNumCurve < 1 )
		return TRUE ;
	CSmartSect *pSect = NULL ;
	PNT2D tmp ;
	CSmartCurve *pHead = pLib->m_pHead ;
	CSmartSect *pTmpSect = NULL ;
	for( ; pHead ; pHead = pHead->next )
	{
		// 如果上层中有曲线到这条的距离比半径小，返回FASLE
		pSect = pHead->m_pHead ;
		for( ; pSect ; pSect = pSect->next )
		{
			pSect->GetStart ( start ) ;
			pSect->GetPoint ( 0.5, mid ) ;
			dist = GetPntToCurveDist( start, pCurve, pTmpSect, tmp ) ;
			if( fabs( dist - m_dStep ) < 0.005 )
				return FALSE ;
			dist = GetPntToCurveDist( mid,   pCurve, pTmpSect, tmp ) ;
            if( fabs( dist - m_dStep ) < 0.005 )
				return FALSE ;
		}
	}

	return TRUE ;
}
double CSmtLoopPath::GetPntToCurveDist ( PNT2D p, CSmartCurve *pCurve, CSmartSect *&pNearSect, PNT2D nearpt )
{
	PNT2D end ;
	double dMinDist = 1.0e8, dDist ;
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
		if( fabs( m_dStep - dDist ) <= 0.001 )
		{
			return dDist ;
		}
	}
	return dMinDist ;
}

void CSmtLoopPath::GetEndPoint ( CPathEntity *pEntity, BOOL bFlag, PNT3D point )
{
	pEntity->GetEndPoint ( bFlag, point ) ;
	point[2] = m_dDepth ;
}
void CSmtLoopPath::GetEndPoint( CPathCombine *PComb, BOOL bFlag, PNT3D point )
{
	PComb->GetEndPoint ( bFlag, point ) ;
	point[2] = m_dDepth ;
}
CSmtCutPath *CSmtLoopPath::CreatePathLine ( PNT3D start, PNT3D end, double dDepth )
{
	UNUSED_ALWAYS( dDepth ) ;
	CSmtCutPath *pPath = new CSmtCutPath( MINI_CONNECT_PATH ) ;
	pPath->m_bFeedType = JDNC_FEEDTYPE_CONNECT ;
	pPath->AddPoint ( start ) ;
	pPath->AddPoint ( end   ) ;
	return pPath ;
}
CSmtCutPath *CSmtLoopPath::CreateAdsorbLine ( CSmtCheckMdl *DriveMdl, PNT3D start, PNT3D end, JDNC_TOL &cTol )
{
	CSmtCutPath *pLine = NULL ;
	FPNT3D fIntPnt[100] ;
	int i = 0 ;
	for( i = 0 ; i < 3 ; i++ )
	{
		fIntPnt[0][i] = TFLOAT( start[i] ) ;
		fIntPnt[1][i] = TFLOAT( end[i]   ) ;
	}
	double dDepth = max( start[2], end[2] ) + 0.1 ;
	// 吸附
	int nCnt = DriveMdl->AdsorbConnectLine ( cTol, fIntPnt, 100, TRUE ) ;
	BOOL bConnect = TRUE ;
	// 过滤得到的点
	for( i = 0 ; i < nCnt ; i++ )
	{
		if( fIntPnt[i][2] > dDepth )
		{
			bConnect = FALSE ;	
			break ;
		}
	}
	if( nCnt >= 2 && bConnect )
	{
		pLine = new CSmtCutPath( MINI_CONNECT_PATH ) ;
		pLine->m_bFeedType = JDNC_FEEDTYPE_CONNECT ;
		for( i = 0 ; i < nCnt ; i++ )
			pLine->AddPoint ( fIntPnt[i] ) ;
	}
	return pLine ;
}
// 添加下刀和曲线
void CSmtLoopPath::AddPlungeCurve ( CSmtCheckMdl *CheckMdl, CSmtCPathLib &cPathLib, int  i, CSmartLoop *pPlunge, CSmartCurve *pCurve, 
								   JDNC_SETUP &cSetup, JDNC_PLUNGE &cPlunge, BOOL &bAddClose, BOOL bIsDel, BOOL bLine, BOOL bZigZag, BOOL bLeadOut )
{
	if( !pCurve ) return ;
	if( pCurve->IsClosed () && pPlunge )
	{
		AddPlungeCurveForCloseCurve( CheckMdl, cPathLib, i, pPlunge, pCurve, cSetup, 
									 cPlunge, bAddClose, bIsDel, bLine, bLeadOut ) ;
	}
	else
	{
		AddPlungeCurveForOpenCurve( CheckMdl, cPathLib, i, pCurve, cSetup, bIsDel, bZigZag ) ;
	}
}
void CSmtLoopPath::AddPlungeCurveForCloseCurve ( CSmtCheckMdl *CheckMdl, CSmtCPathLib &cPathLib, int  nLevel, CSmartLoop *pPlunge, CSmartCurve *pCurve,
												 JDNC_SETUP &cSetup, JDNC_PLUNGE &cPlunge, BOOL &bAddClose, BOOL bIsDel, BOOL bLine, BOOL bLeadOut )
{
	PNT2D start ;
	pCurve->GetStart ( start ) ;
	// 得到环上距离该点最近的点
//	double dLink = m_cFeed.m_cConnectDef.m_dMaxDist ;
//	if( !m_bRemain ) dLink = CheckMdl->m_pTool->m_fRadius * 4 ;
	double dLink = CheckMdl->m_pTool->m_fRadius * 2 ;
	BOOL bFind = FALSE ;
	
	// 寻找合格的下刀点,并标记
	CPointList allPnt ;
	bFind = FindAllPlungePnt( allPnt, start, dLink ) ;
	
	// 闭合判断顺逆铣
	AdjustCloseCurveDir( pCurve, cSetup ) ;
	
	CSmtCutPath *pPath = NULL ;
	// 生成下刀
	if( bFind && bLine )
	{ 
		// 生成圆弧下刀路径
		if( !AddArcForCloseCurve( CheckMdl, cPathLib, pCurve, allPnt, cSetup, TRUE ) )
		{
			// 圆弧不成生成直线
			PNT3D bgn, pos = { start[0], start[1], m_dDepth } ;
			mathCpyPnt( allPnt.m_pHead->m_dPoint, bgn ) ;
			bgn[2] = m_dDepth ;
			pPath = CreateAdsorbLine( CheckMdl, bgn, pos, cSetup.m_cTolDef ) ;
			if( pPath )
			{
				pPath->m_bFeedType = JDNC_FEEDTYPE_LEAD ;
				RedefineLeadPathHeight( CheckMdl, pPath, TRUE ) ;
				cPathLib.AddToTail ( pPath ) ;
			}
			else 
			{
				// 添加环切路径
				if( !bAddClose )
					bAddClose = AddPlungeForClosedCurve( cPathLib, pCurve, pPlunge, cPlunge ) ;
			}
		}
		else
		{
			if ( cPathLib.GetNumPath() == 1 )
			{
				RedefineLeadPathHeight( CheckMdl, cPathLib.m_cAllPath.GetHead(), TRUE ) ;
			}
		}
	}
	else
	{
		// 为闭合线添加下刀路径
		if( !bAddClose )
			bAddClose = AddPlungeForClosedCurve( cPathLib, pCurve, pPlunge, cPlunge ) ;
	}
	
	CSmtCPathLib tmpLib ;
	if( bLeadOut )
	{
		// 为闭合曲线添加退出圆弧
		AddArcForCloseCurve( CheckMdl, tmpLib, pCurve, allPnt, cSetup, FALSE ) ;
		if ( tmpLib.GetNumPath() == 1 )
		{
			RedefineLastLeadoutPathH(CheckMdl, tmpLib.m_cAllPath.GetHead() ) ;
		}
	}

	AddCurveToPathLib( pCurve, cPathLib, FALSE, nLevel, cSetup.m_cTolDef ) ;
	if( bLeadOut )
	{
		if ( tmpLib.GetNumPath () > 0 )
		{
			AddTmpPathToAllPath( cPathLib, tmpLib ) ;
		} 
		else
		{
			pPath = GetLeadoutSplinePath( CheckMdl, cSetup, pCurve, allPnt ) ;
			if ( pPath )
			{
				cPathLib.AddToTail( pPath ) ;
			}
		}
	}
	if ( bIsDel )
	{
		delete pCurve ;
	}
}
BOOL CSmtLoopPath::FindAllPlungePnt ( CPointList &allPnt, PNT2D pos, double dMaxDist )
{
	CSmartSect *pSect = NULL ;
	CSmartCurve *pHead = NULL ;
	BOOL bFind = FALSE ;
	CSmartLoop *pLoop = NULL ;
	PNT2D seed ;
	PNT3D pt ;
	double dDist = 0. ;
	for( pHead = m_cCLib.m_pHead ; pHead ; pHead = pHead->next )
	{
		pSect = pHead->GetHead () ;
		for( ; pSect ; pSect = pSect->next )
		{
            dDist = pSect->MinDistPoint ( pos, seed ) ;
			if( dDist > dMaxDist ) continue ;
			for( pLoop = m_pPtLoop ; pLoop ; pLoop = pLoop->next )
			{
				if( pLoop->IsPtOnContour ( seed ) == 1    || 
					pLoop->IntLineContour ( seed, pos ) ||
					LineIntAllCurve( seed, pos ) )
					break ;
			}
			if( pLoop ) continue ;
			pt[0] = seed[0], pt[1] = seed[1], pt[2] = dDist ;
			CSmartLPoint* pTPnt = new CSmartLPoint( pt, 0, 1 ) ;
			pTPnt->m_pSect = pSect ;
			allPnt.InsertAfter( pTPnt, allPnt.m_pTail ) ;
			bFind = TRUE ;
		}
	}
	allPnt.SortPoint() ;//距离从小到大排序
	return bFind ;
}
BOOL CSmtLoopPath::AddArcForCloseCurve( CSmtCheckMdl *CheckMdl, CSmtCPathLib &cPathLib, CSmartCurve *pCurve, 
										CPointList &allPnt, JDNC_SETUP &cSetup, BOOL bHead  )
{
	// 首先得到pCurve首点的切向和法向
	PNT3D bgn, pos = { 0., 0., 0. }, tang = { 0., 0., 0. }, nor = { 0., 0., 0. } ;
	if( bHead )
	{
		pCurve->m_pHead->GetStart ( pos ) ;
		pCurve->m_pHead->GetTangent ( 0, tang ) ;
		pCurve->m_pHead->GetNormal ( 0, nor ) ;
	}
	else
	{
		pCurve->m_pTail->GetEnd ( pos ) ;
		pCurve->m_pTail->GetTangent ( 1, tang ) ;
		pCurve->m_pTail->GetNormal ( 1, nor ) ;
	}
	if( allPnt.m_pHead != NULL )
	{//判断法向是否应该反向
		VEC3D vDir = { 0.0, 0.0, 0.0 };
		nc_VectorCopy( bgn, allPnt.m_pHead->m_dPoint, 2 ) ;
		bgn[2] = 0.0 ;
		mathGetVecByPP2D( pos, bgn, MIN_LEN, vDir )  ;
		if ( mathGetAngleUnit( nor, vDir ) > PI1_2 )
		{
			mathRevVec( nor ) ;
		}
	}

	double dAng = ANGLE_TO_RADIAN( 3 ) ;
	CSmartCurve *LeadCurve = NULL ;
	int nTry = 0 ;
	CSmtCPathLib tmpLib ;
	CSmartLPoint *pHead = ( CSmartLPoint *)allPnt.m_pHead ;
	for( ; pHead ; pHead = (CSmartLPoint*)pHead->next )
	{
		mathCpyPnt( pHead->m_dPoint, bgn ) ;
		bgn[2] = 0. ;
		for( nTry = 0 ; nTry < 3 ; nTry++ )
		{
			// 得到曲线
			LeadCurve = CreateLeadArc( pos, tang, nor, bgn, dAng * nTry, bHead ) ;
			if( !LeadCurve ) continue ;
			// 将LeadCurve转换为CSmtCutPath
			TransfSmartCurveToCPath( LeadCurve, tmpLib, m_dDepth, cSetup.m_cTolDef ) ;
			delete LeadCurve ;
			// 判断曲线是否合格
			if( IsLeadArcPath( CheckMdl, tmpLib ) )
			{
				// 将路径放到cPathLib中
				AddTmpPathToAllPath( cPathLib, tmpLib ) ;
				return TRUE ;
			}
			else
			{
				tmpLib.ClearAllPath () ;
			}
		}
	}
	return FALSE ;
}
BOOL CSmtLoopPath::IsLeadArcPath ( CSmtCheckMdl *CheckMdl, CSmtCPathLib &cPathLib )
{
	CSmtCutPath *pPath = NULL ;
	CSmtCutPoint *pHead = NULL ;
	FPNT3D fNode ;
	POSITION pos = cPathLib.m_cAllPath.GetHeadPosition () ;
	while( pos )
	{
		pPath = cPathLib.m_cAllPath.GetNext ( pos ) ;
		CheckMdl->LabelCheckByBox ( pPath->m_fBox ) ;
		pHead = pPath->m_pHead ;
		for( ; pHead ; pHead = pHead->next )
		{
			mathFCpyPnt( pHead->m_fPoint, fNode ) ;
			CheckMdl->DefineHeight ( fNode ) ;
			if( fNode[2] - pHead->m_fPoint[2] > 1.0e-3)
				return FALSE ;
		}
	}
	return TRUE ;
}
CSmartCurve *CSmtLoopPath::CreateLeadArc ( PNT3D pos, PNT3D tang, PNT3D nor, PNT3D bgn, double dAng, BOOL bHead )
{
	double dDist = mathDist2D( bgn, pos ), r = 0.0 ;
	int i = 0 ;
	VEC3D vz, rot_tan, rot_nor, vDir = { 0., 0., 0. } ; 
	PNT2D cen, start, end, mid ;
	// 首先旋转角度
	if( mathVProductUnit( nor, tang, vz ) == ERUNSUC )
		return NULL ;
	if( bHead )
	{
		mathRotVec( vz, pos, dAng, tang, rot_tan ) ;
		mathRotVec( vz, pos, dAng, nor , rot_nor ) ;
		r = min( dDist * 0.25, 1.0 )  ;
	}
	else
	{
		mathRotVec( vz, pos, -dAng, tang, rot_tan ) ;
		mathRotVec( vz, pos, -dAng, nor , rot_nor ) ;
		r = min( dDist * 0.4, 2.0 ) ;
	}
	// 生成圆弧信息cen, start, end, mid
	mathCpyPnt2D( pos, end ) ;
	for( i = 0 ; i < 2 ; i++ )
	{
		cen[i] = pos[i] + rot_nor[i] * r ;
	}
	if ( bHead )
	{
		// 计算bgn->pos方向
		if( mathGetVecByPP2D( pos, bgn, MIN_LEN, vDir ) == ERUNSUC )
			return NULL ;
		// 判断能否生成圆弧
		double dAngle = mathGetAngleUnit( rot_nor, vDir ) ;
		if( dAngle > PI1_2 * 5/6 ) return NULL ;
		double dTmp = mathDist2D( cen, bgn ) ;

		double tanDist =  dTmp * dTmp - r * r ;
		ASSERT( tanDist >0 ) ;
		if( tanDist < 0 ) return NULL ;
		tanDist = sqrt( tanDist ) ;

		VEC3D rot_tmpV, tmpV = { 0., 0., 0. } ;
		if( mathGetVecByPP2D( bgn, cen, MIN_LEN, tmpV ) == ERUNSUC )
			return NULL ;
		double tanAng = asin( r / dTmp ) ;
		if( bHead )
		{
			mathRotVec( vz, bgn, tanAng, tmpV, rot_tmpV ) ;
		}
		else
		{
			mathRotVec( vz, bgn, -tanAng, tmpV, rot_tmpV ) ;
		}
		for( i = 0 ; i < 2 ; i++ )
		{
			start[i] = bgn[i] + rot_tmpV[i] * tanDist ;
			mid[i] = ( start[i] + end[i] ) * 0.5 ;
		}
		if( fabs( mathDist2D( start, cen ) - r ) > 0.01 ) return NULL ;
		if( fabs( mathDist2D( start, end ) - r * 2 ) > 0.005 )
		{ // 小圆弧
			if( mathGetVecByPP2D( cen, mid, MIN_LEN, tmpV ) == ERUNSUC )
				return NULL ;
			for( i = 0 ; i < 2 ; i++ )
			{
				mid[i] = cen[i] + r * tmpV[i] ;
			}
		}
		else
		{ // 半圆弧
			PNT2D tmpMid[2] ;
			mathRotVec( vz, mid, PI1_2, vDir, tmpV ) ;
			for( i = 0 ; i < 2 ; i++ )
			{
				tmpMid[0][i] = mid[i] + r * tmpV[i] ;
				tmpMid[1][i] = mid[i] - r * tmpV[i] ;
			}
			if( mathDist2D( tmpMid[0], bgn ) > mathDist2D( tmpMid[1], bgn ) )
			{
				mathCpyPnt2D( tmpMid[0], mid ) ;
			}
			else
			{
				mathCpyPnt2D( tmpMid[1], mid ) ;
			}
		}
	} 
	else
	{
		//生成40度角退刀圆弧
		VEC3D tmpDir = { 0.0, 0.0, 0.0 } ;
		double dang = ANGLE_TO_RADIAN( 40 ) ;
		if( mathGetVecByPP2D( cen, end, MIN_LEN, tmpDir ) == ERUNSUC )
			return NULL ;
		PNT3D pivot = { 0.0, 0.0, 0.0 } ;
		VEC3D rot_dir = { 0.0, 0.0, 0.0 } ;
		nc_VectorCopy( pivot, cen, 2 ) ;
		mathRotVec( vz, pivot, -dang, tmpDir, rot_dir ) ;
		start[0] = cen[0] + r * rot_dir[0] ;
		start[1] = cen[1] + r * rot_dir[1] ;
		mathRotVec( vz, pivot, -dang/2, tmpDir, rot_dir ) ;
		mid[0] = cen[0] + r * rot_dir[0] ;
		mid[1] = cen[1] + r * rot_dir[1] ;
	}

	CSmartCurve *pCurve = new CSmartCurve() ;
	if( bHead )
	{
		// 第一个为直线
		CSmartLine *pLine = new CSmartLine( bgn, start ) ;
		pCurve->AddSect ( pLine ) ;

		// 生成圆弧
		CSmartArc *pArc = new CSmartArc() ;
		if( pArc->Define3PArc( start, mid, end, 0.01 ) )
		{
			VEC2D v1,v2 ;
			pCurve->m_pTail->GetTangent ( 1, v1 ) ;
			pArc->GetTangent ( 0, v2 ) ;
			double dTmpAng = mathGetAngle2DUnit( v1, v2 ) ;
			if( dTmpAng > PI1 * 0.1 )
			{
				delete pArc ;
				delete pCurve ;
				pCurve = NULL ;
			}
			else
			{
				pCurve->AddSect ( pArc ) ;
				pCurve->DefineBox () ;
			}
		}
		else
		{
			delete pArc ;
			delete pCurve ;
			pCurve = NULL ;
		}
	}
	else
	{
		CSmartArc *pArc = new CSmartArc() ;
		if( pArc->Define3PArc ( end, mid, start, 0.01 ) )
		{
			pCurve->AddSect ( pArc ) ;
			pCurve->DefineBox () ;
		}
		else
		{
			delete pCurve ;
			delete pArc ;
			pCurve = NULL ;
		}
	}
	return pCurve ;
}
void CSmtLoopPath::AddPlungeCurveForOpenCurve ( CSmtCheckMdl *CheckMdl, CSmtCPathLib &cPathLib, int  nLevel,  
												CSmartCurve *pCurve, JDNC_SETUP &cSetup, BOOL bIsDel, BOOL bZigZag )
{
	if( !pCurve ) return ;
	// 调整曲线的方向
	if( !bZigZag ) AdjustCloseCurveDir( pCurve, cSetup ) ;
    
	PNT2D start, end, seed, p1, p2 ;
	pCurve->GetStart ( start ), pCurve->GetEnd ( end ) ;
	// 得到环上距离该点最近的点
	double dMinDist = 1.0e12, dDist = 0. ;
	CSmartSect *pSect = NULL ;
	CSmartCurve *pHead = NULL ;
	BOOL bFind = FALSE ;
	for( int i = 0 ; i < 2 ; i++ )
	{
		dMinDist = 1.0e12 ;
		for( pHead = m_cCLib.m_pHead ; pHead ; pHead = pHead->next )
		{
			pSect = pHead->GetHead () ;
			for( ; pSect ; pSect = pSect->next )
			{
				if( i == 0 )
                    dDist = pSect->MinDistPoint ( start, seed ) ;
				else
					dDist = pSect->MinDistPoint ( end  , seed ) ;
				if( dDist > dMinDist ) continue ;
				bFind = TRUE ;
				dMinDist = dDist ;
				if( i == 0 ) mathCpyPnt2D( seed, p1 ) ;
				else		 mathCpyPnt2D( seed, p2 ) ;
			}
		}
	}
	
	CSmtCutPath *pLeadInn = NULL, *pLeadOut = NULL ;
	
	// 添加切出切入圆弧
	if( bFind )
	{
	//	if( !m_bRemain || m_bRemain && m_cFeed.m_cConnectDef.m_nLead3DType == NCDEF_LEAD3D_TAN )
		{
			pLeadInn = CreatePlungeArc( CheckMdl, pCurve, p1, TRUE, cSetup ) ;
			pLeadOut = CreatePlungeArc( CheckMdl, pCurve, p2, FALSE, cSetup ) ;
		}
	}
	CSmtCutPoint *pPnt = NULL ;
	if( pLeadInn )
	{
		pPnt = new CSmtCutPoint( start[0], start[1], m_dDepth ) ;
		pLeadInn->AddTail ( pPnt ) ;
		RedefineLeadPathHeight( CheckMdl, pLeadInn, TRUE ) ;
		cPathLib.AddToTail ( pLeadInn ) ;
	}
	// 添加曲线
	AddCurveToPathLib( pCurve, cPathLib, bIsDel, nLevel, cSetup.m_cTolDef ) ;
	if( pLeadOut )
	{
		pPnt = new CSmtCutPoint( end[0], end[1], m_dDepth ) ;
		pLeadOut->AddHead ( pPnt ) ;
		RedefineLeadPathHeight( CheckMdl, pLeadOut, FALSE ) ;
		cPathLib.AddToTail ( pLeadOut ) ;
	}
}
void CSmtLoopPath::AdjustCloseCurveDir ( CSmartCurve *pCurve, JDNC_SETUP &cSetup )
{
	double dArea = pCurve->CalcArea () ;

/*	if( m_pLoop->GetIsland () )
	{
		if( cSetup.m_cOrderDef.m_nMillDir == 1 && dArea > 0 )
			pCurve->Reverse() ;
		else if( cSetup.m_cOrderDef.m_nMillDir == 0 && dArea < 0 )
			pCurve->Reverse () ;
	}
	else
	{
		if( cSetup.m_cOrderDef.m_nMillDir == 1 && dArea < 0 )
			pCurve->Reverse() ;
		else if( cSetup.m_cOrderDef.m_nMillDir == 0 && dArea > 0 )
			pCurve->Reverse () ;
	}*/
	CSmartLoop *pIsland = m_pLoop->GetIsland() ;
	if( pIsland )
	{
		// 两种情况，一种是岛是零件边界，另外一种岛是毛胚边界
		if( pIsland->m_pCurve->IsBlankCurve() )
		{ // 岛是毛胚边界
			if( cSetup.m_cOrderDef.m_nMillDir == 1 && dArea < 0 )
			{// 顺铣，采用刨的方法进行加工(顺时针路径)，路径顺时针面积应该小于0，如果面积大于0,反向
				pCurve->Reverse() ;
			}
			else if( cSetup.m_cOrderDef.m_nMillDir == 0 && dArea > 0 )
			{//	逆铣，采用挖的方法进行加工(逆时针路径)，路径应该逆时针面积大于0，如果面积小于0，反向
				pCurve->Reverse () ;
			}
		}
		else
		{
			if( cSetup.m_cOrderDef.m_nMillDir == 1 && dArea > 0 )
			{// 顺铣，采用刨的方法进行加工(顺时针路径)，路径顺时针面积应该小于0，如果面积大于0,反向
				pCurve->Reverse() ;
			}
			else if( cSetup.m_cOrderDef.m_nMillDir == 0 && dArea < 0 )
			{//	逆铣，采用挖的方法进行加工(逆时针路径)，路径应该逆时针面积大于0，如果面积小于0，反向
				pCurve->Reverse () ;
			}
		}
	}
	else
	{	
		if( cSetup.m_cOrderDef.m_nMillDir == 1 && dArea > 0 )
		{// 顺铣，采用刨的方法进行加工(顺时针路径)，路径顺时针面积应该小于0，如果面积大于0,反向
			pCurve->Reverse() ;
		}
		else if( cSetup.m_cOrderDef.m_nMillDir == 0 && dArea < 0 )
		{//	逆铣，采用挖的方法进行加工(逆时针路径)，路径应该逆时针面积大于0，如果面积小于0，反向
			pCurve->Reverse () ;
		}
	}
}
BOOL CSmtLoopPath::LineIntAllCurve ( PNT2D p1, PNT2D p2 )
{
	int  i,  nSize = (int)m_cLibArr.GetSize (), nCnt = 0 ;
	if( nSize < 1 ) return FALSE ;

	CSmartCurveLib *CurveLib = NULL ;
	CSmartCurve *pHead = NULL ;
	BOOL bInt = FALSE ;
	CSmartLine *pLine = new CSmartLine( p1, p2 ) ;
	CSmartSect *pSect = NULL ;
	PNT2D intpt[5] ;
	BOX2D box1, box2 ;
	mathDefBox2D( p1, p2, &box1 ) ;
	for( i = nSize - 1 ; i >= 0 ; i-- )
	{
		CurveLib = m_cLibArr[i] ;
		if( CurveLib->m_nNumCurve < 1 )
			continue ;
		
		pHead = CurveLib->m_pHead ;
		for( ; pHead ; pHead = pHead->next )
		{
			// 包围盒粗判
			box2.min[0] = pHead->m_dBox[0][0] ;
			box2.min[1] = pHead->m_dBox[0][1] ;
			box2.max[0] = pHead->m_dBox[1][0] ;
			box2.max[1] = pHead->m_dBox[1][1] ;
			if( mathChkBox2DInt( &box1, &box2, 0.1 ) == IDNOINT )
				continue ;
			pSect = pHead->m_pHead ;
			for( ; pSect ; pSect = pSect->next )
			{
				nCnt =  Mini_SectSectInt( *pLine, *pSect, intpt ) ;
				if( nCnt > 0 )
				{
					bInt = TRUE ;
					break ;
				}
			}
			if( bInt ) break ;
		}
		if( bInt ) break ;
	}
	delete pLine ;
	return bInt ;
}
CSmtCutPath *CSmtLoopPath::CreatePlungeArc ( CSmtCheckMdl *DriveMdl, CSmartCurve *pCurve, PNT2D p, BOOL bHead, JDNC_SETUP& cSetup )
{
	VEC2D vec, rot_v[2], tmpvec ;
	PNT2D end ;
	VEC3D vCen, vTail ;
	PNT3D cen, pnt  ;
	// 获得曲线头部或者尾部的切矢
	if( bHead ) 
	{
		pCurve->GetHead()->GetTangent ( 0, vec ) ;
		pCurve->GetStart ( end ) ;
	}
	else
	{
		pCurve->GetTail()->GetTangent ( 1, vec ) ;
		pCurve->GetEnd   ( end ) ;
	}
	// 计算法失
	rot_v[0][0] = -vec[1], rot_v[0][1] =  vec[0] ; // 90度
	rot_v[1][0] =  vec[1], rot_v[1][1] = -vec[0] ; // 270
	if( mathGetVecByPP2D( end, p, MIN_LEN, tmpvec ) != ERSUCSS )
		return NULL ;
	double dAng[2] = { 0., 0. } ;
	dAng[0] = mathGetAngle2DUnit( tmpvec, rot_v[0] ) ;
	dAng[1] = mathGetAngle2DUnit( tmpvec, rot_v[1] ) ;
	if( dAng[0] > dAng[1] )
	{
		vCen[0] = rot_v[1][0], vCen[1] = rot_v[1][1] ;
	}
	else
	{
		vCen[0] = rot_v[0][0], vCen[1] = rot_v[0][1] ;
	}
	vCen[2] = 0. ;
	for( int i = 0 ; i < 2 ; i++ )
	{
		cen[i] = end[i] + m_dRadius * vCen[i] ;
	}
	cen[2] = m_dDepth, vTail[0] = vec[0], vTail[1] = vec[1], vTail[2] = 0. ;
	pnt[0] = end[0], pnt[1] = end[1], pnt[2] = m_dDepth ;
	CLinkArc cLink( cSetup, m_cFeed, m_dRadius ) ;
	CSmtCutPath *pPath = NULL ;
	if( bHead )
	{
        pPath = cLink.AddRemainLeadInn( DriveMdl, vCen, cen, vTail, pnt ) ;
	}
	else
	{
		pPath = cLink.AddRemainLeadOut( DriveMdl, vCen, cen, vTail, pnt ) ;
	}
	return pPath ;
}
BOOL CSmtLoopPath::GetSmtCPathLibEndPoint( CSmtCPathLib &tmpLib, PNT3D p, BOOL bHead )
{
	if( tmpLib.m_cAllPath.GetCount () < 1 ) return FALSE ;
	CSmtCutPath *pPath = NULL ;
	int i = 0 ;
	if( bHead )
	{
		pPath = tmpLib.m_cAllPath.GetHead () ;
		for( i = 0 ; i < 3 ; i++ )
			p[i] = pPath->m_pHead->m_fPoint[i] ;
	}
	else
	{
		pPath = tmpLib.m_cAllPath.GetTail () ;
		for( i = 0 ; i < 3 ; i++ )
			p[i] = pPath->m_pTail->m_fPoint[i] ;
	}
	return TRUE ;
}
CSmtCutPath *CSmtLoopPath::GetPlungeLine ( CSmtCheckMdl* CheckMdl, PNT2D start, PNT2D end, JDNC_TOL& cTol ) 
{
	PNT4D dStart, dEnd ;
	dStart[0] = start[0], dStart[1] = start[1] ;
	dEnd[0]	  = end[0]  , dEnd[1]   = end[1]   ;
	dStart[2] = dEnd[2] = m_dDepth ;
	dStart[3] = dEnd[3] = 0. ;

	CSmtCutPath *pPath = new CSmtCutPath( MINI_CONNECT_PATH ) ;
	pPath->AddPoint ( dStart );
    pPath->AddPoint ( dEnd ) ;
	pPath->InsertCPoint ( cTol.m_dMaxStep ) ;
	pPath->VerifyLinePath ( *CheckMdl, cTol, m_cPrgDef ) ;
	
	pPath->DeletePoint ( pPath->m_pTail ) ;
	CSmtCutPoint *pTail = pPath->m_pTail ;
	int nCount = 0 ;
	for( ; pTail ; pTail = pTail->prev )
	{
		if( pTail->m_fPoint[2] > m_dDepth + cTol.m_dArcTol )
		{
			if( nCount == 0 )
			{
				delete pPath ;
				return NULL ;
			}
			else 
			{
				start[0] = pTail->m_fPoint[0], start[1] = pTail->m_fPoint[1] ;
				break ;
			}
		}
		nCount++ ;
	}
	return pPath ;
}
BOOL CSmtLoopPath::AddPlungeForClosedCurve ( CSmtCPathLib &cPathLib, CSmartCurve *pCurve, CSmartLoop *pCurr, JDNC_PLUNGE &cPlunge )
{
	CSmtCPathLib AllPath ;
	PNT3D pt ;
	pCurve->GetStart ( pt ) ;
	pt[2] = m_dDepth ;
	// 获得下刀路径
	MathCAM_AddPlungePathAtPoint( AllPath, cPlunge, pt, pCurve, pCurr ) ;
	BOOL bAdd = FALSE ;
	CSmtCutPath *pPath = NULL ;

	POSITION pos, atpos ;
	pos = AllPath.m_cAllPath.GetHeadPosition () ;
	while( pos )
	{
		atpos = pos ;
		pPath = AllPath.m_cAllPath.GetNext ( pos ) ;
		cPathLib.AddToTail ( pPath ) ;
		AllPath.m_cAllPath.RemoveAt ( atpos ) ;
		bAdd = TRUE ;
	}
	AllPath.ClearAllPath () ;
	return bAdd ;
}
void CSmtLoopPath::AddPathToGroup ( CSmtCheckMdl *DriveMdl, CPathGroup &NewPath )
{
    DriveMdl ;
	if( m_cPathLib.m_cAllPath.GetCount () < 1 ) return ;

	CPathCombine *PComb = new CPathCombine( NC_WPROCESS_ROUGH ) ;
	m_cPathLib.AddToPathCombine ( *PComb ) ;
	NewPath.AddData ( 0, PComb ) ;
	m_cPathLib.ClearAllPath () ;
	/*
	// 添加切出切入圆弧
	if( bFind )
	{
		pLeadInn = CreatePlungeArc( CheckMdl, pCurve, p1, TRUE, cSetup ) ;
		pLeadOut = CreatePlungeArc( CheckMdl, pCurve, p2, FALSE, cSetup ) ;
	}
	CSmtCutPoint *pPnt = NULL ;
	if( pLeadInn )
	{
		pPnt = new CSmtCutPoint( start[0], start[1], m_dDepth ) ;
		pLeadInn->AddTail ( pPnt ) ;
		cPathLib.AddToTail ( pLeadInn ) ;
	}
	// 添加曲线
	AddCurveToPathLib( pCurve, cPathLib, bIsDel, nLevel, cSetup.m_cTolDef ) ;
	if( pLeadOut )
	{
		pPnt = new CSmtCutPoint( end[0], end[1], m_dDepth ) ;
		pLeadOut->AddHead ( pPnt ) ;
		cPathLib.AddToTail ( pLeadOut ) ;
	}
	*/
}

BOOL CSmtLoopPath::ConnectAllCurve_ArcLink ( CSmtCheckMdl *CheckMdl, CSmartLoop *pPlunge, CSmartCurveLib &BndCurve, 
											BOOL bZigZag, BOOL bPrev, PNT3D prev, JDNC_SETUP &cSetup, double dDepth )
{
	if( m_cPathLib.m_cAllPath.GetCount () > 0 ) 
		return TRUE ;
	int  i,  nSize = (int)m_cLibArr.GetSize () ;

	if( nSize < 1 ) return TRUE ;
	CSmartCurve *pCurve = NULL ;
	m_cCLib = BndCurve ;
	m_pPlunge = pPlunge ;
	BOOL bAllCurve = TRUE, bFst = FALSE ;
	BOOL  bDefPrev = bPrev, bAddClose = FALSE, bLeadOut = FALSE ;	PNT3D dSeedPnt = { 0.0, 0.0, 0.0 } ;
	if( bDefPrev ) nc_VectorCopy( dSeedPnt, prev , 3 ) ;
	// 从相同的曾数之间进行
	while( 1 )
	{
		bFst = TRUE, bAllCurve = TRUE, bAddClose = FALSE ;
		for( i = nSize - 1 ; i >= 0 ; i-- )
		{
			if( m_cLibArr[i]->m_nNumCurve < 1 )
				continue ;

			if( bFst )
			{
				if( bDefPrev)
				{
					pCurve = FindFstCurve( *m_cLibArr[i], dSeedPnt, bZigZag ) ;
				}
				else
				{
					pCurve = m_cLibArr[i]->m_pHead ;
					m_cLibArr[i]->RemoveCurve ( pCurve ) ;
				}

				// 添加退刀
				if( nSize == 1 && m_cLibArr[i]->m_nNumCurve == 0 && pCurve->IsClosed () )
				{
					bLeadOut = TRUE ;
				}
				else
				{
					bLeadOut = FALSE ;
				}
				// 添加进刀
				AddPlungeCurve( CheckMdl, m_cPathLib, i, m_pPlunge, pCurve, cSetup, m_cPlunge, bAddClose, FALSE, bFst, bZigZag, bLeadOut ) ;
			}
			else
			{
				pCurve = FindNearCurve_ArcLink( CheckMdl, m_cPathLib, m_pPlunge, m_cLibArr[i], pCurve,
					i, bZigZag, cSetup, m_cPlunge, dDepth, bAddClose ) ;
			}
			bFst = FALSE ;
			bAllCurve = FALSE ;
			if( m_cPathLib.m_cAllPath.GetCount() )
			{
				CSmtCutPath *pTPath = m_cPathLib.m_cAllPath.GetTail() ;
				nc_FloatToDouble( dSeedPnt, pTPath->m_pTail->m_fPoint, 3 ) ;
				bDefPrev = TRUE ;
			}
		}
		if( pCurve )
		{ 
			delete pCurve ;
			pCurve = NULL ;
		}
		if( bAllCurve ) break ;
	}
	// 删除曲线
	RemoveLibArr() ;
	m_cCLib.RemoveAll () ;
	return TRUE ;
}

CSmartCurve* CSmtLoopPath::FindNearCurve_ArcLink ( CSmtCheckMdl *CheckMdl, CSmtCPathLib &cPathLib, CSmartLoop *pPlunge, CSmartCurveLib* pLib, CSmartCurve *&pPrev,
												  int  i, BOOL bZigZag, JDNC_SETUP &cSetup, JDNC_PLUNGE &cPlunge, double dDepth, BOOL &bAddClose )
{
	UNUSED_ALWAYS( dDepth ) ;
	if( pLib->m_nNumCurve < 1 )
	{
		delete pPrev ;
		pPrev = NULL ;
		return NULL ;
	}
	CSmartCurve *pHead = pLib->m_pHead, *pObj = NULL ;
	CSmtCutPath *pPath  = NULL ;
	// 如果空,添加第一条
	if( cPathLib.m_cAllPath.GetCount () < 1 )
	{
		pLib->RemoveCurve ( pHead ) ;
		pPath = TransfCurveToPath( pHead, cSetup.m_cTolDef , TRUE ) ;
		cPathLib.AddToTail ( pPath ) ;
	}
	PNT3D start, end ;
	BOOL bFind = FALSE ;
	// 取得当前路经(非进退刀\非连刀)最后一点
	POSITION pos,atpos ;
	pos = cPathLib.m_cAllPath.GetTailPosition () ;
	while( pos )
	{
		pPath = cPathLib.m_cAllPath.GetPrev ( pos ) ;
		if( pPath->GetCutMode () == MINI_MILL_PATH )
		{
			GetSmtPathEndPnt( pPath, 1, start ) ;
			bFind = TRUE ;
			break ;
		}
	}
	if( !bFind ) 
	{
		delete pPrev ;
		pPrev = NULL ;
		return NULL;
	}
	// 寻找最近曲线(加上进退刀就不准了)
	PNT3D p1, p2 ;
	PNT2D nearpt ;
	CSmartSect *pFind = NULL ;
	for( pHead = pLib->m_pHead ; pHead ; pHead = pHead->next )
	{
		if( IsNextCurve_ArcLink( pHead, pPrev, start, i, pFind , nearpt, cSetup.m_cTolDef.m_dArcTol ) )
		{
			pObj = pHead ;
			break ;
		}
	}
	if( !pObj )
	{
		delete pPrev ;
		pPrev = NULL ;
		return NULL;
	}
	pLib->RemoveCurve ( pObj ) ;
	BOOL bClose = pObj->IsClosed () ;
	// 得到当前连刀或者进退刀的最后点
	pPath = cPathLib.m_cAllPath.GetTail () ;
	GetSmtPathEndPnt( pPath, 1, start ) ;

	//	double dLink = m_cFeed.m_cConnectDef.m_dMaxDist  ;
	//	if( !m_bRemain ) dLink = CheckMdl->m_pTool->m_fRadius * 4 ;
	double dLink = CheckMdl->m_pTool->m_fRadius * 2 , dLen = 1.0e12 ;
	//计算两曲线的距离
	double dDist = 0.0, dPYDist = 0.0 ;
	if ( pObj && pPrev )
	{
		dDist = MinDistBetweenCurve( pObj, pPrev ) ;
		dPYDist = min( dDist*0.8, 1.0 ) ;
	}
	// 为pObj添加切入切出路径
	CSmtCPathLib tmpLib ;
	if( bClose )
	{// 如果闭合,判断是否需要添加下刀路径
		if( pFind ) 
		{
			pObj->SetStartPoint ( pFind, nearpt ) ;
		}
		pObj->GetStart ( p1 ) ;
		p1[2] = m_dDepth ;
		BOOL bAdd = FALSE ;
		dLen = mathDist( start, p1 ) ;
		if( dLen> dLink )
		{
			AddPlungeCurve( CheckMdl, tmpLib, i, pPlunge, pObj, cSetup, cPlunge, bAdd, FALSE, FALSE, bZigZag ) ;
			bAddClose = bAdd ;
		}
		else
		{
			AdjustCloseCurveDir( pObj, cSetup ) ;
			AdjustCurveDir( cPathLib, pPrev, pObj ) ;
			ResetCurveStart( pObj, dPYDist ) ;
			AddCurveToPathLib( pObj, tmpLib, FALSE, i, cSetup.m_cTolDef ) ;
		}
		GetSmtCPathLibEndPoint( tmpLib, end, TRUE ) ;
	}
	else
	{
		AddPlungeCurve( CheckMdl, tmpLib, i, pPlunge, pObj, cSetup, cPlunge, bAddClose, FALSE, FALSE, bZigZag ) ;
		if( !bZigZag )
		{
			GetSmtCPathLibEndPoint( tmpLib, end, TRUE ) ;
			dLen = mathDist( start, end ) ;
		}
		else
		{
			GetSmtCPathLibEndPoint( tmpLib, p1, TRUE ) ;
			GetSmtCPathLibEndPoint( tmpLib, p2, FALSE ) ;
			double l1 = mathDist( start, p1 ) ;
			double l2 = mathDist( start, p2 ) ;
			if( l1 > l2 )
			{
				dLen = l2 ;
				mathCpyPnt( p2, end ) ;
				if( !bClose )
					ReverseAllPath( tmpLib ) ;
			}
			else
			{
				dLen = l1 ;
				mathCpyPnt( p1, end ) ;
			}
		}
	}
	if( dLen < dLink )
	{
		BOOL bLine = FALSE ;
		if ( bClose )
		{
			CSmartCurve *LeadCurve =  CreateConnectCurve( pObj, pPrev, dDist ) ;
			CSmtCPathLib LeadLib ;
			TransfSmartCurveToCPath( LeadCurve, LeadLib, m_dDepth, cSetup.m_cTolDef ) ;
			if( LeadCurve ) delete LeadCurve ;
			// 判断曲线是否合格
			if( LeadLib.GetNumPath() && IsLeadArcPath( CheckMdl, LeadLib ) )
			{
				POSITION pos = LeadLib.m_cAllPath.GetHeadPosition() ;
				while ( pos )
				{
					CSmtCutPath * pPath = LeadLib.m_cAllPath.GetNext( pos ) ;
					pPath->m_bFeedType = JDNC_FEEDTYPE_CONNECT ;
				}
				AddTmpPathToAllPath( cPathLib, LeadLib ) ;
			}
			else
			{
				LeadLib.ClearAllPath () ;
				//用样条连接
				PNT3D pnt[2], dir[2] ;
				pnt[0][2] = pnt[1][2] = m_dDepth ;
				dir[0][2] = dir[1][2] = 0.0 ;
				pPrev->m_pTail->GetEnd( pnt[0] ) ;
				pObj->m_pHead->GetStart(pnt[1] ) ;
				pPrev->m_pTail->GetTangent( 1.0, dir[0] ) ;
				pObj->m_pHead->GetTangent( 0.0, dir[1] ) ;
				mathRevVec( dir[1] ) ;
				pPath = CreateAdsobSpline( CheckMdl, pnt, dir, cSetup.m_cTolDef ) ;
				if ( pPath )
				{
					cPathLib.AddToTail( pPath ) ;
				}
				else 
				{
					bAddClose = FALSE ;
					bLine = TRUE ;
				}
			}
		}
		if( !bClose || bLine )
		{
			pPath = CreateAdsorbLine( CheckMdl, start, end, cSetup.m_cTolDef ) ;
			if( pPath )
			{
				cPathLib.AddToTail ( pPath ) ;
			}
			else
				bAddClose = FALSE ;
		}
	}
	else
		bAddClose = FALSE ;
	pos = tmpLib.m_cAllPath.GetHeadPosition () ;
	while( pos )
	{
		atpos = pos ;
		pPath = tmpLib.m_cAllPath.GetNext ( pos ) ;
		cPathLib.AddToTail ( pPath ) ;
		tmpLib.m_cAllPath.RemoveAt ( atpos ) ;
	}
	tmpLib.m_cAllPath.RemoveAll () ;

	if ( bClose && pPrev && i == 0 && pLib->m_nNumCurve == 0 )
	{// 为闭合曲线添加退出圆弧
		CPointList allPnt ;
		PNT3D point = { 0, 0, 0 } ;
		if( pPrev->IsClosed() )
		{
			pPrev->GetPointByLength( dPYDist*2, point) ; 
		}
		else
		{
			pPrev->GetPoint( 1.0, point ) ;
		}
		allPnt.AddTail( point ) ;
		if ( AddArcForCloseCurve( CheckMdl, tmpLib, pObj, allPnt, cSetup, FALSE ) )
		{
			if ( tmpLib.GetNumPath() == 1 )
			{
				RedefineLastLeadoutPathH( CheckMdl, tmpLib.m_cAllPath.GetHead() ) ;
			}
			AddTmpPathToAllPath( cPathLib, tmpLib ) ;
		} 
		else
		{//生成样条
			tmpLib.ClearAllPath() ;
			pPath = GetLeadoutSplinePath( CheckMdl, cSetup, pObj, allPnt ) ;
			if ( pPath )
			{
				pPath->m_bFeedType = JDNC_FEEDTYPE_LEAD ;
				RedefineLastLeadoutPathH( CheckMdl, pPath ) ;
				cPathLib.AddToTail( pPath ) ;
			}
		}
	}
	if( pPrev )
	{
		delete pPrev ;
		pPrev = NULL ;
	}
	return pObj ;
}

BOOL CSmtLoopPath::IsNextCurve_ArcLink( CSmartCurve * pCurve, CSmartCurve *pPrev, PNT3D p, int  i, CSmartSect *&pNearSect, PNT2D nearpt, double dTol )
{
	if( !pCurve ) return FALSE ;
	// 判断点p 到直线pCurve的距离是否是m_dStep 
	PNT2D start, mid ;
	start[0] = p[0], start[1] = p[1] ;

	// 然后利用点到线的距离进行判断
	double dist = GetPntToCurveDist( start, pCurve, pNearSect, nearpt ) ;
	if ( pPrev )
	{
		PNT2D tmpP, tmpnearpt ;
		CSmartSect *ptmpSect ;
		pCurve->GetMidPoint( tmpP ) ;
		double tmpDist = GetPntToCurveDist( tmpP, pPrev, ptmpSect, tmpnearpt ) ;
		dist = min( tmpDist, dist ) ;
	}
	double dStep = m_dStep ;
	if( m_bBoundTol && i == 0 ) dStep = m_dBoundTol ;

	if( fabs( dist - dStep ) > dTol * 1.5 ) return FALSE ;

	// 还要判断上面一层中的曲线到这条线的距离都比较远
	CSmartCurveLib* pLib = m_cLibArr[i+1] ;
	if( pLib->m_nNumCurve < 1 )
		return TRUE ;
	CSmartSect *pSect = NULL ;
	PNT2D tmp ;
	CSmartCurve *pHead = pLib->m_pHead ;
	CSmartSect *pTmpSect = NULL ;
	for( ; pHead ; pHead = pHead->next )
	{
		// 如果上层中有曲线到这条的距离比半径小，返回FASLE
		pSect = pHead->m_pHead ;
		for( ; pSect ; pSect = pSect->next )
		{
			pSect->GetStart ( start ) ;
			pSect->GetPoint ( 0.5, mid ) ;
			dist = GetPntToCurveDist( start, pCurve, pTmpSect, tmp ) ;
			if( fabs( dist - m_dStep ) < 0.005 )
				return FALSE ;
			dist = GetPntToCurveDist( mid,   pCurve, pTmpSect, tmp ) ;
			if( fabs( dist - m_dStep ) < 0.005 )
				return FALSE ;
		}
	}

	return TRUE ;
}

CSmtCutPath * CSmtLoopPath::CreateAdsobSpline(CSmtCheckMdl *DriveMdl, PNT3D IntPnt[2], PNT3D TanDir[2], JDNC_TOL &cTol )
{
	CSmtCutPath *pSpline = NULL ;
	FPNT3D fIntPnt[100], fTandir[2] ;
	int i = 0 ;
	for( i = 0 ; i < 3 ; i++ )
	{
		fIntPnt[0][i] = TFLOAT( IntPnt[0][i] ) ;
		fIntPnt[1][i] = TFLOAT( IntPnt[1][i] ) ;
		fTandir[0][i] = TFLOAT( TanDir[0][i] ) ;
		fTandir[1][i] = TFLOAT( TanDir[1][i] ) ;
	}
	double dDepth = max( IntPnt[0][2], IntPnt[1][2] ) + 0.1 ;
	// 吸附
	int nCnt = DriveMdl->SmoothConnectLine( cTol, fIntPnt, 100, fTandir ) ;
	BOOL bConnect = TRUE ;
	// 过滤得到的点
	for( i = 0 ; i < nCnt ; i++ )
	{
		if( fIntPnt[i][2] > dDepth )
		{
			bConnect = FALSE ;	
			break ;
		}
	}
	if( nCnt >= 2 && bConnect )
	{
		pSpline = new CSmtCutPath( MINI_CONNECT_PATH ) ;
		pSpline->m_bFeedType = JDNC_FEEDTYPE_CONNECT ;
		for( i = 0 ; i < nCnt ; i++ )
			pSpline->AddPoint ( fIntPnt[i] ) ;
	}
	return pSpline ;
}

CSmtCutPath * CSmtLoopPath::GetLeadoutSplinePath( CSmtCheckMdl *CheckMdl, JDNC_SETUP & cSetup, CSmartCurve * pCurve, CPointList &allPnt ) 
{
	if( !pCurve || allPnt.GetNum() ==0 ) return NULL ;

	CSmtCutPath * pPath = NULL ;
	PNT3D bgn, pos = { 0., 0., 0. }, tang = { 0., 0., 0. }, nor = { 0., 0., 0. } ;
	pCurve->m_pTail->GetEnd ( pos ) ;
	pCurve->m_pTail->GetTangent ( 1, tang ) ;
	pCurve->m_pTail->GetNormal ( 1, nor ) ;
	if( allPnt.m_pHead != NULL )
	{//判断法向是否应该反向
		VEC3D vDir = { 0.0, 0.0, 0.0 } ;
		nc_VectorCopy( bgn, allPnt.m_pHead->m_dPoint, 2 ) ;
		bgn[2] = 0.0 ;
		mathGetVecByPP2D( pos, bgn, MIN_LEN, vDir )  ;
		if ( mathGetAngleUnit( nor, vDir ) > PI1_2 )
		{
			mathRevVec( nor ) ;
		}
	}
	double dDist = mathDist2D( bgn, pos ), r = dDist*0.4 ;
	int i = 0 ;
	PNT2D cen, start, end ;
	VEC3D vz = { 0., 0., 0. } ; 
	if( mathVProductUnit( nor, tang, vz ) == ERUNSUC )
		return NULL ;
	// 生成圆弧信息cen, start, end, mid
	mathCpyPnt2D( pos, end ) ;
	for( i = 0 ; i < 2 ; i++ )
	{
		cen[i] = pos[i] + nor[i] * r ;
	}
	//生成40度角退刀圆弧
	VEC3D tmpDir = { 0.0, 0.0, 0.0 } ;
	if( mathGetVecByPP2D( cen, end, MIN_LEN, tmpDir ) == ERUNSUC )
		return NULL ;
	PNT3D pivot = { 0.0, 0.0, 0.0 } ;
	VEC3D rot_dir = { 0.0, 0.0, 0.0 } ;
	nc_VectorCopy( pivot, cen, 2 ) ;
	mathRotVec( vz, pivot, ANGLE_TO_RADIAN( -40 ), tmpDir, rot_dir ) ;
	start[0] = cen[0] + r * rot_dir[0] ;
	start[1] = cen[1] + r * rot_dir[1] ;
	mathRotVec( vz, pivot, ANGLE_TO_RADIAN( 50 ), tmpDir, rot_dir ) ;

	//得到样条起末点位置和切矢生成样条
	PNT3D pnt[2], dir[2] ;
	pnt[0][2] = pnt[1][2] = m_dDepth ;
	dir[0][2] = dir[1][2] = 0.0 ;
	nc_VectorCopy( pnt[0], end, 2 ) ;
	nc_VectorCopy( pnt[1], start, 2 ) ;
	pnt[0][2] = pnt[1][2] = m_dDepth ;
	nc_VectorCopy( dir[0], tang, 3 ) ;
	nc_VectorCopy( dir[1], rot_dir, 3 ) ;
	pPath = CreateAdsobSpline( CheckMdl, pnt, dir, cSetup.m_cTolDef ) ;

	return pPath ;
}

//重新定义进退刀路径高度避免在模具平面处留下划痕
BOOL CSmtLoopPath::RedefineLeadPathHeight(CSmtCheckMdl *Drivemdl, CSmtCutPath * pPath, BOOL bLeadIn )
{
	if( !Drivemdl || !pPath || 
		pPath->m_bFeedType != JDNC_FEEDTYPE_LEAD )
	{
		return FALSE ;
	}
	FPNT3D fTemp, fBox[2] ;
	if ( bLeadIn )
	{
		mathFCpyPnt( pPath->m_pHead->m_fPoint, fTemp ) ;
	} 
	else
	{
		mathFCpyPnt( pPath->m_pTail->m_fPoint, fTemp ) ;
	}
	mathFDefBox3D( fTemp, fTemp, fBox, 0.1 ) ;
	fBox[1][2] = 1.0e6f ;
	Drivemdl->LabelCheckByBox( fBox ) ;
	fTemp[2] -= 0.1f ;
	Drivemdl->DefineHeight( fTemp ) ;
	double dMove = 0.0;
	if ( bLeadIn )
	{
		dMove = min( 0.1, 0.1 - ( pPath->m_pHead->m_fPoint[2] - fTemp[2] )) ;
	} 
	else
	{
		dMove = min( 0.1, 0.1 - ( pPath->m_pTail->m_fPoint[2] - fTemp[2] )) ;
	}
	if( dMove > 0.001 )
	{
		pPath->NormalizeLen() ;
		CSmtCutPoint *pHead = pPath->m_pHead ;
		for( ; pHead ; pHead = pHead->next )
		{
			if ( bLeadIn )
			{
				pHead->m_fPoint[2] += float( ( 1 - pHead->m_fPoint[3] ) * dMove ) ;
			} 
			else
			{
				pHead->m_fPoint[2] += float( pHead->m_fPoint[3]  * dMove ) ;
			}
		}
	}
	return TRUE ;
}

// 参照区域环切功能,把最后一条路径的退刀路径整体抬高0.05
BOOL CSmtLoopPath::RedefineLastLeadoutPathH(CSmtCheckMdl *Drivemdl, CSmtCutPath * pPath)
{
	if( !Drivemdl || !pPath || 
		pPath->m_bFeedType != JDNC_FEEDTYPE_LEAD )
	{
		return FALSE ;
	}
	FPNT3D fTemp, fBox[2] ;
	mathFCpyPnt( pPath->m_pTail->m_fPoint, fTemp ) ;
	mathFDefBox3D( fTemp, fTemp, fBox, 0.1 ) ;
	fBox[1][2] = 1.0e6f ;
	Drivemdl->LabelCheckByBox( fBox ) ;
	fTemp[2] -= 0.1f ;
	Drivemdl->DefineHeight( fTemp ) ;
	double dMove = 0.0;
	dMove = min( 0.1, 0.1 - ( pPath->m_pTail->m_fPoint[2] - fTemp[2] )) ;
	dMove = max( dMove, 0.05 ) ;
	if( dMove > 0.001 )
	{
		pPath->NormalizeLen() ;
		CSmtCutPoint *pHead = pPath->m_pHead ;
		for( ; pHead ; pHead = pHead->next )
		{
			pHead->m_fPoint[2] += float( pHead->m_fPoint[3]  * dMove ) ;
		}
	}
	return TRUE ;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////
void MathCam_GetLPathBox ( CSmtLoopPath* LPathHead, BOX3D* box )
{
	if( !LPathHead->m_pLoop ) return ;
	CSmtLoopPath * pLPath = LPathHead ;
	mathIniBox3D( pLPath->m_pLoop->m_dBox[0], pLPath->m_pLoop->m_dBox[1], box ) ;
	
	for( pLPath = LPathHead ; pLPath ; pLPath = pLPath->next )
	{
		mathEnlargeBox3D( pLPath->m_pLoop->m_dBox[0], box ) ;
		mathEnlargeBox3D( pLPath->m_pLoop->m_dBox[1], box ) ;
	}
}
CSmtLoopPath *MathCam_AddLPath( CSmtLoopPath *LPathHead, CSmtLoopPath *LPathNew )
{
	if( !LPathNew ) return LPathHead ;
	if( !LPathHead) return LPathNew  ;
	CSmtLoopPath *pLPath = LPathHead ;
	while( pLPath && pLPath->next )
		pLPath = pLPath->next ;
	pLPath->next = LPathNew ;
	LPathNew->prev = pLPath ;
	return LPathHead ;
}
void MathCam_DeleteLPath( CSmtLoopPath *&LPathHead )
{
	CSmtLoopPath *pLPath = NULL ;
	while( LPathHead )
	{
		pLPath = LPathHead->next ;
		delete LPathHead ;
		LPathHead = pLPath ; 
	}
}
CSmtLoopPath *MathCam_RemoveLPath( CSmtLoopPath *LPathHead, CSmtLoopPath *pObj )
{
	if( !LPathHead || !pObj ) return LPathHead ; 
    if( LPathHead == pObj )
    {
       LPathHead = LPathHead->next ; 
	   if( LPathHead ) LPathHead->prev = NULL ;
    }
    else    
    {
        if( pObj->next )
            pObj->next->prev = pObj->prev;
        pObj->prev->next = pObj->next;    
    }
	pObj->next = pObj->prev = NULL ; 
	return LPathHead ; 
}
int GetLPathCount( CSmtLoopPath *LPathHead )
{
	if( !LPathHead ) return 0 ;
	int nCount = 0 ;
	CSmtLoopPath *pLPath = LPathHead ;
	for( ; pLPath ; pLPath = pLPath->next )
	{
		nCount++ ;
	}
	return nCount ;
}

void ReverseAllLPath( CSmtLoopPath *&LPathHead ) 
{
	if( !LPathHead ) return ;

	CSmtLoopPath *pTail = LPathHead, *pPrev = NULL, *pOrder = NULL ;
	while( pTail && pTail->next ) pTail = pTail->next ;
	while( pTail )
	{
		pPrev = pTail->prev ;
		pTail->next = pTail->prev = NULL ;
		pOrder = MathCam_AddLPath( pOrder, pTail ) ;
		pTail = pPrev ;
	}

	LPathHead = pOrder ;
}
CSmtLoopPath *SortLPathByRemainLoop( CSmtLoopPath *LPathHead, double dSpaceTol, double dSafeDist )
{
	CSmartLoop *pHead = NULL, *pLoop = NULL ;
	CSmtLoopPath *pLPath = LPathHead, *pFind = NULL ;
	for( ; pLPath ; pLPath = pLPath->next )
	{
		pLoop = pLPath->m_pLoop ;
		pHead = Mini_AddContours( pHead, pLoop ) ;
	}

	//重新赋行号
	INT nIndex = 0 , LineNum[5000], *pLineNum = NULL, nSize = 0 ;
	pLoop = pHead ;
	while ( pLoop )
	{//统计层数
		if( pLoop == pHead )
		{
			nSize ++ ;
		}
		else if( pLoop->m_nDepth != pLoop->prev->m_nDepth )
		{	
			nSize ++ ;
		}
		pLoop = pLoop->next ;
	}
	BOOL bNewFlag = FALSE ;
	if( nSize > 5000)
	{//避免数组溢出
		pLineNum = new INT[nSize] ;
		bNewFlag = TRUE ;
	}
	else
	{
		pLineNum = LineNum ;
	}
	pLoop = pHead ;
	while ( pLoop )
	{
		if ( pLoop == pHead )
		{
			pLineNum[nIndex] = pLoop->m_nDepth ;
			pLoop->m_nDepth = nIndex ;
		} 
		else
		{
			if( pLineNum[nIndex] == pLoop->m_nDepth )
			{
				pLoop->m_nDepth = nIndex ;
			}
			else
			{
				nIndex ++ ;
				pLineNum[nIndex] = pLoop->m_nDepth ;
				pLoop->m_nDepth = nIndex ;
			}
		}
		pLoop = pLoop->next ;
	}

	// 将环排序
	pHead = Mini_SortSRemainContours( pHead,  dSpaceTol, dSafeDist ) ;

	//恢复行号
	pLoop = pHead ;
	while ( pLoop )
	{
		pLoop->m_nDepth = pLineNum[pLoop->m_nDepth] ;
		pLoop = pLoop->next ;
	}
	//删除内存
	if( bNewFlag && pLineNum )
	{
		delete[] pLineNum ;
		pLineNum = NULL ;
	}
	if( !pHead ) return LPathHead ;
	// 按照pHead的顺序重新组环
	CSmartLoop *pNext = NULL ;	
	CSmtLoopPath *LPathNew = NULL ;
	while( pHead )
	{
		pNext = pHead->next ;
		pHead->next = pHead->prev = NULL ;
		
		pFind = NULL ;
		pLPath = LPathHead ;
		for( ; pLPath ; pLPath = pLPath->next )
		{
			if( pHead == pLPath->m_pLoop )
			{
				pFind = pLPath ;
				break ;
			}
		}
		if( pFind )
		{
			LPathHead = MathCam_RemoveLPath( LPathHead, pFind ) ;
			LPathNew = MathCam_AddLPath( LPathNew, pFind ) ;
		}
		pHead = pNext ;
	}
	if( LPathHead )
	{
		LPathNew = MathCam_AddLPath( LPathNew, LPathHead ) ;
	}
	return LPathNew ;
}

/////////////////////////////////////////////////////////////////////////////
// 以下函数为粗加工和残料补加工公用
CSmartLoop *MathCam_CreateBigLoop ( CSmartLoop **pDriveLoop, CSmartLoop *AllLoop, int  nCount, double r ) 

{
	CSmartLoop *pBigLoop = NULL ;
	// 首先得到加工环的最大包围盒
	BOX3D box, maxBox ;
	mathClnBox3D( &maxBox ) ;
	for( int  i = 0 ; i < nCount ; i++ )
	{
		if( pDriveLoop[i] )
            MathCam_GetLoopBox( pDriveLoop[i], &box ) ;
		else
			MathCam_GetLoopBox( AllLoop,      &box ) ;
        mathCalBox3DUnion( &box, &maxBox, &maxBox ) ;
	}
	mathExpandBox3D( r, &maxBox ) ;
	pBigLoop = new CSmartLoop() ;
	pBigLoop->CreateLoop ( maxBox.min, maxBox.max ) ;
	return pBigLoop ;
}
CSmartLoop *MathCam_CreateBigLoopAndBox( CSmartLoop **pDriveLoop, CSmartLoop *AllLoop, BOX3D &dMaxBox, int nCount, double r ) 
{
	CSmartLoop *pBigLoop = NULL ;
	// 首先得到加工环的最大包围盒
	BOX3D box ;
	for( int i = 0 ; i < nCount ; i++ )
	{
		if( pDriveLoop[i] )
            MathCam_GetLoopBox( pDriveLoop[i], &box ) ;
		else
			MathCam_GetLoopBox( AllLoop,      &box ) ;
        mathCalBox3DUnion( &box, &dMaxBox, &dMaxBox ) ;
	}
	mathExpandBox3D( r, &dMaxBox ) ;
	pBigLoop = new CSmartLoop() ;
	pBigLoop->CreateLoop ( dMaxBox.min, dMaxBox.max ) ;
	return pBigLoop ;
}
void AddLoopToLoopArr ( CSmtLoopArr& LoopArr, CSmartLoop* pHead, int  Layer, double dMinArea, BOOL bDel )
{
	if( !pHead ) return ;
	CSmartLoop* pLoop = pHead ;
	CSmartLoop* pN = NULL ;
	while( pLoop  )
	{
		pN = pLoop->next ;
		pLoop->prev = pLoop->next = NULL ;
		pLoop->m_nDepth = int( Layer ) ;
		// 判断环的面积大小
		if( bDel )
		{
			if( GetLoopsArea( pLoop ) < dMinArea )
				Mini_DeleteContours( pLoop ) ;
		}
		if( pLoop )
		{
			pLoop->DefineBox () ;
            LoopArr.Add ( pLoop ) ;
		}
		pLoop = pN ;
	}
}

void TransfPCombineToCPath( CPathCombine &PComb, CSmtCPathLib &cPathLib, double dFeedRate, double h, JDNC_TOL& Tol ) 
{
    UNUSED_ALWAYS( Tol ) ;
//	UNUSED_ALWAYS( h   ) ;
	// STEP 1 : 将曲线转化成三维路径
	CSmtCutPath *pPath = NULL ;
	CSmtCutPoint *pPoint = NULL ;
	CPathEntity *pEntity = NULL ;
	int nType = -1 ;
	for( pEntity = PComb.m_pHead; pEntity  ; pEntity = pEntity->next )
	{
		nType = pEntity->GetType() ;
		if( nType == NC_PATH_PLINE3D )
		{ // 3D 折线
			CPathPLine3D *pPLine = ( CPathPLine3D*) pEntity ;
		    pPath     = new CSmtCutPath() ;
			pPath->m_bFeedType = pEntity->m_bFeedType ;
			if( fabs( pEntity->m_fFeedScale - 1 ) > 0.01 )
			{
				pPath->m_fFeedRate = float ( pEntity->m_fFeedScale * dFeedRate ) ;
			}
			for( int i = 0 ; i <= pPLine->m_nCount  ; i ++ )
			{
				pPoint = new CSmtCutPoint( pPLine->m_pTAPos[i][0],
					                       pPLine->m_pTAPos[i][1],
										   pPLine->m_pTAPos[i][2] + h, //h
										   0.0 ) ;
				pPath->AddTail( pPoint ) ;
			}
		    pPath->DefineBox()  ;
		}
		else if( nType == NC_PATH_LINE3D )
		{
		    pPath     = new CSmtCutPath() ;
			pPath->m_bFeedType = pEntity->m_bFeedType ;
			if( fabs( pEntity->m_fFeedScale - 1 ) > 0.01 )
			{
				pPath->m_fFeedRate = float ( pEntity->m_fFeedScale * dFeedRate ) ;
			}
			CPathLine3D * pLine3D = ( CPathLine3D*) pEntity ;
			DOUBLE  dPoint[4] ;
			dPoint[3] = 0.0 ;
			mathCpyPnt( pLine3D->m_fStart, dPoint ) ;
			dPoint[2] += h ;
			pPath->AddPoint( dPoint ) ;

			mathCpyPnt( pLine3D->m_fEnd, dPoint ) ;
			dPoint[2] += h ;
			pPath->AddPoint( dPoint ) ;
			pPath->DefineBox() ;
		}
		else if( nType == NC_PATH_ARC3D )
		{
		    pPath     = new CSmtCutPath() ;
			pPath->m_bFeedType = pEntity->m_bFeedType ;
			if( fabs( pEntity->m_fFeedScale - 1 ) > 0.01 )
			{
				pPath->m_fFeedRate = float ( pEntity->m_fFeedScale * dFeedRate ) ;
			}
			CPathArc3D * pArc3D = ( CPathArc3D*) pEntity ;
			TPNT3D dBuff[500] ;
			int n = pArc3D->Discrete( Tol.m_dArcTol,ANGLE_TO_RADIAN( Tol.m_dAngTol), dBuff, 500 ) ;
			for( int i = 0 ; i <= n ; i ++ ) 
			{
                pPoint = new CSmtCutPoint( dBuff[i] ) ;
				pPoint->m_fPoint[2] = TFLOAT( h + dBuff[i][2] ) ;
                pPath->AddTail( pPoint ) ;
			}
			pPath->DefineBox() ;
		}
		else pPath = NULL ;
		if( pPath )
			cPathLib.AddToTail ( pPath ) ;
	}

}

void TransfPCombineToSmartCurve( CPathCombine &PComb, CPtrList &SmtCurList, JDNC_TOL& Tol )
{
	CSmartCurve* pSmtCurve = NULL;
	CPathEntity *pEntity = NULL ;
	int nType = -1 ;
	for( pEntity = PComb.m_pHead; pEntity  ; pEntity = pEntity->next )
	{
		nType = pEntity->GetType() ;
		if( nType == NC_PATH_PLINE3D )
		{ // 3D 折线
			CPathPLine3D *pPLine = ( CPathPLine3D*) pEntity ;
			pSmtCurve = new CSmartCurve;
			for( int i = 1 ; i <= pPLine->m_nCount  ; i ++ )
			{
				CSmartLine* pSmtLine = new CSmartLine( pPLine->m_pTAPos[i-1], pPLine->m_pTAPos[i]) ;
				pSmtCurve->AddSect(pSmtLine);
			}
		}
		else if( nType == NC_PATH_LINE3D )
		{
			CPathLine3D * pLine3D = ( CPathLine3D*) pEntity ;
			pSmtCurve = new CSmartCurve;
			CSmartLine* pSmtLine = new CSmartLine(pLine3D->m_fStart, pLine3D->m_fEnd);
			pSmtCurve->AddSect(pSmtLine);
		}
		else if( nType == NC_PATH_ARC3D )
		{
			CPathArc3D * pArc3D = ( CPathArc3D*) pEntity ;
			pSmtCurve = new CSmartCurve;
			if (pArc3D->m_bArcPlane == 0)
			{// xy
				CSmartArc* pSmtArc = new CSmartArc(pArc3D->m_fCenter, pArc3D->m_fAngle, pArc3D->m_fRadius);
				pSmtCurve->AddSect(pSmtArc);
			}
			else
			{// yz or zx or user define
				CPathArc3D * pArc3D = ( CPathArc3D*) pEntity ;
				TPNT3D dBuff[500] ;
				int n = pArc3D->Discrete( Tol.m_dArcTol,ANGLE_TO_RADIAN( Tol.m_dAngTol), dBuff, 500 ) ;
				for( int i = 1 ; i <= n ; i ++ ) 
				{
					CSmartLine* pSmtLine = new CSmartLine(dBuff[i-1], dBuff[i]);
					pSmtCurve->AddSect(pSmtLine);
				}
			}
		}
		else pSmtCurve = NULL ;
		if( pSmtCurve )
			SmtCurList.AddTail(pSmtCurve);
	}
}

void TransfSmartCurveToCPath( CSmartCurve *pCurve, CSmtCPathLib &cPathLib, double h, JDNC_TOL &Tol )
{
	// STEP 1 : 将曲线转化成三维路径
	CSmtCutPath *pPath = NULL ;
	CSmtCutPoint *pPoint = NULL ;
	CSmartSect *pSect = NULL ;
	CSmartCurve *pTmpCurve = pCurve ;
	int nType = -1 ;
	pPath = new CSmtCutPath() ;
	pPath->m_bFeedType = JDNC_FEEDTYPE_LEAD ;
	for( ; pTmpCurve ; pTmpCurve = pTmpCurve->next )
	{
		pSect = pTmpCurve->m_pHead ;
		for( ; pSect ; pSect = pSect->next )
		{
			nType = pSect->GetType () ;
			if( nType == NC_SECT_ARC )
			{
				CSmartArc * pArc = ( CSmartArc*) pSect ;
				TPNT2D dBuff[500] ;
				int n = pArc->DiscreteEx( Tol.m_dArcTol, ANGLE_TO_RADIAN( Tol.m_dAngTol ), dBuff, 499 ) ;
				for( int i = 0 ; i <= n ; i ++ ) 
				{
					pPoint = new CSmtCutPoint( dBuff[i][0], dBuff[i][1], h, 0. ) ;
					pPath->AddTail( pPoint ) ;
				}
			}
			else if( nType == NC_SECT_LINE )
			{
				CSmartLine * pLine = ( CSmartLine*) pSect ;
				double  start[3] = { 0., 0., h }, end[3] = { 0., 0., h } ;
				pLine->GetStart ( start ), pLine->GetEnd ( end ) ;
			
				pPath->AddPoint( start ), pPath->AddPoint ( end ) ; ;
			}
			else
			{
				ASSERT( 0 ) ;
			}
		}
	}
	if ( pPath->m_nNumPnt != 0)
	{
		pPath->DefineBox() ;
		cPathLib.AddToTail ( pPath ) ;
	} 
	else
	{
		delete pPath ;
	}
}

void		TransfAllLoopToCPath( CSmartLoop *AllBnd, CSmtCPathLib &AllPath, double h, JDNC_TOL &Tol )
{
	UNUSED_ALWAYS( h ) ;
	if( !AllBnd ) return ;
	// STEP 1 : 将曲线转化成三维路径
	CSmartCurve *pCurve = NULL ;
	CSmartLoop *pHead = AllBnd, *pIsland = NULL ;
	for( ; pHead ; pHead = pHead->next )
	{
		pCurve = pHead->m_pCurve ;
		for( ; pCurve ; pCurve = pCurve->next )
		{
			TransfSmartCurveToCPath( pCurve, AllPath, pCurve->m_dDepth, Tol ) ;
		}
		pIsland = pHead->GetIsland () ;
		if( pIsland )
		{
			for( ; pIsland ; pIsland = pIsland->next )
			{
				pCurve = pIsland->m_pCurve ;
				for( ; pCurve ; pCurve = pCurve->next )
				{
					TransfSmartCurveToCPath( pCurve, AllPath, pCurve->m_dDepth, Tol ) ;
				}
			}
		}
	}
}

BOOL SubtractContour( CSmartLoop *pCurr, CSmartLoop *pLast, CSmartLoop *AllLoop, CSmartCurveLib& CurLib, double dMinLen, BOOL bBtw )
{
	if( !pCurr || !pLast ) return NULL ;
	
	CSmartCurveLib tmpLib ;
	// STEP 0 : 轮廓初始化, 轮廓求交
	CSmartLoop *pLoopA = NULL, *pLoopB = NULL,  *pIsland = NULL ;
	for( pLoopA = pCurr ; pLoopA ; pLoopA = pLoopA->next )
	{
		pLoopA->InitBuffer() ;
	}
	for( pLoopB = pLast ; pLoopB ; pLoopB = pLoopB->next )
	{
		pLoopB->InitBuffer() ;
	}
	for( pLoopA = pCurr ; pLoopA ; pLoopA = pLoopA->next )
	{
		for( pLoopB = pLast ; pLoopB ; pLoopB = pLoopB->next )
		{
            pLoopA->ContourContourInt( pLoopB );
        }
	}
	// STEP 1 轮廓A打断
	CSmartCurve *pCurveTemp, *pCurveHead ;
	PNT2D dMidPnt, start, end ;
	int bIntFlag = 0 ;
	for( pLoopA = pCurr ; pLoopA ; pLoopA = pLoopA->next )
	{
        pCurveHead = pLoopA->ContourBreak();
        for(  ; pCurveHead  ; pCurveHead = pCurveTemp )
        {
			pCurveTemp = pCurveHead->next ;
			pCurveHead->next = NULL ;
			pCurveHead->GetMidPoint(dMidPnt);
			pCurveHead->GetStart ( start ) ;
			pCurveHead->GetEnd ( end ) ;
		    bIntFlag = Mini_IsPointOnContours( pLast, dMidPnt ) ;
			if( bIntFlag < 1 )   //(不落在上把刀区域里)    
			{ 
				if( !bBtw ) tmpLib.AddCurves ( pCurveHead ) ;
				else		delete pCurveHead ;
			}
			else 
			{
				if( !bBtw ) delete pCurveHead ;
				else		tmpLib.AddCurves ( pCurveHead ) ;
			}
		} 
	}
	
	// STEP 2 : 使用AllLoop对得到的曲线进行裁减,保留AllLoop中的部分
	for( pLoopB = AllLoop ; pLoopB ; pLoopB = pLoopB->next )
	{
		pLoopB->InitBuffer() ;
	}
	CSmartCurve *pHead = tmpLib.m_pHead, *pNext = NULL ;
	for( ; pHead ; pHead = pHead->next )
	{
		pHead->DefineBox () ;
		for( pLoopB = AllLoop ; pLoopB ; pLoopB = pLoopB->next )
		{
            pHead->CurveCurveInt ( pLoopB->m_pCurve ) ;
			for( pIsland = pLoopB->GetIsland () ; pIsland ; pIsland = pIsland->next )
			{
				pHead->CurveCurveInt ( pIsland->m_pCurve ) ;
			}
        }
	}
	for( pHead = tmpLib.m_pHead ; pHead ; pHead = pHead->next )
	{
		pCurveHead = pHead->BreakCurve () ;
		for(  ; pCurveHead  ; pCurveHead = pCurveTemp )
        {
			pCurveTemp = pCurveHead->next ;
			pCurveHead->next = NULL ;
			pCurveHead->GetMidPoint(dMidPnt);
		    bIntFlag = Mini_IsPointOnContours( AllLoop, dMidPnt ) ;
			if( bIntFlag > 0 )   //(落在边界里)    
			{ // 保留曲线,保留重线
				CurLib.AddCurves ( pCurveHead ) ;
			}
			else 
			{
				delete pCurveHead ;
			}
		} 
	}
	tmpLib.DeleteAllCurves () ;
	// STEP 3 : 连接曲线并删除特别小的
	CurLib.ConnectCurve ( 0.0001 ) ;
	pHead = CurLib.m_pHead ;
	while( pHead )
	{
		pNext = pHead->next ;
		pHead->DefineBox () ;
		if( pHead->GetLength () < dMinLen )
		{
			CurLib.DeleteCurve ( pHead ) ;
		}
		pHead = pNext ;
	}
	if( CurLib.m_nNumCurve < 1 ) return FALSE ;	
	return TRUE ;
}
int  IsPointOnContours( CSmartLoop* ConHead, PNT2D Point )
{
	int bFlag = 0 ; 
    for( CSmartLoop* pLoop = ConHead; pLoop ; pLoop = pLoop->next )
	{   
	    bFlag = pLoop->IsOnContourEx( Point, 0.001 ) ;
		if( bFlag != 0 ) break ;
   }
	return bFlag ;
}
BOOL AddCurveToLPath( CSmtLPathArr& LPathArr, CSmartCurveLib& CurLib, int  nLevel, BOOL bSetClosed )
{
	int  i, nSize = (int)LPathArr.GetSize () ;
	CSmtLoopPath *pLPath = NULL ;
	CSmartCurve *pHead , *pNext = NULL ;
	PNT2D mid ;
	BOOL bRet = FALSE ;

	for( i = 0 ; i < nSize ; i ++ )
	{
		pLPath = LPathArr.GetAt ( i ) ;
		if( pLPath->m_bClosedFlag )
		{
			 continue ;
		}
		CSmartCurveLib *pLib = new CSmartCurveLib() ;
		pHead = CurLib.m_pHead ;
		while( pHead )
		{
			pNext = pHead->next ;
			pHead->GetMidPoint ( mid ) ;
			if( IsPointOnContours ( pLPath->m_pLoop, mid ) )
			{
				CurLib.RemoveCurve ( pHead ) ;
				pHead->m_nLevelNo = int( nLevel ) ;
				pLib->AddTail ( pHead ) ;
				bRet = TRUE ;
			}
			pHead = pNext ;
		}
		if( pLib->m_nNumCurve > 0 )
		{
			// 判断pLPath的标记是否为闭合
			if( nLevel == 1 && bSetClosed)
			{
				SetPathClosedFlag( pLPath, pLib ) ;
			}
            pLPath->m_cLibArr.Add ( pLib ) ;
		}
		else
			delete pLib ;
	}
	return bRet ;
}
BOOL SetPathClosedFlag( CSmtLoopPath *pLPath, CSmartCurveLib *pLib )
{
	if( !pLPath || !pLib ) return FALSE ;
	CSmartCurve *pHead = pLib->m_pHead , *pNext = NULL ;
	PNT2D mid ;
	BOOL bClosed = TRUE ;
	while( pHead )
	{
		pNext = pHead->next ;
		pHead->GetMidPoint ( mid ) ;
		if( IsPointOnContours ( pLPath->m_pLoop, mid ) )
		{
			if ( !pHead->IsClosed() )
			{
				bClosed = FALSE ;
			}
		}
		pHead = pNext ;
	}
	pLPath->m_bClosedFlag = bClosed ;
	return TRUE ;
}
///////////////////////用于路径连接//////////////////////////////
void AddCutPathToAllPath( CPathGroup *NewPath, CSmtCPathLib &AllPath, CSmtCheckMdl *DriveMdl, 
						  double dMaxDist, CSmtCPathLib &cPathLib, JDNC_TOL &cTol, BOOL bAbsorb ) 
{
	int  i, nCnt = (int)cPathLib.m_cAllPath.GetCount () ;
	if( nCnt < 1 ) return ;
	CSmtCutPath *pPath = NULL ;
	PNT3D start, mid, end ;
	double dDist = 0., dDepth = 0. ;
	CSmtCutPath *pLine = NULL ;
	double dLen = dMaxDist ;
	FPNT3D fIntPnt[100] ;
	BOOL bConnect = FALSE ;
	// 如果NewPath中空,则取数组中第一个添加到newPath中
	if( NewPath && NewPath->m_pTail && bAbsorb )
	{ // 为AllPath添加一段连接线
		NewPath->m_pTail->GetEndPoint ( 1, start ) ;
		pPath = cPathLib.m_cAllPath.GetHead () ;
		GetSmtPathEndPnt( pPath, 0, end ) ;
		dDist = mathDist( start, end ) ;
		if( dDist < dLen && dDist > 0.001 )
		{
			if( start[2] > end[2] )
			{
				nc_VectorCopy( mid, end, 3 ) ;
				mid[2] = start[2] ;
				nc_VectorCopy( fIntPnt[0], start, 3 ) ;
				nc_VectorCopy( fIntPnt[1], mid, 3 ) ;
			}
			else
			{
				nc_VectorCopy( mid,start, 3 ) ;
				mid[2] = end[2] ;
				nc_VectorCopy( fIntPnt[0], mid, 3 ) ;
				nc_VectorCopy( fIntPnt[1], end, 3 ) ;
			}
			dDepth = max( start[2], end[2] ) + 0.1 ;
			nCnt = DriveMdl->AdsorbConnectLine ( cTol, fIntPnt, 100, TRUE ) ;
			bConnect = TRUE ;
			for( i = 0 ; i < nCnt ; i++ )
			{
				if( fIntPnt[i][2] > dDepth )
				{
					bConnect = FALSE ;	
					break ;
				}
			}
			if( nCnt >= 2 && bConnect )
			{
				pLine = new CSmtCutPath( MINI_CONNECT_PATH ) ;
				pLine->m_bFeedType = JDNC_FEEDTYPE_CONNECT ;
				for( i = 0 ; i < nCnt ; i++ )
				{
					pLine->AddPoint ( fIntPnt[i] ) ;
				}
				if ( start[2] > end[2] )
				{
					pLine->AddTail( new CSmtCutPoint( end) ) ;
				} 
				else
				{
					pLine->AddHead( new CSmtCutPoint( start ) ) ;
				}
				AllPath.AddToTail ( pLine ) ;
			}
		}
	}
	// 添加剩余曲线
	if( AllPath.m_cAllPath.GetCount () < 1 )
	{
		pPath = cPathLib.m_cAllPath.RemoveHead () ;
		AllPath.AddToTail ( pPath ) ;
	}
	// 然后依次连接路径
	POSITION pos, atpos ;
	pos = cPathLib.m_cAllPath.GetHeadPosition () ;
	
	while( pos )
	{
		pPath = AllPath.m_cAllPath.GetTail () ;
		GetSmtPathEndPnt( pPath, 1, start ) ;

		atpos = pos ;
		pPath = cPathLib.m_cAllPath.GetNext ( pos ) ;
		GetSmtPathEndPnt( pPath, 0, end ) ;
		cPathLib.m_cAllPath.RemoveAt ( atpos ) ;
		dDist = mathDist( start, end ) ;
		if( dDist < dLen && dDist > 0.001 )
		{
			if( bAbsorb )
			{
				if( start[2] > end[2] )
				{
					nc_VectorCopy( mid, end, 3 ) ;
					mid[2] = start[2] ;
					nc_VectorCopy( fIntPnt[0], start, 3 ) ;
					nc_VectorCopy( fIntPnt[1], mid, 3 ) ;
				}
				else
				{
					nc_VectorCopy( mid,start, 3 ) ;
					mid[2] = end[2] ;
					nc_VectorCopy( fIntPnt[0], mid, 3 ) ;
					nc_VectorCopy( fIntPnt[1], end, 3 ) ;
				}
				dDepth = max( start[2], end[2] ) + 0.1 ;
				nCnt = DriveMdl->AdsorbConnectLine ( cTol, fIntPnt, 100, TRUE ) ;
				bConnect = TRUE ;
				for( i = 0 ; i < nCnt ; i++ )
				{
					if( fIntPnt[i][2] > dDepth )
					{
						bConnect = FALSE ;	
						break ;
					}
				}
				if( nCnt >= 2 && bConnect )
				{
					pLine = new CSmtCutPath( MINI_CONNECT_PATH ) ;
					pLine->m_bFeedType = JDNC_FEEDTYPE_CONNECT ;
					for( i = 0 ; i < nCnt ; i++ )
					{
						pLine->AddPoint ( fIntPnt[i] ) ;
					}
					if ( start[2] > end[2] )
					{
						pLine->AddTail( new CSmtCutPoint( end) ) ;
					} 
					else
					{
						pLine->AddHead( new CSmtCutPoint( start ) ) ;
					}
					AllPath.AddToTail ( pLine ) ;
				}
			}
		}
		AllPath.AddToTail ( pPath ) ;
	}
	cPathLib.m_cAllPath.RemoveAll () ;
}

BOOL GetAllPathEndPoint ( CSmtCPathLib &AllPath, PNT3D end )
{
	// 如果没有路径返回FALSE
	if( AllPath.m_cAllPath.GetCount () < 1 )
		return FALSE ;

	CSmtCutPath *pPath = AllPath.m_cAllPath.GetTail () ;
	GetSmtPathEndPnt( pPath, 1, end ) ;
	return TRUE ;
}

void DeleteInvalidLoop ( CSmartLoop *&pHead, JDNC_TOL &cTol, double dTol )
{
	if( !pHead ) return ;
	
	CSmartLoop* pLoop = pHead, *pRemain = NULL, *pN = NULL ;
	
	while( pLoop  )
	{
		pN = pLoop->next ;
		pLoop->prev = pLoop->next = NULL ;
		if( IsRemainLoopValid( pLoop, cTol, dTol ) )
		{
			pRemain = Mini_AddContours( pRemain, pLoop ) ;
		}
		else
		{
			Mini_DeleteContours( pLoop ) ;
		}
        pLoop = pN ;
	}
	pHead = pRemain ;
}
// 判断一个残料区域环是否有效
BOOL IsRemainLoopValid( CSmartLoop *pReLoop, JDNC_TOL &cTol, double dTol ) 
{
	// 如果有岛,有效,返回
	if( pReLoop->m_pIsland ) return TRUE ;

	// 如果没有空线段返回
	BOOL bFind = FALSE ;
	CSmartSect *pHead = pReLoop->m_pCurve->GetHead () ;
	for( ; pHead ; pHead = pHead->next )
	{
		if( pHead->m_bEndFlag & NC_BLANK_SECT )
		{
			bFind = TRUE ;
			break ;
		}
	}
	if( !bFind ) return TRUE ;

	// 计算UnBlank和Blank之间最小距离的最大值
	pHead = pReLoop->m_pCurve->GetHead () ;
	for( ; pHead ; pHead = pHead->next )
	{
		if( pHead->m_bEndFlag & NC_BLANK_SECT ) 
		{
			//计算blank线段到unblank线段
			if( BlankSectDistLoop( pReLoop, pHead,cTol, dTol ) )
			{
				return TRUE ;
			}
		}
		else
		{
			// 计算unblank线段到blank线段
			if( UnBlankSectDistLoop( pReLoop, pHead, cTol, dTol ) )
			{
				return TRUE ;
			}
		}
	}
	
	return FALSE ;
}

// 计算Blank线段到当前刀具Sect的最近距离
BOOL BlankSectDistLoop( CSmartLoop *pReLoop, CSmartSect *pSect, JDNC_TOL &cTol, double dTol ) 
{
	PNT2D Buff[501], near_p ;
	int i = 0, nSize = 0 ;
	if( pSect->GetType () == NC_SECT_ARC )
	{
		nSize = ( ( CSmartArc *)pSect)->DiscreteEx ( 0.01, ANGLE_TO_RADIAN( cTol.m_dAngTol ), Buff, 500 ) + 1 ;
	}
	else
	{
		nSize = 2 ;
		pSect->GetStart ( Buff[0] ) ;
		pSect->GetEnd   ( Buff[1] ) ;
	}
	
	double dMinDist = 1000, dDist = 1.0e10 ;

	CSmartSect *pHead = NULL ;
	for( i = 0 ; i < nSize ; i++ )
	{
		// 计算Buff[i]到pReloop中非Blank段的最近距离
		dMinDist = 1000 ;

		pHead = pReLoop->m_pCurve->GetHead () ;
		for( ; pHead ; pHead = pHead->next )
		{
			// 如果blank段,不判断
			if( pHead->m_bEndFlag & NC_BLANK_SECT )	
				continue ;
			dDist = pHead->MinDistPoint ( Buff[i], near_p ) ;
			if( dDist < dMinDist )
				dMinDist = dDist ;
		}
		// 如果最近距离大于dTol,该pReloop保留
		if( dMinDist > dTol )
			return TRUE ;
	}

	return FALSE ;
}

// 计算UnBlank线段到当前刀具Sect的最近距离
BOOL UnBlankSectDistLoop( CSmartLoop *pReLoop, CSmartSect *pSect, JDNC_TOL &cTol, double dTol ) 
{
	PNT2D Buff[501], near_p ;
	int i = 0, nSize = 0 ;
	if( pSect->GetType () == NC_SECT_ARC )
	{
		nSize = ( ( CSmartArc *)pSect)->DiscreteEx ( 0.01, ANGLE_TO_RADIAN( cTol.m_dAngTol ), Buff, 500 ) + 1 ;
	}
	else
	{
		nSize = 2 ;
		pSect->GetStart ( Buff[0] ) ;
		pSect->GetEnd   ( Buff[1] ) ;
	}
	
	double dMinDist = 1000, dDist = 1.0e10 ;

	CSmartSect *pHead = NULL ;
	for( i = 0 ; i < nSize ; i++ )
	{
		// 计算Buff[i]到pReloop中非Blank段的最近距离
		dMinDist = 1000 ;

		pHead = pReLoop->m_pCurve->GetHead () ;
		for( ; pHead ; pHead = pHead->next )
		{
			// 如果Unblank段,不判断
			if( !( pHead->m_bEndFlag & NC_BLANK_SECT ) )
				continue ;
			dDist = pHead->MinDistPoint ( Buff[i], near_p ) ;
			if( dDist < dMinDist )
				dMinDist = dDist ;
		}
		// 如果最近距离大于dTol,该pReloop保留
		if( dMinDist > dTol )
			return TRUE ;
	}

	return FALSE ;
}
void AddToPathCombine( CSmtCutPath *pPath, CPathCombine &PComb, double dFeedRate )
{
	if( pPath->m_nNumPnt < 2 ) return  ;
	int nCnt = pPath->m_nNumPnt ;
	CSmtCutPoint *pPoint = pPath->m_pHead ;

	CPathPLine3D *pPLine3D = new CPathPLine3D() ;
	pPLine3D->m_pTAPos = new TPNT3D[ pPath->m_nNumPnt ] ;
	pPLine3D->m_nCount = pPath->m_nNumPnt - 1 ;
	for( nCnt = 0 ; pPoint ; pPoint = pPoint->next )
	{
		pPLine3D->m_pTAPos[nCnt][0] = pPoint->m_fPoint[0] ;
		pPLine3D->m_pTAPos[nCnt][1] = pPoint->m_fPoint[1] ;
		pPLine3D->m_pTAPos[nCnt][2] = pPoint->m_fPoint[2] ;
		nCnt++ ; 
	}
	pPLine3D->m_bFeedType = pPath->m_bFeedType ;
	if( pPath->m_fFeedRate > 0 )
	{
		pPLine3D->m_fFeedScale = float( pPath->m_fFeedRate / dFeedRate ) ;
	}
	pPLine3D->m_fFeedRate = pPath->m_fFeedRate ;
	PComb.AddEntity( pPLine3D ) ;
}
void AddToPathCombine( CSmtCPathLib &AllPath, CPathCombine *PComb, double dFeedRate )
{
	CSmtCutPath * pPath = NULL ;
	POSITION pos = AllPath.m_cAllPath.GetHeadPosition() ;
	while( pos )
	{
		pPath = AllPath.m_cAllPath.GetNext( pos ) ;
		if( ! pPath ) continue ;
		AddToPathCombine( pPath, *PComb, dFeedRate ) ;
		delete pPath ;
	}
	AllPath.m_cAllPath.RemoveAll() ;
}
void SetContourBlank( CSmartLoop *Contour )
{
	CSmartLoop *pHead = Contour, *pIsland = NULL ;
	for( ; pHead ; pHead = pHead->next )
	{
		pHead->m_pCurve->SetBlank ( TRUE ) ;
		pIsland = pHead->GetIsland () ;
		for( ; pIsland ; pIsland = pIsland->next )
		{
			pIsland->m_pCurve->SetBlank( TRUE ) ;
		}
	}
}

BOOL HasBlankSect( CSmartLoop *AllLoop )
{
	CSmartLoop *pHead = AllLoop, *pIsland = NULL ;
	CSmartSect *pSect = NULL ;

	for( ; pHead ; pHead = pHead->next )
	{
		pSect = pHead->GetSectHead () ;
		for( ; pSect ; pSect = pSect->next )
		{
			if( pSect->m_bEndFlag & NC_BLANK_SECT )
				return TRUE ;
		}
		pIsland = pHead->GetIsland () ;
		for( ; pIsland ; pIsland = pIsland->next )
		{
			pSect = pIsland->GetSectHead () ;
			for( ; pSect ; pSect = pSect->next )
			{
				if( pSect->m_bEndFlag & NC_BLANK_SECT )
					return TRUE ;
			}
		}
	}
	return FALSE ;
}

void	DeleteBlindLoop( CSmartLoop *&pHead, double dRadius ) 
{
	if( !pHead ) return ;

	CSmartLoop *Contour = pHead, *pRemain = NULL, *pNext = NULL ;
	while( Contour )
	{
		pNext = Contour->next ;
		Contour->prev = Contour->next = NULL ;
		if( IsBlindContour( Contour, dRadius ) )
		{
			Mini_DeleteContours( Contour ) ;
		}
		else
		{
			pRemain = Mini_AddContours( pRemain, Contour ) ;
		}

		Contour = pNext ;
	}
	pHead = pRemain ;
}

BOOL	IsBlindContour( CSmartLoop *Contour, double dRadius )
{
	if( !Contour ) return TRUE ;
	// 消除加工中的盲区
	double dDia = dRadius * 2 ;
	if( dDia > 0.05 && 
		Contour->m_dBox[1][0] - Contour->m_dBox[0][0] < dDia &&
		Contour->m_dBox[1][1] - Contour->m_dBox[0][1] < dDia )
	{
		// 半径过滤
		CSmartSect *pSect = Contour->GetSectHead() ;
		for( ; pSect ; pSect = pSect->next ) 
		{
			if( pSect->m_bEndFlag & NC_BLANK_SECT )
				break ;
		}
		if( pSect == NULL )
		{
			// 没有发现毛坯边界过滤成功
			return TRUE ;
		}
	}
	
	return FALSE ;
}

BOOL	SetContourHeadAtBlank(	CSmartLoop *AllLoop,	// <I> 输入换
								double	 dist	,		// <I> 最短长度
								PNT3D InOutPnt )		// <IO> 给定点
{
	if( !AllLoop ) return FALSE ;
	if( !HasBlankSect( AllLoop ) )
		return FALSE ;

	PNT2D tmp, find, pnt, start, end  ;
	AllLoop->m_pCurve->GetStart( pnt ) ;
	double dMinDist = 1.0e6, dDist = 0. ;
	CSmartCurve *pCurve = AllLoop->m_pCurve ;
	CSmartSect *pHead = pCurve->m_pHead, *pFindSect = NULL, *pNext = NULL ;
	
	pnt[0] = InOutPnt[0], pnt[1] = InOutPnt[1] ;
	// 搜索外轮廓
	while( pHead )
	{
		pNext = pHead->next ;
		if( !pNext ) break ;
		dDist = pHead->GetLength () ;
		if( pHead->m_bEndFlag & NC_BLANK_SECT && dDist >= dist )
		{
			// 取最近点   
			pHead->MinDistPoint ( pnt, tmp ) ;
			pHead->GetStart ( start ) ;
			pHead->GetEnd   ( end	) ;
			if( mathDist2D( start, tmp ) < 0.01 )
			{
				pHead->GetPoint ( 0.3, tmp ) ;
			}
			if( mathDist2D( end  , tmp ) < 0.01 )
			{
				pHead->GetPoint ( 0.7, tmp ) ;
			}
			dDist = mathDist2D( tmp, InOutPnt ) ;
			if( dDist < dMinDist )
			{
				dMinDist = dDist ;
				pFindSect = pHead ;
				mathCpyPnt2D( tmp, find ) ;
			}
		}
		pHead = pNext ;
	}
	BOOL bFind = FALSE ;
	if( pFindSect )
	{
		pCurve->SetStartPoint( pFindSect, find ) ;
		InOutPnt[0] = find[0] ;
		InOutPnt[1] = find[1] ;
		bFind = TRUE ;
	}
	return bFind ;
}

BOOL AllPathIntLine(	CSmtCPathLib &AllPath	,	// <I> 输入路径 
						PNT3D start, PNT3D end	,	// <I> 输入直线
						BOOL bIncludeHead )			// <I> 是否包括起末点
{
	if( AllPath.m_cAllPath.GetCount () < 1 ) return FALSE ;

	CSmtCutPath *pPath = NULL ;
	POSITION pos = AllPath.m_cAllPath.GetHeadPosition () ;
	FPNT3D p1 = { float( start[0]), float( start[1] ), 0.f }, p2 = { float( end[0] ), float( end[1] ), 0.f } ;
	FPNT3D st, ed, fIntpt, head, tail ;
	pPath = AllPath.m_cAllPath.GetHead() ;
	head[0] = pPath->m_pHead->m_fPoint[0], head[1] = pPath->m_pHead->m_fPoint[1], head[2] = 0.f ;
	pPath = AllPath.m_cAllPath.GetTail() ;
	tail[0] = pPath->m_pTail->m_fPoint[0], tail[1] = pPath->m_pTail->m_fPoint[1], tail[2] = 0.f ;
	FPNT3D fBox[2], pathBox[2] ;
	mathFDefBox3D( p1, p2, fBox, 0.001 ) ;
	CSmtCutPoint *pHead = NULL, *pNext = NULL ;
	while( pos )
	{
		pPath = AllPath.m_cAllPath.GetNext ( pos ) ;
		if( !pPath ) continue ;
		pathBox[0][0] = pPath->m_fBox[0][0], pathBox[0][1] = pPath->m_fBox[0][1] ;
		pathBox[1][0] = pPath->m_fBox[1][0], pathBox[1][1] = pPath->m_fBox[1][1] ;
		pathBox[0][2] = pathBox[1][2] = 0.f ;

		if( !mathFChkBox3DInt( fBox, pathBox, 0.1, 2 ) )
			continue ;

		pHead = pPath->m_pHead ;
		st[0] = pHead->m_fPoint[0], st[1] = pHead->m_fPoint[1], st[2] = 0.f ;
		while( pHead )
		{
			pNext = pHead->next ;
			if( !pNext ) break ;
			ed[0] = pNext->m_fPoint[0], ed[1] = pNext->m_fPoint[1], ed[2] = 0.f ;
			if( MathCam_FIntLine2D( p1, p2, st, ed, fIntpt, 1.0e-4 ) )
			{
				if( bIncludeHead ) 
				{
					return TRUE ;
				}
				else
				{ // 如果不包括头尾两个点
					if( mathFDist2D( fIntpt, head ) < 0.001 || mathFDist2D( fIntpt, tail ) < 0.001 )
					{
					}
					else
					{
						return TRUE ;
					}
				}
			}
			pHead = pNext ;
			mathFCpyPnt( ed, st ) ;
		}
	}
	return FALSE ;
}

// 判断直线是否同SmtLoopArr中的loop相交
BOOL AllLoopIntLine(	CSmartLoop *AllCont		,	// <I> 输入区域
						PNT3D start, PNT3D end )	// <I> 输入直线
{
	CSmartLoop *Contour = AllCont ;
	
	for(  ; Contour ; Contour = Contour->next )
	{
		if( Contour->IntLineContour ( start, end ) )
		{
			return TRUE ;
		}
	}
	return FALSE ;
}
// 判断直线是否同smartcurve相交
BOOL		AllCurveIntLine(	CPtrList &AllCurve		,	// <I> 输入所有曲线
						PNT3D start, PNT3D end )		// <I> 输入直线
{
	POSITION pos = AllCurve.GetHeadPosition () ;
	CSmartCurve *pCurve = NULL ;
	CSmartSect *pSect = NULL ;
	PNT2D intpt[5] ;
	CSmartLine tmpLine( start, end ) ;
	tmpLine.DefineBox() ;
	int i = 0, num = 0 ;
	while( pos )
	{
		pCurve = ( CSmartCurve *)AllCurve.GetNext ( pos ) ;
		if( !pCurve ) continue ;
		if( nc_BoxBoxInt( tmpLine.m_dBox, pCurve->m_dBox, 0.01 ) == 0 )
			continue ;
		for( ; pCurve ; pCurve = pCurve->next )
		{
			pSect = pCurve->GetHead() ;
			for( ; pSect ; pSect = pSect->next )
			{
				if( nc_BoxBoxInt( pSect->m_dBox, tmpLine.m_dBox, 0.01 ) == 0 )
					continue ;
				num = Mini_SectSectInt( *pSect, tmpLine, intpt ) ;
				for( i = 0 ; i < num ; i ++ )
				{
					if( mathDist2D( start, intpt[i] ) > 0.002 && 
						mathDist2D( end  , intpt[i] ) > 0.002  )
					{
						return TRUE ;
					}
				}
			}
		}
	}
	return FALSE ;
}
// 该函数用于判断区域加工中间的连刀是否与路径相交
BOOL AllPathEntIntLine( CPathEntity *pEnt, PNT3D start, PNT3D end , int nMethodType, BOOL bNext)
{
	if( !pEnt ) return FALSE ;
	if( nMethodType != surfncMethodPocket ) return FALSE ;
	CPathEntity *pHead = pEnt, *pNext = NULL ;
	int nType = -1 ;
	FPNT3D p1, p2, st, ed, fIntpt ;
	double t[2] = { -1,-1 }, dTol = 1.0e-4 ; //设置正加工精度，为了确保在首末点相交时，也判断出相交 qqs 2014.10.31
	CPathArc3D *pArc = NULL ;
	CPathPLine3D *pLine3D = NULL ;
	CPathLine3D *pLine = NULL ;
	nc_VectorCopy(  st, start, 3 ) ;
	nc_VectorCopy(  ed, end, 3 ) ;
	while( pHead )
	{
		pNext = pHead->next ;
		nType = pHead->GetType() ;
		if( nType == NC_PATH_LINE3D )
		{
			pLine = ( CPathLine3D *)pHead ;
			nc_VectorCopy(  p1, pLine->m_fStart, 3 ) ;
			nc_VectorCopy(  p2, pLine->m_fEnd, 3 ) ;
			
			if( MathCam_FIntLine3D( st, ed, p1, p2, fIntpt, t, dTol ) )
			{
				return TRUE ;
			}
			else
			{// 当连刀路径的非首末点与切割路径的首末点相交时，认为连刀与切割路径相交。 qqs 2014.04.28
				if ((t[0] < 1+dTol && t[0] > -dTol ) && 
					(fabs(t[1])<1.0e-4 ||fabs(t[1]-1)<1.0e-4)) 
					return TRUE ;
			}
		}
		else if( nType == NC_PATH_PLINE3D )
		{
			pLine3D = ( CPathPLine3D *)pHead ;
			int i = 0 , nCnt = pLine3D->m_nCount ;
			nc_VectorCopy(  p1, pLine3D->m_pTAPos[0], 3 ) ;
			for( i = 1 ; i <= nCnt ; i++ )
			{
				nc_VectorCopy( p2, pLine3D->m_pTAPos[i], 3 ) ;

				if( MathCam_FIntLine3D( st, ed, p1, p2, fIntpt, t, dTol ) )
				{
					return TRUE ;
				}
				else
				{// 当连刀路径的非首末点与切割路径的首末点相交时，认为连刀与切割路径相交。 qqs 2014.04.28
					if ((t[0] < 1+dTol && t[0] > -dTol ) && 
						(fabs(t[1])<1.0e-4 ||fabs(t[1]-1)<1.0e-4)) 
						return TRUE ;
				}
				mathFCpyPnt( p2, p1 ) ;	
			}
		}
		else if( nType == NC_PATH_ARC3D )
		{
			pArc = ( CPathArc3D *)pHead ;
			TPNT3D Buff[2000] ;
			int i = 0 ;
			int nCnt = pArc->Discrete( 0.01, ANGLE_TO_RADIAN( 10. ), Buff, 2000) ;
			nc_VectorCopy( p1, Buff[0], 3 ) ;
			for( i = 1 ;i < nCnt ; i++ )
			{
				nc_VectorCopy( p2, Buff[i], 3 ) ;
				if( MathCam_FIntLine3D( st, ed, p1, p2, fIntpt, t, dTol ) )
				{
					return TRUE ;
				}
				else
				{// 当连刀路径的非首末点与切割路径的首末点相交时，认为连刀与切割路径相交。 qqs 2014.04.28
					if ((t[0] < 1+dTol && t[0] > -dTol ) && 
						(fabs(t[1])<1.0e-4 ||fabs(t[1]-1)<1.0e-4)) 
						return TRUE ;
				}
				mathFCpyPnt( p2, p1 ) ;	
			}
		}
		else
		{}
		pHead = pNext ;

		if( !bNext ) break ;
	}

	return FALSE ;
}
// 该函数用于判断区域加工中间的连刀是否与路径相交(首末点相交除外)
// 只要确保中间部分不与路径相交即可，所以dTol可以设为-0.0001
BOOL AllPathEntIntCurve( CPathEntity *pEnt, CSmartCurve &tmpCurve , int nMethodType, BOOL bNext)
{
	if( !pEnt ) return FALSE ;
	if( nMethodType != surfncMethodPocket ) return FALSE ;
	CPathEntity *pHead = pEnt, *pNext = NULL ;
	int nType = -1 ;
	PNT3D start = {0}, end = {0} ;
	CSmartSect *pSect = NULL ;
	FPNT3D p1, p2, st, ed, fIntpt ;
	double t[2] = { 0 }, dTol = -1.0e-4 ;
	CPathArc3D *pArc = NULL ;
	CPathPLine3D *pLine3D = NULL ;
	CPathLine3D *pLine = NULL ;
	
	while( pHead )
	{
		pNext = pHead->next ;
		nType = pHead->GetType() ;
		if( nType == NC_PATH_LINE3D )
		{
			pLine = ( CPathLine3D *)pHead ;
			nc_VectorCopy(  p1, pLine->m_fStart, 3 ) ;
			nc_VectorCopy(  p2, pLine->m_fEnd, 3 ) ;

			pSect = tmpCurve.m_pHead ;
			for( ; pSect ; pSect = pSect->next )
			{
				if( pSect->GetType () == NC_SECT_LINE )
				{
					pSect->GetPoint( 0., start ) ;
					pSect->GetPoint( 1., end   ) ;

					nc_VectorCopy(  st, start, 3 ) ;
					nc_VectorCopy(  ed, end, 3 ) ;
				}
				else
				{
					ASSERT( 0 ) ;
					continue ;
				}
				if( MathCam_FIntLine3D( st, ed, p1, p2, fIntpt, t, dTol ) )
				{
					return TRUE ;
				}
			}
			
		}
		else if( nType == NC_PATH_PLINE3D )
		{
			pLine3D = ( CPathPLine3D *)pHead ;
			int i = 0 , nCnt = pLine3D->m_nCount ;
			nc_VectorCopy(  p1, pLine3D->m_pTAPos[0], 3 ) ;

			for( i = 1 ; i <= nCnt ; i++ )
			{
				nc_VectorCopy( p2, pLine3D->m_pTAPos[i], 3 ) ;

				pSect = tmpCurve.m_pHead ;
				for( ; pSect ; pSect = pSect->next )
				{
					if( pSect->GetType () == NC_SECT_LINE )
					{
						pSect->GetPoint( 0., start ) ;
						pSect->GetPoint( 1., end   ) ;

						nc_VectorCopy(  st, start, 3 ) ;
						nc_VectorCopy(  ed, end, 3 ) ;
					}
					else
					{
						ASSERT( 0 ) ;
						continue ;
					}
					if( MathCam_FIntLine3D( st, ed, p1, p2, fIntpt, t, dTol ) )
					{
						return TRUE ;
					}
				}
				mathFCpyPnt( p2, p1 ) ;
			}
		}
		else if( nType == NC_PATH_ARC3D )
		{
			pArc = ( CPathArc3D *)pHead ;
			TPNT3D Buff[2000] ;
			int i = 0 ;
			int nCnt = pArc->Discrete( 0.01, ANGLE_TO_RADIAN( 10. ), Buff, 2000) ;
			nc_VectorCopy( p1, Buff[0], 3 ) ;
			for( i = 1 ;i < nCnt ; i++ )
			{
				nc_VectorCopy( p2, Buff[i], 3 ) ;
				pSect = tmpCurve.m_pHead ;
				for( ; pSect ; pSect = pSect->next )
				{
					if( pSect->GetType () == NC_SECT_LINE )
					{
						pSect->GetPoint( 0., start ) ;
						pSect->GetPoint( 1., end   ) ;

						nc_VectorCopy(  st, start, 3 ) ;
						nc_VectorCopy(  ed, end, 3 ) ;
					}
					else
					{
						ASSERT( 0 ) ;
						continue ;
					}
					if( MathCam_FIntLine3D( st, ed, p1, p2, fIntpt, t, dTol ) )
					{
						return TRUE ;
					}
				}
				mathFCpyPnt( p2, p1 ) ;
			}
		}
		else
		{}
		pHead = pNext ;

		if( !bNext ) break ;
	}
	/**/
	return FALSE ;
}

// 判断单个CSmartCurv是否与直线相交
BOOL		SmartCurveIntLine(	CSmartCurve *pCurve			,	// <I> 输入曲线
								PNT3D start, PNT3D end	)		// <I> 输入直线
{
	if ( !pCurve ) return FALSE ;
		
	CSmartSect *pSect = NULL ;
	PNT2D intpt[5] ;
	CSmartLine tmpLine( start, end ) ;
	tmpLine.DefineBox() ;
	int i = 0, num = 0 ;
	// 首先判断包围和是否相交
	if( nc_BoxBoxInt( tmpLine.m_dBox, pCurve->m_dBox, 0.01 ) == 0 )
		return FALSE ;
	// 然后判断字段是否相交
	for( ; pCurve ; pCurve = pCurve->next )
	{
		pSect = pCurve->GetHead() ;
		for( ; pSect ; pSect = pSect->next )
		{
			if( nc_BoxBoxInt( pSect->m_dBox, tmpLine.m_dBox, 0.01 ) == 0 )
				continue ;
			num = Mini_SectSectInt( *pSect, tmpLine, intpt ) ;
			for( i = 0 ; i < num ; i ++ )
			{
				if( mathDist2D( start, intpt[i] ) > 0.002 && 
					mathDist2D( end  , intpt[i] ) > 0.002  )
				{
					return TRUE ;
				}
			}
		}
	}
	
	return FALSE ;
}
// 输入曲线，返回环
CSmartLoop *FormAllCurveToContour( CSmartCurveLib &AllLib ) 	// <I> 输入曲线
{
	if( AllLib.m_nNumCurve < 1 ) return NULL ;
	CSmartLoop loopLib, *AllBnd = NULL, *pLoop = NULL ;
	// 首先将AllLib的曲线首尾连接
	CSmartCurveLib tmpLib, closeLib ;
	CSmartCurve *pCurve = NULL, *pCurNext = NULL, *pFind = NULL ;
	pCurve = AllLib.m_pHead ;
	while( pCurve )
	{
		pCurNext = pCurve->next ;
		pCurve->next = NULL ;
		AllLib.RemoveCurve ( pCurve ) ;
		if( !pCurve->IsClosed() )
		{		
			tmpLib.AddTail ( pCurve ) ;
		}
		else
		{
			closeLib.AddTail ( pCurve ) ;
		}
		pCurve = pCurNext ;
	}
	AllLib.RemoveAll () ;
	pCurve = closeLib.m_pHead ;
	while( pCurve )
	{
		pCurNext = pCurve->next ;
		pCurve->next = NULL ;
		closeLib.RemoveCurve ( pCurve ) ;
		AllLib.AddTail ( pCurve ) ;
		pCurve = pCurNext ;
	}
	// 将非闭合的曲线进行连接
	double dStart[2], dEnd[2] ; 
	double dMinDist = 0. , dLen = 0. ; 
	while( tmpLib.m_nNumCurve > 0 )
	{
		pCurve = tmpLib.m_pHead ;
		tmpLib.RemoveCurve( pCurve ) ;
		pCurve->GetEnd( dEnd ) ;
		BOOL bFind = FALSE ;
		while( 1 )
		{
			dMinDist = 1.0e6 ;
			pCurNext = tmpLib.m_pHead ;
			for( ; pCurNext ; pCurNext = pCurNext->next )
			{
				pCurNext->GetStart ( dStart ) ;
				dLen = mathDist2D( dStart, dEnd ) ;
				if( dLen < dMinDist )
				{
					pFind = pCurNext ;
					dMinDist = dLen ;
					bFind = TRUE ;
				}
			}
			if( pFind && dMinDist < 0.01 )
			{
				bFind = TRUE ;
				tmpLib.RemoveCurve ( pFind ) ;
				CSmartSect *pHead = pFind->m_pHead, *pNext = NULL ;
				while( pHead )
				{
					pNext = pHead->next ;
					pFind->RemoveSect ( pHead ) ;
					pCurve->AddSect ( pHead ) ;
					pHead = pNext ;
				}
				delete pFind ;
				if( pCurve->IsClosed ( 0.01 ) )
				{
					AllLib.AddTail( pCurve ) ;
					break ;
				}
				else
				{
					pCurve->GetEnd( dEnd ) ;
				}
			}
			else
			{
				break ;
			}
		}	
		if( !bFind )
		{
			delete pCurve ;
		}
	}
	// 然后将闭合当前取出组成环
	pCurve = AllLib.m_pHead ;
	while( pCurve )
	{
		pCurNext = pCurve->next ;
		AllLib.RemoveCurve ( pCurve ) ;
		pCurve->next = NULL ;
		if( pCurve->IsClosed() )
		{
			pLoop = new CSmartLoop() ;
			pLoop->m_pCurve = pCurve ;
			pLoop->m_dArea = pLoop->Area () ;
			pLoop->DefineBox () ;
			AllBnd = Mini_AddContours( AllBnd, pLoop ) ;
		}
		else
		{
			delete pCurve ;
		}
		pCurve = pCurNext ;
	}
	AllBnd = loopLib.BuildContour ( AllBnd ) ;
	return AllBnd ;
}

/////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////
// 定义一个结构用于平坦面加工
CPlaneLoop::CPlaneLoop( CSmartLoop *pLoop, CSmartLoop *pOrgLoop )
{
	m_pPlaneLoop = pLoop ;
	m_pOrgLoop	 = pOrgLoop ;
	if( pLoop )
        m_dHeight = pLoop->m_dHeight ;
	else m_dHeight = 0. ;
	m_pRoughLoop = NULL ;
	m_pRemainLoop = NULL ;
	m_bValid = FALSE ;
	
	m_bSearch3 = FALSE ;
}
CPlaneLoop::~CPlaneLoop()
{
	Mini_DeleteContours( m_pPlaneLoop ) ;
	Mini_DeleteContours( m_pOrgLoop   ) ;
	Mini_DeleteContours( m_pRoughLoop ) ;
	Mini_DeleteContours( m_pRemainLoop ) ;
}
void CPlaneLoop::SetRoughLoop ( CSmartLoop *pRoughLoop )
{
	m_pRoughLoop = pRoughLoop ;
}
void CPlaneLoop::SetPlaneLoop ( CSmartLoop *pPlaneLoop ) 
{
	m_pPlaneLoop = pPlaneLoop ;
}
void CPlaneLoop::SetRemainLoop ( CSmartLoop *pRemainLoop )
{
	m_pRemainLoop = pRemainLoop ;
}
void CPlaneLoop::SetValid ( BOOL bValid )
{
	m_bValid = bValid ;
}
void CPlaneLoop::AddPreLoop(CSmartLoop * pPreLoop )
{
    if( pPreLoop == NULL ) return ;
    BOOL bAdd = TRUE ;
    CSmartLoop * tmpLoop = NULL ;
    
    for ( int i=0; i<m_PreLoopArr.GetSize(); i++ )
    {//避免重复元素
        tmpLoop = m_PreLoopArr.GetAt( i ) ;
        if( tmpLoop == pPreLoop )
        {
            bAdd = FALSE;
            break ;
        }
    }
    if ( bAdd )
    {
        m_PreLoopArr.Add( pPreLoop ) ;
    }
}
void ClearAllPlaneLoop( CPlaneList *pList )
{
	POSITION pos, atpos ;
	CPlaneLoop *pLoop = NULL ;
	pos = pList->GetHeadPosition ()  ;
	while( pos )
	{
		atpos = pos ;
		pLoop = pList->GetNext ( pos ) ;
		delete pLoop ;
		pList->RemoveAt ( atpos ) ;
	}
	pList->RemoveAll () ;
}
BOOL GetAllPlaneHeight( CPlaneList *pList, double *&dLayer, int  &nLayer )
{
	int  nCnt = (int)pList->GetCount (), n = 0 ;
	if( nCnt < 1 ) return FALSE ;
	double *dZ = new double[nCnt] ;
	double dCurrZ = 0, dLastZ = 0 ;
	CPlaneLoop *pPlane = pList->GetHead () ;
	dLastZ = dZ[0] = pPlane->m_dHeight ;
	POSITION pos = pList->GetHeadPosition () ;
	while( pos )
	{
		pPlane = pList->GetNext ( pos ) ;
		dCurrZ = pPlane->m_dHeight ;
		if( fabs( dCurrZ - dLastZ ) > 0.0002 )
		{
			n++ ;
			dZ[n] = dCurrZ ;
			dLastZ = dCurrZ ;
		}
	}
	dLayer = dZ ;
	nLayer = n + 1 ;
	return TRUE ;
}
BOOL  GetAllBndContHeight( CSmartLoop *BndCont, double *&dLayer, int  &nLayer )
{
	if( !BndCont ) return FALSE ;
	int  n = 0, nCnt = 0 ;
	CSmartLoop *pHead = BndCont, *pNext = NULL ;
	for( ; pHead ; pHead = pHead->next )
	{
		nCnt++ ;
	}
	double *dZ = new double[nCnt] ;
	double dCurrZ = 0, dNextZ = 0 ;
	
	dCurrZ = dZ[0] = BndCont->m_dHeight ;
	pHead = BndCont ;
	while( pHead )
	{
		pNext = pHead->next ;
		if( !pNext ) break ;
		dNextZ = pNext->m_dHeight ;
		if( fabs( dNextZ - dCurrZ ) > 0.0002 )
		{
			n++ ;
			dZ[n] = dNextZ ;
			dCurrZ = dNextZ ;
		}
		pHead = pNext ;
	}
	dLayer = dZ ;
	nLayer = n + 1 ;
	return TRUE ;
}

int	SortByDZ( const void *arg1, const void *arg2 ) 
{
	if( (( LayerFlag* )arg2)->m_dCurZ == (( LayerFlag* )arg1)->m_dCurZ )
	{
		return 0 ;
	}

	return (( LayerFlag* )arg2)->m_dCurZ > (( LayerFlag* )arg1)->m_dCurZ ? 1 : -1 ;
}
void  CombineAllHeight( double *dZ1, int  nCnt1, double *dZ2, int  nCnt2, double *&dAlldZ, int  &nCnt, BOOL *&bFlag )
{
	// 借助一个结构排序
	int  n = nCnt1 + nCnt2, i = 0 ;
	LayerFlag *pLayer = new LayerFlag[n] ;
	n = 0 ;
	for( i = 0 ; i < nCnt1 ; i++ )
	{
		pLayer[n].m_bFlag = TRUE ;
		pLayer[n].m_dCurZ = dZ1[i] ;
		n++ ;
	}
	for( i = 0 ; i < nCnt2 ; i++ )
	{
		pLayer[n].m_bFlag = FALSE ;
		pLayer[n].m_dCurZ = dZ2[i];
		n++ ;
	}
	// 对tmpArr中的Z,按照高度排序
	
	qsort( pLayer, n, sizeof( LayerFlag ), SortByDZ ) ;

	double *dTmpZ = new double[n] ;
	BOOL   *bTmpG = new BOOL  [n] ;
	for( i = 0 ; i < n ; i++ )
	{
		dTmpZ[i] = pLayer[i].m_dCurZ ;
		bTmpG[i] = pLayer[i].m_bFlag ;
	}
	delete[] pLayer;
	dAlldZ = dTmpZ ;
	bFlag  = bTmpG ;
	nCnt   = n     ;
}
double MinDistBetweenLoop( CSmartLoop *pLoop1, CSmartLoop *pLoop2 )
{
	if( !pLoop1 || !pLoop2 ) return MIN_LEN ;
	double dMin = 1.0e10, dDist = 0 ;
	CSmartSect *pHead = pLoop1->GetSectHead () ;
	for( ; pHead ; pHead = pHead->next )
	{
		dDist = MinDistBetweenLoopAndSect( pLoop2, pHead ) ;
		if( dDist < dMin )
		{
			dMin = dDist ;
		}
	}
	CSmartLoop *pIsland = pLoop1->GetIsland () ;
	if( pIsland )
	{
		for( ; pIsland ; pIsland = pIsland->next )
		{
			pHead = pIsland->GetSectHead () ;
			for( ; pHead ; pHead = pHead->next )
			{
				dDist = MinDistBetweenLoopAndSect( pLoop2, pHead ) ;
				if( dDist < dMin )
				{
					dMin = dDist ;
				}
			}
		}
	}
	return dMin ;
}

double MinDistBetweenLoopAndSect( CSmartLoop *pLoop, CSmartSect *pSect )
{
	if( !pLoop || !pSect ) return MIN_LEN ;
	double dMin = 1.0e10, dDist = 0 ;
	CSmartSect *pHead = pLoop->GetSectHead () ;
	for( ; pHead ; pHead = pHead->next )
	{
		dDist = pHead->MinDistBetween ( pSect ) ;
		if( dDist < dMin )
		{
			dMin = dDist ;
		}
	}
	CSmartLoop *pIsland = pLoop->GetIsland () ;
	if( pIsland )
	{
		for( ; pIsland ; pIsland = pIsland->next )
		{
			pHead = pIsland->GetSectHead () ;
			for( ; pHead ; pHead = pHead->next )
			{
				dDist = pHead->MinDistBetween ( pSect ) ;
				if( dDist < dMin )
				{
					dMin = dDist ;
				}
			}
		}
	}
	return dMin ;
}
BOOL IsBottomPlaneValid( CSmartLoop *pCutLoop, CSmartLoop *PlaneLoop, double dTol, double dRatio )
{
	// 判断两个环的重叠率有多大
	double dLength = 0., dOverLap = 0. ;
	CSmartCurve *pCurve = PlaneLoop->m_pCurve ;
	for( ; pCurve ; pCurve = pCurve->next )
	{
		dLength += pCurve->GetLength () ;
	}
	
	double dDist[2] = { 0., 0. } ;
	CSmartSect *pSect = PlaneLoop->GetSectHead () ;
	PNT2D start, end ;
	for( ; pSect ; pSect = pSect->next )
	{
		pSect->GetEnd ( end ) ;
		pSect->GetStart ( start ) ;

		dDist[0] = MinDistPntAndContour( pCutLoop, start ) ;
		dDist[1] = MinDistPntAndContour( pCutLoop, end ) ;
		if( dDist[0] < dTol && dDist[1] < dTol )
		{
			dOverLap += pSect->GetLength () ;
		}
	}
	if( dOverLap / dLength > dRatio )
		return FALSE ;
	return TRUE ;
}
double MinDistPntAndContour( CSmartLoop *Contour, PNT2D pt )
{
	double dMinDist = 1000, dDist = 0. ;
	CSmartLoop *pHead = Contour, *pIsland = NULL ;
	for( ; pHead ; pHead = pHead->next )
	{
		dDist = pHead->MinDistPoint ( pt ) ;
		if( dDist < dMinDist )
		{
			dMinDist = dDist ;
		}
		pIsland = pHead->GetIsland() ;
		for( ; pIsland ; pIsland = pIsland->next)
		{
			dDist = pIsland->MinDistPoint( pt ) ;
			if( dDist < dMinDist )
			{
				dMinDist = dDist ;
			}
		}
	}
	return dMinDist ;
}

double MinDistPntAndAllCurve( CPtrList &AllCurve, PNT2D pt )
{
	double dMinDist = 1000, dDist = 0. ;
	POSITION pos = AllCurve.GetHeadPosition() ;
	CSmartCurve *pCurve = NULL ;
	CSmartSect *pHead = NULL ;
	double dSeed[2] ;
	while ( pos ) 
	{
		pCurve = ( CSmartCurve *)AllCurve.GetNext ( pos ) ;
		if( !pCurve ) continue ;
		for( ; pCurve ; pCurve = pCurve->next )
		{
			pHead = pCurve->GetHead() ;
			for( ; pHead ; pHead = pHead->next )
			{
				dDist = pHead->MinDistPoint( pt, dSeed ) ;
				if( dDist < dMinDist ) dMinDist = dDist ;
			}
		}
	}

	return dMinDist ;
}

void TrimAllPathByHeight ( CSmtCPathLib &AllPath, double dTop, double dBot )
{
	CSmtCutPath *pPath = NULL ;
	POSITION pos, atpos ;
	CSmtCPathList tmpList ;
	CSmtCutPoint *pHead = NULL ;
	pos = AllPath.m_cAllPath.GetHeadPosition () ;
	while( pos )
	{
		atpos = pos ;
		pPath = AllPath.m_cAllPath.GetNext ( pos ) ;
		if( pPath->m_fBox[0][2] > dTop && pPath->m_fBox[1][2] > dTop ||
			pPath->m_fBox[0][2] < dBot && pPath->m_fBox[1][2] < dBot || pPath->m_nNumPnt < 2 )
		{
			delete pPath ;
			AllPath.m_cAllPath.RemoveAt ( atpos ) ;
		}
		else
		{
			pHead = pPath->m_pHead ;
			for( ; pHead ; pHead = pHead->next )
			{
				pHead->m_bType = 0 ;
			}
			TrimPathByZValue( pPath, dTop ) ;
			TrimPathByZValue( pPath, dBot ) ;
		}
	}
	// 对曲线进行判断
	pos = AllPath.m_cAllPath.GetHeadPosition () ;
	while( pos )
	{
		atpos = pos ;
		pPath = AllPath.m_cAllPath.GetNext ( pos ) ;
		BreakSmtPathAtBreak( pPath, tmpList ) ;
		delete pPath ;
		AllPath.m_cAllPath.RemoveAt ( atpos ) ;
	}
	// 将tmpList中曲线放到AllPath中
	PNT3D mid ;
	pos = tmpList.GetHeadPosition () ;
	while( pos )
	{
		atpos = pos ;
		pPath = tmpList.GetNext ( pos ) ;
		tmpList.RemoveAt ( atpos ) ;
		GetMiddlePoint( pPath, 0.5, mid ) ;
		if( mid[2] >= dBot && mid[2] <= dTop )
		{
			AllPath.m_cAllPath.AddTail ( pPath ) ;
		}
		else
		{
			delete pPath ;
		}
	}
	tmpList.RemoveAll () ;
}
BOOL TrimPathByZValue ( CSmtCutPath *pPath, double dZValue )
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
			if( LineIntLine( pPath, pStart, pEnd, dZValue ) )
				bInt = TRUE ;
		}
		pStart = pEnd ;
	}
	return bInt ;
}
BOOL LineIntLine( CSmtCutPath *pPath, CSmtCutPointEx *pStart, CSmtCutPointEx *pEnd, double dZValue )
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
			pInsert = GetMiddlePntEx( pStart, pEnd, t ) ;
			pInsert->m_bType = SMART_CUTPNT_BREAK ;
			pPath->InsertBefore ( pInsert, pEnd ) ;
		}
	}
	return TRUE ;
}
CSmtCutPointEx *GetMiddlePntEx ( CSmtCutPointEx *pStart, CSmtCutPointEx *pEnd, double u )
{
	if( !pStart || !pEnd ) return NULL ;
	CSmtCutPointEx *pMidd = new CSmtCutPointEx() ;
	pMidd->m_bType = 0 ;
	for( int i = 0 ; i < 3 ; i++ )
	{
		pMidd->m_fPoint[i] = pStart->m_fPoint[i] +TFLOAT( u * ( pEnd->m_fPoint[i] - pStart->m_fPoint[i] ) ) ;
		pMidd->m_fSurfNor[i] = pStart->m_fSurfNor[i] +TFLOAT( u * ( pEnd->m_fSurfNor[i] - pStart->m_fSurfNor[i] ) ) ;
		pMidd->m_fSurfPos[i] = pStart->m_fSurfPos[i] +TFLOAT( u * ( pEnd->m_fSurfPos[i] - pStart->m_fSurfPos[i] ) ) ;
		pMidd->m_fTempPos[i] = pStart->m_fTempPos[i] +TFLOAT( u * ( pEnd->m_fTempPos[i] - pStart->m_fTempPos[i] ) ) ;
		pMidd->m_fProjDir[i] = pStart->m_fProjDir[i] +TFLOAT( u * ( pEnd->m_fProjDir[i] - pStart->m_fProjDir[i] ) ) ;
	}
	return pMidd ;
}
////////////////////////////////////////////////////////////////////////////////////////
// CNcSmartPath.cpp
CNcSmartPath::CNcSmartPath()
{

}
CNcSmartPath::~CNcSmartPath()
{
}
// 根据 Z 高度修剪路径
int CNcSmartPath::TrimByZValue0( CSmtCPathLib& OldLib,
							     CSmtCPathLib& TopLib,     /*顶部路径*/
		                         DOUBLE  ZValue )          /*修剪高度*/      
								 
								
{
    CSmtCutPath *pPath ;
	CSmtCPathLib TempLib ;
    // STEP 0 : 根据打断原有路径
	POSITION   atCurr , atLast;
	atCurr = OldLib.m_cAllPath.GetHeadPosition() ;
	while( atCurr )
	{
		atLast = atCurr ;
		pPath = OldLib.m_cAllPath.GetNext( atCurr ) ;
		OldLib.m_cAllPath.RemoveAt( atLast ) ;
		CSmtCutPath* pNew = TrimPath0( TempLib, pPath, ZValue ) ;
		if( pNew ) 
		{ // 获得新路径
			RefineSmtPath( pNew ) ;
			TopLib.AddToTail( pNew ) ;
		}
		else
		{ // 增加新路径
			pPath->next = pPath->prev = NULL ;
			RefineSmtPath( pPath ) ;
			TopLib.AddToTail( pPath ) ;
		}
		delete pPath ;
	}
	POSITION pos = TopLib.m_cAllPath.GetHeadPosition() ;
	while( pos )
	{
		pPath = TopLib.m_cAllPath.GetNext( pos ) ;
		pPath->DefineBox() ;
	}
	pos = TempLib.m_cAllPath.GetHeadPosition() ;
	while( pos )
	{
		atCurr = pos ;
		pPath = TempLib.m_cAllPath.GetNext( pos ) ;
		TempLib.m_cAllPath.RemoveAt( atCurr ) ;
		pPath->DefineBox() ;
		OldLib.m_cAllPath.AddTail( pPath ) ;
	}
	TempLib.m_cAllPath.RemoveAll() ;
	return 1 ; 
}

CSmtCutPath* CNcSmartPath::TrimPath0 ( CSmtCPathLib& TempLib,
					   				   CSmtCutPath* pPath, 
									   DOUBLE ZValue )
{
	CSmtCutPath * pNewPath = new CSmtCutPath() ;
	pNewPath->m_nLineNo  = pPath->m_nLineNo ;
	pNewPath->m_nLayerNo = pPath->m_nLayerNo ;
	CSmtCutPath* pDepPath = NULL ;// = new CSmtCutPath() 
	FPNT4D dPoint ; 
	DOUBLE  t ; 
	CSmtCutPoint * pStart, *pEnd, *pPoint, *pHead ;
	
	pHead = pPath->m_pHead ;
	int n = 0 ;
	while( pHead ) 
	{
		pStart = pHead ;
		pHead = pHead->next ;
		pStart->next = pStart->prev = NULL ;
		pPath->RemovePoint( pStart ) ;

		/*过滤路径点*/
		if( pStart->m_fPoint[2] > ZValue  )
		{
			if( n > 0 )
			{
				TempLib.m_cAllPath.AddTail( pDepPath ) ;
				pDepPath = NULL ;
			}
			pNewPath->AddTail( pStart ) ;
			n = 0 ;
		}
		else 
		{
			if( n== 0 && !pDepPath )
			{
				pDepPath = new CSmtCutPath() ;
				pDepPath->m_nLineNo  = pPath->m_nLineNo ;
				pDepPath->m_nLayerNo = pPath->m_nLayerNo ;
			}
			pDepPath->AddTail( pStart ) ;
			nc_VectorCopy( dPoint, pStart->m_fPoint, 4 ) ;
			dPoint[2] = (TFLOAT)ZValue ;
			pNewPath->AddPoint( dPoint ) ;
			n++ ;
		}
		if( pHead == NULL ) break  ;
		/*增加中点*/
		pEnd  = pHead ;

		if( ! pStart || ! pEnd ) break ;
	    if( pStart->m_fPoint[2] > ZValue && pEnd->m_fPoint[2] < ZValue || 
			pStart->m_fPoint[2] < ZValue && pEnd->m_fPoint[2] > ZValue  )
		{
		    t = pEnd->m_fPoint[2] - pStart->m_fPoint[2] ;
			if( fabs( t ) > 1.0e-6 )
			{
				t = ( ZValue - pStart->m_fPoint[2] ) / t ;
				for( int i = 0 ; i < 3; i ++ ) 
				{
					dPoint[i] = pStart->m_fPoint[i] + 
						TFLOAT(t) * ( pEnd->m_fPoint[i] - pStart->m_fPoint[i] ) ;
				}
				dPoint[2] = (TFLOAT)ZValue ;
				pNewPath->AddPoint( dPoint ) ;
		        pPoint = new CSmtCutPoint( dPoint, dPoint[3] ) ;
				if( !pDepPath ) 
				{
					pDepPath = new CSmtCutPath() ;
					pDepPath->m_nLineNo  = pPath->m_nLineNo ;
					pDepPath->m_nLayerNo = pPath->m_nLayerNo ;
					pDepPath->AddTail( pPoint ) ;
					n = 0 ;
				}
				else
					pDepPath->AddTail( pPoint ) ;
			}
			else
			{
//				int err = 0 ;
			}
		}
	}
	if( n > 0 )
		TempLib.m_cAllPath.AddTail( pDepPath ) ;
	if( pDepPath && !pDepPath->m_pHead )
		delete pDepPath ;
	if( pNewPath->NumPoint() == 0 )
	{
		delete pNewPath ; 
		return NULL ; 
	}
	return pNewPath ; 
}

// 用于拷贝分层根据 Z 高度修剪路径
int CNcSmartPath::TrimByZValue1( CSmtCPathLib& OldLib,
							     CSmtCPathLib& LowLib,     /*顶部路径*/
		                         DOUBLE  ZValue,		   /*修剪高度*/
								 DOUBLE  h)          
								 
								
{
    CSmtCutPath *pPath ;
    // STEP 0 : 根据打断原有路径
	POSITION   atCurr , atLast;
	OldLib.DefineBox() ;
	if( OldLib.m_fBox[0][2] + h >= ZValue )
		return 0 ;
	atCurr = OldLib.m_cAllPath.GetHeadPosition() ;
	while( atCurr )
	{
		atLast = atCurr ;
		pPath = OldLib.m_cAllPath.GetNext( atCurr ) ;
		
		TrimPath1( pPath, ZValue, h, LowLib ) ;
	}
	return 1 ; 
}

void CNcSmartPath::TrimPath1 ( CSmtCutPath* pPath, 
							   DOUBLE ZValue,
							   DOUBLE h,
							   CSmtCPathLib& LowLib )
{
	CSmtCutPath * pNewPath = new CSmtCutPath() ;
	pNewPath->m_nLineNo  = pPath->m_nLineNo ;
	pNewPath->m_nLayerNo = pPath->m_nLayerNo ;
	FPNT4D dPoint, pt ; 
	DOUBLE  t ; 
	CSmtCutPoint  *pEnd, *pHead, * pStart = NULL ;
		
	pHead = pPath->m_pHead ;
	while( pHead ) 
	{
		short type = pHead->m_bType ;
		nc_VectorCopy( pt, pHead->m_fPoint, 4 ) ;	
		pHead = pHead->next ;

		/*过滤路径点*/
		if( pt[2] + h < ZValue  )
		{
			pStart = new CSmtCutPoint( ) ;
			nc_VectorCopy( pStart->m_fPoint, pt, 4 ) ;
			pStart->m_bType = type ;

			pStart->m_fPoint[2] += (TFLOAT) h ;
			pNewPath->AddTail( pStart ) ;
		}
//		else 
//		{
//			nc_VectorCopy( dPoint, pStart->m_fPoint, 3 ) ;
//			dPoint[2] = ZValue ;
//			pNewPath->AddPoint( dPoint ) ;
//		}
		if( pHead == NULL ) 
			break  ;
		/*增加中点*/
		pEnd  = pHead ;

	    if( pt[2] + h > ZValue && pEnd->m_fPoint[2] + h < ZValue || 
			pt[2] + h < ZValue && pEnd->m_fPoint[2] + h > ZValue  )
		{
		    t = pEnd->m_fPoint[2] - pt[2] ;
			if( fabs( t ) > 1.0e-6 )
			{
				t = ( ZValue - ( pt[2] + h ) ) / t ;
				for( int i = 0 ; i < 3; i ++ ) 
				{
					dPoint[i] = pt[i] + 
						TFLOAT(t) * ( pEnd->m_fPoint[i] - pt[i] ) ;
				}
				dPoint[2] = (TFLOAT)ZValue ;
				pNewPath->AddPoint( dPoint ) ;
				// 路径在此处打断
				RefineSmtPath(pNewPath ) ;
				pNewPath->DefineBox() ;
				LowLib.AddToTail( pNewPath ) ;

				pNewPath = new CSmtCutPath() ;
				pNewPath->m_nLineNo  = pPath->m_nLineNo ;
				pNewPath->m_nLayerNo = pPath->m_nLayerNo ;
			}
			else
			{
//				int err = 0 ;
			}
		}
	}

	if( pNewPath->NumPoint() == 0 )
		delete pNewPath ; 
	else
	{
		RefineSmtPath( pNewPath ) ;
		pNewPath->DefineBox() ;
		LowLib.AddToTail( pNewPath ) ;
	}

}

CSmtCutPath* CNcSmartPath::CopyPath ( CSmtCutPath* pPath, 
									  DOUBLE h )
{
	CSmtCutPath * pNewPath = new CSmtCutPath()  ;
	pNewPath->m_nLineNo    = pPath->m_nLineNo   ;
	pNewPath->m_nLayerNo   = pPath->m_nLayerNo  ;
	pNewPath->m_bFeedType      = pPath->m_bFeedType     ;
//	pNewPath->m_nNumPnt    = pPath->m_nNumPnt   ;
	pNewPath->m_fFeedRate  = pPath->m_fFeedRate ;

	CSmtCutPoint * pStart, *pHead ;
	
	pHead = pPath->m_pHead ;

	while( pHead ) 
	{
		pStart = pHead ;
		pHead = pHead->next ;
		CSmtCutPoint* pNew = new CSmtCutPoint() ;
		pNew->m_bType = pStart->m_bType ;
		nc_VectorCopy( pNew->m_fPoint, pStart->m_fPoint, 4 ) ;
		
		pNew->m_fPoint[2] += (TFLOAT)h ;

		pNewPath->AddTail( pNew ) ;
	}
	
	pNewPath->DefineBox() ;
	return pNewPath ;
}

/************************************************
 *这个函数将生成的三维加工路径进行处理，如果三  *
 *个点在一条直线上，则将直线连接，形成一条直线。*
************************************************/
int CNcSmartPath::RefineSmtPath( CSmtCutPath* pPath ) 
{
	if( pPath->NumPoint() < 2 ) return 0 ;
	CSmtCutPoint *preP, *curP, *nxtP, *posP ;
	FPNT3D preD, curD, nxtD ;
	PNT3D dLine[2], dPnt ;
	posP = pPath->m_pHead ;
	while( posP )
	{
		preP = posP, posP = posP->next  ;
		nc_VectorCopy( preD, preP->m_fPoint, 3 ) ;
		if( !posP )
			break ;	
		curP = posP, posP = posP->next ;
		nc_VectorCopy( curD, curP->m_fPoint, 3 ) ;
		if( !posP )
			break ;
		while( posP )
		{
			nxtP = posP , posP = posP->next ;
			nc_VectorCopy( nxtD, nxtP->m_fPoint, 3 ) ;
			nc_FloatToDouble( dLine[0], preD, 3 ) ;
			nc_FloatToDouble( dLine[1], nxtD, 3 ) ;
			nc_FloatToDouble( dPnt, curD, 3 ) ;
			double d = mathDistPntLinEx( dPnt, dLine[0], dLine[1] ) ;
			if(  fabs( d ) > 0.001 )
				break ;
			pPath->DeletePoint( curP ) ;
			curP = nxtP ;
			nc_VectorCopy(  curD, nxtD, 3 ) ;
		}
		posP = preP->next ;
	}
	return 1 ; 
}

int CNcSmartPath::TrimByZValue2( CSmtCPathLib& OldLib,
							     CSmtCPathLib& TopLib,     /*顶部路径*/
		                         DOUBLE  ZValue,           /*修剪高度*/
								 int nCutMode )          
								 
								
{
    CSmtCutPath *pPath ;
	CSmtCPathLib TempLib ;
    // STEP 0 : 根据打断原有路径
	POSITION   atCurr , atLast;
	atCurr = OldLib.m_cAllPath.GetHeadPosition() ;
	while( atCurr )
	{
		atLast = atCurr ;
		pPath = OldLib.m_cAllPath.GetNext( atCurr ) ;
		OldLib.m_cAllPath.RemoveAt( atLast ) ;
		CSmtCutPath* pNew = TrimPath2( TempLib, pPath, ZValue, nCutMode ) ;
		if( pNew ) 
		{ // 获得新路径
			RefineSmtPath( pNew ) ;
			TopLib.AddToTail( pNew ) ;
		}
		else
		{ // 增加新路径
			pPath->next = pPath->prev = NULL ;
			RefineSmtPath( pPath ) ;
			TopLib.AddToTail( pPath ) ;
		}
		delete pPath ;
	}
	POSITION pos = TopLib.m_cAllPath.GetHeadPosition() ;
	while( pos )
	{
		pPath = TopLib.m_cAllPath.GetNext( pos ) ;
		pPath->DefineBox() ;
	}
	pos = TempLib.m_cAllPath.GetHeadPosition() ;
	while( pos )
	{
		atCurr = pos ;
		pPath = TempLib.m_cAllPath.GetNext( pos ) ;
		TempLib.m_cAllPath.RemoveAt( atCurr ) ;
		pPath->DefineBox() ;
		OldLib.m_cAllPath.AddTail( pPath ) ;
	}
	TempLib.m_cAllPath.RemoveAll() ;
	return 1 ; 
}

CSmtCutPath* CNcSmartPath::TrimPath2 ( CSmtCPathLib& TempLib,
					   				   CSmtCutPath* pPath, 
									   DOUBLE ZValue,
									   int /*nCutMode*/ )
{
	CSmtCutPath * pNewPath = new CSmtCutPath() ;
	pNewPath->m_nLineNo  = pPath->m_nLineNo ;
	pNewPath->m_nLayerNo = pPath->m_nLayerNo ;
	CSmtCutPath* pDepPath = NULL ;// = new CSmtCutPath() 
	FPNT4D dPoint, pt ; 
	DOUBLE  t ; 
	CSmtCutPoint * pStart, *pEnd, *pPoint, *pHead ;
	
	pHead = pPath->m_pHead ;
	int n = 0 ;
	while( pHead ) 
	{
		pStart = pHead ;
		pHead = pHead->next ;
		pStart->next = pStart->prev = NULL ;
		pPath->RemovePoint( pStart ) ;
		nc_VectorCopy( pt, pStart->m_fPoint, 4 ) ;

		/*过滤路径点*/
		if( pStart->m_fPoint[2] > ZValue  )
		{
			if( n > 0 )
			{
				TempLib.m_cAllPath.AddTail( pDepPath ) ;
				pDepPath = NULL ;
			}
			pStart->m_fPoint[2] += pStart->m_fPoint[3] ;
			pNewPath->AddTail( pStart ) ;
			n = 0 ;
		}
		else 
		{
			if( n== 0 && !pDepPath )
			{
				pDepPath = new CSmtCutPath() ;
				pDepPath->m_nLineNo  = pPath->m_nLineNo ;
				pDepPath->m_nLayerNo = pPath->m_nLayerNo ;
			}
			pDepPath->AddTail( pStart ) ;
			nc_VectorCopy( dPoint, pStart->m_fPoint, 3 ) ;
			dPoint[2] = (TFLOAT)ZValue + pStart->m_fPoint[3] ;
			pNewPath->AddPoint( dPoint ) ;
			n++ ;
		}
		if( pHead == NULL ) break  ;
		/*增加中点*/
		pEnd  = pHead ;

		if( ! pStart || ! pEnd ) break ;
	    if( pt[2] > ZValue && pEnd->m_fPoint[2] < ZValue || 
			pt[2] < ZValue && pEnd->m_fPoint[2] > ZValue  )
		{
		    t = pEnd->m_fPoint[2] - pt[2] ;
			if( fabs( t ) > 1.0e-6 )
			{
				t = ( ZValue - pt[2] ) / t ;
				for( int i = 0 ; i <= 3; i ++ ) 
				{
					dPoint[i] = pt[i] + 
						TFLOAT(t) * ( pEnd->m_fPoint[i] - pt[i] ) ;
				}
				dPoint[2] = (TFLOAT) ZValue + dPoint[3] ;
				pNewPath->AddPoint( dPoint ) ;
				dPoint[2] = (TFLOAT)ZValue ;
		        pPoint = new CSmtCutPoint( dPoint, dPoint[3] ) ;
				if( !pDepPath ) 
				{
					pDepPath = new CSmtCutPath() ;
					pDepPath->m_nLineNo  = pPath->m_nLineNo ;
					pDepPath->m_nLayerNo = pPath->m_nLayerNo ;
					pDepPath->AddTail( pPoint ) ;
					n = 0 ;
				}
				else
					pDepPath->AddTail( pPoint ) ;
			}
			else
			{
//				int err = 0 ;
			}
		}
	}
	if( n > 0 )
		TempLib.m_cAllPath.AddTail( pDepPath ) ;
	if( pDepPath && !pDepPath->m_pHead )
		delete pDepPath ;
	if( pNewPath->NumPoint() == 0 )
	{
		delete pNewPath ; 
		return NULL ; 
	}
	return pNewPath ; 
}

// 用于拷贝分层根据 Z 高度修剪路径
int CNcSmartPath::TrimByZValue3( CSmtCPathLib& OldLib,
							     CSmtCPathLib& LowLib,     /*顶部路径*/
		                         DOUBLE  ZValue,		   /*修剪高度*/
								 DOUBLE  h )          
								 
								
{
    CSmtCutPath *pPath ;
    // STEP 0 : 根据打断原有路径
	POSITION   atCurr , atLast;
	OldLib.DefineBox() ;
	if( OldLib.m_fBox[0][2] + h >= ZValue )
		return 0 ;
	atCurr = OldLib.m_cAllPath.GetHeadPosition() ;
	while( atCurr )
	{
		atLast = atCurr ;
		pPath = OldLib.m_cAllPath.GetNext( atCurr ) ;
		TrimPath3( pPath, ZValue, h, LowLib ) ;
	}
	return 1 ; 
}

void  CNcSmartPath::TrimPath3 ( CSmtCutPath* pPath, 
							    DOUBLE ZValue,
							    DOUBLE h,
							    CSmtCPathLib& LowLib )
{
	CSmtCutPath * pNewPath = new CSmtCutPath() ;
	pNewPath->m_nLineNo  = pPath->m_nLineNo ;
	pNewPath->m_nLayerNo = pPath->m_nLayerNo ;
	FPNT4D dPoint, pStart, pEnd  ; 
	DOUBLE  t ; 
	CSmtCutPoint  *pHead ;
	
	pHead = pPath->m_pHead ;
	while( pHead ) 
	{
		nc_VectorCopy( pStart, pHead->m_fPoint, 4 ) ;
//		pStart[2] += h  ;
		pHead = pHead->next ;

		/*过滤路径点*/
		if( pStart[2] + h < ZValue  )
		{
			nc_VectorCopy( dPoint, pStart, 4 ) ;
			dPoint[2] += (TFLOAT)h + pStart[3] ;//
			pNewPath->AddPoint( dPoint ) ;
		}
//		else 
//		{
//			nc_VectorCopy( dPoint, pStart, 4 ) ;
//			if( pStart[2] + pStart[3] > CheckH ) // 此时该点掉下去
//				dPoint[2] += ZValue + pStart[3] ;
//			else 
//				dPoint[2] = ZValue + pStart[3] ;
//			pNewPath->AddPoint( dPoint ) ;
//		}
		if( pHead == NULL ) 
		{
			break  ;
		}
		/*增加中点*/
		nc_VectorCopy( pEnd, pHead->m_fPoint, 4 ) ;

	    if( pStart[2] + h > ZValue && pEnd[2] + h < ZValue || 
			pStart[2] + h < ZValue && pEnd[2] + h > ZValue  )
		{
		    t = pEnd[2] - pStart[2] ;
			if( fabs( t ) > 1.0e-6 )
			{
				t = ( ZValue - ( pStart[2] + h ) ) / t ;
				for( int i = 0 ; i <= 3; i ++ ) 
				{
					dPoint[i] = pStart[i] + 
						TFLOAT(t) * ( pEnd[i] - pStart[i] ) ;
				}
				dPoint[2] = (TFLOAT)ZValue + dPoint[3] ;
				pNewPath->AddPoint( dPoint ) ;
				// 此时路径打断，生成一个新的pNewPath ;
				RefineSmtPath( pNewPath ) ;
				pNewPath->DefineBox() ;
				LowLib.AddToTail( pNewPath ) ;
				pNewPath = new CSmtCutPath() ;
				pNewPath->m_nLineNo  = pPath->m_nLineNo ;
				pNewPath->m_nLayerNo = pPath->m_nLayerNo ;
			}
			else
			{
//				int err = 0 ;
			}
		}/**/
	}

	if( pNewPath->NumPoint() == 0  )
		delete pNewPath ;  
	else
	{
		RefineSmtPath( pNewPath ) ;
		pNewPath->DefineBox() ;
		LowLib.AddToTail( pNewPath ) ;
	}
}

/////////////////////////////////////////////////
// 粗加工和残料补加工的公共函数
/////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////
void AddContourToGroup( CSmartLoop *pHead, double h, CPathGroup& NewPath )
{
	if( !pHead ) return ;
	CPathCombine* PComb = new CPathCombine( NC_WPROCESS_ROUGH ) ;
	CSmartLoop *pIsland = NULL ;
	for( ; pHead ; pHead = pHead->next )
	{
		PComb->AddCurve ( pHead->m_pCurve, FALSE ) ;
		pIsland = pHead->GetIsland () ;
		for( ; pIsland ; pIsland = pIsland->next )
		{
			PComb->AddCurve ( pIsland->m_pCurve, FALSE ) ;
		}
	}
	NewPath.AddData ( -h, PComb ) ;
}
void AddCurveLibToGroup( CSmartCurveLib &curveLib, double h, CPathGroup &NewPath )
{
	CPathCombine* PComb = new CPathCombine( NC_WPROCESS_ROUGH ) ;
	CSmartCurve *pCurve = curveLib.m_pHead ;
	for( ; pCurve ; pCurve = pCurve->next ) 
	{
		PComb->AddCurve ( pCurve, FALSE, h ) ;
	}
	NewPath.AddData ( 0, PComb ) ;
}
void AddCurveLibToGroup( CSmtLPathArr &LPathArr, double h, CPathGroup &NewPath )
{
	int  i = 0, j = 0, nSize = (int)LPathArr.GetSize (), nCnt = 0 ;
	CSmtLoopPath *pLPath = NULL ;
	CSmartCurveLib *pCurveLib = NULL ;
	for( i = 0 ; i < nSize ; i++ )
	{
		pLPath = LPathArr[i] ;
		nCnt =(int) pLPath->m_cLibArr.GetSize () ;
		for( j = 0 ; j < nCnt ; j++ )
		{
			pCurveLib = pLPath->m_cLibArr.GetAt ( j ) ;
			AddCurveLibToGroup( *pCurveLib, h, NewPath ) ;
		}
	}
}
void AddLoopToGroup( CSmartLoop *pLoop, double h, CPathGroup& NewPath )
{
	if( !pLoop ) return ;
	CPathCombine* PComb = new CPathCombine( NC_WPROCESS_ROUGH ) ;
	CSmartLoop *pIsland = NULL ;
	for( ; pLoop ; pLoop = pLoop->next )
	{
		PComb->AddCurve ( pLoop->m_pCurve, FALSE ) ;
		pIsland = pLoop->GetIsland () ;
		for( ; pIsland ; pIsland = pIsland->next )
		{
			PComb->AddCurve ( pIsland->m_pCurve, FALSE ) ;
		}
	}
	NewPath.AddData ( -h, PComb ) ;
}
void AddLoopToGroup ( CSmtLPathArr& LPathArr, double h, CPathGroup& NewPath )
{
	int  i, n = (int)LPathArr.GetSize () ;
//	CSmartLoop *pLoop = NULL, *pIsland = NULL ;
	CSmtLoopPath *pLPath = NULL ;
	for( i = 0 ; i < n ; i++ )
	{
		pLPath = LPathArr[i] ;
		AddLoopToGroup( pLPath->m_pLoop, h, NewPath ) ;
	}
}
void AddPlaneLoopToGroup( CSmartLoop *pLoop, CPathGroup &NewPath )
{
	CSmartLoop *pHead = pLoop, *pIsland = NULL ;
	for( ; pHead ; pHead = pHead->next )
	{
		CPathCombine* PComb = new CPathCombine( NC_WPROCESS_ROUGH ) ;
		PComb->AddCurve ( pHead->m_pCurve, FALSE ) ;
		pIsland = pHead->GetIsland () ;
		for( ; pIsland ; pIsland = pIsland->next )
		{
			PComb->AddCurve ( pIsland->m_pCurve, FALSE ) ;
		}
		NewPath.AddData ( -pHead->m_dHeight, PComb ) ;
	}
}
void TestLoopsArr( CSmtLoopArr* LoopArr, double *dZ, int  nCount, CPathGroup& NewPath )
{
	if( !LoopArr || !dZ ) return ;
	for( int  i = 0 ; i < nCount ; i++ )
	{
		AddLoopToGroup( LoopArr[i], dZ[i], NewPath ) ;
	}
}
void TestLoopArr( CSmartLoop ** DriveLoop, double* dZ, int  nCount, CPathGroup& NewPath )
{
	if( !DriveLoop || !dZ ) return ;
	for( int  i = 0 ; i < nCount ; i++ )
	{
		AddLoopToGroup( DriveLoop[i], dZ[i], NewPath ) ;
	}
}
void ClearLoopArr ( CSmtLoopArr& LoopArr )
{
	CSmartLoop* pLoop = NULL ;
	int  nCount = (int)LoopArr.GetSize () ;
	for( int  i = 0 ; i < nCount ; i++ )
	{
		pLoop = LoopArr.GetAt ( i ) ;
		pLoop->next = pLoop->prev = NULL ;
		Mini_DeleteContours( pLoop ) ;
	}
	LoopArr.RemoveAll () ;
}
void ClearLPathArr( CSmtLPathArr& LPathArr )
{
	CSmtLoopPath *pLPath = NULL ;
	int  nSize = (int)LPathArr.GetSize () ;
	for( int  i = 0 ; i < nSize ; i++ )
	{
		pLPath = LPathArr.GetAt ( i ) ;
		if( pLPath->m_pLoop )
		{
			Mini_DeleteContours( pLPath->m_pLoop ) ;
			pLPath->m_pLoop = NULL ;
		}
		delete pLPath ;
	}
	LPathArr.RemoveAll () ;
} 
void ClearAllLPathArr( CSmtLPathArr*& pTmpArr, int  nCount )
{
	if( !pTmpArr ) return ;
	for( int  i = 0 ; i < nCount ; i++ )
	{
		ClearLPathArr( pTmpArr[i] ) ;
	}
	delete[] pTmpArr ;
	pTmpArr = NULL ;
}
void ClearAllLoopArr( CSmtLoopArr*& pTmpArr, int  nCount )
{
	if( !pTmpArr ) return ;
	for( int  i = 0 ; i < nCount ; i++ )
	{
		ClearLoopArr( pTmpArr[i] ) ;
	}
	delete[] pTmpArr ;
	pTmpArr = NULL ;
}
void ClearAllLoopHead( CSmartLoop**& pLoopArr, int  nCount )
{
	if( !pLoopArr ) return ;
	for( int  i = 0 ; i < nCount ; i++ )
	{
		if( pLoopArr[i] ) Mini_DeleteContours( pLoopArr[i] ) ;
	}
	delete[] pLoopArr ;
	pLoopArr = NULL ;
}
void TestRemainMdl( CSmtRemainMdl *pRemainMdl, CPathGroup& NewPath )
{
	CSmtCutPath *pPath = NULL ;
	CSmtCutPoint *pHead = NULL, *pCopy = NULL ;
	CSmtCPathLib AllPath ;
	for( int i = 0 ; i < pRemainMdl->m_nNumXDir ; i++ )
	{
		pHead = pRemainMdl->m_aXDirPath[i].m_pHead ;
		pPath = new CSmtCutPath() ;
		for( ; pHead ; pHead = pHead->next )
		{
			pCopy = pHead->CopyMyself () ;
			pPath->AddTail ( pCopy ) ;
		}
		AllPath.AddToTail ( pPath ) ;
	}
	for( i = 0 ; i < pRemainMdl->m_nNumYDir ; i++ )
	{
		pHead = pRemainMdl->m_aYDirPath[i].m_pHead  ;
		pPath = new CSmtCutPath() ;
		for( ; pHead ; pHead = pHead->next )
		{
			pCopy = pHead->CopyMyself () ;
			pPath->AddTail ( pCopy ) ;
		}
		AllPath.AddToTail ( pPath ) ;
	}
	CPathCombine *PComb = new CPathCombine( NC_WPROCESS_ROUGH ) ;
	AllPath.AddToPathCombine ( *PComb ) ;
	NewPath.AddData ( 0, PComb, TRUE ) ;
}
void AddAllPathToGroup( CSmtCPathLib &AllPath, CPathGroup &NewPath )
{
	CPathCombine *PComb = new CPathCombine( NC_WPROCESS_ROUGH ) ;
	AllPath.AddToPathCombine ( *PComb ) ;
	NewPath.AddData ( 0, PComb, TRUE ) ;
}
void AddPathArrToGroup( CSmtLPathArr &PathArr, CPathGroup &NewPath )
{
	int  i = 0 , nSize = (int)PathArr.GetSize() ;
	CSmtLoopPath *pLPath = NULL ;
	for( i = 0 ; i < nSize ; i++ )
	{
		pLPath = PathArr.GetAt ( i ) ;
		AddAllPathToGroup( pLPath->m_cPathLib, NewPath ) ;
	}
}
void AllCutPathToGroup( CSmtCutPath *pPath, CPathGroup &NewPath )
{
	if( !pPath )return ;
	CPathCombine *PComb = new CPathCombine( NC_WPROCESS_ROUGH ) ;
	pPath->AddToPathCombine ( *PComb ) ;
	NewPath.AddData ( 0, PComb, TRUE ) ;
}
CPathCombine *AddLoopToPComb( CSmartLoop* pLoop, WORD nType )
{
	if( !pLoop ) return NULL ;
	CPathCombine *PComb = new CPathCombine( nType ) ;
	CSmartLoop *pIsland = NULL ;
	
	for( ; pLoop ; pLoop = pLoop->next )
	{
		PComb->AddCurve ( pLoop->m_pCurve, FALSE ) ;
		pIsland = pLoop->GetIsland () ;
		for( ; pIsland ; pIsland = pIsland->next )
		{
			PComb->AddCurve ( pIsland->m_pCurve, FALSE ) ;
		}
	}
	return PComb ;
}
void TestDriveMdl( CSmtCheckMdl *DriveMdl, CPathGroup &NewPath )
{
	CPathCombine *	PComb = CreateDriveMdl ( DriveMdl ) ;
	NewPath.AddData( 0, PComb ) ;
}

void TestMeshMdl( CSmtMeshMdl *MeshMdl, CPathGroup &NewPath )
{
	if( !MeshMdl ) return ;

	CSmtCPathLib AllPath ;
	CSmtCutPath *pPath = NULL ;
	CSmtCutPoint *pHead = NULL, *pCopy = NULL ;
	for( int i = 0 ; i < MeshMdl->m_nNumXDir ; i++ )
	{
		pHead = MeshMdl->m_aXDirPath[i].m_pHead ;
		pPath = new CSmtCutPath() ;
		for( ; pHead ; pHead = pHead->next )
		{
			pCopy = pHead->CopyMyself () ;
			pPath->AddTail ( pCopy ) ;
		}
		AllPath.AddToTail ( pPath ) ;
	}
	for( i = 0 ; i < MeshMdl->m_nNumYDir ; i++ )
	{
		pHead = MeshMdl->m_aYDirPath[i].m_pHead  ;
		pPath = new CSmtCutPath() ;
		for( ; pHead ; pHead = pHead->next )
		{
			pCopy = pHead->CopyMyself () ;
			pPath->AddTail ( pCopy ) ;
		}
		AllPath.AddToTail ( pPath ) ;
	}
	CPathCombine *PComb = new CPathCombine( NC_WPROCESS_ROUGH ) ;
	AllPath.AddToPathCombine( *PComb ) ;
	NewPath.AddData( 0., PComb ) ;
}
void TestAllPlaneLoop( CPlaneList *pList, CPathGroup &NewPath, int nType )
{
	POSITION pos = pList->GetHeadPosition () ;
	CPlaneLoop *pPlane = NULL ;
	while( pos )
	{
		pPlane = pList->GetNext ( pos ) ;
		
		if( nType == 0 )
		{
			AddContourToGroup( pPlane->m_pPlaneLoop, pPlane->m_dHeight, NewPath ) ;
			AddContourToGroup( pPlane->m_pOrgLoop  , pPlane->m_dHeight, NewPath ) ;
			AddContourToGroup( pPlane->m_pRemainLoop,pPlane->m_dHeight, NewPath ) ;
			AddContourToGroup( pPlane->m_pRoughLoop, pPlane->m_dHeight, NewPath ) ;
		}
		else if( nType == 1 )
            AddContourToGroup( pPlane->m_pPlaneLoop, pPlane->m_dHeight, NewPath ) ;
		else if( nType == 2 )
            AddContourToGroup( pPlane->m_pOrgLoop  , pPlane->m_dHeight, NewPath ) ;
		else if( nType == 3 )
			AddContourToGroup( pPlane->m_pRemainLoop,pPlane->m_dHeight, NewPath ) ;
		else if( nType == 4 )
			AddContourToGroup( pPlane->m_pRoughLoop, pPlane->m_dHeight, NewPath ) ;
	}
}
void TestAllPlaneLoop( CSmartLoop *Planeloop, CPathGroup &NewPath )
{
	CSmartLoop *pHead = Planeloop, *pNext = NULL ;
	while( pHead )
	{
		pNext = pHead->next ;
		pHead->next = pHead->prev = NULL ;
		AddContourToGroup( pHead, pHead->m_dHeight, NewPath ) ;
		pHead = pNext ;
	}
}
BOOL CheckCutLoop ( CSmtLoopArr *pTmpArr, int & nCount )
{
	int  i, nSize = nCount ;
	double dLastArea = 0., dCurrArea = 0. ;
	for( i = 0 ; i < nSize ; i ++ )
	{
		dCurrArea = GetLoopsArea( pTmpArr[i] ) ;
		if( i > 0 )
		{
			if( dCurrArea > dLastArea + 10. )
			{
				nCount = i ;
				return FALSE ;
			}
		}
		dLastArea = dCurrArea ;
	}
	return TRUE ;
}
CSmartLoop* LoopLoopInt( CSmartLoop* pHead, CSmartLoop* pLoop ) 
{
	CSmartLoop *pL = NULL, *head = NULL ;
	CSmartLoop loop ;
	if( !pHead )
	{
		pHead = Mini_AddContours( pHead, pLoop ) ;
		return pHead ;
	}
	head = pHead  ;
	while( head )
	{
		pL = head->next ;
		int bFlag = head->ContourContourInt( pLoop ) ;
		if( bFlag )
		{
			pHead = Mini_RemoveLoop( pHead, head ) ;
			pLoop = Mini_AddContours( pLoop, head ) ;
			pLoop->m_nGroupId = 1 ;
			pLoop = loop.WeldContours( pLoop ) ;
			pLoop->m_nGroupId = 1 ;
		}
		head = pL ;
	}
	pLoop->m_nGroupId = 0 ;
	pHead = Mini_AddContours( pHead, pLoop ) ;
	return pHead ;
}
CSmartLoop *CreateBigLoop ( CSmartLoop **pDriveLoop, CSmartLoop *AllLoop, int  nCount, double r ) 
{
	CSmartLoop *pBigLoop = NULL ;
	// 首先得到加工环的最大包围盒
	BOX3D box, maxBox ;
	mathClnBox3D( &maxBox ) ;
	for( int  i = 0 ; i < nCount ; i++ )
	{
		if( pDriveLoop[i] )
            MathCam_GetLoopBox( pDriveLoop[i], &box ) ;
		else
			MathCam_GetLoopBox( AllLoop,      &box ) ;
        mathCalBox3DUnion( &box, &maxBox, &maxBox ) ;
	}
	mathExpandBox3D( r, &maxBox ) ;
	pBigLoop = new CSmartLoop() ;
	pBigLoop->CreateLoop ( maxBox.min, maxBox.max ) ;
	return pBigLoop ;
}
CSmartLoop *CreateBigLoopAndBox( CSmartLoop **pDriveLoop, CSmartLoop *AllLoop, 
								 BOX3D &dMaxBox, int  nCount, double r ) 
{
	CSmartLoop *pBigLoop = NULL ;
	// 首先得到加工环的最大包围盒
	BOX3D box ;
	for( int  i = 0 ; i < nCount ; i++ )
	{
		if( pDriveLoop[i] )
            MathCam_GetLoopBox( pDriveLoop[i], &box ) ;
		else
			MathCam_GetLoopBox( AllLoop,      &box ) ;
        mathCalBox3DUnion( &box, &dMaxBox, &dMaxBox ) ;
	}
	mathExpandBox3D( r, &dMaxBox ) ;
	pBigLoop = new CSmartLoop() ;
	pBigLoop->CreateLoop ( dMaxBox.min, dMaxBox.max ) ;
	return pBigLoop ;
}
CSmartLoop* ManageLoop ( CSmartLoop* pHead, CSmartTool* pTool, CSmartLoop* AllLoop, 
						 BOOL bIsRoughCast, BOOL bBtwLayer )
{
	CSmartLoop *BndHead, *BndTail, *pLoop ;
	BndHead = BndTail = pLoop = NULL ;

	CSmartLoop* pTail = pHead ;
	while( pTail && pTail->next )	pTail = pTail->next ;

	CSmartLoop  LoopLib ;
	// STEP 4 :获取所有环的最大边界
	if( 1 ) 
	{
		BOX3D box1, box2, maxbox ;
		MathCam_GetLoopBox( pHead, &box1 ) ;
		MathCam_GetLoopBox( AllLoop, &box2 ) ;
		mathCalBox3DUnion( &box1, &box2, &maxbox ) ;
		double tol = 2 * pTool->m_fRadius  ;
		if( tol == 0. ) return FALSE ;
		mathExpandBox3D( tol, &maxbox ) ;
		CSmartLoop* pLarge = new CSmartLoop() ;
		pLarge->CreateLoop ( maxbox.min , maxbox.max ) ;
		pHead = Mini_AddContours( pHead, pLarge ) ;
		while( pTail && pTail->next )	pTail = pTail->next ;
	}

	// 重新组环,存到相应的层中.	
	pHead = LoopLib.BuildContour( pHead ) ;
	// 两组环求交
	if( !bIsRoughCast && bBtwLayer )
	{
		BndHead = Mini_OperateContours( pHead, AllLoop, 0 ) ;
		MathCAM_ValidOutMostLoop( BndHead ) ;
	}
	else
		BndHead = Mini_CopyContours( pHead ) ;
	// 删除计算后的环
	Mini_DeleteContours( pHead ) ;
	// 求交返回值为假
	if( !BndHead ) return FALSE ;
	return BndHead ;
}
BOOL IsErrorPComb( CPathCombine *PComb )
{
	if( !PComb ) return FALSE ;

	for( CPathEntity *pEnt = PComb->m_pHead ; pEnt ; pEnt = pEnt->next )
	{
		int nType = pEnt->GetType() ;
		if( pEnt->m_bFeedType == JDNC_FEEDTYPE_QUICK &&
			nType == NC_PATH_ARC3D )
		{
				ASSERT( 0 ) ;
                return TRUE ;
		}
	}
	return FALSE ;
}
int     SortHeight( const void *arg1, const void *arg2 ) 
{
	if( *((double*)arg2) == *((double* )arg1))
		return 0 ;

	return *((double* )arg2) > *((double*)arg1) ? 1 : -1 ;
}

void GetUsedLayerZ( double *dSurfZ, int  nNum, double *dZ, int & nCount, 
				    double dMinStepZ, double dBot, double dTop )
{
	// STEP 1 : 添加平面
	int  i, j, k ;
	nCount = 0 ;
	
	dZ[0] = dBot ;

	for( i = 0 ; i < nNum ; i++ )
	{
		if( dSurfZ[i] > dBot && dSurfZ[i] < dTop )
		{
			nCount++ ;
			dZ[nCount] = dSurfZ[i] ;
		}
	}
	nCount++ ;
	dZ[nCount] = dTop ;
	nCount++ ;
	// STEP 2 : 并按照高度递减的顺序排好
	qsort( dZ, nCount, sizeof( double ), SortHeight ) ;
    DOUBLE dZTol = min( dMinStepZ, 0.01 ) ;
    if( nCount > 2 && fabs( dZ[nCount-1] - dZ[nCount-2] ) < dZTol )
    {// ZBQ : 处理最低层, 避免最低的平面被过滤掉
        nCount -- ;
    }
	// STEP 3 : 将高度相同的平面和上下高度差小于dMinZ的过滤
	for( i = 0 ; i < nCount - 1 ; i++ )
	{
		int  nFind = -1 ;
		for( j = i+1 ; j < nCount ; j++ )
		{
			if( dZ[i] - dZ[j] >= dMinStepZ )
			{
				nFind = j ;
				break ;
			}
		}
		if( nFind > i+1 )
		{
			for( k = nFind ; k < nCount ; k++ )
			{
				dZ[i+1+k-nFind] = dZ[k] ;
			}
			nCount = nCount - ( nFind - i - 1 ) ;
		}
		else if( nFind < 0 )
		{
			// 找不到,则跳到最后一层
			dZ[i+1] = dZ[nCount-1] ;
			nCount = i+2 ;
			break ;
		}
	}
}
// 将环组反向
void ReverseAllContour( CSmartLoop *&pHead ) 
{
	if( !pHead ) return ;

	CSmartLoop *pTail = pHead, *pPrev = NULL, *pOrder = NULL ;
	while( pTail && pTail->next ) pTail = pTail->next ;
	while( pTail )
	{
		pPrev = pTail->prev ;
		pTail->next = pTail->prev = NULL ;
		pOrder = Mini_AddContours( pOrder, pTail ) ;
		pTail = pPrev ;
	}

	pHead = pOrder ;
}
void SetCutPathZValue( CSmtCutPath *pPath, TFLOAT fZValue )
{
	CSmtCutPoint *pHead = pPath->m_pHead ;
	for( ; pHead ; pHead = pHead->next )
	{
		pHead->m_fPoint[2] = fZValue ;
	}
}

void TrimCutPathZValue( CSmtCutPath *&pPath, TFLOAT fZValue )
{
	CSmtCutPoint *pHead = pPath->m_pHead, *pNext = NULL ;
	for( ; pHead ; pHead = pHead->next )
	{
		if( pHead->m_fPoint[2] < fZValue )
			break ;
	}
	if( !pHead ) return ;

	CSmtCutPath *PathHead = NULL, *pNewPath = new CSmtCutPath() ;
	pHead = pPath->m_pHead ;
	while( pHead )
	{
		pNext = pHead->next ;
		pPath->RemovePoint ( pHead ) ;
		if( pHead->m_fPoint[2] > fZValue )
		{
			pNewPath->AddTail ( pHead ) ;
		}
		else
		{
			delete pHead ;
			if( pNewPath->m_nNumPnt > 1 )
			{
				PathHead = AddSmtCutPath( PathHead, pNewPath ) ;
			}
			else
			{
				delete pNewPath ;
			}
			pNewPath = new CSmtCutPath() ;
		}
		pHead = pNext ;
	}
	if( pNewPath->m_nNumPnt > 1 )
		PathHead = AddSmtCutPath( PathHead, pNewPath ) ;
	else
		delete pNewPath ;
	delete pPath ;
	pPath = PathHead ;
}
CSmtCutPath *AddSmtCutPath( CSmtCutPath *PathHead, CSmtCutPath *NewPath )
{
	if( !NewPath ) return PathHead ;
	if( !PathHead ) return NewPath ;
	CSmtCutPath *pPath = PathHead ;
	while( pPath && pPath->next )
		pPath = pPath->next ;
	pPath->next = NewPath ;
	NewPath->prev = pPath ;
	return PathHead ;
}

/////////////////////////////////////////////////////////////////
//路径变换函数
////////////////////////////////////////////////////////////////
int GetLocalFrame( CPathArc3D* pArc, RFRAME& locMtx )
{
	mathInitRFrame( &locMtx ) ;
	memcpy( locMtx.O, pArc->m_fCenter, sizeof( PNT3D) ) ;
	if( pArc->m_bArcPlane == 3 )
	{ // 用户定义
		if( ! pArc->m_fXYZ ) return 0 ;
		memcpy( locMtx.X, pArc->m_fXYZ[0], sizeof( VEC3D) ) ;
		memcpy( locMtx.Y, pArc->m_fXYZ[1], sizeof( VEC3D) ) ;
		memcpy( locMtx.Z, pArc->m_fXYZ[2], sizeof( VEC3D) ) ;
	}
	else if( pArc->m_bArcPlane == 0 ) 
	{//XY
		locMtx.X[0] = 1.0,locMtx.X[1] = 0.0,locMtx.X[2] = 0.0 ;
		locMtx.Y[0] = 0.0,locMtx.Y[1] = 1.0,locMtx.Y[2] = 0.0 ;
		locMtx.Z[0] = 0.0,locMtx.Z[1] = 0.0,locMtx.Z[2] = 1.0 ;
	}
	else if( pArc->m_bArcPlane == 1 ) 
	{//YZ
		locMtx.X[0] = 0.0,locMtx.X[1] = 1.0,locMtx.X[2] = 0.0 ;
		locMtx.Y[0] = 0.0,locMtx.Y[1] = 0.0,locMtx.Y[2] = 1.0 ;
		locMtx.Z[0] = 1.0,locMtx.Z[1] = 0.0,locMtx.Z[2] = 0.0 ;
	}
	else if( pArc->m_bArcPlane == 2 )
	{//XZ
		locMtx.X[0] = 0.0,locMtx.X[1] = 0.0,locMtx.X[2] = 1.0 ;
		locMtx.Y[0] = 1.0,locMtx.Y[1] = 0.0,locMtx.Y[2] = 0.0 ;
		locMtx.Z[0] = 0.0,locMtx.Z[1] = 1.0,locMtx.Z[2] = 0.0 ;
	}
	else return 0 ;
	return 1 ;
}
void RotatePComb( CPathCombine* PComb, PNT3D ptbase, VEC3D dir, double angle)
{
	if( !PComb ) return ;
	CPathEntity *pEntity = NULL ;
	for( ; PComb ; PComb = PComb->next )
	{
		pEntity = PComb->m_pHead ;
		for( ; pEntity ; pEntity = pEntity->next )
		{
			int nType = pEntity->GetType() ;
			int i ;
			if( nType == NC_PATH_POINT3D )
			{// 钻孔
				CPathPoint3D* pDrill = ( CPathPoint3D* )pEntity ;
				mathRotPnt( dir, ptbase, angle, pDrill->m_fPoint, pDrill->m_fPoint ) ;	
				for( i = 0 ; i < pDrill->m_nIncTime ; i++ ) 
				{
					mathRotPnt( dir, ptbase, angle, pDrill->m_fIncAt[i], pDrill->m_fIncAt[i] ) ;	
				}
			}
			else if( nType == NC_PATH_ARC3D )
			{// 圆弧
				CPathArc3D* pArc = ( CPathArc3D* )pEntity ;
				RFRAME  locMtx ; 
				if( ! GetLocalFrame( pArc, locMtx ) ) return  ;
				mathRotateRFrame( ptbase, dir, angle, &locMtx ) ;
				pArc->m_bArcPlane = 3 ;
				if( ! pArc->m_fXYZ )  pArc->m_fXYZ = new VEC3D[3] ;
				mathCpyPnt( locMtx.O, pArc->m_fCenter ) ;
				mathCpyPnt( locMtx.X, pArc->m_fXYZ[0] ) ;
				mathCpyPnt( locMtx.Y, pArc->m_fXYZ[1] ) ;
				mathCpyPnt( locMtx.Z, pArc->m_fXYZ[2] ) ;	
			}
			else if( nType == NC_PATH_LINE3D )
			{// 直线
				CPathLine3D* pLine = ( CPathLine3D* )pEntity ;
				mathRotPnt( dir, ptbase, angle, pLine->m_fStart, pLine->m_fStart ) ;
				mathRotPnt( dir, ptbase, angle, pLine->m_fEnd  , pLine->m_fEnd   ) ;
			}
			else if( nType == NC_PATH_PLINE3D )
			{// 折线
				CPathPLine3D* pPLine = ( CPathPLine3D* )pEntity ;
				for( i = 0 ; i <= pPLine->m_nCount ; i++ )
				{
					mathRotPnt( dir, ptbase, angle, pPLine->m_pTAPos[i], pPLine->m_pTAPos[i] ) ;
				}
			}

		}

	}
}
void MirrorPComb( CPathCombine* PComb, PNT3D ptbase, VEC3D dir )
{
	if( !PComb ) return ;
	CPathEntity *pEntity = NULL ;
	for( ; PComb ; PComb = PComb->next )
	{
		pEntity = PComb->m_pHead ;
		for( ; pEntity ; pEntity = pEntity->next )
		{
			int nType = pEntity->GetType() ;
			int i ;
			if( nType == NC_PATH_POINT3D )
			{// 钻孔
				CPathPoint3D* pDrill = ( CPathPoint3D* )pEntity ;
				mathMirPnt( ptbase, dir, pDrill->m_fPoint, pDrill->m_fPoint ) ;	
				for( i = 0 ; i < pDrill->m_nIncTime ; i++ ) 
				{
					mathMirPnt( ptbase, dir, pDrill->m_fIncAt[i], pDrill->m_fIncAt[i] ) ;	
				}
			}
			else if( nType == NC_PATH_ARC3D )
			{// 圆弧
				CPathArc3D* pArc = ( CPathArc3D* )pEntity ;
				RFRAME  locMtx ; 
				if( ! GetLocalFrame( pArc, locMtx ) ) return  ;
				if( ! pArc->m_fXYZ )  pArc->m_fXYZ = new VEC3D[3] ;
				for( int k = 0 ; k < 3 ; k ++ ) 
				{
					pArc->m_fCenter[k] += pArc->m_fDepth * locMtx.Z[k] ;
				}
				pArc->m_bArcPlane = 3 ;
				mathMirPnt(ptbase, dir, locMtx.O, pArc->m_fCenter) ;
				mathMirVec(ptbase, dir, locMtx.X, pArc->m_fXYZ[0]) ;
				mathMirVec(ptbase, dir, locMtx.Y, pArc->m_fXYZ[1]) ;
				mathVProductUnit(pArc->m_fXYZ[0], pArc->m_fXYZ[1], pArc->m_fXYZ[2]);
				pArc->m_fDepth = - pArc->m_fDepth ;
			}
			else if( nType == NC_PATH_LINE3D )
			{// 直线
				CPathLine3D* pLine = ( CPathLine3D* )pEntity ;
				mathMirPnt( ptbase, dir, pLine->m_fStart, pLine->m_fStart ) ;
				mathMirPnt( ptbase, dir, pLine->m_fEnd  , pLine->m_fEnd   ) ;
			}
			else if( nType == NC_PATH_PLINE3D )
			{// 折线
				CPathPLine3D* pPLine = ( CPathPLine3D* )pEntity ;
				for( i = 0 ; i <= pPLine->m_nCount ; i++ )
				{
					mathMirPnt( ptbase, dir, pPLine->m_pTAPos[i], pPLine->m_pTAPos[i] ) ;
				}
			}

		}

	}

}
void ScalePComb( CPathCombine* PComb, PNT3D ptbase, VEC3D scale)
{
	if( !PComb ) return ;
	CPathEntity *pEntity = NULL ;
	for( ; PComb ; PComb = PComb->next )
	{
		pEntity = PComb->m_pHead ;
		for( ; pEntity ; pEntity = pEntity->next )
		{
			int nType = pEntity->GetType() ;
			int i ;
			if( nType == NC_PATH_POINT3D )
			{// 钻孔
				CPathPoint3D* pDrill = ( CPathPoint3D* )pEntity ;
				mathScalePnt( ptbase, scale, pDrill->m_fPoint, pDrill->m_fPoint ) ;	
				for( i = 0 ; i < pDrill->m_nIncTime ; i++ ) 
				{
					mathScalePnt( ptbase, scale, pDrill->m_fIncAt[i], pDrill->m_fIncAt[i] ) ;	
				}
			}
			else if( nType == NC_PATH_ARC3D )
			{// 圆弧
				CPathArc3D* pArc = ( CPathArc3D* )pEntity ;
				mathScalePnt( ptbase,scale, pArc->m_fCenter, pArc->m_fCenter ) ;
				pArc->m_fRadius *= scale[0] ;
				pArc->m_fDepth *= scale[2]  ;
			}
			else if( nType == NC_PATH_LINE3D )
			{// 直线
				CPathLine3D* pLine = ( CPathLine3D* )pEntity ;
				mathScalePnt( ptbase, scale, pLine->m_fStart, pLine->m_fStart ) ;
				mathScalePnt( ptbase, scale, pLine->m_fEnd  , pLine->m_fEnd   ) ;
			}
			else if( nType == NC_PATH_PLINE3D )
			{// 折线
				CPathPLine3D* pPLine = ( CPathPLine3D* )pEntity ;
				for( i = 0 ; i <= pPLine->m_nCount ; i++ )
				{
					mathScalePnt( ptbase, scale, pPLine->m_pTAPos[i], pPLine->m_pTAPos[i] ) ;
				}
			}

		}

	}

}
void TransfPath( CPathGroup& NewPath, JDNC_SPACETRAN cSpaceDef )
{
	BOOL bKeep = cSpaceDef.m_bSpacetranFlag & NCDEF_SPACETRAN_KEEPORIG;
	int nType = cSpaceDef.m_nSpacetranType;
	int nNum = cSpaceDef.m_nNum;
	int i, j; 
	CPathCombine* pNewPComb = NULL;
	if(nType == 1)
	{//平移
		VEC3D vecMove;
		for(i=0; i<3; i++)
		{
			vecMove[i] = cSpaceDef.m_vecMove[i];
		}
		for(j=1; j<=nNum; j++)
		{
			for( CPathCombine* pPComb = NewPath.m_pHead; pPComb ; pPComb = pPComb->next )
			{
				pNewPComb = pPComb->CopyMyself();
				pNewPComb->MoveCombine(vecMove[0], vecMove[1], vecMove[2]);
			}
			NewPath.AddData(0.0, pNewPComb);
		}
	}
	else if(nType == 2)
	{//旋转
		PNT3D pntAxis;
		VEC3D dirAxis;
		DOUBLE dAngle = cSpaceDef.m_dAngle;
		dAngle = ANGLE_TO_RADIAN(dAngle) ;
		BOOL bTotalAng = cSpaceDef.m_bSpacetranFlag & NCDEF_SPACETRAN_TOTALANG;
		for(i=0; i<3; i++)
		{
			pntAxis[i] = cSpaceDef.m_pntAxis[i];
			dirAxis[i] = cSpaceDef.m_dirAxis[i];
		}
		for(j=1; j<=nNum; j++)
		{
			for( CPathCombine* pPComb = NewPath.m_pHead; pPComb ; pPComb = pPComb->next )
			{
				pNewPComb = pPComb->CopyMyself();
				if(bTotalAng)
				{//均分旋转角度
					RotatePComb( pNewPComb, pntAxis, dirAxis, dAngle/nNum );
				}
				else
				{
					RotatePComb( pNewPComb, pntAxis, dirAxis, dAngle );	
				}	
			}
			NewPath.AddData(0.0, pNewPComb);	
		}
	}
	else if(nType == 3)
	{//镜像
		PNT3D pntBase;
		VEC3D dirNormal;
		for(i=0; i<3; i++)
		{
			pntBase[i] = cSpaceDef.m_dPivot[i];
			dirNormal[i] = cSpaceDef.m_dNormal[i];
		}
		for( CPathCombine* pPComb = NewPath.m_pHead; pPComb ; pPComb = pPComb->next )
		{
			pNewPComb = pPComb->CopyMyself();
			MirrorPComb(pNewPComb, pntBase, dirNormal);
		}
		NewPath.AddData(0.0, pNewPComb);
	}
	else if(nType == 4)
	{//缩放
		PNT3D pntBase;
		VEC3D vecFactor;
		for(i=0; i<3; i++)
		{
			pntBase[i] = cSpaceDef.m_pntBase[i] ;
			vecFactor[i] = cSpaceDef.m_dFactor[i]/100.0 ;
		}
		for( CPathCombine* pPComb = NewPath.m_pHead; pPComb ; pPComb = pPComb->next )
		{
			pNewPComb = pPComb->CopyMyself();
			ScalePComb(pNewPComb, pntBase, vecFactor);
		}
		NewPath.AddData(0.0, pNewPComb);
	}
	else
	{
	}
	if(!bKeep)
	{
		CPathCombine* pHComb = NewPath.m_pHead;
		NewPath.RemoveCombine(pHComb);
	}
}
void TransLocalGroup( CPathGroup &NewPath, RFRAME *Local )
{
	if( !Local ) return ;
	CPathCombine *pHead = NewPath.m_pHead ;
	CPathEntity *pEntity = NULL ;
	int nType = 0, i = 0 ;
	CPathPoint3D *pPoint = NULL ;
	CPathArc3D *pArc = NULL ;
	CPathLine3D *pLine = NULL ;
	CPathPLine3D *pLine3D = NULL ;
	for( ; pHead ; pHead = pHead->next )
	{
		pEntity = pHead->m_pHead ;
		for( ; pEntity ; pEntity = pEntity->next )
		{
			nType = pEntity->GetType () ;
			if( nType == NC_PATH_PLINE3D )
			{
				pLine3D = ( CPathPLine3D *)pEntity ;
				for( i = 0 ; i <= pLine3D->m_nCount ; i++ )
				{
					mathTransLocalPnt3D( Local, pLine3D->m_pTAPos[i], pLine3D->m_pTAPos[i] ) ;
				}
			}
			else if( nType == NC_PATH_LINE3D )
			{
				pLine = ( CPathLine3D *)pEntity ;
				mathTransLocalPnt3D( Local, pLine->m_fStart, pLine->m_fStart ) ;
				mathTransLocalPnt3D( Local, pLine->m_fEnd  , pLine->m_fEnd   ) ;
			}
			else if( nType == NC_PATH_ARC3D )
			{
				pArc = ( CPathArc3D *)pEntity ;
				RFRAME lf ;
				mathInitRFrame( &lf ) ;
				if( !GetLocalFrame( pArc, lf ) ) continue ;
				if( !pArc->m_fXYZ ) pArc->m_fXYZ = new VEC3D[3] ;
				for( int i = 0 ; i < 3 ; i++ )
				{
					pArc->m_fCenter[i] += pArc->m_fDepth * lf.Z[i] ;
				}
				pArc->m_bArcPlane = 3 ;	
				mathTransLocalPnt3D( Local, lf.O, pArc->m_fCenter ) ;
				mathTransLocalVec3D( Local, lf.X, pArc->m_fXYZ[0] ) ;
				mathTransLocalVec3D( Local, lf.Y, pArc->m_fXYZ[1] ) ;
				mathVProductUnit( pArc->m_fXYZ[0], pArc->m_fXYZ[1], pArc->m_fXYZ[2] ) ;
				pArc->m_fDepth = -pArc->m_fDepth ;
				/*
				if( !local_frame )return ;
				RFRAME  locMtx ; 
				if( ! GetLocalFrame( locMtx ) ) return  ;
				if( ! m_fXYZ )  m_fXYZ = new VEC3D[3] ;
				for( int k = 0 ; k < 3 ; k ++ ) 
				{
					m_fCenter[k] += m_fDepth * locMtx.Z[k] ;
				}
				m_bArcPlane = NC_ARCPLANE_USER ;
				mathTransLocalPnt3D( local_frame, locMtx.O, m_fCenter ) ;
				mathTransLocalVec3D( local_frame, locMtx.X, m_fXYZ[0] ) ;
				mathTransLocalVec3D( local_frame, locMtx.Y, m_fXYZ[1] ) ;
				mathVProductUnit(m_fXYZ[0], m_fXYZ[1], m_fXYZ[2]);
 				m_fDepth = - m_fDepth ;
				*/
			}
			else if( nType == NC_PATH_POINT3D )
			{
				pPoint = ( CPathPoint3D *)pEntity ;
				mathTransLocalPnt3D( Local, pPoint->m_fPoint, pPoint->m_fPoint ) ;
				for( i = 0 ; i < pPoint->m_nIncTime ; i++ )
				{
					mathTransLocalPnt3D( Local, pPoint->m_fIncAt[i], pPoint->m_fIncAt[i] ) ;
				}
			}
			else
			{
				ASSERT( 0 ) ;
			}
		}
	}
}
///////////////////////////////////////////////////////////////
// class CBtwLPath
CBtwLPath::CBtwLPath( CSmtLoopPath *pLPath )
{
	m_pLPath = pLPath	;
	m_bSearch3 = FALSE	; 
	n_UnderDepth = -1	;
}
CBtwLPath::~CBtwLPath()
{
	if( m_pLPath )
	{
		MathCam_DeleteLPath( m_pLPath ) ;
	}
	m_UnderLoops.RemoveAll () ;
}
void CBtwLPath::AddUnderLoop(CSmartLoop * pLoop )
{
    if( !pLoop ) return ;
    CSmartLoop * pTmpLoop = NULL ;
    BOOL bAdd = TRUE ;
    for ( int i=0; i < m_UnderLoops.GetSize(); i++ )
    {
        pTmpLoop = m_UnderLoops.GetAt( i ) ;
        if ( pTmpLoop == pLoop )
        {
            bAdd = FALSE ;
            break ;
        }
    }
    if ( bAdd )
    {
        m_UnderLoops.Add( pLoop ) ;
    }
 }

int GetLoopCount( CSmartLoop * pLoop )
{
   if( !pLoop ) return 0 ;
   int nCount = 0; 
   CSmartLoop * pTmpLoop = pLoop ;
   while ( pTmpLoop )
   {
       nCount ++ ;
       pTmpLoop = pTmpLoop->next ;
   }
   return nCount ;
}

// 层间精加工使用，用两轮廓修剪环
BOOL TrimLoopByTwoContours( CSmartLoop * pLoop, CSmartLoop *pContour1, CSmartLoop *pContour2, CSmartCurveLib& CurLib )
{
	if( !pLoop || !pContour1 || !pContour2 ) return FALSE ;

	CSmartCurveLib tmpLib ;
	// STEP0 : 轮廓初始化, 轮廓求交
	CSmartLoop *pLoopA = NULL, *pLoopB = NULL,  *pIsland = NULL ;
	for( pLoopA = pLoop ; pLoopA ; pLoopA = pLoopA->next )
	{
		pLoopA->InitBuffer() ;
	}
	for( pLoopB = pContour1 ; pLoopB ; pLoopB = pLoopB->next )
	{
		pLoopB->InitBuffer() ;
	}
	for( pLoopA = pLoop ; pLoopA ; pLoopA = pLoopA->next )
	{
		for( pLoopB = pContour1 ; pLoopB ; pLoopB = pLoopB->next )
		{
			pLoopA->ContourContourInt( pLoopB );
		}
	}
	// STEP1 使用pContour1对环进行修剪，保留pContour1内的部分
	CSmartCurve *pCurveTemp, *pCurveHead ;
	PNT2D dMidPnt ;
	int bIntFlag = 0 ;
	for( pLoopA = pLoop ; pLoopA ; pLoopA = pLoopA->next )
	{
		pCurveHead = pLoopA->ContourBreak();
		for(  ; pCurveHead  ; pCurveHead = pCurveTemp )
		{
			pCurveTemp = pCurveHead->next ;
			pCurveHead->next = NULL ;
			pCurveHead->GetMidPoint(dMidPnt);
			bIntFlag = Mini_IsPointOnContours( pContour1, dMidPnt ) ;
			if( bIntFlag >0 )   //(落在轮廓1内)    
			{ // 保留曲线,保留重线
				tmpLib.AddCurves ( pCurveHead ) ;
			}
			else 
			{
				delete pCurveHead ;
			}
		} 
	}

	// STEP2 : 使用pContour2对得到的曲线进行裁减,保留pContour2中的部分
	for( pLoopB = pContour2 ; pLoopB ; pLoopB = pLoopB->next )
	{
		pLoopB->InitBuffer() ;
	}
	CSmartCurve *pHead = tmpLib.m_pHead, *pNext = NULL ;
	for( ; pHead ; pHead = pHead->next )
	{
		pHead->DefineBox () ;
		for( pLoopB = pContour2 ; pLoopB ; pLoopB = pLoopB->next )
		{
			pHead->CurveCurveInt ( pLoopB->m_pCurve ) ;
			for( pIsland = pLoopB->GetIsland () ; pIsland ; pIsland = pIsland->next )
			{
				pHead->CurveCurveInt ( pIsland->m_pCurve ) ;
			}
		}
	}
	for( pHead = tmpLib.m_pHead ; pHead ; pHead = pHead->next )
	{
		pCurveHead = pHead->BreakCurve () ;
		for(  ; pCurveHead  ; pCurveHead = pCurveTemp )
		{
			pCurveTemp = pCurveHead->next ;
			pCurveHead->next = NULL ;
			pCurveHead->GetMidPoint(dMidPnt);
			bIntFlag = Mini_IsPointOnContours( pContour2, dMidPnt ) ;
			if( bIntFlag > 0 )   //(落在轮廓2内)    
			{ // 保留曲线,保留重线
				CurLib.AddCurves ( pCurveHead ) ;
			}
			else 
			{
				delete pCurveHead ;
			}
		} 
	}
	tmpLib.DeleteAllCurves () ;
	// STEP3 : 连接曲线并删除特别小的
	CurLib.ConnectCurve ( 0.0001 ) ;
	pHead = CurLib.m_pHead ;
	while( pHead )
	{
		pNext = pHead->next ;
		pHead->DefineBox () ;
		if( pHead->GetLength () < 0.02 )
		{
			CurLib.DeleteCurve ( pHead ) ;
		}
		pHead = pNext ;
	}
	if( CurLib.m_nNumCurve < 1 ) return FALSE ;	
	return TRUE ;
}

void CBtwCurve::AddUnderLoop( CSmartLoop * pLoop )
{
	if( !pLoop ) return ;
	CSmartLoop * pTmpLoop = NULL ;
	BOOL bAdd = TRUE ;
	for ( int i=0; i < m_UnderLoops.GetSize(); i++ )
	{
		pTmpLoop = m_UnderLoops.GetAt( i ) ;
		if ( pTmpLoop == pLoop )
		{
			bAdd = FALSE ;
			break ;
		}
	}
	if ( bAdd )
	{
		m_UnderLoops.Add( pLoop ) ;
	}
}

double  MinDistBetweenCurveAndSect( CSmartCurve * pCurve, CSmartSect * pSect )
{
	double dMin = 1.0e10, dDist = 0 ;
	CSmartSect *pHead = pCurve->m_pHead ;
	for( ; pHead ; pHead = pHead->next )
	{
		dDist = pHead->MinDistBetween ( pSect ) ;
		if( dDist < dMin )
		{
			dMin = dDist ;
		}
	}
	return dMin ;
}
double  MinDistBetweenCurve( CSmartCurve * pCurve1, CSmartCurve * pCurve2 )
{
	double dMin = 1.0e10, dDist = 0 ;
	CSmartSect *pHead = pCurve1->m_pHead ;
	for( ; pHead ; pHead = pHead->next )
	{
		dDist = MinDistBetweenCurveAndSect( pCurve2, pHead ) ;
		if( dDist < dMin )
		{
			dMin = dDist ;
		}
	}
	return dMin ;
}
double MinDistBetweenLoopAndCurve( CSmartLoop * pLoop, CSmartCurve * pCurve )
{
	double dMin = 1.0e10, dDist = 0 ;
	CSmartCurve *pHead = pLoop->m_pCurve ;
	for( ; pHead ; pHead = pHead->next )
	{
		dDist = MinDistBetweenCurve( pHead, pCurve ) ;
		if( dDist < dMin )
		{
			dMin = dDist ;
		}
	}
	CSmartLoop * pIsland = pLoop->m_pIsland ;
	while( pIsland )
	{
		pHead = pIsland->m_pCurve ;
		for ( ; pHead; pHead = pHead->next )
		{
			dDist = MinDistBetweenCurve( pHead, pCurve ) ;
			if ( dDist < dMin )
			{
				dMin = dDist ;
			}
		}
		pIsland = pIsland->next ;
	}
	return dMin ;
}

BOOL ChkBox3DInt(BOX3D* box1, BOX3D* box2, double tol)
{
	if( box1->min[0] > box2->max[0]+tol ||
		box1->max[0] < box2->min[0]-tol ||
		box1->min[1] > box2->max[1]+tol ||
		box1->max[1] < box2->min[1]-tol )
		return FALSE ;

	return TRUE ;
}

//判断两个轮廓是否相交
// 说明:CSmartLoop::ContourContourInt()要求出所有交点,很费时,
// 而这里只需要判断是否相交
BOOL Is2ContoursInt( CSmartLoop * pLoop1, CSmartLoop * pLoop2, double dTol )
{
	if( !pLoop1 || !pLoop2 ) return FALSE ;
	//(1)包围盒粗判
	BOX3D box1, box2 ;
	MathCam_GetLoopBox( pLoop1, &box1) ;
	MathCam_GetLoopBox( pLoop2, &box2) ;
	if( !ChkBox3DInt( &box1, &box2, dTol ) ) return FALSE ;
	//(2)判断pLoop1外环与pLoop2的距离
	double dDist = 0. ;
	CSmartLoop *pIsland1 = NULL, *pIsland2 = NULL ;
	CSmartSect * pSectHead1 = NULL, *pSectHead2 = NULL ;
	pSectHead1 = pLoop1->GetSectHead() ;
	for ( ; pSectHead1; pSectHead1 = pSectHead1->next )
	{
		pSectHead2 = pLoop2->GetSectHead() ;
		for ( ; pSectHead2; pSectHead2 = pSectHead2->next )
		{
			dDist = pSectHead1->MinDistBetween( pSectHead2 ) ;
			if( dDist < dTol ) return TRUE ;
		}
		pIsland2 = pLoop2->GetIsland() ;
		for ( ; pIsland2; pIsland2 = pIsland2->next )
		{
			pSectHead2 = pIsland2->GetSectHead() ;
			for ( ; pSectHead2; pSectHead2 = pSectHead2->next )
			{
				dDist = pSectHead1->MinDistBetween( pSectHead2 ) ;
				if( dDist < dTol ) return TRUE ;
			}
		}
	}
	//(3)判断pLoop1孤岛与pLoop2的距离
	pIsland1 = pLoop1->GetIsland() ;
	for ( ; pIsland1; pIsland1 = pIsland1->next )
	{
		pSectHead1 = pIsland1->GetSectHead() ;
		for ( ; pSectHead1; pSectHead1 = pSectHead1->next )
		{
			pSectHead2 = pLoop2->GetSectHead() ;
			for ( ; pSectHead2; pSectHead2 = pSectHead2->next )
			{
				dDist = pSectHead1->MinDistBetween( pSectHead2 ) ;
				if( dDist < dTol ) return TRUE ;
			}
			pIsland2 = pLoop2->GetIsland() ;
			for ( ; pIsland2; pIsland2 = pIsland2->next )
			{
				pSectHead2 = pIsland2->GetSectHead() ;
				for ( ; pSectHead2; pSectHead2 = pSectHead2->next )
				{
					dDist = pSectHead1->MinDistBetween( pSectHead2 ) ;
					if( dDist < dTol ) return TRUE ;
				}
			}
		}
	}
	return FALSE ;
}

//判断两个轮廓是否在同一区域内
BOOL IsInSameArea( CSmartLoop * pLoop1, CSmartLoop * pLoop2, double dTol )
{
	if( !pLoop1 || !pLoop2 ) return FALSE ;
	//(1)判断是否相交
	if( Is2ContoursInt( pLoop1, pLoop2, dTol ) ) return TRUE ;
	//(2)判断是否落在另一个区域内
	CSmartLoop *pIsland = NULL ;
	PNT2D point ;
	pLoop1->m_pCurve->GetStart( point ) ;
	if ( pLoop2->IsPointIn( point ) )
	{
		pIsland = pLoop2->GetIsland() ;
		for ( ; pIsland; pIsland = pIsland->next )
		{
			if( pIsland->IsPointIn( point ) )  return FALSE ;
		}
		return TRUE ;
	}
	pLoop2->m_pCurve->GetStart( point ) ;
	if ( pLoop1->IsPointIn( point ) )
	{
		pIsland = pLoop1->GetIsland() ;
		for ( ; pIsland; pIsland = pIsland->next )
		{
			if( pIsland->IsPointIn( point ) ) return FALSE ;
		}
		return TRUE ;
	}
	return FALSE ;
}

//把pNewLPath插入链表pLPathHead中
void InsertLPath( CSmtLoopPath * pNewLPath, CSmtLoopPath * &pLPathHead )
{
	if( !pNewLPath || !pLPathHead ) return ;
	INT nDepth = pNewLPath->m_pLoop->m_nDepth ;
	CSmtLoopPath* pLPath1 = pLPathHead ;
	CSmtLoopPath* pLPath2 = NULL ;
	while ( pLPath1 &&  pLPath1->m_pLoop->m_nDepth > nDepth )
	{
		pLPath2 = pLPath1 ;
		pLPath1 = pLPath1->next ;
	}
	if ( pLPath2 == NULL)
	{
		pLPathHead->prev = pNewLPath ;
		pNewLPath->next = pLPathHead ;
		pLPathHead = pNewLPath ;
	}
	else
	{
		if ( pLPath1 == NULL )
		{
			pLPath2->next = pNewLPath ;
			pNewLPath->prev = pLPath2 ;
		} 
		else
		{
			pLPath2->next = pNewLPath ;
			pNewLPath->prev = pLPath2 ;
			pNewLPath->next = pLPath1 ;
			pLPath1->prev = pNewLPath ;
		}	                
	}
}
//判断环pLoop和等高环链表pDriveLoop是否相交
//其中pLoop为孤立环
BOOL IsRelatedLoop( CSmartLoop *pLoop, CSmartLoop *pDriveLoop, double dTol )
{
	if( !pLoop || !pDriveLoop ) return FALSE ;
	while ( pDriveLoop )
	{
		if( Is2ContoursInt( pLoop, pDriveLoop, dTol ) ) 
			return TRUE ;
		pDriveLoop = pDriveLoop->next ;
	}
	return FALSE ;
}

//生成连接两条曲线的曲线
CSmartCurve *  CreateConnectCurve( CSmartCurve * pCurve, CSmartCurve * pPrev, double dDist )
{
	if( !pCurve || !pPrev ) return NULL ;
	//STEP 1: 得到两曲线的起点,切矢和法向
	PNT3D pos1 = { 0.0, 0.0, 0.0 }, pos2 = { 0.0, 0.0, 0.0 }, pos2_2 = { 0.0, 0.0, 0.0 }, 
		  tang1 = { 0.0, 0.0, 0.0 }, tang2 = { 0.0, 0.0, 0.0 }, tang2_2 = { 0.0, 0.0, 0.0 }, 
		  nor1 = { 0.0, 0.0, 0.0 }, nor2 = { 0.0, 0.0, 0.0 }, nor2_2 = { 0.0, 0.0, 0.0 } ;
	pPrev->m_pTail->GetEnd ( pos1 ) ;
	pPrev->m_pTail->GetTangent ( 1, tang1 ) ;
	pPrev->m_pTail->GetNormal ( 1, nor1 ) ;
	pCurve->m_pHead->GetStart ( pos2 ) ;	
	pCurve->m_pHead->GetTangent( 0, tang2 ) ;
	pCurve->m_pHead->GetNormal ( 0, nor2 ) ;
	pCurve->m_pTail->GetEnd( pos2_2 ) ;
	pCurve->m_pTail->GetTangent( 1.0, tang2_2 ) ;
	pCurve->m_pTail->GetNormal( 1.0, nor2_2 ) ;
	//STEP 2: 调整法线方向
	VEC3D vDir = { 0.0, 0.0, 0.0 } ;
	PNT3D tmpPos = { 0.0, 0.0, 0.0 } ;
	if ( FindNearestPnt( pCurve, pos1, tmpPos, dDist, FALSE ) )
	{
		if( mathGetVecByPP2D( pos1, tmpPos, MIN_LEN, vDir ) == ERUNSUC )
		{
			return NULL ;
		}
		if ( mathGetAngleUnit( nor1, vDir ) > PI1_2 )
		{
			mathRevVec( nor1 ) ;
		}
	}
	else
	{
		return NULL ;
	}
	if ( FindNearestPnt( pPrev, pos2, tmpPos, dDist, TRUE ) )
	{
		if( mathGetVecByPP2D( pos2, tmpPos, MIN_LEN, vDir ) == ERUNSUC )
		{
			return NULL ;
		}
		if ( mathGetAngleUnit( nor2, vDir ) > PI1_2 )
		{
			mathRevVec( nor2 ) ;
		}
	} 
	else
	{
		return NULL ;
	}
	if ( FindNearestPnt( pPrev, pos2_2, tmpPos, dDist, TRUE ) )
	{
		if ( mathGetVecByPP2D( pos2_2, tmpPos, MIN_LEN, vDir ) == ERUNSUC )
		{
			return NULL ;
		}
		if ( mathGetAngleUnit( nor2_2, vDir ) > PI1_2 )
		{
			mathRevVec( nor2_2 ) ;
		}
	} 
	else
	{
		return NULL ;
	}
	double dAng = mathGetAngleUnit( nor2, nor2_2 ) ;
	if ( dAng > ANGLE_TO_RADIAN( 3.0 ) )
	{
		PNT3D tmpNor = { 0.0, 0.0, 0.0 } ;
		if( mathVProductUnit( tang1, nor1, vDir ) == ERUNSUC)
		{
			return NULL ;
		}
		mathRotVec( vDir, pos2, dAng, nor2_2, tmpNor ) ;
		dAng = mathGetAngleUnit( nor2, tmpNor ) ;
		if ( dAng < ANGLE_TO_RADIAN( 3.0 ) )
		{
			nc_VectorCopy( pos2, pos2_2, 3 ) ;
			nc_VectorCopy( tang2, tang2_2, 3 ) ;
			nc_VectorCopy( nor2, nor2_2, 3 ) ;
		}
	}
	//STEP 3: 求两个圆的圆心,和圆心线的中点
	INT i ;
	double dr = min( dDist*0.2, 0.2 ) ;
	double dMindr = max( dr - 0.005, 0.01 ) ;
	PNT2D cen1, cen2 ;
	for ( i = 0; i < 2; i++ )
	{
		cen1[i] = pos1[i] + dr * nor1[i] ;
		cen2[i] = pos2[i] + dr * nor2[i] ;
	}
	PNT3D bgn ;
	bgn[0] = ( cen1[0] + cen2[0] ) / 2 ;
	bgn[1] = ( cen1[1] + cen2[1] ) / 2 ;
	bgn[2] = ( pos1[2] + pos2[2] ) / 2 ;
	//STEP 4:  添加圆弧连接
	//(1)生成第一条圆弧
	if( mathGetVecByPP2D( pos1, bgn, MIN_LEN, vDir ) == ERUNSUC )
		return NULL ;
	if( mathGetAngleUnit( nor1, vDir )  > PI1_2 * 5/6 )
	{
		return NULL ;
	}
	double dTmp = mathDist2D( cen1, bgn ) ;
	double tanDist =  dTmp * dTmp - dr * dr ;
	ASSERT( tanDist >0 ) ;
	if( tanDist < 0 ) return NULL ;
	tanDist = sqrt( tanDist ) ;
	VEC3D vz, rot_tmpV, tmpV = { 0., 0.} ;
	if( mathVProductUnit( tang1, nor1, vz ) == ERUNSUC)
	{
		return NULL ;
	}
	if( mathGetVecByPP2D( bgn, cen1, MIN_LEN, tmpV ) == ERUNSUC )
	{
		return NULL ;
	}
	double tanAng = asin( dr / dTmp ) ;
	mathRotVec( vz, bgn, tanAng, tmpV, rot_tmpV ) ;
	PNT2D start1, mid1, end1 ;
	mathCpyPnt2D( pos1, start1 ) ;
	for( i = 0 ; i < 2 ; i++ )
	{
		end1[i] = bgn[i] + rot_tmpV[i] * tanDist ;
		mid1[i] = ( start1[i] + end1[i] ) * 0.5 ;
	}
	if( mathGetVecByPP2D( cen1, mid1, MIN_LEN, tmpV ) == ERUNSUC )
	{
		return NULL ;
	}
	for( i = 0 ; i < 2 ; i++ )
	{
		mid1[i] = cen1[i] + dr * tmpV[i] ;
	}
	CSmartArc *pArc1 = new CSmartArc() ;
	if ( !pArc1->Define3PArc( start1, mid1, end1, dMindr ) )
	{
		delete pArc1 ;
		return NULL ;
	}
	//(2)生成第二条圆弧
	if( mathGetVecByPP2D( pos2, bgn, MIN_LEN, vDir ) == ERUNSUC )
	{
		delete pArc1 ;
		return NULL ;
	}
	if( mathGetAngleUnit( nor2, vDir ) > PI1_2 * 5/6 )
	{
		delete pArc1 ;
		return NULL ;
	}
	dTmp = mathDist2D( cen2, bgn ) ;
	tanDist = dTmp * dTmp - dr * dr ;
	ASSERT( tanDist > 0 ) ; 
	if ( tanDist < 0 )
	{
		delete pArc1 ;
		return NULL ;
	}
	tanDist = sqrt( tanDist ) ;
	if( mathVProductUnit( tang2, nor2, vz ) == ERUNSUC )
	{
		delete pArc1 ;
		return NULL ;
	}
	if( mathGetVecByPP2D( bgn, cen2, MIN_LEN, tmpV ) == ERUNSUC )
	{
		delete pArc1 ;
		return NULL ;
	}
	tanAng = asin( dr / dTmp ) ;
	mathRotVec( vz, bgn, -tanAng, tmpV, rot_tmpV ) ;
	PNT2D start2, mid2, end2 ;
	mathCpyPnt2D( pos2, end2 ) ;
	for ( i = 0; i < 2; i++ )
	{
		start2[i] = bgn[i] + rot_tmpV[i] * tanDist ;
		mid2[i] = ( start2[i] + end2[i] ) * 0.5 ;
	}
	if( mathGetVecByPP2D( cen2, mid2, MIN_LEN, tmpV ) == ERUNSUC )
	{
		delete pArc1 ;
		return NULL ;
	}
	for( i = 0 ; i < 2 ; i++ )
	{
		mid2[i] = cen2[i] + dr * tmpV[i] ;
	}
	CSmartArc *pArc2 = new CSmartArc() ;
	if ( !pArc2->Define3PArc( start2, mid2, end2, dMindr ) )
	{
		delete pArc1 ;
		delete pArc2 ;
		return NULL ;
	}
	//(3)生成中间直线段
	CSmartLine *pLine ;
	pLine = new CSmartLine( end1, start2 ) ;
	CSmartCurve *LeadCurve = new CSmartCurve() ;
	LeadCurve->AddSect( pArc1 ) ;
	LeadCurve->AddSect( pLine ) ;
	LeadCurve->AddSect( pArc2 ) ;

	return LeadCurve ;
}

//把封闭曲线pCurve的起始点沿曲线方向偏移dDist
BOOL ResetCurveStart( CSmartCurve * pCurve, double dDist )
{
	if( !pCurve || !pCurve->IsClosed() ) return FALSE ;
	PNT2D start ;
	pCurve->GetStart( start ) ;
	double dLen = pCurve->GetLength() ;
	if( dLen/10 < dDist ) dDist = dLen / 10 ;
	PNT2D p, p1 ;
	pCurve->GetPointByLength( dDist, p ) ;
	CSmartSect *pSect = pCurve->GetHead() ;
	for( ; pSect ; pSect = pSect->next )
	{
		double dDist = pSect->MinDistPoint ( p, p1 ) ;
		if( dDist < 1.0e-3 )
		{
			mathCpyPnt2D( p1, p ) ;
			break ;
		}
	}
	pCurve->SetStartPoint( pSect, p ) ;
	return TRUE ;
}

//说明:
//(1)把pPrev方向调整和用其生成的路径方向一致；
//(2)把pCurve方向调整成和pPrev方向一致.
void AdjustCurveDir( CSmtCPathLib & cPathLib, CSmartCurve *pPrev, CSmartCurve *pCurve )
{
	if( !pPrev || !pCurve || cPathLib.GetNumPath()==0 ) return ;
	BOOL bClosed = pPrev->IsClosed() ;
	if( !bClosed )
	{//生成连刀线前先删除最后对一条开曲线生成的退刀路径
		CSmtCutPath * pPath = NULL ;
		POSITION pos = cPathLib.m_cAllPath.GetTailPosition () , atpos ;
		while( pos )
		{
			atpos = pos ;
			pPath = cPathLib.m_cAllPath.GetPrev ( pos ) ;
			if( pPath->GetCutMode () == MINI_CONNECT_PATH )
			{
				cPathLib.m_cAllPath.RemoveAt( atpos ) ;
				delete pPath ;
				pPath = NULL ;
			}
			else break ;
		}
		PNT3D start ;
		if( pPath )
		{
			GetSmtPathEndPnt( pPath, 1, start ) ;
		}
		PNT2D head ;
		pPrev->m_pTail->GetEnd( head ) ;
		double dDist = nc_Distance( start, head, 2 ) ;
		if ( dDist > 1.0e-2 )
		{
			pPrev->Reverse() ;
		}
	}
	
	if ( bClosed )
	{
		if ( ( pPrev->CalcArea() >0 ) != ( pCurve->CalcArea() > 0 ) )
		{
			pCurve->Reverse() ;
		}
	} 
	else
	{
		PNT3D tang1 = { 0.0, 0.0, 0.0 }, tang2 = { 0.0, 0.0, 0.0 } ;
		pPrev->m_pTail->GetTangent ( 1, tang1 ) ;
		pCurve->m_pTail->GetTangent ( 1, tang2 ) ;
		if ( mathGetAngleUnit( tang1, tang2 ) > PI1_2 )
		{
			pCurve->Reverse() ;
		}
	}
}

//查找点from到曲线pCurve上最近点to
BOOL FindNearestPnt( CSmartCurve *pCurve, PNT3D from, PNT3D to, double dTol, BOOL bFromHead )
{
	if( !pCurve ) return FALSE ;
	PNT3D tmpPos = { 0.0, 0.0, 0.0 } ;
	PNT2D end ;
	double dMinDist = 1.0e8, dDist ;
	CSmartSect *pSect = NULL;
	if ( bFromHead )
	{
		pSect = pCurve->GetHead() ;
	}
	else
	{
		pSect = pCurve->GetTail() ;
	}
	while( pSect )
	{
		dDist = pSect->MinDistPoint ( from, end ) ;
		if( dDist < dMinDist )
		{
			dMinDist = dDist ;
			mathCpyPnt2D( end, tmpPos ) ;
		}
		if( fabs( dDist - dTol ) <= 2.0e-2 )
		{
			break ;
		}
		if ( bFromHead )
		{
			pSect = pSect->next ;
		} 
		else
		{
			pSect = pSect->prev ;
		}
	}
	nc_VectorCopy( to, tmpPos, 3 ) ;
	return TRUE ;
}
// SmtFlowlineGen2.cpp: implementation of the CFlowlineGen class.
// 曲面精加工计算类CFlowlineGen的定义
//
//////////////////////////////////////////////////////////////////////
#include "stdafx.h"
#include "SurfGeo.H"
#include "Entity.h"
#include "SurfEtt.h"

#include "SmtPathGen3D.h"
#include "SmtFlowlineGen.h"
#include "mathcam.h"
#include "SmartCheck.h"
#include "SmtAutoFinishGen.H"
#include "Nc5DToolAxis.h"
#include "Nc5DSGuide.h"
#include "Nc5DPathTreat.h"
#include "Nc5DUWMesh.h"
#include "Convert.h"
#include "GeoSSInter.h"

#ifdef _DEBUG
#undef THIS_FILE
static char THIS_FILE[]=__FILE__;
#define new DEBUG_NEW
#endif

//////////////////////////////////////////////////////////////////////////
// 生成单张面的五轴曲面流线路径线程函数
static UINT MathCAM_CreateOne5AxSurfPathSubProc(LPVOID lpParam)
{
	if (lpParam == NULL)
		return 0;
	CO5SPT_DATA *pData = (CO5SPT_DATA *)lpParam;
	return pData->pFlowlineGen->CreateOne5AxSurfPathSubProc(pData->nUW, pData->uwStep, pData->DriveMdl, pData->bZigZag, 
		*pData->cRFrame, pData->pSurf, *pData->pAxisCtrl, pData->AllLoop, *pData->AllPath, pData->PrgDef, pData->nAtCore,pData->nCoreNum, pData->NewPath);
}

//////////////////////另外一种方法/////////////////////////////

// 生成参数域的步长
void CFlowlineGen::GetSurfStep1( C3DSurfAttArray& c3DSurfAttArr, 
								 double dOverStep, 
								 int nCutUW ) 
{
	C3DSurfAttribute* pSurf = NULL ;
	double maxDist = 0., len = 0. ;
	INT_PTR n = c3DSurfAttArr.GetSize () ;
	
	if( nCutUW == 1 )
		ChangePCurve( c3DSurfAttArr, FALSE ) ;
	// 得到w向中最长线段，计算最多行数
	double l1, l2 ;
	for( INT_PTR i = 0 ; i < n ; i++ )
	{
		pSurf = c3DSurfAttArr[i] ;
		if( i == 0 )
			l1 = 0. ;
		else 
			l1 = GetSurfBndLength( pSurf, 0 ) ;
		if( i == n-1 )
			l2 = 0. ;
		else
			l2 = GetSurfBndLength( pSurf, 1 ) ;
		len = max( l1, l2 ) ;

		if( len > maxDist ) maxDist = len ;
	}
	m_nNum = int ( maxDist / dOverStep + 1 ) ;
	// 开启磨削调整后，路径条数设置为3，取中间条
	if (m_bGrandMdfy)
	{
		m_nNum = 2;
	}
	

	PNT3D t1[2], t2[2], start[2], end[2] ;
	for( i = 0 ; i < n ; i++ )
	{
		pSurf = c3DSurfAttArr[i] ;
		pSurf->m_pPCurvStart = pSurf->m_pBoundStart->m_pPCurve ;
		pSurf->m_pPCurvEnd   = pSurf->m_pBoundEnd->m_pPCurve ;
		pSurf->m_pPCurvStart->GetEndPoint ( t1[0], t2[0] ) ;
		pSurf->m_pPCurvEnd->GetEndPoint ( t1[1], t2[1] ) ;
		pSurf->m_pBoundStart->m_pEdge->m_pCurve->GetEndPoint ( start[0], end[0] ) ;
		pSurf->m_pBoundEnd->m_pEdge->m_pCurve->GetEndPoint ( start[1], end[1] ) ;
	}
}

int CFlowlineGen::GenSurfPathLib1( CUWLineParam& cUWLine,
								   RFRAME& cRFrame    , 
								   JDNC_SETUP& Setup,
								   JDNC_FUWLINE& UWLineCut,
								   JDNC_PRGDEF &PrgDef )
{
	int nCutDir      = UWLineCut.m_nCutDir    ;
	double dOverStep = UWLineCut.m_dOverStep  ;
	C3DSurfAttribute* pSurfAtt = NULL ;
	BOOL bZigZag = ( UWLineCut.m_bUWLineFlag & NCDEF_UWLINE_ZIGZAG ) ? TRUE : FALSE ;
	// 存放共边面上的精确参数域上的点
	PNT4D *dStart = NULL, *dEnd = NULL ;
	if( nCutDir == 0 )
	{
		dStart = new PNT4D[m_nNum+1] ;
		dEnd   = new PNT4D[m_nNum+1] ;
	}

	CSmtCutPath *pPath = NULL ;
	INT_PTR n = cUWLine.m_cSurfAttArr.GetSize (), nNumLine = 0 ;
	for( INT_PTR i = 0 ; i < n ; i++ )
	{
		pSurfAtt = cUWLine.m_cSurfAttArr[i] ;
		if( !pSurfAtt ) 
		{
			MathCAM_MovePrgStep(PrgDef);
            continue ;
		}
		if( SurfNC_IsAbort() ) 
		{
			delete[] dStart, delete[] dEnd ;
			return FALSE ;
		}
	
		//生成参数域上的路径
		CSmtCPathLib AllPath ;
		POSITION pos ;
		CSmartPathGen cGen ;
		CSmartLoop* pLoop = cGen.ExtractSurfLoop( pSurfAtt->m_pSurf, 
			                                      Setup.m_cTolDef,
												  &cRFrame ) ;
		// u向生成路径
		if( nCutDir == 0 )
		{
			if( i == 0 )
			{
				if( !GenUParamPath( pSurfAtt, pLoop, AllPath.m_cAllPath, dStart, dEnd, bZigZag ) )
				{
					delete[] dStart, delete[] dEnd ;
					return FALSE ;
				}
			}
			else if( i == n-1 && cUWLine.m_IsClosed )
			{
				// 首先计算层号为0的路径
				GenRestUPath( pSurfAtt, pLoop, AllPath.m_cAllPath, TRUE, TRUE, bZigZag ) ;
				// 然后计算层号为1的路径
				// 首点
				GetParamPTList( cUWLine.m_cSurfAttArr[i-1], pSurfAtt, dEnd, TRUE ) ;
				// 末点
				GetParamPTList( cUWLine.m_cSurfAttArr[0], pSurfAtt, dStart, FALSE ) ;

				int nLineNo = 0;
				for( int j = 0 ; j <= m_nNum ; j++ )
				{
					if (m_bGrandMdfy && j != 1) continue;
					CSmtCutPath *pPath = new CSmtCutPath() ;
					AddPathPointEx( pPath, dEnd[j]   ) ;
					AddPathPointEx( pPath, dStart[j] ) ;
					AllPath.m_cAllPath.AddTail ( pPath ) ;

					pPath->m_nLayerNo = 1 ;
					pPath->m_nLineNo  = nLineNo ;
					nLineNo++;
				}
				// 最后计算层号为2的路径
				GenRestUPath( pSurfAtt, pLoop, AllPath.m_cAllPath, FALSE, TRUE, bZigZag ) ;
			}
			else
			{
				GetParamPTList( cUWLine.m_cSurfAttArr[i-1], pSurfAtt, dEnd, TRUE ) ;
				GenUParamPath( pSurfAtt, pLoop, AllPath.m_cAllPath, dEnd, bZigZag ) ;
			}
		}
		else
		{
			GenWParamPath( pSurfAtt, AllPath.m_cAllPath, dOverStep ) ;
		}

		if( nCutDir == 1 )
			ReverseLineOrder( AllPath.m_cAllPath ) ;
		// 如果是四边面并且可以螺旋
		if( pSurfAtt->m_bSpiral && IsQuadSurf( pSurfAtt->m_pSurf->m_pLoop ) &&
			m_cFeedDef.m_cConnectDef.m_bConnect3DFlag & NCDEF_FCONNECT3D_SPIRAL && !m_bGrandMdfy)  
			AdjustSpiralPath( AllPath ) ;
		// 进度条
		nNumLine = AllPath.m_cAllPath.GetCount () ;
		if( nNumLine < 1 ) nNumLine = 1 ;
		PrgDef.m_dIncStep = PrgDef.m_dTotalMove / nNumLine ;
		// PrgDef.m_dStepAt  = 0.0 ;
		PrgDef.m_dLimitAt = 1.0 ;
		//将参数域上的路径映射到空间域上
		int num = 0;
		pos = AllPath.m_cAllPath.GetHeadPosition () ;
		while( pos )
		{
			pPath = AllPath.m_cAllPath.GetNext ( pos ) ;

			//qqs 曲面磨削调整
			//该部分用于处理GenWParamPath生成的W向加工时，只生成中间段路径
			if (nCutDir == 1)
			{
				if (m_bGrandMdfy && num != 1) 
				{
					num++;
					pPath->ClearAllPoint();
					pSurfAtt->m_cSmtCPathLib.AddToTail( pPath ) ;
					continue;
				}
			}
			
			if (m_bGrandMdfy)
			{
				PNT4D			dPnt1={0.0, 0.0, 0.0, 0.0};
				PNT4D			dPnt2={0.0, 0.0, 0.0, 0.0};
				for (int m = 0; m < 2; m++)
				{
					dPnt1[m] = pPath->m_pHead->m_fPoint[m];
					dPnt2[m] = pPath->m_pTail->m_fPoint[m];
				}
				pPath->ClearAllPoint();
				GrindParamPathGen(pPath, UWLineCut, cRFrame, pSurfAtt->m_pSurf, nCutDir, Setup.m_cTolDef.m_dArcTol, dPnt1, dPnt2);
			}

			// 将参数域路径映射到曲面上
			TransfPathToSurf( pSurfAtt, m_pMiller, cRFrame, pPath );
			pSurfAtt->m_cSmtCPathLib.AddToTail( pPath ) ;
			MathCAM_MovePrgStep(PrgDef);
			num++;
		}
		AllPath.m_cAllPath.RemoveAll () ;
		// 将前一条路径中相同层号和列号的路径与第二个路径中的相连
		if( nCutDir == 0 && i > 0 )
			ConnectNearSurfPath( cUWLine.m_cSurfAttArr[i-1], pSurfAtt ) ;
		// 生成参数域曲线的路径 for test 
	/*	if( i == 0 )
		{
			double dArea = pLoop->Area () ;
			CSmartCurve* pHead = pLoop->m_pCurve ;
			for( ; pHead ; pHead = pHead->next )
			{
				CSmtCutPath* pPath = millface_CurveToCPath( *pHead, Setup.m_cTolDef ) ;
				pSurfAtt->m_cSmtCPathLib.AddToTail( pPath ) ;
			}
		}*/
		Mini_DeleteContours( pLoop ) ;
	}
	if( nCutDir == 0 )
	{
		delete[] dStart ;
		delete[] dEnd   ;
	}
	return 1;
}

BOOL CFlowlineGen::GenUParamPath ( C3DSurfAttribute* pSurfAtt,
								   CSmartLoop* pLoop,
								   CSmtCPathList& pathList,
								   PNT4D* dStart, PNT4D* dEnd, BOOL bZigZag )
{
	// STEP 0 : 初始化数据
	CGeoCurve *pCur1, *pCur2 ;
	CGeoCurve *pPara1, *pPara2 ;
	pCur1 = pSurfAtt->m_pBoundStart->m_pEdge->m_pCurve  ;
	pCur2 = pSurfAtt->m_pBoundEnd->m_pEdge->m_pCurve    ;
	pPara1 = pSurfAtt->m_pBoundStart->m_pPCurve ;
	pPara2 = pSurfAtt->m_pBoundEnd->m_pPCurve   ;

	pCur2->Reverse() ;
	pPara2->Reverse() ;

	PNT3D t1, t2, t3, t4 ;
	pPara1->GetEndPoint( t1, t2 ) ;
	pPara2->GetEndPoint( t3, t4 ) ;
	
	// 利用空间曲线上的点得到参数域上的准确点
	int n = m_nNum ;
	// 首先计算层号为0的路径
	GenRestUPath( pSurfAtt, pLoop, pathList, TRUE, FALSE, bZigZag ) ;
	// 然后计算层号为1的路径
	GetPointFromCurve( pSurfAtt->m_pSurf, pCur1, pPara1, dStart, n ) ;
	GetPointFromCurve( pSurfAtt->m_pSurf, pCur2, pPara2, dEnd, n ) ;
	CSmtCutPath *pPath = NULL ;
	int nLineNo = 0;
	for( int i = 0 ; i <= n ; i++ )
	{
		if ( m_bGrandMdfy && i != 1) continue;

		pPath = new CSmtCutPath() ;
		AddPathPointEx( pPath, dStart[i] ) ;
		AddPathPointEx( pPath, dEnd[i]   ) ;
		pathList.AddTail ( pPath ) ;

		pPath->m_nLayerNo = 1 ;
		pPath->m_nLineNo  = nLineNo ;
		nLineNo++;
	}
	// 最后计算层号为2的路径
	GenRestUPath( pSurfAtt, pLoop, pathList, FALSE, FALSE, bZigZag ) ;

	pCur2->Reverse() ;
	pPara2->Reverse() ;
	return TRUE ;
}

void CFlowlineGen::GenUParamPath ( C3DSurfAttribute* pSurfAtt,
								   CSmartLoop* pLoop,
								   CSmtCPathList& pathList,
								   PNT4D* dStart, BOOL bZigZag )
{
	// STEP 0 : 初始化数据
	CGeoCurve *pCur, *pPara ;
	pCur  = pSurfAtt->m_pBoundEnd->m_pEdge->m_pCurve    ;
	pPara = pSurfAtt->m_pBoundEnd->m_pPCurve   ;

	PNT3D p1, p2 ;
	pCur->GetEndPoint( p1, p2 ) ;

	pCur->Reverse() ;
	pPara->Reverse() ;

	// 首先计算层号为0的路径
	GenRestUPath( pSurfAtt, pLoop, pathList, TRUE, FALSE, bZigZag ) ;
	// 然后计算层号为1的路径,利用空间曲线上的点得到参数域上的准确点
	PNT4D *dEnd = new PNT4D[m_nNum+1] ;
	GetPointFromCurve( pSurfAtt->m_pSurf, pCur, pPara, dEnd, m_nNum ) ;
	CSmtCutPath *pPath = NULL ;
	int nLineNo = 0;
	for( int i = 0 ; i <= m_nNum ; i++ )
	{
		if (m_bGrandMdfy && i != 1) continue;

		pPath = new CSmtCutPath() ;

		AddPathPointEx( pPath, dStart[i] ) ;
		AddPathPointEx( pPath, dEnd[i]   ) ;

		pathList.AddTail ( pPath ) ;

		pPath->m_nLayerNo = 1 ;
		pPath->m_nLineNo  = nLineNo ;
		nLineNo++;
		
		nc_VectorCopy( dStart[i], dEnd[i], 4 ) ;
	}
	// 最后计算层号为2的路径
	GenRestUPath( pSurfAtt, pLoop, pathList, FALSE, FALSE, bZigZag ) ;

	pCur->Reverse() ;
	pPara->Reverse() ;
	delete[] dEnd ;
}

void CFlowlineGen::GenRestUPath ( C3DSurfAttribute* pSurfAtt, 
								  CSmartLoop* pLoop,
								  CSmtCPathList& pathList,
								  BOOL bHead, BOOL bReverse, BOOL bZigZag )
{
	CGeoCurve *pCur1, *pCur2 ;
	CGeoCurve *pPara1, *pPara2 ;
	pCur1 = pSurfAtt->m_pBoundStart->m_pEdge->m_pCurve  ;
	pCur2 = pSurfAtt->m_pBoundEnd->m_pEdge->m_pCurve    ;
	pPara1 = pSurfAtt->m_pBoundStart->m_pPCurve ;
	pPara2 = pSurfAtt->m_pBoundEnd->m_pPCurve   ;
	if( bReverse )
	{
		pCur2->Reverse() ;
		pPara2->Reverse() ;
	}
	PNT2D start, end ;
	PNT4D dStart, dEnd ;
	PNT2D dPtArr[200] ;
	int n = 1 ;
	BOOL bPathReverse = FALSE ;

	while( 1 )
	{
		GetLinePoint( pSurfAtt, start, end, n, bHead ) ;
		int nCount = pLoop->GetLineIntContour ( start, end, dPtArr, 200 ) ;
		n++ ;
		if( nCount == 0 ) break ;
		if( nCount % 2 ) continue ;
		CSmtCPathList tmpList ;
		for( int i = 0 ; i < nCount ; i += 2 )
		{
			dStart[0] = dPtArr[i  ][0], dStart[1] = dPtArr[i ][1] ;
			dEnd[0]   = dPtArr[i+1][0], dEnd[1]   = dPtArr[i+1][1] ;
			dStart[2] = dStart[3] = dEnd[2] = dEnd[3] = 0. ;
			CSmtCutPath* pPath = new CSmtCutPath() ;
			AddPathPointEx( pPath, dStart ) ;
			AddPathPointEx( pPath, dEnd   ) ;
			if( bZigZag && bPathReverse )
				pPath->ReverseDirect () ;
			pPath->DefineBox () ;
			if( bHead )
			{
				pPath->m_nLineNo = -n ;
				pPath->m_nLayerNo = 0 ;
			}
			else
			{
				pPath->m_nLineNo = m_nNum + n ;
				pPath->m_nLayerNo = 2 ;
			}
			tmpList.AddTail ( pPath ) ;
		}
		POSITION pos, atpos ;
		CSmtCutPath *pPath ;
		if( bZigZag && bPathReverse )
			pos = tmpList.GetTailPosition () ;
		else
			pos = tmpList.GetHeadPosition () ;

		while( pos )
		{
			atpos = pos ;
			if( bZigZag && bPathReverse )
				pPath = tmpList.GetPrev ( pos ) ;
			else
				pPath = tmpList.GetNext ( pos ) ;
			tmpList.RemoveAt ( atpos ) ;
			if( bHead )
				pathList.AddHead ( pPath ) ;
			else
				pathList.AddTail ( pPath ) ;
		}
		
		bPathReverse = !bPathReverse ;
	}
	if( bReverse )
	{
		pCur2->Reverse() ;
		pPara2->Reverse() ;
	}
}

void CFlowlineGen::GetLinePoint ( C3DSurfAttribute* pSurfAtt,  
								  PNT2D start, PNT2D end,
								  int n, BOOL bHead )
{
	CGeoCurve *pCur1, *pCur2 ;
	CGeoCurve *pPara1, *pPara2 ;
	pCur1 = pSurfAtt->m_pBoundStart->m_pEdge->m_pCurve  ;
	pCur2 = pSurfAtt->m_pBoundEnd->m_pEdge->m_pCurve    ;
	pPara1 = pSurfAtt->m_pBoundStart->m_pPCurve ;
	pPara2 = pSurfAtt->m_pBoundEnd->m_pPCurve   ;	
	// 计算步长
	double dStartStep = pPara1->GetLength() / m_nNum ;
	double dEndStep   = pPara2->GetLength() / m_nNum ;
	VEC3D vec1, vec2, vec ;
	PNT3D pos1, pos2 ;
	int i ;
	// 计算首末端点和法矢
	if( bHead )
	{
		pPara1->GetTangent( 0., pos1, vec1 ) ;
		pPara2->GetTangent( 0., pos2, vec2 ) ;
		for( i = 0 ; i < 3 ; i++ )
		{
			vec1[i] = -vec1[i] ;
			vec2[i] = -vec2[i] ;
		}
	}
	else
	{
		pPara1->GetTangent( 1., pos1, vec1 ) ;
		pPara2->GetTangent( 1., pos2, vec2 ) ;
	}
	// 计算参数域延长线上的两点
	for( i = 0 ; i < 2 ; i++ )
	{
		start[i] = pos1[i] + n * dStartStep * vec1[i] ;
		end[i]   = pos2[i] + n * dEndStep * vec2[i] ;
		vec[i] = start[i] - end[i] ;
	}
	// 将两点沿着其连线的切线方向延长
	for( i = 0 ; i < 2 ; i++ )
	{
		start[i] += vec[i] * 2. ;
		end[i]   += -vec[i] * 2. ;
	}
}

void CFlowlineGen::GenWParamPath (	C3DSurfAttribute* pSurfAtt,		// <I>:输入曲面
									CSmtCPathList& pathList,		// <O>:输出路径
									double dOverStep )				// <I>:输入间距
										 				
{
	// STEP 0 : 初始化数据
	CGeoCurve *pCur1, *pCur2 ;
	CGeoCurve *pPara1, *pPara2 ;
	pCur1 = pSurfAtt->m_pBoundStart->m_pEdge->m_pCurve  ;
	pCur2 = pSurfAtt->m_pBoundEnd->m_pEdge->m_pCurve    ;
	
	pPara1 = pSurfAtt->m_pBoundStart->m_pPCurve ;
	pPara2 = pSurfAtt->m_pBoundEnd->m_pPCurve   ;

	pCur2->Reverse() ;
	pPara2->Reverse() ;

	// 利用空间曲线上的点得到参数域上的准确点
	
	double l1 = pCur1->GetLength() ;
	double l2 = pCur2->GetLength() ;
	double len = max( l1, l2 ) ;
	int n = int ( len / dOverStep + 1 ) ;

	if (m_bGrandMdfy)
	{
		n = 2;
	}
	PNT4D* dStart = new PNT4D[n+1] ;
	PNT4D* dEnd   = new PNT4D[n+1] ;

	GetPointFromCurve( pSurfAtt->m_pSurf, pCur1, pPara1, dStart, n ) ;
	GetPointFromCurve( pSurfAtt->m_pSurf, pCur2, pPara2, dEnd, n ) ;

	CSmtCutPath *pPath = NULL ;
	for( int i = 0 ; i <= n ; i++ )
	{
		pPath = new CSmtCutPath() ;
		AddPathPointEx( pPath, dStart[i] ) ;
		AddPathPointEx( pPath, dEnd[i]   ) ;
		pathList.AddTail ( pPath ) ;

		pPath->m_nLayerNo = 0 ;
		pPath->m_nLineNo  = i ;
	}
	
	delete[] dStart ;
	delete[] dEnd   ;

	// 然后判断是否重合
	PNT3D	st1, ed1, st2, ed2;
	
	pCur1->GetEndPoint( st1, ed1 ) ;
	pCur2->GetEndPoint( st2, ed2 ) ;

	if( mathDist( st1, st2 ) < MIN_DIS && mathDist( ed1, ed2 ) < MIN_DIS )
	{
		// 采样点判断两根线是否重合
		double u = 0 ;
		BOOL bOverLap = TRUE ;
		for( u = 0.05 ; u < 1. ; u += 0.05 )
		{
			if( pCur1->GetType () == OBJ3D_CURVE_POLYLINE )
				GetPoint( ( CGeoPLine3d *)pCur1, u, 0., st1 ) ;
			else
                pCur1->GetPoint ( u, st1 ) ;
			if( pCur2->GetType () == OBJ3D_CURVE_POLYLINE )
				GetPoint( ( CGeoPLine3d *)pCur2, u, 0., st2 ) ;
			else
				pCur2->GetPoint ( u, st2 ) ;
			if( mathDist( st1, st2 ) > 0.001 )
			{
				bOverLap = FALSE ;
				break ;
			}
		}
		if( bOverLap ) pSurfAtt->m_bSpiral = TRUE ;
	}
	pCur2->Reverse() ;
	pPara2->Reverse() ;
}

void CFlowlineGen::GetPointFromCurve ( CGeoTrmSurf* pSurf, 
									   CGeoCurve* pCur, 
									   CGeoCurve* pPara, 
									   PNT4D* dArray, int num )
{
	double dLen = pCur->GetLength() ;
	double dStep = dLen / num ;
	int i = 0 ;
	double t, leng = 0., o_dis = 0, u = 0, v = 0 ;
	PNT3D t1, t2, p1, p2 ;
	CGeoInter cInter ;
	PNT3D p, near_p ;
	// 参数域端点
	pPara->GetEndPoint( t1, t2 ) ;
	pCur->GetEndPoint( p1, p2 ) ;
	int nType = pCur->GetType () ;
	for( i = 0 ; i <= num ; i++ )
	{
		leng = i * dStep ;
		if( fabs( dLen ) > 1.0e-6 )
		{
			t = leng / dLen ;
			if( dStep < 1.0e-3  ) t = 1.0 * i / num ;

			if( nType == OBJ3D_CURVE_POLYLINE )
				GetPoint( ( CGeoPLine3d  *)pCur, leng, dLen, p ) ;
			else if( nType == OBJ3D_CURVE_LINE )
				GetPoint( ( CGeoLine *) pCur, leng, dLen, p ) ;
			else
				pCur->GetPoint ( t, p ) ;
		}
		else
		{
			t = 1.0 * i / num ;
			pCur->GetPoint( t, p ) ;
		}
		dArray[i][0] = t1[0] + t * ( t2[0] - t1[0] ) ;
		dArray[i][1] = t1[1] + t * ( t2[1] - t1[1] ) ;
		dArray[i][2] = dArray[i][3] = 0. ;

		u = dArray[i][0], v = dArray[i][1] ;
 		// 为了防止旋转面在跌代时候过界,加以判断.
		cInter.IterateSurfNearestPnt( pSurf->m_pSurface, p, u, v, near_p, o_dis );
		if( fabs( u - dArray[i][0] ) < 0.5 && fabs( v - dArray[i][1] ) < 0.5 )
		{
			dArray[i][0] = u, dArray[i][1] = v ;
		}
	}
}

void CFlowlineGen::GetParamPTList ( C3DSurfAttribute* pSurfAtt1,
								    C3DSurfAttribute* pSurfAtt2,
								    PNT4D* dArray, BOOL bHead )
{
	// STEP 0 : 初始化数据
	CGeoCurve *pCur, *pPara ;
	if( bHead )
	{
		pCur = pSurfAtt2->m_pBoundStart->m_pEdge->m_pCurve  ;
		pPara = pSurfAtt2->m_pBoundStart->m_pPCurve ;
	}
	else
	{
		pCur = pSurfAtt2->m_pBoundEnd->m_pEdge->m_pCurve  ;
		pPara = pSurfAtt2->m_pBoundEnd->m_pPCurve ;
	}
		
	CGeoInter cInter ;
	cInter.m_OptTol = 1.0e-8 ;
	PNT3D t1, t2, p, o_pt ;
	double t, o_dis, u = 0, v = 0 ;
	double leng = pCur->GetLength() ;
	double dStep = leng / m_nNum ;
	pPara->GetEndPoint( t1, t2 ) ;
	// 利用上个面上的点这个面上得到参数域上的准确点
	for( int i = 0 ; i <= m_nNum ; i++ )
	{
		if (m_bGrandMdfy && i != 1) continue;

		pSurfAtt1->m_pSurf->GetPoint( dArray[i][0], dArray[i][1], p ) ;
		if( fabs( leng ) > 1.0e-6 )
		{
			t = dStep * i / leng ;
			if( dStep < 1.0e-3 ) t = 1.0 * i / m_nNum ;
		}
		else
			t = 1.0 * i / m_nNum ;
/*
		if( nType == OBJ3D_CURVE_POLYLINE )
            GetPoint( ( CGeoPLine3d  *)pCur, leng, dLen, p ) ;
		else if( nType == OBJ3D_CURVE_LINE )
			GetPoint( ( CGeoLine *) pCur, leng, dLen, p ) ;
		else
			pCur->GetPoint( t, p ) ;
*/
		dArray[i][0] = t1[0] + t * ( t2[0] - t1[0] ) ;
		dArray[i][1] = t1[1] + t * ( t2[1] - t1[1] ) ;

		u = dArray[i][0], v = dArray[i][1] ;
		if( cInter.IterateSurfNearestPnt( pSurfAtt2->m_pSurf->m_pSurface, p, u, v, o_pt, o_dis ) )
		{
			dArray[i][0] = u, dArray[i][1] = v ;
		}
	}
}

void CFlowlineGen::ConnectAllUPath1 ( CUWLineParam& cUWLine, 
									  BOOL bZigZag, 
									  CSmtCPathLib& cSmtCPathLib )
{
	CSmtCutPath *pPath = NULL ;
	BOOL  bIsReverse = FALSE ;
	INT_PTR nSize = cUWLine.m_cSurfAttArr.GetSize () ;
	INT_PTR i, j ;
	CSmtCPathLib tmpList ;
	POSITION pos, atpos ;
	// 按照面的顺序，找到所有m_nLayerNo = 0的路径进行首尾连接
	ConnectPathByLayer02( cUWLine.m_cSurfAttArr,cSmtCPathLib, 0 ) ;
	// 按照面的顺序，找到所有m_nLayerNo = 2的路径进行首尾连接
	ConnectPathByLayer02( cUWLine.m_cSurfAttArr,tmpList, 2 ) ;
	// 按照面的顺序，找到所有m_nLayerNo = 1的路径进行首尾连接
	if( cUWLine.m_bSmooth )
	{
		for( i = 0 ; i <= m_nNum ; i++ )
		{
			for( j = 0 ; j < nSize ; j++ )
			{
				C3DSurfAttribute* pSurf = cUWLine.m_cSurfAttArr[j] ;
				
				if( pSurf->m_cSmtCPathLib.m_cAllPath.IsEmpty() )
					continue ;
				pPath = pSurf->m_cSmtCPathLib.m_cAllPath.RemoveHead() ;
				
				if( cUWLine.m_IsClosed )
				{
					cSmtCPathLib.AddToTail ( pPath ) ;
				}
				else
				{
					if( bZigZag && bIsReverse )
						pPath->ReverseDirect () ;
					cSmtCPathLib.AddToTail ( pPath ) ;
				}/**/
			}
			bIsReverse = !bIsReverse ;
		}
	}
	else
	{
		for( i = 0 ; i < nSize ; i++ )
		{
			C3DSurfAttribute* pSurf = cUWLine.m_cSurfAttArr[i] ;
			if (pSurf->m_cSmtCPathLib.m_cAllPath.IsEmpty())
				continue;
			pos = pSurf->m_cSmtCPathLib.m_cAllPath.GetHeadPosition () ;
			while( pos )
			{
				atpos = pos ;
				pPath = pSurf->m_cSmtCPathLib.m_cAllPath.GetNext( pos ) ;
				pSurf->m_cSmtCPathLib.m_cAllPath.RemoveAt ( atpos ) ;

				if( bZigZag && bIsReverse )
					pPath->ReverseDirect () ;
				cSmtCPathLib.AddToTail ( pPath ) ;
				bIsReverse = !bIsReverse ;
			
			}
		}
	}
	// 将临时路径组中的路径导到cSmtCPathLib中
	pos = tmpList.m_cAllPath.GetHeadPosition () ;
	while( pos )
	{
		atpos = pos ;
		pPath = tmpList.m_cAllPath.GetNext ( pos ) ;
		tmpList.m_cAllPath.RemoveAt ( atpos ) ;
		cSmtCPathLib.m_cAllPath.AddTail ( pPath ) ;
	}
}

void CFlowlineGen::ConnectAllWPath1 ( CUWLineParam& cUWLine, 
									  BOOL bZigZag, 
									  CSmtCPathLib& cSmtCPathLib ) 
{
	C3DSurfAttribute * pSurf = NULL ;
	INT_PTR nSize = cUWLine.m_cSurfAttArr.GetSize () ;

	POSITION pos, atpos ;
	CSmtCutPath *pPath = NULL ;
//	PNT3D end ;
	BOOL  bIsReverse = FALSE ;
	for( INT_PTR i = 0 ; i < nSize ; i++ )
	{
		pSurf = cUWLine.m_cSurfAttArr[i] ;
		if (pSurf->m_cSmtCPathLib.m_cAllPath.IsEmpty())
			continue;

		// 如果闭合，删除重边的路径
		if( i > 0  )//&& cUWLine.m_IsClosed
		{
			pPath = pSurf->m_cSmtCPathLib.m_cAllPath.RemoveHead () ;
			delete pPath ;
		}
		if( i == nSize-1 && cUWLine.m_IsClosed )
		{
			pPath = pSurf->m_cSmtCPathLib.m_cAllPath.RemoveTail () ;
			delete pPath ;
		}
		
		pos = pSurf->m_cSmtCPathLib.m_cAllPath.GetHeadPosition () ;
		while( pos )
		{
			atpos = pos ;
			pPath = pSurf->m_cSmtCPathLib.m_cAllPath.GetNext ( pos ) ;
			pSurf->m_cSmtCPathLib.m_cAllPath.RemoveAt ( atpos ) ;

			if( !pSurf->m_bSpiral && bZigZag )
			{
				if( bIsReverse )
				{
					pPath->ReverseDirect () ;
				}
				bIsReverse = !bIsReverse ;
			}
			cSmtCPathLib.AddToTail ( pPath ) ;
		}
		pPath = cSmtCPathLib.m_cAllPath.GetTail() ;
	}
}

///////////////////第三种计算曲面流线的方法////////////////////////////////
int CFlowlineGen::GenSurfPathLib3( CUWLineParam& cUWLine,
								   RFRAME& cRFrame    , 
								   JDNC_SETUP& Setup,
								   JDNC_FUWLINE& UWLineCut,
								   JDNC_PRGDEF &PrgDef )
{

	int nCutDir      = UWLineCut.m_nCutDir    ;
	double dOverStep = UWLineCut.m_dOverStep  ;
	C3DSurfAttribute* pSurfAtt = NULL ;
	BOOL bZigZag = ( UWLineCut.m_bUWLineFlag & NCDEF_UWLINE_ZIGZAG ) ? TRUE : FALSE ;
	// 存放共边面上的精确参数域上的点
	PNT4D *dStart = NULL, *dEnd = NULL, *dMidd = NULL ;
	if( nCutDir == 0 )
	{
		dStart = new PNT4D[m_nNum+1] ;
		dEnd   = new PNT4D[m_nNum+1] ;
		dMidd  = new PNT4D[m_nNum+1] ;
	}

	CSmtCutPath *pPath = NULL ;
	INT_PTR n = cUWLine.m_cSurfAttArr.GetSize (), nNumLine = 0 ;
	PNT3D end ;
	for( INT_PTR i = 0 ; i < n ; i++ )
	{
		pSurfAtt = cUWLine.m_cSurfAttArr[i] ;
		if( !pSurfAtt )
		{
			MathCAM_MovePrgStep(PrgDef);
            continue ;
		}
		if ( SurfNC_IsAbort() ) 
		{
			delete[] dStart, delete[] dEnd,	delete[] dMidd ;
			return FALSE ;
		}
		// 生成参数域的环
		CSmartPathGen cGen ;
		CSmartLoop* pLoop = cGen.ExtractSurfLoop( pSurfAtt->m_pSurf, 
			                                      Setup.m_cTolDef,
												  &cRFrame ) ;
		//生成参数域上的路径
		CSmtCPathList pathList ;
		POSITION pos ;
		
		// u向生成路径
		if( nCutDir == 0 )
		{
			if( i == 0 )
			{
				if( !GenUParamPath( pSurfAtt, pLoop, pathList, dStart, dEnd, bZigZag ) )
				{
					delete[] dStart, delete[] dEnd,	delete[] dMidd ;
					return FALSE ;
				}
			}
			else if( i == n-1 && cUWLine.m_IsClosed )
			{
				// 首先计算层号为0的路径
				GenRestUPath( pSurfAtt, pLoop, pathList, TRUE, TRUE, bZigZag ) ;
				// 然后计算层号为1的路径
				// 首点
				GetParamPTList( cUWLine.m_cSurfAttArr[i-1], pSurfAtt, dEnd, TRUE ) ;
				// 末点
				GetParamPTList( cUWLine.m_cSurfAttArr[0], pSurfAtt, dStart, FALSE ) ;
				int nLineNo = 0;
				for( int j = 0 ; j <= m_nNum ; j++ )
				{
					if (m_bGrandMdfy && j != 1) continue;
					CSmtCutPath *pPath = new CSmtCutPath() ;
					AddPathPointEx( pPath, dEnd[j]   ) ;
					AddPathPointEx( pPath, dStart[j] ) ;
					pathList.AddTail ( pPath ) ;

					pPath->m_nLayerNo = 1 ;
					pPath->m_nLineNo  = nLineNo ;
					nLineNo++;
				}
				// 最后计算层号为2的路径
				GenRestUPath( pSurfAtt, pLoop, pathList, FALSE, TRUE, bZigZag ) ;/**/
			}
			else
			{
				GetParamPTList( cUWLine.m_cSurfAttArr[i-1], pSurfAtt, dEnd, TRUE ) ;
				for( int k = 0 ; k < m_nNum ; k++ )
					nc_VectorCopy( dMidd[k], dEnd[k], 4 ) ;
				GenUParamPath( pSurfAtt, pLoop, pathList, dEnd, bZigZag ) ;
			}
		}
		else
		{
			GenWParamPath3( pSurfAtt, pathList, pLoop, cRFrame, UWLineCut, dOverStep, i ) ;
		}
		// 调整路径的顺序
		if( nCutDir == 1 )
			AdjustWLineOrder( pSurfAtt, pathList ) ;
		// 进度条
		nNumLine = pathList.GetCount () ;
		if( nNumLine < 1 ) nNumLine = 1 ;
		PrgDef.m_dIncStep = PrgDef.m_dTotalMove / nNumLine ;
		// PrgDef.m_dStepAt  = 0.0 ;
		PrgDef.m_dLimitAt = 1.0 ;
		//将参数域上的路径映射到空间域上
		pos = pathList.GetHeadPosition () ;
		while( pos )
		{
			pPath = pathList.GetNext ( pos ) ;
			//qqs 曲面磨削调整
			if (m_bGrandMdfy)
			{
				PNT4D			dPnt1={0.0, 0.0, 0.0, 0.0};
				PNT4D			dPnt2={0.0, 0.0, 0.0, 0.0};
				for (int m = 0; m < 2; m++)
				{
					dPnt1[m] = pPath->m_pHead->m_fPoint[m];
					dPnt2[m] = pPath->m_pTail->m_fPoint[m];
				}
				pPath->ClearAllPoint();
				GrindParamPathGen(pPath, UWLineCut, cRFrame, pSurfAtt->m_pSurf, nCutDir, Setup.m_cTolDef.m_dArcTol, dPnt1, dPnt2);
			}
			
			// 将参数域路径映射到曲面上
			TransfPathToSurf( pSurfAtt, m_pMiller, cRFrame, pPath );
			pSurfAtt->m_cSmtCPathLib.AddToTail( pPath ) ;
			MathCAM_MovePrgStep(PrgDef);
		}
		
		// 保持两组路径的方向是一致的
		if( nCutDir == 1 )
			KeepSameLineDir( pSurfAtt->m_cSmtCPathLib.m_cAllPath, end, i ) ;
		// 将前一条路径中相同层号和列号的路径与第二个路径中的相连,前提是路径是光滑的
		if( nCutDir == 0 && i > 0  )//&& cUWLine.m_bSmooth
		{
			ConnectNearSurfPath( cUWLine.m_cSurfAttArr[i-1], pSurfAtt, dMidd ) ;
			// 如果是最后一个，并且是闭合的，首尾相连
			if( i == n - 1 && cUWLine.m_IsClosed ) 
			{
				TrimCloseSurfPath( pSurfAtt, dEnd ) ;
			}
		}
		// 生成参数域曲线的路径 for test 
	/*	if( i == 0 )
		{
			double dArea = pLoop->Area () ;
			CSmartCurve* pHead = pLoop->m_pCurve ;
			for( ; pHead ; pHead = pHead->next )
			{
				CSmtCutPath* pPath = millface_CurveToCPath( *pHead, Setup.m_cTolDef ) ;
				pSurfAtt->m_cSmtCPathLib.AddToTail( pPath ) ;
			}
		}*/
		Mini_DeleteContours( pLoop ) ;
	}
	if( nCutDir == 0 )
	{
		delete[] dStart ;
		delete[] dEnd   ;
		delete[] dMidd  ;
	}		
	return 1;
}

//从原始曲面中取与加工方向垂直方向上的0、0.5、1三条参数域上的加工路径，计算长度，并返回最长的长度值 qqs 2013.05.22
double CFlowlineGen::GetMdfyHeit(CSmartTool *pMill, JDNC_UWLINE cParam, RFRAME &lf, CGeoTrmSurf* pSurf, int nCutDir, BOOL bOffDir, double dArcTol)
{
	int		i = 0 ;
	FPNT3D st[3], ed[3] ;
	CSmtCutPath *pPath[3] = { NULL, NULL, NULL } ;
	CSmtCutPointEx *pnt1 = NULL,  *pnt2 = NULL ;
	for( i = 0 ; i < 3 ; i++ )
	{
		st[0][i] = st[1][i] = st[2][i] = 0.f ;
		ed[0][i] = ed[1][i] = ed[2][i] = 0.f ;
	}
	if( nCutDir == 1 )
	{
		st[0][0] = st[1][0] = st[2][0] = float( pSurf->m_pLoop->m_dBox2d[0][0] ) ;
		ed[0][0] = ed[1][0] = ed[2][0] = float( pSurf->m_pLoop->m_dBox2d[1][0] ) ;
		st[0][1] = ed[0][1] = float( pSurf->m_pLoop->m_dBox2d[0][1] ) ;
		st[1][1] = ed[1][1] = float( ( pSurf->m_pLoop->m_dBox2d[0][1] + pSurf->m_pLoop->m_dBox2d[1][1]) * 0.5 ) ;
		st[2][1] = ed[2][1] = float( pSurf->m_pLoop->m_dBox2d[1][1] ) ;

		for( i = 0 ; i < 3 ; i++ )
		{	
			pPath[i] = new CSmtCutPath() ;
			pnt1 = new CSmtCutPointEx() ;
			pnt2 = new CSmtCutPointEx() ;
			mathFCpyPnt( st[i], pnt1->m_fPoint ), mathFCpyPnt( ed[i], pnt2->m_fPoint ) ;
			pPath[i]->AddTail( pnt1 ), pPath[i]->AddTail( pnt2 ) ;
		}
	}
	else
	{
		st[0][1] = st[1][1] = st[2][1] = float( pSurf->m_pLoop->m_dBox2d[0][1] ) ;
		ed[0][1] = ed[1][1] = ed[2][1] = float( pSurf->m_pLoop->m_dBox2d[1][1] ) ;
		st[0][0] = ed[0][0] = float( pSurf->m_pLoop->m_dBox2d[0][0] ) ;
		st[1][0] = ed[1][0] = float( ( pSurf->m_pLoop->m_dBox2d[0][0] + pSurf->m_pLoop->m_dBox2d[1][0] ) * 0.5 ) ;
		st[2][0] = ed[2][0] = float( pSurf->m_pLoop->m_dBox2d[1][0] ) ;

		for( i = 0 ; i < 3 ; i++ )
		{	
			pPath[i] = new CSmtCutPath() ;
			pnt1 = new CSmtCutPointEx() ;
			pnt2 = new CSmtCutPointEx() ;
			mathFCpyPnt( st[i], pnt1->m_fPoint ), mathFCpyPnt( ed[i], pnt2->m_fPoint ) ;
			pPath[i]->AddTail( pnt1 ), pPath[i]->AddTail( pnt2 ) ;
		}
	}


	CSmtCutPath* pOrgPath[3] = {NULL, NULL, NULL};
	CSmtCutPointEx *pOrgPnt1 = NULL, *pOrgPnt2 = NULL;
	if( nCutDir == 1 )
	{
		for (int j = 0; j < 3; j++)
		{
			pOrgPath[j] = new CSmtCutPath();
			pOrgPnt1 = new CSmtCutPointEx();
			pOrgPnt2 = new CSmtCutPointEx();

			pOrgPnt1->m_fPoint[0] = 0.;
			pOrgPnt1->m_fPoint[1] = 0.5f * j;
			pOrgPnt2->m_fPoint[0] = 1.;
			pOrgPnt2->m_fPoint[1] = 0.5f * j;
			pOrgPath[j]->AddTail(pOrgPnt1), pOrgPath[j]->AddTail(pOrgPnt2);
		}
	}
	else
	{
		for (int j = 0; j < 3; j++)
		{
			pOrgPath[j] = new CSmtCutPath();
			pOrgPnt1 = new CSmtCutPointEx();
			pOrgPnt2 = new CSmtCutPointEx();

			pOrgPnt1->m_fPoint[0] = 0.5f * j;
			pOrgPnt1->m_fPoint[1] = 0.;
			pOrgPnt2->m_fPoint[0] = 0.5f * j;
			pOrgPnt2->m_fPoint[1] = 1.;
			pOrgPath[j]->AddTail(pOrgPnt1), pOrgPath[j]->AddTail(pOrgPnt2);
		}
	}
	double dLength[3] = {0., 0., 0.};
	double dLength2[3] = {0., 0., 0.};
	double dMaxLength = -1.; // 原始曲面上的最长路径长度
	double dMaxLength2 = -1.; // 裁剪曲面上的最长路径长度

	for( int m = 0 ; m < 3 ; m++ )
	{
		pOrgPath[m]->DefineBox () ;
		pPath[m]->DefineBox();
		// 得到刀心点路径
		CutPathToSurf( pMill, lf, pOrgPath[m], pSurf, bOffDir, dArcTol ) ;
		CutPathToSurf( pMill, lf, pPath[m], pSurf, bOffDir, dArcTol ) ;
		dLength[m] = pOrgPath[m]->GetLength () ;
		dLength2[m] = pPath[m]->GetLength () ;
		if( dLength[m] > dMaxLength )
		{
			dMaxLength = dLength[m] ;
		}
		if( dLength2[m] > dMaxLength2 )
		{
			dMaxLength2 = dLength2[m] ;
		}
	}
	for (int m = 0; m < 3; m++)
	{
		delete pOrgPath[m] ;
		pOrgPath[m] = NULL ;
		delete pPath[m];
		pPath[m] = NULL;
	}

	double dMdfyHeit = 0.;
	if (cParam.m_dMdfyHeit > dMaxLength2) 
	{
		dMdfyHeit = dMaxLength2 / dMaxLength;
	}
	else
	{
		dMdfyHeit = cParam.m_dMdfyHeit / dMaxLength;
	}

	if ( dMdfyHeit > 1.) dMdfyHeit = 1.;
	return dMdfyHeit;
}

void CFlowlineGen::GenWParamPath3 ( C3DSurfAttribute* pSurfAtt,
								    CSmtCPathList& pathList,
									CSmartLoop* pLoop,
									RFRAME& cRFrame, 
									JDNC_UWLINE& cParam,
									double dOverStep, INT_PTR n )
{
	// STEP 0 : 初始化数据
	CGeoCurve *pPara1 = pSurfAtt->m_pBoundStart->m_pPCurve ;
	CGeoCurve *pPara2 = pSurfAtt->m_pBoundEnd->m_pPCurve ;
	
	PNT3D t1, t2 ;//, t3, t4 ;
	// 找到参数域上
	int dir = 0 ;
	if( n == 0 )
		pPara2->GetEndPoint( t1, t2 ) ;
	else
		pPara1->GetEndPoint( t1, t2 ) ;


	double dx = fabs( t2[0] - t1[0] ) ;
	double dy = fabs( t2[1] - t1[1] ) ;
	if( dx < dy )
		dir = 1 ;
	double *dParam = NULL;
	int nNum = 0;
	BOOL bOffDir = GetOffSetDir(pSurfAtt->m_pSurf, cRFrame, cParam);
	CSmtCPathLib PathLib;
	GenParamSerialByStep(m_pMiller, cRFrame, pSurfAtt->m_pSurf, dOverStep, 
		dir, dParam, nNum, bOffDir, m_cSetupDef.m_cTolDef.m_dArcTol, PathLib);
	
 
	// STEP 0 : 初始化数据
	if( !pLoop ) return ; 
	PNT2D dPtArr[200], Line[2];
	Line[0][0] = pLoop->m_dBox[0][0] - NCSF_TOL_01;
	Line[0][1] = pLoop->m_dBox[0][1] - NCSF_TOL_01;
	Line[1][0] = pLoop->m_dBox[1][0] + NCSF_TOL_01;
	Line[1][1] = pLoop->m_dBox[1][1] + NCSF_TOL_01;
	

	// STEP 1 : 递增计算交点
	int nCount = 0 ;
	PNT4D dStart, dEnd ;
	for (int i=0; i<=nNum; i++)
	{
		if (dir == 0)
			Line[0][1] = Line[1][1] = dParam[i];
		else
			Line[0][0] = Line[1][0] = dParam[i];
		nCount = pLoop->GetLineIntContour(Line[0], Line[1], dPtArr, 200);

		if( nCount == 0 || nCount % 2 ) continue ;
		for( int i = 0 ; i < nCount ; i += 2 )
		{
			dStart[0] = dPtArr[i  ][0], dStart[1] = dPtArr[i ][1] ;
			dEnd[0]   = dPtArr[i+1][0], dEnd[1]   = dPtArr[i+1][1] ;
			dStart[2] = dStart[3] = dEnd[2] = dEnd[3] = 0. ;
			CSmtCutPath* pPath = new CSmtCutPath() ;
			AddPathPointEx( pPath, dStart ) ;
			AddPathPointEx( pPath, dEnd   ) ;
			
			pPath->DefineBox () ;
			pathList.AddTail( pPath ) ;
		}
	}
	delete dParam;
/**/
	
}

int CFlowlineGen::GetWParamOverStep3 ( C3DSurfAttribute* pSurfAtt, RFRAME& cRFrame, CSmartLoop* pLoop,
									   JDNC_UWLINE& cParam, double dOverStep, int nDir )
{	
	BOOL bOffDir = GetOffSetDir(pSurfAtt->m_pSurf, cRFrame, cParam);
	CSmtCutPath *pPath = GetMaxLenPath(m_pMiller, cRFrame, pSurfAtt->m_pSurf, pLoop, nDir, bOffDir,
		m_cSetupDef.m_cTolDef.m_dArcTol);
	if( !pPath ) 
		return 0;
	int num = int ( pPath->GetLength () / dOverStep + 1 ) ;
	delete pPath ;
	return num ;
}

void CFlowlineGen::ConnectAllWPath3 ( CUWLineParam& cUWLine, 
									  BOOL bZigZag, 
									  CSmtCPathLib& cSmtCPathLib ) 
{
	C3DSurfAttribute * pSurf = NULL ;
	INT_PTR nSize = cUWLine.m_cSurfAttArr.GetSize () ;

	POSITION pos, atpos ;
	CSmtCutPath *pPath = NULL ;
//	PNT3D end ;
	BOOL  bIsReverse = FALSE ;
	for( INT_PTR i = 0 ; i < nSize ; i++ )
	{
		pSurf = cUWLine.m_cSurfAttArr[i] ;
		if (pSurf->m_cSmtCPathLib.m_cAllPath.IsEmpty())
			continue;
		pos = pSurf->m_cSmtCPathLib.m_cAllPath.GetHeadPosition () ;
		while( pos )
		{
			atpos = pos ;
			pPath = pSurf->m_cSmtCPathLib.m_cAllPath.GetNext ( pos ) ;
			pSurf->m_cSmtCPathLib.m_cAllPath.RemoveAt ( atpos ) ;

			if( bZigZag )
			{
				if( bIsReverse )
				{
					pPath->ReverseDirect () ;
				}
				bIsReverse = !bIsReverse ;
			}
			cSmtCPathLib.AddToTail ( pPath ) ;
		}
		pPath = cSmtCPathLib.m_cAllPath.GetTail() ;
	}
}

void CFlowlineGen::AdjustWLineOrder ( C3DSurfAttribute* pSurfAtt,
									  CSmtCPathList& pathLib )
{
	// 非空判断
	if (pathLib.IsEmpty())
		return;
	PNT3D start, end ;
	CSmtCutPath* pPath = pathLib.GetHead () ;
    nc_FloatToDouble( start, pPath->m_pHead->m_fPoint,3 ) ;
    nc_FloatToDouble( end ,  pPath->m_pTail->m_fPoint,3 ) ;

    PNT3D p1, p2, p3, p4 ;
	pSurfAtt->m_pBoundStart->m_pPCurve->GetEndPoint( p1, p2 ) ;
	pSurfAtt->m_pBoundEnd->m_pPCurve->GetEndPoint( p3, p4 ) ;
	
	double dist1 = min( mathDist( start, p1 ), mathDist( start, p2 ) ) ;
	double dist2 = min( mathDist( start, p3 ), mathDist( start, p4 ) ) ;
	
	if( dist1 > dist2 )
		ReverseLineOrder( pathLib ) ;

}

void CFlowlineGen::KeepSameLineDir ( CSmtCPathList& pathLib, PNT3D pt, INT_PTR n )
{
	// 非空判断
	if (pathLib.IsEmpty())
		return;
	CSmtCutPath* pPath ;
	PNT3D start, end ;
	// 两个面生成的路径保持同向
	pPath = pathLib.GetHead () ;
	if( n > 0 )
	{
		nc_FloatToDouble( start, pPath->m_pHead->m_fPoint , 3) ;
		nc_FloatToDouble( end , pPath->m_pTail->m_fPoint   , 3 ) ;
		if( mathDist( pt, start ) < mathDist( pt, end ) )
		{
			POSITION pos = pathLib.GetHeadPosition () ;
			while( pos )
			{
				pPath = pathLib.GetNext ( pos ) ;
				pPath->ReverseDirect () ;
				pPath->DefineBox () ;
			}
		}
	}
	pPath = pathLib.GetTail () ;
	nc_FloatToDouble( pt, pPath->m_pTail->m_fPoint, 3) ;
}

///////////////////第二种计算曲面流线的方法////////////////////////////////
double CFlowlineGen::GetSurfBndLength ( C3DSurfAttribute* pSurfAtt, int nFlag )
{
	CGeoCurve *pCurve = NULL ;
	if( nFlag )
		pCurve = pSurfAtt->m_pBoundEnd->m_pEdge->m_pCurve ;
	else
		pCurve = pSurfAtt->m_pBoundStart->m_pEdge->m_pCurve ;
	// 得到参数的长度
	double len = 0. ;
	if( pCurve )
		len = pCurve->GetLength() ;
	
	return len ;
}

void CFlowlineGen::ConnectPathByLayer02 ( C3DSurfAttArray& c3DSurfAttArr, 
										  CSmtCPathLib& cSmtCPathLib,
										  int nLayer )
{
	C3DSurfAttribute * pSurf = NULL ;
	INT_PTR nSize = c3DSurfAttArr.GetSize () ;
	// 初始化进度条－无用
	JDNC_PRGDEF cPrgDef ;
	cPrgDef.m_dLimitAt = 100 / ( 100 * 1.00  ) ;
	cPrgDef.m_dStepAt  = 0 ;
	cPrgDef.m_dIncStep = 1. ;
	cPrgDef.m_pBrkFunc = NULL ;
	cPrgDef.m_pPrgFunc = NULL ;

	// step1: 按照面的顺序，找到m_nLayerNo = 0的路径进行连接
	for( INT_PTR i = 0 ; i < nSize ; i++ )
	{
		CSmtCPathLib pathlib ;
		pSurf = c3DSurfAttArr[i] ;
		POSITION pos, atpos ;
		if (pSurf->m_cSmtCPathLib.m_cAllPath.IsEmpty())
			continue;
		pos = pSurf->m_cSmtCPathLib.m_cAllPath.GetHeadPosition () ;
		while( pos )
		{
			atpos = pos ;
			CSmtCutPath* path = pSurf->m_cSmtCPathLib.m_cAllPath.GetNext ( pos ) ;
			if( path->m_nLayerNo == nLayer )
			{
				pathlib.AddToTail ( path ) ;
				pSurf->m_cSmtCPathLib.m_cAllPath.RemoveAt ( atpos ) ;
			}
		}

		pos = pathlib.m_cAllPath.GetHeadPosition () ;		
		while( pos )
		{
			atpos = pos ;
			CSmtCutPath* path = pathlib.m_cAllPath.GetNext ( pos ) ;
			cSmtCPathLib.AddToTail ( path ) ;
			pathlib.m_cAllPath.RemoveAt ( atpos ) ;
		}
	}
}




void CFlowlineGen::ConnectNearSurfPath ( C3DSurfAttribute* pSurf1,
										 C3DSurfAttribute* pSurf2 ) 
{
	// 将前一条路径中相同层号和列号的路径与第二个路径中的相连
	CSmtCutPoint * pPoint ; 
	FPNT4D  dStart, dEnd ;

	dStart[3] = dEnd[3] = 0.0 ; 
	CSmtCutPath * pPath, *pObject = NULL ;
	POSITION pos, pos1, atpos1=NULL ;

	// 初始化进度条－无用
	JDNC_PRGDEF cPrgDef ;
	cPrgDef.m_dLimitAt = 100 / ( 100 * 1.00  ) ;
	cPrgDef.m_dStepAt  = 0 ;
	cPrgDef.m_dIncStep = 1. ;
	cPrgDef.m_pBrkFunc = NULL ;
	cPrgDef.m_pPrgFunc = NULL ;
	
	pos = pSurf2->m_cSmtCPathLib.m_cAllPath.GetHeadPosition () ;
	while( pos )
	{
		// 找到第二个曲面中共边上的路径
		pPath = pSurf2->m_cSmtCPathLib.m_cAllPath.GetNext ( pos ) ;
		if( pPath->m_nLayerNo != 1 )
			continue ;
		int nLineNo = pPath->m_nLineNo ;
		nc_VectorCopy( dEnd, pPath->m_pHead->m_fPoint,  3  )  ;
		// 找到第一个曲面上共边上的路径
		pObject = NULL ;
		pos1 = pSurf1->m_cSmtCPathLib.m_cAllPath.GetHeadPosition () ;
		while( pos1 )
		{
			atpos1 = pos1 ;
			pObject = pSurf1->m_cSmtCPathLib.m_cAllPath.GetNext ( pos1 ) ;
			if( pObject->m_nLayerNo != 1 || pObject->m_nLineNo != nLineNo )
				continue ;
			break ;
		}
		if( pObject )
		{
			nc_VectorCopy( dStart, pObject->m_pTail->m_fPoint,  3  ) ;
			pSurf1->m_cSmtCPathLib.m_cAllPath.RemoveAt( atpos1 ) ;
			while( pObject->m_pTail )
			{ // 连接路径
				pPoint = pObject->m_pTail ;
				pObject->RemovePoint( pPoint ) ;
				pPath->AddHead( pPoint ) ;
			}
			delete pObject ;
			pObject = NULL ;
		}
	}
}

void CFlowlineGen::ConnectNearSurfPath ( C3DSurfAttribute* pSurf1,
										 C3DSurfAttribute* pSurf2,
										 PNT4D* dArray ) 
{
	if (pSurf1->m_cSmtCPathLib.m_cAllPath.IsEmpty() && pSurf2->m_cSmtCPathLib.m_cAllPath.IsEmpty())
		return;
	// 将前一条路径中相同层号和列号的路径与第二个路径中的相连
	FPNT4D pnt[3] ;
	pnt[0][3] = pnt[1][3] = pnt[2][3] = 0. ;
	PNT3D p ;
	int i = 0 ;
	CSmtCutPath * pPath, *pObject = NULL ;
	POSITION pos, pos1, atpos1 = NULL ;
	pos = pSurf2->m_cSmtCPathLib.m_cAllPath.GetHeadPosition () ;
	while( pos )
	{
		// 找到第二个曲面中共边上的路径
		pPath = pSurf2->m_cSmtCPathLib.m_cAllPath.GetNext ( pos ) ;
		if( pPath->m_nLayerNo != 1 )
			continue ;
		int nLineNo = pPath->m_nLineNo ;
		nc_VectorCopy( pnt[2], pPath->m_pHead->m_fPoint,  3  )  ;
		// 找到第一个曲面上共边上的路径
		pObject = NULL ;
		pos1 = pSurf1->m_cSmtCPathLib.m_cAllPath.GetHeadPosition () ;
		while( pos1 )
		{
			atpos1 = pos1 ;
			pObject = pSurf1->m_cSmtCPathLib.m_cAllPath.GetNext ( pos1 ) ;
			if( pObject->m_nLayerNo != 1 || pObject->m_nLineNo != nLineNo )
				continue ;
			break ;
		}
		if( pObject )
		{
			nc_VectorCopy( pnt[0], pObject->m_pTail->m_fPoint,  3  ) ;
			i = nLineNo ;
			pSurf2->m_pSurf->GetPoint ( dArray[i][0], dArray[i][1], p ) ;
			for( i = 0 ; i < 3 ; i++ )
				pnt[1][i] = TFLOAT( p[i] ) ;
			
			pSurf1->m_cSmtCPathLib.m_cAllPath.RemoveAt( atpos1 ) ;
			ConnectNearPath( pObject, pPath, pnt ) ;
			pObject = NULL ;
		}
	}
}

void CFlowlineGen::ConnectNearPath ( CSmtCutPath *pPrev, CSmtCutPath *pCurr, FPNT4D pnt[3] )
{
	PNT3D p[3], cen ;
	VEC3D v[2] ;
	int i, j ;
	for( i = 0 ; i < 3 ; i++ )
	{
		for( j = 0 ; j < 3 ; j++ )
		{
			p[i][j] = pnt[i][j] ;
		}
	}
	// 利用这三个点，得到圆心坐标
	if( !CreateArcCenter( p, cen ) )
	{//	直接连接
		return ConnectPathDirect( pPrev, pCurr ) ;
	}
    // 得到路径的方向
	if( !GetPathDirect( pPrev, pCurr, v ) )
	{// 直接连接
		return ConnectPathDirect( pPrev, pCurr ) ;
	}
	// 求交点
	PNT3D intpt1, intpt2 ;
	if( !mathIntLin( p[0], v[0], p[2], v[1], MIN_LEN, MIN_ANG, intpt1, intpt2 ) )
	{// 直接连接
		return ConnectPathDirect( pPrev, pCurr ) ;
	}
	mathGetVec( p[0], intpt1, v[1] ) ;

	double ang = mathGetAngleUnit( v[0], v[1] ) ;
	if( ang < PI1_2 )
	{// 同向连接
		if (m_bGrandMdfy) // 磨削调整不进行延伸连接，需要延伸的路径采取之间连接 qqs
			ConnectPathDirect( pPrev, pCurr );
		else
			ConnectPathExtend( pPrev, pCurr, intpt1 ) ;
	}
	else
	{// 反向裁减连接
		ConnectPathTrim( pPrev, pCurr, p, intpt1 ) ;
	}
}

void CFlowlineGen::ConnectPathDirect( CSmtCutPath* pPrev, CSmtCutPath* pCurr )
{
	// 将前一个路径中的点加到当前路径中，并删除前一个路径
	CSmtCutPoint *pPoint = NULL ;
	while( pPrev->m_pTail )
	{ // 连接路径
		pPoint = pPrev->m_pTail ;
		pPrev->RemovePoint( pPoint ) ;
		pCurr->AddHead( pPoint ) ;
	}
	delete pPrev ;
}
void CFlowlineGen::ConnectPathExtend ( CSmtCutPath* pPrev, CSmtCutPath* pCurr, PNT3D p )
{
	// 将交点连接到当前路径的头点
	CSmtCutPointEx *pPoint = new CSmtCutPointEx() ;
	for( int i = 0 ; i < 3 ; i++ )
	{
		pPoint->m_fPoint[i] = TFLOAT( p[i] ) ;
	}
	pCurr->AddHead ( pPoint ) ;
	// 将前一个路径中的点加到当前路径中，并删除前一个路径
	ConnectPathDirect( pPrev, pCurr ) ;
}
void CFlowlineGen::ConnectPathTrim(  CSmtCutPath* pPrev, CSmtCutPath* pCurr, PNT3D p[3], PNT3D intpt )
{
	double dLen = mathDist( p[0], intpt ) ;
	// 裁减第一条路径,从后往前裁减
	TrimSmtCutPath( pPrev, dLen, FALSE ) ;
	dLen = mathDist( p[2], intpt ) ;
	// 裁减第二条路径,从前往后裁减
	TrimSmtCutPath( pCurr, dLen, TRUE  ) ;
	// 将前一个路径中的点加到当前路径中，并删除前一个路径
	ConnectPathDirect( pPrev, pCurr ) ;
}
void CFlowlineGen::TrimSmtCutPath ( CSmtCutPath* pPath, double dLength, BOOL bFlag )
{
	if( dLength > GetSmtPathLength( pPath ) )
		return  ;
	CSmtCutPointEx *Start = NULL, *End = NULL ;
	double dTotal = 0. ;
	double dist   = 0. ;
	if( bFlag )
        Start = (CSmtCutPointEx *)pPath->m_pHead ;
	else
		Start = (CSmtCutPointEx *)pPath->m_pTail ;

	while( Start )
	{
		if( bFlag )
            End = (CSmtCutPointEx *)Start->next ;
		else
			End = (CSmtCutPointEx *)Start->prev ;
        dist = mathDistCutPoint( Start, End ) ;
		dTotal += dist ;
		if( dTotal > dLength ) 
			break ;
		Start = End ;
	}
	
	double del = dTotal - dLength ;
	double u = 1. - del / dist ;
	if( !Start || !End )
		return ;
	CSmtCutPointEx* pNew = new CSmtCutPointEx() ;
	for( int i = 0 ; i < 3 ; i++ )
	{
		pNew->m_fPoint[i] = TFLOAT( Start->m_fPoint[i] + u * ( End->m_fPoint[i] - Start->m_fPoint[i] ) ) ;
		pNew->m_fSurfNor[i] = TFLOAT( Start->m_fSurfNor[i] + u * ( End->m_fSurfNor[i] - Start->m_fSurfNor[i] ) ) ;
	}
	pNew->m_bType = Start->m_bType ;
	
	while( Start )
	{
		if( bFlag )
            End = (CSmtCutPointEx *)Start->prev ;
		else
			End = (CSmtCutPointEx *)Start->next ;
		pPath->DeletePoint ( Start ) ;
		Start = End ;
	}
	if( bFlag )
		pPath->AddHead ( pNew ) ;
	else
		pPath->AddTail ( pNew ) ;
}
BOOL CFlowlineGen::CreateArcCenter ( PNT3D p[3], PNT3D cen )
{
	VEC3D z, v1, v[2] ;
	double len, ang1, ang2 ;
	// 如果长度过短或者得不到方向
	len = mathDist( p[0], p[2] ) ;
	if( len < MIN_LEN || mathGetVecUnit( p[1], p[0], v[0] ) != ERSUCSS || mathGetVecUnit( p[1], p[2], v[1] ) != ERSUCSS )
		return FALSE ;
	// 计算向量v1和向量v2的夹角
	ang1 = mathGetAngleUnit( v[0], v[1] ) ;

    if( ang1 < MIN_ANG || ang1 > PI1 - MIN_ANG )
        return FALSE ;

	mathVProductUnit( v[1], v[0], z ) ;

    ang2 = fabs(ang1 - PI1_2 ) ;

    if( ang2 < MIN_ANG ) // 半圆
	{
        cen[0] = 0.5 * ( p[0][0] + p[2][0] ) ; 
		cen[1] = 0.5 * ( p[0][1] + p[2][1] ) ; 
		cen[2] = 0.5 * ( p[0][2] + p[2][2] ) ; 
	}
    else
    {
        mathGetVecUnit( p[0], p[2], v1 ) ;
        mathVProduct( z, v1, v1 ) ;
        if( ang1 < PI1_2 )
			mathRevVec( v1 ) ;
        double d = 0.5 * len * tan( ang2 ) ;
        cen[0] = 0.5 * ( p[0][0] + p[2][0] ) + v1[0] * d ;
		cen[1] = 0.5 * ( p[0][1] + p[2][1] ) + v1[1] * d ; 
		cen[2] = 0.5 * ( p[0][2] + p[2][2] ) + v1[2] * d ; 
    }

	return TRUE ;
}

BOOL CFlowlineGen::GetSmtPathDir ( CSmtCutPath* pPath, VEC3D v, BOOL bFlag )
{
	PNT3D p1, p2 ;
	int i ;
	CSmtCutPoint *Start = NULL, *End = NULL ;
	Start = pPath->m_pHead ;
	End   = pPath->m_pTail ;
	// 起点
	if( bFlag )
	{
		End = Start->next ;
		while( End )
		{
			if( mathDistCutPoint( Start, End ) > 0.01 )
				break ;
			End = End->next ;
		}
	}
	else
	{
		Start = End->prev ;
		while( Start )
		{
			if( mathDistCutPoint( Start, End ) > 0.01 )
				break ;
			Start = Start->prev ;
		}
	}
	if( !Start || !End ) return FALSE ;
	for( i = 0 ; i < 3 ; i++ )
	{
		p1[i] = double( Start->m_fPoint[i] ) ;
		p2[i] = double( End->m_fPoint[i]   ) ;
	}
	if( !mathGetVecUnit( p1, p2, v ) )
		return FALSE ;
	return TRUE ;
}

BOOL CFlowlineGen::GetPathDirect ( CSmtCutPath* pPrev, CSmtCutPath* pCurr, VEC3D v[2] )
{
	if( !GetSmtPathDir( pPrev, v[0], FALSE ) )
		return FALSE ;
	if( !GetSmtPathDir( pCurr, v[1], TRUE ) )
		return FALSE ;
	for( int i = 0 ; i < 3 ; i++ )
		v[1][i] = -v[1][i] ;
	return TRUE ;
}
												
void CFlowlineGen::TrimCloseSurfPath ( C3DSurfAttribute* pSurf,
										 PNT4D* dArray ) 
{
	if (pSurf->m_cSmtCPathLib.m_cAllPath.IsEmpty())
		return;
	// 将前一条路径中相同层号和列号的路径与第二个路径中的相连
	PNT3D p[3] ;
	VEC3D v[2] ;
	CSmtCutPath * pPath ;
	int i ;
	POSITION pos ;
	pos = pSurf->m_cSmtCPathLib.m_cAllPath.GetHeadPosition () ;
	while( pos )
	{
		// 找到第二个曲面中共边上的路径
		pPath = pSurf->m_cSmtCPathLib.m_cAllPath.GetNext ( pos ) ;
		if( pPath->m_nLayerNo != 1 )
			continue ;
		for( i = 0 ; i < 3 ; i++ )
		{
			p[0][i] = pPath->m_pTail->m_fPoint[i] ;
			p[2][i] = pPath->m_pHead->m_fPoint[i] ;
		}
		pSurf->m_pSurf->GetPoint ( dArray[i][0], dArray[i][1], p[1] ) ;

		// 得到路径的方向
		if( !GetSmtPathDir( pPath, v[0], FALSE ) ||
			!GetSmtPathDir( pPath, v[1], TRUE  ) )
			continue ;
		for( i = 0 ; i < 3 ; i++ )
			v[1][i] = -v[1][i] ;
		// 求交点
		PNT3D intpt1, intpt2 ;
		if( !mathIntLin( p[0], v[0], p[2], v[1], MIN_LEN, MIN_ANG, intpt1, intpt2 ) )
		{
			// 不裁减
			continue ;
		}
		mathGetVec( p[0], intpt1, v[1] ) ;
		double ang = mathGetAngleUnit( v[0], v[1] ) ;
		if( ang > PI1_2 )
		{// 反向裁减连接
			double dLen = mathDist( p[0], intpt1 ) ;
			// 裁减第一条路径,从后往前裁减
			TrimSmtCutPath( pPath, dLen, FALSE ) ;
			dLen = mathDist( p[2], intpt1 ) ;
			// 裁减第二条路径,从前往后裁减
			TrimSmtCutPath( pPath, dLen, TRUE  ) ;
		}
	}
}
BOOL CFlowlineGen::Create5AxUWLinePath( CSmtCheckMdl *DriveMdl, CSmtCPathLib& AllPath, C3DSurfArray& AllSurf ,
									    RFRAME& cRFrame, JDNC_FUWLINE& UWLineCut , JDNC_5DCTRL &c5DCtrl	,
									    C5XGraph &allGraph, JDNC_PRGDEF &PrgDef, CSmartLoop *AllLoop, 
										double dCur, int *pErrorType)
{
	if (NULL == pErrorType)
		return FALSE;
	m_pErrorType = pErrorType;
	
 	if( !DriveMdl ) return FALSE ;
	
	m_dStep     = UWLineCut.m_dOverStep ;
	m_dZMove	= m_cStockDef.m_dDriveZMove[0] - m_cStockDef.m_dSparkGap  ;
	CGeoTrmSurf *pSurf = NULL ;
	m_c5DCtrlDef= c5DCtrl ;
	// 判断曲面的有效性
	GetValidSurf( AllSurf ) ;
	INT_PTR i = 0, nSurf = AllSurf.GetSize () ;
	if( nSurf < 1 )  return FALSE ;
	PrgDef.m_dTotalMove = dCur / nSurf ;

	// 先对所有曲面排序,然后进行加工
	SortAllSurfByDist( AllSurf, m_cSetupDef.m_cTolDef.m_dArcTol * 3 ) ;
	// 然后生成路径
// 	CTrmLoop *pLoop = NULL ;

	// 五轴刀轴控制
	CNc5DAxisCtrl axisCtrl( m_c5DCtrlDef ) ;
	if( axisCtrl.UpdateAxisGraph( allGraph ) == FALSE )
	{// 更新刀轴控制图形
		if (m_pErrorType) *m_pErrorType = JDERROR_GENPATH5AX_NOAXISCURVE;
		return FALSE;
	}
	RFRAME ncMtx  ;
	mathInitRFrame( &ncMtx ) ;
	if( mathIsRFrameUnit(&DriveMdl->m_dNcMtx) == 0 )
	{
		ncMtx = DriveMdl->m_dNcMtx ;
		axisCtrl.m_dNcMtx = &ncMtx ;
	}

	CSmtCPathLib PathLib ;
	CSmtCutPath *pPath = NULL ;
	POSITION pos = NULL, atpos = NULL ;
	for( i = 0 ; i < nSurf ; i++ )
	{
		pSurf = ( CGeoTrmSurf *)AllSurf.GetAt ( i ) ;
		// 生成单张面的五轴流线路径
		if(UWLineCut.m_bUWLineFlag &NCDEF_UWLINE_TOOLPOS )
		{
			if( !CreateOne5AxToolPosPath( DriveMdl, axisCtrl, cRFrame, pSurf,
										UWLineCut, AllLoop, PathLib, PrgDef ) )
			{
				AllPath.ClearAllPath();
				return FALSE;
			}
		}
		else
		{
			if (!CreateOne5AxSurfPath( DriveMdl, axisCtrl, cRFrame, pSurf,
				UWLineCut, AllLoop, PathLib, PrgDef ))
			{
				AllPath.ClearAllPath();
				return FALSE;
			}
		}
		
		// 添加两个路径组之间的连刀路径
		ConnectPathLib(AllPath, PathLib, *DriveMdl, axisCtrl);
		pos = PathLib.m_cAllPath.GetHeadPosition () ;
		while( pos )
		{
			atpos = pos ;
			pPath = PathLib.m_cAllPath.GetNext ( pos ) ;
			PathLib.m_cAllPath.RemoveAt ( atpos ) ;

			pPath->DefineBox () ;
			if( pPath->m_nNumPnt < 2 )
				delete pPath ;
			else
				AllPath.AddToTail ( pPath ) ;
		}
		PathLib.ClearAllPath () ;
	}

	return TRUE ;
}

BOOL CFlowlineGen::CreateOne5AxSurfPath( CSmtCheckMdl *DriveMdl, CNc5DAxisCtrl &axisCtrl, RFRAME& cRFrame, 
										CGeoTrmSurf* pSurf, JDNC_UWLINE& cParam, CSmartLoop *AllLoop, 
										CSmtCPathLib& PathLib, JDNC_PRGDEF &PrgDef )
{
	double uwStep[2], *dParam = NULL ;
	int		nNum = 0, nUW = 0 ;
    int uwCnt[2] ;
	// 计算曲面的剖分精度
    pSurf->GetDiscreteStep( 0.002, uwStep[0], uwStep[1], uwCnt[0], uwCnt[1] ) ;
	
	BOOL bZigZag = FALSE, bSpiral = FALSE ;
	if( cParam.m_bUWLineFlag & NCDEF_UWLINE_ZIGZAG && !m_bGrandMdfy) bZigZag = TRUE ;
	
	// 判断曲线是否闭合,如果闭合并且方向相同,则沿曲面加工
	CSmartPathGen  ncPathGen ;
	JDNC_TOL		cCurveTol = { 0.002, 10, 0, 0, 0, 0 } ;
	CSmartLoop* pLoop = ncPathGen.ExtractSurfLoop( pSurf, cCurveTol ) ;
	if( !pLoop ) return 0 ;
	CalcSurfUWCut( pSurf, pLoop, cParam, uwStep, nUW, bZigZag, bSpiral ) ;
	// 计算路径间距
	if( !Calc5AxCutStep( DriveMdl, &axisCtrl, pSurf, AllLoop, cRFrame, cParam.m_dOverStep, 
						 uwStep, nUW, dParam, nNum, PrgDef ) ) 
	{
		return 0 ;
	}
	
	// 生成参数域上的路径
	CreateLinearNew( dParam, nNum, nUW, pLoop, PathLib, cRFrame, m_cSetupDef.m_cTolDef.m_dArcTol, pSurf, cParam, TRUE) ;
	//CreateLinear( dParam, nNum, nUW, FALSE, pLoop, PathLib ) ;
	
	if( cParam.m_nStartAt && !m_bGrandMdfy)//调整起点位置
		AdjustStartAt( cParam.m_nCutDir, cParam.m_nStartAt, PathLib );

	if( bSpiral && !m_bGrandMdfy )
        AdjustSpiralPath( PathLib ) ;
	ModifyLayerNo(PathLib);		// 把路径组的层号改为原值0

	// 需要多线程的拷贝，则拷贝一份DriveMdl
	BOOL bLocalCopy = FALSE ;	
	INT_PTR nPathNum = PathLib.GetNumPath();
	if (nPathNum>=2 && MathCAM_IsNeedMultiCopy(*DriveMdl, m_nCalcThreadNum))
	{
		bLocalCopy = TRUE ;
		DriveMdl->CreateMultiCopy(m_nCalcThreadNum - 1) ;
	}	

	BOOL bRet = TRUE ;
	// 支持多线程，即拷贝数据不为空	
	CVecPointer NewPath;
	NewPath.resize(nPathNum + 5);
	ZeroMemory(&NewPath[0], sizeof(LPVOID) * (nPathNum + 5));
	if (nPathNum>=2 && DriveMdl->GetMultiCopy() != NULL)
	{
		CO5SPT_DATA ThreadData[NC_CFG_CPU_MAXNUM];
		LPVOID lpParam[NC_CFG_CPU_MAXNUM] = {NULL};
		JDNC_PRGDEF tmpPrg = PrgDef;
		tmpPrg.m_pPrgFunc = NULL ; 
		tmpPrg.m_pPosFunc = NULL ; 
		int nThreadNum = min((int)nPathNum, m_nCalcThreadNum), k=1;
		for (int i=0; i<nThreadNum; i++)
        {
			ThreadData[i].DriveMdl = i == 0 ? DriveMdl : DriveMdl->GetMultiCopy(i-k);
			if (ThreadData[i].DriveMdl == NULL)
			{
				--i, --k, --nThreadNum;
				continue;
			}
			ThreadData[i].nAtCore = i;
			ThreadData[i].nUW = nUW;
			ThreadData[i].uwStep = uwStep;	
			ThreadData[i].PrgDef = i==0 ? PrgDef : tmpPrg;			
			ThreadData[i].cRFrame = &cRFrame;
			ThreadData[i].pSurf = pSurf;
			ThreadData[i].pAxisCtrl = &axisCtrl;
            ThreadData[i].AllLoop = AllLoop;			
			ThreadData[i].AllPath = &PathLib;		
			ThreadData[i].NewPath = &NewPath[0];		
			ThreadData[i].pFlowlineGen = this;
			lpParam[i] = &ThreadData[i];
        }
		for (i=0; i<nThreadNum; i++)
		{
			ThreadData[i].nCoreNum = nThreadNum;
		}
		bRet = MathCAM_ThreadMainFunc(MathCAM_CreateOne5AxSurfPathSubProc, lpParam, nThreadNum);
	}
	else
	{
		bRet = CreateOne5AxSurfPathSubProc(nUW, uwStep, DriveMdl, bZigZag, cRFrame, 
			pSurf, axisCtrl, AllLoop, PathLib, PrgDef, 0, 1, &NewPath[0]);
	}

	PathLib.m_cAllPath.RemoveAll();
	if (bRet)
	{
		Nc5D_AddPathArrayToLib(&NewPath[0], nPathNum, PathLib);
	}

	if( dParam ) delete dParam;
	delete pLoop;

	// DriveMdl的数据在本函数拷贝，则在本函数删除
	if( bLocalCopy == TRUE ) 
	{
		DriveMdl->DeleteMultiCopy() ;
	}
	
	// 对路径进行连接
	if( bRet )
        LinkAll5AxisPath( DriveMdl, PathLib, axisCtrl, bZigZag ) ;


	return bRet ;
}

// 生成单张面的五轴曲面流线路径的主函数
BOOL CFlowlineGen::CreateOne5AxSurfPathSubProc( int nUW, double uwStep[2], CSmtCheckMdl *DriveMdl,
	                                            BOOL bZigZag, RFRAME& cRFrame, CGeoTrmSurf* pSurf, 
												CNc5DAxisCtrl &axisCtrl, CSmartLoop *AllLoop, CSmtCPathLib& PathLib, 
												JDNC_PRGDEF &PrgDef, int nAtCore, int nCoreNum, LPVOID* NewPath)
{
	int nNumLine = PathLib.GetNumPath () ;
	if( nNumLine < 1 ) return TRUE;
	
	PrgDef.m_dIncStep = PrgDef.m_dTotalMove / nNumLine ;
	PrgDef.m_dLimitAt = 1.0 ;

	BOOL bRet = TRUE ;	// 函数执行结果
	TFLOAT fLineTol = 2.0e-4f ;
	if( m_cSetupDef.m_cModelTol.m_nMdlFlag & NCDEF_SETUP_NOMOSAIC ) 
		fLineTol = 1.0e-6f ;
	//将参数域上的路径映射到空间域上,如果bSpiral,并且首末端点距离过远,将两点合并
	double uwLine[2][2] ;
	CSmtCutPath *PathHead = NULL, *PathNext = NULL ;
	BOOL bReverse = FALSE ;
	int nIndex = 0;
	POSITION pos = PathLib.m_cAllPath.GetHeadPosition() ;
	while( pos )
	{// 生成路径
		if( PrgDef.m_pBrkFunc && PrgDef.m_pBrkFunc () )
		{
			bRet = FALSE ;
			if (m_pErrorType) *m_pErrorType = JDERROR_GENPATH_ABORT;
			break;
		}
		MathCAM_MovePrgStep(PrgDef);
		PathHead = PathLib.m_cAllPath.GetNext( pos ) ;
		if (nIndex++ % nCoreNum != nAtCore || PathHead == NULL)
		{
			continue;
		}
		int nLayerNo = PathHead->m_nLayerNo;	// 之前的层号跟之后的层号必须保持不变
		int nLineNo = PathHead->m_nLineNo;		// 之前的行号跟之后的行号必须保持不变
		bReverse = FALSE ;
		if( bZigZag && PathHead->m_bMoveFlag == 1 )
			bReverse = TRUE ;
		uwLine[0][0] = PathHead->m_pHead->m_fPoint[0], uwLine[0][1] = PathHead->m_pHead->m_fPoint[1] ;
		uwLine[1][0] = PathHead->m_pTail->m_fPoint[0], uwLine[1][1] = PathHead->m_pTail->m_fPoint[1] ;
		CSmtCutPath* temp = PathHead->CopyMyself();
		delete PathHead ;		
		PathHead = CreateSGuideTPathNew(temp, *DriveMdl, &axisCtrl, pSurf->m_pSurface, AllLoop, cRFrame,
			uwStep, nUW, m_cSetupDef.m_cTolDef, FALSE, PrgDef ) ;
		delete temp;
		temp = NULL;
		if( !PathHead ) 
		{
			bRet = FALSE ;
			break; ;
		}
		if( m_cSetupDef.m_cOrderDef.m_nMillDir == 1 )
			PathHead->m_bPathFlag |= SMARTNC_CUTPATH_CLIMB ;
		else
			PathHead->m_bPathFlag &= ~SMARTNC_CUTPATH_CLIMB ;

		if( PathHead->IsClosed () )
		{// 优先将无效的点设置为起点,避免封闭路径出现断开现象
			for( CSmtCutPoint *p = PathHead->m_pHead ; p ; p = p->next )
			{
				if( p->m_bType == 0 )
				{
					PathHead->SetClosedPathHead ( p ) ;
					break ;
				}
			}
		}
		PathNext = PathHead->BreakAtNullPoint ( FALSE ) ;
		if( PathHead ) delete PathHead ;
		PathHead = PathNext ;
		NewPath[nIndex - 1] = PathNext;
		while( PathHead )
		{
			PathNext = PathHead->next ;
			PathHead->m_nLayerNo = nLayerNo ;	// 把之前的层号赋给新生成的路径
			PathHead->m_nLineNo = nLineNo ;		// 把之前的行号赋给新生成的路径
			if( bReverse ) PathHead->ReverseDirect () ;
			Nc5D_Simp5xPath( PathHead, fLineTol, 1.0e-4f, ( TFLOAT)m_cSetupDef.m_cTolDef.m_dMaxStep ) ;
			PathHead = PathNext ;
		}		
	}

	if (!bRet)
	{// 计算失败清除数据
		Nc5D_DestroyMultiCalcData(NewPath, nIndex, &PathLib, pos, nCoreNum, nAtCore);
	}

	return bRet;
}

CSmtCutPath *CFlowlineGen::CreateSGuideTPath ( CSmtCheckMdl &DriveMdl, CNc5DAxisCtrl* pAxisCtrl, CSurface *GeoSurf,
											   CSmartLoop *AllLoop, RFRAME &/*lf*/, DOUBLE UWLine[2][2], DOUBLE UWStep[2], 
											   int MoveDir, JDNC_TOL &Tol, BOOL bParam, JDNC_PRGDEF &ProgDef )
{
	DOUBLE dAngStep = ANGLE_TO_RADIAN( 3.0 ) ;
	if( pAxisCtrl )
	{
		dAngStep = ANGLE_TO_RADIAN( pAxisCtrl->m_cToolAxis.m_dMaxAngleStep ) ;
	}
	double dAng = 0.0;
	if( MoveDir == 0 ) 
	{
		dAng = atan2( 0.0,UWLine[1][0] - UWLine[0][0] ) ;
	}
	else if( MoveDir == 1 )
	{
		dAng = atan2( UWLine[1][1]- UWLine[0][1],0.0) ;
	}
	else
	{
		dAng = atan2( UWLine[1][1]- UWLine[0][1],UWLine[1][0] - UWLine[0][0] ) ;
	}
	CSmtCutPath *pPath = new CSmtCutPath( MINI_MILL5AX_PATH ) ;
	double dAngRot[2] ;
	dAngRot[0]  = cos( dAng ) , dAngRot[1] = sin( dAng ) ;
	int nCnt1 = (int)fabs( (UWLine[1][0] - UWLine[0][0] ) / UWStep[0] ) ;
	int nCnt2 = (int)fabs( (UWLine[1][1] - UWLine[0][1] ) / UWStep[1] ) ;
	int nCnt = max( nCnt1, nCnt2 ) + 2 ;
	double uwInc[2], uw[2],tInc ;
	uwInc[0] = (UWLine[1][0] - UWLine[0][0] ) / (nCnt-1) ;
	uwInc[1] = (UWLine[1][1] - UWLine[0][1] ) / (nCnt-1) ;
	tInc = 1.0 / (nCnt-1) ;
	uw[0] = UWLine[0][0] , uw[1] = UWLine[0][1] ;
	CSmtCutPointEx* pTPnt ;
	RFRAME  *ncMtx = NULL ; 
	if( mathIsRFrameUnit( &DriveMdl.m_dNcMtx) == 0 )
	{
		ncMtx = &DriveMdl.m_dNcMtx ;
	}
	for( int i = 0 ; i < nCnt ; i ++ ) 
	{
		if( ProgDef.m_pBrkFunc && ProgDef.m_pBrkFunc() )
		{
			delete pPath ;
			if (m_pErrorType) *m_pErrorType = JDERROR_GENPATH_ABORT;
			return NULL ;
		}
		pTPnt = new CSmtCutPointEx() ;
		if( i == nCnt-1 )
		{
			uw[0] = UWLine[1][0], uw[1] = UWLine[1][1] ;
		}
		Gen5AxisPoint( GeoSurf, pAxisCtrl, uw, ncMtx, dAngRot, *pTPnt, bParam ) ;
		pTPnt->m_fPoint[3] = TFLOAT( i * tInc) ;
		pPath->AddTail( pTPnt ) ;
		uw[0] += uwInc[0], uw[1] += uwInc[1] ;
	}
	TFLOAT fLineTol = 2.0e-4f ;
	if( m_cSetupDef.m_cModelTol.m_nMdlFlag & NCDEF_SETUP_NOMOSAIC ) 
		fLineTol = 1.0e-6f ;
	Nc5D_Simp5xPath( pPath, fLineTol, (TFLOAT)dAngStep, (TFLOAT)Tol.m_dMaxStep ) ;
	
	double dDist = Tol.m_dMaxStep ;
	if( m_cSetupDef.m_cModelTol.m_nMdlFlag & NCDEF_SETUP_NOMOSAIC )
		dDist = min( dDist, 0.15 ) ;
	// STEP 2 : 曲线跌代,确保在导动曲面上的角度误差和弦高误差
	double dMinCos = cos( dAngStep), dCosA ;
	CSmtCutPointEx tmpTPnt, *pTNext = NULL ;
	for( pTPnt = (CSmtCutPointEx*)pPath->m_pHead ; pTPnt && pTPnt->next;  )
	{
		if( ProgDef.m_pBrkFunc && ProgDef.m_pBrkFunc() )
		{
			delete pPath ;
			if (m_pErrorType) *m_pErrorType = JDERROR_GENPATH_ABORT;
			return NULL ;
		}
		pTNext = ( CSmtCutPointEx*) pTPnt->next ;
		if( fabs( pTPnt->m_fPoint[3] - pTNext->m_fPoint[3] ) < 2.0e-5  )
		{
			pTPnt = pTNext ; 
			continue ;
		}
		tInc = ( pTPnt->m_fPoint[3] + pTNext->m_fPoint[3] ) * 0.5 ;
		uw[0] = UWLine[0][0] + tInc * ( UWLine[1][0] - UWLine[0][0] ) ;
		uw[1] = UWLine[0][1] + tInc * ( UWLine[1][1] - UWLine[0][1] ) ;
		Gen5AxisPoint( GeoSurf, pAxisCtrl, uw, ncMtx, dAngRot, tmpTPnt, bParam ) ;

		tmpTPnt.m_fPoint[3] = TFLOAT( tInc )  ;
		dCosA = nc_OProduct( pTPnt->m_fSurfNor, pTNext->m_fSurfNor, 3 ) ;
		if( dCosA < dMinCos ||
			nc_Distance( pTPnt->m_fPoint, pTNext->m_fPoint,3 ) > dDist ||
			nc_PointLineDist( pTPnt->m_fPoint, pTNext->m_fPoint, tmpTPnt.m_fPoint ) > Tol.m_dArcTol )
		{
			CSmtCutPointEx* pTNew = new CSmtCutPointEx( tmpTPnt ) ;
			pPath->InsertAfter( pTNew, pTPnt ) ;
		}
		else 
		{
			pTPnt = pTNext ;
			continue ;
		}
	}
	// STEP 3 : 加工模型法向投影路径点，确保在加工模型上的误差
	for( pTPnt = (CSmtCutPointEx*)pPath->m_pHead ; pTPnt ; pTPnt = (CSmtCutPointEx*)pTPnt->next )
	{
		if( ProgDef.m_pBrkFunc && ProgDef.m_pBrkFunc() )
		{
			delete pPath ;
			if (m_pErrorType) *m_pErrorType = JDERROR_GENPATH_ABORT;
			return NULL ;
		}
		Def5AxisPoint( DriveMdl, *pTPnt, Tol, AllLoop ) ;
	}
	for( pTPnt = (CSmtCutPointEx*)pPath->m_pHead ; pTPnt && pTPnt->next;  )
	{
		if( ProgDef.m_pBrkFunc && ProgDef.m_pBrkFunc() )
		{
			delete pPath ;
			if (m_pErrorType) *m_pErrorType = JDERROR_GENPATH_ABORT;
			return NULL ;
		}
		pTNext = ( CSmtCutPointEx*) pTPnt->next ;
		if( pTPnt->m_bType <= 0 && pTNext->m_bType <= 0 )
		{//无效点
			pTPnt = pTNext ; 
			continue ;
		}
		if( fabs( pTPnt->m_fPoint[3] - pTNext->m_fPoint[3] ) < 2.0e-5  || 
			nc_Distance(pTPnt->m_fPoint,pTNext->m_fPoint, 3 ) < 0.02   )
		{//接近点，无须插点
			pTPnt = pTNext ; 
			continue ;
		}
		tInc = ( pTPnt->m_fPoint[3] + pTNext->m_fPoint[3] ) * 0.5 ;
		uw[0] = UWLine[0][0] + tInc * ( UWLine[1][0] - UWLine[0][0] ) ;
		uw[1] = UWLine[0][1] + tInc * ( UWLine[1][1] - UWLine[0][1] ) ;
		Gen5AxisPoint( GeoSurf, pAxisCtrl, uw, ncMtx,dAngRot, tmpTPnt, bParam ) ;

		tmpTPnt.m_fPoint[3] = TFLOAT( tInc )  ;
		Def5AxisPoint( DriveMdl, tmpTPnt, Tol, AllLoop ) ;

		if( nc_Distance( pTPnt->m_fPoint, pTNext->m_fPoint,3 ) > Tol.m_dMaxStep ||
			nc_PointLineDist( pTPnt->m_fPoint, pTNext->m_fPoint, tmpTPnt.m_fPoint ) > Tol.m_dArcTol )
		{
			CSmtCutPointEx* pTNew = new CSmtCutPointEx( tmpTPnt ) ;
			pPath->InsertAfter( pTNew, pTPnt ) ;
		}
		else 
		{
			pTPnt = pTNext ;
			continue ;
		}
	}
	// STEP 4 : 精确迭代有效点与无效点的分解位置
	CSmtCutPointEx startAt, endAt, *pTObj ;
	pTNext = NULL ; 
	for( pTPnt = (CSmtCutPointEx*)pPath->m_pHead ; pTPnt && pTPnt->next;  pTPnt = pTNext)
	{
		if( ProgDef.m_pBrkFunc && ProgDef.m_pBrkFunc() )
		{
			delete pPath ;
			if (m_pErrorType) *m_pErrorType = JDERROR_GENPATH_ABORT;
			return NULL ;
		}
		pTNext = ( CSmtCutPointEx*) pTPnt->next ;
		if( pTPnt->m_bType <= 0 && pTNext->m_bType > 0 )
		{//无效点
			startAt = *pTPnt, endAt = *pTNext, pTObj = pTNext ;
		}
		else if( pTPnt->m_bType > 0 && pTNext->m_bType <= 0 )
		{
			startAt = *pTNext, endAt = *pTPnt, pTObj = pTPnt ;
		}
		else 
		{
			continue ;
		}
		double fDist = nc_Distance( startAt.m_fPoint, endAt.m_fPoint, 3 ) ;
		while( fDist > Tol.m_dArcTol * 0.5 ) 
		{
			tInc = ( startAt.m_fPoint[3] + endAt.m_fPoint[3] ) * 0.5 ;
			uw[0] = UWLine[0][0] + tInc * ( UWLine[1][0] - UWLine[0][0] ) ;
			uw[1] = UWLine[0][1] + tInc * ( UWLine[1][1] - UWLine[0][1] ) ;
			Gen5AxisPoint( GeoSurf, pAxisCtrl, uw, ncMtx, dAngRot,tmpTPnt, bParam ) ;

			tmpTPnt.m_fPoint[3] = TFLOAT( tInc )  ;
			Def5AxisPoint( DriveMdl, tmpTPnt, Tol, AllLoop ) ;
			fDist *= 0.5f  ;
			if( tmpTPnt.m_bType <= 0 ) startAt = tmpTPnt ;
			else endAt = tmpTPnt ;
		}
		if( nc_Distance( endAt.m_fPoint , pTObj->m_fPoint , 3 ) > Tol.m_dArcTol * 0.5 )
		{
			CSmtCutPointEx* pTNew = new CSmtCutPointEx( endAt ) ;
			pPath->InsertAfter( pTNew, pTPnt ) ;
		}
	}
	// STEP 5 : 断开无效点
	pTNext = NULL ; 
	for( pTPnt = (CSmtCutPointEx*)pPath->m_pHead ; pTPnt && pTPnt->next;  pTPnt = pTNext)
	{
		if( ProgDef.m_pBrkFunc && ProgDef.m_pBrkFunc() )
		{
			delete pPath ;
			if (m_pErrorType) *m_pErrorType = JDERROR_GENPATH_ABORT;
			return NULL ;
		}
		pTNext = ( CSmtCutPointEx*) pTPnt->next ;
		if( pTPnt->m_bType <= 0 || pTNext->m_bType <= 0 )
		{
			continue ;
		}
		BOOL bValidLine = TRUE ;
		double dA = nc_ACos( nc_OProduct( pTPnt->m_fSurfNor, pTNext->m_fSurfNor, 3 )) ;

		Nc5D_CalcMidPoint5Ax( *pTPnt, *pTNext, 0.5f, 0, tmpTPnt ) ;
		if( dA > ANGLE_TO_RADIAN( 6.0 ) )
		{//角度过大，线段无效
			bValidLine = FALSE ;
		}
		
		if( bValidLine == FALSE )
		{
			CSmtCutPointEx* pTNew = new CSmtCutPointEx( tmpTPnt ) ;
			pTNew->m_bType = 0 ;
			pPath->InsertAfter( pTNew, pTPnt ) ;
		}
	}
	for( pTPnt = (CSmtCutPointEx*)pPath->m_pHead ; pTPnt && pTPnt->next;  pTPnt = pTNext)
	{//删除孤立的有效点
		pTNext = ( CSmtCutPointEx*) pTPnt->next ;
		if( pTPnt->m_bType == 0 ) continue ;
		if( (pTPnt->prev == NULL || pTPnt->prev->m_bType == 0 ) &&
			(pTPnt->next == NULL || pTPnt->next->m_bType == 0 )   )
		{
			pTPnt->m_bType = 0 ;
		}
	}
	// test
/*	for( pTPnt = (CSmtCutPointEx*)pPath->m_pHead ; pTPnt && pTPnt->next;  pTPnt = pTNext)
	{//删除孤立的有效点
		pTNext = ( CSmtCutPointEx*) pTPnt->next ;	
		DriveMdl.Define5AXHeight ( *pTPnt ) ;
		if( DriveMdl.Is5AxPointOvercut( *pTPnt, Tol.m_dArcTol*2.0 ) )
		{
			ASSERT( 0 ) ;
		}
	}*/
	return pPath ;
}

CSmtCutPath *CFlowlineGen::CreateSGuideTPathNew (CSmtCutPath* temp, CSmtCheckMdl &DriveMdl, CNc5DAxisCtrl* pAxisCtrl, CSurface *GeoSurf,
											   CSmartLoop *AllLoop, RFRAME &/*lf*/, DOUBLE UWStep[2], 
											   int MoveDir, JDNC_TOL &Tol, BOOL bParam, JDNC_PRGDEF &ProgDef )
{
	CSmtCutPath *finalPath = new CSmtCutPath( MINI_MILL5AX_PATH ) ;
	DOUBLE UWLine[2][2];
	CSmtCutPointEx* pTPnt ;
	CSmtCutPoint* pt = temp->m_pHead;
	DOUBLE dAngStep = ANGLE_TO_RADIAN( 3.0 ) ;
	TFLOAT fLineTol = 2.0e-4f ;

	while(pt && pt->next)
	{
		UWLine[0][0] =pt->m_fPoint[0], UWLine[0][1] = pt->m_fPoint[1] ;
		UWLine[1][0] = pt->next->m_fPoint[0], UWLine[1][1] = pt->next->m_fPoint[1] ;
		pt = pt->next;
		
		if( pAxisCtrl )
		{
			dAngStep = ANGLE_TO_RADIAN( pAxisCtrl->m_cToolAxis.m_dMaxAngleStep ) ;
		}
		double dAng = 0.0;
		if( MoveDir == 0 ) 
		{
			dAng = atan2( 0.0,UWLine[1][0] - UWLine[0][0] ) ;
		}
		else if( MoveDir == 1 )
		{
			dAng = atan2( UWLine[1][1]- UWLine[0][1],0.0) ;
		}
		else
		{
			dAng = atan2( UWLine[1][1]- UWLine[0][1],UWLine[1][0] - UWLine[0][0] ) ;
		}
		CSmtCutPath *pPath = new CSmtCutPath( MINI_MILL5AX_PATH ) ;
		double dAngRot[2] ;
		dAngRot[0]  = cos( dAng ) , dAngRot[1] = sin( dAng ) ;
		int nCnt1 = (int)fabs( (UWLine[1][0] - UWLine[0][0] ) / UWStep[0] ) ;
		int nCnt2 = (int)fabs( (UWLine[1][1] - UWLine[0][1] ) / UWStep[1] ) ;
		int nCnt = max( nCnt1, nCnt2 ) + 2 ;
		double uwInc[2], uw[2],tInc ;
		uwInc[0] = (UWLine[1][0] - UWLine[0][0] ) / (nCnt-1) ;
		uwInc[1] = (UWLine[1][1] - UWLine[0][1] ) / (nCnt-1) ;
		tInc = 1.0 / (nCnt-1) ;
		uw[0] = UWLine[0][0] , uw[1] = UWLine[0][1] ;
		
		RFRAME  *ncMtx = NULL ; 
		if( mathIsRFrameUnit( &DriveMdl.m_dNcMtx) == 0 )
		{
			ncMtx = &DriveMdl.m_dNcMtx ;
		}

		// 此步计算出参数域上进行插点后对应的曲面点
		for( int i = 0 ; i < nCnt ; i ++ ) 
		{
			if( ProgDef.m_pBrkFunc && ProgDef.m_pBrkFunc() )
			{
				delete pPath ;
				if (m_pErrorType) *m_pErrorType = JDERROR_GENPATH_ABORT;
				return NULL ;
			}
			pTPnt = new CSmtCutPointEx() ;
			if( i == nCnt-1 )
			{
				uw[0] = UWLine[1][0], uw[1] = UWLine[1][1] ;
			}
			Gen5AxisPoint( GeoSurf, pAxisCtrl, uw, ncMtx, dAngRot, *pTPnt, bParam ) ;
			pTPnt->m_fPoint[3] = TFLOAT( i * tInc) ;//该值为改点在该段上的参数域值 不是整段路径的参数域值
			pPath->AddTail( pTPnt ) ;
			uw[0] += uwInc[0], uw[1] += uwInc[1] ;
		}
		
		if( m_cSetupDef.m_cModelTol.m_nMdlFlag & NCDEF_SETUP_NOMOSAIC ) 
			fLineTol = 1.0e-6f ;
		Nc5D_Simp5xPath( pPath, fLineTol, (TFLOAT)dAngStep, (TFLOAT)Tol.m_dMaxStep ) ;

		double dDist = Tol.m_dMaxStep ;
		if( m_cSetupDef.m_cModelTol.m_nMdlFlag & NCDEF_SETUP_NOMOSAIC )
			dDist = min( dDist, 0.15 ) ;
		// STEP 2 : 曲线跌代,确保在导动曲面上的角度误差和弦高误差
		double dMinCos = cos( dAngStep), dCosA ;
		CSmtCutPointEx tmpTPnt, *pTNext = NULL ;
		for( pTPnt = (CSmtCutPointEx*)pPath->m_pHead ; pTPnt && pTPnt->next;  )
		{
			if( ProgDef.m_pBrkFunc && ProgDef.m_pBrkFunc() )
			{
				delete pPath ;
				if (m_pErrorType) *m_pErrorType = JDERROR_GENPATH_ABORT;
				return NULL ;
			}
			pTNext = ( CSmtCutPointEx*) pTPnt->next ;
			if( fabs( pTPnt->m_fPoint[3] - pTNext->m_fPoint[3] ) < 2.0e-5  )
			{
				pTPnt = pTNext ; 
				continue ;
			}
			tInc = ( pTPnt->m_fPoint[3] + pTNext->m_fPoint[3] ) * 0.5 ;
			uw[0] = UWLine[0][0] + tInc * ( UWLine[1][0] - UWLine[0][0] ) ;
			uw[1] = UWLine[0][1] + tInc * ( UWLine[1][1] - UWLine[0][1] ) ;
			Gen5AxisPoint( GeoSurf, pAxisCtrl, uw, ncMtx, dAngRot, tmpTPnt, bParam ) ;

			tmpTPnt.m_fPoint[3] = TFLOAT( tInc )  ;
			dCosA = nc_OProduct( pTPnt->m_fSurfNor, pTNext->m_fSurfNor, 3 ) ;
			if( dCosA < dMinCos ||
				nc_Distance( pTPnt->m_fPoint, pTNext->m_fPoint,3 ) > dDist ||
				nc_PointLineDist( pTPnt->m_fPoint, pTNext->m_fPoint, tmpTPnt.m_fPoint ) > Tol.m_dArcTol )
			{
				CSmtCutPointEx* pTNew = new CSmtCutPointEx( tmpTPnt ) ;
				pPath->InsertAfter( pTNew, pTPnt ) ;
			}
			else 
			{
				pTPnt = pTNext ;
				continue ;
			}
		}
		Nc5D_Simp5xPath( pPath, fLineTol, (TFLOAT)dAngStep, (TFLOAT)Tol.m_dMaxStep ) ;
	
	//pPath->DelPointOverlap();
	//finalPath->DelPointOnLine();
	//finalPath->NormalizeLen();

		// STEP 3 : 加工模型法向投影路径点，确保在加工模型上的误差
		for( pTPnt = (CSmtCutPointEx*)pPath->m_pHead ; pTPnt ; pTPnt = (CSmtCutPointEx*)pTPnt->next )
		{
			if( ProgDef.m_pBrkFunc && ProgDef.m_pBrkFunc() )
			{
				delete pPath ;
				if (m_pErrorType) *m_pErrorType = JDERROR_GENPATH_ABORT;
				return NULL ;
			}
			Def5AxisPoint( DriveMdl, *pTPnt, Tol, AllLoop ) ;
		}
		for( pTPnt = (CSmtCutPointEx*)pPath->m_pHead ; pTPnt && pTPnt->next;  )
		{
			if( ProgDef.m_pBrkFunc && ProgDef.m_pBrkFunc() )
			{
				delete pPath ;
				if (m_pErrorType) *m_pErrorType = JDERROR_GENPATH_ABORT;
				return NULL ;
			}
			pTNext = ( CSmtCutPointEx*) pTPnt->next ;
			if( pTPnt->m_bType <= 0 && pTNext->m_bType <= 0 )
			{//无效点
				pTPnt = pTNext ; 
				continue ;
			}
			if( fabs( pTPnt->m_fPoint[3] - pTNext->m_fPoint[3] ) < 2.0e-5  || 
				nc_Distance(pTPnt->m_fPoint,pTNext->m_fPoint, 3 ) < 0.02   )
			{//接近点，无须插点
				pTPnt = pTNext ; 
				continue ;
			}
			tInc = ( pTPnt->m_fPoint[3] + pTNext->m_fPoint[3] ) * 0.5 ;
			uw[0] = UWLine[0][0] + tInc * ( UWLine[1][0] - UWLine[0][0] ) ;// 此处的uwline值为最后一段路径的uwline值，所以通过该方法得出的uw值不正确，导致插点错误
			uw[1] = UWLine[0][1] + tInc * ( UWLine[1][1] - UWLine[0][1] ) ;
			Gen5AxisPoint( GeoSurf, pAxisCtrl, uw, ncMtx,dAngRot, tmpTPnt, bParam ) ;

			tmpTPnt.m_fPoint[3] = TFLOAT( tInc )  ;
			Def5AxisPoint( DriveMdl, tmpTPnt, Tol, AllLoop ) ;

			if( nc_Distance( pTPnt->m_fPoint, pTNext->m_fPoint,3 ) > Tol.m_dMaxStep ||
				nc_PointLineDist( pTPnt->m_fPoint, pTNext->m_fPoint, tmpTPnt.m_fPoint ) > Tol.m_dArcTol )
			{
				CSmtCutPointEx* pTNew = new CSmtCutPointEx( tmpTPnt ) ;
				pPath->InsertAfter( pTNew, pTPnt ) ;
			}
			else 
			{
				pTPnt = pTNext ;
				continue ;
			}
		}
		Nc5D_Simp5xPath( pPath, fLineTol, (TFLOAT)dAngStep, (TFLOAT)Tol.m_dMaxStep ) ;
		// STEP 4 : 精确迭代有效点与无效点的分解位置
		CSmtCutPointEx startAt, endAt, *pTObj ;
		pTNext = NULL ; 
		for( pTPnt = (CSmtCutPointEx*)pPath->m_pHead ; pTPnt && pTPnt->next;  pTPnt = pTNext)
		{
			if( ProgDef.m_pBrkFunc && ProgDef.m_pBrkFunc() )
			{
				delete pPath ;
				if (m_pErrorType) *m_pErrorType = JDERROR_GENPATH_ABORT;
				return NULL ;
			}
			pTNext = ( CSmtCutPointEx*) pTPnt->next ;
			if( pTPnt->m_bType <= 0 && pTNext->m_bType > 0 )
			{//无效点
				startAt = *pTPnt, endAt = *pTNext, pTObj = pTNext ;
			}
			else if( pTPnt->m_bType > 0 && pTNext->m_bType <= 0 )
			{
				startAt = *pTNext, endAt = *pTPnt, pTObj = pTPnt ;
			}
			else 
			{
				continue ;
			}
			double fDist = nc_Distance( startAt.m_fPoint, endAt.m_fPoint, 3 ) ;
			while( fDist > Tol.m_dArcTol * 0.5 ) 
			{
				tInc = ( startAt.m_fPoint[3] + endAt.m_fPoint[3] ) * 0.5 ;
				uw[0] = UWLine[0][0] + tInc * ( UWLine[1][0] - UWLine[0][0] ) ;
				uw[1] = UWLine[0][1] + tInc * ( UWLine[1][1] - UWLine[0][1] ) ;
				Gen5AxisPoint( GeoSurf, pAxisCtrl, uw, ncMtx, dAngRot,tmpTPnt, bParam ) ;

				tmpTPnt.m_fPoint[3] = TFLOAT( tInc )  ;
				Def5AxisPoint( DriveMdl, tmpTPnt, Tol, AllLoop ) ;
				fDist *= 0.5f  ;
				if( tmpTPnt.m_bType <= 0 ) startAt = tmpTPnt ;
				else endAt = tmpTPnt ;
			}
			if( nc_Distance( endAt.m_fPoint , pTObj->m_fPoint , 3 ) > Tol.m_dArcTol * 0.5 )
			{
				CSmtCutPointEx* pTNew = new CSmtCutPointEx( endAt ) ;
				pPath->InsertAfter( pTNew, pTPnt ) ;
			}
		}
		// STEP 5 : 断开无效点
		pTNext = NULL ; 
		for( pTPnt = (CSmtCutPointEx*)pPath->m_pHead ; pTPnt && pTPnt->next;  pTPnt = pTNext)
		{
			if( ProgDef.m_pBrkFunc && ProgDef.m_pBrkFunc() )
			{
				delete pPath ;
				if (m_pErrorType) *m_pErrorType = JDERROR_GENPATH_ABORT;
				return NULL ;
			}
			pTNext = ( CSmtCutPointEx*) pTPnt->next ;
			if( pTPnt->m_bType <= 0 || pTNext->m_bType <= 0 )
			{
				continue ;
			}
			BOOL bValidLine = TRUE ;
			double dA = nc_ACos( nc_OProduct( pTPnt->m_fSurfNor, pTNext->m_fSurfNor, 3 )) ;

			//Nc5D_CalcMidPoint5Ax( *pTPnt, *pTNext, 0.5f, 0, tmpTPnt ) ;
			if( dA > ANGLE_TO_RADIAN( 6.0 ) )
			{//角度过大，线段无效
				bValidLine = FALSE ;
			}
		
			if( bValidLine == FALSE )
			{
				CSmtCutPointEx* pTNew = new CSmtCutPointEx( tmpTPnt ) ;
				pTNew->m_bType = 0 ;
				pPath->InsertAfter( pTNew, pTPnt ) ;
			}
		}
		for( pTPnt = (CSmtCutPointEx*)pPath->m_pHead ; pTPnt && pTPnt->next;  pTPnt = pTNext)
		{//删除孤立的有效点
			pTNext = ( CSmtCutPointEx*) pTPnt->next ;
			if( pTPnt->m_bType == 0 ) continue ;
			if( (pTPnt->prev == NULL || pTPnt->prev->m_bType == 0 ) &&
				(pTPnt->next == NULL || pTPnt->next->m_bType == 0 )   )
			{
				pTPnt->m_bType = 0 ;
			}
		}


	 	CSmtCutPoint* tempPoint = pPath->m_pHead;
	 	while(tempPoint)
	 	{
			finalPath->AddTail(tempPoint->CopyMyself());
	 		tempPoint = tempPoint->next;					
	 	}
	 	delete pPath;
	 	pPath = NULL;
	}
	//	
	// test
/*	for( pTPnt = (CSmtCutPointEx*)pPath->m_pHead ; pTPnt && pTPnt->next;  pTPnt = pTNext)
	{//删除孤立的有效点
		pTNext = ( CSmtCutPointEx*) pTPnt->next ;	
		DriveMdl.Define5AXHeight ( *pTPnt ) ;
		if( DriveMdl.Is5AxPointOvercut( *pTPnt, Tol.m_dArcTol*2.0 ) )
		{
			ASSERT( 0 ) ;
		}
	}*/
	return finalPath ;
}

BOOL CFlowlineGen::Gen5AxisPoint( CSurface *GeoSurf, CNc5DAxisCtrl* pAxisCtrl, 
								  double uwAt[2], RFRAME *NcMtx, double AngRot[2], 
								  CSmtCutPointEx &CutPoint, BOOL bParam, BOOL bToolPos )
{
	if( pAxisCtrl == NULL || GeoSurf == NULL ) 
	{
		return FALSE ;
	}
	// STEP 1 : 计算投影方向和初始点
	PNT3D dPoint ;
	VEC3D dPrjDir, dNormal, du, dw, dTan, dTAxis ;
	// 该函数在退化点处经常返回000
	GeoSurf->GetDerivative10( NcMtx, uwAt[0], uwAt[1], dPoint, du, dw, dNormal ) ;
	
	if( mathVecLen( dNormal ) < 1.0e-6 )
	{
		GeoSurf->GetNormal( NcMtx, uwAt[0], uwAt[1], dPoint, dNormal ) ;
	}
	nc_VectorCopy( dPrjDir, dNormal, 3 ) ;
	
	// STEP 2 : 计算刀轴方向
	BOOL bTiltedMode = FALSE ;
	if( pAxisCtrl->m_cToolAxis.m_nAxisType == NCDEF_AXIS_TILTED_TO_CUTDIR )
	{
		bTiltedMode = TRUE ;
	}
	if( bTiltedMode && (pAxisCtrl->m_cToolAxis.m_nAxisFlag & NCDEF_AXIS_FLAG_TILTED_LOWEDGE) )
	{
		PNT3D dTmpPnt ;
		GeoSurf->GetDerivative10( NcMtx,uwAt[0], 1.0, dTmpPnt, du, dw, dNormal ) ;
		if( mathVecLen( dNormal ) < 1.0e-6 )
            GeoSurf->GetNormal ( NcMtx, uwAt[0], 1., dTmpPnt, dNormal ) ;
	}
	int k = 0 ;
	if( bToolPos )
	{
		dTan[0] = AngRot[0], dTan[1] = AngRot[1], dTan[2] = 0. ;
	}
	else
	{
		for( k = 0 ; k < 3; k ++ ) 
		{
			dTan[k] = du[k] * AngRot[0] + dw[k] * AngRot[1]  ; 
		}
	}
	
	if( bTiltedMode && (pAxisCtrl->m_cToolAxis.m_nAxisFlag & NCDEF_AXIS_FLAG_TILTED_ISODIR) )
	{//侧铣投影模式
		mathVProduct( dw, dNormal, dTAxis ) ;
		if( mathOProduct( dTAxis, dTan ) < 0.0 )
		{
			mathRevVec( dTAxis ) ;
		}
		mathCpyPnt( dTAxis, dTan ) ;
	}
	mathUniVec( dTan, 1.0e-10 ) ;
// 	if( bTiltedMode && pAxisCtrl->m_cAxisApp.m_nFormAxisType != NCDEF_FORMDIR_SURFNORM )
// 	{
// 		if( pAxisCtrl->m_cAxisApp.m_nFormAxisType == NCDEF_FORMDIR_VERT )
// 		{
// 			dNormal[0] = dNormal[1] = 0.0 , dNormal[2] = 1.0 ;
// 		}
// 		else if( pAxisCtrl->m_cAxisApp.m_nFormAxisType == NCDEF_FORMDIR_USERDIR )
// 		{
// 			nc_VectorCopy( dNormal, pAxisCtrl->m_cAxisApp.m_dUserTAxis, 3 ) ;
// 		}
// 	}

	if( !pAxisCtrl->AdjustToolAxisEx( dPoint, dPoint, uwAt, dTan, dNormal, dTAxis ))
	{
		mathCpyPnt( dNormal, dTAxis ) ;
	}
	
	// STEP 3 : 设置加工路径点
	CutPoint.m_bType = 0 ; 
	nc_DoubleToFloat( CutPoint.m_fPoint  , dPoint , 3 ) ;
	nc_DoubleToFloat( CutPoint.m_fSurfNor, dTAxis, 3 ) ;
	if( bParam )
	{
		CutPoint.m_fSurfPos[0] = TFLOAT( uwAt[0] ) ;
        CutPoint.m_fSurfPos[1] = TFLOAT( uwAt[1] ) ;
		nc_DoubleToFloat( CutPoint.m_fTempPos, dNormal, 3 ) ;
		if (IsSet3DRCompMask())
		{
			nc_DoubleToFloat( CutPoint.m_fSurfPos, dPrjDir, 3 ) ; // 将五轴路径中的曲面法向存储在SurfPos中
		}
	}
	else
	{
		nc_DoubleToFloat( CutPoint.m_fSurfPos, dPrjDir, 3 ) ;
	}
	// 计算刀具偏移
	PNT3D Offset = {0} ;
	double dLen = mathOProduct( dNormal, dTAxis ) ;
	if( fabs( dLen - 1 ) > MIN_LEN )
	{
		m_pMiller->Get5AxToolOffset ( dTAxis, dNormal, Offset ) ;
    }
	
    Offset[0] += dTAxis[0] * m_cStockDef.m_dDriveZMove[0];
    Offset[1] += dTAxis[1] * m_cStockDef.m_dDriveZMove[0];
    Offset[2] += dTAxis[2] * m_cStockDef.m_dDriveZMove[0];
	for( k = 0 ; k < 3 ; k ++ ) 
	{//刀心转换成刀尖
		CutPoint.m_fPoint[k] += TFLOAT( Offset[k] ) ;
	}
	/**/
    return TRUE ;
}

// 投影多轴加工路径点
BOOL CFlowlineGen::Def5AxisPoint( CSmtCheckMdl &DriveMdl, CSmtCutPointEx &CutPoint, JDNC_TOL &Tol, CSmartLoop *AllLoop )
{
	CutPoint.m_bType = 0 ;    
    //判断是否过切
	if( !DriveMdl.Is5AxPointOvercut ( CutPoint, Tol.m_dArcTol * 2. ) )
	{
		CutPoint.m_bType = 5 ;
	}
	if( CutPoint.m_bType && DriveMdl.m_pHolderShape )
	{
		if( DriveMdl.Is5AxPointCutHolder ( CutPoint, 0.0002 ) )
		{
			CutPoint.m_bType = 0 ;
		}
	}
    
	// 判断点是否在轮廓线以内
	if (CutPoint.m_bType > 0 && AllLoop)
	{
		PNT2D CP2D = {CutPoint.m_fPoint[0], CutPoint.m_fPoint[1]};
		BOOL bPointInContour = FALSE;
		for (CSmartLoop *pLoop = AllLoop; pLoop; pLoop = pLoop->next)
		{
			if (pLoop->IsOnContourEx(CP2D) != 0)
			{
				bPointInContour = TRUE;
				break;
			}
		}
		if (bPointInContour == FALSE)
			CutPoint.m_bType = 0 ;
	}
    return TRUE ;
}
void CFlowlineGen::LinkAll5AxisPath ( CSmtCheckMdl *DriveMdl, CSmtCPathLib &AllPath, CNc5DAxisCtrl &axisCtrl, BOOL bZigZag )
{
	if( AllPath.m_cAllPath.GetCount () < 1 ) return ;

	// 对路径进行连接处理
	double dDriveOffset = m_cStockDef.m_dDriveOffset[0] - m_cStockDef.m_dSparkGap ;
	double dCheckOffset = m_cStockDef.m_dCheckOffset[0] - m_cStockDef.m_dSparkGap ;
	CSmartTool* pNewTool = NULL , *pNewSafe = NULL, *pTool = NULL, *pSafe = NULL ; 
	pTool = DriveMdl->m_pTool ;
	if( DriveMdl->m_pCheckMdl )
		pSafe = DriveMdl->m_pCheckMdl->m_pTool ;

    pNewTool = SmartNC_CreateTool( m_cToolDef, dDriveOffset - 0.01, m_cStockDef.m_dPlanarGap ) ;
	pNewSafe = SmartNC_CreateTool( m_cToolDef, dCheckOffset - 0.01, m_cStockDef.m_dPlanarGap ) ;

	if( pNewTool )
	{
		DriveMdl->m_pTool = pNewTool ;
		DriveMdl->m_dZShift -= 0.01 ;
	}
	if( DriveMdl->m_pCheckMdl && pNewSafe )
	{
		DriveMdl->m_pCheckMdl->m_pTool = pNewSafe ;
		DriveMdl->m_dZShift -= 0.01 ;
	}

	if( axisCtrl.m_cToolAxis.m_nAxisType != NCDEF_AXIS_VERT && 
	    ( axisCtrl.m_cToolAxis.m_nAxisFlag & NCDEF_AXIS_FLAG_LIMITANG ) )
	{
        SmartNC_SetACoordLimit( axisCtrl.m_cToolAxis.m_nRotAxis,
                                axisCtrl.m_cToolAxis.m_dElevationAng[0],
								axisCtrl.m_cToolAxis.m_dElevationAng[1] ) ;
	}
    JDNC_CONNECT3D tmpCnt3D = m_cFeedDef.m_cConnectDef ;
    if( bZigZag ) 
    {
        tmpCnt3D.m_bConnect3DFlag |= NCDEF_FCONNECT3D_ZIGZAG ;
    }
    else
    {
        tmpCnt3D.m_bConnect3DFlag &= ~NCDEF_FCONNECT3D_ZIGZAG ;
    }
    tmpCnt3D.m_dSafeDist = m_cFeedDef.m_cStepDef.m_dOverStep ;
	if( axisCtrl.m_cToolAxis.m_nAxisType == NCDEF_AXIS_VERT )
	{
		tmpCnt3D.m_bConnect3DFlag |= NCDEF_FCONNECT3D_VERTAXIS ;
	}
	// 根据行号连接多段刀具路径
	AllPath.ConnectPathByLineNo3D( *DriveMdl,m_cSetupDef.m_cTolDef, tmpCnt3D, m_cPrgDef) ; 

    SmartNC_SetACoordLimit( 2,-360., 360. ) ;
	if( pNewTool )
	{
		DriveMdl->m_pTool = pTool ;
		DriveMdl->m_dZShift += 0.01 ;
	}
	if( DriveMdl->m_pCheckMdl && pNewSafe )
	{
		DriveMdl->m_pCheckMdl->m_pTool = pSafe ;
		DriveMdl->m_pCheckMdl->m_dZShift += 0.01 ;
	}
	if( pNewTool ) delete pNewTool ;
	if( pNewSafe ) delete pNewSafe ;

	if( axisCtrl.m_cToolAxis.m_nAxisType == NCDEF_AXIS_VERT ) 
	{
		AllPath.SetCutMode ( MINI_MILL_PATH ) ;
//		AllPath.DelPointOnLine ( 0.0002 ) ;
	}
}
BOOL CFlowlineGen::Calc5AxCutStep( CSmtCheckMdl *DriveMdl, CNc5DAxisCtrl* pAxisCtrl,  
								   CGeoTrmSurf* pSurf, CSmartLoop *AllLoop, RFRAME &lf, double dStep, double uwStep[2], 
								   int nCutDir, double*& dParam, int& nNum, JDNC_PRGDEF &ProgDef )
{
	if( !pSurf || !DriveMdl || !pAxisCtrl ) return FALSE ;

	int	i = 0 ;
	CSmtCutPath *pPath[3] = { NULL, NULL, NULL } ;
	double uwLine[2][2] ;
	BOOL bBreak = FALSE ;
	if( nCutDir == 1 )
	{	
		for( i = 0 ; i < 3 ; i++ )
		{	
			if( i == 0 )
			{
				uwLine[0][0] = float( pSurf->m_pLoop->m_dBox2d[0][0] ) ;
				uwLine[0][1] = uwLine[1][1] = float( pSurf->m_pLoop->m_dBox2d[0][1] ) ;
				uwLine[1][0] = float( pSurf->m_pLoop->m_dBox2d[1][0] ) ;
			}
			else if( i == 1 )
			{
				uwLine[0][0] = float( pSurf->m_pLoop->m_dBox2d[0][0] ) ;
				uwLine[0][1] = uwLine[1][1] = float( ( pSurf->m_pLoop->m_dBox2d[0][1] + pSurf->m_pLoop->m_dBox2d[1][1]) * 0.5 ) ;
				uwLine[1][0] = float( pSurf->m_pLoop->m_dBox2d[1][0] ) ;
			}
			else
			{
				uwLine[0][0] = float( pSurf->m_pLoop->m_dBox2d[0][0] ) ;
				uwLine[0][1] = uwLine[1][1] = float( pSurf->m_pLoop->m_dBox2d[1][1] ) ;
				uwLine[1][0] = float( pSurf->m_pLoop->m_dBox2d[1][0] ) ;
			}
			
			pPath[i] = CreateSGuideTPath ( *DriveMdl, pAxisCtrl, pSurf->m_pSurface, AllLoop, lf, uwLine, 
											uwStep, nCutDir, m_cSetupDef.m_cTolDef, TRUE, ProgDef ) ;
			if( !pPath[i] ) 
			{
				bBreak = TRUE ;
				break ;
			}
		}
	}
	else
	{
		for( i = 0 ; i < 3 ; i++ )
		{	
			if( i == 0 )
			{
				uwLine[0][1] = float( pSurf->m_pLoop->m_dBox2d[0][1] ) ;
				uwLine[0][0] = uwLine[1][0] = float( pSurf->m_pLoop->m_dBox2d[0][0] ) ;
				uwLine[1][1] = float( pSurf->m_pLoop->m_dBox2d[1][1] ) ;
			}
			else if( i == 1 )
			{
				uwLine[0][1] = float( pSurf->m_pLoop->m_dBox2d[0][1] ) ;
				uwLine[0][0] = uwLine[1][0] = float( ( pSurf->m_pLoop->m_dBox2d[0][0] + pSurf->m_pLoop->m_dBox2d[1][0] ) * 0.5 ) ;
				uwLine[1][1] = float( pSurf->m_pLoop->m_dBox2d[1][1] ) ;
			}
			else
			{
				uwLine[0][1] = float( pSurf->m_pLoop->m_dBox2d[0][1] ) ;
				uwLine[0][0] = uwLine[1][0] = float( pSurf->m_pLoop->m_dBox2d[1][0] ) ;
				uwLine[1][1] = float( pSurf->m_pLoop->m_dBox2d[1][1] ) ;
			}

			pPath[i] = CreateSGuideTPath ( *DriveMdl, pAxisCtrl, pSurf->m_pSurface, AllLoop, lf, uwLine, 
											uwStep,	nCutDir, m_cSetupDef.m_cTolDef, TRUE, ProgDef ) ;
			if( !pPath[i] ) 
			{
				bBreak = TRUE ;
				break ;
			}
		}
	}
	
	if( bBreak )
	{
		for( i = 0 ; i < 3 ; i++ )
		{
			if( pPath[i] ) delete pPath[i] ;
		}
		return FALSE ;
	}

	// 得到最长的那根路径
	double dDist[3] = { 0., 0., 0. } ;
	double dMax = -1. ;
	CSmtCutPath *pFind = NULL ;		
	for( i = 0 ; i < 3 ; i++ )
	{
		if( !pPath[i] ) continue ;
		pPath[i]->DefineBox () ;
		dDist[i] = pPath[i]->GetLength () ;
		if( dDist[i] > dMax )
		{
			dMax = dDist[i] ;
			pFind = pPath[i] ;
		}
	}
	// 删除非选择
	for( i = 0 ; i < 3 ; i++ )
	{
		if( pPath[i] != pFind )
		{
			delete pPath[i] ;
			pPath[i] = NULL ;
		}
	}
	if( dMax < 0.01 * dStep )
	{
		nNum = 0 ;
		return FALSE ;
	}
	// 均分pFind,参数域采用线性插值
	RenewPathByStepAndSurf( pFind, dStep, pSurf, lf, FALSE, TRUE ) ;
	
	nNum = pFind->m_nNumPnt ;
	dParam = new double[nNum] ;

	i = 0 ;
	CSmtCutPointEx *pHead = ( CSmtCutPointEx *)pFind->m_pHead, *pNext = NULL ;
	while( pHead )
	{
		pNext = ( CSmtCutPointEx *)pHead->next ;
		if( nCutDir == 1 )
		{
			dParam[i] = pHead->m_fSurfPos[0] ;
		}
		else
		{
			dParam[i] = pHead->m_fSurfPos[1] ;
		}
		i++ ;
		pHead = pNext ;
	}
	if( nNum > 1 )
	{
		dParam[0] = 0., dParam[nNum-1] = 1. ;
	}
	nNum -= 1 ;
	delete pFind ;
	
	return TRUE ;
}

void CFlowlineGen::CalcSurfUWCut ( CGeoTrmSurf* pSurf, CSmartLoop* pLoop, JDNC_UWLINE &cParam, double uwStep[2], 
								   int &nUW, BOOL &bZigZag, BOOL &bSpiral )
{
	CSmtCutPath uwCurve[2] ;
	CSmtCutPoint *pPnt = NULL ;
	double uwBox[2][2], uw[2] ;
	PNT3D p ;
	int uwCnt[2] = { 0, 0 }, i = 0, k = 0 ;
	
	if( pSurf->m_pLoop )
	{
		memcpy( uwBox, pSurf->m_pLoop->m_dBox2d, sizeof( double[2][2] ) ) ;
	}
	else
	{
		uwBox[0][0] = uwBox[0][1] = 0.0 ;
		uwBox[1][0] = uwBox[1][1] = 1.0 ;
	}
    
	PNT2D		dIntPnt[200];
	CSmartSect*	pSectList[200];
	PNT2D		MidLine[2];						// 中线
	BOOL		UWInt0[2] = {TRUE, TRUE};		// U向中线和W向中线与曲面交点个数是否大于0


	int nLineDir = -1;							// 中线的方向
	for( i = 0 ; i < 2 ;  i++ )
	{
		if( i == 0 )
		{
			nLineDir = SURF_UDIR;
			MidLine[0][0] = uwBox[0][0] - 0.01; MidLine[1][0] = uwBox[1][0] + 0.01; 
			MidLine[0][1] = MidLine[1][1] = 0.5 * ( uwBox[0][1] + uwBox[1][1] ) ;
		}
		else
		{
			nLineDir = SURF_WDIR;
			MidLine[0][0] = MidLine[1][0] = 0.5 * ( uwBox[1][0] + uwBox[0][0] ) ;
			MidLine[0][1] = uwBox[0][1] - 0.01; MidLine[1][1] = uwBox[1][1] + 0.01; 
		}

		// 得到中线与曲面交点的个数
		UINT nSize = 200;
		nSize = pLoop->GetLineInt(MidLine[0], MidLine[1], dIntPnt, pSectList, nSize);
		if(nLineDir == SURF_WDIR)
		{
			for( UINT j=0; j<nSize; j++ ) 
			{
				double	tmp=dIntPnt[j][0];
				dIntPnt[j][0]=dIntPnt[j][1];
				dIntPnt[j][1]=tmp ;
			}
		}
		nSize = pLoop->SortPointSect( dIntPnt, pSectList, nSize ) ;
		if( nSize > 2 )
		{// 如果交点个数大于2，说明曲面肯定不闭合
            UWInt0[i] = FALSE;
			continue;
		}
		if(nLineDir == SURF_WDIR)
		{
			for( UINT j=0; j<nSize; j++ ) 
			{
				double	tmp=dIntPnt[j][0];
				dIntPnt[j][0]=dIntPnt[j][1];
				dIntPnt[j][1]=tmp ;
			}
		}
		memcpy( uwBox, dIntPnt, sizeof( double[2][2] ) ) ;

		// 把参数线投影到曲面
		if( uwStep[i] > 0.05 ) uwStep[i] = 0.05 ;
		else if( uwStep[i] < 0.002 ) uwStep[i] = 0.002 ;
		uwCnt[i] = ( int ) ceil( ( uwBox[1][i] - uwBox[0][i] ) / uwStep[i] ) ;
		uwStep[i] = ( uwBox[1][i] - uwBox[0][i] ) / ( uwCnt[i] - 1 ) ;
		
		for( k = 0 ; k < uwCnt[i] ; k++ )
		{
			if( i == 0 )
			{
				uw[0] = uwBox[0][0] + k * uwStep[0] ;
				uw[1] = 0.5 * ( uwBox[0][1] + uwBox[1][1] ) ;
			}
			else
			{
				uw[0] = 0.5 * ( uwBox[1][0] + uwBox[0][0] ) ;
				uw[1] = uwBox[0][1] + k * uwStep[1] ;
			}
			pSurf->GetPoint ( uw[0], uw[1], p ) ;
			pPnt = new CSmtCutPoint( p ) ;
			pPnt->m_fPoint[3] = TFLOAT( k * uwStep[i] ) ;
			uwCurve[i].AddTail ( pPnt ) ;
		}
		uwCurve[i].NormalizeLen () ;
	}
    
    // STEP 2 : 计算平铺参数域的上的路径分布
    nUW = cParam.m_nCutDir ;
	// 判断曲线是否闭合,如果闭合并且方向相同,则沿曲面加工
	if( m_cFeedDef.m_cConnectDef.m_bConnect3DFlag & NCDEF_FCONNECT3D_SPIRAL )
	{
		if( UWInt0[1] == TRUE && uwCurve[1].IsClosed () )
		{
			nUW = 1, bZigZag = FALSE, bSpiral = TRUE ;
		}
		else if( UWInt0[0] == TRUE && uwCurve[0].IsClosed () )
		{
			nUW = 0, bZigZag = FALSE, bSpiral = TRUE ;
		}
	}
	uwCurve[0].ClearAllPoint () ;
	uwCurve[1].ClearAllPoint () ;
}

// 添加两个路径组之间的连刀路径
BOOL CFlowlineGen::ConnectPathLib(CSmtCPathLib &PathLib1, CSmtCPathLib &PathLib2, 
								  CSmtCheckMdl &DriveMdl, CNc5DAxisCtrl &AxisCtrl)
{
	if (PathLib1.m_cAllPath.GetCount() == 0 || PathLib2.m_cAllPath.GetCount() == 0)
		return FALSE;
	CSmtCutPath *pPath = NULL;
	CSmtCutPointEx* pStart = (CSmtCutPointEx*)PathLib1.m_cAllPath.GetTail()->m_pTail;
	CSmtCutPointEx* pEnd = (CSmtCutPointEx*)PathLib2.m_cAllPath.GetHead()->m_pHead;
	pPath = new CSmtCutPath( MINI_MILL5AX_PATH ) ;
	pPath->m_bFeedType = JDNC_FEEDTYPE_QUICK ;
	CSmtCutPointEx* pTAt = (CSmtCutPointEx*)pStart->CopyMyself() ;
	DriveMdl.Define5AXHeight( *pTAt , TRUE ) ;
	pPath->AddTail( pStart->CopyMyself() ) ;
	pPath->AddTail( pTAt ) ;
	pTAt = (CSmtCutPointEx*)pEnd->CopyMyself() ;
	DriveMdl.Define5AXHeight( *pTAt , TRUE ) ;
	pPath->AddTail( pTAt ) ;
	pPath->AddTail( pEnd->CopyMyself() ) ;
	Math5Ax_ProjectConncetPath5D( DriveMdl, *pPath, m_cSetupDef.m_cTolDef, m_cSpeedDef.m_dRapidHeight, AxisCtrl ) ;
	PathLib1.AddToTail( pPath ) ;
	return TRUE;
}

BOOL CFlowlineGen::CreateOne5AxToolPosPath( CSmtCheckMdl *DriveMdl, CNc5DAxisCtrl &axisCtrl, RFRAME& cRFrame, 
											CGeoTrmSurf* pSurf, JDNC_UWLINE& cParam, CSmartLoop *AllLoop, 
											CSmtCPathLib& AllPath, JDNC_PRGDEF &PrgDef )
{
	BOOL bZigZag = FALSE ;
	if( cParam.m_bUWLineFlag & NCDEF_UWLINE_ZIGZAG ) bZigZag = TRUE ;
	
	// 计算初始路径
	CalcOriginalPath( pSurf, cParam, cRFrame, AllPath ) ;

	if( AllPath.m_cAllPath.GetCount () < 1 )
	{
		return FALSE ;
	}

	// 根据初始i路径，计算平面与曲面的交线路径
	BOOL bRet = TRUE ;

	bRet = CalcPlaneSurfIntPath( DriveMdl, axisCtrl,  pSurf, cRFrame, 
								cParam, AllLoop, AllPath, PrgDef )	;
	
	// 对路径进行排序，类似于平行截线那样，首点距离最近
	SortAllPath( AllPath ) ;
	// 对路径进行连接
	if( bRet )
		LinkAll5AxisPath( DriveMdl, AllPath, axisCtrl, bZigZag ) ;

	return bRet ;
}


void MathCAM_RotateByZAxis( CSmtCPathLib& AllPath, DOUBLE Angle ) ;

void CFlowlineGen::CalcOriginalPath( CGeoTrmSurf* pSurf, JDNC_UWLINE& cParam, RFRAME& cRFrame, CSmtCPathLib &AllPath )
{
	BOX3D box ;
	pSurf->CalcLFBox ( &cRFrame, &box ) ;
	mathExpandBox3D( m_pMiller->m_fRadius * 6, &box ) ;

	CSmartLoop *AllLoop = new CSmartLoop() ;
	AllLoop->CreateLoop ( box.min, box.max ) ;

	DOUBLE dRotAng = ANGLE_TO_RADIAN( cParam.m_dAngle ) ; 
	if(fabs( dRotAng ) > 1.0e-4 )
	{ 
		AllLoop->RotateContour ( -dRotAng ) ;
	}

	int nGroupNo = 0 , nLineNo = 0 ;
	PNT2D dBox2D[2] ;
	DOUBLE y = 0.0 ;
	DOUBLE yStep = m_cFeedDef.m_cStepDef.m_dOverStep  ;
	PNT4D dStart, dEnd  ;
	dStart[2] = dEnd[2] = 0.0 ;
	dStart[3] = dEnd[3] = 0.0 ;
	PNT2D dPtArr[1000];
	
	nc_VectorCopy ( dBox2D[0], AllLoop->m_dBox[0], 2 ) ;
	nc_VectorCopy ( dBox2D[1], AllLoop->m_dBox[1], 2 ) ;
	
	for( y = dBox2D[0][1], nLineNo = 0 ; y < dBox2D[1][1]; y += yStep, nLineNo ++ ) 
	{
		if( y < AllLoop->m_dBox[0][1] ) continue ;
		if( y > AllLoop->m_dBox[1][1] ) break    ;
		//////////////////////////////
		UINT nCount = AllLoop->GetYLineIntContour( y , dPtArr, 1000 ) ;
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
			AllPath.AddToTail( pNewPath )  ;
		}
	}
	
	if( fabs( dRotAng ) > 1.0e-4 )
	{ 
		MathCAM_RotateByZAxis( AllPath, dRotAng ) ;
	}
	Mini_DeleteContours( AllLoop ) ;
}

void MathCam_DeleteSSICurve( CSSICurve * pIntCur) 
{
	if( !pIntCur ) return ;
	while( pIntCur ) 
	{
		CSSICurve *pHeadCrv = pIntCur->m_next ;
		delete pIntCur ;
		pIntCur = pHeadCrv ;
	}
}

CSSICurve * CFlowlineGen::CalcPlaneSurfInt( CGeoTrmSurf* pSurf, PNT3D pivot, VEC3D normal ) 
{
	if( !pSurf ) return NULL ;
	CGeoInter cGeoInter ;
	CGeoConvert convert;
	JDNC_TOL Tol = m_cSetupDef.m_cTolDef ;
	cGeoInter.m_dTriLeng = Tol.m_dMaxStep ;
	if( m_cSetupDef.m_cModelTol.m_nMdlFlag & NCDEF_SETUP_NOMOSAIC )
		cGeoInter.m_dTriLeng = min( cGeoInter.m_dTriLeng, 0.15 ) ;
	cGeoInter.m_dTriChord = 0.002 ;
	
	// 平面与直线求交
	CSSICurve * pIntCur = NULL, *pHeadCrv = NULL ;
	cGeoInter.PlaneSurfaceInt( pivot, normal, pSurf->m_pSurface, pIntCur ) ;
	if( !pIntCur ) return NULL ;

	// 处理交线huan
	if( pSurf->IsTrimed() )
	{		
		tsLoop*  trmLp = convert.ConvertTrmLoop( pSurf->m_pLoop, 1.0e-4) ;
		CSSICurve *subHead = NULL, *subTail = NULL;
		for( pHeadCrv = pIntCur ; pHeadCrv ; pHeadCrv = pHeadCrv->m_next )
		{
			pHeadCrv->IntWithContour( trmLp, 0 ) ;
			CSSICurve* subCur = pHeadCrv->GetSubIntcur( 0 ) ;
			if( subCur == NULL ) continue ;
			if( subTail == NULL ) 
			{
				subHead = subTail = subCur ;
			}
			else
			{
				subTail->m_next = subCur ;
				subCur->m_prev = subTail ;
			}
			while( subTail && subTail->m_next )
			{
				subTail = subTail->m_next ;
			}
		}
		delete trmLp ;
		MathCam_DeleteSSICurve( pIntCur ) ;
		pIntCur = subHead ;
	}
	return pIntCur ;
}

CSSICurve * CFlowlineGen::CalcSurfSurfInt( CGeoTrmSurf* pSurf, CSurface *pSurface ) 
{
	if( !pSurf ) return NULL ;
	CGeoInter cGeoInter ;
	CGeoConvert convert;
	
	// 平面与直线求交
	CSSICurve * pIntCur = NULL, *pHeadCrv = NULL ;

	CFacet *pFacet = NULL ;
	TwoSurfaceIntByDisc( pSurf->m_pSurface, pSurf->m_pFacet, pSurface, pFacet, pIntCur ) ;
	if( pFacet )
	{
		pFacet->Destory() ;
		delete pFacet ;
	}
//	cGeoInter.TwoSurfaceInt( pSurf->m_pSurface, pSurface, pIntCur ) ;
	if( !pIntCur ) return NULL ;

	// 处理交线huan
	if( pSurf->IsTrimed() )
	{		
		tsLoop*  trmLp = convert.ConvertTrmLoop( pSurf->m_pLoop, 1.0e-4) ;
		CSSICurve *subHead = NULL, *subTail = NULL;
		for( pHeadCrv = pIntCur ; pHeadCrv ; pHeadCrv = pHeadCrv->m_next )
		{
			pHeadCrv->IntWithContour( trmLp, 0 ) ;
			CSSICurve* subCur = pHeadCrv->GetSubIntcur( 0 ) ;
			if( subCur == NULL ) continue ;
			if( subTail == NULL ) 
			{
				subHead = subTail = subCur ;
			}
			else
			{
				subTail->m_next = subCur ;
				subCur->m_prev = subTail ;
			}
			while( subTail && subTail->m_next )
			{
				subTail = subTail->m_next ;
			}
		}
		delete trmLp ;
		MathCam_DeleteSSICurve( pIntCur ) ;
		pIntCur = subHead ;
	}
	return pIntCur ;
}
BOOL CFlowlineGen::CalcPlaneSurfIntPath( CSmtCheckMdl *DriveMdl, CNc5DAxisCtrl &axisCtrl, CGeoTrmSurf* pSurf,  
										RFRAME& cRFrame, JDNC_UWLINE& cParam, CSmartLoop *AllLoop, 
										CSmtCPathLib &AllPath , JDNC_PRGDEF &PrgDef )
{
	if( !DriveMdl || !pSurf || !pSurf->m_pSurface || AllPath.m_cAllPath.GetCount () < 1 )
	{
		return FALSE ;
	}

	int nNumLine = AllPath.GetNumPath () ;

	PrgDef.m_dIncStep = PrgDef.m_dTotalMove / nNumLine ;
	PrgDef.m_dLimitAt = 1.0 ;
	int i = 0, nCnt = 0  ;

	BOX3D box ;
	pSurf->CalcLFBox ( &cRFrame, &box ) ;
	// 计算曲面pSurf的极值点
	PNT3D pt[4], uv[4] ;
	// 生成参数域的环
	CSmartPathGen cGen ;
	CSmartLoop* pLoop = cGen.ExtractSurfLoop( pSurf, m_cSetupDef.m_cTolDef, &cRFrame ) ;
	if( pLoop )
	{
		pLoop->DefineBox() ;
		uv[0][0] = pLoop->m_dBox[0][0], uv[0][1] = pLoop->m_dBox[0][1] ;
		uv[1][0] = pLoop->m_dBox[0][0], uv[1][1] = pLoop->m_dBox[1][1] ;
		uv[2][0] = pLoop->m_dBox[1][0], uv[2][1] = pLoop->m_dBox[0][1] ;
		uv[3][0] = pLoop->m_dBox[1][0], uv[3][1] = pLoop->m_dBox[1][1] ;
		for( i = 0 ; i < 4 ; i++ )
		{
			pSurf->m_pSurface->GetPoint( &cRFrame, uv[i][0], uv[i][1], pt[i] ) ;
		}
	}

	PNT3D pivot, vec, normal, start, end, vecZ = { 0., 0., 1. } ;
	CSmtCutPath *pPath = NULL ;
	CSmtCPathLib PathLib ;
	BOOL bRet = TRUE ;
	POSITION pos = AllPath.m_cAllPath.GetHeadPosition (), atpos = NULL ;
	while( pos )
	{
		// 生成路径
		if( PrgDef.m_pBrkFunc && PrgDef.m_pBrkFunc () )
		{
			bRet = FALSE ;
			PathLib.ClearAllPath () ;
			if (m_pErrorType) *m_pErrorType = JDERROR_GENPATH_ABORT;
			break;
		}
		MathCAM_MovePrgStep(PrgDef);

		atpos = pos ;
		pPath = AllPath.m_cAllPath.GetNext ( pos ) ;
		for( i = 0 ; i < 3 ; i++ )
		{
			start[i] = pPath->m_pHead->m_fPoint[i] ;
			end[i]	 = pPath->m_pTail->m_fPoint[i] ;
		}
		for( i = 0 ; i < 3 ; i++ )
		{
			pivot[i] = ( start[i] + end[i] ) * 0.5 ;
		}
		mathGetVecUnit( start, end, vec ) ;
		mathRotVec( vecZ, pivot, PI1_2, vec, normal ) ;

		CSSICurve * pIntCur = NULL ;
		CRulSur *pRulSurf = CalcRulSurfByLine( start, end, box ) ;
		if( pSurf )
		{
			pIntCur = CalcSurfSurfInt( pSurf, pRulSurf ) ;
		}
		else
		{
			pIntCur = CalcPlaneSurfInt( pSurf, pivot, normal ) ;
		}
		
		if( pIntCur )
		{
#ifdef _DEBUG
			if( nCnt == 145 )
			{
				nCnt = 145 ;
			}
#endif
			// 调整多条交线的重合位置,处理交点在极值点附近的情况
			if( pIntCur && pIntCur->m_next && pLoop )
			{
				AdjustIntCurve( pIntCur, pt, uv ) ;
			}
			// 根据交线生成路径
			bRet = Calc5axPathFromIntCurve( DriveMdl, axisCtrl, pSurf, cRFrame, 
										 pIntCur, cParam, AllLoop, PathLib, PrgDef ) ;
			// 删除交线
			MathCam_DeleteSSICurve( pIntCur ) ;
			if( !bRet ) 
			{
				PathLib.ClearAllPath () ;
				if( pRulSurf )  delete pRulSurf ;
				break ;
			}
			nCnt++ ;
		}
		if( pRulSurf )
		{
			delete pRulSurf ;
		}
		// 删除pPath
		AllPath.m_cAllPath.RemoveAt ( atpos ) ;
		delete pPath ;		
	}
	if( pLoop )
	{
		Mini_DeleteContours( pLoop ) ;
	}
	AllPath.ClearAllPath () ;
	// 将PathLib中路径赋予号
	int nLineNo = 0 ;
	pos = PathLib.m_cAllPath.GetHeadPosition () ;
	while( pos )
	{
		atpos = pos ;
		pPath = PathLib.m_cAllPath.GetNext ( pos ) ;
		PathLib.m_cAllPath.RemoveAt ( atpos ) ;
		pPath->m_nLineNo = nLineNo ;
		AllPath.AddToTail ( pPath ) ;
		nLineNo++ ;
	}
	PathLib.m_cAllPath.RemoveAll () ;
	return bRet ;
}

CRulSur *CFlowlineGen::CalcRulSurfByLine ( PNT3D start, PNT3D end, BOX3D &box )
{
	PNT3D st[2], ed[2] ;
	mathCpyPnt( start, st[0] ) ;
	mathCpyPnt( start, st[1] ) ;
	mathCpyPnt( end	 , ed[0] ) ;
	mathCpyPnt( end  , ed[1] ) ;

	st[0][2] = ed[0][2] = box.min[2] - 10. ;
	st[1][2] = ed[1][2] = box.max[2] + 10. ;

	CGeoLine *pLine1 = new CGeoLine( st[0], ed[0] ) ;
	CGeoLine *pLine2 = new CGeoLine( st[1], ed[1] ) ;

	CRulSur *pSurf = new CRulSur( pLine1, pLine2 ) ;
	if( pSurf->IsValid() )
		return pSurf ;
	return NULL ;
}

BOOL CFlowlineGen::Calc5axPathFromIntCurve( CSmtCheckMdl *DriveMdl, CNc5DAxisCtrl &axisCtrl,  CGeoTrmSurf* pSurf, 
											RFRAME& cRFrame,  CSSICurve *pIntCurve, JDNC_UWLINE& cParam,
											CSmartLoop *AllLoop, CSmtCPathLib &AllPath, JDNC_PRGDEF &PrgDef )
{
	if( !pIntCurve || !DriveMdl || !pSurf || !pSurf->m_pSurface ) return FALSE ;
	CSSICurve *pHeadCrv = pIntCurve ;
	DOUBLE dAngStep = ANGLE_TO_RADIAN( 3.0 ) ;
	dAngStep = ANGLE_TO_RADIAN( axisCtrl.m_cToolAxis.m_dMaxAngleStep ) ;

	JDNC_TOL Tol = m_cSetupDef.m_cTolDef ;
	double dDist = m_cSetupDef.m_cTolDef.m_dMaxStep ;
	if( m_cSetupDef.m_cModelTol.m_nMdlFlag & NCDEF_SETUP_NOMOSAIC )
		dDist = min( dDist, 0.15 ) ;
	
	double dAngRot[2], dAng = ANGLE_TO_RADIAN( cParam.m_dAngle ) ;
	dAngRot[0]  = cos( dAng ) , dAngRot[1] = sin( dAng ) ;
	CSmtCPathLib tmpLib ;
	double uw[2], pos[3], near_p[3] ;
//	double pos[3], du[3], dw[3], dnor[3] ;
	double uw0, uw1, dist = 100. ;
	int i= 0 ;
	BOOL bRet = TRUE ;
	for( ; pHeadCrv ; pHeadCrv = pHeadCrv->m_next )
	{
		CSmtCutPath *pNewPath = new CSmtCutPath( MINI_MILL5AX_PATH ) ;
		tagSSIntpt *pIntAt = pHeadCrv->m_headssipt ;
		CSmtCutPointEx tmpTPnt, *pTPnt = NULL, *pTNext = NULL ;
		// 首先将初始点转为路径
		for( ; pIntAt ; pIntAt = pIntAt->next )
		{
			pTPnt = new CSmtCutPointEx() ;
			uw[0] = pIntAt->uv[0], uw[1] = pIntAt->uv[1] ;
			pSurf->m_pSurface->GetPoint( &cRFrame, uw[0], uw[1], pos ) ;
	//		pSurf->m_pSurface->GetDerivative10( &cRFrame, uw[0], uw[1], pos, du, dw, dnor ) ;
	//		nc_DoubleToFloat( pTPnt->m_fPoint, pos , 3 ) ;
	//		nc_DoubleToFloat( pTPnt->m_fSurfNor, dnor, 3 ) ;
			Gen5AxisPoint( pSurf->m_pSurface, &axisCtrl, uw, &cRFrame, dAngRot, *pTPnt, TRUE, TRUE ) ;
			nc_DoubleToFloat( pTPnt->m_fProjDir, pos, 3 ) ;
			pNewPath->AddTail( pTPnt ) ;
		}
	//	AllPath.AddToTail ( pNewPath ) ;
		// STEP 2 : 曲线跌代,确保在导动曲面上的角度误差和弦高误差
		double dMinCos = cos( dAngStep), dCosA = 0 ;
		for( pTPnt = (CSmtCutPointEx*)pNewPath->m_pHead ; pTPnt && pTPnt->next;  )
		{
			if( PrgDef.m_pBrkFunc && PrgDef.m_pBrkFunc() )
			{
				if (m_pErrorType) *m_pErrorType = JDERROR_GENPATH_ABORT;
				bRet = FALSE ;
				break ;
			}
			pTNext = ( CSmtCutPointEx*) pTPnt->next ;
			
			uw[0] = ( pTPnt->m_fSurfPos[0] + pTNext->m_fSurfPos[0] ) * 0.5 ;
			uw[1] = ( pTPnt->m_fSurfPos[1] + pTNext->m_fSurfPos[1] ) * 0.5 ;
			for( i = 0 ; i < 3 ; i++ )
			{
				pos[i] = ( pTPnt->m_fProjDir[i] + pTNext->m_fProjDir[i] ) * 0.5 ;
			}
			uw0 = uw[0], uw1 = uw[1], dist = 100. ; 
			if( pSurf->GetNearestPt( pos, uw0, uw1, near_p, dist ) && dist < 0.001 )
			{
				uw[0] = uw0, uw[1] = uw1 ;
			}
			else
			{
				pSurf->m_pSurface->GetPoint( &cRFrame, uw[0], uw[1], near_p ) ;
			}
			Gen5AxisPoint( pSurf->m_pSurface, &axisCtrl, uw, &cRFrame,dAngRot, tmpTPnt, TRUE, TRUE ) ;

			dCosA = nc_OProduct( pTPnt->m_fSurfNor, pTNext->m_fSurfNor, 3 ) ;
			if( dCosA < dMinCos ||
				nc_Distance( pTPnt->m_fPoint, pTNext->m_fPoint,3 ) > dDist ||
				nc_PointLineDist( pTPnt->m_fPoint, pTNext->m_fPoint, tmpTPnt.m_fPoint ) > Tol.m_dArcTol )
			{
				CSmtCutPointEx* pTNew = new CSmtCutPointEx( tmpTPnt ) ;
				nc_DoubleToFloat( pTNew->m_fProjDir, near_p, 3 ) ;
				pNewPath->InsertAfter( pTNew, pTPnt ) ;
			}
			else 
			{
				pTPnt = pTNext ;
				continue ;
			}
		}
		if( !bRet ) goto EndLine ;
		// STEP 3 : 加工模型法向投影路径点，确保在加工模型上的误差
		for( pTPnt = (CSmtCutPointEx*)pNewPath->m_pHead ; pTPnt ; pTPnt = (CSmtCutPointEx*)pTPnt->next )
		{
			if( PrgDef.m_pBrkFunc && PrgDef.m_pBrkFunc() )
			{
				if (m_pErrorType) *m_pErrorType = JDERROR_GENPATH_ABORT;
				bRet = FALSE ;
				break ;
			}
			Def5AxisPoint( *DriveMdl, *pTPnt, Tol, AllLoop ) ;
		}
		if( !bRet ) goto EndLine ;
		// 插点
		for( pTPnt = (CSmtCutPointEx*)pNewPath->m_pHead ; pTPnt && pTPnt->next;  )
		{
			if( PrgDef.m_pBrkFunc && PrgDef.m_pBrkFunc() )
			{
				if (m_pErrorType) *m_pErrorType = JDERROR_GENPATH_ABORT;
				bRet = FALSE ;
				break ;
			}
			pTNext = ( CSmtCutPointEx*) pTPnt->next ;
			if( pTPnt->m_bType <= 0 && pTNext->m_bType <= 0 )
			{//无效点
				pTPnt = pTNext ; 
				continue ;
			}
			if( nc_Distance(pTPnt->m_fPoint,pTNext->m_fPoint, 3 ) < 0.02   )
			{//接近点，无须插点
				pTPnt = pTNext ; 
				continue ;
			}
			uw[0] = ( pTPnt->m_fSurfPos[0] + pTNext->m_fSurfPos[0] ) * 0.5 ;
			uw[1] = ( pTPnt->m_fSurfPos[1] + pTNext->m_fSurfPos[1] ) * 0.5 ;

			for( i = 0 ; i < 3 ; i++ )
			{
				pos[i] = ( pTPnt->m_fProjDir[i] + pTNext->m_fProjDir[i] ) * 0.5 ;
			}
			uw0 = uw[0], uw1 = uw[1], dist = 100. ; 
			if( pSurf->GetNearestPt( pos, uw0, uw1, near_p, dist ) && dist < 0.001 )
			{
				uw[0] = uw0, uw[1] = uw1 ;
			}
			else
			{
				pSurf->m_pSurface->GetPoint( &cRFrame, uw[0], uw[1], near_p ) ;
			}

			Gen5AxisPoint( pSurf->m_pSurface, &axisCtrl, uw, &cRFrame,dAngRot, tmpTPnt, TRUE, TRUE ) ;

			Def5AxisPoint( *DriveMdl, tmpTPnt, Tol, AllLoop ) ;

			if( nc_Distance( pTPnt->m_fPoint, pTNext->m_fPoint,3 ) > Tol.m_dMaxStep ||
				nc_PointLineDist( pTPnt->m_fPoint, pTNext->m_fPoint, tmpTPnt.m_fPoint ) > Tol.m_dArcTol )
			{
				CSmtCutPointEx* pTNew = new CSmtCutPointEx( tmpTPnt ) ;
				nc_DoubleToFloat( pTNew->m_fProjDir, near_p, 3 ) ;
				pNewPath->InsertAfter( pTNew, pTPnt ) ;
			}
			else 
			{
				pTPnt = pTNext ;
				continue ;
			}
		}
		for( pTPnt = (CSmtCutPointEx*)pNewPath->m_pHead ; pTPnt && pTPnt->next;  pTPnt = pTNext)
		{//删除孤立的有效点
			pTNext = ( CSmtCutPointEx*) pTPnt->next ;
			if( pTPnt->m_bType == 0 ) continue ;
			if( (pTPnt->prev == NULL || pTPnt->prev->m_bType == 0 ) &&
				(pTPnt->next == NULL || pTPnt->next->m_bType == 0 )   )
			{
				pTPnt->m_bType = 0 ;
			}
		}

EndLine:
		if( !bRet )
		{
			delete pNewPath ;
			break ;
		}
		
		// 将路径在断点处打断
		CSmtCutPath* pNewHead = pNewPath->BreakAtNullPoint( FALSE, 0.02 ) ;
		delete pNewPath ;
		while( pNewHead )
		{
			pNewPath = pNewHead ;
			pNewHead = pNewHead->next ;
			pNewPath->prev = pNewPath->next = NULL ; 
			tmpLib.AddToTail( pNewPath );
		}/**/
	}
	if( bRet )
	{
		// 对tempLib中的路径进行连接
		ConnectAllPathLib( tmpLib ) ;
		// 将路径放到AllPath中
		AllPath.AppendCPathLib ( tmpLib ) ;
	}
	else
	{
		AllPath.ClearAllPath() ;
	}
	return bRet ;
}

void CFlowlineGen::GetPathPntAndNor( CSmtCutPath *pPath, PNT3D pnt[2], VEC3D vec[2] )
{
	int i = 0 ;
	for( i = 0 ; i < 3 ; i++ )
	{
		pnt[0][i] = pPath->m_pHead->m_fPoint[i] ;
		pnt[1][i] = pPath->m_pTail->m_fPoint[i] ;
		vec[0][i] = ( ( CSmtCutPointEx *)pPath->m_pHead)->m_fSurfNor[i] ;
		vec[1][i] = ( ( CSmtCutPointEx *)pPath->m_pTail)->m_fSurfNor[i] ;
	}
}

void CFlowlineGen::ConnectAllPathLib( CSmtCPathLib &AllPath )
{
	if( AllPath.m_cAllPath.GetCount () < 2 )
		return ;
	
	CSmtCPathLib tmpLib ;
	CSmtCutPath *pHead = AllPath.m_cAllPath.RemoveHead() ;
	PNT3D pnt[2], nextp[2] ;
	VEC3D vec[2], nextv[2] ;
	GetPathPntAndNor( pHead, pnt, vec ) ;
	double dNear[2] = { 100., 100. } ;
	
	while( 1 )
	{
		CSmtCutPath *pNext = NULL, *pFind = NULL ;
		double dDist[4], dAngle[4] ;
		int i = 0, nFind = -1 ;
		dNear[0] = 100., dNear[1] = 100. ;
		POSITION pos = AllPath.m_cAllPath.GetHeadPosition (), atpos = NULL, atFind = NULL ;
		while( pos )
		{
			atpos = pos ;
			pNext = AllPath.m_cAllPath.GetNext ( pos ) ;
			GetPathPntAndNor( pNext, nextp, nextv ) ;
						
			// 计算角度和距离
			dDist[0] = mathDist( pnt[0], nextp[0] ) ;
			dDist[1] = mathDist( pnt[0], nextp[1] ) ;
			dDist[2] = mathDist( pnt[1], nextp[0] ) ;
			dDist[3] = mathDist( pnt[1], nextp[1] ) ;

			dAngle[0] = mathGetAcuteAngleUnit( vec[0], nextv[0] ) ;
			dAngle[1] = mathGetAcuteAngleUnit( vec[0], nextv[1] ) ;
			dAngle[2] = mathGetAcuteAngleUnit( vec[1], nextv[0] ) ;
			dAngle[3] = mathGetAcuteAngleUnit( vec[1], nextv[1] ) ;

			for( i = 0 ; i < 4 ; i++ )
			{
				if( dDist[i] < dNear[0] && dAngle[i] < dNear[1] )
				{
					dNear[0] = dDist[i], dNear[1] = dAngle[i] ;
					nFind = i ;
					atFind = atpos ;
					pFind = pNext ;
				}
			}
		}
		if( nFind > -1 && dNear[0] < 0.002 && dNear[1] < 0.001 && pFind )
		{
			// 将pFind与pHead连接
			AllPath.m_cAllPath.RemoveAt ( atFind ) ;
			if( nFind == 0 || nFind == 2 )
			{
				pFind->DeletePoint ( pFind->m_pHead ) ;
				CSmtCutPoint *pCHead = pFind->m_pHead, *pCNext = NULL ;
				while( pCHead )
				{
					pCNext = pCHead->next ;
					pCHead->prev = pCHead->next = NULL ;
					pFind->RemovePoint ( pCHead ) ;
					if( nFind == 0 )
						pHead->AddHead( pCHead ) ;
					else 
						pHead->AddTail( pCHead ) ;
					pCHead = pCNext ;
				}
			}
			else
			{
				pFind->DeletePoint ( pFind->m_pTail ) ;
				CSmtCutPoint *pTail = pFind->m_pTail, *pPrev = NULL ;
				while( pTail )
				{
					pPrev = pTail->prev ;
					pTail->prev = pTail->next = NULL ;
					pFind->RemovePoint ( pTail ) ;
					if( nFind == 1 )
						pHead->AddHead ( pTail ) ;
					else
						pHead->AddTail ( pTail ) ;
					pTail = pPrev ;
				}
			}
			delete pFind ;
			GetPathPntAndNor( pHead, pnt, vec ) ;
		}
		else
		{
			tmpLib.AddToTail ( pHead ) ;
			if( AllPath.m_cAllPath.GetCount () > 0 )
			{
				pHead = AllPath.m_cAllPath.RemoveHead() ;
				GetPathPntAndNor( pHead, pnt, vec ) ;
			}
			else
			{
				break ;
			}
		}
	}
	AllPath.ClearAllPath() ;
	AllPath.AppendCPathLib( tmpLib ) ;

}

void CFlowlineGen::SortAllPath( CSmtCPathLib &AllPath )
{
	if( AllPath.m_cAllPath.GetCount () < 2 ) 
		return ;

	PNT3D pnt[2], nxt[2] ;
	VEC3D vec[2], nxtvec[2] ;
	double dDist[2] = { 0., 0. } ;
	CSmtCutPath *pPath = NULL, *pNext = NULL ;
	POSITION pos = AllPath.m_cAllPath.GetHeadPosition () ;
	pPath = AllPath.m_cAllPath.GetNext ( pos ) ;
	GetPathPntAndNor( pPath, pnt, vec ) ;

	while( pos )
	{
		pNext = AllPath.m_cAllPath.GetNext ( pos ) ;
		GetPathPntAndNor( pNext, nxt, nxtvec ) ;
		dDist[0] = mathDist( pnt[0], nxt[0] ) ;
		dDist[1] = mathDist( pnt[0], nxt[1] ) ;
		if( dDist[0] > dDist[1] )
		{
			pNext->ReverseDirect() ;
			mathCpyPnt( nxt[1], pnt[0] ) ;
			mathCpyPnt( nxt[0], pnt[1] ) ;
		}
		else
		{
			mathCpyPnt( nxt[0], pnt[0] ) ;
			mathCpyPnt( nxt[1], pnt[1] ) ;
		}
	}
}

void CFlowlineGen::AdjustIntCurve(  CSSICurve *pIntCurve, PNT3D pt[4], PNT3D uv[4] )
{
	UNUSED_ALWAYS(uv);
	if(  !pIntCurve ) return ;
	int i = 0 ;
	double dTol = 0.001 ;
	CSSICurve *pHead = pIntCurve, *pNext = NULL ;
	double dDist[4] = { 100., 100., 100., 100. }, dMinDist[4] = { 100., 100., 100., 100. } ;
	for( ; pHead ; pHead = pHead->m_next )
	{
		pNext = pHead->m_next ;
		if( !pNext ) break ;
		dDist[0] = mathDist( pHead->m_headssipt->p, pNext->m_headssipt->p ) ;
		dDist[1] = mathDist( pHead->m_headssipt->p, pNext->m_tailssipt->p ) ;
		dDist[2] = mathDist( pHead->m_tailssipt->p, pNext->m_headssipt->p ) ;
		dDist[3] = mathDist( pHead->m_tailssipt->p, pNext->m_tailssipt->p ) ;
		if( dDist[0] < dTol )
		{
			for( i = 0 ; i < 4 ; i++ )
			{
				dMinDist[i] = mathDist( pt[i], pHead->m_headssipt->p ) ;				
				if( dMinDist[i] < dTol )
				{
					pNext->m_headssipt->uv[0] = pHead->m_headssipt->uv[0] ;
					pNext->m_headssipt->uv[1] = pHead->m_headssipt->uv[1] ;
				}
			}
		}
		else if( dDist[1] < dTol )
		{
			for( i = 0 ; i < 4 ; i++ )
			{
				dMinDist[i] = mathDist( pt[i], pHead->m_headssipt->p ) ;				
				if( dMinDist[i] < dTol )
				{
					pNext->m_tailssipt->uv[0] = pHead->m_headssipt->uv[0] ;
					pNext->m_tailssipt->uv[1] = pHead->m_headssipt->uv[1] ;
				}
			}
		}
		else if( dDist[2] < dTol )
		{
			for( i = 0 ; i < 4 ; i++ )
			{
				dMinDist[i] = mathDist( pt[i], pHead->m_tailssipt->p ) ;				
				if( dMinDist[i] < dTol )
				{
					pNext->m_headssipt->uv[0] = pHead->m_tailssipt->uv[0] ;
					pNext->m_headssipt->uv[1] = pHead->m_tailssipt->uv[1] ;
				}
			}
		}
		else if( dDist[3] < dTol )
		{
			for( i = 0 ; i < 4 ; i++ )
			{
				dMinDist[i] = mathDist( pt[i], pHead->m_tailssipt->p ) ;				
				if( dMinDist[i] < dTol )
				{
					pNext->m_tailssipt->uv[0] = pHead->m_headssipt->uv[0] ;
					pNext->m_tailssipt->uv[1] = pHead->m_headssipt->uv[1] ;
				}
			}
		}
	}
}

void CFlowlineGen::AdjustFrontAngle( CSmtCheckMdl &DriveMdl, CSmtCutPointEx &CutPoint, CNc5DAxisCtrl &axisCtrl ) 
{
	if( CutPoint.m_bType == 0 ) return ;    
	if( m_pMiller->GetType () == smtToolBall || 
		m_pMiller->GetType () == smtToolABall )
		return ;
	if( axisCtrl.m_cToolAxis.m_nAxisType != NCDEF_AXIS_TILTED_TO_CUTDIR )
		return ;

	PNT3D surfpos, pretip, dPivot = { 0.0, 0.0, 0.0 } ;
	VEC3D ToolAxis, dTAxis ,  dNormal ;
	for( int i = 0 ; i < 3 ; i++ )
	{
		surfpos[i] = CutPoint.m_fProjDir[i] ; 
		ToolAxis[i] = CutPoint.m_fSurfNor[i] ;
		dNormal[i] = CutPoint.m_fTempPos[i] ;
		pretip[i] = CutPoint.m_fPoint[i] ;
	}
	double dFrontAng = axisCtrl.m_cToolAxis.m_dFrontAng ;

	VEC3D dRotAxis ;
	if( axisCtrl.m_cToolAxis.m_nRotAxis == 0 )
	{// 旋转轴 0：X轴
		dRotAxis[0] = 1.0, dRotAxis[1] = dRotAxis[2] = 0.0 ;
	}
	else if( axisCtrl.m_cToolAxis.m_nRotAxis == 1 )
	{// 旋转轴 1：Y轴
		dRotAxis[0] = 0.0, dRotAxis[1] = 1.0, dRotAxis[2] = 0.0 ;
	}
	else
	{// 旋转轴 2：Z轴 
		dRotAxis[0] = dRotAxis[1] = 0.0, dRotAxis[2] = 1.0 ;
	}
	DOUBLE dAngStep = ANGLE_TO_RADIAN( 1 ) ;
	//DOUBLE dAngle = ANGLE_TO_RADIAN( dFrontAng ) ;
	int nTry = 10 ;
	CSmtCutPointEx tmpPnt( CutPoint ) ;
	BOOL bFind = FALSE ;
	int k = 0, n = 0 ;
	for( n = 0 ; n < 2 ; n++ )
	{
		for( k = 0 ; k < 3 ; k++ )
			ToolAxis[k] = CutPoint.m_fSurfNor[k] ;
		for( i = 0 ; i < nTry ; i++ )
		{
			if ( fabs( dFrontAng ) > 1.0e-6 )
			{
				if( n == 0 )
					mathRotVec( dRotAxis, dPivot, dAngStep , ToolAxis, ToolAxis ) ;
				else
					mathRotVec( dRotAxis, dPivot, -dAngStep , ToolAxis, ToolAxis ) ;
			}
			// 计算刀具偏移
			PNT3D Offset = {0} ;
			double dLen = mathOProduct( dNormal, ToolAxis ) ;
			if( fabs( dLen - 1 ) > MIN_LEN )
			{
				m_pMiller->Get5AxToolOffset ( ToolAxis, dNormal, Offset ) ;
			}

			Offset[0] += ToolAxis[0] * m_cStockDef.m_dDriveZMove[0];
			Offset[1] += ToolAxis[1] * m_cStockDef.m_dDriveZMove[0];
			Offset[2] += ToolAxis[2] * m_cStockDef.m_dDriveZMove[0];
			for( k = 0 ; k < 3 ; k ++ ) 
			{//刀心转换成刀尖
				tmpPnt.m_fPoint[k] = TFLOAT ( surfpos[k] + Offset[k] ) ;
				tmpPnt.m_fSurfNor[k] = TFLOAT ( ToolAxis[k] ) ;
			}
			//判断是否过切
			if( !DriveMdl.Is5AxPointOvercut ( tmpPnt, m_cSetupDef.m_cTolDef.m_dArcTol * 2. ) )
			{
				tmpPnt.m_bType = 5 ;
				for( k = 0 ; k < 3 ; k++ )
				{
					pretip[k] = tmpPnt.m_fPoint[k] ;
					dTAxis[k] = tmpPnt.m_fSurfNor[k] ;
				}
			}
			else
			{
				bFind = TRUE ;
				break ;
			}
		}
		if( bFind ) break ;
	}
	
		
	if( bFind )
	{
		for( k = 0 ; k < 3 ; k++ )
		{
			CutPoint.m_fPoint[k]	= FLOAT( pretip[k] ) ;
			CutPoint.m_fSurfNor[k]	= FLOAT( dTAxis[k] ) ;
		}
	}
}

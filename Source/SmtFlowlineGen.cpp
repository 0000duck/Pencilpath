// SmtFlowlineGen.cpp: implementation of the CFlowlineGen class.
// 曲面精加工计算类CFlowlineGen的定义
//
//////////////////////////////////////////////////////////////////////
#include "stdafx.h"
#include "SmtPathGen.H"
#include "PencilMdl.h"
#include "PencilLink.h"
#include "SmtPathGen3D.H"
#include "SmtFlowlineGen.h"
#include "mathcam.h"
#include "GeoShell.h"
#include "Nc5DToolAxis.h"
#include "SmtAutoFinishGen.H"
#include "geo_inter.h"
#ifdef _DEBUG
#undef THIS_FILE
static char THIS_FILE[]=__FILE__;
#define new DEBUG_NEW
#endif

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////
static UINT MathCAM_CreateOneSurfPathSubProc(LPVOID lpParam)
{
	if (lpParam == NULL)
		return 0;
	COSPT_DATA *pData = (COSPT_DATA *)lpParam;
	return pData->pFlowlineGen->CreateOneSurfPathSubProc(pData->dArcTol, pData->bOffDir,  pData->pMiller, *pData->cRFrame, 
		pData->pSurf, *pData->PathLib, pData->PrgDef, pData->nAtCore, pData->nCoreNum);
}

// 自身干涉检查
static UINT MathCAM_CheckAllPathBySelfSubProc(LPVOID lpParam)
{
	if (lpParam == NULL)
		return 0;
	CAPT_DATA *pData = (CAPT_DATA *)lpParam;
	return pData->pFlowlineGen->CheckAllPathBySelfSubProc(*pData->pDriveMdl, *pData->AllPath, *pData->pTol, 
		pData->PrgDef, pData->dCur, pData->nAtCore, pData->nCoreNum, pData->NewPath);
}

// 干涉面干涉检查
static UINT MathCAM_CheckAllPathBySurfSubProc(LPVOID lpParam)
{
	if (lpParam == NULL)
		return 0;
	CAPT_DATA *pData = (CAPT_DATA *)lpParam;
	return pData->pFlowlineGen->CheckAllPathBySurfSubProc(*pData->pDriveMdl, *pData->AllPath, *pData->pTol, 
		pData->PrgDef, pData->dCur, pData->bCheck, pData->nAtCore, pData->nCoreNum, pData->NewPath);
}

// 干涉检查 new qqs 2013.11.07
static UINT MathCAM_CheckAllPathByCheckMdlSubProc(LPVOID lpParam)
{
	if (lpParam == NULL)
		return 0;
	CAPT_DATA *pData = (CAPT_DATA *)lpParam;
	return pData->pFlowlineGen->CheckAllPathByCheckMdlSubProc(*pData->pDriveMdl, *pData->AllPath, *pData->pTol, 
		pData->PrgDef, pData->dCur, pData->bCheck, pData->nAtCore, pData->nCoreNum, pData->NewPath);
}

CFlowlineGen::CFlowlineGen( JDNC_SETUP &cSetup, JDNC_FEED &cFeed,
						    JDNC_STOCKEX &cStock, JDNC_TOOLEX &cTool,
							JDNC_SPEED &cSpeedDef, CSmartTool *pTool, 
							int nThreadNum, JDNC_UWLINE &cParam)
{
	m_cSetupDef = cSetup ;
	m_cFeedDef  = cFeed  ;
	m_cStockDef = cStock ;
	m_cToolDef  = cTool  ;
	m_pMiller	= pTool  ;
	m_cSpeedDef	= cSpeedDef;
	m_cPrgDef.m_dLimitAt = 100 / ( 100 * 1.00  ) ;
	m_cPrgDef.m_dStepAt  = 0 ;
	m_cPrgDef.m_dIncStep = 1. ;
	m_cPrgDef.m_pBrkFunc = NULL ;
	m_cPrgDef.m_pPrgFunc = NULL ;
	m_cPrgDef.m_pNewFunc = NULL ;
	m_cPrgDef.m_pPosFunc = NULL ; 
	m_pErrorType = NULL;
	m_nCalcThreadNum = nThreadNum;
	m_dZMove = m_cStockDef.m_dDriveZMove[0] - m_cStockDef.m_dSparkGap  ;
	m_bGrandMdfy = (cParam.m_bUWLineFlag & NCDEF_UWLINE_GRNDMDFY)? TRUE: FALSE;
}

CFlowlineGen::~CFlowlineGen()
{
}

///////////////////////////////////////////////////////////////////////////////
//以下为关于流线走刀的函数
///////////////////////////////////////////////////////////////////////////////

void CFlowlineGen::SearchAdjacentEdge(CGeoTrmSurf* pSurf1, CGeoTrmSurf* pSurf2, CAdjEdgeArray& cAdjEdgeArr)
{
	if( (!pSurf1) || (!pSurf2) || (!pSurf1->m_pLoop) || (!pSurf2->m_pLoop) ) return;
	CTrmFin	*pBnd1, *pBnd2;
	CGeoCurve	*pCurve1, *pCurve2;

	for(pBnd1=pSurf1->m_pLoop->m_pFinHead; pBnd1; pBnd1=pBnd1->GetNext())
	{
		pCurve1=pBnd1->m_pEdge->m_pCurve;
		if(!pCurve1) continue;

		for(pBnd2=pSurf2->m_pLoop->m_pFinHead; pBnd2; pBnd2=pBnd2->GetNext())
		{
			pCurve2=pBnd2->m_pEdge->m_pCurve;
			if(!pCurve2) continue;

			//判断pCurve1和pCurve2是否重合
			int nSameDir = IsSameBound( pBnd1, pBnd2  ) ;
			if(nSameDir!=0)//认为一条边界仅为两个面所共有,不必再搜索;但两个面有可能有两共边
			{
				CAdjEdge* p3DEdge=new CAdjEdge(pCurve1, pSurf1, pSurf2, pBnd1, pBnd2, nSameDir);
				cAdjEdgeArr.Add(p3DEdge);
				break;
			}
		}
	}
}

void CFlowlineGen::ChangePCurve(C3DSurfAttArray& c3DSurfAttArr, bool bRestore)
{
	INT_PTR n=c3DSurfAttArr.GetSize();
	for(INT_PTR i=0; i<n; i++)
	{
		ChangePCurve(c3DSurfAttArr[i], bRestore);
	}
}

BOOL IsEdgeValid(CTrmFin *pEdge)
{
	if (NULL == pEdge || NULL == pEdge->m_pPCurve)
		return FALSE;
	if (pEdge->m_pPCurve->GetLength() < MIN_DIS)
		return FALSE;
	PNT3D pt;
	VEC3D vec;
	pEdge->m_pPCurve->GetTangent(0., pt, vec);
	if (mathVecLen(vec) < MIN_DIS)
		return FALSE;
	pEdge->m_pPCurve->GetTangent(1., pt, vec);
	if (mathVecLen(vec) < MIN_DIS)
		return FALSE;
	return TRUE;
}

CTrmFin* CFlowlineGen::GetSubtense(CTrmFin* pBndSrc) //找对边
{
	if( !pBndSrc || !pBndSrc->m_pLoop ) return NULL;
	CTrmFin *pBnd , *pBndDes = NULL;
	// 取输入边的邻边
	CTrmFin *pBndSrcNext = pBndSrc->GetNext(), *pBndSrcPrev = pBndSrc->GetPrev();
	// 后一边为空，则取头边
	if (NULL == pBndSrcNext)
		pBndSrcNext = pBndSrc->m_pLoop->m_pFinHead;
	// 前一边为空，则取尾边
	if (NULL == pBndSrcPrev)
	{
		pBndSrcPrev = pBndSrc->m_pLoop->m_pFinHead;
		while (pBndSrcPrev->GetNext())
			pBndSrcPrev = pBndSrcPrev->GetNext();
	}
	
	
	double dMaxLength = 0.;
	// 找到非邻边的最长的边
	for( pBnd = pBndSrc->m_pLoop->m_pFinHead ; pBnd ; pBnd = pBnd->GetNext() )
	{
		if (pBnd == pBndSrc || pBnd == pBndSrcNext || pBnd == pBndSrcPrev)
			continue;
		if (!IsEdgeValid(pBnd))
			continue;
		double dLength = pBnd->m_pPCurve->GetLength();
		if (dMaxLength < dLength)
		{
			pBndDes = pBnd;
			dMaxLength = dLength;
		}
	}

	return pBndDes;
}

//根据pEdge和pBnd的信息构造面属性单元
C3DSurfAttribute* CFlowlineGen::CreateSurfAtt(CGeoTrmSurf* pSurf, CTrmFin* pBnd1, CTrmFin* pBnd2, int nCurDir)
{
	if(!pSurf || (!pBnd1 && !pBnd2) )	return NULL;

	if(!pBnd1) pBnd1=GetSubtense(pBnd2);
	if(!pBnd2) pBnd2=GetSubtense(pBnd1);
	// 邻边错误
	if( !pBnd1 || !pBnd2 ) return NULL ;

	if(pSurf->m_pLoop!=pBnd1->m_pLoop || pSurf->m_pLoop!=pBnd2->m_pLoop) return NULL;
//	if( !pBnd1 || !pBnd2 || pBnd1->GetNext()==pBnd2 || pBnd1->GetPrev()==pBnd2) return NULL;//邻边错误

	C3DSurfAttribute* pSurfAtt=new C3DSurfAttribute(pSurf, pBnd1, pBnd2, nCurDir);
	if( pSurfAtt->m_pPCurvStart )
	{
		PNT3D	pnt1, pnt2;
		pSurfAtt->m_pPCurvStart->GetEndPoint(pnt1, pnt2);
		if( fabs(pnt2[1]-pnt1[1])>fabs(pnt2[0]-pnt1[0]) )
			pSurfAtt->m_nCutDir=1;
	}
	return pSurfAtt;
}

//根据cAdjEdgeArr信息对面排序,调整参数域的方向,并排序后的面置于c3DSurfAttArr中
int CFlowlineGen::OrderAllSurf(C3DSurfArray& c3DSurfArr, CAdjEdgeArray& cAdjEdgeArr, C3DSurfAttArray& c3DSurfAttArr)
{
	INT_PTR		nSurf=c3DSurfArr.GetSize();
	INT_PTR		nEdge=cAdjEdgeArr.GetSize();

	if(nSurf==0 || nEdge==0) return 0;
	if(nSurf!=nEdge+1 && nSurf!=nEdge)	//曲面组的共同边界不够或过多，不具备生成流线加工的条件
		return 0;
	
	INT_PTR				i, j;
	CAdjEdge*		pEdge;
	bool			bReversed=false;

	//对邻边和面进行排序
	for(i=0; i<nEdge-1; i++)
	{
		for(j=i+1; j<nEdge; j++)
		{
			if( !cAdjEdgeArr[i] || !cAdjEdgeArr[j] ) continue;
			if( cAdjEdgeArr[i]->m_pRefer[1]==cAdjEdgeArr[j]->m_pRefer[0] ) break;
			else if( cAdjEdgeArr[i]->m_pRefer[1]==cAdjEdgeArr[j]->m_pRefer[1] )
			{
				cAdjEdgeArr[j]->SwapBounds();
				break;
			}
			else ; //未匹配上,试下一个
		}

		if(j>i+1 && j<nEdge)//需要将找到的边放置到当前查找边后面
		{
			pEdge=cAdjEdgeArr[i+1]; cAdjEdgeArr[i+1]=cAdjEdgeArr[j]; cAdjEdgeArr[j]=pEdge;
		}
		else if(j==nEdge)//当前边cAdjEdgeArr[i]->m_pRefer[1]所指面为最右侧面
		{
			if(bReversed) return 0;//已经反转过,这些面不连成面环,返回错误
			//将0到i的边都反转并置已反转标志
			bReversed=true; 

			for(j=0; j<i/2+1; j++)
			{
				if( cAdjEdgeArr[j] ) cAdjEdgeArr[j]->SwapBounds();
				if(j==i-j) break;//指向同一条边

				if( cAdjEdgeArr[i-j] ) cAdjEdgeArr[i-j]->SwapBounds();
				pEdge=cAdjEdgeArr[j]; cAdjEdgeArr[j]=cAdjEdgeArr[i-j]; cAdjEdgeArr[i-j]=pEdge;
			}
			i--;
		}
		else ;//找到的边恰好在当前边后面,不必操作
	}

	int					nCurDir=1;
	C3DSurfAttribute*	pSurfAtt;

	if(nSurf==nEdge)//若为面环
	{	//查看面是否真正成面环
		if( !cAdjEdgeArr[0] || !cAdjEdgeArr[nEdge-1] ) return 0;
		if(cAdjEdgeArr[0]->m_pRefer[0] != cAdjEdgeArr[nEdge-1]->m_pRefer[1]) return 0;
		pSurfAtt=CreateSurfAtt(cAdjEdgeArr[0]->m_pRefer[0], cAdjEdgeArr[nEdge-1]->m_pFins[1], 
								cAdjEdgeArr[0]->m_pFins[0], nCurDir);
		if(!pSurfAtt) return 0;
		c3DSurfAttArr.Add(pSurfAtt);
	}
	else
	{	
		if( !cAdjEdgeArr[0]  ) return 0;
		pSurfAtt=CreateSurfAtt(cAdjEdgeArr[0]->m_pRefer[0], NULL, 
								cAdjEdgeArr[0]->m_pFins[0], nCurDir);
		if(!pSurfAtt) return 0;
		c3DSurfAttArr.Add(pSurfAtt);
	}

	//搜索面的参数域连接情况
	for(i=0; i<nEdge-1; i++)
	{
		if( !cAdjEdgeArr[i] || !cAdjEdgeArr[i]->m_pFins[1] || !cAdjEdgeArr[i+1] ) continue;
		nCurDir*=(-cAdjEdgeArr[i]->m_pFins[1]->m_nSense);
		pSurfAtt=CreateSurfAtt(cAdjEdgeArr[i]->m_pRefer[1], cAdjEdgeArr[i]->m_pFins[1], 
								cAdjEdgeArr[i+1]->m_pFins[0], nCurDir);
		if(!pSurfAtt) return 0;
		c3DSurfAttArr.Add(pSurfAtt);
	}

	if(nSurf==nEdge+1)
	{
		if( !cAdjEdgeArr[nEdge-1] || !cAdjEdgeArr[nEdge-1]->m_pFins[1] ) return 0;
		nCurDir*=(-cAdjEdgeArr[nEdge-1]->m_pFins[1]->m_nSense);
		pSurfAtt=CreateSurfAtt(cAdjEdgeArr[nEdge-1]->m_pRefer[1], cAdjEdgeArr[nEdge-1]->m_pFins[1], 
								NULL, nCurDir);
		if(!pSurfAtt) return 0;
		c3DSurfAttArr.Add(pSurfAtt);
	}

	return 1;
}

void CFlowlineGen::ImagePntToSurf(CSmartTool *pMiller, RFRAME& cRFrame,
								  CGeoTrmSurf* pSurf, CSmtCutPath* pPath,
								  double* dParam, BOOL bOffDir, PNT3D dPoint[2] )
{
	if( !pSurf || !pMiller ) return;

	VEC3D	dNormal, dOffset;
/*
	if(	pSurf->GetNormal( dParam[0], dParam[1], dPoint, dNormal) != ERSUCSS )
	{
		pSurf->GetPoint( dParam[0], dParam[1] , dPoint ) ;
		dNormal[0] = dNormal[1] = 0.0, dNormal[2] = 1.0 ;
	}
*/
	PNT2D param ;
	param[0] = dParam[0], param[1] = dParam[1] ;
	PNT3D Point ;

	GetPointNormal( pSurf, pPath, pMiller, param, dNormal, Point ) ;

	mathTransWorldPnt3D(&cRFrame, Point, Point );
	mathTransWorldVec3D(&cRFrame, dNormal, dNormal);

	if( bOffDir ) mathRevVec( dNormal ) ; 

	pMiller->GetFSafeVector( dNormal, dOffset ) ;
	dPoint[0][0] = Point[0] + dOffset[0] ;
	dPoint[0][1] = Point[1] + dOffset[1] ;
	dPoint[0][2] = Point[2] + dOffset[2] + m_dZMove ;
	mathCpyPnt( dNormal, dPoint[1] ) ;
}

void CFlowlineGen::BisectInsertPnt( CSmartTool *pMiller, RFRAME& cRFrame, CGeoTrmSurf* pSurf, BOOL bOffDir, 
								    CSmtCutPath* pCutPath, CSmtCutPointEx *dStart, CSmtCutPointEx *dEnd )
{
	CSmtCutPointEx *AtFrm = NULL, *AtTo = NULL, *pInsert = NULL ;
	double dArc = 0., t = 0. ;
	double dDist = min( m_cSetupDef.m_cTolDef.m_dMaxStep, 0.15 ) ;

	float fCoef[3] = { 0.5f, 0.25f, 0.75f } ;
	int i = 0, j = 0 ;
	PNT2D mid ;
	PNT3D point[2], from, to, pnt ;
	BOOL bInsert = FALSE ;

	AtFrm = dStart ;
	while( AtFrm && AtFrm->next )
	{
		if( AtFrm == dEnd ) break ;
		AtTo = (CSmtCutPointEx*)AtFrm->next  ;

		// 判断两点的UV参数是否相等，相等则不插点，执行下一循环
		if (fabs(AtFrm->m_fSurfPos[0] - AtTo->m_fSurfPos[0]) < 1e-6 &&
			fabs(AtFrm->m_fSurfPos[1] - AtTo->m_fSurfPos[1]) < 1e-6)
		{
			AtFrm = (CSmtCutPointEx*)AtFrm->next ;
			continue;
		}
		
		// 插入三个点
		for( i = 0 ; i < 3 ; i++ )
		{
			bInsert = FALSE ;
			for( j = 0 ; j < 2 ; j++ )
			{
				mid[j] = AtFrm->m_fSurfPos[j] + fCoef[i] * ( AtTo->m_fSurfPos[j] - AtFrm->m_fSurfPos[j] ) ;
			}
			// 映射到空间域
			ImagePntToSurf( pMiller, cRFrame, pSurf, pCutPath, mid, bOffDir, point ) ;
			nc_FloatToDouble( from, AtFrm->m_fPoint, 3 ) ;
			nc_FloatToDouble( to  , AtTo->m_fPoint  ,3 ) ;
			nc_VectorCopy( pnt , point[0] , 3 ) ;

			// 精度是否合适
			dArc = mathDistPntLinEx( pnt, from, to ) ;
			t = GetLineParam3D( from, to, pnt ) ;
			if( dArc > 1.0e-4 || ( m_cSetupDef.m_cModelTol.m_nMdlFlag & NCDEF_SETUP_NOMOSAIC ) && mathDist( from, to ) > dDist )
			{
				bInsert = TRUE ;
				break ;
			}
		}
		
		if( bInsert )
		{
			// 插入节点
			pInsert = new CSmtCutPointEx() ;
			nc_DoubleToFloat( pInsert->m_fPoint, point[0], 3 ) ;
			nc_DoubleToFloat( pInsert->m_fSurfNor, point[1], 3 ) ;

			pInsert->m_fSurfPos[0] = float( mid[0] ) ;
			pInsert->m_fSurfPos[1] = float( mid[1] ) ;
			pCutPath->InsertAfter( pInsert, AtFrm ) ;
		}
		else
		{
			AtFrm = (CSmtCutPointEx*)AtFrm->next ;
		}
	}
}

void CFlowlineGen::BisectInsert( CSmartTool *pMiller, RFRAME& cRFrame, CGeoTrmSurf* pSurf, 
								 CSmtCutPath* pCutPath, CSmtCutPointEx* pPnt, BOOL bOffDir,
								 PNT3D pt1, PNT3D pt2, double* param1, double* param2, 
								 double dArcTol, int &nCnt )
{
	if( !pSurf || !pMiller || !pCutPath ) return ;
	double param[2];
	PNT3D pnt[2] ;

	param[0]=0.5 *( param1[0]+param2[0]);
	param[1]=0.5 *( param1[1]+param2[1]);

	ImagePntToSurf( pMiller, cRFrame, pSurf, pCutPath, param, bOffDir, pnt );
	double d = mathDistPntLinEx( pnt[0], pt1, pt2 ) ;
	// 为了防止在退化点处递归调用死循环
	if( d < 1.0e-6 ) return ;
	// 保证连续两个点在允许的误差范围之内
	if( d < dArcTol ) 	nCnt++ ;
	else				nCnt = 0 ;
	
	double dDist = min( m_cSetupDef.m_cTolDef.m_dMaxStep, 0.15 ) ;
	if( nCnt < 2 || ( m_cSetupDef.m_cModelTol.m_nMdlFlag & NCDEF_SETUP_NOMOSAIC ) && mathDist( pt1, pt2 ) > dDist )
	{
		CSmtCutPointEx*	pNewPnt = new CSmtCutPointEx() ;
		nc_DoubleToFloat( pNewPnt->m_fPoint, pnt[0], 3 ) ;
		nc_DoubleToFloat( pNewPnt->m_fSurfNor, pnt[1], 3 ) ;
		pNewPnt->m_fSurfPos[0] = float( param[0] ) ;
		pNewPnt->m_fSurfPos[1] = float( param[1] ) ;
		
		pCutPath->InsertBefore( pNewPnt, pPnt ) ;
		int nNum = nCnt ;
		BisectInsert( pMiller, cRFrame, pSurf, pCutPath, pNewPnt,bOffDir, pt1, pnt[0], param1, param, dArcTol, nCnt );
		BisectInsert( pMiller, cRFrame, pSurf, pCutPath, pPnt,   bOffDir, pnt[0], pt2, param, param2, dArcTol, nNum );
	}
}

BOOL CFlowlineGen::CutPathToSurf(CSmartTool *pMiller, RFRAME& cRFrame, CSmtCutPath* pCutPath, CGeoTrmSurf* pSurf, BOOL bOffDir, double dArcTol)
{	
	UNUSED_ALWAYS( dArcTol ) ;
	if( (!pCutPath) || (!pSurf) || (!pMiller) || pCutPath->m_nNumPnt <= 1 ) return FALSE;
	
	// 曲面流线磨削调整 qqs 2013.05.20 
	// 将参数域路径投影到曲面的函数，修改后可以处理节点数大于2个的参数域路径
	pCutPath->DefineBox () ;
	CSmtCutPath* pTempCutPath = pCutPath->CopyMyself();
	pCutPath->ClearAllPoint();
	CSmtCutPoint* St = pTempCutPath->m_pHead;
	while(St&&St->next)
	{
		CSmtCutPath* pNewPath = new CSmtCutPath;
		CSmtCutPointEx* dStart = (CSmtCutPointEx*)St->CopyMyself() ;
		CSmtCutPointEx* dEnd = (CSmtCutPointEx*)St->next->CopyMyself();
		pNewPath->AddTail(dStart);
		pNewPath->AddTail(dEnd);
		PNT2D start, end ;
		PNT3D pStart[2], pEnd[2] ;

		nc_FloatToDouble( start, dStart->m_fPoint, 2 ) ;
		nc_FloatToDouble( end  , dEnd->m_fPoint  , 2 ) ;

		ImagePntToSurf( pMiller, cRFrame, pSurf, pNewPath, start, bOffDir, pStart ) ;
		ImagePntToSurf( pMiller, cRFrame, pSurf, pNewPath, end, bOffDir, pEnd ) ;

		nc_DoubleToFloat( dStart->m_fPoint, pStart[0], 3 ) ;
		nc_DoubleToFloat( dStart->m_fSurfNor, pStart[1], 3 ) ;
		nc_DoubleToFloat( dEnd->m_fPoint, pEnd[0], 3 ) ;
		nc_DoubleToFloat( dEnd->m_fSurfNor, pEnd[1], 3 ) ;
		for( int i = 0 ; i < 2 ; i++ )
		{
			dStart->m_fSurfPos[i] = float( start[i] ) ;
			dEnd->m_fSurfPos[i] = float( end[i] ) ;
		}

		BisectInsertPnt( pMiller, cRFrame, pSurf, bOffDir, pNewPath, dStart, dEnd ) ;

		CSmtCutPoint* ptIn = pNewPath->m_pHead;
		pCutPath->AddTail(ptIn->CopyMyself());
		ptIn = ptIn->next;
		while(ptIn)
		{			
			pCutPath->AddTail(ptIn->CopyMyself());		
			ptIn = ptIn->next;
		}
		St = St->next;
		delete pNewPath;
		pNewPath = NULL;
	}
	delete pTempCutPath;
	pTempCutPath = NULL;
	return TRUE;
}

int CFlowlineGen::GenParamSerialByStep( CGeoPLine3d* pPLine, double dStep, double* dParam, int& nNum )
{
	if( !pPLine || !dParam ) { nNum=0; return 0; }

	double	dCurLen=0, dLen, dPrevLen=0;
	int		k=0;
	bool	bMaxNum=false;

	for( int i=0; i<pPLine->m_num; i++ )
	{
		dLen=mathDist( pPLine->m_dPoint[i], pPLine->m_dPoint[i+1] );

		for( dCurLen-=dPrevLen; dCurLen<=dLen; dCurLen+=dStep )
		{
			dParam[k]=((double) i)/pPLine->m_num+dCurLen*1.0/(pPLine->m_num*dLen);
			if(k<nNum) k++;
			else bMaxNum=true;
		}
		dPrevLen=dLen;
	}

	if( !bMaxNum ) nNum=k;
	dParam[nNum]=1.0;

	return 1;
}

// 根据流线方向，得到最长的那根路径（从u/v = 0，0.5，1中选择）
CSmtCutPath *CFlowlineGen::GetMaxLenPath(CSmartTool *pMill, RFRAME &lf, CGeoTrmSurf* pSurf, 
										 CSmartLoop* pLoop, int nCutDir, BOOL bOffDir, double dArcTol)
{
	if( NULL == pMill || NULL == pSurf || NULL == pLoop) 
		return NULL;

	// 得到UW参数边界
	PNT2D uwBox[2];
	if( pSurf->m_pLoop )
	{
		nc_VectorCopy(uwBox[0], pLoop->m_dBox[0], 2);
		nc_VectorCopy(uwBox[1], pLoop->m_dBox[1], 2);
	}
	else
	{
		uwBox[0][0] = uwBox[0][1] = 0.0 ;
		uwBox[1][0] = uwBox[1][1] = 1.0 ;
	}

	int i = 0;
	PNT3D st[3], ed[3] ;		// 三条参数线的超末点
	CSmtCutPath *pPath[3] = { NULL, NULL, NULL }, *pFind = NULL ;
	CSmtCutPointEx *pnt1 = NULL,  *pnt2 = NULL ;
	for( i = 0 ; i < 3 ; i++ )
	{
		st[0][i] = st[1][i] = st[2][i] = 0. ;
		ed[0][i] = ed[1][i] = ed[2][i] = 0. ;
	}

	int nLineDir = -1;
	if( nCutDir == SURF_WDIR )	// 加工方向W向，取U向三条线
	{
		nLineDir = SURF_UDIR;
		st[0][0] = st[1][0] = st[2][0] = uwBox[0][0] - 0.01;
		st[0][1] = uwBox[0][1] + 0.01, st[1][1] = 0.5 * (uwBox[0][1] + uwBox[1][1]), st[2][1] = uwBox[1][1] - 0.01;
		ed[0][0] = ed[1][0] = ed[2][0] = uwBox[1][0] + 0.01;
		ed[0][1] = uwBox[0][1] + 0.01, ed[1][1] = 0.5 * (uwBox[0][1] + uwBox[1][1]), ed[2][1] = uwBox[1][1] - 0.01;
	}
	else						// 加工方向U向，取W向三条线
	{
		nLineDir = SURF_WDIR;
		st[0][0] = uwBox[0][0] + 0.01, st[1][0] = 0.5 * (uwBox[0][0] + uwBox[1][0]), st[2][0] = uwBox[1][0] - 0.01;
		st[0][1] = st[1][1] = st[2][1] = uwBox[0][1] - 0.01;
		ed[0][0] = uwBox[0][0] + 0.01, ed[1][0] = 0.5 * (uwBox[0][0] + uwBox[1][0]), ed[2][0] = uwBox[1][0] - 0.01;
		ed[0][1] = ed[1][1] = ed[2][1] = uwBox[1][1] + 0.01;
	}

	PNT2D		dIntPnt[200];
	CSmartSect*	pSectList[200];

	for( i = 0 ; i < 3 ; i++ )
	{	
		// 得到与曲面的交点
		UINT nSize = 200;
		nSize = pLoop->GetLineInt(st[i], ed[i], dIntPnt, pSectList, nSize);
		if(nLineDir == SURF_WDIR)		// 若为W向，先交换交点到U向
		{
			for( UINT j=0; j<nSize; j++ ) 
			{
				double tmp = dIntPnt[j][0];
				dIntPnt[j][0] = dIntPnt[j][1];
				dIntPnt[j][1] = tmp;
			}
		}
		// 对交点排序
		nSize = pLoop->SortPointSect( dIntPnt, pSectList, nSize ) ;
		if (nSize >= 2)
		{
			if(nLineDir == SURF_WDIR)	// 若为W向，交换交点到初始状态
			{
				for( UINT j=0; j<nSize; j++ ) 
				{
					double tmp = dIntPnt[j][0];
					dIntPnt[j][0] = dIntPnt[j][1];
					dIntPnt[j][1] = tmp;
				}
			}
			// 取首末点
			nc_VectorCopy(st[i], dIntPnt[0], 2);
			nc_VectorCopy(ed[i], dIntPnt[nSize - 1], 2);
		}

		pPath[i] = new CSmtCutPath() ;
		pnt1 = new CSmtCutPointEx() ;
		pnt2 = new CSmtCutPointEx() ;
		nc_DoubleToFloat( pnt1->m_fPoint, st[i], 3 );
		nc_DoubleToFloat( pnt2->m_fPoint, ed[i], 3 ) ;
		pPath[i]->AddTail( pnt1 ), pPath[i]->AddTail( pnt2 ) ;
	}

	// 得到最长的那根路径
	double dMax = -1. ;
	for( i = 0 ; i < 3 ; i++ )
	{
		pPath[i]->DefineBox () ;
		// 得到刀心点路径
		CutPathToSurf( pMill, lf, pPath[i], pSurf, bOffDir, dArcTol ) ;
		double dDist = pPath[i]->GetLength () ;
		if( dDist > dMax )
		{
			dMax = dDist ;
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
	return pFind;
}

void CFlowlineGen::GenParamSerialByStep( CSmartTool *pMill, RFRAME &lf, CGeoTrmSurf* pSurf,   
										 double dStep, int nCutDir, double*& dParam, int& nNum, 
										 BOOL bOffDir, double dArcTol, CSmtCPathLib &AllPath )
{
	UNUSED_ALWAYS( AllPath ) ;
	if( !pSurf || !pMill ) return ;

	int		nDirUW, i = 0 ;
	FPNT3D st[3], ed[3] ;
	CSmtCutPath *pPath[3] = { NULL, NULL, NULL }, *pFind = NULL ;
	CSmtCutPointEx *pnt1 = NULL,  *pnt2 = NULL ;
	for( i = 0 ; i < 3 ; i++ )
	{
		st[0][i] = st[1][i] = st[2][i] = 0.f ;
		ed[0][i] = ed[1][i] = ed[2][i] = 0.f ;
	}
	if( nCutDir == 1 )
	{
		nDirUW = SURF_UDIR ;	
		st[0][0] = st[1][0] = st[2][0] = float( pSurf->m_pLoop->m_dBox2d[0][0] ) ;
		ed[0][0] = ed[1][0] = ed[2][0] = float( pSurf->m_pLoop->m_dBox2d[1][0] ) ;
		st[0][1] = ed[0][1] = float( pSurf->m_pLoop->m_dBox2d[0][1] ) ;
		st[1][1] = ed[1][1] = float( ( pSurf->m_pLoop->m_dBox2d[0][1] + pSurf->m_pLoop->m_dBox2d[1][1]) * 0.5 ) ;
		st[2][1] = ed[2][1] = float( pSurf->m_pLoop->m_dBox2d[1][1] ) ;

//		st[0][1] = 0.f, st[1][1] = 0.5f, st[2][1] = 1.0f ;
//		ed[0][0] = ed[1][0] = ed[2][0] = 1.f ;
//		ed[0][1] = 0.f, ed[1][1] = 0.5f, ed[2][1] = 1.0f ;
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
		nDirUW = SURF_WDIR ;
		st[0][1] = st[1][1] = st[2][1] = float( pSurf->m_pLoop->m_dBox2d[0][1] ) ;
		ed[0][1] = ed[1][1] = ed[2][1] = float( pSurf->m_pLoop->m_dBox2d[1][1] ) ;
		st[0][0] = ed[0][0] = float( pSurf->m_pLoop->m_dBox2d[0][0] ) ;
		st[1][0] = ed[1][0] = float( ( pSurf->m_pLoop->m_dBox2d[0][0] + pSurf->m_pLoop->m_dBox2d[1][0] ) * 0.5 ) ;
		st[2][0] = ed[2][0] = float( pSurf->m_pLoop->m_dBox2d[1][0] ) ;
//		st[0][0] = 0.f, st[1][0] = 0.5f, st[2][0] = 1.0f ;
//		ed[0][0] = 0.f, ed[1][0] = 0.5f, ed[2][0] = 1.0f ;
//		ed[0][1] = ed[1][1] = ed[2][1] = 1.f ;
		for( i = 0 ; i < 3 ; i++ )
		{	
			pPath[i] = new CSmtCutPath() ;
			pnt1 = new CSmtCutPointEx() ;
			pnt2 = new CSmtCutPointEx() ;
			mathFCpyPnt( st[i], pnt1->m_fPoint ), mathFCpyPnt( ed[i], pnt2->m_fPoint ) ;
			pPath[i]->AddTail( pnt1 ), pPath[i]->AddTail( pnt2 ) ;
		}

	}
	// 得到最长的那根路径
	double dDist[3] = { 0., 0., 0. } ;
	double dMax = -1. ;
	for( i = 0 ; i < 3 ; i++ )
	{
		pPath[i]->DefineBox () ;
		// 得到刀心点路径
		CutPathToSurf( pMill, lf, pPath[i], pSurf, bOffDir, dArcTol ) ;
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
		delete pFind ;
		return ;
	}
	// 均分pFind,参数域采用线性插值
	RenewPathByStepAndSurf( pFind, dStep, pSurf, lf, FALSE ) ;
/////////////////////////for test/////////////////////
//	AllPath.AddToTail ( pFind->CopyMyself () ) ;
//////////////////////////////////////////////////
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
	// 记录首尾的参数域值 qqs 2013.05.20
	// 注释掉原因：因为此处需要保存裁剪面真实的参数域范围，所以不需要进行置0和1的操作。 qqs 2014.04.01
// 	if (i != 1)
// 	{
// 		dParam[0] = 0., dParam[nNum-1] = 1. ;
// 	}
	
	nNum -= 1 ;
	delete pFind ;
	return ;
}

BOOL CFlowlineGen::CalcToolPos( CGeoTrmSurf *pSurf, CSmartTool *pTool, JDNC_TOL &cTol, PNT3D Tip, PNT3D Nor, PNT3D Axis, RFRAME &lf, double &du, double &dw )
{
	if ((!nc_Normalize(Axis, 3)) || (!nc_Normalize(Nor, 3)))
		return FALSE;
	double dist = 100.;
	PNT3D surfpos, pos;
	VEC3D Offset = {0};
 	double dLen = mathOProduct( Nor, Axis ) ;
	if( fabs( dLen - 1 ) > MIN_LEN )
		pTool->Get5AxToolOffset ( Axis, Nor, Offset ) ;

	for(int i = 0 ; i < 3 ; i++ )
	{// 得到刀触点
		pos[i] = Tip[i] - Offset[i] ;
	}
	pos[2] = pos[2] - m_dZMove ;
	// 得到曲面上的最近点
	if( !pSurf || ! pSurf->m_pSurface )
	{
		return FALSE ;
	}

	CGeoInter cGeoInter ;
	mathTransLocalPnt3D( &lf, pos,  pos ) ;
	// 已知初始参数, 精确迭代求解点到几何曲面最近距离
	int nRet = cGeoInter.IterateSurfNearestPnt( pSurf->m_pSurface, pos, du, dw, surfpos, dist ) ;
	if( nRet == 1 && dist < cTol.m_dArcTol * 0.5 )
	{
		// 更新Nor
		pSurf->m_pSurface->GetNormal( &lf, du, dw, surfpos, Nor ) ;
		return TRUE ;
	}
	// 或者dist比较小,du或者dw趋近于0或者1表示在跟踪边界处,也可以
	if( dist < 2.0e-4 && ( fabs( dw ) < 0.001 || fabs( du ) < 0.001 ||
		fabs( du - 1. ) < 0.001 || fabs( dw - 1. ) < 0.001	) )
	{
		// 更新Nor
		pSurf->m_pSurface->GetNormal( &lf, du, dw, surfpos, Nor ) ;
		return TRUE ;
	}
	return FALSE ;
}

void CFlowlineGen::RenewPathByStepAndSurf ( CSmtCutPath *&pPath, double dStep, CGeoTrmSurf *pSurf, RFRAME &lf, BOOL bFlag, BOOL b5Axis )
{
	double t, dLen = pPath->GetLength (), dDist = 0., dAtDist = 0 ;

	BOOL bGrandMdfy ;
	// bFlag为TRUE时则正常路径间距进行插点，不论是否开启磨削调整
	// 为FALSE时，进行判断，如果开启了磨削调整，则按照磨削调整方式进行插点，没开启则正常插点
	if (bFlag) bGrandMdfy = FALSE; 
	else bGrandMdfy = m_bGrandMdfy;	
	
	// 未开启磨削调整情况下，路径间距过大时，则只生成中间一条曲面流线路径
	if (!bGrandMdfy && dLen <= dStep && !bFlag) 
	{
		bGrandMdfy = TRUE;
	}

	CSmtCutPointEx *pHead = (CSmtCutPointEx *)pPath->m_pHead, *pNew = NULL ;
	CSmtCutPointEx tmpPnt, *pNext = NULL, *pTail = ( CSmtCutPointEx *)pPath->m_pTail ;
	PNT3D tip, pos, nor, axis = {0.,0.,1.} ;
	double du = 0., dw = 0. ;
	int i = 0,  n = 0, nCnt = 0 ;

	// 曲面流线磨削调整 qqs 2013.05.20 
	// 当开启曲面流线磨削调整后，只生成曲面中间的一根流线路径，进行磨削调整变换
	if( !bGrandMdfy)
	{
		if(bFlag)
			nCnt = ( int ) ceil( dLen / dStep ) ;
		else
			nCnt = ( int ) ceil( dLen / dStep + 0.5 ) ;
		dStep = dLen / nCnt ;
	}
	else
	{
		nCnt = 1 ;
		dStep = dLen * 0.5 ;
	}
	CSmtCutPath *pNewPath = new CSmtCutPath() ;
	// 添加第一个点
	if( !bGrandMdfy)
	{
		mathFCpyPnt( pHead->m_fPoint  , tmpPnt.m_fPoint   ) ;
		mathFCpyPnt( pHead->m_fSurfPos, tmpPnt.m_fSurfPos ) ;
		mathFCpyPnt( pHead->m_fSurfNor, tmpPnt.m_fSurfNor ) ;
		pNew = new CSmtCutPointEx( tmpPnt ) ;
		pNewPath->AddTail ( pNew ) ;
	}
	while( pHead )
	{
		pNext = ( CSmtCutPointEx *)pHead->next ;
		if( !pNext ) break ;
		dDist = mathFDist( pHead->m_fPoint, pNext->m_fPoint ) ;
		dAtDist += dDist ;
		while( dAtDist >= dStep )
		{
			t = 1 - ( dAtDist - dStep ) / dDist ;
			n++ ;
			for( i = 0 ; i < 3 ; i++ )
			{
				tip[i] = pHead->m_fPoint[i]   + t * ( pNext->m_fPoint[i] - pHead->m_fPoint[i] )  ;
				pos[i] = pHead->m_fSurfPos[i] + t * ( pNext->m_fSurfPos[i] - pHead->m_fSurfPos[i] ) ;
				if( !b5Axis )
				{
					nor[i] = pHead->m_fSurfNor[i] + t * ( pNext->m_fSurfNor[i] - pHead->m_fSurfNor[i] ) ;
				}
				else
				{
					nor[i] = pHead->m_fTempPos[i] + t * ( pNext->m_fTempPos[i] - pHead->m_fTempPos[i] ) ;
					axis[i] = pHead->m_fSurfNor[i] + t * ( pNext->m_fSurfNor[i] - pHead->m_fSurfNor[i] ) ;
				}
			}
			if( !b5Axis )
			{
				du = pHead->m_fSurfPos[0], dw = pHead->m_fSurfPos[1] ;
			}
			else
			{
				du = pHead->m_fSurfPos[0] + t * ( pNext->m_fSurfPos[0] - pHead->m_fSurfPos[0] ) ;
				dw = pHead->m_fSurfPos[1] + t * ( pNext->m_fSurfPos[1] - pHead->m_fSurfPos[1] ) ;
			}
			if( CalcToolPos( pSurf, m_pMiller, m_cSetupDef.m_cTolDef, tip, nor, axis, lf, du, dw ) )
			{
				
				tmpPnt.m_fSurfPos[0] = float( du ), tmpPnt.m_fSurfPos[1] = float( dw ) ;
			}
			else
			{
				tmpPnt.m_fSurfPos[0] = float( pos[0] ), tmpPnt.m_fSurfPos[1] = float( pos[1] ) ;
			}
			nc_DoubleToFloat( tmpPnt.m_fPoint  , tip, 3 ) ;
			nc_DoubleToFloat( tmpPnt.m_fSurfNor, nor, 3 ) ;
			pNew = new CSmtCutPointEx( tmpPnt ) ;
			pNewPath->AddTail ( pNew ) ;
			dAtDist -= dStep ;
		}
		pHead = pNext ;
	}
	if( bGrandMdfy && n > nCnt )
	{ // 如果开启磨削调整只要保存一个点即可 qqs 2013.05.20
		pNewPath->DeletePoint ( pNewPath->m_pTail ) ;
	}
	else if( n < nCnt )
	{
		mathFCpyPnt( pTail->m_fPoint  , tmpPnt.m_fPoint   ) ;
		mathFCpyPnt( pTail->m_fSurfPos, tmpPnt.m_fSurfPos ) ;
		mathFCpyPnt( pTail->m_fSurfNor, tmpPnt.m_fSurfNor ) ;
		pNew = new CSmtCutPointEx( tmpPnt ) ;
		pNewPath->AddTail ( pNew ) ;
	}

	delete pPath ;
	pPath = pNewPath ;
}

void CFlowlineGen::CreateLinear( double* dParam, int nNum, int nCutDir, BOOL /*bZigZag*/, CSmartLoop* pLoop, CSmtCPathLib& cSmtCPathLib )
{
	if( !dParam || !pLoop ) return;

	PNT2D		dIntPnt[200];
	CSmartSect*	pSectList[200];
	PNT2D		pnt1={pLoop->m_dBox[0][0]-NCSF_TOL_01, pLoop->m_dBox[0][1]-NCSF_TOL_01}, 
		pnt2={pLoop->m_dBox[1][0]+NCSF_TOL_01, pLoop->m_dBox[1][1]+NCSF_TOL_01};

	for( int i=0; i<=nNum; i++ )
	{
		unsigned int	nSize=200;
		if( nCutDir==1 )	pnt1[0]=pnt2[0]=dParam[i];
		else			pnt1[1]=pnt2[1]=dParam[i];

		nSize=pLoop->GetLineInt(pnt1, pnt2, dIntPnt, pSectList, nSize);
		if(nCutDir==1)
		{
			for( unsigned int k=0; k<nSize; k++ ) 
			{
				double	tmp=dIntPnt[k][0];
				dIntPnt[k][0]=dIntPnt[k][1];
				dIntPnt[k][1]=tmp ;
			}
		}

		nSize = pLoop->SortPointSect( dIntPnt, pSectList, nSize ) ;
		if( nSize<=1 || nSize%2 ) continue;
		if(nCutDir==1)
		{
			for( unsigned int k=0; k<nSize; k++ ) 
			{
				double	tmp=dIntPnt[k][0];
				dIntPnt[k][0]=dIntPnt[k][1];
				dIntPnt[k][1]=tmp ;
			}
		}

		for(unsigned int j=0; j<nSize; j+=2)
		{
			CSmtCutPath*	pNew=new CSmtCutPath;
			PNT4D			dPnt1={dIntPnt[j][0], dIntPnt[j][1], 0.0, 0.0};
			PNT4D			dPnt2={dIntPnt[j+1][0], dIntPnt[j+1][1], 0.0, 0.0};

			AddPathPointEx( pNew, dPnt1 ) ;
			AddPathPointEx( pNew, dPnt2 ) ;



			pNew->m_nLineNo = i ;
			if( nSize > 2 ) 	pNew->m_nLayerNo = -1 ;
			else				pNew->m_nLayerNo = 0 ;
			cSmtCPathLib.AddToTail(pNew);
		}
	}

}

void CFlowlineGen::CreateLinearNew( double* dParam, int nNum, int nCutDir, CSmartLoop* pLoop, 
	                                CSmtCPathLib& cSmtCPathLib, RFRAME& cRFrame, double dArcTol,
									CGeoTrmSurf* pSurf, JDNC_UWLINE& cParam, BOOL b5Axis )
{
	if( !dParam || !pLoop ) return;

	PNT2D		dIntPnt[200];
	CSmartSect*	pSectList[200];
	PNT2D		pnt1={pLoop->m_dBox[0][0]-NCSF_TOL_01, pLoop->m_dBox[0][1]-NCSF_TOL_01}, 
				pnt2={pLoop->m_dBox[1][0]+NCSF_TOL_01, pLoop->m_dBox[1][1]+NCSF_TOL_01};

	for( int i=0; i<=nNum; i++ )
	{
		unsigned int	nSize=200;
		if( nCutDir==1 )	pnt1[0]=pnt2[0]=dParam[i];
		else			pnt1[1]=pnt2[1]=dParam[i];

		nSize=pLoop->GetLineInt(pnt1, pnt2, dIntPnt, pSectList, nSize);
		if(nCutDir==1)
		{
			for( unsigned int k=0; k<nSize; k++ ) 
			{
				double	tmp=dIntPnt[k][0];
				dIntPnt[k][0]=dIntPnt[k][1];
				dIntPnt[k][1]=tmp ;
			}
		}
	
		nSize = pLoop->SortPointSect( dIntPnt, pSectList, nSize ) ;
		if( nSize<=1 || nSize%2 ) continue;
		if(nCutDir==1)
		{
			for( unsigned int k=0; k<nSize; k++ ) 
			{
				double	tmp=dIntPnt[k][0];
				dIntPnt[k][0]=dIntPnt[k][1];
				dIntPnt[k][1]=tmp ;
			}
		}

		for(unsigned int j=0; j<nSize; j+=2)
		{
			CSmtCutPath*	pNew=new CSmtCutPath;
			PNT4D			dPnt1={dIntPnt[j][0], dIntPnt[j][1], 0.0, 0.0};
			PNT4D			dPnt2={dIntPnt[j+1][0], dIntPnt[j+1][1], 0.0, 0.0};

			if (!m_bGrandMdfy)
			{
				AddPathPointEx( pNew, dPnt1 ) ;
				AddPathPointEx( pNew, dPnt2 ) ;
				pNew->m_nLineNo = i ;
				if( nSize > 2 ) 	pNew->m_nLayerNo = -1 ;
				else				pNew->m_nLayerNo = 0 ;
				cSmtCPathLib.AddToTail(pNew);
			}
			else
			{// 开启磨削调整 qqs 2013.05.20
				GrindParamPathGen(pNew, cParam, cRFrame, pSurf, nCutDir, dArcTol, dPnt1, dPnt2, b5Axis);

				pNew->m_nLineNo = i ;
				if( nSize > 2 ) 	pNew->m_nLayerNo = -1 ;
				else				pNew->m_nLayerNo = 0 ;
				cSmtCPathLib.AddToTail(pNew);
			}
		}
	}
}

void CFlowlineGen::GrindParamPathGen(CSmtCutPath* pNew, JDNC_UWLINE& cParam, RFRAME& cRFrame,  CGeoTrmSurf* pSurf, 
	                                  int nCutDir, double dArcTol, PNT4D dHead, PNT4D dTail, BOOL b5Axis)
{
	BOOL bOffDir = GetOffSetDir(pSurf, cRFrame, cParam);
	//1. 计算参数域路径上的振幅值
	double dMdfyHeit = GetMdfyHeit(m_pMiller, cParam, cRFrame,pSurf,nCutDir,bOffDir,dArcTol);

	// 2. 将参数域路径进行投影，将投影出的最长路径按照磨削调整周期长度进行插点
	CSmtCutPath* pPath = new CSmtCutPath;
	AddPathPointEx( pPath, dHead ) ;
	AddPathPointEx( pPath, dTail ) ;
	CutPathToSurf(m_pMiller,cRFrame, pPath,pSurf, bOffDir, dArcTol  );
	RenewPathByStepAndSurf(pPath,cParam.m_dMdfyDist,pSurf,cRFrame,TRUE, b5Axis);

	// 3. 取进行插点后的路径的每个周期，映射回参数域，并进行磨削调整变换。
	CSmtCutPointEx* pSt = (CSmtCutPointEx*)pPath->m_pHead;
	while(pSt && pSt->next)
	{
		CSmtCutPath* tempPath = new CSmtCutPath;
		CSmtCutPointEx* pEnd = (CSmtCutPointEx*)pSt->next;
		AddPathPointEx( tempPath, pSt->m_fSurfPos ) ;
		AddPathPointEx(tempPath,pEnd->m_fSurfPos);
		ModifyGrindParamPath(pNew, tempPath, cParam.m_nMdfyType, dMdfyHeit,cParam.m_nMdfySegs,1,nCutDir);
		pSt = (CSmtCutPointEx*)pSt->next;
		delete tempPath;
		tempPath = NULL;
	}
	delete pPath;
	pPath = NULL;
	return;
}

BOOL CFlowlineGen::IsSet3DRCompMask ()
{
	if (!m_bGrandMdfy) return FALSE;

	if ( m_cSetupDef.m_cModelTol.m_nMdlFlag & NCDEF_SETUP_WEARCOMP_INC ||
		 m_cSetupDef.m_cModelTol.m_nMdlFlag & NCDEF_SETUP_WEARCOMP_DEC ) 
	{
		return TRUE; 
	}
	else
	{
		return FALSE ;
	}
}

// 将参数域路径进行磨削调整变换
BOOL CFlowlineGen::ModifyGrindParamPath(CSmtCutPath* pParamPath, CSmtCutPath* pCutPath, int nType, DOUBLE depthInc,int  SinSegs, int nSect, int nCutDir )
{
	//在等分段处插入分割点
	pCutPath->NormalizeLen();		

	if (SinSegs < 20) SinSegs = 20;
	int   nSect2 = 0 ;

	if(nType == 1)
	{
		nSect2 = SinSegs * nSect ;
	}
	else if (nType == 2)
	{
		nSect2 = 4 * nSect ;
	}
	else
	{
		nSect2 = 2 * nSect ; 
	}

	//插入新等分的节点
	InsertNewCutPoint(pCutPath,nSect2);

	//在分段路径内构造波动路径点并连接		
	if(nType == 0)//三角形
	{
		CreateTriWave(pCutPath,depthInc,nSect,nCutDir);
	}
	else if(nType == 1)//类正弦
	{	
		CreateSinWave(pCutPath,depthInc,nSect,nCutDir);			
	}
	else if(nType == 2)//梯形
	{				
		CreateTrapWave(pCutPath,depthInc,nSect,nCutDir);
	}
	pCutPath->DelPointOnLine(2.0e-6);//由于是对参数域进行操作，整体数值较小，所以提高精度

	CSmtCutPoint* tempPoint = pCutPath->m_pHead;
	while(tempPoint)
	{
		PNT4D ptIn = {0,0,0,0};
		for (int a = 0; a < 4; a++)
		{
			ptIn[a] = tempPoint->m_fPoint[a];
		}
		AddPathPointEx(pParamPath, ptIn);
		tempPoint = tempPoint->next;					
	}
	return TRUE ;
}

// 插入等分点 qqs 2013.05.20
void CFlowlineGen::InsertNewCutPoint(CSmtCutPath * CutPath ,int nPoint)
{
	TFLOAT t = 0.0,  fBnd1 = 0.0;

	CSmtCutPoint* pCPnt = NULL ;		
	for( int i = 1 ; i < nPoint; i++ )
	{
		fBnd1 = (TFLOAT) i/nPoint ;

		for( pCPnt = CutPath->m_pHead ; pCPnt ; pCPnt = pCPnt->next )
		{
			CSmtCutPoint* pNewCPnt = NULL ;
			if( fBnd1 <= pCPnt->m_fPoint[3])
			{
				ASSERT(pCPnt->prev) ;						
				t = fBnd1 - pCPnt->prev->m_fPoint[3] ;
				t /= TFLOAT( pCPnt->m_fPoint[3] - pCPnt->prev->m_fPoint[3] ) ;
				pNewCPnt = new CSmtCutPoint() ;
				pCPnt->prev->CalcMidPoint(pCPnt, t, pNewCPnt) ;
				CutPath->InsertBefore(pNewCPnt, pCPnt) ;
				break ;
			}
		}
	}
}

//生成三角形波动 qqs 2013.05.20
void CFlowlineGen::CreateTriWave(CSmtCutPath * CutPath,DOUBLE depthInc,int nSect, int nCutDir)
{
	nCutDir = !nCutDir;
	if( nSect > 10000 ) nSect = 10000 ;
	CSmtCutPoint* pCPnt =NULL;
	TFLOAT fBnd1 = 0.0, fBnd2 = 0.0, fMid = 0.0;
	DOUBLE dCoef = 0.0 ;
	int k = 0 ;    //标记等分点序号
	BOOL bReverse= TRUE;//错峰标记
	if(!bReverse)
	{
		if (nCutDir)
		{
			CutPath->TransPath(0.0f,TFLOAT(-depthInc), 0.0f );
		}
		else
		{
			CutPath->TransPath(TFLOAT(-depthInc),0.0f, 0.0f );
		}
	}
	for( k = 0 ; k < nSect ; k++)
	{
		fBnd1 = (TFLOAT)k/nSect ;
		fBnd2 = (TFLOAT)(k+1)/nSect ; 
		fMid = (fBnd1 + fBnd2) / 2.0f ;
		for( pCPnt = CutPath->m_pHead ; pCPnt ; pCPnt = pCPnt->next )
		{
			if(pCPnt->m_fPoint[3] < fBnd1)
				continue ;
			if(pCPnt->m_fPoint[3] > fBnd2)
			{
				break ;
			}
			if(pCPnt->m_fPoint[3] < fMid )
			{
				ASSERT(fMid - fBnd1 > 1.0e-8);
				dCoef = (pCPnt->m_fPoint[3] - fBnd1)/(fMid - fBnd1);
				if(!bReverse)
					pCPnt->m_fPoint[nCutDir] += TFLOAT(dCoef * depthInc) ;
				else
					pCPnt->m_fPoint[nCutDir] -= TFLOAT(dCoef * depthInc) ;
			}
			else if(pCPnt->m_fPoint[3] > fMid)
			{
				ASSERT(fBnd2 - fMid > 1.0e-8);
				dCoef = (fBnd2 - pCPnt->m_fPoint[3])/(fBnd2 - fMid);
				if(!bReverse)
					pCPnt->m_fPoint[nCutDir] += TFLOAT(dCoef * depthInc) ;
				else
					pCPnt->m_fPoint[nCutDir] -= TFLOAT(dCoef * depthInc) ;
			}
			else
			{
				if(!bReverse)
					pCPnt->m_fPoint[nCutDir] += TFLOAT(depthInc) ;
				else
					pCPnt->m_fPoint[nCutDir] -= TFLOAT(depthInc) ;
			}
		}
	}
	if (nCutDir)
	{
		CutPath->TransPath(0.0f,TFLOAT(0.5 * depthInc), 0.0f );
	}
	else
	{
		CutPath->TransPath(TFLOAT(0.5 * depthInc),0.0f, 0.0f );	
	}
}

//生成类正弦波动 qqs 2013.05.20
void CFlowlineGen::CreateSinWave(CSmtCutPath * CutPath,DOUBLE depthInc,int nSect, int nCutDir)
{
	nCutDir = !nCutDir; 
	CSmtCutPoint* pCPnt =NULL;
	TFLOAT fBnd1 = 0.0, fBnd2 = 0.0;
	DOUBLE dCoef = 0.0 ;
	int k = 0 ;    //标记等分点序号
	BOOL bReverse = TRUE;//错峰标记
	for( k = 0 ; k < nSect ; k++)
	{
		fBnd1 = (TFLOAT)k/nSect ;
		fBnd2 = (TFLOAT)(k+1)/nSect ; 

		for( pCPnt = CutPath->m_pHead ; pCPnt ; pCPnt = pCPnt->next )
		{
			if(pCPnt->m_fPoint[3] <= fBnd1&&k !=0)
				continue ;
			if(pCPnt->m_fPoint[3] > fBnd2)
			{
				break ;
			}
			dCoef = (pCPnt->m_fPoint[3]-fBnd1)/(fBnd2-fBnd1);							

			if(!bReverse)//类正弦波形
			{	
				dCoef = sin(dCoef*PI2);
				DOUBLE h = (depthInc/2)*(dCoef-1);
				pCPnt->m_fPoint[nCutDir] += TFLOAT(h);				
			}
			else//类余弦波形
			{
				dCoef = cos(dCoef*PI2);
				DOUBLE h = depthInc*(1-dCoef)/2;
				pCPnt->m_fPoint[nCutDir] -= TFLOAT(h);								
			}
		}
	}	
	if (nCutDir)
	{
		CutPath->TransPath(0.0f,TFLOAT(0.5 * depthInc), 0.0f );
	}
	else
	{
		CutPath->TransPath(TFLOAT(0.5 * depthInc),0.0f, 0.0f );
	}
}

//生成梯形波动 qqs 2013.05.20
void CFlowlineGen::CreateTrapWave(CSmtCutPath * CutPath,DOUBLE depthInc,int nSect, int nCutDir)
{
	nCutDir = !nCutDir;
	CSmtCutPoint* pCPnt =NULL;
	TFLOAT fBnd1 = 0.0, fBnd2 = 0.0;	
	int k = 0 ;    //标记等分点序号
	BOOL bReverse = TRUE;//错峰标记

	CSmtCutPoint* pNewCPnt = NULL ;
	for( k = 0 ; k < nSect ; k++)
	{
		fBnd1 = (TFLOAT)k/nSect ;
		fBnd2 = (TFLOAT)(k+1)/nSect ;
		double eachSect = (fBnd2 - fBnd1)/4.;

		//取路径中需要出现在波谷的节点，将节点拷贝、移动到波谷，然后插入原来路径中
		for( pCPnt = CutPath->m_pHead ; pCPnt ; pCPnt = pCPnt->next )
		{
			if(fBnd1 - pCPnt->m_fPoint[3] >1e-8)
			{
				continue ;
			}
			if(pCPnt->m_fPoint[3] - fBnd2 >1e-8)
			{		
				break ;
			}
			else if(fabs(pCPnt->m_fPoint[3] - eachSect) <=1e-8)
			{//路径点从上往下运动到波谷
				pNewCPnt = pCPnt->CopyMyself();
				pNewCPnt->m_fPoint[nCutDir] -=TFLOAT(depthInc);
				CutPath->InsertAfter(pNewCPnt,pCPnt);
				pCPnt =pNewCPnt;
				bReverse = FALSE;
			}
			else if(fabs(pCPnt->m_fPoint[3] - 2.* eachSect) <=1e-8)
			{//路径点保持在梯形波的波谷
				pCPnt->m_fPoint[nCutDir] -=TFLOAT(depthInc);
			}
			else if(fabs(pCPnt->m_fPoint[3] - 3. * eachSect) <=1e-8)
			{//路径点从下往上运动波峰
				pNewCPnt = pCPnt->CopyMyself();
				pNewCPnt->m_fPoint[nCutDir] -=TFLOAT(depthInc);
				CutPath->InsertBefore(pNewCPnt,pCPnt);
			}
		}
	}

	if (nCutDir)
	{
		CutPath->TransPath(0.0f,TFLOAT(0.5 * depthInc), 0.0f );
	}
	else
	{
		CutPath->TransPath(TFLOAT(0.5 * depthInc),0.0f, 0.0f );
	}
}

//查找最近点,根据pnt1(某边退为点时可能用pnt2)和nCutCross在cSmtCPathLib中路径找到最近点的提取方法,
//并根据通过cSmtCPathLib后的末点为pnt1和pnt2赋值,以便搜索下一个面的.
//nNearestPnt[0]的0x0001表示路径反向，0x0002表示从cSmtCPathLib的头部提取
int CFlowlineGen::FindNearestPnt( PNT3D pnt1, PNT3D pnt2, CSmtCPathLib& cSmtCPathLib, int nCutCross )
{
	if( cSmtCPathLib.m_cAllPath.GetCount()==0 ) return -1;
	CSmtCutPath* pHead=cSmtCPathLib.m_cAllPath.GetHead();
	CSmtCutPath* pTail=cSmtCPathLib.m_cAllPath.GetTail();
	if( !pHead || !pTail ) return -1;

	int		nNearestPnt[4]={0, -1, -1, -1}, nNearestPntCnt=0;
	double	dMin, d[4];
    FPNT3D  fPnt1 , fPnt2;
    nc_DoubleToFloat( fPnt1, pnt1, 3 ) ;
    nc_DoubleToFloat( fPnt2, pnt2, 3 ) ;
	d[0]=mathFDist( fPnt1, pTail->m_pHead->m_fPoint );
	d[1]=mathFDist( fPnt1, pTail->m_pTail->m_fPoint );
	d[2]=mathFDist( fPnt1, pHead->m_pHead->m_fPoint );
	d[3]=mathFDist( fPnt1, pHead->m_pTail->m_fPoint );

	dMin=d[0];
	for( int i=1; i<4; i++ ) //先找到最近点
	{
		if( d[i]<dMin+NCSF_TOL_M4 )
			dMin=d[i]; 
	}

	for( i=0; i<4; i++ ) //再查看是否存在多个最近点
	{
		if( d[i]<dMin+NCSF_TOL_M4 )
			nNearestPnt[nNearestPntCnt++]=i;
	}

	if( nNearestPntCnt==2 ) //有等距点，需要使用pnt2辅助判断
	{
		int n=nNearestPnt[0]+nNearestPnt[1];
		if( n==1 ) //尾段缩为0，用首段判断
		{
			d[2]=mathFDist( fPnt2, pHead->m_pHead->m_fPoint );
			d[3]=mathFDist( fPnt2, pHead->m_pTail->m_fPoint );
			if( d[2]<d[3] ) nNearestPnt[0]=0;
			else nNearestPnt[0]=1;
		}
		else if( n==2 ) //首末段的起点重合
		{
			d[1]=mathFDist( fPnt2, pTail->m_pTail->m_fPoint );
			d[3]=mathFDist( fPnt2, pHead->m_pTail->m_fPoint );
			if( d[1]<d[3] ) nNearestPnt[0]=2;
			else nNearestPnt[0]=0;
		}
		else if( n==4 ) //首末段的终点重合
		{
			d[0]=mathFDist( fPnt2, pTail->m_pHead->m_fPoint );
			d[2]=mathFDist( fPnt2, pHead->m_pHead->m_fPoint );
			if( d[0]<d[2] ) nNearestPnt[0]=3;
			else nNearestPnt[0]=1;
		}
		else if( n==5 ) //首段首尾相连
		{
			d[0]=mathFDist( fPnt2, pTail->m_pHead->m_fPoint );
			d[1]=mathFDist( fPnt2, pTail->m_pTail->m_fPoint );
			if( d[0]<d[1] ) nNearestPnt[0]=2;
			else nNearestPnt[0]=3;
		}
		else ;//n==3首末线的倒向点相连，0-3,1-2不可能出现

		if( nCutCross!=0 ) nNearestPnt[0]=n-nNearestPnt[0];
	}

	//设置末点，即新的起点
	if( nCutCross==0 )
	{
		if( nNearestPnt[0]==0 )
		{
			nc_FloatToDouble( pnt1, pTail->m_pTail->m_fPoint,  3 );
			nc_FloatToDouble( pnt2, pHead->m_pTail->m_fPoint,  3 );
		}
		else if( nNearestPnt[0]==1 )
		{
			nc_FloatToDouble( pnt1, pTail->m_pHead->m_fPoint, 3 );
			nc_FloatToDouble( pnt2, pHead->m_pHead->m_fPoint, 3 );
		}
		else if( nNearestPnt[0]==2 )
		{
			nc_FloatToDouble( pnt1, pHead->m_pTail->m_fPoint,  3 );
			nc_FloatToDouble( pnt2, pTail->m_pTail->m_fPoint,  3 );
		}
		else
		{
			nc_FloatToDouble( pnt1, pHead->m_pHead->m_fPoint, 3 );
			nc_FloatToDouble( pnt2, pTail->m_pHead->m_fPoint, 3 );
		}
	}
	else
	{
		INT_PTR nCnt=cSmtCPathLib.m_cAllPath.GetCount()+(nNearestPnt[0]&0x0001);
		if( nNearestPnt[0]==0 || nNearestPnt[0]==1 ) //从队尾线起
		{
			if( nCnt%2 == 1 )
			{
				nc_FloatToDouble( pnt1, pHead->m_pTail->m_fPoint, 3 );
				nc_FloatToDouble( pnt2, pHead->m_pHead->m_fPoint, 3);
			}
			else
			{
				nc_FloatToDouble( pnt1, pHead->m_pHead->m_fPoint, 3);
				nc_FloatToDouble( pnt2, pHead->m_pTail->m_fPoint, 3 );
			}
		}
		else
		{
			if( nCnt%2 == 1 )
			{
				nc_FloatToDouble( pnt1, pTail->m_pTail->m_fPoint, 3);
				nc_FloatToDouble( pnt2, pTail->m_pHead->m_fPoint, 3);
			}
			else
			{
				nc_FloatToDouble( pnt1, pTail->m_pHead->m_fPoint, 3 );
				nc_FloatToDouble( pnt2, pTail->m_pTail->m_fPoint, 3 );
			}
		}
	}

	return nNearestPnt[0];
}

void CFlowlineGen::SetTakeOutOrder(C3DSurfAttArray& c3DSurfAttArr, int nCutCross)
{
	PNT3D	pnt1, pnt2;
	INT_PTR		nSurf=c3DSurfAttArr.GetSize();
	C3DSurfAttribute* pSurfAtt=c3DSurfAttArr[0];
	if( !pSurfAtt || !pSurfAtt->m_pBoundStart || !pSurfAtt->m_pBoundStart->m_pEdge
			|| !pSurfAtt->m_pBoundStart->m_pEdge->m_pCurve ) return;

	if( nSurf==1 )
	{
		pSurfAtt->m_bTakeOutFromHead=true;
		pSurfAtt->m_bPathReverse=false;
		return;
	}

	pSurfAtt->m_pBoundStart->m_pEdge->m_pCurve->GetEndPoint( pnt1, pnt2 );
	if( pnt2[2]>pnt1[2]+NCSF_TOL_M4 )
	{
		double d;
		d=pnt1[0]; pnt1[0]=pnt2[0]; pnt2[0]=d;
		d=pnt1[1]; pnt1[1]=pnt2[1]; pnt2[1]=d;
		d=pnt1[2]; pnt1[2]=pnt2[2]; pnt2[2]=d;
	}

	for( INT_PTR i=0; i<nSurf; i++ )
	{
		pSurfAtt=c3DSurfAttArr[i];
		if( !pSurfAtt ) continue;

		int	nNearestPnt=FindNearestPnt( pnt1, pnt2, pSurfAtt->m_cSmtCPathLib, nCutCross );
		pSurfAtt->m_bTakeOutFromHead=nNearestPnt&0x0002;
		pSurfAtt->m_bPathReverse=nNearestPnt&0x0001;
	}
}

void CFlowlineGen::OrderAllPath(C3DSurfAttArray& c3DSurfAttArr, int nCutCross, BOOL bZigZag, CSmtCPathLib& cSmtCPathLib )
{
	SetTakeOutOrder(c3DSurfAttArr, nCutCross);
	INT_PTR					nSurf=c3DSurfAttArr.GetSize();
	CSmtCutPath*		pCutPath;
	C3DSurfAttribute*	pSurfAtt;
	if( !c3DSurfAttArr[0] ) return;

	if( nCutCross==0 )//横切
	{
		INT_PTR		nCnt=c3DSurfAttArr[0]->m_cSmtCPathLib.m_cAllPath.GetCount();
		bool	bReverse=false;
		for( INT_PTR n=0; n<nCnt; n++)
		{
			for( INT_PTR i=0; i<nSurf; i++ )
			{
				if( bReverse )	pSurfAtt=c3DSurfAttArr[nSurf-1-i];
				else			pSurfAtt=c3DSurfAttArr[i];
				if( !pSurfAtt || pSurfAtt->m_cSmtCPathLib.m_cAllPath.GetCount()==0 ) continue;

				if( pSurfAtt->m_bTakeOutFromHead )
				{
					pCutPath=pSurfAtt->m_cSmtCPathLib.m_cAllPath.GetHead();
					pSurfAtt->m_cSmtCPathLib.m_cAllPath.RemoveHead();
				}
				else
				{
					pCutPath=pSurfAtt->m_cSmtCPathLib.m_cAllPath.GetTail();
					pSurfAtt->m_cSmtCPathLib.m_cAllPath.RemoveTail();
				}
				if( !pCutPath ) continue;

				if( pSurfAtt->m_bPathReverse ) pCutPath->ReverseDirect();
				if( bZigZag ) 
					pSurfAtt->m_bPathReverse=!pSurfAtt->m_bPathReverse;
				pCutPath->next=NULL;
				pCutPath->m_nLineNo= int( n );
				cSmtCPathLib.AddToTail( pCutPath );
			}
			if( bZigZag ) bReverse=!bReverse;
		}
	}
	else //纵切
	{
		int		n=0;
		for( int i=0; i<nSurf; i++ )
		{
			pSurfAtt=c3DSurfAttArr[i];
			if( !pSurfAtt ) continue;

			while( pSurfAtt->m_cSmtCPathLib.m_cAllPath.GetCount()>0 )
			{
				if( pSurfAtt->m_bTakeOutFromHead )
				{
					pCutPath=pSurfAtt->m_cSmtCPathLib.m_cAllPath.GetHead();
					pSurfAtt->m_cSmtCPathLib.m_cAllPath.RemoveHead();
				}
				else
				{
					pCutPath=pSurfAtt->m_cSmtCPathLib.m_cAllPath.GetTail();
					pSurfAtt->m_cSmtCPathLib.m_cAllPath.RemoveTail();
				}
				if( !pCutPath ) continue;

				if( pSurfAtt->m_bPathReverse ) 
					pCutPath->ReverseDirect();
				if( bZigZag ) 
					pSurfAtt->m_bPathReverse=!pSurfAtt->m_bPathReverse;
				pCutPath->next=NULL;
				pCutPath->m_nLineNo=n++;
				cSmtCPathLib.AddToTail( pCutPath );
			}
		}
	}
}

void CFlowlineGen::AdjustStartAt( int nCutDir, int nStartAt, CSmtCPathLib& cSmtCutPathLib )
{
	bool	bReverseLine=false, bReverseLineOrder=false;

	if( nCutDir==0 ) //U向加工
	{
		if( nStartAt==1 ) bReverseLine=true;
		else if( nStartAt==3 ) bReverseLineOrder=true;
		else bReverseLine=bReverseLineOrder=true;
	}
	else
	{
		if( nStartAt==1 ) bReverseLineOrder=true;
		else if( nStartAt==3 ) bReverseLine=true;
		else bReverseLine=bReverseLineOrder=true;
	}

	// 反向同行号路径
	if( bReverseLine ) 
		ReverseSameLineNoPath(cSmtCutPathLib.m_cAllPath);

	// 调换路径的顺序
	if( bReverseLineOrder ) 
		ReverseLinePathOrder(cSmtCutPathLib.m_cAllPath);
}

// 反向同行号的路径
void CFlowlineGen::ReverseSameLineNoPath(CSmtCPathList &PathList)
{
	// 反向路径时，同行号的要颠倒路径顺序
	CSmtCPathList TPathList;
	while(PathList.GetCount() > 0)
	{
		CSmtCPathList SubPathList;
		CSmtCutPath *pPath = PathList.GetHead();
		int nLineNo = pPath->m_nLineNo;
		while (PathList.GetCount() > 0)
		{
			pPath = PathList.GetHead();
			if (pPath->m_nLineNo != nLineNo)
				break;
			pPath->ReverseDirect();
			SubPathList.AddHead(pPath);
			PathList.RemoveHead();
		}
		TPathList.AddTail(&SubPathList);
	}
	PathList.AddTail(&TPathList);
}

// 调换路径的顺序
void CFlowlineGen::ReverseLinePathOrder(CSmtCPathList &PathList)
{
	if (PathList.IsEmpty())
		return;
	CSmtCPathList HeadList, TailList;
	while(PathList.GetCount() > 0)
    {
		// 得到头部同行号的路径
		CSmtCPathList SubHeadList;
		CSmtCutPath *pPath = PathList.GetHead();
		int nHeadLineNo = pPath->m_nLineNo;
		while (PathList.GetCount() > 0)
		{
			pPath = PathList.GetHead();
			if (pPath->m_nLineNo != nHeadLineNo)
				break;
			SubHeadList.AddTail(pPath);
			PathList.RemoveHead();
		}

		// 路径空时退出
		if (PathList.IsEmpty())
		{
			TailList.AddHead(&SubHeadList);
			break;
		}

		// 得到头部同行号的路径
		CSmtCPathList SubTailList;
		pPath = PathList.GetTail();
		int nTailLineNo = pPath->m_nLineNo;
		while (PathList.GetCount() > 0)
		{
			pPath = PathList.GetTail();
			if (pPath->m_nLineNo != nTailLineNo)
				break;
			SubTailList.AddHead(pPath);
			PathList.RemoveTail();
		}
        
		// 换行号
		SetLineNo(SubHeadList, nTailLineNo);
		SetLineNo(SubTailList, nHeadLineNo);
		// 加入路径组
		TailList.AddHead(&SubHeadList);
		HeadList.AddTail(&SubTailList);      
    }
	PathList.AddTail(&HeadList);
	PathList.AddTail(&TailList);
}

// 设置路径组行号
void CFlowlineGen::SetLineNo(CSmtCPathList &PathList, int nLineNo)
{
	POSITION pos = PathList.GetHeadPosition();
	while (pos)
	{
		CSmtCutPath *pPath = PathList.GetNext(pos);
		if (!pPath)
			continue;
		pPath->m_nLineNo = nLineNo;
	}
}

// 调整路径方向
void CFlowlineGen::ModifyPathDirection(CSmtCPathList &PathList)
{
	BOOL bReverse = FALSE;
	CSmtCPathList TPathList;
	while(PathList.GetCount() > 0)
	{
		CSmtCPathList SubPathList;
		CSmtCutPath *pPath = PathList.GetHead();
		int nLineNo = pPath->m_nLineNo;
		if (bReverse)
		{
			while (PathList.GetCount() > 0)
			{
				pPath = PathList.GetHead();
				if (pPath->m_nLineNo != nLineNo)
					break;
				pPath->ReverseDirect();
				pPath->m_bMoveFlag = 1;
				SubPathList.AddHead(pPath);
				PathList.RemoveHead();
			}
		}
		else
		{
			while (PathList.GetCount() > 0)
			{
				pPath = PathList.GetHead();
				if (pPath->m_nLineNo != nLineNo)
					break;
				SubPathList.AddTail(pPath);
				PathList.RemoveHead();
			}
		}
		TPathList.AddTail(&SubPathList);
		bReverse = !bReverse;
	}
	PathList.AddTail(&TPathList);
}

void CFlowlineGen::AdjustSpiralPath ( CSmtCPathLib &cSmtCutPathLib )
{
	if( cSmtCutPathLib.m_cAllPath.GetCount () < 2 ) return  ;
	POSITION pos, atpos ;
	CSmtCutPath *pHead = NULL, *pNext = NULL ;
	FPNT4D start, tmp, end ;
	int nLineNo = 0, nLayerNo = 0 ;
	pos = cSmtCutPathLib.m_cAllPath.GetHeadPosition () ;
	pHead = cSmtCutPathLib.m_cAllPath.GetNext ( pos ) ;
	mathFCpyPnt4D( pHead->m_pHead->m_fPoint, start ) ;
	BOOL bAddPath = FALSE, bAddLast = FALSE ;
	while( pos )
	{
		atpos = pos ;
		pNext = cSmtCutPathLib.m_cAllPath.GetNext ( pos ) ;

//		mathFCpyPnt( pHead->m_pTail->m_fPoint, pNext->m_pHead->m_fPoint ) ;
		if( pNext->m_nLineNo == pHead->m_nLineNo || pNext->m_nLayerNo < 0 || pHead->m_nLayerNo < 0 )
		{
			if( bAddLast )
			{
				// 丢失一条路径,添加
				CSmtCutPath *pPath = new CSmtCutPath() ;
				AddPathPointEx( pPath, start ) ;
				AddPathPointEx( pPath, end   ) ;
				pPath->m_nLayerNo = nLayerNo ;
				pPath->m_nLineNo  = nLineNo  ;
				cSmtCutPathLib.m_cAllPath.InsertBefore ( atpos, pPath ) ;
				bAddLast = FALSE ;
			}
			mathFCpyPnt( pNext->m_pHead->m_fPoint, start ) ;
		}	
		else 
		{
			mathFCpyPnt4D( pNext->m_pHead->m_fPoint, tmp ) ;
			mathFCpyPnt4D( pNext->m_pTail->m_fPoint, end ) ;
			mathFCpyPnt4D( start, pNext->m_pHead->m_fPoint ) ;
			mathFCpyPnt4D( tmp, start ) ;
			nLineNo = pNext->m_nLineNo ;
			nLayerNo = pNext->m_nLayerNo ;
			bAddLast = TRUE ;
			// 如果最后一条路径变为螺旋路径，则再加一条路径
			if (pNext == cSmtCutPathLib.m_cAllPath.GetTail())
				bAddPath = TRUE;
		}

		pHead = pNext ;
	}
	if (bAddPath)
	{
		start[3] = end[3] = 0 ;
		CSmtCutPath *pTail = new CSmtCutPath() ;
		AddPathPointEx( pTail, start ) ;
		AddPathPointEx( pTail, end   ) ;
		pTail->m_nLineNo = cSmtCutPathLib.m_cAllPath.GetTail()->m_nLineNo + 1;
		cSmtCutPathLib.AddToTail ( pTail ) ;
	}
}

void CFlowlineGen::AdjustSpiralCurve ( CSmartCurve &AllLine )
{
	CSmartSect *pSect ;
	CSmtCPathLib cSmtCPathLib;
	int nLineNo = 0;
	for( pSect = AllLine.GetHead() ; pSect ; pSect = pSect->next)
	{
		CSmartLine* pLine = (CSmartLine*) pSect ;

		CSmtCutPath* path = new CSmtCutPath;
		PNT3D ptSt = {0., 0., 0.};
		PNT3D ptEnd ={0., 0., 0.};
		for (int m = 0; m < 2; m++)
		{
			ptSt[m] = pLine->m_aPoint[0][m];
			ptEnd[m] = pLine->m_aPoint[1][m];
		}
		path->AddPoint(ptSt);
		path->AddPoint(ptEnd);
		// 更改层号和行号
		path->m_nLineNo = nLineNo;
		nLineNo++;
		path->m_nLayerNo = 0;
		cSmtCPathLib.AddToTail(path);

	}

	AllLine.ClearAll();
	AdjustSpiralPath( cSmtCPathLib ) ;
	CSmtCutPath* pPath = NULL;
	POSITION pos = cSmtCPathLib.m_cAllPath.GetHeadPosition();
	while(pos)
	{
		pPath = cSmtCPathLib.m_cAllPath.GetNext(pos);
		CSmtCutPoint* ptSt = pPath->m_pHead;
		while(ptSt && ptSt->next)
		{
			PNT2D st, end;
			st[0] = ptSt->m_fPoint[0];
			st[1] = ptSt->m_fPoint[1];
			end[0] = ptSt->next->m_fPoint[0];
			end[1] = ptSt->next->m_fPoint[1];
			CSmartLine* pLine = new CSmartLine( st, end ) ;		
			AllLine.AddSect( pLine ) ;
			ptSt = ptSt->next;
		}
	}
	cSmtCPathLib.ClearAllPath();
}


// 把路径组的层号改为原值0
void CFlowlineGen::ModifyLayerNo(CSmtCPathLib& PathLib)
{
	CSmtCutPath *pPath = NULL;
	POSITION pos = PathLib.m_cAllPath.GetHeadPosition();
	while(pos)
	{
		pPath = PathLib.m_cAllPath.GetNext(pos);
		if (NULL == pPath)
			continue;
		pPath->m_nLayerNo = 0;
	}
}

int CFlowlineGen::CreateOneSurfPath(CSmartTool* pMiller, RFRAME& cRFrame, CGeoTrmSurf* pSurf, JDNC_UWLINE& cParam, 
									int nCutDir, double dArcTol, CSmtCPathLib& cPathLib, JDNC_PRGDEF &PrgDef, BOOL &bSpiral)
{
	if(!pSurf) return 0;

	BOOL bOffDir = GetOffSetDir(pSurf, cRFrame, cParam);
	
	double uwStep[2], *dParam = NULL ;
    int uwCnt[2], nNum = 0, nUW = 0 ;
    pSurf->GetDiscreteStep( 0.002, uwStep[0], uwStep[1], uwCnt[0], uwCnt[1] ) ;

	BOOL bZigZag = FALSE;
	if( cParam.m_bUWLineFlag & NCDEF_UWLINE_ZIGZAG ) bZigZag = TRUE ;
	
	// 判断曲线是否闭合,如果闭合并且方向相同,则沿曲面加工
	CSmartPathGen  ncPathGen ;
	JDNC_TOL		cCurveTol = { 0.002, 10, 0, 0, 0, 0 } ;
	CSmartLoop* pLoop = ncPathGen.ExtractSurfLoop(pSurf, cCurveTol);
	if( !pLoop ) return 0 ;
	CalcSurfUWCut( pSurf, pLoop, cParam, uwStep, nUW, bZigZag, bSpiral ) ;

	// 然后计算路径间距	
	GenParamSerialByStep( pMiller, cRFrame, pSurf, cParam.m_dOverStep, 
						  nUW, dParam, nNum, bOffDir, dArcTol, cPathLib );
	//生成参数域上的路径
	CreateLinearNew( dParam, nNum, nUW, pLoop, cPathLib, cRFrame, dArcTol, pSurf , cParam);

	// 重设起点是将多条曲面流线路径进行起点重设，而磨削调整只对应一条曲面流线路径，顾不需重设
	if( cParam.m_nStartAt && !m_bGrandMdfy)//调整起点位置
		AdjustStartAt( nCutDir, cParam.m_nStartAt, cPathLib );

	// 调整路径方向
	if (bZigZag && !m_bGrandMdfy)
		ModifyPathDirection(cPathLib.m_cAllPath);

	if( bSpiral && !m_bGrandMdfy)
        AdjustSpiralPath( cPathLib ) ;
	ModifyLayerNo(cPathLib);		// 把路径组的层号改为原值0

	BOOL bRet = TRUE ;
	INT_PTR nPathNum = cPathLib.GetNumPath();
	if (nPathNum>=2 && MathCAM_IsSupportMultiThread())
	{
		LPVOID lpParam[NC_CFG_CPU_MAXNUM] = {NULL};
		COSPT_DATA ThreadData[NC_CFG_CPU_MAXNUM];
		JDNC_PRGDEF tmpPrg = PrgDef;
		tmpPrg.m_pPrgFunc = NULL ; 
		tmpPrg.m_pPosFunc = NULL ; 
		int nThreadNum = min(m_nCalcThreadNum, (int)nPathNum);
		for (int i=0; i<nThreadNum; i++)
		{
			ThreadData[i].nAtCore = i;
			ThreadData[i].nCoreNum = nThreadNum;
			ThreadData[i].dArcTol = dArcTol;
			ThreadData[i].bOffDir = bOffDir;
			ThreadData[i].pMiller = pMiller;
			ThreadData[i].cRFrame = &cRFrame;
			ThreadData[i].pSurf = pSurf;
			ThreadData[i].PrgDef = i==0 ? PrgDef : tmpPrg;
			ThreadData[i].PathLib = &cPathLib;			
			ThreadData[i].pFlowlineGen = this;
			lpParam[i] = &ThreadData[i];
		}
		bRet = MathCAM_ThreadMainFunc(MathCAM_CreateOneSurfPathSubProc, lpParam, nThreadNum);
	}
	else
	{
		bRet = CreateOneSurfPathSubProc(dArcTol, bOffDir, pMiller, cRFrame, pSurf, cPathLib, PrgDef, 0, 1);
	}

	if (!bRet)
	{
		cPathLib.ClearAllPath();
	}
	
	// 如果是螺旋连刀，合并相近的路径点
	if (bRet && bSpiral)
	{
		bRet = CombinePathPoint(cPathLib);
	}

	if( dParam ) delete dParam;
	delete pLoop;
	return bRet;
}

//按照距离对路径排序
BOOL CFlowlineGen::CreateOneSurfPathSubProc(double dArcTol, BOOL bOffDir, CSmartTool* pMiller, RFRAME& cRFrame, 
											CGeoTrmSurf* pSurf, CSmtCPathLib& cParamPathLib, JDNC_PRGDEF &PrgDef, int nAtCore, int nCoreNum)
{
	// 路径个数为零，直接退出
	int nNumLine = cParamPathLib.GetNumPath () ;
	if (nNumLine < 1)
	{
		return TRUE;
	}
	PrgDef.m_dIncStep = PrgDef.m_dTotalMove / nNumLine ;
	PrgDef.m_dLimitAt = 1.0 ;
	
	//将参数域上的路径映射到空间域上,如果bSpiral,并且首末端点距离过远,将两点合并
	CSmtCutPath *pCutPath = NULL ;
	POSITION	pos=cParamPathLib.m_cAllPath.GetHeadPosition();
	int nCounter = 0;	// 计数器
	while(pos)
	{
		pCutPath=cParamPathLib.m_cAllPath.GetNext(pos);
		MathCAM_MovePrgStep(PrgDef);
		if (nCounter++ % nCoreNum != nAtCore || !pCutPath )
		{
			continue;
		}
		pCutPath->DefineBox () ;
		if( !CutPathToSurf(pMiller, cRFrame, pCutPath, pSurf, bOffDir, dArcTol) )
		{	//竖直面或被遮挡
			return FALSE;
			break;
		}		
		if( PrgDef.m_pBrkFunc && PrgDef.m_pBrkFunc () )
		{
            if (m_pErrorType) *m_pErrorType = JDERROR_GENPATH_ABORT;
			return FALSE;
		}
	}
	
	return TRUE;
}

// 如果是螺旋连刀，合并相近的路径点
BOOL CFlowlineGen::CombinePathPoint(CSmtCPathLib& PathLib)
{
	FPNT3D start, end ;
	double dDist = 0., dStep = min( 0.1, m_dStep ) ;
	CSmtCutPath *pCutPath;
	POSITION pos = PathLib.m_cAllPath.GetHeadPosition();
	if (NULL == pos)
		return FALSE;
	pCutPath = PathLib.m_cAllPath.GetNext(pos);
	if (NULL == pCutPath)
		return TRUE;
	mathFCpyPnt( pCutPath->m_pTail->m_fPoint, end ) ;
    
	while(pos)
	{
		pCutPath = PathLib.m_cAllPath.GetNext(pos);
		if( !pCutPath )
			continue;
		// 为了避免在重边处映射的误差,将比较近的两个点并为一个
		mathFCpyPnt( pCutPath->m_pHead->m_fPoint, start ) ;
		dDist = mathFDist( start, end ) ;
		// 两点之间的距离比较近
		if( dDist < dStep && dDist > 1.0e-6 )
			mathFCpyPnt( end, pCutPath->m_pHead->m_fPoint ) ;
		mathFCpyPnt( pCutPath->m_pTail->m_fPoint, end ) ;
	}
	return TRUE;
}

int CFlowlineGen::CreateEachSurfPath(CSmartTool* pMiller, RFRAME& cRFrame, C3DSurfArray& AllSurf, JDNC_UWLINE& cParam, 
									 double dArcTol, CSmtCPathLib& cSmtCPathLib, int &nStartLayerNo, JDNC_PRGDEF &PrgDef, BOOL *arrbSpiral )
{
	INT_PTR				n=AllSurf.GetSize();
	CSmtCPathLib	cParamPathLib;
	int				nCutDir=cParam.m_nCutDir;
	
	// 对所有曲面排序
	SortAllSurfByDist( AllSurf, dArcTol * 3 ) ;
//	SurfCAM_SetNewStep(PATHGEN_STEP_ORGPATH);
	for(INT_PTR i=0; i<n; i++)
	{
		if ( SurfNC_IsAbort() ) 
		{
			cSmtCPathLib.ClearAllPath () ;
			if (m_pErrorType) *m_pErrorType = JDERROR_GENPATH_ABORT;
			return FALSE ;
		}

		CGeoSurf* pSurf=AllSurf[i];
		if(!pSurf || pSurf->GetType()!=OBJ3D_SURFACE_GEO)
			continue;

		BOOL bSpiral = FALSE;
		if( !CreateOneSurfPath(pMiller, cRFrame, (CGeoTrmSurf*) pSurf, cParam, 
							   nCutDir, dArcTol, cParamPathLib, PrgDef, bSpiral))
		{
			if( PrgDef.m_pBrkFunc && PrgDef.m_pBrkFunc () )
			{
				cParamPathLib.ClearAllPath () ;
				return FALSE ;
			}
			else
			{
				continue ;
			}
		}
		if(cParamPathLib.m_cAllPath.GetCount()==0)
			continue; //没有生成路径


		if (arrbSpiral)
			arrbSpiral[nStartLayerNo] = bSpiral;
		// 修改路径的层号，传出路径
		ModifyPathAndCpy(cParamPathLib, nStartLayerNo, cSmtCPathLib, FALSE);
	}
	return TRUE;
}

// 设置路径的层号和行号，并把路径导入新路径组中
void CFlowlineGen::ModifyPathAndCpy(CSmtCPathLib& OldPathLib, int &nStartLayerNo, CSmtCPathLib& NewPathLib, BOOL bModifyLineNo/* = TRUE*/)
{
	// 路径个数小于等于0，返回
	if (OldPathLib.m_cAllPath.GetCount() <= 0)
		return;
	CSmtCutPath *pCutPath = NULL ;
	POSITION pos = OldPathLib.m_cAllPath.GetHeadPosition() ;
	int nLineNo = 0;
	if (bModifyLineNo)
	{// 设置起始行号
		pCutPath = OldPathLib.m_cAllPath.GetHead();
		if (pCutPath != NULL && pCutPath->m_nLineNo >0)
			nLineNo = pCutPath->m_nLineNo;
	}

	while( pos )
	{
		pCutPath=OldPathLib.m_cAllPath.GetNext(pos);
		if( !pCutPath ) 
			continue ;
		pCutPath->m_nLayerNo = nStartLayerNo;
		if (bModifyLineNo)
		{
			pCutPath->m_nLineNo = nLineNo;
			nLineNo++;	// 行号加1
		}
		NewPathLib.AddToTail( pCutPath ) ;
	}	
	OldPathLib.m_cAllPath.RemoveAll();
	nStartLayerNo ++;	// 起始路径层号加1
}

void CFlowlineGen::SortAllSurfByDist ( C3DSurfArray &AllSurf, double dTol )
{
	// 默认从最左边开始加工
	C3DSurfArray tmpSurf ;
	BOX3D dBox, dMaxMinBox;//添加记录最高点坐标的变量  20140731 xh
	double x = 1.0e10, dDist = 1.0e10 ;
	PNT3D p1, p2 ;
	CGeoTrmSurf *pSurf = NULL, *pObj = NULL ;
	INT_PTR i, j, nSize = AllSurf.GetSize (), nFind = -1 ;
	if( nSize < 2 ) return ;
	pSurf = (CGeoTrmSurf *)AllSurf.GetAt ( 0 ) ;
	pSurf->UpdateBox ( &dMaxMinBox ) ;

	for( i = 1 ; i < nSize ; i++ )
	{
		pSurf = (CGeoTrmSurf *)AllSurf.GetAt ( i ) ;
		pSurf->UpdateBox ( &dBox ) ;
		BOOL bFind = FALSE;
		if ( NCDEF_SORTTYPE_ZFIRST == m_cSetupDef.m_cOrderDef.m_nSortType )
		{//按Z向由上到下排序  20140806 xh
			if (dBox.max[2] > dMaxMinBox.max[2] + dTol)
				bFind = TRUE;
			else if (fabs(dBox.max[2] - dMaxMinBox.max[2]) < dTol)
			{
				if (dBox.min[2] > dMaxMinBox.min[2] + dTol)
				{
					bFind = TRUE;
				}
				else if (fabs(dBox.min[2] - dMaxMinBox.min[2]) < dTol)
				{
					if (dBox.min[0] < dMaxMinBox.min[0] - dTol)		
						bFind = TRUE;
					else if (fabs(dBox.min[0] - dMaxMinBox.min[0]) < dTol)
					{
						if (dBox.max[0] < dMaxMinBox.max[0] - dTol)
							bFind = TRUE;
						else if(fabs(dBox.max[0] - dMaxMinBox.max[0]) < dTol)
						{
							if (dBox.min[1] < dMaxMinBox.min[1] - dTol)
								bFind = TRUE;
							else if (fabs(dBox.min[1] - dMaxMinBox.min[1]) < dTol)
							{
								if (dBox.max[1] < dMaxMinBox.max[1] - dTol)
									bFind = TRUE;
							}
						}
					}

				}
			}
		}
		else if ( NCDEF_SORTTYPE_YFIRST == m_cSetupDef.m_cOrderDef.m_nSortType )
		{//按Y向由小到大排序  20140806 xh
			if (dBox.min[1] < dMaxMinBox.min[1] - dTol)		
				bFind = TRUE;
			else if (fabs(dBox.min[1] - dMaxMinBox.min[1]) < dTol)
			{
				if (dBox.max[1] < dMaxMinBox.max[1] - dTol)
					bFind = TRUE;
				else if(fabs(dBox.max[1] - dMaxMinBox.max[1]) < dTol)
				{
					if (dBox.min[0] < dMaxMinBox.min[0] - dTol)
						bFind = TRUE;
					else if (fabs(dBox.min[0] - dMaxMinBox.min[0]) < dTol)
					{
						if (dBox.max[0] < dMaxMinBox.max[0] - dTol)
							bFind = TRUE;
						else if (fabs(dBox.max[0] - dMaxMinBox.max[0]) < dTol)
						{
							if (dBox.max[2] > dMaxMinBox.max[2] + dTol)
								bFind = TRUE;
							else if (fabs(dBox.max[2] - dMaxMinBox.max[2]) < dTol)
							{
								if (dBox.min[2] > dMaxMinBox.min[2] + dTol)
								{
									bFind = TRUE;
								}
						    }
						}
					}
				}
			}
		}
		else if ( NCDEF_SORTTYPE_XFIRST == m_cSetupDef.m_cOrderDef.m_nSortType )
		{//按X向由小到大排序  20140806 xh
			if (dBox.min[0] < dMaxMinBox.min[0] - dTol)		
				bFind = TRUE;
			else if (fabs(dBox.min[0] - dMaxMinBox.min[0]) < dTol)
			{
				if (dBox.max[0] < dMaxMinBox.max[0] - dTol)
					bFind = TRUE;
				else if(fabs(dBox.max[0] - dMaxMinBox.max[0]) < dTol)
				{
					if (dBox.min[1] < dMaxMinBox.min[1] - dTol)
						bFind = TRUE;
					else if (fabs(dBox.min[1] - dMaxMinBox.min[1]) < dTol)
					{
						if (dBox.max[1] < dMaxMinBox.max[1] - dTol)
							bFind = TRUE;
						else if (fabs(dBox.max[1] - dMaxMinBox.max[1]) < dTol)
						{
							if (dBox.max[2] > dMaxMinBox.max[2] + dTol)
								bFind = TRUE;
							else if (fabs(dBox.max[2] - dMaxMinBox.max[2]) < dTol)
							{
								if (dBox.min[2] > dMaxMinBox.min[2] + dTol)
								{
									bFind = TRUE;
								}
							}
						}
					}
				}
			}
		}
		if (bFind)
		{
			mathCpyPnt(dBox.min, dMaxMinBox.min);
			mathCpyPnt(dBox.max, dMaxMinBox.max);
			pObj = pSurf ;
			nFind = i ;
		}
	}
	if (NULL == pObj)
		return;
	tmpSurf.Add ( pObj ) ;
	AllSurf.RemoveAt ( nFind ) ;
	for( i = 0 ; i < 3 ; i++ )
	{
		p1[i] = ( pObj->m_dBox.max[i] + pObj->m_dBox.min[i] ) * 0.5 ;
	}
	//按照曲面包围盒中心的距离，从起始曲面开始排序
	while( AllSurf.GetSize () > 0 )
	{
		dDist = 1.0e10 ;
		nFind = -1 ;
		pObj = NULL ;
		nSize = AllSurf.GetSize () ;
		for( i = 0 ; i < nSize ; i++ )
		{
			pSurf = ( CGeoTrmSurf *)AllSurf.GetAt ( i ) ;
			for( j = 0 ; j < 3 ; j++ )
			{
				p2[j] = ( pSurf->m_dBox.max[j] + pSurf->m_dBox.min[j] ) * 0.5 ;
			}
			x = mathDist( p1, p2 ) ;
			if( x < dDist )
			{
				dDist = x ;
				pObj = pSurf ;
				nFind = i ;
			}
		}
		if( pObj && nFind >= 0 && nFind < nSize )
		{
			tmpSurf.Add ( pObj ) ;
			AllSurf.RemoveAt ( nFind ) ;
			for( j = 0 ; j < 3 ; j++ )
			{
				p1[j] = ( pObj->m_dBox.max[j] + pObj->m_dBox.min[j] ) * 0.5 ;
			}
		}
		else
		{
			break ;
		}
	}
	AllSurf.Append(tmpSurf);
}

// 得到偏移方向
BOOL CFlowlineGen::GetOffSetDir(CGeoTrmSurf* pSurf, RFRAME& cRFrame, JDNC_UWLINE& cParam)
{
	// m_nOffsetDir: 0, 正方向;1,负方向;2,自动.
	if( cParam.m_nOffsetDir == 0 ) // 正方向
	{// 方向不变
		return FALSE;
	}
	else if( cParam.m_nOffsetDir == 1 ) // 负方向
	{// 方向反向
		return TRUE;
	}
	else if( cParam.m_nOffsetDir == 2 ) // 自动
	{// 如果负的多，反向	
		if( MathCAM_GetSurfNormalDir( pSurf, cRFrame ) < 0 ) 
			return TRUE ;
		else
			return FALSE;
	}
	return FALSE;
}

////////////////////////////以下函数为Yul所加////////////////////////////////
//重新构造曲面的边界线
void CFlowlineGen::RebuildSurfBnd ( CGeoTrmSurf* pSurf ) 
{
	// 判断参数域线是否连接
	CTrmLoop *pLoop ;
	CTrmFin  *pBnd, *pNewBnd ;

	CGeoCurve   *pCurve ;
	CGeoPLine3d *pNewLine ;
//	int          nType  ;
//	int          num, i ;

	PNT3D        pos    ;
	VEC3D        predu, curdu ;
	
	PNT3D		 t1, t2  ;
	PNT3D        p1, p2  ;

	double		 dAngTol = 15./180. * PI1 ;
	// 返回环的最前面
	while( pSurf->m_pLoop->GetPrev () )
	{
		pSurf->m_pLoop = pSurf->m_pLoop->GetPrev () ;
	}

	// 曲线的参数域基本上都是折线类型，从参数域入手!
	for( pLoop = pSurf->m_pLoop ; pLoop ; pLoop = pLoop->GetNext () )
	{
		PTList ptList ; // 记录参数域的点
		BOOL bFst = TRUE ;
		for( pBnd = pLoop->m_pFinHead ; pBnd ; )
		{
			pCurve = pBnd->m_pPCurve ;
			
			pCurve->GetEndPoint( t1, t2 ) ;
			if( pBnd->m_pEdge && pBnd->m_pEdge->m_pCurve )
                pBnd->m_pEdge->m_pCurve->GetEndPoint( p1, p2 ) ;

//			pCurve->GetDerivative10( 0., pos, curdu ) ;
			GetLineTangent( pCurve, 0., pos, curdu ) ;

			// 当读入了多个曲线，并且切矢不连续
			if( !bFst && mathGetAngleUnit( predu, curdu ) > dAngTol )
			{
				pNewBnd = new CTrmFin() ;
				pNewLine = CreatePLine( ptList ) ;
				pNewBnd->m_pPCurve = pNewLine;

				AddBound( pBnd, pNewBnd, pLoop ) ;	
				
				pNewBnd->m_pNext = pBnd  ;
				pNewBnd->m_pLoop = pLoop ;
				pBnd->m_pPrev = pNewBnd  ;
				bFst = TRUE ;
			}
			
//			pCurve->GetDerivative10( 1., pos, predu ) ;
			GetLineTangent( pCurve, 1., pos, predu ) ;
			// 得到点串
			DiscreteCurve( pCurve, ptList, bFst ) ;
			bFst = FALSE ;
			// 删除读入点串的边界
			pBnd = DeleteBound( pBnd, pLoop ) ;
		}
		if( ptList.GetCount() > 0 )
		{
			pNewBnd = new CTrmFin() ;
			pNewLine = CreatePLine( ptList ) ;
			pNewBnd->m_pPCurve = pNewLine;

			AddBound( pBnd, pNewBnd, pLoop ) ;
			pNewBnd->m_pLoop   = pLoop  ;
		}
		// 最后察看首曲线和末曲线是否可以连接
		if( pLoop->m_pFinHead && pLoop->m_pFinHead->GetNext () )
			ConnectHeadTailCurve( pLoop ) ;
		int nEdge = 0 ;
		for( pBnd = pLoop->m_pFinHead ; pBnd ; pBnd = pBnd->GetNext () )
			nEdge++ ;

		CreateSurfEdge( pSurf, pLoop ) ;
	}
}
BOOL CFlowlineGen::IsGeoSurfClosed ( CGeoTrmSurf *pSurf, int &nUW )
{
	// 返回环的最前面,如果有多个边界环,返回空
	while( pSurf->m_pLoop->GetPrev () )
	{
		pSurf->m_pLoop = pSurf->m_pLoop->GetPrev () ;
	}
	if( pSurf->m_pLoop->GetNext () ) return FALSE ;
	// 判断参数域线是否连接
	CTrmFin  *pBnd[4] = { NULL, NULL, NULL, NULL }, *pHead = NULL ;

	pHead = pSurf->m_pLoop->m_pFinHead ;
	int i = 0 ;
	for( i = 0 ; pHead ; pHead = pHead->GetNext (), i++ ) 
	{
		if( i > 3 || !pHead ) return FALSE ;
		pBnd[i] = pHead ;
	}
	if( i < 3 ) return FALSE ;
	double dTol = m_cSetupDef.m_cTolDef.m_dArcTol * 3 ;
	BOOL bSameDir = FALSE, bRet = FALSE, bReverse = FALSE ;
	PNT3D start[2], end[2] ;
	pBnd[0]->m_pPCurve->GetEndPoint ( start[0], end[0] ) ;
	pBnd[1]->m_pPCurve->GetEndPoint ( start[1], end[1] ) ;
	VEC3D v1, v2, v3 = { 1, 0., 0. } ;
	mathGetVecUnit( start[0], end[0], v1 ) ;
	mathGetVecUnit( start[1], end[1], v2 ) ;
	double dAng1 = mathGetAcuteAngleUnit( v1, v3 ) ;
	double dAng2 = mathGetAcuteAngleUnit( v2, v3 ) ;
	if( dAng1 > dAng2 ) bReverse = TRUE ;
	if( JudgeTwoBoundOverlap( pBnd[0], pBnd[2], dTol, bSameDir ) )
	{
		bRet = TRUE ;
		if( bReverse )	nUW = 0 ;
		else			nUW = 1 ;
	}
	else if( JudgeTwoBoundOverlap( pBnd[1], pBnd[3], dTol, bSameDir ) )
	{
		bRet = TRUE ;
		if( bReverse )	nUW = 1 ;
		else			nUW = 0 ;
	}
	return bRet ;
}
void CFlowlineGen::GetLineTangent ( CGeoCurve* pCurve, double u, 
								    PNT3D pos, VEC3D du ) 
{
	int nType = pCurve->GetType() ;
	if( nType != OBJ3D_CURVE_PLINE2D )
	{
		pCurve->GetDerivative10( u, pos, du ) ;
		mathUniVec( du, MIN_LEN ) ;
	}
	else
	{
		CGeoPLine2d* pLine2d = ( CGeoPLine2d* )pCurve ;
		PNT2D start, end ;
		int n = pLine2d->m_num ;
		if( u < 1 )
		{
			start[0] = pLine2d->m_dPoint[0][0], start[1] = pLine2d->m_dPoint[0][1] ;
			end[0] = pLine2d->m_dPoint[1][0], end[1] = pLine2d->m_dPoint[1][1] ;
		}
		else
		{
			start[0] = pLine2d->m_dPoint[n-1][0], start[1] = pLine2d->m_dPoint[n-1][1] ;
			end[0] = pLine2d->m_dPoint[n][0], end[1] = pLine2d->m_dPoint[n][1] ;
		}
		du[0] = end[0] - start[0] ;
		du[1] = end[1] - start[1] ;
		du[2] = 0 ;
		mathUniVec( du, MIN_LEN ) ;
	}
}

void CFlowlineGen::DiscreteCurve ( CGeoCurve* pCurve, PTList& ptList, BOOL bFst )
{
	// 得到点串
	int nType = pCurve->GetType() ;
	int i, num, from ;

	if( nType != OBJ3D_CURVE_PLINE2D )
	{
		STRPT * strpt = pCurve->DiscreteToStrpt( 0.001 ) ;
		num = strpt->m_np ;
		if( bFst )
			AddPTListPt( strpt->m_ps[0], ptList ) ;
		for( i = 1 ; i < num ; i++ )
		{
			AddPTListPt( strpt->m_ps[i], ptList ) ;
		}
		delete strpt ;
	}
	else
	{
		CGeoPLine2d* pLine2d = ( CGeoPLine2d* )pCurve ;
		PNT3D pt ;
		num = pLine2d->m_num ;
		from = 1 ;
		if( bFst )
			from = 0 ;
		for( i = from ; i <= num ; i++ )
		{
			pt[0] = pLine2d->m_dPoint[i][0] ;
			pt[1] = pLine2d->m_dPoint[i][1] ;
			pt[2] = pLine2d->m_height ;
			AddPTListPt( pt, ptList ) ;
		}
	}
}
void CFlowlineGen::ConnectHeadTailCurve ( CTrmLoop* pLoop )
{
	CGeoPLine3d *pNewLine ;
	CGeoCurve   *pHead, *pTail ;
//	int          num, i ;
	CTrmFin     *pBnd ;
	PNT3D        pos    ;
	VEC3D        predu, curdu ;
	
	PNT3D        t1, t2, t3, t4 ;
	PTList ptList ; // 记录参数域的点
	pHead = pLoop->m_pFinHead->m_pPCurve ;
//	pHead->GetDerivative10( 0., pos, curdu ) ;
	GetLineTangent( pHead, 0., pos, curdu ) ;
	pHead->GetEndPoint( t1, t2 ) ;

	for( pBnd = pLoop->m_pFinHead ; pBnd->GetNext () ; pBnd = pBnd->GetNext () ) ;
			
	pTail = pBnd->m_pPCurve ;
//	pTail->GetDerivative10( 1, pos, predu ) ;
	GetLineTangent( pTail, 1., pos, predu ) ;
	pTail->GetEndPoint( t3, t4 ) ;

	// 当读入了多个曲线，并且切矢不连续
	if( mathGetAngleUnit( predu, curdu ) > 0.3 )
		return ;
	// 末线

	DiscreteCurve( pTail, ptList, TRUE ) ;
	pBnd = DeleteBound( pBnd, pLoop ) ;
	// 首线

	DiscreteCurve( pHead, ptList, FALSE ) ;
	delete pLoop->m_pFinHead->m_pPCurve ;
	
	pNewLine = CreatePLine( ptList ) ;
	pLoop->m_pFinHead->m_pPCurve = pNewLine ;

}
void CFlowlineGen::CreateSurfEdge ( CGeoTrmSurf* pSurf, CTrmLoop* pLoop )
{
	if( !pSurf  ) return ;

	CTrmFin * pFin = pLoop->m_pFinHead ;
	int nPoint ; 
	
	PNT5D *dBuff = new PNT5D[5002]  ;

	double dTol=0.001;
 
	while( pFin )
	{
		if(pFin->m_pEdge==NULL)
		{
			if( pFin->DiscreteBound( pSurf, dTol, dBuff, nPoint, 5000 ) )
			{ // 将边界剖分成折线段，保证满足设定的精度
				CGeoCurve* pSpace = NULL ;
				if( nPoint == 1 )
				{
					CGeoLine* pLine3d = new CGeoLine ;
					pLine3d->m_begin[0] = dBuff[0][0];
					pLine3d->m_begin[1] = dBuff[0][1];
					pLine3d->m_begin[2] = dBuff[0][2];

					pLine3d->m_end[0] = dBuff[0][0];
					pLine3d->m_end[1] = dBuff[0][1];
					pLine3d->m_end[2] = dBuff[0][2];
					pSpace = pLine3d ;
				}
				else if(nPoint == 2)
				{
					CGeoLine* pLine3d =new CGeoLine;
					pLine3d->m_begin[0] = dBuff[0][0];
					pLine3d->m_begin[1] = dBuff[0][1];
					pLine3d->m_begin[2] = dBuff[0][2];

					pLine3d->m_end[0] = dBuff[1][0];
					pLine3d->m_end[1] = dBuff[1][1];
					pLine3d->m_end[2] = dBuff[1][2];
					pSpace = pLine3d ;
				}
				else if(nPoint > 2)
				{
					CGeoPLine3d* pPLine=new CGeoPLine3d(nPoint-1);
					for(int i=0; i<nPoint; i++)
					{
						pPLine->m_dPoint[i][0]=dBuff[i][0];
						pPLine->m_dPoint[i][1]=dBuff[i][1];
						pPLine->m_dPoint[i][2]=dBuff[i][2];
					}
					pSpace = pPLine ;
				}

				pFin->m_pEdge= new CTrmEdge(pSpace) ;  // 对应的空间曲线
				pFin->m_pEdge->m_pRefer[0] = pSurf  ;  // 引用的曲面1
				pFin->m_pEdge->m_pRefer[1] = NULL   ;  // 引用的曲面2
				pFin->m_pEdge->m_pFins[0]  = pFin   ;  // 对应的参数边界1
				pFin->m_pEdge->m_pFins[1]  = NULL   ;  // 对应的参数边界2
			}
		}
		pFin = pFin->GetNext() ;
	}

	delete[] dBuff;
}

void CFlowlineGen::AddBound ( CTrmFin* pOldBnd, CTrmFin* pNewBnd, CTrmLoop* pLoop )
{
	if( !pNewBnd || !pLoop )
		return ;

	CTrmFin *pPreBnd ;
	if( pOldBnd == pLoop->m_pFinHead )
	{
		pLoop->m_pFinHead = pNewBnd ;
	}
	else if( pOldBnd )
	{
		pPreBnd = pOldBnd->GetPrev () ;
		pPreBnd->m_pNext = pNewBnd ;
		pNewBnd->m_pPrev = pPreBnd ;
	}
	else
	{
		pPreBnd = pLoop->m_pFinHead ;
		while( pPreBnd && pPreBnd->GetNext () )
		{
			pPreBnd = pPreBnd->GetNext () ;
		}
		pPreBnd->m_pNext = pNewBnd ;
		pNewBnd->m_pPrev = pPreBnd ;
	}
}

CTrmFin* CFlowlineGen::DeleteBound ( CTrmFin* pOldBnd, CTrmLoop* pLoop )
{
	if( !pOldBnd || !pLoop )
		return pOldBnd ;

	CTrmFin *pPreBnd ;
	if( pOldBnd == pLoop->m_pFinHead )
	{
		pPreBnd = pOldBnd->GetNext () ;
		delete pOldBnd ;
		pOldBnd = pPreBnd  ;
		if( pOldBnd )
			pOldBnd->m_pPrev = NULL ;
		pLoop->m_pFinHead = pOldBnd ;
	}
	else
	{
		pPreBnd = pOldBnd->GetPrev () ;
		pPreBnd->m_pNext = pOldBnd->GetNext () ;
		if( pOldBnd->GetNext() )
			pOldBnd->GetNext ()->m_pPrev = pPreBnd ;
		delete pOldBnd ;
		pOldBnd = pPreBnd->GetNext () ;
	}

	return pOldBnd ;
}

CGeoPLine3d* CFlowlineGen::CreatePLine ( PTList& ptList )
{
	INT_PTR n = ptList.GetCount () ;
	
	CGeoPLine3d* pLine = new CGeoPLine3d( (int)n-1 ) ;
	
	POSITION pos, atpos ;
	INT_PTR i = 0 ;
	pos = ptList.GetHeadPosition() ;
	while( pos )
	{
		atpos = pos ;
		PtSeg* seg = ptList.GetNext ( pos ) ;
		mathCpyPnt( seg->pnt, pLine->m_dPoint[i] ) ;
		i++ ;
		delete seg ;
		ptList.RemoveAt ( atpos ) ;
	}
	return pLine ;
}

void CFlowlineGen::AddPTListPt ( PNT3D pt, PTList& ptList )
{
	PtSeg* seg = new PtSeg ;
	mathCpyPnt( pt, seg->pnt ) ;
	ptList.AddTail ( seg ) ;
}


void CFlowlineGen::ChangePCurve(C3DSurfAttribute* pSurfAtt, bool bRestore)//找邻边
{
	if( !pSurfAtt ) return;

	if(bRestore)
	{
		if( pSurfAtt->m_pBoundStart) pSurfAtt->m_pPCurvStart=pSurfAtt->m_pBoundStart->m_pPCurve;
		if( pSurfAtt->m_pBoundEnd )  pSurfAtt->m_pPCurvEnd  =pSurfAtt->m_pBoundEnd->m_pPCurve;
	}
	else
	{
		if( !pSurfAtt->m_pSurf || !pSurfAtt->m_pSurf->m_pLoop || !pSurfAtt->m_pSurf->m_pLoop->m_pFinHead ) return;
		CTrmFin* pBnd=pSurfAtt->m_pSurf->m_pLoop->m_pFinHead;
		for(int j=0; j<4 && pBnd!=pSurfAtt->m_pBoundStart; j++ ) 
			pBnd=pBnd->GetNext();
		if( j==1 || j==2 ) //0->2 1->3
		{
			pSurfAtt->m_pBoundStart = pBnd->GetPrev () ;
			pSurfAtt->m_pBoundEnd   = pBnd->GetNext () ;
		}
		else if( j==0 ) //3->1
		{
			pBnd=pBnd->GetNext();
			pSurfAtt->m_pBoundEnd = pBnd ;
			pBnd=pBnd->GetNext(); 
			pBnd=pBnd->GetNext();
			pSurfAtt->m_pBoundStart = pBnd ;
		}
		else if( j==3 ) //2->0
		{
			pBnd=pBnd->GetPrev();
			pSurfAtt->m_pBoundStart = pBnd ;
			pBnd=pBnd->GetPrev();
			pBnd=pBnd->GetPrev();
			pSurfAtt->m_pBoundEnd   = pBnd ;
		}
		else ; //不应当出现这种情况

		//在共边线反向时需要更换两参数线的起始顺序????
		if( pSurfAtt->m_nSense== -1 )
		{
			CGeoCurve* pCurve=pSurfAtt->m_pPCurvStart;
			pSurfAtt->m_pPCurvStart=pSurfAtt->m_pPCurvEnd;
			pSurfAtt->m_pPCurvEnd=pCurve;
		}
		PNT3D p1, p2, p3, p4 ;
		if( pSurfAtt->m_pBoundStart->m_pEdge->m_pCurve )
            pSurfAtt->m_pBoundStart->m_pEdge->m_pCurve->GetEndPoint( p1, p2 ) ;
		if( pSurfAtt->m_pBoundEnd->m_pEdge->m_pCurve )
			pSurfAtt->m_pBoundEnd->m_pEdge->m_pCurve->GetEndPoint( p3, p4 ) ;
	}
}

int CFlowlineGen::IsSameBound(CTrmFin* Bnd1, CTrmFin* Bnd2)
{
	if( (!Bnd1) || (!Bnd2) ) return 0;
	PNT3D		startPt1, endPt1, startPt2, endPt2;
	int			nSameDir;

	CGeoTrmSurf *Surf1=NULL, *Surf2=NULL;
	if(Bnd1->m_pLoop) Surf1= Bnd1->m_pLoop->m_pSurf ;
	if(Bnd2->m_pLoop) Surf2 = Bnd2->m_pLoop->m_pSurf ;
    CGeoCurve *pCurve1 = Bnd1->m_pEdge->m_pCurve ;
    CGeoCurve *pCurve2 = Bnd2->m_pEdge->m_pCurve ;
	if( (!Surf1) || (!Surf2) || (!pCurve1) || (!pCurve2) ) return 0;

	pCurve1->GetEndPoint( startPt1, endPt1 ) ;
	pCurve2->GetEndPoint( startPt2, endPt2 ) ;

	if( mathDist(startPt1, startPt2)<1e-4 && mathDist(endPt1, endPt2)<1e-4 ) nSameDir=1;//两曲线同向
	else if( mathDist(startPt1, endPt2)<1e-4 && mathDist(startPt2, endPt1)<1e-4 ) nSameDir=-1;//两曲线反向
	else nSameDir=0;

	if(nSameDir!=0)
	{   //进一步判断曲线上的其他点是否重合
		double u;
		for(u=0.02; u<1; u+=0.02)
		{
			if( pCurve1->GetType () == OBJ3D_CURVE_POLYLINE )
				GetPoint( ( CGeoPLine3d *)pCurve1, u, 0., startPt1 ) ;
			else
                pCurve1->GetPoint(u, startPt1);

			if(nSameDir==1)
			{
				if( pCurve2->GetType () == OBJ3D_CURVE_POLYLINE )
					GetPoint( ( CGeoPLine3d *)pCurve2, u, 0., startPt2 ) ;
				else
                    pCurve2->GetPoint(u, startPt2);
			}
			else
			{
				if( pCurve2->GetType () == OBJ3D_CURVE_POLYLINE )
					GetPoint( ( CGeoPLine3d *)pCurve2, 1-u, 0., startPt2 ) ;
				else
                    pCurve2->GetPoint(1-u, startPt2);
			}

			if( mathDist(startPt1, startPt2)>1e-4 )	break;
		}

		if( fabs(u-1)>1e-6 ) nSameDir=0;//两曲线不重合
	}

	return nSameDir;
}
BOOL CFlowlineGen::SearchAdjacentBound( CGeoTrmSurf* pSurf1, CGeoTrmSurf* pSurf2, 
									    CAdjEdgeArray& cAdjEdgeArr, BOOL bSmooth )
{
	if( (!pSurf1) || (!pSurf2) || (!pSurf1->m_pLoop) || (!pSurf2->m_pLoop) ) 
		return FALSE ;
	CTrmFin	*pBnd1, *pBnd2;
	CGeoCurve	*pCurve1, *pCurve2;
	BOOL bOverLap = FALSE ;
	for(pBnd1=pSurf1->m_pLoop->m_pFinHead; pBnd1; pBnd1=pBnd1->GetNext())
	{
		pCurve1=pBnd1->m_pEdge->m_pCurve;
		if(!pCurve1) continue;

		for(pBnd2=pSurf2->m_pLoop->m_pFinHead; pBnd2; pBnd2=pBnd2->GetNext())
		{
			pCurve2=pBnd2->m_pEdge->m_pCurve;
			if(!pCurve2) continue;

			//判断pCurve1和pCurve2是否重合
			BOOL bSameDir = FALSE ;
		/*	if( JudgeTwoBoundOverlap( pBnd1, pBnd2, 0.01, bSameDir ) )
			{
				CAdjEdge* p3DEdge=new CAdjEdge(pCurve1, pSurf1, pSurf2, pBnd1, pBnd2, bSameDir );
				cAdjEdgeArr.Add(p3DEdge);
				return TRUE;
			}*/
			if( JudgeTwoBoundSmooth( pSurf1, pBnd1, pSurf2, pBnd2, 0.01, bSameDir, bSmooth ) )
			{
				CAdjEdge* p3DEdge=new CAdjEdge(pCurve1, pSurf1, pSurf2, pBnd1, pBnd2, bSameDir );
				cAdjEdgeArr.Add(p3DEdge);
				bOverLap = TRUE ;
			}
		}
	}
	return bOverLap ;
}

// 判断所有曲面是否存在光滑连接或位置连续曲面组合，存在是置到ParaArr中，把其他曲面置到otherSurf中
BOOL CFlowlineGen::AllSurfAdjacentConnect ( C3DSurfArray& AllSurf, 
											C3DSurfArray& otherSurf,
											CUWLParaArray& ParaArr, 
											BOOL bSmooth )
{
	SearchAllSurfBound(AllSurf, otherSurf, ParaArr, bSmooth);
	//根据m_cAdjEdgeArr信息对面排序,调整参数域的方向,并排序后的面置于m_cSurfAttArr中
	for (INT_PTR i=0; i<ParaArr.GetSize(); )
	{            
		BOOL bTemp = OrderAllSurf(ParaArr[i].m_Surfs, ParaArr[i].m_cAdjEdgeArr, ParaArr[i].m_cSurfAttArr);
		if (!bTemp)		// 不成功，删除此参数数据
		{
			otherSurf.Append(ParaArr[i].m_Surfs);
			ParaArr.RemoveAt(i);
		}
		else
			i++;
	}
	return ParaArr.GetSize() != 0;
}
//调整所选曲面的顺序，并判断曲面是否符合流线加工的条件
BOOL CFlowlineGen::AdjustAllSurfType( C3DSurfArray& AllSurf, CUWLParaArray& ParaArr )
{
	if( AllSurf.GetSize () < 2 )
		return FALSE ;
	CUWLParaArray SmhParaArr, CntParaArr;		// 光滑连接和位置连续的曲面流线参数

	C3DSurfArray TmpSurf;
	// 把曲面存入临时曲面中
	TmpSurf.Append(AllSurf);

	// 光滑连接判断
	BOOL bSmhRet = JudgeSmoothSurf(TmpSurf, SmhParaArr);
	if (bSmhRet)
	{
		AllSurf.RemoveAll();
		MoveUWLArrayData(SmhParaArr, ParaArr);	// 移动参数数据
		JudgeConnectSurf(TmpSurf, CntParaArr);	// 位置连续判断
		MoveUWLArrayData(CntParaArr, ParaArr);	// 移动参数数据
		if (TmpSurf.GetSize() != 0)	// 剩余面移入原始面集
			AllSurf.Append(TmpSurf);
	}
	else
	{
		BOOL bCntRet = JudgeConnectSurf(TmpSurf, CntParaArr);	// 位置连续判断
		if (bCntRet)
		{
			AllSurf.RemoveAll();
			MoveUWLArrayData(CntParaArr, ParaArr);	// 移动参数数据
			if (TmpSurf.GetSize() != 0) // 剩余面移入原始面集
				AllSurf.Append(TmpSurf);
		}
		else
			return FALSE;
	}
	return TRUE;
}

// 根据加工参数设置流线参数
void CFlowlineGen::SetUWLParaData(CUWLParaArray &ParaArr, JDNC_FUWLINE& UWLineCut)
{
	for (int i=0; i<ParaArr.GetSize(); i++)
	{
		CUWLineParam &UWLPara = ParaArr.GetAt(i);
		UWLPara.Set(UWLineCut.m_nCutDir, UWLineCut.m_nOffsetDir, UWLineCut.m_nStartAt);
	}
}

// 把曲面流线参数数组中的数据从一个移向另一个
BOOL CFlowlineGen::MoveUWLArrayData(CUWLParaArray &ParaFrm, CUWLParaArray &ParaTo)
{
	for (INT_PTR i=0; i<ParaFrm.GetSize(); i++)
	{
		CUWLineParam &UWLPara = ParaFrm.GetAt(i);
		CUWLineParam TmpPara;
		INT_PTR n = ParaTo.Add(TmpPara);
		UWLPara.MoveTo(ParaTo.GetAt(n));
	}
	return TRUE;
}

// 判断是否存在光滑的曲面，如果存在，把曲面的参数存入ParaArr中
BOOL CFlowlineGen::JudgeSmoothSurf(C3DSurfArray& AllSurf, CUWLParaArray& ParaArr)
{
	if( AllSurf.GetSize () < 2 )
		return FALSE;

	C3DSurfArray LeftSurf;
	// 判断曲面是否光滑连接
	BOOL bRet = AllSurfAdjacentConnect( AllSurf, LeftSurf, ParaArr, TRUE ) ;

	if(bRet)
	{
		for (int i=0; i<ParaArr.GetSize(); i++)
		{
			CUWLineParam &UWLPara = ParaArr.GetAt(i);
			UWLPara.m_IsAllQuad = IsAllSurfQuad(UWLPara.m_Surfs);
			UWLPara.m_bSmooth = TRUE;
		}
		
		AllSurf.RemoveAll();
		// 曲面还有剩余，移入原始曲面组
		if (LeftSurf.GetSize() != 0)
		{
			AllSurf.Append(LeftSurf);
			LeftSurf.RemoveAll();			
		}		
	}

	return bRet;
}

// 判断所有曲面的环是否都是四边形
BOOL CFlowlineGen::IsAllSurfQuad(C3DSurfArray& AllSurf)
{
	CGeoTrmSurf* pSurf;
	// 判断是否是四边形
	for(int j=0; j<AllSurf.GetSize(); j++ )
	{
		pSurf = ( CGeoTrmSurf* )AllSurf.GetAt(j);
		if( !IsQuadSurf( pSurf->m_pLoop ) )
			return FALSE;
	}
	return TRUE;
}

// 判断是否存在位置连续的曲面，如果存在，把曲面的参数存入ParaArr中
BOOL CFlowlineGen::JudgeConnectSurf(C3DSurfArray& AllSurf, CUWLParaArray& ParaArr)
{
	if( AllSurf.GetSize () < 2 )
		return FALSE;

	C3DSurfArray LeftSurf;
	// 判断曲面是否位置连续
	BOOL bRet = AllSurfAdjacentConnect( AllSurf, LeftSurf, ParaArr, FALSE ) ;

	if(bRet)
	{
		for (int i=0; i<ParaArr.GetSize(); i++)
		{
			CUWLineParam &UWLPara = ParaArr.GetAt(i);
			UWLPara.m_bSmooth = FALSE;
			UWLPara.m_IsAllQuad = FALSE;
		}
		AllSurf.RemoveAll();
		// 曲面还有剩余，移入原始曲面组
		if (LeftSurf.GetSize() != 0)
		{
			AllSurf.Append(LeftSurf);
			LeftSurf.RemoveAll();			
		}		
	}

	return bRet;
}

// 创建相邻曲面流线路径
BOOL CFlowlineGen::CreateAdjSurfUWPath(CSmtCPathLib &AllPath, RFRAME &cRFrame, JDNC_FUWLINE &UWLineCut, 
									   CUWLineParam &cUWLine, int &nStartLayerNo, JDNC_PRGDEF & PrgDef)
{
	if (cUWLine.m_cSurfAttArr.GetSize() <= 0)
		return FALSE;
	BOOL bRet = TRUE;
	// 判断曲面组是否闭合
	if( IsSurfArrayColsed( cUWLine.m_cSurfAttArr ) )
		cUWLine.m_IsClosed = TRUE  ;
	else
		cUWLine.m_IsClosed = FALSE ;

	// 参数调整之后，如果出现不合理的情况，返回FALSE
	if (!IsUWLineParamValid(cUWLine))
		return FALSE;
	// 按照曲面法矢的方向，调整曲面边界线的方向和参数域的方向
	AdjustAllSurfDir( cUWLine.m_cSurfAttArr, cRFrame, UWLineCut.m_nOffsetDir ) ;
	// 得到加工的法矢
	if( cUWLine.m_IsAllQuad )
	{
		// 第一种方法
		bRet = CreateUWLineByReConstruct( AllPath, cUWLine, UWLineCut, cRFrame, m_cSetupDef, nStartLayerNo, PrgDef ) ;
		// 第二种方法, 没有修改完
		//	CreateUWLineByOriginalLine ( cSmtCPathLib, cUWLine, UWLineCut, cRFrame, Setup ) ;
	}
	else
	{
		// 第三种生成路径的方法
		bRet = CreateUWLineByWDirection( AllPath, cUWLine, UWLineCut, cRFrame, m_cSetupDef, nStartLayerNo, PrgDef ) ;
	}
	if( bRet ) 
	{
		INT_PTR nSize = cUWLine.m_cSurfAttArr.GetSize ();
		int nType = -1 ;
		for(INT_PTR i = 0 ; i < nSize ; i++ )
		{
			if( i == 0 ) nType = 0 ;
			else if( i == nSize - 1 ) nType = 2 ;
			else nType = 1 ;
			if( cUWLine.m_cSurfAttArr[i]->m_bReverseSurf )
				ReverseAttSurf( cUWLine.m_cSurfAttArr[i], FALSE, nType ) ;
		}
	}
	return bRet;
}


void CFlowlineGen::AddAdjEdgeToArray ( CAdjEdgeArray& OldEdgeArr,
									   CAdjEdgeArray& NewEdgeArr )
{
	INT_PTR i, n = OldEdgeArr.GetSize () ;
	for( i = 0 ; i < n ; i++ )
	{
		CAdjEdge* pEdge = OldEdgeArr[i] ;
		NewEdgeArr.Add ( pEdge ) ;
	}
	OldEdgeArr.RemoveAll () ;
}
void CFlowlineGen::AddAttSurfToArray ( C3DSurfAttArray& OldSurfArr,
									   C3DSurfAttArray& NewSurfArr )
{
	INT_PTR i , n = OldSurfArr.GetSize () ;
	for( i = 0 ; i < n ; i++ )
	{
		C3DSurfAttribute* pSurf = OldSurfArr[i] ;
		NewSurfArr.Add ( pSurf ) ;
	}
	OldSurfArr.RemoveAll () ;
}
void CFlowlineGen::DeleteAdjEdgeArr ( CAdjEdgeArray& cAdjEdgeArr )
{
	INT_PTR i, n = cAdjEdgeArr.GetSize () ;
	for( i = 0 ; i < n ; i++ )
	{
		CAdjEdge* pEdge = cAdjEdgeArr[i] ;
		pEdge->m_pCurve = NULL ;
		delete pEdge ;
	}
	cAdjEdgeArr.RemoveAll () ;
}
void CFlowlineGen::DeleteAdjSurfArr ( C3DSurfAttArray& cSurfArr )
{
	INT_PTR i , n = cSurfArr.GetSize () ;
	for( i = 0 ; i < n ; i++ )
	{
		C3DSurfAttribute* pSurf = cSurfArr[i] ;
		delete pSurf ;
	}
	cSurfArr.RemoveAll () ;
}

// 搜索面的邻边
void CFlowlineGen::SearchAllSurfBound(C3DSurfArray& AllSurfArr, C3DSurfArray& LeftSurfArr, 
									  CUWLParaArray& ParaArr, BOOL bSmooth)
{
	INT_PTR			  i, j, n ;
	CGeoTrmSurf  *pSurf1, *pSurf2;
	C3DSurfArray TmpSurfArr;//包含原始曲面组所有曲面，慢慢删除有临面的面，最后剩余单独面
	C3DSurfArray tempSurfArr;//存储所有原始曲面，并将搜索过的没有下一个临面的曲面组从其中删除
	C3DSurfArray tempSurfArr1;//存储进行过临面搜索的面中有临面的面
	C3DSurfArray tempSurfArr2;//存储所有进行过临面搜索的面，其中包含没有临面的面
	TmpSurfArr.Append(AllSurfArr);
	tempSurfArr.Append(AllSurfArr);

	// 找共边 qqs 2013.01.31改
	n = TmpSurfArr.GetSize () ;
	for( i = 0 ; i < n-1 ; i++ )
	{
		pSurf1 = ( CGeoTrmSurf* )tempSurfArr[i] ;
		if (pSurf1 == NULL)	continue;

		//如果该面已经进行过临面搜索，则不再进行搜索
		int nCount = (int)tempSurfArr2.GetSize();
		for( int m=0; m < nCount; m++ )
		{
			CGeoTrmSurf *tpSurf = ( CGeoTrmSurf* )tempSurfArr2.GetAt(m);
			if (pSurf1 == tpSurf) continue;
		}
		tempSurfArr.SetAt(i,NULL);//删除已经进行过搜索的面
		tempSurfArr2.Add(pSurf1);
		BOOL bFlag = TRUE;
        while(bFlag)
		{
			for( j = 0 ; j < n ; j++ )
			{
				pSurf2 = ( CGeoTrmSurf* ) tempSurfArr[j] ;

				if (pSurf2 == NULL )
				{
					if(j < n-1) continue;//pSurf2还没有搜索到最后一个面，继续本轮搜索

					//本轮搜索结束，如果tempSurfArr1为空，则bFlag = FALSE，
					// 如果不为空，则令pSurf1等于tempSurfArr1中最后一个面，继续进pSurf2搜索
					ResetFirstSurf(tempSurfArr1, &pSurf1, bFlag);
					if (!bFlag) continue;
					break;
				}

				int nCount = (int)tempSurfArr2.GetSize();
				BOOL flag = FALSE;					
				//判断面2是否已经进行过临面搜索
				for (int m = 0; m < nCount; m++)
				{
					CGeoTrmSurf *tpSurf = ( CGeoTrmSurf* )tempSurfArr2.GetAt(m);
					if (pSurf2 != tpSurf) continue;
					flag = TRUE;
					break;
				}
				if (flag == TRUE)
				{
					if(j < n-1) continue;
					ResetFirstSurf(tempSurfArr1, &pSurf1, bFlag);
					if (!bFlag) continue;
					break;
				}

				CAdjEdgeArray AdjEdges;
				if( SearchAdjacentBound ( pSurf1, pSurf2, AdjEdges, bSmooth ) )  
				{
				
					// 增加邻面，得到邻面的最终位置
					int nIndex = SearchAndSetSurfArray(ParaArr, pSurf1, pSurf2);
					if (-1 == nIndex)
					{
						ParaArr.RemoveAll();
						LeftSurfArr.Append(AllSurfArr);
						return;
					}

					//将有临边的面从TmpSurfArr中删除，最终剩余面用于加入剩余面操作
					int tmpCount = (int)TmpSurfArr.GetSize();
					for (int m = 0; m<tmpCount; m++)
					{
						CGeoTrmSurf* tmpSurf = ( CGeoTrmSurf* )TmpSurfArr.GetAt(m);
						if (tmpSurf == pSurf1 || tmpSurf == pSurf2)
							TmpSurfArr.SetAt(m,NULL);
					}

					tempSurfArr.SetAt(j,NULL);//清空进行过临面搜索，且有临面的面（面2马上要付给面1，所以确定面2会进行里面搜索）
					tempSurfArr1.Add(pSurf1);//面1有临面，存储到tempSurfArr1中
					tempSurfArr2.Add(pSurf2);//面2要进行临面搜索，存储到tempSurfArr2中
					pSurf1 = pSurf2;//将面2存储到面1，用于下次循环对面2进行临面搜索
					CUWLineParam &UWLPara = ParaArr.GetAt(nIndex);
					UWLPara.m_cAdjEdgeArr.Append(AdjEdges);	// 增加邻边
					UWLPara.m_bSmooth = bSmooth;
					break;
				}
				else if (j == n-1)
				{
 					ResetFirstSurf(tempSurfArr1, &pSurf1, bFlag);
 					if (!bFlag) continue;
 					break; 					
				}
			}
		}		
	}

	// 加入到剩余面中
	for (i=0; i<TmpSurfArr.GetSize(); i++)
	{
		if (NULL == TmpSurfArr.GetAt(i))
			continue;
		LeftSurfArr.Add(TmpSurfArr.GetAt(i));
	}
}

// 用于找面的公共边，如果tempSurfArr为空，则将bFlag设为FALSE，进行下一个pSurf1搜索
// 如果tempSurfArr不为空，则将其最后一个面取出来，赋值给pSurf   qqs 2013.01.31
void CFlowlineGen::ResetFirstSurf(C3DSurfArray& tempSurfArr1, CGeoTrmSurf** pSurf1, BOOL& bFlag)
{
	//本轮最后一次搜索时，如果tempSurfArr1不为空，则取出tempSurfArr1中的最后一个面保存在pSurf1，进行下一轮比较
	int p = (int)tempSurfArr1.GetSize();
	if( p<1 ) 
	{
		bFlag = FALSE;
		return;
	}
	*pSurf1 = ( CGeoTrmSurf* )tempSurfArr1.GetAt(p-1);
	tempSurfArr1.RemoveAt(p-1);
	return;
}

// 搜索包含临面组合的数组，如果找到邻面中的一个面，则把两邻面添加到相应的位置，否则，在新的位置添加两邻面
int CFlowlineGen::SearchAndSetSurfArray(CUWLParaArray& ParaArr, CGeoSurf *pSurf1, CGeoSurf *pSurf2)
{
	// 容错判断
	if (NULL == pSurf1 || NULL == pSurf2)
		return -1;
	
	CIntArray Surf1Locs, Surf2Locs;
	for( int i=0; i<(int)ParaArr.GetSize(); i++)
	{
		CUWLineParam &UWLPara = ParaArr[i];
		if (UWLPara.FindSurf(pSurf1) != -1)
            Surf1Locs.Add(i);
		if (UWLPara.FindSurf(pSurf2) != -1)
			Surf2Locs.Add(i);		
	}

	INT_PTR nSize1 = Surf1Locs.GetSize(), nSize2 = Surf2Locs.GetSize();
	// 单个面不可能在面组的两个位置存在
	if (nSize1 > 1 || nSize2 > 1)
		return -1;
	// 两个面都在面组内
	if (nSize1 == 1 && nSize2 == 1)
	{
		// 两个面的位置一致，直接返回该位置
		if (Surf1Locs[0] == Surf2Locs[0])
			return Surf1Locs[0];
        // 两个面位置不一致，则把两个位置的面组合并为一个面组
		CUWLineParam &UWLPara1 = ParaArr[Surf1Locs[0]], &UWLPara2 = ParaArr[Surf2Locs[0]];
		if (Surf1Locs[0] < Surf2Locs[0])
		{
			UWLPara1.m_Surfs.Append(UWLPara2.m_Surfs);
			ParaArr.RemoveAt(Surf2Locs[0]);
			return Surf1Locs[0];
		}
		else
		{
			UWLPara2.m_Surfs.Append(UWLPara1.m_Surfs);
			ParaArr.RemoveAt(Surf1Locs[0]);
			return Surf2Locs[0];
		}
	}
    // 只有面1在面组内，把面2添加到相应位置
	if (nSize1 == 1)
	{
		CUWLineParam &UWLPara = ParaArr[Surf1Locs[0]];
		UWLPara.m_Surfs.Add(pSurf2);
		return Surf1Locs[0];
	}
	// 只有面2在面组内，把面1添加到相应位置
	if (nSize2 == 1)
	{
		CUWLineParam &UWLPara = ParaArr[Surf2Locs[0]];
		UWLPara.m_Surfs.Add(pSurf1);
		return Surf2Locs[0];
	}
	// 新增加一个曲面流线参数
	INT_PTR nIndex = ParaArr.GetSize();
	CUWLineParam UWLPara;
	ParaArr.Add(UWLPara);
	ParaArr[nIndex].m_Surfs.Add(pSurf1);
	ParaArr[nIndex].m_Surfs.Add(pSurf2);
	return int(nIndex);
}

BOOL CFlowlineGen::SortSurf ( C3DSurfArray& AllSurf, 
							  C3DSurfArray& QuadSurf )
{
	CGeoTrmSurf    *pSurf1 ;

	INT_PTR n = AllSurf.GetSize() ;
	for( INT_PTR i = 0 ; i < n ; i++ )
	{
		pSurf1 = ( CGeoTrmSurf* )AllSurf[i] ;
		if( !pSurf1 ) return FALSE ;// 有的面为空
		// 如果为非裁减面，返回
		if( pSurf1->GetType () == OBJ3D_SURFACE_GEO )
			return FALSE ;

		// 检查曲面是否为四边面
		if( IsQuadSurf( pSurf1->m_pLoop ) )
		{
			QuadSurf.Add ( pSurf1 ) ;
			AllSurf.RemoveAt ( i ) ;
		}
	}
	return TRUE ;
}

BOOL CFlowlineGen::IsQuadSurf ( CTrmLoop* pLoop )
{
	if( !pLoop ) return FALSE ;
	
	PNT3D start, end, pnt1, pnt2 ;
	// 得到参数边界
	CTrmFin  *pFin = pLoop->m_pFinHead ;
	// 察看边界曲线的空间线,得到第一条曲线的首末端点
	if( pFin->m_pEdge && pFin->m_pEdge->m_pCurve )
	{
		pFin->m_pEdge->m_pCurve->GetEndPoint( start, end ) ;
		pFin = pFin->GetNext () ;
	}
	else
		return FALSE ;

	int j = 0 ;
	for( j = 1; pFin ; j++ )
	{
		if( pFin->m_pEdge && pFin->m_pEdge->m_pCurve )
		{
			pFin->m_pEdge->m_pCurve->GetEndPoint( pnt1, pnt2 ) ;
			// 判断首末是否连接
			if( mathDist( end, pnt1   ) > 1.0e-4 &&
				mathDist( start, pnt2 ) > 1.0e-4 ) 
				return FALSE ;
		}

		pFin = pFin->GetNext();
		mathCpyPnt( pnt2, end ) ;
	}
	if(j == 4 && mathDist( start, pnt2 ) < 1.0e-4 ) 
		return TRUE ;
		
	return FALSE ;
}
//判断两段边界是否重合
BOOL CFlowlineGen::JudgeTwoBoundOverlap ( CTrmFin* pFin1, CTrmFin* pFin2,
										  double dTol, BOOL& bSameDir )
{
	PNT3D st1, ed1, md1, st2, ed2, md2, pt ;
	CGeoCurve *pCurve1, *pCurve2 ;
	double t, dist ;

	//step1: 判断有效性
	if( !pFin1 || !pFin2 || !pFin1->m_pEdge || !pFin2->m_pEdge )
		return FALSE ;
	pCurve1 = pFin1->m_pEdge->m_pCurve ;
	pCurve2 = pFin2->m_pEdge->m_pCurve ;
	if( !pCurve1 || !pCurve2 )
		return FALSE ;
	
	//step2: 粗判
	pCurve1->GetEndPoint( st1, ed1 ) ;
	pCurve1->GetPoint   ( 0.5, md1 ) ;
	pCurve2->GetEndPoint( st2, ed2 ) ;
	pCurve2->GetPoint   ( 0.5, md2 ) ;

	//1. 长度粗判
	double len1 = pCurve1->GetLength() ;
	double len2 = pCurve2->GetLength() ;
	if( len1 < MIN_LEN || len2 < MIN_LEN )
		return FALSE ;

	if( fabs( len1 - len2 ) > 10. )
		return FALSE ;

	//2. 包围盒粗判
	if( !mathChkBox3DInt(&pCurve1->m_dBox,&pCurve2->m_dBox,dTol) != IDINT)
		return FALSE;
	//3. 类型粗判
	int type1 = pCurve1->GetType() ;
	int type2 = pCurve2->GetType() ;
	if( ( type1 == OBJ3D_CURVE_LINE || type1 == OBJ3D_CURVE_POLYLINE || type1 == OBJ3D_CURVE_PLINE2D ) && type2 == OBJ3D_CURVE_ARC ) 
		return FALSE ;
	if( type1 == OBJ3D_CURVE_ARC && ( type2 == OBJ3D_CURVE_LINE || type2 == OBJ3D_CURVE_POLYLINE || type2 == OBJ3D_CURVE_PLINE2D ) )
		return FALSE ;

	//4. 闭合粗判
	BOOL bClose1 = pCurve1->IsClosed() ;
	BOOL bClose2 = pCurve2->IsClosed() ;
	// 如果一个闭合，一个不闭合
	if( bClose1 != bClose2 ) return FALSE ;
	// 两个都是闭合
	if( bClose1 && bClose2 )
	{
		if( mathDist( st1, st2 ) < dTol )
		{
			pCurve2->GetNearestPnt( md1, t, pt, dist ) ;
			if( dist > dTol ) return FALSE ;

			pCurve1->GetNearestPnt( md2, t, pt, dist ) ;
			if( dist > dTol ) return FALSE ;
			// 只判断首末点和中点,不完善
			return TRUE ;
		}	
		else
			return FALSE ;
	}

	//step5: 两个都不是闭合的
	int nRet, num1 = 0, num2 = 0 ;
	PNT2D param1[3], param2[3] ;
	// 曲线1到曲线2
	nRet = pCurve2->GetNearestPnt( st1, t, pt, dist ) ;
	if( nRet && dist < dTol && t <= 1.+MIN_LEN && t >-MIN_LEN )
	{
		param1[num1][0] = 0., param1[num1][1] = t ;
		num1++ ;
	}
	else
		return FALSE ;
	nRet = pCurve2->GetNearestPnt( md1, t, pt, dist ) ;
	if( nRet && dist < dTol && t <= 1.+MIN_LEN && t >-MIN_LEN )
	{
		param1[num1][0] = 0.5, param1[num1][1] = t ;
		num1++ ;
	}
	else
		return FALSE ;
	nRet = pCurve2->GetNearestPnt( ed1, t, pt, dist ) ;
	if( nRet && dist < dTol && t <= 1.+MIN_LEN && t >-MIN_LEN )
	{
		param1[num1][0] = 1., param1[num1][1] = t ;
		num1++ ;
	}

	// 曲线2到曲线1
	nRet = pCurve1->GetNearestPnt( st2, t, pt, dist ) ;
	if( nRet && dist < dTol && t <= 1.+MIN_LEN && t >-MIN_LEN )
	{
		param2[num2][0] = 0., param2[num2][1] = t ;
		num2++ ;
	}
	else
		return FALSE ;
	nRet = pCurve1->GetNearestPnt( md2, t, pt, dist ) ;
	if( nRet && dist < dTol && t <= 1.+MIN_LEN && t >-MIN_LEN )
	{
		param2[num2][0] = 0.5, param2[num2][1] = t ;
		num2++ ;
	}
	else
		return FALSE ;
	nRet = pCurve1->GetNearestPnt( ed2, t, pt, dist ) ;
	if( nRet && dist < dTol && t <= 1.+MIN_LEN && t >-MIN_LEN )
	{
		param2[num2][0] = 1., param2[num2][1] = t ;
		num2++ ;
	}
	else
		return FALSE ;
	if( num1 < 3 || num2 < 3 ) return FALSE ;

	// 判断两条边界线是否反向
	bSameDir = FALSE ;
	if( mathDist( st1, st2 ) < MIN_LEN )
		bSameDir = TRUE ;
	return TRUE ;
}

BOOL CFlowlineGen::JudgeTwoBoundSmooth ( CGeoTrmSurf* pSurf1, CTrmFin* pFin1,	
										 CGeoTrmSurf* pSurf2, CTrmFin* pFin2,
										 double dTol, BOOL& bSameDir, BOOL bSmooth )
{

	// 首先判断两个边界是否位置连续
	if( !JudgeTwoBoundOverlap( pFin1, pFin2, dTol, bSameDir ) )
		return FALSE ;
	// 位置连续
	if( !bSmooth )
		return TRUE ;
	// 法矢连续
	PNT3D stpa1, edpa1, stpa2, edpa2 ;
	PNT3D stpos1, edpos1, stpos2, edpos2 ;
	VEC3D stnor1, ednor1, stnor2, ednor2 ;

	CGeoCurve *pParaCur1, *pParaCur2, *pCurve1, *pCurve2 ;
	//step1: 得到边界的参数线和空间线及一些判断的参数
	pParaCur1 = pFin1->m_pPCurve ;
	pParaCur2 = pFin2->m_pPCurve ;
	pCurve1   = pFin1->m_pEdge->m_pCurve ;
	pCurve2   = pFin2->m_pEdge->m_pCurve ;
	// 参数线的首末uw参数
	pParaCur1->GetEndPoint( stpa1, edpa1 ) ;
	pParaCur2->GetEndPoint( stpa2, edpa2 ) ;

	// 得到首末端点法矢和中点法矢
	pSurf1->GetNormal ( stpa1[0], stpa1[1], stpos1, stnor1 ) ;
	pSurf1->GetNormal ( edpa1[0], edpa1[1], edpos1, ednor1 ) ;
	pSurf2->GetNormal ( stpa2[0], stpa2[1], stpos2, stnor2 ) ;
	pSurf2->GetNormal ( edpa2[0], edpa2[1], edpos2, ednor2 ) ;

	mathUniVec( stnor1, MIN_LEN ), mathUniVec( ednor1, MIN_LEN ) ;
	mathUniVec( stnor2, MIN_LEN ), mathUniVec( ednor2, MIN_LEN ) ;
	
	double dAngTol = 15./180. * PI1 ;
	//step2: 用得到的参数进行判断是否切矢连续

	if( IsEligibleVec( stnor1, stnor2, dAngTol ) &&
		IsEligibleVec( ednor1, ednor2, dAngTol ) )//&&IsEligibleVec( midnor1, midnor2, dAngTol )
		 
		return TRUE ;

	if( IsEligibleVec( stnor1, ednor2, dAngTol ) &&
		IsEligibleVec( ednor1, stnor2, dAngTol ) )//&&IsEligibleVec( midnor1, midnor2, dAngTol ) 
		return TRUE ;

	return FALSE ;
}
void CFlowlineGen::GetCurveMidNormal ( CGeoTrmSurf* pSurf, CTrmFin* pFin, VEC3D normal )
{
	PNT3D mid, near_p, t ;
	double o_dis ;
	pFin->m_pEdge->m_pCurve->GetPoint( 0.5, mid ) ;
	// 得到初始的叠代值
	pFin->m_pPCurve->GetPoint( 0.5, t ) ;
	// 得到中点距离曲面最近的点
	CGeoInter cInter ;
	cInter.IterateSurfNearestPnt( pSurf->m_pSurface, mid, t[0], 
			                      t[1], near_p, o_dis ) ;
	
	pSurf->GetNormal ( t[0], t[1], near_p, normal ) ;
}

BOOL CFlowlineGen::IsEligibleVec ( VEC3D u1, VEC3D u2, double Tol )
{
	mathUniVec( u1, MIN_LEN ) ;
	mathUniVec( u2, MIN_LEN ) ;
	double ang = mathGetAngleUnit( u1, u2 ) ;
	if( ang < Tol || fabs( ang - PI1 ) < Tol )
		return TRUE	 ;
	return FALSE ;
}
BOOL CFlowlineGen::IsSurfArrayColsed ( C3DSurfAttArray& cSurfArr )
{
	INT_PTR n = cSurfArr.GetSize () ;
	if( n < 1 ) 
		return FALSE ;
	CTrmFin *pFin1 = cSurfArr[0]->m_pBoundStart ;
	CTrmFin *pFin2 = cSurfArr[n-1]->m_pBoundEnd ;

	BOOL bSameDir ;
	BOOL bClosed = JudgeTwoBoundOverlap( pFin1, pFin2, 0.01, bSameDir ) ;
	return bClosed ;
}

void CFlowlineGen::GetValidSurf ( C3DSurfArray& AllSurf )
{
	CGeoTrmSurf* pSurf ;
	C3DSurfArray tmpSurf ;
	CPtrList shellSurf ;
	POSITION pos, atpos ;
	INT_PTR i, n = AllSurf.GetSize() ;
	int nType ;
	for( i = 0 ; i < n ; i++ )
	{
		pSurf =( CGeoTrmSurf* )AllSurf[i] ;
		//有的面不存在，出错返回
		if( !pSurf )
			continue ;
		nType = pSurf->GetType () ;
		//几何曲面
		if( nType == OBJ3D_SURFACE_GEO )
		{
			tmpSurf.Add ( pSurf ) ;
		}
		else if( nType == OBJ3D_SURFACE_SHELL )
		{
			CGeoShell *pShell = ( CGeoShell * )pSurf ;
			pShell->ExtractGeoSurf ( shellSurf ) ;
			pos = shellSurf.GetHeadPosition () ;
			while( pos )
			{
				atpos = pos ;
				pSurf = ( CGeoTrmSurf *)shellSurf.GetNext ( pos ) ;
				tmpSurf.Add ( pSurf ) ;
				shellSurf.RemoveAt ( atpos ) ;
			}
		}
	}
	AllSurf.RemoveAll () ;
	BOX3D box ;
	for( i = 0 ; i < tmpSurf.GetSize () ; i++ )
	{
		pSurf = ( CGeoTrmSurf* )tmpSurf[i] ;
		pSurf->UpdateBox ( &box ) ;
		AllSurf.Add ( pSurf ) ;
	}
	tmpSurf.RemoveAll () ;
}
CTrmLoop* CFlowlineGen::CopySurfLoop ( CGeoTrmSurf* pSurf )
{
	if( !pSurf ) return NULL ;

	CTrmLoop *pHeadLoop, *pTailLoop, *pNewLoop ;
	pHeadLoop = pTailLoop = pNewLoop = NULL ;
	
	CTrmLoop* pOldLoop = pSurf->m_pLoop ;
	while( pOldLoop )
	{
		// 复制环
		pNewLoop = new CTrmLoop() ;
		pNewLoop->m_pFinHead = NULL ;
		pNewLoop->m_pSurf    = pSurf ;
		pNewLoop->m_dBox2d[0][0] = pOldLoop->m_dBox2d[0][0] ; 
		pNewLoop->m_dBox2d[0][1] = pOldLoop->m_dBox2d[0][1] ;
		pNewLoop->m_dBox2d[1][0] = pOldLoop->m_dBox2d[1][0] ;
		pNewLoop->m_dBox2d[1][1] = pOldLoop->m_dBox2d[1][1] ;

		//复制环中的CTrmFin 边界曲线
		//BEGIN
		CTrmFin *pHeadBnd, *pTailBnd, *pNewBnd;
		pHeadBnd = pTailBnd=NULL;

		CTrmFin* pOldBound = pOldLoop->m_pFinHead;
		while (pOldBound)
		{
			pNewBnd = new CTrmFin;

			pNewBnd->m_nSense = pOldBound->m_nSense;
			if (pOldBound->m_pPCurve)
				pNewBnd->m_pPCurve = pOldBound->m_pPCurve->CopyMyself();
			pNewBnd->m_pEdge    = NULL  ;  // 边界的曲线
			pNewBnd->m_pLoop    = pNewLoop;
		
			//Copy edge of this bound
			CTrmEdge* pOldEdge = pOldBound->m_pEdge;
			if (pOldEdge)
			{
				CTrmEdge* pnewEdge  = new CTrmEdge;
				if (pOldEdge->m_pCurve)
					pnewEdge->m_pCurve = pOldEdge->m_pCurve->CopyMyself() ;  // 对应的曲线
				pnewEdge->m_pRefer[0] = pSurf;    // 引用的曲面
				pnewEdge->m_pRefer[1] = NULL;     // 引用的曲面
				pnewEdge->m_pFins[0] = pNewBnd;  // 对应的参数边界
				pnewEdge->m_pFins[1] = NULL;  // 对应的参数边界

				pNewBnd->m_pEdge      = pnewEdge;
			}

			if (pHeadBnd==NULL) 
			{
				pHeadBnd = pTailBnd = pNewBnd;
			}
			else
			{
				pNewBnd->m_pPrev = pTailBnd;
				pTailBnd->m_pNext = pNewBnd;
				pTailBnd = pNewBnd;
			}

			pOldBound = pOldBound->GetNext();
		}
		pNewLoop->m_pFinHead   = pHeadBnd;  // 边界曲线
		//END 复制环中的CTrmFin
			
		if (pHeadLoop==NULL) 
		{
			pHeadLoop = pTailLoop = pNewLoop;
		}
		else			  
		{
			pTailLoop->m_pNext = pNewLoop;
			pNewLoop->m_pPrev = pTailLoop;
			pTailLoop = pNewLoop;
		}

		pOldLoop = pOldLoop->GetNext();
	}
	return pNewLoop ;
}
void CFlowlineGen::CreateStitchBnd ( C3DSurfArray &AllSurf, C3DSurfArray &tmpSurf, CSurfLoopArr &cLoopArr )
{
	INT_PTR i, nSize = AllSurf.GetSize () ;
	CGeoTrmSurf *pSurf = NULL ;
	CTrmLoop *pLoop = NULL ;
	CSurfLoop *pSurfLoop = NULL ;
	// 复制曲面的边界
    for( i = 0 ; i < nSize ; i++ )
	{
		pSurf = ( CGeoTrmSurf*)AllSurf[i] ;
		pLoop = CopySurfLoop( pSurf ) ;
		pSurfLoop = new CSurfLoop( pSurf, pSurf->m_pLoop ) ;
		cLoopArr.Add ( pSurfLoop ) ;
		tmpSurf.Add ( pSurf ) ;
		pSurf->m_pLoop = pLoop ;
	}
}
void CFlowlineGen::CopyAllSurf ( C3DSurfArray &AllSurf, C3DSurfArray &tmpSurf )
{
	INT_PTR i, nSize = AllSurf.GetSize () ;
	CGeoTrmSurf *pSurf = NULL, *pCopy = NULL ;

	// 复制曲面
    for( i = 0 ; i < nSize ; i++ )
	{
		pSurf = ( CGeoTrmSurf*)AllSurf[i] ;
		pCopy = ( CGeoTrmSurf *)pSurf->CopyMyself () ;
		tmpSurf.Add ( pCopy ) ;
	}
}
void CFlowlineGen::DestroyAllSurf ( C3DSurfArray &AllSurf, CUWLParaArray& ParaArr )
{
	CGeoTrmSurf *pSurf = NULL ;
	INT_PTR i = 0 , nSize = AllSurf.GetSize () ;
	for( i = 0 ; i < nSize ; i++ )
	{
		pSurf = ( CGeoTrmSurf *)AllSurf.GetAt ( i ) ;
		if (pSurf)
			delete pSurf ;
	}
	AllSurf.RemoveAll () ;

	C3DSurfAttribute *pSurfAtt = NULL ;
	nSize = ParaArr.GetSize () ;
	for( i = 0 ; i < nSize ; i++ )
	{
		for (int j=0; j<ParaArr.GetAt(i).m_cSurfAttArr.GetSize(); j++)
		{
			pSurfAtt = ParaArr.GetAt(i).m_cSurfAttArr.GetAt(j);
			if (pSurfAtt != NULL && pSurfAtt->m_pSurf != NULL)
			{
				delete pSurfAtt->m_pSurf;
				pSurfAtt->m_pSurf = NULL;
			}			
		}
 	}
}
void CFlowlineGen::DestroyBnd ( C3DSurfArray &AllSurf, CSurfLoopArr &cLoopArr )
{
	INT_PTR i, nSize = AllSurf.GetSize () ;
	CGeoTrmSurf *pSurf = NULL ;
	CSurfLoop *pSurfLoop = NULL ;
	// 换回曲面的边界
	for( i = 0 ; i < nSize ; i++ )
	{
		pSurf = ( CGeoTrmSurf*)AllSurf[i] ;
		pSurfLoop = cLoopArr[i] ;
		DeleteTrmLoop( pSurf->m_pLoop ) ;
		pSurf->m_pLoop = pSurfLoop->m_pLoop ;
		delete pSurfLoop ;
	}
}
void CFlowlineGen::ResetAllSurf ( C3DSurfArray& AllSurf )
{	
	CGeoTrmSurf* pSurf ;
	INT_PTR i, nSize = AllSurf.GetSize () ;
	for( i = 0 ; i < nSize ; i++ )
	{
		pSurf = ( CGeoTrmSurf* )AllSurf[i] ;
		RebuildSurfBnd( pSurf ) ;

		if( pSurf->m_pLoop->CalcLoopArea() < 0 )
			pSurf->m_pLoop->ReverseLoop () ;
		CTrmFin* pBnd = pSurf->m_pLoop->m_pFinHead ;
		for( ; pBnd ; pBnd = pBnd->GetNext () )
		{
			if( pBnd->m_nSense == -1 )
			{
				pBnd->ReverseBound () ;
				pBnd->m_nSense = 1 ;
			}
		}
	}
}

void CFlowlineGen::GetSurfNormal ( C3DSurfAttArray& cSurfArr, 
								   RFRAME& cRFrame, 
								   int nOffsetDir )
{
	CGeoTrmSurf* pSurf = NULL ;
	INT_PTR i ;
	INT_PTR nSize = cSurfArr.GetSize() ;
	for( i = 0 ; i < nSize ; i++ )
	{
		pSurf = cSurfArr[i]->m_pSurf ;
		// 得到法矢的方向(+1, -1, 0 )
		int nSameNormal = MathCAM_GetSurfNormalDir( pSurf, cRFrame ) ;

		if( nSameNormal == 0 )
		{
			// 法矢为0的时候加工，要用手工将法矢的方向调为向内或者向外
		}
		else if( nSameNormal == 1 ) // 法矢向上
		{
			 // 正向和自由方向不动,负向-1
			if( nOffsetDir == 1 ) 
				nSameNormal = -1 ;
			else if( nOffsetDir == 0 )
				nSameNormal = 1 ;
		}
		else					   // 法矢向下
		{
			// 负向和自由方向不动,-1
			if( nOffsetDir == 0 )
				nSameNormal = -1 ;
			else if( nOffsetDir == 1 )
				nSameNormal = 1 ;
		}

		cSurfArr[i]->m_nSameNormal = nSameNormal ;
	}
}

// 利用曲面的pBoundEnd线的末端点来得到该点的切矢,并判断曲面的法矢是否相同
void CFlowlineGen::AdjustAllSurfDir ( C3DSurfAttArray& cSurfArr, RFRAME& cRFrame, int nOffsetDir )
{
	// nOffsetDir: 0, 正法矢;1, 负法矢; 2, 自动
	INT_PTR nSize = cSurfArr.GetSize (), i = 0 ;
	int nNormal, nType = -1 ;
	VEC3D nor1, nor2 ;
	double ang ;
	C3DSurfAttribute *pSurf ;
	for( i = 0 ; i < nSize ; i++ )
	{
		if( i == 0 ) nType = 0 ;
		else if( i == nSize - 1 ) nType = 2 ;
		else		nType = 1 ;
		pSurf = cSurfArr[i] ;
		pSurf->m_bReverseSurf = FALSE ;
		nNormal  = MathCAM_GetSurfNormalDir( pSurf->m_pSurf, cRFrame ) ;
		// 正法矢:方向不变;负法矢:方向反向;自动:如果法矢-1,反向
		if( nNormal > 0 )
		{
			if( nOffsetDir == 1 )
				ReverseAttSurf( pSurf, TRUE, nType ) ;
		}
		else if( nNormal < 0 )
		{
			if( nOffsetDir == 0 || nOffsetDir == 2 )
				ReverseAttSurf( pSurf, TRUE , nType ) ;
		}
		else
		{
/*			if( nOffsetDir == 1 )
				ReverseAttSurf( pSurf, TRUE ) ;*/
			if( i == 0 )
			{
				if( nOffsetDir == 1 )
					ReverseAttSurf( pSurf, TRUE, nType ) ;
			}
			else
			{
				GetAttSurfSameLinePntNor( cSurfArr[i-1], FALSE, nor1 ) ;
				GetAttSurfSameLinePntNor( pSurf, TRUE, nor2 ) ;
				ang = mathGetAngle( nor1, nor2, MIN_LEN ) ;
				if( ang > PI1_2 ) 
					ReverseAttSurf( pSurf, TRUE, nType ) ;
			}
		}
	}
}

// 判断曲面流线参数是否合理
BOOL CFlowlineGen::IsUWLineParamValid(const CUWLineParam &cUWLine)
{
	VEC3D vec1, vec2;
	PNT3D pos1, pos2;
	// 参数线
	CGeoCurve *pPara1 = NULL, *pPara2 = NULL ;
	// 空间线
	CGeoCurve *pCurve1 = NULL, *pCurve2 = NULL ;
	C3DSurfAttribute* pSurfAtt;

	double dStep = m_dStep;//防止路径间距过大时，出现路径断开情况
	if (m_bGrandMdfy)
	{
		dStep = 0.02;
	}
	double dMinDis = min((double)m_pMiller->m_fRadius, dStep);
	for(int i=0; i < cUWLine.m_cSurfAttArr.GetSize(); i++ )
	{
		pSurfAtt = cUWLine.m_cSurfAttArr[i];
		pPara1 = pSurfAtt->m_pBoundStart->m_pPCurve;
		pPara2 = pSurfAtt->m_pBoundEnd->m_pPCurve;
		pCurve1 = pSurfAtt->m_pBoundStart->m_pEdge->m_pCurve ;
		pCurve2 = pSurfAtt->m_pBoundEnd->m_pEdge->m_pCurve ;
		// 如果曲线参数长度比较小，则不合理
		if (pPara1->GetLength() < MIN_DIS || pPara2->GetLength() < MIN_DIS)
			return FALSE;

		pPara1->GetTangent( 0., pos1, vec1 ) ;
		pPara2->GetTangent( 0., pos2, vec2 ) ;
		// 如果两向量方向相同或无大偏差，则不合理
		if (mathOProduct(vec1, vec2) > -MIN_DBL)
			return FALSE;
		pPara1->GetTangent( 1., pos1, vec1 ) ;
		pPara2->GetTangent( 1., pos2, vec2 ) ;
		// 如果两向量方向相同或无大偏差，则不合理
		if (mathOProduct(vec1, vec2) > -MIN_DBL)
			return FALSE;

		// 如果曲线实际长度小于间距或刀具半径，则不合理
		PNT3D Start, End;
		if( pCurve1 )
		{
			if( pCurve1->GetLength () < dMinDis)
				return FALSE ;
		}
		else if( !pPara1->IsClosed () )
		{
			pPara1->GetEndPoint(Start, End);
			pSurfAtt->m_pSurf->GetPoint(Start[0], Start[1], Start);
			pSurfAtt->m_pSurf->GetPoint(End[0], End[1], End);
			// 如果曲线实际长度小于间距或刀具半径，则不合理
			if (mathDist(Start, End) < dMinDis)
				return FALSE;
		}
		if( pCurve2)
		{
			if( pCurve2->GetLength () < dMinDis)
				return FALSE ;
		}
		else if( !pPara2->IsClosed ())
		{
			pPara2->GetEndPoint(Start, End);
			pSurfAtt->m_pSurf->GetPoint(Start[0], Start[1], Start);
			pSurfAtt->m_pSurf->GetPoint(End[0], End[1], End);
			if (mathDist(Start, End) < dMinDis)
				return FALSE;
		}
	}
	return TRUE;
}

void CFlowlineGen::GetAttSurfNormal ( C3DSurfAttribute* pSurfAtt, BOOL bStart, VEC3D normal )
{
	CGeoCurve *pCur1, *pCur2 ;
	PNT3D start, end ;
	VEC3D vec1, vec2 ;

	if( bStart )
	{
		pCur2 = pSurfAtt->m_pBoundStart->m_pEdge->m_pCurve ;
		pCur1 = GetPrevCurve( pSurfAtt, pCur2 ) ;
	}
	else
	{
		pCur1 = pSurfAtt->m_pBoundEnd->m_pEdge->m_pCurve ;
		pCur2 = GetNextCurve( pSurfAtt, pCur1 ) ;
	}
	PNT3D p1, p2 , p3, p4 ;
	pCur1->GetEndPoint( p1, p2 ) ;
	pCur2->GetEndPoint( p3, p4 ) ;

	// 得到第一条曲线末点的切矢
//	pCur1->GetDerivative10( 1.0, end, vec1  ) ;
	GetLineTangent( pCur1, 1., end, vec1 ) ;
	// 得到第二条曲线首点的切矢
//	pCur2->GetDerivative10( 0., start, vec2 ) ;
	GetLineTangent( pCur2, 0., start, vec2 ) ;
	nc_VProduct( vec1, vec2, normal ) ;
}
void CFlowlineGen::GetAttSurfSameLinePntNor ( C3DSurfAttribute* pSurfAtt, BOOL bStart, VEC3D nor )
{
	CGeoCurve *pCur ;
	PNT3D start, end, near_pt ;
	
	if( bStart )
	{
		pCur = pSurfAtt->m_pBoundStart->m_pEdge->m_pCurve ;
	}
	else
	{
		pCur = pSurfAtt->m_pBoundEnd->m_pEdge->m_pCurve ;
	}
	pCur->GetEndPoint( start, end ) ;
	
	double u = 0., w = 0., dis = 1.0e10 ;

	pSurfAtt->m_pSurf->GetNearestPt ( start, u, w, near_pt, dis ) ;
	pSurfAtt->m_pSurf->GetNormal ( u, w, start, nor ) ;

}
CGeoCurve* CFlowlineGen::GetNextCurve ( C3DSurfAttribute* pSurfAtt,
									    CGeoCurve* pCurve )
{
	CGeoCurve *pTemp = NULL ;
	CTrmFin *pBnd, *pNext ;
	pBnd = pSurfAtt->m_pSurf->m_pLoop->m_pFinHead ;
	for( ; pBnd ; pBnd = pBnd->GetNext () )
	{
		pTemp = pBnd->m_pEdge->m_pCurve ;
		if( pTemp == pCurve )
			break ;
	}
	pNext = pBnd->GetNext () ;
	// 如果pNext为空,则pNext为最后一个,他的下一个为首
	if( !pNext )
		pNext = pSurfAtt->m_pSurf->m_pLoop->m_pFinHead ;

	return pNext->m_pEdge->m_pCurve ;
}
CGeoCurve* CFlowlineGen::GetPrevCurve ( C3DSurfAttribute* pSurfAtt,
									    CGeoCurve* pCurve )
{
	CGeoCurve *pTemp = NULL ;
	CTrmFin *pBnd, *pPrev ;
	pBnd = pSurfAtt->m_pSurf->m_pLoop->m_pFinHead ;
	for( ; pBnd ; pBnd = pBnd->GetNext () )
	{
		pTemp = pBnd->m_pEdge->m_pCurve ;
		if( pTemp == pCurve )
			break ;
	}
	pPrev = pBnd->GetPrev () ;
	// 如果pPrev为空,则pBnd为第一个,它前面一个为最后一个
	if( !pPrev )
	{
		for( ; pBnd ; pBnd = pBnd->GetNext() )
		{
			pPrev = pBnd ;
		}
	}
	return pPrev->m_pEdge->m_pCurve ;
}
void CFlowlineGen::ReverseAttSurf ( C3DSurfAttribute* pSurfAtt, BOOL bReverse, int nType )
{
	// 曲面反向,并设置反向属性
	pSurfAtt->m_pSurf->ReverseDir () ;
	pSurfAtt->m_bReverseSurf = bReverse ;
	
	// 曲面参数边界已经反向，但空间边界没有
	// 所以反向曲面的空间边界
	PNT3D t1, t2, start, end ;
	CTrmFin *pBnd = pSurfAtt->m_pSurf->m_pLoop->m_pFinHead ;
	for( ; pBnd ; pBnd = pBnd->GetNext () )
	{
		pBnd->m_pEdge->m_pCurve->Reverse() ;
		pBnd->m_pPCurve->GetEndPoint ( t1, t2 ) ;
		pBnd->m_pEdge->m_pCurve->GetEndPoint ( start, end ) ;
	}/**/

	if( nType == 0 )
	{ // 第一个
		pSurfAtt->m_pBoundStart = GetSubtense( pSurfAtt->m_pBoundEnd ) ;
	}
	else if( nType == 2 )
	{ // 最后一个
		pSurfAtt->m_pBoundEnd = GetSubtense( pSurfAtt->m_pBoundStart ) ;
	}
}
void CFlowlineGen::ReverseSurfEdge ( CGeoTrmSurf* pSurf )
{
	CTrmFin   *pBound, *pPrev ;
    pBound = pSurf->m_pLoop->m_pFinHead ;
	pSurf->m_pLoop->m_pFinHead = NULL ;
	PNT3D p1, p2, t1, t2 ;

	while( pBound && pBound->m_pNext)
	{
		pBound->m_pPCurve->GetEndPoint( t1, t2 ) ;
		pBound->m_pEdge->m_pCurve->GetEndPoint( p1, p2 ) ;
		
		pBound = pBound->GetNext() ;
	}
	
	while( pBound )
	{
		pPrev = pBound->GetPrev() ; 
		//  参数域反向
		pBound->m_pPCurve->GetEndPoint( t1, t2 ) ;
		pBound->m_pEdge->m_pCurve->GetEndPoint( p1, p2 ) ;
		pBound->ReverseBound() ;
		pBound->m_pPCurve->GetEndPoint( t1, t2 ) ;
		//  空间曲线反向
		pBound->m_pEdge->m_pCurve->Reverse() ;
		pBound->m_pEdge->m_pCurve->GetEndPoint( p1, p2 ) ;
		pSurf->m_pLoop->AddBound(pBound) ;
		pBound = pPrev ;
	}
}

void CFlowlineGen::ReverseLineOrder ( CSmtCPathList& pathList )
{
	CSmtCPathList tmpList ;
	CSmtCutPath *pPath = NULL ;
	POSITION pos, atpos ;
	pos = pathList.GetHeadPosition () ;
	while( pos )
	{
		atpos = pos ;
		pPath = pathList.GetNext ( pos ) ;
		tmpList.AddHead ( pPath ) ;
		pathList.RemoveAt ( atpos ) ;
	}
	pos = tmpList.GetHeadPosition () ;
	while( pos )
	{
		atpos = pos ;
		pPath = tmpList.GetNext ( pos ) ;
		
		pPath->m_nLineNo = -pPath->m_nLineNo ;
		
		pathList.AddTail ( pPath ) ;
		tmpList.RemoveAt ( atpos ) ;
	}
}

void CFlowlineGen::ReverseLineOrder ( C3DSurfAttribute* pSurfAtt )
{
	CSmtCPathList tmpList ;
	CSmtCutPath *pPath = NULL ;
	POSITION pos, atpos ;
	pos = pSurfAtt->m_cSmtCPathLib.m_cAllPath.GetHeadPosition () ;
	while( pos )
	{
		atpos = pos ;
		pPath = pSurfAtt->m_cSmtCPathLib.m_cAllPath.GetNext ( pos ) ;
		tmpList.AddHead ( pPath ) ;
		pSurfAtt->m_cSmtCPathLib.m_cAllPath.RemoveAt ( atpos ) ;
	}
	pos = tmpList.GetHeadPosition () ;
	while( pos )
	{
		atpos = pos ;
		pPath = tmpList.GetNext ( pos ) ;
		
		pPath->m_nLineNo = -pPath->m_nLineNo ;
		
		pSurfAtt->m_cSmtCPathLib.m_cAllPath.AddTail ( pPath ) ;
		tmpList.RemoveAt ( atpos ) ;
	}
}

void CFlowlineGen::AdjustPathStart ( CUWLineParam& cUWLine )
{
	int nCutDir  = cUWLine.m_nCutCross ;
	int nStartAt = cUWLine.m_nStartAt  ;

	BOOL bReverseLine      = FALSE ;
	BOOL bReverseLineOrder = FALSE ;
	BOOL bReverseSurfOrder = FALSE ;

	if( nStartAt == 1 || nStartAt == 2 )
		bReverseSurfOrder = TRUE ;
	if( nCutDir == 0 )  // U向
	{
		if( nStartAt == 1 )
			bReverseLine = TRUE ;
		else if( nStartAt == 2 )
		{
			bReverseLine = TRUE ;
			bReverseLineOrder = TRUE ;
		}
		else if( nStartAt == 3 )
			bReverseLineOrder = TRUE ;
	}
	else				// V向
	{
		if( nStartAt == 1 )
			bReverseLineOrder = TRUE ;
		else if( nStartAt == 2 )
		{
			bReverseLineOrder = TRUE ;
			bReverseLine = TRUE ;
		}
		else if( nStartAt == 3 )
			bReverseLine = TRUE ;
	}

	C3DSurfAttribute  *pSurfAtt ;
	INT_PTR nSize = cUWLine.m_cSurfAttArr.GetSize () ;
	int i = 0 ;
	if( bReverseSurfOrder )
	{
		for( i = 0 ; i < nSize / 2 ; i++ )
		{
			pSurfAtt = cUWLine.m_cSurfAttArr[i] ;
			cUWLine.m_cSurfAttArr[i] = cUWLine.m_cSurfAttArr[nSize-1-i] ;
			cUWLine.m_cSurfAttArr[nSize-1-i] = pSurfAtt ;
		}
	}
	POSITION pos ;
	CSmtCutPath *pPath1 = NULL ;
	for( i = 0 ; i < nSize ; i++ )
	{
		pSurfAtt = cUWLine.m_cSurfAttArr[i] ;
		// 如果螺旋加工并且曲面和直线的排序都不调整，线也不能调整
		if( pSurfAtt->m_bSpiral && ( !bReverseSurfOrder ) && ( !bReverseLineOrder ) )
			continue ;
		if(  bReverseLine || bReverseSurfOrder && pSurfAtt->m_bSpiral )
		{
			pos = pSurfAtt->m_cSmtCPathLib.m_cAllPath.GetHeadPosition() ;
			while( pos )
			{
				pPath1 = pSurfAtt->m_cSmtCPathLib.m_cAllPath.GetNext ( pos ) ;
				pPath1->ReverseDirect () ;
			}
		}
	}
	
	if( bReverseLineOrder )
	{	for( i = 0 ; i < nSize ; i++ )
		{
			pSurfAtt = cUWLine.m_cSurfAttArr[i] ;
			ReverseLineOrder( pSurfAtt ) ;
			// 将nLayer=0和nLayer=2的调换
			pos = pSurfAtt->m_cSmtCPathLib.m_cAllPath.GetHeadPosition () ;
			while( pos )
			{
				pPath1 = pSurfAtt->m_cSmtCPathLib.m_cAllPath.GetNext ( pos ) ;
				if( pPath1->m_nLayerNo == 0 )
					pPath1->m_nLayerNo = 2 ;
				else if( pPath1->m_nLayerNo == 2 )
					pPath1->m_nLayerNo = 0 ;
			}
		}
	}
}

BOOL CFlowlineGen::CreateUWLinePath(CSmtCPathLib& AllPath, C3DSurfArray& AllSurf, RFRAME& cRFrame,  
									JDNC_FUWLINE& UWLineCut , JDNC_PRGDEF &PrgDef, double dCur, 
									int *pErrorType, BOOL *arrbSpiral/* = NULL*/)
{
	if (NULL == pErrorType)
		return FALSE;
	m_pErrorType = pErrorType;
	CUWLParaArray ParaArr;			// 相邻曲面流线参数数组
	
	m_dStep     = UWLineCut.m_dOverStep ;
	m_dZMove	= m_cStockDef.m_dDriveZMove[0] - m_cStockDef.m_dSparkGap  ;
	C3DSurfArray tmpSurf ;
//	CSurfLoopArr cLoopArr ;

	// 判断曲面的有效性
	GetValidSurf( AllSurf ) ;
	INT_PTR nSurf = AllSurf.GetSize () ;
	if( nSurf < 1 ) nSurf = 1 ;
	PrgDef.m_dTotalMove = dCur / nSurf ;

	int nLayerNo = 0;		// 路径的层号，单个面的路径为一层，成组面的路径为一层
	if( !( UWLineCut.m_bUWLineFlag & NCDEF_UWLINE_ONEBYONE ) )
	{
		// 复制曲面的边界
//		CreateStitchBnd( AllSurf, tmpSurf, cLoopArr ) ;
		CopyAllSurf( AllSurf, tmpSurf ) ;

		// step1 重新处理曲面，让边界都为逆时针，重构曲面边界，剔除网格面
		ResetAllSurf( tmpSurf ) ;	
		// 调整所选曲面的顺序，并判断曲面是否符合流线加工的条件,判断是否光滑
		AdjustAllSurfType( tmpSurf, ParaArr ) ;
		SetUWLParaData(ParaArr, UWLineCut);	// 根据加工参数设置流线参数
		// 单个面生成路径
		if( tmpSurf.GetSize () > 0 )
		{	//生成单个面的流线加工路径
			if( !CreateEachSurfPath( m_pMiller, cRFrame, tmpSurf, UWLineCut, 
									m_cSetupDef.m_cTolDef.m_dArcTol, AllPath, nLayerNo, PrgDef, arrbSpiral ) )
			{
				DestroyAllSurf( tmpSurf, ParaArr ) ;
				return  FALSE ;
			}
		}

		// 创建相邻曲面流线路径
		C3DSurfArray SurfLeft;		// 剩余的曲面，这些曲面生成相邻的曲面路径不成功
		for (int i=0; i<ParaArr.GetSize(); i++)
		{
			CUWLineParam &UWLPara = ParaArr.GetAt(i);
			if (!CreateAdjSurfUWPath(AllPath, cRFrame, UWLineCut, ParaArr[i], nLayerNo, PrgDef))
			{
				for (int j=0; j<UWLPara.m_cSurfAttArr.GetSize(); j++)
				{
                    C3DSurfAttribute *pSurfAttr = UWLPara.m_cSurfAttArr.GetAt(j);
                    SurfLeft.Add(pSurfAttr->m_pSurf);
				}
			}
		}

		BOOL bSuccess = TRUE;
		if( SurfLeft.GetSize() > 0 )
		{	
			bSuccess = CreateEachSurfPath( m_pMiller, cRFrame, SurfLeft, UWLineCut, 
										   m_cSetupDef.m_cTolDef.m_dArcTol, AllPath, nLayerNo, PrgDef, arrbSpiral) ;
		}
		// 换回曲面的边界
		DestroyAllSurf( tmpSurf, ParaArr ) ;
		if( !bSuccess )
		{
			AllPath.ClearAllPath () ;
			return FALSE ;
		}
	}
	else
	{
		//生成单个面的流线加工路径
		if( AllSurf.GetSize ()  > 0 )
		{	
			if( !CreateEachSurfPath(m_pMiller, cRFrame, AllSurf, UWLineCut, 
									m_cSetupDef.m_cTolDef.m_dArcTol, AllPath, nLayerNo, PrgDef, arrbSpiral ) )
				return FALSE ;
		}
	}
	CSmtCutPath *pPath = NULL ;
	POSITION pos, atpos ;
	pos = AllPath.m_cAllPath.GetHeadPosition () ;
	while( pos )
	{
		atpos = pos ;
		pPath = AllPath.m_cAllPath.GetNext ( pos ) ;
		if( pPath->m_nNumPnt < 0 )
		{// 这个路径是无效的,应该去掉
			delete pPath ;
			AllPath.m_cAllPath.RemoveAt ( atpos ) ;
		}
		else if( pPath->m_nNumPnt< 2 )
		{// 可以删除的路径
			delete pPath ;
			AllPath.m_cAllPath.RemoveAt ( atpos ) ;
		}
		else
            pPath->DefineBox () ;
	}
	return TRUE ;
}

void CFlowlineGen::DeleteTrmLoop ( CTrmLoop* pLoop )
{
	CTrmLoop *pTmp ;
	while( pLoop )
	{
		pTmp = pLoop->GetNext () ;
		pLoop->m_pNext = NULL ;
		delete pLoop ;
		pLoop = pTmp ;
	}
}

BOOL CFlowlineGen::CreateUWLineByReConstruct ( CSmtCPathLib& cSmtCPathLib,
											   CUWLineParam& cUWLine,
											   JDNC_FUWLINE& UWLineCut,
											   RFRAME& cRFrame, 
											   JDNC_SETUP& Setup,
											   int &nStartLayerNo,			// 起始的路径层号
											   JDNC_PRGDEF &PrgDef )
{
	GetSurfStep1( cUWLine.m_cSurfAttArr,
				  UWLineCut.m_dOverStep , UWLineCut.m_nCutDir ) ;
	int nRet = GenSurfPathLib1( cUWLine, cRFrame, Setup, UWLineCut, PrgDef ) ;
	if( !nRet ) return FALSE ;
	// 调整路径的起点
	AdjustPathStart( cUWLine ) ;
	BOOL bZigZag = ( UWLineCut.m_bUWLineFlag & NCDEF_UWLINE_ZIGZAG ) ? TRUE : FALSE ;
	CSmtCPathLib TmpPathLib;	// 临时路径
	// 路径排序
	if( UWLineCut.m_nCutDir == 0 )
	{
		ConnectAllUPath1( cUWLine, bZigZag, TmpPathLib ) ;		
	}
	else
	{
		ConnectAllWPath1( cUWLine ,  bZigZag, TmpPathLib ) ;
	}

	// 修改路径的行号和层号，传出路径
	ModifyPathAndCpy(TmpPathLib, nStartLayerNo, cSmtCPathLib);	

	return 1 ;
}


BOOL CFlowlineGen::CreateUWLineByWDirection ( CSmtCPathLib& cSmtCPathLib,
											  CUWLineParam& cUWLine,
											  JDNC_FUWLINE& UWLineCut,
											  RFRAME& cRFrame, 
											  JDNC_SETUP& Setup,
											  int &nStartLayerNo,			// 起始的路径层号
											  JDNC_PRGDEF &PrgDef )
{
	if( UWLineCut.m_nCutDir == 0 )
		GetSurfStep1( cUWLine.m_cSurfAttArr,
					  UWLineCut.m_dOverStep , 0 ) ;
	// 计算曲面流线W向路径
	int nRet = GenSurfPathLib3( cUWLine, cRFrame, Setup, UWLineCut, PrgDef ) ;
	if( !nRet ) return FALSE ;
	// 调整路径的起点
	AdjustPathStart( cUWLine ) ;
	BOOL bZigZag = ( UWLineCut.m_bUWLineFlag & NCDEF_UWLINE_ZIGZAG ) ? TRUE : FALSE ;

	CSmtCPathLib TmpPathLib;	// 临时路径
	// 路径排序
	if( UWLineCut.m_nCutDir == 0 )
	{
		ConnectAllUPath1( cUWLine, bZigZag, TmpPathLib ) ;		
	}
	else
	{
		ConnectAllWPath3( cUWLine , bZigZag, TmpPathLib ) ;
	}
	/**/
	
	// 修改路径的行号和层号，传出路径
	ModifyPathAndCpy(TmpPathLib, nStartLayerNo, cSmtCPathLib);	
	return 1 ;
}

void CFlowlineGen::GetPointNormal ( CGeoTrmSurf* pSurf, CSmtCutPath* pPath, CSmartTool* pMiller, 
								    PNT2D param, VEC3D dNormal, PNT3D dPoint )
{
	PNT2D from, to ;
	nc_FloatToDouble( from, pPath->m_pHead->m_fPoint, 2) ;
	nc_FloatToDouble( to  , pPath->m_pTail->m_fPoint, 2) ;

	int nRet = pSurf->GetNormal( param[0], param[1], dPoint, dNormal) ;
	if( !nRet )
	{
		pSurf->GetPoint( param[0], param[1] , dPoint ) ;
		dNormal[0] = dNormal[1] = 0.0, dNormal[2] = 1.0 ;
		return ;
	}
	int type = pMiller->GetType () ;
	if( type == smtToolBall || type == smtToolABall )
		return ;

	// 防止平底刀、牛鼻刀、锥刀等底刃平的刀在法矢为竖直的地方偏移错误
	if( 1 - fabs(dNormal[2]) < 1.0e-6 )
	{
		VEC3D nextnor, sidenor ;
		PNT3D nextpnt, sidepnt ;

		GetNextNormal( pSurf, pPath, param, nextnor, nextpnt ) ;
		// 如果中间点
		if( 1 - fabs( nextnor[2] ) < 1.0e-6 )
		{
			GetSideNormal( pSurf, pPath, param, sidenor, sidepnt ) ;
			if( 1 - fabs( sidenor[2] ) > 1.0e-6 )
			{
				mathCpyPnt( sidenor, dNormal ) ;
				mathCpyPnt( sidepnt, dPoint ) ;
			}
		}
		else
		{
			mathCpyPnt( nextnor, dNormal ) ;
			mathCpyPnt( nextpnt, dPoint ) ;
		}
	}
}

void CFlowlineGen::GetNextNormal ( CGeoTrmSurf* pSurf, CSmtCutPath* pPath, 
								   PNT2D param, VEC3D dNormal, PNT3D dPoint )
{
	PNT2D from, to ;
	nc_FloatToDouble( from, (( CSmtCutPointEx *)pPath->m_pHead)->m_fSurfNor, 2 ) ;
	nc_FloatToDouble( to  , (( CSmtCutPointEx *)pPath->m_pTail)->m_fSurfNor, 2 ) ;
	VEC2D vec ;
	double d1, d2 ;

	mathGetVec2D( from, to, vec ) ;
	mathUniVec2D( vec, MIN_LEN  ) ;
	if( mathDist2D( from, param ) < MIN_DIS )
	{
		d1 = vec[0] * 0.001 ;
		d2 = vec[1] * 0.001 ;
		pSurf->GetNormal( param[0]+d1, param[1]+d2, dPoint, dNormal ) ;
	}
	else if( mathDist2D( to, param ) < MIN_DIS )
	{
		d1 = -vec[0] * 0.001 ;
		d2 = -vec[1] * 0.001 ;
		pSurf->GetNormal( param[0]+d1, param[1]+d2, dPoint, dNormal ) ;
	}
	else
	{
		d1 = vec[0] * 0.001 ;
		d2 = vec[1] * 0.001 ;
		pSurf->GetNormal( param[0]+d1, param[1]+d2, dPoint, dNormal ) ;
	}
}

int CFlowlineGen::GetPoint ( CGeoLine *pLine, double tmp, double dLength, PNT3D pos ) 
{
	if( !pLine ) return 0 ;
	int i ;
	double u = tmp / dLength ;
	for( i = 0 ; i < 3 ; i++ )
	{
		pos[i] = pLine->m_begin[i] + u * ( pLine->m_end[i] - pLine->m_begin[i] ) ;
	}
	
	return 1 ;
}
int CFlowlineGen::GetPoint( CGeoPLine3d *pLine, 
                            double tmp, 
                            double /*dLength*/, 
                            PNT3D pos )
{
	if( !pLine->IsValid () ) return 0 ;
	double dDist = 0., dTotal = 0 ;
	for( int i = 1 ; i <= pLine->m_num ; i++ )
	{
		dDist = mathDist( pLine->m_dPoint[i-1], pLine->m_dPoint[i] ) ;
		dTotal += dDist ;
		if( dTotal >= tmp )
			break ;
	}
	double t = 1 - ( dTotal - tmp ) / dDist ;
	for( int k = 0 ; k < 3 ; k++ )
	{
		pos[k] = pLine->m_dPoint[i-1][k] + t * ( pLine->m_dPoint[i][k] - pLine->m_dPoint[i-1][k] ) ;
	}
	return 1 ; 
}
void CFlowlineGen::AddPathPointEx ( CSmtCutPath *pPath, PNT4D pnt )
{
	CSmtCutPointEx *pEx = new CSmtCutPointEx() ;
	for( int i = 0 ; i < 4 ; i++ )
	{
		pEx->m_fPoint[i] = float( pnt[i] ) ;
	}
	pPath->AddTail ( pEx ) ;
}
void CFlowlineGen::AddPathPointEx ( CSmtCutPath *pPath, float pnt[4] )
{
	CSmtCutPointEx *pEx = new CSmtCutPointEx() ;
	for( int i = 0 ; i < 4 ; i++ )
	{
		pEx->m_fPoint[i] = pnt[i] ;
	}
	pPath->AddTail ( pEx ) ;
}
void CFlowlineGen::GetSideNormal ( CGeoTrmSurf* pSurf, CSmtCutPath* pPath, 
								   PNT2D param, VEC3D dNormal, PNT3D dPoint )
{
	PNT2D from, to ;
	nc_FloatToDouble( from, ((CSmtCutPointEx *)pPath->m_pHead)->m_fSurfNor, 2 ) ;
	nc_FloatToDouble( to  , ((CSmtCutPointEx *)pPath->m_pTail)->m_fSurfNor, 2 ) ;
	VEC2D vec1, vec2 ;
	double d1, d2 ;

	mathGetVec2D( from, to, vec1 ) ;
	mathUniVec2D( vec1, MIN_LEN  ) ;
	// 逆时针旋转90
	mathRotVec2D( PI1_2, vec1, vec2 ) ;
	d1 = vec2[0] * 0.001 ;
	d2 = vec2[1] * 0.001 ;

	pSurf->GetNormal( param[0]+d1, param[1]+d2, dPoint, dNormal ) ;
	
}
void CFlowlineGen::ImagePntToSurf ( CGeoTrmSurf* pSurf, CSmtCutPath* pPath, CSmartTool* pMiller, 
								    RFRAME& cRFrame,  PNT2D param, PNT3D dPoint[2] )
{
	if( !pSurf || !pMiller ) 
		return ;

	VEC3D dNormal, dOffset ;
	PNT3D Point ;
	GetPointNormal( pSurf, pPath, pMiller, param, dNormal, Point ) ;

	mathTransWorldPnt3D( &cRFrame, Point,  Point  ) ;
	mathTransWorldPnt3D( &cRFrame, dNormal, dNormal ) ;

	pMiller->GetFSafeVector ( dNormal, dOffset ) ;
	dPoint[0][0] = Point[0] + dOffset[0] ;
	dPoint[0][1] = Point[1] + dOffset[1] ;
	dPoint[0][2] = Point[2] + dOffset[2] + m_dZMove ;
	
	mathCpyPnt( dNormal, dPoint[1] ) ;
}

void CFlowlineGen::BisectInsertPnt ( CGeoTrmSurf* pSurf, CSmartTool* pMiller, 
									 CSmtCutPath* pPath, RFRAME& cRFrame,
									 CSmtCutPointEx* dStart, CSmtCutPointEx* dEnd )
{
	CSmtCutPointEx *AtFrm = NULL, *AtTo = NULL, *pInsert = NULL ;
	double dArc = 0., t = 0. ;
	double dDist = min( m_cSetupDef.m_cTolDef.m_dMaxStep, 0.15 ) ;

	float fCoef[3] = { 0.5f, 0.25f, 0.75f } ;
	int i = 0, j = 0 ;
	PNT2D mid ;
	PNT3D point[2], from, to, pnt ;
	BOOL bInsert = FALSE ;

	AtFrm = dStart ;
	while( AtFrm && AtFrm->next )
	{
		if( AtFrm == dEnd ) break ;
		AtTo = (CSmtCutPointEx*)AtFrm->next  ;
		
		// 插入三个点
		for( i = 0 ; i < 3 ; i++ )
		{
			bInsert = FALSE ;
			for( j = 0 ; j < 2 ; j++ )
			{
				mid[j] = AtFrm->m_fSurfPos[j] + fCoef[i] * ( AtTo->m_fSurfPos[j] - AtFrm->m_fSurfPos[j] ) ;
			}
			// 映射到空间域
			ImagePntToSurf( pSurf, pPath, pMiller, cRFrame, mid, point ) ;
			nc_FloatToDouble( from, AtFrm->m_fPoint, 3 ) ;
			nc_FloatToDouble( to  , AtTo->m_fPoint  ,3 ) ;
			nc_VectorCopy( pnt , point[0] , 3 ) ;

			// 精度是否合适
			dArc = mathDistPntLinEx( pnt, from, to ) ;
			t = GetLineParam3D( from, to, pnt ) ;
			if( dArc > 1.0e-4 || ( m_cSetupDef.m_cModelTol.m_nMdlFlag & NCDEF_SETUP_NOMOSAIC ) && mathDist( from, to ) > dDist )
			{
				bInsert = TRUE ;
				break ;
			}
		}
		
		if( bInsert )
		{
			// 插入节点
			pInsert = new CSmtCutPointEx() ;
			nc_DoubleToFloat( pInsert->m_fPoint, point[0], 3 ) ;
			nc_DoubleToFloat( pInsert->m_fSurfNor, point[1], 3 ) ;

			pInsert->m_fSurfPos[0] = float( mid[0] ) ;
			pInsert->m_fSurfPos[1] = float( mid[1] ) ;
			pPath->InsertAfter( pInsert, AtFrm ) ;
		}
		else
		{
			AtFrm = (CSmtCutPointEx*)AtFrm->next ;
		}
	}
}
void CFlowlineGen::BisectInsert( CGeoTrmSurf* pSurf, CSmartTool* pMiller, 
								 CSmtCutPath* pPath, RFRAME& cRFrame,
								 CSmtCutPointEx* dStart, PNT2D start,
								 CSmtCutPointEx* dEnd,   PNT2D end )
{
	if( !pSurf || !pMiller || !pPath )
		return ;
	PNT2D mid ;
	PNT3D point[2] ;
	// 这个函数很不安全，如果在中点处恰好符合条件就错了
	mid[0] = 0.5 * ( start[0] + end[0] ) ;
	mid[1] = 0.5 * ( start[1] + end[1] ) ;

	ImagePntToSurf( pSurf, pPath, pMiller, cRFrame, mid, point ) ;

	PNT3D from, to, pnt ;
	nc_FloatToDouble( from, dStart->m_fPoint, 3 ) ;
	nc_FloatToDouble( to  , dEnd->m_fPoint  ,3 ) ;
	nc_VectorCopy( pnt , point[0] , 3 ) ;

	double d = mathDistPntLinEx( pnt, from, to ) ;
	double t = GetLineParam3D( from, to, pnt ) ;
	// 为了防止在退化点处递归调用死机
	if( d < 1.0e-6 )
	{
		if( t > -0.0001 && t < 1.0001 )
			return ;
		else
			d = 1. ;
	}
	double dDist = min( m_cSetupDef.m_cTolDef.m_dMaxStep, 0.15 ) ;
	if( d > 1.0e-4 || ( m_cSetupDef.m_cModelTol.m_nMdlFlag & NCDEF_SETUP_NOMOSAIC ) && mathDist( from, to ) > dDist )
	{
		CSmtCutPointEx* dMid = new CSmtCutPointEx() ;
		nc_DoubleToFloat( dMid->m_fPoint, point[0], 3 ) ;
		nc_DoubleToFloat( dMid->m_fSurfNor, point[1], 3 ) ;
		dMid->m_fSurfPos[0] = float( mid[0] ) ;
		dMid->m_fSurfPos[1] = float( mid[1] ) ;
		pPath->InsertBefore( dMid, dEnd ) ;

		BisectInsert( pSurf, pMiller, pPath, cRFrame, 
					  dStart, start, dMid, mid  ) ;
		BisectInsert( pSurf, pMiller, pPath, cRFrame,
					  dMid, mid, dEnd, end ) ;
	}
}

BOOL CFlowlineGen::TransfPathToSurf( C3DSurfAttribute* pSurf, CSmartTool *pMiller, 
									 RFRAME& cRFrame, CSmtCutPath* pCutPath )
{
	if( (!pCutPath) || (!pSurf) || (!pMiller) || pCutPath->m_nNumPnt<=1 ) return FALSE;

    // 用于路径向曲面投影，修改后可以处理多个路径点的路径 qqs 2013.05.20
	pCutPath->DefineBox () ;
	CSmtCutPath* pTempPath = pCutPath->CopyMyself();
	pCutPath->ClearAllPoint();
	CSmtCutPoint* St = pTempPath->m_pHead;
	while(St&&St->next)
	{
		CSmtCutPath* pNewPath = new CSmtCutPath;
		CSmtCutPointEx* dStart = (CSmtCutPointEx*)St->CopyMyself();
		CSmtCutPointEx* dEnd = (CSmtCutPointEx*)St->next->CopyMyself() ;
		pNewPath->AddTail(dStart);
		pNewPath->AddTail(dEnd);

		PNT2D start, end ;
		PNT3D pStart[2], pEnd[2] ;

		nc_FloatToDouble( start, dStart->m_fPoint, 2 ) ;
		nc_FloatToDouble( end  , dEnd->m_fPoint  , 2 ) ;

		ImagePntToSurf( pSurf->m_pSurf, pNewPath, pMiller, cRFrame, start, pStart ) ;
		ImagePntToSurf( pSurf->m_pSurf, pNewPath, pMiller, cRFrame, end  , pEnd ) ;

		nc_DoubleToFloat( dStart->m_fPoint, pStart[0], 3 ) ;
		nc_DoubleToFloat( dStart->m_fSurfNor, pStart[1], 3 ) ;
		nc_DoubleToFloat( dEnd->m_fPoint, pEnd[0], 3 ) ;
		nc_DoubleToFloat( dEnd->m_fSurfNor, pEnd[1], 3 ) ;
		for( int i = 0 ; i < 2 ; i++ )
		{
			dStart->m_fSurfPos[i] = float( start[i] ) ;
			dEnd->m_fSurfPos[i] = float( end[i] ) ;
		}

		BisectInsertPnt( pSurf->m_pSurf, pMiller, pNewPath, cRFrame, dStart, dEnd ) ;
		CSmtCutPoint* ptIn = pNewPath->m_pHead;
		pCutPath->AddTail(ptIn->CopyMyself());
		ptIn = ptIn->next;
		while(ptIn)
		{
			pCutPath->AddTail(ptIn->CopyMyself());
			ptIn = ptIn->next;
		}
		St = St->next;
		delete pNewPath;
		pNewPath = NULL;
	}
	delete pTempPath;
	pTempPath = NULL;

	return TRUE;
}

// 平面曲线转换成刀距路径 
CSmtCutPath* CFlowlineGen::millface_CurveToCPath( CSmartCurve&  Curve ,JDNC_TOL& Tol ) 
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

BOOL CFlowlineGen::CheckAllPathBySelf ( CSmtCheckMdl& DriveMdl, CSmtCPathLib& AllPath, JDNC_TOL& Tol, 
									    JDNC_PRGDEF& PrgDef, double dCur, int *pErrorType)
{
	if (NULL == pErrorType)
		return FALSE;
	m_pErrorType = pErrorType;

	BOOL bRet = TRUE ;
	// 对DriveMdl的刀具作偏移处理
	double dTol = Tol.m_dArcTol * 2.5 ;
	CSmartTool *pTool = DriveMdl.m_pTool ;
	CSmartTool *pCheck = DriveMdl.m_pTool->Offset ( -dTol ) ;
	if( pCheck )
		DriveMdl.UpdateCurrTool ( pCheck ) ;
	
	// 需要多线程的拷贝，则拷贝一份DriveMdl
	BOOL bLocalCopy = FALSE ;	
	if (MathCAM_IsNeedMultiCopy(DriveMdl, m_nCalcThreadNum))
	{
		bLocalCopy = TRUE ;
		DriveMdl.CreateMultiCopy(m_nCalcThreadNum - 1) ;
	}	

	// 支持多线程，即拷贝数据不为空
	INT_PTR nPathNum = AllPath.GetNumPath();
	CVecPointer NewPath;
	NewPath.resize(nPathNum + 5);
	ZeroMemory(&NewPath[0], sizeof(LPVOID) * (nPathNum + 5));
	if (nPathNum >= 2 && DriveMdl.GetMultiCopy() != NULL)
	{
		LPVOID lpParam[NC_CFG_CPU_MAXNUM] = {NULL};
		CAPT_DATA ThreadData[NC_CFG_CPU_MAXNUM];
		JDNC_PRGDEF tmpPrg = PrgDef;
		tmpPrg.m_pPrgFunc = NULL ; 
		tmpPrg.m_pPosFunc = NULL ; 
		int nThreadNum = min(m_nCalcThreadNum, (int)nPathNum), k=1;
		for (int i=0; i<nThreadNum; i++)
		{
			ThreadData[i].pDriveMdl = i == 0 ? &DriveMdl : DriveMdl.GetMultiCopy(i-k); 
			if (ThreadData[i].pDriveMdl == NULL)
			{
				--i, --k, --nThreadNum;
				continue;
			}
			ThreadData[i].nAtCore = i;
			ThreadData[i].dCur = dCur;			
			ThreadData[i].pTol = &Tol;
			ThreadData[i].PrgDef = i==0 ? PrgDef : tmpPrg;
			ThreadData[i].AllPath = &AllPath;
			ThreadData[i].NewPath = &NewPath[0];
			ThreadData[i].pFlowlineGen = this;
			lpParam[i] = &ThreadData[i];
		}
		for (i=0; i<nThreadNum; i++)
		{
			ThreadData[i].nCoreNum = nThreadNum;
		}
		bRet = MathCAM_ThreadMainFunc(MathCAM_CheckAllPathBySelfSubProc, lpParam, nThreadNum);
	}
	else
	{
		bRet = CheckAllPathBySelfSubProc(DriveMdl, AllPath, Tol, PrgDef, dCur, 0, 1, &NewPath[0]);
	}

	AllPath.m_cAllPath.RemoveAll();
	if (bRet)
	{
		Nc5D_AddPathArrayToLib(&NewPath[0], nPathNum, AllPath);
	}

	// DriveMdl的数据在本函数拷贝，则在本函数删除
	if( bLocalCopy == TRUE ) 
	{
		DriveMdl.DeleteMultiCopy() ;
	}

	// 删除临时检查刀具，并更新到初始状态
	if( pCheck )
		delete pCheck ;
	DriveMdl.UpdateCurrTool ( pTool ) ;

	return bRet;
}

// 自身干涉检查主函数
BOOL CFlowlineGen::CheckAllPathBySelfSubProc(CSmtCheckMdl& DriveMdl, CSmtCPathLib& AllPath, JDNC_TOL& Tol, 
											 JDNC_PRGDEF& PrgDef, double dCur, int nAtCore, int nCoreNum, LPVOID* NewPath)
{	
	int nNum = AllPath.GetNumPath () ;
	if( nNum < 1 ) return TRUE;

	PrgDef.m_dTotalMove = dCur;							// 前进的总量
	PrgDef.m_dIncStep = PrgDef.m_dTotalMove / nNum ;	// 每次计算的前进量						
	PrgDef.m_dLimitAt = 1.0 ;							// 前进一次的计算量

	CSmtCutPath *pPath = NULL, *pNext = NULL ;
	BOOL bRet = TRUE ;	
	CSmtCPathList tmpList ;
	int nIndex = 0;
	POSITION pos = AllPath.m_cAllPath.GetHeadPosition () ;
	while( pos )
	{
		if( PrgDef.m_pBrkFunc && PrgDef.m_pBrkFunc () )
		{
			bRet = FALSE ;
			if (m_pErrorType) *m_pErrorType = JDERROR_GENPATH_ABORT;
			break ;
		}
		MathCAM_MovePrgStep(PrgDef);
		pPath = AllPath.m_cAllPath.GetNext ( pos ) ;		
		if (nIndex++ % nCoreNum != nAtCore || pPath == NULL)
		{
			continue;
		}						
		if( pPath->m_nNumPnt < 2 )
		{
			delete pPath ;
			continue ;
		}
		pPath->DefineBox () ;
		// 首先判断本身加工的路径是否干涉,如果干涉,进行自动修剪连接
		DriveMdl.LabelCheckByBox( pPath->m_fBox ) ;
		// 防止过长的直线中间没有插点,提前差点
		InsertCPointEx( pPath, Tol.m_dMaxStep ) ;
		// 判断有效和无效点，插入断点
		CheckPathBySelf( DriveMdl, pPath, Tol ) ;
		// 在断点处打断
		pNext = BreakAndDelPnt( pPath ) ;	
		pPath = pNext;		
		while( pNext )
		{
			pNext->SetCutMode ( MINI_CONNECT_PATH ) ;
			VerifyCutPathEx ( &DriveMdl, pNext, TRUE, Tol.m_dArcTol ) ;
			pNext = pNext->next;
		}
		NewPath[nIndex-1] = pPath;// 将得到的路径添加到组中		
	}

	if( PrgDef.m_pBrkFunc && PrgDef.m_pBrkFunc () )
	{
		Nc5D_DestroyMultiCalcData(NewPath, nIndex, &AllPath, pos, nCoreNum, nAtCore);		
	}

	return bRet ;
}

void CFlowlineGen::VerifyCutPathEx ( CSmtCheckMdl *DriveMdl, CSmtCutPath *pPath, BOOL bCheck, double dArcTol )
{
	FPNT3D fBox[2] ;
	
	BOOL bFst = TRUE ;
	CSmtCutPointEx *pHead = (CSmtCutPointEx *)pPath->m_pHead, *pNext = NULL ;
	pHead->m_bType = SMART_CUTPNT_IVER ;
	pHead->m_fPoint[3] = pHead->m_fPoint[2] ;
	while( pHead )
	{
		pNext = (CSmtCutPointEx *)pHead->next ;
		if( pNext )
		{
			pNext->m_bType = SMART_CUTPNT_IVER ;
			pNext->m_fPoint[3] = pNext->m_fPoint[2] ;
			mathFDefBox3D( pHead->m_fPoint, pNext->m_fPoint, fBox, 0.01 ) ;
			DriveMdl->LabelCheckByBox ( fBox ) ;
			if( bFst )
			{
 				if( bCheck && !IsSet3DRCompMask()) 
 					DriveMdl->DefineHeightEx ( pHead->m_fPoint, pHead->m_fSurfNor ) ;
 				else
					DriveMdl->DefineHeight ( pHead->m_fPoint ) ;
				bFst = FALSE ;
			}
 			if( bCheck && !IsSet3DRCompMask())
                 DriveMdl->DefineHeightEx ( pNext->m_fPoint, pNext->m_fSurfNor ) ;
 			else
				DriveMdl->DefineHeight ( pNext->m_fPoint ) ;
            InsertPathPntEx( DriveMdl, pPath, bCheck, pHead, pNext, dArcTol ) ;
		}
		else
			break ;
		if( SurfNC_IsAbort() ) 
		{
			break ;
		}
		pHead = pNext ;
	}
}
void CFlowlineGen::InsertPathPntEx ( CSmtCheckMdl *DriveMdl, CSmtCutPath *pPath, BOOL bCheck, 
								    CSmtCutPointEx *pStart, CSmtCutPointEx *pEnd, double dArcTol )
{
	CSmtCutPointEx *AtFrm = NULL, *AtTo = NULL, *pInsert = NULL ;
	double dStep = dArcTol * 0.5, dArc = 0. ;
	float fCoef[3] = { 0.5f, 0.25f, 0.75f } ;
	int i = 0, j = 0 ;
	FPNT4D AtPnt;
	FPNT3D AtNor, AtPos ;
	double dDist2D = mathFDist2D( pStart->m_fPoint, pEnd->m_fPoint ) ;
	double dH = pEnd->m_fPoint[3] - pStart->m_fPoint[3], dTmpH = 0. ;

	AtFrm = pStart ;
	while( AtFrm && AtFrm->next )
	{
		if( AtFrm == pEnd ) break ;
		AtTo = (CSmtCutPointEx*)AtFrm->next  ;
		if( mathFDist2D( AtFrm->m_fPoint, AtTo->m_fPoint ) < dStep )
		{ // 合格不需要继续插点
			AtFrm = (CSmtCutPointEx*)AtFrm->next ;
		}
		else
		{ // 插入三个点
			for( i = 0 ; i < 3 ; i++ )
			{
				for( j = 0 ; j < 3 ; j++ )
				{
					AtPnt[j] = AtFrm->m_fPoint[j] + fCoef[i] * ( AtTo->m_fPoint[j] - AtFrm->m_fPoint[j] ) ;
					AtNor[j] = AtFrm->m_fSurfNor[j] + fCoef[i] * ( AtTo->m_fSurfNor[j] - AtFrm->m_fSurfNor[j] ) ;
					AtPos[j] = AtFrm->m_fSurfPos[j] + fCoef[i] * ( AtTo->m_fSurfPos[j] - AtFrm->m_fSurfPos[j] ) ;
				}
				mathFUniVec( AtNor, MIN_LEN ) ;
				AtPnt[3] = AtPnt[2];		// 把Z坐标临时存储于位置3
				if( bCheck )
					DriveMdl->DefineHeightEx ( AtPnt, AtNor ) ;
				else
					DriveMdl->DefineHeight ( AtPnt ) ;
				// 精度是否合适
				dArc = mathFDistPntLinEx( AtPnt, AtFrm->m_fPoint, AtTo->m_fPoint ) ;
				if( dArc > dArcTol ) break ;
			}
			// 防止点掉下去
			dTmpH = pStart->m_fPoint[2] + dH * mathFDist2D( AtPnt, pStart->m_fPoint )/dDist2D ;
			if( i < 3 && AtPnt[2] >= dTmpH )
			{
				// 插入节点
				pInsert = new CSmtCutPointEx() ;
				mathFCpyPnt4D( AtPnt, pInsert->m_fPoint ) ;
				mathFCpyPnt( AtNor, pInsert->m_fSurfNor ) ;
				mathFCpyPnt( AtPos, pInsert->m_fSurfPos ) ;
				pPath->InsertAfter( pInsert, AtFrm ) ;
			}
			else
			{
				AtFrm = (CSmtCutPointEx*)AtFrm->next ;
			}
		}
	}
}
void CFlowlineGen::CheckPathBySelf ( CSmtCheckMdl& DriveMdl, CSmtCutPath* pPath, JDNC_TOL& Tol )
{
	if( !pPath ) return ;
	CSmtCutPoint *pHead = NULL, *Start = NULL, *Next = NULL ;
	
	// 对路径进行干涉检查
	VerifyCutPathEx( &DriveMdl, pPath, TRUE, Tol.m_dArcTol ) ;
	Start = Next = NULL ;
	// 为新增加的点中间插入m_fPoint[3] ;
	Start = pPath->m_pHead ;
	for( pHead = pPath->m_pHead ; pHead ; pHead = pHead->next )
	{
		// 如果当前点== next
		if( pHead == Next ) 
		{
			Start = Next ;
			Next  = NULL ;
		}
		// 如果Next== null,搜索到下一个btype == SMART_CUTPNT_IVER
		if( Next == NULL )
		{
			for( Next = pHead->next ; Next ; Next = Next->next )
			{
				if( Next->m_bType == SMART_CUTPNT_IVER ) 
					break ;
			}
		}
		
		if( pHead->m_bType == SMART_CUTPNT_NULL && Next && Start)
		{ // 插入点
			double dLen = GetCutPointDist( Start, Next ) ;
			double del  = GetCutPointDist( Start, pHead ) ;
			if( dLen < 1.0e-6 )
			{
				Start = Next, Next = NULL ;
				continue ;
			}
			double t = del / dLen ;
			pHead->m_fPoint[3] = float( Start->m_fPoint[3] + t*( Next->m_fPoint[3] - Start->m_fPoint[3] ) ) ;
		}
		if( pHead->m_fPoint[2] > pHead->m_fPoint[3] + Tol.m_dArcTol * 2.5 )
			pHead->m_bType = SMART_CUTPNT_NULL ;
		else
			pHead->m_bType = SMART_CUTPNT_IVER ;
	}/**/
	// 在有效和无效之间将有效点改为break，然后在该点处断开
	Start = pPath->m_pHead ;
	while( Start )
	{
		Next = Start->next ;
		if( !Next ) break ;
		if( Start->m_bType == SMART_CUTPNT_NULL && Next->m_bType == SMART_CUTPNT_IVER )
		{
			Next->m_bType = SMART_CUTPNT_BREAK ;
		}
		else if( Start->m_bType == SMART_CUTPNT_IVER && Next->m_bType == SMART_CUTPNT_NULL )
		{
			Start->m_bType = SMART_CUTPNT_BREAK ;
		}
		Start = Next ;
	}/**/
}
BOOL CFlowlineGen::CheckAllPathBySurf ( CSmtCheckMdl& CheckMdl, CSmtCPathLib& AllPath, JDNC_TOL& Tol, 
									    BOOL bCheck, JDNC_PRGDEF& PrgDef, double dCur, int *pErrorType)
{
	if (NULL == pErrorType)
		return FALSE;
	m_pErrorType = pErrorType;
	BOOL bRet = TRUE;

	// 需要多线程的拷贝，则拷贝一份CheckMdl
	BOOL bLocalCopy = FALSE ;	
	if (MathCAM_IsNeedMultiCopy(CheckMdl, m_nCalcThreadNum))
	{
		bLocalCopy = TRUE ;
		CheckMdl.CreateMultiCopy(m_nCalcThreadNum - 1) ;
	}	

	// 支持多线程，即拷贝数据不为空
	INT_PTR nPathNum = AllPath.GetNumPath();
	CVecPointer NewPath;
	NewPath.resize(nPathNum + 5);
	ZeroMemory(&NewPath[0], sizeof(LPVOID) * (nPathNum + 5));
	if (nPathNum >=2 && CheckMdl.GetMultiCopy() != NULL)
	{
// 		CSmtCPathLib Path[NC_CFG_CPU_MAXNUM];
		CAPT_DATA ThreadData[NC_CFG_CPU_MAXNUM];
		LPVOID lpParam[NC_CFG_CPU_MAXNUM] = {NULL};
		JDNC_PRGDEF tmpPrg = PrgDef;
		tmpPrg.m_pPrgFunc = NULL ; 
		tmpPrg.m_pPosFunc = NULL ; 
		int nThreadNum = min((int)nPathNum, m_nCalcThreadNum), k=1;
		for (int i=0; i<nThreadNum; i++)
		{
			ThreadData[i].pDriveMdl = i == 0 ? &CheckMdl : CheckMdl.GetMultiCopy(i-k); 
			if (ThreadData[i].pDriveMdl == NULL)
			{
				--i, --k, --nThreadNum;
				continue;
			}
			ThreadData[i].nAtCore = i;
			ThreadData[i].dCur = dCur;			
			ThreadData[i].bCheck = bCheck;
			ThreadData[i].pTol = &Tol;
			ThreadData[i].PrgDef = i==0 ? PrgDef : tmpPrg;
// 			ThreadData[i].AllPath = &Path[i];
			ThreadData[i].AllPath = &AllPath;//guomin path[i]未赋值为空 12/09/12
			ThreadData[i].NewPath = &NewPath[0];
			ThreadData[i].pFlowlineGen = this;
			lpParam[i] = &ThreadData[i];
		}
		for (i=0; i<nThreadNum; i++)
		{
			ThreadData[i].nCoreNum = nThreadNum;
		}
		bRet = MathCAM_ThreadMainFunc(MathCAM_CheckAllPathBySurfSubProc, lpParam, nThreadNum);
	}
	else
	{
		bRet = CheckAllPathBySurfSubProc(CheckMdl, AllPath, Tol, PrgDef, dCur, bCheck, 0, 1, &NewPath[0]);
	}

	AllPath.m_cAllPath.RemoveAll();
	if (bRet)
	{
		Nc5D_AddPathArrayToLib(&NewPath[0], nPathNum, AllPath);
	}

	// DriveMdl的数据在本函数拷贝，则在本函数删除
	if( bLocalCopy == TRUE ) 
	{
		CheckMdl.DeleteMultiCopy() ;
	}

	return bRet;
}

// 干涉面检查主函数
BOOL CFlowlineGen::CheckAllPathBySurfSubProc(CSmtCheckMdl& CheckMdl, CSmtCPathLib& AllPath, JDNC_TOL& Tol, 
											 JDNC_PRGDEF& PrgDef, double dCur, BOOL bCheck, int nAtCore, int nCoreNum, LPVOID* NewPath)
{
	int nNum = AllPath.GetNumPath () ;
	if( nNum < 1 ) return TRUE;
	
	PrgDef.m_dTotalMove = dCur;							// 前进的总量
	PrgDef.m_dIncStep = PrgDef.m_dTotalMove / nNum ;	// 每次计算的前进量						
	PrgDef.m_dLimitAt = 1.0 ;							// 前进一次的计算量

	CSmtCutPath *pPath/*, *pNext = NULL*/ ;
	POSITION pos = AllPath.m_cAllPath.GetHeadPosition () ;
	int nIndex = 0;
	while( pos )
	{
		if( PrgDef.m_pBrkFunc && PrgDef.m_pBrkFunc () )
		{
			if (m_pErrorType) *m_pErrorType = JDERROR_GENPATH_ABORT;
			break ;
		}
		MathCAM_MovePrgStep(PrgDef);
		pPath = AllPath.m_cAllPath.GetNext ( pos ) ;
		if(nIndex++ % nCoreNum != nAtCore || pPath==NULL)
		{
            continue ;
		}
		pPath->DefineBox () ;
		pPath = TrimPathBySurf( CheckMdl, pPath, bCheck, Tol ) ;
		// 将得到的路径添加到组中
		if( pPath != NULL)
		{
			NewPath[nIndex-1] = BreakAndDelPnt( pPath ) ;
		}		
	}
	if( PrgDef.m_pBrkFunc && PrgDef.m_pBrkFunc () )
	{
		Nc5D_DestroyMultiCalcData(NewPath, nIndex, &AllPath, pos, nCoreNum, nAtCore);	
		return FALSE ;
	}
	return TRUE ;
}

CSmtCutPath* CFlowlineGen::TrimPathBySurf ( CSmtCheckMdl& CheckMdl, CSmtCutPath* pPath, BOOL bCheck, JDNC_TOL& Tol )
{
	CSmtCutPointEx *Start, *Next ;
	Start = Next = NULL ;
// 	Start = (CSmtCutPointEx*)pPath->m_pHead ;
	// 防止过长的直线中间没有插点,提前差点
	InsertCPointEx( pPath, Tol.m_dMaxStep ) ;
	/*
	for( ; Start ; Start = (CSmtCutPointEx*)Start->next )
	{
		Start->m_fPoint[3] = Start->m_fPoint[2] ;
	}
	CheckMdl.LabelCheckByBox ( pPath->m_fBox ) ;
	Start = (CSmtCutPointEx*)pPath->m_pHead ;
	for( ; Start ; Start = (CSmtCutPointEx*)Start->next )
	{
		if( bCheck )
			CheckMdl.DefineHeightEx( (*Start).m_fPoint, (*Start).m_fSurfNor ) ;
		else	
			CheckMdl.DefineHeight( (*Start).m_fPoint ) ;
	}*/

	VerifyCutPathEx( &CheckMdl, pPath, bCheck, Tol.m_dArcTol ) ;
	//	pPath->VerifyCutPathEx ( CheckMdl, Tol, m_cPrgDef ) ;
	// 交换点的坐标
	TFLOAT tmp ;
	for( Start = (CSmtCutPointEx*)pPath->m_pHead ; Start; Start = (CSmtCutPointEx*)Start->next )
	{
		tmp = Start->m_fPoint[2] ;
		Start->m_fPoint[2] = Start->m_fPoint[3] ;
		Start->m_fPoint[3] = tmp ;
		if( Start->m_fPoint[2] < Start->m_fPoint[3] )
			Start->m_bType = SMART_CUTPNT_NULL ;
		else
			Start->m_bType = SMART_CUTPNT_IVER ;


	}
	Start = (CSmtCutPointEx*)pPath->m_pHead ;
	for( ; Start ; Start = (CSmtCutPointEx*)Start->next )
	{
		Next = (CSmtCutPointEx*)Start->next ;
		if( !Next ) break ;
		if( fabs(Start->m_fPoint[2] - Start->m_fPoint[3]) < MIN_FLT && 
			fabs(Next->m_fPoint[2] - Next->m_fPoint[3]) < MIN_FLT)
			continue;
		if( Start->m_fPoint[2] > Start->m_fPoint[3] && Next->m_fPoint[2] > Next->m_fPoint[3] ||
			Start->m_fPoint[2] < Start->m_fPoint[3] && Next->m_fPoint[2] < Next->m_fPoint[3] )
			continue ;
		// 两段直线求交,得到交点
		PNT3D intpt[3] ;
		if( !GetLineIntPt( Start, Next, intpt ) ) 	continue ;
		CSmtCutPointEx* pBreak = new CSmtCutPointEx() ;
		nc_DoubleToFloat( pBreak->m_fPoint, intpt[0], 3 ) ;
		nc_DoubleToFloat( pBreak->m_fSurfNor, intpt[1], 3 ) ;
		nc_DoubleToFloat( pBreak->m_fSurfPos, intpt[2], 2 ) ;
		pBreak->m_bType = SMART_CUTPNT_BREAK ;
		pPath->InsertAfter ( pBreak, Start ) ;
		Start = pBreak ;
	}
	// 以下部分被封于11-01-10，由lgq编写，插入的点不合理
	// 取两点中点，但其中一点已经不合理了，所以插入点基本不合理
/*	Start = (CSmtCutPointEx*)pPath->m_pHead ;
	for( ; Start ; Start = (CSmtCutPointEx*)Start->next )
	{
		Next = (CSmtCutPointEx*)Start->next ;
		if( !Next ) break ;
		// 如果都不是干涉点或者都是干涉点，走下一循环
        if (JudgeIverPt(Start) == JudgeIverPt(Next))
			continue;
		// 在干涉点和非干涉点之间插入临界点
		CSmtCutPointEx *pBreak = InsertBreakPnt(CheckMdl, bCheck, pPath, Start, Next);
		if (!pBreak)
			break;
		if (pBreak != Next)
			Start = pBreak;
	}*/
/*	Start = pPath->m_pHead ;
	while( Start )
	{
		Next = Start->next ;
		if( Start->m_bType == SMART_CUTPNT_NULL )
			pPath->DeletePoint ( Start ) ;
		Start = Next ;
	}
	CSmtCutPath* pNew = pPath->BreakAtNullPoint() ;
	delete pPath ;
	return pNew ;*/
	return pPath ;/**/
}

// 判断输入的点是否为内部点（即非干涉点）
BOOL CFlowlineGen::JudgeIverPt(CSmtCutPointEx *pPoint)
{
	if (NULL == pPoint)
		return FALSE;
	return pPoint->m_fPoint[2] + MIN_FLT > pPoint->m_fPoint[3];
}

// 用二分法查找路径干涉的临界点，并插入到路径中
CSmtCutPointEx *CFlowlineGen::InsertBreakPnt(CSmtCheckMdl& CheckMdl, BOOL bCheck, CSmtCutPath *pCutPath, 
											 CSmtCutPointEx *pStartPt, CSmtCutPointEx *pEndPt)
{
	// 参数合理性判断
	if (NULL == pStartPt || NULL == pEndPt)
		return NULL;
	CSmtCutPointEx *pBreakPt = NULL, *pMidPt = NULL;
	// 内部点和干涉点
	CSmtCutPointEx *pIverPt, *pVertPt;
	
	if (JudgeIverPt(pStartPt))
	{// 终点为干涉点
		pIverPt = pStartPt;
		pVertPt = pEndPt;
	}
	else
	{// 起点为干涉点
		pIverPt = pEndPt;
		pVertPt = pStartPt;
	}
    
	FPNT3D fBox[2] ;
	mathFDefBox3D( pStartPt->m_fPoint, pEndPt->m_fPoint, fBox, 0.1 ) ;
	CheckMdl.LabelCheckByBox ( fBox ) ;
	// 用二分法循环一般不会超过20次，否则程序出问题
	for (int i=0; i<20; i++)
	{
		// 得到中点
		pMidPt = GetMiddlePntEx(pIverPt, pVertPt, 0.5);
		pMidPt->m_fPoint[3] = pMidPt->m_fPoint[2];	// 把Z坐标临时存储于位置3
		if( bCheck )
			CheckMdl.DefineHeightEx ( pMidPt->m_fPoint, pMidPt->m_fSurfNor ) ;
		else
			CheckMdl.DefineHeight ( pMidPt->m_fPoint ) ;
		// 交换Z坐标
		TFLOAT temp = pMidPt->m_fPoint[2];
		pMidPt->m_fPoint[2] = pMidPt->m_fPoint[3];	
		pMidPt->m_fPoint[3] = temp;
		// 该点非干涉点
		if (JudgeIverPt(pMidPt))
		{
			// 内部点不是首末点，释放内存
			if (pIverPt != pStartPt && pIverPt != pEndPt)
				delete pIverPt;
			pBreakPt = pIverPt = pMidPt;
		}
		else
		{
			// 干涉点不是首末点，释放内存
			if (pVertPt != pStartPt && pVertPt != pEndPt)
				delete pVertPt;
			pVertPt = pMidPt;
		}
        
		// 精度是否合适
		if( mathFDist( pIverPt->m_fPoint, pVertPt->m_fPoint ) < 0.005 )
			break;
	}
	// 如果pBreakPt为空，说明输入的内部点pIverPt为干涉临界点，则不进行插点
	// 干涉点不是首末点，释放内存
	if (pVertPt != pStartPt && pVertPt != pEndPt)
		delete pVertPt;
	if (NULL == pBreakPt)
		return pStartPt;
	else
	{// 插入临界点
		pBreakPt->m_bType = SMART_CUTPNT_BREAK;
		pCutPath->InsertAfter(pBreakPt, pStartPt);
		return pBreakPt;
	}
}

CSmtCutPath* CFlowlineGen::BreakAndDelPnt ( CSmtCutPath* pPath)
{
	CSmtCPathList cList;
	CSmtCutPointEx *Start = NULL, *Next = NULL ;
	// 将pPath分解
	CSmtCutPath *pNewPath = new CSmtCutPath() ;
	pNewPath->m_bFeedType = pPath->m_bFeedType ;
	pNewPath->m_nLayerNo = pPath->m_nLayerNo ;
	pNewPath->m_nLineNo  = pPath->m_nLineNo ;
	Start = (CSmtCutPointEx*)pPath->m_pHead ;
	while( Start )
	{
		Next = (CSmtCutPointEx*)Start->next ;
		pPath->RemovePoint ( Start ) ;
		pNewPath->AddTail ( Start ) ;
		if( Start->m_bType == SMART_CUTPNT_BREAK )
		{
			CSmtCutPointEx *pnt = new CSmtCutPointEx() ;
			mathFCpyPnt( Start->m_fPoint, pnt->m_fPoint ) ;
			mathFCpyPnt( Start->m_fSurfNor, pnt->m_fSurfNor ) ;
			mathFCpyPnt( Start->m_fSurfPos, pnt->m_fSurfPos ) ;
			if( pNewPath->m_nNumPnt > 1 )
				cList.AddTail ( pNewPath ) ;
			else
				delete pNewPath ;
			pNewPath = new CSmtCutPath() ;
			pNewPath->m_bFeedType = pPath->m_bFeedType ;
			pNewPath->m_nLayerNo = pPath->m_nLayerNo ;
			pNewPath->m_nLineNo  = pPath->m_nLineNo ;
			pNewPath->AddTail ( pnt ) ;
		}
		Start = Next ;
	}
	if( pNewPath->m_nNumPnt > 1 )
		cList.AddTail ( pNewPath ) ;
	else
		delete pNewPath ;
	delete pPath ;
	// 判断点是否都是bValid类型
	POSITION pos, atpos ;
	pos = cList.GetHeadPosition () ;
	while( pos )
	{
		atpos = pos ;
		pPath = cList.GetNext ( pos ) ;
		if( IsAllInValidPnt( pPath ) )
		{
			delete pPath ;
			cList.RemoveAt ( atpos ) ;
		}
	}
	if (cList.IsEmpty() || cList.GetHead()==NULL)
	{
		return NULL;
	}
	CSmtCutPath* pNew = cList.GetHead();
	pos = cList.GetHeadPosition () ;
	cList.GetNext ( pos ) ;
	while( pos )
	{
		pPath = cList.GetNext ( pos ) ;
		if (pPath != NULL) 
		{
			pNew->next = pPath;
			pPath->prev = pNew;
			pNew = pPath;
		}
	}
	return cList.GetHead();
}
BOOL CFlowlineGen::IsAllInValidPnt ( CSmtCutPath* pPath )
{
	if( !pPath ) return TRUE ;
	int nCnt[3] = { 0, 0, 0 } ; // 分别为NULL,> 0, BREAK类型
	CSmtCutPoint *Start = pPath->m_pHead ;
	for( ; Start ; Start = Start->next )
	{
		if( Start->m_bType == SMART_CUTPNT_BREAK )
			nCnt[2]++ ;
		else if( Start->m_bType == SMART_CUTPNT_NULL )
			nCnt[0]++ ;
		else 
			nCnt[1]++ ;
	}
	// 无效的路径的条件:全是BREAK点,全是NULL点,全是NULL和BREAK点
	if( nCnt[0] == pPath->m_nNumPnt ||
		nCnt[2] == pPath->m_nNumPnt ||
		nCnt[0] + nCnt[2] == pPath->m_nNumPnt )
		return TRUE ;
	return FALSE ;
}


BOOL CFlowlineGen::CheckAllPathByCheckMdl ( CSmtCheckMdl& CheckMdl, CSmtCPathLib& AllPath, JDNC_TOL& Tol, 
									    BOOL bCheck, BOOL bInCheckBySelf, JDNC_PRGDEF& PrgDef, double dCur, int *pErrorType)
{
	if (NULL == pErrorType)
		return FALSE;
	m_pErrorType = pErrorType;
	BOOL bRet = TRUE;
	CSmartTool *pCheck = NULL, *pTool = NULL;
	if (bInCheckBySelf)
	{
		// 对DriveMdl的刀具作偏移处理
		double dTol = Tol.m_dArcTol * 2.5 ;
		pTool = CheckMdl.m_pTool ;
		pCheck = CheckMdl.m_pTool->Offset ( -dTol ) ;
		if( pCheck )
			CheckMdl.UpdateCurrTool ( pCheck ) ;
	}

	// 需要多线程的拷贝，则拷贝一份CheckMdl
	BOOL bLocalCopy = FALSE ;	
	if (MathCAM_IsNeedMultiCopy(CheckMdl, m_nCalcThreadNum))
	{
		bLocalCopy = TRUE ;
		CheckMdl.CreateMultiCopy(m_nCalcThreadNum - 1) ;
	}	

	// 支持多线程，即拷贝数据不为空
	INT_PTR nPathNum = AllPath.GetNumPath();
	CVecPointer NewPath;
	NewPath.resize(nPathNum + 5);
	ZeroMemory(&NewPath[0], sizeof(LPVOID) * (nPathNum + 5));
	if (nPathNum >=2 && CheckMdl.GetMultiCopy() != NULL)
	{
		LPVOID lpParam[NC_CFG_CPU_MAXNUM] = {NULL};
		CAPT_DATA ThreadData[NC_CFG_CPU_MAXNUM];
		JDNC_PRGDEF tmpPrg = PrgDef;
		tmpPrg.m_pPrgFunc = NULL ; 
		tmpPrg.m_pPosFunc = NULL ; 
		int nThreadNum = min(m_nCalcThreadNum, (int)nPathNum), k=1;
		for (int i=0; i<nThreadNum; i++)
		{
			ThreadData[i].pDriveMdl = i == 0 ? &CheckMdl : CheckMdl.GetMultiCopy(i-k); 
			if (ThreadData[i].pDriveMdl == NULL)
			{
				--i, --k, --nThreadNum;
				continue;
			}
			ThreadData[i].nAtCore = i;
			ThreadData[i].dCur = dCur;			
			ThreadData[i].bCheck = bCheck;
			ThreadData[i].pTol = &Tol;
			ThreadData[i].PrgDef = i==0 ? PrgDef : tmpPrg;
			ThreadData[i].AllPath = &AllPath;//guomin path[i]未赋值为空 12/09/12
			ThreadData[i].NewPath = &NewPath[0];
			ThreadData[i].pFlowlineGen = this;
			lpParam[i] = &ThreadData[i];
		}
		for (i=0; i<nThreadNum; i++)
		{
			ThreadData[i].nCoreNum = nThreadNum;
		}
		bRet = MathCAM_ThreadMainFunc(MathCAM_CheckAllPathByCheckMdlSubProc, lpParam, nThreadNum);
	}
	else
	{
		bRet = CheckAllPathByCheckMdlSubProc(CheckMdl, AllPath, Tol, PrgDef, dCur, bCheck, 0, 1, &NewPath[0]);
	}

	AllPath.m_cAllPath.RemoveAll();
	if (bRet)
	{
		Nc5D_AddPathArrayToLib(&NewPath[0], nPathNum, AllPath);
	}

	// DriveMdl的数据在本函数拷贝，则在本函数删除
	if( bLocalCopy == TRUE ) 
	{
		CheckMdl.DeleteMultiCopy() ;
	}

	if (bInCheckBySelf)
	{
		// 删除临时检查刀具，并更新到初始状态
		if( pCheck )
			delete pCheck ;
		CheckMdl.UpdateCurrTool ( pTool ) ;
	}

	return bRet;
}

// 干涉检查主函数
BOOL CFlowlineGen::CheckAllPathByCheckMdlSubProc(CSmtCheckMdl& CheckMdl, CSmtCPathLib& AllPath, JDNC_TOL& Tol, 
											 JDNC_PRGDEF& PrgDef, double dCur, BOOL bCheck, int nAtCore, int nCoreNum, LPVOID* NewPath)
{
	int nNum = AllPath.GetNumPath () ;
	if( nNum < 1 ) return TRUE;

	PrgDef.m_dTotalMove = dCur;							// 前进的总量
	PrgDef.m_dIncStep = PrgDef.m_dTotalMove / nNum ;	// 每次计算的前进量						
	PrgDef.m_dLimitAt = 1.0 ;							// 前进一次的计算量

	CSmtCutPath *pPath = NULL ;
	BOOL bRet = TRUE ;
	POSITION pos = AllPath.m_cAllPath.GetHeadPosition () ;
	int nIndex = 0;
	while( pos )
	{
		if( PrgDef.m_pBrkFunc && PrgDef.m_pBrkFunc () )
		{
			bRet = FALSE;
			if (m_pErrorType) *m_pErrorType = JDERROR_GENPATH_ABORT;
			break ;
		}
		MathCAM_MovePrgStep(PrgDef);
		pPath = AllPath.m_cAllPath.GetNext ( pos ) ;
		if(nIndex++ % nCoreNum != nAtCore || pPath==NULL)
		{
			continue ;
		}
		if( pPath->m_nNumPnt < 2 )
		{
			CSmtCutPoint* pCutPnt = pPath->m_pHead;
			while (pCutPnt)
			{
				CSmtCutPoint* pPntNext = pCutPnt->next;
				pPath->DeletePoint(pCutPnt);
				pCutPnt = pPntNext;		
			}
			pPath->ClearAllPoint();
			delete pPath;
			pPath = NULL;
			continue ;
		}
		pPath->DefineBox () ;
		// 首先判断本身加工的路径是否干涉,如果干涉,进行自动修剪连接
		CheckMdl.LabelCheckByBox( pPath->m_fBox ) ;

		// 对过切路径插入临界点，并对所有路径点进行路径点属性标记，
		// 然后通过BreakAndDelPnt将过切的点删除 qqs 2013.07.04
		InsertAndLabelCPoint( pPath, Tol.m_dMaxStep, &CheckMdl, bCheck) ;

		// 将得到的路径添加到组中
		if( pPath != NULL)
		{
			NewPath[nIndex-1]  = BreakAndDelPntNew(pPath );
		}		
	}
	if( PrgDef.m_pBrkFunc && PrgDef.m_pBrkFunc () )
	{
		Nc5D_DestroyMultiCalcData(NewPath, nIndex, &AllPath, pos, nCoreNum, nAtCore);	
		return FALSE ;
	}
	return TRUE ;
}

CSmtCutPath* CFlowlineGen::BreakAndDelPntNew ( CSmtCutPath* pPath)
{
	CSmtCPathList cList;
	CSmtCutPointEx *Start = NULL, *Next = NULL ;
	// 将pPath分解
	CSmtCutPath *pNewPath = new CSmtCutPath() ;
	pNewPath->m_bFeedType = pPath->m_bFeedType ;
	pNewPath->m_nLayerNo = pPath->m_nLayerNo ;
	pNewPath->m_nLineNo  = pPath->m_nLineNo ;
	Start = (CSmtCutPointEx*)pPath->m_pHead ;
	while( Start )
	{
		Next = (CSmtCutPointEx*)Start->next ;
		pPath->RemovePoint ( Start ) ;
		pNewPath->AddTail ( Start ) ;
		if( Start->m_bType == SMART_CUTPNT_BREAK )
		{
			CSmtCutPointEx *pnt = new CSmtCutPointEx() ;
			mathFCpyPnt( Start->m_fPoint, pnt->m_fPoint ) ;
			mathFCpyPnt( Start->m_fSurfNor, pnt->m_fSurfNor ) ;
			mathFCpyPnt( Start->m_fSurfPos, pnt->m_fSurfPos ) ;
			if( pNewPath->m_nNumPnt > 1 )
				cList.AddTail ( pNewPath ) ;
			else
				delete pNewPath ;
			pNewPath = new CSmtCutPath() ;
			pNewPath->m_bFeedType = pPath->m_bFeedType ;
			pNewPath->m_nLayerNo = pPath->m_nLayerNo ;
			pNewPath->m_nLineNo  = pPath->m_nLineNo ;
			pNewPath->AddTail ( pnt ) ;
		}
		Start = Next ;
	}
	if( pNewPath->m_nNumPnt > 1 )
		cList.AddTail ( pNewPath ) ;
	else
		delete pNewPath ;
	delete pPath ;
	// 判断点是否都是bValid类型
	POSITION pos, atpos ;
	pos = cList.GetHeadPosition () ;
	while( pos )
	{
		atpos = pos ;
		pPath = cList.GetNext ( pos ) ;
		if (!pPath) continue;

		if( IsAllInValidPnt( pPath ) )
		{
			delete pPath ;
			cList.RemoveAt ( atpos ) ;
			continue;
		}

		// 该部分用于删除第一步插入的路径节点
		CSmtCutPoint* pCutPnt = pPath->m_pHead;
		while(pCutPnt)
		{
			CSmtCutPoint* pNextPnt = pCutPnt->next;
			if (pCutPnt->m_bType == CUTPNT_INSERT_TMP) 
				pPath->DeletePoint(pCutPnt);

			pCutPnt = pNextPnt;
		}
	}
	if (cList.IsEmpty() || cList.GetHead()==NULL)
	{
		return NULL;
	}
	CSmtCutPath* pNew = cList.GetHead();
	pos = cList.GetHeadPosition () ;
	cList.GetNext ( pos ) ;
	while( pos )
	{
		pPath = cList.GetNext ( pos ) ;
		if (pPath != NULL) 
		{
			pNew->next = pPath;
			pPath->prev = pNew;
			pNew = pPath;
		}
	}
	return cList.GetHead();
}

BOOL CFlowlineGen::GetLineIntPt ( CSmtCutPointEx *Start, CSmtCutPointEx * Next, PNT3D intpt[3] )
{
	PNT3D p[4], p1, p2 ;
	for( int i = 0 ; i < 3 ; i++ )
	{
		p[0][i] = Start->m_fPoint[i] ;
		p[1][i] = Start->m_fPoint[i] ;
		p[2][i] = Next->m_fPoint[i] ;
		p[3][i] = Next->m_fPoint[i] ;
	}
	p[1][2] = Start->m_fPoint[3] ;
	p[3][2] = Next->m_fPoint[3] ;
	// p[0]p[2]为Line1,p[1]p[3]为Line2
	double dis, t1, t2 ;
	t1 = t2 = -MIN_LEN ;
	if( !mathIntLine( p[0], p[2], p[1], p[3], MIN_LEN, MIN_ANG, p1, p2, &t1, &t2, &dis ) )
	{
		return FALSE ;
	}
	if( t1 - 1.0 > MIN_DBL || t1 < -MIN_DBL || t2 - 1.0 > MIN_DBL || t2 < -MIN_DBL )
		return FALSE ;
	if( dis > 0.001 ) return FALSE ;
	mathCpyPnt( p1, intpt[0] ) ;
	double t = GetLineParam3D( p[0], p[2], p1 ) ;
	for( int i = 0 ; i < 3 ; i++ )
	{
		intpt[1][i] = Start->m_fSurfNor[i] + t * ( Next->m_fSurfNor[i] - Start->m_fSurfNor[i] ) ;
		intpt[1][i] = Start->m_fSurfPos[i] + t * ( Next->m_fSurfPos[i] - Start->m_fSurfPos[i] ) ;
	}
	return TRUE ;
}
double CFlowlineGen::GetLineParam2D ( PNT2D start, PNT2D end, PNT2D mid )
{
	double t = 0. ;
	if( fabs( start[1] - end[1] ) > fabs( start[0] - end[0] ) )
	{
		if( fabs( end[0] - start[1] ) > MIN_LEN )
            t = ( mid[1] - start[1] ) / ( end[1] - start[1] ) ;
		else
			t = -1. ;
	}
	else
	{
		if( fabs( end[0] - start[0] ) > MIN_LEN )
            t = ( mid[0] - start[0] ) / ( end[0] - start[0] ) ;
		else
			t = -1. ;
	}
	return t ;
}
double CFlowlineGen::GetLineParam3D( PNT3D start, PNT3D end, PNT3D mid ) 
{
	double d[3], t = -1. ;
	d[0] = fabs( start[0] - end[0] ) ;
	d[1] = fabs( start[1] - end[1] ) ;
	d[2] = fabs( start[2] - end[2] ) ;

	if( d[2] > d[0] && d[2] > d[1] && d[2] > MIN_LEN )
	{
		t = ( mid[2] - start[2] ) / ( end[2] - start[2] ) ;
	}
	else
	{
		t = GetLineParam2D( start, end, mid ) ;
	}
	return t ;
}
double CFlowlineGen::GetCutPointDist ( CSmtCutPoint* Start, CSmtCutPoint* End )
{
	PNT3D start, end ;
	for( int i = 0 ; i < 2 ; i++ )
	{
		start[i] = double( Start->m_fPoint[i] ) ;
		end[i] =   double( End->m_fPoint[i] ) ; 
	}
	start[2] = double( Start->m_fPoint[3] ) ;
	end[2]   = double( End->m_fPoint[3] ) ;
	return mathDist( start, end ) ;
}

void CFlowlineGen::InsertCPointEx( CSmtCutPath *pPath, double dStep )
{
	CSmtCutPointEx *pHead = NULL, *pNext = NULL, *pInsert = NULL ;
	double t = 0 ;
	pHead = (CSmtCutPointEx *)pPath->m_pHead ;
	while( pHead && (CSmtCutPointEx *)pHead->next  )
	{
		pNext = (CSmtCutPointEx *)pHead->next ;
		t = nc_Distance( pNext->m_fPoint, pHead->m_fPoint,2 ) ;
		int num = ( int )ceil( t / dStep ) ;
		if( num > 1000 ) num = 1000 ;
		if( num >= 2 ) 
		{
			
			t = 1.0 / num ; 
			for( int i = 1 ; i < num ; i ++ )
			{
				pInsert = GetMiddlePntEx( pHead, pNext, t * i ) ;
				if( pInsert )
				{
					pPath->InsertBefore( pInsert, pNext ) ;
				}
			}
		}
		pHead = pNext ; 
	}
}


BOOL CFlowlineGen::IsPntOverCut(CSmtCutPointEx CutPnt, CSmtCheckMdl *DriveMdl, BOOL bCheck, double tol)
{
	double height1, height2;
	height1 = CutPnt.m_fPoint[2];

	if( bCheck )
		DriveMdl->DefineHeightEx ( CutPnt.m_fPoint, CutPnt.m_fSurfNor ) ;
	else
		DriveMdl->DefineHeight ( CutPnt.m_fPoint ) ;

	height2 = CutPnt.m_fPoint[2];

	if (height1 + tol < height2)
		return TRUE;

	return FALSE;
}

void CFlowlineGen::InsertAndLabelCPoint( CSmtCutPath *pPath, double dStep, CSmtCheckMdl *DriveMdl, BOOL bCheck)
{
	CSmtCutPointEx *pHead = NULL, *pNext = NULL, *pInsert = NULL ;
	double t = 0 ;
	pHead = (CSmtCutPointEx *)pPath->m_pHead ;
	while( pHead && (CSmtCutPointEx *)pHead->next  )
	{
		pNext = (CSmtCutPointEx *)pHead->next ;
		t = nc_Distance( pNext->m_fPoint, pHead->m_fPoint,3 ) ;

		// 当开启消除马赛克时，路径上的节点是根据曲面平坦系数进行插入的，
		// 此时仍以曲面平坦系数进行插入来匹配
		if (m_cSetupDef.m_cModelTol.m_nMdlFlag & NCDEF_SETUP_NOMOSAIC)
			dStep = min(m_pMiller->m_fRadius, dStep);
		else
			dStep = min(m_pMiller->m_fRadius, 0.5);
		
		int num = ( int )ceil( t / dStep) ;
		if( num > 1000 ) num = 1000 ;
		// 插点越多，可以使计算结果越精确，但点数过多会影响计算速度
		if( num >= 2 ) 
		{
			t = 1.0 / num ; 
			for( int i = 1 ; i < num ; i ++ )
			{
				pInsert = GetMiddlePntEx( pHead, pNext, t * i ) ;
				if( pInsert )
				{
					pInsert->m_bType = CUTPNT_INSERT_TMP; // 用于标记插入点，最终需将插入点删除，防止路径节点过多
					pPath->InsertBefore( pInsert, pNext ) ;
				}
			}
		}
		pHead = pNext ; 
	}

	pHead = (CSmtCutPointEx *)pPath->m_pHead;	
	while(pHead && pHead->next)
	{
		CSmtCutPointEx* pInsert = NULL;
		pNext = (CSmtCutPointEx *) pHead->next;
		int bflag = CalInsertPnt(pHead, pNext, pInsert, DriveMdl, bCheck);

		if (bflag == 0) // 退出 
			break;
		if (bflag == 1) // 两点都过切
		{
			pInsert = GetMiddlePntEx( pHead, pNext, 0.5 ) ;
				
			if( pInsert )
			{
				if (IsPntOverCut(*pInsert, DriveMdl, bCheck))
				{
					pHead->m_bType = SMART_CUTPNT_BREAK;
					pNext->m_bType = SMART_CUTPNT_BREAK;
					delete pInsert;
					pHead = pNext;
				}
				else
				{
					pInsert->m_bType = CUTPNT_INSERT_TMP; // 用于标记插入点，最终需将插入点删除，防止路径节点过多
					pPath->InsertBefore( pInsert, pNext ) ;
				}
			}	
			continue;
		}
		if (bflag == 2) // 都不过切
		{
			pInsert = GetMiddlePntEx( pHead, pNext, 0.5 ) ;

			if( pInsert )
			{
				if (!IsPntOverCut(*pInsert, DriveMdl, bCheck))
				{
					if (pHead->m_bType != CUTPNT_INSERT_TMP)
					{
						pHead->m_bType = SMART_CUTPNT_IVER;
					}
					if (pNext->m_bType != CUTPNT_INSERT_TMP)
					{
						pNext->m_bType = SMART_CUTPNT_IVER;
					}
					delete pInsert;
					pHead = pNext;
				}
				else
				{
					pInsert->m_bType = CUTPNT_INSERT_TMP; // 用于标记插入点，最终需将插入点删除，防止路径节点过多
					pPath->InsertBefore( pInsert, pNext ) ;
				}
			}	
			continue;
		}
		if ( bflag == 3) // 当第一个点为过切点，第二个点为非过切点时
		{
			pHead->m_bType = SMART_CUTPNT_BREAK;
			if (pNext->m_bType != CUTPNT_INSERT_TMP)
			{
				pNext->m_bType = SMART_CUTPNT_IVER;
			}
		}
		if (bflag == 4) // 第一个点不过切，第二个点过切
		{
			if (pHead->m_bType != CUTPNT_INSERT_TMP)
			{
				pHead->m_bType = SMART_CUTPNT_IVER;
			}
			pNext->m_bType = SMART_CUTPNT_BREAK;
		}
		
		pPath->InsertBefore( pInsert, pNext ) ;
		pHead = pNext;			
	}
}

BOOL CFlowlineGen::CalInsertPnt(CSmtCutPointEx* pPathHead, CSmtCutPointEx* pPathNext, CSmtCutPointEx*& pInsert,CSmtCheckMdl* DriveMdl, BOOL bCheck)
{
	FPNT3D fBox[2] ;
	BOOL bFst = TRUE ;
	int bRet = FALSE;
	BOOL bFstChange = FALSE, bNextChange = FALSE;
	CSmtCutPath* tempPath = new CSmtCutPath;
	tempPath->AddTail(pPathHead->CopyMyself());
	tempPath->AddTail(pPathNext->CopyMyself());

	CSmtCutPointEx *pHead = (CSmtCutPointEx *)tempPath->m_pHead, *pNext = NULL ;
	pHead->m_fPoint[3] = pHead->m_fPoint[2] ;
	while( pHead )
	{
		pNext = (CSmtCutPointEx *)pHead->next ;
		if( pNext )
		{
			pNext->m_fPoint[3] = pNext->m_fPoint[2] ;
			mathFDefBox3D( pHead->m_fPoint, pNext->m_fPoint, fBox, 0.01 ) ;
			DriveMdl->LabelCheckByBox ( fBox ) ;

			if( bFst && IsPntOverCut(*pHead,DriveMdl,bCheck))
					bFstChange = TRUE;

			if (IsPntOverCut(*pNext,DriveMdl,bCheck))
				bNextChange = TRUE;
		}
		else
			break ;

		if( SurfNC_IsAbort()) break ;

		pHead->m_fPoint[2] = pHead->m_fPoint[3];
		pNext->m_fPoint[2] = pNext->m_fPoint[3];

		if (bFstChange == bNextChange && bFst) 
		{
			if(bFstChange == 1)	 bRet = 1; // 起末两点都过切
			if (bFstChange == 0) bRet = 2; // 起末两点不过切
			break;
		}
 		if (bFstChange == TRUE)	 bRet = 3; // 起点过切
		if (bNextChange == TRUE) bRet = 4; // 末点过切

		bFst = FALSE;
		
		if (bFstChange != bNextChange)
		{
			CSmtCutPointEx* tempInsert = NULL;
			tempInsert = GetMiddlePntEx( pHead, pNext, 0.5 ) ;
			if( tempInsert )
			{
				tempPath->InsertBefore( tempInsert, pNext ) ;
	
				if (GetCutPointDist(pHead, pNext) < 0.002)
				{
					if (IsPntOverCut(*tempInsert, DriveMdl, bCheck, MIN_FLT))
					{
						pInsert = (CSmtCutPointEx*)tempInsert->CopyMyself();
					}
					else if (bFstChange == FALSE)
					{
						pInsert = (CSmtCutPointEx*)pHead->CopyMyself();
					}
					else if (bNextChange == FALSE)
					{
						pInsert = (CSmtCutPointEx*)pNext->CopyMyself();
					}

					pInsert->m_bType = SMART_CUTPNT_BREAK;
					break;
				}
				bNextChange = FALSE;
				continue;
			}
		}

		bFstChange = bNextChange;
		bNextChange = FALSE;
		pHead = pNext ;
	}
	if (tempPath)
	{
		CSmtCutPoint* pCutPnt = tempPath->m_pHead;
		while (pCutPnt)
		{
			CSmtCutPoint* pPntNext = pCutPnt->next;
			tempPath->DeletePoint(pCutPnt);
			pCutPnt = pPntNext;		
		}
		tempPath->ClearAllPoint();
		delete tempPath;
		tempPath = NULL;
	}	
	return bRet;
}
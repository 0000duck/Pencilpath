#include "StdAfx.H"
#include "global.h"
#include "surfGeo.h"
#include "SurfNc.h"
#include "SmartNC.h"
#include "SysPrompt.h"
#include "SmtPathGen.H"
#include "SmtPathGen2D.H"
#include "SmtPathGen3D.H"
#include "SmtAutoFinishGen.H"
#include "SmtThreadMillGen.H"
#include "Nc3DStepAndSpiral.h"
#include "Mathcam.h"
#include "SmartWrap.H"
#include "SmartPathEx.h"
#include <vector>
#include "Nc5DToolAxis.h"
#include "SurfDraft.h"
#include "CrvPullToSurf.h"
#include "SurfEditor.h"
#include "PathSplFit.h"
//#include "Nc5axQuick.H"
#include "productDef.h"
#include "PathPointsDistribute.h"

#include "PathEtt.H"

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char BASED_CODE THIS_FILE[] = __FILE__;
#endif

#include "dog_main.h"

CJDToolGroup*	g_pCurrToolGroup=NULL;

void	MathCAM_SetCurrToolGroup(CJDToolGroup* pCurrToolGroup)
{
	g_pCurrToolGroup=pCurrToolGroup;
}

CJDToolGroup* MathCAM_GetCurrToolGroup()
{
	return g_pCurrToolGroup;
}
void MathCAM_AddPComb(CPathCombine* pPComb)
{	//添加PComb
    if(  ! pPComb ) return ;
    delete pPComb ;
    return ;
}
BOOL MathCAM_HasSameValue(double, int , int )
{
    return FALSE ;
}

void NcOffsetContour(CSmartLoop* pContour, JDEnum  OffType,
				DOUBLE  Dist, DOUBLE  IslOff ,  JDNC_COR& CorDef, CSmartLoop*& pResult)
{
/*	CSmartLoop*	AllLoop=NULL;
	double		dOffsetTol[7]={ 0, 0.01, -0.01, 0.02, -0.02, 0.03, -0.03 };

	for(CSmartLoop* p=pContour; p; p=p->next)
	{
		CSmartLoop*	pLoop=NULL;
		for(int i=0; i<7 && !pLoop; i++)
			pLoop=p->OffsetContour(OffType, Dist+dOffsetTol[i], IslOff+dOffsetTol[i], CorDef);
		AllLoop=Mini_AddContours(AllLoop, pLoop);
	}
	
	pResult=AllLoop;
*/

	double		dOffsetTol[7]={ 0, 0.01, -0.01, 0.02, -0.02, 0.03, -0.03 };
	double		dOffsetDist[2]={Dist, IslOff};
	pResult=NULL;
	for(int i=0; i<7 && !pResult; i++)
	{
		dOffsetDist[0]=Dist+dOffsetTol[i];
		dOffsetDist[1]=IslOff+dOffsetTol[i];
		pResult=Mini_OffsetContourList( pContour, OffType, dOffsetDist, CorDef);
	}

	for(CSmartLoop*	pLoop=pResult, *pNextLoop; pLoop; pLoop=pNextLoop)
	{//删除面积小于NCSF_TOL_01的环
		pNextLoop=pLoop->next;
		if( fabs(pLoop->m_dArea)<0.01 )
		{
			if(pLoop->prev) pLoop->prev->next=pNextLoop;
			else pResult=pNextLoop;

			if(pNextLoop) pNextLoop->prev=pLoop->prev;
			delete pLoop;
		}
	}
}
void MathCAM_SetZValue( CSmartCurve& Curve, DOUBLE ZStart, DOUBLE ZEnd )
{
    double dTotal = Curve.GetLength(), dZ1, dZ2 ;
    double dLen = 0.0 ;
    dZ1 = ZStart ;
    for( CSmartSect* pSect = Curve.m_pHead ; pSect ; pSect = pSect->next )
    {
        dLen += pSect->GetLength() ;
        if( pSect->next == NULL ) dZ2 = ZEnd ;
        else 
        {
            dZ2 = ZStart + ( ZEnd - ZStart ) * ( dLen / dTotal)  ;
        }
        pSect->SetZValue( dZ1, dZ2 ) ;
        dZ1 = dZ2 ;
    }
}

void MathCAM_SmoothLeadINOUT( CSmartCurve& Curve, DOUBLE ZStart, DOUBLE ZEnd/*, bool FlagArc = FALSE */, 
							 const double DisTol = 0.0, const double AngTol = 0.0)
{
	if ( !Curve.m_pHead )
	{
		return;
	}
	double  dZ1 = 0.0, dZ2 = 0.0, dTotal = Curve.GetLength() ;
	if ( abs( ZStart - ZEnd ) < 0.0001)
	{
		MathCAM_SetZValue( Curve, ZStart, ZEnd );
		return;
	}
	//轮廓切割沿轮廓光滑进退刀   20140814  xh
	double depth     = abs( ZStart - ZEnd ); //沿轮廓长度
	double r         = 0.0;
	bool  bScale     = FALSE;                //放缩标志
	double dScale[3] = { 1.0, 1.0, 1.0 };    //各坐标放缩比例

	//计算光滑圆弧半径，若dTotal / depth > sqrt(3.0),则按实际计算，否则按dTotal计算r，并在Y方向进行放缩
	if ( ( dTotal / depth ) - sqrt( 3.0 ) > 0.00001 )
	{
		r = ( depth * depth + dTotal * dTotal ) / ( 2 * depth );
	}
	else
	{
		bScale    = TRUE;
		r         = dTotal / sqrt(3.0) * 2;
		dScale[1] = 2 * depth / r;
	}
	PNT3D pStartP  = { dTotal, bScale ? r / 2 : depth , 0 };          //圆弧起点
	PNT3D pEndP    = { 0, 0, 0 };                                     //圆弧终点
	PNT3D pCenterP = { 0, r, 0 };                                     //圆心
	VEC3D vNormal  = { 0, 0, 1 };                                     //圆弧所在平面 
	CGeoArc * tempArc = new CGeoArc();                                //创建初始圆弧
	if ( !tempArc->CreateBy2PCen( 0, pStartP, pEndP, pCenterP, vNormal ) )
	{
		delete tempArc;
		return;
	}
	//CGeoCurve * DesCurve = tempCurve->Scale( EndP, dScale );        //放缩之后的目标曲线

	//离散圆弧曲线，获得离散段数，之后在原始曲线上插点
	CStrpt *pStrpt   = NULL ;
	int nNum         = 1 ;                                            //轮廓曲线分段数
	pStrpt           = tempArc->DiscreteToStrpt( DisTol, AngTol );
	int nDivNum      = pStrpt->m_np;
	double dAverLen  = dTotal / nDivNum;                              //按照理论圆弧离散段数平分轮廓曲线段所得的平均长度
	bool ReverseFlag = ( ZStart - ZEnd > 0.0001 ) ? FALSE: TRUE;      //判断进退刀路径

	//插点
	CSmartCurve   *first = NULL,    *second = NULL;
	CSmartSect *pCurSect = NULL, *pNextSect = NULL;
	pCurSect = Curve.m_pHead;
	while ( pCurSect )
	{
		pNextSect = pCurSect->next;
		double dSectLength = pCurSect->GetLength();
		double dRatio = dSectLength / dAverLen ;
		if ( dRatio - 1 > -0.0001 )
		{
			nNum = ( int )ceil( dRatio );
		}

		CSmartCurve *tempCurve = new CSmartCurve();
		CSmartSect  *pCopySect = NULL, *pLastSect  = pCurSect;
		tempCurve->AddSect( pCurSect->CopyMyself() );
			 
		//根据离散点个数在psect上均匀插点
		for ( int j = 0; j < nNum ; j++ )
		{
			tempCurve->BreakCurveAt( dSectLength / nNum , first, second );
			pCopySect = first->m_pHead->CopyMyself();
			Curve.InsertAfter( pCopySect, pLastSect );
			pLastSect = pCopySect;
			first->ClearAll();
			delete first;
			tempCurve->ClearAll();
			delete tempCurve;
			tempCurve = second;
		}
		Curve.RemoveSect( pCurSect );
		delete pCurSect;
		pCurSect = pNextSect;
		nNum = 1;                               //恢复初值
	}
	//改变Z坐标
	double dLen = 0.0;
	dZ1 = ZStart;
	double dZ0 = sqrt( r * r - dTotal * dTotal );	
	for( CSmartSect* pSect = Curve.m_pHead; pSect ; pSect = pSect->next )
	{
		dLen += pSect->GetLength() ;
		if( pSect->next == NULL ) dZ2 = ZEnd ;
		else 
		{
			if ( !ReverseFlag )
			{
				double tempLen = dTotal - dLen;
				dZ2 = ZStart - ( sqrt(r * r - tempLen * tempLen ) - dZ0 ) * dScale[1];
			}
			else
				dZ2 = ZStart + ( r - sqrt(r * r - dLen * dLen ) ) * dScale[1];
		}
		pSect->SetZValue( dZ1, dZ2 ) ;
		dZ1 = dZ2 ;
	}

	//释放内存空间
	delete tempArc;
	tempArc = NULL;
	pStrpt->Destroy();
	delete pStrpt;
	pStrpt = NULL;
}



void MathCam_DeleteAllRedepthCur( CPtrList &cListCur )
{
	CRedepthCurve *pDepCur = NULL ;
	POSITION pos = cListCur.GetHeadPosition (), atpos = NULL ;
	while( pos )
	{
		atpos = pos ;
		pDepCur = ( CRedepthCurve *)cListCur.GetNext ( pos ) ;
		if( pDepCur )
		{
			delete pDepCur ;
		}
		cListCur.RemoveAt ( atpos ) ;
	}
	cListCur.RemoveAll () ;
}

void MathCam_ReverseAllRedepthCur(CPtrList &ListCur)
{
	CRedepthCurve *pDepCur = NULL ;
	POSITION pos = ListCur.GetHeadPosition () ;
	while( pos )
	{
		pDepCur = ( CRedepthCurve *)ListCur.GetNext ( pos ) ;
		pDepCur->m_pCurve2D->Reverse();
		pDepCur->m_pCurve3D->Reverse();
	}
}

CPathLayerLoop::CPathLayerLoop(CPathEntity* HPEnt, CPathEntity* TPEnt, CSmartLoop* Loop)
{
	m_pHead = HPEnt;
	m_pTail = TPEnt;
	m_pLoop = Loop;
}

CPathLayerLoop::~CPathLayerLoop()
{
	if (m_pLoop)
	{
		Mini_DeleteContours(m_pLoop);
	}
}

BOOL CPathLayerLoop::IsValid()
{
	if (m_pHead && m_pTail && m_pLoop)
	{
		return TRUE;
	}
	return FALSE;
}

CSmartCurve* MathCam_ConvertPathEntToCur(CPathEntity* PathEnt)
{
	if (!PathEnt)
	{
		return NULL;
	}
	PNT2D ptSt, ptEd;
	CSmartCurve* pCur = NULL;
	int nType = PathEnt->GetType();
	if (nType == NC_PATH_LINE3D)
	{
		CPathLine3D* pLine = (CPathLine3D*)PathEnt;
		mathCpyPnt2D(pLine->m_fStart, ptSt);
		mathCpyPnt2D(pLine->m_fEnd, ptEd);
		CSmartLine* pSmtLine = new CSmartLine(ptSt, ptEd);
		pCur = new CSmartCurve();
		pCur->AddSect(pSmtLine);
	}
	else if (nType == NC_PATH_PLINE3D)
	{
		CPathPLine3D* pPLine3D = (CPathPLine3D*)PathEnt;
		pCur = new CSmartCurve();
		for (int i = 1; i <= pPLine3D->m_nCount; ++i)
		{
			mathCpyPnt2D(pPLine3D->m_pTAPos[i-1], ptSt);
			mathCpyPnt2D(pPLine3D->m_pTAPos[i], ptEd);
			CSmartLine* pSmtLine = new CSmartLine(ptSt, ptEd);
			pCur->AddSect(pSmtLine);
		}
	}
	else if (nType == NC_PATH_ARC3D)
	{
		CPathArc3D* pArc3D = (CPathArc3D*)PathEnt;
		pCur = new CSmartCurve();
		CSmartArc* pSmtArc = new CSmartArc(pArc3D->m_fCenter, pArc3D->m_fAngle, pArc3D->m_fRadius);
		pCur->AddSect(pSmtArc);
	}
	else
	{
		return NULL;
	}
	return pCur;
}

//获取CPathEntity的三维长度 20140901 xh
DOUBLE MathCam_PathEntityLen3D(CPathEntity* pEnt)
{
	if (!pEnt)
	{
		return NULL;
	}
	DOUBLE dLen3D = 0.0;
	TPNT3D pPStart, pPEnd;
	int nType = pEnt->GetType();
	if (nType == NC_PATH_LINE3D)
	{
		CPathLine3D* pLine = (CPathLine3D*)pEnt;
		pLine->GetEndPoint(0 , pPStart);
		pLine->GetEndPoint(1 , pPEnd);
		dLen3D = mathDist(pPStart, pPEnd);
	}
	else if (nType == NC_PATH_PLINE3D)
	{
		CPathPLine3D* pPLine3D = (CPathPLine3D*)pEnt;
		for (int i = 1; i <= pPLine3D->m_nCount; ++i)
		{
			mathCpyPnt(pPLine3D->m_pTAPos[i-1], pPStart);
			mathCpyPnt(pPLine3D->m_pTAPos[i], pPEnd);
			dLen3D += mathDist(pPStart, pPEnd);
		}
	}
	else if (nType == NC_PATH_ARC3D)
	{
		CPathArc3D* pArc3D = (CPathArc3D*)pEnt;
		DOUBLE dLen2D = pArc3D->GetLength();
		dLen3D = sqrt( pArc3D->m_fDepth * pArc3D->m_fDepth + dLen2D * dLen2D );
	}
	else
	{
		return -1;
	}
	return dLen3D;
}
CSmartCurve* MathCam_ConvertPathToCur(CPtrList &List)
{
	if (List.GetCount() < 1)
	{
		return NULL;
	}
	CSmartCurve* pCur = NULL, *pResultCur = NULL;
	CPathEntity* pEnt = NULL;
	for (POSITION pos = List.GetHeadPosition(); pos;)
	{
		pEnt = (CPathEntity*)List.GetNext(pos);
		if (!pEnt)
		{
			continue;
		}
		pCur = MathCam_ConvertPathEntToCur(pEnt);
		if (pCur)
		{
			if (!pResultCur)
			{
				pResultCur = new CSmartCurve;
			}
			pResultCur->AddCurve(pCur);
		}
	}
	return pResultCur;
}

void MathCam_DelOverlapSect(CSmartCurve* Curve)
{
	if (!Curve || !Curve->m_pHead)
	{
		return;
	}

	BOOL bFind = FALSE;
	PNT2D ptArr[4];
	CSmartSect*pPrevSect = NULL, * pSect = NULL, *pNextSect = NULL;
	for (pSect = Curve->m_pHead; pSect && pSect->next;)
	{
		bFind = FALSE;
		pNextSect = pSect;
		pNextSect = pNextSect->next;
		while (pNextSect)
		{
			pSect->GetStart(ptArr[0]);
			pSect->GetEnd(ptArr[1]);
			pNextSect->GetStart(ptArr[2]);
			pNextSect->GetEnd(ptArr[3]);
			if (mathDist2D(ptArr[0], ptArr[3]) < MIN_DIS &&
				mathDist2D(ptArr[1], ptArr[2]) < MIN_DIS)
			{
				bFind = TRUE;
				Curve->RemoveSect(pSect);
				delete pSect;
				Curve->RemoveSect(pNextSect);
				delete pNextSect;
				break;
			}
			else
			{
				pNextSect = pNextSect->next;
			}
		}
		if (!bFind)
		{
			pPrevSect = pSect;
			pSect = pSect->next;
		}
		else
		{
			if (!pPrevSect)
			{
				pSect = Curve->m_pHead;
			}
			else
			{
				pSect = pPrevSect->next;
			}
		}
	}
}

void MathCam_GetPathLayerLoop(CPathCombine* PComb, CPtrList &LayerLoop)
{
	TPNT3D ptArr[2];
	CPtrList list;
	CPathEntity* pEnt = NULL, *pNEnt = NULL;
	pEnt = PComb->m_pHead;
	while (pEnt)
	{		
		list.AddTail(pEnt);
		while (pEnt->next)
		{
			pNEnt = pEnt->next;
			pEnt->GetEndPoint(1, ptArr[0]);
			pNEnt->GetEndPoint(0, ptArr[1]);
			if (nc_Distance(ptArr[0], ptArr[1], 3) < MIN_DIS)
			{
				list.AddTail(pNEnt);
				pEnt = pNEnt;
				continue;
			}
			break;
		}
		CSmartCurve* pCur = MathCam_ConvertPathToCur(list);
		if (!pCur)
		{
			pEnt = pEnt->next;
			list.RemoveAll();
			continue;
		}
		// 删除重复段
		MathCam_DelOverlapSect(pCur);
		CPathLayerLoop* pCLayerLoop = new CPathLayerLoop((CPathEntity*)(list.GetHead()),
														 (CPathEntity*)(list.GetTail()),
														 pCur->FormLoop());
		LayerLoop.AddTail((void*)pCLayerLoop);

		if (pCur)
		{
			pCur->ClearAll();
			delete pCur;
			pCur = NULL;
		}

		pEnt = pEnt->next;
		list.RemoveAll();
	}
}

static double CalSurfPosExtendScale(CSmtCutPointEx* CPntEx0, CSmtCutPointEx* CPntEx1)
{
	if (!CPntEx0 || !CPntEx1)
	{
		return 1.0;
	}
	double dPntLen = 0.0, dSPntLen = 0.0, dScale = 1.0;
	dPntLen = nc_Distance(CPntEx0->m_fPoint, CPntEx1->m_fPoint, 3);
	dSPntLen = nc_Distance(CPntEx0->m_fSurfPos, CPntEx1->m_fSurfPos, 3);
	if (dPntLen > MIN_DIS)
	{
		dScale = dSPntLen/dPntLen;
	}
	if (dScale < MIN_DIS)
	{
		dScale = 1.0;
	}
	return dScale;
}

static void CalcLeadInPathSurfPos(CSmtCutPath* CutPath5x, CSmtCutPath* LeadIn)
{
	if (!CutPath5x || !LeadIn ||
		!CutPath5x->m_pHead || !CutPath5x->m_pHead->next ||
		!CutPath5x->m_pTail || !CutPath5x->m_pTail->prev)
	{
		return;
	}

	int i = 0;
	FPNT3D fVec;
	float fScale = 1.0f;
	CSmtCutPointEx* pCPntEx = NULL, *pPPntEx = NULL, *pNPntEx = NULL;
	CSmtCutPointEx* pHead = (CSmtCutPointEx*)CutPath5x->m_pHead;
	if (pHead)
	{			
		pNPntEx = (CSmtCutPointEx*)pHead->next;
		if (pNPntEx)
		{
			fScale = TFLOAT(CalSurfPosExtendScale(pHead, pNPntEx));
		}
		pCPntEx = (CSmtCutPointEx*)LeadIn->m_pTail;
		nc_VectorCopy(pCPntEx->m_fSurfPos, pHead->m_fSurfPos, 3);
		while (pCPntEx && pCPntEx->prev)
		{
			pPPntEx = (CSmtCutPointEx*)pCPntEx->prev;
			nc_VectorMinus(pPPntEx->m_fPoint, pCPntEx->m_fPoint, fVec, 3);
			for (i = 0; i < 3; ++i)
			{
				pPPntEx->m_fSurfPos[i] = pCPntEx->m_fSurfPos[i] + fVec[i] * fScale;
			}
			pCPntEx = pPPntEx;
		}
	}
}

static void CalcLeadOutPathSurfPos(CSmtCutPath* CutPath5x, CSmtCutPath* LeadOut)
{
	if (!CutPath5x || !LeadOut ||
		!CutPath5x->m_pHead || !CutPath5x->m_pHead->next ||
		!CutPath5x->m_pTail || !CutPath5x->m_pTail->prev)
	{
		return;
	}

	int i = 0;
	FPNT3D fVec;
	float fScale = 1.0f;
	CSmtCutPointEx* pCPntEx = NULL, *pPPntEx = NULL, *pNPntEx = NULL;
	CSmtCutPointEx* pTail = (CSmtCutPointEx*)CutPath5x->m_pTail;
	if (pTail)
	{
		pPPntEx = (CSmtCutPointEx*)pTail->prev;
		if (pPPntEx)
		{
			fScale = TFLOAT(CalSurfPosExtendScale(pPPntEx, pTail));
		}
		pCPntEx = (CSmtCutPointEx*)LeadOut->m_pHead;
		nc_VectorCopy(pCPntEx->m_fSurfPos, pTail->m_fSurfPos, 3);
		while (pCPntEx && pCPntEx->next)
		{
			pNPntEx = (CSmtCutPointEx*)pCPntEx->next;
			nc_VectorMinus(pCPntEx->m_fPoint, pNPntEx->m_fPoint, fVec, 3);
			for (i = 0; i < 3; ++i)
			{
				pNPntEx->m_fSurfPos[i] = pCPntEx->m_fSurfPos[i] + fVec[i] * fScale;
			}
			pCPntEx = pNPntEx;
		}
	}
}

CSmartGraphic::CSmartGraphic() 
{
	m_pAtClass = NULL ; 
    m_pAtPComb = NULL ;
	m_pGetAllEnt  = NULL ; 
	m_pGetFeaturePoint = NULL ;
	m_pGet5AxisPoint   = NULL ;
	m_pGet5AxisObject  = NULL ;
	m_pGetSurfPoint = NULL ;
	m_bFindFlag     = 0    ;
	m_pHasCheckSurf = NULL;
	m_pBuildCheckMdl = NULL ; 
    m_pCreateCheckMdl = NULL ;
	m_pBuildStockMdl = NULL ;
	m_pBuildProjectMdl = NULL ;
	m_pBuildImpStockMdl = NULL;
	m_pGetProjectSurfTop = NULL ;
	m_pGetGeoSurf	 = NULL ;
	m_pGetBitmapInfo = NULL ; 
	m_pGetTopHeight  = NULL ;
	m_pGetEntBox     = NULL ;
	m_pHasRoughSurf  = NULL ;
	m_pGetRoughType  = NULL ;
	m_pGetRoughStock = NULL ;
	m_pHasPlaneSurf  = NULL ;
	m_pGetTPlaneFrame= NULL ;
	m_pFindNetSurf   = NULL ;
	m_pGetStockBox	 = NULL ;
	m_pGetShapeRange = NULL ;
	m_pGetAllPathBefore = NULL ; 
	m_pBuildRulSurfMdl = NULL ;
	m_pBuildExtendSurfMdl = NULL ;
	m_pGetGuideCurve = NULL ;
	m_pGetAllCurveAndVerticalPath = NULL ;

	//guomin
	m_pBuildCheckMdlnew = NULL ;
}
CSmartGraphic::~CSmartGraphic()
{
}

BOOL CSmartGraphic::GetAllEntity( JDNC_SETUP& Setup, 
								  CPtrList& AllEnt ,
								  int   nType      ,
                                  RFRAME* PrjMtx   )
{
	if( ! m_pGetAllEnt || ! m_pAtClass) 
	{
		return FALSE ;
	}
	(*m_pGetAllEnt)( m_pAtClass, Setup, nType, AllEnt, PrjMtx ) ;
	if( AllEnt.GetCount() == 0 ) return FALSE ;
	return TRUE ;
}

BOOL CSmartGraphic::GetAll5XPoint(  JDNC_SETUP& Setup , 
								    CPtrList& AllEnt , 
									CPtrList& AllEnt5X ) 
{
	if( ! m_pGet5AxisPoint || ! m_pAtClass) 
	{
		return FALSE ;
	}
	(*m_pGet5AxisPoint)( m_pAtClass, Setup, AllEnt, AllEnt5X ) ;
	if( AllEnt.GetCount() + AllEnt5X.GetCount() == 0 ) return FALSE ;
	return TRUE ;
}

BOOL CSmartGraphic::GetAll5XObject(  JDNC_SETUP& Setup , int nType, void* pData ) 
{
	if( ! m_pGet5AxisObject || ! m_pAtClass) 
	{
		return FALSE ;
	}
	return (*m_pGet5AxisObject)( m_pAtClass, Setup, nType, pData ) ;
}

CSmartLoop* CSmartGraphic::GetAllContour( JDNC_SETUP& Setup, BOOL Redepth ) 
{
	CPtrList AllCont ;
	int bEntType = SMARTGRAPH_TYPE_CONTOUR ;
	if( Redepth ) 
	{
		bEntType = SMARTGRAPH_TYPE_CONTOUR3D ;
	}
	if( ! GetAllEntity( Setup , AllCont, bEntType ) ) 
	{
		return NULL ; 
	} 
	CSmartLoop* AllLoop = NULL ; 
	POSITION pos = AllCont.GetHeadPosition() ;
	while( pos ) 
	{
		CSmartLoop* pLoop = (CSmartLoop*)AllCont.GetNext( pos ) ;
		pLoop->next = pLoop->prev = NULL ;
		AllLoop = Mini_AddContours( AllLoop, pLoop ) ;
	}
    CSmartLoop  LoopTmp ;
    AllLoop = LoopTmp.ResortContour( AllLoop, Setup.m_cOrderDef.m_nSortType , FALSE) ;
	AllCont.RemoveAll() ;
	return AllLoop ; 
}
/*
CSmartLoop* CSmartGraphic::GetAllPartContour( JDNC_SETUP& Setup, BOOL Redepth ) 
{
	CPtrList AllCont ;
	int bEntType = SMARTGRAPH_TYPE_CONTOUR ;
	if( Redepth ) 
	{
		bEntType = SMARTGRAPH_TYPE_CONTOUR3D ;
	}
	if( ! GetAllEntity( Setup , AllCont, bEntType ) ) 
	{
		return NULL ; 
	} 
	CSmartLoop* AllLoop = NULL ; 
	POSITION pos = AllCont.GetHeadPosition() ;
	while( pos ) 
	{
		CSmartLoop* pLoop = (CSmartLoop*)AllCont.GetNext( pos ) ;
		pLoop->next = pLoop->prev = NULL ;
		AllLoop = Mini_AddContours( AllLoop, pLoop ) ;
	}
	AllCont.RemoveAll() ;
	return AllLoop ; 
}

CSmartLoop* CSmartGraphic::GetAllStockBnd( JDNC_SETUP& Setup ) 
{
	CPtrList AllCont ;
	int bEntType = SMARTGRAPH_TYPE_STOCKBND ;

	if( ! GetAllEntity( Setup , AllCont, bEntType ) ) 
	{
		return NULL ; 
	} 
	CSmartLoop* AllLoop = NULL ; 
	POSITION pos = AllCont.GetHeadPosition() ;
	while( pos ) 
	{
		CSmartLoop* pLoop = (CSmartLoop*)AllCont.GetNext( pos ) ;
		pLoop->next = pLoop->prev = NULL ;
		AllLoop = Mini_AddContours( AllLoop, pLoop ) ;
	}
	
	AllCont.RemoveAll() ;
	return AllLoop ; 
}
*/
CSmartLoop* CSmartGraphic::GetRoughContour( JDNC_SETUP& Setup ) 
{
	CPtrList AllCont ;
	int bEntType = SMARTGRAPH_TYPE_ROUTHCONTOUR ;
	
	if( ! GetAllEntity( Setup , AllCont, bEntType ) ) 
	{
		return NULL ; 
	} 
	CSmartLoop* AllLoop = NULL ; 
	POSITION pos = AllCont.GetHeadPosition() ;
	while( pos ) 
	{
		CSmartLoop* pLoop = (CSmartLoop*)AllCont.GetNext( pos ) ;
		pLoop->next = pLoop->prev = NULL ;
		AllLoop = Mini_AddContours( AllLoop, pLoop ) ;
	}
    CSmartLoop  LoopTmp ;
    AllLoop = LoopTmp.ResortContour( AllLoop, Setup.m_cOrderDef.m_nSortType , FALSE) ;
	AllCont.RemoveAll() ;
	return AllLoop ; 
}

BOOL CSmartGraphic::GetGuideCurve ( JDNC_SETUP& Setup, 
																 CPtrList& AllEnt   )
{
	if( ! m_pGetGuideCurve || ! m_pAtClass) 
	{
		return FALSE ;
	}
	(*m_pGetGuideCurve)( m_pAtClass, Setup, AllEnt ) ;
	if( AllEnt.GetCount() == 0 ) return FALSE ;
	return TRUE ;
}


BOOL CSmartGraphic::GetAllRemainPath( JDNC_SETUP& Setup , CPtrList& AllEnt, double* dMinStock )
{
	if( ! m_pGetAllPathBefore || ! m_pAtClass) 
	{
		return FALSE ;
	}
	return (*m_pGetAllPathBefore)( m_pAtClass, Setup, AllEnt, m_pAtPComb, dMinStock ) ;
}

BOOL CSmartGraphic::GetBitmapInfo( BITMAP_INFO& BmpInfo ) 
{
	if( ! m_pGetBitmapInfo || ! m_pAtClass) 
	{
		return FALSE ;
	}
	return (*m_pGetBitmapInfo)( m_pAtClass, BmpInfo) ;
}
// 获得特征点
BOOL CSmartGraphic::GetFeaturePoint( JDNC_SETUP& Setup, CPtrList& AllEnt, JDNC_POINT& Point) 
{
	if( ! m_pGetFeaturePoint || ! m_pAtClass) 
	{
		return FALSE ;
	}
	return (*m_pGetFeaturePoint)( m_pAtClass, Setup, AllEnt, Point ) ;
}
// 获得特征点
BOOL  CSmartGraphic::GetSurfaceTopHeight( int MethodType, int SurfMode, DOUBLE& TopHeight ) 
{
	if( ! m_pGetTopHeight || ! m_pAtClass) 
	{
		return FALSE ;
	}
	return (*m_pGetTopHeight)( m_pAtClass, MethodType, SurfMode, TopHeight ) ;
}
// 获得投影基面高度
BOOL CSmartGraphic::GetProjectSurfTop( double &dTop )
{
	if( !m_pGetProjectSurfTop || !m_pAtClass )
	{
		return FALSE ;
	}
	return ( *m_pGetProjectSurfTop )( m_pAtClass, dTop ) ;
}
// 获得毛胚包围盒
BOOL CSmartGraphic::GetStockBox ( BOX3D& box )
{
	if( ! m_pGetStockBox || ! m_pAtClass) 
	{
		return FALSE ;
	}
	return (*m_pGetStockBox)( m_pAtClass, box ) ;
}
// 获得加工范围信息
BOOL CSmartGraphic::GetShapeRange ( CJDMethod &cParam, double &dTop, double &dCutDepth )
{
	if( !m_pGetShapeRange || !m_pAtClass )
	{
		return FALSE ;
	}
	return ( *m_pGetShapeRange )( m_pAtClass, cParam, dTop, dCutDepth ) ;
}
// 获得特征点
BOOL  CSmartGraphic::GetEntBox( DOUBLE dBox[2][3], int nType ) 
{
	if( ! m_pGetEntBox || ! m_pAtClass) 
	{
		return FALSE ;
	}
	return (*m_pGetEntBox)( m_pAtClass, dBox, nType ) ;
}

BOOL CSmartGraphic::FindNetSurf( int nSurfMode ) 
{
//	return m_bFindFlag & SMTFIND_NETSURF ? TRUE : FALSE ;
	if( ! m_pFindNetSurf || !m_pAtClass )
		return FALSE ;
	return (*m_pFindNetSurf)( m_pAtClass, nSurfMode ) ;
}
BOOL CSmartGraphic::GetAllSurface( JDNC_SETUP& Setup, C3DSurfArray& AllSurf )
{
	CPtrList AllEnt ;
    if( ! GetAllEntity( Setup, AllEnt, SMARTGRAPH_TYPE_GEOSURF ) ) 
	{
		return FALSE ;
	}
	POSITION pos = AllEnt.GetHeadPosition() ;
	while( pos )
	{
		AllSurf.Add( (CGeoSurf*)AllEnt.GetNext( pos ) ) ;
	}
	return TRUE ; 
}
BOOL CSmartGraphic::GetRoughSurface( JDNC_SETUP& Setup, C3DSurfArray& AllSurf )
{
	CPtrList AllEnt ;
    if( ! GetAllEntity( Setup, AllEnt, SMARTGRAPH_TYPE_ROUGHSURF ) ) 
	{
		return FALSE ;
	}
	POSITION pos = AllEnt.GetHeadPosition() ;
	while( pos )
	{
		AllSurf.Add( (CGeoSurf*)AllEnt.GetNext( pos ) ) ;
	}
	return TRUE ; 
}
BOOL CSmartGraphic::HasCheckSurf()
{
	if( !m_pHasCheckSurf || !m_pAtClass ) return FALSE ;
	return (* m_pHasCheckSurf)(m_pAtClass);
}

BOOL CSmartGraphic::HasRoughSurf ()
{
	if( !m_pHasRoughSurf || !m_pAtClass ) return FALSE ;
	return (* m_pHasRoughSurf )( m_pAtClass ) ;
}
int CSmartGraphic::GetRoughType ()
{
	if( !m_pGetRoughType || !m_pAtClass ) return FALSE ;
	return (* m_pGetRoughType )( m_pAtClass ) ;
}
int CSmartGraphic::SetRoughType ( int nType )
{
	if( !m_pSetRoughType || !m_pAtClass ) return FALSE ;
	return (* m_pSetRoughType )( m_pAtClass, nType ) ;
}
double CSmartGraphic::GetRoughStock ()
{
	if( !m_pGetRoughStock || !m_pAtClass ) return 0. ;
	return ( *m_pGetRoughStock )( m_pAtClass ) ;
}
BOOL CSmartGraphic::HasPlaneSurf ( CPtrArray* AllSurf, int nType, double dTol )
{
	if( !m_pHasPlaneSurf || !m_pAtClass ) return FALSE ;
	return( *m_pHasPlaneSurf )( m_pAtClass, AllSurf, nType, dTol ) ;
}
BOOL CSmartGraphic::GetTPlaneFrame ( RFRAME& lf ) 
{
	if( !m_pGetTPlaneFrame || !m_pAtClass ) return NULL ;
	return(*m_pGetTPlaneFrame)(m_pAtClass, lf ) ;
}
// 构建检查模型
BOOL CSmartGraphic::BuildCheckModel( JDNC_SETUP& Setup, CSmtCheckMdl& CheckMdl, int nSurfMode ) 
{
	if( ! m_pBuildCheckMdl || ! m_pAtClass ) return FALSE ;
	return  (*m_pBuildCheckMdl)( m_pAtClass, Setup,  CheckMdl,nSurfMode ) ;
}

//guomin
BOOL CSmartGraphic::BuildCheckModelnew( JDNC_SETUP& Setup   , CSmtCheckMdl& CheckMdl,CGeoNetSurf *&pRectSurf, int nSurfMode)
{
	if( ! m_pBuildCheckMdlnew || ! m_pAtClass ) return FALSE ;
	return  (*m_pBuildCheckMdlnew)( m_pAtClass, Setup,  CheckMdl,nSurfMode,pRectSurf ) ;
}

BOOL CSmartGraphic::BuildCheckModelByMask( JDNC_SETUP& Setup, CSmtCheckMdl& CheckMdl, int JDSelEntMask, JDNC_PRGDEF& PrgDef ) 
{
	if( ! m_pCreateCheckMdl || ! m_pAtClass ) return FALSE ;
	return  (*m_pCreateCheckMdl)( m_pAtClass, Setup,  CheckMdl,JDSelEntMask,PrgDef ) ;
}

// 构建毛坯模型
BOOL CSmartGraphic::BuildStockModel ( JDNC_SETUP &Setup, CSmtCheckMdl &StockMdl, BOX3D &box )
{
	if( !m_pBuildStockMdl || !m_pAtClass ) return FALSE ;
	return ( *m_pBuildStockMdl ) ( m_pAtClass,
							       Setup,
								   StockMdl,
								   box ) ;
}
// 构建投影变换模型
BOOL CSmartGraphic::BuildProjectModel( JDNC_SETUP &Setup, CSmtCheckMdl &ProjMdl )
{
	if( !m_pBuildProjectMdl || !m_pAtClass ) return FALSE ;
	return ( *m_pBuildProjectMdl )( m_pAtClass,
									Setup,
									ProjMdl ) ;
}
// 构建叶轮毛坯模型
BOOL CSmartGraphic::BuildImpStockModel( JDNC_SETUP &Setup, CSmtCheckMdl &StockMdl, BOX3D &box )
{
	if( !m_pBuildImpStockMdl || !m_pAtClass ) return FALSE ;
	CPtrArray AllSurf ;
	CreateImpStockSurf( Setup, AllSurf ) ;
	if( AllSurf.GetCount() == 0 ) return FALSE ;
	return ( *m_pBuildImpStockMdl ) ( m_pAtClass,
								   Setup,
								   StockMdl,
								   box,
								   AllSurf) ;
}
// 建立指定曲面的模型（两曲线侧铣加工时使用）
BOOL CSmartGraphic::BuildWallMdlBy2Curves( JDNC_SETUP &Setup, CSmtCheckMdl &WallMdl, CGeoCurve* pTopCurve, CGeoCurve* pBtmCurve )
{
	if( !m_pBuildRulSurfMdl || !pTopCurve || !pBtmCurve )  return FALSE ;
	return (*m_pBuildRulSurfMdl)( m_pAtClass, 
								  Setup, 
								  WallMdl,
							      pTopCurve,
								  pBtmCurve ) ;
}
// 得到所有曲线和竖直的曲线路径
BOOL CSmartGraphic::GetAllCurveAndVerticalPath( JDNC_SETUP& Setup, CPtrList& AllCurve, CPtrList& AllComb )
{
	if( !m_pGetAllCurveAndVerticalPath ) return FALSE ;
	return (*m_pGetAllCurveAndVerticalPath)( m_pAtClass, 
								             Setup, 
								             AllCurve,
								             AllComb ) ;
}

// 建立加工面的延伸面模型(等高精加工使用2011.04.13)
BOOL CSmartGraphic::BuildExtendSurfMdl( JDNC_SETUP &Setup, CSmtCheckMdl &ExtndSurfMdl, double dExtndDist )
{
	if( !m_pBuildExtendSurfMdl ) return FALSE ;
	// 生成延伸面
	CPtrArray AllSurf, AllExtndSurf ;
	GetGeoSurf( Setup, AllSurf, NCDEF_SURF_DRIVE ) ;
	ExtendDriveSurfaces( AllSurf, dExtndDist, AllExtndSurf ); 
	// 生成模型
	BOOL bRet =  (*m_pBuildExtendSurfMdl)( m_pAtClass,
									       Setup,
									       ExtndSurfMdl,
									       AllExtndSurf ) ;
	INT_PTR nSize = AllExtndSurf.GetSize() ;
	for ( INT_PTR i = nSize -1; i >= 0 ; i--)
	{//释放内存
		CGeoTrmSurf* pSurf = static_cast<CGeoTrmSurf*>( AllExtndSurf.GetAt(i)) ;
		AllExtndSurf.RemoveAt( i ) ;
		delete pSurf ;
	}
	return bRet ;
}
//延伸加工面保存到AllExtndSurf里
BOOL CSmartGraphic::ExtendDriveSurfaces( CPtrArray &AllSurf, double dExtndDist, CPtrArray &AllExtndSurf )
{
	if( AllSurf.GetSize() == 0 )
	{
		return FALSE ;
	}
	CSurfEditor     editor ;
	SURFEXTEND_PARAM se ;
	se.m_nMethod = 0 ;
	se.m_dLength = dExtndDist ;
	se.m_nBound = 1 ;
	se.m_bSameSurf = FALSE ;
	se.m_bSameAtri = TRUE ;
	INT_PTR nSize = AllSurf.GetSize() ;
	for ( INT_PTR i = 0; i < nSize; i++ )
	{
		CGeoTrmSurf *pTrmSurf = static_cast<CGeoTrmSurf*>( AllSurf.GetAt(i) ); 
		CTrmFin* pTrmFin = pTrmSurf->m_pLoop->m_pFinHead ;
		while( pTrmFin )
		{
			BOX3D Box; 
			CTrmEdge* pEdge = pTrmFin->m_pEdge ;
			if( pEdge == NULL ) break ;
			pEdge->UpdateBox(&Box) ;
			double dMin = Box.min[2] ;
			double dMax = Box.max[2] ;
			if ( dMax - dMin < 2.0e-2 )
			{
				CGeoTrmSurf * pNewSurf = editor.SurfExtend( pTrmSurf, pEdge , &se ) ;
				AllExtndSurf.Add( pNewSurf ) ;
			}
			pTrmFin = static_cast<CTrmFin*>(pTrmFin->m_pNext) ;
		}	
	}
	return TRUE ;
}

// 生成叶轮毛坯面
void CSmartGraphic::CreateImpStockSurf( JDNC_SETUP &Setup, CPtrArray & AllSurf )
{
	// STEP 1 : 获得原始曲面
	C5XGraph allGraph;
	GetAll5XObject(Setup, 0, &allGraph);
	CGeoTrmSurf *topGeo = NULL ;
	CGeoTrmSurf *btmGeo = NULL ;
	for(POSITION pos = allGraph.m_pPtrSurface.GetHeadPosition(); pos != NULL;)
	{
		CGeoSurf* pSurf = (CGeoSurf*)allGraph.m_pPtrSurface.GetNext(pos);
		if( pSurf && pSurf->GetType() == OBJ3D_SURFACE_GEO )
		{
			if( pSurf->nExtData == JDSELENT_IMPELLER_SHROUD )
			{
				topGeo = (CGeoTrmSurf*)pSurf ;
			}
			else if( pSurf->nExtData == JDSELENT_IMPELLER_HUB )
			{
				btmGeo = (CGeoTrmSurf*)pSurf ;
			}
		}
	}
	if( topGeo == NULL || btmGeo == NULL )  return ;
	// STEP 2 : 计算等参数曲线
	CGeoCurve *pRvBtm = NULL , *pRvTop = NULL ;
	PNT3D dStart, dEnd, dMid ;
	btmGeo->GetPoint( 0.0, 0.0, dStart ) ;
	btmGeo->GetPoint( 1.0, 0.0, dEnd   ) ;
	btmGeo->GetPoint( 0.0, 1.0, dMid   ) ;
	if( fabs( dStart[2] - dEnd[2] ) > fabs( dStart[2] - dMid[2] ) )
	{
		pRvBtm = btmGeo->GenIsoCurve( SURF_UDIR, 0.0 ) ;
	}
	else
	{
		pRvBtm = btmGeo->GenIsoCurve( SURF_WDIR, 0.0 ) ;
	}
	topGeo->GetPoint( 0.0, 0.0, dStart ) ;
	topGeo->GetPoint( 1.0, 0.0, dEnd   ) ;
	topGeo->GetPoint( 0.0, 1.0, dMid   ) ;
	if( fabs( dStart[2] - dEnd[2] ) > fabs( dStart[2] - dMid[2] ) )
	{
		pRvTop =topGeo->GenIsoCurve( SURF_UDIR, 0.0 ) ;
	}
	else
	{
		pRvTop = topGeo->GenIsoCurve( SURF_WDIR, 0.0 ) ;
	}
	if( pRvBtm == NULL || pRvTop == NULL ) 
	{
		if( pRvBtm ) delete pRvBtm ;
		if( pRvTop ) delete pRvTop ;
		return  ;
	}
	// STEP 3 : 调整曲线方向，使得曲线从下向上
	pRvBtm->GetPoint( 0.0, dStart ) ;
	pRvBtm->GetPoint( 1.0, dEnd  ) ;
	if( dStart[2] > dEnd[2] ) pRvBtm->Reverse() ;
	pRvTop->GetPoint( 0.0, dStart ) ;
	pRvTop->GetPoint( 1.0, dEnd  ) ;
	if( dStart[2] > dEnd[2] ) pRvTop->Reverse() ;
	// STEP 4 : 调整曲线的方位角对齐
	PNT3D dPivot = { 0.0, 0.0, 0.0 } ;
	VEC3D dZDir  = { 0.0, 0.0, -1.0} ;
	pRvBtm->GetPoint( 0.0, dStart ) ;
	pRvTop->GetPoint( 0.0, dEnd  ) ;
	dStart[2] = dEnd[2] = 0.0 ;
	nc_Normalize( dStart, 2 ) ;
	nc_Normalize( dEnd, 2 ) ;
	double dAng = nc_ACos( nc_OProduct( dStart, dEnd, 2 )) ;
	if( fabs( dAng ) > 1.0e-4 )
	{
		VEC3D dAxis ;
		nc_VProduct( dStart, dEnd, dAxis ) ;
		if( dAxis[2] < 0.0 ) dAng = -dAng ;
		RFRAME  lf;
		mathInitRFrame( &lf ) ;
		mathRotateRFrame( dPivot, dZDir, dAng, &lf) ;
		pRvTop->TransformLocal( &lf ) ;
	}
    // SETP 5 : 得到流线上的起末点
	PNT3D dStart2, dEnd2 ;
	pRvTop->GetPoint( 0.0, dStart );
	pRvTop->GetPoint( 1.0, dEnd );
	pRvBtm->GetPoint( 0.0, dStart2 );
	pRvBtm->GetPoint( 1.0, dEnd2 );
	// STEP 6 : 得到旋转面
	PNT3D tmpPnt = {0.0, 0.0, 0.0 } ;
	double dAngle[2] = { 0.0, MiniPai2 } ;
	CGeoTrmSurf *pTrmSurf = NULL ;
	CRevSur* pSurf = NULL ;
	CGeoLine * pLine = NULL ;
	//(i)得到顶盖
	if ( ( dEnd[2] - dEnd2[2] ) < 1.0e-4 )
	{
		tmpPnt[2] = dEnd2[2] ;
		pLine = new CGeoLine( dEnd2, tmpPnt ) ;
		pSurf = new CRevSur( pLine, dPivot, dZDir, dAngle ) ;
		if ( pSurf )
		{
			pTrmSurf = new CGeoTrmSurf( pSurf ) ;
			AllSurf.Add ( pTrmSurf ) ;
		}
		pLine = new CGeoLine( dEnd, dEnd2 ) ;
		pSurf = new CRevSur( pLine, dPivot, dZDir, dAngle ) ;
		if ( pSurf )
		{
			pTrmSurf = new CGeoTrmSurf( pSurf ) ;
			AllSurf.Add( pTrmSurf ) ;
		}
	}
	else
	{
		tmpPnt[2] = dEnd[2] ;
		pLine = new CGeoLine( dEnd, tmpPnt ) ;
		pSurf = new CRevSur( pLine, dPivot, dZDir, dAngle ) ;
		if ( pSurf )
		{
			pTrmSurf = new CGeoTrmSurf( pSurf ) ;
			AllSurf.Add( pTrmSurf ) ;
		}
	}
	//(ii)得到侧面
	pSurf = new CRevSur( pRvTop , dPivot, dZDir, dAngle ) ;
	if ( pSurf )
	{
		pTrmSurf = new CGeoTrmSurf( pSurf ) ;
		AllSurf.Add( pTrmSurf ) ;
	}
	//(iii)得到底面
	if( ( dStart[2] - dStart2[2] ) > 1.0e-4 )
	{
		pLine = new CGeoLine( dStart, dStart2 ) ;
		pSurf = new CRevSur( pLine, dPivot, dZDir, dAngle ) ;
		if ( pSurf )
		{
			pTrmSurf = new CGeoTrmSurf( pSurf ) ;
			AllSurf.Add( pTrmSurf ) ;
		}
		tmpPnt[2] = dStart2[2] ;
		pLine = new CGeoLine( dStart2, tmpPnt ) ;
		pSurf = new CRevSur( pLine, dPivot, dZDir, dAngle ) ;
		if ( pSurf )
		{
			pTrmSurf = new CGeoTrmSurf( pSurf ) ;
			AllSurf.Add( pTrmSurf ) ;
		}
	}
	else
	{
		tmpPnt[2] = dStart[2] ;
		pLine = new CGeoLine( dStart, tmpPnt ) ;
		pSurf = new CRevSur( pLine, dPivot, dZDir, dAngle ) ;
		if ( pSurf )
		{
			pTrmSurf = new CGeoTrmSurf( pSurf ) ;
			AllSurf.Add( pTrmSurf ) ;
		}
	}
	// 删除内存
	if( pRvBtm ) delete pRvBtm ;
}
BOOL CSmartGraphic::GetGeoSurf( JDNC_SETUP& Setup, CPtrArray& AllSurf, int nSurfMode ) 
{
	if( ! m_pGetGeoSurf || ! m_pAtClass ) return FALSE ;
	// 构建加工模型
	return (* m_pGetGeoSurf)( m_pAtClass ,  /*图形对象*/
		                      Setup      ,  /*图形对象*/
		                      AllSurf,		/*加工模型*/
							  nSurfMode );  /*检查模型*/
}

CSmartLoop* CSmartGraphic::Get5AxTrimContour( JDNC_SETUP& Setup , RFRAME& TrimMtx )
{
	CPtrList AllCont ;
	if( ! GetAllEntity( Setup , AllCont, SMARTGRAPH_TYPE_CONTOUR, &TrimMtx ) ) 
	{
		return NULL ; 
	} 
	CSmartLoop* AllLoop = NULL ; 
	POSITION pos = AllCont.GetHeadPosition() ;
	while( pos ) 
	{
		CSmartLoop* pLoop = (CSmartLoop*)AllCont.GetNext( pos ) ;
		pLoop->next = pLoop->prev = NULL ;
		AllLoop = Mini_AddContours( AllLoop, pLoop ) ;
	}
    CSmartLoop  LoopTmp ;
    AllLoop = LoopTmp.ResortContour( AllLoop, Setup.m_cOrderDef.m_nSortType , FALSE) ;
	AllCont.RemoveAll() ;
	return AllLoop ; 
}
BOOL CSmartGraphic::GetSharpEdges( JDNC_SETUP& Setup,CPtrList& SharpEdges) 
{
	return GetAllEntity( Setup , SharpEdges, SMARTGRAPH_TYPE_SHARPEDGE);
}
/////////////////////////////////////////////////////////////////////////////////////////
//CSmartPathGen类的实现
CSmartPathGen::CSmartPathGen()          
{
	m_bRComb = 1;
	m_pTool = NULL ;
	m_pSafeTool = NULL ;
    m_pHolderTool=NULL ;
	m_pStockBox=NULL ;
	mathInitRFrame( &m_dNcMtx ) ;
	m_cPrgDef.m_dLimitAt = 100 / ( 100 * 1.00  ) ;
	m_cPrgDef.m_dStepAt  = 0 ;
	m_cPrgDef.m_dIncStep = 1. ;
	m_cPrgDef.m_pBrkFunc = NULL ;
	m_cPrgDef.m_pPrgFunc = NULL ;
	m_cPrgDef.m_pNewFunc = NULL ;
	m_cPrgDef.m_pPosFunc = NULL ; 
	m_cFeedDef.m_cSlotDef.m_dSlotDepth = NULL  ;
	m_cFeedDef.m_cLayerDef.m_dLayerDepth = NULL  ;
	m_nErrorType  = 0  ;
	m_nData	= 0		; // 私有数据传递	
	m_pData	= NULL	; // 私有数据传递	
	m_pGraph = NULL ;
	m_bMoveFlag=0   ;
    m_bFollowOuter  = 1 ;
	m_dTopHeight	= 0. ;
	m_dBotHeight	= 0. ;
	m_dCutDepth		= 0. ;

	m_nCalcThreadNum = Nc5D_GetSysSupportCoreNum();
	SmartNC_SetThreadCount( m_nCalcThreadNum ) ;
}

CSmartPathGen::~CSmartPathGen() 
{
	if( m_pTool ) delete m_pTool ;
	if( m_pSafeTool ) delete m_pSafeTool ;
    if( m_pHolderTool) delete m_pHolderTool ;
	if( m_pStockBox  ) delete m_pStockBox;
	if( m_cFeedDef.m_cLayerDef.m_dLayerDepth )
	{
		delete[] m_cFeedDef.m_cLayerDef.m_dLayerDepth ;
		m_cFeedDef.m_cLayerDef.m_dLayerDepth = NULL  ;
	}
	if( m_cFeedDef.m_cSlotDef.m_dSlotDepth ) 
	{
		delete[] m_cFeedDef.m_cSlotDef.m_dSlotDepth ;
		m_cFeedDef.m_cSlotDef.m_dSlotDepth = NULL  ;
	}
}
CSmartPathGen& CSmartPathGen::operator = ( CSmartPathGen& PathGen ) 
{
	m_cShapeDef  = PathGen.m_cShapeDef  ; // 雕刻形状
	m_cStockDef  = PathGen.m_cStockDef  ; //加工余量
	m_cToolDef   = PathGen.m_cToolDef   ; // 进刀控制
	m_cSpeedDef  = PathGen.m_cSpeedDef  ; // 下刀控制
	m_cSetupDef  = PathGen.m_cSetupDef  ; // 计算设置
	m_cFeedDef   = PathGen.m_cFeedDef   ; // 进给设置
    m_cPrgDef    = PathGen.m_cPrgDef    ; // 计算进度
	m_cPathPntDef= PathGen.m_cPathPntDef  ;// 路径起点
	m_cMethodDef = PathGen.m_cMethodDef ;
	m_cProcessDef= PathGen.m_cProcessDef  ;
	m_bMoveFlag = PathGen.m_bMoveFlag   ;
    m_cFeedDef.m_cLayerDef.m_dLayerDepth = NULL  ;
    m_cFeedDef.m_cSlotDef.m_dSlotDepth   = NULL  ;
	m_c5DCtrlDef = PathGen.m_c5DCtrlDef ; // 多轴控制参数
	m_cTransfDef = PathGen.m_cTransfDef ; //路径变换
	m_cSlayerDef = PathGen.m_cSlayerDef ; //侧向分层
	m_dTopHeight = PathGen.m_dTopHeight	;
	m_dBotHeight = PathGen.m_dBotHeight	;
	m_dCutDepth = PathGen.m_dCutDepth ;
	return  (*this) ;
}
void  CSmartPathGen::SetEngraveParam( CPathGroup& NewPath, CSmartGraphic& Graph ) 
{
	NewPath.m_nToolID  = m_cToolDef.m_nToolID ;
    NewPath.m_nHCompID = m_cProcessDef.m_nHCompID ;
    NewPath.m_nRCompID = m_cProcessDef.m_nRCompID ;
	CPtrArray cAllSurf ;
	DOUBLE dMaxHeight = m_dTopHeight ;
	DOUBLE dHeight = 0.0, dRapidHeight = dMaxHeight, dSafeHeight = dMaxHeight ;
	if( m_cTransfDef.m_cProjtranDef.m_nProjtranType != 0 )
	{
		if( Graph.GetProjectSurfTop( dHeight ) )
		{
			if( dMaxHeight < dHeight )	dMaxHeight = dHeight ;
			dSafeHeight = max(dSafeHeight, dMaxHeight);
			dRapidHeight = max(dRapidHeight, dMaxHeight);
		}
	}
	else if( m_cMethodDef.m_nMethodType == surfncMethodCurve  ||//&& m_cMethodDef.m_cCurveCut.m_bCurveFlag & NCDEF_FCURVE_REDEPTH
			 m_cMethodDef.m_nMethodType == surfncMethodLoop   )//&& m_cMethodDef.m_cLoopCut.m_bLoopFlag & NCDEF_FLOOP_REDEPTH
	{
		PNT3D dBox[2] ;
		if( Graph.GetEntBox ( dBox, SMARTGRAPH_TYPE_CURVE3D ))
		{
			dHeight = dBox[1][2] ;
			if (m_pStockBox != NULL)
			{// 使用毛胚时安全高度取二者最大值
				dSafeHeight = max(dHeight, m_pStockBox->max[2]);
				if (m_cSpeedDef.m_nRapidMode == NCDEF_RAPIDMODE_ABSOLUTE)
				{
					dRapidHeight = dSafeHeight ;
				}
			}
			if( dMaxHeight < dHeight )	dMaxHeight = dHeight ;
			dSafeHeight = max(dSafeHeight, dMaxHeight);
			dRapidHeight = max(dRapidHeight, dMaxHeight);
		}
	}
	else if( Graph.GetSurfaceTopHeight( m_cMethodDef.m_nMethodType,NCDEF_SURF_ALL, dHeight )  )
	{
	    if( SurfNC_IsMethodNeedSurfaces( m_cMethodDef.m_nMethodType, 0 ) )
	    {
			dHeight += max( 0.0, max( GetDriveSurfZMove(), GetCheckSurfZMove()) ) ;
			if (m_pStockBox != NULL)
			{// 使用毛胚时安全高度取二者最大值
				dSafeHeight = max(dHeight, m_pStockBox->max[2]);
				if (m_cSpeedDef.m_nRapidMode == NCDEF_RAPIDMODE_ABSOLUTE)
				{
					dRapidHeight = dSafeHeight ;
				}
			}
        }
		if( dMaxHeight < dHeight )	dMaxHeight = dHeight ;
		dSafeHeight = max(dSafeHeight, dMaxHeight);
		dRapidHeight = max(dRapidHeight, dMaxHeight);
	}
	NewPath.m_fRapidHeight = m_cSpeedDef.m_dRapidHeight + dRapidHeight ;
	NewPath.m_fSafeHeight  = m_cSpeedDef.m_dSafeHeight + dSafeHeight ;
	NewPath.m_nCoolant     = m_cSpeedDef.m_nCoolant     ; // 刀具冷却  
	NewPath.m_nSpindle     = m_cSpeedDef.m_nSpindle     ; // 主轴转速 
	NewPath.m_fPlungeDist  = m_cSpeedDef.m_dPlungeDist  ; // 慢速下刀
	NewPath.m_fFeedRate    = (FLOAT)m_cSpeedDef.m_dFeedRate    ; // 进给速度
	NewPath.m_fDelayTime   = (FLOAT)m_cSpeedDef.m_dDelayTime   ; // 落刀延迟 
	NewPath.m_fSlotRate    = (FLOAT)m_cSpeedDef.m_dSlotRate    ; // 开槽速度
	NewPath.m_fPlungeRate  = (FLOAT)m_cSpeedDef.m_dPlungeRate  ; // 下刀速度
	NewPath.m_fLeadRate    = (FLOAT)m_cSpeedDef.m_dLeadRate    ; // 进刀速度
	NewPath.m_fRetractRate = (FLOAT)m_cSpeedDef.m_dRetractRate ; // 提刀速度
	NewPath.m_fConnectRate = (FLOAT)m_cSpeedDef.m_dConnectRate ; // 连刀速度
}

double CSmartPathGen::GetLeadArcLen()
{
	double	dArcLen=0.0;
	switch(m_cMethodDef.m_nMethodType)
	{
	case surfncMethodSFinish : // 曲面精雕刻
	case surfncMethodSGuide  : // 导动加工
	case surfncMethodCleanup : // 曲面清根加工
        dArcLen = m_cFeedDef.m_cConnectDef.m_dLead3DRadius ;
		break;
	default:
		break;
	}

	return dArcLen;
}

DOUBLE CSmartPathGen::GetDriveSurfOffset() 
{
	return m_cStockDef.m_dDriveOffset[0] - m_cStockDef.m_dSparkGap  ;
}
DOUBLE CSmartPathGen::GetDriveSurfZMove() 
{
    return m_cStockDef.m_dDriveZMove[0] - m_cStockDef.m_dSparkGap  ;
}
DOUBLE CSmartPathGen::GetCheckSurfOffset() 
{
	return m_cStockDef.m_dCheckOffset[0] - m_cStockDef.m_dSparkGap;
}
DOUBLE CSmartPathGen::GetCheckSurfZMove() 
{
	return m_cStockDef.m_dCheckZMove[0] - m_cStockDef.m_dSparkGap;
}
DOUBLE CSmartPathGen::GetWedmPlanarGap() 
{
    return m_cStockDef.m_dPlanarGap ;
}

DOUBLE CSmartPathGen::GetToolBotArea ( CSmartTool *pTool )
{
	DOUBLE dArea = 0. ;
	if( !pTool ) return dArea ;
	int nType = pTool->GetType () ;
	if( nType == surfncToolFlat || nType == surfncToolBall || 
		nType == surfncToolNose || nType == surfncToolDrill )
	{
		dArea = PI1 * pTool->m_fRadius2 ;
	}
	else if( nType == surfncToolAFlat )
	{
		dArea = PI1 * pTool->m_fBottom2 ;
	}
	else if( nType == surfncToolABall )
	{
		dArea = PI1 * pTool->m_fCorner2 ;
	}
	else
	{
		dArea = 1. ;
	}

	return dArea ;
}

BOOL CSmartPathGen::IsDelBndPoint() 
{
    if( m_cMethodDef.m_nMethodType != surfncMethodSFinish && 
        m_cMethodDef.m_nMethodType != surfncMethodSGuide  && 
        m_cMethodDef.m_nMethodType != surfncMethodCleanup &&
        m_cMethodDef.m_nMethodType != surfncMethodAutoFinish)
    {
        return FALSE ;
    }
  	if( m_cSetupDef.m_cModelTol.m_nMdlFlag & NCDEF_SETUP_DELBNDPNT )
    {
        return TRUE ;
    }
    return FALSE ;
}


BOOL CSmartPathGen::IsDelFlatPoint() 
{
    if( m_cMethodDef.m_nMethodType == surfncMethodAutoFinish )
    {
        if( m_cMethodDef.m_cAutoFinish.m_bFinishFlag & NCDEF_FAUTOFINISH_DELFLATPNT )
        {
            return TRUE ;
        }
        return FALSE ;
    }
    else if( m_cMethodDef.m_nMethodType == surfncMethodSFinish )
    {
        if( (m_cMethodDef.m_cSFinishCut.m_bMoveFlag & NCDEF_SFINISH_DELFLATPNT ) &&
            (m_cMethodDef.m_cSFinishCut.m_nMoveType != surfncMoveHeight ) && 
            (m_cMethodDef.m_cSFinishCut.m_nMoveType != surfncMoveUWLine )   )
        {
            return TRUE ;
        }
    }
    return FALSE ;
}
DOUBLE CSmartPathGen::GetLoopCheckTol() 
{
    double dTol = max( 0.01, m_cSetupDef.m_cCombineDef.m_dSpacing) ;
    if( dTol < 3.0 * m_cSetupDef.m_cTolDef.m_dArcTol )
    {
        dTol = 3.0 * m_cSetupDef.m_cTolDef.m_dArcTol ;
    }
    if( m_pTool && dTol > m_pTool->m_fRadius * 2.0 ) 
    {
        dTol = m_pTool->m_fRadius * 2.0 ;
    }
    return dTol ;
}

CSmartTool* CSmartPathGen::CreateSmartTool( JDNC_TOOL& Tool, 
										    DOUBLE SurfStock, 
											double Planar  )
{
	if (Tool.m_nToolType > surfncToolTap)
	{
		ASSERT(FALSE);
		return NULL;
	}

	JDNC_TOOLEX cTemp;
	SurfNC_InitPathParam(cTemp);
	SurfNC_CopyToolInforFromOld(Tool, cTemp);

	return CreateSmartTool(cTemp, SurfStock, Planar);
}
CSmartTool* CSmartPathGen::CreateAllTypeTool( JDNC_TOOLEX& Tool, 
										      DOUBLE SurfStock, 
											  double Planar  )
{
    double	dAngle  = ANGLE_TO_RADIAN(Tool.m_dSideAng*0.5);
    double  dRadius = Tool.m_dTopDiam * 0.5 ;
	double	dCorner = Tool.m_dCorRadi       ;
	double	dBottom = Tool.m_dBtmDiam * 0.5 ;
    CSmartTool* pMiller = NULL ; 
    if( Tool.m_nToolType == surfncToolLol )
    {/*糖果刀	*/
		double dShankStock = max( 0.0, m_cProcessDef.m_dShankTol) + SurfStock;
        dRadius += dShankStock, dCorner += SurfStock, dBottom+= dShankStock ;
		if( dRadius < 0.0 ) return NULL ; 
        if( dCorner < 0.0 ) dCorner = 0.0 ;
        if( dBottom < 0.0 ) dBottom = 0.0 ;
        CSmtToolALol * pTLol = new CSmtToolALol() ;
        if( dRadius - dBottom > 0.01 && Tool.m_dConeLen  > 0.01 )
        {
            double dAngle =  atan2( ( dRadius-dBottom),Tool.m_dConeLen ) ;
            pTLol->m_pShaft = new CSmtToolAFlat() ;
            pTLol->m_pShaft->SetParam( dRadius, dBottom, dAngle, 0.0 ) ;
        }
        else 
        {
            pTLol->m_pShaft = new CSmtToolFlat() ;
            pTLol->m_pShaft->SetParam( dRadius, 0.0, 0.0, 0.0 ) ;
        }
        pTLol->m_cBallEx.Create( dCorner ) ;
        double dBtmRad = min( dCorner, dBottom ) ;
        pTLol->m_dZShift = dCorner + nc_Sqrt( dCorner * dCorner - dBtmRad * dBtmRad ) ;
        pTLol->SetParam( max( dRadius, dCorner ) , 0.0, 0.0, 0.0 ) ;
        pMiller = pTLol ;
    }
    else if( Tool.m_nToolType == surfncToolSlot )
    {/*槽铣刀	*/
		double dShankStock = max( 0.0, m_cProcessDef.m_dShankTol) + SurfStock;
        dRadius += dShankStock, dCorner += SurfStock, dBottom+= dShankStock ;
		if( dRadius < 0.0 ) return NULL ; 
        if( dCorner < 0.0 ) dCorner = 0.0 ;
        if( dBottom < 0.0 ) dBottom = 0.0 ;
        CSmtToolSpecial * pTSlot = new CSmtToolSpecial() ;
        double dSlotRadius = Tool.m_dSlotDiam*0.5+SurfStock ;
        double dSlotHeight = Tool.m_dHeight + 2.0 * SurfStock ;
        if( dSlotRadius < 0.0 || dSlotHeight < 0.0 ) 
        {
            return NULL ; 
        }
        pTSlot->m_nNumShape = 0 ;
        if( dCorner < 0.001 )
        {/*底部退化成圆柱*/
            CSmtToolConeEx * pConeEx = new CSmtToolConeEx() ;
            pConeEx->Create( dSlotRadius, dSlotRadius, dSlotHeight ) ;
            pTSlot->m_aAllShape[pTSlot->m_nNumShape] = pConeEx ;
            pTSlot->m_dZShift[pTSlot->m_nNumShape] = 0.0 ;
            pTSlot->m_nNumShape ++ ;
        }
        else if( dSlotHeight - 2.0 * dCorner < 0.001 )
        {/*底部退化成圆盘*/
            if( dSlotRadius - dCorner < 0.001 )
            {/*退化成圆球*/
                CSmtToolBallEx * pBallEx = new CSmtToolBallEx() ;
                pBallEx->Create( dSlotRadius ) ;
                pTSlot->m_aAllShape[pTSlot->m_nNumShape] = pBallEx ;
            }
            else
            {
                CSmtToolTorusEx *pTorus = new CSmtToolTorusEx() ;
                pTorus->Create( dSlotRadius, dCorner, dSlotHeight - 2.0 * dCorner) ;
                pTSlot->m_aAllShape[pTSlot->m_nNumShape] = pTorus ;
            }
            pTSlot->m_dZShift[pTSlot->m_nNumShape] = 0.0 ;
            pTSlot->m_nNumShape ++ ;
        }
        else
        {
            CSmtToolTorusEx *pTorus = new CSmtToolTorusEx() ;
            pTorus->Create( dSlotRadius, dCorner, dSlotHeight - 2.0 * dCorner) ;
            pTSlot->m_aAllShape[pTSlot->m_nNumShape] = pTorus ;
            pTSlot->m_dZShift[pTSlot->m_nNumShape] = 0.0 ;
            pTSlot->m_nNumShape ++ ;
        }
        if( fabs( dBottom - dRadius ) < 0.001 )
        {
            CSmtToolFlat * pFlat = new CSmtToolFlat() ;
            pFlat->SetParam( dRadius, 0.0, 0.0, 0.0 ) ;
            pTSlot->m_aAllShape[pTSlot->m_nNumShape ] = pFlat ;
            pTSlot->m_dZShift[pTSlot->m_nNumShape ] = dSlotHeight ;
            pTSlot->m_nNumShape ++ ;
        }
        else
        {
            double dAngle =  atan2( ( dRadius-dBottom),Tool.m_dConeLen ) ;
            CSmtToolAFlat * pAFlat = new CSmtToolAFlat() ;
            pAFlat->SetParam( dRadius, dBottom, dAngle, 0.0 ) ;
            pTSlot->m_aAllShape[pTSlot->m_nNumShape ] = pAFlat ;
            pTSlot->m_dZShift[pTSlot->m_nNumShape ] = dSlotHeight ;
            pTSlot->m_nNumShape ++ ;
        }
        pTSlot->SetParam( max( dSlotRadius,max( dRadius, dBottom) ) , 0.0, 0.0, 0.0 ) ;
        pMiller = pTSlot ;
    }
    else if( Tool.m_nToolType == surfncToolDove )
    {/*燕尾铣刀	*/
        if( fabs( Tool.m_dSideAng) < 0.01 || dBottom < dRadius )
        {/*无效参数*/
            return NULL  ;
        }
        dBottom += SurfStock ;
        dCorner += SurfStock;
        if( dCorner < 0.0 ) dCorner = 0.0 ;
        dRadius += SurfStock / cos( dAngle ) ; 
        if( dRadius < 0.005 || dBottom < 0.005 )
        {
            return NULL ; 
        }
        double dTorusHgt = dCorner + dCorner* sin( dAngle )              ; // 圆环部分的高度
        double dConeBtm  = ( dBottom - dCorner) + dCorner * cos( dAngle ); // 圆椎底部半径
        double dConeHgt  = fabs( dConeBtm - dRadius ) / tan( dAngle )    ; // 圆锥部分的高度
        double dDoveHeight = dTorusHgt + dConeHgt ;
        CSmtToolSpecial * pTDove = new CSmtToolSpecial() ;
        pTDove->m_nNumShape = 0 ;
        if( dCorner < 0.001 )
        {
            CSmtToolConeEx * pConeEx = new CSmtToolConeEx() ;
            pConeEx->Create( dRadius, dBottom, dDoveHeight ) ;
            pTDove->m_aAllShape[pTDove->m_nNumShape] = pConeEx ;
            pTDove->m_dZShift[pTDove->m_nNumShape] = 0.0 ;
            pTDove->m_nNumShape ++ ;
        }
        else 
        {
            CSmtToolTorusEx *pTorus = new CSmtToolTorusEx() ;
            pTorus->Create( dBottom, dCorner, 0.0 ) ;
            pTDove->m_aAllShape[pTDove->m_nNumShape] = pTorus ;
            pTDove->m_dZShift[pTDove->m_nNumShape] = 0.0 ;
            pTDove->m_nNumShape ++ ;
            CSmtToolConeEx * pConeEx = new CSmtToolConeEx() ;
            pConeEx->Create( dRadius, dConeBtm, dConeHgt ) ;
            pTDove->m_aAllShape[pTDove->m_nNumShape] = pConeEx ;
            pTDove->m_dZShift[pTDove->m_nNumShape] = dTorusHgt ;
            pTDove->m_nNumShape ++ ;
        }
        CSmtToolFlat * pFlat = new CSmtToolFlat() ;
        pFlat->SetParam( dRadius, 0.0, 0.0, 0.0 ) ;
        pTDove->m_aAllShape[pTDove->m_nNumShape ] = pFlat ;
        pTDove->m_dZShift[pTDove->m_nNumShape ] = dDoveHeight ;
        pTDove->m_nNumShape ++ ;
        double dMaxRad = max( dBottom, dRadius) ;
        pTDove->SetParam( dMaxRad , 0.0, 0.0, 0.0 ) ;
        pMiller = pTDove ;
    }
    if( pMiller )
    {
        pMiller->m_nToolID=Tool.m_nToolID;
	    pMiller->m_fLength=Tool.m_dLength;
	    if(pMiller->m_fShoulder<Tool.m_dHeight)
	    {
		    pMiller->m_fShoulder = Tool.m_dHeight;
	    }
		if (Tool.m_nToolType == surfncToolLol ||
			Tool.m_nToolType == surfncToolSlot )
		{
			double dShankStock = max( 0.0, m_cProcessDef.m_dShankTol);
			pMiller->m_fLength -= dShankStock;
		}
        return pMiller ;
    }
	
	return CreateSmartTool( Tool, SurfStock, Planar ) ;
}

CSmartTool* CSmartPathGen::CreateSmartTool( JDNC_TOOLEX& Tool, 
										    DOUBLE SurfStock, 
											double Planar  )
{
    double		dAngle=ANGLE_TO_RADIAN(Tool.m_dSideAng/2.0);
	double		dRadius = Tool.m_dTopDiam/2.0 ;
	double		dBottom = Tool.m_dBtmDiam/2.0 ;
	double		dCorner = Tool.m_dCorRadi     ;
	CSmartTool*	pMiller = NULL ; 
	double dTmpCor = dCorner ;
	if( Tool.m_nToolType == surfncToolBall )
    {
		dTmpCor = dRadius ;
    }
	// 球头刀\锥球刀不能有平动量
	if( Tool.m_nToolType == surfncToolFlat && Planar > dRadius ||
		Tool.m_nToolType != surfncToolFlat && Planar > dBottom )
	{
	    m_nErrorType = JDERROR_GENPATH_PLANARERROR ;
	    return NULL ;
	}
	
	if( dRadius+SurfStock - Planar < 0.01 ) 
	{//刀具半径小于0.01
		m_nErrorType=JDERROR_GENPATH_STOCK;
		pMiller = new CSmtToolFlat(); 
		dRadius = 0.01 ; 
		dBottom = dCorner = dAngle = 0.0 ; 
	}
	else if( Tool.m_nToolType == surfncToolFlat ||
		    Tool.m_nToolType == surfncToolDrill )
	{//平底刀　|| 钻孔刀具
		if( SurfStock < 1.0e-3 )
		{
			pMiller=new CSmtToolFlat(); 
			dRadius += SurfStock - Planar ;
			dBottom = dCorner = dAngle = 0.0 ; 
			m_nErrorType=JDERROR_GENPATH_STOCK;
		}
		else
		{
			pMiller=new CSmtToolNose();
			dRadius += SurfStock - Planar ;
			dBottom = dRadius ;
			dCorner = SurfStock ;
			dAngle = 0.0 ; 
		}
	}
	else if( Tool.m_nToolType == surfncToolBall )
	{ //球头刀
		pMiller=new CSmtToolBall(); 
		dRadius += SurfStock ;
	}
	else if( Tool.m_nToolType == surfncToolNose )
	{//牛鼻刀
        if( SurfStock+dCorner > 1.0e-3 )
		{
			pMiller = new CSmtToolNose();
			dRadius += SurfStock - Planar ;
			dCorner += SurfStock ;
			dAngle = 0.0 ;
		}
		else
		{
			pMiller = new CSmtToolFlat();
			dRadius += SurfStock - Planar ;
			dBottom = dCorner = dAngle = 0.0;
			m_nErrorType=JDERROR_GENPATH_STOCK;
		}
	}
	else if( Tool.m_nToolType == surfncToolAFlat || 
		     Tool.m_nToolType == surfncToolAFlat2  )
	{//锥度平底刀 || 大头刀 
		if( SurfStock < 1.0e-3 )
		{
			dRadius += SurfStock ;
			dBottom = dBottom + SurfStock - Planar  ;
			if( dBottom < 0.005 ) dBottom = 0.005 ;	
			dCorner = 0.0 ;
			if( dBottom >= dRadius-0.001 )
			{
				pMiller = new CSmtToolFlat();
				dBottom = dAngle = 0.0 ;
			}
			else
			{
				pMiller = new CSmtToolAFlat();
			}
			m_nErrorType=JDERROR_GENPATH_STOCK;
		}
		else
		{
			pMiller = new CSmtToolANose();
			dRadius += SurfStock ;
			dBottom -= Planar   ;
			dCorner =  SurfStock ;
		}
	}
	else if( Tool.m_nToolType == surfncToolABall )
	{//锥度球头刀
		if( dCorner+SurfStock > 1.0e-3 )
		{
          pMiller = new CSmtToolABall(); 
		   dRadius += SurfStock ;
		   dCorner += SurfStock ;
		}
		else 
		{
          pMiller = new CSmtToolAFlat(); 
		   dRadius += SurfStock ;
		   dBottom = 0.005 ;
		   dCorner = 0.0  ;
			m_nErrorType=JDERROR_GENPATH_STOCK;
		}
	}
	else if( Tool.m_nToolType == surfncToolANose )
	{//锥度牛鼻刀
		if( SurfStock+dCorner > 1.0e-3 )
		{ 
			pMiller = new CSmtToolANose(); 
			dRadius += SurfStock ;
			dBottom -= Planar   ;
			dCorner += SurfStock ;
		}
		else
		{
			dRadius += SurfStock ;
			dBottom = dBottom + dCorner + SurfStock - Planar ;
			if( dBottom < 0.005 ) dBottom=0.005;
			dCorner = 0.0;
			if( dBottom >= dRadius-0.001 )
			{
				pMiller = new CSmtToolFlat();
				dBottom = dAngle = 0.0 ;
			}
			else
			{
				pMiller = new CSmtToolAFlat();
			}
			m_nErrorType=JDERROR_GENPATH_STOCK;
		}
	}
	else if ( Tool.m_nToolType == surfncToolTap )
	{
		pMiller = new CSmtToolFlat () ;
		dBottom = dAngle = 0.0 ;
		pMiller->m_fShoulder = Tool.m_dCorRadi * Tool.m_dSideAng ;
	}
	if(!pMiller) 
	{
		m_nErrorType=JDERROR_GENPATH_NOTOOL;
		return NULL;
	}
	pMiller->SetParam( dRadius,	//顶半径
					   dBottom,				//底半径
					   dAngle,				//刀具半角
					   dCorner ); //圆角半径
	pMiller->m_nToolID=Tool.m_nToolID;
	pMiller->RefineParam();

	pMiller->m_fLength=Tool.m_dLength;
	if(pMiller->m_fShoulder<Tool.m_dHeight)
	{
		pMiller->m_fShoulder = Tool.m_dHeight;
	}

	return pMiller;	
}

int CSmartPathGen::UpdateTool2D() 
{
	if( m_pTool ) return 1;
	m_pTool = CreateSmartTool(m_cToolDef, 0.0, 0.0);
	if(!m_pTool) return 0 ;
    return 1 ;
}

int CSmartPathGen::UpdateTool3D( CSmartGraphic& /*Graph*/ )
{
    if( m_pHolderTool ) 
    {
        delete m_pHolderTool ;
        m_pHolderTool = NULL ; 
    }
    if( m_cProcessDef.m_bProcessFlag & NCDEF_TOOL_USEHOLDER ||
		m_cToolDef.m_nToolStatus & NCDEF_TOOLSTA_SHANK ) 
    {/*计算过程使用刀柄*/
        CreateHolderTool() ;
    }
	if( m_pTool && m_pSafeTool ) return 1;
	if( m_pTool ) delete m_pTool ;
	m_pTool=CreateSmartTool(m_cToolDef, GetDriveSurfOffset(), GetWedmPlanarGap() );
	if(!m_pTool) return 0 ;
	if( m_pSafeTool ) delete m_pSafeTool ;
	m_pSafeTool=CreateSmartTool( m_cToolDef, GetCheckSurfOffset(), GetWedmPlanarGap()) ;
	if(!m_pSafeTool) return 0 ;
	return 1;
}

void CSmartPathGen::SetCheckMachMdlBox(PNT3D dSrcBox[2], PNT3D dDesBox[2])
{	//将检查模型的包围盒扩大一些
	dDesBox[0][0]=dSrcBox[0][0]-5.0;
	dDesBox[0][1]=dSrcBox[0][1]-5.0;
	dDesBox[1][0]=dSrcBox[1][0]+5.0;
	dDesBox[1][1]=dSrcBox[1][1]+5.0;
	dDesBox[0][2]=GetBottomHeight();
	dDesBox[1][2]=GetTopHeight();
}

// 定义外形
BOOL CSmartPathGen::DefineShapeAngle( JDNC_SHAPE& Shape  /*雕刻形状*/)
{
	if( Shape.m_cDepth.m_dCutDepth < 0.0) Shape.m_cDepth.m_dCutDepth = 0.0 ;
	if( Shape.m_cTaper.m_dBoundAngle < 0.0) Shape.m_cTaper.m_dBoundAngle = 0.0 ;
	if( Shape.m_cTaper.m_dIslandAngle < 0.0) Shape.m_cTaper.m_dIslandAngle = 0.0 ;
	return TRUE ;
}
int MathCam_CalLayerCount(double  dMaxInc, double dTotalDepth)
{
	if( dMaxInc < 0.001 ) dMaxInc = 0.001 ;
	int nIncTime = (int)ceil( dTotalDepth / dMaxInc ) ;
	if( fabs( ( nIncTime - 1 ) * dMaxInc - dTotalDepth ) < 0.001 )
		nIncTime -= 1 ;

	return nIncTime;
}
void MathCam_AssignLayerDepth(int& nLayerCount, double& dTotalDepth, double*& dLayerDepth, bool bNoTop)
{
	ASSERT(dLayerDepth == NULL);

	if( nLayerCount < 1 ) nLayerCount = 1 ;
	if(  dTotalDepth < 1.0e-3 ) nLayerCount = 1 ; 
	dLayerDepth = new double[nLayerCount+1] ;
	double dSideDInc = dTotalDepth /  double(nLayerCount) ;
	for( int i = bNoTop ? 1 : 0 ; i <= nLayerCount ; i ++ ) 
	{
		dLayerDepth[i] = i * dSideDInc ;
	}
}
// 自定义分层
void CSmartPathGen::AssignSelfDefineLayerDepth(int nLayerFlag, double dTop, double dBot, 
											   double*& dDepth, int& nCount, BOOL bSurfMode)
{
	if (nLayerFlag & NCDEF_LAYER_DEFGRAD)
	{
		return UserDefineGradualLayerDepth(nLayerFlag, dTop, dBot, dDepth, nCount, bSurfMode);
	}
	std::vector<DOUBLE> cTempDepth;
	double dCutDepth = dTop - dBot;
	double dAll = 0.0, dSideDInc = 0.0, dSegLen = 0.0, dLeft = 0.0;
	JDNC_LAYERDEF& cDef = m_cSlayerDef.m_cAxisDef;
	
	BOOL bFirst = TRUE;			
	for (int i=0; i<cDef.m_nSegNum; i++)
	{
		if (cDef.m_dSegInc[i] < 0.001) continue;

		dLeft = dCutDepth-dAll;
		dSegLen = cDef.m_dSegInc[i] > dLeft ? dLeft : cDef.m_dSegInc[i];
		if (dSegLen < 0.001) break;
		nCount = MathCam_CalLayerCount(cDef.m_dLayerInc[i], dSegLen) ;
		if( nCount < 1 ) nCount = 1 ;
		dSideDInc = dSegLen /  nCount;
		for( int j = bFirst ? 0 : 1 ; j <= nCount ; j ++ ) 
		{
			cTempDepth.push_back(dAll + j * dSideDInc);
		}
		dAll += dSegLen;
		if (bFirst) bFirst = FALSE;
	}			
	dLeft = dCutDepth-dAll;
	if (dLeft > 0.001)
	{
		nCount = MathCam_CalLayerCount(cDef.m_dGlobalDInc, dLeft) ;
		if( nCount < 1 ) nCount = 1 ;
		dSideDInc = dLeft /  nCount;
		for( int i = bFirst ? 0 : 1 ; i <= nCount ; i ++ ) 
		{
			cTempDepth.push_back(dAll + i * dSideDInc);
		}
	}	
	nCount = (int)cTempDepth.size();
	if (nCount < 2)
	{
		nCount = 2;
		if (nLayerFlag & NCDEF_LAYER_TOPCNT)
		{
			cTempDepth.push_back(0.0);
			cTempDepth.push_back(dCutDepth);
		}
		else
		{
			cTempDepth.push_back(dCutDepth);
			cTempDepth.push_back(0.0);
		}
	}
	nCount = (int)cTempDepth.size();
	dDepth = new double[nCount];
	memcpy(dDepth, &cTempDepth[0], sizeof(double)*(nCount));

	if (!(nLayerFlag & NCDEF_LAYER_TOPCNT))
	{// 底层往上
		for( int i = nCount-1, j=0 ; i>=0; i--, j++ )
			dDepth[i] = dCutDepth - cTempDepth[j];
	}
	
	if (bSurfMode)
	{
		for( int i = 0 ; i < nCount - 1 ; i++ )
			dDepth[i] = dTop - dDepth[i+1];
	}

	nCount = nCount - 1; 
}

void CSmartPathGen::UserDefineGradualLayerDepth(int nLayerFlag, double dTop, double dBot, 
												double*& dDepth, int& nCount, BOOL bSurfMode)
{
	std::vector<DOUBLE> cTempDepth;
	double dCutDepth = dTop - dBot;
	double dAll = 0.0, dSideDInc = 0.0, dSegLen = 0.0, dLeft = 0.0, dFist = 0.0, dLast = 0.0;
	double dCurrTop = dTop, dCurrBot = dBot;
	JDNC_LAYERDEF& cDef = m_cSlayerDef.m_cAxisDef;

	/*
	    从下向上分层时，需要从最后一层开始计算，避免不同段间过渡位置出现分层深度的突变。
	*/
	BOOL bFirst = TRUE;	
	if (nLayerFlag & NCDEF_LAYER_TOPCNT)
	{// 从上向下分层
		for (int i=0; i<cDef.m_nSegNum; i++)
		{
			if (cDef.m_dSegInc[i] < 0.001) continue;
			dLeft = dCutDepth-dAll;
			dSegLen = cDef.m_dSegInc[i] > dLeft ? dLeft : cDef.m_dSegInc[i];
			if (dSegLen < 0.001) break;
			if (i == 0)
			{// 第一段均匀分层
				nCount = MathCam_CalLayerCount(cDef.m_dLayerInc[i], dSegLen) ;
				if( nCount < 1 ) nCount = 1 ;
				dSideDInc = dSegLen /  nCount;
				for( int j = bFirst ? 0 : 1 ; j <= nCount ; j ++ ) 
				{
					cTempDepth.push_back(dAll + j * dSideDInc);
				}
				dFist = dSideDInc;
			}
			else
			{// 渐变深度分层
				dCurrTop = dLeft;
				dCurrBot = dCurrTop - dSegLen;
				dLast = cDef.m_dLayerInc[i];
				double* pDepthArr = NULL;
				DefineGradualLayerDepth(dCurrTop, dCurrBot, dFist, dLast, pDepthArr, nCount);
				for (int j = 1; j <= nCount; ++j)
				{
					cTempDepth.push_back(pDepthArr[j] + dAll);
				}
				if (pDepthArr)
				{
					delete []pDepthArr;
				}
				dFist = dLast;
			}
			dAll += dSegLen;
			if (bFirst) bFirst = FALSE;
		}			
		dLeft = dCutDepth-dAll;
		if (dLeft > 0.001)
		{
			nCount = MathCam_CalLayerCount(cDef.m_dGlobalDInc, dLeft) ;
			if( nCount < 1 ) nCount = 1 ;
			dSideDInc = dLeft /  nCount;
			for( int i = bFirst ? 0 : 1 ; i <= nCount ; i ++ ) 
			{
				cTempDepth.push_back(dAll + i * dSideDInc);
			}
		}
	}
	else
	{// 从下向上分层
		std::vector<DOUBLE> cTempSeg;
		std::vector<DOUBLE> cTempInc;
		int nFirstSegCount = 0;
		for (int i=0; i<cDef.m_nSegNum; i++)
		{
			if (cDef.m_dSegInc[i] < 0.001) continue;
			dLeft = dCutDepth-dAll;
			dSegLen = cDef.m_dSegInc[i] > dLeft ? dLeft : cDef.m_dSegInc[i];
			if (dSegLen < 0.001) break;
			cTempSeg.push_back(dSegLen);
			cTempInc.push_back(cDef.m_dLayerInc[i]);
			dAll += dSegLen;
		}			
		dLeft = dCutDepth-dAll;
		if (dLeft > 0.001)
		{
			nCount = MathCam_CalLayerCount(cDef.m_dGlobalDInc, dLeft) ;
			if( nCount < 1 ) nCount = 1 ;
			dSideDInc = dLeft /  nCount;
			for( int i = 0  ; i <= nCount ; i ++ ) 
			{
				cTempDepth.push_back(/*dAll +*/ i * dSideDInc);
			}
			bFirst = FALSE;
		}

		nCount = (int)cTempSeg.size();
		if (nCount > 0)
		{// 修正第一段的分层深度
			dSegLen = cTempSeg[0];
			dSideDInc = cTempInc[0];
			nFirstSegCount = MathCam_CalLayerCount(dSideDInc, dSegLen) ;
			if( nFirstSegCount < 1 ) nFirstSegCount = 1 ;
			cTempInc[0] = dSegLen / nFirstSegCount;
		}
		if (cTempInc.size() > 1)
		{
			dFist = cTempInc[(int)cTempInc.size() - 1];
		}
		else if (cTempInc.size() == 1)
		{
			dFist = cTempInc[0];
		}
		dAll = dLeft;		
		for (int i = (int)cTempSeg.size() - 1; i > 0; --i)
		{
			dSegLen = cTempSeg[i];			
			dLast = cTempInc[i - 1];
			dLeft = dCutDepth - dAll;
			dCurrTop = dLeft;
			dCurrBot = dCurrTop - dSegLen;
			double* pDepthArr = NULL;
			DefineGradualLayerDepth(dCurrTop, dCurrBot, dFist, dLast, pDepthArr, nCount);
			for (int j = bFirst ? 0 : 1; j <= nCount; ++j)
			{
				cTempDepth.push_back(pDepthArr[j] + dAll);
			}
			if (pDepthArr)
			{
				delete []pDepthArr;
			}
			dAll += dSegLen;
			dFist = dLast;
			bFirst = FALSE;
		}
		if (cTempSeg.size() > 0)
		{// 加入自定义的第一段
			for( int i = bFirst ? 0 : 1; i <= nFirstSegCount ; i ++ ) 
			{
				cTempDepth.push_back(dAll + i * cTempInc[0]);
			}
		}
	}

	nCount = (int)cTempDepth.size();
	if (nCount < 2)
	{
		nCount = 2;
		cTempDepth.push_back(0.0);
		cTempDepth.push_back(dCutDepth);
	}
	nCount = (int)cTempDepth.size();
	dDepth = new double[nCount];
	memcpy(dDepth, &cTempDepth[0], sizeof(double)*(nCount));
	if (bSurfMode)
	{
		for( int i = 0 ; i < nCount - 1 ; i++ )
			dDepth[i] = dTop - dDepth[i+1];
	}

	nCount = nCount - 1; 
}
/*
    保证最后一层深度为用户设置的深度，第一层深度可能会比用户实际设置的值小
*/
void CSmartPathGen::DefineGradualLayerDepth(double dTop, double dBot, double &dFirstDepth, double &dLastDepth,
											double*& dDepth, int& nCount)
{
	double dCutDepth = dTop - dBot;
	double dFirst = dFirstDepth;
	double dLast = dLastDepth;
	double dAverage = 0.5 * (dFirst + dLast);
	nCount = MathCam_CalLayerCount(dAverage, dCutDepth);
	double dInc = 0.0;
	
	dDepth = new double[nCount+1];
	if (nCount > 1)
	{
		dDepth[0] = 0.0;
		dDepth[nCount] = dCutDepth;
		dAverage = dCutDepth/(nCount);
		dFirst = 2.0 * dAverage - dLast; 
		dInc = (dFirst - dLast)/(nCount-1);
		for (int i = 1; i < nCount; ++i)
		{
			dDepth[i] = dDepth[i-1] + dFirst - dInc * (i - 1);
		}
		dFirstDepth = dFirst;
	}
	else
	{
		dFirstDepth = dCutDepth;
		dLast = dCutDepth;
	}
}

// 定义分层　
BOOL CSmartPathGen::DefineLayerDepth( JDNC_LAYER& Layer ) /*分层深度*/
{
	JDNC_LAYERDEF& cDef = m_cSlayerDef.m_cAxisDef;
	if( Layer.m_dTotalDepth < 0.0 ) Layer.m_dTotalDepth = 0.0 ;
	if (Layer.m_dLayerDepth != NULL) 
	{
		delete[] Layer.m_dLayerDepth;
		Layer.m_dLayerDepth = NULL;
	}
	if( Layer.m_nLayerType == NCDEF_LAYER_CLOSE)
	{ // 关闭分层
		Layer.m_nLayerCount = 1 ;
	}
	else if( Layer.m_nLayerType == NCDEF_LAYER_DEPTH)
	{ // 等深分层
		Layer.m_nLayerCount = MathCam_CalLayerCount(Layer.m_dSideDInc, Layer.m_dTotalDepth) ;
	}

	if ( Layer.m_nLayerType == NCDEF_LAYER_DEF)
	{
		AssignSelfDefineLayerDepth(Layer.m_bLayerFlag, Layer.m_dTotalDepth, 0., Layer.m_dLayerDepth, Layer.m_nLayerCount, false);
	}
	else if (Layer.m_nLayerType == NCDEF_LAYER_GRAD)
	{//渐变
		double dFirst = cDef.m_dSegInc[9], dLast = cDef.m_dLayerInc[9];
		DefineGradualLayerDepth(Layer.m_dTotalDepth, 0., dFirst, dLast, Layer.m_dLayerDepth, Layer.m_nLayerCount);
	}
	else
	{
		MathCam_AssignLayerDepth(Layer.m_nLayerCount, Layer.m_dTotalDepth, Layer.m_dLayerDepth, false);
	}
	return TRUE  ;
}
// 计算侧向分层
BOOL CSmartPathGen::DefineSideLayerInc( JDNC_LAYERDEF& cDef )
{
	std::vector<DOUBLE> cTempDepth;
	int& nSideTime = m_cSlayerDef.m_nLayerCount;
	double dAll=0.0, dSideDInc=0.0;
	for (int i=0; i<cDef.m_nSegNum; i++)
	{
		if (cDef.m_dSegInc[i] < 0.001) 
			continue;

		nSideTime = MathCam_CalLayerCount(cDef.m_dLayerInc[i], cDef.m_dSegInc[i]) ;
		if (nSideTime < 1) nSideTime = 1;
		dSideDInc = cDef.m_dSegInc[i] /  nSideTime;
		for( int j = 0 ; j < nSideTime; j ++ ) 
		{
			cTempDepth.push_back(dAll + j * dSideDInc);
		}
		if (i == cDef.m_nSegNum-1)
		{// 光滑连刀路径时多一层
			cTempDepth.push_back(dAll + j * dSideDInc);
		}
		dAll += cDef.m_dSegInc[i];
	}	

	nSideTime = (int)cTempDepth.size();
	if (nSideTime < 2)
	{
		nSideTime = 2;
		cTempDepth.push_back(0.0);
		cTempDepth.push_back(dAll);
	}

	nSideTime = (int)cTempDepth.size();
	m_cSlayerDef.m_dTotalDepth = dAll;
	m_cSlayerDef.m_dLayerDepth = new double[nSideTime];
	memcpy(m_cSlayerDef.m_dLayerDepth, &cTempDepth[0], sizeof(double)*(nSideTime));
	nSideTime = nSideTime - 1; 

	return TRUE;
}
BOOL CSmartPathGen::RepairBlindRadius( JDNC_PLUNGE& Plunge )
{
	if( Plunge.m_nPlungeType == NCDEF_PLUNGE_CLOSE || 
		Plunge.m_nPlungeType == NCDEF_PLUNGE_VERT  ||
		!( Plunge.m_bPlungeFlag & NCDEF_PLUNGE_DELBLINDAREA) )
	{
		return FALSE ;
	}
	double dMinRad = Plunge.m_dBlindRadius  ;
//	if( m_pTool ) 
//	{
//		double dBtmRad = m_pTool->GetRadiusComp ( 0., 0. ) ;
//		if( dMinRad > dBtmRad ) dMinRad = dBtmRad ;
//	}
//	Plunge.m_dBlindRadius = dMinRad ; 
	if( Plunge.m_nPlungeType == NCDEF_PLUNGE_HELIX )
	{
		if( Plunge.m_dRadius <  dMinRad )
		{
			Plunge.m_dRadius =  dMinRad ; 
		}
		Plunge.m_nErrorTreat = NCDEF_ERRPLUNGE_CONTOUR ;
	}
	else if( Plunge.m_nPlungeType == NCDEF_PLUNGE_RAMP )
	{
		if( Plunge.m_dRadius < 2.0 * dMinRad )
		{
			Plunge.m_dRadius = 2.0 * dMinRad ; 
		}
		Plunge.m_nErrorTreat = NCDEF_ERRPLUNGE_CONTOUR ;
	}
	return TRUE ;
}

DOUBLE CSmartPathGen::GetDownStep()  
{ 
	double top = GetTopHeight() ;
	double bot = GetBottomHeight() ;
	if( m_cFeedDef.m_cLayerDef.m_nLayerType == NCDEF_LAYER_COUNT )
	{
	    if( m_cFeedDef.m_cLayerDef.m_nLayerCount < 1 )
		{
			ASSERT( 0 ) ;
			m_cFeedDef.m_cLayerDef.m_nLayerCount = 1 ;
		}
		return ( top - bot ) / m_cFeedDef.m_cLayerDef.m_nLayerCount ;
	}
	else if( m_cFeedDef.m_cLayerDef.m_nLayerType == NCDEF_LAYER_DEPTH )
		return m_cFeedDef.m_cLayerDef.m_dSideDInc ; 
	else 
		return ( top - bot ) ;
}
// 定义开槽分层　
BOOL CSmartPathGen::DefineSlotDepth( JDNC_SLOT& Slot , /*开槽深度*/
									 JDNC_TOOLEX * Tool) /*雕刻刀具*/
{
	if( Slot.m_dDepthInc < 0.0 ) Slot.m_dDepthInc = 0.0 ;
	DOUBLE  dMaxInc = Slot.m_dSlotDInc ;
	if( dMaxInc < 0.01 ) dMaxInc = 0.01 ;
	if( Slot.m_nSlotType == NCDEF_TSLOT_DEPTH)
	{ // 等深分层
	    int nIncTime = (int)ceil( Slot.m_dDepthInc / dMaxInc ) ;
	    if( fabs( ( nIncTime - 1 ) * dMaxInc - Slot.m_dDepthInc ) < 0.001 )
	        nIncTime -= 1 ;
		Slot.m_nSlotCount = nIncTime ;
	}
    if( Slot.m_nSlotCount < 1 ) Slot.m_nSlotCount = 1 ;
	if( Slot.m_dSlotDepth ) delete[] Slot.m_dSlotDepth ;
	Slot.m_dSlotDepth = new double[Slot.m_nSlotCount+1] ;
	dMaxInc  = Slot.m_dDepthInc /  double(Slot.m_nSlotCount) ;
	for( int i = 0 ; i <= Slot.m_nSlotCount ; i ++ ) 
	{
		Slot.m_dSlotDepth[i] = i * dMaxInc ;
	}
	if( ( Slot.m_bSlotFlag & NCDEF_FSLOT_ISOVOL ) &&
		( Tool && Tool->m_nToolType == surfncToolAFlat && Tool->m_dSideAng > 5  ) && 
		( Slot.m_nSlotCount > 1 && Slot.m_dDepthInc > 0.05 ) )
	{ // 等切削量开槽 && 锥刀 && 多次分层 && 存在深度
		DOUBLE dSolv[2] ;
		DOUBLE a = tan( ANGLE_TO_RADIAN( Tool->m_dSideAng/2.0) ) ;
		DOUBLE b = Tool->m_dBtmDiam;
		DOUBLE c = ( b + Slot.m_dDepthInc * a  ) * Slot.m_dDepthInc/ Slot.m_nSlotCount;
		BOOL bFindErr = FALSE  ;
		for( i = 1 ; i <= Slot.m_nSlotCount ; i ++ ) 
		{
			int nNum = mathSolQuadEqua( a, b, -c * i , 1.0e-6, dSolv ) ;
			if( nNum == 2 && dSolv[1] > dSolv[0] ) dSolv[0] = dSolv[1] ;
			if( nNum == 0 || dSolv[0] < 0.0 || dSolv[0] > Slot.m_dDepthInc + 0.001 )
			{
				bFindErr = TRUE ;	break; 
			}
			Slot.m_dSlotDepth[i] = dSolv[0] ; 
		}
        if( bFindErr )
		{
			for( i = 0 ; i <= Slot.m_nSlotCount ; i ++ ) 
			{ 
			  Slot.m_dSlotDepth[i] = i * dMaxInc ;
			} 
		}
	}
	return TRUE  ;
}
BOOL MathCAM_IsValidConnectLine( CSmartLine& TmpLine, 
                                 CSmartLoop& Contour,
                                 CSmartLoop* AllCont )
{
    for( CSmartLoop* pLoop = AllCont ; pLoop ; pLoop = pLoop->next )
    {/*必须落在轮廓中间*/
        if( pLoop->IsPtOnContour( TmpLine.m_aPoint[1]) ) 
        {
            break ;
        }
    }
    if( pLoop == NULL ) 
    {
        return FALSE ;
    }
    PNT2D dMidPnt ;
    TmpLine.GetPoint( 0.5, dMidPnt ) ;
	if( Contour.IsPtOnContour( dMidPnt ) &&
        Contour.IsPtOnContour( TmpLine.m_aPoint[1] ) &&
		!Contour.IntSectContour( &TmpLine ) ) 
	{  
        return TRUE ;
	}
    return FALSE ;
}
int  CSmartPathGen::FindPlungePoint( JDNC_PLUNGE& Plunge ,  /*下刀路径*/ 
									 CSmartLoop&  Contour,  /*原始轮廓*/ 
								     CSmartLoop* AllCont ,  /*边界曲线*/
								     PNT3D     ToPoint   ,  /*目标位置*/
								     PNT3D     AtPoint   ,  /*下刀位置*/
                                     CSmartCurve* OrgCurve) /*原始曲线*/ 
{
    PNT3D dTmpPnt;
    memcpy( AtPoint, ToPoint, sizeof( PNT3D ) ) ;
	if( Plunge.m_nPlungeType == NCDEF_PLUNGE_RAMP )
	{
        CSmartLine tmpLine ;
        tmpLine.m_aPoint[0][0] = ToPoint[0] ;
        tmpLine.m_aPoint[0][1] = ToPoint[1] ;
        if( (Plunge.m_nPositionType == NCDEF_PLUNGEPOS_REFER ) )
        {/*在指定位置下刀*/
            tmpLine.m_aPoint[1][0] = Plunge.m_dPlungePos[0] ;
            tmpLine.m_aPoint[1][1] = Plunge.m_dPlungePos[1] ;
            tmpLine.DefineBox() ;
            if( MathCAM_IsValidConnectLine(tmpLine, Contour, AllCont ) )
            {
                AtPoint[0] = tmpLine.m_aPoint[1][0] ;
                AtPoint[1] = tmpLine.m_aPoint[1][1] ;
                return TRUE ;
		    }
        }
        DOUBLE dAngle[2] = {0.0, MiniPai2 }, dInc = 1.0 / 36 ;
        if( OrgCurve )
        {
            PNT2D dNorm ;
            OrgCurve->m_pHead->GetNormal( 0.0, dNorm ) ;
            if( GetMillDir() == 0 ) 
            {
                nc_VectorReverse( dNorm, 2 ) ;
            }
            dAngle[0] = atan2( dNorm[1], dNorm[0] ) ;
            dAngle[1] = dAngle[0] + MiniPai2   ;
        }
        CSmartArc  tmpArc( ToPoint, dAngle, Plunge.m_dRadius) ;
        for( int nTry = 0 ; nTry <= 35 ; nTry ++ ) 
        {
            tmpArc.GetPoint( nTry * dInc, tmpLine.m_aPoint[1] ) ;
            tmpLine.DefineBox() ;
            if( MathCAM_IsValidConnectLine( tmpLine, Contour, AllCont ) )
            {
                AtPoint[0] = tmpLine.m_aPoint[1][0] ;
                AtPoint[1] = tmpLine.m_aPoint[1][1] ;
                return TRUE ;
		    }
        }
        return FALSE ;
	}
    else if( Plunge.m_nPlungeType == NCDEF_PLUNGE_HELIX )
	{/*螺旋下刀*/
		if( (Plunge.m_nPositionType == NCDEF_PLUNGEPOS_REFER ) )
		{/*在指定位置下刀*/
			CSmartLine tmpLine ;
			tmpLine.m_aPoint[0][0] = ToPoint[0] ;
			tmpLine.m_aPoint[0][1] = ToPoint[1] ;
			tmpLine.m_aPoint[1][0] = Plunge.m_dPlungePos[0] ;
			tmpLine.m_aPoint[1][1] = Plunge.m_dPlungePos[1] ;
			tmpLine.DefineBox() ; 
			if( MathCAM_IsValidConnectLine(tmpLine, Contour, AllCont ) )
			{
				VEC2D dNormal ;
				tmpLine.GetNormal( 0.0, dNormal ); 
				if( GetMillDir() == 1 ) 
				{
					nc_VectorReverse( dNormal, 2 ) ;
				}
				for ( int i = 0; i < 2; i++ )
				{
					AtPoint[i] = tmpLine.m_aPoint[1][i]+ Plunge.m_dRadius * dNormal[i] ;
				}
				for( CSmartLoop* pLoop = AllCont ; pLoop ; pLoop = pLoop->next )
				{
					if( pLoop->IsPtOnContour( AtPoint ) &&
						pLoop->IntLineContour( tmpLine.m_aPoint[0], tmpLine.m_aPoint[1] ) == FALSE ) 
					{ 
						return HELIX_REFF_TRUE ;
					}
				}
				AtPoint[0] = tmpLine.m_aPoint[1][0] ;
				AtPoint[1] = tmpLine.m_aPoint[1][1] ;
				return TRUE ;
			}
		}
		if( OrgCurve )
		{
			VEC2D dNormal , dStart;
			OrgCurve->m_pHead->GetNormal( 0.0, dNormal ) ;
			OrgCurve->m_pHead->GetPoint( 0.0, dStart ) ;
			nc_Normalize( dNormal, 2 ) ;
			if( GetMillDir() == 0 ) 
			{
				nc_VectorReverse( dNormal, 2 ) ;
			}
			for ( int i = 0; i < 2; i++ )
			{
				AtPoint[i] = ToPoint[i]+ Plunge.m_dRadius * dNormal[i] ;
			}
			for( CSmartLoop* pLoop = AllCont ; pLoop ; pLoop = pLoop->next )
			{
				if( pLoop->IsPtOnContour( AtPoint )&&
					( nc_DistanceSquare( dStart, ToPoint, 2) < 1.0e-8|| 
					  pLoop->IntLineContour( dStart, ToPoint ) == FALSE) )
				{ 
					return TRUE ;
				}
			}
		}
		for( CSmartLoop* pLoop = AllCont ; pLoop ; pLoop = pLoop->next )
		{
			if( pLoop->IsPtOnContour( ToPoint ) )
			{ 
				memcpy( AtPoint, ToPoint, sizeof( PNT3D ) ) ;
				return TRUE ;
			}
		}
		DOUBLE dDist, dMinDist = 1.0e12 ;
		for( pLoop = AllCont ; pLoop ; pLoop = pLoop->next )
		{/*判断是否落在轮廓内部*/
			CSmartLoop* pIsl = pLoop ;
			while( pIsl )
			{
				dTmpPnt[2] = ToPoint[2] ; 
				if( pIsl->m_pCurve->MinDistSect( ToPoint, dTmpPnt ,dDist ) && 
					dDist < dMinDist )
				{ 
					nc_VectorCopy( AtPoint, dTmpPnt,3 ) ;
					dMinDist = dDist ;
				} 
				if( pIsl == pLoop ) pIsl = pLoop->GetIsland() ;
				else pIsl = pIsl->next ;
			}
		}
		if( dMinDist < 0.005 ) return TRUE  ;
		else if( dMinDist < 1.0e3 )
		{ 
			CSmartLine cLine( AtPoint, ToPoint) ;
			cLine.GetPoint( 0.5, dTmpPnt ) ;
			if( Contour.IsPtOnContour( dTmpPnt ) && 
				!Contour.IntSectContour( &cLine ) ) 
			{  
				return TRUE ;
			}
		} 
	}
    else if( Plunge.m_nPlungeType == NCDEF_PLUNGE_VERT )
    {/*判断是否落在轮廓内部*/
        if( (Plunge.m_nPositionType == NCDEF_PLUNGEPOS_REFER ) )
        {/*在指定位置下刀*/
            CSmartLine tmpLine ;
            tmpLine.m_aPoint[0][0] = ToPoint[0] ;
            tmpLine.m_aPoint[0][1] = ToPoint[1] ;
            tmpLine.m_aPoint[1][0] = Plunge.m_dPlungePos[0] ;
            tmpLine.m_aPoint[1][1] = Plunge.m_dPlungePos[1] ;
            tmpLine.DefineBox() ;
            if( MathCAM_IsValidConnectLine(tmpLine, Contour, AllCont ) )
            {
                AtPoint[0] = tmpLine.m_aPoint[1][0] ;
                AtPoint[1] = tmpLine.m_aPoint[1][1] ;
                return TRUE ;
		    }
        }
	    for( CSmartLoop* pLoop = AllCont ; pLoop ; pLoop = pLoop->next )
	    {
	        if( pLoop->IsPtOnContour( ToPoint ) )
	        { 
			    memcpy( AtPoint, ToPoint, sizeof( PNT3D ) ) ;
			    return TRUE ;
	        }
	    }
	    DOUBLE dDist, dMinDist = 1.0e12 ;
	    for( pLoop = AllCont ; pLoop ; pLoop = pLoop->next )
	    {/*判断是否落在轮廓内部*/
		    CSmartLoop* pIsl = pLoop ;
		    while( pIsl )
		    {
			    dTmpPnt[2] = ToPoint[2] ; 
			    if( pIsl->m_pCurve->MinDistSect( ToPoint, dTmpPnt ,dDist ) && 
			        dDist < dMinDist )
			    { 
				    nc_VectorCopy( AtPoint, dTmpPnt,3 ) ;
			        dMinDist = dDist ;
			    } 
			    if( pIsl == pLoop ) pIsl = pLoop->GetIsland() ;
			    else pIsl = pIsl->next ;
		    }
	    }
	    if( dMinDist < 0.005 ) return TRUE  ;
	    else if( dMinDist < 1.0e3 )
	    { 
		    CSmartLine cLine( AtPoint, ToPoint) ;
		    cLine.GetPoint( 0.5, dTmpPnt ) ;
	        if( Contour.IsPtOnContour( dTmpPnt ) && 
			    !Contour.IntSectContour( &cLine ) ) 
		    {  
		        return TRUE ;
		    }
	    } 
    }
	return FALSE ;
}
BOOL CSmartPathGen::FindPlungeCurve(CSmartLoop&   Contour ,
								    PNT3D        Position,
								    CSmartCurve*& ObjCurve)
{
	PNT3D dEnd , dMid, dStart  ;
	double dDist, dMinDist = 1.0e12 ; 
	CSmartSect * pObjSect = NULL, *pSect ; 
	CSmartLoop* pObjLoop = NULL   ;
	ObjCurve = NULL ; 
	for( CSmartLoop  * pLoop = & Contour ; pLoop ;  )
	{
		pSect = pLoop->m_pCurve->MinDistSect( Position, dMid , dDist ) ;
		if( dDist < dMinDist )
		{
			pObjLoop = pLoop , pObjSect = pSect  ;
			dMinDist = dDist ; 
			memcpy( dEnd , dMid, sizeof( double[2] ) ) ;
		}
		if( pLoop == & Contour ) pLoop = Contour.GetIsland() ;
		else pLoop = pLoop->next ;
	}
	BOOL bFindAt = FALSE ;
	if( dMinDist < 1.0e-3 ) bFindAt = TRUE ;
	else if( dMinDist < 1.0e3 )
	{
		CSmartLine cLine( Position, dEnd) ;
		cLine.GetPoint( 0.5, dMid ) ;
	    if( Contour.IsPtOnContour( dMid ) && 
			!Contour.IntSectContour( &cLine ) ) 
			bFindAt = TRUE ;
	}
	if( bFindAt && pObjSect )
	{
		ObjCurve = new CSmartCurve() ; 
	    CSmartSect* pSub[2] ; 
		memcpy( dMid, dEnd , sizeof( double[2]) ) ;
        dMid[2] = pObjSect->GetParam( dMid ) ; 
	    pObjSect->GetPoint( 0.0, dStart ) , dStart[2] = 0.0 ;
	    pObjSect->GetPoint( 1.0, dEnd  ) , dEnd[2] = 1.0 ; 
	    pSub[0] = pObjSect->GetSubSect( dStart, dMid ) ;
	    pSub[1] = pObjSect->GetSubSect( dMid, dEnd ) ;
	    ObjCurve->AddSect( pSub[1] ) ;
        for( pSect = pObjSect->next ; pSect; pSect = pSect->next  )
		{
			ObjCurve->AddSect( pSect->CopyMyself() ) ;
		} 
		for( pSect = pObjLoop->GetSectHead() ; pSect ; pSect = pSect->next )
		{
			if( pSect == pObjSect ) break ;
			ObjCurve->AddSect( pSect->CopyMyself() ) ;
		}
	    if( pSub[0] ) ObjCurve->AddSect( pSub[0] ) ;
        if( ObjCurve->IsClosed())
        {
            if( GetMillDir() == 0 && m_bFollowOuter == 1 || 
                GetMillDir() == 1 && m_bFollowOuter == 0   )
            {
                ObjCurve->Reverse() ;
            }
        }
		return TRUE ;
	}
	return TRUE ;
}
int  CSmartPathGen::AddPlungePathBy( CPathCombine&   PComb  ,  /*路径组  */
							         JDNC_PLUNGE&    Plunge ,  /*下刀方式*/
								     PNT3D           ToPoint,  /*刀具位置*/
								     CSmartCurve& OrgCurve  ,  /*轮廓曲线*/
								     BOOL           IsStart )  /*对齐基准*/ 
{
	// STEP 0 : 计算长度
	CSmartCurve   closeCv;
    if( OrgCurve.IsClosed() == 0 )
    {
        PNT2D dStart, dEnd ;
        OrgCurve.m_pHead->GetStart( dStart ) ;
        for( CSmartSect* pSect = OrgCurve.m_pHead ; pSect ; pSect = pSect->next )
        {
            pSect->GetEnd( dEnd ) ;
            if( nc_Distance( dEnd, dStart, 2 ) < 0.0002 )
            {
                for( CSmartSect* pObj = OrgCurve.m_pHead ; pObj ; pObj = pObj->next )
                {
                    closeCv.AddSect( pObj->CopyMyself() ) ;
                    if( pObj == pSect ) break ;
                }
            }
        }
        if( closeCv.GetLength() < 0.02 )
        {
            closeCv.ClearAll() ;
        }
    }
    CSmartCurve& ByCurve = (closeCv.m_pHead) ? closeCv : OrgCurve ; 
	CSmartCurve   PlgCurve ;
	DOUBLE dCurveLen = ByCurve.GetLength() ;
    if( dCurveLen < 0.02 ) return FALSE ;
	DOUBLE dTan = tan( ANGLE_TO_RADIAN( Plunge.m_dAngle)) ;
    if( Plunge.m_dIncStep > Plunge.m_dMaxZInc )
    {
        double dMaxTan = Plunge.m_dMaxZInc / dCurveLen ;
        if( dTan > dMaxTan ) dTan = dMaxTan ;
    }
    DOUBLE dSlantLen = Plunge.m_dIncStep / dTan;
	int nLoop = (int)( dSlantLen / dCurveLen ) ;
	if( nLoop > 30 && dCurveLen < 1.0 ) nLoop  =  30  ;/*短线段保护*/
	// STEP 1 : 路径分段
	DOUBLE dLeft = dSlantLen - dCurveLen * nLoop ;
    CSmartCurve *pFirst = NULL, *pSecond = NULL  ;
    CSmartCurve TmpCurve ;
    if( ByCurve.IsClosed() == 0 )
	{ // 开口曲线
	    ByCurve.BreakCurveAt(  dLeft, pFirst, pSecond ) ;
		if( pFirst )
		{
		   pFirst->Reverse() ;
		   PlgCurve.AddCurve( pFirst ) ;
		}
		if( pSecond ) delete pSecond ;
	    if(  nLoop %2   )  nLoop ++ ;
	    for( int i = 0 ; i < nLoop ; i ++ ) 
		{
		   pFirst =  ByCurve.CopyMyself() ;
		   if( i % 2 ) pFirst->Reverse() ;
		   PlgCurve.AddCurve( pFirst ) ;
		}
	}
	else
	{ // 封闭曲线
	    if( IsStart )
	    {
	        ByCurve.BreakCurveAt(  dCurveLen - dLeft , pSecond, pFirst ) ;
	    }
	    else 
	    {
	        ByCurve.BreakCurveAt(  dLeft, pFirst , pSecond) ;
	    }
        if( pFirst ) PlgCurve.AddCurve( pFirst->CopyMyself() ) ;
        for( int i = 0 ; i < nLoop ; i ++  )
        {
	        if( pSecond ) PlgCurve.AddCurve( pSecond->CopyMyself() ) ;
	        if( pFirst  ) PlgCurve.AddCurve( pFirst->CopyMyself() ) ;
        }
        if( pFirst ) delete pFirst ;
        if( pSecond ) delete pSecond ;
	}		
	PComb.AddCurveEx( PlgCurve, Plunge.m_dIncStep, Plunge.m_dIncStep ) ;
	PNT3D dAtPoint ;
	if( PlgCurve.m_pHead &&  PComb.GetEndPoint( 1, dAtPoint ) && 
		nc_Distance( dAtPoint,ToPoint, 3 )  > 2.0e-4 )
	{
		CPathLine3D* pLine3D = new CPathLine3D( dAtPoint , ToPoint ) ;
		pLine3D->m_bFeedType = JDNC_FEEDTYPE_CONNECT ;
		PComb.AddEntity( pLine3D ) ;
	}
	return TRUE ;
}
int  CSmartPathGen::AddPlungePathRamp( CPathCombine&   PComb  ,  /*路径组  */
							           JDNC_PLUNGE&    Plunge ,  /*下刀方式*/
								       PNT3D           ToPoint,  /*刀具位置*/
								       CSmartCurve&   ByCurve ,  /*轮廓曲线*/
								       BOOL       /*IsStart*/ )  /*对齐基准*/ 
{
	// STEP 0 : 计算长度
	CSmartCurve   PlgCurve ;
	DOUBLE dCurveLen = ByCurve.GetLength() ;
	if( Plunge.m_dRadius < dCurveLen )
    {
        dCurveLen = Plunge.m_dRadius ;
    }
	DOUBLE dTan = tan( ANGLE_TO_RADIAN( Plunge.m_dAngle)) ;
    if( Plunge.m_dIncStep > Plunge.m_dMaxZInc )
    {
        double dMaxTan = Plunge.m_dMaxZInc / dCurveLen ;
        if( dTan > dMaxTan ) dTan = dMaxTan ;
    }
    DOUBLE dSlantLen = Plunge.m_dIncStep / dTan;
	int nLoop = (int)( dSlantLen / dCurveLen ) ;
	if( nLoop > 30 && dCurveLen < 1.0 ) nLoop  =  30 ;/*短线段保护*/
	else if( nLoop < 2 ) nLoop = 2 ;
    if(  nLoop %2   )  nLoop ++ ;
	// STEP 1 : 路径分段
    CSmartCurve *pFirst = NULL, *pSecond = NULL, *pNew  ;
    CSmartCurve TmpCurve ;
	ByCurve.BreakCurveAt(  dCurveLen, pFirst, pSecond ) ;
	if( pFirst == NULL )
	{
		pFirst = ByCurve.CopyMyself() ;
	}
	for( int i = 0 ; i < nLoop ; i ++ ) 
	{
		pNew =  pFirst->CopyMyself() ;
		if( i % 2 ) pNew->Reverse() ;
		PlgCurve.AddCurve( pNew ) ;
	}
	if( pFirst ) delete pFirst ;
	if( pSecond ) delete pSecond ;
	PComb.AddCurveEx( PlgCurve, Plunge.m_dIncStep, Plunge.m_dIncStep ) ;
	PNT3D dAtPoint ;
	if( PlgCurve.m_pHead &&  PComb.GetEndPoint( 1, dAtPoint ) && 
		nc_Distance( dAtPoint,ToPoint, 3 )  > 2.0e-4 )
	{
		CPathLine3D* pLine3D = new CPathLine3D( dAtPoint , ToPoint ) ;
		pLine3D->m_bFeedType = JDNC_FEEDTYPE_CONNECT ;
		PComb.AddEntity( pLine3D ) ;
	}
	return TRUE ;
}
BOOL CSmartPathGen::CreatePlungeAtBlank( CSmartLoop& Contour  , 
                                         PNT3D Position       , 
										 PNT3D dInfo[2]		  ,
									     DOUBLE MaxDist[2]    , 
                                         CSmartLoop* PartBnd  , 
                                         CSmartLoop* BlankBnd ,
										 int nMoveType,
									     CSmartCurve& PlgCurve)
{
    if( BlankBnd != NULL ) 
    {
        return CreatePlungeAtBlankEx( Position , dInfo, MaxDist, 
                                      PartBnd,BlankBnd, nMoveType, PlgCurve ) ;
    }
	// STEP 1 : 搜索开阔曲线段
	CSmartSect* pObjSect = NULL  ;
	DOUBLE dDist , dMinDist = MaxDist[0];
	PNT2D dLine[3], dPoint, dMidPnt ;
	nc_VectorCopy( dLine[0], Position, 2 ) ;
	PlgCurve.ClearAll() ;
    DOUBLE dNearPoint[3] = { Position[0], Position[1], dMinDist+0.1} ;
	for( CSmartSect* pSect = Contour.GetSectHead() ; pSect ; pSect = pSect->next )
	{
		if( !( pSect->m_bEndFlag & NC_BLANK_SECT ) ) continue ;
		dDist = pSect->MinDistPoint( Position, dPoint ) ;
        if( dDist < dNearPoint[2] )
        {
            nc_VectorCopy( dNearPoint, dPoint, 2 ) ;
            dNearPoint[2] = dDist ;
        }
		if( dDist > dMinDist ) continue ;
		nc_GetPointAtLine( Position, dPoint, 0.5, dMidPnt, 2 ) ;
		if(  Contour.IsPtOnContour( dMidPnt ) == 0 ||
			 Contour.IntLineContour( Position, dPoint, TRUE )  )
		{
			continue ;
		}
		dMinDist = dDist ;
		pObjSect = pSect ;
		nc_VectorCopy( dLine[1], dPoint, 2 ) ;
	}
	if( pObjSect == NULL && dNearPoint[2] < dMinDist ) 
    {
        CSmartLine tmpLine( Position, Position) ;
        DOUBLE dAngle = atan2( dNearPoint[1]- Position[1], dNearPoint[0]- Position[0]) ;
        PNT2D dIntPnt[5] ;
        for( int nTry = 0  ; nTry < 36 ; nTry ++ ) 
        {
            dPoint[0] = Position[0] + MaxDist[0] * cos( dAngle ) ;
            dPoint[1] = Position[1] + MaxDist[0] * sin( dAngle ) ;
            dAngle += ANGLE_TO_RADIAN( 10.0 ) ;
            nc_VectorCopy( tmpLine.m_aPoint[1], dPoint, 2 ) ;
            tmpLine.DefineBox() ;
	        for( pSect = Contour.GetSectHead() ; pSect ; pSect = pSect->next )
	        {
		        if( !( pSect->m_bEndFlag & NC_BLANK_SECT ) ) continue ;
                if( !nc_BoxBoxInt( pSect->m_dBox, tmpLine.m_dBox, 0.001 ) )
                {
                    continue ;
                }
                int nIntCnt = Mini_SectSectInt( tmpLine, *pSect, dIntPnt ) ;
                if( nIntCnt == 0 ) continue ;
                else if( nIntCnt == 2 )
                {
                    if( nc_Distance( dIntPnt[0], Position, 2 ) > 
                        nc_Distance( dIntPnt[1], Position, 2 )  )
                    {
                        nc_VectorCopy( dIntPnt[0], dIntPnt[1], 2 ) ;
                    }
                }
                nc_VectorCopy( dPoint, dIntPnt[0], 2 ) ;
		        dDist = nc_Distance( Position, dPoint, 2 ) ;
		        if( dDist > dMinDist ) continue ;
		        nc_GetPointAtLine( Position, dPoint, 0.5, dMidPnt, 2 ) ;
		        if(  Contour.IsPtOnContour( dMidPnt ) == 0 ||
			         Contour.IntLineContour( Position, dPoint, TRUE )  )
		        {
			        continue ;
		        }
		        dMinDist = dDist ;
		        pObjSect = pSect ;
		        nc_VectorCopy( dLine[1], dPoint, 2 ) ;
	        }
        }
    }
    if( pObjSect == NULL ) return FALSE ; 
	// STEP 2 : 延伸下刀路径段
	VEC2D dVec ;
	double u = pObjSect->GetParam( dLine[1] ) ;
	pObjSect->GetNormal( u, dVec ) ;
	dLine[2][0]  =  dLine[1][0] - dVec[0] * MaxDist[1] ;
	dLine[2][1]  =  dLine[1][1] - dVec[1] * MaxDist[1] ;
    for( CSmartLoop* pLoop = PartBnd ; pLoop ; pLoop = pLoop->next)
	{
		if(  pLoop->IsPtOnContour( dLine[2] ) == 1 ||
			 pLoop->IntLineContour( dLine[1], dLine[2] )  )
		{
			return  FALSE;
		}
	}
	// STEP 3 : 生成下刀路径
	if( nc_Distance( dLine[0], dLine[1], 2 ) < 0.01 || 
		nc_Distance( dLine[1], dLine[2], 2 ) < 0.01 || 
		nc_PointLineDist2D( dLine[0], dLine[2], dLine[1][0], dLine[1][1] ) < 0.01 )
	{
		CSmartLine* pLine = new CSmartLine( dLine[2], dLine[0] ) ;
		PlgCurve.AddSect( pLine ) ;
	}
	else
	{
		CSmartLine* pLine = new CSmartLine( dLine[2], dLine[1] ) ;
		PlgCurve.AddSect( pLine ) ;
		pLine = new CSmartLine( dLine[1], dLine[0] ) ;
		PlgCurve.AddSect( pLine ) ;
	}
	return  TRUE ;
}
BOOL CSmartPathGen::AddPlungeLeadPath( CSmtCheckMdl& DriveMdl    , 
                                       JDNC_CONNECT3D& ConnectDef, 
                                       CSmtCPathLib& AllPath     )
{/*添加下刀进退刀路径*/
    if( ConnectDef.m_nLead3DType == NCDEF_LEAD3D_CLOSE )
    {
        return TRUE ;
    }
	BOOL bLeadEnd = ConnectDef.m_bConnect3DFlag & NCDEF_FCONNECT3D_LEADEND; //仅在起末位置增加进退刀
    JDNC_CONNECT3D tmpCnt3D = ConnectDef ;
    tmpCnt3D.m_dLead3DLine = 0.0 ;
    JDNC_TOL tmpTol = GetSurfaceTol() ;
    CSmtCutPath tmpPath ;
	FPNT3D fPoint , fTan ;
    POSITION atTPos, atCurr ;
    CSmtCutPath* pTPath = NULL , *pTPrev = NULL, *pTNext = NULL ;
    atTPos = AllPath.m_cAllPath.GetHeadPosition() ;
    while( atTPos ) 
    {
        atCurr = atTPos, pTPrev = pTPath ;
        pTPath = AllPath.m_cAllPath.GetNext( atTPos ) ;
        if( atTPos == NULL ) pTNext = NULL ;
        else pTNext = AllPath.m_cAllPath.GetAt( atTPos ) ;
		if( pTPath->GetCutMode() == MINI_DRILL_PATH || pTPath->NumPoint () < 2 )
        {
			continue ;
        }
        if( pTPath->m_bFeedType == JDNC_FEEDTYPE_LEAD || 
            pTPath->m_bFeedType == JDNC_FEEDTYPE_PLUNGE  )
        {
            continue ;
        }
        if( pTPrev == NULL || 
            pTPrev->GetCutMode() == MINI_DRILL_PATH || pTPrev->NumPoint () < 2 ||
            (!bLeadEnd && nc_Distance( pTPrev->m_pTail->m_fPoint, pTPath->m_pHead->m_fPoint, 3 ) > 0.002 ) )
        {/*添加起点进刀路径*/
            nc_VectorCopy( fPoint, pTPath->m_pHead->m_fPoint, 3 ) ;
            if( pTPath->GetLeadTangent( 0, fTan ) && 
                MathCAM_GenNormalLeadPathEx(DriveMdl, tmpTol, tmpCnt3D, fPoint, fTan, tmpPath ) )
            {
                tmpPath.ReverseDirect() ;
                CSmtCutPath* pLeadPath = new CSmtCutPath(MINI_CONNECT_PATH) ;
                pLeadPath->m_bFeedType = JDNC_FEEDTYPE_LEAD ;
                pLeadPath->AppendCutPoint( tmpPath.m_pHead ) ;
                tmpPath.m_pHead = tmpPath.m_pTail = NULL ;
                tmpPath.m_nNumPnt = 0 ;
                AllPath.m_cAllPath.InsertBefore( atCurr, pLeadPath ) ;
            }
        }
        if( pTNext == NULL || 
            pTNext->GetCutMode() == MINI_DRILL_PATH || pTNext->NumPoint () < 2 ||
            (!bLeadEnd && nc_Distance( pTPath->m_pTail->m_fPoint, pTNext->m_pHead->m_fPoint, 3 ) > 0.002 ) )
        {/*添加末点退刀路径*/
            nc_VectorCopy( fPoint, pTPath->m_pTail->m_fPoint, 3 ) ;
            if( pTPath->GetLeadTangent( 1, fTan ) &&
                MathCAM_GenNormalLeadPathEx(DriveMdl, tmpTol, tmpCnt3D, fPoint, fTan, tmpPath ) )
            {
                CSmtCutPath* pLeadPath = new CSmtCutPath(MINI_CONNECT_PATH) ;
                pLeadPath->m_bFeedType = JDNC_FEEDTYPE_LEAD ;
                pLeadPath->AppendCutPoint( tmpPath.m_pHead ) ;
                tmpPath.m_pHead = tmpPath.m_pTail = NULL ;
                tmpPath.m_nNumPnt = 0 ;
                AllPath.m_cAllPath.InsertAfter( atCurr, pLeadPath ) ;
            }
        }
    }
    return TRUE ;
}


////////////////////////////
BOOL MathCAM_ValidPlungePoint( PNT2D Position, PNT2D Point, VEC2D Vec, DOUBLE MaxDist,
                               CSmartLoop* PartBnd  , CSmartLoop* BlankBnd )
{
    PNT2D dOrgPnt , dMidPnt ; 
    nc_VectorCopy( dOrgPnt, Point, 2 ) ;
    DOUBLE dCos  = cos( MiniPai2 / 36.0 ) ;
  	DOUBLE dSin  = sin( MiniPai2 / 36.0 ) ;
    BOOL bGoodPlunge = FALSE ;
    for( int nTry = 0 ; nTry < 35 ; nTry ++ ) 
    {
        if( nTry != 0  )
        {
   	        double x = Vec[0] * dCos - Vec[1] * dSin ;
	        double y = Vec[0] * dSin + Vec[1] * dCos ;
            Vec[0] = x , Vec[1] = y ; 
        }
	    Point[0]  =  dOrgPnt[0] - Vec[0] * MaxDist ;
	    Point[1]  =  dOrgPnt[1] - Vec[1] * MaxDist ;
		nc_GetPointAtLine( Position, Point, 0.5, dMidPnt, 2 ) ;
        for( CSmartLoop* pLoop = PartBnd ; pLoop ; pLoop = pLoop->next)
	    {/*不能与加工边界环相交*/
		    if(  pLoop->IsPtOnContour( dMidPnt ) == 1 ||
			    pLoop->IntLineContour( Position, Point )  )
		    {
			    break;
		    }
	    }
        if( PartBnd && pLoop ) continue ;
        for( pLoop = BlankBnd ; pLoop ; pLoop = pLoop->next)
	    {/*不能与毛坯边界环相交*/
            if( pLoop->IsPtOnContour( Point ) == 1 ||
		        pLoop->IntLineContour( dOrgPnt, Point )  )
            {
			    break;
		    }
        }
        if( BlankBnd && pLoop ) continue ;
        bGoodPlunge = TRUE ;
        break ;
    }
    return bGoodPlunge ;
}
BOOL MathCAM_PlungeCurveValid( CSmartCurve *LeadCurve, CSmartLoop *PartBnd, CSmartLoop *BlankBnd )
{
	UNUSED_ALWAYS( BlankBnd ) ;
	PNT2D mid ;
	double del = 0.25 ;
	CSmartLoop *pLoop = NULL ;
	// 首先判断点是否在轮廓里,采样三个点
	for( int i = 1 ; i < 4 ; i++ )
	{
		LeadCurve->GetPoint ( del * i, mid ) ;
		
		for( pLoop = PartBnd ; pLoop ; pLoop = pLoop->next )
		{
			if( pLoop->IsPtOnContour ( mid ) == 1 )
				return FALSE ;
		}
	}

	// 判断LeadCurve不与PartBnd\BlankBnd相交
	CSmartCurve *pCurve = NULL ;
	for( pLoop = PartBnd ; pLoop ; pLoop = pLoop->next )
	{
		pCurve = pLoop->m_pCurve ;
		for( ; pCurve ; pCurve = pCurve->next )
		{
			if( pCurve->CurveCurveInt ( LeadCurve ) )
				return FALSE ;
		}
		CSmartLoop *pIsland = pLoop->GetIsland() ;
		for( ; pIsland ; pIsland = pIsland->next )
		{
			pCurve = pIsland->m_pCurve ;
			if( pCurve->CurveCurveInt ( LeadCurve ) )
				return FALSE ;
		}
	}
	for( pLoop = BlankBnd ; pLoop ; pLoop = pLoop->next )
	{
		pCurve = pLoop->m_pCurve ;
		for( ; pCurve ; pCurve = pCurve->next )
		{
			if( pCurve->CurveCurveInt( LeadCurve ) )
				return FALSE ;
		}
		CSmartLoop *pIsland = pLoop->GetIsland() ;
		for( ; pIsland ; pIsland = pIsland->next )
		{
			pCurve = pIsland->m_pCurve ;
			if( pCurve->CurveCurveInt ( LeadCurve ) )
				return FALSE ;
		}
	}
	
	return TRUE ;
}
CSmartCurve * MathCAM_CreateArcLeadPath( PNT3D bgn, PNT3D pos, PNT3D dInfo[2], double dAng, CSmartLoop *PartBnd, CSmartLoop *BlankBnd )
{
	double dDist = mathDist2D( bgn, pos ) ;
	VEC3D vDir = { 0., 0., 0. } ;
	int i = 0 ;
	PNT2D cen, start, end, mid ;
	dInfo[0][2] = dInfo[1][2] = 0. ;
	VEC3D vz, rot_tan, rot_nor ;
	if( mathVProductUnit( dInfo[1], dInfo[0], vz ) == ERUNSUC )
		return NULL ;
	mathRotVec( vz, pos, dAng, dInfo[0], rot_tan ) ;
	mathRotVec( vz, pos, dAng, dInfo[1], rot_nor ) ;

	// 得到Bgn->pos的方向
	if( mathGetVecByPP2D( pos, bgn, MIN_LEN, vDir ) == ERUNSUC )
		return NULL ;
	
	double dAngle = mathGetAngleUnit( dInfo[1], vDir ) ;
	if( dAngle > PI1_2 * 5/6 ) return NULL ;
	
	// 生成圆弧
	mathCpyPnt2D( pos, end ) ;
	// 试验不同长度曲线
	double dR = min( 0.5, dDist * 0.25 ), dTry[4] = { 1, 0.75, 0.5, 0.25 }, r = 0 ;

	CSmartCurve *pCurve = NULL ;
	for( int nTry = 0 ; nTry < 4 ; nTry++ )
	{
		r = dR * dTry[nTry] ;
		for( i = 0 ; i < 2 ; i++ )
		{
			cen[i] = pos[i] + rot_nor[i] * r ;
		}
		// 从圆心向bgn->-rot_nor方向投影,得到垂足为圆弧start点
		double dTmp = mathDist2D( cen, bgn ) ;
		double tanDist = sqrt( dTmp * dTmp - r * r ) ;
		VEC3D tmpV = { 0., 0., 0. }, rot_tmpV ;
		if( mathGetVecByPP2D( bgn, cen, MIN_LEN, tmpV ) == ERUNSUC )
			continue ;

		double tanAng = asin( r / dTmp ) ;
		mathRotVec( vz, bgn, tanAng, tmpV, rot_tmpV ) ;
		for( i = 0 ; i < 2 ; i++ )
		{
			start[i] = bgn[i] + rot_tmpV[i] * tanDist ;
			mid[i] = ( start[i] + end[i] ) * 0.5 ;
		}
		if( fabs( mathDist2D( start, cen ) - r ) > 0.01 ) 
			continue ;

		if( fabs( mathDist2D( start, end ) - r * 2 ) > 0.005 )
		{ // 小圆弧
			if( mathGetVecByPP2D( cen, mid, MIN_LEN, tmpV ) == ERUNSUC )
				continue ;
			for( i = 0 ; i < 2 ; i++ )
			{
				mid[i] = cen[i] + r * tmpV[i] ;
			}
		}
		else
		{ 
			continue ;
		}

		// 第一个为直线
		pCurve = new CSmartCurve() ;
		CSmartLine *pLine = new CSmartLine( bgn, start ) ;
		pCurve->AddSect ( pLine ) ;
		
		// 生成圆弧
		CSmartArc *pArc = new CSmartArc() ;
		if( pArc->Define3PArc( start, mid, end ) )
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
		if( !pCurve ) continue ;

		if( !MathCAM_PlungeCurveValid( pCurve, PartBnd, BlankBnd ) )
		{
			delete pCurve ;
			pCurve = NULL ;
		}
		else
		{
			break ;
		}
	}
	return pCurve ;
}

BOOL MathCAM_ValidArcPlunge( PNT2D Position, PNT3D dInfo[2], 
                             CPointList &allPnt, DOUBLE MaxDist,
							 CSmartLoop* PartBnd, CSmartLoop* BlankBnd, 
							 CSmartCurve & PlgCurve)
{	
	double u = 0. ;
	PNT2D dVec ;
	PNT3D dPoint, pos = { Position[0], Position[1], 0. } ;
	CSmartLPoint *pHead = ( CSmartLPoint *)allPnt.m_pHead ;
	int nTry = 0 ; // 试验三次
	CSmartCurve *LeadCurve = NULL ;
	double dAngle = ANGLE_TO_RADIAN( 3 ) ;
	// 生成圆弧切入
	for( ; pHead ; pHead = (CSmartLPoint*)pHead->next )
    {
        nc_VectorCopy( dPoint, pHead->m_dPoint, 3 ) ;
        u = pHead->m_pSect->GetParam( dPoint ) ;
	    pHead->m_pSect->GetNormal( u, dVec ) ;
		// 首先判断CPointList中点的可靠性
        if( !MathCAM_ValidPlungePoint( Position, dPoint, dVec, MaxDist, PartBnd, BlankBnd ) )
			continue ;
		
		//计算进刀线,并判断是否与BlankBnd和PartBnd相交
		for( nTry = 0 ; nTry < 3 ; nTry++ )
		{
			LeadCurve = MathCAM_CreateArcLeadPath( dPoint, pos, dInfo, dAngle * nTry, PartBnd, BlankBnd ) ;
			if( !LeadCurve ) 
				continue ;
			else
			{
				PlgCurve.AddCurve ( LeadCurve ) ;
				return TRUE ;
			}
		}
	}
	return FALSE ;
}
// 在毛坯外部生成下刀路径
int CSmartPathGen::CreatePlungeAtBlankEx( PNT3D Position       , /*下刀位置               */
										  PNT3D dInfo[2]	   , // 切向法向
						                  DOUBLE MaxDist[2]    , /*[0]最大长度,[1]延伸长度*/
                                          CSmartLoop* PartBnd  , /*模型边界, 用于检查是否过切*/
                                          CSmartLoop* BlankBnd , /*毛坯边界, 用于查找引入位置*/
										  int	nMoveType	   , //走刀类型
							              CSmartCurve& PlgCurve) /*下刀路径*/
{
	// STEP 1 : 最近点搜索毛坯边界上的点
	CSmartSect* pObjSect = NULL, *pSect  ;
	DOUBLE dDist , dMinDist = MaxDist[0];
	PNT2D dLine[3], dVec ;
	PNT3D dPoint ;
	nc_VectorCopy( dLine[0], Position, 2 ) ;
	PlgCurve.ClearAll() ;
    DOUBLE dNearPoint[3] = { Position[0], Position[1], dMinDist+0.1} ;
    CSmtLoopArr  blankArr ;
    for( CSmartLoop* pBlank = BlankBnd ; pBlank ; pBlank = pBlank->next )
    {
        pBlank->ExtractAllLoop( 0, blankArr ) ;
    }
    INT_PTR nNumBlk = blankArr.GetSize() ;
    CPointList  allSPnt ;
    for( INT_PTR i = 0 ; i < nNumBlk ; i ++ ) 
    {
        CSmartLoop* pIsl = blankArr.GetAt(i) ;
	    for( pSect = pIsl->GetSectHead(); pSect ; pSect = pSect->next )
	    {
			// 必须在NC_BLANK_SECT段
			if( !( pSect->m_bEndFlag & NC_BLANK_SECT ) )
				continue ;
		    dPoint[2] = pSect->MinDistPoint( Position, dPoint ) ;
            if( dPoint[2] < dNearPoint[2] )
            {
                nc_VectorCopy( dNearPoint, dPoint, 3 ) ;
            }
            if( dPoint[2] > dMinDist )
            {
                continue ;
            }
            CSmartLPoint* pTPnt = new CSmartLPoint( dPoint, 0, 1 ) ;
            pTPnt->m_pSect = pSect ;
            allSPnt.InsertAfter( pTPnt, allSPnt.m_pTail ) ;
        }
    }
    allSPnt.SortPoint() ;/*距离从小到大排序*/
	// 启用圆弧连接
	if( PartBnd && BlankBnd && nMoveType == surfncMoveFollow &&
		MathCAM_ValidArcPlunge( Position, dInfo, allSPnt, MaxDist[1], 
								PartBnd, BlankBnd, PlgCurve ) )
	{
		return TRUE ;
	}
	
    for( CSmartLPoint* pTAt = (CSmartLPoint*)allSPnt.m_pHead ; pTAt ; pTAt = (CSmartLPoint*)pTAt->next )
    {
        nc_VectorCopy( dPoint, pTAt->m_dPoint, 3 ) ;
        double u = pTAt->m_pSect->GetParam( dPoint ) ;
	    pTAt->m_pSect->GetNormal( u, dVec ) ;
        if( MathCAM_ValidPlungePoint( Position, dPoint, dVec,MaxDist[1],PartBnd, BlankBnd ) )
        {
		    dMinDist = dPoint[2] ;
		    pObjSect = pTAt->m_pSect ;
		    nc_VectorCopy( dLine[1], dPoint, 2 ) ;
            break ;
        }
	}
	// STEP 2 : 直线旋转求交计算可能的点
	if( pObjSect == NULL && dNearPoint[2] < dMinDist ) 
    {
        CSmartLine tmpLine( Position, Position) ;
        DOUBLE dAngle = atan2( dNearPoint[1]- Position[1], dNearPoint[0]- Position[0]) ;
        PNT2D dIntPnt[5] ;
        for( int nTry = 0  ; nTry < 36 ; nTry ++ ) 
        {
            dPoint[0] = Position[0] + MaxDist[0] * cos( dAngle ) ;
            dPoint[1] = Position[1] + MaxDist[0] * sin( dAngle ) ;
            dAngle += ANGLE_TO_RADIAN( 10.0 ) ;
            nc_VectorCopy( tmpLine.m_aPoint[1], dPoint, 2 ) ;
            tmpLine.DefineBox() ;
            if( tmpLine.m_pInterPt ) 
            {
                delete tmpLine.m_pInterPt ;
                tmpLine.m_pInterPt = NULL  ;
            }
            for( i = 0 ; i < nNumBlk ; i ++ ) 
            {
                CSmartLoop* pIsl = blankArr.GetAt(i) ;
	            for( pSect = pIsl->GetSectHead(); pSect ; pSect = pSect->next )
	            {
					// 必须在NC_BLANK_SECT段
					if( !( pSect->m_bEndFlag & NC_BLANK_SECT ) )
						continue ;
                    if( !nc_BoxBoxInt( pSect->m_dBox, tmpLine.m_dBox, 0.001 ) )
                    {
                        continue ;
                    }
                    int nIntCnt = Mini_SectSectInt( tmpLine, *pSect, dIntPnt ) ;
                    if( nIntCnt == 0 ) continue ;
                    else if( nIntCnt == 2 )
                    {
                        if( nc_Distance( dIntPnt[0], Position, 2 ) > 
                            nc_Distance( dIntPnt[1], Position, 2 )  )
                        {
                            nc_VectorCopy( dIntPnt[0], dIntPnt[1], 2 ) ;
                        }
                    }
                    nc_VectorCopy( dPoint, dIntPnt[0], 2 ) ;
		            dDist = nc_Distance( Position, dPoint, 2 ) ;
		            if( dDist > dMinDist ) continue ;
                    double u = pSect->GetParam( dPoint ) ;
	                pSect->GetNormal( u, dVec ) ;
                    if( MathCAM_ValidPlungePoint( Position, dPoint, dVec,MaxDist[1],PartBnd, BlankBnd ) )
                    {
		                dMinDist = dDist ;
		                pObjSect = pSect ;
		                nc_VectorCopy( dLine[1], dPoint, 2 ) ;
                    }
                }
            }
        }
    }
    if( pObjSect == NULL ) return FALSE ; 
	// STEP 3 : 生成下刀路径
	CSmartLine* pLine = new CSmartLine( dLine[1], dLine[0] ) ;
	PlgCurve.AddSect( pLine ) ;
	return  TRUE ;
}
////////////////////////////////////////
// 在固定的位置生成下刀路径, 不考虑干涉
int  CSmartPathGen::CreateHelixPlunge( JDNC_PLUNGE& Plunge  ,  /*下刀方式*/
								        CSmartLoop& Contour  ,  /*边界轮廓*/ 
								        CPathEntity* CurrEnt ,  /*当前路径*/
                                        CPathCombine& TComb  )  /*路径组  */

{
    // STEP 1 : 计算相切位置
    if( ! CurrEnt || Plunge.m_nPlungeType != NCDEF_PLUNGE_HELIX )
	{ /*螺旋下刀*/
        return FALSE ;
    }
    if( Plunge.m_nPositionType == NCDEF_PLUNGEPOS_REFER )
    {
        return FALSE ;
    }
    TPNT3D toPoint, toNor, toTan ;
    if( ! CurrEnt->GetEndPoint( 0, toPoint ) ||
        ! CurrEnt->GetEndNormal(0, toNor  )  || 
        ! CurrEnt->GetEndTangent(0, toTan  )    )
    {
        return FALSE ;
    }
    toPoint[2] = toTan[2] = 0.0 ;
    // STEP 2 : 计算螺旋圈数
    DOUBLE dRadius = Plunge.m_dRadius ;
    DOUBLE dBldRad = 0.05 ;
    if( Plunge.m_bPlungeFlag & NCDEF_PLUNGE_DELBLINDAREA )
    {
        dBldRad = max( 0.05, Plunge.m_dBlindRadius - 0.01 ) ;
    }
 	if( dRadius < dBldRad ) return FALSE ;
	DOUBLE dTan   = tan( ANGLE_TO_RADIAN( Plunge.m_dAngle ) ) ;
	DOUBLE dDepth = Plunge.m_dIncStep ;
	double dLen = MiniPai2 * dRadius ;
    double dInc = min( dLen * dTan, Plunge.m_dMaxZInc) ; 
	int  nLoop = int ( dDepth / dInc ) ;
    double dLeft =  dDepth / dInc - nLoop ;
    if( dLeft < 0.05 && nLoop )
    {
	    dInc = dDepth / nLoop   ; 
        dLeft = 0.0 ;
    }
    else if( dLeft < 0.05 && nLoop == 0 )
    {
        dLeft = 0.05 ;
    }
	// STEP 3 : 寻找不过切的圆弧位置
    CSmartArc tmpArc2D, leftArc2D, *pTestArc = NULL ;
    tmpArc2D.m_dRadius = leftArc2D.m_dRadius = dRadius ;  
    if( nLoop == 0 ) pTestArc = &leftArc2D ;
    else pTestArc = &tmpArc2D ;
    if( GetMillDir() == 0 ) nc_VectorReverse( toNor, 2 ) ;
    for( int nTry = 0 ; nTry <= 1 ; nTry ++ )
    {
        if( nTry == 1 ) nc_VectorReverse( toNor, 2 ) ;
        tmpArc2D.m_aCenter[0] = toPoint[0] + toNor[0] * dRadius ;
        tmpArc2D.m_aCenter[1] = toPoint[1] + toNor[1] * dRadius ;
        tmpArc2D.m_aAngle[0]  = atan2( -toNor[1], -toNor[0] ) ;
        tmpArc2D.m_aAngle[1]  = tmpArc2D.m_aAngle[0] + MiniPai2 ;
        tmpArc2D.DefineBox() ; 
        VEC2D dArcTan ;
        tmpArc2D.GetTangent( 1.0, dArcTan ) ;
        if( nc_OProduct( dArcTan, toTan, 2 ) < 0.0 )
        {
            tmpArc2D.Reverse() ;
        }
        nc_VectorCopy( leftArc2D.m_aCenter, tmpArc2D.m_aCenter, 2) ;
        nc_VectorCopy( leftArc2D.m_aAngle , tmpArc2D.m_aAngle , 2) ;
        leftArc2D.m_aAngle[0] = leftArc2D.m_aAngle[0] + (1.0-dLeft) * (leftArc2D.m_aAngle[1]-leftArc2D.m_aAngle[0]) ;
        leftArc2D.DefineBox() ; 
        PNT2D dMidPnt ;
        pTestArc->GetPoint( 0.5, dMidPnt ) ;
        if( Contour.IsPtOnContour( dMidPnt ) &&
            !Contour.IntSectContour( pTestArc ) )
        {
            break ;
        }
    }
    if( nTry == 2 ) 
    {
        return FALSE ;
    }
	// STEP 4 : 生成螺旋下刀路径
    if( dLeft != 0.0 && leftArc2D.GetLength() > 0.01 )
    {
		CPathArc3D* pArc3D = new CPathArc3D() ;
		pArc3D->Create( dDepth, leftArc2D ) ;
		pArc3D->m_fDepth = - (dDepth- nLoop * dInc) ;
	    TComb.AddEntity( pArc3D ) ;
    }
	for(int i = 0 ; i < nLoop ; i ++ ) 
	{
		CPathArc3D* pArc3D = new CPathArc3D() ;
		pArc3D->Create( (nLoop- i) * dInc, tmpArc2D ) ;
		pArc3D->m_fDepth = - dInc ;
	    TComb.AddEntity( pArc3D ) ;
	} 
	return TRUE ; 
}
//三维清角在指定点处生成下刀路径
int CSmartPathGen::CreateVertPlungeAtPos( CPathCombine& TComb  , /*路径*/ 
	                                      JDNC_PLUNGE&  Plunge )  /*下刀*/
{
	if( ! TComb.m_pHead )
	{ // 不必插入下刀路径
		return  0 ;
	}	
	PNT2D dSeed[2];
	dSeed[0][0] = Plunge.m_dPlungePos[0];
	dSeed[0][1] = Plunge.m_dPlungePos[1];
	CPathCombine NewComb( NC_WPROCESS_ROUGH ) ;
	CPathEntity* pEnt = TComb.m_pHead ;
	TPNT3D dTPoint, dAt[2] ;
	DOUBLE dDist = 0.0 ;
	pEnt->GetEndPoint(0, dTPoint);
	DOUBLE dTmp = dTPoint[2];
	PNT2D st2d, end2d;
	int i = 0;
	BOOL bBreak = FALSE ;
	while( pEnt )
	{
		if(pEnt->GetType() != NC_PATH_PLINE3D)
		{
			CPathCombine TmpComb( NC_WPROCESS_ROUGH ) ;
			TmpComb.AddEntity(pEnt->CopyMyself());
			NewComb.AppendCombine(TmpComb);
			pEnt = pEnt->next ;
			continue ;
		}
		CPathPLine3D* pPLine3D = (CPathPLine3D*)pEnt;
		//step1:先把路径转为折线段并标记每段序号
		CSmartCurve* pCur = new CSmartCurve();
		for( i = 0 ; i < pPLine3D->m_nCount; i++)
		{
			st2d[0] = pPLine3D->m_pTAPos[i][0];
			st2d[1] = pPLine3D->m_pTAPos[i][1];
			end2d[0] = pPLine3D->m_pTAPos[i+1][0];
			end2d[1] = pPLine3D->m_pTAPos[i+1][1];
			CSmartLine* pSmtLn = new CSmartLine(st2d, end2d);
			pSmtLn->m_nSectId = i+1; 
			pCur->AddSect(pSmtLn);
			if(pPLine3D->m_pTAPos[i][2] < dTmp)
				dTmp = pPLine3D->m_pTAPos[i][2] ; 
		}
		if(pCur)
		{
			//step2:找到离指定点最近的下刀位置
			CSmartSect* pSect = NULL ;
			pSect = pCur->MinDistSect(dSeed[0], dSeed[1], dDist);
			if( pSect == NULL || pSect->m_nSectId == pPLine3D->m_nCount )
			{
				pCur->ClearAll();
				delete pCur;
				bBreak = TRUE ;
				break;
			}
			if(nc_Distance(dSeed[1], pPLine3D->m_pTAPos[0], 2) < MIN_DIS )
			{//下刀点在原始路径起点
				nc_VectorCopy(dAt[0], pPLine3D->m_pTAPos[0], 3);
				nc_VectorCopy(dAt[1], pPLine3D->m_pTAPos[0], 3);
				dAt[0][2] += Plunge.m_dIncStep ;
				CPathLine3D  *pPlunge = new CPathLine3D(dAt[0], dAt[1]) ;
				pPlunge->m_bFeedType = JDNC_FEEDTYPE_PLUNGE ;
				CPathCombine TmpComb( NC_WPROCESS_ROUGH ) ;
				TmpComb.AddEntity(pPlunge);
				TmpComb.InsertAfter(pEnt->CopyMyself(), pPlunge);
				NewComb.AppendCombine(TmpComb);
				pCur->ClearAll();
				delete pCur;
				pEnt = pEnt->next ;
				continue;
			}
			int iSect = pSect->m_nSectId ;

			//step3:确定下刀起点
			dAt[0][0] = dAt[1][0] = dSeed[1][0];
			dAt[0][1] = dAt[1][1] = dSeed[1][1];
			dAt[0][2] = dTmp + Plunge.m_dIncStep ;
			dAt[1][2] = dTmp ;
			
			if( nc_Distance(dAt[1], pPLine3D->m_pTAPos[iSect], 2) < MIN_DIS &&
				fabs( pPLine3D->m_pTAPos[iSect][2] - pPLine3D->m_pTAPos[iSect-1][2]) > MIN_DIS &&
				fabs( pPLine3D->m_pTAPos[iSect][2] - pPLine3D->m_pTAPos[iSect+1][2]) > MIN_DIS )//保证尖点处下刀点合理性
			{
				dAt[1][2] = pPLine3D->m_pTAPos[iSect][2] ;
				dAt[0][2] = dAt[1][2] + Plunge.m_dIncStep ;
			}
			if( nc_Distance(dAt[1], pPLine3D->m_pTAPos[iSect], 2) > MIN_DIS &&
				nc_Distance(dAt[1], pPLine3D->m_pTAPos[iSect-1], 2) > MIN_DIS &&
				fabs(pPLine3D->m_pTAPos[iSect-1][2]- pPLine3D->m_pTAPos[iSect][2]) > MIN_DIS)
			{
				dAt[0][0] = dAt[1][0] = pPLine3D->m_pTAPos[iSect][0];
				dAt[0][1] = dAt[1][1] = pPLine3D->m_pTAPos[iSect][1];
				dAt[0][2] = pPLine3D->m_pTAPos[iSect][2] + Plunge.m_dIncStep ;
				dAt[1][2] = pPLine3D->m_pTAPos[iSect][2] ;
			}
			//step4:在下刀点打断原来路径
			CPathPLine3D* pFirst = new CPathPLine3D(); 
			CPathPLine3D* pSecond = new CPathPLine3D();
			pFirst->Create(iSect);
			pSecond->Create(pPLine3D->m_nCount-iSect + 1);
			pFirst->m_bFeedType = pSecond->m_bFeedType = pPLine3D->m_bFeedType;
			pFirst->m_fFeedRate = pSecond->m_fFeedRate = pPLine3D->m_fFeedRate;

			for(i = 0 ; i <= pPLine3D->m_nCount; i++)
			{
				if(i < iSect)
				{
					mathCpyPnt( pPLine3D->m_pTAPos[i], pFirst->m_pTAPos[i] );
				}
				else if( i == iSect)
				{
					mathCpyPnt( dAt[1], pFirst->m_pTAPos[i] ) ;
					mathCpyPnt( dAt[1], pSecond->m_pTAPos[0]);
					mathCpyPnt( pPLine3D->m_pTAPos[i], pSecond->m_pTAPos[1] );
				}
				else
				{
					mathCpyPnt( pPLine3D->m_pTAPos[i], pSecond->m_pTAPos[i-iSect+1] );
				}
			}

			//step5:添加下刀生成新路径
			CPathCombine TmpComb( NC_WPROCESS_ROUGH ) ;
			CPathLine3D  *pPlunge = new CPathLine3D(dAt[0], dAt[1]) ;
			pPlunge->m_bFeedType = JDNC_FEEDTYPE_PLUNGE ;
			if(pPlunge)
				TmpComb.AddEntity(pPlunge);
			if(pSecond)
				TmpComb.InsertAfter(pSecond, pPlunge);
			if(pFirst)
				TmpComb.InsertAfter(pFirst, pSecond);		
			pCur->ClearAll();
			delete pCur;
			NewComb.AppendCombine(TmpComb);
		}
		pEnt = pEnt->next;
		pCur = NULL ;
	}
	//step6:新路径替换旧路径
	if( !bBreak && NewComb.m_pHead)
	{
		TComb.ClearAll();
		NewComb.DelOverlapPoint();
		TComb.AppendCombine(NewComb);
	}
	else if(NewComb.m_pHead)
	{
		NewComb.ClearAll();
	}
	return 1 ;
}
//在尖角前后指定长度处插入路径点 bFlag:0为当前点向前搜索插入点, 1为当前点向后搜索插入点。pCurPnt记录找到的插入位置
CSmtCutPoint* GetCPntAtDist(DOUBLE dist, CSmtCutPoint* p, CSmtCutPoint*& pCurPnt, BOOL bFlag)
{
	if( p == NULL)
		return NULL;
	DOUBLE dLen = 0.0 ;
	pCurPnt = NULL ;
	if(bFlag == 0)
	{
		while(p)
		{
			if(!p->prev)
			{
				return NULL ;
			}
			if(fabs(p->m_fPoint[2] - p->prev->m_fPoint[2]) > 1.0e-4)
			{
				return NULL ;
			}
			dLen += nc_Distance(p->m_fPoint, p->prev->m_fPoint, 3);
			if(dLen > dist)
			{
				float u = float((dLen - dist)/nc_Distance(p->m_fPoint, p->prev->m_fPoint, 2)) ;
				CSmtCutPoint* pInsert = p->prev->GenMidPoint(p, u);
				pCurPnt = p ;
				return pInsert ;
			}
			if( p->prev->m_bType == SMART_CUTPNT_BREAK )
			{
				return NULL;
			}
			p = p->prev ;
		}
	}
	else if(bFlag == 1)
	{
		while(p)
		{
			if(!p->next )
			{
				return NULL ;
			}
			if(fabs(p->next->m_fPoint[2] - p->m_fPoint[2]) > 1.0e-4)
			{
				return NULL ;
			}
			dLen += nc_Distance(p->m_fPoint, p->next->m_fPoint, 3);
			if(dLen > dist)
			{
				float u = 1.0f - float((dLen - dist)/nc_Distance(p->m_fPoint, p->next->m_fPoint, 2)) ;
				CSmtCutPoint* pInsert = p->GenMidPoint(p->next, u);
				pCurPnt = p ;
				return pInsert ;
			}
			if(p->next->m_bType == SMART_CUTPNT_BREAK)
			{
				return NULL;
			}
			p = p->next ;
		}
	}
	return NULL;
}
void MathCam_InsertCPntAtCornerDist(CSmtCutPath & Path, DOUBLE fSlowDist)
{
	if(!Path.m_pHead)
		return ;
	CSmtCutPoint *pCPnt = NULL ;
	CSmtCutPoint* pTmp = NULL ;
	CSmtCutPoint *&pFlagPnt = pTmp ;
	for( pCPnt = Path.m_pHead; pCPnt; pCPnt = pCPnt->next )
	{
		if(pCPnt && pCPnt->next)
		{
			if( pCPnt->next->m_fPoint[2] - pCPnt->m_fPoint[2] > 1.0e-4)
			{
				CSmtCutPoint* pNewPnt = GetCPntAtDist(fSlowDist, pCPnt, pFlagPnt, 0);
				if(pNewPnt != NULL)
				{
					pNewPnt->m_bType = SMART_CUTPNT_BREAK ;
					ASSERT(pFlagPnt != NULL);
					Path.InsertBefore(pNewPnt, pFlagPnt);
				}
			}
		}	

	}
	for( pCPnt = Path.m_pHead; pCPnt; pCPnt = pCPnt->next )
	{
		if(pCPnt && pCPnt->prev)
		{
			if( pCPnt->prev->m_fPoint[2] - pCPnt->m_fPoint[2] > 1.0e-4)
			{
				CSmtCutPoint* pNewPnt = GetCPntAtDist(fSlowDist, pCPnt, pFlagPnt, 1);
				if(pNewPnt != NULL)
				{
					pNewPnt->m_bType = SMART_CUTPNT_BREAK ;
					ASSERT(pFlagPnt != NULL);
					Path.InsertAfter(pNewPnt, pFlagPnt);
				}
			}
		}
	}
}

BOOL CSmartPathGen::SlowAtCorner( CPathGroup& PathGroup, 
								  float fSlowDist,   /*降速距离*/
								  float fSlowRate )  /*降速比率*/
{
	CSmtCPathLib AllPath ;
	CSmtCPathLib NewAllPath ;
	//将PathGroup转为CSmtCutPath
	MathCAM_AddPathGroup( AllPath, PathGroup, m_cSetupDef.m_cTolDef , TRUE ) ;
	if( AllPath.m_cAllPath.GetCount() < 1 )
		return FALSE ;
	if( fSlowDist <= MIN_DIS)
		return FALSE ;
	if( fSlowRate <= MIN_DIS)
		return FALSE ;
	CSmtCutPath* pCutPath = NULL ;
	//1.在断点处打断路径
	POSITION pos = AllPath.m_cAllPath.GetHeadPosition();
	while(pos)
	{
		pCutPath = AllPath.m_cAllPath.GetNext(pos);
		if( pCutPath->m_bFeedType != JDNC_FEEDTYPE_ROUGH)
		{
			CSmtCutPath* pCopyPath = pCutPath->CopyMyself();
			pCopyPath->m_bFeedType = pCutPath->m_bFeedType ;
			pCopyPath->m_fFeedRate = pCutPath->m_fFeedRate ;
			pCopyPath->m_bMoveFlag = pCutPath->m_bMoveFlag ;
			NewAllPath.m_cAllPath.AddTail(pCopyPath);
			continue ;
		}
		MathCam_InsertCPntAtCornerDist(*pCutPath, fSlowDist);
		CSmtCutPath * pNewPath = new CSmtCutPath();
		pNewPath->m_bFeedType = JDNC_FEEDTYPE_ROUGH;
		pNewPath->m_fFeedRate = pCutPath->m_fFeedRate ;
		pNewPath->m_bMoveFlag = pCutPath->m_bMoveFlag ;
		for(CSmtCutPoint* pCPnt = pCutPath->m_pHead ; pCPnt ; pCPnt = pCPnt->next)
		{
			pNewPath->AddPoint(pCPnt->m_fPoint);
			if(pCPnt == pCutPath->m_pTail)
			{
				NewAllPath.m_cAllPath.AddTail(pNewPath);
				break ;
			}
			if(pCPnt->m_bType == SMART_CUTPNT_BREAK)
			{
				NewAllPath.m_cAllPath.AddTail(pNewPath);
				pNewPath = new CSmtCutPath();
				pNewPath->m_bFeedType = JDNC_FEEDTYPE_ROUGH;
				pNewPath->m_fFeedRate = pCutPath->m_fFeedRate ;
				pNewPath->m_bMoveFlag = pCutPath->m_bMoveFlag ;
				pNewPath->AddPoint(pCPnt->m_fPoint);	
			}
		}
	}
	//2.在尖角路径处降速
	pos = NewAllPath.m_cAllPath.GetHeadPosition();
	while(pos)
	{
		pCutPath = NewAllPath.m_cAllPath.GetNext(pos);
		if(pCutPath->m_bFeedType != JDNC_FEEDTYPE_ROUGH)
			continue;
		for(CSmtCutPoint* pCutPnt = pCutPath->m_pHead; pCutPnt ; pCutPnt = pCutPnt->next)
		{
			if(pCutPnt && pCutPnt->next)
			{
				if(fabs( pCutPnt->m_fPoint[2] - pCutPnt->next->m_fPoint[2] )> 1.0e-4 )
				{
					pCutPath->m_fFeedRate *= fSlowRate ; 
					break;
				}
			}
		}
	}
	CPathCombine * pPComb = new CPathCombine( NC_WPROCESS_ROUGH ) ;
	pPComb->m_pEntFrom  = PathGroup.m_pEntFrom  ;
	NewAllPath.DelPointOnLine(2.0e-4) ; // 删除同一直线上的点
	NewAllPath.AddToPathCombine(*pPComb);
	CPathGroup tmpPGroup(NC_PATHTYPE_CPOCKET) ;
	tmpPGroup.AddData( 0.0 , pPComb ) ;
	NewAllPath.ClearAllPath() ;
	AllPath.ClearAllPath() ;
	PathGroup.ClearAll () ;   //删除原始路径
	while( tmpPGroup.m_pHead )
	{
		pPComb = tmpPGroup.m_pHead ;
		tmpPGroup.RemoveCombine( pPComb ) ;
		PathGroup.AddData( 0.0, pPComb ) ;
	}
	return TRUE;
}

void CSmartPathGen::ReDefinePathStartPntAtPlungePos(CPathCombine *PComb, JDNC_PLUNGE&  Plunge)
{
	if(!PComb 
		|| Plunge.m_nPlungeType == 0 
		|| Plunge.m_nPositionType != NCDEF_PLUNGEPOS_REFER)
		return;

	int nType = 0;
	PNT2D dSeed[2];
	dSeed[0][0] = Plunge.m_dPlungePos[0];
	dSeed[0][1] = Plunge.m_dPlungePos[1];
	TPNT3D dTPoint, dAt[2] ;
	DOUBLE dDist = 0.0, dMinDist = 1.0e6;
	int i = 0;
	BOOL bBreak = FALSE ;
	TPNT3D ptStart, ptEnd;
	CPathCombine pNewComb( NC_WPROCESS_ROUGH ) ;
	CPathEntity* pHead = NULL, *pTail = NULL;
	CPathEntity* pEnt = NULL, *pNextEnt = NULL;
	pEnt = PComb->m_pHead;
	pEnt->GetEndPoint(0, dTPoint);
	DOUBLE dTmp = dTPoint[2];
	while (pEnt)
	{
		dMinDist = 1.0e6;
		// step 1: 找出同一层路径
		CPtrList list;
		list.AddTail(pEnt);
		while (pEnt->next)
		{
			pNextEnt = pEnt->next;
			pEnt->GetEndPoint(1, ptEnd);
			pNextEnt->GetEndPoint(0, ptStart);
			if (nc_Distance(ptStart, ptEnd, 3) < MIN_DIS)
			{
				list.AddTail(pNextEnt);
				pEnt = pNextEnt;
				continue;
			}
			break;
		}
		
		pHead = (CPathEntity*)list.GetHead();
		pTail = (CPathEntity*)list.GetTail();
		if (pHead && pTail)
		{
			pHead->GetEndPoint(0, ptStart);
			pTail->GetEndPoint(1, ptEnd);
			if (mathDist(ptStart, ptEnd) > MIN_DIS)
			{// 不封闭路径不重设起点
				while (!list.IsEmpty())
				{
					pHead = (CPathEntity*)list.RemoveHead();
					if (pHead)
					{
						pNewComb.AddEntity(pHead);
					}
				}
				pEnt = pEnt->next;
				continue;
			}
		}

		// step 2: 转成CSmtCurve,并找出最近的下刀位置
		PNT2D st2d, end2d;
		PNT2D dFindSeed[2];
		CSmartCurve* pFindCurve = NULL;
		CSmartSect* pFindSect = NULL;
		CPathEntity* pPath = NULL, *pFindPath = NULL;
		for (POSITION pos = list.GetHeadPosition(); pos;)
		{
			pPath = (CPathEntity*)list.GetNext(pos);
			if (!pPath)
			{
				continue;
			}
			CSmartCurve* pCur = new CSmartCurve();
			nType = pPath->GetType();
			if (nType == NC_PATH_PLINE3D)
			{
				CPathPLine3D* pPLine3D = (CPathPLine3D*)pPath;				
				for(i = 0 ; i < pPLine3D->m_nCount; i++)
				{
					st2d[0] = pPLine3D->m_pTAPos[i][0];
					st2d[1] = pPLine3D->m_pTAPos[i][1];
					end2d[0] = pPLine3D->m_pTAPos[i+1][0];
					end2d[1] = pPLine3D->m_pTAPos[i+1][1];
					CSmartLine* pSmtLn = new CSmartLine(st2d, end2d);
					pSmtLn->m_nSectId = i+1;
					pCur->AddSect(pSmtLn);
				}
				CSmartSect* pSect = NULL ;
				pSect = pCur->MinDistSect(dSeed[0], dSeed[1], dDist);
				if (pSect)
				{
					if (dDist < dMinDist)
					{
						dMinDist = dDist;
						pFindPath = pPath;
						if (pFindCurve != NULL)
						{
							pFindCurve->ClearAll();
							delete pFindCurve;
							pFindCurve = NULL;
						}
						pFindSect = pSect;
						pFindCurve = pCur;
						mathCpyPnt2D(dSeed[0], dFindSeed[0]);
						mathCpyPnt2D(dSeed[1], dFindSeed[1]);
						pCur = NULL;
					}					
				}
			}
			else if (nType == NC_PATH_ARC3D)
			{
				CPathArc3D* pArc3D = (CPathArc3D*)pPath;
				CSmartArc* pSmtArc = new CSmartArc(pArc3D->m_fCenter, pArc3D->m_fAngle, pArc3D->m_fRadius);
				pCur->AddSect(pSmtArc);
				CSmartSect* pSect = NULL ;
				pSect = pCur->MinDistSect(dSeed[0], dSeed[1], dDist);
				if (pSect)
				{
					if (dDist < dMinDist)
					{
						dMinDist = dDist;
						pFindPath = pPath;
						if (pFindCurve != NULL)
						{
							pFindCurve->ClearAll();
							delete pFindCurve;
							pFindCurve = NULL;
						}
						pFindSect = pSect;
						pFindCurve = pCur;
						mathCpyPnt2D(dSeed[0], dFindSeed[0]);
						mathCpyPnt2D(dSeed[1], dFindSeed[1]);
						pCur = NULL;
					}					
				}
			}
			if (pCur)
			{
				pCur->ClearAll();
				delete pCur;
				pCur = NULL;
			}
		}
		// step 3: 重新连接路径
		CPtrList list1;		// 存放打断点前面的路径
		CPtrList list2;		// 存放打断点后面的路径
		bool bCanAddToList2 = false; // bCanAddToList2 = true时开始向list2中添加路径，否则向list1中添加路径
		for (POSITION pos = list.GetHeadPosition(); pos;)
		{
			pPath = (CPathEntity*)list.GetNext(pos);
			if (!pPath)
				continue;
			nType = pPath->GetType();
			if (pPath == pFindPath && pFindCurve && pFindSect)
			{
				bCanAddToList2 = true;
				if (nType == NC_PATH_PLINE3D)
				{
					CPathPLine3D* pPLine3D = (CPathPLine3D*)pPath;
					if( pFindSect->m_nSectId == pPLine3D->m_nCount )
					{
						list1.AddTail(pPath->CopyMyself());
						break;
					}
					if(nc_Distance(dFindSeed[1], pPLine3D->m_pTAPos[0], 2) < MIN_DIS )
					{//下刀点在原始路径起点
						list2.AddTail(pPath->CopyMyself());
						break;
					}
					int iSect = pFindSect->m_nSectId ;
					dTmp = min(pPLine3D->m_pTAPos[iSect][2], pPLine3D->m_pTAPos[iSect-1][2]);

					dAt[0][0] = dAt[1][0] = dFindSeed[1][0];
					dAt[0][1] = dAt[1][1] = dFindSeed[1][1];
					dAt[0][2] = dTmp + Plunge.m_dIncStep ;
					dAt[1][2] = dTmp ;

					if( nc_Distance(dAt[1], pPLine3D->m_pTAPos[iSect], 2) < MIN_DIS &&
						fabs( pPLine3D->m_pTAPos[iSect][2] - pPLine3D->m_pTAPos[iSect-1][2]) > MIN_DIS &&
						fabs( pPLine3D->m_pTAPos[iSect][2] - pPLine3D->m_pTAPos[iSect+1][2]) > MIN_DIS )//保证尖点处下刀点合理性
					{
						dAt[1][2] = pPLine3D->m_pTAPos[iSect][2] ;
						dAt[0][2] = dAt[1][2] + Plunge.m_dIncStep ;
					}
					if( nc_Distance(dAt[1], pPLine3D->m_pTAPos[iSect], 2) > MIN_DIS &&
						nc_Distance(dAt[1], pPLine3D->m_pTAPos[iSect-1], 2) > MIN_DIS &&
						fabs(pPLine3D->m_pTAPos[iSect-1][2]- pPLine3D->m_pTAPos[iSect][2]) > MIN_DIS)
					{
						dAt[0][0] = dAt[1][0] = pPLine3D->m_pTAPos[iSect][0];
						dAt[0][1] = dAt[1][1] = pPLine3D->m_pTAPos[iSect][1];
						dAt[0][2] = pPLine3D->m_pTAPos[iSect][2] + Plunge.m_dIncStep ;
						dAt[1][2] = pPLine3D->m_pTAPos[iSect][2] ;
					}
					CPathPLine3D* pFirst = new CPathPLine3D(); 
					CPathPLine3D* pSecond = new CPathPLine3D();
					pFirst->Create(iSect);
					pSecond->Create(pPLine3D->m_nCount-iSect + 1);
					pFirst->m_bFeedType = pSecond->m_bFeedType = pPLine3D->m_bFeedType;
					pFirst->m_fFeedRate = pSecond->m_fFeedRate = pPLine3D->m_fFeedRate;

					for(i = 0 ; i <= pPLine3D->m_nCount; i++)
					{
						if(i < iSect)
						{
							mathCpyPnt( pPLine3D->m_pTAPos[i], pFirst->m_pTAPos[i] );
						}
						else if( i == iSect)
						{
							mathCpyPnt( dAt[1], pFirst->m_pTAPos[i] ) ;
							mathCpyPnt( dAt[1], pSecond->m_pTAPos[0]);
							mathCpyPnt( pPLine3D->m_pTAPos[i], pSecond->m_pTAPos[1] );
						}
						else
						{
							mathCpyPnt( pPLine3D->m_pTAPos[i], pSecond->m_pTAPos[i-iSect+1] );
						}
					}
					list1.AddTail(pFirst);
					list2.AddTail(pSecond);
				}
				else if (nType == NC_PATH_ARC3D)
				{
					CPathArc3D* pArc3D = (CPathArc3D*)pPath;
					CSmartArc* pSmtArc = (CSmartArc*)pFindSect;
					VEC2D vFind, vX;
					vX[0] = 1.0;
					vX[1] = 0.0;
					mathGetVec2D(pSmtArc->m_aCenter, dFindSeed[1], vFind);
					double dAngle = mathGetAngle2D(vX, vFind, MIN_LEN);
					if (vFind[1] < 0)
					{
						dAngle = PI2 - dAngle;
					}
					if (dAngle < pArc3D->m_fAngle[0])
					{
						while (dAngle < pArc3D->m_fAngle[0])
						{
							dAngle += PI2;
						}
					}
					else
					{
						while (dAngle > pArc3D->m_fAngle[1])
						{
							dAngle -= PI2;
						}
					}

					if ((dAngle - pSmtArc->m_aAngle[0]) < MIN_DIS)
					{// 与首点重合
						list2.AddTail(pPath->CopyMyself());
					}
					else if ((pSmtArc->m_aAngle[1] - dAngle) < MIN_DIS)
					{// 与末点重合
						list1.AddTail(pPath->CopyMyself());
					}
					else
					{
						CPathArc3D* pFirst = (CPathArc3D*)pArc3D->CopyMyself();
						CPathArc3D* pSecond = (CPathArc3D*)pArc3D->CopyMyself();
						pFirst->m_fAngle[1] = dAngle;
						pSecond->m_fAngle[0] = dAngle;
						list1.AddTail(pFirst);
						list2.AddTail(pSecond);
					}					
				}
			}
			else
			{
				if (bCanAddToList2)
				{
					list2.AddTail(pPath->CopyMyself());
				}
				else
				{
					list1.AddTail(pPath->CopyMyself());
				}
			}
		}
		// step 4:排序路径
		for (POSITION pos = list1.GetHeadPosition(); pos;)
		{
			pPath = (CPathEntity*)list1.GetNext(pos);
			if (pPath)
			{
				list2.AddTail(pPath);
			}
		}
		while (!list2.IsEmpty())
		{
			pPath = (CPathEntity*)list2.RemoveHead();
			if (pPath)
			{
				pNewComb.AddEntity(pPath);
			}
		}

		if (pFindCurve)
		{
			pFindCurve->ClearAll();
			delete pFindCurve;
			pFindCurve = NULL;
		}
		pFindPath = NULL;
		pFindSect = NULL;

		pEnt = pEnt->next;
	}
	if( !bBreak && pNewComb.m_pHead)
	{
		PComb->ClearAll();
		pNewComb.DelOverlapPoint();
		PComb->AppendCombine(pNewComb);
	}
	else if(pNewComb.m_pHead)
	{
		pNewComb.ClearAll();
	}
}

void CSmartPathGen::AddLinkPathBetweenLayers(CPathCombine *PComb, CSmartLoop* AllCont, DOUBLE dAngle)
{
	if (!PComb || !AllCont || !m_pTool)
	{
		return;
	}
	JDNC_LAYER* pLayer = GetLayerDef();
	if (!pLayer)
		return;
	double dDepthInc = 0.0, dMaxDepthInc = 0.0;
	for (int i = 1; i < pLayer->m_nLayerCount; ++i)
	{
		dDepthInc = fabs(pLayer->m_dLayerDepth[i] - pLayer->m_dLayerDepth[i-1]);
		if (dDepthInc > dMaxDepthInc)
		{
			dMaxDepthInc = dDepthInc;
		}
	}
	double dMaxDist = m_pTool->GetRadiusComp(dMaxDepthInc, dAngle) - 
					  m_pTool->GetRadiusComp(0.0, 0.0) ;
	dMaxDist *= 4.0;
	// step 0: AllCont转成曲线
	CPtrList allCur;
	CSmartLoop* pIsLand = NULL;
	CSmartCurve* pCurve = NULL;
	for(CSmartLoop* pContour = AllCont ; pContour ; pContour = pContour->next )
	{
		pCurve = pContour->GetCurve();
		if (pCurve)
		{
			allCur.AddTail(pCurve);
		}
		pIsLand = pContour->m_pIsland;
		while (pIsLand)
		{
			pCurve = pIsLand->GetCurve();
			if (pCurve)
			{
				allCur.AddTail(pCurve);
			}
			pIsLand = pIsLand->next;
		}
	}

	// step 1: 创建loop
	BOOL bIntWithCur = FALSE;
	PNT2D ptInt[2];
	TPNT3D ptArr[2];
	CPtrList cLoopList;
	MathCam_GetPathLayerLoop(PComb, cLoopList);
	
	// step 2: 连接loop
	CPathLayerLoop* pCLoop = NULL, *pNLoop = NULL; 
	for (POSITION pos = cLoopList.GetHeadPosition(); pos;)
	{
		pCLoop = (CPathLayerLoop*)cLoopList.GetNext(pos);
		if (!pCLoop || !pCLoop->IsValid() || !pos)
		{
			continue;
		}
		pNLoop = (CPathLayerLoop*)cLoopList.GetAt(pos);
		if (!pNLoop || !pNLoop->IsValid())
		{
			continue;
		}
		pCLoop->m_pTail->GetEndPoint(1, ptArr[0]);
		pNLoop->m_pHead->GetEndPoint(0, ptArr[1]);
		if (ptArr[0][2] > ptArr[1][2] + MIN_DIS && mathDist2D(ptArr[0], ptArr[1]) < dMaxDist)
		{
			CSmartLine line(ptArr[0], ptArr[1]);
			// 使用曲线判断
			bIntWithCur = FALSE;
			for (POSITION curpos = allCur.GetHeadPosition(); curpos;)
			{
				CSmartCurve* pCurve = (CSmartCurve*)allCur.GetNext(curpos);
				if (!pCurve)
				{
					continue;
				}
				for (CSmartSect* pSect = pCurve->m_pHead; pSect; pSect = pSect->next)
				{
					if (0 != Mini_SectSectInt(*pSect, line, ptInt))
					{
						bIntWithCur = TRUE;
						break;
					}
				}
				if (bIntWithCur)
				{
					break;
				}
			}
			if (bIntWithCur)
			{
				continue;
			}
			// 使用路径判断
			// 通过面积的正负判断当前环与下一个环的方向是否相同，防止当前环与内部岛间连刀，2014.2.13 liuxin
			// 当前环和下一个环必须是包含关系并且连接路径与当前环和下一个环都不相交, 2014.3.5 liuxin
			BOOL bLoopIn[2] = {FALSE}, bSectInt[2] = {FALSE};
			bLoopIn[0] = pNLoop->m_pLoop->IsLoopOn(pCLoop->m_pLoop);
			bLoopIn[1] = pCLoop->m_pLoop->IsLoopOn(pNLoop->m_pLoop);
			bSectInt[0] = pCLoop->m_pLoop->IntSectContour(&line);
			bSectInt[1] = pNLoop->m_pLoop->IntSectContour(&line);
			if ((bLoopIn[0] || bLoopIn[1]) &&
				(!bSectInt[0] && !bSectInt[1]) &&
				((pCLoop->m_pLoop->m_dArea > 0 && pNLoop->m_pLoop->m_dArea > 0) ||
				 (pCLoop->m_pLoop->m_dArea < 0 && pNLoop->m_pLoop->m_dArea < 0)))
			{
				CPathPLine3D* pPLine3D = new CPathPLine3D( ) ;
				pPLine3D->m_pTAPos = new TPNT3D[3] ;
				mathCpyPnt(ptArr[0], pPLine3D->m_pTAPos[0]);
				mathCpyPnt(ptArr[1], pPLine3D->m_pTAPos[2]);
				pPLine3D->m_pTAPos[1][0] = pPLine3D->m_pTAPos[2][0];
				pPLine3D->m_pTAPos[1][1] = pPLine3D->m_pTAPos[2][1];
				pPLine3D->m_pTAPos[1][2] = max(pPLine3D->m_pTAPos[0][2], pPLine3D->m_pTAPos[2][2]);
				pPLine3D->m_nCount = 2 ;
				{
					pPLine3D->m_bFeedType = JDNC_FEEDTYPE_CONNECT;
				}
				if (pPLine3D)
				{
					PComb->InsertAfter(pPLine3D, pCLoop->m_pTail);
				}
			}
		}
	}

	// step 3: 删除loop
	while (!cLoopList.IsEmpty())
	{
		pCLoop = (CPathLayerLoop*)cLoopList.RemoveHead();
		if (pCLoop)
		{
			delete pCLoop;
		}
	}
}

void GetStartNormalAndTang( CPathEntity *pEnt, CSmartLoop *pLoop, int nFlag, PNT3D dInfo[2] )
{
	PNT3D start, p[2] ;
	pEnt->GetEndPoint ( 0, start ) ;
	pEnt->GetEndTangent ( 0, dInfo[0] ) ;
	pEnt->GetEndNormal ( 0, dInfo[1] ) ;
	dInfo[0][2] = dInfo[1][2] = 0. ;
	for( int i = 0 ; i < 3 ; i++ )
	{	// 正常p[0]在外,p[1] 在内
		p[0][i] = start[i] + 0.002 * dInfo[1][i] ;
		p[1][i] = start[i] - 0.002 * dInfo[1][i] ;
	}
	BOOL bFlag[2] = { FALSE, FALSE } ;
	bFlag[0] = IsPtInContour( pLoop, p[0], 0.0002 ) ;
	bFlag[1] = IsPtInContour( pLoop, p[1], 0.0002 ) ;
	if( bFlag[0] && bFlag[1] || bFlag[1] && bFlag[0] )
	{ // 点判断法失误
		if( nFlag == 0 )
		{ // 逆铣反向
			dInfo[1][0] = -dInfo[1][0], dInfo[1][1] = -dInfo[1][1], dInfo[1][2] = -dInfo[1][2] ;
		}
	}
	else
	{ // 如果p[0]在内p[1]在外,反向
		if( bFlag[0] && !bFlag[1] )
		{
			mathRevVec( dInfo[1] ) ;
		}
	}
}
int MathCAM_FindPlungeCurveOnTComb( CPathEntity* TEnt, JDNC_PLUNGE& Plunge, CSmartCurve& Curve )
{
    PNT3D dMin, dMax, dEnd, dPoint ;
    TEnt->GetEndPoint( 0, dPoint ) ;
    for( CPathEntity* pTNext = TEnt ; pTNext ; pTNext = pTNext->next )
    {
        pTNext->GetBoundBox( dMin, dMax ) ;
        if( dMax[2] - dMin[2]> 0.001 ) break ;
        if( pTNext->GetType() == NC_PATH_LINE3D )
        {
            CPathLine3D* pLine3D = (CPathLine3D*)pTNext ;
            Curve.AddSect( new CSmartLine( pLine3D->m_fStart, pLine3D->m_fEnd )) ;
        }
        else if( pTNext->GetType() == NC_PATH_ARC3D )
        {
            CPathArc3D* pArc3D = (CPathArc3D*)pTNext ;
            if( pArc3D->m_bArcPlane != 0 ) break ;
            Curve.AddSect( new CSmartArc( pArc3D->m_fCenter, pArc3D->m_fAngle, pArc3D->m_fRadius)) ;
        }
        else if( pTNext->GetType() == NC_PATH_PLINE3D  )
        {
            CPathPLine3D* pLine3D = (CPathPLine3D*)pTNext ;
            for( int k = 1 ; k <= pLine3D->m_nCount ; k ++ ) 
            {
                Curve.AddSect( new CSmartLine( pLine3D->m_pTAPos[k-1], pLine3D->m_pTAPos[k])) ;
            }
        }
        else
        {
            break ;
        }
        pTNext->GetEndPoint( 1, dEnd ) ;
        if( nc_Distance( dEnd, dPoint, 3 ) < 0.001 )
        {
           break ;
        }
        if( pTNext->next )
        {
            PNT3D dNextEnd ;
            pTNext->next->GetEndPoint( 0, dNextEnd ) ;
            if( nc_Distance( dNextEnd, dEnd , 3 ) > 0.001 ) 
            {
                break ;
            }
        }
    }
    if( Curve.m_pHead == NULL )
    {
        return FALSE ;
    }
    if( Curve.IsClosed() == FALSE )
    {
        Curve.ClearAll() ;
        return FALSE ;
    }
    DOUBLE dBldRad = 0.05 ;
    if( Plunge.m_bPlungeFlag & NCDEF_PLUNGE_DELBLINDAREA )
    {
        dBldRad = max( 0.05, Plunge.m_dBlindRadius ) ;
    }
    Curve.DefineBox( dMin, dMax ) ;
    if( (dMax[0] - dMin[0] ) < dBldRad && 
        (dMax[1] - dMin[1] ) < dBldRad   )
    {
        Curve.ClearAll() ;
        return FALSE ;
    }
    return TRUE ;
}

int CSmartPathGen::InsertPlungePath(CPathCombine&    TComb  , /*路径*/ 
								    CSmartLoop&      Contour, /*边界*/ 
								    JDNC_PLUNGE&     Plunge , /*下刀*/ 
									CSmartLoop*      PartBnd, /*零件边界*/
                                    CSmartLoop*  BlankBnd   , /*毛坯边界*/
									DOUBLE MaxDist[2]       , /*搜索距离[0]， 延伸距离[1]*/
									int	nFlag[2]			) //走刀类型\顺逆铣
{
	// STEP 0 : 偏移轮廓
	if( ! TComb.m_pHead )
	{ // 不必插入下刀路径
		return  0 ;
	}
	CSmartLoop* pChild = NULL ; 
	if( Plunge.m_nPlungeType == NCDEF_PLUNGE_VERT ||
	    Plunge.m_nPlungeType == NCDEF_PLUNGE_HELIX    || 
		Plunge.m_nPlungeType == NCDEF_PLUNGE_RAMP    )
	{
		DOUBLE dOffset = Plunge.m_dSideTol ; 
		if( Plunge.m_nPlungeType == NCDEF_PLUNGE_HELIX ) 
        {
			dOffset += Plunge.m_dRadius ;
        }
		pChild = Contour.OffsetContour( NCDEF_OFFSET_INNER ,
										dOffset , dOffset  ,
										m_cSetupDef.m_cCorDef ) ;
	}

	// STEP 1 : 搜索孤立的点
	CPathEntity* pEnt = TComb.m_pHead, *pPrev = NULL ;
	TPNT3D dTPoint, dAt[2] ;
	PNT3D dPoint, dEnd, dInfo[2] ;
	CSmartCurve tmpCurve ;
	while( pEnt )
	{
		CPathCombine cTPlunge( NC_WPROCESS_PLUNGE ) ;
		pEnt->GetEndPoint( 0, dTPoint ) ;
		GetStartNormalAndTang( pEnt, &Contour, nFlag[1], dInfo ) ;
		if( dTPoint[2] > 0.001 )
		{
			while( pEnt )
			{
				pPrev = pEnt, pEnt = pEnt->next  ;
				if( ! pEnt ) break ;
				pPrev->GetEndPoint( 1 , dAt[0] ) ;
				pEnt->GetEndPoint( 0, dAt[1] ) ;
				if( nc_Distance( dAt[0], dAt[1],2 ) < 0.005 )
				{
					continue ;
				}
				else break ;
			}
			continue ;
		}
		for( int m = 0 ; m < 3 ; m ++) dPoint[m] = dTPoint[m] ;
		if( TComb.FindPlungePoint( dPoint,dEnd, MaxDist[0], pEnt, &Contour  ) )
		{// 尽量在已经切割的位置下刀
			//将该判断条件放入大括号内部，是为了防止当连刀路径与切割路径相交时，在下刀点生成下刀路径。qqs 2014.04.28
			if(!AllPathEntIntLine( pEnt, dEnd, dPoint, m_cMethodDef.m_nMethodType, TRUE ))
			{
				if( nc_Distance( dEnd, dPoint, 3 ) > 2.0e-4 )
				{
					CPathLine3D  *pLine3D = new CPathLine3D(dEnd, dPoint) ;
					pLine3D->m_bFeedType = JDNC_FEEDTYPE_CONNECT ;
					cTPlunge.AddEntity( pLine3D ) ;
				}
				else
				{
					nc_VectorCopy( dEnd, dPoint, 3 ) ;
				}
			}
		}
        else 
        {// 采用参数中的下刀方式
			
            if( CreatePlungeAtBlank( Contour, dPoint, dInfo, MaxDist,PartBnd,BlankBnd, nFlag[0], tmpCurve ) && 
				!AllPathEntIntCurve( pEnt, tmpCurve, m_cMethodDef.m_nMethodType, TRUE )/**/ )
		    {
				CPathCombine cntPath( NC_WPROCESS_CONNECT ) ;
				cntPath.AppendCurve2D( tmpCurve, dPoint[2] ) ;
				cTPlunge.AppendCombine( cntPath ) ;
		    }
            else if( CreateHelixPlunge( Plunge, Contour, pEnt, cTPlunge ) )
            {// 优先生成相切的螺旋下刀路径
            }
		    else if( FindPlungePoint( Plunge, Contour, pChild, dPoint , dEnd ) )
		    {// 重新确定下刀位置
		        AddPlungePathAt( cTPlunge, Plunge, dPoint, dEnd , &Contour) ;
		    }
		    if( cTPlunge.m_pHead == NULL )
		    { /*从路径组上寻找一段封闭的轮廓，插入轮廓下刀*/
                CSmartCurve tmpCv ;
 			    if( MathCAM_FindPlungeCurveOnTComb( pEnt, Plunge, tmpCv ) )
			    {
				    AddPlungePathBy( cTPlunge, Plunge, dPoint, tmpCv, TRUE ) ;
			    }
            }
		    if( cTPlunge.m_pHead == NULL )
		    { /*插入轮廓下刀*/
			    CSmartCurve* pBound = NULL ; 
			    if( FindPlungeCurve( Contour, dPoint , pBound ) && pBound )
			    {
				    AddPlungePathBy( cTPlunge, Plunge, dPoint, *pBound, TRUE ) ;
			    }
			    if( pBound ) delete pBound ; 
		    }
        }
		if( cTPlunge.m_pHead )
		{ 
			if( ! pPrev ) 
			{
				cTPlunge.m_pTail->next = TComb.m_pHead ;
				TComb.m_pHead->prev = cTPlunge.m_pTail ; 
				TComb.m_pHead = cTPlunge.m_pHead ;
			}
			else 
			{
				cTPlunge.m_pTail->next = pPrev->next ;
				pPrev->next->prev = cTPlunge.m_pTail ;
				pPrev->next = cTPlunge.m_pHead ; 
				cTPlunge.m_pHead->prev = pPrev ;
			}
			cTPlunge.m_pHead = cTPlunge.m_pHead = NULL ; 
		}
		while( pEnt )
		{
			pPrev = pEnt, pEnt = pEnt->next  ;
			if( ! pEnt ) break ;
			pPrev->GetEndPoint( 1 , dAt[0] ) ;
		    pEnt->GetEndPoint( 0, dAt[1] ) ;
			if( nc_Distance( dAt[0], dAt[1],2 ) < 0.005 )
			{
				continue ;
			}
			else break ;
		}
	}
	// STEP 3 :  清除临时数据
	Mini_DeleteContours( pChild ) ; 
	return 1 ; 
}
////////////////////////////////////////
// 在固定的位置生成下刀路径, 不考虑干涉
int  CSmartPathGen::AddPlungePathAt( CPathCombine& PComb  ,  /*路径组  */
								   JDNC_PLUNGE& Plunge  ,  /*下刀方式*/
								   PNT3D   ToPoint      ,  /*连接位置*/
								   PNT3D   AtPoint      ,  /*刀具位置*/
								   CSmartLoop* Cont     ,   /*边界轮廓*/
								   int     nHexRef )       /*指定下刀点标记*/

{
	PNT3D dStart, dEnd ; 
	CPathLine3D  *pLine = NULL  ; 
	DOUBLE dInc , dLen  ;
	DOUBLE dTan   = tan( ANGLE_TO_RADIAN( Plunge.m_dAngle ) ) ;
	DOUBLE dDepth = Plunge.m_dIncStep ;
	// STEP 1 : 计算螺旋下刀路径
	if( Plunge.m_nPlungeType == NCDEF_PLUNGE_HELIX )
	{ /*螺旋下刀*/
		DOUBLE dRange[2] , dRadius = Plunge.m_dRadius ;
		PNT3D tmpToPnt ;
		nc_VectorCopy( tmpToPnt, ToPoint, 3 ) ;
		if ( nHexRef == HELIX_REFF_TRUE )
		{
			tmpToPnt[0] = Plunge.m_dPlungePos[0] ;
			tmpToPnt[1] = Plunge.m_dPlungePos[1] ;
			tmpToPnt[2] = 0.0 ;
		}
		if( nc_Distance( tmpToPnt, AtPoint , 2 ) > 1.0e-3 )
			dRange[0] = atan2( tmpToPnt[1] - AtPoint[1], tmpToPnt[0] -AtPoint[0]) ;
		else dRange[0] = 0.0 ; 
	    dRange[1] = dRange[0] + MiniPai2 ;
        DOUBLE dBldRad = 0.05 ;
        if( Plunge.m_bPlungeFlag & NCDEF_PLUNGE_DELBLINDAREA )
        {
            dBldRad = max( 0.05, Plunge.m_dBlindRadius - 0.01 ) ;
        }
 	    if( dRadius < dBldRad ) return FALSE ;
        if( Plunge.m_nErrorTreat == NCDEF_ERRPLUNGE_SUBDIST && Cont)
	    {
	       for( int nTry = 1 ; nTry <= 4 ; nTry ++ ) 
		   {
                if( dRadius < dBldRad ) return FALSE ;
                CSmartArc tmpArc2D( AtPoint, dRange, dRadius-0.001 ) ;
                if( Cont->IntSectContour( &tmpArc2D ) == FALSE ) break ;
                dRadius *= 0.8 ;
		   }
		   if( nTry == 5 )
		   {
				ASSERT( 0 ) ;
				return FALSE ;
		   }
		}
		CSmartArc Arc2D( AtPoint, dRange, dRadius ) ;
		dLen = MiniPai2 * dRadius ;
	    dInc = min( dLen * dTan,Plunge.m_dMaxZInc ) ;
		int nLoop = (int)ceil( dDepth / dInc ) ;
		if( nLoop <= 0 ) nLoop = 1 ; 
		dInc = dDepth / nLoop   ; 
			if( GetMillDir() != 1 ) Arc2D.Reverse() ;
		for(int i = 0 ; i < nLoop ; i ++ ) 
		{
			CPathArc3D* pArc3D = new CPathArc3D() ;
			pArc3D->Create( dDepth- i * dInc, Arc2D ) ;
			pArc3D->m_fDepth = - dInc ;
		    PComb.AddEntity( pArc3D ) ;
		} 
		Arc2D.GetPoint( 1.0, dStart ) ;
		if( nc_Distance( dStart , ToPoint, 2 ) > 2.0e-4 )
		{ /*连接段*/
		    dEnd[0] = ToPoint[0] , dEnd[1] = ToPoint[1]  ;
			dStart[2] = dEnd[2] = 0.0 ;
		    pLine  = new CPathLine3D( dStart, dEnd ) ;
		    PComb.AddEntity( pLine ) ;
		}
		return TRUE ; 
    }
	else if( Plunge.m_nPlungeType == NCDEF_PLUNGE_RAMP  )
	{ // 折线下刀
		/*计算高度*/
		double t = nc_Distance( AtPoint , ToPoint , 2 ) ; 
		if(  t < 0.01 ) return FALSE ;
		double dLineLen = Plunge.m_dRadius  ;
		if( dLineLen > t ) dLineLen  = t ;
		t = dLineLen / t ; 
		if( dLineLen < 1.0e-3 ) return FALSE ;
        if( Plunge.m_bPlungeFlag & NCDEF_PLUNGE_DELBLINDAREA )
        {
            if( dLineLen < Plunge.m_dBlindRadius * 2.0 -0.05 )
            {
                return FALSE ;
            }
        }
	    dLen = 2.0 * dLineLen ;
	    dInc = min( dLen * dTan,Plunge.m_dMaxZInc ) ;
		int nLoop = (int)ceil( dDepth / dInc ) ;
		if( nLoop <= 0 ) nLoop = 1 ; 
		dInc = dDepth / nLoop   ; 
        /*计算坐标*/ 
		for( int k = 0 ; k < 2 ; k ++ ) 
		{
			dStart[k]  = AtPoint[k] + t * ( ToPoint[k] - AtPoint[k] ) ;
			dEnd[k]    = AtPoint[k] ;
		}
		dStart[2] = dDepth  ;
		dEnd[2]   = dStart[2] - dInc/2.0 ;
		/* 生成折线*/
		CPathPLine3D * pLine3D = new CPathPLine3D() ;
		pLine3D->m_pTAPos = new TPNT3D[ nLoop * 2 + 1 ] ;
		pLine3D->m_nCount = nLoop * 2  ;
		for( int i = 0 ; i < nLoop ; i ++ ) 
		{
			for( int nDim = 0 ; nDim < 3 ; nDim ++ ) 
			{
				pLine3D->m_pTAPos[i*2][nDim] =  (TFLOAT)dStart[nDim] ;
		        pLine3D->m_pTAPos[i*2+1][nDim] = (TFLOAT)dEnd[nDim];
			}
		   dStart[2] -= dInc , dEnd[2] -= dInc ;
		}  
		pLine3D->m_pTAPos[nLoop*2][0] = TFLOAT(ToPoint[0]) ;
		pLine3D->m_pTAPos[nLoop*2][1] = TFLOAT(ToPoint[1]) ;
		pLine3D->m_pTAPos[nLoop*2][2] = 0.0f ;
		PComb.AddEntity( pLine3D ) ;
		return TRUE ; 
	} 
	else if( Plunge.m_nPlungeType == NCDEF_PLUNGE_VERT  )
	{ // 垂直下刀
		dStart[0] = AtPoint[0]  , dStart[1] = AtPoint[1]  ;
		dEnd[0]   = AtPoint[0] , dEnd[1]    = AtPoint[1]  ;
		dStart[2] = dDepth , dEnd[2] = 0.0  ; 
		pLine  = new CPathLine3D( dStart, dEnd ) ;
		PComb.AddEntity( pLine ) ;
		if( nc_Distance( AtPoint , ToPoint, 2 ) > 2.0e-4 )
		{ /*连接段*/
			dStart[0] = AtPoint[0]  , dStart[1] = AtPoint[1] ;
		    dEnd[0] = ToPoint[0]  , dEnd[1] = ToPoint[1] ; 
			dStart[2] = dEnd[2] = 0.0 ;
		    pLine  = new CPathLine3D( dStart, dEnd ) ;
		    PComb.AddEntity( pLine ) ;
		}
		return TRUE ; 
	}
	return FALSE ;
}

CSmartSect*  CSmartPathGen::CreateLeadEnt(	JDNC_LEAD&   LeadDef ,
											PNT2D  Point ,
											VEC2D  Normal,
											int    IsOut ,
											CSmartLoop*   Contour ) 

{
	int    nType    = LeadDef.m_nLeadType  ;
	DOUBLE dLength  = LeadDef.m_dLength ;
	DOUBLE dRadius  = LeadDef.m_dRadius ; 
	DOUBLE dAngle   = ANGLE_TO_RADIAN( LeadDef.m_dAngle )  ;
	CSmartSect* pSect = NULL ;
	PNT2D dSeedPnt[2] ;
	if( nType == NCDEF_LEAD_SLINE || 
		nType == NCDEF_LEAD_LINE     )
	{ // 直线连接 
		PNT2D dEnd ;
		dEnd[0] = Point[0] + Normal[0] * dLength ;
		dEnd[1] = Point[1] + Normal[1] * dLength ;
		if( IsOut )
		{
			pSect = new CSmartLine( Point , dEnd ) ;
		}
		else
		{
			pSect = new CSmartLine( dEnd, Point ) ;
		}
		pSect->m_bFeedType = JDNC_FEEDTYPE_LEAD ; 
		if( Contour )
		{
			pSect->GetPoint( 0.5, dSeedPnt[0] ) ;
			//pSect->GetPoint( 1.0, dSeedPnt[1] ) ;
			if( IsOut ) 	
			{
				pSect->GetPoint( 1.0, dSeedPnt[1] ) ;
			}
			else 
			{
				pSect->GetPoint( 0.0, dSeedPnt[1] ) ;

			}/**/
			if( ! Contour->IsPtOnContour( dSeedPnt[0] ) || 
				! Contour->IsPtOnContour( dSeedPnt[1] ) || 
				Contour->IntSectContour( pSect )  ) 
			{ 
				delete pSect ;
				return NULL  ;
			} 
		}
		return pSect ;
	}
	else if( nType == NCDEF_LEAD_ARC || nType == NCDEF_LEAD_INARC   )
	{
		PNT2D dCent ;
		DOUBLE dOA, dAng[2], dNor[2], dCos, dSin, dRotAng = 0.0 ; 
		DOUBLE dTmpNor[2] ;
		for( int nSide = 0 ; nSide <= 1 ; nSide ++ ) 
		{
			if( nType == NCDEF_LEAD_ARC && nSide == 0 )
			{
				continue ;
			}
			nc_VectorCopy( dTmpNor, Normal, 2 ) ;
			double dRotInc = ANGLE_TO_RADIAN( -1.0 ) ; 
			if( nSide == 0 ) 
			{
				dRotInc = - dRotInc ;
				nc_VectorReverse( dTmpNor, 2 ) ;
			}
			dRotAng = dOA = 0.0 ;
			for( int nTry = 1 ; nTry <= 15 ; nTry ++ ) 
			{
				dCos = cos( dRotAng ) , dSin = sin( dRotAng ) ;
				if( IsOut ) dSin = - dSin ;
				dNor[0] = dTmpNor[0] * dCos - dTmpNor[1] * dSin ;
				dNor[1] = dTmpNor[0] * dSin + dTmpNor[1] * dCos ;
				dRotAng += dRotInc ;
				dCent[0] = Point[0] + dNor[0] * dRadius ;
				dCent[1] = Point[1] + dNor[1] * dRadius ;
				dOA = atan2( -dNor[1], -dNor[0] ) ;
				if( IsOut )
				{
					dAng[0] = dOA  ;
					if( nSide == 1 ) dAng[1] = dOA + dAngle ;
					else dAng[1] = dOA - dAngle ;
				}
				else
				{
					dAng[1] = dOA ;
					if( nSide == 1 ) dAng[0] = dOA - dAngle ;
					else dAng[0] = dOA + dAngle ;
				}
				pSect = new CSmartArc( dCent,dAng,dRadius) ;
				pSect->m_bFeedType = JDNC_FEEDTYPE_LEAD ; 
				if( Contour )
				{
					pSect->GetPoint( 0.5, dSeedPnt[0] ) ;
					if( IsOut )  pSect->GetPoint( 1.0, dSeedPnt[1] ) ;
					else pSect->GetPoint( 0.0, dSeedPnt[1] ) ;
					if( !Contour->IsPtOnContour( dSeedPnt[0] ) ||
						!Contour->IsPtOnContour( dSeedPnt[1] ) ||
						Contour->IntSectContour( pSect )  ) 
					{ 
						delete pSect ;
						pSect = NULL ;
					} 
				}
				if( pSect )   return pSect ;
			}
		}
	}
	return NULL ; 
}

// 计算退刀路径
CSmartSect* CSmartPathGen::CreateLeadOutEnt(	JDNC_LEAD& LeadDef, /*结构定义*/
												PNT2D   Point ,     /*端点*/
												VEC2D   Tangent,    /*切失*/
												VEC2D   Normal,     /*法失*/
												BOOL IsOut , 
												CSmartLoop* AllCont)/*干涉检查*/

{
	int    nType    = LeadDef.m_nLeadType ;
	DOUBLE dLength  = LeadDef.m_dLength   ;
	DOUBLE dRadius  = LeadDef.m_dRadius   ; 
	DOUBLE dAngle   = ANGLE_TO_RADIAN( LeadDef.m_dAngle )  ;
	if( m_cSetupDef.m_cOrderDef.m_nMillDir == 0 )
	{
		dAngle = -dAngle  ;
	}
	CSmartSect* pSect = NULL ;
	PNT2D dEnd, dSeedPnt[2] ;
	CSmartLoop* pContour ;
	if( nType == NCDEF_LEAD_SLINE  )
	{// 直线连接 
		for( int nTry = 1 ; nTry <= 10 ; nTry ++ )
		{
			if( dLength < 0.02 ) return NULL ; 
			if( nTry != 1 ) dLength *= 0.8 ;
			dEnd[0] = Point[0] + Normal[0] * dLength ;
			dEnd[1] = Point[1] + Normal[1] * dLength ;
			CSmartLine  tmpLine( Point, dEnd ) ;
			if( AllCont )
			{
				tmpLine.GetPoint( 0.5, dSeedPnt[0] ) ;
				tmpLine.GetPoint( 1.0, dSeedPnt[1] ) ;
				for( pContour = AllCont ; pContour ; pContour = pContour->next )
				{
					if( pContour->IsPtOnContour( dSeedPnt[0] ) &&
						pContour->IsPtOnContour( dSeedPnt[1] ) &&
						pContour->IsPtOnContour( dEnd    )  )
					{
						break ;
					}
				}
				if( pContour == NULL )  continue ;
				for( pContour = AllCont ; pContour ; pContour = pContour->next )
				{
					if( pContour->IntSectContour( &tmpLine ) ) 
					{
						break ;
					}
				}
				if( pContour != NULL ) continue ;
			}
			pSect = tmpLine.CopyMyself() ;
			break ;
		}
	}
	else if( nType == NCDEF_LEAD_LINE )
	{// 直线延长
		DOUBLE dRot = 0.0,dCosA, dSinA, dAngInc = ANGLE_TO_RADIAN( 1.0);
		for( int nTry = 1 ; nTry <= 178 ; nTry ++ )
		{
			dCosA = cos( dRot ) * dLength ;
			dSinA = sin( dRot ) * dLength ;
			dEnd[0] = Point[0] + Tangent[0] * dCosA + Normal[0]* dSinA ;
			dEnd[1] = Point[1] + Tangent[1] * dCosA + Normal[1]* dSinA ;
			dRot += dAngInc ;
			CSmartLine  tmpLine( Point, dEnd ) ;
			if( AllCont )
			{
				tmpLine.GetPoint( 0.5, dSeedPnt[0] ) ;
				tmpLine.GetPoint( 1.0, dSeedPnt[1] ) ;
				for( pContour = AllCont ; pContour ; pContour = pContour->next )
				{
					if( pContour->IsPtOnContour( dSeedPnt[0] ) &&
						pContour->IsPtOnContour( dSeedPnt[1] ) &&
						pContour->IsPtOnContour( dEnd    )  )
					{
						break ;
					}
				}
				if( pContour == NULL )  continue ;
				for( pContour = AllCont ; pContour ; pContour = pContour->next )
				{
					if( pContour->IntSectContour( &tmpLine ) ) 
					{
						break ;
					}
				}
				if( pContour != NULL ) continue ;
			}
			pSect = tmpLine.CopyMyself() ;
			break ;
		}
	}
	else if( nType == NCDEF_LEAD_ARC || nType == NCDEF_LEAD_INARC)
	{
		PNT2D dCent, dTmpNor ;
		DOUBLE dRotAng=0.0, dCosA, dSinA, dOA = 0.0, dAng[2];
		for( int nSide = 0 ; nSide <= 1 ; nSide ++ ) 
		{
			if( nType == NCDEF_LEAD_ARC && nSide == 0 ) 
			{
				continue ;
			}
			double dRotInc = ANGLE_TO_RADIAN( 1.0 ) ;
			nc_VectorCopy( dTmpNor, Normal, 2 ) ;
			if( nSide == 0 )  
			{
				dRotInc = - dRotInc ;
				nc_VectorReverse( dTmpNor, 2 ) ;
			}
			dRotAng=0.0 , dOA = 0.0;
			for( int nTry = 1 ; nTry <= 15 ; nTry ++ ) 
			{
				dCosA = cos( dRotAng ) * dRadius ;
				dSinA = sin( dRotAng ) * dRadius ;
				dCent[0] = Point[0] - Tangent[0] * dSinA + dTmpNor[0]* dCosA ;
				dCent[1] = Point[1] - Tangent[1] * dSinA + dTmpNor[1]* dCosA ;
				dRotAng += dRotInc ;
				dOA = atan2( Point[1] - dCent[1], Point[0] - dCent[0] ) ;
				if( IsOut && nSide == 1 || !IsOut && nSide == 0 )
				{
					dAng[0] = dOA , dAng[1] = dOA + dAngle ;
				}
				else
				{
					dAng[0] = dOA , dAng[1] = dOA - dAngle;
				}
				CSmartArc  tmpArc( dCent,dAng,dRadius) ;
				if( AllCont )
				{
					tmpArc.GetPoint( 0.5, dSeedPnt[0]) ;
					tmpArc.GetPoint( 1.0, dSeedPnt[1] ) ;
					for( pContour = AllCont ; pContour ; pContour = pContour->next )
					{
						if( pContour->IsPtOnContour( dSeedPnt[0] ) &&
							pContour->IsPtOnContour( dSeedPnt[1] )  )
						{
							break ;
						}
					}
					if( pContour == NULL )  continue ;
					for( pContour = AllCont ; pContour ; pContour = pContour->next )
					{
						if( pContour->IntSectContour( &tmpArc ) ) 
						{
							break ;
						}
					}
					if( pContour != NULL ) continue ;
				}
				pSect = tmpArc.CopyMyself() ;
				break ;
			}
			if( pSect ) break ;
		}
	}
	if( pSect )
	{
		pSect->m_bFeedType = JDNC_FEEDTYPE_LEAD ; 
		if( IsOut == FALSE ) pSect->Reverse() ;
		return pSect ;
	}
	return NULL ; 
}

BOOL	CSmartPathGen::CreateLeadEntNew(	JDNC_LEAD&   LeadDef ,
										PNT2D  Point ,
										VEC2D  Normal,
										int    IsOut ,
										CSmartLoop* Contour,
										CSmartCurve& leadCv)
{
	int    nType    = LeadDef.m_nLeadType  ;
	DOUBLE dLength  = LeadDef.m_dLength ;
	DOUBLE dRadius  = LeadDef.m_dRadius ; 
	DOUBLE dAngle   = ANGLE_TO_RADIAN( LeadDef.m_dAngle )  ;
	CSmartSect* pSect = NULL ;
	PNT2D dSeedPnt[2] ;
	if( nType == NCDEF_LEAD_SLINE || 
		nType == NCDEF_LEAD_LINE     )
	{ // 直线连接 
		PNT2D dEnd ;
		dEnd[0] = Point[0] + Normal[0] * dLength ;
		dEnd[1] = Point[1] + Normal[1] * dLength ;
		if( IsOut )
		{
			pSect = new CSmartLine( Point , dEnd ) ;
		}
		else
		{
			pSect = new CSmartLine( dEnd, Point ) ;
		}

		if( Contour )
		{
			pSect->GetPoint( 0.5, dSeedPnt[0] ) ;

			if( IsOut ) 	
			{
				pSect->GetPoint( 1.0, dSeedPnt[1] ) ;
			}
			else 
			{
				pSect->GetPoint( 0.0, dSeedPnt[1] ) ;

			}
			if( ! Contour->IsPtOnContour( dSeedPnt[0] ) || 
				! Contour->IsPtOnContour( dSeedPnt[1] ) || 
				Contour->IntSectContour( pSect )  ) 
			{ 
				delete pSect ;
				return FALSE  ;
			} 
		}
		leadCv.AddSect(pSect);
	}
	else if( nType == NCDEF_LEAD_ARC || nType == NCDEF_LEAD_INARC   )
	{
		PNT2D dCent ;
		DOUBLE dOA, dAng[2], dNor[2], dCos, dSin, dRotAng = 0.0 ; 
		DOUBLE dTmpNor[2] ;
		for( int nSide = 0 ; nSide <= 1 ; nSide ++ ) 
		{
			if( nType == NCDEF_LEAD_ARC && nSide == 0 )
			{
				continue ;
			}
			nc_VectorCopy( dTmpNor, Normal, 2 ) ;
			double dRotInc = ANGLE_TO_RADIAN( -1.0 ) ; 
			if( nSide == 0 ) 
			{
				dRotInc = - dRotInc ;
				nc_VectorReverse( dTmpNor, 2 ) ;
			}
			dRotAng = dOA = 0.0 ;
			int nTry = 1 ;
			for( nTry = 1 ; nTry <= 15 ; nTry ++ ) 
			{
				dCos = cos( dRotAng ) , dSin = sin( dRotAng ) ;
				if( IsOut ) dSin = - dSin ;
				dNor[0] = dTmpNor[0] * dCos - dTmpNor[1] * dSin ;
				dNor[1] = dTmpNor[0] * dSin + dTmpNor[1] * dCos ;
				dRotAng += dRotInc ;
				dCent[0] = Point[0] + dNor[0] * dRadius ;
				dCent[1] = Point[1] + dNor[1] * dRadius ;
				dOA = atan2( -dNor[1], -dNor[0] ) ;
				if( IsOut )
				{
					dAng[0] = dOA  ;
					if( nSide == 1 ) dAng[1] = dOA + dAngle ;
					else dAng[1] = dOA - dAngle ;

				}
				else
				{
					dAng[1] = dOA ;
					if( nSide == 1 ) dAng[0] = dOA - dAngle ;
					else dAng[0] = dOA + dAngle ;
				}
				pSect = new CSmartArc( dCent,dAng,dRadius) ;
				pSect->m_bFeedType = JDNC_FEEDTYPE_LEAD ; 
				if( Contour )
				{		
					pSect->GetPoint( 0.5, dSeedPnt[0] ) ;
					if( IsOut )  pSect->GetPoint( 1.0, dSeedPnt[1] ) ;
					else pSect->GetPoint( 0.0, dSeedPnt[1] ) ;
					if( !Contour->IsPtOnContour( dSeedPnt[0] ) ||
						!Contour->IsPtOnContour( dSeedPnt[1] ) ||
						Contour->IntSectContour( pSect )  ) 
					{ 
						delete pSect ;
						pSect = NULL ;
					} 
				}
				if( pSect != NULL)  break;
			}
			if (pSect == NULL)
			{
				continue;
			}
			leadCv.AddSect(pSect);
			if( nType == NCDEF_LEAD_INARC && nSide == 0 && 
				NeedOutArcAtArcIn(LeadDef, IsOut)) 
			{// 内切进刀检查是否需要外切引入
				double dOutRad, dOutAng, dOutStart[2], dOutNormal[2];
				if (IsOut)
				{
					pSect->GetEnd(dOutStart);
					pSect->GetNormal(1.0, dOutNormal);
				}
				else
				{
					pSect->GetStart(dOutStart);
					pSect->GetNormal(0.0, dOutNormal);
				}			

				GetParamOfOutArcAtArcIn(IsOut, dOutRad, dOutAng);

				dRotInc = ANGLE_TO_RADIAN( -1.0 ) ; 			
				dRotAng=0.0 , dOA = 0.0;
				pSect = NULL;
				for( nTry = 1 ; nTry <= 15 ; nTry ++ ) 
				{
					dCos = cos( dRotAng ) , dSin = sin( dRotAng ) ;
					if( IsOut ) dSin = - dSin ;
					dNor[0] = dOutNormal[0] * dCos - dOutNormal[1] * dSin ;
					dNor[1] = dOutNormal[0] * dSin + dOutNormal[1] * dCos ;
					dRotAng += dRotInc ;
					dCent[0] = dOutStart[0] + dNor[0] * dOutRad ;
					dCent[1] = dOutStart[1] + dNor[1] * dOutRad ;
					dOA = atan2( -dNor[1], -dNor[0] ) ;
					if( IsOut )
					{
						dAng[0] = dOA  ;
						dAng[1] = dOA + dOutAng ;
					}
					else
					{
						dAng[1] = dOA ;
						dAng[0] = dOA - dOutAng ;
					}					
					CSmartArc  tmpArc( dCent,dAng,dOutRad) ;
					if( Contour )
					{		
						tmpArc.GetPoint( 0.5, dSeedPnt[0] ) ;
						if( IsOut )  tmpArc.GetPoint( 1.0, dSeedPnt[1] ) ;
						else tmpArc.GetPoint( 0.0, dSeedPnt[1] ) ;
						if( !Contour->IsPtOnContour( dSeedPnt[0] ) ||
							!Contour->IsPtOnContour( dSeedPnt[1] ) ||
							Contour->IntSectContour( &tmpArc )  ) 
						{ 
							continue;
						} 
					}
					pSect = tmpArc.CopyMyself() ;
					break ;
				}
				if (pSect != NULL)
				{
					if (IsOut)
					{
						leadCv.AddSect(pSect);
					}
					else
					{
						leadCv.InsertAfter(pSect, NULL);
					}
				}
			}
			else if( nType == NCDEF_LEAD_ARC && nSide == 1 && 
					NeedLineAtArcOut(LeadDef, IsOut)) 
			{ // 外切直线引入
				PNT2D start, end, tangent ;
				double dLine = 0. ;
				if( IsOut ) 
				{
					pSect->GetEnd( start ) ;
					pSect->GetTangent ( 1., tangent ) ;
					dLine = m_cProcessDef.m_dLeadOutArcLandRad ;
					end[0] = start[0] + tangent[0] * dLine ;
					end[1] = start[1] + tangent[1] * dLine ;
				}
				else
				{
					pSect->GetStart ( start ) ;
					pSect->GetTangent ( 0., tangent ) ;
					dLine = m_cProcessDef.m_dLeadInArcLandRad ;
					end[0] = start[0] - tangent[0] * dLine ;
					end[1] = start[1] - tangent[1] * dLine ;
				}
				CSmartLine *pSect = new CSmartLine( start, end ) ;

				if( Contour )
				{
					pSect->GetPoint ( 0.5, dSeedPnt[0] ) ;
					if( IsOut ) pSect->GetPoint( 1, dSeedPnt[1] ) ;

					if( !Contour->IsPtOnContour( dSeedPnt[0] ) ||
						!Contour->IsPtOnContour( dSeedPnt[1] ) ||
						 Contour->IntSectContour( pSect )  ) 
					{ 
						delete pSect ;
						pSect = NULL ;
					}
				}
				if ( pSect )
				{
					if (IsOut)
					{
						leadCv.AddSect(pSect);
					}
					else
					{
						pSect->Reverse() ;
						leadCv.InsertAfter(pSect, NULL);
					}
				}
			}
			break ;
		}
	}
	if (leadCv.m_pHead != NULL)
	{
		leadCv.SetFeedType(JDNC_FEEDTYPE_LEAD);
		return TRUE;
	}
	return FALSE ; 
}
// 计算退刀路径
BOOL CSmartPathGen::CreateLeadOutEntNew(JDNC_LEAD& LeadDef, /*结构定义*/
										PNT2D   Point ,     /*端点*/
										VEC2D   Tangent,    /*切失*/
										VEC2D   Normal,     /*法失*/
										BOOL IsOut ,		/*切出*/
										CSmartLoop* AllCont,/*干涉检查*/
										CSmartCurve& leadCv) /*退刀路径*/
{
	int    nType    = LeadDef.m_nLeadType ;
	DOUBLE dLength  = LeadDef.m_dLength   ;
	DOUBLE dRadius  = LeadDef.m_dRadius   ; 
	DOUBLE dAngle   = ANGLE_TO_RADIAN( LeadDef.m_dAngle )  ;
	if( m_cSetupDef.m_cOrderDef.m_nMillDir == 0 )
	{
		dAngle = -dAngle  ;
	}
	CSmartSect* pSect = NULL ;
	PNT2D dEnd, dMidPnt ;
	CSmartLoop* pContour ;
	if( nType == NCDEF_LEAD_SLINE  )
	{// 直线连接 
		for( int nTry = 1 ; nTry <= 5 ; nTry ++ )
		{
			if( dLength < 0.02 ) return NULL ; 
			dEnd[0] = Point[0] + Normal[0] * dLength ;
			dEnd[1] = Point[1] + Normal[1] * dLength ;
			CSmartLine  tmpLine( Point, dEnd ) ;
			if( AllCont )
			{
				tmpLine.GetPoint( 0.5, dMidPnt ) ;
				for( pContour = AllCont ; pContour ; pContour = pContour->next )
				{
					if( pContour->IsPtOnContour( dMidPnt ) &&
						pContour->IsPtOnContour( dEnd    )  )
					{
						break ;
					}
				}
				if( pContour == NULL )  continue ;
				for( pContour = AllCont ; pContour ; pContour = pContour->next )
				{
					if( pContour->IntSectContour( &tmpLine ) ) 
					{
						break ;
					}
				}
				if( pContour != NULL ) continue ;
			}
			pSect = tmpLine.CopyMyself() ;
			break ;
		}
		if( pSect != NULL)
		{
			if( IsOut == FALSE ) pSect->Reverse() ;
			leadCv.AddSect(pSect);			
		}
	}
	else if( nType == NCDEF_LEAD_LINE )
	{// 直线延长
		DOUBLE dRot = 0.0,dCosA, dSinA, dAngInc = ANGLE_TO_RADIAN( 1.0);
		for( int nTry = 1 ; nTry <= 50 ; nTry ++ )
		{
			dCosA = cos( dRot ) * dLength ;
			dSinA = sin( dRot ) * dLength ;
			dEnd[0] = Point[0] + Tangent[0] * dCosA + Normal[0]* dSinA ;
			dEnd[1] = Point[1] + Tangent[1] * dCosA + Normal[1]* dSinA ;
			dRot += dAngInc ;
			CSmartLine  tmpLine( Point, dEnd ) ;
			if( AllCont )
			{
				tmpLine.GetPoint( 0.5, dMidPnt ) ;
				for( pContour = AllCont ; pContour ; pContour = pContour->next )
				{
					if( pContour->IsPtOnContour( dMidPnt ) &&
						pContour->IsPtOnContour( dEnd    )  )
					{
						break ;
					}
				}
				if( pContour == NULL )  continue ;
				for( pContour = AllCont ; pContour ; pContour = pContour->next )
				{
					if( pContour->IntSectContour( &tmpLine ) ) 
					{
						break ;
					}
				}
				if( pContour != NULL ) continue ;
			}
			pSect = tmpLine.CopyMyself() ;
			break ;
		}
		if( pSect != NULL)
		{
			if( IsOut == FALSE ) pSect->Reverse() ;
			leadCv.AddSect(pSect);
		}
	}
	else if( nType == NCDEF_LEAD_ARC || nType == NCDEF_LEAD_INARC)
	{
		PNT2D dCent, dTmpNor ;
		DOUBLE dRotAng=0.0, dCosA, dSinA, dOA = 0.0, dAng[2];
		for( int nSide = 0 ; nSide <= 1 ; nSide ++ ) 
		{
			if( nType == NCDEF_LEAD_ARC && nSide == 0 ) 
			{
				continue ;
			}
			double dRotInc = ANGLE_TO_RADIAN( 1.0 ) ;
			nc_VectorCopy( dTmpNor, Normal, 2 ) ;
			if( nSide == 0 )  
			{
				dRotInc = - dRotInc ;
				nc_VectorReverse( dTmpNor, 2 ) ;
			}
			dRotAng=0.0 , dOA = 0.0;
			for( int nTry = 1 ; nTry <= 15 ; nTry ++ ) 
			{
				dCosA = cos( dRotAng ) * dRadius ;
				dSinA = sin( dRotAng ) * dRadius ;
				dCent[0] = Point[0] - Tangent[0] * dSinA + dTmpNor[0]* dCosA ;
				dCent[1] = Point[1] - Tangent[1] * dSinA + dTmpNor[1]* dCosA ;
				dRotAng += dRotInc ;
				dOA = atan2( Point[1] - dCent[1], Point[0] - dCent[0] ) ;
				if( IsOut && nSide == 1 || !IsOut && nSide == 0 )
				{
					dAng[0] = dOA , dAng[1] = dOA + dAngle ;
				}
				else
				{
					dAng[0] = dOA , dAng[1] = dOA - dAngle;
				}
				CSmartArc  tmpArc( dCent,dAng,dRadius) ;
				if( AllCont )
				{
					tmpArc.GetPoint( 0.5, dMidPnt ) ;
					tmpArc.GetEnd( dEnd ) ;
					for( pContour = AllCont ; pContour ; pContour = pContour->next )
					{
						if( pContour->IsPtOnContour( dMidPnt ) &&
							pContour->IsPtOnContour( dEnd    )  )
						{
							break ;
						}
					}
					if( pContour == NULL )  continue ;
					for( pContour = AllCont ; pContour ; pContour = pContour->next )
					{
						if( pContour->IntSectContour( &tmpArc ) ) 
						{
							break ;
						}
					}
					if( pContour != NULL ) continue ;
				}
				pSect = tmpArc.CopyMyself() ;
				break ;
			}
			if( pSect == NULL)
			{
				continue;
			}
			if( IsOut == FALSE ) pSect->Reverse() ;
			leadCv.AddSect(pSect);
			if( nType == NCDEF_LEAD_INARC && nSide == 0 && 
				NeedOutArcAtArcIn(LeadDef, IsOut)) 
			{// 内切进刀检查是否需要外切引入
				double dOutRad, dOutAng, dOutStart[2], dOutTangent[2], dOutNormal[2];
				if( IsOut ) 
				{
					pSect->GetEnd(dOutStart);
					pSect->GetNormal(1.0, dOutNormal);
					pSect->GetTangent(1.0, dOutTangent);
				}
				else
				{
					pSect->GetStart(dOutStart);
					pSect->GetNormal(0.0, dOutNormal);
					pSect->GetTangent(0.0, dOutTangent);
				}				
				GetParamOfOutArcAtArcIn(IsOut, dOutRad, dOutAng);

				dRotInc = ANGLE_TO_RADIAN( 1.0 ) ;			
				dRotAng=0.0 , dOA = 0.0;
				pSect = NULL;
				for( int nTry = 1 ; nTry <= 15 ; nTry ++ ) 
				{
					dCosA = cos( dRotAng ) * dOutRad ;
					dSinA = sin( dRotAng ) * dOutRad ;
					dCent[0] = dOutStart[0] - dOutTangent[0] * dSinA + dOutNormal[0]* dCosA ;
					dCent[1] = dOutStart[1] - dOutTangent[1] * dSinA + dOutNormal[1]* dCosA ;
					dRotAng += dRotInc ;
					dOA = atan2( dOutStart[1] - dCent[1], dOutStart[0] - dCent[0] ) ;
					if( IsOut )
					{
						dAng[0] = dOA , dAng[1] = dOA + dOutAng ;
					}
					else
					{
						dAng[0] = dOA , dAng[1] = dOA - dOutAng;
					}
					CSmartArc  tmpArc( dCent,dAng,dOutRad) ;
					if( AllCont )
					{
						tmpArc.GetPoint( 0.5, dMidPnt ) ;
						tmpArc.GetEnd( dEnd ) ;
						for( pContour = AllCont ; pContour ; pContour = pContour->next )
						{
							if( pContour->IsPtOnContour( dMidPnt ) &&
								pContour->IsPtOnContour( dEnd    )  )
							{
								break ;
							}
						}
						if( pContour == NULL )  continue ;
						for( pContour = AllCont ; pContour ; pContour = pContour->next )
						{
							if( pContour->IntSectContour( &tmpArc ) ) 
							{
								break ;
							}
						}
						if( pContour != NULL ) continue ;
					}
					pSect = tmpArc.CopyMyself() ;
					break ;
				}
				if (pSect != NULL)
				{
					if (IsOut)
					{
						leadCv.AddSect(pSect);
					}
					else
					{
						pSect->Reverse() ;
						leadCv.InsertAfter(pSect, NULL);
					}
				}
			}
			else if( nType == NCDEF_LEAD_ARC && nSide == 1 && 
						NeedLineAtArcOut(LeadDef, IsOut)) 
			{ // 外切时候直线引入
				PNT2D start, end, tangent ;
				double dLine = 0. ;
				if( IsOut ) 
				{
					pSect->GetEnd( start ) ;
					pSect->GetTangent ( 1., tangent ) ;
					dLine = m_cProcessDef.m_dLeadOutArcLandRad ;
					end[0] = start[0] + tangent[0] * dLine ;
					end[1] = start[1] + tangent[1] * dLine ;
				}
				else
				{
					pSect->GetStart ( start ) ;
					pSect->GetTangent ( 0., tangent ) ;
					dLine = m_cProcessDef.m_dLeadInArcLandRad ;
					end[0] = start[0] - tangent[0] * dLine ;
					end[1] = start[1] - tangent[1] * dLine ;
				}
				CSmartLine *pSect = new CSmartLine( start, end ) ;

				if( AllCont )
				{
					pSect->GetPoint ( 0.5, dMidPnt ) ;
					pSect->GetEnd( dEnd ) ;
					for( pContour = AllCont ; pContour ; pContour = pContour->next )
					{
						if( pContour->IsPtOnContour( dMidPnt ) &&
							pContour->IsPtOnContour( dEnd    )  )
						{
							break ;
						}
					}
					if( pContour == NULL )  
					{
						delete pSect ;
						pSect = NULL ;
						goto EndLine ;
					}
					for( pContour = AllCont ; pContour ; pContour = pContour->next )
					{
						if( pContour->IntSectContour( pSect ) ) 
						{
							break ;
						}
					}
					if( pContour != NULL ) 
					{
						delete pSect ;
						pSect = NULL ;
						goto EndLine ;
					}
				}
EndLine:
				if ( pSect )
				{
					if (IsOut)
					{
						leadCv.AddSect(pSect);
					}
					else
					{
						pSect->Reverse() ;
						leadCv.InsertAfter(pSect, NULL);
					}
				}
			}
			break ;
		}
	}
	if( leadCv.m_pHead != NULL )
	{
		leadCv.SetFeedType( JDNC_FEEDTYPE_LEAD ) ;        
		return  TRUE;

	}
	return FALSE ; 
}
int CSmartPathGen::AddLeadPathAtEnd( JDNC_LEAD& LeadDef, 
                                     CPathCombine& TComb, 
                                     CSmartLoop&  Contour ) 
{
    PNT3D dPoint[2], dTan[2], dNor[2], dMidNor;
    TComb.m_pHead->GetEndPoint( 0, dPoint[0] ) ;
    TComb.m_pHead->GetEndTangent( 0, dTan[0] ) ;
    nc_VectorReverse( dTan[0], 2 ) ;
    TComb.m_pHead->GetEndNormal( 0, dNor[0] ) ;
    TComb.m_pTail->GetEndPoint( 1, dPoint[1] ) ;
    TComb.m_pTail->GetEndTangent( 1, dTan[1] ) ;
    TComb.m_pTail->GetEndNormal( 1, dNor[1] ) ;
    nc_Normalize( dTan[0], 2 ) ;
    nc_Normalize( dTan[1], 2 ) ;
    if( GetMillDir() == 0 ) 
    {
        nc_VectorReverse( dNor[0], 2 ) ;
        nc_VectorReverse( dNor[1], 2 ) ;
    }
    BOOL bIsClosed = FALSE ;
    if( nc_Distance( dPoint[0], dPoint[1], 2 ) < 0.01 )
    {
        dMidNor[0] = dNor[0][0] + dNor[1][0] ;
        dMidNor[1] = dNor[0][1] + dNor[1][1] ;
        nc_Normalize( dMidNor, 2 ) ;
        bIsClosed = TRUE ;
    }
    CSmartLoop * tmpCont = Contour.CopyContour() ;
    CSmartSect* pSect = CreateLeadOutEnt( LeadDef, dPoint[0], dTan[0], dNor[0], FALSE, tmpCont ) ;
    if( pSect == NULL && bIsClosed ) 
    {
        JDNC_LEAD tmpLead = LeadDef ;
        tmpLead.m_nLeadType =  NCDEF_LEAD_SLINE ;
        pSect = CreateLeadOutEnt( tmpLead, dPoint[0], dTan[0], dMidNor, FALSE, tmpCont ) ;
    }
    CPathEntity* pLead = GetLeadEnt( pSect, dPoint[0][2] ) ;
    if( pLead )
    {
		pLead->m_bFeedType = pSect->m_bFeedType ;
        TComb.InsertAfter( pLead, NULL ) ;
	}
    if( pSect)    delete pSect ;
    pSect = CreateLeadOutEnt( LeadDef, dPoint[1], dTan[1], dNor[1], TRUE, tmpCont ) ;
    if( pSect == NULL && bIsClosed ) 
    {
        JDNC_LEAD tmpLead = LeadDef ;
        tmpLead.m_nLeadType =  NCDEF_LEAD_SLINE ;
        pSect = CreateLeadOutEnt( tmpLead, dPoint[1], dTan[1],dMidNor, TRUE, tmpCont ) ;
    }
    pLead = GetLeadEnt( pSect, dPoint[1][2] ) ;
    if( pLead )
	{
		pLead->m_bFeedType = pSect->m_bFeedType ;
        TComb.AddEntity( pLead) ;
	}
    if( pSect ) delete pSect ;
    Mini_DeleteContours( tmpCont ) ;
    return 1 ;
}
int CSmartPathGen::AddLeadSectAtEnd( JDNC_LEAD& LeadDef, 
                                     CSmartCurve& Curve , 
                                     CSmartLoop*  AllCont ) 
{
    PNT3D dPoint[2], dTan[2], dNor[2], dMidNor;
    if( ! Curve.m_pHead ) return 0 ;
    Curve.m_pHead->GetStart(dPoint[0] ) ;
    Curve.m_pHead->GetTangent( 0.0, dTan[0] ) ;
    Curve.m_pHead->GetNormal( 0.0, dNor[0] ) ;
    nc_VectorReverse( dTan[0], 2 ) ;
    Curve.m_pTail->GetEnd( dPoint[1] ) ;
    Curve.m_pTail->GetTangent( 1.0, dTan[1] ) ;
    Curve.m_pTail->GetNormal( 1.0, dNor[1] ) ;
    if( GetMillDir() == 0 ) 
    {
        nc_VectorReverse( dNor[0], 2 ) ;
        nc_VectorReverse( dNor[1], 2 ) ;
    }
    BOOL bIsClosed = FALSE ;
    if( nc_Distance( dPoint[0], dPoint[1], 2 ) < 0.01 )
    {
        dMidNor[0] = dNor[0][0] + dNor[1][0] ;
        dMidNor[1] = dNor[0][1] + dNor[1][1] ;
        nc_Normalize( dMidNor, 2 ) ;
        bIsClosed = TRUE ;
    }
    CSmartSect* pSect = CreateLeadOutEnt( LeadDef, dPoint[0], dTan[0], dNor[0], FALSE, AllCont ) ;
    if( pSect == NULL && bIsClosed ) 
    {
        JDNC_LEAD tmpLead = LeadDef ;
        tmpLead.m_nLeadType =  NCDEF_LEAD_SLINE ;
        pSect = CreateLeadOutEnt( tmpLead, dPoint[0], dTan[0], dMidNor, FALSE, AllCont ) ;
    }
    if( pSect )  Curve.InsertAfter( pSect, NULL ) ;
    pSect = CreateLeadOutEnt( LeadDef, dPoint[1], dTan[1], dNor[1], TRUE, AllCont ) ;
    if( pSect == NULL && bIsClosed ) 
    {
        JDNC_LEAD tmpLead = LeadDef ;
        tmpLead.m_nLeadType =  NCDEF_LEAD_SLINE ;
        pSect = CreateLeadOutEnt( tmpLead, dPoint[1], dTan[1], dMidNor, TRUE, AllCont ) ;
    }
    if( pSect ) Curve.AddSect( pSect ) ;
    return 1 ;
}
int CSmartPathGen::AddLeadPathAtEndEx( JDNC_LEAD& LeadInDef , 
									   JDNC_LEAD& LeadOutDef, 
									    CPathCombine& TComb, 
									    CSmartLoop&  Contour ,
                                        DOUBLE Depth,
                                        DOUBLE ZShift[2],
										DOUBLE DepthInc)
{
	if( LeadInDef.m_nLeadType == 0 && LeadOutDef.m_nLeadType == 0 )
	{
		return 0 ;
	}
    CPathCombine leadInPath(NC_WPROCESS_LEAD), leadOutPath(NC_WPROCESS_LEAD) ;
	BOOL bFlag1 = TRUE, bFlag2 = TRUE;
    bFlag1 = GenLeadPathAtEndEx( LeadInDef , FALSE , TComb, Contour, Depth, ZShift[0], leadInPath, DepthInc ) ;
    bFlag2 = GenLeadPathAtEndEx( LeadOutDef, TRUE  , TComb, Contour, Depth, ZShift[1], leadOutPath, DepthInc ) ;
    while( leadInPath.m_pTail )
    {
        CPathEntity* pTEnt = leadInPath.m_pTail ;
        leadInPath.RemoveEntity( pTEnt ) ;
        TComb.InsertAfter( pTEnt, NULL ) ;
    }
	if (leadOutPath.m_pHead)
	{
		TComb.AppendCombine( leadOutPath ) ;
	}
   
	return bFlag1&&bFlag2 ;
}
int CSmartPathGen::AddLeadSectAtEndEx( JDNC_LEAD& LeadInDef , 
									   JDNC_LEAD& LeadOutDef, 
									   CSmartCurve& Curve   , 
									   CSmartLoop*  AllCont ,
                                       DOUBLE Depth, 
                                       DOUBLE ZShift[2],
									   BOOL bRComp )
{
	// 该函数未使用
	if( ! Curve.m_pHead ) return 0 ;
    CSmartCurve leadInCv, leadOutCv ;
	BOOL bFlag1 = TRUE, bFlag2 = TRUE;
    bFlag1 = GenLeadSectAtEndEx( LeadInDef, FALSE, Curve, AllCont, Depth, ZShift[0], leadInCv ) ;
	bFlag2 = GenLeadSectAtEndEx( LeadOutDef,TRUE , Curve, AllCont, Depth, ZShift[1], leadOutCv ) ;

    CSmartSect* pSect ;
	if( !bRComp )
	{
		for( pSect = leadInCv.GetHead() ; pSect ; pSect = pSect->next )
		{
			pSect->m_dZValue[0] += Curve.m_pHead->m_dZValue[0]  ;
			pSect->m_dZValue[1] += Curve.m_pHead->m_dZValue[0]  ;
		}
		for( pSect = leadOutCv.GetHead() ; pSect ; pSect = pSect->next )
		{
			pSect->m_dZValue[0] += Curve.m_pTail->m_dZValue[1]  ;
			pSect->m_dZValue[1] += Curve.m_pTail->m_dZValue[1]  ;
		}
	}
    Curve.InsertAfter( leadInCv, NULL ) ;
    Curve.AppendCurve( leadOutCv ) ;
	return bFlag1&&bFlag2 ;
}

int CSmartPathGen::AddLeadSectAtEndExNew(  JDNC_LEAD& LeadInDef, JDNC_LEAD& LeadOutDef,CSmartCurve& Curve, 
											CSmartCurve &leadInCv, CSmartCurve &leadOutCv,CSmartLoop*  AllCont
											,DOUBLE Depth, DOUBLE ZShift[2], BOOL bRComp, DOUBLE DepthInc) 
{
	if( ! Curve.m_pHead ) return 0 ;

	BOOL bFlag1 = TRUE, bFlag2 = TRUE;
    bFlag1 = GenLeadSectAtEndEx( LeadInDef, FALSE, Curve, AllCont, Depth, ZShift[0], leadInCv, DepthInc ) ;
	bFlag2 = GenLeadSectAtEndEx( LeadOutDef,TRUE , Curve, AllCont, Depth, ZShift[1], leadOutCv, DepthInc ) ;

    CSmartSect* pSect ;
	if( !bRComp )
	{
		for( pSect = leadInCv.GetHead() ; pSect ; pSect = pSect->next )
		{
			pSect->m_dZValue[0] += Curve.m_pHead->m_dZValue[0]  ;
			pSect->m_dZValue[1] += Curve.m_pHead->m_dZValue[0]  ;
		}
		for( pSect = leadOutCv.GetHead() ; pSect ; pSect = pSect->next )
		{
			pSect->m_dZValue[0] += Curve.m_pTail->m_dZValue[1]  ;
			pSect->m_dZValue[1] += Curve.m_pTail->m_dZValue[1]  ;
		}
	}   
	return bFlag1&&bFlag2 ;
}

void MathCAM_GetSubComb( CPathCombine& OrgTComb, DOUBLE SubLen, BOOL FromEnd, CPathCombine& SubComb )
{
    if( FromEnd == TRUE )
    {
        OrgTComb.ReverseDirect() ;
    }
    DOUBLE dLen = 0.0, dDist  ;
    for( CPathEntity* pTEnt = OrgTComb.m_pHead ; pTEnt ; pTEnt = pTEnt->next )
    {
        //dDist = pTEnt->GetLength() ;
		dDist = MathCam_PathEntityLen3D(pTEnt);//获取pTEnt的3D长度 20140901 xh
        if( dDist + dLen < SubLen )
        {
            SubComb.AddEntity( pTEnt->CopyMyself() ) ;
            dLen += dDist ;
            continue ;
        }
        CPathEntity* pFirst = NULL, * pSecond = NULL ;
        pTEnt->BreakEntity( SubLen - dLen, pFirst, pSecond ) ;
        delete pSecond ;
        if( pFirst ) 
        {
            SubComb.AddEntity( pFirst ) ;
        }
        break ;
    }
    if( FromEnd == TRUE )
    {
        OrgTComb.ReverseDirect() ;
        SubComb.ReverseDirect() ;
    }
}
void MathCAM_SlantPathComb( CPathCombine& TComb , DOUBLE ZStart, DOUBLE ZEnd )
{
    DOUBLE dTotal = TComb.GetPathLen() ;
    DOUBLE dLen = 0.0 ;
    DOUBLE dZ1 = ZStart, dZ2 ;
    for( CPathEntity* pTEnt = TComb.m_pHead ; pTEnt ; pTEnt = pTEnt->next )
    {
        dLen += pTEnt->GetLength() ;
        dZ2 = ZStart + ( ZEnd - ZStart )  * dLen / dTotal ;
        if( pTEnt->next == NULL ) dZ2 = ZEnd ;
        pTEnt->MoveEntity(0.0, 0.0, dZ1 ) ;
        pTEnt->BiasEntity( dZ1-dZ2 ) ;
        dZ1 = dZ2 ;
    }
}
//沿轮廓下刀时，光滑进退刀   xh 20140826
void MathCAM_SmoothLeadINOUT( CPathCombine& TComb , DOUBLE ZStart, DOUBLE ZEnd, const double DisTol = 0.0, 
							  const double AngTol = 0.0/*, const DOUBLE ZPlane = 0.0*/)
{
	if ( !TComb.m_pHead )
	{
		return;
	}
	double dTotal3D = 0.0, dTotal2D = 0.0, dZ1 = 0.0, dZ2 = 0.0 ;
	for ( CPathEntity *pEnt = TComb.m_pHead; pEnt; pEnt = pEnt->next ) //获取CPathCombine的3D长度
	{
		dTotal3D += MathCam_PathEntityLen3D( pEnt );
	}
	dTotal2D = TComb.GetPathLen();                                   //获取CPathCombine的2D长度
	if ( abs( ZStart - ZEnd ) < 0.0001 )
	{
		MathCAM_SlantPathComb( TComb, ZStart, ZEnd );
		return;
	}
	//轮廓切割沿轮廓光滑进退刀   20140814  xh
	double depth     = abs( ZStart - ZEnd );  //高度
	double r         = 0.0;
	bool  bScale     = FALSE;                 //放缩标志
	double dScale[3] = { 1.0, 1.0, 1.0 };     //各坐标放缩比例

	//计算光滑圆弧半径，若dTotal2D / depth > sqrt(3.0),则按实际计算，否则按dTotal2D计算r，并在Y方向进行放缩
	if ( ( dTotal2D / depth ) - sqrt( 3.0 ) > 0.00001 )
	{
		r = ( depth * depth + dTotal2D * dTotal2D ) / ( 2 * depth );
	}
	else
	{
		bScale    = TRUE;
		r         = dTotal2D / sqrt( 3.0 ) * 2;
		dScale[1] = 2 * depth / r;
	}
	PNT3D pStartP  = { dTotal2D, bScale ? r / 2 : depth , 0 };          //圆弧起点
	PNT3D pEndP    = { 0, 0, 0 };                                       //圆弧终点
	PNT3D pCenterP = { 0, r, 0 };                                       //圆心
	VEC3D vNormal  = { 0, 0, 1 };                                       //圆弧所在平面 
	CGeoArc * tempArc = new CGeoArc();                                  //创建初始圆弧
	if ( !tempArc->CreateBy2PCen( 0, pStartP, pEndP, pCenterP, vNormal ) )
	{
		delete tempArc;
		return;
	}
	//CGeoCurve * DesCurve = tempCurve->Scale( EndP, dScale );          //放缩之后的目标曲线

	//离散圆弧曲线，获得离散段数，之后在原始曲线上插点
	CPathEntity  *first = NULL, *second = NULL;
	CStrpt *pStrpt   = NULL;
	int nNum         = 1 ;                                              //轮廓曲线分段数
	pStrpt           = tempArc->DiscreteToStrpt( DisTol, AngTol );
	int nDivNum      = pStrpt->m_np;
	double dAverLen  = dTotal3D / nDivNum;                              //按照理论圆弧离散段数平分轮廓曲线段所得的3D平均长度
	bool ReverseFlag = ( ZStart - ZEnd > 0.0001 ) ? FALSE: TRUE;        //判断进退刀路径

	//将多段折线打断成两点折线段
	CPathEntity *pCurEnt = NULL, *pNextEnt = NULL;
	pCurEnt = TComb.m_pHead;
	while ( pCurEnt )
	{
		pNextEnt = pCurEnt->next;
		if ( pCurEnt->GetType() == NC_PATH_PLINE3D)
		{
			PNT3D pArry[2];
			CPathEntity *pLastEnt = pCurEnt;			
			CPathPLine3D *pNewPLine = NULL, *pPLine3D = ( CPathPLine3D* )pCurEnt;
			for (int i = 1; i <= pPLine3D->m_nCount; ++i)
			{
				mathCpyPnt( pPLine3D->m_pTAPos[i-1], pArry[0] );
				mathCpyPnt( pPLine3D->m_pTAPos[i], pArry[1] );
				pNewPLine = new CPathPLine3D();
				pNewPLine->Copy( pArry, 1 );//ncount代表折线段数

				TComb.InsertAfter( pNewPLine, pLastEnt );
				pLastEnt = pNewPLine;
			}
			TComb.RemoveEntity( pCurEnt );
			delete pCurEnt;
			pCurEnt = NULL;
		}
		pCurEnt = pNextEnt;
	}

	pCurEnt = TComb.m_pHead;
	while ( pCurEnt )
	{
		pNextEnt = pCurEnt->next;
		double dSectLength = MathCam_PathEntityLen3D( pCurEnt );//获得CPathEntity的3D长度
		double dRatio = dSectLength / dAverLen ;
		if ( dRatio - 1 > -0.0001 )
		{
			nNum = ( int )ceil( dRatio );
		}
		//根据离散点个数在psect上均匀插点
		CPathEntity *pLastEnt = pCurEnt;
		for ( int j = 0; j < nNum ; j++ )
		{
			pCurEnt->BreakEntity( dSectLength / nNum , first, second );
			TComb.InsertAfter( first, pLastEnt );
			pLastEnt = first;
			if ( j == 0)
			{
				TComb.RemoveEntity( pCurEnt );
			}
			delete pCurEnt;
			pCurEnt = NULL;
			pCurEnt = second;
		}
		nNum = 1;                               //恢复初值
		pCurEnt = pNextEnt;
	}

	//改变Z坐标
	dZ1         = ZStart;
	double dLen = 0.0;
	double dZ0  = sqrt( r * r - dTotal2D * dTotal2D );	
	for( CPathEntity* pTEnt = TComb.m_pHead; pTEnt ; pTEnt = pTEnt->next )
	{
		dLen += pTEnt->GetLength() ;
		if( pTEnt->next == NULL ) dZ2 = ZEnd ;
		else 
		{
			if ( !ReverseFlag )//是否退刀路径
			{
				double tempLen = dTotal2D - dLen;
				dZ2 = ZStart - ( sqrt( r * r - tempLen * tempLen ) - dZ0 ) * dScale[1];
			}
			else
				dZ2 = ZStart + ( r - sqrt( r * r - dLen * dLen ) ) * dScale[1];
		}
		pTEnt->MoveEntity( 0.0, 0.0, dZ1 ) ;
		pTEnt->BiasEntity( dZ1 - dZ2 ) ;
		dZ1 = dZ2 ;
	}
	//释放内存空间
	delete tempArc;
	tempArc = NULL;
	pStrpt->Destroy();
	delete pStrpt;
	pStrpt = NULL;
}

static void SetLeadSectZValue(CSmartCurve& leadCv, double ZShift, BOOL IsOut)
{
	CSmartSect* pHead = leadCv.m_pHead;
	if (pHead == NULL)
	{
		return;
	}
	CSmartSect* pNext = pHead->next; 
	if (pNext != NULL)
	{// 内切时可能存在外切圆弧导入的情形	
		double dLen1 = pHead->GetLength();
		double dLen2 = pNext->GetLength();
		double dH = ZShift * 0.5;
		double dLen = dLen1 + dLen2;
		if (dLen > 1.0e-4)
		{
			dH = ZShift * (dLen1 / dLen);
		}
		if( IsOut == FALSE ) 
		{
			dH = ZShift - dH;
			pNext->SetZValue( dH, 0.0 ) ;
			pHead->SetZValue( ZShift, dH ) ;
		}
		else 
		{			
			pHead->SetZValue( 0.0, dH ) ;
			pNext->SetZValue( dH, ZShift ) ;
		}
	}
	else
	{
		if( IsOut == FALSE ) pHead->SetZValue( ZShift, 0.0 ) ;
		else pHead->SetZValue( 0.0, ZShift ) ;
	}
}

static BOOL MathCAM_SubLeadDist( JDNC_LEAD& tmpLead );
//生成沿轮廓下刀时的重复加工路径,将原始路径打断 20140831 xh
void CSmartPathGen::GenOverLappedPath(CPathCombine& TComb, CPathCombine& overLappedPath, DOUBLE OverLapLen)
{
	if ( TComb.m_pHead == NULL || abs( OverLapLen ) < 0.0001 )
	{
		return;
	}

	CPathCombine tmpTComb( NC_WPROCESS_ROUGH);
	PNT3D dPoint[2];
	DOUBLE dEnd[3], dtmpTComb = 0.0;
	TComb.m_pHead->GetEndPoint( 0, dPoint[0] ) ;
	TComb.m_pTail->GetEndPoint( 1, dPoint[1] ) ;

	for( CPathEntity* pTEnt = TComb.m_pTail ; pTEnt ; pTEnt = pTEnt->prev )
	{//寻找与末点相邻的路径段
		pTEnt->GetEndPoint( 0, dEnd ) ;
		dtmpTComb += MathCam_PathEntityLen3D( pTEnt );
		if( nc_Distance( dPoint[1], dEnd, 3 ) < 0.002 )
		{
			//从TComb的末尾取路径段，在tmpTComb逆序填入，直到末点相邻的路径段
			for( CPathEntity* pTAt = TComb.m_pTail ; pTAt ; pTAt = pTAt->prev )
			{
				tmpTComb.InsertAfter(pTAt->CopyMyself(), NULL ) ; 
				if( pTAt == pTEnt ) break ;
			}
			break ;
		}
	}
	if ( dtmpTComb < OverLapLen )
	{
		//指定的重复加工长度过长！
		OverLapLen = dtmpTComb;
	}
	if( tmpTComb.m_pHead )
	{
		MathCAM_GetSubComb( tmpTComb, OverLapLen, 0, overLappedPath ) ;
	}
	else
	{
		MathCAM_GetSubComb( TComb, OverLapLen, 1, overLappedPath ) ;
		overLappedPath.ReverseDirect() ;
	}
	
	//在重复加工路径的末点打断原始路径，以便于后续进退刀路径生成
	DOUBLE dDist = 0.0, dLen = 0.0, dBreakLen = 0.0 ;
	if ( tmpTComb.m_pHead != NULL )
	{
		dBreakLen = dtmpTComb - OverLapLen;
		
	}
	else
	{
		dBreakLen = OverLapLen;
	}
	for( CPathEntity* pTEnt = TComb.m_pTail ; pTEnt ; pTEnt = pTEnt->prev )
	{  
		if ( pTEnt->m_bFeedType != JDNC_FEEDTYPE_ROUGH )
		{
			continue;
		}
		dDist = MathCam_PathEntityLen3D( pTEnt ) ;
		if(  dDist - (dBreakLen - dLen)  < -0.0001)
		{
			dLen += dDist ;
			continue ;
		}
		if ( abs( dDist - (dBreakLen - dLen) ) <= 0.0001)
		{
			return;
		}
		CPathEntity* pFirst = NULL, * pSecond = NULL ;
		pTEnt->BreakEntity( dDist - (dBreakLen - dLen), pFirst, pSecond ) ;
		if( pFirst ) 
		{
			TComb.InsertAfter( pFirst, pTEnt ) ;
		}
		if ( pSecond )
		{
			TComb.InsertAfter( pSecond, pFirst ) ;
		}
		TComb.RemoveEntity( pTEnt );
		if ( pTEnt != NULL)
		{
			delete pTEnt;
		}
		break ;
	}
}
//获得重复加工路径的curve，并将原始curve打断 20141009 xh
void CSmartPathGen::GenOverLappedPath( CSmartCurve& Curve, CSmartCurve& overLapped, DOUBLE OverLapLen)
{
	if ( Curve.m_pHead == NULL || abs( OverLapLen ) < 0.0001 )
	{
		return;
	}

	PNT2D dPoint ;
	DOUBLE dStart[2], dLen = 0.0;
	Curve.GetEnd( dPoint ) ;
	CSmartCurve tmpCurve;
	
	CSmartSect *pTEnt = Curve.m_pTail;
	if ( abs( Curve.m_pTail->GetLength()) < 0.002 )
	{
		pTEnt = Curve.m_pTail->prev;	
	}
	while ( pTEnt != NULL )
	{
		pTEnt->GetStart( dStart );
		dLen += pTEnt->GetLength();
		if( nc_Distance( dPoint, dStart, 2 ) < 0.002 )
		{
			//从curve的末尾取子段，在tmpCurve逆序填入，直到末点相邻的路径段
			for( CSmartSect* pTAt = Curve.m_pTail ; pTAt ; pTAt = pTAt->prev )
			{
				tmpCurve.InsertAfter( pTAt->CopyMyself(), NULL ) ; 
				if( pTAt == pTEnt ) 
				{
					break ;
				}			
			}
			break ;
		}
		pTEnt = pTEnt->prev;
	}
	if ( dLen < OverLapLen )
	{
		//指定的重复加工长度过长！
		OverLapLen = dLen;
	}
	CSmartCurve *first = NULL, *second = NULL;
	if( tmpCurve.m_pHead )
	{
		tmpCurve.BreakCurveAt( OverLapLen, first, second);
		overLapped.AddCurve( first );
	}
	else
	{
		Curve.Reverse();
		Curve.BreakCurveAt( OverLapLen, first, second);
		overLapped.AddCurve( first );
		overLapped.Reverse() ;
		Curve.Reverse();
	}
	if ( second != NULL )
	{
		delete second;
		second = NULL;
	}
	//打断原始曲线，以便于后续生成进退刀路径
	double dBreakLen = 0.0;
	if ( tmpCurve.m_pHead != NULL )
	{
		dBreakLen = Curve.GetLength() - tmpCurve.GetLength() + OverLapLen;
	}
	else
	{
		dBreakLen = Curve.GetLength() - OverLapLen;
	}
	Curve.BreakCurveAt( dBreakLen, first, second );
	Curve.ClearAll();
	Curve.AddCurve(first);
	Curve.AddCurve(second);
}

int CSmartPathGen::GenLeadPathAtEndEx( JDNC_LEAD& LeadDef , 
									   BOOL IsOut ,
									   CPathCombine& TComb, 
									   CSmartLoop&  Contour ,
                                       DOUBLE Depth ,
                                       DOUBLE ZShift,
                                       CPathCombine& LeadPath,
									   DOUBLE DepthInc)
{
	UNUSED_ALWAYS(Depth);
	if( LeadDef.m_nLeadType == 0 )
	{
		return 0 ;
	}
    DOUBLE ZPlane = 0.0 ;
    PNT3D dPoint[2] ; 
    CSmartCurve leadCv ;
	TComb.m_pHead->GetEndPoint( 0, dPoint[0] ) ;
	TComb.m_pTail->GetEndPoint( 1, dPoint[1] ) ;
	if( IsOut == FALSE ) ZPlane = dPoint[0][2] ;
	else ZPlane = dPoint[1][2] ;
    if( LeadDef.m_nLeadType == NCDEF_LEAD_BYCONTOUR )
    {
		// 注释掉原因:避免保留曲线高度时，不能生成进退刀路径
        //if( ZPlane >= Depth )
        //{
        //    return FALSE ;
        //}
        DOUBLE dEnd[3] ;
        CPathCombine tmpTComb( NC_WPROCESS_LEAD) ;
        if( IsOut == FALSE )
        {/*沿轮廓进刀*/
            for( CPathEntity* pTEnt = TComb.m_pHead ; pTEnt ; pTEnt = pTEnt->next )
            {
                pTEnt->GetEndPoint( 1, dEnd ) ;
	            if( nc_Distance( dPoint[0], dEnd, 3 ) < 0.002 )
                {
                    for( CPathEntity* pTAt = TComb.m_pHead ; pTAt ; pTAt = pTAt->next )
                    {
                        tmpTComb.AddEntity( pTAt->CopyMyself() ) ;
                        if( pTAt == pTEnt ) break ;
                    }
                    break ;
                }
            }
            if( tmpTComb.m_pHead )
	        {
                MathCAM_GetSubComb( tmpTComb, LeadDef.m_dLength, 1, LeadPath ) ;
            }
            else
            {
                MathCAM_GetSubComb( TComb, LeadDef.m_dLength, 0, LeadPath ) ;
                LeadPath.ReverseDirect() ;
            }
        }
        else 
        {/*沿轮廓退刀*/
            for( CPathEntity* pTEnt = TComb.m_pTail ; pTEnt ; pTEnt = pTEnt->prev )
            {
                pTEnt->GetEndPoint( 0, dEnd ) ;
	            if( nc_Distance( dPoint[1], dEnd, 3 ) < 0.002 )
                {
                    for( CPathEntity* pTAt = TComb.m_pTail ; pTAt ; pTAt = pTAt->prev )
                    {
                        tmpTComb.InsertAfter(pTAt->CopyMyself(), NULL ) ; 
                        if( pTAt == pTEnt ) break ;
                    }
                    break ;
                }
            }
            if( tmpTComb.m_pHead )
	        {
                MathCAM_GetSubComb( tmpTComb, LeadDef.m_dLength, 0, LeadPath ) ;
            }
            else
            {
                MathCAM_GetSubComb( TComb, LeadDef.m_dLength, 1, LeadPath ) ;
                LeadPath.ReverseDirect() ;
            }
        }
        for( CPathEntity* pTEnt = LeadPath.m_pHead ; pTEnt ; pTEnt = pTEnt->next )
        {
            pTEnt->m_bFeedType = JDNC_FEEDTYPE_LEAD  ;
        }
		bool FlagArc = (m_cProcessDef.m_bProcessFlag & NCDEF_SMOOTHLEAD) ? TRUE : FALSE;//20140819 xh
        if( IsOut == FALSE )
        {
			if (FlagArc)
			{//沿轮廓下刀时关闭进刀，因此暂不考虑进刀路径保留曲线高度的情况 20140827 xh
				MathCAM_SmoothLeadINOUT(LeadPath,  DepthInc/*-ZPlane*/, 0.0 , 
					m_cSetupDef.m_cTolDef.m_dArcTol, m_cSetupDef.m_cTolDef.m_dAngTol);
			}
			else
			{
				MathCAM_SlantPathComb( LeadPath, DepthInc/*-ZPlane*/, 0.0 );//20140815 xh
			}
        }
        else
        {
			if ( !m_cMethodDef.m_cLoopCut.m_cLeadOut.m_bSameLeadIn )
			{
				FlagArc = (m_cProcessDef.m_bProcessFlag & NCDEF_SMOOTHLEADOUT) ? TRUE : FALSE;//20140819 xh
			}
			if (FlagArc)
			{//光滑退刀路径   20140827 xh
				BOOL bKeep[2] = { FALSE, FALSE } ;
				if(  m_cMethodDef.m_cLoopCut.m_bLoopFlag & NCDEF_FLOOP_REDEPTH  )
				{ 
					bKeep[0] = TRUE ;
				}
				MathCAM_SmoothLeadINOUT(LeadPath, 0.0, DepthInc/*-ZPlane*/, m_cSetupDef.m_cTolDef.m_dArcTol, 
					                    m_cSetupDef.m_cTolDef.m_dAngTol );	
			}
			else
			{
				MathCAM_SlantPathComb( LeadPath, 0.0, DepthInc/*-ZPlane*/);//20140815  xh
			}
            
        }
        return TRUE ;
    }
    else
    {
		CSmartCurve leadCv ;
	    PNT3D dTan[2], dNor[2];
		// 获取路径在起末点处的切向和法向，并避免闭合曲线取得是前后两点的角平分线方向
		// 从而在尖点处引起进退刀扭曲
		GetTanAndNorAtPathCombineEnd(TComb, dTan, dNor);

	    if( GetMillDir() == 0 ) 
	    {
		    nc_VectorReverse( dNor[0], 2 ) ;
		    nc_VectorReverse( dNor[1], 2 ) ;
	    }

        BOOL nId = ( IsOut == FALSE ) ? 0 : 1 ;
	    CSmartLoop * tmpCont = Contour.CopyContour() ;
		BOOL bRet = CreateLeadOutEntNew( LeadDef, dPoint[nId], dTan[nId], 
										 dNor[nId], IsOut, tmpCont, leadCv) ;
		BOOL bSplineH = TRUE ;
		if( !bRet ) 
		{
			// 此处增加对进退刀“计算失败时”处理方式的支持，改修改涉及加工方法包括扩孔、区域修边和轮廓切割 qqs2014.3.21
			JDNC_LEAD tmpLead = LeadDef ;
			while (MathCAM_SubLeadDist( tmpLead )&&!bRet) 
			{
				bRet = CreateLeadOutEntNew( tmpLead, dPoint[nId], dTan[nId], dNor[nId], IsOut, tmpCont, leadCv ) ;
			}
			bSplineH = FALSE ;
		}
		Mini_DeleteContours( tmpCont ) ;
		if( !bRet || leadCv.m_pHead == NULL ) return 0 ;

		if( bSplineH )
		{
			bSplineH = ChangeLeadArcHeightBySpline( &leadCv, LeadDef, ZShift, !IsOut ) ;
		}
		if( !bSplineH )
			SetLeadSectZValue(leadCv, ZShift, IsOut);
		leadCv.SetFeedType( JDNC_FEEDTYPE_LEAD ) ;
		LeadPath.AppendCurve3D( leadCv, ZPlane, TRUE ) ; 

		return TRUE ;
	}   
}

// 获取CPathCombine起末点的切向方向和法向方向，用于生成进退刀路径 qqs 2014.03.05
void CSmartPathGen::GetTanAndNorAtPathCombineEnd(CPathCombine& TPComb, VEC3D dTan[2], VEC3D dNor[2])
{
	if (TPComb.m_pHead->GetType() == NC_PATH_PLINE3D )
	{
		// 闭合的折线类型CPathEntity，直接通过GetEndTangent和GetEndNormal获取的尖点处的法向值为尖点角
		// 平分线的方向，切向值与之垂直，生成的路径将不会满足切向进退刀需求，需要单独求解。qqs 2014.03.05
		CPathPLine3D* pLine3D = (CPathPLine3D*) TPComb.m_pHead;
		mathGetVec2D(pLine3D->m_pTAPos[0], pLine3D->m_pTAPos[1], dTan[0]);
		mathUniVec2D(dTan[0],1.0e-4);
		dNor[0][0] = -dTan[0][1];
		dNor[0][1] = dTan[0][0];
		nc_VectorReverse( dTan[0], 2 ) ;
	}
	else
	{
		TPComb.m_pHead->GetEndTangent( 0, dTan[0] ) ;
		nc_VectorReverse( dTan[0], 2 ) ;
		TPComb.m_pHead->GetEndNormal( 0, dNor[0] ) ;
	}

	if (TPComb.m_pTail->GetType() == NC_PATH_PLINE3D )
	{
		// 闭合的折线类型CPathEntity，直接通过GetEndTangent和GetEndNormal获取的尖点处的法向值为尖点角
		// 平分线的方向，切向值与之垂直，生成的路径将不会满足切向进退刀需求，需要单独求解。qqs 2014.03.05
		CPathPLine3D* pLine3D = (CPathPLine3D*) TPComb.m_pTail;
		mathGetVec2D(pLine3D->m_pTAPos[pLine3D->m_nCount-1], pLine3D->m_pTAPos[pLine3D->m_nCount], dTan[1]);
		mathUniVec2D(dTan[1],1.0e-4);

		dNor[1][0] = -dTan[1][1];
		dNor[1][1] = dTan[1][0];
	}
	else
	{
		TPComb.m_pTail->GetEndTangent( 1, dTan[1] ) ;
		TPComb.m_pTail->GetEndNormal( 1, dNor[1] ) ;
	}

	nc_Normalize( dTan[0], 2 ) ;
	nc_Normalize( dTan[1], 2 ) ;
}

static DOUBLE MathCAM_MinDistToContour( CSmartLoop& Contour, PNT2D Point )
{
	double dMinDist = Contour.MinDistPoint( Point ) ;
	for( CSmartLoop * pIsl = Contour.GetIsland() ; pIsl; pIsl = pIsl->next )
	{
		double dDist = pIsl->MinDistPoint( Point ) ;
		if( dDist < dMinDist ) dMinDist = dDist ;
	}
	return dMinDist; 
}

int CSmartPathGen::GenLeadSectAtEndEx( JDNC_LEAD& LeadDef , 
									   BOOL IsOut ,
									   CSmartCurve& Curve   , 
									   CSmartLoop*  AllCont , 
                                       DOUBLE Depth ,
                                       DOUBLE ZShift,
                                       CSmartCurve& LeadCv ,
									   DOUBLE DepthInc)
{
	if( ! Curve.m_pHead ) return 0 ;
	if( LeadDef.m_nLeadType == 0)
	{
		return 0 ;
	}
    if( LeadDef.m_nLeadType == NCDEF_LEAD_BYCONTOUR )
    {
        double dLen = Curve.GetLength() ;
        CSmartCurve* pTemp = NULL, *pLeadCv = NULL ; 
        if( IsOut == FALSE )
        {/*沿轮廓进刀*/
			if( Curve.IsClosed() )
			{
				Curve.BreakCurveAt( dLen - LeadDef.m_dLength, pTemp, pLeadCv ) ;
			}
			else
			{
				// edit by liuxin 2013.8.9，解决螺旋路径沿轮廓进刀反向问题
				PNT2D dPoint[2];
				CSmartCurve tmpCur;
				CSmartSect* pSect = NULL, *pPrevSect = NULL;
				Curve.GetEnd(dPoint[1]);
				for (pSect = Curve.m_pTail; pSect; pSect = pSect->prev)
				{
					pPrevSect = pSect->prev;
					if (pSect && pPrevSect)
					{
						pPrevSect->GetStart(dPoint[0]);
						if (mathDist2D(dPoint[0], dPoint[1]) < MIN_DIS)
						{
							tmpCur.InsertAfter(pSect->CopyMyself(), NULL);
							tmpCur.InsertAfter(pPrevSect->CopyMyself(), NULL);
							break;
						}
						else
						{
							tmpCur.InsertAfter(pSect->CopyMyself(), NULL);
						}
					}
				}
				if (tmpCur.m_pHead)
				{
					CSmartLoop* pLoop = tmpCur.FormLoop();
					if (pLoop)
					{
						Curve.GetStart(dPoint[0]);
						Curve.GetEnd(dPoint[1]);
						double dDist = max( 0.0, MathCAM_MinDistToContour( *pLoop, dPoint[0] )) ;
						CSmartLoop* pBndCount = pLoop->OffsetContourEx(dDist, dDist, m_cSetupDef.m_cCorDef);
						if (pBndCount)
						{
							pSect = pBndCount->m_pCurve->MinDistSect(dPoint[0], dPoint[1], dDist);
							if (pSect && dDist < MIN_DIS && pSect->m_nSectId >= 0 &&
								pSect->m_nSectId < pBndCount->m_pCurve->m_nNumSect)
							{
								pBndCount->m_pCurve->SetStartPoint(pSect, dPoint[1]);
								pBndCount->m_pCurve->BreakCurveAt( pBndCount->m_pCurve->GetLength() - LeadDef.m_dLength, 
																pTemp, pLeadCv ) ;
							}
							Mini_DeleteContours(pBndCount);
						}
						Mini_DeleteContours(pLoop);
					}					
				}
				if (!pLeadCv)
				{
					Curve.BreakCurveAt( LeadDef.m_dLength, pLeadCv, pTemp ) ;
					if( pLeadCv ) pLeadCv->Reverse() ;
				}
			}
        }
        else
        {/*沿轮廓退刀*/
			if( Curve.IsClosed() )
			{
				Curve.BreakCurveAt( LeadDef.m_dLength, pLeadCv, pTemp ) ;
			}
			else
			{
				// edit by liuxin 2013.8.9，解决螺旋路径沿轮廓退刀反向问题
				PNT2D dPoint[2];
				CSmartCurve tmpCur;
				CSmartSect* pSect = NULL, *pPrevSect = NULL;
				Curve.GetEnd(dPoint[1]);
				for (pSect = Curve.m_pTail; pSect; pSect = pSect->prev)
				{
					pPrevSect = pSect->prev;
					if (pSect && pPrevSect)
					{
						pPrevSect->GetStart(dPoint[0]);
						if (mathDist2D(dPoint[0], dPoint[1]) < MIN_DIS)
						{
							tmpCur.InsertAfter(pSect->CopyMyself(), NULL);
							tmpCur.InsertAfter(pPrevSect->CopyMyself(), NULL);
							break;
						}
						else
						{
							tmpCur.InsertAfter(pSect->CopyMyself(), NULL);
						}
					}
				}
				if (tmpCur.m_pHead)
				{
					CSmartLoop* pLoop = tmpCur.FormLoop();
					if (pLoop)
					{
						pLoop->m_pCurve->BreakCurveAt(LeadDef.m_dLength, pLeadCv, pTemp);
						Mini_DeleteContours(pLoop);
					}
				}
				if (!pLeadCv)
				{
					Curve.BreakCurveAt( dLen - LeadDef.m_dLength, pTemp, pLeadCv ) ;
					if( pLeadCv ) pLeadCv->Reverse() ;
				}
			}
        }
        if( pTemp ) delete pTemp ;
		bool FlagArc = (m_cProcessDef.m_bProcessFlag & NCDEF_SMOOTHLEAD) ? TRUE : FALSE;//20140819 xh
        if( pLeadCv )
        {
            pLeadCv->SetFeedType( JDNC_FEEDTYPE_LEAD ) ;
            if( IsOut == FALSE )
            {//是否选择光滑进刀 xh
				if ( FlagArc )
				    MathCAM_SmoothLeadINOUT( *pLeadCv, Depth, Depth - DepthInc );
				else
                    MathCAM_SetZValue( *pLeadCv, Depth, Depth - DepthInc );
            }
            else
            {
				if ( !m_cMethodDef.m_cLoopCut.m_cLeadOut.m_bSameLeadIn )//退刀参数是否与进刀相同 xh
				{
					FlagArc = (m_cProcessDef.m_bProcessFlag & NCDEF_SMOOTHLEADOUT) ? TRUE : FALSE;//20140819 xh
				}//是否选择光滑退刀 xh
				if ( FlagArc )
					MathCAM_SmoothLeadINOUT( *pLeadCv, Depth - DepthInc, Depth );
				else
                    MathCAM_SetZValue( *pLeadCv, Depth - DepthInc, Depth );
            }
            LeadCv.AppendCurve( * pLeadCv ) ;
            delete pLeadCv ;
            return TRUE ;
        }
        return FALSE ;
    }
    else
    {
	    PNT3D dPoint[2], dTan[2], dNor[2], dMidNor;
	    Curve.m_pHead->GetStart(dPoint[0] ) ;
	    Curve.m_pHead->GetTangent( 0.0, dTan[0] ) ;
	    Curve.m_pHead->GetNormal( 0.0, dNor[0] ) ;
	    nc_VectorReverse( dTan[0], 2 ) ;
	    Curve.m_pTail->GetEnd( dPoint[1] ) ;
	    Curve.m_pTail->GetTangent( 1.0, dTan[1] ) ;
	    Curve.m_pTail->GetNormal( 1.0, dNor[1] ) ;
	    if( GetMillDir() == 0 ) 
	    {
		    nc_VectorReverse( dNor[0], 2 ) ;
		    nc_VectorReverse( dNor[1], 2 ) ;
	    }
	    BOOL bIsClosed = FALSE ;
	    if( nc_Distance( dPoint[0], dPoint[1], 2 ) < 0.01 )
	    {
		    dMidNor[0] = dNor[0][0] + dNor[1][0] ;
		    dMidNor[1] = dNor[0][1] + dNor[1][1] ;
		    nc_Normalize( dMidNor, 2 ) ;
		    bIsClosed = TRUE ;
	    }
        int nId = ( IsOut == FALSE ) ? 0 : 1 ;
		BOOL bRet = CreateLeadOutEntNew( LeadDef, dPoint[nId], dTan[nId], 
										 dNor[nId], IsOut, AllCont, LeadCv) ;
		BOOL bSplineH = TRUE ;
		if( !bRet && bIsClosed ) 
		{
			JDNC_LEAD tmpLead = LeadDef ;
			tmpLead.m_nLeadType =  NCDEF_LEAD_SLINE ;
			bRet = CreateLeadOutEntNew( tmpLead, dPoint[nId], dTan[nId], dMidNor, IsOut, AllCont, LeadCv ) ;
			bSplineH = FALSE ;
		}
		if( !bRet || LeadCv.m_pHead == NULL ) return 0 ;
		if( bSplineH )
		{
			bSplineH = ChangeLeadArcHeightBySpline( &LeadCv, LeadDef, ZShift, !IsOut ) ;
		}
		if( !bSplineH ) SetLeadSectZValue(LeadCv, ZShift, IsOut);

		LeadCv.SetFeedType( JDNC_FEEDTYPE_LEAD ) ;
    }
	return 1 ;
}

void CSmartPathGen::SetAllLoopHeadAtBlank ( CSmartLoop *AllLoop )
{
	if( !AllLoop ) return ;
	CSmartLoop *pLoop = AllLoop, *pIsland = NULL ;
	for( ; pLoop ; pLoop = pLoop->next )
	{
		SetLoopHeadAtBlank( pLoop ) ;
		pIsland = pLoop->GetIsland () ;
		for( ; pIsland ; pIsland = pIsland->next )
		{
			SetLoopHeadAtBlank( pIsland ) ;
		}
	}
}
void CSmartPathGen::SetLoopHeadAtBlank ( CSmartLoop *pLoop )
{
	if( !pLoop ) return ;
	
	CSmartSect *pHead = NULL, *pNext = NULL, *Start = NULL, *End = NULL ;
	PNT2D tmp ;
	double dDist = 0, dTotal = 0., dTmp = 0., u = 0 ;
	BOOL bFind = FALSE ;
	
	CSmartCurve *pCurve = pLoop->m_pCurve ;
	pHead = pCurve->m_pHead ;
	bFind = FALSE ;
	while( pHead )
	{
		pNext = pHead->next ;
		if( !pNext ) break ;
		if( pHead->m_bEndFlag & NC_BLANK_SECT && pNext->m_bEndFlag & NC_BLANK_SECT )
		{
			dTotal = pHead->GetLength () + pNext->GetLength () ;
			Start = pHead, End = pNext->next ;
			for( ; End ; End = End->next )
			{
				if( End->m_bEndFlag & NC_BLANK_SECT )
                    dTotal += End->GetLength () ;
				else
					break ;
			}
			dTotal = dTotal * 0.5, dDist = 0 ;
			// 取中点   
			for( ; Start ; Start = Start->next )
			{
				if( Start == End ) break ;
				dTmp = Start->GetLength () ;
				dDist += dTmp ;
				if( dDist > dTotal )
				{
					u = 1 - ( dDist - dTotal ) / dTmp ;
					Start->GetPoint( u, tmp ) ;
					pCurve->SetStartPoint ( Start, tmp ) ;
					bFind = TRUE ;
					return ;
				}
			}
		}
		pHead = pNext ;
	}
	if( !bFind ) 
	{
		for( pHead = pCurve->m_pHead ; pHead ; pHead = pHead->next )
		{
			if( pHead->m_bEndFlag & NC_BLANK_SECT )
			{
				pHead->GetPoint ( 0.5, tmp ) ;
				pCurve->SetStartPoint ( pHead, tmp ) ;
				break ;
			}
		}
	}
}
BOOL CSmartPathGen::HasBlankSectInContour( CSmartLoop *BndHead )
{
	if( !BndHead ) return FALSE ;
	CSmartLoop *pIsland = NULL ;
	CSmartSect *pHead = NULL ;

	CSmartCurve *pCurve = BndHead->m_pCurve ;
	for( ; pCurve ; pCurve = pCurve->next )
	{
		pHead = pCurve->m_pHead ;
		for( ; pHead ; pHead = pHead->next )
		{
			if( pHead->m_bEndFlag & NC_BLANK_SECT )
				return TRUE ;
		}
	}
	pIsland = BndHead->GetIsland () ;
	for( ; pIsland ; pIsland = pIsland->next )
	{
		pCurve = pIsland->m_pCurve ;
		for( ; pCurve ; pCurve = pCurve->next )
		{
			pHead = pCurve->m_pHead ;
			for( ; pHead ; pHead = pHead->next )
			{
				if( pHead->m_bEndFlag & NC_BLANK_SECT )
					return TRUE ;
			}
		}
	}
	return FALSE ;
}
int CSmartPathGen::InsertVertPlunge(CPathCombine&    TComb  , /*路径*/ 
								      JDNC_PLUNGE&     Plunge ) /*下刀*/ 
{
	// STEP 0 : 偏移轮廓
	if( ! TComb.m_pHead )
	{ // 不必插入下刀路径
		return  0 ;
	}
	// STEP 1 : 搜索孤立的点
	CPathEntity* pEnt = TComb.m_pHead, *pPrev = NULL ;
	TPNT3D dTPoint, dAt[2] ;
	PNT3D dPoint, dEnd  ;
	CSmartCurve tmpCurve ;
	while( pEnt )
	{
		CPathCombine cTPlunge( NC_WPROCESS_PLUNGE ) ;
		pEnt->GetEndPoint( 0, dTPoint ) ;
		if( dTPoint[2] > 0.001 )
		{
			while( pEnt )
			{
				pPrev = pEnt, pEnt = pEnt->next  ;
				if( ! pEnt ) break ;
				pPrev->GetEndPoint( 1 , dAt[0] ) ;
				pEnt->GetEndPoint( 0, dAt[1] ) ;
				if( nc_Distance( dAt[0], dAt[1],2 ) < 0.005 )
				{
					continue ;
				}
				else break ;
			}
			continue ;
		}
        nc_VectorCopy( dPoint, dTPoint, 3 ) ;
		if( TComb.FindPlungePoint( dPoint,dEnd, 0.002 , 
				                   pEnt, NULL ) )
		{// 尽量在已经切割的位置下刀
            if( nc_Distance( dEnd, dPoint, 3 ) > 0.8e-4 )
            {
                CPathLine3D  *pLine3D = new CPathLine3D(dEnd, dPoint) ;
                pLine3D->m_bFeedType = JDNC_FEEDTYPE_CONNECT ;
                cTPlunge.AddEntity( pLine3D ) ;
            }
            else
            {
                nc_VectorCopy( dEnd, dPoint, 3 ) ;
            }
		}
        else 
        {// 采用参数中的下刀方式
            TPNT3D dPrevEnd ;
            double dPlgDist = min( Plunge.m_dIncStep, 0.0 - dPoint[2] ) ;
            if( dPlgDist > 0.01 )
            {
                nc_VectorCopy( dEnd, dPoint, 3 ) ;
                dPrevEnd[0] = dEnd[0], dPrevEnd[1] = dEnd[1] ;
                dPrevEnd[2] = dEnd[2] + dPlgDist ;
                CPathLine3D  *pLine3D = new CPathLine3D(dPrevEnd, dEnd) ;
                pLine3D->m_bFeedType = JDNC_FEEDTYPE_PLUNGE ;
                cTPlunge.InsertAfter( pLine3D, NULL ) ;
            }
        }
		if( cTPlunge.m_pHead )
		{ 
			if( ! pPrev ) 
			{
				cTPlunge.m_pTail->next = TComb.m_pHead ;
				TComb.m_pHead->prev = cTPlunge.m_pTail ; 
				TComb.m_pHead = cTPlunge.m_pHead ;
			}
			else 
			{
				cTPlunge.m_pTail->next = pPrev->next ;
				pPrev->next->prev = cTPlunge.m_pTail ;
				pPrev->next = cTPlunge.m_pHead ; 
				cTPlunge.m_pHead->prev = pPrev ;
			}
			cTPlunge.m_pHead = cTPlunge.m_pHead = NULL ; 
		}
		while( pEnt )
		{
			pPrev = pEnt, pEnt = pEnt->next  ;
			if( ! pEnt ) break ;
			pPrev->GetEndPoint( 1 , dAt[0] ) ;
		    pEnt->GetEndPoint( 0, dAt[1] ) ;
			if( nc_Distance( dAt[0], dAt[1],2 ) < 0.005 )
			{
				continue ;
			}
			else break ;
		}
    }
	return 1 ; 
}

int  CSmartPathGen::CreateBoundLead( JDNC_LEAD&   LeadDef ,
								     CSmartCurve&  Curve, 
								     CSmartSect*&  LeadIn ,
								     CSmartSect*&  LeadOut,
								     CSmartLoop*   Contour ) 
{
	LeadIn = LeadOut = NULL ; 
	if( ! Curve.m_pHead )  return  0 ; 
	PNT2D dPoint[2], dNorm[2] ; 
	CSmartSect * SectHead = Curve.GetHead() ; 
	int  bEscape, nState = 1 ;
	JDNC_LEAD tmpLead = LeadDef ;
	while( 1 )
	{
		Curve.m_pHead->GetPoint( 0.0, dPoint[0] ) ;
		Curve.m_pHead->GetNormal( 0.0, dNorm[0] ) ;
		Curve.m_pTail->GetPoint( 1.0, dPoint[1] ) ;
		Curve.m_pTail->GetNormal( 1.0, dNorm[1] ) ;
		bEscape = FALSE  ;
		if( tmpLead.m_nLeadType == NCDEF_LEAD_LINE && nState == 1 )
		{// 直线相切
			if( Curve.m_pHead->IsSectLine() || 
				Curve.m_pTail->IsSectLine()    )
			{
				bEscape = TRUE ; 
			}
			else
			{
				Curve.m_pHead->GetTangent( 0.0, dNorm[0] ) ;
			    nc_VectorReverse( dNorm[0], 2 ) ;
			    Curve.m_pTail->GetTangent( 1.0, dNorm[1] ) ;
			}
		}
		else if( tmpLead.m_nLeadType == NCDEF_LEAD_LINE && nState == 2 )
		{// 直线相切
			if( Curve.m_pHead->IsSectArc() )
			{
				Curve.m_pHead->GetTangent( 0.5, dNorm[0] ) ;
			    Curve.m_pHead->GetPoint( 0.5, dPoint[0] ) ;
			    memcpy( dNorm[1] ,dNorm[0], sizeof(PNT2D) ) ;
			    memcpy( dPoint[1],dPoint[0], sizeof(PNT2D) ) ;
			    nc_VectorReverse( dNorm[0], 2 ) ;
			}
			else bEscape = TRUE ;
		}
		else if( tmpLead.m_nLeadType == NCDEF_LEAD_SLINE )
		{ // 直线连接
			if( nc_Distance( dPoint[0], dPoint[1],2 ) < 1.0e-2 )
			{
				dNorm[0][0] += dNorm[1][0], dNorm[0][1] += dNorm[1][1] ;
	            nc_Normalize( dNorm[0], 2 ) ;
				dNorm[1][0] = dNorm[0][0], dNorm[1][1] = dNorm[0][1] ;
			}
		}
		else if( tmpLead.m_nLeadType == NCDEF_LEAD_ARC && nState == 1 ||
                 tmpLead.m_nLeadType == NCDEF_LEAD_INARC && nState == 1 )
		{
			if( nc_Distance( dPoint[0], dPoint[1],2 ) < 1.0e-2 )
			{
				DOUBLE dOA = dNorm[1][0] * dNorm[0][1] - dNorm[1][1] * dNorm[0][0] ;
				if( dOA > 0.17 )
				{
					bEscape = TRUE ;
				}
			}
		}
		else if( tmpLead.m_nLeadType == NCDEF_LEAD_ARC && nState == 2 ||
                 tmpLead.m_nLeadType == NCDEF_LEAD_INARC && nState == 2  )
		{
			Curve.m_pHead->GetNormal( 0.5, dNorm[0] ) ;
			Curve.m_pHead->GetPoint( 0.5, dPoint[0] ) ;
			memcpy( dNorm[1] ,dNorm[0], sizeof(PNT2D) ) ;
			memcpy( dPoint[1],dPoint[0], sizeof(PNT2D) ) ;
		}
	    if( LeadIn ) delete LeadIn  ;
		if( LeadOut) delete LeadOut ; 
		LeadIn = LeadOut = NULL ; 
		if( bEscape == FALSE )
		{
			LeadIn = CreateLeadEnt( tmpLead, dPoint[0], dNorm[0], 0 , Contour ) ;
            if( LeadIn )
            {
                LeadOut= CreateLeadEnt( tmpLead, dPoint[1], dNorm[1], 1 , Contour ) ;
                if( ! LeadOut )
                {
                    delete LeadIn ;
                    LeadIn = NULL ;
                }
            }
		}
		if( LeadIn && LeadOut ) 
		{
			if( nState == 2 )
			{
				CSmartSect * pSect, *pNew ;
				pSect = Curve.m_pHead ;
				PNT3D dStart, dMid, dEnd ;
				pSect->GetPoint(0.0, dStart ) , dStart[2] = 0.0 ;
				pSect->GetPoint(0.5, dMid   ) , dMid[2]   = 0.5 ;
				pSect->GetPoint(1.0, dEnd   ) , dEnd[2]   = 1.0 ;
				pNew = pSect->GetSubSect( dMid, dEnd ) ;
				Curve.InsertAfter( pNew, pSect ) ;
				pNew = pSect->GetSubSect( dStart, dMid ) ;
				Curve.AddSect( pNew ) ;
				Curve.RemoveSect( pSect ) ;
				delete pSect ; 
			}
			break ;
		}
		if( tmpLead.m_nPositionType == NCDEF_LEADPOS_AUTO )
		{ // 自动查找合适的位置
			CSmartSect* pSect = Curve.m_pHead->next ;
			if( pSect ) Curve.SetStartSect( pSect ) ;
			else break ;
			if( pSect == SectHead ) 
			{
				if( nState  == 1 )
				{
					nState = 2 ;
					continue   ;
				}
			}
			else continue  ;
		}
		if( tmpLead.m_nErrTreat == NCDEF_ERRLEAD_ABORT ) 
		{ 
			break; 
		}
		else if( tmpLead.m_nErrTreat == NCDEF_ERRLEAD_SUBDIST )
		{
			 if( tmpLead.m_nLeadType == NCDEF_LEAD_LINE  ||
			     tmpLead.m_nLeadType == NCDEF_LEAD_SLINE   )
			 {
				 tmpLead.m_dLength *= 0.8 ;
				 if( tmpLead.m_dLength < 0.05 ) break;
			 }
			 else 
			 {
				 tmpLead.m_dRadius *= 0.8 ;
				 if( tmpLead.m_dRadius < 0.05 ) break; 
			 }
             nState = 1 ;
		}
		else if( tmpLead.m_nErrTreat == NCDEF_ERRLEAD_LINE )
		{ // 直线替代
			if( tmpLead.m_nLeadType == NCDEF_LEAD_SLINE ) break; 
			tmpLead.m_nLeadType = NCDEF_LEAD_SLINE ; 
		}
	}
	return 1 ; 
}
static BOOL MathCAM_SubLeadDist( JDNC_LEAD& tmpLead )
{
	if( tmpLead.m_nLeadType == NCDEF_LEAD_CLOSE )
	{
		return TRUE ;
	}
	if( tmpLead.m_nErrTreat == NCDEF_ERRLEAD_SUBDIST )
	{
		if( tmpLead.m_nLeadType == NCDEF_LEAD_LINE  ||
			tmpLead.m_nLeadType == NCDEF_LEAD_SLINE   )
		{
			tmpLead.m_dLength *= 0.8 ;
			if( tmpLead.m_dLength > 0.05 ) 
			{
				return TRUE ;
			}
		}
		else if( tmpLead.m_nLeadType == NCDEF_LEAD_ARC || 
			tmpLead.m_nLeadType == NCDEF_LEAD_INARC )
		{
			if( tmpLead.m_nLeadType == NCDEF_LEAD_ARC )
			{
				tmpLead.m_dRadius *= 0.8;
			}
			else
			{
				tmpLead.m_dAngle *= 0.8;
			}
			if( tmpLead.m_dRadius * ANGLE_TO_RADIAN( tmpLead.m_dAngle) > 0.05 )
			{
				return TRUE ;
			}
		}
	}
	else if( tmpLead.m_nErrTreat == NCDEF_ERRLEAD_LINE )
	{ // 直线替代
		if( tmpLead.m_nLeadType == NCDEF_LEAD_SLINE )
		{
			return FALSE ;
		}
		tmpLead.m_nLeadType = NCDEF_LEAD_SLINE ; 
		return TRUE ;
	}
	return FALSE ;
}
BOOL  CSmartPathGen::CreateLeadEntEx(CSmartCurve& OrgCurve ,
									 JDNC_LEAD& LeadDef    ,
                                     CSmartCurve& LeadCv   ,
									 PNT2D  Point          ,
									 VEC2D  Normal         ,
									 int    IsOut          ,
									 CSmartLoop* ContHead  ) 
{
    if( LeadDef.m_nLeadType == NCDEF_LEAD_BYCONTOUR )
    {
        double dLen = OrgCurve.GetLength() ;
        CSmartCurve* pTemp = NULL, *pObjCv = NULL ; 
        CSmartCurve  closeCv ;
        if( OrgCurve.IsClosed() ) 
        {
            PNT2D dStart ;
            OrgCurve.GetStart( dStart ) ;
            if( nc_Distance( dStart, Point, 2 ) > 2.0e-4 ) 
            {
                for( CSmartSect* pSect = OrgCurve.m_pHead ; pSect ; pSect = pSect->next )
                {
                    closeCv.AddSect( pSect->CopyMyself() ) ;
                }
                closeCv.SetStartPoint( NULL,Point  ) ;
            }
        }
        CSmartCurve&  ByCurve = (closeCv.m_pHead ) ? closeCv : OrgCurve;
        if( IsOut == FALSE )
        {/*沿轮廓进刀*/
            if( ByCurve.IsClosed() )
            {
                ByCurve.BreakCurveAt( dLen - LeadDef.m_dLength, pTemp, pObjCv ) ;
            }
            else
            {
                ByCurve.BreakCurveAt( LeadDef.m_dLength, pObjCv, pTemp ) ;
                if( pObjCv ) pObjCv->Reverse() ;
            }
        }
        else
        {/*沿轮廓退刀*/
            if( ByCurve.IsClosed() )
            {
                ByCurve.BreakCurveAt( LeadDef.m_dLength, pObjCv, pTemp ) ;
            }
            else
            {
                ByCurve.BreakCurveAt( dLen - LeadDef.m_dLength, pTemp, pObjCv ) ;
                if( pObjCv ) pObjCv->Reverse() ;
           }
        }
        if( pTemp ) delete pTemp ;
        if( pObjCv == NULL ) return FALSE ;
        LeadCv.AppendCurve( *pObjCv ) ;
        delete pObjCv ;
    }
    else
    {
		BOOL bRet = CreateLeadEntNew( LeadDef, Point , Normal, IsOut, ContHead, LeadCv) ;
		if( !bRet ) return FALSE ; 
		double dDist ;
		PNT2D dEnd, dSnapAt ;
		if( IsOut == TRUE ) LeadCv.GetEnd( dEnd ) ;
		else LeadCv.GetStart( dEnd ) ;
		OrgCurve.MinDistSect( dEnd, dSnapAt, dDist ) ;
		// yul 修改于2012/06/26苏州加工苹果的小孔要内切，其距离最小为0.000005左右
		if( dDist < MIN_LEN ) 
		{
			LeadCv.ClearAll() ;
			return FALSE  ; 
		}

    }
    LeadCv.SetFeedType( JDNC_FEEDTYPE_LEAD ) ;
    return TRUE ;
}
int  CSmartPathGen::CreateLocalLead( CSmartCurve&  Curve    , 
									JDNC_LEAD&   tmpLeadIn ,
									JDNC_LEAD&   tmpLeadOut,
									PNT2D        Point[2]  ,
									VEC2D        Norm[2]   ,
									CSmartCurve&  LeadInCv ,
									CSmartCurve&  LeadOutCv,
									CSmartLoop*   Contour  , 
                                    DOUBLE     DepthAt     ,  /*路径所在高度*/
                                    DOUBLE     ZShift[2]   ,   /*起末点抬高  */
									DOUBLE     DepthInc)

{
    LeadInCv.ClearAll() ;
    LeadOutCv.ClearAll() ;
	CreateLeadEntEx( Curve, tmpLeadIn, LeadInCv, Point[0], Norm[0], 0 , Contour ) ;
	CreateLeadEntEx( Curve, tmpLeadOut,LeadOutCv, Point[1], Norm[1], 1 , Contour ) ;
    if( tmpLeadIn.m_nLeadType  != NCDEF_LEAD_CLOSE && LeadInCv.m_pHead  == NULL  || 
        tmpLeadOut.m_nLeadType != NCDEF_LEAD_CLOSE && LeadOutCv.m_pHead == NULL   )
    {
        LeadInCv.ClearAll() ;
        LeadOutCv.ClearAll() ;
        return FALSE ;
    }

	bool FlagArc = (m_cProcessDef.m_bProcessFlag & NCDEF_SMOOTHLEAD) ? TRUE : FALSE;//20140819 xh
	if( LeadInCv.m_pHead )
	{
		if( tmpLeadIn.m_nLeadType == NCDEF_LEAD_BYCONTOUR )
		{
			if( DepthAt > 0.0 )
			{
				if (FlagArc)
				{
					MathCAM_SmoothLeadINOUT( LeadInCv, DepthAt, DepthAt - DepthInc, 
						m_cSetupDef.m_cTolDef.m_dArcTol, m_cSetupDef.m_cTolDef.m_dAngTol) ;//20140825 xh
				}
				else
				{
					MathCAM_SetZValue( LeadInCv, DepthAt, DepthAt - DepthInc) ;            //20140815 xh
				}
				
			}
		}
		else if( ZShift[0] > 0. )
		{
			if( !ChangeLeadArcHeightBySpline( &LeadInCv, tmpLeadIn, ZShift[0] , FALSE) )
			{
				MathCAM_SetZValue( LeadInCv, ZShift[0], 0.0 ) ;
			}
		}
	} 
	if ( !m_cMethodDef.m_cLoopCut.m_cLeadOut.m_bSameLeadIn )
	{
		FlagArc = (m_cProcessDef.m_bProcessFlag & NCDEF_SMOOTHLEADOUT) ? TRUE : FALSE;//20140819 xh
	}
	if( LeadOutCv.m_pHead )
	{
		if( tmpLeadOut.m_nLeadType == NCDEF_LEAD_BYCONTOUR )
		{
			if( DepthAt > 0.0 )
			{
				if (FlagArc)
				{
					MathCAM_SmoothLeadINOUT( LeadOutCv,DepthAt - DepthInc, DepthAt, 
						m_cSetupDef.m_cTolDef.m_dArcTol, m_cSetupDef.m_cTolDef.m_dAngTol) ;//20140825 xh
				}
				else
				{
					MathCAM_SetZValue( LeadOutCv,DepthAt - DepthInc, DepthAt) ;            //20140815 xh
				}
			}
		}
		else if( ZShift[1] > 0. )
		{
			if( !ChangeLeadArcHeightBySpline( &LeadOutCv, tmpLeadOut, ZShift[1], FALSE ) )
			{
				MathCAM_SetZValue( LeadOutCv, 0.0, ZShift[1] ) ;
			}
		}
	}
    return TRUE ;
}

// 如果切入切出有高度，则将圆弧段的高度转为样条高度
BOOL CSmartPathGen::ChangeLeadArcHeightBySpline(	CSmartCurve* LeadCurve ,// Inn:曲线 
													JDNC_LEAD&   tmpLead   ,
													DOUBLE ZShift		   ,// Inn:高度
													BOOL	bLeadIn ) 		// Inn:切入 )
{
	// 高度ZShift不合格，退出
	if( ZShift < 0.001) return FALSE ;

	// 非圆弧内切或者外切，退出
	if( tmpLead.m_nLeadType != NCDEF_LEAD_ARC && tmpLead.m_nLeadType != NCDEF_LEAD_INARC )
		return FALSE ;

	double dLength = LeadCurve->GetLength() ;
	CSmartSect *pArc = LeadCurve->m_pHead, *pLine = NULL ;
	if( bLeadIn ) pArc = LeadCurve->m_pTail ;

	if( pArc->GetType () != NC_SECT_ARC ) return FALSE ;

	double dLen = 0., dArcLen = pArc->GetLength() ;
	double ZShift0 = dArcLen * ZShift / dLength ;

	PNT3D start, end, mid ;
	VEC3D vec ;
	pArc->GetStart ( start ) ;
	pArc->GetEnd   ( end   ) ;
	if( bLeadIn )
	{
		start[2] = ZShift0, end[2] = 0. ;
		mathGetVecByPP2D( end, start, MIN_LEN, vec ) ;
	}
	else
	{
		start[2] = 0., end[2] = ZShift0 ;
		mathGetVecByPP2D( start, end, MIN_LEN, vec ) ;
	}
	double dDist3D = mathDist( start, end ), dDist2D = mathDist2D( start, end ) ;
	dLen = dDist3D * dDist3D * 0.5 / dDist2D ;
	int i = 0 ;
	if( bLeadIn )
	{
		for( i = 0 ; i < 2 ; i++ )
		{
			mid[i] = end[i] + dLen * vec[i] ;
		}
	}
	else
	{
		for( i = 0 ; i < 2 ; i++ )
		{
			mid[i] = start[i] + dLen * vec[i] ;
		}
	}
	mid[2] = 0. ;

	// 已知三个点，构造样条
	CGeoSpline *pSpline = new CGeoSpline( 2, 2 ) ;
	mathCpyPnt( start, pSpline->m_Pw[0] ) ;
	mathCpyPnt( mid,   pSpline->m_Pw[1] ) ;
	mathCpyPnt( end  , pSpline->m_Pw[2] ) ;
	pSpline->m_Pw[0][3] = pSpline->m_Pw[1][3] = pSpline->m_Pw[2][3] = 1. ;
	pSpline->m_n = 2, pSpline->m_p = 2 ;
	if( nbcurGetKnotVect( 2, 2, pSpline->m_Pw, pSpline->m_U ))
	{
		delete pSpline ;
		return FALSE ;
	}
	pSpline->DelAuxData () ;
	// 离散圆弧成折线段并添加高度
	LeadCurve->RemoveSect ( pArc ) ;
	if( bLeadIn )
		MathCAM_SetZValue( *LeadCurve, ZShift, ZShift0 ) ;
	else
		MathCAM_SetZValue( *LeadCurve, ZShift0, ZShift ) ;
	
	PNT2D buffer[500] ;
	double dArc = m_cSetupDef.m_cTolDef.m_dArcTol * 0.5, dAng = m_cSetupDef.m_cTolDef.m_dAngTol, t[2] = { 0} ;
	
	int nCnt = ( ( CSmartArc *)pArc )->DiscreteEx ( dArc, dAng, buffer, 499 ) ;
	delete pArc ;
	dDist2D = 0., t[0] = 0. ;
	CSmartCurve tmpCurve ;
	for( i = 1 ; i <= nCnt ; i++ )
	{
		start[0] = buffer[i-1][0], start[1] = buffer[i-1][1] ;
		end[0] = buffer[i][0], end[1] = buffer[i][1] ;
		pLine = new CSmartLine( start, end ) ;
		pSpline->GetPoint( t[0], mid ) ;
		pLine->m_dZValue[0] = mid[2] ;

		dDist2D += mathDist2D( buffer[i-1], buffer[i] ) ;
		t[1] = dDist2D / dArcLen ;
		if( i == nCnt ) t[1] = 1. ;
		pSpline->GetPoint( t[1], mid ) ;
		pLine->m_dZValue[1] = mid[2] ; 
		pLine->m_bFeedType = JDNC_FEEDTYPE_LEAD ;

		tmpCurve.AddSect ( pLine ) ;
		t[0] = t[1] ;
	}
/*	CStrpt *pStrip = pSpline->DiscreteToStrpt ( dArc, dAng ) ;
	for( i = 1 ; i < pStrip->m_np ; i++ )
	{
		mathCpyPnt( pStrip->m_ps[i-1], start ) ;
		mathCpyPnt( pStrip->m_ps[i], end ) ;
		pLine = new CSmartLine( start, end ) ;
		pLine->SetZValue( start[2], end[2] ) ;
		pLine->m_bFeedType = JDNC_FEEDTYPE_LEAD ;

		tmpCurve.AddSect ( pLine ) ;
	}
	delete pStrip ;*/
	delete pSpline ;
	
	CSmartSect *pNext = NULL ;
	if( !bLeadIn )
	{
		LeadCurve->Reverse() ;
		tmpCurve.Reverse() ;
	}

	pLine = tmpCurve.GetHead() ;
	while( pLine )
	{
		pNext = pLine->next ;
		LeadCurve->AddSect ( pLine ) ;
		pLine = pNext ;
	}
	tmpCurve.m_pHead = tmpCurve.m_pTail = NULL ;
	tmpCurve.m_nNumSect = 0 ;

	if( !bLeadIn )	LeadCurve->Reverse() ;
	
	return TRUE ;
}
int  CSmartPathGen::CreateBoundLeadEx( JDNC_LEAD&  LeadInDef  ,  /*切入定义*/         
		                               JDNC_LEAD&  LeadOutDef ,  /*切出    */
		                               CSmartCurve&  Curve    ,  /*原始曲线*/
		                               CSmartCurve&  LeadInCv ,  /*导入段  */
		                               CSmartCurve&  LeadOutCv,  /*导出段  */
		                               CSmartLoop*   ContHead ,  /*干涉检查*/
		                               DOUBLE     OverlapLen  ,  /*重叠距离*/
                                       DOUBLE     DepthAt     ,  /*路径所在高度*/
                                       DOUBLE     ZShift[2]   )  /*起末点抬高  */  
{
	// 该函数未使用
    LeadInCv.ClearAll() ;
    LeadOutCv.ClearAll() ;
	if( ! Curve.m_pHead )  return  0 ;
	if( LeadInDef.m_nLeadType == 0 && LeadOutDef.m_nLeadType == 0 )
	{
		return 1 ;
	}
	PNT2D dPoint[2], dNorm[2] ; 
	CSmartSect * SectHead = Curve.GetHead(), *pSect ; 
	int nTry = 0 ; 
	int  bEscape = FALSE , bIsClosed = Curve.IsClosed( 0.0002 ) ;
	JDNC_LEAD tmpLeadIn = LeadInDef ;
	JDNC_LEAD tmpLeadOut = LeadOutDef ;
	double dCurveLen = Curve.GetLength() ;
	if(  bIsClosed && OverlapLen > 0.05 && dCurveLen > 0.1  )
	{/*CASE1 :尝试延长进退刀*/
		if( OverlapLen > dCurveLen - 0.05 ) OverlapLen = dCurveLen - 0.05 ;
		PNT3D dEnd, dStart ;
		DOUBLE dDist ;
		for( nTry = 0 ; nTry < 5 ; nTry ++ ) 
		{
			while( 1 )
			{
				CSmartSect* pEndSect = NULL  ;
				Curve.m_pHead->GetPoint( 0.0, dPoint[0] ) ;
				if( tmpLeadIn.m_nLeadType == NCDEF_LEAD_LINE )
				{
					Curve.m_pHead->GetTangent( 0.0, dNorm[0] ) ;
					nc_VectorReverse( dNorm[0],2 ) ;
				}
				else
				{
					Curve.m_pHead->GetNormal( 0.0, dNorm[0] ) ;
				}
				Curve.GetPointByLength( OverlapLen, dPoint[1]) ;
				pEndSect = Curve.MinDistSect( dPoint[1], dEnd , dDist ) ;
				if( ! pEndSect ) break; 
				dEnd[2] = pEndSect->GetParam( dEnd ) ;
				if( tmpLeadOut.m_nLeadType == NCDEF_LEAD_LINE  )
				{// 直线相切
					pEndSect->GetTangent( dEnd[2], dNorm[1] ) ;
				}
				else
				{
					pEndSect->GetNormal( dEnd[2] , dNorm[1] ) ;
				}
				if( CreateLocalLead( Curve, tmpLeadIn, tmpLeadOut, 
					                 dPoint, dNorm,LeadInCv, LeadOutCv, ContHead,DepthAt,
                                     ZShift) )
				{
					for( pSect = Curve.m_pHead ; pSect ; pSect = pSect->next )
					{
						if( pSect == pEndSect ) break ;
						Curve.AddSect( pSect->CopyMyself() ) ;
					}
					pEndSect->GetStart( dStart ) ;
					dStart[2] = 0.0 ;
					pSect = pEndSect->GetSubSect( dStart, dEnd ) ;
					if( pSect )
					{
						Curve.AddSect( pSect ) ;
					}
					return 1 ;
				}
				if( tmpLeadIn.m_nPositionType == NCDEF_LEADPOS_AUTO && Curve.m_nNumSect>1 )
				{
					pSect = Curve.m_pHead->next ;
					if( pSect ) Curve.SetStartSect( pSect ) ;
					else break ;
					if( pSect == SectHead ) 
					{
						break ;
					}
				}
				else 
				{
					break ;
				}
			}
			if( ! MathCAM_SubLeadDist( tmpLeadIn ) ||
				! MathCAM_SubLeadDist( tmpLeadOut )  )
			{
				break; 
			}
		}
		Curve.SetStartSect( SectHead ) ;
		for( nTry = 0 ; nTry < 5 ; nTry ++ ) 
		{
			while( 1 )
			{
				CSmartSect* pEndSect = NULL ;
				Curve.m_pHead->GetPoint( 0.5, dPoint[0] ) ;
				if( tmpLeadIn.m_nLeadType == NCDEF_LEAD_LINE )
				{
					Curve.m_pHead->GetTangent( 0.5, dNorm[0] ) ;
					nc_VectorReverse( dNorm[0],2 ) ;
				}
				else
				{
					Curve.m_pHead->GetNormal( 0.5, dNorm[0] ) ;
				}
				double dMove = Curve.m_pHead->GetLength() * 0.5 ;
				Curve.GetPointByLength( OverlapLen+dMove, dPoint[1]) ;
				pEndSect = Curve.MinDistSect( dPoint[1], dEnd , dDist ) ;
				if( ! pEndSect ) break; 
				dEnd[2] = pEndSect->GetParam( dEnd ) ;
				if( tmpLeadOut.m_nLeadType == NCDEF_LEAD_LINE  )
				{// 直线相切
					pEndSect->GetTangent( dEnd[2], dNorm[1] ) ;
				}
				else
				{
					pEndSect->GetNormal( dEnd[2] , dNorm[1] ) ;
				}
				if( CreateLocalLead( Curve, tmpLeadIn, tmpLeadOut, 
					                 dPoint, dNorm,LeadInCv, LeadOutCv,  ContHead,
                                     DepthAt, ZShift) )
				{
					Curve.SetStartPoint( Curve.m_pHead , dPoint[0] ) ;
					pEndSect = Curve.MinDistSect( dPoint[1], dEnd , dDist ) ;
					for( pSect = Curve.m_pHead ; pSect ; pSect = pSect->next )
					{
						if( pSect == pEndSect ) break ;
						Curve.AddSect( pSect->CopyMyself() ) ;
					}
					pEndSect->GetStart( dStart ) ;
					dStart[2] = 0.0 ;
					dEnd[2] = pEndSect->GetParam( dEnd ) ;
					pSect = pEndSect->GetSubSect( dStart, dEnd ) ;
					if( pSect ) Curve.AddSect( pSect ) ;
					return 1 ;
				}
				if( tmpLeadIn.m_nPositionType == NCDEF_LEADPOS_AUTO && Curve.m_nNumSect>1 )
				{
					pSect = Curve.m_pHead->next ;
					if( pSect ) Curve.SetStartSect( pSect ) ;
					else break ;
					if( pSect == SectHead ) 
					{
						break ;
					}
				}
				else 
				{
					break ;
				}
			}
			if( ! MathCAM_SubLeadDist( tmpLeadIn ) ||
				! MathCAM_SubLeadDist( tmpLeadOut )  )
			{
				break; 
			}
		}
		Curve.SetStartSect( SectHead ) ;
	}
	bEscape = FALSE ;
	tmpLeadIn = LeadInDef ;
	tmpLeadOut = LeadOutDef ;
	for( nTry = 0 ; nTry < 5 ; nTry ++ ) 
	{/*CASE 2 :在起点处插入进退刀*/
		while( 1 )
		{
			Curve.m_pHead->GetPoint( 0.0, dPoint[0] ) ;
			if( tmpLeadIn.m_nLeadType == NCDEF_LEAD_LINE )
			{
				Curve.m_pHead->GetTangent( 0.0, dNorm[0] ) ;
				nc_VectorReverse( dNorm[0], 2 ) ;
			}
			else
			{
				Curve.m_pHead->GetNormal( 0.0, dNorm[0] ) ;
			}
			Curve.m_pTail->GetPoint( 1.0, dPoint[1] ) ;
			if( tmpLeadOut.m_nLeadType == NCDEF_LEAD_LINE  )
			{// 直线相切
				Curve.m_pTail->GetTangent( 1.0, dNorm[1] ) ;
			}
			else
			{
				Curve.m_pTail->GetNormal( 1.0, dNorm[1] ) ;
			}
			bEscape = FALSE  ;
			if( tmpLeadIn.m_nLeadType == NCDEF_LEAD_ARC   ||
				tmpLeadIn.m_nLeadType == NCDEF_LEAD_INARC ||
				tmpLeadOut.m_nLeadType == NCDEF_LEAD_ARC  ||
				tmpLeadOut.m_nLeadType == NCDEF_LEAD_INARC )
			{
				if( nc_Distance( dPoint[0], dPoint[1],2 ) < 1.0e-2 )
				{
					DOUBLE dOA = dNorm[1][0] * dNorm[0][1] - dNorm[1][1] * dNorm[0][0] ;
					if( dOA > 0.17 )
					{
						bEscape = TRUE ;
					}
				}
			}
			else if( tmpLeadIn.m_nLeadType == NCDEF_LEAD_SLINE || 
				     tmpLeadOut.m_nLeadType == NCDEF_LEAD_SLINE   )
			{ // 直线连接
				if( nc_Distance( dPoint[0], dPoint[1],2 ) < 1.0e-2 )
				{
					VEC2D dNMid ;
					dNMid[0] = dNorm[0][0] + dNorm[1][0] ;
					dNMid[1] = dNorm[0][1] + dNorm[1][1] ;
					nc_Normalize( dNMid, 2 ) ;
					if( tmpLeadIn.m_nLeadType == NCDEF_LEAD_SLINE  )
					{
						nc_VectorCopy( dNorm[0], dNMid, 2 ) ;
					}
					if( tmpLeadOut.m_nLeadType == NCDEF_LEAD_SLINE  )
					{
						nc_VectorCopy( dNorm[1], dNMid, 2 ) ;
					}
				}
			}
			if( bEscape == FALSE && 
				CreateLocalLead( Curve, tmpLeadIn, tmpLeadOut, 
				                 dPoint, dNorm, LeadInCv, LeadOutCv, ContHead,
                                 DepthAt, ZShift) )
			{
				return  TRUE ;
			}
			if( !bIsClosed ) break ;
			if( tmpLeadIn.m_nPositionType == NCDEF_LEADPOS_AUTO && Curve.m_nNumSect>1 )
			{ // 自动查找合适的位置
				pSect = Curve.m_pHead->next ;
				if( pSect ) Curve.SetStartSect( pSect ) ;
				else break ;
				if( pSect == SectHead ) 
				{
					break ;
				}
			}
			else
			{
				break; 
			}
		}
		if( ! MathCAM_SubLeadDist( tmpLeadIn ) || 
			! MathCAM_SubLeadDist( tmpLeadOut )  )
		{
			break ;
		}
	}
	bEscape = FALSE ;
	tmpLeadIn = LeadInDef ;
	tmpLeadOut = LeadOutDef ;
	if( tmpLeadIn.m_nPositionType != NCDEF_LEADPOS_AUTO || 
		bIsClosed == FALSE  )
	{
		return FALSE ;
	}
	for( nTry = 0 ; nTry < 5 ; nTry ++ ) 
	{/*CASE 3 :在中点起点处插入进退刀*/
		while( 1 )
		{
			Curve.m_pHead->GetPoint( 0.5, dPoint[0] ) ;
			if( tmpLeadIn.m_nLeadType == NCDEF_LEAD_LINE )
			{
				Curve.m_pHead->GetTangent( 0.5, dNorm[0] ) ;
				nc_VectorReverse( dNorm[0], 2 ) ;
			}
			else
			{
				Curve.m_pHead->GetNormal( 0.5, dNorm[0] ) ;
			}
			Curve.m_pHead->GetPoint( 0.5, dPoint[1] ) ;
			if( tmpLeadOut.m_nLeadType == NCDEF_LEAD_LINE )
			{
				Curve.m_pHead->GetTangent( 0.5, dNorm[1] ) ;
			}
			else
			{
				Curve.m_pHead->GetNormal( 0.5, dNorm[1] ) ;
			}
			bEscape = FALSE  ;
			if( bEscape == FALSE && 
				CreateLocalLead( Curve, tmpLeadIn, tmpLeadOut, 
				                 dPoint, dNorm, LeadInCv, LeadOutCv, ContHead,
                                 DepthAt, ZShift) )
			{
				pSect = Curve.m_pHead ;
				PNT3D dStart, dMid, dEnd ;
				pSect->GetPoint(0.0, dStart ) , dStart[2] = 0.0 ;
				pSect->GetPoint(0.5, dMid   ) , dMid[2]   = 0.5 ;
				pSect->GetPoint(1.0, dEnd   ) , dEnd[2]   = 1.0 ;
				CSmartSect *pNew = pSect->GetSubSect( dMid, dEnd ) ;
				Curve.InsertAfter( pNew, pSect ) ;
				pNew = pSect->GetSubSect( dStart, dMid ) ;
				Curve.AddSect( pNew ) ;
				Curve.RemoveSect( pSect ) ;
				delete pSect ; 
				return  TRUE ;
			}
			if( tmpLeadIn.m_nPositionType == NCDEF_LEADPOS_AUTO && Curve.m_nNumSect>1 )
			{ // 自动查找合适的位置
				pSect = Curve.m_pHead->next ;
				if( pSect ) Curve.SetStartSect( pSect ) ;
				else break ;
				if( pSect == SectHead ) 
				{
					break ;
				}
			}
			else
			{
				break ;
			}
		}
		if( ! MathCAM_SubLeadDist( tmpLeadIn ) || 
			! MathCAM_SubLeadDist( tmpLeadOut )  )
		{
			break ;
		}
		Curve.SetStartSect( SectHead ) ;
	}
	Curve.SetStartSect( SectHead ) ;
	return 0 ; 
}

int  CSmartPathGen::CreateBoundLeadExNew( JDNC_LEAD&  LeadInDef  ,  /*切入定义*/         
										JDNC_LEAD&  LeadOutDef ,  /*切出    */
										CSmartCurve&  Curve    ,  /*原始曲线*/
										CSmartCurve	& OverlapCv,	/*重复加工段*/
										CSmartCurve&  LeadInCv ,  /*导入段  */
										CSmartCurve&  LeadOutCv,  /*导出段  */
										CSmartLoop*   ContHead ,  /*干涉检查*/
										DOUBLE     OverlapLen  ,  /*重叠距离*/
										DOUBLE     DepthAt     ,  /*路径所在高度*/
										DOUBLE     ZShift[2]   ,  /*起末点抬高  */
										DOUBLE     DepthInc)
{
	LeadInCv.ClearAll() ;
	LeadOutCv.ClearAll() ;
	if( ! Curve.m_pHead )  return  0 ;
	if( LeadInDef.m_nLeadType == 0 && LeadOutDef.m_nLeadType == 0 )
	{
		return 1 ;
	}
	PNT2D dPoint[2], dNorm[2] ; 
	CSmartSect * SectHead = Curve.GetHead(), *pSect ; 
	int nTry = 0 ; 
	int  bEscape = FALSE , bIsClosed = Curve.IsClosed( 0.0002 ) ;
	JDNC_LEAD tmpLeadIn = LeadInDef ;
	JDNC_LEAD tmpLeadOut = LeadOutDef ;
	double dCurveLen = Curve.GetLength() ;
	if(  bIsClosed && OverlapLen > 0.05 && dCurveLen > 0.1  )
	{/*CASE1 :尝试延长进退刀*/
		if( OverlapLen > dCurveLen - 0.05 ) OverlapLen = dCurveLen - 0.05 ;
		PNT3D dEnd, dStart ;
		DOUBLE dDist ;
		for( nTry = 0 ; nTry < 5 ; nTry ++ ) 
		{
			while( 1 )
			{
				CSmartSect* pEndSect = NULL  ;
				Curve.m_pHead->GetPoint( 0.0, dPoint[0] ) ;
				if( tmpLeadIn.m_nLeadType == NCDEF_LEAD_LINE )
				{
					Curve.m_pHead->GetTangent( 0.0, dNorm[0] ) ;
					nc_VectorReverse( dNorm[0],2 ) ;
				}
				else
				{
					Curve.m_pHead->GetNormal( 0.0, dNorm[0] ) ;
				}
				Curve.GetPointByLength( OverlapLen, dPoint[1]) ;
				pEndSect = Curve.MinDistSect( dPoint[1], dEnd , dDist ) ;
				if( ! pEndSect ) break; 
				dEnd[2] = pEndSect->GetParam( dEnd ) ;
				if( tmpLeadOut.m_nLeadType == NCDEF_LEAD_LINE  )
				{// 直线相切
					pEndSect->GetTangent( dEnd[2], dNorm[1] ) ;
				}
				else
				{
					pEndSect->GetNormal( dEnd[2] , dNorm[1] ) ;
				}
				if( CreateLocalLead( Curve, tmpLeadIn, tmpLeadOut, 
					dPoint, dNorm,LeadInCv, LeadOutCv, ContHead,DepthAt,
					ZShift, DepthInc) )//20140825  XH
				{
					for( pSect = Curve.m_pHead ; pSect ; pSect = pSect->next )
					{
						if( pSect == pEndSect ) break ;
						OverlapCv.AddSect( pSect->CopyMyself() ) ;
					}
					pEndSect->GetStart( dStart ) ;
					dStart[2] = 0.0 ;
					pSect = pEndSect->GetSubSect( dStart, dEnd ) ;
					if( pSect )
					{
						OverlapCv.AddSect( pSect ) ;
					}
					return 1 ;
				}
				if( tmpLeadIn.m_nPositionType == NCDEF_LEADPOS_AUTO && Curve.m_nNumSect>1 )
				{
					pSect = Curve.m_pHead->next ;
					if( pSect ) Curve.SetStartSect( pSect ) ;
					else break ;
					if( pSect == SectHead ) 
					{
						break ;
					}
				}
				else 
				{
					break ;
				}
			}
			if( ! MathCAM_SubLeadDist( tmpLeadIn ) ||
				! MathCAM_SubLeadDist( tmpLeadOut )  )
			{
				break; 
			}
		}
		Curve.SetStartSect( SectHead ) ;
		for( nTry = 0 ; nTry < 5 ; nTry ++ ) 
		{
			while( 1 )
			{
				CSmartSect* pEndSect = NULL ;
				Curve.m_pHead->GetPoint( 0.5, dPoint[0] ) ;
				if( tmpLeadIn.m_nLeadType == NCDEF_LEAD_LINE )
				{
					Curve.m_pHead->GetTangent( 0.5, dNorm[0] ) ;
					nc_VectorReverse( dNorm[0],2 ) ;
				}
				else
				{
					Curve.m_pHead->GetNormal( 0.5, dNorm[0] ) ;
				}
				double dMove = Curve.m_pHead->GetLength() * 0.5 ;
				Curve.GetPointByLength( OverlapLen+dMove, dPoint[1]) ;
				pEndSect = Curve.MinDistSect( dPoint[1], dEnd , dDist ) ;
				if( ! pEndSect ) break; 
				dEnd[2] = pEndSect->GetParam( dEnd ) ;
				if( tmpLeadOut.m_nLeadType == NCDEF_LEAD_LINE  )
				{// 直线相切
					pEndSect->GetTangent( dEnd[2], dNorm[1] ) ;
				}
				else
				{
					pEndSect->GetNormal( dEnd[2] , dNorm[1] ) ;
				}
				if( CreateLocalLead( Curve, tmpLeadIn, tmpLeadOut, 
					dPoint, dNorm,LeadInCv, LeadOutCv,  ContHead,
					DepthAt, ZShift) )
				{
					Curve.SetStartPoint( Curve.m_pHead , dPoint[0] ) ;
					pEndSect = Curve.MinDistSect( dPoint[1], dEnd , dDist ) ;
					for( pSect = Curve.m_pHead ; pSect ; pSect = pSect->next )
					{
						if( pSect == pEndSect ) break ;
						OverlapCv.AddSect( pSect->CopyMyself() ) ;
					}
					pEndSect->GetStart( dStart ) ;
					dStart[2] = 0.0 ;
					dEnd[2] = pEndSect->GetParam( dEnd ) ;
					pSect = pEndSect->GetSubSect( dStart, dEnd ) ;
					if( pSect )
					{
						OverlapCv.AddSect( pSect ) ;
					}
					return 1 ;
				}
				if( tmpLeadIn.m_nPositionType == NCDEF_LEADPOS_AUTO && Curve.m_nNumSect>1 )
				{
					pSect = Curve.m_pHead->next ;
					if( pSect ) Curve.SetStartSect( pSect ) ;
					else break ;
					if( pSect == SectHead ) 
					{
						break ;
					}
				}
				else 
				{
					break ;
				}
			}
			if( ! MathCAM_SubLeadDist( tmpLeadIn ) ||
				! MathCAM_SubLeadDist( tmpLeadOut )  )
			{
				break; 
			}
		}
		Curve.SetStartSect( SectHead ) ;
	}
	bEscape = FALSE ;
	tmpLeadIn = LeadInDef ;
	tmpLeadOut = LeadOutDef ;
	for( nTry = 0 ; nTry < 5 ; nTry ++ ) 
	{/*CASE 2 :在起点处插入进退刀*/
		while( 1 )
		{
			Curve.m_pHead->GetPoint( 0.0, dPoint[0] ) ;
			if( tmpLeadIn.m_nLeadType == NCDEF_LEAD_LINE )
			{
				Curve.m_pHead->GetTangent( 0.0, dNorm[0] ) ;
				nc_VectorReverse( dNorm[0], 2 ) ;
			}
			else
			{
				Curve.m_pHead->GetNormal( 0.0, dNorm[0] ) ;
			}
			Curve.m_pTail->GetPoint( 1.0, dPoint[1] ) ;
			if( tmpLeadOut.m_nLeadType == NCDEF_LEAD_LINE  )
			{// 直线相切
				Curve.m_pTail->GetTangent( 1.0, dNorm[1] ) ;
			}
			else
			{
				Curve.m_pTail->GetNormal( 1.0, dNorm[1] ) ;
			}
			bEscape = FALSE  ;
			if( tmpLeadIn.m_nLeadType == NCDEF_LEAD_ARC   ||
				tmpLeadIn.m_nLeadType == NCDEF_LEAD_INARC ||
				tmpLeadOut.m_nLeadType == NCDEF_LEAD_ARC  ||
				tmpLeadOut.m_nLeadType == NCDEF_LEAD_INARC )
			{
				if( nc_Distance( dPoint[0], dPoint[1],2 ) < 1.0e-2 )
				{
					DOUBLE dOA = dNorm[1][0] * dNorm[0][1] - dNorm[1][1] * dNorm[0][0] ;
					if( dOA > 0.17 )
					{
						bEscape = TRUE ;
					}
				}
			}
			else if( tmpLeadIn.m_nLeadType == NCDEF_LEAD_SLINE || 
				tmpLeadOut.m_nLeadType == NCDEF_LEAD_SLINE   )
			{ // 直线连接
				if( nc_Distance( dPoint[0], dPoint[1],2 ) < 1.0e-2 )
				{
					VEC2D dNMid ;
					dNMid[0] = dNorm[0][0] + dNorm[1][0] ;
					dNMid[1] = dNorm[0][1] + dNorm[1][1] ;
					nc_Normalize( dNMid, 2 ) ;
					if( tmpLeadIn.m_nLeadType == NCDEF_LEAD_SLINE  )
					{
						nc_VectorCopy( dNorm[0], dNMid, 2 ) ;
					}
					if( tmpLeadOut.m_nLeadType == NCDEF_LEAD_SLINE  )
					{
						nc_VectorCopy( dNorm[1], dNMid, 2 ) ;
					}
				}
			}
			if( bEscape == FALSE && 
				CreateLocalLead( Curve, tmpLeadIn, tmpLeadOut, 
				dPoint, dNorm, LeadInCv, LeadOutCv, ContHead,
				DepthAt, ZShift, DepthInc) )
			{
				return  TRUE ;
			}
			if( !bIsClosed ) break ;
			if( tmpLeadIn.m_nPositionType == NCDEF_LEADPOS_AUTO && Curve.m_nNumSect>1 )
			{ // 自动查找合适的位置
				pSect = Curve.m_pHead->next ;
				if( pSect ) Curve.SetStartSect( pSect ) ;
				else break ;
				if( pSect == SectHead ) 
				{
					break ;
				}
			}
			else
			{
				break; 
			}
		}
		if( ! MathCAM_SubLeadDist( tmpLeadIn ) || 
			! MathCAM_SubLeadDist( tmpLeadOut )  )
		{
			break ;
		}
	}
	bEscape = FALSE ;
	tmpLeadIn = LeadInDef ;
	tmpLeadOut = LeadOutDef ;
	if( tmpLeadIn.m_nPositionType != NCDEF_LEADPOS_AUTO || 
		bIsClosed == FALSE  )
	{
		return FALSE ;
	}
	for( nTry = 0 ; nTry < 5 ; nTry ++ ) 
	{/*CASE 3 :在中点起点处插入进退刀*/
		while( 1 )
		{
			Curve.m_pHead->GetPoint( 0.5, dPoint[0] ) ;
			if( tmpLeadIn.m_nLeadType == NCDEF_LEAD_LINE )
			{
				Curve.m_pHead->GetTangent( 0.5, dNorm[0] ) ;
				nc_VectorReverse( dNorm[0], 2 ) ;
			}
			else
			{
				Curve.m_pHead->GetNormal( 0.5, dNorm[0] ) ;
			}
			Curve.m_pHead->GetPoint( 0.5, dPoint[1] ) ;
			if( tmpLeadOut.m_nLeadType == NCDEF_LEAD_LINE )
			{
				Curve.m_pHead->GetTangent( 0.5, dNorm[1] ) ;
			}
			else
			{
				Curve.m_pHead->GetNormal( 0.5, dNorm[1] ) ;
			}
			bEscape = FALSE  ;
			if( bEscape == FALSE && 
				CreateLocalLead( Curve, tmpLeadIn, tmpLeadOut, 
				dPoint, dNorm, LeadInCv, LeadOutCv, ContHead,
				DepthAt, ZShift) )
			{
				pSect = Curve.m_pHead ;
				PNT3D dStart, dMid, dEnd ;
				pSect->GetPoint(0.0, dStart ) , dStart[2] = 0.0 ;
				pSect->GetPoint(0.5, dMid   ) , dMid[2]   = 0.5 ;
				pSect->GetPoint(1.0, dEnd   ) , dEnd[2]   = 1.0 ;
				CSmartSect *pNew = pSect->GetSubSect( dMid, dEnd ) ;
				Curve.InsertAfter( pNew, pSect ) ;
				pNew = pSect->GetSubSect( dStart, dMid ) ;
				Curve.AddSect( pNew ) ;
				Curve.RemoveSect( pSect ) ;
				delete pSect ; 
				return  TRUE ;
			}
			if( tmpLeadIn.m_nPositionType == NCDEF_LEADPOS_AUTO && Curve.m_nNumSect>1 )
			{ // 自动查找合适的位置
				pSect = Curve.m_pHead->next ;
				if( pSect ) Curve.SetStartSect( pSect ) ;
				else break ;
				if( pSect == SectHead ) 
				{
					break ;
				}
			}
			else
			{
				break ;
			}
		}
		if( ! MathCAM_SubLeadDist( tmpLeadIn ) || 
			! MathCAM_SubLeadDist( tmpLeadOut )  )
		{
			break ;
		}
		Curve.SetStartSect( SectHead ) ;
	}
	Curve.SetStartSect( SectHead ) ;
	return 0 ; 
}

BOOL CSmartPathGen::ConnectLeadInOutPos (	CPathCombine &PComb	,	// <I> 输入路径
											CSmartLoop *AllCont ,	// <I> 区域
											double	dRComp[2]	,	// <I> 补偿值
											int	nRCompSide		,	// <I> 补偿方向
											double pos[2]	) 		// <I> 进刀点)
{
	// 如果非指定点下刀，返回
	if( m_cFeedDef.m_cLeadDef.m_nPositionType != NCDEF_LEADPOS_REFER )
		return FALSE ;
	//// 如果有侧向分层，退出
	//if( m_cSlayerDef.m_nLayerType != NCDEF_SLAYER_CLOSE )
	//	return FALSE ;
	// 轴向分层多层，退出，2013.9.27，liuxin
	if (m_cFeedDef.m_cLayerDef.m_nLayerCount > 1)
	{
		return FALSE;
	}
	// 如果进刀方式为沿轮廓\圆弧内切\直线相切，返回
	if( m_cFeedDef.m_cLeadDef.m_nLeadType == NCDEF_LEAD_BYCONTOUR ||
		m_cFeedDef.m_cLeadDef.m_nLeadType == NCDEF_LEAD_LINE	||
		m_cFeedDef.m_cLeadDef.m_nLeadType == NCDEF_LEAD_INARC )
		return FALSE ;
	// 如果非直线下刀或者关闭下刀，返回
	if( m_cFeedDef.m_cPlungeDef.m_nPlungeType > NCDEF_PLUNGE_VERT )
		return FALSE ;
	BOOL bLeadIn = FALSE, bLeadStart = TRUE ;
	if( m_cProcessDef.m_bProcessFlag & NCDEF_LEADIN_CONNECTPOS) 
		bLeadIn = TRUE ;
	// 目前只是用LeadIn来控制二者的连接
	if( !bLeadIn ) return FALSE ;
	if( !PComb.m_pHead  || !AllCont ) return FALSE ;

	PNT3D start, end, pnt = { pos[0], pos[1], 0.} ;
	PComb.GetEndPoint( 0, start ) ;
	PComb.GetEndPoint( 1, end ) ;

	CPathLine3D *pLine = NULL ;
	// 将路径转换为AllPath
	//CSmtCPathLib AllPath ;
	//TransfPCombineToCPath( PComb, AllPath, 0., 0., m_cSetupDef.m_cTolDef ) ;
	CPtrList listSmtCurve;
	TransfPCombineToSmartCurve(PComb, listSmtCurve, m_cSetupDef.m_cTolDef);

	if( m_cFeedDef.m_cLeadDef.m_nLeadType == NCDEF_LEAD_SLINE )
	{ // 如果直线连接则判断距离
		double dist = mathDist2D( pnt, start ) ;
		if( dist < m_cFeedDef.m_cLeadDef.m_dLength )
		{
			bLeadStart = FALSE ;
		}
	}
	// 如果为关闭
	if( nRCompSide == 2 )
	{
		pnt[2] = start[2] ;
		if( mathDist( start, pnt ) > 1.0e-4 && bLeadStart && 
			!AllCurveIntLine( listSmtCurve, start, pnt )  )
		{
			pLine = new CPathLine3D( pnt, start ) ;
			pLine->m_bFeedType = JDNC_FEEDTYPE_LEAD ;
			PComb.InsertAfter( pLine, NULL ) ;
		}
		pnt[2] = end[2] ;
		if( mathDist( end  , pnt ) > 1.0e-4 &&
			!AllCurveIntLine( listSmtCurve, end, pnt ) )
		{
			pLine = new CPathLine3D( end  , pnt ) ;
			pLine->m_bFeedType = JDNC_FEEDTYPE_LEAD ;
			PComb.AddEntity ( pLine ) ;
		}
	}
	else
	{
		double dDist = MinDistPntAndContour( AllCont, pnt ) ;

		if( dDist < fabs( dRComp[0] ) || dDist < fabs( dRComp[1]) )
			return FALSE ;

		if( !AllLoopIntLine( AllCont, pnt, start ) && bLeadStart && 
			!AllCurveIntLine( listSmtCurve, pnt, start ))
		{
			pnt[2] = start[2] ;
			if( mathDist( start, pnt ) > 1.0e-4 )
			{
				pLine = new CPathLine3D( pnt, start ) ;
				pLine->m_bFeedType = JDNC_FEEDTYPE_LEAD ;
				PComb.InsertAfter( pLine, NULL ) ;
			}
		}
		if( !AllLoopIntLine( AllCont, pnt, end ) &&
			!AllCurveIntLine( listSmtCurve, pnt, end ) )
		{
			pnt[2] = end[2] ;
			if( mathDist( end  , pnt ) > 1.0e-4 )
			{
				pLine = new CPathLine3D( end  , pnt ) ;
				pLine->m_bFeedType = JDNC_FEEDTYPE_LEAD ;
				PComb.AddEntity( pLine ) ;
			}
		}
	}
	//AllPath.ClearAllPath () ;
	while (!listSmtCurve.IsEmpty())
	{
		CSmartCurve* pCurve = (CSmartCurve*)listSmtCurve.RemoveHead();
		if (pCurve)
		{
			pCurve->ClearAll();
			delete pCurve;
		}
	}
	return TRUE ;
}

BOOL CSmartPathGen::ConnectLeadInOutPos (	CPathCombine &PComb	,	// <I> 输入路径
											CSmartLoop *AllCont ,	// <I> 区域
											double	dRComp[2]	,	// <I> 补偿值
											int		nRCompSide	, 	// <I> 补偿方向 )	
											JDNC_LEADOUT &cLead	, 	// <I> 退刀参数
											BOOL	bCnt[2]		)	// <O> 成功返回TRUE
{
	// 判断数据的有效性
	if( !PComb.m_pHead  || !AllCont ) return FALSE ;
	// 轴向分层多层，退出，2013.9.27，liuxin
	if (m_cFeedDef.m_cLayerDef.m_nLayerCount > 1)
	{
		return FALSE;
	}
	// 如果非指定点下刀，转到LeadOut
	BOOL bLeadIn = FALSE, bLeadStart = TRUE ;
	//CSmtCPathLib AllPath ;
	CPathLine3D *pLine = NULL ;
	BOOL bSame = cLead.m_bSameLeadIn ;
	// 将路径转换为AllPath
	//TransfPCombineToCPath( PComb, AllPath, 0., 0., m_cSetupDef.m_cTolDef ) ;

	CPtrList listSmtCurve;
	TransfPCombineToSmartCurve(PComb, listSmtCurve, m_cSetupDef.m_cTolDef);

	// step 1 : 生成起点连接路径
	if( m_cFeedDef.m_cLeadDef.m_nPositionType != NCDEF_LEADPOS_REFER )
		goto EndLine1 ;
	// 如果进刀方式为沿轮廓\圆弧内切\直线相切，转到LeadOut
	if( m_cFeedDef.m_cLeadDef.m_nLeadType == NCDEF_LEAD_BYCONTOUR ||
		m_cFeedDef.m_cLeadDef.m_nLeadType == NCDEF_LEAD_LINE	||
		m_cFeedDef.m_cLeadDef.m_nLeadType == NCDEF_LEAD_INARC )
		goto EndLine1 ;
	// 如果非直线下刀或者关闭下刀，转到LeadOut
	if( m_cFeedDef.m_cPlungeDef.m_nPlungeType > /*NCDEF_PLUNGE_VERT*/NCDEF_PLUNGE_CONTOUR )
		goto EndLine1 ;
	
	if( m_cProcessDef.m_bProcessFlag & NCDEF_LEADIN_CONNECTPOS) 
		bLeadIn = TRUE ;
	// 目前只是用LeadIn来控制二者的连接
	if( !bLeadIn ) 
		goto EndLine1 ;
	
	PNT3D start, pnt = { m_cFeedDef.m_cLeadDef.m_dLeadPos[0], m_cFeedDef.m_cLeadDef.m_dLeadPos[1], 0.} ;
	PComb.GetEndPoint( 0, start ) ;

	if( m_cFeedDef.m_cLeadDef.m_nLeadType == NCDEF_LEAD_SLINE )
	{ // 如果直线连接则判断距离
		double dist = mathDist2D( pnt, start ) ;
		if( dist < m_cFeedDef.m_cLeadDef.m_dLength )
		{
			bLeadStart = FALSE ;
		}
	}
	// 如果为关闭
	if( nRCompSide == 2 )
	{
		pnt[2] = start[2] ;
		if( mathDist( start, pnt ) > 1.0e-4 && bLeadStart && 
			!AllCurveIntLine( listSmtCurve, start, pnt )  )
		{
			pLine = new CPathLine3D( pnt, start ) ;
			pLine->m_bFeedType = JDNC_FEEDTYPE_LEAD ;
			PComb.InsertAfter( pLine, NULL ) ;
			bCnt[0] = TRUE ;
		}
	}
	else
	{
		double dDist = MinDistPntAndContour( AllCont, pnt ) ;

		if( dDist < fabs( dRComp[0] ) || dDist < fabs( dRComp[1]) )
			goto EndLine1 ;

		if( !AllLoopIntLine( AllCont, pnt, start ) && bLeadStart && 
			!AllCurveIntLine( listSmtCurve, pnt, start ))
		{
			pnt[2] = start[2] ;
			if( mathDist( start, pnt ) > 1.0e-4 )
			{
				pLine = new CPathLine3D( pnt, start ) ;
				pLine->m_bFeedType = JDNC_FEEDTYPE_LEAD ;
				PComb.InsertAfter( pLine, NULL ) ;
				bCnt[0] = TRUE ;
			}
		}
	}

EndLine1:
	// step 2 : 生成末点连接路径
	BOOL bLeadOut = FALSE, bLeadEnd = TRUE ;
	// 如果非指定点下刀，返回
	if( m_cProcessDef.m_nLeadOutPosType != NCDEF_LEADPOS_REFER && !bSame )//&& !bSame yul于2013、09、26
		goto EndLine2 ;
	
	// 如果退刀方式为沿轮廓\圆弧内切\直线相切，转到EndLine2
	if( bSame )
	{
		if( m_cFeedDef.m_cLeadDef.m_nLeadType == NCDEF_LEAD_BYCONTOUR ||
			m_cFeedDef.m_cLeadDef.m_nLeadType == NCDEF_LEAD_LINE	||
			m_cFeedDef.m_cLeadDef.m_nLeadType == NCDEF_LEAD_INARC )
			goto EndLine2 ;
		if( m_cFeedDef.m_cLeadDef.m_nPositionType != NCDEF_LEADPOS_REFER )
			goto EndLine2 ;
	}
	else
	{
		if( cLead.m_nLeadType == NCDEF_LEAD_BYCONTOUR ||
			cLead.m_nLeadType == NCDEF_LEAD_LINE	||
			cLead.m_nLeadType == NCDEF_LEAD_INARC )
			goto EndLine2 ;
		if( m_cProcessDef.m_nLeadOutPosType != NCDEF_LEADPOS_REFER )//&& !bSame yul于2013、09、26
			goto EndLine2 ;
	}
	
	if( m_cProcessDef.m_bProcessFlag & NCDEF_LEADOUT_CONNECTPOS || 
		bSame && m_cProcessDef.m_bProcessFlag & NCDEF_LEADIN_CONNECTPOS ) 
		bLeadOut = TRUE ;
	// 目前只是用LeadIn来控制二者的连接
	if( !bLeadOut ) 
		goto EndLine2 ;

	PNT3D end ;
	if( !bSame ) 
	{
		pnt[0] = m_cProcessDef.m_dLeadOutPos[0] ;
		pnt[1] = m_cProcessDef.m_dLeadOutPos[1] ;
	}
	
	PComb.GetEndPoint( 1, end ) ;

	if( bSame && m_cFeedDef.m_cLeadDef.m_nLeadType == NCDEF_LEAD_SLINE || 
		cLead.m_nLeadType == NCDEF_LEAD_SLINE )
	{ // 如果直线连接则判断距离
		double dist = mathDist2D( pnt, end ) ;
		if( dist < cLead.m_dLength )
		{
			bLeadEnd = FALSE ;
		}
	}

	// 如果为关闭
	if( nRCompSide == 2 )
	{
		pnt[2] = end[2] ;
		if( mathDist( end  , pnt ) > 1.0e-4 && bLeadEnd &&
			!AllCurveIntLine( listSmtCurve, end, pnt ) )
		{
			pLine = new CPathLine3D( end  , pnt ) ;
			pLine->m_bFeedType = JDNC_FEEDTYPE_LEAD ;
			PComb.AddEntity ( pLine ) ;
			bCnt[1] = TRUE ;
		}
	}
	else
	{
		double dDist = MinDistPntAndContour( AllCont, pnt ) ;

		if( dDist < fabs( dRComp[0] ) || dDist < fabs( dRComp[1]) )
			goto EndLine2 ;

		if( !AllLoopIntLine( AllCont, pnt, end ) && bLeadOut && 
			!AllCurveIntLine( listSmtCurve, pnt, end ) )
		{
			pnt[2] = end[2] ;
			if( mathDist( end  , pnt ) > 1.0e-4 )
			{
				pLine = new CPathLine3D( end  , pnt ) ;
				pLine->m_bFeedType = JDNC_FEEDTYPE_LEAD ;
				PComb.AddEntity( pLine ) ;
				bCnt[1] = TRUE ;
			}
		}
	}/**/
EndLine2:
	//AllPath.ClearAllPath () ;
	while (!listSmtCurve.IsEmpty())
	{
		CSmartCurve* pCurve = (CSmartCurve*)listSmtCurve.RemoveHead();
		if (pCurve)
		{
			pCurve->ClearAll();
			delete pCurve;
		}
	}
	return TRUE ;
}

int CSmartPathGen::AddRCompMask( CPathCombine& TComb, 
                                 int           RCompMask,
								 BOOL		   bAutoClose)
{
    if( RCompMask == 0 ) return 0;
    for( CPathEntity* pTEnt = TComb.m_pHead ; pTEnt; pTEnt = pTEnt->next )
    {
        if( pTEnt->next == NULL && bAutoClose)
        {/*末段关闭半径补偿*/ 
            break ;
        }
        pTEnt->m_bMoveFlag |= RCompMask ;
    }
    return 1 ;
}

int CSmartPathGen::Add3DRCompMask( CPathCombine& TComb, 
								   int           RCompMask,
								   BOOL		     bAutoClose)
{
	if( RCompMask == 0 ) return 0;
	for( CPathEntity* pTEnt = TComb.m_pHead ; pTEnt; pTEnt = pTEnt->next )
	{
		if( pTEnt->next == NULL && bAutoClose)
		{/*末段关闭半径补偿*/ 
			break ;
		}
		if (pTEnt->m_bFeedType == JDNC_FEEDTYPE_LEAD ||
			pTEnt->m_bFeedType == JDNC_FEEDTYPE_ROUGH ||
			pTEnt->m_bFeedType == JDNC_FEEDTYPE_RCOMPOPEN)
		{
			pTEnt->m_bMoveFlag |= RCompMask ;
		}
	}
	return 1 ;
}

void CSmartPathGen::RemoveNousedLoop( CSmtLoopArr& AllLoop )
{
    INT_PTR nLoop = AllLoop.GetSize() ;
    for( INT_PTR i = 0 ; i < nLoop ; i ++ )
    {
        CSmartLoop * pLoop = AllLoop[i] ;
		if( (pLoop->m_bUseFlag & NC_LOOP_OUTER) || 
			pLoop->m_pCurve   == NULL  )
		{
            AllLoop.RemoveAt( i ) ;
            nLoop-- , i -- ;
			continue  ;
		}
    }
}
void CSmartPathGen::GetSubLoopGroup( CSmtLoopArr& AllLoop, CSmtLoopArr& SubArr ) 
{
    if( AllLoop.GetSize() == 0 ) return ;
    CSmartLoop* pLoop, *pLastLp ;
    pLastLp = AllLoop[0] ;
    AllLoop.RemoveAt( 0 ) ;
    SubArr.Add( pLastLp ) ;
    while( 1 )
    {
        INT_PTR nLoop = AllLoop.GetSize() ;
        BOOL bFind = FALSE ;
        for( INT_PTR i = 0 ; i < nLoop ; i ++ ) 
        {
            pLoop = AllLoop[i] ;
            if( pLoop->m_dArea > 0.0 && pLastLp->m_dArea < 0.0 ||
                pLoop->m_dArea < 0.0 && pLastLp->m_dArea > 0.0 ||
                pLoop->m_dArea < pLastLp->m_dArea ||
                pLoop->m_nDepth != pLastLp->m_nDepth +1  )
            {
                continue ;
            }
            PNT2D dPoint ;
            if( pLoop->m_dArea < 0.0 )
            {
                pLoop->m_pCurve->GetPoint( 0.5, dPoint ) ;
                if( !pLastLp->IsPointIn( dPoint, 1.0e-4 ) )
                {
                    continue ;
                }
            }
            else
            {
                pLastLp->m_pCurve->GetPoint( 0.5, dPoint ) ;
                if( !pLoop->IsPointIn( dPoint, 1.0e-4 ) )
                {
                    continue ;
                }
            }
            AllLoop.RemoveAt( i ) ;
            SubArr.Add( pLoop ) ;
            pLastLp = pLoop ;
            bFind = TRUE ;
            break ;
        }
        if( ! bFind ) break ;
    }
}
CSmartCurve* CSmartPathGen::SmoothConnectLoops( CSmtLoopArr& LoopArr, DOUBLE CntLen, CSmartLoop& Contour ) 
{
    PNT2D dStart , dEnd, dObj , dNewPt[2] ;
    CSmartLoop * pLoop = NULL ;
    CSmartCurve *pTPath = NULL, *pCurve ;
    CSmartCurve * cntPart[2] , *remPart[2] ;
    CSmartSect* pSnapSect, *pSect ;
    INT_PTR nLoop = LoopArr.GetSize() ;
    if( nLoop < 1 )
    {
        return NULL ;
    }
    pLoop = LoopArr.GetAt( nLoop-1) ;
    pCurve = pLoop->m_pCurve->CopyMyself() ;
    if( GetMillDir() != 0  ) pCurve->Reverse() ;
    pTPath = pCurve->CopyMyself() ;
    BOOL bFinishCnt = FALSE ;
    if( pLoop->m_pCurve->m_pHead->m_bEndFlag & NC_BOUND_SECT )
    {
        bFinishCnt = TRUE ;
    }
    BOOL bAllLoop = FALSE ;
    DOUBLE dLen , dDist, dAtPos ;
    if( pCurve->GetLength() < CntLen - 0.1 )
    {
        bAllLoop = TRUE ;
        cntPart[0] = pCurve ;
    }
    else
    {
        pCurve->BreakCurveAt( CntLen , cntPart[0], remPart[0] ) ;
        delete pCurve ;
        delete remPart[0] ;
    }
    for( INT_PTR i = nLoop-2 ; i >= 0 ; i --  )
    {
        pLoop = LoopArr.GetAt(i ) ;
        pCurve = pLoop->m_pCurve->CopyMyself() ;
        if( i != nLoop -2 ) bFinishCnt = FALSE ;
        if( GetMillDir() != 0  ) pCurve->Reverse() ;
        if( cntPart[0] == NULL ) 
        {
            if( pLoop->m_nDepth != 0 ) pTPath->AddCurve( pCurve ) ;
            continue ;
        }
        cntPart[0]->GetPoint( 0.0 , dStart) ;
        cntPart[0]->GetPoint( 1.0 , dEnd  ) ;
        pSnapSect = pCurve->MinDistSect( dStart, dObj , dDist ) ;
        pCurve->SetStartPoint( pSnapSect, dObj ) ; 
        if( bAllLoop ) 
        {
            cntPart[1] = pCurve->CopyMyself() ;
            remPart[1] = NULL ; 
        }
        else
        {
            pSnapSect= pCurve->MinDistSect( dEnd, dObj, dDist ) ;
            dAtPos = 0.0 ;
            for( pSect = pCurve->m_pHead ; pSect; pSect = pSect->next )
            {
                dDist = pSect->GetLength() ;
                if( pSect == pSnapSect )
                {
                    double t = pSect->GetParam( dObj ) ;
                    dAtPos += ( t * dDist ) ;
                    break ;
                }
                else
                {
                    dAtPos += dDist ;
                }
            }
            pCurve->BreakCurveAt( dAtPos, cntPart[1], remPart[1] ) ;
            pCurve->SetStartPoint( pSnapSect, dObj ) ; 
        }
        cntPart[0]->DiscreteCurve( 0.002 ) ;
        cntPart[0]->m_pHead->GetStart( dNewPt[0] ) ;
        CSmartCurve cntCurve ;
        if( cntPart[1] && cntPart[0] )
        {
            dLen = cntPart[0]->GetLength() ;
            dAtPos = 0.0 ;
            for( pSect = cntPart[0]->m_pHead ; pSect && cntPart[1];pSect = pSect->next )
            {
                if( pSect->next == NULL ) 
                {
                    cntPart[1]->m_pTail->GetEnd( dNewPt[1] ) ;
                }
                else 
                {
                    dAtPos += pSect->GetLength() ;
                    pSect->GetEnd( dStart ) ;
                    cntPart[1]->MinDistSect( dStart, dEnd , dDist ) ;
                    nc_GetPointAtLine( dStart, dEnd , dAtPos / dLen , dNewPt[1], 2 ) ;
                }
                cntCurve.AddSect( new CSmartLine( dNewPt[0], dNewPt[1]) ) ;
                nc_VectorCopy( dNewPt[0], dNewPt[1], 2 ) ;
            }
        }
        for( CSmartSect* sect = cntCurve.m_pHead ; sect ; sect = sect->next )
        {
            if( bFinishCnt ) sect->m_bEndFlag |= NC_BOUND_SECT ;
			if( !Contour.IntSectContour( sect ) )
            {
                PNT2D dMidPt ;
                sect->GetPoint( 0.5, dMidPt ) ;
				if( !Contour.IsPtOnContour( dMidPt ) ) 
				{
                    break ;
                }
            }
        }
        if( cntCurve.m_pHead && sect == NULL )
        {
            pTPath->AppendCurve( cntCurve ) ;
        }
        else
        {
            pTPath->m_pHead->GetStart( dNewPt[0] ) ;
            pCurve->m_pTail->GetStart( dNewPt[1] ) ;
            cntCurve.AddSect( new CSmartLine( dNewPt[0], dNewPt[1]) ) ;
        }
        if( remPart[1]) 
        {
            if( i == 0 ) delete remPart[1] ;
            else pTPath->AddCurve( remPart[1] ) ;
        }
        delete pCurve ;
        delete cntPart[0] ;
        cntPart[0] = cntPart[1] ;
    }
    delete cntPart[0] ;
    pTPath->Reverse() ;
    return pTPath ; 
}

CSmartCurve *CSmartPathGen::ConnectLoopsByLine(	CSmtLoopArr &LoopArr, 
												double CntLen, 
												CSmartLoop &Contour )
{
	UNUSED_ALWAYS( CntLen ) ;
	PNT2D dStart , dEnd, dObj ;
	CSmartLoop * pLoop = NULL ;
	CSmartCurve *pTPath = NULL, *pCurve = NULL ;
	CSmartSect* pSnapSect = NULL ;
	INT_PTR nLoop = LoopArr.GetSize() ;
	if( nLoop < 1 )
	{
		return NULL ;
	}
	pLoop = LoopArr.GetAt( nLoop-1) ;
	pCurve = pLoop->m_pCurve->CopyMyself() ;
	if( GetMillDir() != 0  ) pCurve->Reverse() ;
	pTPath = pCurve ;
	BOOL bFinishCnt = FALSE ;
	if( pLoop->m_pCurve->m_pHead->m_bEndFlag & NC_BOUND_SECT )
	{
		bFinishCnt = TRUE ;
	}
	DOUBLE  dDist = 0  ;
	
	for( INT_PTR i = nLoop-2 ; i >= 0 ; i --  )
	{
		pLoop = LoopArr.GetAt(i ) ;
		pCurve = pLoop->m_pCurve->CopyMyself() ;
		if( i != nLoop -2 ) bFinishCnt = FALSE ;
		if( GetMillDir() != 0  ) pCurve->Reverse() ;
		if( pTPath == NULL ) 
		{
			if( pLoop->m_nDepth != 0 ) pTPath->AddCurve( pCurve ) ;
			continue ;
		}
		pTPath->GetPoint( 0.0 , dStart) ;
		pTPath->GetPoint( 1.0 , dEnd  ) ;
		pSnapSect = pCurve->MinDistSect( dEnd, dObj , dDist ) ;
		pCurve->SetStartPoint( pSnapSect, dObj ) ; 
		
	
		CSmartCurve tmpCurve ;
		tmpCurve.AddSect( new CSmartLine( dEnd, dObj) ) ; 
		CSmartSect *pSect = tmpCurve.m_pHead ;		
		for( pSect ; pSect ; pSect = pSect->next )
		{
			if( bFinishCnt ) pSect->m_bEndFlag |= NC_BOUND_SECT ;
			if( !Contour.IntSectContour( pSect ) )
			{
				PNT2D dMidPt ;
				pSect->GetPoint( 0.5, dMidPt ) ;
				if( !Contour.IsPtOnContour( dMidPt ) ) 
				{
					break ;
				}
			}
		}
		if( tmpCurve.m_pHead && pSect == NULL )
		{
			pTPath->AppendCurve( tmpCurve ) ;
		}
			
		pTPath->AppendCurve ( *pCurve ) ;
		
		delete pCurve ;
	}

	pTPath->Reverse() ;
	return pTPath ; 
}
int CSmartPathGen::CreateECutPath( CPathCombine& PComb,    /*数据*/
							       CSmartLoop& Contour ,    /*轮廓*/
							       JDNC_SLOT&  ECut   ,    /*参数*/
							       JDNC_PLUNGE& Plunge)    /*下刀*/
{
	CSmtLoopArr  AllLoop ;
	Contour.ExtractAllLoop( m_cSetupDef.m_cOrderDef.m_nSortType, AllLoop ) ;
	INT_PTR n = AllLoop.GetSize(), bKeepDown = FALSE ; 
	BOOL bFindAt , bPlungeAt ;
	PNT2D dAt ,  dTo ;
	if( m_cFeedDef.m_cLayerDef.m_bLayerFlag & NCDEF_LAYER_KEEPDOWN )
	{
		bKeepDown = TRUE ;
	}
    // STEP 0 : 偏移轮廓,获得下刀的位置
	CSmartLoop* pChild = NULL ; 
	CSmtLoopArr TmpArr      ;
	if( Plunge.m_nPlungeType == NCDEF_PLUNGE_VERT ||
	    Plunge.m_nPlungeType == NCDEF_PLUNGE_HELIX    || 
		Plunge.m_nPlungeType == NCDEF_PLUNGE_RAMP    )
	{
		DOUBLE dOffset = Plunge.m_dSideTol ; 
		if( Plunge.m_nPlungeType != NCDEF_PLUNGE_VERT ) 
			dOffset = + Plunge.m_dRadius ;
		pChild = Contour.OffsetContour( NCDEF_OFFSET_INNER ,
										dOffset , dOffset  ,
										m_cSetupDef.m_cCorDef ) ;
		for( CSmartLoop* pLoop = pChild ; pLoop ; pLoop = pLoop->next )
		{
			pLoop->ExtractAllLoop( NCDEF_SORTTYPE_TOINNER , TmpArr ) ;
		} 
	}
	INT_PTR nOff = TmpArr.GetSize() ;
	CSmartSect* pSect , *pInner; 
	for( INT_PTR i = 0 ; i < n ; i ++ )
	{
		bFindAt = FALSE ;
		if( ( AllLoop[i]->m_bUseFlag & NC_LOOP_REMAIN ) ||
			(AllLoop[i]->m_bUseFlag & NC_LOOP_OUTER )  )
		{
			continue ;
		}
		for( pSect = AllLoop[i]->GetSectHead() ; pSect && pChild ; pSect = pSect->next )
		{ // 查找合适的下刀位置 , 并调整曲线的起点
			pInner = NULL  ; 
			for( INT_PTR k = 0 ; k < nOff ; k ++ ) 
			{
				for( pInner = TmpArr[k]->GetSectHead() ; pInner ; pInner = pInner->next )
				{
					if( pSect == pInner->parent ) 
					{
						PNT2D  dTmpPnt[2], dMidPt ;
						pSect->GetPoint( 0.0, dTmpPnt[0] ) ;
						pInner->GetPoint(0.0, dTmpPnt[1] ) ;
						CSmartLine tmpLine( dTmpPnt[0], dTmpPnt[1] ) ;
						tmpLine.GetPoint( 0.5, dMidPt ) ;
						if( Contour.IntSectContour( &tmpLine ) == FALSE && 
							Contour.IsPtOnContour( dMidPt ) == 1 ) 
						{
							break  ;
						}
					}
				}
				if( pInner ) break ;
			}
			if(  pInner  )
			{
				AllLoop[i]->m_pCurve->SetStartSect( pSect ) ;
				pInner->GetPoint( 0.0, dAt ) ;
				bFindAt = TRUE ; 
				break ;
			} 
		}
		// 复制曲线
		CSmartCurve* pCurve = AllLoop[i]->m_pCurve->CopyMyself()   ;
	    if( m_cSetupDef.m_cOrderDef.m_nMillDir == 0 ) pCurve->Reverse() ;
		// 添加路径
		for( int k = 1 ; k <= ECut.m_nSlotCount ; k ++ )
		{
			Plunge.m_dIncStep = ECut.m_dSlotDepth[ k ] - ECut.m_dSlotDepth[ k-1 ] ;
			if(  k == 1 ) Plunge.m_dIncStep += Plunge.m_dTopTol ; 
			bPlungeAt = FALSE ; 
			if( bFindAt )
			{
				pCurve->GetPoint( 0.0, dTo ) ;
				CPathCombine  tmpComb( NC_WPROCESS_PLUNGE )  ;
			    bPlungeAt = AddPlungePathAt( tmpComb, Plunge, dTo ,dAt, &Contour ) ; 
				if( bPlungeAt )
				{
					tmpComb.MoveCombine( 0.0, 0.0, ECut.m_dDepthInc-ECut.m_dSlotDepth[k] ) ;
					PComb.AppendCombine( tmpComb ) ; 
				}
			}
			if( bPlungeAt )
			{
				PComb.AddCurve( pCurve, FALSE,  ECut.m_dDepthInc - ECut.m_dSlotDepth[k] ) ;
			}
			else if( Plunge.m_nPlungeType != NCDEF_PLUNGE_CLOSE  )
			{  /* 斜线下刀 */
				DOUBLE dAngle = ANGLE_TO_RADIAN( Plunge.m_dAngle);
                if( Plunge.m_dIncStep > Plunge.m_dMaxZInc  )
                {
                    double dAngDepth = atan2( Plunge.m_dMaxZInc, pCurve->GetLength()) ;
                    if( dAngle > dAngDepth ) dAngle = dAngDepth ;
                }
				if( bKeepDown )
				{// 分层不抬刀
		            CSmartCurve* pNewCurve = NULL  ;
					BOOL bOverLap = (k == ECut.m_nSlotCount) ? TRUE : FALSE ;
					PComb.AddSlantCurve( *pCurve ,dAngle , 
						                 Plunge.m_dIncStep, 
					                     ECut.m_dDepthInc-ECut.m_dSlotDepth[k],
										 bOverLap ,
									     &pNewCurve ) ;
					if( pNewCurve && pNewCurve != pCurve )
					{
						delete pCurve ;
					    pCurve = pNewCurve  ;
						pCurve->LinearCurve( 0.001 ) ;
					} 
				}
				else
				{
					Plunge.m_dIncStep = ECut.m_dSlotDepth[ k ] - ECut.m_dSlotDepth[ k-1 ] ;
					Plunge.m_dIncStep += Plunge.m_dTopTol ; 
					PComb.AddSlantCurve( *pCurve ,dAngle ,
						                 Plunge.m_dIncStep, 
					                     ECut.m_dDepthInc-ECut.m_dSlotDepth[k],
										 TRUE ) ;
				}
			}
			else 
			{  // 不下刀 
				if(   bKeepDown && k != 1 )
				{ // 分层不抬刀
				    PNT3D dStart, dEnd ;
					pCurve->GetPoint( 0.0, dEnd ) ;
					dEnd[2] = ECut.m_dDepthInc - ECut.m_dSlotDepth[k-1] ;
					nc_VectorCopy( dStart, dEnd, 3 ) ;
					dEnd[2] = ECut.m_dDepthInc - ECut.m_dSlotDepth[k] ;
					CPathEntity* pLine = new CPathLine3D( dStart, dEnd ) ;
					pLine->m_bFeedType = JDNC_FEEDTYPE_PLUNGE ; 
					PComb.AddEntity( pLine ) ;
				}
				PComb.AddCurve( pCurve, FALSE,  ECut.m_dDepthInc-ECut.m_dSlotDepth[k] ) ;
			}
		}
		delete pCurve ;
	}
	Mini_DeleteContours( pChild ) ;
	return TRUE ;
}
// 计算行切路径
int  CSmartPathGen::CreateLinearPath( CPathCombine& PComb    ,  /*保存路径*/
								      CSmartLoop& Contour    ,  /*轮廓曲线*/
		                              JDNC_LINEAR&  Linear )  /*行切参数*/ 
{
	DOUBLE dAngle =  ANGLE_TO_RADIAN( Linear.m_dLineAngle ) ;
	Contour.RotateContour( - dAngle ) ;

	// STEP 0 : 偏移轮廓曲线
	CSmartLoop *pObject ;
	Contour.SetContourInfo( 0.0, FALSE ) ;
	DOUBLE dSideTol[2] = { Linear.m_dSideTol, Linear.m_dSideTol} ;
	if( Contour.m_bUseFlag & NC_LOOP_OUTER ) dSideTol[0] = 0.0 ;
	pObject = Contour.OffsetContour( NCDEF_OFFSET_INNER, dSideTol[0], dSideTol[1],
		                         m_cSetupDef.m_cCorDef ) ;
	for( CSmartLoop *pLoop = pObject ; pLoop ; pLoop = pLoop->next )
	{
		pLoop->SetContourInfo( 0.0, TRUE ) ; 
		pLoop->m_pParent = NULL  ;
	}
	// STEP 1 :计算行切交点,提取刀具路径
    CSmartLPoint * pPtHead ; 
	CPathEntity* pTail = PComb.m_pTail  ;
	pPtHead = Contour.GenerateScanPoint( pObject, Linear.m_dOverStep, 20000 ) ;
    if((m_cSetupDef.m_cModelTol.m_nMdlFlag & NCDEF_SETUP_SMOOTHCONNECT) &&
       (Linear.m_bLineFlag & Linear.m_bLineFlag & NCDEF_LINEAR_ZIGZAG)   )
    {
 	    Contour.ExtractSmoothLinearPath( PComb, Linear, m_cSetupDef.m_cOrderDef, pPtHead,pObject) ;
    }
    else
    {
 	    Contour.ExtractLinearPath( PComb, Linear, m_cSetupDef.m_cOrderDef, pPtHead ) ;
    }
    Mini_DeleteContours( pObject ) ;
	DOUBLE dCos = cos( dAngle ) ;
	DOUBLE dSin = sin( dAngle ) ;
	if( ! pTail ) pTail = PComb.m_pHead ;
	else pTail = pTail->next ;
	while( pTail ) 
	{
		pTail->RotateEntity( dCos, dSin ) ;
		pTail = pTail->next ;
	}
	Contour.RotateContour( dAngle ) ;
	return 1 ; 
}

///////////////////////////
// 计算环切路径
int  CSmartPathGen::CreateFollowPath( CPathCombine& PComb  ,  /*保存路径*/
								      CSmartLoop& Contour  ,  /*轮廓曲线*/
		                              JDNC_FOLLOW& Follow  ,  /*环切参数*/ 
									  bool /*bSmooth*/)
{
	CSmartLoop*			pChild = Contour.CopyContour() ;
	BOOL bCreatePath = FALSE ;
	if( ( Follow.m_bFollowFlag & NCDEF_FOLLOW_OPENBND ) &&
		( Contour.m_bUseFlag & NC_LOOP_OUTER ) && Contour.m_pIsland  )
	{ /*最外面的环,使用从外内向加工*/
		CSmartCurveLib CurveLib ;
		bCreatePath = pChild->FollowIsland( Follow, m_cSetupDef, CurveLib) ;
		if( bCreatePath == TRUE )
		{
			for( CSmartCurve* pCurve = CurveLib.m_pHead; pCurve ; pCurve = pCurve->next ) 
			{
				PComb.AddCurve( pCurve, FALSE, 0.0, TRUE  ) ;
			}
		}
	}
	if( bCreatePath == FALSE )
	{
        Follow.m_nFollowType = NCDEF_FOLLOW_INNER ;
	    pChild->FollowContour( Follow, m_cSetupDef.m_cCorDef, 50000 ) ;
        pChild->ExtractFollowPath( PComb, Follow, m_cSetupDef.m_cOrderDef  ) ;
	}
	Mini_DeleteContours( pChild ) ;
	return 1 ;
}


int  CSmartPathGen::CreateFollowPathEx( CSmartLoop&   Contour,  /*轮廓曲线*/
		                             JDNC_FOLLOW&  Follow ,  /*环切参数*/ 
                                    CSmtCPathLib&  AllPath)  /*保存路径*/
{
	// STEP 1 : 生成环切加工路径
	Follow.m_nFollowType = NCDEF_FOLLOW_MIDDLE ;
	CSmartCurveLib AllCurve ;
	CSmartCurve * pCurve ;
	CSmtCPathLib tmpTLib ;
	JDNC_STARTPNT  startAt  ;
	SurfNC_InitPathParam( startAt ) ;
	Contour.CreateFollowCurve( Follow, m_cSetupDef,startAt, AllCurve ) ;
	for( pCurve = AllCurve.m_pHead ; pCurve ; pCurve = pCurve->next )
	{
		CSmtCutPath* pTPath = new CSmtCutPath() ;
		pTPath->AddCurve( *pCurve , m_cSetupDef.m_cTolDef ) ;
		pTPath->m_nLineNo = pCurve->m_nLevelNo ;
		tmpTLib.AddToTail( pTPath ) ;
	}
	AllCurve.DeleteAllCurves() ;
	// STEP 2 : 光滑连接环切加工路径
	CSmtCheckMdl DriveMdl ;
	JDNC_PRGDEF tmpPrg = m_cPrgDef ;
	tmpPrg.m_pPosFunc = NULL  ;
	tmpPrg.m_pPrgFunc = NULL  ;
    BOOL bZigzag = ( Follow.m_bFollowFlag & NCDEF_FOLLOW_ZIGZAG ) ? TRUE : FALSE ;
    JDNC_CONNECT3D tmpCnt3D = m_cFeedDef.m_cConnectDef ;
    if( bZigzag ) tmpCnt3D.m_bConnect3DFlag |= NCDEF_FCONNECT3D_ZIGZAG ;
    else tmpCnt3D.m_bConnect3DFlag &= ~NCDEF_FCONNECT3D_ZIGZAG ;
    tmpCnt3D.m_dSafeDist = Follow.m_dOverStep ;
    SmartNC_RegCreateConnect3DPath( MathCAM_GenZDirLeadPathEx );

	if ( m_cSetupDef.m_cModelTol.m_nMdlFlag & NCDEF_SETUP_SMOOTHCORNER )
	{/*光滑尖角*/
		AllPath.SmoothAllPath ( DriveMdl, m_cSetupDef, m_cPrgDef ) ;
	}
	if (m_cFeedDef.m_cConnectDef.m_bConnect3DFlag & NCDEF_FCONNECT3D_SPIRAL ) 
	{/*螺旋连接*/
		NcBound_ConnectAllPath( DriveMdl, m_cSetupDef, m_cFeedDef,
								Follow.m_dOverStep,tmpTLib, 
                                tmpPrg,m_cFeedDef.m_cConnectDef.m_dMaxDist,
								TFLOAT(25.0*Follow.m_dOverStep),
								bZigzag ) ;
	}
	else
	{/*根据行号连接多段刀具路径*/
		tmpTLib.ConnectPathByLineNo3D( DriveMdl,m_cSetupDef.m_cTolDef, tmpCnt3D,tmpPrg );


	}
	while( tmpTLib.m_cAllPath.GetCount() ) 
	{
		CSmtCutPath* pTPath = tmpTLib.m_cAllPath.RemoveHead() ;
		if( pTPath->GetCutMode() == MINI_CONNECT_PATH )
		{
			pTPath->m_bFeedType = JDNC_FEEDTYPE_CONNECT;
		}
		pTPath->SetCutMode( MINI_MILL_PATH ) ;
		AllPath.AddToTail( pTPath ) ;
	}
	return TRUE ;
}
void MathCAM_DeleteBndSurf( CGeoSurf* Surf )
{
    delete Surf ;
}
BOOL MathCAM_GetSurfPos( CGeoSurf* Surf, DOUBLE UW[2], DOUBLE Pos[3] )
{
    if( Surf->GetType() != OBJ3D_SURFACE_GEO )
    {
        return FALSE ;
    }
    return (( CGeoTrmSurf*) Surf)->GetPoint( UW[0], UW[1], Pos ) ;
}

// 计算螺旋走刀路径
int  CSmartPathGen::CreateSpiralPath( CPathCombine& PComb ,  /*保存路径*/
								      CSmartLoop& Contour     ,  /*轮廓曲线*/
		                              JDNC_SPIRAL&  Spiral  )  /*行切参数*/ 
{
    if( m_pTool )
    {
        CSmartNcSpiral  SpiralGen( m_pTool,Spiral, m_cFeedDef, m_cSetupDef ) ;
        SpiralGen.m_pCreateBndSurf = Nc5D_CreateBndSurf ;
        SpiralGen.m_pDeleteBndSurf = MathCAM_DeleteBndSurf ;
        SpiralGen.m_pGetSurfPos    = MathCAM_GetSurfPos  ;
        return  SpiralGen.CreatePocketPath( Contour,  PComb) ;
    }
	// STEP 0 : 偏移轮廓曲线
    CSmartLoop *pObject = &Contour , *pNextLoop = Contour.next  ;
	Contour.next = NULL ; 
	Contour.SetContourInfo( 0.0, FALSE ) ;
	DOUBLE dSideTol[2] = { Spiral.m_dSideTol, Spiral.m_dSideTol} ;
	if( Contour.m_bUseFlag & NC_LOOP_OUTER ) dSideTol[0] = 0.0 ;
	pObject = Contour.OffsetContour( NCDEF_OFFSET_INNER, dSideTol[0], dSideTol[1],
		                         m_cSetupDef.m_cCorDef ) ;
	for( CSmartLoop *pLoop = pObject ; pLoop ; pLoop = pLoop->next )
	{
		pLoop->SetContourInfo( 0.0, TRUE ) ; 
		pLoop->m_pParent = NULL  ;
	}
	// STEP 1 :计算环绕交点
	CSmartArc   SemiArc[2]  ;
	DOUBLE dCenter[2], dRadius ;
	if( Contour.m_pCurve->GetCenter( dCenter, dRadius ) == 0 )
	{
		dCenter[0] = ( Contour.m_dBox[1][0] - Contour.m_dBox[0][0] ) / 2.0 ;
		dCenter[1] = ( Contour.m_dBox[1][1] - Contour.m_dBox[0][1] ) / 2.0 ;
		dRadius = nc_VectorLen( dCenter, 2 ) + 1.0 ;
		dCenter[0] = ( Contour.m_dBox[0][0] + Contour.m_dBox[1][0] ) / 2.0 ;
		dCenter[1] = ( Contour.m_dBox[0][1] + Contour.m_dBox[1][1] ) / 2.0 ;
	}
	if( !m_cSetupDef.m_cOrderDef.m_nMillDir )
	{ // 逆铣
		SemiArc[0].m_aAngle[0] = 0.0, SemiArc[0].m_aAngle[1] = PI1 ;
		memcpy( SemiArc[0].m_aCenter, dCenter, sizeof(PNT2D) ) ;
		SemiArc[0].m_aCenter[0] += Spiral.m_dOverStep/2.0 ;
		SemiArc[0].m_dRadius = dRadius - Spiral.m_dOverStep/2.0 ;
		SemiArc[1].m_aAngle[0] = PI1 , SemiArc[1].m_aAngle[1] = PI2 ;
		memcpy( SemiArc[1].m_aCenter, dCenter, sizeof(PNT2D) ) ;
		SemiArc[1].m_dRadius = dRadius - Spiral.m_dOverStep;

	}
	else
	{
		SemiArc[0].m_aAngle[0] = PI1, SemiArc[0].m_aAngle[1] = 0 ;
		memcpy( SemiArc[0].m_aCenter, dCenter, sizeof(PNT2D) ) ;
		SemiArc[0].m_aCenter[0] -= Spiral.m_dOverStep/2.0 ;
		SemiArc[0].m_dRadius = dRadius - Spiral.m_dOverStep/2.0;

		SemiArc[1].m_aAngle[0] = PI2 , SemiArc[1].m_aAngle[1] = PI1 ;
		memcpy( SemiArc[1].m_aCenter, dCenter, sizeof(PNT2D) ) ;
		SemiArc[1].m_dRadius = dRadius - Spiral.m_dOverStep;
	}
	// STEP 2 : 提取刀具路径
	CPathCombine TComb( NC_WPROCESS_ROUGH ) ;
	CSmartCurveLib AllCurve ;
    CSmartLPoint * pPtHead  ; 
    pPtHead = Contour.GenerateSpiralPoint( pObject,           /*轮廓曲线*/
		                                   SemiArc,           /*半圆    */
										   Spiral.m_dOverStep,/*路径间距*/
										   AllCurve ) ;       /*所有曲线*/ 
 	Contour.ExtractSpiralPath( TComb    ,  /*路径集合*/
		                       Spiral   ,  /*螺旋定义*/
							   m_cSetupDef.m_cOrderDef,  /*走刀次序*/
		                       pPtHead  ,  /*交点序列*/
							   AllCurve ); /*所有交线*/
    if( pObject != &Contour ) Mini_DeleteContours( pObject ) ;
	Contour.next = pNextLoop ;
	PComb.AppendCombine( TComb ) ;
	return 1 ;
}


CPathEntity* CSmartPathGen::GetLeadEnt( CSmartSect* LeadEnt, DOUBLE ZValue )
{
	if( ! LeadEnt ) return NULL  ;
	PNT3D dStart, dEnd ; 
	CPathEntity *pPathEnt = NULL ; 
	if( LeadEnt->IsSectLine() )
	{
		LeadEnt->GetPoint( 0.0, dStart ) ;
		LeadEnt->GetPoint( 1.0, dEnd ) ;
		dStart[2] = dEnd[2] = ZValue ;
		CPathLine3D* pLine3D = new CPathLine3D( dStart, dEnd ) ;
		pPathEnt = pLine3D ;
	}
	else 
	{ // 圆弧逼近方式
		CPathArc3D *pArc3D = new CPathArc3D() ;
		pArc3D->Create( ZValue, *(CSmartArc*) LeadEnt ) ;
		pPathEnt = pArc3D ; 
	}
	return pPathEnt ;
}

BOOL CSmartPathGen::OffsetBorder(CSmartLoop*& AllLoop)
{	//偏移边界
	if(!AllLoop) return TRUE;
	
	//有边界，则整理边界
	for(CSmartLoop* pLoop = AllLoop ; pLoop ; pLoop = pLoop->next)
		if( pLoop->m_dArea < 0.0 ) pLoop->ReverseLoop() ;

	CSmartLoop	LoopLib;
	AllLoop=LoopLib.BuildContour(AllLoop);		

//	MachMdl.AutoOffsetBorder(m_cSetupDef.m_cCorDef, m_cShapeDef.m_bShapeFlag, AllLoop);
	double	dOffDist=GetSideStock();

	CSmartTool *pTool = CreateSmartTool( m_cToolDef, 0., 0. ) ;

	if( m_cShapeDef.m_bShapeFlag & NCDEF_SHAPE_OUTER )
		dOffDist-=pTool->GetRadiusComp(GetCutDepth(), 0);
	else if( m_cShapeDef.m_bShapeFlag & NCDEF_SHAPE_INNER )
		dOffDist+=pTool->GetRadiusComp(GetCutDepth(), 0);
	else ;
	
	delete pTool ;

	pLoop=AllLoop;
	if(dOffDist>0.0)
	{
		NcOffsetContour(pLoop, NCDEF_OFFSET_INNER, dOffDist,
			dOffDist, m_cSetupDef.m_cCorDef, AllLoop );
	}
	else
	{
		NcOffsetContour(pLoop, NCDEF_OFFSET_OUTER, -dOffDist,
			-dOffDist, m_cSetupDef.m_cCorDef, AllLoop );
	}

	Mini_DeleteContours(pLoop);
	if(!AllLoop) return FALSE;
	return TRUE;
}

BOOL  CSmartPathGen::InitPathGen( JDNC_PARAM& PathDef ) 
{
	m_cShapeDef = PathDef.m_cShapeDef ;
	m_cStockDef = PathDef.m_cStockDef ;
	m_cToolDef  = PathDef.m_cToolDef  ;
	m_cSpeedDef = PathDef.m_cSpeedDef ;
	m_cSetupDef = PathDef.m_cSetupDef ;
	m_cFeedDef  = PathDef.m_cFeedDef  ;
	m_cPathPntDef = PathDef.m_cPathPntDef ;// 五轴参数
	m_cProcessDef = PathDef.m_cProcessDef ;
	m_cFeedDef.m_cLayerDef.m_dLayerDepth = NULL ; 
	m_c5DCtrlDef = PathDef.m_c5DCtrlDef ; // 多轴控制参数
	m_cTransfDef = PathDef.m_cTransfDef ; //路径变换
	m_cSlayerDef = PathDef.m_cSlayerDef ; //侧向分层
	m_dMaxStep   = m_cSetupDef.m_cTolDef.m_dMaxStep ;//保留参数设置中的平坦系数设置，环绕等距消除马赛克中提速用
    if( m_cSetupDef.m_cModelTol.m_nMdlFlag & NCDEF_SETUP_NOMOSAIC )
    {/*消除马赛克*/
        if( !SurfNC_IsSupportSetupMask( m_cMethodDef.m_nMethodType, NCDEF_SETUP_NOMOSAIC ) )
        {
            m_cSetupDef.m_cModelTol.m_nMdlFlag &= ~NCDEF_SETUP_NOMOSAIC ; 
        }
        else 
        {
            m_cSetupDef.m_cTolDef.m_dArcTol = min( m_cSetupDef.m_cTolDef.m_dArcTol , 0.002 ) ;
            m_cSetupDef.m_cTolDef.m_dMaxStep= min( m_cSetupDef.m_cTolDef.m_dMaxStep, 0.15  ) ;
        }
    }
    if( m_cSetupDef.m_cModelTol.m_nMdlFlag & NCDEF_SETUP_EXACTMODE ) 
    {/*精确计算*/
        if( !SurfNC_IsSupportSetupMask( m_cMethodDef.m_nMethodType,NCDEF_SETUP_EXACTMODE ) )
        {
            m_cSetupDef.m_cModelTol.m_nMdlFlag &= ~NCDEF_SETUP_EXACTMODE ; 
        }
        else 
        {
            m_cSetupDef.m_cTolDef.m_dArcTol = min( m_cSetupDef.m_cTolDef.m_dArcTol , 0.002 ) ;
            m_cSetupDef.m_cTolDef.m_dMaxStep= min( m_cSetupDef.m_cTolDef.m_dMaxStep, 0.15  ) ;
        }
    }
    if( m_cSetupDef.m_cModelTol.m_nMdlFlag & NCDEF_SETUP_TOUCHMODE ) 
    {/*刀触点计算*/
        if( !SurfNC_IsSupportSetupMask( m_cMethodDef.m_nMethodType,NCDEF_SETUP_TOUCHMODE ) )
        {
            m_cSetupDef.m_cModelTol.m_nMdlFlag &= ~NCDEF_SETUP_TOUCHMODE ; 
        }
    }
	return TRUE ;
}

void CSmartPathGen::ConvertPGroupToSmtCutPathLib(CPathGroup& PGroup   ,
												 CSmtCPathLib& AllPath, 
												 JDNC_TOL& Tol)
{
	for(CPathCombine* pPComb=PGroup.m_pHead; pPComb; pPComb=pPComb->next)
	{
		AllPath.CreateCutPath( *pPComb, Tol ) ;
	}
}


// 平面曲线转换成刀距路径 
CSmtCutPath* CSmartPathGen::CurveToCPath( CSmartCurve&  Curve ,JDNC_TOL& Tol ) 
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
			int nCnt = pArc->DiscreteEx( Tol.m_dArcTol, ANGLE_TO_RADIAN( Tol.m_dAngTol) , Buff, 499 ) ;
			for( int i = 1 ; i <= nCnt ; i ++ ) 
			{
				dPoint[0] = Buff[i][0], dPoint[1] = Buff[i][1] ;
				pPath->AddPoint( dPoint ) ;
			}
		}
	}
	return pPath  ; 
}

// 提取曲面的参数边界环，并将他们转换成平面轮廓,并变换到加工平面
CSmartLoop* CSmartPathGen::ExtractSurfLoop( CGeoSurf* Surf , 
									    JDNC_TOL& Tol  ,
										RFRAME* /*NcMtx*/  ) 
{
	if( ! Surf ) return NULL ;
	CSmartLoop* pLoop = NULL ;
	CSmartCurve *pCurve ;
	CSmartSect  *pSect ;
	if( Surf->GetType() == OBJ3D_SURFACE_GEO )
	{ // 裁剪曲面
		PNT2D dStart, dEnd ; 
		CGeoTrmSurf *pTSurf = ( CGeoTrmSurf*) Surf ;
		CTrmLoop * entLp = pTSurf->m_pLoop ;
		while( entLp )
		{
			CTrmFin* pBnd = entLp->m_pFinHead ;
			entLp = entLp->GetNext() ;
			pCurve = new CSmartCurve() ;
			for( ; pBnd ; pBnd = pBnd->GetNext() )
			{
				CStrpt*	pStrpt = pBnd->m_pPCurve->DiscreteToStrpt ( Tol.m_dArcTol, ANGLE_TO_RADIAN( Tol.m_dAngTol ) ) ;
				for( int i = 1 ; i < pStrpt->m_np ; i++ )
				{
					dStart[0] = pStrpt->m_ps[i-1][0], dStart[1] = pStrpt->m_ps[i-1][1] ;
					dEnd[0]   = pStrpt->m_ps[i][0],   dEnd[1]   = pStrpt->m_ps[i][1]   ;
					pSect = new CSmartLine( dStart, dEnd ) ;
					pCurve->AddSect( pSect ) ;
				}
				delete pStrpt ;
			}
			// 防止参数域不闭合
			if( pCurve ) pCurve->ConnectAllSect( TRUE ) ;
			if( ! pLoop )
			{
				pLoop = new CSmartLoop() ;
				pLoop->UpdateSect( pCurve ) ;
			}
			else 
			{
				CSmartLoop* pIsland = new CSmartLoop() ;
				pIsland->UpdateSect( pCurve ) ;
				pLoop->AddIsland( pIsland ) ;
			}
		}
	}
	else if( Surf->GetType() == OBJ3D_SURFACE_NET )
	{
		CGeoNetSurf * pNet = ( CGeoNetSurf*) Surf ;
		PT_3D   * Vt = pNet->m_aVertex ;
		if( pNet->m_nNumEdge )
		{ // 存在边界
			for( int edge = 0 ; edge < pNet->m_nNumEdge ; edge ++ )
			{
				pCurve = new CSmartCurve() ;
				int  *n = pNet->m_aEdge[edge] ;
			    for( int i = 0 ; i < pNet->m_aNumVertEdge[edge]-1 ; i ) 
				{
					PNT3D	a={Vt[n[i]][0], Vt[n[i]][1], Vt[n[i]][2]}, 
							b={Vt[n[i+1]][0], Vt[n[i+1]][1], Vt[n[i+1]][2]};
					pSect = new CSmartLine( a, b ) ;
					pCurve->AddSect( pSect ) ;
				}
				if( ! pLoop ) 
				{
					pLoop = new CSmartLoop() ;
			        pLoop->UpdateSect( pCurve ) ;
				}
				else 
				{
					CSmartLoop* pIsland = new CSmartLoop() ;
					pIsland->UpdateSect( pCurve ) ;
					pLoop->AddIsland( pIsland ) ;
				}

			}

		}
	}
	else if( Surf->GetType()== OBJ3D_SURFACE_BITMAP )
	{
		CGeoBmpSurf * pBmp = ( CGeoBmpSurf*) Surf ;
		PNT3D  dBox[2] ;
		dBox[0][0] = dBox[0][1] = dBox[0][2] = 0.0 ;
		dBox[1][0] = pBmp->m_dStepXY[0] * (pBmp->m_nNumXY[0]-1) ;
		dBox[1][1] = pBmp->m_dStepXY[1] * (pBmp->m_nNumXY[1]-1) ;
		dBox[1][2] = 0.0 ;
		pLoop = new CSmartLoop() ;
		pLoop->CreateLoop( dBox[0], dBox[1] ) ;
	}
	else
	{ //非裁剪曲面
		PNT2D  dBox[2] ;
        dBox[0][0] = dBox[0][1] = 0.0 , dBox[1][0] = dBox[1][1] = 1.0 ;
		pLoop = new CSmartLoop() ;
		pLoop->CreateLoop( dBox[0], dBox[1] ) ;
	}
	return pLoop ; 
}

//根据距离将PathEttity分解,放在不同的PathComb中
void CSmartPathGen::SeparatePathComb(CPathGroup& cPathGroup)
{
	PNT3D			pnt1, pnt2;
	CPathEntity		*pPathEnt, *pPathEntPrev;
	CPathCombine	*pPathComb, *pPathCombPrev=NULL, *pNewPathComb;

	for(pPathComb=cPathGroup.m_pHead; pPathComb; pPathCombPrev=pPathComb, pPathComb=pPathComb->next)
	{
		pPathEntPrev=pPathComb->m_pHead;
		if(pPathEntPrev==NULL) continue;
		for(pPathEnt=pPathEntPrev->next; pPathEnt; pPathEntPrev=pPathEnt, pPathEnt=pPathEnt->next)
		{
			pPathEntPrev->GetEndPoint(1, pnt1);
			pPathEnt->GetEndPoint(0, pnt2);
			if( mathDist(pnt1, pnt2)<1e-4 ) continue;//相连则继续搜索
			
			//将pPathComb中pPathEnt之前的PathEnt挪出,放入pNewPathComb中
			pNewPathComb=new CPathCombine(pPathComb->m_nType);
			for(CPathEntity* p=pPathComb->m_pHead; p!=pPathEnt; p=pPathEntPrev)
			{
				pPathEntPrev=p->next; //pPathEntPrev作临时变量使用
				pPathComb->RemoveEntity(p);
				pNewPathComb->AddEntity(p);
			}

			//将pNewPathComb放置在pPathComb链节前
			if(pPathCombPrev) pPathCombPrev->next=pNewPathComb;
			else	cPathGroup.m_pHead=pNewPathComb;
			pNewPathComb->next=pPathComb;
			pPathCombPrev=pNewPathComb;
		}
	}
}

//添加3D加工的下刀路径
void CSmartPathGen::AddAuxPathComb(CPathGroup& /*cPathGroup*/, JDNC_CONNECT3D& /*cPlungeRetract*/)
{
    return  ;
#if 0 

	if(cPlungeRetract.m_nPlungeRetractType==NCDEF_PLUNGE_RETRACT_CLOSE) return;

//	if(m_cSetup.m_cOrder.m_nMillDir==0) m_nLeadPathDir=-1; //逆铣
//	else m_nLeadPathDir=1;//顺铣

	CPathEntity*	pPathEnt;
	CPathEntity*	pPathEnt2;
	CPathCombine	*pPathComb, *pPathCombPrev=NULL, *pNewPathComb;
	TPNT3D			pnt1, pnt2;

	//循环为每一组路径添加下刀路径
	for(pPathComb=cPathGroup.m_pHead; pPathComb; pPathCombPrev=pPathComb, pPathComb=pPathComb->next)
	{
		if(pPathCombPrev!=NULL)
		{
			pPathCombPrev->m_pTail->GetEndPoint(1, pnt1);
			pPathComb->m_pHead->GetEndPoint(0, pnt2);
			if( mathDist(pnt1, pnt2)<1e-4 ) continue;
		}

		//添加下刀路径
		pPathEnt=pPathComb->m_pHead;
		pPathEnt=AddPlungePath(cPlungeRetract, pPathEnt);	
		if(pPathEnt)
		{
			pNewPathComb=new CPathCombine( NC_WPROCESS_PLUNGE );
			pPathEnt2=pPathEnt->next;
			pNewPathComb->AddEntity(pPathEnt);
			if(pPathEnt2) pNewPathComb->AddEntity(pPathEnt2);

			pNewPathComb->next=pPathComb;
			if(pPathCombPrev==NULL) cPathGroup.m_pHead=pNewPathComb;
			else pPathCombPrev->next=pNewPathComb;
		}

		//添加抬刀路径
		pPathEnt=pPathComb->m_pTail;
		pPathEnt=AddRetractPath(cPlungeRetract, pPathEnt);	
		if(pPathEnt)
		{
			pNewPathComb=new CPathCombine( NC_WPROCESS_RETRACT );
			pPathEnt2=pPathEnt->next;
			pNewPathComb->AddEntity(pPathEnt);
			if(pPathEnt2) pNewPathComb->AddEntity(pPathEnt2);

			pNewPathComb->next=pPathComb->next;
			if(pPathComb->next==NULL) cPathGroup.m_pTail=pNewPathComb;
			pPathComb->next=pNewPathComb;

			pPathComb=pNewPathComb; //跳过新添加的抬刀路径
		}
	}
#endif
}

//添加3D曲面加工下刀路径
CPathEntity* CSmartPathGen::AddPlungePath(JDNC_CONNECT3D& /*cPlungeRetract*/, CPathEntity* /*pPathEnt*/)
{
    return NULL ;
#if 0 
	if(pPathEnt==NULL) return NULL;

	PNT3D		pnt, pntStart;
	VEC3D		tangent;
	int			nType=cPlungeRetract.m_nPlungeRetractType;

	pPathEnt->GetEndPoint(0 , pntStart ) ;
	pPathEnt->GetEndTangent(0, tangent ) ;
	if( fabs( fabs(tangent[2])-1)>1e-4 )
	{
		tangent[2]=0;
		mathUniVec(tangent, 1e-8);
	}
	else //起始线在竖直方向，直线进刀
		nType=NCDEF_PLUNGE_RETRACT_CLOSE;

	double dPlungeAngle=cPlungeRetract.m_dPlungeAngle*MiniPai/180;
	double dXYAngle=cPlungeRetract.m_dPlungeXYAngle*MiniPai/180;
	if( nType==NCDEF_PLUNGE_RETRACT_ON && fabs(dPlungeAngle-MiniPai*0.5)>1e-4 )
	{	
		double	t=cPlungeRetract.m_dPlungeLength*cos(dPlungeAngle);
		if(cPlungeRetract.m_nPlungeRelative==NCDEF_RELATIVE_CUTDIR)
			dXYAngle+=mathGetAngleX2D(tangent);

		pnt[0]=pntStart[0]-t*cos(dXYAngle);
		pnt[1]=pntStart[1]-t*sin(dXYAngle);
		pnt[2]=pntStart[2]+cPlungeRetract.m_dPlungeLength*sin(dPlungeAngle);
	}
	else 
	{
		pnt[0]=pntStart[0];
		pnt[1]=pntStart[1];
		pnt[2]=pntStart[2]+cPlungeRetract.m_dPlungeLength;
	}

	return new CPathLine3D(pnt, pntStart);
#endif
}

//添加3D曲面加工抬刀路径
CPathEntity* CSmartPathGen::AddRetractPath(JDNC_CONNECT3D& /*cPlungeRetract*/, CPathEntity* /*pPathEnt*/)
{
    return NULL ;
#if 0 
	if(pPathEnt==NULL) return NULL;

	PNT3D		pnt, pntEnd;
	VEC3D		tangent;
	int			nType=cPlungeRetract.m_nPlungeRetractType;

	pPathEnt->GetEndPoint(1 , pntEnd ) ;
	pPathEnt->GetEndTangent(1, tangent ) ;
	if( fabs( fabs(tangent[2])-1)>1e-4 )
	{
		tangent[2]=0;
		mathUniVec(tangent, 1e-8);
	}
	else //起始线在竖直方向，直线抬刀
		nType=NCDEF_PLUNGE_RETRACT_CLOSE;

	double dRetractAngle=cPlungeRetract.m_dRetractAngle*MiniPai/180;
	double dXYAngle=cPlungeRetract.m_dRetractXYAngle*MiniPai/180;
	if( nType==NCDEF_PLUNGE_RETRACT_ON && fabs(dRetractAngle-MiniPai*0.5)>1e-4 )
	{	
		double	t=cPlungeRetract.m_dRetractLength*cos(dRetractAngle);
		if(cPlungeRetract.m_nRetractRelative==NCDEF_RELATIVE_CUTDIR)
			dXYAngle+=mathGetAngleX2D(tangent);

		pnt[0]=pntEnd[0]+t*cos(dXYAngle);
		pnt[1]=pntEnd[1]+t*sin(dXYAngle);
		pnt[2]=pntEnd[2]+cPlungeRetract.m_dPlungeLength*sin(dRetractAngle);
	}
	else 
	{
		pnt[0]=pntEnd[0];
		pnt[1]=pntEnd[1];
		pnt[2]=pntEnd[2]+cPlungeRetract.m_dPlungeLength;
	}

	return new CPathLine3D(pntEnd, pnt);
#endif
}
CSmartLoop* CSmartPathGen::ResortContour(CSmartLoop* AllLoop )  
{
	CSmartLoop LoopLib ;
	if( ! AllLoop ) return FALSE ;
    CSmartLoop *pLoop ;
	for( pLoop = AllLoop ; pLoop ; pLoop = pLoop->next )
	{
		if( pLoop->m_dArea < 0.0 ) pLoop->ReverseLoop() ;
	}
    AllLoop = LoopLib.BuildContour( AllLoop ) ;
	AllLoop = LoopLib.ResortContour( AllLoop, GetSortType(), TRUE ) ;
	return AllLoop ; 
}
CSmartLoop* CSmartPathGen::OffsetPathBound( CSmartLoop& Contour, 
										    DOUBLE Depth       , 
											DOUBLE SideTol     ,
                                            BOOL SortCont ) 
{
	DOUBLE dRComp[2], dAngle[2]  ;
	CSmartLoop* ContHead = NULL ; 
	JDNC_LAYER* pLayer = GetLayerDef() ;
	dAngle[0] = ANGLE_TO_RADIAN( m_cShapeDef.m_cTaper.m_dBoundAngle  ) ;
	dAngle[1] = ANGLE_TO_RADIAN( m_cShapeDef.m_cTaper.m_dIslandAngle ) ;
	if( m_cShapeDef.m_bAntiFace )
	{ // 底面雕刻效果
		if( !( pLayer->m_bLayerFlag & NCDEF_LAYER_COPYMODE ) )
		{ /* 不是拷贝分层 */ 
			dRComp[0] =  (pLayer->m_dTotalDepth-Depth) * tan( dAngle[0] ) ; 
		    dRComp[1] =  (pLayer->m_dTotalDepth-Depth) * tan( dAngle[1] ) ; 
			if( Contour.m_bUseFlag & NC_LOOP_OUTER ) dRComp[0] = 0.0 ;
			if( dRComp[0] > 1.0e-3 || dRComp[1] > 1.0e-3 )
			{
				ContHead  = Contour.OffsetContour( NCDEF_OFFSET_OUTER , dRComp[0], dRComp[1],GetCorDef() ) ; 
			}
		}
		dRComp[0] = m_pTool->GetRadiusComp(0.0, 0.0 ) + GetSideStock()  ; 
		dRComp[1] = m_pTool->GetRadiusComp(0.0, 0.0 ) + GetSideStock() ; 
		if( Contour.m_bUseFlag & NC_LOOP_OUTER ) dRComp[0] = 0.0 ;
		if( ContHead == NULL ) 
		{
			dRComp[0] += SideTol , dRComp[1] += SideTol ;
			if( Contour.m_bUseFlag & NC_LOOP_OUTER ) dRComp[0] = 0.0 ;
			ContHead  = Contour.OffsetContourEx( dRComp[0]+ SideTol, dRComp[1]+ SideTol,GetCorDef() ) ; 
		}
		else if( fabs( dRComp[0] ) > 1.0e-4 || fabs( dRComp[1] ) > 1.0e-4)
		{
			CSmartLoop* tmpLoop = ContHead ;
			if( dRComp[0] > 0.0 || dRComp[1] > 0.0 )
		       ContHead  = Mini_OffsetContourList( tmpLoop, NCDEF_OFFSET_INNER, dRComp, GetCorDef()) ; 
			else 
			{
				dRComp[0] = - dRComp[0] , dRComp[1] = -dRComp[1] ;
		       ContHead  = Mini_OffsetContourList( tmpLoop, NCDEF_OFFSET_OUTER, dRComp, GetCorDef() ) ; 
			}
			Mini_DeleteContours( tmpLoop )  ;
		} 
	}
	else
	{
		if( pLayer->m_bLayerFlag & NCDEF_LAYER_COPYMODE )
		{ /* 拷贝分层 */ 
			dRComp[0] = m_pTool->GetRadiusComp( pLayer->m_dTotalDepth, dAngle[0] ) + GetSideStock(); 
		    dRComp[1] = m_pTool->GetRadiusComp( pLayer->m_dTotalDepth, dAngle[1] ) + GetSideStock(); 
		}
		else
		{
			dRComp[0] = m_pTool->GetRadiusComp( Depth, dAngle[0] ) + GetSideStock(); 
		    dRComp[1] = m_pTool->GetRadiusComp( Depth, dAngle[1] ) + GetSideStock(); 
		}
		dRComp[0] += SideTol ,  dRComp[1]+= SideTol ;
		if( Contour.m_bUseFlag & NC_LOOP_OUTER ) dRComp[0] = 0.0 ;
        ContHead  = Contour.OffsetContourEx( dRComp[0], dRComp[1],GetCorDef() ) ; 
	}
    if( SortCont && ContHead )
    {
	    ContHead = Contour.ResortContour( ContHead, m_cSetupDef.m_cOrderDef.m_nSortType, FALSE ) ;
    }
	return ContHead ;
}
CSmartLoop* CSmartPathGen::OffsetPathBound( CSmartLoop& Contour, DOUBLE Depth, 
										    CSmartTool* Tool, JDNC_LAYER* Layer,
											DOUBLE dSideStock ) 
{
	DOUBLE dRComp[2], dAngle[2] ;
	if( ! Tool || ! Layer )
	{
		ASSERT( 0 ) ;
		return NULL ; 
	}
	CSmartLoop* ContHead = NULL ; 
	dAngle[0] = ANGLE_TO_RADIAN( m_cShapeDef.m_cTaper.m_dBoundAngle  ) ;
	dAngle[1] = ANGLE_TO_RADIAN( m_cShapeDef.m_cTaper.m_dIslandAngle ) ;
	if( m_cShapeDef.m_bAntiFace )
	{ // 底面雕刻效果
		if( !( Layer->m_bLayerFlag & NCDEF_LAYER_COPYMODE ) )
		{ /* 不是拷贝分层 */ 
			dRComp[0] =  (Layer->m_dTotalDepth-Depth) * tan( dAngle[0] ) ; 
		    dRComp[1] =  (Layer->m_dTotalDepth-Depth) * tan( dAngle[1] ) ; 
			if( dRComp[0] > 1.0e-3 )
			{
				ContHead  = Contour.OffsetContour( NCDEF_OFFSET_OUTER , dRComp[0], dRComp[1],GetCorDef() ) ; 
			}
		}
		dRComp[0] = Tool->GetRadiusComp(0.0, 0.0 ) + dSideStock  ; 
		dRComp[1] = Tool->GetRadiusComp(0.0, 0.0 ) + dSideStock ; 
		if( ContHead == NULL ) 
		{
			ContHead  = Contour.OffsetContourEx(dRComp[0], dRComp[1],GetCorDef() ) ; 
		}
		else if( fabs( dRComp[0] ) > 1.0e-4 )
		{
			CSmartLoop* tmpLoop = ContHead ;
			if( dRComp[0] > 0.0 )
		       ContHead  = Mini_OffsetContourList( tmpLoop, NCDEF_OFFSET_INNER, dRComp, GetCorDef()) ; 
			else 
			{
				dRComp[0] = dRComp[1] = -dRComp[0] ;
		        ContHead  = Mini_OffsetContourList( tmpLoop, NCDEF_OFFSET_OUTER, dRComp, GetCorDef() ) ; 
			}
			Mini_DeleteContours( tmpLoop )  ;
		} 
	}
	else
	{
		if( Layer->m_bLayerFlag & NCDEF_LAYER_COPYMODE )
		{ /* 拷贝分层 */ 
			dRComp[0] = Tool->GetRadiusComp( Layer->m_dTotalDepth, dAngle[0] ) + dSideStock; 
		    dRComp[1] = Tool->GetRadiusComp( Layer->m_dTotalDepth, dAngle[1] ) + dSideStock; 
		}
		else
		{
			dRComp[0] = Tool->GetRadiusComp( Depth, dAngle[0] ) + dSideStock; 
		    dRComp[1] = Tool->GetRadiusComp( Depth, dAngle[1] ) + dSideStock; 
		}
        ContHead  = Contour.OffsetContourEx( dRComp[0], dRComp[1],GetCorDef() ) ; 
	}
	return ContHead ;
}
CSmartLoop* CSmartPathGen::OffsetPathBoundEx( CSmartLoop& Contour, 
											 DOUBLE Depth       , 
											 DOUBLE SideTol     ,
											 BOOL SortCont,
											 int* nRev ) 
{
	DOUBLE dRComp[2], dAngle[2]  ;
	CSmartLoop* ContHead = NULL ; 
	JDNC_LAYER* pLayer = GetLayerDef() ;
	dAngle[0] = ANGLE_TO_RADIAN( m_cShapeDef.m_cTaper.m_dBoundAngle  ) ;
	dAngle[1] = ANGLE_TO_RADIAN( m_cShapeDef.m_cTaper.m_dIslandAngle ) ;
	if( m_cShapeDef.m_bAntiFace )
	{ // 底面雕刻效果
		if( !( pLayer->m_bLayerFlag & NCDEF_LAYER_COPYMODE ) )
		{ /* 不是拷贝分层 */ 
			dRComp[0] =  (pLayer->m_dTotalDepth-Depth) * tan( dAngle[0] ) ; 
			dRComp[1] =  (pLayer->m_dTotalDepth-Depth) * tan( dAngle[1] ) ; 
			if( Contour.m_bUseFlag & NC_LOOP_OUTER ) 
			{
				dRComp[0] = 0.0 ;
				*nRev = 0 ;
			}
			if( dRComp[0] > 1.0e-3 || dRComp[1] > 1.0e-3 )
			{
				if( m_cSetupDef.m_cCorDef.m_nCorType == NCDEF_CORNER_EXTARC &&
					fabs( dRComp[0]) > 1.0e-4 && fabs( dRComp[1]) > 1.0e-4)
				{
					m_cSetupDef.m_cCorDef.m_nCorType = NCDEF_CORNER_EXTUSER ;
					ContHead  = Contour.OffsetContour( NCDEF_OFFSET_OUTER , dRComp[0], dRComp[1],GetCorDef() ) ; 
					m_cSetupDef.m_cCorDef.m_nCorType = NCDEF_CORNER_EXTARC ;
					*nRev = 1 ;
				}
				else
				{
					ContHead  = Contour.OffsetContour( NCDEF_OFFSET_OUTER , dRComp[0], dRComp[1],GetCorDef() ) ; 
				}
			}
		}
		dRComp[0] = m_pTool->GetRadiusComp(0.0, 0.0 ) + GetSideStock()  ; 
		dRComp[1] = m_pTool->GetRadiusComp(0.0, 0.0 ) + GetSideStock() ; 

		if( Contour.m_bUseFlag & NC_LOOP_OUTER ) dRComp[0] = 0.0 ;
		if( ContHead == NULL ) 
		{
			dRComp[0] += SideTol , dRComp[1] += SideTol ;
			if( Contour.m_bUseFlag & NC_LOOP_OUTER )
			{
				dRComp[0] = 0.0 ;
				*nRev = 0 ;
			}

			if( m_cSetupDef.m_cCorDef.m_nCorType == NCDEF_CORNER_EXTARC && fabs(dRComp[0]) > 1.0e-4)
			{
				m_cSetupDef.m_cCorDef.m_nCorType = NCDEF_CORNER_EXTUSER ;
				ContHead  = Contour.OffsetContourEx( dRComp[0]+ SideTol, dRComp[1]+ SideTol,GetCorDef() ) ; 
				m_cSetupDef.m_cCorDef.m_nCorType = NCDEF_CORNER_EXTARC ;
				*nRev = ( dRComp[0]+ SideTol < 0 ) ? 1 : 2 ;
			}
			else
			{
				ContHead  = Contour.OffsetContourEx( dRComp[0]+ SideTol, dRComp[1]+ SideTol,GetCorDef() ) ; 
			}
		}
		else if( fabs( dRComp[0] ) > 1.0e-4 || fabs( dRComp[1] ) > 1.0e-4)
		{
			CSmartLoop* tmpLoop = ContHead ;
			if( dRComp[0] > 0.0 || dRComp[1] > 0.0 )
			{
				if( m_cSetupDef.m_cCorDef.m_nCorType == NCDEF_CORNER_EXTARC && fabs(dRComp[0]) > 1.0e-4)
				{
					m_cSetupDef.m_cCorDef.m_nCorType = NCDEF_CORNER_EXTUSER ;
					ContHead  = Mini_OffsetContourList( tmpLoop, NCDEF_OFFSET_INNER, dRComp, GetCorDef()) ; 
					m_cSetupDef.m_cCorDef.m_nCorType = NCDEF_CORNER_EXTARC ;
					*nRev = ( dRComp[0] < 0 ) ? 1 : 2 ;
				}
				else
				{
					ContHead  = Mini_OffsetContourList( tmpLoop, NCDEF_OFFSET_INNER, dRComp, GetCorDef()) ; 
				}
			}
			else 
			{
				dRComp[0] = - dRComp[0] , dRComp[1] = -dRComp[1] ;

				if( m_cSetupDef.m_cCorDef.m_nCorType == NCDEF_CORNER_EXTARC && fabs(dRComp[0]) > 1.0e-4)
				{
					m_cSetupDef.m_cCorDef.m_nCorType = NCDEF_CORNER_EXTUSER ;
					ContHead  = Mini_OffsetContourList( tmpLoop, NCDEF_OFFSET_OUTER, dRComp, GetCorDef() ) ;
					m_cSetupDef.m_cCorDef.m_nCorType = NCDEF_CORNER_EXTARC ;
				}
				else
				{
					ContHead  = Mini_OffsetContourList( tmpLoop, NCDEF_OFFSET_OUTER, dRComp, GetCorDef() ) ;
				}				
				//ContHead  = Mini_OffsetContourList( tmpLoop, NCDEF_OFFSET_OUTER, dRComp, GetCorDef() ) ; }
			}
			Mini_DeleteContours( tmpLoop )  ;
		} 
	}
	else
	{
		if( pLayer->m_bLayerFlag & NCDEF_LAYER_COPYMODE )
		{ /* 拷贝分层 */ 
			dRComp[0] = m_pTool->GetRadiusComp( pLayer->m_dTotalDepth, dAngle[0] ) + GetSideStock(); 
			dRComp[1] = m_pTool->GetRadiusComp( pLayer->m_dTotalDepth, dAngle[1] ) + GetSideStock(); 
		}
		else
		{
			dRComp[0] = m_pTool->GetRadiusComp( Depth, dAngle[0] ) + GetSideStock(); 
			dRComp[1] = m_pTool->GetRadiusComp( Depth, dAngle[1] ) + GetSideStock(); 
		}
		dRComp[0] += SideTol ,  dRComp[1]+= SideTol ;
		*nRev = ( dRComp[0] < 0 ) ? 1 : 2 ;
		if( Contour.m_bUseFlag & NC_LOOP_OUTER ) dRComp[0] = 0.0 ;

		if( m_cSetupDef.m_cCorDef.m_nCorType == NCDEF_CORNER_EXTARC )
		{
			m_cSetupDef.m_cCorDef.m_nCorType = NCDEF_CORNER_EXTUSER ;
			ContHead  = Contour.OffsetContourEx( dRComp[0], dRComp[1],GetCorDef() ) ;
			m_cSetupDef.m_cCorDef.m_nCorType = NCDEF_CORNER_EXTARC ;
		}
		else
		{
			ContHead  = Contour.OffsetContourEx( dRComp[0], dRComp[1],GetCorDef() ) ;
		}	

		// ContHead  = Contour.OffsetContourEx( dRComp[0], dRComp[1],GetCorDef() ) ; 
	}
	if( SortCont && ContHead )
	{
		ContHead = Contour.ResortContour( ContHead, m_cSetupDef.m_cOrderDef.m_nSortType, FALSE ) ;
	}
	return ContHead ;
}

DOUBLE CSmartPathGen::GetMaxRadius ( PNT2D Min , PNT2D Max, PNT2D Center )
{
	PNT2D p1, p2 ;
	p1[0] = Min[0], p1[1] = Max[1] ;
	p2[0] = Max[0], p2[1] = Min[1] ;

	DOUBLE dRadius , r  ;
	dRadius = mathDist2D( Center, Min ) ;
	r = mathDist2D( Center, Max ) ;
	if( r > dRadius ) dRadius = r ;
	r = mathDist2D( Center, p1 )  ;
	if( r > dRadius ) dRadius = r ;
	r = mathDist2D( Center, p2 ) ;
	if( r > dRadius ) dRadius = r ;
	return dRadius ;
}
CSmartLoop* CSmartPathGen::DefineAntifaceContours( CSmartLoop* LpHead )
{
	double dTotalDepth = GetCutDepth()  ;
	if( dTotalDepth < 1.0e-3 ) return LpHead ;
	double dAngle = m_cToolDef.m_dSideAng / 2.0   ;
	if( dAngle < m_cShapeDef.m_cTaper.m_dBoundAngle ) 
		dAngle = m_cShapeDef.m_cTaper.m_dBoundAngle ;
	if( dAngle < 1.0e-3 )  return LpHead ;
	dAngle = ANGLE_TO_RADIAN( dAngle ) ;
	double dRComp[2] ;
	dRComp[0] = dRComp[1] = tan( dAngle ) * dTotalDepth ;
	if( dRComp[0] < 1.0e-3 ) return LpHead ;
	int nID = 1 ;
	CSmartLoop* pLpHead = NULL , *pContour, *pLpNew ; 
    for( pContour = LpHead ; pContour ; pContour = pContour->next )
    {
		pLpNew = pContour->OffsetContour( NCDEF_OFFSET_OUTER, dRComp[0],dRComp[1], GetCorDef() ) ;
		pLpHead = Mini_AddContours( pLpHead, pLpNew ) ;
		for( ; pLpNew ; pLpNew = pLpNew->next ) pLpNew->m_nGroupId = nID ;
		nID ++ ;
    }
	Mini_DeleteContours( LpHead ) ;
	return pLpHead ;
}
BOOL CSmartPathGen::GeneratePathEx( CPathGroup& NewPath   , 
								    CSmartGraphic& Graph  )
{
	UNUSED_ALWAYS(  NewPath ) ;
	UNUSED_ALWAYS(  Graph   ) ;
	return FALSE ;
}
BOOL MathCAM_IsPntOnFacet ( PNT3D pt[4], int n, PNT3D p )
{
	PNT3D start, end, nearpt ;
	double fIntPt[10], t ;
	int nCnt, i, j ;
	// step 0 : 判断与边的距离
	for( i = 0 ; i < n ; i++ )
	{
		if( i == 0 ) mathCpyPnt( pt[n-1], start ) ;
		else		 mathCpyPnt( pt[i-1], start ) ;
		mathCpyPnt( pt[i], end ) ;
        if( mathGetPntSegmNDT(p, start, end, nearpt) < 2.0e-4 )
        {
            return TRUE ;
        }
    }
	// 判断x向和y向同多边形有两个交点
	// step 1 : 首先判断Y向同三角片有两个交点
	nCnt = 0 ;
	for( i = 0 ; i < n ; i++ )
	{
		if( i == 0 ) mathCpyPnt( pt[n-1], start ) ;
		else		 mathCpyPnt( pt[i-1], start ) ;
		mathCpyPnt( pt[i], end ) ;
		if( start[0] > p[0] && end[0] > p[0] ||
			start[0] < p[0] && end[0] < p[0]  ) 
			continue ;

		t = end[0] - start[0] ;
		if( fabs( t ) < 1.0e-6 ) continue ;

		t = ( p[0] - start[0] ) / t ;
		fIntPt[nCnt] = start[1] + t * ( end[1] - start[1] ) ;
		// 过滤重点
		for( j = 0 ; j < nCnt ; j++ )
		{
			if( fabs( fIntPt[j] - fIntPt[nCnt] ) < 1.0e-6 )
				break ;
		}
		if( j == nCnt ) nCnt++ ;

	}
	if( nCnt == 2 )
	{
		t = ( p[1] - fIntPt[0] ) / ( fIntPt[1] - fIntPt[0] ) ;
		if( t >= 0  && t <= 1 ) return TRUE ;
	}
	// step 2 : 判断X向同三角片有两个交点
	nCnt = 0 ;
	for( i = 0 ;i < n ; i++ )
	{
		if( i == 0 ) mathCpyPnt( pt[n-1], start ) ;
		else		 mathCpyPnt( pt[i-1], start ) ;
		mathCpyPnt( pt[i], end ) ;

		if( start[1] > p[1] && end[1] > p[1] ||
			start[1] < p[1] && end[1] < p[1]  )
			continue ;

		t = end[1] - start[1] ;
		if( fabs( t ) < 1.0e-6 ) continue ;

		t = ( p[1] - start[1] ) / t ;
		fIntPt[nCnt] = start[0] + t * ( end[0] - start[0] ) ;
		// 过滤重点
		for( j = 0 ; j < nCnt ; j++ )
		{
			if( fabs( fIntPt[j] - fIntPt[nCnt] ) < 1.0e-6 )
				break ;
		}
		if( j == nCnt ) nCnt++ ;
	}
	if( nCnt == 2 )
	{
		t = ( p[0] - fIntPt[0] ) / ( fIntPt[1] - fIntPt[0] ) ;
		if( t >= 0 && t <= 1. ) return TRUE ;
	}
	return FALSE ;
}

BOOL MathCAM_CalcDistToFacet ( PNT3D Center, PNT3D p[4], int n, DOUBLE /*XSize*/[2], DOUBLE Radius[2] )
{
	// 面片方向
	VEC3D v[2], nor, vec = { 1., 0., 0.  } ;
	if( n == 3 )
	{
		if( mathGetVecUnit( p[0], p[1], v[0] ) == ERUNSUC ) return FALSE ;
		if( mathGetVecUnit( p[1], p[2], v[1] ) == ERUNSUC ) return FALSE ;
	}
	else
	{
		if( mathGetVecUnit( p[0], p[2], v[0] ) == ERUNSUC ) return FALSE ;
		if( mathGetVecUnit( p[1], p[3], v[1] ) == ERUNSUC ) return FALSE ;
	}
	// 片面退化
	if( mathVProductUnit( v[0], v[1], nor ) == ERUNSUC )
		return FALSE ;
	int i = 0 ;
	PNT3D prj, pt, start, end, intpt[2] ;
	double t = 0., dist = 0., delta = 10. ;
	
	t = nor[1] * ( p[0][1] - Center[1] ) +
        nor[2] * ( p[0][2] - Center[2] ) ;
	
	delta = fabs( vec[0]*nor[0]+vec[1]*nor[1] + vec[2]*nor[1] ) ;
	if( delta < MIN_ANG )
	{ // 线面平行
		if( fabs( t ) > MIN_LEN )
		{ // 线不在面上
			mathPrjPntPln( Center, p[0], nor, pt ) ;
		}
		else
		{ // 线在面上
			pt[0] = Center[0], pt[1] = Center[1], pt[2] = Center[2] ;
		}
		// 计算各点到直线的距离
		for( i = 0 ; i < n ; i++ )
		{
			mathPrjPntLin( p[i], pt, vec, prj ) ;
			if( MathCAM_IsPntOnFacet( p, n, prj ) )
				dist = 0. ;
			else
                dist = mathDist( prj, p[i] ) ;
			if( fabs( t ) > MIN_LEN )
			{
				dist = sqrt( dist * dist + fabs(t)*fabs(t) ) ;
			}
			if( dist < Radius[0] ) Radius[0] = dist ;
		}
	}
	else
	{
		// 计算线段到射线的最短距离
		for( i = 0 ; i < n ; i++ )
		{
			if( i == 0 ) mathCpyPnt( p[n-1], start ) ;
			else		 mathCpyPnt( p[i-1], start ) ;
			mathCpyPnt( p[i], end ) ;
			mathGetVecUnit( start, end, v[0] ) ;
			mathIntLin( start, v[0], Center, vec, MIN_LEN, MIN_ANG, intpt[0], intpt[1] ) ;

			t = GetLineParam3D( start, end, intpt[0] ) ;
			if( t > 1. ) 
			{
				mathCpyPnt( end, intpt[0] ) ;
			}
			else if( t < 0. )
			{
				mathCpyPnt( start, intpt[0] ) ;
			}
			dist = mathDistPntLin( intpt[0], Center, vec ) ;
			if( dist < Radius[0] ) Radius[0] = dist ;
		}
	}
	return TRUE ;
}

BOOL MathCAM_CalcDriveMdlRotateSize( CSmtCheckMdl& DriveMdl ,
									 PNT3D  Center, 
									 DOUBLE XSize[2], 
									 DOUBLE Radius[2] )
{
    INT_PTR nSize = DriveMdl.m_cAllChk.GetSize() ;
	double dRadius ; 
	int i = 0, j = 0, k = 0 ;
	PNT3D p[4] ;

    for( INT_PTR nSurf = 0 ; nSurf < nSize ; nSurf ++ ) 
    {
        CSmartCheck* pSurf = DriveMdl.m_cAllChk.GetAt( nSurf ) ;
        for( i = 0 ; i < pSurf->m_nNumVert ; i ++ ) 
	    {
            dRadius = ( Center[1]-pSurf->m_aVertex[i][1] ) * ( Center[1]-pSurf->m_aVertex[i][1] ) + 
				      ( Center[2]-pSurf->m_aVertex[i][2] ) * ( Center[2]-pSurf->m_aVertex[i][2] ) ;  
		    dRadius = sqrt( dRadius ) ;
		    if( dRadius > Radius[1] ) Radius[1] = dRadius ;
		    if( dRadius < Radius[0] ) Radius[0] = dRadius ;
		    if( pSurf->m_aVertex[i][0] > XSize[1] ) XSize[1] = pSurf->m_aVertex[i][0] ;
		    if( pSurf->m_aVertex[i][0] < XSize[0] ) XSize[0] = pSurf->m_aVertex[i][0] ;
	    }
        int* id , nNumVt ;
        for( i = 0 ; i < pSurf->m_nNumTria ; i++ )
	    {
            id = pSurf->m_aAllTria[i].m_nID ;
		    for( j = 0 ; j < 3 ; j++ )
		    {
			    p[0][j] = pSurf->m_aVertex[id[0]][j] ;
			    p[1][j] = pSurf->m_aVertex[id[1]][j] ;
			    p[2][j] = pSurf->m_aVertex[id[2]][j] ;
		    }
		    MathCAM_CalcDistToFacet( Center, p, 3, XSize, Radius ) ;
	    }
        for( i = 0 ; i < pSurf->m_nNumQuad; i++ )
	    {
            id = pSurf->m_aAllQuad[i].m_nID ;
		    for( j = 0 ; j < 3 ; j++ )
		    {
			    p[0][j] = pSurf->m_aVertex[id[0]][j] ;
			    p[1][j] = pSurf->m_aVertex[id[1]][j] ;
			    p[2][j] = pSurf->m_aVertex[id[2]][j] ;
			    p[3][j] = pSurf->m_aVertex[id[3]][j] ;
		    }
		    MathCAM_CalcDistToFacet( Center, p, 4, XSize, Radius ) ;
	    }
        for( i = 0 ; i < pSurf->m_nNumMix; i++ )
	    {
            id = pSurf->m_aAllMix[i].m_nID ;
            nNumVt = (id[3] == -1) ? 3 : 4 ;
		    for( j = 0 ; j < 3 ; j++ )
            for( k = 0 ; k < nNumVt ; k ++ ) 
		    {
			    p[k][j] = pSurf->m_aVertex[id[k]][j] ;
		    }
		    MathCAM_CalcDistToFacet( Center, p, nNumVt, XSize, Radius ) ;
        }
    }
	return TRUE ;
}

// 根据加工模型自动校正深度范围
BOOL CSmartPathGen::AutoAdjustCutDepth(CSmtCheckMdl& DriveMdl)
{
    if( !( m_cShapeDef.m_bShapeFlag & NCDEF_SHAPE_AUTODEPTH ) )
    {
        return FALSE ;
    }
    if( !SurfNC_IsMethodNeedSurfaces( m_cMethodDef.m_nMethodType, 0 ) )
    {
        return FALSE ;
    }
    DOUBLE dZTop = 0.0 , dZBtm = 0.0;
    double dZMove = GetDriveSurfZMove() ;
	if( m_cMethodDef.m_nMethodType == surfncMethodRotate3D || 
		m_cMethodDef.m_nMethodType == surfncMethodRotate45D ) 
	{
		double dCenter[3], dXSize[2], dRadius[2] ;
		dCenter[0] = dCenter[1] = dCenter[2] = 0.0 ;
        // 初始化dXSize与dRadius
        dXSize[0] = MAX_DBL, dXSize[1] = -MAX_DBL ;
        dRadius[0] = MAX_DBL, dRadius[1] = -MAX_DBL ;
		if( !MathCAM_CalcDriveMdlRotateSize( DriveMdl, dCenter, dXSize, dRadius )  )
		{
            return FALSE ;
        }
		if( dRadius[0] < 0.0 ) dRadius[0] = 0.0 ;
        dZBtm = dRadius[0] , dZTop = dRadius[1] ;
	}
    else 
    {
        FPNT3D fBox3D[2] ;
        if( ! DriveMdl.CalcFacetBox( fBox3D[0], fBox3D[1] ) )
        {
            return FALSE ;
        }
        dZTop = fBox3D[1][2], dZBtm = fBox3D[0][2]  ;
    }
    // 修正计算误差
    dZTop +=  4.0e-4,   dZBtm -=  4.0e-4 ;
    if( dZMove > 0.0 ) dZTop += dZMove ;
    else if( dZMove < 0.0 ) dZBtm += dZMove ;
    m_dTopHeight = dZTop ; 
    m_dCutDepth = dZTop - dZBtm ;
    return TRUE ;
}
BOOL MathCAM_FitBySpline( CPathCombine& AllPath, JDNC_TOL& Tol, PNT3D Point[], int NumPnt ) 
{
	CPathSplFit cPathSplFit;
	return cPathSplFit.FitBySpline( AllPath, Tol, Point, NumPnt ) ;
}
BOOL CSmartPathGen::GeneratePath( CPathGroup& NewPath   , 
							      CSmartGraphic& Graph  )
{
	MathCAM_ClearErrorZLayer() ;
	if( m_cSetupDef.m_cModelTol.m_nMdlFlag & NCDEF_SETUP_SMOOTHCONNECT )
	{
		m_cFeedDef.m_cConnectDef.m_bConnect3DFlag |= NCDEF_FCONNECT3D_SMOOTH ;
	}
	else
	{
		m_cFeedDef.m_cConnectDef.m_bConnect3DFlag &= ~NCDEF_FCONNECT3D_SMOOTH ;
	}
    if( m_cFeedDef.m_cPlungeDef.m_dMaxZInc < 0.002 )
    {
        m_cFeedDef.m_cPlungeDef.m_dMaxZInc = 0.002 ;
    }
	m_pGraph = &Graph ;
	// 获取毛胚包围盒
	BOX3D box;
	if (Graph.GetStockBox(box))
	{
		if (m_pStockBox == NULL)
		{
			m_pStockBox = new BOX3D;
		}
		*m_pStockBox = box;
	}	
	else
	{
		if (m_pStockBox != NULL)
		{
			delete m_pStockBox;
			m_pStockBox = NULL;
		}
	}
	if ( !(glbf_GetAllPlugMask() & PLUG_ENABLE_NC5AXIS) && m_c5DCtrlDef.m_cToolAxis.m_nAxisType )
	{ // 如果非多轴授权并且刀轴不竖直,退出
		m_nErrorType = JDERROR_GENPATH5AX_PLUGTOOLAXIS ;
		return FALSE ;
	}
	if( !SurfNC_IsSupport5AxMethod() && ( SurfNC_Is5DMethod( m_cMethodDef.m_nMethodType ) && 
		( m_cMethodDef.m_nMethodType != surfncMethodSGuide5D && 
		  m_cMethodDef.m_nMethodType != surfncMethodCMorph5D && 
		  m_cMethodDef.m_nMethodType != surfncMethodSwarf5D   )) )
	{
		m_nErrorType = JDERROR_GENPATH5AX_PLUG5AX ;
		return FALSE ;
	}
	if( !SurfNC_IsSupport5AxMethod() && m_cMethodDef.m_nMethodType == surfncMethodFeatureHole &&
		m_c5DCtrlDef.m_cToolAxis.m_nAxisType != NCDEF_AXIS_VERT )
	{
		m_nErrorType = JDERROR_GENPATH5AX_PLUG5AX ;
		return FALSE ;
	}
	if( !SurfNC_IsSupportCleanup5D() && m_cMethodDef.m_nMethodType == surfncMethodCleanup5D )
	{
		m_nErrorType = JDERROR_GENPATH5AX_PLUG5DCLEAN ;
		return FALSE ;
	}	
	if( !SurfNC_IsSupportImpellerMill() && m_cMethodDef.m_nMethodType == surfncMethodImpeller5D )
	{
		m_nErrorType = JDERROR_GENPATH5AX_PLUGIMPELLER ;
		return FALSE ;
	}
	if( m_cMethodDef.m_nMethodType == surfncMethodSFinish &&
		m_cMethodDef.m_cSFinishCut.m_nMoveType != surfncMoveUWLine &&
		m_c5DCtrlDef.m_cToolAxis.m_nAxisType != NCDEF_AXIS_VERT && 
		( m_cToolDef.m_nToolType != surfncToolBall &&
		m_cToolDef.m_nToolType != surfncToolABall) )
	{	// 精加工三轴转五轴路径只支持球头刀和锥球刀
		m_nErrorType = JDERROR_GENPATH_SUPPORTBALLONLY;
		return FALSE ;
	}
	// 计算加工范围
	m_dTopHeight = m_cShapeDef.m_cDepth.m_dTopHeight ;
	m_dCutDepth  = m_cShapeDef.m_cDepth.m_dCutDepth ;
	m_dBotHeight = m_dTopHeight - m_dCutDepth ;
	if( m_pGraph && SurfNC_IsMethodNeedSurfaces( m_cMethodDef.m_nMethodType, m_cParamter.GetMoveType() ) &&
		( m_cShapeDef.m_bShapeFlag & NCDEF_SHAPE_AUTODEPTH ) )
	{
		double dTmpTop = 0., dTmpDepth = 0. ;
		if( m_pGraph->GetShapeRange ( m_cParamter, dTmpTop, dTmpDepth ) )
		{
			m_dTopHeight = dTmpTop ;
			m_dCutDepth = dTmpDepth ;
			m_dBotHeight = m_dTopHeight - m_dCutDepth ;
		}
	}

	if (glbf_GetProductID() == PRODUCT_ID_DENTFORM)
	{
		CString sMethodName ;
		sMethodName.Format(m_cParamter.m_sName);
		//CString PrgMsg = "名称：" + sMethodName;
		m_cPrgDef.m_pNewFunc(sMethodName.GetBuffer(sMethodName.GetLength()));
	}

	BOOL IsMethod5D = (m_cMethodDef.m_nMethodType == surfncMethodSGuide5D || m_cMethodDef.m_nMethodType == surfncMethodCMorph5D
		|| m_cMethodDef.m_nMethodType == surfncMethodThreadMill5D || m_cMethodDef.m_nMethodType == surfncMethodDrill5D);
	BOOL bMdfyGrnd = FALSE;
	if (( m_cMethodDef.m_nMethodType == surfncMethodLoop && (m_cMethodDef.m_cLoopCut.m_bLoopFlag & NCDEF_FLOOP_GRNDMDFY)) ||
		(m_cMethodDef.m_nMethodType == surfncMethodCurve && (m_cMethodDef.m_cCurveCut.m_bCurveFlag & NCDEF_FCURVE_GRNDMDFY)))
	{
		bMdfyGrnd = TRUE;
	}
	//guomin 控制节点步长的情况下需要将误差分配
	BOOL bMaxDist = ((m_cSetupDef.m_cModelTol.m_nMdlFlag & NCDEF_SETUP_REDISTRIBUTE) || bMdfyGrnd) ? TRUE : FALSE;
	if( bMaxDist && !IsMethod5D)
	{
		m_cSetupDef.m_cTolDef.m_dArcTol /= 2 ;
		m_cSetupDef.m_cTolDef.m_dAngTol /= 2 ;
	}
	BOOL bRet = GeneratePathEx( NewPath, Graph ) ;
	//	if( m_c5DCtrlDef.m_cToolAxisApp.m_nFrameFlag )
	//	{// 将路径转为当前加工面下路径,临时坐标系的任务完成
	//		TransLocalGroup( NewPath, &m_c5DCtrlDef.m_cToolAxisApp.m_dIndexFrame ) ;
	//	}
	if( ! bRet ) return bRet ;
	if( (m_cMethodDef.m_nMethodType == surfncMethodProject && 
			m_cMethodDef.m_cProjectCut.m_bProjectFlag & NCDEF_FPROJECT_DELETE) ||
		(m_cMethodDef.m_nMethodType == surfncMethodWrap && 
		m_cMethodDef.m_cWrapCut.m_bWrapFlag & NCDEF_FWRAP_DELETE) )
	{// 不能合并路径组
	}
	else
	{// 合并路径组
		NewPath.CombineAllPComb() ;
	}

	SetEngraveParam( NewPath, Graph ) ;
	// 添加慢速下刀路径
	//Add5DSafeQuickPath(NewPath);
	AddSlowPlungePath(&NewPath, Graph);	
	NewPath.DelOverlapPoint() ;

	/*路径变换*/
	if(m_cTransfDef.m_cProjtranDef.m_nProjtranType != 0)
	{//投影变换
		ProjAndWrapPath( NewPath, Graph);
	}

	if ( NewPath.m_nType != NC_PATHTYPE_DRILL5AX && NewPath.m_nType != NC_PATHTYPE_ENGRAVE5AX && !IsMethod5D )
	{// 添加安全路径
		Add3DSafeQuickPath(NewPath);
	}
	NewPath.UpdateAllRate() ;
	if( !IsMethod5D )
		PointDistributeFor3DPath(NewPath,bMdfyGrnd,bMaxDist) ;//guomin 路径点分布处理
	// 磨削调整变换
	MdfyGrndFor3DPath(NewPath, Graph);
	// 使用刀触点匹配路径速度
	MatchSpeedByToolPosFor3DPath(NewPath, Graph);

//#ifdef _DEBUG
//	// 转成刮铣路径 
//	ConvertShavePath(NewPath, m_cSetupDef.m_cTolDef);
//#endif

	if( m_cSetupDef.m_cModelTol.m_nMdlFlag & NCDEF_SETUP_SLOWCORNER )
    {/*尖角降速*/
        if(m_cMethodDef.m_nMethodType == surfncMethodCorner)
		{
			float fDist = (float)m_cMethodDef.m_cCornerCut.m_dSlowCornerDist ;
			float fRatio = (float)m_cMethodDef.m_cCornerCut.m_dSlowCornerRatio ;
			SlowAtCorner(NewPath, fDist, fRatio);
		}
		else
		{
			NewPath.SlowBeforeCorner( 65.0f, 0.5f, min( (float)m_cSpeedDef.m_dConnectRate, 0.15f)) ;
		}
    }
	//guomin 恢复误差值
	if( bMaxDist && !IsMethod5D )
	{
		m_cSetupDef.m_cTolDef.m_dArcTol *= 2 ;
		m_cSetupDef.m_cTolDef.m_dAngTol *= 2 ;
	}

	return m_bRComb;
}

void CSmartPathGen::ProcessNewFunc ( JDNC_PRGDEF &ProgDef, int nIDS )
{
	char strTmp[200] ;
	if( glbf_LoadSysString( nIDS, strTmp, 199 ) == 0 )
	{
		strTmp[0] = 0 ;
	}
	ProgDef.m_pNewFunc( strTmp ) ;
}
char* CSmartPathGen::GetErrorMsg()
{
	UINT uID ;
	switch(m_nErrorType)
	{
	case JDERROR_GENPATH_NOGRAPH:
		uID=IDS_ERROR_GENPATH_NOGRAPH;
		break;
	case JDERROR_GENPATH_NOLOOP:
		uID=IDS_ERROR_GENPATH_NOLOOP;
		break;
	case JDERROR_GENPATH_NOSURFACE:
		uID=IDS_ERROR_GENPATH_NOSURFACE;
		break;
	case JDERROR_GENPATH_NOPOINT:
		uID=IDS_ERROR_GENPATH_NOPOINT;
		break;
	case JDERROR_GENPATH_NOPATH:
		uID=IDS_ERROR_GENPATH_NOPATH;
		break;
	case JDERROR_GENPATH_NOCURVE:
		uID=IDS_ERROR_GENPATH_NOCURVE;
		break;
	case JDERROR_GENPATH_NOTOOL:
		uID=IDS_ERROR_GENPATH_NOTOOL;
		break;
	case JDERROR_GENPATH_STOCK:
		uID=IDS_ERROR_GENPATH_STOCK;
		break;
	case JDERROR_GENPATH_NOPARAM:
		uID=IDS_ERROR_GENPATH_NOPARAM;
		break;
	case JDERROR_GENPATH_NOGEN:
		uID=IDS_ERROR_GENPATH_NOGEN;
		break;
    case JDERROR_GENPATH_NOAREA :
		uID=IDS_ERROR_GENPATH_NOAREA;
		break;
	case JDERROR_GENPATH_ERRDATA:
		uID=IDS_ERROR_GENPATH_ERRDATA;
		break;
	case JDERROR_GENPATH_ERRDIAM:
		uID=IDS_ERROR_GENPATH_ERRDIAM;
		break;
	case JDERROR_GENPATH_ABORT:
		uID=IDS_ERROR_GENPATH_ABORT;
		break;
	case JDERROR_GENPATH_NOTGENPATH:
		uID=IDS_ERROR_GENPATH_NOTGENPATH;
		break;
	case JDERROR_GENPATH_NOROUGHSURF:
		uID = IDS_ERROR_GRAPHIC_NOROUGHCAST ;
		break ;
	case JDERROR_GENPATH_NOCHECKSURF:
		uID = IDS_ERROR_GRAPHIC_NOCHECKSURF ;
		break ;
	case JDERROR_GENPATH_LOOPERROR:
		uID = IDS_ERROR_GENPATH_ERRLOOP ;
		break ;
	case JDERROR_GENPATH_SURFSTOCK:
		uID = IDS_ERROR_GENPATH_SURFSTOCK ;
		break ;
	case JDERROR_GENPATH_BNDLOOP:
		uID = IDS_ERROR_GENPATH_BNDLOOP ;
		break ;
	case JDERROR_GENPATH_PLANARERROR:
		uID = IDS_ERROR_GENPATH_PLANARERROR ;
		break ;
	case JDERROR_GENPATH_SPARKERROR:
		uID = IDS_ERROR_GENPATH_SPARKERROR ;
		break ;
	case JDERROR_GENPATH_CLEANUPTOOL:
		uID = IDS_ERROR_GENPATH_CLEANUPTOOL ;
		break ;
	case JDERROR_GENPATH_AUTOCLEAN  :
		uID = IDS_ERROR_GENPATH_AUTOCLEAN ;
		break ;
	case JDERROR_GENPATH_NETSURFUW :
		uID = IDS_ERROR_GENPATH_NETSURFUW ;
		break ;
	case JDERROR_GENPATH_TAPTOOL :
		uID = IDS_ERROR_GENPATH_TAPTOOL ;
		break ;
	case JDERROR_GENPATH_PITCHDIF :
		uID = IDS_ERROR_GENPATH_PITCHDIFFERENT ;
		break ;
	case JDERROR_GENPATH_NOSTOCK :
		uID = IDS_ERROR_GENPATH_NOSTOCK ;
		break ;
	case JDERROR_GENPATH_CLEANERROR :
		uID = IDS_ERROR_GENPATH_CLEANERROR ;
		break ;
	case JDERROR_GENPATH_ERRORDER :
		uID = IDS_ERROR_GENPATH_ERRORDER ;
		break ;
	case JDERROR_GENPATH_NOREMAINMDL :
		uID = IDS_ERROR_GENPATH_NOREMAINMDL ;
		break ;
	case JDERROR_GENPATH_SUPPORTBALLONLY:	// 三轴转五轴路径只支持球头刀和锥球刀
		uID = IDS_ERROR_GENPATH_SUPPORTBALLONLY ;
		break ;
	case JDERROR_GENPATH5AX_NOAXISCURVE:
		uID = IDS_ERROR_GENPATH5AX_NOAXISCURVE ;
		break ;
	case JDERROR_GENPATH5AX_NOAXISCURVE5AX:
		uID = IDS_ERROR_GENPATH5AX_NOAXISCURVE5AX ;
		break ;
	case JDERROR_GENPATH5AX_NOGUIDESURF  :
		uID = IDS_ERROR_GENPATH5AX_NOGUIDESURF ;
		break ;
	case JDERROR_GENPATH5AX_NOMESHCURVE1  :
	case JDERROR_GENPATH5AX_NOMESHCURVE2  :
		uID = IDS_ERROR_GENPATH5AX_NOMESHCURVE ;
		break ;
	case JDERROR_GENPATH5AX_NOMESHSURF1  :
	case JDERROR_GENPATH5AX_NOMESHSURF2  :
		uID = IDS_ERROR_GENPATH5AX_NOMESHSURF ;
		break ;
	case JDERROR_GENPATH5AX_NOSWARFTOPCUR :
		uID = IDS_ERROR_GENPATH5AX_NOSWARFTOPCUR ;
		break ;
	case JDERROR_GENPATH5AX_NOSWARFBTMCUR :
		uID = IDS_ERROR_GENPATH5AX_NOSWARFBTMCUR ;
		break ;
	case JDERROR_GENPATH5AX_MAPFAILURE :
		uID = IDS_ERROR_GENPATH5AX_MAPFAILURE ;
		break ;
	case JDERROR_GENPATH5AX_NOBLADENUM:
		uID = IDS_ERROR_GENPATH5AX_NOBLADENUM ;
		break ;
	case JDERROR_GENPATH5AX_SUPPORTBALLONLY:
		uID = IDS_ERROR_GENPATH5AX_SUPPORTBALLONLY;
		break;
	case JDERROR_GENPATH5AX_CSGAP:
		uID = IDS_ERROR_GENPATH5AX_CSGAP;
		break;
	case JDERROR_GENPATH5AX_FINDNETSURF:
		uID = IDS_ERROR_GENPATH5AX_FINDNETSURF ;
		break ;
	case JDERROR_FOLLOWPATH_NOGUIDEPNT:
		uID = IDS_NO_GUIDEPNT ;
		break ;
	case JDERROR_FOLLOWPATH_NOGUIDECURVE:
		uID = IDS_NO_GUIDECURVE ;
		break ;
	case JDERROR_GENPATH5AX_SWARFTOOLAXIS :
		uID = IDS_SWARF_TOOLAXIS_CTRL ;
		break ;
	case JDERROR_GENPATH5AX_SIDEANGTOOBIG :
		uID = IDS_SWARF_SIDEANGTOOBIG ;
		break ;
	case JDERROR_GENPATH5AX_PLUGTOOLAXIS :
		uID = IDS_ERROR_PLUG3_TOOLAXIS ;
		break ;
	case JDERROR_GENPATH5AX_PLUG5AX :
		uID = IDS_ERROR_PLUG_NO5AX ;
		break ;
	case JDERROR_GENPATH5AX_PLUG5DCLEAN:
		uID = IDS_ERROR_PLUG_NO5DCLEAN ;
		break ;
	case JDERROR_GENPATH5AX_PLUGIMPELLER :
		uID = IDS_ERROR_PLUG_NOIMPELLER ;
		break ;
	case JDERROR_FEATURESLOT_TOOLERROR:
		uID = IDS_ERROR_FEATURESLOT_TOOLERROR ;
		break;
	default:
		uID = IDS_ERROR_GENPATH_UNKNOWN;
		break;
	}

	if( SurfNC_IsAbort() )
		uID=IDS_ERROR_GENPATH_ABORT;

	LPSTR lpBuffer = glbf_GetPromptString();
	glbf_LoadSysString( uID, lpBuffer, 100) ;
	return lpBuffer;
}

void CSmartPathGen::ProjLoopOnMdl( CSmtCheckMdl& DriveMdl,
								   CSmtCheckMdl& RoughMdl,
								   CSmtCPathLib& AllPath,
								   CSmartLoop* pHead, 
								   JDNC_PRGDEF& PrgDef,
								   int bFlag ) 
{
	CSmtCutPath *pPath = NULL ;
	CSmartLoop *pLoop = NULL , *pIsland = NULL ;
	for( pLoop = pHead; pLoop ; pLoop = pLoop->next )
	{
		pPath = CurveToCPath( *(pLoop->m_pCurve), GetCurveTol() ) ;
		pPath->InsertCPoint( GetSurfaceTol().m_dMaxStep ) ;
		pPath->VerifyCutPath( DriveMdl, GetSurfaceTol(), PrgDef ) ;
		if( bFlag )
		{
			SetCutPathZ( pPath, 0 ) ;
			pPath->VerifyCutPath( RoughMdl, GetSurfaceTol(), PrgDef ) ;
			SetCutPathZ( pPath, 1 ) ;
		}

		pPath->m_nLineNo = -1 ;
		pPath->m_nLayerNo = -1 ;
		AllPath.AddToTail( pPath ) ;
		pPath = NULL ;
		for( pIsland = pLoop->m_pIsland ; pIsland ; pIsland = pIsland->next )
		{
			pPath = CurveToCPath( *(pIsland->m_pCurve), GetCurveTol() ) ;
			pPath->VerifyCutPath( DriveMdl, GetSurfaceTol(), PrgDef ) ;
			if( bFlag )
			{
				SetCutPathZ( pPath, 0 ) ;
				pPath->VerifyCutPath( RoughMdl, GetSurfaceTol(), PrgDef ) ;
			}
			pPath->m_nLineNo = -1 ;
			pPath->m_nLayerNo = -1 ;
			AllPath.AddToTail( pPath ) ;
			pPath = NULL ;
		}
	}
}

void CSmartPathGen::SetCutPathZ ( CSmtCutPath* pPath, int bFlag )
{
	CSmtCutPoint *pHead, *pPoint ;
	pHead = pPath->m_pHead ;
	while( pHead )
	{
		pPoint = pHead ;
		pHead = pHead->next ;
		if( !bFlag  )//将z坐标转移
		{
			pPoint->m_fPoint[3] = pPoint->m_fPoint[2] ;
		}
		else if ( bFlag == 1 )
		{   // 记录原始路径的Z坐标
			double z = pPoint->m_fPoint[3] ;
			// 将毛坯路径的Z坐标放到第四坐标
			pPoint->m_fPoint[3] = pPoint->m_fPoint[2] ;
			// 设置路径的Z坐标为原始路径Z-毛坯路径Z
			pPoint->m_fPoint[2] = TFLOAT(z) - pPoint->m_fPoint[3] ;
		}
		else
		{   // 还原路径的Z坐标
			pPoint->m_fPoint[2] += pPoint->m_fPoint[3] ;
		}
	}
}
// 一组区域按照高度优先排序, LoopArr 的 m_nDepth必须有效
BOOL  CSmartPathGen::SortLoopArray( CSmtLoopArr& LoopArr, /*轮廓组  */  
		                            DOUBLE Tol          ) /*轮廓误差*/
{
	CSmartLoop *pChild, *pLoop, *pLeft ;
    CSmtLoopArr tmpArr ;
	PNT3D dStart, dEnd ;
	INT_PTR nAt =  0 ;
	while( LoopArr.GetSize() > 0 )
	{
		pLoop = LoopArr.GetAt(0) ;
		LoopArr.RemoveAt(0) ;
		tmpArr.Add( pLoop ) ;
		nAt = 0 ; 
		while( pLoop )
		{
			pLoop->m_pCurve->GetPoint( 0.0, dStart ) ;
			INT_PTR nCount = LoopArr.GetSize() ;
			pChild = NULL ; 
			for( INT_PTR k = nAt ; k < nCount ; k ++ ) 
			{ 
				pChild = LoopArr.GetAt( k ) ;
			    if( pChild->m_nDepth != pLoop->m_nDepth + 1 )
				{
					continue ;
				} 
				pChild->m_pCurve->GetPoint( 0.0, dEnd ) ;
				if( nc_Distance( dStart, dEnd , 2 ) < 1.0e-3 )
				{
					break ;
				}
  		        if( !pChild->IsLoopOverLap( *pLoop , Tol ) )
				{
					continue ;
				} 
			    for( INT_PTR j = 0 ; j < k ; j ++ )
				{
					pLeft = LoopArr.GetAt( j ) ;
					if( pLeft == pLoop ) continue ;
				    if( pLeft->m_nDepth == pLoop->m_nDepth && 
					    pLeft->IsLoopOverLap( *pChild , Tol )  )
					{
						 break ;
					} 
				} 
				if( j == k )
				{
					break ;
				}
			}
			pLoop = NULL ; 
			if( k < nCount && pChild )
			{
				LoopArr.RemoveAt( k ) ;
				tmpArr.Add( pChild ) ;
				pLoop = pChild ;
				nAt = k ;
			}
		}
	}
	ASSERT( LoopArr.GetSize() == 0 ) ;
	LoopArr.Copy( tmpArr ) ; 

    return TRUE ;
}
int CSmartPathGen::GenCheckMdl(  CSmartGraphic& Graph  , 
							     CSmtCheckMdl& CheckMdl, 
								 int nSurfMode    ) 
{
	if( ! Graph.BuildCheckModel( m_cSetupDef, CheckMdl, nSurfMode ) )
	{
		m_nErrorType = JDERROR_GENPATH_NOSURFACE ;
		return 0 ; 
	}
    CheckMdl.UpdateCurrTool( m_pTool )  ;
	CheckMdl.m_fBottom  =  (TFLOAT) GetBottomHeight() ;
	return 1 ;
}
void CSmartPathGen::ResetToolCutDepth( CSmtCheckMdl& DriveMdl,
                                       CSmartTool*  Tool )
{
    if( ! Tool ) return ;
    int nToolType = Tool->GetType() ;
    if( nToolType != smtToolAFlat && 
        nToolType != smtToolABall && 
        nToolType != smtToolANose   )
    {
        return ; 
    }
    FPNT3D fMinPt, fMaxPt ;
    if( ! DriveMdl.CalcFacetBox( fMinPt, fMaxPt ) )
    {
        return ;
    }
    double dMaxDepth = fMaxPt[2] - DriveMdl.m_fBottom + 0.05;
    dMaxDepth += max( 0.0, DriveMdl.m_dZShift );
    if( m_cMethodDef.m_nMethodType == surfncMethodRPocket || 
        m_cMethodDef.m_nMethodType == surfncMethodSRemain   )
    {
        dMaxDepth += 0.5 ;
    }
    else if( m_cMethodDef.m_nMethodType == surfncMethodSFinish &&
             (m_cMethodDef.m_cSFinishCut.m_nMoveType == surfncMoveHeight ||
             m_cMethodDef.m_cSFinishCut.m_nMoveType == surfncMoveMixture) )
    {
        dMaxDepth += 0.5 ;
    }
	Tool->SetCutDepth( dMaxDepth ) ;
}
void CSmartPathGen::SetCheckFltBoxByDriveMdl( CSmtCheckMdl& DriveMdl ,
                                              CSmtCheckMdl& CheckMdl )
{
    DriveMdl ;
    CheckMdl ;
}

BOOL CSmartPathGen::BuildDriveAndCheckMdl ( CSmartGraphic& Graph, 
										   CSmtCheckMdl& DriveMdl, 
										   CSmtCheckMdl& CheckMdl,
										   double RotAngle )
{
	//STEP 1 : 构建加工面模型
    if( m_c5DCtrlDef.m_cToolAxis.m_nAxisType != NCDEF_AXIS_VERT )
    {/*多轴模式, 设置多轴剖分模式*/
        DriveMdl.SetMultiAxisCellMode( TRUE ) ;
        CheckMdl.SetMultiAxisCellMode( TRUE ) ;
    }
    JDNC_SETUP drvSetup = m_cSetupDef ;
    if( m_cSetupDef.m_cModelTol.m_nMdlFlag & NCDEF_SETUP_EXACTMODE )
    {
        drvSetup.m_cTolDef.m_dArcTol  = min( 2.0e-4,drvSetup.m_cTolDef.m_dArcTol ) ;
        drvSetup.m_cTolDef.m_dAngTol  = min( 10.0  ,drvSetup.m_cTolDef.m_dAngTol ) ;
        DriveMdl.SetExactCalcMode(TRUE) ;
    }
	if( ! Graph.BuildCheckModel( drvSetup, DriveMdl, NCDEF_SURF_DRIVE ) ) 
	{
		return FALSE		;
	}
	if (glbf_GetProductID() == PRODUCT_ID_DENTFORM)
	{
		SurfNC_SetCurPos(10, 100);
	}
	AutoAdjustCutDepth	( DriveMdl ) ;
/*封掉原因：等高路径向上延伸暂时不采取构造延伸面的方法，
            而是采用直接对路径进行延伸(2011.7.25)*/
// #ifdef _DEBUG
// 	BOOL bFlag = m_cMethodDef.m_nMethodType == surfncMethodSFinish 
// 				 && m_cMethodDef.m_cSFinishCut.m_nMoveType == surfncMoveHeight
// 				 && IsDelBndPoint() ;
// 	if ( bFlag )
// 	{// 构建延伸面模型
// 		double dDist = m_pTool->m_fRadius ; //延伸距离（界面参数）
// 		CSmtCheckMdl ExtndSurfMdl ;
// 		Graph.BuildExtendSurfMdl( drvSetup, ExtndSurfMdl, dDist ) ;
// 		INT_PTR nSize = ExtndSurfMdl.m_cAllChk.GetSize() ;
// 		for ( INT_PTR i = nSize-1 ; i >= 0 ; i-- )
// 		{
// 			CSmartCheck* pObjChk = ExtndSurfMdl.m_cAllChk.GetAt( i ) ;
// 			ExtndSurfMdl.m_cAllChk.RemoveAt( i ) ;
// 			DriveMdl.AddCheck( pObjChk ) ;
// 		}
// 	}
// #endif

	TFLOAT fZBottom =  (TFLOAT)GetBottomHeight() ;
	if( IsDelBndPoint() ) 
    {
        fZBottom -= (TFLOAT)GetSurfaceTol().m_dArcTol ;
    }
	DriveMdl.ZMoveModel( GetDriveSurfZMove() ) ;
    DriveMdl.m_dTouchTol = max( m_cSetupDef.m_cTolDef.m_dArcTol*3.0, m_cSetupDef.m_cModelTol.m_dTouchTol ) ;
	DriveMdl.m_fBottom =  fZBottom ;
    ResetToolCutDepth( DriveMdl, m_pTool ) ;
	//STEP 2 : 构建保护面模型
    JDNC_SETUP checkSetup = m_cSetupDef ;
    checkSetup.m_cModelTol.m_nMdlFlag &= ~NCDEF_SETUP_NOMOSAIC ;
    checkSetup.m_cModelTol.m_nMdlFlag &= ~NCDEF_SETUP_EXACTMODE ;
    SetCheckFltBoxByDriveMdl( DriveMdl,CheckMdl );
	if( Graph.BuildCheckModel( checkSetup, CheckMdl, NCDEF_SURF_CHECK ) )
	{
		CheckMdl.ZMoveModel( GetCheckSurfZMove() ) ;
		CheckMdl.m_fBottom =  fZBottom ;
        CheckMdl.m_dTouchTol = DriveMdl.m_dTouchTol ;
        ResetToolCutDepth( CheckMdl, m_pSafeTool ) ;
		if( CheckMdl.m_cAllChk.GetSize() > 0 ) 
		{
			DriveMdl.SetCheckMdl( & CheckMdl ) ;
		}
	}
	if (glbf_GetProductID() == PRODUCT_ID_DENTFORM)
	{
		SurfNC_SetCurPos(20, 100);
	}
	//STEP 3 : 设置加工面属性
    if( IsDelFlatPoint() )
    {/*删除平面路径点*/
        DriveMdl.m_bCheckFlag |= SMARTNC_CHECKMDL_DELFLATPNT ;
        DriveMdl.ZMovePlaneSurf( m_cSetupDef.m_cTolDef.m_dArcTol );
    }
    if( ( m_cSetupDef.m_cModelTol.m_nMdlFlag & NCDEF_SETUP_TOUCHMODE ) && 
        ( IsDelBndPoint() || DriveMdl.m_pCheckMdl ) )
    {/*设置刀触点模式*/
        DriveMdl.SetTouchCalcMode( TRUE ) ;
        if( m_cSetupDef.m_cModelTol.m_nMdlFlag & NCDEF_SETUP_TOUCHSURF )
        {
            DriveMdl.m_bCheckFlag |= SMARTNC_CHECKMDL_TOUCHSURF ;
        }
    }
    if( m_cSetupDef.m_cModelTol.m_nMdlFlag & NCDEF_SETUP_EXACTMODE )
    {/*设置精确模式,必须启动消除马赛克选项*/
        m_cSetupDef.m_cModelTol.m_nMdlFlag |= NCDEF_SETUP_NOMOSAIC ;
    }
	if( IsDelBndPoint() )
    {/*设置删除边界路径点模式*/
        DriveMdl.m_bCheckFlag |= SMARTNC_CHECKMDL_DELBOUNDPNT ;
        DriveMdl.DefineBoundEdge();
    }
	if( fabs( RotAngle ) > 1.0e-4 )
	{/*设置旋转模型模式*/
		DriveMdl.RotateByZAxis( RotAngle ) ;
        CheckMdl.RotateByZAxis( RotAngle ) ;
	}
    DriveMdl.UpdateAllTool( m_pTool, m_pSafeTool ) ;
	return TRUE ;
}
// 生成五轴加工面和干涉面的混合模型
BOOL CSmartPathGen::Build5AxDriveAndCheckMdl( CSmartGraphic& Graph , 
                                              CSmtCheckMdl& DriveMdl, 
						                      CSmtCheckMdl& CheckMdl) 
{
    if( ! m_pTool || ! m_pSafeTool )
    {
        return 0 ;
    }
    DriveMdl.m_bCheckMode = SMART_MODEL_ROTATE;
    CheckMdl.m_bCheckMode = SMART_MODEL_ROTATE;
    if(!Graph.BuildCheckModel(m_cSetupDef, DriveMdl, NCDEF_SURF_DRIVE)) 
    { 
        m_nErrorType = JDERROR_GENPATH_NOAREA ;
        return 0;
    }
    AutoAdjustCutDepth( DriveMdl ) ;
    Graph.BuildCheckModel(m_cSetupDef, CheckMdl, NCDEF_SURF_CHECK);    
    DriveMdl.m_fBottom = (TFLOAT)GetBottomHeight();
    DriveMdl.m_dZShift = GetDriveSurfZMove(); 
    if(CheckMdl.m_cAllChk.GetCount() > 0)
    {
        CheckMdl.m_fBottom = (TFLOAT)GetBottomHeight();
        CheckMdl.m_dZShift = GetCheckSurfZMove(); 
        DriveMdl.SetCheckMdl(&CheckMdl);
    }
    if( m_c5DCtrlDef.m_cToolAxis.m_nAxisType == NCDEF_AXIS_VERT && 
        m_pHolderTool == NULL && m_cToolDef.m_nToolType < 7 )
    {
        DriveMdl.m_bCheckMode = SMART_MODEL_PROJECT;
        CheckMdl.m_bCheckMode = SMART_MODEL_PROJECT;
    }
    DriveMdl.UpdateAllTool( m_pTool, m_pSafeTool ) ;
    /*构建包围盒曲面，用于路径计算提速*/
    DriveMdl.UpdateBoundSurf() ;
    CheckMdl.UpdateBoundSurf() ;
    DriveMdl.m_pHolderShape = m_pHolderTool ;
    CheckMdl.m_pHolderShape = m_pHolderTool ;
    double dMStep = m_pTool->GetRadiusComp( m_pTool->m_fRadius,0.0 ) ;
    if( m_cSetupDef.m_cTolDef.m_dMaxStep > dMStep )
    {
        m_cSetupDef.m_cTolDef.m_dMaxStep = dMStep ;
    }
    return TRUE ;
}
// 生成五轴加工面和干涉面的混合模型
BOOL CSmartPathGen::TransTo5AxDriveAndCheckMdl(CSmtCheckMdl& DriveMdl) 
{
    if( ! m_pTool || ! m_pSafeTool )
    {
        return 0 ;
    }
    DriveMdl.DeleteMultiCopy() ;
    CSmtCheckMdl*  chkMdl[2] = { & DriveMdl, DriveMdl.m_pCheckMdl } ;
    CSmartTool*    chkTool[2]= { m_pTool, m_pSafeTool } ;
    for( int nMdl = 0 ; nMdl < 2 ; nMdl ++ ) 
    {
        if( chkMdl[nMdl] == NULL ) break ;
        CSmtCheckMdl&  surfMdl = * chkMdl[nMdl] ;
        surfMdl.m_bCheckMode = SMART_MODEL_ROTATE;
        int nNumChk, i ;
        nNumChk = (int) surfMdl.m_cAllChk.GetSize() ;
        for( i = 0 ; i < nNumChk ; i ++ ) 
        {
            CSmartCheck* pChk = surfMdl.m_cAllChk.GetAt( i ) ;
            pChk->TransToMixFacet() ;
        }
        surfMdl.UpdateSmoothMdl( NULL, 0.0 ) ;
        surfMdl.UpdateModelTool( chkTool[nMdl] ) ;
        surfMdl.UpdateBoundSurf() ;
        surfMdl.m_pHolderShape = m_pHolderTool ;
    }
    double dMStep = m_pTool->GetRadiusComp( m_pTool->m_fRadius,0.0 ) ;
    if( m_cSetupDef.m_cTolDef.m_dMaxStep > dMStep )
    {
        m_cSetupDef.m_cTolDef.m_dMaxStep = dMStep ;
    }
    return TRUE ;
}
CSmartLoop* CSmartPathGen::OffsetBndLoop ( CSmartLoop* AllLoop )
{
	CSmartLoop LoopLib ;
	if( !AllLoop ) return NULL ;
	CSmartLoop	*pAllLoop = NULL ;

	// 曲面边界补偿偏移和加工余量偏移
	DOUBLE dDist = 0. ;
	DOUBLE dSurfOffDist = 0 ;
	
	CSmartTool *pTool = CreateSmartTool( m_cToolDef, 0., 0. ) ;


	if( m_cShapeDef.m_bShapeFlag & NCDEF_SHAPE_OUTER )
	{
		dDist = -pTool->GetRadiusComp( GetCutDepth(), 0. ) ;
	}
	else if( m_cShapeDef.m_bShapeFlag & NCDEF_SHAPE_INNER )
	{
		dDist = pTool->GetRadiusComp( GetCutDepth(), 0. ) ;
	}

	dSurfOffDist = pTool->GetRadiusComp( GetCutDepth(), 0. ) ;
	delete pTool ;
	
	// 如果没有选择边界
    dDist += GetSideStock() ;
	
	if( AllLoop && fabs( dDist ) > 1.0e-4    )
	{ 
		int nOffSide = NCDEF_OFFSET_INNER ;
		if( dDist < 0.0 ) 
		{
			dDist = -dDist, nOffSide = NCDEF_OFFSET_OUTER ;
		}
		double dIsDist = dDist ;
		if( nOffSide == NCDEF_OFFSET_INNER && dDist > 0.1 && 
			fabs( dDist - dSurfOffDist) < 0.002  )
		{ /*避免边界与轮廓重合*/
			dDist -= 2.5 * m_cSetupDef.m_cTolDef.m_dArcTol ;
		}
		double OffDist[2] = { dDist, dIsDist } ;
		pAllLoop = Mini_OffsetContourList( AllLoop, nOffSide, OffDist, GetCorDef() ) ;
		if( pAllLoop )
			Mini_DeleteContours( AllLoop ) ;
		else
            pAllLoop = AllLoop ;
	}
	else
	{
		pAllLoop = AllLoop ;
	}

	return pAllLoop ;
}
void MathCAM_SetAsLimitLoop( CSmartLoop& Loop )
{
    for( CSmartSect* pSect = Loop.GetSectHead() ; pSect ; pSect = pSect->next )
    {
        pSect->m_bEndFlag |= NC_LIMIT_SECT ;
    }
}
void CSmartPathGen::AddOpenFlag ( CSmartLoop* AllLoop )
{
	// 非分层粗加工返回
	if( m_cMethodDef.m_nMethodType != surfncMethodRPocket ) return ;
	
	// 处理外轮廓开阔
	CSmartLoop *pLoop = NULL ;
	BOOL  bIsOpenBnd = m_cMethodDef.m_cSRegionCut.m_bRPocketFlag & NCDEF_RPOCKET_OPENBND ? TRUE : FALSE ;
	if( bIsOpenBnd == TRUE )
	{
		// 外轮廓开阔,忽略外边框
		for( pLoop = AllLoop ; pLoop ; pLoop = pLoop->next )
		{
			if( MathCAM_IsOutMostLoop( AllLoop, pLoop, FALSE ) )
			{
				pLoop->m_bUseFlag|= NC_LOOP_OUTER ;
				pLoop->m_pCurve->SetBlank( TRUE ) ;
			}
		}
	}
}
CSmartLoop * CSmartPathGen::CreateAreaLoop ( CSmartGraphic &Graph, BOX3D &sBox, double r, BOOL &bAllLoop )
{
	//////////////得到加工边界////////////////////////
	CSmartLoop *pAllLoop = Graph.GetAllContour( m_cSetupDef ) ;
	// 曲面边界补偿偏移和加工余量偏移
	if( pAllLoop )
	{
		pAllLoop = OffsetBndLoop( pAllLoop ) ;
//		AddOpenFlag( pAllLoop ) ;
		bAllLoop = TRUE ;
	}
	if( !pAllLoop )
	{
		bAllLoop = FALSE ;
		for( int k = 0 ; k < 2 ; k++ )
		{
			sBox.max[k] += r ;
			sBox.min[k] -= r ;
		}
		pAllLoop = new CSmartLoop() ;
		pAllLoop->CreateLoop ( sBox.min, sBox.max ) ;
		CSmartLoop loop ;
		pAllLoop = loop.BuildContour ( pAllLoop ) ;
	}
	for( CSmartLoop *pLoop = pAllLoop ; pLoop ; pLoop = pLoop->next )
	{
        MathCAM_SetAsLimitLoop( *pLoop ) ;
        for( CSmartLoop* pIsl = pLoop->GetIsland() ; pIsl; pIsl = pIsl->next)
        {
            MathCAM_SetAsLimitLoop( *pIsl ) ;
        }
	}
	return pAllLoop ;
}
void  CSmartPathGen::PromptLoopErr ( double h )
{
	char strPrev[100], strNext[100] ;
	glbf_LoadSysString( IDS_PATHGEN_LOOPERR_PREV, strPrev, 99 ) ;
	glbf_LoadSysString( IDS_PATHGEN_LOOPERR_NEXT, strNext, 99 ) ;
	CString strWaring ;
	strWaring.Format( "%.4f", h ) ;
	strWaring = strPrev + strWaring + strNext ;
	MathCAM_ReportErrorGenPath( strWaring, MB_ICONSTOP ) ;
}
CSmartLoop* CSmartPathGen::GetSurfLoop ( CSmartGraphic& Graph, int nSurfaceType )
{/*计算曲面的边界, 用于粗加工, 残料补加工*/
	DOUBLE dBox[2][3] ;
	if( !Graph.GetEntBox( dBox, nSurfaceType ) )
    {
        return NULL ; 
    }
	PNT2D min, max ;
	for( int i = 0 ; i < 2 ; i++ )
	{
		min[i] = dBox[0][i] ;
		max[i] = dBox[1][i] ;
	}
	CSmartLoop* pLoop = new CSmartLoop() ;
	pLoop->CreateLoop ( min, max ) ;
	CSmartLoop loop ;
	pLoop = loop.BuildContour ( pLoop ) ;
	return pLoop ;
}
void CSmartPathGen::InitProg ()
{
	m_cPrgDef.m_dLimitAt = 0. ;
	m_cPrgDef.m_dStepAt  = 0 ;
	m_cPrgDef.m_dIncStep = 1. ;
	m_cPrgDef.m_pBrkFunc = NULL ;
	m_cPrgDef.m_pPrgFunc = NULL ;
	m_cPrgDef.m_pPosFunc = NULL ;
}

// 计算高度
DOUBLE CSmartPathGen::SnapNearestPoint( CPtrList& Curve3D, PNT3D Point ) 
{
	DOUBLE dAt , dHeight , dDist, dMinDist ;
	dHeight = 0.0 , dMinDist = 1.0e6 ;
	POSITION pos = Curve3D.GetHeadPosition() ;
	while( pos )
	{
		CSmartCurve * pCurve = (CSmartCurve*) Curve3D.GetNext( pos ) ;
		if( ! pCurve ) continue ;
		dAt = pCurve->SnapNearestHeight( Point, dDist ) ;
		if( dDist < dMinDist )
		{
			dHeight = dAt ;
			dMinDist = dDist; 
		}
	}
	return dHeight ;
}

// 计算高度
BOOL  CSmartPathGen::SnapNearestPointNew(	CPtrList &CurveList,	//<I> CRedepthCurve 
											PNT3D point		,		//<I> point
											double	i_t[2]		,	//<I> 输入初始参数t
											RFRAME &LocFrame ,		//<I> 相对坐标系
											BOOL	bKeep[2],		//<1> [0]高度,[1]速度
											double	dKeep[2] )		//<O> [0],高度,[1]速度
{
	if( !bKeep[0] && !bKeep[1] ) return FALSE ;
	DOUBLE dDist[2] = { 0., 0. } ;
	DOUBLE dMinDist[2] = { 1.0e6, 1.0e6 }, tmpt[2] = { i_t[0], i_t[1] } , t[2] = { 0., 0. } ;
	int nRet = 0 ;
	PNT3D o_p[2], near_p[2] ;
	CGeoInter geoint;
	BOOL bMultiCurve = FALSE ;
	// 如果是单根线，使用i_t用于追踪样条曲线，保证点的精度
	// 因为GetNearestPnt函数中的初始迭代函数精度较低，容易丢失点
	// 如果有多条曲线则直接取-1
	if( CurveList.GetCount () > 1 ) 
	{
		bMultiCurve = TRUE ;
		tmpt[0] = tmpt[1] =  -1.  ;
	}

	CRedepthCurve *pDepCurve = NULL, *pFind1 = NULL, *pFind2 = NULL ;
	CGeoCurve *pCurve3D = NULL, *pCurve2D = NULL ;
	POSITION pos = CurveList.GetHeadPosition() ;
	while( pos )
	{
		pDepCurve = ( CRedepthCurve *) CurveList.GetNext( pos ) ;
		if( ! pDepCurve ) continue ;
		pCurve3D = pDepCurve->m_pCurve3D ;
		pCurve2D = pDepCurve->m_pCurve2D ;
		if( !pCurve2D || !pCurve3D ) continue ;

		// 计算点到2D曲线的最近点，得到参数域t,保留曲线高度
		if( bKeep[0] )
		{
			if( tmpt[0] < 0 )
			{
				nRet = pCurve2D->GetNearestPnt( point, tmpt[0], o_p[0], dDist[0] ) ;
			}
			else
			{
				nRet = geoint.IterateCurveNearestPnt( pCurve2D, point, tmpt[0], o_p[0], dDist[0] ) ;
			}
			if( nRet && dDist[0] < dMinDist[0] && bKeep[0] )
			{
				dMinDist[0] = dDist[0] ;
				t[0] = tmpt[0] ;
				pFind1 = pDepCurve ;
				mathCpyPnt( o_p[0], near_p[0] ) ;
			}
		}
		// 读取曲线速度
		if( bKeep[1] && pCurve3D->GetType () == OBJ3D_CURVE_CURVE5AXIS )
		{
			// 计算点到2D曲线的最近点，得到参数域t
			if( tmpt[1] < 0 )
			{
				nRet = pCurve2D->GetNearestPnt( point, tmpt[1], o_p[1], dDist[1] ) ;
			}
			else
			{
				nRet = geoint.IterateCurveNearestPnt( pCurve2D, point, tmpt[1], o_p[1], dDist[1] ) ;
			}
			if( nRet && dDist[1] < dMinDist[1] )
			{
				dMinDist[1] = dDist[1] ;
				t[1] = tmpt[1] ;
				pFind2 = pDepCurve ;
				mathCpyPnt( o_p[1], near_p[1] ) ;
			}
		}

		if( bMultiCurve )
		{
			tmpt[0] = tmpt[1] = -1. ;
		}

	}
	// 获取保留曲线高度的参数
	if( !pFind1 || !pFind1->m_pCurve3D || !bKeep[0] ) 
	{
		dKeep[0] = point[2] ;
	}
	else
	{
		pFind1->m_pCurve3D->GetPoint( &LocFrame, t[0], near_p[0] ) ;
		dKeep[0] = near_p[0][2] ;
		i_t[0] = t[0] ;
	}
	// 获取读取曲线速度的参数
	if( !pFind2 || !pFind2->m_pCurve3D || !bKeep[1] ||
		pFind2->m_pCurve3D->GetType() != OBJ3D_CURVE_CURVE5AXIS )
	{
		dKeep[1] = 0. ;
	}
	else
	{
		CGeoCurve5ax *p5AxCur = ( CGeoCurve5ax *)pFind2->m_pCurve3D ;
		if( p5AxCur->m_nCurveType != CURVE5DTYPE_COMMON )
			dKeep[1] = p5AxCur->GetFeedRate( t[1] ) ;
		else
			dKeep[1] = 0. ;
	}
	return TRUE ;
}

// 重新设置高度
BOOL CSmartPathGen::RedepthPathEntity( CPtrList& Curve3D,    /*3D图形 CSmartCurve */ 
                                       CPathEntity* pEnt )   /*路径单元           */   
{
	if( ! pEnt ) return FALSE ;
	switch( pEnt->GetType() )
	{
	case NC_PATH_LINE3D :
		{
			CPathLine3D *pLine = (CPathLine3D*) pEnt ;
			pLine->m_fStart[2] += SnapNearestPoint( Curve3D,pLine->m_fStart ) ;
			pLine->m_fEnd[2] += SnapNearestPoint( Curve3D,pLine->m_fEnd) ;
		}
		break ;
	case NC_PATH_PLINE3D :
		{
			CPathPLine3D *pPLine = (CPathPLine3D*) pEnt ;
			for( int i = 0 ;  i <= pPLine->m_nCount ; i ++ ) 
			{
				pPLine->m_pTAPos[i][2] += SnapNearestPoint( Curve3D,pPLine->m_pTAPos[i] ) ;
			}
		}
		break ;
	case NC_PATH_ARC3D :
		{
			CPathArc3D *pArc3D = (CPathArc3D*) pEnt ;
			TPNT3D dStart, dEnd ;
			pArc3D->GetEndPoint( 0, dStart ) ;
			pArc3D->GetEndPoint( 1, dEnd   ) ;
			dStart[2] += SnapNearestPoint( Curve3D,dStart ) ;
			dEnd[2]   += SnapNearestPoint( Curve3D,dEnd   ) ;
			pArc3D->m_fCenter[2] = dStart[2] ;
			pArc3D->m_fDepth = dEnd[2] - dStart[2] ;
		}
		break ;
	default :
		ASSERT( 0 ) ;
		break ;
	}
	return TRUE ;
}

BOOL CSmartPathGen::RedepthPathEntity(CPathEntity *PathEnt, BOOL IsOut, DOUBLE &Depth)
{
	if( ! PathEnt ) return FALSE ;
	double dZ = 0.0;
	switch( PathEnt->GetType() )
	{
	case NC_PATH_LINE3D :
		{
			CPathLine3D *pLine = (CPathLine3D*) PathEnt ;
			if (IsOut)
			{
				dZ = Depth - pLine->m_fStart[2];
			}
			else
			{
				dZ = Depth - pLine->m_fEnd[2];
			}
			pLine->m_fStart[2] += dZ;
			pLine->m_fEnd[2] += dZ;
		}
		break ;
	case NC_PATH_PLINE3D :
		{
			CPathPLine3D *pPLine = (CPathPLine3D*) PathEnt ;
			if (IsOut)
			{
				dZ = Depth - pPLine->m_pTAPos[0][2];
			}
			else
			{
				dZ = Depth - pPLine->m_pTAPos[pPLine->m_nCount][2];
			}
			for( int i = 0 ;  i <= pPLine->m_nCount ; i ++ ) 
			{
				pPLine->m_pTAPos[i][2] += dZ ;
			}
		}
		break ;
	case NC_PATH_ARC3D :
		{
			TPNT3D dStart, dEnd ;
			CPathArc3D *pArc3D = (CPathArc3D*) PathEnt ;
			pArc3D->GetEndPoint( 0, dStart ) ;
			pArc3D->GetEndPoint( 1, dEnd   ) ;
			if (IsOut)
			{
				dZ = Depth - dStart[2];
			}
			else
			{
				dZ = Depth - dEnd[2];
			}
			dStart[2] += dZ;
			dEnd[2]   += dZ;
			pArc3D->m_fCenter[2] = dStart[2] ;
			pArc3D->m_fDepth = dEnd[2] - dStart[2] ;
		}
		break ;
	default :
		ASSERT( 0 ) ;
		break ;
	}
	return TRUE ;
}

double CSmartPathGen::GetPathEntityFeedRate(CPathEntity* PathEnt, JDNC_SPEED SpeedDef)
{
	if (!PathEnt)
	{
		return SpeedDef.m_dFeedRate;
	}
	double dFeedRate = SpeedDef.m_dFeedRate;
	switch(PathEnt->m_bFeedType)
	{
	case JDNC_FEEDTYPE_ROUGH:
		dFeedRate = SpeedDef.m_dFeedRate;
		break;
	case JDNC_FEEDTYPE_CONNECT:
		dFeedRate = SpeedDef.m_dConnectRate;
		break;
	case JDNC_FEEDTYPE_PLUNGE:
		dFeedRate = SpeedDef.m_dPlungeRate;
		break;
	case JDNC_FEEDTYPE_LEAD:
		dFeedRate = SpeedDef.m_dLeadRate;
		break;
	case JDNC_FEEDTYPE_SLOT:
		dFeedRate = SpeedDef.m_dSlotRate;
		break;
	case JDNC_FEEDTYPE_RETRACT:
		dFeedRate = SpeedDef.m_dRetractRate;
		break;
	default:
		break;
	}

	return dFeedRate;
}

BOOL CSmartPathGen::RedepthPathEntityNew(	CPtrList & CurveList,	//<I> CRedepthCurve 
											CPathEntity* pEnt , 	//<I> 路径单元   
											BOOL	bKeep[2],		// <I> [0]高度,[1]速度
											RFRAME &LocFrame )		//<I> 输入相对坐标系
{
	if( ! pEnt ) return FALSE ;
	double t[2] = {-1., -1. }, dKeep[2] = { 0., 0. } ;
	double dFeed1 = 0., dFeed2 = 0., dFeedRate = m_cSpeedDef.m_dFeedRate ;
	switch( pEnt->GetType() )
	{
	case NC_PATH_LINE3D :
		{
			CPathLine3D *pLine = (CPathLine3D*) pEnt ;
			// 起点高度及速度
			SnapNearestPointNew( CurveList,pLine->m_fStart, t, LocFrame, bKeep, dKeep ) ;
			if( bKeep[0] )
				pLine->m_fStart[2] += dKeep[0] ;
			if( bKeep[1] ) 	dFeed1 = dKeep[1]  ;

			if (pEnt->m_bFeedType != JDNC_FEEDTYPE_ROUGH)
			{
				t[0] = t[1] = -1.0;
			}
			// 末点高度及速度
			SnapNearestPointNew( CurveList,pLine->m_fEnd, t, LocFrame, bKeep, dKeep ) ;
			if( bKeep[0] )
				pLine->m_fEnd[2] += dKeep[0] ;
			if( bKeep[1] ) 
			{
				dFeed2 = dKeep[1] ;
				if (dFeed1 < MIN_LEN && dFeed2 < MIN_LEN)
				{
					pLine->m_fFeedScale = 1.0;
				}
				else
				{
					dFeedRate = GetPathEntityFeedRate(pEnt, m_cSpeedDef);
					pLine->m_fFeedScale = float( max( dFeed1, dFeed2 ) / dFeedRate ) ;
				}				
			}
		}
		break ;
	case NC_PATH_PLINE3D :
		{
			CPathPLine3D *pPLine = (CPathPLine3D*) pEnt ;
			dFeed1 = -1.0;
			for( int i = 0 ;  i <= pPLine->m_nCount ; i ++ ) 
			{
				SnapNearestPointNew( CurveList,pPLine->m_pTAPos[i], t, LocFrame, bKeep, dKeep ) ;
				if( bKeep[0] )
					pPLine->m_pTAPos[i][2] += dKeep[0] ;
				if( bKeep[1] )
				{
					if (dKeep[1] > dFeed1)
					{
						dFeed1 = dKeep[1];
					}				
				}
			}
			if( bKeep[1] )
			{
				if (dFeed1 < MIN_LEN)
				{
					pPLine->m_fFeedScale = 1.0;
				}
				else
				{
					dFeedRate = GetPathEntityFeedRate(pEnt, m_cSpeedDef);
					pPLine->m_fFeedScale = float( dFeed1 / dFeedRate ) ;
				}					
			}
		}
		break ;
	case NC_PATH_ARC3D :
		{
			TPNT3D dStart, dEnd ;
			CPathArc3D *pArc3D = (CPathArc3D*) pEnt ;
			pArc3D->GetEndPoint( 0, dStart ) ;
			pArc3D->GetEndPoint( 1, dEnd   ) ;
			// 起点高度和速度
			SnapNearestPointNew( CurveList,dStart, t, LocFrame, bKeep, dKeep ) ;
			if( bKeep[0] )
				dStart[2] += dKeep[0] ;
			if( bKeep[0] ) dFeed1 = dKeep[1] ;

			if (pEnt->m_bFeedType != JDNC_FEEDTYPE_ROUGH)
			{
				t[0] = t[1] = -1.0;
			}			
			// 末点高度和速度
			SnapNearestPointNew( CurveList,dEnd  , t, LocFrame, bKeep, dKeep ) ;
			if( bKeep[0] )
				dEnd[2]   += dKeep[0] ;
			if( bKeep[1] ) 
			{
				dFeed2 = dKeep[1] ;
				if (dFeed1 < MIN_LEN && dFeed2 < MIN_LEN)
				{
					pArc3D->m_fFeedScale = 1.0;
				}
				else
				{
					dFeedRate = GetPathEntityFeedRate(pEnt, m_cSpeedDef);
					pArc3D->m_fFeedScale = float( max( dFeed1, dFeed2 ) / dFeedRate ) ;
				}				
			}
			pArc3D->m_fCenter[2] = dStart[2] ;
			pArc3D->m_fDepth = dEnd[2] - dStart[2] ;
		}
		break ;
	default :
		ASSERT( 0 ) ;
		break ;
	}
	return TRUE ;
}

BOOL CSmartPathGen::IsLoopCutSameOffset()
{

	DOUBLE dAngle[2]; 
	dAngle[0] = ANGLE_TO_RADIAN( m_cShapeDef.m_cTaper.m_dBoundAngle  ) ;
	dAngle[1] = ANGLE_TO_RADIAN( m_cShapeDef.m_cTaper.m_dIslandAngle ) ;
	if (m_cMethodDef.m_nMethodType == surfncMethodLoop)
	{
		if( m_cMethodDef.m_cLoopCut.m_nRCompSide ==  2 )
		{ /* 关闭补偿 */
			return TRUE ;
		}
		else if(m_cMethodDef.m_cLoopCut.m_bLoopFlag & NCDEF_FLOOP_USERRCOMP ) 
		{ /* 定义补偿值 */
			return TRUE ;
		}
		else if( m_cShapeDef.m_bAntiFace && dAngle[0] > 0.001 )
		{/* 底面效果 */
			if( m_cFeedDef.m_cLayerDef.m_bLayerFlag & NCDEF_LAYER_COPYMODE )
			{
				return TRUE ;
			}
		}
		else if( m_cFeedDef.m_cLayerDef.m_bLayerFlag & NCDEF_LAYER_COPYMODE )
		{ /*  拷贝分层  */
			return TRUE ;
		}
		else if( dAngle[0] < 0.001 && m_cToolDef.m_nToolType == surfncToolFlat )
		{
			return TRUE ;
		}
		else if (m_cFeedDef.m_cLayerDef.m_nLayerCount == 1)
		{/*只有一层路径*/
			return TRUE;
		}
	}

	return FALSE ;
}

BOOL CSmartPathGen::RedepthCPathCombine (CPtrList & CurveList, CPathCombine * PComb, BOOL bKeep[2], RFRAME & LocFrame)
{
	if (!PComb || CurveList.IsEmpty())
	{
		return FALSE;
	}

	CPathEntity *pEnt = NULL, *pNext = NULL, *pPrev = NULL ;
	int nType = 0, nCnt = 0, nNum = 0, nSize = 2000, i = 0, j = 0, k = 0 ;
	CPathPLine3D *pSectPLine = NULL ;
	TPNT3D /*start, end, */Buff[2000] ;
	double dArcTol = m_cSetupDef.m_cTolDef.m_dArcTol ;
	double dAngTol = ANGLE_TO_RADIAN(m_cSetupDef.m_cTolDef.m_dAngTol) ;
	double dLen = 0., t1 = 0., t2 = 0. ;
	double dMaxStep = m_c5DCtrlDef.m_cToolAxisApp.m_dMaxStepLen, dDMaxStep = dMaxStep * 2.0;
	// STEP 1 : 遍历CurveList搜索是否含有速度曲线
	BOOL bHasFeedCur = FALSE ;
	CRedepthCurve *pDepCurve = NULL ;
	CGeoCurve *pCurve3D = NULL ;
	POSITION pos = CurveList.GetHeadPosition() ;
	while( pos )
	{
		pDepCurve = ( CRedepthCurve *) CurveList.GetNext( pos ) ;
		if( ! pDepCurve ) continue ;
		pCurve3D = pDepCurve->m_pCurve3D ;
		
		if( pCurve3D && pCurve3D->GetType() == OBJ3D_CURVE_CURVE5AXIS )
		{
			CGeoCurve5ax *p5axCur = ( CGeoCurve5ax *)pCurve3D ;
			if( p5axCur->m_nCurveType != CURVE5DTYPE_COMMON )
			{
				bHasFeedCur = TRUE ;
				break;
			}
		}
	}
	// STEP 2 : 如果读取曲线速度，将PComb离散
	if( bKeep[1] && bHasFeedCur )
	{
		pEnt = PComb->m_pHead ;
		while( pEnt )
		{
			pNext = pEnt->next ;
			nType = pEnt->GetType () ;
			if( nType == NC_PATH_PLINE3D )
			{
				CPathPLine3D *pPLine = ( CPathPLine3D *)pEnt ;
				nCnt = pPLine->m_nCount ;
				pPrev = pEnt ;
				for( i = 0 ; i < nCnt ; i++ )
				{
					dLen = mathDist( pPLine->m_pTAPos[i], pPLine->m_pTAPos[i+1] ) ;
					if( dLen > dDMaxStep )
					{
						nNum = int( dLen / dMaxStep + 0.5 ) ;
						for( j = 0 ; j < nNum ; j++ )
						{
							pSectPLine = new CPathPLine3D ;
							pSectPLine->m_nCount = 1;
							pSectPLine->m_pTAPos = new TPNT3D[2];

							t1 = j * 1. / nNum ;
							t2 = ( j + 1 ) * 1. / nNum ;
							for( k = 0 ; k < 3 ; k++ )
							{
								pSectPLine->m_pTAPos[0][k] = pPLine->m_pTAPos[i][k] * ( 1 - t1 ) + t1 * pPLine->m_pTAPos[i+1][k] ;
								pSectPLine->m_pTAPos[1][k] = pPLine->m_pTAPos[i][k] * ( 1 - t2 ) + t2 * pPLine->m_pTAPos[i+1][k] ;
							}
							pSectPLine->m_bMoveFlag = pEnt->m_bMoveFlag ;
							pSectPLine->m_bFeedType = pEnt->m_bFeedType ;
							PComb->InsertAfter( pSectPLine, pPrev ) ;
							pPrev = pSectPLine ;
						}
					}
					else
					{
						pSectPLine = new CPathPLine3D ;
						pSectPLine->m_nCount = 1;
						pSectPLine->m_pTAPos = new TPNT3D[2];
						mathCpyPnt( pPLine->m_pTAPos[i], pSectPLine->m_pTAPos[0] ) ;
						mathCpyPnt( pPLine->m_pTAPos[i+1], pSectPLine->m_pTAPos[1] ) ;
						pSectPLine->m_bMoveFlag = pEnt->m_bMoveFlag ;
						pSectPLine->m_bFeedType = pEnt->m_bFeedType ;
						PComb->InsertAfter( pSectPLine, pPrev ) ;
						pPrev = pSectPLine ;
					}
				}

				PComb->RemoveEntity ( pEnt ) ;
				delete pEnt ;
			}
			else if( nType == NC_PATH_LINE3D )
			{ // 长直线也需要离散
				CPathLine3D *pLine3D = ( CPathLine3D *)pEnt ;
				dLen = pLine3D->GetLength () ;
				if( dLen > dDMaxStep )
				{
					pPrev = pEnt ;
					nCnt = int( dLen / dMaxStep + 0.5 ) ;
					for( i = 0 ; i < nCnt ; i++ )
					{
						pSectPLine = new CPathPLine3D ;
						pSectPLine->m_nCount = 1;
						pSectPLine->m_pTAPos = new TPNT3D[2];

						t1 = i * 1. / nCnt ;
						t2 = ( i + 1 ) * 1. / nCnt ;
						for( k = 0 ; k < 3 ; k++ )
						{
							pSectPLine->m_pTAPos[0][k] = pLine3D->m_fStart[k] * ( 1 - t1 ) + t1 * pLine3D->m_fEnd[k] ;
							pSectPLine->m_pTAPos[1][k] = pLine3D->m_fStart[k] * ( 1 - t2 ) + t2 * pLine3D->m_fEnd[k] ;
						}

						pSectPLine->m_bMoveFlag = pEnt->m_bMoveFlag ;
						pSectPLine->m_bFeedType = pEnt->m_bFeedType ;
						PComb->InsertAfter( pSectPLine, pPrev ) ;
						pPrev = pSectPLine ;
					}
					PComb->RemoveEntity ( pEnt ) ;
					delete pEnt ;
				}
			}
			else if( nType == NC_PATH_ARC3D )
			{
				CPathArc3D *pArc = ( CPathArc3D *)pEnt ;
				pPrev = pEnt ;
				nCnt = pArc->Discrete( dArcTol, dAngTol, Buff, nSize ) ;
				for( i = 0 ; i < nCnt ; i++ )
				{
					dLen = mathDist( Buff[i], Buff[i+1]);
					if (dLen > dDMaxStep)
					{
						nNum = int( dLen / dMaxStep + 0.5 ) ;
						for( j = 0 ; j < nNum ; j++ )
						{
							pSectPLine = new CPathPLine3D ;
							pSectPLine->m_nCount = 1;
							pSectPLine->m_pTAPos = new TPNT3D[2];

							t1 = j * 1. / nNum ;
							t2 = ( j + 1 ) * 1. / nNum ;
							for( k = 0 ; k < 3 ; k++ )
							{
								pSectPLine->m_pTAPos[0][k] = Buff[i][k] * ( 1 - t1 ) + t1 * Buff[i+1][k] ;
								pSectPLine->m_pTAPos[1][k] = Buff[i][k] * ( 1 - t2 ) + t2 * Buff[i+1][k] ;
							}
							pSectPLine->m_bMoveFlag = pEnt->m_bMoveFlag ;
							pSectPLine->m_bFeedType = pEnt->m_bFeedType ;
							PComb->InsertAfter( pSectPLine, pPrev ) ;
							pPrev = pSectPLine ;
						}
					}
					else
					{
						pSectPLine = new CPathPLine3D ;
						pSectPLine->m_nCount = 1;
						pSectPLine->m_pTAPos = new TPNT3D[2];
						mathCpyPnt( Buff[i], pSectPLine->m_pTAPos[0] ) ;
						mathCpyPnt( Buff[i+1], pSectPLine->m_pTAPos[1] ) ;
						pSectPLine->m_bFeedType = pEnt->m_bFeedType ;
						pSectPLine->m_bMoveFlag = pEnt->m_bMoveFlag ;
						PComb->InsertAfter( pSectPLine, pPrev ) ;
						pPrev = pSectPLine ;
					}
				}
				PComb->RemoveEntity ( pEnt ) ;
				delete pEnt ;
			}
//			else if( nType == NC_PATH_NURBS3D )
//			{
//				CPathNurbs3D *pNurbs = ( CPathNurbs3D *)pEnt ;
//				PComb->RemoveEntity ( pEnt ) ;
//			}
			else
			{
			}

			pEnt = pNext ;
		}
	}
	// STEP 3 : 更新速度和高度
	for(  pEnt = PComb->m_pHead ; pEnt ; pEnt = pEnt->next )
	{
		RedepthPathEntityNew( CurveList, pEnt, bKeep, LocFrame ) ;
	}

	return TRUE;
}

BOOL CSmartPathGen::RedepthNewPath( CPathGroup& NewPath,  /*雕刻路径*/ 
                                    CSmartGraphic& Graph,  /*图形函数*/   
									BOOL	bKeep[2],		// <I> [0]高度,[1]速度
									DOUBLE& dTop )  /*最大高度*/
{
	CPtrList    Curve3D ;
	Graph.GetAllEntity( m_cSetupDef, 
		                Curve3D, 
		                SMARTGRAPH_TYPE_CURVE2D ) ;
	if( Curve3D.GetCount() == 0 ) 
	{
		return TRUE ;
	}
	RFRAME LocFrame ;
	if( !Graph.GetTPlaneFrame( LocFrame ) )
	{
		mathInitRFrame( &LocFrame ) ;
	}

	if (m_c5DCtrlDef.m_cToolAxisApp.m_nFrameFlag != NCDEF_FRAME_DEFAULT)
	{
		mathMultRFrame( &LocFrame, &m_c5DCtrlDef.m_cToolAxisApp.m_dIndexFrame, &LocFrame ) ;
	}

	// 防止保留曲线高度时，进退刀路径或下刀路径高度错误，edit by liuxin, 2013.9.24
	for( CPathCombine* pComb = NewPath.m_pHead ; pComb ; pComb = pComb->next )
	{
		RedepthCPathCombine(Curve3D, pComb, bKeep, LocFrame);
	}
/*	POSITION pos = Curve3D.GetHeadPosition() ;
	while( pos )
	{
		CSmartCurve* pCurve = (CSmartCurve*) Curve3D.GetNext(pos) ;
		for( CSmartSect* pSect = pCurve->GetHead(); pSect ; pSect = pSect->next )
		{
			if( pSect->m_dZValue[0]  > dTop ) dTop = pSect->m_dZValue[0] ;
			if( pSect->m_dZValue[1]  > dTop ) dTop = pSect->m_dZValue[1] ;
		}
		if( pCurve ) delete pCurve ;
	}*/
		
//	CModel *pModel = glbf_GetModel() ;
//	pModel->BeginUndo ( "Test_AddCurve") ;
	BOX3D box ;
	POSITION pos, atpos = NULL ;
	pos = Curve3D.GetHeadPosition() ;
	while( pos )
	{
		atpos = pos ;
		CRedepthCurve *pCurve = ( CRedepthCurve *)Curve3D.GetNext ( pos ) ;
		if( pCurve->m_pCurve3D )
		{
			mathTransWorldBox3D( &m_dNcMtx, &pCurve->m_pCurve3D->m_dBox, &box ) ;
			if( box.max[2] > dTop )
				dTop = box.max[2] ;
		}
//		CEttCurve *pEttCurve = new CEttCurve( pCurve->m_pCurve2D->CopyMyself () ) ;
//		pModel->UD_AddEntity ( pEttCurve ) ;
		delete pCurve ;
		Curve3D.RemoveAt ( atpos ) ;
	}
//	pModel->EndUndo () ;
	return TRUE ;
}

// 重新确定边界高度
CSmartLoop*  CSmartPathGen::RedefineBoundLoops( CSmartLoop* AllCont ) 
{
	if( AllCont == NULL ) return AllCont ;
	double dOffset = 0.0 ;
	if( ( m_cShapeDef.m_bShapeFlag & NCDEF_SHAPE_ZEROBASED ) &&	m_dTopHeight < -0.001 )
	{ /*外轮廓高度置零*/
	    double dAngle = ANGLE_TO_RADIAN( m_cShapeDef.m_cTaper.m_dBoundAngle  ) ;
		dOffset = m_pTool->GetRadiusComp( 0.0, dAngle ) ;
		dOffset = m_pTool->GetRadiusComp( fabs(m_dTopHeight), dAngle ) - dOffset ;
	}
	if( dOffset < 0.001 ) return AllCont ;
	CPtrArray tmpArr ;
	CSmartLoop* pNewCont, *pLoop ; 
	for( pLoop = AllCont ; pLoop ; pLoop = pLoop->next )
	{
		if( MathCAM_IsOutMostLoop( AllCont, pLoop ) )
		{
			tmpArr.Add( pLoop ) ;
		}
	}
	if( tmpArr.GetSize() == 0 ) return AllCont ;
	for( int k = 0 ; k < tmpArr.GetSize() ; k ++ )
	{
		pLoop = ( CSmartLoop*) tmpArr.GetAt( k ) ;
		AllCont = Mini_RemoveLoop( AllCont, pLoop ) ;
		pNewCont = pLoop->OffsetContour( NCDEF_OFFSET_INNER, dOffset, 0.0, GetCorDef() ) ;
		AllCont = Mini_AddContours( AllCont, pNewCont ) ;
		delete pLoop ;
	}
	return AllCont ;
}
// 自动分层
int SortStepZ( const void *arg1, const void *arg2 ) 
{
	if( *((double*)arg2) == *((double* )arg1))
		return 0 ;

	return *((double* )arg2) > *((double*)arg1) ? -1 : 1 ;
}
void CSmartPathGen::GetAllSurfLayer ( double *dSurfZ, INT_PTR nNum, double dStepZ, double*& dZ, INT_PTR& nCount )
{
	double dTop = GetTopHeight() ;
	double dBot = GetBottomHeight() ;
	
	nCount = nNum + 2 ;
	dZ = new double[nCount] ;
		
	// STEP 1 : 寻找平面
	INT_PTR i, j, k ;
	nCount = 0 ;
	
	dZ[0] = dBot ;
			
	for( i = 0 ; i < nNum ; i++ )
	{
		if( fabs( dBot - dSurfZ[i] ) < m_cSetupDef.m_cTolDef.m_dArcTol && dZ[0] < dTop )
			dZ[0] = dZ[0] + m_cSetupDef.m_cTolDef.m_dArcTol ;
		else if( dSurfZ[i] > dBot && dSurfZ[i] < dTop )
		{ 
			nCount++ ;
			dZ[nCount] = dSurfZ[i] ;
		}
	}
	
	nCount++ ;
	dZ[nCount] = dTop ;
	nCount++ ;

	// STEP 2 : 并按照高度递增的顺序排好
	qsort( dZ, nCount, sizeof( double ), SortStepZ ) ;
	// 处理最低层
	INT_PTR nFind = -1 ;
	
	// 设定最小层高
	double dMinStepZ = dStepZ / 4 ;
	if( dZ[1]-dZ[0]<dMinStepZ ) dZ[0]=dZ[1];

	// STEP 3 : 将高度相同的平面和上下高度差小于dMinZ的过滤
	for( i = 0 ; i < nCount - 1 ; i++ )
	{
		nFind = -1 ;
		for( j = i+1 ; j < nCount ; j++ )
		{
			if( dZ[j] - dZ[i] >= dMinStepZ )
			{
				nFind = j ;
				break ;
			}
		}
		if( nFind > i+1 )
		{// 如果找到
			for( k = nFind ; k < nCount ; k++ )
			{
				dZ[i+1+k-nFind] = dZ[k] ;
			}
			nCount = nCount - ( nFind - i - 1 ) ;
		}
		else if( nFind < 0 )
		{// 找不到,则跳到最后一层
			dZ[i+1] = dZ[nCount-1] ;
			nCount = i+2 ;
			break ;
		}
	}
	
	// 然后分层,先估算分几层
	int n, nLayer = 0 ;
	
	for( i = 1 ; i < nCount ; i++ )
	{
		n = ( int ) ceil ( ( dZ[i] - dZ[i-1] )/ dMinStepZ ) ;
		nLayer += n ;
	}
	double *tmpZ = new double[nLayer+5] ;
	double tmpH = 0. ;
	nLayer = 0 ;
	tmpZ[0] = dZ[0] ;
	// 开始分
	for( i = 1 ; i < nCount ; i++ )
	{
		tmpH = dZ[i] - dZ[i-1] ;
		if( tmpH <= dStepZ )
		{
			nLayer++ ;
			tmpZ[nLayer] = dZ[i] ;
		}
		else
		{
			n = ( int ) ceil( tmpH / dStepZ - 0.001) ;
			tmpH = tmpH / n ;
			for( j = 1 ; j <= n ; j++ )
			{
				nLayer++ ;
				tmpZ[nLayer] = dZ[i-1] + j * tmpH ;
			}
		}
	}
	delete[] dZ ;
	nCount = nLayer + 1 ;
	dZ = new double[nCount] ;
	for( i = 0 ; i < nCount ; i++ )
		dZ[i] = tmpZ[nCount-i-1] ;
	delete[] tmpZ ;

	// 等高加工如果只有两层,选择最高层
	if( nCount == 2 && ( dZ[0] - dZ[1] ) < dStepZ )
	{
		nCount = 1 ;
	}
	else if( nCount > 2 )
	{
		// 如果第一层和最高处查不多,去掉第一层
		double del = dTop - dZ[0] ;
		if( fabs(del) < 0.005 )
		{
			for( i = 1 ; i < nCount ; i++ )
				dZ[i-1] = dZ[i] ;
			nCount -= 1 ;
		}
	}
}

void CSmartPathGen::Add3DSafeQuickPath(CPathGroup &NewPath)
{
	for( CPathCombine* pTComb = NewPath.m_pHead ; pTComb ; pTComb = pTComb->next )
	{/*添加安全高度*/
		if( ! pTComb->m_pHead )
		{
			ASSERT( 0 ) ;
			continue ;
		}
		TPNT3D dStart, dEnd ;
		CPathLine3D* pLine3D = NULL ;
		if( pTComb->GetEndPoint( 0, dEnd ) && 
			NewPath.m_fSafeHeight - dEnd[2] > 0.01   )
		{
			nc_VectorCopy( dStart, dEnd , 2 ) ;
			dStart[2] = NewPath.m_fSafeHeight ;
			pLine3D = new CPathLine3D( dStart, dEnd ) ;
			pLine3D->m_bFeedType = JDNC_FEEDTYPE_QUICK ;
			pTComb->InsertAfter( pLine3D , NULL ) ;
		}
		BOOL bRet = pTComb->GetEndPoint ( 1, dStart ) ;
		if( bRet && pTComb->m_pTail->GetType () == NC_PATH_POINT3D )
		{
			int n = ( ( CPathPoint3D *)pTComb->m_pTail )->m_nIncTime - 1 ;
			dStart[2] -= ( ( CPathPoint3D *)pTComb->m_pTail)->m_fIncAt[n][2] ;
		}
		if( bRet && NewPath.m_fSafeHeight - dStart[2] > 0.01   )
		{
			nc_VectorCopy( dEnd, dStart , 2 ) ;
			dEnd[2] = NewPath.m_fSafeHeight ;
			pLine3D = new CPathLine3D( dStart, dEnd ) ;
			pLine3D->m_bFeedType = JDNC_FEEDTYPE_QUICK ;
			pTComb->AddEntity( pLine3D ) ;
		}
	}
}

void CSmartPathGen::AddPlungeAndRelativeH ( CSmtCheckMdl *DriveMdl, CPathGroup *NewPath, BOOL bConnect, double *dZ /*= NULL*/, INT_PTR nCount /*= 0*/  )
{
	if( !DriveMdl || !NewPath || !NewPath->m_pHead ) return ;
	// 首先合并NewPath
	CPathCombine *PCombHead = NULL, *PCombNext = NULL ;
	while( NewPath->m_pHead && NewPath->m_pHead->next )
	{
		PCombNext = NewPath->m_pHead->next ;
		NewPath->RemoveCombine ( PCombNext ) ;
		NewPath->m_pHead->AppendCombine ( PCombNext ) ;
	}

	CPathEntity *pHead = NULL, *pNext = NULL ;
	CPathLine3D *pLine = NULL ;
	CPathPLine3D *pPLine = NULL ;
	PNT3D start, end;
	double dist = -1., h = 0. ;
	FPNT3D minpt, maxpt ;
	DriveMdl->CalcFacetBox ( minpt, maxpt ) ;
	double dTop = maxpt[2] ;
	BOOL bSuccess = FALSE ;

	PCombHead = NewPath->m_pHead ;
	// 只有一个CPathCombine 
	AddHeadPlungePath( PCombHead ) ;
	pHead = PCombHead->m_pHead ;
	if( !pHead ) return ;
	pHead->GetEndPoint ( 1, start ) ;
	while( pHead )
	{
		pNext = pHead->next ;
		if( !pNext ) break ;
		pNext->GetEndPoint ( 0, end ) ;
		dist = mathDist( start, end ) ;
		if( dist >= 0.0001 ) 
		{
			bSuccess = FALSE ;
			if( bConnect && dist < m_cFeedDef.m_cConnectDef.m_dMaxDist )
			{
				// 然后添加相对高度连刀
				bSuccess = AddConnectLine( DriveMdl, PCombHead, pHead, start, end ) ;
			}
			if( !bSuccess )
			{
				// 首先添加慢速下刀
				if( m_cSpeedDef.m_dPlungeDist > 0.01 )
				{
					PNT3D end1;
					mathCpyPnt( end, end1 ) ;
					end[2] += m_cSpeedDef.m_dPlungeDist ;
					pLine = new CPathLine3D( end, end1 ) ;
					pLine->m_bFeedType = JDNC_FEEDTYPE_PLUNGE ;
					PCombHead->InsertAfter ( pLine, pHead ) ;
				}
				// 然后添加相对高度连刀
				h = CheckRelativeQuickH( DriveMdl, start, end, dZ, nCount, dTop ) ;
				pPLine = CreateRelativeQuick( start, end, h ) ;
				if( pPLine )
					PCombHead->InsertAfter ( pPLine, pHead ) ;
			}
		}
		pNext->GetEndPoint ( 1, start ) ;
		pHead = pNext ;
	}
	
}

void CSmartPathGen::AddHeadPlungePath ( CPathCombine *PComb )
{
	if( !PComb || !PComb->m_pHead || m_cSpeedDef.m_dPlungeDist < 0.01 ) return ;
	PNT3D start, end ;
	PComb->m_pHead->GetEndPoint ( 0, end ) ;
	mathCpyPnt( end, start ) ;
	start[2] += m_cSpeedDef.m_dPlungeDist ;
	CPathLine3D *pLine = new CPathLine3D( start, end ) ;
	pLine->m_bFeedType = JDNC_FEEDTYPE_PLUNGE ;
	// 添加到头部
	CPathEntity *pHead = PComb->m_pHead ;
	pHead->prev = pLine ;
	pLine->next = pHead ;
	pLine->prev = NULL  ;
	PComb->m_pHead = pLine ;
}

double CSmartPathGen::CheckRelativeQuickH ( CSmtCheckMdl *DriveMdl, PNT3D start, PNT3D end, double *dZ, INT_PTR nCount, double dTop )
{
	PNT3D p1, p2 ;
	mathCpyPnt( start, p1 ) ;
	mathCpyPnt( end  , p2 ) ;
    p1[2] = p2[2] = max( p1[2], p2[2] ) ;  
	CSmtCutPath  tmpPath( MINI_CONNECT_PATH ) ;
    tmpPath.AddPoint( p1 ) ;
    tmpPath.AddPoint( p2 ) ;
	tmpPath.InsertCPoint ( m_cSetupDef.m_cTolDef.m_dMaxStep ) ;
	JDNC_PRGDEF cPrgDef ;
	SurfNC_InitPathParam( cPrgDef ) ;
	tmpPath.VerifyLinePath ( *DriveMdl, m_cSetupDef.m_cTolDef, cPrgDef ) ;
    tmpPath.DefineBox() ;
    double h = tmpPath.m_fBox[1][2], dH = GetTopHeight() ;
	// 将h抬刀到当前层的上层
	if( dZ )
	{
		if( h >= (dZ[0]-0.002) ) 
		{
			if( h > dH )	h = max( h , dTop ) ;
			else			h = max( h , dH   ) ;
		}
		else
		{
			for( int i = 1 ; i < nCount ; i++ )
			{
				if( (dZ[i]-0.002) <= h )
				{
					h = dZ[i-1] ;
					break ;
				}
			}
		}
	}	

	if (m_cSpeedDef.m_nRapidMode == NCDEF_RAPIDMODE_ABSOLUTE)
	{// 相对毛胚高度时
		if (m_pStockBox != NULL)
		{// 使用毛胚时取二者最大值
			dH = max(dH, m_pStockBox->max[2]);
		}
		h  = max(dH, h);
	}

	h += m_cSpeedDef.m_dRapidHeight ;
	
	return h ;
}

CPathPLine3D *CSmartPathGen::CreateRelativeQuick ( PNT3D start, PNT3D end, double h )
{
	CPathPLine3D *pLine = new CPathPLine3D() ;
	pLine->m_nCount = 3 ;
	pLine->m_pTAPos = new PNT3D[pLine->m_nCount+1] ;
	PNT3D start1, end1 ;
	mathCpyPnt( start, start1 ) ;
	mathCpyPnt( end, end1 ) ;
	start1[2] = h ;
	end1[2]   = h ;
	mathCpyPnt( start , pLine->m_pTAPos[0] ) ;
	mathCpyPnt( start1, pLine->m_pTAPos[1] ) ;
	mathCpyPnt( end1  , pLine->m_pTAPos[2] ) ;
	mathCpyPnt( end   , pLine->m_pTAPos[3] ) ;
	pLine->m_bFeedType = JDNC_FEEDTYPE_QUICK ;
	return pLine ;
}
BOOL CSmartPathGen::AddConnectLine ( CSmtCheckMdl *DriveMdl, CPathCombine *PComb, CPathEntity *pEntity, PNT3D start, PNT3D end )
{
	CSmtCutPath  tmpPath( MINI_CONNECT_PATH ) ;
    tmpPath.AddPoint( start ) ;
    tmpPath.AddPoint( end   ) ;

	JDNC_PRGDEF cPrgDef ;
	cPrgDef.m_dLimitAt = 0. ;
	cPrgDef.m_dStepAt  = 0 ;
	cPrgDef.m_dIncStep = 1. ;
	cPrgDef.m_pBrkFunc = NULL ;
	cPrgDef.m_pPrgFunc = NULL ;
	cPrgDef.m_pPosFunc = NULL ;
	// 插入节点并使用CheckMdl
	tmpPath.InsertCPoint ( m_cSetupDef.m_cTolDef.m_dMaxStep ) ;
	if( tmpPath.m_nNumPnt < 4 )
		tmpPath.InsertCPoint ( m_cSetupDef.m_cTolDef.m_dMaxStep * 0.3 ) ;
	tmpPath.VerifyCutPath( *DriveMdl, m_cSetupDef.m_cTolDef, cPrgDef ) ;

	// 逐点判断是否合适
	BOOL bConnect = TRUE ;
	double dLen2D = mathDist2D( start, end ), t = 0. ;
	double dDist2D = 0., dTmpH = 0. ;
	PNT3D tmp ;
	CSmtCutPoint *pHead = tmpPath.m_pHead->next ;
	for( ; pHead ; pHead = pHead->next )
	{
		if( pHead == tmpPath.m_pTail )
			break ;
		tmp[0] = pHead->m_fPoint[0] ;
		tmp[1] = pHead->m_fPoint[1] ;
		tmp[2] = pHead->m_fPoint[2] ;
		dDist2D = mathDist2D( tmp, start ) ;
		t = dDist2D / dLen2D ;
		dTmpH = start[2] + t * ( end[2] - start[2] ) ;
		if( tmp[2] > dTmpH + m_cSetupDef.m_cTolDef.m_dArcTol )
		{
			bConnect = FALSE ;
			break ;
		}
	}
	
	if( bConnect )
	{
		CPathPLine3D *pLine = new CPathPLine3D() ;
		pLine->m_nCount = 1 ;
		pLine->m_pTAPos = new PNT3D[pLine->m_nCount+1] ;
		mathCpyPnt( start , pLine->m_pTAPos[0] ) ;
		mathCpyPnt( end   , pLine->m_pTAPos[1] ) ;
		pLine->m_bFeedType = JDNC_FEEDTYPE_CONNECT ;
		PComb->InsertAfter ( pLine, pEntity ) ;
		return TRUE ;
	}
	
	return FALSE ;
}

void CSmartPathGen::AddSlowPlungePath(CPathGroup *NewPath, CSmartGraphic &Graph)
{
	if (!NewPath)
		return ;
	switch (m_cMethodDef.m_nMethodType)
	{
	case surfncMethodCurve:
	case surfncMethodLoop:
	case surfncMethodRemain:
	case surfncMethodBound:
	case surfncMethodCorner:
	case surfncMethodPocket:
	case surfncMethodCurveSlot:
		{
			AddPlungeAndRelativeH(NewPath, Graph);
		}
		break;
	case surfncMethodCurve5D:
		{// 眼镜加工五轴曲线路径没有添加慢速下刀路径
			if (glbf_GetProductID() != PRODUCT_ID_GLASSES)
			{
				AddSlowPlungePathFor5DPath(NewPath);
			}
		}
		break;
	case surfncMethodRotate45D:
	case surfncMethodCleanup5D:
	case surfncMethodSGuide5D:
	case surfncMethodSMorph5D:
	case surfncMethodCMorph5D:
	case surfncMethodSwarf5D:
	case surfncMethodPort5D:
	case surfncMethodBlisk5D:
	case surfncMethodPocket5D:
	case surfncMethodImpeller5D:
		{
			AddSlowPlungePathFor5DPath(NewPath);
		}
		break;
	default:
		break;
	}
}
/*
CSmtCutPath* CSmartPathGen::Cal5DSafeQuickPathPoint(PNT3D point,VEC3D dir)
{

		int nSafeMode = m_c5DCtrlDef.m_pCtrlParam->m_cClearance.m_nSafeMode ;
		double dMaxAngStep = ANGLE_TO_RADIAN ( m_c5DCtrlDef.m_cToolAxis.m_dMaxAngleStep ) ;
		CNc5DQuickLink cQuickLink ; 
		CSmtCutPath* pQuickPath = NULL;

		VEC3D vCldAxis = {0,0,1};
		int nCldAxis = 2 ; //nCldAxis为轴向
		
		if (nSafeMode == NCDEF_CLEARANCE_CYLINDER)
		{
			if ( m_c5DCtrlDef.m_pCtrlParam->m_cClearanceApp.m_nAxisSel == NCDEF_AXISSEL_X )
			{
				nCldAxis = 0 ;
				vCldAxis[0] = 1;vCldAxis[1] = 0;vCldAxis[2] = 0;
			}
			else if ( m_c5DCtrlDef.m_pCtrlParam->m_cClearanceApp.m_nAxisSel == NCDEF_AXISSEL_Y )
			{
				nCldAxis = 1 ;
				vCldAxis[0] = 0;vCldAxis[1] = 1;vCldAxis[2] = 0;
			}	
			
			mathUniVec(dir,MIN_LEN);
			mathUniVec(vCldAxis,MIN_LEN);
			if (mathIsParallel(dir,vCldAxis,ANGLE_TO_RADIAN(10)) == ID_YES)//若安全定位路径刀轴方向与柱面轴向平行(两者夹角小于10度)，则转为自动模式计算
			{
				nSafeMode = NCDEF_CLEARANCE_AUTO;
			}				
		}

		if ( nSafeMode == NCDEF_CLEARANCE_SPHERE )
		{
			pQuickPath = cQuickLink.Gen5XApproachQuickPathSphere(point,dir,NULL,NULL,
				                                                 0,dMaxAngStep,5,
																 SUP_DBL,
				                                                 m_c5DCtrlDef.m_pCtrlParam->m_cClearance.m_dCenter,
				                                                 2,
				                                                 m_c5DCtrlDef.m_pCtrlParam->m_cClearance.m_dSafeRad);
		}
		else if (nSafeMode == NCDEF_CLEARANCE_CYLINDER)
		{
			pQuickPath = cQuickLink.Gen5XApproachQuickPathCylinder(point,dir,
				                                                   NULL,NULL,
				                                                   0,dMaxAngStep,
				                                                   5.0,
				                                                   SUP_DBL,
				                                                   m_c5DCtrlDef.m_pCtrlParam->m_cClearance.m_dCenter,
												                   nCldAxis,
													               m_c5DCtrlDef.m_pCtrlParam->m_cClearance.m_dSafeRad );
		}
		else if (nSafeMode == NCDEF_CLEARANCE_AUTO)
		{
			pQuickPath = cQuickLink.Gen5XApproachQuickPathAuto ( point, dir,
				                                                 NULL,NULL,
				                                                 0,dMaxAngStep,
				                                                 m_c5DCtrlDef.m_pCtrlParam->m_cClearance.m_dSafeHeight,
			                                                     0);
           	                                                 
		}
		
		//若柱面或球面安全模式下未生成路径，转为自动安全模式
		if (nSafeMode != NCDEF_CLEARANCE_AUTO && pQuickPath == NULL)
		{
			pQuickPath = cQuickLink.Gen5XApproachQuickPathAuto ( point, dir,
				                                                 NULL,NULL,
				                                                 0,dMaxAngStep,
				                                                 m_c5DCtrlDef.m_pCtrlParam->m_cClearance.m_dSafeHeight,
				                                                 0);
		}

	pQuickPath->DelPointOverlap();

	return pQuickPath;
}
BOOL CSmartPathGen::Add5DSafeQuickPath(CPathGroup& pGroup)//xiejunxian
{
	if (!SurfNC_IsUse5DSafeModeForSurface(m_cMethodDef.m_nMethodType)) 
		return FALSE;                               //过滤三轴路径

	//////////////////////
	//安全模式下添加定位路径
	/////////////////////
	CPathEntity* pHead = (CPathEntity*)pGroup.m_pHead->m_pHead;
	CPathEntity* pTail = (CPathEntity*)pGroup.m_pTail->m_pTail;

	VEC3D dDir;
	CSmtCutPath* pQuickPath;
	BOOL bQuickPath = TRUE;//原始路径是否存在安全定位路径

	//step 1：添加下刀安全路径
	if (pGroup.m_nType == NC_PATHTYPE_ENGRAVE5AX)
	{
		nc_VectorCopy(dDir,pHead->m_pTDir->m_pTDir[0],3);
	}
	else
	{
		dDir[0] = 0;dDir[1] = 0;dDir[2] = 1;//五轴转三轴路径刀轴方向取为竖直向上
	}


	if (pHead->m_bFeedType == JDNC_FEEDTYPE_QUICK)
	{	
		if (pHead->GetType() == NC_PATH_LINE3D)
		{
			CPathLine3D* pH = (CPathLine3D*)pHead;
			pQuickPath = Cal5DSafeQuickPathPoint(pH->m_fEnd,dDir);
		}
		else if (pHead->GetType() == NC_PATH_PLINE3D)
		{
			CPathPLine3D* pH = (CPathPLine3D*)pHead;
			pQuickPath = Cal5DSafeQuickPathPoint(pH->m_pTAPos[pH->m_nCount],dDir);			
		}
	}
	else
	{
		TPNT3D fEnd;
		bQuickPath = FALSE;
		pHead->GetEndPoint(0,fEnd);
		pQuickPath = Cal5DSafeQuickPathPoint(fEnd,dDir);
	}

	if (!pQuickPath)return FALSE;
	
	CPathPLine3D* pHeadPLine = new CPathPLine3D;
	pHeadPLine->m_fFeedRate = pHead->m_fFeedRate;
	pHeadPLine->m_bMoveFlag = pHead->m_bMoveFlag;
	pHeadPLine->m_pTAPos = new TPNT3D [pQuickPath->m_nNumPnt];
	pHeadPLine->m_nCount = pQuickPath->m_nNumPnt - 1;

	if (pGroup.m_nType == NC_PATHTYPE_ENGRAVE5AX)
	{
		pHeadPLine->m_pTDir = new CNcToolDir;
		pHeadPLine->m_pTDir->m_nNumDir = pQuickPath->m_nNumPnt;
		pHeadPLine->m_pTDir->m_pTDir = new TPNT3D[pQuickPath->m_nNumPnt];
	}

	int n = 0;
	for (CSmtCutPoint*p=pQuickPath->m_pHead; p; p=p->next)
	{
		nc_VectorCopy(pHeadPLine->m_pTAPos[n],p->m_fPoint,3);
		if (pGroup.m_nType == NC_PATHTYPE_ENGRAVE5AX)
		{
			nc_VectorCopy(pHeadPLine->m_pTDir->m_pTDir[n],pHead->m_pTDir->m_pTDir[0],3);
		}		
		n++;
	}
	if (bQuickPath)
	{
		pHeadPLine->m_bFeedType = pHead->m_bFeedType;
		pGroup.m_pHead->RemoveEntity(pHead);	
		delete pHead;
		pHead = NULL;
	}
	else
	{
		pHeadPLine->m_bFeedType = JDNC_FEEDTYPE_QUICK;
	}
	pGroup.m_pHead->InsertAfter(pHeadPLine , NULL ) ;
	delete pQuickPath;
	pQuickPath = NULL;

	
	//step 2：添加抬刀安全路径
	bQuickPath = TRUE;
	if (pGroup.m_nType == NC_PATHTYPE_ENGRAVE5AX)
	{
		nc_VectorCopy(dDir,pTail->m_pTDir->m_pTDir[0],3);
	}
	else
	{
		dDir[0] = 0;dDir[1] = 0;dDir[2] = 1;//五轴转三轴路径刀轴方向取为竖直向上
	}

	if (pTail->m_bFeedType == JDNC_FEEDTYPE_QUICK)
	{	
		if (pTail->GetType() == NC_PATH_LINE3D)
		{
			CPathLine3D* pT = (CPathLine3D*)pHead;
			pQuickPath = Cal5DSafeQuickPathPoint(pT->m_fEnd,dDir);
		}
		else if (pTail->GetType() == NC_PATH_PLINE3D)
		{
			CPathPLine3D* pT = (CPathPLine3D*)pTail;
			pQuickPath = Cal5DSafeQuickPathPoint(pT->m_pTAPos[0],dDir);
		}
	}
	else
	{
		TPNT3D fStart;
		VEC3D dDir1 = {0,0,1};
		bQuickPath = FALSE;
		pTail->GetEndPoint(1,fStart);
		int nDir = pTail->m_pTDir->m_nNumDir;
		if (pGroup.m_nType == NC_PATHTYPE_ENGRAVE5AX)
		{
			nc_VectorCopy(dDir1,pTail->m_pTDir->m_pTDir[nDir-1],3);
		}
		pQuickPath = Cal5DSafeQuickPathPoint(fStart,dDir1);
	}
	
	if (!pQuickPath)return FALSE;

	pQuickPath->ReverseDirect();
	CPathPLine3D* pTailPLine = new CPathPLine3D;
	pTailPLine->m_fFeedRate = pTail->m_fFeedRate;
	pTailPLine->m_bMoveFlag = pTail->m_bMoveFlag;
	pTailPLine->m_bFeedType = pTail->m_bFeedType;
	pTailPLine->m_pTAPos = new TPNT3D [pQuickPath->m_nNumPnt];
	pTailPLine->m_nCount = pQuickPath->m_nNumPnt - 1;

	if (pGroup.m_nType == NC_PATHTYPE_ENGRAVE5AX)
	{
		pTailPLine->m_pTDir = new CNcToolDir;
		pTailPLine->m_pTDir->m_nNumDir = pQuickPath->m_nNumPnt;
		pTailPLine->m_pTDir->m_pTDir = new TPNT3D[pQuickPath->m_nNumPnt];
	}

	n = 0;
	for (CSmtCutPoint*p=pQuickPath->m_pHead; p; p=p->next)
	{
		nc_VectorCopy(pTailPLine->m_pTAPos[n],p->m_fPoint,3);
		if (pGroup.m_nType == NC_PATHTYPE_ENGRAVE5AX)
		{
			nc_VectorCopy(pTailPLine->m_pTDir->m_pTDir[n],pTail->m_pTDir->m_pTDir[0],3);
		}		
		n++;
	}

	if (bQuickPath)
	{
		pTailPLine->m_bFeedType = pTail->m_bFeedType;
		pGroup.m_pHead->RemoveEntity(pTail);	
		delete pTail;
		pTail = NULL;
	}
	else
	{
		pTailPLine->m_bFeedType = JDNC_FEEDTYPE_QUICK;
	}

	pGroup.m_pTail->AddEntity(pTailPLine) ;
	delete pQuickPath;
	pQuickPath = NULL;

	return TRUE;
}
*/
void CSmartPathGen::AddPlungeAndRelativeH(CPathGroup *NewPath, CSmartGraphic &Graph)
{
	if( !NewPath || !NewPath->m_pHead )
		return ;

	// 首先合并NewPath
	CPathCombine *PCombHead = NULL, *PCombNext = NULL ;
	while( NewPath->m_pHead && NewPath->m_pHead->next )
	{
		PCombNext = NewPath->m_pHead->next ;
		NewPath->RemoveCombine ( PCombNext ) ;
		NewPath->m_pHead->AppendCombine ( PCombNext ) ;
	}

	CPathEntity *pHead = NULL, *pNext = NULL ;
	CPathLine3D *pLine = NULL ;
	CPathPLine3D *pPLine = NULL ;
	PNT3D start, end, end1 ;
	double dist = -1., h = 0. ;

	PCombHead = NewPath->m_pHead ;
	// 只有一个CPathCombine 
	AddHeadPlungePath( PCombHead ) ;
	pHead = PCombHead->m_pHead ;
	if( !pHead ) return ;
	pHead->GetEndPoint ( 1, start ) ;
	while( pHead )
	{
		pNext = pHead->next ;
		if( !pNext ) break ;
		pNext->GetEndPoint ( 0, end ) ;
		dist = mathDist( start, end ) ;
		if( dist >= 0.0001 ) 
		{
			// 首先添加慢速下刀
			if( m_cSpeedDef.m_dPlungeDist > 0.01 )
			{
				mathCpyPnt( end, end1 ) ;
				end[2] += m_cSpeedDef.m_dPlungeDist ;
	//			if (end[2] > NewPath->m_fRapidHeight)
	//			{
	//				end[2] = NewPath->m_fRapidHeight;
	//			}
				pLine = new CPathLine3D( end, end1 ) ;
				pLine->m_bFeedType = JDNC_FEEDTYPE_PLUNGE ;
				PCombHead->InsertAfter ( pLine, pHead ) ;
			}
			// 然后添加相对高度连刀
			h = max(start[2], end[2]);
			h = max(h, NewPath->m_fRapidHeight);			

			double dTop = GetTopHeight() + m_cSpeedDef.m_dRapidHeight;
			DOUBLE box[2][3] ;
			// 保留曲线高度有可能加工域比路径高，取最高点
			if( Graph.GetEntBox( box, SMARTGRAPH_TYPE_CONTOUR ) )
			{
				if (dTop < (box[1][2] + m_cSpeedDef.m_dRapidHeight))
					dTop = box[1][2] + m_cSpeedDef.m_dRapidHeight ;
			}
			h = max(h, dTop);
	
			pPLine = CreateRelativeQuick( start, end, h ) ;
			if( pPLine )
			{
				PCombHead->InsertAfter ( pPLine, pHead ) ;
			}
		}
		pNext->GetEndPoint ( 1, start ) ;
		pHead = pNext ;
	}
}

void CSmartPathGen::AddSlowPlungePathFor5DPath(CPathGroup *NewPath)
{
	if( !NewPath || !NewPath->m_pHead )
		return ;

	// 首先合并NewPath
	CPathCombine *PCombHead = NULL, *PCombNext = NULL ;
	while( NewPath->m_pHead && NewPath->m_pHead->next )
	{
		PCombNext = NewPath->m_pHead->next ;
		NewPath->RemoveCombine ( PCombNext ) ;
		NewPath->m_pHead->AppendCombine ( PCombNext ) ;
	}

	CPathEntity *pHead = NULL, *pNext = NULL, *pPrev = NULL ;
	CPathEntity* pNewPathEnt = NULL;
	PNT3D start, end ;
	double dist = -1. ;
	bool bSuccess = false;

	PCombHead = NewPath->m_pHead ;
	pHead = PCombHead->m_pHead ;
	if( !pHead ) return ;
	pPrev = pHead;
	pHead->GetEndPoint ( 1, start ) ;
	while( pHead )
	{
		pNext = pHead->next;
		if (!pNext)
			break;
		pNext->GetEndPoint(0, end);
		dist = mathDist(start, end);
		if (pHead->m_bFeedType == JDNC_FEEDTYPE_QUICK || pHead->m_bFeedType == JDNC_FEEDTYPE_RAPIDCNT)
		{
			if ( NewPath->m_nType != NC_PATHTYPE_DRILL5AX && NewPath->m_nType != NC_PATHTYPE_ENGRAVE5AX )//xiejunxian 2013/5/21
			{    //为由五轴转成的三轴路径添加慢速下刀
				if(pHead->GetType() == NC_PATH_LINE3D)
				{
					pHead->m_pTDir = new CNcToolDir;
					pHead->m_pTDir->m_nNumDir = 2;
					pHead->m_pTDir->m_pTDir = new double[2][3];
					pHead->m_pTDir->m_pTDir[0][0] = 0; pHead->m_pTDir->m_pTDir[1][0] = 0;
					pHead->m_pTDir->m_pTDir[0][1] = 0; pHead->m_pTDir->m_pTDir[1][1] = 0;
					pHead->m_pTDir->m_pTDir[0][2] = 1; pHead->m_pTDir->m_pTDir[1][2] = 1;
				}
				else if(pHead->GetType() == NC_PATH_PLINE3D)
				{
					CPathPLine3D* pPLine3D = (CPathPLine3D*)pHead;
					pHead->m_pTDir = new CNcToolDir;
					pHead->m_pTDir->m_nNumDir = pPLine3D->m_nCount + 1;
					pHead->m_pTDir->m_pTDir = new double[pPLine3D->m_nCount + 1][3];
					for (int i=0; i<=pPLine3D->m_nCount; i++)
					{
						pHead->m_pTDir->m_pTDir[i][0] = 0;
						pHead->m_pTDir->m_pTDir[i][1] = 0;
						pHead->m_pTDir->m_pTDir[i][2] = 1;
					}
				}
			}
			if (pNext->m_bFeedType != JDNC_FEEDTYPE_QUICK && pNext->m_bFeedType !=  JDNC_FEEDTYPE_RAPIDCNT && dist < MIN_DIS)
			{
				CPtrList listPath;
				pNewPathEnt = NULL;
				if (ConvertQuickPathToPlungeAndQuickPath(pHead, m_cSpeedDef.m_dPlungeDist, listPath))
				{
					bSuccess = false;
					while (!listPath.IsEmpty())
					{
						pNewPathEnt = (CPathEntity*)listPath.RemoveHead();
						if (pNewPathEnt)
						{
							bSuccess = true;
							PCombHead->InsertAfter(pNewPathEnt, pPrev);
							pPrev = pNewPathEnt;
						}
					}
					if (bSuccess)
					{
						PCombHead->RemoveEntity(pHead);
						delete pHead;
						pHead = pNext->prev;
					}
				}
			}
			if ( NewPath->m_nType != NC_PATHTYPE_DRILL5AX && NewPath->m_nType != NC_PATHTYPE_ENGRAVE5AX )
			{
				CNcToolDir* delDir = pHead->m_pTDir;     //释放内存 xiejunxian 2013/5/21
				TPNT3D* delTDir = pHead->m_pTDir->m_pTDir;
				pHead->m_pTDir->m_pTDir = NULL;
				pHead->m_pTDir = NULL;
				delete delDir;
				delete []delTDir;
			}
		}
		pNext->GetEndPoint(1, start);
		pPrev = pHead;
		pHead = pNext ;
	}
}

BOOL CSmartPathGen::ConvertQuickPathToPlungeAndQuickPath(CPathEntity	* &QuickPath,    // 快速下刀路径
														 DOUBLE		      PlungeDist,   // 慢下距离
														 CPtrList		& AllPath)	// 输出慢下路径和快速下刀路径
{
	if (!QuickPath || (QuickPath->GetType() != NC_PATH_PLINE3D && QuickPath->GetType() != NC_PATH_LINE3D) )
		return FALSE;

	double dMinPlungeDis = 0.01;
	if (PlungeDist < dMinPlungeDis)
	{
		return FALSE;
	}

	int i = 0;
	DOUBLE dDis = 0.0;
	DOUBLE dPlungeDis = PlungeDist;
	double dt = 0.0;
	PNT3D ptSt, ptEd, ptInsert;
	VEC3D vSt, vEd, vInsert;
	
	if (QuickPath->GetType() == NC_PATH_LINE3D)
	{
		CPathLine3D* pLine3D = (CPathLine3D*)QuickPath;
		dDis = pLine3D->GetLength();
		if (dDis < dMinPlungeDis)
			return FALSE;
		if (dDis < dPlungeDis + dMinPlungeDis)
		{
			// 直接转成慢速下刀路径
			CPathLine3D* pNewLine3D = (CPathLine3D*)pLine3D->CopyMyself();
			pNewLine3D->m_bFeedType = JDNC_FEEDTYPE_PLUNGE;
			AllPath.AddTail(pNewLine3D);
		}
		else
		{
			// 中间插点，转成快速定位路径和慢速下刀路径
			dt = dPlungeDis/dDis;
			pLine3D->GetEndPoint(1, ptEd);
			pLine3D->GetEndPoint(0, ptSt);
			mathCpyPnt(pLine3D->m_pTDir->m_pTDir[0], vSt);
			mathCpyPnt(pLine3D->m_pTDir->m_pTDir[1], vEd);
			nc_GetPointAtLine(ptEd, ptSt, dt, ptInsert, 3);
			nc_GetPointAtLine(vSt, vEd, dt, vInsert, 3);
			if (ERUNSUC == mathUniVec(vInsert, MIN_LEN))
				return FALSE;
			CPathLine3D * pNewLine3D = new CPathLine3D(ptInsert, ptEd);
			CNcToolDir* pToolDir = new CNcToolDir(2);
			pNewLine3D->m_bFeedType = JDNC_FEEDTYPE_PLUNGE;
			mathCpyPnt(vInsert, pToolDir->m_pTDir[0]);
			mathCpyPnt(vEd, pToolDir->m_pTDir[1]);
			pNewLine3D->m_pTDir = pToolDir;
			AllPath.AddTail(pNewLine3D);

			pNewLine3D = new CPathLine3D(ptSt, ptInsert);
			pToolDir = new CNcToolDir(2);
			pNewLine3D->m_bFeedType = pLine3D->m_bFeedType;
			mathCpyPnt(vSt, pToolDir->m_pTDir[0]);
			mathCpyPnt(vInsert, pToolDir->m_pTDir[1]);
			pNewLine3D->m_pTDir = pToolDir;
			AllPath.AddHead(pNewLine3D);
		}
	}
	else if (QuickPath->GetType() == NC_PATH_PLINE3D)
	{
		CPathPLine3D* pPLine3D = (CPathPLine3D*)QuickPath;
		mathCpyPnt(pPLine3D->m_pTAPos[pPLine3D->m_nCount - 1], ptSt);
		mathCpyPnt(pPLine3D->m_pTDir->m_pTDir[pPLine3D->m_nCount - 1], vSt);
		mathCpyPnt(pPLine3D->m_pTAPos[pPLine3D->m_nCount], ptEd);
		mathCpyPnt(pPLine3D->m_pTDir->m_pTDir[pPLine3D->m_nCount], vEd);

		dDis = mathDist(ptSt, ptEd);
		if (dDis < dMinPlungeDis)
		{
			return FALSE;
		}
		if (dDis < dPlungeDis + dMinPlungeDis )
		{
			// 最后一段直接转成慢速下刀路径，定位路径点数减1
			CPathPLine3D * pNewPLine3D = new CPathPLine3D;
			CNcToolDir* pToolDir = new CNcToolDir(2);
			pNewPLine3D->m_pTAPos = new TPNT3D[2];			
			for (i = 0; i < 3; ++i)
			{
				pNewPLine3D->m_pTAPos[0][i] = ptSt[i];
				pNewPLine3D->m_pTAPos[1][i] = ptEd[i];
				pToolDir->m_pTDir[0][i] = vSt[i];
				pToolDir->m_pTDir[1][i] = vEd[i];
			}
			pNewPLine3D->m_pTDir = pToolDir;
			pNewPLine3D->m_bFeedType = JDNC_FEEDTYPE_PLUNGE;
			pNewPLine3D->m_nCount = 1;			
			AllPath.AddTail(pNewPLine3D);
			 
			if (pPLine3D->m_nCount > 2)
			{
				int nCnt = pPLine3D->m_nCount;
				int nNum = nCnt - 1;
				pNewPLine3D = new CPathPLine3D;
				pNewPLine3D->m_pTAPos = new TPNT3D[nCnt];
				pToolDir = new CNcToolDir(nCnt);
				for (i = 0; i < nCnt; ++i)
				{
					mathCpyPnt(pPLine3D->m_pTAPos[i], pNewPLine3D->m_pTAPos[i]);
					mathCpyPnt(pPLine3D->m_pTDir->m_pTDir[i], pToolDir->m_pTDir[i]);
				}
				pNewPLine3D->m_pTDir = pToolDir;
				pNewPLine3D->m_bFeedType = pPLine3D->m_bFeedType;
				pNewPLine3D->m_nCount = nNum;
				AllPath.AddHead(pNewPLine3D);
			}
			else if (pPLine3D->m_nCount == 2)
			{
				pNewPLine3D = new CPathPLine3D;
				pNewPLine3D->m_pTAPos = new TPNT3D[2];				
				pToolDir = new CNcToolDir(2);
				for (i = 0; i < 3; ++i)
				{
					pNewPLine3D->m_pTAPos[0][i] = pPLine3D->m_pTAPos[0][i];
					pNewPLine3D->m_pTAPos[1][i] = pPLine3D->m_pTAPos[1][i];
					pToolDir->m_pTDir[0][i] = pPLine3D->m_pTDir->m_pTDir[0][i];
					pToolDir->m_pTDir[1][i] = pPLine3D->m_pTDir->m_pTDir[1][i];
				}
				pNewPLine3D->m_pTDir = pToolDir;
				pNewPLine3D->m_bFeedType = pPLine3D->m_bFeedType;
				pNewPLine3D->m_nCount = 1;
				AllPath.AddHead(pNewPLine3D);
			}
			else 
			{

			}
		}
		else
		{
			// 最后一段中间插点，转成快速定位路径和慢速下刀路径；定位路径点数保持不变
			dt = dPlungeDis/dDis;
			nc_GetPointAtLine(ptEd, ptSt, dt, ptInsert, 3);
			nc_GetPointAtLine(vEd,  vSt, dt, vInsert, 3);
			if(ERUNSUC == mathUniVec(vInsert, MIN_LEN))
			{
				return FALSE;
			}
			CPathPLine3D* pNewPLine3D = new CPathPLine3D;
			CNcToolDir* pToolDir = new CNcToolDir(2);
			pNewPLine3D->m_pTAPos = new TPNT3D[2];
			for (i = 0; i < 3; ++i)
			{
				pNewPLine3D->m_pTAPos[0][i] = ptInsert[i];
				pNewPLine3D->m_pTAPos[1][i] = ptEd[i];
				pToolDir->m_pTDir[0][i] = vInsert[i];
				pToolDir->m_pTDir[1][i] = vEd[i];
			}
			pNewPLine3D->m_pTDir = pToolDir;
			pNewPLine3D->m_bFeedType = JDNC_FEEDTYPE_PLUNGE;
			pNewPLine3D->m_nCount = 1;			
			AllPath.AddTail(pNewPLine3D);

			int nCnt = pPLine3D->m_nCount + 1;
			int nNum = nCnt - 1;
			pNewPLine3D = new CPathPLine3D;
			pNewPLine3D->m_bFeedType = pPLine3D->m_bFeedType;
			pNewPLine3D->m_pTAPos = new TPNT3D[nCnt];
			pToolDir = new CNcToolDir(nCnt);
			for (int i = 0; i < nNum; ++i)
			{
				mathCpyPnt(pPLine3D->m_pTAPos[i], pNewPLine3D->m_pTAPos[i]);
				mathCpyPnt(pPLine3D->m_pTDir->m_pTDir[i], pToolDir->m_pTDir[i]);
			}
			mathCpyPnt(ptInsert, pNewPLine3D->m_pTAPos[nNum]);
			mathCpyPnt(vInsert, pToolDir->m_pTDir[nNum]);
			pNewPLine3D->m_pTDir = pToolDir;
			pNewPLine3D->m_nCount = nNum;
			AllPath.AddHead(pNewPLine3D);			
		}
	}

	return TRUE;
}

void	CSmartPathGen::GetLeadCutShift(JDNC_LEADOUT& out, double dShift[2])
{
	dShift[0] = m_cProcessDef.m_dLeadInArcH;
	if (out.m_bSameLeadIn)
	{
		dShift[1] = dShift[0];
	}
	else
	{
		dShift[1] = m_cProcessDef.m_dLeadOutArcH;
	}
}
BOOL    CSmartPathGen::GetToolHolder( JDNC_HOLDER& Holder ) 
{
    if( g_pCurrToolGroup == NULL ) 
    {
        return FALSE ;
    }
    CJDTool* pTool = g_pCurrToolGroup->GetByID( m_cToolDef.m_nToolID ) ;
    if( pTool == NULL || !( m_cProcessDef.m_bProcessFlag & NCDEF_TOOL_USEHOLDER)  )
    {
        return FALSE ;
    }
    if( !pTool->HasValidHolder() ) 
    {
        return FALSE ;
    }
    Holder = *pTool->m_pHolderDef ;
    return TRUE ;
}

CJDTool* CSmartPathGen::GetTool()
{
	if( !g_pCurrToolGroup ) 
    {
        return NULL ;
    }
    CJDTool* pTool = g_pCurrToolGroup->GetByID( m_cToolDef.m_nToolID ) ;
    return pTool ;
}

int CSmartPathGen::CreateHolderTool() 
{
    if( m_pHolderTool ) delete m_pHolderTool ;
    m_pHolderTool = NULL  ;
    JDNC_HOLDER holder ;
	BOOL bHolder = TRUE ;
	JDNC_TOOLEX cOrgTool = m_cToolDef;	// 备份，处理刀杆时可能被修改
    if( GetToolHolder( holder ) == FALSE )
    {
        bHolder = FALSE ;
		memset( &holder, 0, sizeof( JDNC_HOLDER ) ) ;
		holder.m_nHolderID = -1 ;
    }
	BOOL bSpecial = FALSE; // 糖果刀、槽铣刀刀杆参数在刀具中处理
	if (m_cToolDef.m_nToolType == surfncToolLol ||
		m_cToolDef.m_nToolType == surfncToolSlot )
	{
		bSpecial = TRUE;
	}
	if(!bSpecial && m_cToolDef.m_nToolStatus & NCDEF_TOOLSTA_SHANK )
	{ // 将holder参数后移,刀杆为第一级刀柄
		if( CreateToolShank( holder ) ) bHolder = TRUE ;
	}
	if( !bHolder ) return FALSE ;
    CSmtToolSpecial * pToolComb = new CSmtToolSpecial() ;
	pToolComb->CreateHolderTool( holder, m_cToolDef.m_dLength, max( 0., m_cProcessDef.m_dHolderTol)) ;
	m_cToolDef = cOrgTool;	// 还原
	m_pHolderTool = pToolComb ;
    return TRUE ;
}
BOOL CSmartPathGen::CreateToolShank ( JDNC_HOLDER &Holder )
{
	if( m_cToolDef.m_dShankHeight < 0.01 ) return FALSE ;
	BOOL bCone = TRUE ;
	if( m_cToolDef.m_dShankConeLen < 0.01 ) bCone = FALSE ;
	int i = 0, k = 0 ;
	double dTol = max( 0., m_cProcessDef.m_dShankTol);
	dTol -= max( 0., m_cProcessDef.m_dHolderTol);
	m_cToolDef.m_dLength = max (0.0, m_cToolDef.m_dLength - dTol);// 补偿余量差
	for( k = 0 ; k < 2 ; k++ )
	{
		if( k == 1 && !bCone ) break ;
		// 向后移一位
		if( Holder.m_nLevelNum <= 9 && Holder.m_nLevelNum > 0)
		{
			Holder.m_dTopDiam[Holder.m_nLevelNum] = Holder.m_dTopDiam[Holder.m_nLevelNum-1] ;
			Holder.m_dBtmDiam[Holder.m_nLevelNum] = Holder.m_dBtmDiam[Holder.m_nLevelNum-1] ;
			Holder.m_dHeight [Holder.m_nLevelNum] = Holder.m_dHeight [Holder.m_nLevelNum-1] ;
		}
		for( i = Holder.m_nLevelNum - 1 ; i > 0 ; i-- )
		{
			Holder.m_dTopDiam[i] = Holder.m_dTopDiam[i-1] ;
			Holder.m_dBtmDiam[i] = Holder.m_dBtmDiam[i-1] ;
			Holder.m_dHeight [i] = Holder.m_dHeight [i-1] ;
		}
		if( k == 0 )
		{
			Holder.m_dTopDiam[0] = m_cToolDef.m_dShankTopDiam + dTol * 2.;
			if( bCone )
				Holder.m_dBtmDiam[0] = m_cToolDef.m_dShankTopDiam + dTol * 2.;
			else
                Holder.m_dBtmDiam[0] = m_cToolDef.m_dShankBtmDiam + dTol * 2.;
			Holder.m_dHeight [0] = m_cToolDef.m_dShankHeight - m_cToolDef.m_dShankConeLen;
			Holder.m_dHeight [0] = max (0.0, Holder.m_dHeight [0] + dTol);
		}
		else
		{
			Holder.m_dTopDiam[0] = m_cToolDef.m_dShankTopDiam + dTol * 2.;
			Holder.m_dBtmDiam[0] = m_cToolDef.m_dShankBtmDiam + dTol * 2.;
//			Holder.m_dHeight [0] = m_cToolDef.m_dShankConeLen;//guomin 2013/12/13 碰撞检查,此处未考虑间隙刀具不匹配
			Holder.m_dHeight [0] = max(0.0,m_cToolDef.m_dShankConeLen + dTol );
		}
		if( Holder.m_nLevelNum <= 9 )Holder.m_nLevelNum ++ ;
	}
	return TRUE ;
}

void CSmartPathGen::UpdateLeadOutParam(JDNC_LEADOUT& leadOut, JDNC_LEAD& Out)
{
	Out = m_cFeedDef.m_cLeadDef ;
	if( leadOut.m_bSameLeadIn )
	{
		BOOL bSet = m_cProcessDef.m_bProcessFlag & NCDEF_LEADIN_ARCIN_OUTARC ? TRUE : FALSE;
		if (bSet)
		{
			m_cProcessDef.m_bProcessFlag |= NCDEF_LEADOUT_ARCIN_OUTARC;
		}
		else
		{
			m_cProcessDef.m_bProcessFlag &= ~NCDEF_LEADOUT_ARCIN_OUTARC;
		}

		bSet = m_cProcessDef.m_bProcessFlag & NCDEF_LEADIN_LINEIN_TANG ? TRUE : FALSE ;
		if( bSet )
		{
			m_cProcessDef.m_bProcessFlag |= NCDEF_LEADOUT_LINEIN_TANG ;
		}
		else
		{
			m_cProcessDef.m_bProcessFlag &= ~NCDEF_LEADOUT_LINEIN_TANG ;
		}
		m_cProcessDef.m_dLeadOutArcLandRad = m_cProcessDef.m_dLeadInArcLandRad;
		m_cProcessDef.m_dLeadOutArcLandAng = m_cProcessDef.m_dLeadInArcLandAng;
	}
	else
	{
		Out.m_nLeadType = leadOut.m_nLeadType ;
		Out.m_dLength   = leadOut.m_dLength   ;
		Out.m_dAngle    = leadOut.m_dAngle    ;
		Out.m_dRadius   = leadOut.m_dRadius   ;
	}
}

// 圆弧内切进刀时是否需要外切引入
BOOL CSmartPathGen::NeedOutArcAtArcIn(JDNC_LEAD& LeadDef, BOOL IsOut)
{
	if (LeadDef.m_nLeadType != NCDEF_LEAD_INARC)
	{
		return FALSE;
	}

	DWORD nFlag = IsOut ? NCDEF_LEADOUT_ARCIN_OUTARC : NCDEF_LEADIN_ARCIN_OUTARC;
	return m_cProcessDef.m_bProcessFlag & nFlag ? TRUE : FALSE;	
}

// 圆弧外切是否需要直线引入
BOOL CSmartPathGen::NeedLineAtArcOut( JDNC_LEAD &LeadDef, BOOL IsOut ) 
{
	if (LeadDef.m_nLeadType != NCDEF_LEAD_ARC)
	{
		return FALSE;
	}

	DWORD nFlag = IsOut ? NCDEF_LEADOUT_LINEIN_TANG : NCDEF_LEADIN_LINEIN_TANG;
	return m_cProcessDef.m_bProcessFlag & nFlag ? TRUE : FALSE;	
}
// 获取圆弧内切进刀时外切引入参数
void CSmartPathGen::GetParamOfOutArcAtArcIn(BOOL IsOut, double& dRad, double& dAng)
{
	if (IsOut)
	{
		dRad = m_cProcessDef.m_dLeadOutArcLandRad;
		dAng = ANGLE_TO_RADIAN(m_cProcessDef.m_dLeadOutArcLandAng);
	}
	else
	{
		dRad = m_cProcessDef.m_dLeadInArcLandRad;
		dAng = ANGLE_TO_RADIAN(m_cProcessDef.m_dLeadInArcLandAng);
	}
}

BOOL CSmartPathGen::AddLeadPathAndAddInPGroup(CSmtCheckMdl& DriveMdl, 
                                              CSmtCPathLib& AllPath ,
                                              CPathGroup&   NewPath ) 
{
    // 转化路径
	AddPlungeLeadPath( DriveMdl, m_cFeedDef.m_cConnectDef ,AllPath );	
    if( SurfNC_IsAbort() )
    {
       m_nErrorType = JDERROR_GENPATH_ABORT  ;
       return FALSE ;
    }
    CPathCombine * pPComb = new CPathCombine(NC_WPROCESS_ROUGH) ;
    AllPath.AddToPathCombine( *pPComb ) ;
    NewPath.AddData( 0.0, pPComb  ) ;
	AddPlungeAndRelativeH( &DriveMdl, &NewPath ) ;
    return TRUE ;
}
BOOL CSmartPathGen::BuildMeshMdl(CSmtMeshMdl& MeshMdl  ,  /*网格模型*/
                                 CSmtCheckMdl& DriveMdl,  /*加工模型*/
                                 CSmartLoop* AllCont   ,  /*边界轮廓*/
                                 BOOL   TrimByZSize    )  /*高度修剪有效*/     
                              
{
	// STEP 1 : 根据范围初始化
	JDNC_TOL  tmpTol = GetSurfaceTol() ;
    TFLOAT   fNetStep = (TFLOAT) max( tmpTol.m_dMaxStep, 0.05 ) ;
    if( tmpTol.m_dArcTol < 0.005 )
    {/*高精度计算*/
        fNetStep = (TFLOAT)min( fNetStep, 0.25 * m_cToolDef.m_dTopDiam ) ;
    }
    else
    {
        DOUBLE dCoef = min( 1.0, (tmpTol.m_dArcTol / 0.005) * 0.25 ) ;
        fNetStep = (TFLOAT)min( fNetStep, dCoef * m_cToolDef.m_dTopDiam ) ;
    }
    FPNT3D fBox3D[2] ;
    DriveMdl.CalcFacetBox(fBox3D[0], fBox3D[1]) ;
    MathCAM_TrimBox3DByContours( AllCont, fBox3D ) ;
    if( fBox3D[0][0] > fBox3D[1][0] || fBox3D[0][1] > fBox3D[1][1] )
    {
		m_nErrorType = JDERROR_GENPATH_NOAREA  ;
		return FALSE ;
    }
	TFLOAT fMinStep = 0.1f * min( fBox3D[1][0] - fBox3D[0][0], fBox3D[1][1] - fBox3D[0][1] ) ;
	if( fNetStep > fMinStep ) fNetStep = fMinStep ;
	if( ! DriveMdl.m_pTool )
	{
		return FALSE ;
	}
    nc_ExpandBox3D( fBox3D, TFLOAT( 2.0 * DriveMdl.m_pTool->m_fRadius ) , FALSE ) ;
    // STEP 2 : 构建初始模型
    MeshMdl.m_cSetupDef = m_cSetupDef ;
    MathCAM_RegisterMeshMdlFunc( MeshMdl ) ;
    MeshMdl.InitMeshMdl( fBox3D , fNetStep ) ;

    // STEP 3 : 标记无效点
    /* 干涉检查模型, 检查面和落空处的点为 0,　其他点的属性为 1 */
	if( !MeshMdl.ModifyByCheckMdl( DriveMdl , tmpTol, m_cPrgDef ) )
	{
		m_nErrorType = JDERROR_GENPATH_ABORT ;
		return FALSE;
	}
    if( TrimByZSize )
    {
	    /* 根据加工的深度范围限定有效点, 高度范围内部的点的属性不变,范围外部点的属性为 0　*/
	    TFLOAT fZMax  = (TFLOAT) ( GetTopHeight()  ) ;//+ tmpTol.m_dArcTol
	    TFLOAT fZMin  = (TFLOAT) GetBottomHeight() ;
	    MeshMdl.LabelPointFlagByZValue( fZMin, fZMax ) ;
    }
	/* 根据轮廓限定有效点, 轮廓内部的点的属性不变,轮廓外部点的属性为 0　*/
	MeshMdl.LabelPointFlagByContour( AllCont ) ;
    return TRUE ;
}
/* 构建三轴加工的虚拟加工模型 */
BOOL CSmartPathGen::BuildVirtualNcMdl3Ax( CSmartGraphic& Graph       ,  /*加工图形*/
                                          CSmtVirtualNcMdl& VirNcMdl  ) /*加工模型*/
{
    // STEP 1 : 构建碰撞刀具模型\刀柄模型 
    int bCheckMode = SMART_MODEL_PROJECT ;
    VirNcMdl.m_fBottom =  (TFLOAT)GetBottomHeight() ; 
    VirNcMdl.m_pTool   =   m_pTool  ;
    CSmartTool * pCheckTool = CreateSmartTool(m_cToolDef, 0.0, 0.0);
    VirNcMdl.m_pCheckTool = pCheckTool ;
    JDNC_HOLDER holderDef ;
    if( GetToolHolder( holderDef ) ) 
    {
        CSmartHolder *pHolder = new CSmartHolder() ;
        pHolder->CreateHolder( holderDef, m_cToolDef.m_dLength, 0.1 ) ;
        if( pHolder->m_nNumLevel <= 0 ) 
        {
            delete pHolder ;
        }
        else
        {
            VirNcMdl.m_pCheckHolder = pHolder ;
            bCheckMode = SMART_MODEL_ROTATE  ;
            JDNC_SETUP tmpSetup = m_cSetupDef ;
            tmpSetup.m_cTolDef.m_dAngTol = 30.0 ;
            tmpSetup.m_cTolDef.m_dArcTol = 0.025;
            tmpSetup.m_cModelTol.m_nMdlFlag = 0 ;
            CSmtCheckMdl tmpChkMdl ;
            tmpChkMdl.m_bCheckMode  = SMART_MODEL_ROTATE ;
            tmpChkMdl.m_dFltBox[0][2] = (TFLOAT)GetBottomHeight()  + pHolder->m_fZShift[0] ;
            tmpChkMdl.m_dFltBox[1][2] = tmpChkMdl.m_dFltBox[0][2] + 10000 ;
	        Graph.BuildCheckModel( tmpSetup, tmpChkMdl, NCDEF_SURF_DRIVE|NCDEF_SURF_CHECK ) ; 
	        if( SurfNC_IsAbort() )
            {/*用户中断*/
		        m_nErrorType =  JDERROR_GENPATH_ABORT;
                return FALSE ;
            }
            else if( tmpChkMdl.m_cAllChk.GetSize() == 0 )
            {/*没有有效曲面*/
                delete VirNcMdl.m_pCheckHolder ; 
                VirNcMdl.m_pCheckHolder = NULL ; 
            }
            else
            {/*找到可能干涉的曲面*/
                tmpChkMdl.m_bCheckMode  = SMART_MODEL_PROJECT ;
	            tmpChkMdl.m_fBottom =  (TFLOAT)GetBottomHeight() ;
                VirNcMdl.AddHolderCheckMdl( tmpChkMdl ) ;
            }
        }
    }
    // STEP 2 : 构建加工面模型 
    CSmtCheckMdl *pDriveMdl = new CSmtCheckMdl() ;
    pDriveMdl->m_bCheckMode = bCheckMode    ;
    pDriveMdl->m_bSurfType  = SMARTNC_SURFTYPE_DRIVE ;
	if( ! Graph.BuildCheckModel( m_cSetupDef, *pDriveMdl, NCDEF_SURF_DRIVE ) ||
        pDriveMdl->m_cAllChk.GetSize() == 0 ) 
	{ 
        delete pDriveMdl ;
		m_nErrorType = JDERROR_GENPATH_NOSURFACE ;
		return FALSE ;
	}
    pDriveMdl->m_bCheckMode = SMART_MODEL_PROJECT    ;
	pDriveMdl->ZMoveModel( GetDriveSurfZMove() ) ;
	if( IsDelBndPoint() )
    {
        pDriveMdl->DefineBoundEdge();
        pDriveMdl->m_bCheckFlag |= SMARTNC_CHECKMDL_DELBOUNDPNT ;
    }
    if( IsDelFlatPoint() )
    {
        pDriveMdl->m_bCheckFlag |= SMARTNC_CHECKMDL_DELFLATPNT ;
    }
	pDriveMdl->UpdateCurrTool( m_pTool )  ;
	pDriveMdl->m_fBottom =  (TFLOAT)GetBottomHeight() ;
	if( IsDelBndPoint() ) 
    {
        pDriveMdl->m_fBottom -= (TFLOAT)GetSurfaceTol().m_dArcTol ;
    }
    pDriveMdl->UpdateCurrTool( m_pTool ) ;
    VirNcMdl.m_aDriveList.AddTail( pDriveMdl ) ;
    // STEP 3 : 构建保护面模型 
    CSmtCheckMdl *pCheckMdl = new CSmtCheckMdl() ;
    pCheckMdl->m_bCheckMode = bCheckMode    ;
    pCheckMdl->m_bSurfType  = SMARTNC_SURFTYPE_CHECK ;
	if( Graph.BuildCheckModel( m_cSetupDef, *pCheckMdl, NCDEF_SURF_CHECK ) && 
        pCheckMdl->m_cAllChk.GetSize() > 0 ) 
	{ 
        pCheckMdl->m_bCheckMode = SMART_MODEL_PROJECT    ;
		pCheckMdl->ZMoveModel ( GetCheckSurfZMove() ) ;
		pCheckMdl->UpdateCurrTool( m_pSafeTool )  ;
		pCheckMdl->m_fBottom =  (TFLOAT)GetBottomHeight()  ;
		if( IsDelBndPoint() )
		{
			pCheckMdl->m_fBottom -= TFLOAT( GetSurfaceTol().m_dArcTol + 0.001 ) ;
		}
        VirNcMdl.m_aCheckList.AddTail( pCheckMdl ) ;
    }
    else
    {
        delete pCheckMdl ;
	}
    // STEP 4 : 构建夹具模型
    /**/
    return TRUE ;
}
/* 构建多轴加工的虚拟加工模型 */
BOOL CSmartPathGen::BuildVirtualNcMdl5Ax( CSmartGraphic&  Graph       ,  /*加工图形*/
                                          CSmtVirtualNcMdl& VirNcMdl  ) /*加工模型*/
{
    // STEP 1 : 构建旋转工作台模型
    MATHCAM_GenVirtualNcMdl( VirNcMdl ) ;
    // STEP 2 : 构建碰撞刀具模型\刀柄模型 
    VirNcMdl.m_fBottom =  (TFLOAT)GetBottomHeight() ; 
    VirNcMdl.m_pTool   =   m_pTool  ;
    CSmartTool * pCheckTool = CreateSmartTool(m_cToolDef, 0.0, 0.0);
    VirNcMdl.m_pCheckTool = pCheckTool ;
    JDNC_HOLDER holderDef ;
    if( GetToolHolder( holderDef ) ) 
    {
        CSmartHolder *pHolder = new CSmartHolder() ;
        pHolder->CreateHolder( holderDef, m_cToolDef.m_dLength, 0.1 ) ;
        if( pHolder->m_nNumLevel <= 0 ) 
        {
            delete pHolder ;
        }
        else
        {/*需要大误差重建模型,否则计算速度很慢*/
            VirNcMdl.m_pCheckHolder = pHolder ;
            JDNC_SETUP tmpSetup = m_cSetupDef ;
            tmpSetup.m_cTolDef.m_dAngTol = 30.0 ;
            tmpSetup.m_cTolDef.m_dArcTol = 0.025;
            tmpSetup.m_cModelTol.m_nMdlFlag = 0 ;
            CSmtCheckMdl tmpChkMdl ;
            tmpChkMdl.m_bCheckMode  = SMART_MODEL_ROTATE ;
            tmpChkMdl.m_dFltBox[0][2] = (TFLOAT)GetBottomHeight()  + pHolder->m_fZShift[0] ;
            tmpChkMdl.m_dFltBox[1][2] = tmpChkMdl.m_dFltBox[0][2] + 10000 ;
	        Graph.BuildCheckModel( tmpSetup, tmpChkMdl, NCDEF_SURF_DRIVE|NCDEF_SURF_CHECK ) ; 
	        if( SurfNC_IsAbort() )
            {/*用户中断*/
		        m_nErrorType =  JDERROR_GENPATH_ABORT;
                return FALSE ;
            }
            else if( tmpChkMdl.m_cAllChk.GetSize() == 0 )
            {/*没有有效曲面*/
                delete VirNcMdl.m_pCheckHolder ; 
                VirNcMdl.m_pCheckHolder = NULL ; 
            }
            else
            {/*找到可能干涉的曲面*/
                tmpChkMdl.m_bCheckMode  = SMART_MODEL_ROTATE ;
	            tmpChkMdl.m_fBottom =  (TFLOAT)GetBottomHeight() ;
                VirNcMdl.AddHolderCheckMdl( tmpChkMdl ) ;
            }
        }
    }
    // STEP 3 : 构建加工面模型 
    CSmtCheckMdl *pDriveMdl = new CSmtCheckMdl() ;
    pDriveMdl->m_bCheckMode = SMART_MODEL_ROTATE    ;
    pDriveMdl->m_bSurfType  = SMARTNC_SURFTYPE_DRIVE ;
	if( ! Graph.BuildCheckModel( m_cSetupDef, *pDriveMdl, NCDEF_SURF_DRIVE ) ||
        pDriveMdl->m_cAllChk.GetSize() == 0 ) 
	{ 
        delete pDriveMdl ;
		m_nErrorType = JDERROR_GENPATH_NOSURFACE ;
		return FALSE ;
	}
	pDriveMdl->UpdateRotateTool( m_pTool )  ;
	pDriveMdl->m_fBottom =  (TFLOAT)GetBottomHeight() ;
    pDriveMdl->UpdateCurrTool( m_pTool ) ;
    VirNcMdl.m_aDriveList.AddTail( pDriveMdl ) ;
    // STEP 4 : 构建保护面模型 
    CSmtCheckMdl *pCheckMdl = new CSmtCheckMdl() ;
    pCheckMdl->m_bCheckMode = SMART_MODEL_ROTATE    ;
    pCheckMdl->m_bSurfType  = SMARTNC_SURFTYPE_CHECK ;
	if( Graph.BuildCheckModel( m_cSetupDef, *pCheckMdl, NCDEF_SURF_CHECK ) && 
        pCheckMdl->m_cAllChk.GetSize() > 0 ) 
	{ 
		pCheckMdl->UpdateRotateTool( m_pSafeTool )  ;
		pCheckMdl->m_fBottom =  (TFLOAT)GetBottomHeight()  ;
        VirNcMdl.m_aCheckList.AddTail( pCheckMdl ) ;
    }
    else
    {
        delete pCheckMdl ;
	}
    return TRUE ;
}

extern void MathCAM_ResetPathDepth( CSmtCPathLib& AllPath, int Flag );
//路径投影和包裹变换
void CSmartPathGen::ProjAndWrapPath(CPathGroup& NewPath, CSmartGraphic& Graph)
{
	if(m_cTransfDef.m_cProjtranDef.m_nProjtranType == 1)
	{//路径投影
		ProjPath( NewPath, Graph ) ;
	}
	else if(m_cTransfDef.m_cProjtranDef.m_nProjtranType == 2)
	{//路径包裹
		WrapPath( NewPath, Graph ) ;
	}
	else if(m_cTransfDef.m_cProjtranDef.m_nProjtranType == 3)
	{
		PlungeMillPath( NewPath, Graph ) ;
	}
}
//去掉钻孔路径的内部Quick连刀
void RemoveQuickEnt(CPathGroup& NewPath)
{
	for( CPathCombine *pTComb = NewPath.m_pHead ; pTComb ; pTComb = pTComb->next )
	{
		if( pTComb && pTComb->m_nType != NC_WPROCESS_DRILL )
			break ;
		CPathEntity *pCur = NULL , *pNxt = NULL ;
		pCur = pTComb->m_pHead ;
		while( pCur )
		{
			pNxt = pCur->next ;
			if( pCur->m_bFeedType == JDNC_FEEDTYPE_QUICK )
			{
				pTComb->RemoveEntity(pCur);
				delete pCur ;
				pCur = NULL ;
			}
			pCur = pNxt ;
		}
	}
}
//路径投影
int CSmartPathGen::ProjPath(CPathGroup& NewPath, CSmartGraphic& Graph)
{
	//STEP1 : 构建检查模型
	DOUBLE dSurfStock = GetDriveSurfOffset() ;
	DOUBLE dCheckStock = GetCheckSurfOffset() ;
	BOOL bCloseComp = m_cTransfDef.m_cProjtranDef.m_bProjtranFlag & NCDEF_PROJTRAN_CLOSECOMP ? TRUE : FALSE ;
	JDNC_TOOLEX tmpTool = m_cToolDef ;
	if( bCloseComp )
	{
		tmpTool.m_nToolType = surfncToolFlat ;
		tmpTool.m_dTopDiam  = 0.005 ;
		dSurfStock   = dCheckStock = 0.0 ;
	}
	if( m_pTool ) delete m_pTool ;
	if( m_pSafeTool ) delete m_pSafeTool ;
	
	m_pTool = CreateSmartTool( tmpTool,  dSurfStock, 0.0 ) ;
	m_pSafeTool = CreateSmartTool( tmpTool, dCheckStock, 0.) ;
	CSmtCheckMdl  DriveMdl ;
	DOUBLE dBox[2][3], dSideTol = m_pTool->m_fRadius ;
	CJDToolGroup *pCurrToolGroup = MathCAM_GetCurrToolGroup() ;
	NewPath.GetBoundBox(dBox[0], dBox[1]);
	if( !bCloseComp )
	{ 
		CJDTool*  pNcTool = pCurrToolGroup->GetByID( NewPath.m_nToolID ) ;
		if( pNcTool && pNcTool->m_cToolParam.m_dTopDiam/2 > dSideTol )
		{
			dSideTol = pNcTool->m_cToolParam.m_dTopDiam/2 ;
		}
	}
	FPNT3D minp, maxp ;
	// yul封于2012/09/26，当直线与平面平行的时候该方式会将平面过滤掉
//	DriveMdl.m_dFltBox[0][0] = dBox[0][0] - dSideTol ;
//	DriveMdl.m_dFltBox[0][1] = dBox[0][1] - dSideTol ;
//	DriveMdl.m_dFltBox[1][0] = dBox[1][0] + dSideTol ;
//	DriveMdl.m_dFltBox[1][1] = dBox[1][1] + dSideTol ;
	if( ! Graph.BuildProjectModel( m_cSetupDef, DriveMdl ) )
	{
		m_nErrorType = JDERROR_GENPATH_NOSURFACE  ;
		return 0 ; 
	}
	DriveMdl.ZMoveModel( dSurfStock ) ;
	DriveMdl.UpdateCurrTool( m_pTool )  ;
	DriveMdl.CalcFacetBox ( minp, maxp ) ;
	DriveMdl.m_fBottom = (TFLOAT)minp[2]  ;
	DriveMdl.m_bCheckMode = SMART_MODEL_PROJECT ;
	
	SurfNC_SetCurPos( 8, 10 ) ;
	//STEP 2: 干涉检查, 生成无干涉的刀具路径
	RemoveQuickEnt( NewPath );
	CSmtCPathLib AllPath ;
	BOOL  bReDepth = m_cTransfDef.m_cProjtranDef.m_bProjtranFlag & NCDEF_PROJTRAN_KEEPDEPTH ? TRUE : FALSE ;

	ProcessNewFunc( m_cPrgDef, IDS_NCDEF_PROJTRAN_VERT ) ;
	MathCAM_AddPathGroup( AllPath, NewPath, m_cSetupDef.m_cTolDef , bReDepth ) ;
	if( bReDepth ) MathCAM_ResetPathDepth( AllPath, 0 ) ;
	int nPnt = AllPath.GetNumPoint() ;
	if( nPnt < 1 )  nPnt = 1      ;
	m_cPrgDef.m_dTotalMove = 100. ;
	m_cPrgDef.m_dLimitAt  =  nPnt / 100.0   ;   // 前进一次的计算量
	m_cPrgDef.m_dStepAt   =  0.0            ;   // 当前计算进度
	m_cPrgDef.m_dIncStep  =  1.0            ;   // 每次计算的前进量
	if( ! AllPath.VerifyCutPath( DriveMdl, GetSurfaceTol(), m_cPrgDef ) )
	{
		return 0 ;
	}
	CPathCombine * pPComb = new CPathCombine( NC_WPROCESS_IMPORT ) ;
	pPComb->m_pEntFrom  = NewPath.m_pEntFrom  ;
	if( bReDepth ) MathCAM_ResetPathDepth( AllPath, 1 ) ;
	AllPath.DelPointOnLine(2.0e-4) ; // 删除同一直线上的点
	AllPath.AddToPathCombine( * pPComb   ) ;
	AllPath.ClearAllPath() ;
	CPathGroup tmpPGroup(NC_PATHTYPE_PSURFACE) ;
	tmpPGroup.AddData( 0.0 , pPComb ) ;
	AddPlungeAndRelativeH( &DriveMdl, &tmpPGroup ) ;
	NewPath.ClearAll () ;   //删除原始路径
	while( tmpPGroup.m_pHead )
	{
		pPComb = tmpPGroup.m_pHead ;
		tmpPGroup.RemoveCombine( pPComb ) ;
		NewPath.AddData( 0.0, pPComb ) ;
	}
	if( m_cPrgDef.m_pBrkFunc && m_cPrgDef.m_pBrkFunc() )
	{
		m_nErrorType = JDERROR_GENPATH_ABORT  ;
		return 0 ; 
	}
	return 1;
}
//路径包裹
int CSmartPathGen::WrapPath( CPathGroup& NewPath, CSmartGraphic& Graph )
{
	JDNC_TOOLEX tmpTool = m_cToolDef ;
	tmpTool.m_nToolType = surfncToolFlat ;
	tmpTool.m_dTopDiam  = 0.01 ;
	if(m_pTool ) delete m_pTool ;

	m_pTool = CreateSmartTool( tmpTool, 0.0, 0.) ;
	//STEP 1 : 生成加工模型
	PNT3D dMinPt, dMaxPt, dCenter ;
	CSmtCheckMdl  DriveMdl ;
	dCenter[0] = m_cTransfDef.m_cProjtranDef.m_dWrapCenter[0];
	dCenter[1] = m_cTransfDef.m_cProjtranDef.m_dWrapCenter[1] ;
	dCenter[2] = 0.0 ; 
	DOUBLE dAngle = 0.0 ;
	dAngle = ANGLE_TO_RADIAN( m_cTransfDef.m_cProjtranDef.m_dWrapAngle ) ;
	if( ! Graph.BuildProjectModel( m_cSetupDef, DriveMdl ) )
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
	DriveMdl.m_fBottom = (TFLOAT)GetBottomHeight()  ;
	DriveMdl.DefineDriveSurfBox( dMinPt, dMaxPt ) ;
	//STEP 2 : 构建包裹发生器
	CSmartWraper  Wraper ;
	double dStep = m_cSetupDef.m_cTolDef.m_dMaxStep ;
	double dAt[2] ;
	dAt[0] = dAt[1] = 0.0 ;
	Wraper.InitBuffer( dMinPt, dMaxPt, dStep , dStep, dAt ) ;
	Wraper.CheckCloud( DriveMdl, m_cPrgDef ) ;
	Wraper.DefineUWParam() ;
	Wraper.DefineNormal() ;
	// STEP 3 :干涉检查, 生成无干涉的刀具路径
	RemoveQuickEnt( NewPath );
	CSmtCPathLib AllPath ;
	BOOL  bReDepth = m_cTransfDef.m_cProjtranDef.m_bProjtranFlag & NCDEF_PROJTRAN_KEEPDEPTH ? TRUE : FALSE ;
	NewPath.MoveGroup( -dCenter[0], -dCenter[1], 0.0 ) ; 
	if( fabs(dAngle) > 1.0e-3 ) NewPath.RotateGroup( -dAngle ) ;
	MathCAM_AddPathGroup( AllPath, NewPath, m_cSetupDef.m_cTolDef, bReDepth) ;
	// 进度条
	ProcessNewFunc( m_cPrgDef, IDS_NCDEF_PROJTRAN_WRAP ) ;
	int nPnt = AllPath.GetNumPoint() ;
	if( nPnt < 1 )  nPnt = 1      ;
	m_cPrgDef.m_dTotalMove = 100. ;
	m_cPrgDef.m_dLimitAt  =  nPnt / 100.0   ;   // 前进一次的计算量
	m_cPrgDef.m_dStepAt   =  0.0            ;   // 当前计算进度
	m_cPrgDef.m_dIncStep  =  1.0            ;   // 每次计算的前进量
	if( ! Wraper.WrapCutPath( AllPath, m_cSetupDef.m_cTolDef, m_cPrgDef ) )
	{ // 用户中断
		return 0 ;
	}
	AllPath.DelPointOnLine() ; // 删除同一直线上的点
	NewPath.ClearAll () ;     //删除原始路径
	CPathCombine * pPComb = new CPathCombine( NC_WPROCESS_ROUGH ) ;
	pPComb->m_pEntFrom  = NewPath.m_pEntFrom  ;
	AllPath.AddToPathCombine( * pPComb   ) ;
	if( fabs( dAngle ) > 1.0e-3 ) pPComb->RotateCombine( dAngle ) ;
	pPComb->MoveCombine( dCenter[0], dCenter[1], 0.0 ) ;
	NewPath.AddData( 0.0, pPComb ) ;
	AllPath.ClearAllPath() ;
	if( m_cPrgDef.m_pBrkFunc && m_cPrgDef.m_pBrkFunc() )
	{
		m_nErrorType = JDERROR_GENPATH_ABORT  ;
		return 0 ; 
	}
	NewPath.ZMoveGroup( GetDriveSurfZMove() ) ;
	return 1;
}
//插铣变换
int CSmartPathGen::PlungeMillPath( CPathGroup& NewPath, CSmartGraphic& Graph )
{
	UNUSED_ALWAYS(Graph) ;
	CSmtCPathLib AllPath ;
	//将PathGroup转为CSmtCutPath
	MathCAM_AddPathGroup( AllPath, NewPath, m_cSetupDef.m_cTolDef , TRUE ) ;
	int nPnt = AllPath.GetNumPoint() ;
	if( nPnt < 1 )  nPnt = 1      ;
	//得到CSmtCutPath上的刀位点
	if( AllPath.m_cAllPath.GetCount() < 1 )
		return 0 ;
	
	BOOL bFlag = m_cTransfDef.m_cProjtranDef.m_bProjtranFlag & NCDEF_PROJTRAN_KEEPCOR ? TRUE : FALSE; //从界面获取的参数:保留尖角
	DOUBLE dDist = m_cTransfDef.m_cProjtranDef.m_dPntDist ;
	if( dDist < 0.01 )
		dDist = 0.01 ;
	double dAng1 = 5.0 * PI1 / 6.0 ;
	CSmtCPathLib NewAllPath ;
	POSITION pos = AllPath.m_cAllPath.GetHeadPosition() ;
	while(pos)
	{
		CSmtCutPath* pCutPath = AllPath.m_cAllPath.GetNext(pos) ;
		if( pCutPath )
		{
			//1.抬高原始路径,且忽略非切削路径
//			pCutPath->TransPath( 0.0, 0.0, (TFLOAT)m_cTransfDef.m_cProjtranDef.m_dUpDist ) ;
			if( pCutPath->m_bFeedType != JDNC_FEEDTYPE_CONNECT && pCutPath->m_bFeedType != JDNC_FEEDTYPE_ROUGH)
			{
				CSmtCutPath* pNewPath = pCutPath->CopyMyself() ;
				NewAllPath.AddToTail(pNewPath) ;
				pCutPath = NULL ;
				continue ;
			}

			//2.在等分段处插入分割点
			TFLOAT fLen = pCutPath->NormalizeLen();
			int nSect = (int)(ceil(fLen/dDist)); 
			CSmtCutPoint* pCPnt = NULL ;
			TFLOAT t ;
			for( int i = 1 ; i < nSect ; i++ )
			{
				for( pCPnt = pCutPath->m_pHead ; pCPnt ; pCPnt = pCPnt->next )
				{
					TFLOAT fBnd =  (TFLOAT)i/nSect ;
					CSmtCutPoint* pNewCPnt = NULL ;
					if( fBnd <= pCPnt->m_fPoint[3] )
					{
						ASSERT(pCPnt->prev) ;
						t = fBnd - pCPnt->prev->m_fPoint[3] ;
						t /= TFLOAT( pCPnt->m_fPoint[3] - pCPnt->prev->m_fPoint[3] ) ;
						pNewCPnt = new CSmtCutPoint() ;
						pCPnt->prev->CalcMidPoint(pCPnt, t, pNewCPnt) ;
						pCutPath->InsertBefore(pNewCPnt, pCPnt) ;
						break ;
					}
				}
			}

			//3.在等分点处形成插铣段并连接
			CSmtCutPath* pCnctPath = new CSmtCutPath();
			pCnctPath->m_fFeedRate = 10.0 ;
			pCnctPath->m_bFeedType = JDNC_FEEDTYPE_CONNECT ;
			int k = 0 ;    //标记等分点序号
			for( pCPnt = pCutPath->m_pHead ; pCPnt ; pCPnt = pCPnt->next )
			{
				FPNT3D fPnt1, fPnt2;
				fPnt1[0] = fPnt2[0] = pCPnt->m_fPoint[0] ;
				fPnt1[1] = fPnt2[1] = pCPnt->m_fPoint[1] ;
				fPnt1[2] = fPnt2[2] = pCPnt->m_fPoint[2] ;
				BOOL bBreak = FALSE ;
				TFLOAT fBnd =  (TFLOAT)k/nSect ;
				TFLOAT diff = pCPnt->m_fPoint[3] - fBnd ;
				if( fabs(diff) < 1.0e-6 )
				{
					k++ ;
					fPnt2[2] += (TFLOAT)m_cTransfDef.m_cProjtranDef.m_dUpDist ;
					bBreak = TRUE ;
				}
				if( !bBreak && bFlag/*保留尖角*/ )
				{
					if( pCPnt->prev && pCPnt->next )
					{
						PNT3D prevPnt, curPnt, nxtPnt ;
						prevPnt[0] = pCPnt->prev->m_fPoint[0] ;
						prevPnt[1] = pCPnt->prev->m_fPoint[1] ;
						prevPnt[2] = pCPnt->prev->m_fPoint[2] ;
						curPnt[0] = pCPnt->m_fPoint[0] ;
						curPnt[1] = pCPnt->m_fPoint[1] ;
						curPnt[2] = pCPnt->m_fPoint[2] ;
						nxtPnt[0] = pCPnt->next->m_fPoint[0] ;
						nxtPnt[1] = pCPnt->next->m_fPoint[1] ;
						nxtPnt[2] = pCPnt->next->m_fPoint[2] ;
						double dAng = mathGetAngle3P( prevPnt, curPnt, nxtPnt ) ;
						if( dAng <= dAng1 )
						{
							fPnt2[2] += (TFLOAT)m_cTransfDef.m_cProjtranDef.m_dUpDist ;
							bBreak = TRUE ;
						}
					}
				}
				if( pCnctPath )
				{
					if( !bBreak ) fPnt2[2] +=  (TFLOAT)m_cTransfDef.m_cProjtranDef.m_dUpDist ;
					CSmtCutPoint* pLNew = new CSmtCutPoint(fPnt2) ;
					pCnctPath->AddTail(pLNew) ;
				}

				if( bBreak )
				{
					BOOL bNewCnct = FALSE ;
					if( pCnctPath->NumPoint() > 1 )
					{
						NewAllPath.AddToTail(pCnctPath) ;
					}
					else
					{
						delete pCnctPath ;
					}
					pCnctPath = NULL ;
					bNewCnct = TRUE ;
					// 生成插铣路径由高往低
					CSmtCutPath* pDownPath = new CSmtCutPath() ;
					CSmtCutPoint* pNewPnt1 = new CSmtCutPoint(fPnt1) ;
					CSmtCutPoint* pNewPnt2 = new CSmtCutPoint(fPnt2) ;
					pDownPath->AddTail( pNewPnt2 ) ;	// 高点
					pDownPath->AddTail( pNewPnt1 ) ;	// 低点
					// 抬到路径由低往高
					pDownPath->m_fFeedRate = 0.9f ;
					pDownPath->m_bFeedType = JDNC_FEEDTYPE_ROUGH ;
					CSmtCutPath* pUpPath = new CSmtCutPath() ;
					CSmtCutPoint* pNewPnt3 = new CSmtCutPoint(*pNewPnt2) ;	
					CSmtCutPoint* pNewPnt4 = new CSmtCutPoint(*pNewPnt1) ;
					pUpPath->AddTail(pNewPnt4) ;		// 低点
					pUpPath->AddTail(pNewPnt3) ;		// 高点
					
					pUpPath->m_fFeedRate = 10.0 ;
					pUpPath->m_bFeedType = JDNC_FEEDTYPE_QUICK ;
					NewAllPath.AddToTail(pDownPath) ;
					NewAllPath.AddToTail(pUpPath) ;

					if( bNewCnct && pCPnt->next)
					{
						pCnctPath = new CSmtCutPath();
						pCnctPath->m_fFeedRate = 10.0 ;
						pCnctPath->m_bFeedType = JDNC_FEEDTYPE_QUICK ;
					}
				}
				if( pCnctPath )
				{
					CSmtCutPoint* pNew = new CSmtCutPoint(fPnt2) ;
					pCnctPath->AddTail(pNew) ;
				}
			}
		}
	}

	CPathCombine * pPComb = new CPathCombine( NC_WPROCESS_IMPORT ) ;
	pPComb->m_pEntFrom  = NewPath.m_pEntFrom  ;
	MathCAM_ResetPathDepth( NewAllPath, 0 ) ;
	NewAllPath.DelPointOnLine(2.0e-4) ; // 删除同一直线上的点
	NewAllPath.AddToPathCombine(*pPComb);
	NewAllPath.ClearAllPath() ;
	AllPath.ClearAllPath() ;
	CPathGroup tmpPGroup(NC_PATHTYPE_PSURFACE) ;
	tmpPGroup.AddData( 0.0 , pPComb ) ;
	NewPath.ClearAll () ;   //删除原始路径
	while( tmpPGroup.m_pHead )
	{
		pPComb = tmpPGroup.m_pHead ;
		tmpPGroup.RemoveCombine( pPComb ) ;
		NewPath.AddData( 0.0, pPComb ) ;
	}
	return 1;
}
//类正弦曲线点波动系数
DOUBLE GetModCoef(TFLOAT t, TFLOAT tb1, TFLOAT tb2)
{
	DOUBLE tRetrun = 0.0 ;
	TFLOAT tdlt = tb2 - tb1;
	TFLOAT tMid[12];
	tMid[0] = tb1;
	tMid[11] = tb2 ;
	for( int i = 1; i <=10; i++)
	{
		tMid[i] = tMid[0] + (0.05f + 0.1f *(i-1) ) * tdlt ;
	}
	if( (t>tMid[5] && t< tMid[6]) ||
		fabs(t - tMid[5]) < 1.0e-4 ||
		fabs(t - tMid[6])< 1.0e-4 )
	{
		tRetrun = 1.0 ;
	}
	else if(t > tMid[1] && t < tMid[5])
	{
		DOUBLE dCoef = (t-tMid[1])/(tMid[5] - tMid[1]);
		dCoef = dCoef * PI1 / 2 ;
		tRetrun = sin(dCoef);
	}
	else if(t > tMid[6] && t < tMid[10])
	{
		DOUBLE dCoef = (tMid[10]-t) / (tMid[10]-tMid[6]);
		dCoef = dCoef * PI1 / 2 ;
		tRetrun = sin(dCoef);
	}
	if(tRetrun > 1.0)
		tRetrun = 1.0 ;
	return tRetrun ;
}
//磨削调整
BOOL CSmartPathGen::ModifyGrind( CPathGroup& NewPath, int nType, DOUBLE depthInc, DOUBLE dist )
{
	CSmtCPathLib AllPath ;
	//将PathGroup转为CSmtCutPath
	MathCAM_AddPathGroup( AllPath, NewPath, m_cSetupDef.m_cTolDef , TRUE ) ;
	//得到CSmtCutPath上的刀位点
	if( AllPath.m_cAllPath.GetCount() < 1 )
		return FALSE ;
	int nPnt = AllPath.GetNumPoint() ;
	if( nPnt < 1 ) 
		return FALSE ;
	if( nType == 0 || nType == 2)//三角或梯形
	{
		if(dist < 0.01) dist = 0.01 ;
	}
	else 
	{
		if(dist < 0.05) dist = 0.05 ;
	}
	CSmtCPathLib NewAllPath ;
	TFLOAT t = 0.0, fLen = 0.0, fBnd1 = 0.0, fBnd2 = 0.0, fMid = 0.0;
	DOUBLE dCoef = 0.0 ;
	int k = 0 ;    //标记等分点序号
	POSITION pos = AllPath.m_cAllPath.GetHeadPosition() ;
	while(pos)
	{
		CSmtCutPath* pCutPath = AllPath.m_cAllPath.GetNext(pos) ;
		if( pCutPath )
		{
			//1.非切削路径直接保留
			if( pCutPath->m_bFeedType != JDNC_FEEDTYPE_CONNECT && pCutPath->m_bFeedType != JDNC_FEEDTYPE_ROUGH)
			{
				CSmtCutPath* pCopyPath = pCutPath->CopyMyself() ;
				NewAllPath.AddToTail(pCopyPath) ;
				pCutPath = NULL ;
				continue ;
			}

			//2.在等分段处插入分割点
			fLen = pCutPath->NormalizeLen();
			int nSect = (int)(fLen/dist); 
			int nSect2 = nSect ;
			if(nType == 0 || nType == 2)
			{
				nSect2 = 2 * nSect ; 
			}
			else if(nType == 1)
			{
				nSect2 = 20 * nSect ;
			}
			CSmtCutPoint* pCPnt = NULL ;
			for( int i = 1 ; i < nSect2 ; i++ )
			{
				fBnd1 = (TFLOAT) i/nSect2 ;
				for( pCPnt = pCutPath->m_pHead ; pCPnt ; pCPnt = pCPnt->next )
				{
					CSmtCutPoint* pNewCPnt = NULL ;
					if( fBnd1 <= pCPnt->m_fPoint[3])
					{
						ASSERT(pCPnt->prev) ;
						t = fBnd1 - pCPnt->prev->m_fPoint[3] ;
						t /= TFLOAT( pCPnt->m_fPoint[3] - pCPnt->prev->m_fPoint[3] ) ;
						pNewCPnt = new CSmtCutPoint() ;
						pCPnt->prev->CalcMidPoint(pCPnt, t, pNewCPnt) ;
						pCutPath->InsertBefore(pNewCPnt, pCPnt) ;
						break ;
					}
				}
			}

			//3.在分段路径内构造波动路径点并连接
			BOOL bReverse = TRUE; //错峰标记
			CSmtCutPath* pNewPath = NULL ;
			if(nType == 0)//三角形
			{
				if(!bReverse)
				{
					pCutPath->TransPath(0.0f, 0.0f, TFLOAT(-depthInc));
				}
				for( k = 0 ; k < nSect ; k++)
				{
					fBnd1 = (TFLOAT)k/nSect ;
					fBnd2 = (TFLOAT)(k+1)/nSect ; 
					fMid = (fBnd1 + fBnd2) / 2.0f ;
					for( pCPnt = pCutPath->m_pHead ; pCPnt ; pCPnt = pCPnt->next )
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
								pCPnt->m_fPoint[2] += TFLOAT(dCoef * depthInc) ;
							else
								pCPnt->m_fPoint[2] -= TFLOAT(dCoef * depthInc) ;
						}
						else if(pCPnt->m_fPoint[3] > fMid)
						{
							ASSERT(fBnd2 - fMid > 1.0e-8);
							dCoef = (fBnd2 - pCPnt->m_fPoint[3])/(fBnd2 - fMid);
							if(!bReverse)
								pCPnt->m_fPoint[2] += TFLOAT(dCoef * depthInc) ;
							else
								pCPnt->m_fPoint[2] -= TFLOAT(dCoef * depthInc) ;
						}
						else
						{
							if(!bReverse)
								pCPnt->m_fPoint[2] += TFLOAT(depthInc) ;
							else
								pCPnt->m_fPoint[2] -= TFLOAT(depthInc) ;
						}
					}
				}
				pNewPath = pCutPath->CopyMyself();
			}
			else if(nType == 1)//类正弦
			{
				if(!bReverse)
				{
					pCutPath->TransPath(0.0f, 0.0f, TFLOAT(-depthInc));
				}	
				for( k = 0 ; k < nSect ; k++)
				{
					fBnd1 = (TFLOAT)k/nSect ;
					fBnd2 = (TFLOAT)(k+1)/nSect ; 
					for( pCPnt = pCutPath->m_pHead ; pCPnt ; pCPnt = pCPnt->next )
					{
						if(pCPnt->m_fPoint[3] < fBnd1)
							continue ;
						if(pCPnt->m_fPoint[3] > fBnd2)
						{
							break ;
						}
						dCoef = GetModCoef(pCPnt->m_fPoint[3], fBnd1, fBnd2);
						if(!bReverse)
						{
							if(fabs(dCoef - 1.0 ) < 1.0e-4)
								pCPnt->m_fPoint[2] += TFLOAT(depthInc) ;
							else if(fabs(dCoef)>1.0e-4)
								pCPnt->m_fPoint[2] += TFLOAT(dCoef * depthInc) ;
						}
						else
						{
							if(fabs(dCoef - 1.0 ) < 1.0e-4)
								pCPnt->m_fPoint[2] -= TFLOAT(depthInc) ;
							else
								pCPnt->m_fPoint[2] -= TFLOAT(dCoef * depthInc) ;
						}
					}
				}	
				pNewPath = pCutPath->CopyMyself();
			}
			else if(nType == 2)//梯形
			{
				nSect = nSect * 2 ;
				pNewPath = new CSmtCutPath();
				pNewPath->m_bFeedType = JDNC_FEEDTYPE_ROUGH ;
				CSmtCutPath* pCutPath2 = pCutPath->CopyMyself();
				if(!bReverse)
				{
					pCutPath2->TransPath(0.0f, 0.0f, TFLOAT(-depthInc));
				}
				else
				{
					pCutPath->TransPath(0.0f, 0.0f, TFLOAT(-depthInc));
				}
				for( k = 0 ; k < nSect ; k++)
				{
					fBnd1 = (TFLOAT)k/nSect ;
					fBnd2 = (TFLOAT)(k+1)/nSect ;
					if(k %2 == 0)
					{
						for( pCPnt = pCutPath2->m_pHead ; pCPnt ; pCPnt = pCPnt->next )
						{
							if(pCPnt->m_fPoint[3] < fBnd1)
								continue ;
							if(pCPnt->m_fPoint[3] > fBnd2)
							{				
								break ;
							}
							CSmtCutPoint* pNewCPnt = pCPnt->CopyMyself();
							pNewPath->AddTail(pNewCPnt);
						}

					}
					else
					{
						for( pCPnt = pCutPath->m_pHead ; pCPnt ; pCPnt = pCPnt->next )
						{
							if(pCPnt->m_fPoint[3] < fBnd1)
								continue ;
							if(pCPnt->m_fPoint[3] > fBnd2)
							{				
								break ;
							}
							CSmtCutPoint* pNewCPnt = pCPnt->CopyMyself();
							pNewPath->AddTail(pNewCPnt);
						}
					}
				}
				if(!bReverse)
				{
					CSmtCutPoint* pNewCPnt = pCutPath->m_pTail->CopyMyself();
					pNewPath->AddTail(pNewCPnt);
				}
				else
				{
					CSmtCutPoint* pNewCPnt = pCutPath2->m_pTail->CopyMyself();
					pNewPath->AddTail(pNewCPnt);
				}
				if(pCutPath2)
				{
					pCutPath2->ClearAllPoint();
					delete pCutPath2;
				}
				pCutPath2 = NULL ;
			}
			NewAllPath.AddToTail(pNewPath) ;
		}
	}
	
	CPathCombine * pPComb = new CPathCombine( NC_WPROCESS_ROUGH ) ;
	pPComb->m_pEntFrom  = NewPath.m_pEntFrom  ;
	NewAllPath.DelPointOnLine(2.0e-4) ; // 删除同一直线上的点
	NewAllPath.AddToPathCombine(*pPComb);
	CPathGroup tmpPGroup(NC_PATHTYPE_PSURFACE) ;
	tmpPGroup.AddData( 0.0 , pPComb ) ;
	NewAllPath.ClearAllPath() ;
	AllPath.ClearAllPath() ;
	NewPath.ClearAll () ;   //删除原始路径
	while( tmpPGroup.m_pHead )
	{
		pPComb = tmpPGroup.m_pHead ;
		tmpPGroup.RemoveCombine( pPComb ) ;
		NewPath.AddData( 0.0, pPComb ) ;
	}
	return TRUE ;
}
// add by zy 2013.1.17
BOOL CSmartPathGen::ModifyGrindNew( CPathGroup& NewPath, int nType, DOUBLE depthInc, DOUBLE dist ,int  SinSegs)
{
	CSmtCPathLib AllPath ;	
	//将PathGroup转为CSmtCutPath
	MathCAM_AddPathGroup( AllPath, NewPath, m_cSetupDef.m_cTolDef , TRUE ) ;
	//得到CSmtCutPath上的刀位点
	if( AllPath.m_cAllPath.GetCount() < 1 )
		return FALSE ;
	int nPnt = AllPath.GetNumPoint() ;
	if( nPnt < 1 ) 
		return FALSE ;
	CSmtCutPath* pCutPath = NULL ;
	POSITION pos = AllPath.m_cAllPath.GetHeadPosition(), atpos = NULL ;
	while(pos)
	{
		atpos = pos ;
		pCutPath = AllPath.m_cAllPath.GetNext(pos) ;

		if( !pCutPath ) 
		{
			AllPath.m_cAllPath.RemoveAt( atpos ) ;
			continue ;
		}

		//1.非切削路径直接保留
		if( pCutPath->m_bFeedType != JDNC_FEEDTYPE_CONNECT && pCutPath->m_bFeedType != JDNC_FEEDTYPE_ROUGH)
		{
			continue ;
		}		
		//2.在等分段处插入分割点
		TFLOAT fLen = pCutPath->NormalizeLen();		
		//根据波长计算段数，此时进行四舍五入的处理，之前是直接取整；
		double  Sect = 0. ;
		int     nSect = 0 ,nSect2 = 0 ;		
		Sect  = fLen/dist;
		nSect = int(Sect+0.5);
		if(nType == 1)
		{
			nSect2 = SinSegs * nSect ;
		}
		else 
		{
			nSect2 = 2 * nSect ; 
		}
		//插入新等分的节点
		InsertNewCutPoint(pCutPath,nSect2);
//#ifdef _DEBUG
		TransPointToCutPointEx(AllPath,pCutPath);
//#endif
		
			
		//3.在分段路径内构造波动路径点并连接		
		if(nType == 0)//三角形
		{
			CreateTriWave(pCutPath,depthInc,nSect);
		}
		else if(nType == 1)//类正弦
		{	
			CreateSinWave(pCutPath,depthInc,nSect);			
		}
		else if(nType == 2)//梯形
		{				
			CreateTrapWave(pCutPath,depthInc,nSect);
		}
	}

	CPathCombine * pPComb = new CPathCombine( NC_WPROCESS_ROUGH ) ;
	pPComb->m_pEntFrom  = NewPath.m_pEntFrom  ;
	AllPath.DelPointOnLine(2.0e-4);// 删除同一直线上的点
//#ifdef _DEBUG
	MatchSpeedInModifyGrind(AllPath,*pPComb);//重新匹配速度
//#endif
	AllPath.AddToPathCombine(*pPComb);
	NewPath.ClearAll () ;   //删除原始路径
	NewPath.AddData(0,pPComb);
	AllPath.ClearAllPath() ;	
	return TRUE ;
}

void CSmartPathGen::MdfyGrndFor3DPath(CPathGroup &NewPath, CSmartGraphic& Graph)
{
	if (glbf_GetProductID() == PRODUCT_ID_SURFMILL_PRO)
	{// 个人版软件不支持磨削调整功能，2014.10.15 liuxin
		return;
	}
	if (m_cMethodDef.m_nMethodType == surfncMethodLoop)
	{
		JDNC_LOOP cParam = m_cMethodDef.m_cLoopCut;
		if ((cParam.m_bLoopFlag & NCDEF_FLOOP_GRNDMDFY) && 
			cParam.m_nMdfyPathType == NCDEF_FLOOP_MDFY_WAVEPATH)
		{// 磨削调整
			BOOL bAlongCone = cParam.m_bLoopFlag & NCDEF_FLOOP_MDFYALONGCONE ? TRUE : FALSE;
			BOOL bReadTime = cParam.m_bLoopFlag & NCDEF_FLOOP_MDFYREADTIME ? TRUE : FALSE;
			MdfyGrndFor3DPath(NewPath, Graph, cParam.m_nMdfyType, cParam.m_nRCompSide, cParam.m_nMdfyDir,
							  cParam.m_dMdfyHeit, cParam.m_dMdfyDist, bAlongCone, bReadTime);
		}
	}
	else if (m_cMethodDef.m_nMethodType == surfncMethodCurve)
	{
		JDNC_CURVE cParam = m_cMethodDef.m_cCurveCut;
		if ((cParam.m_bCurveFlag & NCDEF_FCURVE_GRNDMDFY) && 
			cParam.m_nMdfyPathType == NCDEF_FCURVE_MDFY_WAVEPATH)
		{// 磨削调整
			BOOL bAlongCone = cParam.m_bCurveFlag & NCDEF_FCURVE_MDFYALONGCONE ? TRUE : FALSE;
			BOOL bReadTime = cParam.m_bCurveFlag & NCDEF_FCURVE_MDFYREADTIME ? TRUE : FALSE;
			MdfyGrndFor3DPath(NewPath, Graph, cParam.m_nMdfyType, cParam.m_nRCompSide, cParam.m_nMdfyDir,
							  cParam.m_dMdfyHeit, cParam.m_dMdfyDist, bAlongCone, bReadTime);
		}
	}
}

BOOL CSmartPathGen::MdfyGrndFor3DPath(CPathGroup	& NewPath,
									  CSmartGraphic & Graph,
									  int			WaveType,
									  int			RCompSide, 
									  int			MdfyDir, 
									  DOUBLE		MdfyHeit,
									  DOUBLE		Freq,
									  BOOL			AlongCone,
									  BOOL			ReadTime)
{
	if (!m_pTool)
	{
		return FALSE;
	}
	// step 1: 将PathGroup转为CSmtCutPath
	CSmtCPathLib AllPath ;	
	MathCAM_AddPathGroup( AllPath, NewPath, m_cSetupDef.m_cTolDef , TRUE ) ;
	//得到CSmtCutPath上的刀位点
	if( AllPath.m_cAllPath.GetCount() < 1 )
		return FALSE ;
	int nPnt = AllPath.GetNumPoint() ;
	if( nPnt < 1 ) 
		return FALSE ;

	double dTotalTime = 0.;
	double dSectCnt = 0.0;
	int nSectCnt = 0, nStart = 0, nEnd = 0;
	double dAngle = 0.0, dHeight = MdfyHeit;;
	if (AlongCone && RCompSide != 2 && m_pTool->GetType() == surfncToolAFlat && m_pTool->m_fAngle < PI1_2 - MIN_LEN)
	{// 锥刀考虑锥角补偿
		dAngle = m_pTool->m_fAngle;
		dHeight /= cos(dAngle);
	}
	// step 2: 转成五轴路径计算刀触点并连接进退刀路径
	// 获取坐标系和原始曲线
	CPtrList listCur;
	if (!Graph.GetAllEntity( m_cSetupDef, listCur, SMARTGRAPH_TYPE_CURVE2D ) || listCur.IsEmpty())
	{
		return FALSE;
	}
	BOOL bReverse = FALSE;
	if ((GetMillDir() == NCDEF_MILLDIR_CLIMB && RCompSide == 1) ||
		(GetMillDir() == NCDEF_MILLDIR_CONVENTIONAL && RCompSide != 1))
	{// 路径方向与曲线方向相反时，需要对曲线反向，2014.9.30 liuxin
		bReverse = TRUE;
		MathCam_ReverseAllRedepthCur(listCur);
	}
	// 获取局部坐标系
	RFRAME LocFrame ;
	if( !Graph.GetTPlaneFrame( LocFrame ) )
	{
		mathInitRFrame( &LocFrame ) ;	
	}
	if (m_c5DCtrlDef.m_cToolAxisApp.m_nFrameFlag != NCDEF_FRAME_DEFAULT)
	{
		mathMultRFrame( &LocFrame, &m_c5DCtrlDef.m_cToolAxisApp.m_dIndexFrame, &LocFrame ) ;
	}
	// 三轴路径转成五轴路径
	int nRetFlag = TransCPathToCPath5x(AllPath.m_cAllPath, listCur, LocFrame, RCompSide, dAngle);
	if (bReverse)
	{// 恢复曲线方向
		MathCam_ReverseAllRedepthCur(listCur);
	}
	MathCam_DeleteAllRedepthCur( listCur ) ;
	if (!nRetFlag)
	{
		return FALSE;
	}

	// step 3: 波动路径
	BOOL bReadTime = ReadTime;
	if (WaveType != 1)
	{
		bReadTime = FALSE;
	}
	int nRoughCnt = 0;
	CSmtCutPath* pCutPath = NULL, *pPrevPath = NULL;
	POSITION pos = AllPath.m_cAllPath.GetHeadPosition(), atpos = NULL ;
	if (bReadTime)
	{// 统计切削路径条数，如果是多条切削路径，则不能读取控制参数，2014.3.19 liuxin
		while (pos)
		{
			pCutPath = AllPath.m_cAllPath.GetNext(pos) ;
			if (pCutPath && pCutPath->m_bFeedType == JDNC_FEEDTYPE_ROUGH)
			{
				++nRoughCnt;
			}
			if (nRoughCnt > 1)
			{
				bReadTime = FALSE;
				LPSTR lpBuffer = glbf_GetPromptString() ;
				glbf_LoadSysString ( IDS_MDFYGRND_ONLYSUPPORTONE, lpBuffer, MAX_STR_LENG ) ;
				AfxMessageBox( lpBuffer );
				break;
			}
		}
	}

	pos = AllPath.m_cAllPath.GetHeadPosition(), atpos = NULL ;
	while(pos)
	{
		atpos = pos ;
		pPrevPath = pCutPath;
		pCutPath = AllPath.m_cAllPath.GetNext(pos) ;
		if( !pCutPath || pCutPath->m_bFeedType != JDNC_FEEDTYPE_ROUGH) 
		{
			continue ;
		}	
		// step 3.1: //根据波长计算段数
		if (pPrevPath == NULL || pPrevPath->m_bFeedType != JDNC_FEEDTYPE_LEAD)
		{
			nStart = 0;
			nEnd = pCutPath->m_nNumPnt;
		}
		else
		{
			nStart = pPrevPath->m_nNumPnt - 1;
			nEnd = pCutPath->m_nNumPnt + nStart;
		}
		// step 3.2: 计算周期数	
		dTotalTime = NormalizeLenByCutTime(pCutPath, nStart, nEnd, bReadTime, WaveType == 3);
		if (dTotalTime < 0)
		{
			continue;
		}
		dSectCnt  = dTotalTime * Freq;
		nSectCnt = int(dSectCnt + 0.5);
		if (nSectCnt < 1)
		{
			nSectCnt = 1;
		}
		if( nSectCnt > 10000 ) 
		{// 限制周期数最多为10000个
			nSectCnt = 10000 ;
		}
		// 在波峰波谷插入节点
		if ((WaveType == 0 || WaveType == 1) && MdfyDir == NCDEF_FCURVE5D_MDFYDIR_MID)
		{
			InsertNewCutPoint(pCutPath, 4 * nSectCnt, FALSE);
		}
		else
		{
			InsertNewCutPoint(pCutPath, 2 * nSectCnt);
		}

		// step 3.3: 在分段路径内构造波动路径点并连接		
		if(WaveType == 0)//三角形
		{
			CreateTriWaveFor3DPath(pCutPath, MdfyDir, dHeight, nSectCnt);
		}
		else if(WaveType == 1)//类正弦
		{	
			CreateSinWaveFor3DPath(pCutPath, MdfyDir, dHeight, nSectCnt);			
		}
		else if(WaveType == 2)//梯形
		{				
			CreateTrapWaveFor3DPath(pCutPath, MdfyDir, dHeight, nSectCnt);			
		}
		else if (WaveType == 3)//摆线
		{
			CreateTrochoidalWaveFor3DPath(pCutPath, MdfyDir, dHeight, nSectCnt);
		}
	}
	// step 4: 将CSmtCutPath转为PathGroup
	CPathCombine * pPComb = new CPathCombine( NC_WPROCESS_ROUGH ) ;
	if (!MatchSpeedForMdfyGrnd(AllPath, *pPComb))
	{//重新匹配速度
		pPComb->ClearAll();
		delete pPComb;
	}
	else
	{
		AllPath.AddToPathCombine ( *pPComb ) ;
		NewPath.ClearAll(); // 删除原始路径
		NewPath.AddData ( 0.0, pPComb ) ;
	}

	return TRUE ;
}

BOOL CSmartPathGen::TransCPathToCPath5x(CSmtCPathList &AllPath, CPtrList& CurveList, RFRAME& LocFrame, int RCompSide /* = 2 */, DOUBLE Angle /* = 0.0 */)
{
	if (AllPath.IsEmpty() || CurveList.IsEmpty())
	{
		return FALSE;
	}
	BOOL bConvertLead = FALSE;
	CSmtCutPointEx *pCPntEx = NULL;
	CSmtCutPath* pCutPath = NULL, *pPrevPath = NULL, *pNextPath = NULL;
	POSITION pos = AllPath.GetHeadPosition(), atpos = NULL;
	// 将退刀路径转成切削路径并连接
	while(pos)
	{
		atpos = pos ;
		pCutPath = AllPath.GetNext(pos) ;
		if (pos)
		{
			pNextPath = AllPath.GetAt(pos);
		}
		else
		{
			pNextPath = NULL;
		}
		if( !pCutPath || pCutPath->m_bFeedType != JDNC_FEEDTYPE_ROUGH) 
		{
			continue ;
		}
		// 转成五轴路径
		CSmtCutPath* pCPath5x = TransCPathToCPath5x(pCutPath, CurveList, LocFrame, RCompSide, Angle);
		if (pCPath5x)
		{
			delete pCutPath;
			pCutPath = pCPath5x;
			AllPath.SetAt(atpos, pCPath5x);
		}
		// 向后找退刀
		if (bConvertLead)
		{
			while (pNextPath && pNextPath->m_bFeedType == JDNC_FEEDTYPE_LEAD && 
				nc_Distance(pCutPath->m_pTail->m_fPoint, pNextPath->m_pHead->m_fPoint, 3) < MIN_DIS)
			{
				// 转成五轴路径
				pCPath5x = TransCPathToCPath5x(pNextPath, CurveList, LocFrame, RCompSide, Angle);
				if (!pCPath5x)
				{
					break;
				}
				CalcLeadOutPathSurfPos(pCutPath, pCPath5x);
				pCPntEx = (CSmtCutPointEx *)pCPath5x->m_pHead;
				pCPath5x->RemovePoint(pCPntEx);
				delete pCPntEx;
				while (pCPath5x->m_pHead)
				{
					pCPntEx = (CSmtCutPointEx *)pCPath5x->m_pHead;
					pCPath5x->RemovePoint(pCPntEx);
					pCutPath->AddTail(pCPntEx);
				}
				delete pNextPath;
				delete pCPath5x;
				AllPath.RemoveAt(pos);
				pos = atpos;
				AllPath.GetNext(pos);
				if (!pos)
				{
					break;
				}
				pNextPath = AllPath.GetAt(pos);
			}
		}
	}
	if (bConvertLead)
	{
		// 将进刀路径转成切割路径并连接
		pos = AllPath.GetTailPosition(), atpos = NULL ;
		while(pos)
		{
			atpos = pos ;
			pCutPath = AllPath.GetPrev(pos) ;
			if (pos)
			{
				pPrevPath = AllPath.GetAt(pos);
			}
			else
			{
				pPrevPath = NULL;
			}
			if( !pCutPath || pCutPath->m_bFeedType != JDNC_FEEDTYPE_ROUGH ) 
			{
				continue ;
			}	
			// 向前找进刀
			while (pPrevPath && pPrevPath->m_bFeedType == JDNC_FEEDTYPE_LEAD && 
				nc_Distance(pCutPath->m_pHead->m_fPoint, pPrevPath->m_pTail->m_fPoint, 3) < MIN_DIS)
			{
				// 转成五轴路径
				CSmtCutPath* pCPath5x = TransCPathToCPath5x(pPrevPath, CurveList, LocFrame, RCompSide, Angle);
				if (!pCPath5x)
				{
					break;
				}
				CalcLeadInPathSurfPos(pCutPath, pCPath5x);
				pCPntEx = (CSmtCutPointEx *)pCPath5x->m_pTail;
				pCPath5x->RemovePoint(pCPntEx);
				delete pCPntEx;
				while (pCPath5x->m_pTail)
				{
					pCPntEx = (CSmtCutPointEx *)pCPath5x->m_pTail;
					pCPath5x->RemovePoint(pCPntEx);
					pCutPath->AddHead(pCPntEx);
				}

				delete pPrevPath;
				delete pCPath5x;
				AllPath.RemoveAt(pos);
				pos = atpos;
				AllPath.GetPrev(pos);
				if (!pos)
				{
					break;
				}
				pPrevPath = AllPath.GetAt(pos);
			}
		}
	}

	return TRUE;
}

CSmtCutPath* CSmartPathGen::TransCPathToCPath5x(CSmtCutPath* CutPath, CPtrList& CurveList, RFRAME& LocFrame, int RCompSide, DOUBLE Angle)
{
	if (!CutPath || !CutPath->m_pHead || CurveList.IsEmpty())
	{
		return NULL;
	}

	int i = 0, nMillDir = GetMillDir();
	PNT3D pivot = {0};
	VEC3D vAxis = {0.0, 0.0, 1.0}, vZ = {0.0, 0., 1.0}, vFeed[3] = {0};
	vFeed[0][0] = vFeed[1][0] = vFeed[2][0] = 1.0;
	CSmtCutPoint* pCPnt = NULL ;
	CSmtCutPointEx* pCPntEx = NULL ;

	CSmtCutPath* pNewPath = new CSmtCutPath( MINI_MILL5AX_PATH ) ;
	pNewPath->m_bFeedType = CutPath->m_bFeedType ;
	pNewPath->m_bMoveFlag = CutPath->m_bMoveFlag;
	pNewPath->m_fFeedRate = CutPath->m_fFeedRate;	
	for( pCPnt = CutPath->m_pHead; pCPnt; pCPnt = pCPnt->next )
	{
		pCPntEx = new CSmtCutPointEx() ;
		mathFCpyPnt4D( pCPnt->m_fPoint, pCPntEx->m_fPoint  ) ;
		mathFCpyPnt( pCPnt->m_fPoint, pCPntEx->m_fSurfPos) ;
		pNewPath->AddTail ( pCPntEx ) ;
	}

	// 设置刀轴方向
	CGeoCurve* pFindCur = NULL;
	double t = 0.0;
	VEC3D vTan = {1.0, 0.0, 0.0};
	PNT3D pnt ={0.}, ptNear = {0.};
	for (pCPntEx = (CSmtCutPointEx*)pNewPath->m_pHead; pCPntEx; pCPntEx = (CSmtCutPointEx *)pCPntEx->next)
	{
		if (pNewPath->m_bFeedType == JDNC_FEEDTYPE_ROUGH)
		{
			nc_FloatToDouble(pnt, pCPntEx->m_fPoint, 3);
			pFindCur = SnapNearestPoint(CurveList, LocFrame, pnt, t, ptNear, vTan);
			if (pFindCur)
			{
				nc_DoubleToFloat(pCPntEx->m_fSurfPos, ptNear, 3);
			}
		}		
		if (RCompSide == 2 || Angle < MIN_LEN || Angle > PI1_2 - MIN_LEN)
		{
			mathCpyPnt(vZ, vAxis);
		}
		else
		{
			if (pCPntEx->prev == NULL)
			{
				for (i = 0; i < 2; ++i)
				{
					vFeed[2][i] = pCPntEx->next->m_fPoint[i] - pCPntEx->m_fPoint[i]; 
				}
				mathUniVec2D(vFeed[2], MIN_LEN);
			}
			else if (pCPntEx->next == NULL)
			{
				for (i = 0; i < 2; ++i)
				{
					vFeed[2][i] = pCPntEx->m_fPoint[i] - pCPntEx->prev->m_fPoint[i];
				}
				mathUniVec2D(vFeed[2], MIN_LEN);
			}
			else
			{
				for (i = 0; i < 2; ++i)
				{
					vFeed[0][i] = pCPntEx->m_fPoint[i] - pCPntEx->prev->m_fPoint[i];
					vFeed[1][i] = pCPntEx->next->m_fPoint[i] - pCPntEx->m_fPoint[i]; 
				}
				mathUniVec2D(vFeed[0], MIN_LEN);
				mathUniVec2D(vFeed[1], MIN_LEN);
				for (i = 0; i < 2; ++i)
				{
					vFeed[2][i] = vFeed[0][i] + vFeed[1][i]; 
				}
				mathUniVec2D(vFeed[2], MIN_LEN);
			}
			if (RCompSide != 2)
			{
				if (nMillDir == NCDEF_MILLDIR_CLIMB)
				{
					mathRotVec(vFeed[2], pivot, Angle, vZ, vAxis);
				}
				else if (nMillDir == NCDEF_MILLDIR_CONVENTIONAL)
				{
					mathRotVec(vFeed[2], pivot, -Angle, vZ, vAxis);
				}
			}
		}		

		nc_DoubleToFloat(pCPntEx->m_fSurfNor, vAxis, 3);
	}

	return pNewPath;
}

void CSmartPathGen::InsertCPntExBy2DStep(CSmtCutPath* CPath5x, DOUBLE Step)
{
	if (!CPath5x || CPath5x->GetCutMode() != MINI_MILL5AX_PATH)
	{
		return;
	}

	int i = 0, nInsert = 0;
	VEC3D vAxis = {0.};
	double dt = 0., dAzimGap = 0., dAzim[3] = {0.}, dElev[3] = {0.};
	double dDist2D = 0.0, dMaxStep = Step + MIN_DIS;
	CSmtCutPointEx* pCPntEx0 = NULL, *pCPntEx1 = NULL;
	pCPntEx0 = (CSmtCutPointEx*)CPath5x->m_pHead;
	pCPntEx1 = (CSmtCutPointEx*)pCPntEx0->next;
	while (pCPntEx0 && pCPntEx1)
	{
		dDist2D = nc_Distance(pCPntEx0->m_fPoint, pCPntEx1->m_fPoint, 2);
		if (dDist2D > dMaxStep)
		{
			nInsert = int(dDist2D/Step + 0.5);
			nc_FloatToDouble(vAxis, pCPntEx0->m_fSurfNor, 3);
			dAzim[0] = cur5ax_CalAzimuth(vAxis);
			dElev[0] = cur5ax_CalElevation(vAxis);
			nc_FloatToDouble(vAxis, pCPntEx1->m_fSurfNor, 3);
			dAzim[1] = cur5ax_CalAzimuth(vAxis);
			dElev[1] = cur5ax_CalElevation(vAxis);
			dAzimGap = dAzim[1] - dAzim[0];
			if (dAzimGap > 90)
			{
				while (dAzimGap > 90)
				{
					dAzim[1] -= 180;
					dAzimGap = dAzim[1] - dAzim[0];
				}
			}
			else if (dAzimGap < -90)
			{
				while (dAzimGap < -90)
				{
					dAzim[1] += 180;
					dAzimGap = dAzim[1] - dAzim[0];
				}
			}
			for (i = 1; i < nInsert; ++i)
			{
				dt = i * 1.0/nInsert;
				dAzim[2] = (1.0 - dt) * dAzim[0] + dt * dAzim[1];
				dElev[2] = (1.0 - dt) * dElev[0] + dt * dElev[1];				
				while (dAzim[2] < 0)
				{
					dAzim[2] += 360;
				}
				while (dAzim[2] > 360)
				{
					dAzim[2] -= 360;
				}
				cur5ax_CalAxisFromAngle(dElev[2], dAzim[2], vAxis);
#ifdef _DEBUG
				if (fabs(dAzim[0] - dAzim[1]) > 180)
				{
					ASSERT(0);
				}
#endif
				CSmtCutPointEx* pNewCPntEx = new CSmtCutPointEx;
				nc_GetPointAtLine(pCPntEx0->m_fPoint, pCPntEx1->m_fPoint, TFLOAT(dt), pNewCPntEx->m_fPoint, 4);
				nc_GetPointAtLine(pCPntEx0->m_fSurfPos, pCPntEx1->m_fSurfPos, TFLOAT(dt), pNewCPntEx->m_fSurfPos, 3);
				nc_DoubleToFloat(pNewCPntEx->m_fSurfNor, vAxis, 3);
				CPath5x->InsertBefore(pNewCPntEx, pCPntEx1);
			}
		}
		pCPntEx0 = pCPntEx1;
		pCPntEx1 = (CSmtCutPointEx*)pCPntEx0->next;
	}
}

CGeoCurve* CSmartPathGen::SnapNearestPoint( CPtrList& CurveList, RFRAME& LocFrame, PNT3D In_Pnt, DOUBLE &In_t, PNT3D Ot_Pnt, VEC3D Ot_Tan)
{
	DOUBLE dDist = 0.;
	DOUBLE dMinDist = 1.0e6, tmpt = In_t, t = 0.;
	int nRet = 0 ;
	PNT3D o_p, near_p ;
	CGeoInter geoint;
	BOOL bMultiCurve = FALSE ;
	// 如果是单根线，使用i_t用于追踪样条曲线，保证点的精度
	// 因为GetNearestPnt函数中的初始迭代函数精度较低，容易丢失点
	// 如果有多条曲线则直接取-1
	if( CurveList.GetCount () > 1 ) 
	{
		bMultiCurve = TRUE ;
		tmpt = -1.;
	}

	CRedepthCurve *pDepCurve = NULL, *pFindCur = NULL;
	CGeoCurve *pCurve3D = NULL, *pCurve2D = NULL ;
	POSITION pos = CurveList.GetHeadPosition() ;
	while( pos )
	{
		pDepCurve = ( CRedepthCurve *) CurveList.GetNext( pos ) ;
		if( ! pDepCurve ) continue ;
		pCurve3D = pDepCurve->m_pCurve3D ;
		pCurve2D = pDepCurve->m_pCurve2D ;
		if( !pCurve2D ) continue ;

		// 计算点到2D曲线的最近点，得到参数域t,保留曲线高度
		if( tmpt < 0 )
		{
			nRet = pCurve2D->GetNearestPnt( In_Pnt, tmpt, o_p, dDist ) ;
		}
		else
		{
			nRet = geoint.IterateCurveNearestPnt( pCurve2D, In_Pnt, tmpt, o_p, dDist ) ;
		}
		if( nRet && dDist < dMinDist )
		{
			dMinDist = dDist ;
			t = tmpt ;
			pFindCur = pDepCurve ;
			mathCpyPnt( o_p, near_p ) ;
		}

		if( bMultiCurve )
		{
			tmpt = -1. ;
		}

	}
	// 获取保留曲线高度的参数
	if( pFindCur && pFindCur->m_pCurve3D) 
	{
		pFindCur->m_pCurve2D->GetTangent(&LocFrame, t, Ot_Pnt, Ot_Tan);
		In_t = t;
	}

	return pFindCur->m_pCurve2D ;
}

void MathCAM_FAddVec ( TFLOAT Pnt[3], TFLOAT Vec[3], TFLOAT Scale, TFLOAT New_p[3] )
{
	New_p[0] = Pnt[0] + Scale * Vec[0] ;
	New_p[1] = Pnt[1] + Scale * Vec[1] ;
	New_p[2] = Pnt[2] + Scale * Vec[2] ;
}

void CSmartPathGen::CreateSinWaveFor3DPath(CSmtCutPath * CutPath5x, int MdfyDir, DOUBLE MdfyHeit, int SectCnt)
{
	if (!CutPath5x || CutPath5x->GetCutMode() != MINI_MILL5AX_PATH)
	{
		return;
	}
	CSmtCutPointEx* pCPntEx = NULL;
	TFLOAT fBnd1 = 0.0, fBnd2 = 0.0;
	DOUBLE dCoef = 0.0, dHeight = MdfyHeit * 0.5, dZOff = 0.0;
	int k = 0 ;    //标记等分点序号

	for( k = 0 ; k < SectCnt ; k++)
	{
		fBnd1 = (TFLOAT)k/SectCnt ;
		fBnd2 = (TFLOAT)(k+1)/SectCnt ;
		for( pCPntEx = (CSmtCutPointEx*)CutPath5x->m_pHead ; pCPntEx ; pCPntEx = (CSmtCutPointEx*)pCPntEx->next )
		{
			if(pCPntEx->m_fPoint[3] <= fBnd1 && k !=0)
				continue ;
			if(pCPntEx->m_fPoint[3] > fBnd2)
			{
				break ;
			}
			dCoef = (pCPntEx->m_fPoint[3] - fBnd1)/(fBnd2 - fBnd1);
			if(MdfyDir == NCDEF_FCURVE5D_MDFYDIR_MID)
			{	
				dCoef = sin(dCoef*PI2);
				dZOff = dHeight * dCoef;
			}
			else if (MdfyDir == NCDEF_FCURVE5D_MDFYDIR_DOWN)
			{
				dCoef = cos(dCoef*PI2);
				dZOff = dHeight * (dCoef - 1.0);
			}
			else /*if (MdfyDir == NCDEF_FCURVE5D_MDFYDIR_UP)*/
			{
				dCoef = cos(dCoef*PI2);
				dZOff = dHeight *(1.0 - dCoef);
			}
			if ( fabs (dZOff) > MIN_LEN )
			{
				pCPntEx->m_fTempPos[1] = TFLOAT(dZOff);
				MathCAM_FAddVec ( pCPntEx->m_fPoint, pCPntEx->m_fSurfNor, TFLOAT(dZOff), pCPntEx->m_fPoint ) ;
			}
			else
			{
				pCPntEx->m_fTempPos[1] = 0.0f;
			}
		}
	}
}

//生成梯形波动
void CSmartPathGen::CreateTrapWaveFor3DPath(CSmtCutPath* CutPath, int MdfyDir, DOUBLE MdfyHeit, int SectCnt)
{
	CSmtCutPoint* pCPnt = NULL;
	CSmtCutPointEx* pCPntEx = NULL;
	TFLOAT fBnd0 = 0.0, fBnd1 = 0.0, fBnd2 = 0.0, fHeight = TFLOAT(MdfyHeit);
	int k = 0 ;    //标记等分点序号
	BOOL bReverse = TRUE;//错峰标记
	if (MdfyDir == NCDEF_LOOP_MDFYDIR_MID)
	{
		fHeight *= 0.5;
	}

	CSmtCutPointEx* pNewCPntEx = NULL ;
	int nSect = SectCnt * 2;
	for( k = 0 ; k < nSect ; k++)
	{
		fBnd1 = (TFLOAT)k/nSect ;
		fBnd2 = (TFLOAT)(k+1)/nSect ;
		if(k %2 != 0)
		{
			fBnd0 = (TFLOAT)(k-1)/nSect;
			//取路径中需要出现在波谷的节点，将节点拷贝、移动到波谷，然后插入原来路径中
			for( pCPnt = CutPath->m_pHead ; pCPnt ; pCPnt = pCPnt->next )
			{
				if (!pCPnt->IsPointEx())
				{
					continue;
				}
				pCPntEx = (CSmtCutPointEx*)pCPnt;
				if(pCPntEx->m_fPoint[3] - fBnd2 >1e-8)
				{
					break ;
				}
				if (MdfyDir == NCDEF_LOOP_MDFYDIR_UP)
				{
					if(fBnd1 - pCPntEx->m_fPoint[3] >1e-8)
						continue ;
					if (fabs(pCPntEx->m_fPoint[3] - fBnd2) <=1e-8)
					{//路径点应该是从下往上运动波峰
						pNewCPntEx = (CSmtCutPointEx*)pCPntEx->CopyMyself();
						MathCAM_FAddVec(pNewCPntEx->m_fPoint, pNewCPntEx->m_fSurfNor, fHeight, pNewCPntEx->m_fPoint);
						CutPath->InsertBefore(pNewCPntEx, pCPntEx);
					}
					else if(bReverse)//
					{//路径点应该是从上往下运动到波谷
						pNewCPntEx = (CSmtCutPointEx*)pCPntEx->CopyMyself();
						MathCAM_FAddVec(pNewCPntEx->m_fPoint, pNewCPntEx->m_fSurfNor, fHeight, pNewCPntEx->m_fPoint);
						CutPath->InsertAfter(pNewCPntEx, pCPntEx);
						pCPnt = pNewCPntEx;
						bReverse = FALSE;
					}
					else
					{//路径点保持在梯形波的波谷
						MathCAM_FAddVec(pCPntEx->m_fPoint, pCPntEx->m_fSurfNor, fHeight, pCPntEx->m_fPoint);
					}
				}
				else if (MdfyDir == NCDEF_LOOP_MDFYDIR_MID)
				{
					if(fBnd0 - pCPntEx->m_fPoint[3] >1e-8)
						continue ;
					if (fabs(fBnd0 - pCPntEx->m_fPoint[3]) <= 1e-8)
					{//路径点应该是从下往上运动波峰
						if (k == 1)
						{
							pNewCPntEx = (CSmtCutPointEx*)pCPntEx->CopyMyself();
							MathCAM_FAddVec(pCPntEx->m_fPoint, pCPntEx->m_fSurfNor, fHeight, pCPntEx->m_fPoint);
							CutPath->InsertBefore(pNewCPntEx, pCPntEx);
						}
						else
						{
							pNewCPntEx = (CSmtCutPointEx*)pCPntEx->CopyMyself();
							MathCAM_FAddVec(pCPntEx->m_fPoint, pCPntEx->m_fSurfNor, 2.0f * fHeight, pCPntEx->m_fPoint);
							CutPath->InsertBefore(pNewCPntEx, pCPntEx);
						}
					}
					else if (fBnd1 - pCPntEx->m_fPoint[3] > 1e-8)
					{// 路径点在波峰
						MathCAM_FAddVec(pCPntEx->m_fPoint, pCPntEx->m_fSurfNor, fHeight, pCPntEx->m_fPoint);
					}
					else if ( k == nSect - 1 && fabs(pCPntEx->m_fPoint[3] - fBnd2) <=1e-8)
					{//路径点应该是从下往上运动波峰
						pNewCPntEx = (CSmtCutPointEx*)pCPntEx->CopyMyself();
						MathCAM_FAddVec(pNewCPntEx->m_fPoint, pNewCPntEx->m_fSurfNor, -fHeight, pNewCPntEx->m_fPoint);
						CutPath->InsertBefore(pNewCPntEx, pCPntEx);
					}
					else if(bReverse)//
					{//路径点应该是从上往下运动到波谷
						pNewCPntEx = (CSmtCutPointEx*)pCPntEx->CopyMyself();
						MathCAM_FAddVec(pNewCPntEx->m_fPoint, pNewCPntEx->m_fSurfNor, -fHeight, pNewCPntEx->m_fPoint);
						MathCAM_FAddVec(pCPntEx->m_fPoint, pCPntEx->m_fSurfNor, fHeight, pCPntEx->m_fPoint);
						CutPath->InsertAfter(pNewCPntEx, pCPntEx);
						pCPnt = pNewCPntEx;
						bReverse = FALSE;
					}
					else
					{//路径点保持在梯形波的波谷
						MathCAM_FAddVec(pCPntEx->m_fPoint, pCPntEx->m_fSurfNor, -fHeight, pCPntEx->m_fPoint);
					}
				}
				else /*if (MdfyDir == NCDEF_LOOP_MDFYDIR_DOWN)*/
				{
					if(fBnd1 - pCPntEx->m_fPoint[3] >1e-8)
						continue ;
					if (fabs(pCPntEx->m_fPoint[3] - fBnd2) <=1e-8)
					{//路径点应该是从下往上运动波峰
						pNewCPntEx = (CSmtCutPointEx*)pCPntEx->CopyMyself();
						MathCAM_FAddVec(pNewCPntEx->m_fPoint, pNewCPntEx->m_fSurfNor, -fHeight, pNewCPntEx->m_fPoint);
						CutPath->InsertBefore(pNewCPntEx, pCPntEx);
					}
					else if(bReverse)//
					{//路径点应该是从上往下运动到波谷
						pNewCPntEx = (CSmtCutPointEx*)pCPntEx->CopyMyself();
						MathCAM_FAddVec(pNewCPntEx->m_fPoint, pNewCPntEx->m_fSurfNor, -fHeight, pNewCPntEx->m_fPoint);
						CutPath->InsertAfter(pNewCPntEx, pCPntEx);
						pCPnt = pNewCPntEx;
						bReverse = FALSE;
					}
					else
					{//路径点保持在梯形波的波谷
						MathCAM_FAddVec(pCPntEx->m_fPoint, pCPntEx->m_fSurfNor, -fHeight, pCPntEx->m_fPoint);
					}
				}
			}
		}
		else
			bReverse = TRUE;		
	}
}

void CSmartPathGen::CreateTrochoidalWaveFor3DPath(CSmtCutPath * CutPath5x, int MdfyDir, DOUBLE MdfyHeit, int SectCnt)
{
	if (!CutPath5x || CutPath5x->GetCutMode() != MINI_MILL5AX_PATH)
	{
		return;
	}
	CSmtCutPointEx* pCPntEx = NULL, *pCPntEx0 = NULL, *pCPntEx1 = NULL, *pNewPntEx = NULL;
	TFLOAT fBnd1 = 0.0, fBnd2 = 0.0;
	DOUBLE dCoef = 0.0, dHeight[3] = {MdfyHeit/3, MdfyHeit* 2/3, MdfyHeit/6}, dZOff = 0.0;
	int k = 0 ;    //标记等分点序号

	CSmtCutPath tmpPath;
	pCPntEx0 = (CSmtCutPointEx*)CutPath5x->m_pHead;
	for( k = 0 ; k < SectCnt ; k++)
	{
		fBnd1 = (TFLOAT)k/SectCnt ;
		fBnd2 = (TFLOAT)(k+1)/SectCnt ;
		// 确定起点
		for( pCPntEx = pCPntEx0 ; pCPntEx ; pCPntEx = (CSmtCutPointEx*)pCPntEx->next )
		{
			if(pCPntEx->m_fPoint[3] < fBnd1)
				continue ;
			pCPntEx0 = pCPntEx;
			break;
		}
		// 确定末点
		for( pCPntEx = pCPntEx0 ; pCPntEx ; pCPntEx = (CSmtCutPointEx*)pCPntEx->next )
		{
			if(pCPntEx->m_fPoint[3] > fBnd2)
			{
				pCPntEx1 = (CSmtCutPointEx*)pCPntEx->prev;
				break;
			}
			else if (pCPntEx == (CSmtCutPointEx*)CutPath5x->m_pTail)
			{
				pCPntEx1 = (CSmtCutPointEx*)pCPntEx;
			}
		}
		for (pCPntEx = pCPntEx0; pCPntEx; pCPntEx = (CSmtCutPointEx*)pCPntEx->next)
		{
			dCoef = (pCPntEx->m_fPoint[3] - fBnd1)/(fBnd2 - fBnd1);
			if(MdfyDir == NCDEF_FCURVE5D_MDFYDIR_MID)
			{
				dCoef = asin(dCoef);
				dCoef = cos(dCoef);
				dZOff = -dHeight[1] * (dCoef);
				dZOff += dHeight[2];
			}
			else if (MdfyDir == NCDEF_FCURVE5D_MDFYDIR_DOWN)
			{
				dCoef = asin(dCoef);
				dCoef = cos(dCoef);
				dZOff = dHeight[1] * (dCoef - 1.0);
			}
			else /*if (MdfyDir == NCDEF_FCURVE5D_MDFYDIR_UP)*/
			{
				dCoef = asin(dCoef);
				dCoef = cos(dCoef);
				dZOff = dHeight[1] * (1.0 - dCoef);
			}
			pNewPntEx = new CSmtCutPointEx(*pCPntEx);
			tmpPath.AddTail(pNewPntEx);
			if ( fabs (dZOff) > MIN_LEN )
			{
				pNewPntEx->m_fTempPos[1] = TFLOAT(dZOff);
				MathCAM_FAddVec ( pNewPntEx->m_fPoint, pNewPntEx->m_fSurfNor, TFLOAT(dZOff), pNewPntEx->m_fPoint ) ;
			}
			else
			{
				pNewPntEx->m_fTempPos[1] = 0.0f;
			}
			if (pCPntEx == pCPntEx1)
			{
				break;
			}
		}

		for (pCPntEx = pCPntEx1; pCPntEx; pCPntEx = (CSmtCutPointEx*)pCPntEx->prev)
		{
			dCoef = (pCPntEx->m_fPoint[3] - fBnd1)/(fBnd2 - fBnd1);
			if(MdfyDir == NCDEF_FCURVE5D_MDFYDIR_MID)
			{
				if (dCoef >= 0.5)
				{
					dCoef = (dCoef - 0.5)*2;
					dCoef = asin(dCoef);
					dCoef = cos(dCoef );
				}
				else
				{
					dCoef = (dCoef - 0.5)*2;
					dCoef = asin(dCoef);
					dCoef = cos(dCoef);
				}
				dZOff = dHeight[0] *(dCoef);
				dZOff += dHeight[2];
			}
			else if (MdfyDir == NCDEF_FCURVE5D_MDFYDIR_DOWN)
			{
				if (dCoef >= 0.5)
				{
					dCoef = (dCoef - 0.5)*2;
					dCoef = asin(dCoef);
					dCoef = cos(dCoef );
				}
				else
				{
					dCoef = (dCoef - 0.5)*2;
					dCoef = asin(dCoef);
					dCoef = cos(dCoef);
				}
				dZOff = -dHeight[0] *(2.0 + dCoef);
			}
			else /*if (MdfyDir == NCDEF_FCURVE5D_MDFYDIR_UP)*/
			{
				if (dCoef >= 0.5)
				{
					dCoef = (dCoef - 0.5)*2;
					dCoef = asin(dCoef);
					dCoef = cos(dCoef );
				}
				else
				{
					dCoef = (dCoef-0.5)*2;
					dCoef = asin(dCoef);
					dCoef = cos(dCoef);
				}
				dZOff = dHeight[0] *(2.0 + dCoef);
			}
			pNewPntEx = new CSmtCutPointEx(*pCPntEx);
			tmpPath.AddTail(pNewPntEx);
			if ( fabs (dZOff) > MIN_LEN )
			{
				pNewPntEx->m_fTempPos[1] = TFLOAT(dZOff);
				MathCAM_FAddVec ( pNewPntEx->m_fPoint, pNewPntEx->m_fSurfNor, TFLOAT(dZOff), pNewPntEx->m_fPoint ) ;
			}
			else
			{
				pNewPntEx->m_fTempPos[1] = 0.0f;
			}
			if (pCPntEx == pCPntEx0)
			{
				break;
			}
		}

		for (pCPntEx = pCPntEx0; pCPntEx; pCPntEx = (CSmtCutPointEx*)pCPntEx->next)
		{
			dCoef = (pCPntEx->m_fPoint[3] - fBnd1)/(fBnd2 - fBnd1);
			if(MdfyDir == NCDEF_FCURVE5D_MDFYDIR_MID)
			{
				dCoef = asin(1.0-dCoef);
				dCoef = cos(dCoef);
				dZOff = -dHeight[1] * (dCoef);
				dZOff += dHeight[2];
			}
			else if (MdfyDir == NCDEF_FCURVE5D_MDFYDIR_DOWN)
			{
				dCoef = asin(1.0-dCoef);
				dCoef = cos(dCoef);
				dZOff = dHeight[1] * (dCoef - 1.0);
			}
			else /*if (MdfyDir == NCDEF_FCURVE5D_MDFYDIR_UP)*/
			{
				dCoef = asin(1.0-dCoef);
				dCoef = cos(dCoef);
				dZOff = dHeight[1] * (1.0 - dCoef);
			}
			pNewPntEx = new CSmtCutPointEx(*pCPntEx);
			tmpPath.AddTail(pNewPntEx);
			if ( fabs (dZOff) > MIN_LEN )
			{
				pNewPntEx->m_fTempPos[1] = TFLOAT(dZOff);
				MathCAM_FAddVec ( pNewPntEx->m_fPoint, pNewPntEx->m_fSurfNor, TFLOAT(dZOff), pNewPntEx->m_fPoint ) ;
			}
			else
			{
				pNewPntEx->m_fTempPos[1] = 0.0f;
			}
			if (pCPntEx == pCPntEx1)
			{
				break;
			}
		}
		pCPntEx0 = pCPntEx1;
	}

	if (MdfyDir == NCDEF_FCURVE5D_MDFYDIR_MID)
	{
		pNewPntEx = new CSmtCutPointEx(*((CSmtCutPointEx*)CutPath5x->m_pHead));
		tmpPath.AddHead(pNewPntEx);
		pNewPntEx = new CSmtCutPointEx(*((CSmtCutPointEx*)CutPath5x->m_pTail));
		tmpPath.AddTail(pNewPntEx);
	}

	CutPath5x->ClearAllPoint();
	CutPath5x->m_pHead = tmpPath.m_pHead;
	CutPath5x->m_pTail = tmpPath.m_pTail;
	CutPath5x->m_nNumPnt = tmpPath.m_nNumPnt;
	tmpPath.m_pHead = tmpPath.m_pTail = NULL;
	tmpPath.m_nNumPnt = 0;
	CutPath5x->DelPointOverlap();
}

BOOL CSmartPathGen::MatchSpeedForMdfyGrnd(CSmtCPathLib &AllPath, CPathCombine &PComb)
{
	CJDJob* pJob = SurfNC_GetCurJobSetup();
	if (!pJob)
	{
		return FALSE;
	}
	CSmtCutPath * pPath = NULL ;
	POSITION pos = AllPath.m_cAllPath.GetHeadPosition () , atpos = NULL ;
	int i = 0 ;
	CPathPLine3D *pPLine = NULL ;
	TFLOAT fPrevFeedRate = 1., fMaxFeedRate = TFLOAT(max(pJob->m_dRateDm[1], 12.0));;
	double dSurfPosDist = 0., dTipPosDist = 0., dWaveDist = 0.;
	CSmtCutPointEx *pStart = NULL, *pEnd = NULL ;
	while( pos )
	{
		atpos = pos ;
		pPath = AllPath.m_cAllPath.GetNext ( pos ) ;
		if (!pPath)
		{
			continue;
		}
		if( pPath->GetCutMode() == MINI_MILL5AX_PATH )
		{
			if( pPath->m_bFeedType != JDNC_FEEDTYPE_ROUGH )
			{
				pPath->AddToPathCombine ( PComb ) ;
			}
			else
			{
				pStart = ( CSmtCutPointEx *)pPath->m_pHead ;
				while( pStart )
				{
					pEnd = (CSmtCutPointEx *)pStart->next ;					
					if( !pEnd ) break ;
					pPLine = new CPathPLine3D() ;
					pPLine->m_pTAPos = new TPNT3D[2] ;
					pPLine->m_nCount = 1 ;					
					for( i = 0 ; i < 3 ; i++ )
					{
						pPLine->m_pTAPos[0][i] = pStart->m_fPoint[i] ;
						pPLine->m_pTAPos[1][i] = pEnd->m_fPoint[i] ;					
					}
					dSurfPosDist = nc_Distance( pStart->m_fSurfPos, pEnd->m_fSurfPos, 2 ) ;
					dTipPosDist = fabs(pEnd->m_fTempPos[0] - pStart->m_fTempPos[0]);
					dWaveDist = nc_Distance( pStart->m_fPoint, pEnd->m_fPoint, 3 ) ;
					pPLine->m_fFeedRate = pPath->m_fFeedRate ;
					pPLine->m_bFeedType = pPath->m_bFeedType ;
					pPLine->m_bMoveFlag = pPath->m_bMoveFlag ;
#ifdef  _DEBUG
					if (pEnd->m_fTempPos[2] > 2)
					{
						//ASSERT(0);
					}
#endif

					if( dSurfPosDist < 0.0001)
					{
						pPLine->m_fFeedRate = fPrevFeedRate ;
					}
					else
					{
						pPLine->m_fFeedRate *= pEnd->m_fTempPos[2]/pPath->m_fFeedRate;
						pPLine->m_fFeedRate *= TFLOAT(dWaveDist/dTipPosDist);
						if (pPLine->m_fFeedRate > fMaxFeedRate)
						{
							pPLine->m_fFeedRate = fMaxFeedRate;
						}
					}					

					PComb.AddEntity ( pPLine ) ;
					fPrevFeedRate = pPLine->m_fFeedRate ;
					pStart = pEnd ;
				}
			}
		}
		else
		{
			pPath->AddToPathCombine(PComb);
		}
		delete pPath ;
		AllPath.m_cAllPath.RemoveAt ( atpos ) ;
	}

	// 设置进退刀和连刀速度
	MatchLeadConnectByAdjacentPath(PComb);

	return TRUE;
}

void CSmartPathGen::MatchSpeedByToolPosFor3DPath(CPathGroup &NewPath, CSmartGraphic& Graph)
{
	if (glbf_GetProductID() == PRODUCT_ID_SURFMILL_PRO)
	{// 个人版软件不支持磨削调整功能，2014.10.15 liuxin
		return;
	}
	if (m_cMethodDef.m_nMethodType == surfncMethodLoop)
	{
		JDNC_LOOP cParam = m_cMethodDef.m_cLoopCut;
		if ((cParam.m_bLoopFlag & NCDEF_FLOOP_TOOLPOSFEED) ||
			((cParam.m_bLoopFlag & NCDEF_FLOOP_GRNDMDFY) && 
			cParam.m_nMdfyPathType == NCDEF_FLOOP_MDFY_ORIGPATH))
		{// 刀触点速度模式
			MatchSpeedByToolPosFor3D(NewPath, Graph, cParam.m_nRCompSide);
		}
	}
	else if (m_cMethodDef.m_nMethodType == surfncMethodCurve)
	{
		JDNC_CURVE cParam = m_cMethodDef.m_cCurveCut;
		if ((cParam.m_bCurveFlag & NCDEF_FCURVE_TOOLPOSFEED) ||
			(cParam.m_bCurveFlag & NCDEF_FCURVE_GRNDMDFY && 
			cParam.m_nMdfyPathType == NCDEF_FCURVE_MDFY_ORIGPATH))
		{// 刀触点速度模式
			MatchSpeedByToolPosFor3D(NewPath, Graph, cParam.m_nRCompSide);
		}
	}
}

BOOL CSmartPathGen::MatchSpeedByToolPosFor3D(CPathGroup &NewPath, CSmartGraphic& Graph, int RCompSide/* = 2*/)
{
	CSmtCPathLib AllPath ;	
	//将PathGroup转为CSmtCutPath
	MathCAM_AddPathGroup( AllPath, NewPath, m_cSetupDef.m_cTolDef , TRUE ) ;
	if( AllPath.m_cAllPath.GetCount() < 1 )
		return FALSE ;
	int nPnt = AllPath.GetNumPoint() ;
	if( nPnt < 1 ) 
		return FALSE ;
	// 获取曲线
	CPtrList listCur;
	if (!Graph.GetAllEntity( m_cSetupDef, listCur, SMARTGRAPH_TYPE_CURVE2D ) || listCur.IsEmpty())

	{
		return FALSE;
	}
	BOOL bReverse = FALSE;
	if ((GetMillDir() == NCDEF_MILLDIR_CLIMB && RCompSide == 1) ||
		(GetMillDir() == NCDEF_MILLDIR_CONVENTIONAL && RCompSide != 1))
	{// 路径方向与曲线方向相反时，需要对曲线反向，2014.9.30 liuxin
		bReverse = TRUE;
		MathCam_ReverseAllRedepthCur(listCur);
	}
	// 获取局部坐标系
	RFRAME LocFrame ;
	if( !Graph.GetTPlaneFrame( LocFrame ) )
	{
		mathInitRFrame( &LocFrame ) ;	
	}
	if (m_c5DCtrlDef.m_cToolAxisApp.m_nFrameFlag != NCDEF_FRAME_DEFAULT)
	{
		mathMultRFrame( &LocFrame, &m_c5DCtrlDef.m_cToolAxisApp.m_dIndexFrame, &LocFrame ) ;
	}

	CSmtCutPath* pCutPath = NULL, *pPrevPath = NULL ;
	POSITION pos = AllPath.m_cAllPath.GetHeadPosition(), atpos = NULL ;
	while(pos)
	{
		atpos = pos ;
		pPrevPath = pCutPath;
		pCutPath = AllPath.m_cAllPath.GetNext(pos) ;
		if( !pCutPath || pCutPath->m_bFeedType != JDNC_FEEDTYPE_ROUGH) 
		{
			continue ;
		}
		// 转成五轴路径
		CSmtCutPath* pCPath5x = TransCPathToCPath5x(pCutPath, listCur, LocFrame);
		if (pCPath5x)
		{
			delete pCutPath;
			pCutPath = pCPath5x;
			AllPath.m_cAllPath.SetAt(atpos, pCPath5x);
		}
	}
	if (bReverse)
	{// 恢复曲线方向
		MathCam_ReverseAllRedepthCur(listCur);
	}
	MathCam_DeleteAllRedepthCur( listCur ) ;

	CPathCombine * pPComb = new CPathCombine( NC_WPROCESS_ROUGH ) ;
	if (!MatchSpeedByToolPosFor3D(AllPath, *pPComb))
	{
		pPComb->ClearAll();
		delete pPComb;
	}
	else
	{
		AllPath.AddToPathCombine ( *pPComb ) ;
		NewPath.ClearAll(); // 删除原始路径
		NewPath.AddData ( 0.0, pPComb ) ;
	}

	return TRUE;
}

BOOL CSmartPathGen::MatchSpeedByToolPosFor3D( CSmtCPathLib &AllPath, CPathCombine &PComb)
{
	CJDJob* pJob = SurfNC_GetCurJobSetup();
	if (!pJob)
	{
		return FALSE;
	}
	BOOL bContourPlunge = FALSE;
	if (m_cFeedDef.m_cPlungeDef.m_nPlungeType == NCDEF_PLUNGE_CONTOUR && m_cFeedDef.m_cPlungeDef.m_dAngle < 90 - MIN_DIS)
	{
		bContourPlunge = TRUE;
	}

	int i = 0;
	CPathPLine3D* pPLine = NULL;
	CPathLine3D line;
	TFLOAT fPrevFeedRate = 1., fMaxFeedRate = TFLOAT(pJob->m_dRateDm[1]);;
	double dTipPosDist = 0., dSurfPosDist = 0.;
	CSmtCutPath * pPath = NULL ;
	POSITION pos = AllPath.m_cAllPath.GetHeadPosition () , atpos = NULL ;
	CSmtCutPointEx *pStartEx = NULL, *pEndEx = NULL ;

	pos = AllPath.m_cAllPath.GetHeadPosition ();
	while( pos )
	{
		atpos = pos ;
		pPath = AllPath.m_cAllPath.GetNext ( pos ) ;
		if (!pPath)
		{
			continue;
		}
		if( pPath->GetCutMode() == MINI_MILL5AX_PATH )
		{
			if ((pPath->m_bFeedType == JDNC_FEEDTYPE_PLUNGE && bContourPlunge) ||
				pPath->m_bFeedType == JDNC_FEEDTYPE_ROUGH)
			{
				pStartEx = ( CSmtCutPointEx *)pPath->m_pHead ;
				while( pStartEx )
				{
					pEndEx = (CSmtCutPointEx *)pStartEx->next ;
					if( !pEndEx ) break ;
					pPLine = new CPathPLine3D ;
					pPLine->m_nCount = 1;
					pPLine->m_pTAPos = new TPNT3D[2];
					pPLine->m_fFeedRate = pPath->m_fFeedRate ;
					pPLine->m_bFeedType = pPath->m_bFeedType ;
					pPLine->m_bMoveFlag = pPath->m_bMoveFlag ;
					for( i = 0 ; i < 3 ; i++ )
					{
						pPLine->m_pTAPos[0][i] = pStartEx->m_fPoint[i] ;
						pPLine->m_pTAPos[1][i] = pEndEx->m_fPoint[i] ;
					}
					dSurfPosDist = nc_Distance( pStartEx->m_fSurfPos, pEndEx->m_fSurfPos, 3 ) ;
					dTipPosDist = nc_Distance( pStartEx->m_fPoint, pEndEx->m_fPoint, 3 ) ;
					if( dSurfPosDist < 0.0001)
					{
						pPLine->m_fFeedRate = fPrevFeedRate;
					}
					else
					{
						pPLine->m_fFeedRate *= TFLOAT( dTipPosDist / dSurfPosDist );
						if (pPLine->m_fFeedRate > fMaxFeedRate)
						{
							pPLine->m_fFeedRate = fMaxFeedRate;
						}
					}

					PComb.AddEntity ( pPLine ) ;
					fPrevFeedRate = pPLine->m_fFeedRate;

					pStartEx = pEndEx ;
				}
			}
			else
			{
				pPath->AddToPathCombine ( PComb ) ;
			}
		}
		else
		{
			pPath->AddToPathCombine ( PComb ) ;
		}
		delete pPath ;
		AllPath.m_cAllPath.RemoveAt ( atpos ) ;
	}

	// 设置进退刀和连刀速度
	MatchLeadConnectByAdjacentPath(PComb);

	return TRUE ;
}

BOOL CSmartPathGen::MatchLeadConnectByAdjacentPath(CPathCombine & PComb)
{
	PNT3D ptEnd[2];
	//double dLeadRate = m_cSpeedDef.m_dLeadRate;
	//double dConnectRate = m_cSpeedDef.m_dConnectRate;
	//double dFeedRate = m_cSpeedDef.m_dFeedRate;
	CPathEntity* pEnt = NULL, *pNextEnt = NULL, *pPrevEnt = NULL;
	for (pEnt = PComb.m_pHead; pEnt; pEnt = pEnt->next)
	{
		if (!pEnt || pEnt->m_bFeedType != JDNC_FEEDTYPE_ROUGH)
		{
			continue;
		}
		// 向前找进刀和下刀
		if (pEnt->prev && pEnt->prev->m_bFeedType != JDNC_FEEDTYPE_ROUGH && pEnt->prev->m_bFeedType != JDNC_FEEDTYPE_PLUNGE)
		{
			//dFeedRate = GetPathEntityFeedRate(pEnt, m_cSpeedDef);
			pEnt->GetEndPoint(0, ptEnd[0]);
			pPrevEnt = pEnt->prev;
			pPrevEnt->GetEndPoint(1, ptEnd[1]);
			while(pPrevEnt && mathDist(ptEnd[0], ptEnd[1]) < MIN_DIS)
			{			
				if (pPrevEnt->m_bFeedType == JDNC_FEEDTYPE_LEAD)
				{
					//pPrevEnt->m_fFeedRate *= TFLOAT(pEnt->m_fFeedScale * dFeedRate/dLeadRate) ;
					pPrevEnt->m_fFeedRate = TFLOAT(pEnt->m_fFeedScale * pEnt->m_fFeedRate) ;
				}
				else
				{
					break;
				}
				pPrevEnt->GetEndPoint(0, ptEnd[0]);
				pPrevEnt = pPrevEnt->prev;
				if (pPrevEnt)
				{
					pPrevEnt->GetEndPoint(1, ptEnd[1]);
				}				
			}
		}
		if (pEnt->next && pEnt->next->m_bFeedType != JDNC_FEEDTYPE_ROUGH)
		{
			// 向后找退刀
			//dFeedRate = GetPathEntityFeedRate(pEnt, m_cSpeedDef);
			pEnt->GetEndPoint(1, ptEnd[1]);
			pNextEnt = pEnt->next;
			pNextEnt->GetEndPoint(0, ptEnd[0]);
			while (pNextEnt && mathDist(ptEnd[0], ptEnd[1]) < MIN_DIS)
			{
				if (pNextEnt->m_bFeedType == JDNC_FEEDTYPE_LEAD)
				{
					//pNextEnt->m_fFeedRate *= TFLOAT(pEnt->m_fFeedScale * dFeedRate /dLeadRate);
					pNextEnt->m_fFeedRate = TFLOAT(pEnt->m_fFeedScale * pEnt->m_fFeedRate) ;
				}
				else if (pNextEnt->m_bFeedType == JDNC_FEEDTYPE_CONNECT)
				{
					//pNextEnt->m_fFeedRate *= TFLOAT(pEnt->m_fFeedScale * dFeedRate/dConnectRate);
					pNextEnt->m_fFeedRate = TFLOAT(pEnt->m_fFeedScale * pEnt->m_fFeedRate) ;
				}
				else
				{
					break;
				}
				pNextEnt->GetEndPoint(1, ptEnd[1]);
				pNextEnt = pNextEnt->next;
				if (pNextEnt)
				{
					pNextEnt->GetEndPoint(0, ptEnd[0]);
				}				
			}
		}
	}

	return TRUE;
}

//生成三角形波动
void CSmartPathGen::CreateTriWaveFor3DPath(CSmtCutPath * CutPath5x, int MdfyDir, DOUBLE MdfyHeit,int SectCnt)
{
	if (!CutPath5x || CutPath5x->GetCutMode() != MINI_MILL5AX_PATH)
	{
		return;
	}
	int nSect = SectCnt;
	CSmtCutPoint* pCPnt = NULL;
	CSmtCutPointEx* pCPntEx = NULL;
	TFLOAT fBnd1 = 0.0, fBnd2 = 0.0, fMid[3] = {0.0};
	DOUBLE dCoef = 0.0, dH = 0.0 ;
	int k = 0 ;    //标记等分点序号
	BOOL bReverse = TRUE;//错峰标记
	if(!bReverse)
	{
		CutPath5x->TransPath(0.0f, 0.0f, TFLOAT(-MdfyHeit));
	}
	for( k = 0 ; k < nSect ; k++)
	{
		fBnd1 = (TFLOAT)k/nSect ;
		fBnd2 = (TFLOAT)(k+1)/nSect ;
		fMid[0] = fBnd1 * 0.75f + fBnd2 * 0.25f;
		fMid[1] = (fBnd1 + fBnd2) * 0.5f;
		fMid[2] = fBnd1 * 0.25f + fBnd2 * 0.75f;
		for( pCPnt = CutPath5x->m_pHead ; pCPnt ; pCPnt = pCPnt->next )
		{
			if (!pCPnt->IsPointEx())
			{
				continue;
			}
			pCPntEx = (CSmtCutPointEx*)pCPnt;
			if(pCPntEx->m_fPoint[3] < fBnd1)
				continue ;
			if(pCPntEx->m_fPoint[3] > fBnd2)
			{
				break ;
			}
			if (MdfyDir == NCDEF_LOOP_MDFYDIR_UP)
			{
				if(pCPntEx->m_fPoint[3] < fMid[1] )
				{
					ASSERT(fMid[1] - fBnd1 > 1.0e-8);
					dCoef = (pCPntEx->m_fPoint[3] - fBnd1)/(fMid[1] - fBnd1);
					if(!bReverse)
						dH = -dCoef * MdfyHeit;
					else
						dH = dCoef * MdfyHeit;
				}
				else if(pCPntEx->m_fPoint[3] > fMid[1])
				{
					ASSERT(fBnd2 - fMid[1] > 1.0e-8);
					dCoef = (fBnd2 - pCPntEx->m_fPoint[3])/(fBnd2 - fMid[1]);
					if(!bReverse)
						dH = -dCoef * MdfyHeit;
					else
						dH = (dCoef * MdfyHeit) ;
				}
				else
				{
					if(!bReverse)
						dH = -MdfyHeit;
					else
						dH = MdfyHeit;
				}
				MathCAM_FAddVec(pCPntEx->m_fPoint, pCPntEx->m_fSurfNor, TFLOAT(dH), pCPntEx->m_fPoint);
			}
			else if (MdfyDir == NCDEF_LOOP_MDFYDIR_MID)
			{
				if(pCPntEx->m_fPoint[3] < fMid[0] )
				{
					ASSERT(fMid[0] - fBnd1 > 1.0e-8);
					dCoef = (pCPntEx->m_fPoint[3] - fBnd1)/(fMid[0] - fBnd1);
					if(!bReverse)
						dH = dCoef * MdfyHeit * 0.5;
					else
						dH = -dCoef * MdfyHeit * 0.5;
				}
				else if (pCPntEx->m_fPoint[3] >= fMid[0] && pCPntEx->m_fPoint[3] < fMid[1])
				{
					dCoef = (fMid[1] - pCPntEx->m_fPoint[3])/(fMid[1] - fMid[0]);
					if(!bReverse)
						dH = dCoef * MdfyHeit * 0.5;
					else
						dH = -(dCoef * MdfyHeit) * 0.5;
				}
				else if(pCPntEx->m_fPoint[3] >= fMid[1] && pCPntEx->m_fPoint[3] < fMid[2])
				{
					dCoef = (pCPntEx->m_fPoint[3] - fMid[1])/(fMid[2] - fMid[1]);
					if(!bReverse)
						dH = -dCoef * MdfyHeit * 0.5;
					else
						dH = (dCoef * MdfyHeit) * 0.5;
				}
				else if (pCPntEx->m_fPoint[3] >= fMid[2] && pCPntEx->m_fPoint[3] < fBnd2)
				{
					dCoef = (fBnd2 - pCPntEx->m_fPoint[3])/(fBnd2 - fMid[2]);
					if(!bReverse)
						dH = -dCoef * MdfyHeit * 0.5;
					else
						dH = (dCoef * MdfyHeit) * 0.5;
				}
				else
				{
					dH = 0.0;
				}
				MathCAM_FAddVec(pCPntEx->m_fPoint, pCPntEx->m_fSurfNor, TFLOAT(dH), pCPntEx->m_fPoint);
			}
			else /*if (MdfyDir == NCDEF_LOOP_MDFYDIR_DOWN)*/
			{
				if(pCPntEx->m_fPoint[3] < fMid[1] )
				{
					ASSERT(fMid[1] - fBnd1 > 1.0e-8);
					dCoef = (pCPntEx->m_fPoint[3] - fBnd1)/(fMid[1] - fBnd1);
					if(!bReverse)
						dH = dCoef * MdfyHeit;
					else
						dH = -dCoef * MdfyHeit;
				}
				else if(pCPntEx->m_fPoint[3] > fMid[1])
				{
					ASSERT(fBnd2 - fMid[1] > 1.0e-8);
					dCoef = (fBnd2 - pCPntEx->m_fPoint[3])/(fBnd2 - fMid[1]);
					if(!bReverse)
						dH = dCoef * MdfyHeit;
					else
						dH = -(dCoef * MdfyHeit) ;
				}
				else
				{
					if(!bReverse)
						dH = MdfyHeit;
					else
						dH = -MdfyHeit;
				}
				MathCAM_FAddVec(pCPntEx->m_fPoint, pCPntEx->m_fSurfNor, TFLOAT(dH), pCPntEx->m_fPoint);
			}
		}
	}
}

//插入新的等分点
void CSmartPathGen::InsertNewCutPoint(CSmtCutPath * CutPath ,int nPoint, BOOL StartAtExtreme/* = TRUE*/)
{
	TFLOAT t = 0.0,  fBnd1 = 0.0;
	if(CutPath->GetCutMode() == MINI_MILL5AX_PATH)//xiejunxian 2013/5/13
	{
		DOUBLE dDist1 = 0.0, dDist2 = 0.0;
		CSmtCutPointEx tmpCPntEx ;
		CSmtCutPointEx* pCPntEx = NULL, *pPrevCPntEx = NULL ;		
		for( int i = 1 ; i < nPoint; i++ )
		{
			if (StartAtExtreme == FALSE)
			{
				if (i % 2 == 1)
				{
					fBnd1 = (TFLOAT) i/nPoint ;
				}
				else
				{
					continue;
				}
			}
			else
			{
				fBnd1 = (TFLOAT) i/nPoint ;
			}
			for(pCPntEx = (CSmtCutPointEx*)CutPath->m_pHead; pCPntEx; pCPntEx = (CSmtCutPointEx*)pCPntEx->next)
			{
				if( fBnd1 < pCPntEx->m_fPoint[3] && pCPntEx->prev)
				{
					pPrevCPntEx = (CSmtCutPointEx*)pCPntEx->prev;				
					t = fBnd1 - pPrevCPntEx->m_fPoint[3] ;
					t /= TFLOAT( pCPntEx->m_fPoint[3] - pPrevCPntEx->m_fPoint[3] ) ;
					pPrevCPntEx->Calc5AxPoint(*pCPntEx, t, tmpCPntEx) ;
					dDist1 = nc_Distance(tmpCPntEx.m_fPoint, pCPntEx->m_fPoint, 3);
					dDist2 = nc_Distance(tmpCPntEx.m_fPoint, pPrevCPntEx->m_fPoint, 3);
					if (dDist1 > MIN_DIS &&	dDist2 > MIN_DIS)
					{
						CutPath->InsertBefore(tmpCPntEx.CopyMyself(), pCPntEx) ;
					}
					else if (dDist1 < dDist2)
					{
						*pCPntEx = tmpCPntEx;
					}
					else
					{
						*pPrevCPntEx = tmpCPntEx;
					}
					break ;
				}
			}
		}
	}
	else
	{
		CSmtCutPoint tmpCPnt;
		CSmtCutPoint* pCPnt = NULL ;		
		for( int i = 1 ; i < nPoint; i++ )
		{
			fBnd1 = (TFLOAT) i/nPoint ;
			for( pCPnt = CutPath->m_pHead ; pCPnt ; pCPnt = pCPnt->next )
			{
				if( fBnd1 <= pCPnt->m_fPoint[3])
				{
					ASSERT(pCPnt->prev) ;						
					t = fBnd1 - pCPnt->prev->m_fPoint[3] ;
					t /= TFLOAT( pCPnt->m_fPoint[3] - pCPnt->prev->m_fPoint[3] ) ;
					pCPnt->prev->CalcMidPoint(pCPnt, t, &tmpCPnt) ;
					if (nc_Distance(tmpCPnt.m_fPoint, pCPnt->m_fPoint, 3) > MIN_DIS &&
						nc_Distance(tmpCPnt.m_fPoint, pCPnt->prev->m_fPoint, 3) > MIN_DIS)
					{
						CutPath->InsertBefore(tmpCPnt.CopyMyself(), pCPnt) ;
					}					
					break ;
				}
			}
		}
	}
}
//生成三角形波动
void CSmartPathGen::CreateTriWave(CSmtCutPath * CutPath,DOUBLE depthInc,int nSect)
{
	if( nSect > 10000 ) nSect = 10000 ;
	CSmtCutPoint* pCPnt =NULL;
	TFLOAT fBnd1 = 0.0, fBnd2 = 0.0, fMid = 0.0;
	DOUBLE dCoef = 0.0 ;
	int k = 0 ;    //标记等分点序号
	BOOL bReverse= TRUE;//错峰标记
	if(!bReverse)
	{
		CutPath->TransPath(0.0f, 0.0f, TFLOAT(-depthInc));
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
					pCPnt->m_fPoint[2] += TFLOAT(dCoef * depthInc) ;
				else
					pCPnt->m_fPoint[2] -= TFLOAT(dCoef * depthInc) ;
			}
			else if(pCPnt->m_fPoint[3] > fMid)
			{
				ASSERT(fBnd2 - fMid > 1.0e-8);
				dCoef = (fBnd2 - pCPnt->m_fPoint[3])/(fBnd2 - fMid);
				if(!bReverse)
					pCPnt->m_fPoint[2] += TFLOAT(dCoef * depthInc) ;
				else
					pCPnt->m_fPoint[2] -= TFLOAT(dCoef * depthInc) ;
			}
			else
			{
				if(!bReverse)
					pCPnt->m_fPoint[2] += TFLOAT(depthInc) ;
				else
					pCPnt->m_fPoint[2] -= TFLOAT(depthInc) ;
			}
		}
	}
}
//生成类正弦波动
void CSmartPathGen::CreateSinWave(CSmtCutPath * CutPath,DOUBLE depthInc,int nSect)
{
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
				pCPnt->m_fPoint[2] += TFLOAT(h);				
			}
			else//类余弦波形
			{
				dCoef = cos(dCoef*PI2);
				DOUBLE h = depthInc*(1-dCoef)/2;
				pCPnt->m_fPoint[2] -= TFLOAT(h) ;								
			}
		}
	}			 
}
//生成梯形波动
void CSmartPathGen::CreateTrapWave(CSmtCutPath * CutPath,DOUBLE depthInc,int nSect)
{
	CSmtCutPoint* pCPnt =NULL;
	TFLOAT fBnd1 = 0.0, fBnd2 = 0.0;	
	int k = 0 ;    //标记等分点序号
	BOOL bReverse = TRUE;//错峰标记

	CSmtCutPoint* pNewCPnt = NULL ;
	nSect = nSect*2;
	for( k = 0 ; k < nSect ; k++)
	{
		fBnd1 = (TFLOAT)k/nSect ;
		fBnd2 = (TFLOAT)(k+1)/nSect ;
		if(k %2 != 0)
		{
			//取路径中需要出现在波谷的节点，将节点拷贝、移动到波谷，然后插入原来路径中
			for( pCPnt = CutPath->m_pHead ; pCPnt ; pCPnt = pCPnt->next )
			{
				if(fBnd1 - pCPnt->m_fPoint[3] >1e-8)
					continue ;
				if(pCPnt->m_fPoint[3] - fBnd2 >1e-8)
				{		

					break ;
				}
				else if (fabs(pCPnt->m_fPoint[3] - fBnd2) <=1e-8)
				{//路径点应该是从下往上运动波峰
					pNewCPnt = pCPnt->CopyMyself();
					pNewCPnt->m_fPoint[2] -=TFLOAT(depthInc);
					CutPath->InsertBefore(pNewCPnt,pCPnt);

				}
				else if(bReverse)//
				{//路径点应该是从上往下运动到波谷
					pNewCPnt = pCPnt->CopyMyself();
					pNewCPnt->m_fPoint[2] -=TFLOAT(depthInc);
					CutPath->InsertAfter(pNewCPnt,pCPnt);
					pCPnt =pNewCPnt;
					bReverse = FALSE;
				}
				else
				{//路径点保持在梯形波的波谷
					pCPnt->m_fPoint[2] -=TFLOAT(depthInc);
				}
			}

		}
		else
			bReverse =TRUE;		
	}
}
//磨削调整路径重新匹配速度，保证水平方向的速度和设置的加工速度一致  add by zy 2013.5.7
BOOL CSmartPathGen::MatchSpeedInModifyGrind( CSmtCPathLib &AllPath, CPathCombine &PComb )
{
	CSmtCutPath * pPath = NULL ;
	POSITION pos = AllPath.m_cAllPath.GetHeadPosition () , atpos = NULL ;
	int i = 0 ;
	CPathPLine3D *pPLine = NULL ;
	TFLOAT fPrevScale = 1. ;
	double dTipPosDist = 0., dPlanePosDist = 0.;
	CSmtCutPointEx *pStart = NULL, *pEnd = NULL ;
	while( pos )
	{
		atpos = pos ;
		pPath = AllPath.m_cAllPath.GetNext ( pos ) ;
		if( pPath->GetCutMode() == MINI_MILL5AX_PATH )
		{
			if( pPath->m_bFeedType != JDNC_FEEDTYPE_ROUGH )
			{
				pPath->AddToPathCombine ( PComb ) ;
			}
			else
			{
				pStart = ( CSmtCutPointEx *)pPath->m_pHead ;
				while( pStart )
				{
					pEnd = (CSmtCutPointEx *)pStart->next ;					
					if( !pEnd ) break ;
					pPLine = new CPathPLine3D() ;
					pPLine->m_pTAPos = new TPNT3D[2] ;
					pPLine->m_nCount = 1 ;					
					for( i = 0 ; i < 3 ; i++ )
					{
						pPLine->m_pTAPos[0][i] = pStart->m_fPoint[i] ;
						pPLine->m_pTAPos[1][i] = pEnd->m_fPoint[i] ;					
					}
					dPlanePosDist = nc_Distance( pStart->m_fSurfPos, pEnd->m_fSurfPos, 2 ) ;
					dTipPosDist = nc_Distance( pStart->m_fPoint, pEnd->m_fPoint, 3 ) ;
					pPLine->m_fFeedRate = pPath->m_fFeedRate ;
					pPLine->m_bFeedType = pPath->m_bFeedType ;
					pPLine->m_bMoveFlag = pPath->m_bMoveFlag ;
					if( dPlanePosDist < 0.0001)
					{
						pPLine->m_fFeedScale = fPrevScale ;
					}
					else
					{
						pPLine->m_fFeedScale = TFLOAT( dTipPosDist / dPlanePosDist ) ;
					}
					/*if( pPLine->m_fFeedScale > 6 )
					{
						pPLine->m_fFeedScale = 6 ;
					}*/
					PComb.AddEntity ( pPLine ) ;
					fPrevScale = pPLine->m_fFeedScale ;
					pStart = pEnd ;
				}
			}
		}
		else
		{
			pPath->AddToPathCombine(PComb);
		}
		delete pPath ;
		AllPath.m_cAllPath.RemoveAt ( atpos ) ;
	}
	return TRUE ;
}

void CSmartPathGen::TransPointToCutPointEx(CSmtCPathLib &AllPath,CSmtCutPath*& pCutPath)
{
	//CSmtCPathLib NewPathlib ;
	//CSmtCutPath *pPath = NULL, *pNewPath = NULL ;
	//CSmtCutPoint *pHead = NULL ;
	//CSmtCutPointEx *pNewPnt = NULL ;
	//
	//POSITION pos = AllPath.m_cAllPath.GetHeadPosition (), atpos ;
	//pos = AllPath.m_cAllPath.GetHeadPosition () ;
	//while( pos )
	//{
	//	atpos = pos ;
	//	pPath = AllPath.m_cAllPath.GetNext ( pos ) ;
	//	AllPath.m_cAllPath.RemoveAt ( atpos ) ;

	//	pNewPath = new CSmtCutPath( MINI_MILL_PATH ) ;
	//	pNewPath->m_bFeedType = pPath->m_bFeedType ;
	//	pNewPath->m_bMoveFlag = pPath->m_bMoveFlag;
	//	pNewPath->m_fFeedRate = pPath->m_fFeedRate;

	//	pHead = pPath->m_pHead ;
	//	for( ; pHead ; pHead = pHead->next )
	//	{
	//		pNewPnt = new CSmtCutPointEx() ;
	//		mathFCpyPnt4D( pHead->m_fPoint, pNewPnt->m_fPoint  ) ;
	//		mathFCpyPnt( pHead->m_fPoint, pNewPnt->m_fSurfPos) ;
	//		pNewPath->AddTail ( pNewPnt ) ;
	//	}
	//	NewPathlib.AddToTail ( pNewPath ) ;
	//	delete pPath ;
	//}

	//// 清空AllPath
	//AllPath.ClearAllPath () ;
	//AllPath.AppendCPathLib ( NewPathlib ) ;
	//NewPathlib.m_cAllPath.RemoveAll () ;
	
	CSmtCutPath *pPath = NULL, *pNewPath = NULL ;
	CSmtCutPoint *pHead = NULL ;
	CSmtCutPointEx *pNewPnt = NULL ;

	POSITION pos = AllPath.m_cAllPath.GetHeadPosition (), atpos = NULL ;
	pos = AllPath.m_cAllPath.GetHeadPosition () ;
	while( pos )
	{
		atpos = pos ;
		pPath = AllPath.m_cAllPath.GetNext ( pos ) ;
		if (pPath == pCutPath)
		{
			break;;
		}
	}
	pNewPath = new CSmtCutPath( MINI_MILL5AX_PATH ) ;
	pNewPath->m_bFeedType = pPath->m_bFeedType ;
	pNewPath->m_bMoveFlag = pPath->m_bMoveFlag;
	pNewPath->m_fFeedRate = pPath->m_fFeedRate;

	pHead = pPath->m_pHead ;
	for( ; pHead ; pHead = pHead->next )
	{
		pNewPnt = new CSmtCutPointEx() ;
		mathFCpyPnt4D( pHead->m_fPoint, pNewPnt->m_fPoint  ) ;
		mathFCpyPnt( pHead->m_fPoint, pNewPnt->m_fSurfPos) ;
		pNewPath->AddTail ( pNewPnt ) ;
	}
	AllPath.m_cAllPath.InsertBefore(pos,pNewPath);
	AllPath.m_cAllPath.RemoveAt ( atpos ) ;
	pCutPath = pNewPath;
	delete pPath;	
	
}

///三轴路径点分布功能
void CSmartPathGen::PointDistributeFor3DPath(CPathGroup &NewPath,BOOL bMdfyGrnd,BOOL bMaxDist)
{
	int nRedistFlag = 0;
	nRedistFlag |= REDISTRIBUTE_ROUTH;
	if (bMdfyGrnd)
	{
		nRedistFlag |= REDISTRIBUTE_LEAD;
	}
	if (m_cFeedDef.m_cPlungeDef.m_nPlungeType == NCDEF_PLUNGE_CONTOUR)
	{
		nRedistFlag |= REDISTRIBUTE_PLUNGE;
	}
	if( m_cSetupDef.m_cTolDef.m_nFitType == NCDEF_FITTYPE_ARC )
	{/*圆弧逼近*/
		NewPath.FittingByArc3D( m_cSetupDef.m_cTolDef, NULL ) ;
	}
	else if( m_cSetupDef.m_cTolDef.m_nFitType == NCDEF_FITTYPE_SPLINE )
	{
		NewPath.FittingBySpline3D( MathCAM_FitBySpline, m_cSetupDef.m_cTolDef,NULL ) ;
	}
	else if(m_cSetupDef.m_cTolDef.m_nFitType == NCDEF_FITTYPE_LINENOARC )//guomin 替代圆弧
	{
		MathCAM_ReplaceArc_PathGroup(&NewPath, nRedistFlag, m_cSetupDef.m_cTolDef.m_dArcTol,m_cSetupDef.m_cTolDef.m_dAngTol,
			bMaxDist, m_c5DCtrlDef.m_cToolAxisApp.m_dMaxStepLen);
	}
	else if( m_cSetupDef.m_cTolDef.m_nFitType == NCDEF_FITTYPE_LINE )///直线保留圆弧
	{
		if( bMaxDist )
			MathCAM_MaxPointDistance_PathGroup(&NewPath, nRedistFlag, m_cSetupDef.m_cTolDef.m_dArcTol,m_cSetupDef.m_cTolDef.m_dAngTol,
			m_c5DCtrlDef.m_cToolAxisApp.m_dMaxStepLen) ;
	}
	else//重新分布目前替代圆弧+均匀步长
	{
		MathCAM_ReplaceArc_PathGroup(&NewPath, nRedistFlag, m_cSetupDef.m_cTolDef.m_dArcTol,m_cSetupDef.m_cTolDef.m_dAngTol,
			bMaxDist, m_c5DCtrlDef.m_cToolAxisApp.m_dMaxStepLen);
	}
}
///////////////////////////////////////////////////////////////////
// 三轴转五轴
///////////////////////////////////////////////////////////////////
#include "Nc5DPathTreat.h"
void CSmartPathGen::GetAll5AxisMdl ( CSmtCheckMdl &DriveMdl, CSmtCheckMdl &CheckMdl )
{
	DriveMdl.DeleteMultiCopy() ;

	DriveMdl.m_bCheckMode = SMART_MODEL_ROTATE ;
	CheckMdl.m_bCheckMode = SMART_MODEL_ROTATE ;
	DriveMdl.UpdateRotateTool ( DriveMdl.m_pTool ) ;
	CheckMdl.UpdateRotateTool( CheckMdl.m_pTool ) ;
	// 构建包围盒曲面，用于路径计算提速
	DriveMdl.UpdateBoundSurf() ;
	CheckMdl.UpdateBoundSurf() ;
	DriveMdl.m_pHolderShape = m_pHolderTool ;
	CheckMdl.m_pHolderShape = m_pHolderTool ;
}
void CSmartPathGen::GetAll5AxisPath( CSmtCPathLib &AllPath )
{
	CSmtCPathLib multiPath ;
	CSmtCutPath *pPath = NULL, *pNewPath = NULL ;
	CSmtCutPoint *pHead = NULL ;
	CSmtCutPointEx *pNewPnt = NULL ;
	// 如果有行号不合理，则需要重新设置行号的值，即bResort为TRUE
	BOOL bResort = FALSE ;
	POSITION pos = AllPath.m_cAllPath.GetHeadPosition () ;
	while( pos )
	{
		pPath = AllPath.m_cAllPath.GetNext ( pos );	
		if (pPath == NULL)
			continue;
		if( pPath->m_nLineNo < 0 )
		{
            bResort = TRUE ;
			break;
		}			
	}
	
	int nLineNo = 0 ;
	POSITION atpos ;
	pos = AllPath.m_cAllPath.GetHeadPosition () ;
	while( pos )
	{
		atpos = pos ;
		pPath = AllPath.m_cAllPath.GetNext ( pos ) ;
		AllPath.m_cAllPath.RemoveAt ( atpos ) ;
		pNewPath = new CSmtCutPath( MINI_MILL5AX_PATH ) ;
		pNewPath->m_bFeedType = pPath->m_bFeedType ;
		if( bResort )
		{
			pNewPath->m_nLayerNo = 0 ;
			pNewPath->m_nLineNo  = nLineNo ;
			nLineNo++ ;
		}
		else
		{
			pNewPath->m_nLayerNo = pPath->m_nLayerNo ;
			pNewPath->m_nLineNo = pPath->m_nLineNo ;
		}
		pHead = pPath->m_pHead ;
		for( ; pHead ; pHead = pHead->next )
		{
			pNewPnt = new CSmtCutPointEx() ;
			mathFCpyPnt4D( pHead->m_fPoint, pNewPnt->m_fPoint  ) ;
            mathFCpyPnt( pHead->m_fPoint, pNewPnt->m_fSurfPos) ;
			pNewPath->AddTail ( pNewPnt ) ;
		}
		multiPath.AddToTail ( pNewPath ) ;
		delete pPath ;
	}

	// 清空AllPath
	AllPath.ClearAllPath () ;
	AllPath.AppendCPathLib ( multiPath ) ;
	multiPath.m_cAllPath.RemoveAll () ;
}

int CSmartPathGen::CreateAll5AxisPath (int nAtCore, int nCoreNum, CSmtCheckMdl &DriveMdl,CSmtCPathLib &OrgPath, 
										LPVOID *AllPath, JDNC_PRGDEF &ProgDef, CNc5DAxisCtrl &axisCtrl, double dCur)
{
	INT_PTR nNum = OrgPath.m_cAllPath.GetCount () ;
	if( nNum < 1 ) return TRUE;

	ProgDef.m_dLimitAt  =  nNum / dCur	;	// 前进一次的计算量
	ProgDef.m_dStepAt   =  0.0			;	// 当前计算进度
	ProgDef.m_dIncStep  =  1.0			;	// 每次计算的前进量

	CSmtCutPath *pPath = NULL, *PathHead = NULL, *PathNext = NULL ;
	int nIndex = 0;
	POSITION pos = OrgPath.m_cAllPath.GetHeadPosition () ;
	while( pos )
	{
		if( ProgDef.m_pBrkFunc && ProgDef.m_pBrkFunc () )
		{
			break ;
		}
		MathCAM_MovePrgStep(ProgDef);
		pPath = OrgPath.m_cAllPath.GetNext ( pos ) ;
		if (nIndex++ % nCoreNum != nAtCore || pPath == NULL)
		{
			continue;
		}
		int nLine = pPath->m_nLineNo;
		Create5AxisPath( DriveMdl, axisCtrl, pPath ) ;
		PathHead = pPath ;
		AllPath[nIndex-1] = pPath;
		while( PathHead )
		{
			PathNext = PathHead->next ;
 			PathHead->m_nLineNo = nLine ;	// 行号可能会发生变化，所以把之前的行号再重新赋一遍值
			Nc5D_Simp5xPath( PathHead, 2.0e-4f, 1.0e-4f, ( TFLOAT)m_cSetupDef.m_cTolDef.m_dMaxStep ) ;
			PathHead = PathNext ;
		}
	}
	if( ProgDef.m_pBrkFunc && ProgDef.m_pBrkFunc () )
	{
		Nc5D_DestroyMultiCalcData(AllPath, nIndex, &OrgPath, pos, nCoreNum, nAtCore);		
		return FALSE;
	}

	return TRUE;
}
void CSmartPathGen::LinkAll5AxisPath ( CSmtCheckMdl &DriveMdl, CNc5DAxisCtrl &axisCtrl, CSmtCPathLib &AllPath  )
{
	if( AllPath.m_cAllPath.GetCount () < 1 ) return ;

	// 对路径进行连接处理
	double dDriveOffset = m_cStockDef.m_dDriveOffset[0] - m_cStockDef.m_dSparkGap ;
	double dCheckOffset = m_cStockDef.m_dCheckOffset[0] - m_cStockDef.m_dSparkGap ;
	CSmartTool* pNewTool = NULL , *pNewSafe = NULL ; 
	
    pNewTool = SmartNC_CreateTool( m_cToolDef, dDriveOffset - 0.01, m_cStockDef.m_dPlanarGap ) ;
	pNewSafe = SmartNC_CreateTool( m_cToolDef, dCheckOffset - 0.01, m_cStockDef.m_dPlanarGap ) ;

	if( pNewTool )
	{
		DriveMdl.m_pTool = pNewTool ;
		DriveMdl.m_dZShift -= 0.01 ;
	}
	if( DriveMdl.m_pCheckMdl && pNewSafe )
	{
		DriveMdl.m_pCheckMdl->m_pTool = pNewSafe ;
		DriveMdl.m_dZShift -= 0.01 ;
	}

	if( axisCtrl.m_cToolAxis.m_nAxisType == NCDEF_AXIS_VERT )
	{
		m_cFeedDef.m_cConnectDef.m_bConnect3DFlag |= NCDEF_FCONNECT3D_VERTAXIS ;
	}
	else if( axisCtrl.m_cToolAxis.m_nAxisFlag & NCDEF_AXIS_FLAG_LIMITANG )
	{
		SmartNC_SetACoordLimit( axisCtrl.m_cToolAxis.m_nRotAxis,
                                axisCtrl.m_cToolAxis.m_dElevationAng[0],
								axisCtrl.m_cToolAxis.m_dElevationAng[1] ) ;
	}
	

	// 添加抬刀路径
	Math5Ax_QuickConnectPath5Ax( DriveMdl, AllPath, m_cSetupDef.m_cTolDef, m_cSpeedDef.m_dRapidHeight, axisCtrl, m_cSpeedDef.m_dSafeHeight ) ;

    SmartNC_SetACoordLimit( 2,-360., 360. ) ;
	if( pNewTool )
	{
		DriveMdl.m_pTool = m_pTool ;
		DriveMdl.m_dZShift += 0.01 ;
	}
	if( DriveMdl.m_pCheckMdl && pNewSafe )
	{
		DriveMdl.m_pCheckMdl->m_pTool = m_pSafeTool ;
		DriveMdl.m_pCheckMdl->m_dZShift += 0.01 ;
	}
	if( pNewTool ) delete pNewTool ;
	if( pNewSafe ) delete pNewSafe ;

	if( axisCtrl.m_cToolAxis.m_nAxisType == NCDEF_AXIS_VERT ) 
	{
		AllPath.SetCutMode ( MINI_MILL_PATH ) ;
		AllPath.DelPointOnLine ( 0.0002 ) ;
	}
}
void CSmartPathGen::Create5AxisPath( CSmtCheckMdl &DriveMdl, CNc5DAxisCtrl &axisCtrl, CSmtCutPath *&pPath ) 
{
    double dCorner = m_cToolDef.m_dTopDiam*0.5 ;
    if( m_cToolDef.m_nToolType == surfncToolABall )
    {
        dCorner = m_cToolDef.m_dCorRadi ;
    }
	int i = 0 ;
	double dUW[2] = { 0., 0. } ;
	VEC3D MoveDir, ToolAxis, SurfNor = { 0., 0., 1. } ;
	PNT3D ToolPos ;
	CSmtCutPointEx *pHead = NULL, *pNext = NULL ;
	pHead = ( CSmtCutPointEx *)pPath->m_pHead ;
	while( pHead )
	{
		pNext = ( CSmtCutPointEx *)pHead->next ;
		if( pNext )
        {
			mathFGetUnitVec( pHead->m_fSurfPos, pNext->m_fSurfPos, MoveDir ) ;
        }
		// 刀心点
		ToolPos[0] = pHead->m_fSurfPos[0] ;
		ToolPos[1] = pHead->m_fSurfPos[1] ;
		ToolPos[2] = pHead->m_fSurfPos[2] + dCorner ;
		// 得到刀轴方向
		if( axisCtrl.AdjustToolAxisEx ( ToolPos, ToolPos, dUW, MoveDir, SurfNor, ToolAxis ) )
		{
			nc_DoubleToFloat( pHead->m_fSurfNor, ToolAxis, 3 ) ;
		}
		// 得到刀尖点
		for( i = 0 ; i < 3 ; i++ )
		{
			pHead->m_fPoint[i] = TFLOAT( ToolPos[i] - dCorner * pHead->m_fSurfNor[i] ) ;
		}
		pHead = pNext ;
	}
	// STEP 2 : 曲线跌代,确保在导动曲面上的角度误差
    double dMinCos = cos( ANGLE_TO_RADIAN( axisCtrl.m_cToolAxis.m_dMaxAngleStep )) ;
	double dTmpCos[2] = { 0., 0. }, dCosA = 0., dDist = 0. ;
	CSmtCutPointEx tmpPnt, *pNew = NULL ;
	
	pHead = ( CSmtCutPointEx *)pPath->m_pHead ;
	for( ; pHead && pHead->next ; )
	{
		pNext = ( CSmtCutPointEx *)pHead->next ;
		dCosA = nc_OProduct( pHead->m_fSurfNor, pNext->m_fSurfNor, 3 ) ;
		dDist = mathFDist( pHead->m_fPoint, pNext->m_fPoint ) ;
		if( dCosA < dMinCos || dDist > m_cSetupDef.m_cTolDef.m_dMaxStep )
		{
			// 得到中点
			Create5AxisPnt( *pHead, *pNext, axisCtrl, tmpPnt ) ;
			dTmpCos[0] = nc_OProduct( pHead->m_fSurfNor, tmpPnt.m_fSurfNor, 3 ) ;
			dTmpCos[1] = nc_OProduct( pNext->m_fSurfNor, tmpPnt.m_fSurfNor, 3 ) ;
			// 如果插入的点不能满足tmpPnt和pHead之间的角度,不要
			if( dTmpCos[0] > dCosA && dTmpCos[1] > dCosA )
			{
				pNew = new CSmtCutPointEx( tmpPnt ) ;
				pPath->InsertAfter ( pNew, pHead ) ;
			}
			else
			{
				pHead = pNext ;
			}
		}
		else
		{
			pHead = pNext ;
		}
	}
	
	// 加工模型刀轴方向投影
	pHead = ( CSmtCutPointEx *)pPath->m_pHead ;
	for( ; pHead ; pHead = ( CSmtCutPointEx *)pHead->next )
	{
		Define5AxisPnt( DriveMdl, *pHead, m_cSetupDef.m_cTolDef ) ;
	}
	
	// 加密有效点
	InsertCutPntEx( DriveMdl, axisCtrl, pPath, m_cSetupDef.m_cTolDef ) ;
	
	// 精确迭代有效点和无效点之间的分界点
	FindDividePntEx( DriveMdl, axisCtrl, pPath, m_cSetupDef.m_cTolDef ) ;

	// 打断pPath
	CSmtCutPath *pResult = pPath->BreakAtNullPoint ( FALSE ) ;
	if( pPath ) delete pPath ;
	pPath = pResult ;
}
void CSmartPathGen::Create5AxisPnt( CSmtCutPointEx& Start, CSmtCutPointEx& End,
									CNc5DAxisCtrl& AxisCtrl, CSmtCutPointEx &tmpPnt )
{
    VEC3D MoveDir = { 1.0,0.0,0.0}, ToolAxis, SurfNor = { 0., 0., 1. } ;
	PNT3D ToolPos ;
    double dCorner = m_cToolDef.m_dTopDiam*0.5 ;
    if( m_cToolDef.m_nToolType == surfncToolABall )
    {
        dCorner = m_cToolDef.m_dCorRadi ;
    }
    double dUW[2] = { 0., 0. } ;
    nc_GetPointAtLine( Start.m_fSurfPos, End.m_fSurfPos, 0.5f, tmpPnt.m_fSurfPos, 3 ) ;
    mathFGetUnitVec( Start.m_fSurfPos, End.m_fSurfPos, MoveDir ) ;
    ToolPos[0] = tmpPnt.m_fSurfPos[0] ; 
    ToolPos[1] = tmpPnt.m_fSurfPos[1] ; 
    ToolPos[2] = tmpPnt.m_fSurfPos[2] + dCorner; 
	// 计算刀轴
	if( AxisCtrl.AdjustToolAxisEx( ToolPos, ToolPos, dUW, MoveDir, SurfNor, ToolAxis ) )
	{
		nc_DoubleToFloat( tmpPnt.m_fSurfNor, ToolAxis, 3 ) ;
	}
    else
    {
        tmpPnt.m_fSurfNor[0] = tmpPnt.m_fSurfNor[1] = 0.0f ;
        tmpPnt.m_fSurfNor[2] = 1.0f ;
    }
	// 得到刀尖点 封掉函数得到的刀尖点与下行函数有偏差，从而导致跌代插点的时候出现死循环
	for( int i = 0 ; i < 3 ; i++ )
	{
		tmpPnt.m_fPoint[i] = TFLOAT( ToolPos[i] - dCorner * tmpPnt.m_fSurfNor[i]) ;
	}
	// 如果得到的刀尖点不合理，则得到线段的中点
	if (!IsCenterPointValid(Start, End, tmpPnt, AxisCtrl))
		nc_GetPointAtLine( Start.m_fPoint, End.m_fPoint, 0.5f, tmpPnt.m_fPoint, 3 ) ;
}

// 判断插入的路径点是否合理
BOOL CSmartPathGen::IsCenterPointValid(CSmtCutPointEx& Start, CSmtCutPointEx& End,
                                       CSmtCutPointEx& Center, CNc5DAxisCtrl &/*AxisCtrl*/)
{
	// 两点必须满足平滑过渡
	VEC3D Dir;		// 两点正向
	for (int k=0; k<3; k++)
		Dir[k] = End.m_fPoint[k] - Start.m_fPoint[k];
	// 单位化
	double dLen = nc_VectorLen( Dir, 3 ) ;
	if( dLen > MIN_DIS ) 
	{
		for(int k = 0 ; k < 3 ; k ++ ) 
			Dir[k] /= dLen ;
	}
	else
	{
		Dir[0] = Dir[1] = Dir[2] = 0.0; 
	}
	double  b = 0.0 , t ;
	for( int k = 0 ; k < 3 ; k++ )
	{
		t = Center.m_fPoint[k] - Start.m_fPoint[k] ;
		b += ( t * Dir[k] ) ;
	}
	// 中点在两点之外
	if( b < 0.0 || b > dLen ) 
		return FALSE ;

	return TRUE;
}

void CSmartPathGen::Define5AxisPnt ( CSmtCheckMdl &DriveMdl, CSmtCutPointEx &CutPnt, JDNC_TOL &cTol )
{
	CutPnt.m_bType = 5 ;
    if( DriveMdl.Is5AxPointOvercut( CutPnt, max( cTol.m_dArcTol * 3.0, 0.01 ) ) )
	{
		CutPnt.m_bType = 0 ;
	}
    
	if( CutPnt.m_bType && DriveMdl.m_pHolderShape )
	{
		if(DriveMdl.Is5AxPointCutHolder ( CutPnt, 2.0e-4 ) )
		{
			CutPnt.m_bType = 0 ;
		}
	}
}

void CSmartPathGen::InsertCutPntEx ( CSmtCheckMdl &DriveMdl, 
                                     CNc5DAxisCtrl &axisCtrl, 
									 CSmtCutPath *pPath    , 
                                     JDNC_TOL &cTol)
{
	if( !pPath ) return ;
	CSmtCutPointEx *pHead = ( CSmtCutPointEx *)pPath->m_pHead, *pNext = NULL ;
	CSmtCutPointEx tmpPnt, *pNew = NULL ;
	
	double dDist = 0., dArcTol = 0., dStep[2] = { 0., 0. } ;
	for( ; pHead && pHead->next ; )
	{
		pNext = ( CSmtCutPointEx *)pHead->next ;
		if( pHead->m_bType <= 0 && pNext->m_bType <= 0 )
		{ // 无效区域
			pHead = pNext ;
			continue ;
		}
		dDist = mathFDist( pHead->m_fPoint, pNext->m_fPoint ) ;
		if( dDist < 0.02 )
		{ // 接近点
			pHead = pNext ;
			continue ;
		}
		// 计算中点
		Create5AxisPnt( *pHead, *pNext, axisCtrl, tmpPnt ) ;
		// 刀轴投影
		Define5AxisPnt( DriveMdl, tmpPnt, cTol ) ;
		// 避免tmpPnt和pHead或者pNext重合
		dStep[0] = mathFDist( tmpPnt.m_fPoint, pHead->m_fPoint ) ;
		dStep[1] = mathFDist( tmpPnt.m_fPoint, pNext->m_fPoint ) ;
		dArcTol = nc_PointLineDist( pHead->m_fPoint, pNext->m_fPoint, tmpPnt.m_fPoint ) ;
		if( ( dDist > cTol.m_dMaxStep || dArcTol > cTol.m_dArcTol ) &&
			dStep[0] > 0.0002 && dStep[1] > 0.0002 )
		{ // 添加点
			pNew = new CSmtCutPointEx( tmpPnt ) ;
			pPath->InsertAfter ( pNew, pHead ) ;
		}
		else
		{
			pHead = pNext ;
		}
	}
}
void CSmartPathGen::FindDividePntEx ( CSmtCheckMdl &DriveMdl, CNc5DAxisCtrl &axisCtrl, 
										    CSmtCutPath *pPath, JDNC_TOL &cTol)
{
	if( !pPath ) return ;
	CSmtCutPointEx *pHead = ( CSmtCutPointEx *)pPath->m_pHead, *pNext = NULL ;
	CSmtCutPointEx tmpPnt, start, end, *pNew = NULL, *pObj = NULL ;
	
	double dDist = 0. ;
	
	for( ; pHead && pHead->next ; pHead = pNext )
	{
		pNext = ( CSmtCutPointEx *)pHead->next ;
		if( pHead->m_bType <= 0 && pNext->m_bType > 0 )
		{
			start = *pHead, end = *pNext, pObj = pNext ;
		}
		else if( pHead->m_bType > 0 && pNext->m_bType <= 0 )
		{
			start = *pNext, end = *pHead, pObj = pHead ;
		}
		else
		{
			continue ;
		}
		if( pHead->m_bType <= 0 && pNext->m_bType <= 0 )
		{ // 无效区域
			pHead = pNext ;
			continue ;
		}
		dDist = mathFDist( pHead->m_fPoint, pNext->m_fPoint ) ;
		while( dDist > cTol.m_dArcTol * 0.5 )
		{ 
			// 计算中点
			Create5AxisPnt( start, end, axisCtrl, tmpPnt ) ;
			// 刀轴投影
			Define5AxisPnt( DriveMdl, tmpPnt, cTol ) ;
			dDist *= 0.5 ;
			if( tmpPnt.m_bType <= 0 )
			{
				start = tmpPnt ;
			}
			else
			{
				end = tmpPnt ;
			}
		}

		if( mathFDist( pObj->m_fPoint, end.m_fPoint ) > cTol.m_dArcTol * 0.5 )
		{ // 添加点
			pNew = new CSmtCutPointEx( end ) ;
			pPath->InsertAfter ( pNew, pHead ) ;
		}
	}
}
static UINT MathCAM_TransFrom3AxisTo5AxisSubProc(LPVOID lpParam)
{
	if (lpParam == NULL)
		return 0;
	TF3X5XT_DATA *pData = (TF3X5XT_DATA *)lpParam;
	return pData->pPathGen->CreateAll5AxisPath(pData->nAtCore, pData->nCoreNum, *pData->DriveMdl, *pData->OrgPath, pData->NewPath, 
		pData->ProgDef, *pData->pAxisCtrl, pData->dCur);
}

// 得到曲面的法矢走向
int MathCAM_GetSurfNormalDir(CGeoTrmSurf* pSurf, RFRAME& cRFrame)
{
	if( pSurf == NULL ) return 0 ;
	FACET* pFacet = pSurf->DiscreteToFacet( 0.1, 0.5235987) ; //30度=0.52359877559829887307710723054658
	if( ! pFacet ) return 0 ;

	VEC3D normal ;
	int i, nSameNormal = 0, nCnt[3] ;
	nCnt[0] = nCnt[1] = nCnt[2] = 0 ;
	for( i = 0 ; i < pFacet->nnode ; i++ )
	{
		mathTransWorldVec3D( &cRFrame, pFacet->nodes[i].normal, normal ) ;
		if( normal[2] > 1.0e-3 ) nCnt[0] ++ ;
		else if( normal[2] < -1.0e-3 ) nCnt[1] ++ ;
		else nCnt[2] ++ ;
	}
	if( nCnt[0] > nCnt[1] && nCnt[0] > nCnt[2] ) nSameNormal = 1 ;
	else if( nCnt[1] > nCnt[0] && nCnt[1] > nCnt[2] ) nSameNormal = -1 ;
	else nSameNormal = 0 ;
	delete pFacet ;
	return nSameNormal ;
}

// 得到合适的UW参数
double MathCAM_UWParam( double uInit )
{
	double u = uInit ;
	if( u < 2.0e-6 ) u = 0.0 ;
	else if( u > 1.0 - 2.0e-6 ) u = 1.0 ;
	return u ;
} 

BOOL CSmartPathGen::TransAllPathFrom3AxisTo5Axis( CSmartGraphic& Graph  ,
                                                  CSmtCheckMdl& DriveMdl, 
                                                  CSmtCPathLib& AllPath ,
                                                  CPathGroup &NewLib   ) 
{
	if( AllPath.m_cAllPath.IsEmpty() )
    {
        return TRUE ;
    }
	//STEP 1 : 生成五轴的路径模型
	C5XGraph allGraph ; // 五轴信息
	Graph.GetAll5XObject( m_cSetupDef, 0, &allGraph ) ;
	CNc5DAxisCtrl axisCtrl( m_c5DCtrlDef ) ;
	if( axisCtrl.UpdateAxisGraph( allGraph ) == FALSE )
	{// 更新刀轴控制图形
		m_nErrorType = JDERROR_GENPATH5AX_NOAXISCURVE;
		return FALSE;
	}
	// 重新生成刀具并且缩小,防止插点过程中在直侧壁将点删除
	DOUBLE dOffRad = max( 0.01, m_cSetupDef.m_cTolDef.m_dArcTol * 3 ) ;
	double dOffset1 = GetDriveSurfOffset() - dOffRad ;
	double dOffset2 = GetCheckSurfOffset() - dOffRad ;
	CSmartTool *pNewTool = CreateSmartTool( m_cToolDef, dOffset1, GetWedmPlanarGap() ) ;
	if( pNewTool ) 
	{
		delete m_pTool ;
		m_pTool = pNewTool ;
	}
	CSmartTool *pCheckTool = CreateSmartTool( m_cToolDef, dOffset2, GetWedmPlanarGap() ) ;
    if( pCheckTool ) 
	{
		delete m_pSafeTool ;
		m_pSafeTool = pCheckTool ;
	}
	if( !TransTo5AxDriveAndCheckMdl( DriveMdl )  )
	{//转换成旋转模型
		m_nErrorType = JDERROR_GENPATH_NOSURFACE ;
		return FALSE ;
	}	
	RFRAME ncMtx  ;
	mathInitRFrame( &ncMtx ) ;
	if( mathIsRFrameUnit(&DriveMdl.m_dNcMtx) == 0 )
	{
		ncMtx = DriveMdl.m_dNcMtx ;
		axisCtrl.m_dNcMtx = &ncMtx ;
	}
	// STEP 2 : 三轴路径转换成五轴路径 
	SurfNC_SetNewStepEx( IDS_PATHGEN_STEP_3To5AXIS ) ;
	// 出示化进度条
    m_cPrgDef.m_dLimitAt = 100 / ( 100 * 1.00  ) ;
	m_cPrgDef.m_dStepAt  = 0 ;
	m_cPrgDef.m_dIncStep = 1. ;
	// 路径的转换
	GetAll5AxisPath(AllPath);
	// 分配进度条
	double dCur1 = 100 ;
	// 支持多线程，即拷贝数据不为空
	INT_PTR nPathNum = AllPath.GetNumPath();
	if(nPathNum>=2 && IsNeedMultiCopy(DriveMdl))
	{
		DriveMdl.CreateMultiCopy(m_nCalcThreadNum - 1) ;
	}
	CVecPointer NewPath;
	NewPath.resize(nPathNum + 5);
	ZeroMemory(&NewPath[0], sizeof(LPVOID) * (nPathNum + 5));
	if (nPathNum>=2 && DriveMdl.GetMultiCopy() != NULL)
	{
		TF3X5XT_DATA ThreadData[NC_CFG_CPU_MAXNUM];
		LPVOID lpParam[NC_CFG_CPU_MAXNUM] = {NULL};
		JDNC_PRGDEF tmpPrg = m_cPrgDef;
		tmpPrg.m_pPrgFunc = NULL ; 
		tmpPrg.m_pPosFunc = NULL ; 
		int nThreadNum = min(DriveMdl.GetMultiCopyCount()+1, (int)nPathNum) ;
		for( int i=0; i<nThreadNum; i++)
		{
			ThreadData[i].DriveMdl = i == 0 ? &DriveMdl : DriveMdl.GetMultiCopy(i-1);
			ThreadData[i].nAtCore = i;
			ThreadData[i].nCoreNum = nThreadNum;
			ThreadData[i].OrgPath = &AllPath;
			ThreadData[i].NewPath = &NewPath[0];
			ThreadData[i].ProgDef = i==0 ? m_cPrgDef : tmpPrg;
			ThreadData[i].pAxisCtrl = &axisCtrl;
			ThreadData[i].dCur = dCur1;
			ThreadData[i].pPathGen = this;
			lpParam[i] = &ThreadData[i];
		}
		MathCAM_ThreadMainFunc(MathCAM_TransFrom3AxisTo5AxisSubProc, lpParam, nThreadNum);
	}
	else
	{//单线程生成五轴路径
		CreateAll5AxisPath(0, 1, DriveMdl, AllPath, &NewPath[0], m_cPrgDef, axisCtrl, dCur1 ) ;
	}
	AllPath.m_cAllPath.RemoveAll() ;
    if( SurfNC_IsAbort() )
    {        
        m_nErrorType = JDERROR_GENPATH_ABORT  ;
        return FALSE ;
    }
	else
	{
		Nc5D_AddPathArrayToLib(&NewPath[0], nPathNum, AllPath);
	}
	//STEP 3 :添加连刀
	if( AllPath.m_cAllPath.GetCount () > 0 )
	{//由大于1改为大于0，是因为如果生成的路径只有一条时，也需要通过该函数为路径起末点增加快速定位路径 qqs 2014.02.26
		LinkAll5AxisPath( DriveMdl, axisCtrl, AllPath ) ;
	}
	NewLib.m_nType = NC_PATHTYPE_ENGRAVE5AX ;
	return TRUE ;
}

// 判断刀轴模型是否合理
BOOL CSmartPathGen::IsAxisCtrlValid(CSmartGraphic &Graph)
{
	C5XGraph allGraph ; // 五轴信息
	CNc5DAxisCtrl axisCtrl( m_c5DCtrlDef ) ;
	if (m_c5DCtrlDef.m_cToolAxis.m_nAxisType != NCDEF_AXIS_VERT)
	{
		Graph.GetAll5XObject( m_cSetupDef, 0, &allGraph ) ;
		if( axisCtrl.UpdateAxisGraph( allGraph ) == FALSE )
		{// 更新刀轴控制图形
			m_nErrorType = JDERROR_GENPATH5AX_NOAXISCURVE;
			return FALSE;
		}
	}
	return TRUE;
}

BOOL CSmartPathGen::IsNeedMultiCopy(CSmtCheckMdl& DriveMdl)
{
	return MathCAM_IsNeedMultiCopy(DriveMdl, m_nCalcThreadNum);
}

// 在固定位置增加螺旋下刀路径
int MathCAM_CreateHelixPlungeAt( CSmtCPathLib& AllPath,  /*路径组  */
							    JDNC_PLUNGE& Plunge   ,  /*下刀方式*/
							    PNT3D   AtPoint       ,  /*刀具位置*/
							    CSmartLoop* BndCont   )  /*边界轮廓*/
{
	if( Plunge.m_nPlungeType != NCDEF_PLUNGE_HELIX )
	{ /*螺旋下刀*/
		return FALSE ;
	}
	PNT3D dStart, dEnd ;
	DOUBLE dInc , dLen  ;
	DOUBLE dTan   = tan( ANGLE_TO_RADIAN( Plunge.m_dAngle ) ) ;
	DOUBLE dDepth = Plunge.m_dIncStep ;
	DOUBLE dRange[2] , dRadius = Plunge.m_dRadius ;
	dRange[0] = 0.0 ;
	dRange[1] = dRange[0] + MiniPai2 ;
    DOUBLE dBldRad = 0.05 ;
    if( Plunge.m_bPlungeFlag & NCDEF_PLUNGE_DELBLINDAREA )
    {
        dBldRad = max( 0.05, Plunge.m_dBlindRadius - 0.01 ) ;
    }
	for( int nTry = 1 ; nTry <= 4 && BndCont ; nTry ++, dRadius *= 0.8 ) 
	{
		if( dRadius < dBldRad ) return FALSE ;
        PNT2D dMidPnt[4] ;
		CSmartArc tmpArc2D( AtPoint, dRange, dRadius-0.001 ) ;
		tmpArc2D.GetPoint( 0.5, dMidPnt[0] ) ;
		tmpArc2D.GetPoint( 0.75,dMidPnt[1]) ;
		tmpArc2D.GetPoint ( 0., dStart ) ;
		CSmartLine tmpLine( AtPoint, dStart ) ;
		tmpLine.GetPoint( 0.5,dMidPnt[2]) ;
		tmpLine.GetPoint( 0.75,dMidPnt[3]) ;
		for( CSmartLoop* pCont = BndCont ; pCont ; pCont = pCont->next )
		{
			if( pCont->IsPtOnContour( dMidPnt[0] ) &&
                pCont->IsPtOnContour( dMidPnt[1] ) &&
                pCont->IsPtOnContour( dMidPnt[2] ) &&
                pCont->IsPtOnContour( dMidPnt[3] )   )
			{
				break ;
			}
		}
		if( pCont == NULL ) continue  ; // 不在任何轮廓中间
		for( pCont = BndCont ; pCont ; pCont = pCont->next )
		{
			if( pCont->IntSectContour( &tmpArc2D,FALSE ) || 
				pCont->IntSectContour( &tmpLine ,FALSE) )
			{
				break ;
			}
		}
		if( pCont == NULL ) break  ; // 与轮廓不相交
	}
	if( BndCont && nTry >= 5 )  return FALSE ;
	CPathCombine  tmpComb( NC_WPROCESS_PLUNGE ) ;
	CSmartArc Arc2D( AtPoint, dRange, dRadius ) ;
	dLen = MiniPai2 * dRadius ;
	dInc = min( dLen * dTan,Plunge.m_dMaxZInc ) ;
	int nLoop = (int)ceil( dDepth / dInc ) ;
	if( nLoop <= 0 ) nLoop = 1 ; 
	dInc = dDepth / nLoop   ; 
	for(int i = 0 ; i < nLoop ; i ++ ) 
	{
		CPathArc3D* pArc3D = new CPathArc3D() ;
		pArc3D->Create( dDepth- i * dInc, Arc2D ) ;
		pArc3D->m_fDepth = - dInc ;
		tmpComb.AddEntity( pArc3D ) ;
	} 
	tmpComb.GetEndPoint( 1, dStart ) ;
	nc_VectorCopy( dEnd , AtPoint, 2 ) ;
	dEnd[2] =  0.0 ;
	CPathLine3D* pLine  = new CPathLine3D( dStart, dEnd ) ;
	tmpComb.AddEntity( pLine ) ;
	tmpComb.MoveCombine( 0.0, 0.0, AtPoint[2] ) ;
	JDNC_TOL  tmpTol ;
	SurfNC_InitPathParam( tmpTol ) ;
	tmpTol.m_dArcTol = 0.004 ;
	AllPath.CreateCutPath( tmpComb, tmpTol  ) ;
	return TRUE ; 
}

int CSmartPathGen::CreatePocketLeadPath(CPathCombine&   tmpTComb  ,/*退刀路径*/
	                                      JDNC_LEAD&     LeadOut ,  /*退刀参数*/                   	
	                                         PNT2D         Point ,		/*端点*/
	                                         VEC2D         Tangent,		/*切失*/
	                                         VEC2D          Normal,		/*法失*/
	                                         int            IsOut ,      /*切出*/
	                                           CSmartLoop&  AllCont,	/*干涉检查*/
	                                           DOUBLE     ZShift[2] )   /*起末点抬高  */  
{	
	CSmartLoop* pContour =&AllCont;
	CSmartCurve LeadOutCv ;
	DOUBLE DepthAt=0;	

	if (LeadOutCv.m_pHead)
	{
		LeadOutCv.ClearAll();
	}
	BOOL bRet= CreateLeadOutEntNew(LeadOut,Point,Tangent,Normal,IsOut,pContour,LeadOutCv);
	
	if (!bRet)return 0;
	
	if(LeadOut.m_nLeadType  != NCDEF_LEAD_CLOSE && LeadOutCv.m_pHead  == NULL)
	{
		LeadOutCv.ClearAll() ;	
		return 0 ;
	}
	if( LeadOutCv.m_pHead )
	{
		if( LeadOut.m_nLeadType == NCDEF_LEAD_BYCONTOUR )
		{
			if( DepthAt > 0.0 )
			{
				MathCAM_SetZValue( LeadOutCv,  0.0, DepthAt) ;
			}
		}
		else if( ZShift[1] > 0. )
		{
			if( !ChangeLeadArcHeightBySpline( &LeadOutCv, LeadOut, ZShift[1], FALSE ) )
			{
				MathCAM_SetZValue( LeadOutCv,  0.0, ZShift[1]) ;
			}
		}
	}	
	
	tmpTComb.AppendCurve3D(LeadOutCv,0,FALSE);
	if(!tmpTComb.m_pHead)
		return 0;
	return 1;
}

// 在固定位置增加折线下刀路径
int MathCAM_CreateRampPlungeAt( CSmtCPathLib&  AllPath,  /*路径组  */
							    JDNC_PLUNGE& Plunge   ,  /*下刀方式*/
							    PNT3D   AtPoint       ,  /*刀具位置*/
							    CSmartLoop* BndCont  )   /*边界轮廓*/
{
	if( Plunge.m_nPlungeType != NCDEF_PLUNGE_RAMP )
	{ /*不是折线下刀*/
		return FALSE ;
	}
    ASSERT( BndCont ) ;
	double dLineLen = max( 0.1, Plunge.m_dRadius ) ;
    if( Plunge.m_bPlungeFlag & NCDEF_PLUNGE_DELBLINDAREA )
    {
        dLineLen = max( dLineLen, Plunge.m_dBlindRadius * 2.0 + 0.01 ) ;
    }
	DOUBLE dAngle = 0.0, dMidPnt[2];
	CSmartLine tmpLine ;
	tmpLine.m_aPoint[0][0] = AtPoint[0] ;
	tmpLine.m_aPoint[0][1] = AtPoint[1] ;
	for( int nTry = 0 ; nTry < 36 && BndCont; nTry ++ )
	{
		tmpLine.m_aPoint[1][0] = AtPoint[0] + dLineLen * cos( dAngle ) ;
		tmpLine.m_aPoint[1][1] = AtPoint[1] + dLineLen * sin( dAngle ) ;
		tmpLine.DefineBox() ;
        tmpLine.GetPoint( 0.5, dMidPnt ) ;
		for( CSmartLoop* pCont = BndCont ; pCont ; pCont = pCont->next )
		{
			if( pCont->IsOnContourEx( tmpLine.m_aPoint[1]) && 
                pCont->IsOnContourEx( dMidPnt )  )
			{
				break ;
			}
		}
		if( pCont == NULL ) 
		{/*不在任何轮廓内部*/
			dAngle += 0.17 ;
			continue ;
		}
		for( pCont = BndCont ; pCont ; pCont = pCont->next )
		{
			if( pCont->IntSectContour( &tmpLine, FALSE ) )
			{
				break ;
			}
		}
		if( pCont == NULL ) break ;
		dAngle += 0.17 ;
	}
	if( nTry == 36 && BndCont ) return FALSE ; 
	DOUBLE dTan   = tan( ANGLE_TO_RADIAN( Plunge.m_dAngle ) ) ;
    double dLen = 2.0 * dLineLen ;
	double dInc = min( dLen * dTan,Plunge.m_dMaxZInc ) ;
	DOUBLE dDepth = Plunge.m_dIncStep ;
	int nLoop = (int)ceil( dDepth / dInc ) ;
	if( nLoop <= 0 ) nLoop = 1 ; 
	dInc = dDepth / nLoop   ; 
    /*计算坐标*/ 
	PNT3D dStart, dEnd ;
	for( int k = 0 ; k < 2 ; k ++ ) 
	{
		dStart[k]  = tmpLine.m_aPoint[0][k] ;
		dEnd[k]    = tmpLine.m_aPoint[1][k] ;
	}
	dStart[2] = dDepth  ;
	dEnd[2]   = dStart[2] - dInc/2.0 ;
	/* 生成折线*/
	CPathCombine  tmpComb( NC_WPROCESS_PLUNGE ) ;
	CPathPLine3D * pLine3D = new CPathPLine3D() ;
	pLine3D->m_pTAPos = new TPNT3D[ nLoop * 2 + 1 ] ;
	pLine3D->m_nCount = nLoop * 2  ;
	for( int i = 0 ; i < nLoop ; i ++ ) 
	{
		for( int nDim = 0 ; nDim < 3 ; nDim ++ ) 
		{
			pLine3D->m_pTAPos[i*2][nDim]   = dStart[nDim] ;
	        pLine3D->m_pTAPos[i*2+1][nDim] = dEnd[nDim];
		}
	   dStart[2] -= dInc , dEnd[2] -= dInc ;
	}  
	pLine3D->m_pTAPos[nLoop*2][0] = AtPoint[0] ;
	pLine3D->m_pTAPos[nLoop*2][1] = AtPoint[1] ;
	pLine3D->m_pTAPos[nLoop*2][2] = 0.0f ;
	tmpComb.AddEntity( pLine3D ) ;
	tmpComb.MoveCombine( 0.0, 0.0, AtPoint[2] ) ;
	JDNC_TOL  tmpTol ;
	SurfNC_InitPathParam( tmpTol ) ;
	tmpTol.m_dArcTol = 0.004 ;
	AllPath.CreateCutPath( tmpComb, tmpTol  ) ;
	return TRUE ; 
}
// 在固定位置增加螺旋下刀路径
int MathCAM_CreateCurvePlungeAt( CSmtCPathLib& AllPath ,  /*路径组  */
							     JDNC_PLUNGE& Plunge  ,  /*下刀方式*/
							     PNT3D   AtPoint      ,  /*刀具位置*/
								 CSmartCurve * pCurve  ,  /*刀具曲线*/
							     CSmartLoop* BndCont  )   /*边界轮廓*/
{
	if( Plunge.m_nPlungeType != NCDEF_PLUNGE_CONTOUR || ! pCurve )
	{ /*螺旋下刀*/
		return FALSE ;
	}
	// STEP 0 : 计算长度
	CPathCombine  tmpComb( NC_WPROCESS_PLUNGE ) ;
	CSmartCurve   PlgCurve ;
	DOUBLE dCurveLen = pCurve->GetLength() ;
    if( dCurveLen < 0.02 ) return FALSE ;
	DOUBLE dTan = tan( ANGLE_TO_RADIAN( Plunge.m_dAngle)) ;
    if( Plunge.m_dIncStep > Plunge.m_dMaxZInc )
    {
        double dMaxTan = Plunge.m_dMaxZInc / dCurveLen ;
        if( dTan > dMaxTan ) dTan = dMaxTan ;
    }
    DOUBLE dSlantLen = Plunge.m_dIncStep / dTan;
	int nLoop = (int)( dSlantLen / dCurveLen ) ;
	if( nLoop > 30 && dCurveLen < 1.0 )nLoop  =  30  ;/*短线段保护*/
	// STEP 1 : 路径分段
	DOUBLE dLeft = dSlantLen - dCurveLen * nLoop ;
    CSmartCurve *pFirst = NULL, *pSecond = NULL  ;
    CSmartCurve TmpCurve ;

	if( pCurve->IsClosed() == 0  )
	{ // 开口曲线
	    pCurve->BreakCurveAt(  dLeft, pFirst, pSecond ) ;
		if( pFirst )
		{
		   pFirst->Reverse() ;
		   PlgCurve.AddCurve( pFirst ) ;
		}
		if( pSecond ) delete pSecond ;
	    if(  nLoop %2   )  nLoop ++ ;
	    for( int i = 0 ; i < nLoop ; i ++ ) 
		{
		   pFirst =  pCurve->CopyMyself() ;
		   if( i % 2 ) pFirst->Reverse() ;
		   PlgCurve.AddCurve( pFirst ) ;
		}
	}
	else
	{ // 封闭曲线
		pCurve->BreakCurveAt(  dCurveLen - dLeft , pSecond, pFirst ) ;
		if( pFirst ) PlgCurve.AddCurve( pFirst->CopyMyself() ) ;
		for( int i = 0 ; i < nLoop ; i ++  )
		{
			if( pSecond ) PlgCurve.AddCurve( pSecond->CopyMyself() ) ;
			if( pFirst  ) PlgCurve.AddCurve( pFirst->CopyMyself() ) ;
		}
		if( pFirst ) delete pFirst ;
		if( pSecond ) delete pSecond ;
	}		
	tmpComb.AddCurveEx( PlgCurve, Plunge.m_dIncStep, Plunge.m_dIncStep ) ;
	if( tmpComb.m_pHead == NULL ) return FALSE ;
	PNT3D dStart , dEnd ;
	tmpComb.GetEndPoint( 1, dStart ) ;
	nc_VectorCopy( dEnd, AtPoint, 2 ) ;
	dEnd[2] = 0.0 ;
	if( nc_Distance( dStart,dEnd, 3 )  > 2.0e-4 )
	{
		CSmartLine tmpLine( dStart, AtPoint ) ;
		for( CSmartLoop * pCont = BndCont ; pCont ; pCont = pCont->next )
		{
			if( pCont->IntSectContour( & tmpLine, FALSE ) )
			{
				return FALSE ;
			}
		}
		CPathLine3D* pLine3D = new CPathLine3D( dStart , dEnd ) ;
		pLine3D->m_bFeedType = JDNC_FEEDTYPE_CONNECT ;
		tmpComb.AddEntity( pLine3D ) ;
	}
	tmpComb.MoveCombine( 0.0, 0.0, AtPoint[2] ) ;
	JDNC_TOL  tmpTol ;
	SurfNC_InitPathParam( tmpTol ) ;
	tmpTol.m_dArcTol = 0.004 ;
	AllPath.CreateCutPath( tmpComb, tmpTol  ) ;
	return TRUE ; 
}

// 在固定位置增加螺旋下刀路径
int MathCAM_CreateVertPlungeAt( CSmtCPathLib&  AllPath ,  /*路径组  */
							    JDNC_PLUNGE& Plunge  ,  /*下刀方式*/
							    PNT3D   AtPoint      ,  /*刀具位置*/
							    CSmartLoop*  /*BndCont*/)  /*边界轮廓*/
{
	if( Plunge.m_nPlungeType != NCDEF_PLUNGE_VERT  )
	{ // 垂直下刀
		return FALSE ;
	}
	PNT3D dStart, dEnd ;
	dStart[0] = AtPoint[0]  , dStart[1] = AtPoint[1]  ;
	dEnd[0]   = AtPoint[0] , dEnd[1]    = AtPoint[1]  ;
	dStart[2] = Plunge.m_dIncStep , dEnd[2] = 0.0  ; 
	CPathCombine  tmpComb( NC_WPROCESS_PLUNGE ) ;
	CPathLine3D *pLine  = new CPathLine3D( dStart, dEnd ) ;
	tmpComb.AddEntity( pLine ) ;
	tmpComb.MoveCombine( 0.0, 0.0, AtPoint[2] ) ;
	JDNC_TOL  tmpTol ;
	SurfNC_InitPathParam( tmpTol ) ;
	tmpTol.m_dArcTol = 0.004 ;
	AllPath.CreateCutPath( tmpComb, tmpTol  ) ;
	return TRUE ; 
}
////////////////////////////////////////
// 在固定的位置生成下刀路径
int  MathCAM_AddPlungePathAtPoint( CSmtCPathLib& AllPath,  /*路径组  */
								    JDNC_PLUNGE& Plunge  ,  /*下刀方式*/
								    PNT3D   AtPoint      ,  /*刀具位置*/
									CSmartCurve* pCurve   , /*下刀曲线*/
								    CSmartLoop* BndCont  )  /*边界轮廓*/

{
	JDNC_PLUNGE tmpPlunge = Plunge ;
	tmpPlunge.m_dIncStep += tmpPlunge.m_dTopTol ;
	tmpPlunge.m_dTopTol = 0.0 ;
	if( tmpPlunge.m_dIncStep < 0.002 )
	{
		return TRUE ;
	}
	if( tmpPlunge.m_nPlungeType == NCDEF_PLUNGE_HELIX )
	{ /*螺旋下刀*/
		if( ! MathCAM_CreateHelixPlungeAt( AllPath, tmpPlunge, AtPoint, BndCont ) )
		{
			tmpPlunge.m_nPlungeType = NCDEF_PLUNGE_CONTOUR  ;
		}
    }
	
	if( tmpPlunge.m_nPlungeType == NCDEF_PLUNGE_CONTOUR )
	{ // 沿轮廓下刀
		if( ! MathCAM_CreateCurvePlungeAt( AllPath, tmpPlunge, AtPoint, pCurve, BndCont ) )
		{
			tmpPlunge.m_nPlungeType = NCDEF_PLUNGE_RAMP  ; 
		}
	}

	if( tmpPlunge.m_nPlungeType == NCDEF_PLUNGE_RAMP  )
	{ // 折线下刀
		if( ! MathCAM_CreateRampPlungeAt( AllPath, tmpPlunge, AtPoint, BndCont ) )
		{
			tmpPlunge.m_nPlungeType = NCDEF_PLUNGE_VERT  ;
		}
	}
	if( tmpPlunge.m_nPlungeType == NCDEF_PLUNGE_VERT  )
	{ // 垂直下刀
		MathCAM_CreateVertPlungeAt( AllPath, tmpPlunge, AtPoint, BndCont ) ;
	}
	return TRUE ;
}

void MathCAM_ClearLoopList(  CPtrList& AllEnt ) 
{
	POSITION pos = AllEnt.GetHeadPosition() ;
	while( pos )
	{
		CSmartLoop* pObj = (CSmartLoop*)AllEnt.GetNext( pos ) ;
		if( pObj ) delete pObj ;
	}
	AllEnt.RemoveAll() ;
}

void MathCAM_ClearCurveList( CPtrList& AllEnt) 
{
	POSITION pos = AllEnt.GetHeadPosition() ;
	while( pos )
	{
		CSmartCurve* pObj = (CSmartCurve*)AllEnt.GetNext( pos ) ;
		if( pObj ) delete pObj ;
	}
	AllEnt.RemoveAll() ;
}
void MathCAM_ClearGeoCurList( CPtrList& AllEnt )
{
	POSITION pos = AllEnt.GetHeadPosition() ;
	while( pos )
	{
		CGeoCurList* pObj = (CGeoCurList*)AllEnt.GetNext( pos ) ;
		if( pObj ) 
		{
			pObj->Destroy();
			delete pObj ;
		}
	}
	AllEnt.RemoveAll() ;
}
// 清除点
void MathCAM_ClearPointList( CPtrList& AllEnt) 
{
	POSITION pos = AllEnt.GetHeadPosition() ;
	while( pos )
	{
		CSmartPoint* pObj = (CSmartPoint*)AllEnt.GetNext( pos ) ;
		if( pObj ) delete pObj ;
	}
	AllEnt.RemoveAll() ;
}
void MathCAM_ClearFacetList( CPtrList& AllEnt) 
{
	POSITION pos = AllEnt.GetHeadPosition() ;
	while( pos )
	{
		FACET* pObj = (FACET*)AllEnt.GetNext( pos ) ;
		delete pObj ;
	}
	AllEnt.RemoveAll() ;
}

BOOL MathCAM_ConvertSFinishToAutoFinish ( JDNC_METHOD& OldMethod, JDNC_METHOD& NewMethod ) 
{
    if( OldMethod.m_cSFinishCut.m_nMoveType == surfncMoveFollow )
    {
        return FALSE ;
    }
	NewMethod.m_nMethodType = surfncMethodAutoFinish ;
	SurfNC_InitPathParam ( NewMethod.m_cAutoFinish ) ;
	NewMethod.m_cAutoFinish.m_nAreaType = NCDEF_AUTOAREA_ALL ;
	NewMethod.m_cAutoFinish.m_bFinishFlag |= NCDEF_FAUTOFINISH_LINEANGLE ;
	if ( OldMethod.m_cSFinishCut.m_bMoveFlag & NCDEF_SFINISH_DELFLATPNT )
		NewMethod.m_cAutoFinish.m_bFinishFlag |= NCDEF_FAUTOFINISH_DELFLATPNT ;

	if ( OldMethod.m_cSFinishCut.m_nMoveType == surfncMoveLinear )
	{
		NewMethod.m_cAutoFinish.m_dSlopeAngle = OldMethod.m_cSFinishCut.m_cLinearCut.m_cSlope.m_dSlopeAngle ;
		NewMethod.m_cAutoFinish.m_dOverlap = OldMethod.m_cSFinishCut.m_cLinearCut.m_cSlope.m_dExtLen ;
		if ( OldMethod.m_cSFinishCut.m_cLinearCut.m_cSlope.m_nAddType == NCDEF_ADDTYPE_XYALL )
		{
			NewMethod.m_cAutoFinish.m_nMove3DType = NCDEF_MAUTOMOVE3D_XPARALLEL ;
		}
		else
		{
			NewMethod.m_cAutoFinish.m_nMove3DType = NCDEF_MAUTOMOVE3D_PARALLEL ;
			switch ( OldMethod.m_cSFinishCut.m_cLinearCut.m_cSlope.m_nAddType )
			{
			case NCDEF_ADDTYPE_XFLAT :
				NewMethod.m_cAutoFinish.m_nAreaType = NCDEF_AUTOAREA_FLAT ;
				break ;
			case NCDEF_ADDTYPE_XNOPLANE :
				NewMethod.m_cAutoFinish.m_nAreaType = NCDEF_AUTOAREA_STEEP ;
				break ;
			}
		}
		
		NewMethod.m_cAutoFinish.m_dLineAngle = OldMethod.m_cSFinishCut.m_cLinearCut.m_dLineAngle ;
		if ( OldMethod.m_cSFinishCut.m_cLinearCut.m_bLineFlag & NCDEF_LINEAR_SIDETOL )
		{
			NewMethod.m_cAutoFinish.m_bFinishFlag |= NCDEF_FAUTOFINISH_SIDETOL ;
			NewMethod.m_cAutoFinish.m_dSideTol = OldMethod.m_cSFinishCut.m_cLinearCut.m_dSideTol ;
		}
	}
	else if ( OldMethod.m_cSFinishCut.m_nMoveType == surfncMoveHeight )
	{
		NewMethod.m_cAutoFinish.m_nMove3DType = NCDEF_MAUTOMOVE3D_HEIGHT ;
		if ( OldMethod.m_cSFinishCut.m_cHeightCut.m_bHeightFlag & NCDEF_HEIGHT_MOVEUP )
		{
			NewMethod.m_cAutoFinish.m_bFinishFlag |= NCDEF_FAUTOFINISH_MOVEUP ;
		}
		if ( OldMethod.m_cSFinishCut.m_cHeightCut.m_bHeightFlag & NCDEF_HEIGHT_DELFLAT )
		{
			NewMethod.m_cAutoFinish.m_nAreaType = NCDEF_AUTOAREA_STEEP ;
			NewMethod.m_cAutoFinish.m_dSlopeAngle = OldMethod.m_cSFinishCut.m_cHeightCut.m_dSlopeAngle ;
		}
		else
			NewMethod.m_cAutoFinish.m_nAreaType = NCDEF_AUTOAREA_ALL ;
	}
	else if ( OldMethod.m_cSFinishCut.m_nMoveType == surfncMoveEmiter )
	{
		NewMethod.m_cAutoFinish.m_nMove3DType = NCDEF_MAUTOMOVE3D_SEMIT ;
		NewMethod.m_cAutoFinish.m_dAngFrom = OldMethod.m_cSFinishCut.m_cEmiterCut.m_dAngFrom ;
		NewMethod.m_cAutoFinish.m_dAngRange = OldMethod.m_cSFinishCut.m_cEmiterCut.m_dAngRange ;
		if ( OldMethod.m_cSFinishCut.m_cEmiterCut.m_bEmitFlag & NCDEF_EMIT_TOOUTER )
		{
			NewMethod.m_cAutoFinish.m_bFinishFlag |= NCDEF_FAUTOFINISH_TOOUTER ;
		}
		if ( OldMethod.m_cSFinishCut.m_cEmiterCut.m_bEmitFlag & NCDEF_EMIT_SIDETOL )
		{
			NewMethod.m_cAutoFinish.m_bFinishFlag |= NCDEF_FAUTOFINISH_SIDETOL ;
			NewMethod.m_cAutoFinish.m_dSideTol = OldMethod.m_cSFinishCut.m_cEmiterCut.m_dSideTol ;
		}
	}
	else if ( OldMethod.m_cSFinishCut.m_nMoveType == surfncMoveUWLine )
	{
		NewMethod.m_cAutoFinish.m_nMove3DType = NCDEF_MAUTOMOVE3D_UWLINE ;
		if ( OldMethod.m_cSFinishCut.m_cUWLineCut.m_nCutDir == 1 )
		{
			NewMethod.m_cAutoFinish.m_bFinishFlag |= NCDEF_FAUTOFINISH_CUTWDIR ;
		}
	}
	else if ( OldMethod.m_cSFinishCut.m_nMoveType == surfncMoveFollow )
	{
		NewMethod.m_cAutoFinish.m_nMove3DType = NCDEF_MAUTOMOVE3D_SFOLLOW ;
		switch ( OldMethod.m_cSFinishCut.m_cFollowCut.m_nFollowType )
		{
		case NCDEF_FOLLOW_INNER :
			NewMethod.m_cAutoFinish.m_nFollowType = NCDEF_AUTOFOLLOW_INNER ;
			break ;
		case NCDEF_FOLLOW_MIDDLE :
			NewMethod.m_cAutoFinish.m_nFollowType = NCDEF_AUTOFOLLOW_MIDDLE ;
			break ;
		case NCDEF_FOLLOW_OUTER :
			NewMethod.m_cAutoFinish.m_nFollowType = NCDEF_AUTOFOLLOW_OUTER ;
			break ;
		case NCDEF_FOLLOW_START :
			NewMethod.m_cAutoFinish.m_nFollowType = NCDEF_AUTOFOLLOW_START ;
			break ;
		case NCDEF_FOLLOW_GUIDE :
			NewMethod.m_cAutoFinish.m_nFollowType = NCDEF_AUTOFOLLOW_GUIDE ;
			break ;
		}
		if ( OldMethod.m_cSFinishCut.m_cFollowCut.m_bFollowFlag & NCDEF_FOLLOW_TOOUTER )
		{
			NewMethod.m_cAutoFinish.m_bFinishFlag |= NCDEF_FAUTOFINISH_TOOUTER ;
		}
	}
	else if ( OldMethod.m_cSFinishCut.m_nMoveType == surfncMoveMixture )
	{
		NewMethod.m_cAutoFinish.m_nAreaType = NCDEF_AUTOAREA_MIXTURE ; 
		NewMethod.m_cAutoFinish.m_dSlopeAngle = OldMethod.m_cSFinishCut.m_cMixtureCut.m_cSlope.m_dSlopeAngle ;
		if ( OldMethod.m_cSFinishCut.m_cMixtureCut.m_bMixtureFlag & NCDEF_MIXTURE_FOLLOW )
		{
			NewMethod.m_cAutoFinish.m_nMove3DType = NCDEF_MAUTOMOVE3D_SFOLLOW ;
		}
		else
		{
			NewMethod.m_cAutoFinish.m_nMove3DType = NCDEF_MAUTOMOVE3D_PARALLEL ;
		}
		if ( OldMethod.m_cSFinishCut.m_cMixtureCut.m_bMixtureFlag & NCDEF_MIXTURE_PARFIRST )
		{
			NewMethod.m_cAutoFinish.m_bFinishFlag |= NCDEF_FAUTOFINISH_PARFIRST ;
		}
		NewMethod.m_cAutoFinish.m_dLineAngle = OldMethod.m_cSFinishCut.m_cMixtureCut.m_dLineAngle ;
		NewMethod.m_cAutoFinish.m_dOverlap = OldMethod.m_cSFinishCut.m_cMixtureCut.m_cSlope.m_dExtLen ;
	}
    return TRUE ;
}
void MathCAM_ConvertFPlaneToAutoFinish ( JDNC_METHOD& OldMethod, JDNC_METHOD& NewMethod) 
{
	NewMethod.m_nMethodType = surfncMethodAutoFinish ;
	SurfNC_InitPathParam ( NewMethod.m_cAutoFinish ) ;
	NewMethod.m_cAutoFinish.m_bFinishFlag |= NCDEF_FAUTOFINISH_LINEANGLE ;
    NewMethod.m_cAutoFinish.m_dSlopeAngle = OldMethod.m_cPlaneCut.m_dSlopeAngle ;
    if( OldMethod.m_cPlaneCut.m_bPlaneFlag & NCDEF_PLANE_FLATSURF )
    {/*平坦区域*/
        NewMethod.m_cAutoFinish.m_nAreaType = NCDEF_AUTOAREA_FLAT ;
    }
    else
    {/*平面区域*/
        NewMethod.m_cAutoFinish.m_nAreaType = NCDEF_AUTOAREA_PLANE ;
    }
    if( OldMethod.m_cPlaneCut.m_nMoveType == surfncMoveLinear )
	{
		NewMethod.m_cAutoFinish.m_nMove3DType = NCDEF_MAUTOMOVE3D_PARALLEL ;
        NewMethod.m_cAutoFinish.m_dLineAngle = OldMethod.m_cPlaneCut.m_cLinear.m_dLineAngle ;
        if( OldMethod.m_cPlaneCut.m_cLinear.m_bLineFlag & NCDEF_LINEAR_SIDETOL )
		{
			NewMethod.m_cAutoFinish.m_bFinishFlag |= NCDEF_FAUTOFINISH_SIDETOL ;
            NewMethod.m_cAutoFinish.m_dSideTol = OldMethod.m_cPlaneCut.m_cLinear.m_dSideTol ;
		}
	}
	else  if( OldMethod.m_cPlaneCut.m_nMoveType == surfncMoveFollow )
	{
		NewMethod.m_cAutoFinish.m_nMove3DType = NCDEF_MAUTOMOVE3D_SFOLLOW ;
        if( OldMethod.m_cPlaneCut.m_cFollow.m_bFollowFlag & NCDEF_FOLLOW_TOOUTER )
        {
            NewMethod.m_cAutoFinish.m_bFinishFlag |= NCDEF_FAUTOFINISH_TOOUTER ;
        }
	}
	else 
	{
		NewMethod.m_cAutoFinish.m_nMove3DType = NCDEF_MAUTOMOVE3D_SFOLLOW ;
        if( OldMethod.m_cPlaneCut.m_cSpiral.m_bSpiralFlag & NCDEF_SPIRAL_TOOUTER )
        {
            NewMethod.m_cAutoFinish.m_bFinishFlag |= NCDEF_FAUTOFINISH_TOOUTER ;
        }
        if( OldMethod.m_cPlaneCut.m_cSpiral.m_bSpiralFlag & NCDEF_SPIRAL_SIDETOL )
        {
			NewMethod.m_cAutoFinish.m_bFinishFlag |= NCDEF_FAUTOFINISH_SIDETOL ;
            NewMethod.m_cAutoFinish.m_dSideTol = OldMethod.m_cPlaneCut.m_cSpiral.m_dSideTol ;
        }
	}
}

CSmartPathGen* MathCAM_CreatePathGen( CJDParameter& Param ) 
{
    JDNC_METHOD& Method = Param.m_cMethodDef ;
	switch( Method.m_nMethodType )
	{
	case surfncMethodDrill :
		{
			CSmartDrillGen* pPathGen = new CSmartDrillGen() ;
			pPathGen->m_cParam = Method.m_cDrillCut ;
			pPathGen->m_cMethodDef = Method ;
			return pPathGen ;
		}
		break ;
	case surfncMethodBore :
		{
			CSmartBoreGen* pPathGen = new CSmartBoreGen() ;
			pPathGen->m_cParam = Method.m_cBoreCut ;
			pPathGen->m_cMethodDef = Method ;
			return pPathGen ;
		}
		break ;
	case surfncMethodCurve :
		{
			CSmartCurveGen* pPathGen = new CSmartCurveGen() ;
			pPathGen->m_cParam = Method.m_cCurveCut ;
			pPathGen->m_cMethodDef = Method ;
			return pPathGen ;
		}
		break ;
	case surfncMethodLoop  :
		{
			CSmartContourGen* pPathGen = new CSmartContourGen() ;
			pPathGen->m_cParam = Method.m_cLoopCut ;
			pPathGen->m_cMethodDef = Method ;
			return pPathGen ;
		}
		break ;
	case surfncMethodPocket :
		{
			CSmartPocketGen* pPathGen = new CSmartPocketGen() ;
			pPathGen->m_cParam = Method.m_cPocketCut ;
			pPathGen->m_cMethodDef = Method ;
			return pPathGen ;
		}
		break ;
	case surfncMethodBound :
		{
			CSmartBoundGen* pPathGen = new CSmartBoundGen() ;
			pPathGen->m_cParam = Method.m_cBoundCut ;
			pPathGen->m_cMethodDef = Method ;
			return pPathGen ;
		}
		break ;
	case surfncMethodCorner :
		{
			if( Method.m_cCornerCut.m_nCornerType == NCDEF_CORNER_EXPEND )
			{
				CSmartExpendGen* pPathGen = new CSmartExpendGen() ;
				pPathGen->m_cParam = Method.m_cCornerCut ;
				pPathGen->m_cMethodDef = Method ;
				return pPathGen ;
			}
			else
			{
				CSmartCornerGen* pPathGen = new CSmartCornerGen() ;
				pPathGen->m_cParam = Method.m_cCornerCut ;
				pPathGen->m_cMethodDef = Method ;
				return pPathGen ;
			}
		}
		break ;
	case surfncMethodRemain :
		{
			CSmartRemainGen* pPathGen = new CSmartRemainGen() ;
			pPathGen->m_cParam = Method.m_cRemainCut ;
			pPathGen->m_cMethodDef = Method ;
			return pPathGen ;
		}
		break ;
	case surfncMethodRPocket :
		{
			CSmartRPocketGen* pPathGen = new CSmartRPocketGen() ;
			pPathGen->m_cParam = Method.m_cSRegionCut ;
			pPathGen->m_cMethodDef = Method ;
			return pPathGen ;
		}
		break;
	case surfncMethodSRough :
		if( Method.m_cSRoughCut.m_nMoveType == surfncMoveLinear )
		{
			CSmartRLinearGen* pPathGen = new CSmartRLinearGen() ;
			pPathGen->m_cParam = Method.m_cSRoughCut.m_cLinearCut ;
			pPathGen->m_cMethodDef = Method ;
			return pPathGen ;
		}
		if( Method.m_cSRoughCut.m_nMoveType == surfncMoveUWLine )
		{
			CSmartRUWLineGen* pPathGen = new CSmartRUWLineGen() ;
			pPathGen->m_cParam = Method.m_cSRoughCut.m_cUWLineCut ;
			pPathGen->m_cMethodDef = Method ;
			return pPathGen ;
		}
		else if( Method.m_cSRoughCut.m_nMoveType == surfncMoveEmiter)
		{
			CSmartREmiterGen* pPathGen = new CSmartREmiterGen() ;
			pPathGen->m_cParam = Method.m_cSRoughCut.m_cEmiterCut ;
			pPathGen->m_cMethodDef = Method ;
			return pPathGen ;
		}
		break ;
	case surfncMethodSRemain :
		{
			CSmartSRemainGen* pPathGen = new CSmartSRemainGen() ;
			pPathGen->m_cParam = Method.m_cSRemainCut ;
			pPathGen->m_cMethodDef = Method ;
			return pPathGen ;
		}
		break ;
	case surfncMethodSFinish : // 曲面精雕刻
		if( Method.m_cSFinishCut.m_nMoveType == surfncMoveLinear )
		{
			CSmartFLinearGen* pPathGen = new CSmartFLinearGen() ;
			pPathGen->m_cParam = Method.m_cSFinishCut.m_cLinearCut ; 
			pPathGen->m_bMoveFlag=Method.m_cSFinishCut.m_bMoveFlag;
			pPathGen->m_cMethodDef = Method ;
			return pPathGen ;
		}
		else if( Method.m_cSFinishCut.m_nMoveType == surfncMoveHeight )
		{
			CSmartFHeightGen* pPathGen = new CSmartFHeightGen() ;
			pPathGen->m_cParam = Method.m_cSFinishCut.m_cHeightCut ; 
			pPathGen->m_bMoveFlag=Method.m_cSFinishCut.m_bMoveFlag;
			pPathGen->m_cMethodDef = Method ;
			return pPathGen ;
		}
		else if( Method.m_cSFinishCut.m_nMoveType == surfncMoveEmiter )
		{
			CSmartFEmiterGen* pPathGen = new CSmartFEmiterGen() ;
			pPathGen->m_cParam = Method.m_cSFinishCut.m_cEmiterCut ; 
			pPathGen->m_cMethodDef = Method ;
			pPathGen->m_bMoveFlag=Method.m_cSFinishCut.m_bMoveFlag;
			return pPathGen ;
		}
		else if( Method.m_cSFinishCut.m_nMoveType == surfncMoveUWLine )
		{
			CSmartFUWLineGen* pPathGen = new CSmartFUWLineGen() ;
			pPathGen->m_cParam = Method.m_cSFinishCut.m_cUWLineCut ; 
			pPathGen->m_cMethodDef = Method ;
			pPathGen->m_bMoveFlag=Method.m_cSFinishCut.m_bMoveFlag;
			return pPathGen ;
		}
		else if( Method.m_cSFinishCut.m_nMoveType == surfncMoveFollow )
		{
			CSmartFFollowGen* pPathGen = new CSmartFFollowGen() ;
			pPathGen->m_cParam = Method.m_cSFinishCut.m_cFollowCut ; 
			pPathGen->m_cMethodDef = Method ;
			pPathGen->m_bMoveFlag=Method.m_cSFinishCut.m_bMoveFlag;
			return pPathGen ;
		}
		else if( Method.m_cSFinishCut.m_nMoveType == surfncMoveMixture )
		{
			CSmartMixtureGen* pPathGen = new CSmartMixtureGen() ;
			pPathGen->m_cParam = Method.m_cSFinishCut.m_cMixtureCut; 
			pPathGen->m_bMoveFlag=Method.m_cSFinishCut.m_bMoveFlag;
			pPathGen->m_cMethodDef = Method ;
			return pPathGen ;
		}
		break ;
	case surfncMethodFPlane :
		{
            CSmartFPlaneGen* pPathGen = new CSmartFPlaneGen() ;
			pPathGen->m_cParam = Method.m_cPlaneCut ; 
			pPathGen->m_cMethodDef = Method ;
			return pPathGen ;
		}
		break ;
	case surfncMethodCleanup :
		{
            CSmartCleanupGen* pPathGen = new CSmartCleanupGen() ;
			pPathGen->m_cParam = Method.m_cCleanupCut ; 
			pPathGen->m_cMethodDef = Method ;
			return pPathGen ;
		}
		break ;
	case surfncMethodRotate3D:
		{
            CSmartRotate3DGen* pPathGen = new CSmartRotate3DGen() ;
			pPathGen->m_cParam = Method.m_cRotate3DCut ; 
			pPathGen->m_cMethodDef = Method ;
			return pPathGen ;
		}
		break ;
	case surfncMethodProject :
		{
            CSmartProjectGen* pPathGen = new CSmartProjectGen() ;
			pPathGen->m_cParam = Method.m_cProjectCut ; 
			pPathGen->m_cMethodDef = Method ;
			return pPathGen ;
		}
		break ;
	case surfncMethodWrap :
		{
            CSmartWrapGen* pPathGen = new CSmartWrapGen() ;
			pPathGen->m_cParam = Method.m_cWrapCut ; 
			pPathGen->m_cMethodDef = Method ;
			return pPathGen ;
		}
		break ;
	case surfncMethodGray :
		{
            CSmartGrayGen* pPathGen = new CSmartGrayGen() ;
			pPathGen->m_cParam = Method.m_cGrayCut ; 
			pPathGen->m_cMethodDef = Method ;
			return pPathGen ;
		}
		break ;
	case surfncMethodBRelief :
		{
            CSmartBReliefGen* pPathGen = new CSmartBReliefGen() ;
			pPathGen->m_cParam = Method.m_cBReliefCut ; 
			pPathGen->m_cMethodDef = Method ;
			return pPathGen ;
		}
		break ;
	case surfncMethodSGuide :
		{
            CSmartSGuideGen* pPathGen = new CSmartSGuideGen() ;
            pPathGen->m_cParam = Method.m_cSGuideCut ; 
            pPathGen->m_cMethodDef = Method ;
            return pPathGen ;
		}
		break ;
	case surfncMethodPlungeMill :
		{
            //CSmartPlungeMillGen* pPathGen = new CSmartPlungeMillGen() ;
            //pPathGen->m_cParam = Method.m_cPlungeMillCut ; 
            //pPathGen->m_cMethodDef = Method ;
            //return pPathGen ;
		}
		break ;
	case surfncMethodThreadMill : // 铣螺纹加工
		{
			CSmartThreadMillGen* pPathGen = new CSmartThreadMillGen() ;
			pPathGen->m_cParam = Method.m_cThreadMillCut ;
			pPathGen->m_cMethodDef = Method ;
			return pPathGen ;
		}
		break ;
	case surfncMethodCombMill : // 组合刀切割
		{
			CSmartCombMillGen* pPathGen = new CSmartCombMillGen() ;
			pPathGen->m_cParam = Method.m_cCombMillCut ;
			pPathGen->m_cMethodDef = Method ;
			return pPathGen ;
		}
		break ;
	case surfncMethodCurveSlot : // 曲线摆槽加工
		{
			CSmartCurveSlotGen* pPathGen = new CSmartCurveSlotGen() ;
            pPathGen->m_cParam = Method.m_cCurveSlot ;
			pPathGen->m_cMethodDef = Method ;
			return pPathGen ;
		}
		break ;
	case surfncMethodFeatureSlot : //特征槽加工
		{
			CSmartFeatureSlotGen* pPathGen = new CSmartFeatureSlotGen() ;
			pPathGen->m_cParam = Method.m_cFeatureSlotCut;
			pPathGen->m_cMethodDef = Method ;
			return pPathGen ;
		}
		break ;
	default :
		break ;
	}
	return NULL ;
}

void MathCAM_InitRegenHeader( REGEN_HEADER& cHeader )
{
	strcpy( cHeader.m_sFileType, "SMART_REGEN" ) ;
	cHeader.m_nFileVesion = SMART_REGEN_VERSION ;
	cHeader.m_nMethodType = surfncMethodRPocket ;
	cHeader.m_nNumLevel   = 0 ;
	for( int i = 0 ; i < 20 ; i ++ ) 
	{
		cHeader.m_nRegenLevel[i] = -1 ;
		cHeader.m_dwDataPos[i] = 0L   ;
	}
	cHeader.m_nVersionLocal = 0 ;
	cHeader.m_nExtData = 0 ;
}
BOOL MathCAM_ValidRegenHeader( REGEN_HEADER& cHeader )
{
	if( strcmp( cHeader.m_sFileType , "SMART_REGEN" ) != 0 || 
		( cHeader.m_nFileVesion != SMART_REGEN_VERSION ) || 
		cHeader.m_nNumLevel == 0 )
	{
		return FALSE ;
	}
	return TRUE ;
}

CSmartLoop* MathCAM_FindOutMostLoop( CSmartLoop* ContHead )
{
	PNT2D  ptMid    ;
	CSmartLoop* pFind = NULL, *pLoop ;
	double dArea = -1.0e12 ;
	for( pLoop = ContHead ; pLoop ; pLoop = pLoop->next )
	{ 
	    if( ! pLoop->m_pIsland ) continue ;
		if( ! pFind || dArea < pLoop->m_dArea ) 
		{
			pFind = pLoop , dArea = pLoop->m_dArea ;
		}
	}
	if( ! pFind ) return NULL ; 
	pFind->m_pCurve->GetPoint( 0.5, ptMid ) ;
	for( pLoop = ContHead ; pLoop ; pLoop = pLoop->next )
	{ 
	    if( pLoop == pFind  ) continue ;
	    if( pLoop->IsPointIn( ptMid ) == 0 )
		{ /*有环落在该环之外，所以它不是最外边的环*/
			return NULL ; 
		}
	}
	return pFind ;
}
BOOL MathCAM_IsOutMostLoop( CSmartLoop* ContHead, CSmartLoop* pFind, BOOL ValidIsl )
{
	PNT2D  ptMid    ;
	CSmartLoop *pLoop ;
	if( pFind == NULL || ValidIsl && (pFind->m_pIsland == NULL) )
	{
		return FALSE ;
	}
	pFind->m_pCurve->GetPoint( 0.5, ptMid ) ;
	for( pLoop = ContHead ; pLoop ; pLoop = pLoop->next )
	{ 
	    if( pLoop == pFind  ) continue ;
	    if( pLoop->IsPointIn( ptMid ) == 1 )
		{ /*它落在该该环之内，所以它不是最外边的环*/
			return FALSE ; 
		}
	}
	return TRUE ;
}
void MathCAM_ValidOutMostLoop( CSmartLoop* LpHead )
{
	CSmartLoop* pLoop ;
	for( pLoop = LpHead ; pLoop ; pLoop = pLoop->next )
	{
		if( ! pLoop->m_pCurve ) continue ;
		if( pLoop->m_pCurve->IsBlankCurve() )
		{
			pLoop->m_bUseFlag |= NC_LOOP_OUTER ;
			pLoop->m_pCurve->m_bUseFlag |= NC_CURVE_DISCARD ;
		}
		else
		{
			pLoop->m_bUseFlag &= ~NC_LOOP_OUTER ;
			pLoop->m_pCurve->m_bUseFlag &= ~NC_CURVE_DISCARD ;
		}
	} 
}
void MathCAM_AddPathGroup( CSmtCPathLib& AllPath  , 
						   CPathGroup&   OldPath  , 
						   JDNC_TOL&     Tol      , 
						   BOOL bReDepth            )
{
	CSmtCutPoint *pPoint  ;
	CPathEntity *pEntity ;
	CSmtCutPath* pPath ; 
	CPathCombine* pPathComb = OldPath.m_pHead ;
    DOUBLE dStart[4] , dEnd[4], dPoint[4] ;
	dStart[3] = dEnd[3] = dPoint[3] = 0.0 ;
	for( ; pPathComb ; pPathComb = pPathComb->next )
	{
		pPath = NULL  ;
		for( pEntity = pPathComb->m_pHead; pEntity ; pEntity = pEntity->next )
		{ 
			int nType = pEntity->GetType() ;
		    if( nType == NC_PATH_POINT3D )
			{
				pPath = new CSmtCutPath( MINI_DRILL_PATH );
				pPath->m_bFeedType = pEntity->m_bFeedType ;
				pPath->m_bMoveFlag = pEntity->m_bMoveFlag ;
		        AllPath.AddToTail( pPath ) ;
				CPathPoint3D* pDrill= ( CPathPoint3D*) pEntity ;
				pPoint = new CSmtCutPoint( pDrill->m_fPoint ) ;
				pPath->AddTail( pPoint  ) ;
				if( bReDepth )
				{
					for( int i = 0  ; i < pDrill->m_nIncTime ; i ++ ) 
					{
				       pPoint = new CSmtCutPoint( pDrill->m_fIncAt[i] ) ;
				       pPath->AddTail( pPoint ) ;
					} 
				}
				pPath = NULL  ; 
				continue ;
			}
			pEntity->GetEndPoint( 0 , dStart ) ;
			if( ! pPath || 	mathDist( dStart, dEnd ) > 1.0e-3 || 
				fabs( pEntity->m_fFeedRate - pPath->m_fFeedRate ) > 0.001 ||
				pEntity->m_bFeedType != pPath->m_bFeedType ||
				pEntity->m_bMoveFlag != pPath->m_bMoveFlag   )// mdf by zy 2013.5.7
			{				
				pPath  = new CSmtCutPath() ;
				pPath->m_bFeedType = pEntity->m_bFeedType ; 
				pPath->m_fFeedRate = pEntity->m_fFeedRate ;				
				pPath->m_bMoveFlag = pEntity->m_bMoveFlag;//mdf by zy 2013.5.7				
				pPath->AddPoint( dStart ) ;
			    AllPath.AddToTail( pPath ) ;
			}
			pEntity->GetEndPoint( 1 , dEnd ) ;
		    if( nType == NC_PATH_PLINE3D )
			{ // 3D 折线
			    CPathPLine3D *pPLine = ( CPathPLine3D*) pEntity ;
			    for( int i = 1 ; i <= pPLine->m_nCount  ; i ++ )
				{
			        pPoint = new CSmtCutPoint( pPLine->m_pTAPos[i] ) ;
				    pPath->AddTail( pPoint ) ;
				} 
			}
		    else if( nType == NC_PATH_LINE3D )
			{ 
				CPathLine3D * pLine3D = ( CPathLine3D*) pEntity ;
		        pPoint = new CSmtCutPoint( pLine3D->m_fEnd ) ;
			    pPath->AddTail( pPoint ) ;
			}
		    else if( nType == NC_PATH_ARC3D )
			{
				TPNT3D dBuff[500] ;
				CPathArc3D * pArc3D = ( CPathArc3D*) pEntity ;
				int n = pArc3D->Discrete( Tol.m_dArcTol,ANGLE_TO_RADIAN( Tol.m_dAngTol), dBuff, 500 ) ;
			    for( int i = 1 ; i <= n ; i ++ ) 
				{ 
		            pPoint = new CSmtCutPoint( dBuff[i] ) ;
				    pPath->AddTail( pPoint ) ;
				} 
			} 
		    else pPath = NULL ;
		}
	}
	POSITION atPos = AllPath.m_cAllPath.GetHeadPosition() ;
	while( atPos ) 
	{
		pPath = AllPath.m_cAllPath.GetNext( atPos ) ;
		if( bReDepth == FALSE )
		{
			for( pPoint = pPath->m_pHead ; pPoint ; pPoint = pPoint->next)
			{ 
				pPoint->m_fPoint[2] = 0.0 ;
			}
		}
	    pPath->DefineBox()  ;
//		pPath->InsertCPoint( Tol.m_dMaxStep ) ;
	}
	AllPath.DefineBox () ;
}

BOOL NcBnd_ExtractMeshBound ( CSmtMeshMdl	& SmtMeshMdl, CSmtCPathLib	& BoundPath ) ; /*边界路径*/  
BOOL MathCAM_GenPencilBound( CSmtMeshMdl &cMeshMdl,		 // 网格模型
							 CSmartLoop  *TrimCont,      // 修剪边界
							 CSmtCPathLib &BoundPath ) ; // 返回路径
BOOL NcBnd_GenLeftoverBound ( CSmtMeshMdl	& MeshMdl ,		/*网格模型*/
                         CSmtCPathLib	& BoundPath,	/*边界路径*/
                         JDNC_CLEANUP	& Cleanup ) ;	/*残料定义*/
BOOL NcBnd_ExtractMeshBoundEx ( CSmtMeshMdl	& SmtMeshMdl,	/*网格模型*/
								CSmtCPathLib	& TrimBound,	/*修剪轮廓*/  
								DOUBLE		TrimDist,		/*修剪长度*/  
								CSmtCPathLib	& BoundPath ) ;	/*边界路径*/


void (*mathcam_EditMeshMdlDebugInfo)( CSmtMeshMdl& Mdl ) = NULL ; 
void MathCAM_ResisterDebugEditor( void (*EditMeshMdlDebugInfo)( CSmtMeshMdl& Mdl ) ) 
{
    mathcam_EditMeshMdlDebugInfo = EditMeshMdlDebugInfo ;
}
int  MathCAM_RegisterMeshMdlFunc( CSmtMeshMdl& MeshMdl ) 
{
    MeshMdl.m_pExtractMeshBoundEx = NcBnd_ExtractMeshBoundEx ;
    MeshMdl.m_pExtractMeshBound = NcBnd_ExtractMeshBound ;
	MeshMdl.m_pGenPencilBound   = MathCAM_GenPencilBound ;
    MeshMdl.m_pGenLeftoverBound = NcBnd_GenLeftoverBound ;
#ifdef _DEBUG
    if( mathcam_EditMeshMdlDebugInfo )
    {
//ZBQ        (*mathcam_EditMeshMdlDebugInfo)( MeshMdl ) ;
    }
#endif 
    return  4 ; 
}

//加入机床检查模型
static MATHCAM_GETVIRTUALNCMDL mathcan_GetVNcMdl = NULL;
void MATHCAM_SETFP_GETVIRTUALNCMDL(MATHCAM_GETVIRTUALNCMDL GetVNcMdl)
{
	mathcan_GetVNcMdl = GetVNcMdl;
}

BOOL MATHCAM_GenVirtualNcMdl(CSmtVirtualNcMdl & VNcMdl)
{
	if(!mathcan_GetVNcMdl)
    {
		return FALSE;
    }
	return mathcan_GetVNcMdl(VNcMdl);
}

void MathCam_CalcMidPoint5Ax( CSmtCutPointEx& Start, CSmtCutPointEx& End,
                              TFLOAT t, int SurfPosType, 
                              CSmtCutPointEx& TNew )
{
    TNew.m_bType = 0 ;
    nc_GetPointAtLine( Start.m_fPoint, End.m_fPoint, t, TNew.m_fPoint, 4 ) ;
    nc_GetPointAtLine( Start.m_fSurfPos, End.m_fSurfPos, t, TNew.m_fSurfPos, 3 ) ;
    TFLOAT fAxis[3] ;
    double dVec[3], dRotAxis[3], dRotBase[3] = { 0.0, 0.0, 0.0 } ;
    double dLen= 1.0, dAngInc ;
    BOOL bDefLen = FALSE ;
    if( fabs(t - 0.5f) > 0.002f )
    {
        nc_VProduct( Start.m_fSurfNor, End.m_fSurfNor, fAxis ) ;
        nc_FloatToDouble( dRotAxis, fAxis, 3 ) ; 
        dLen = nc_VectorLen( fAxis,3 ) ;
        if( dLen > 0.0002 ) bDefLen = TRUE ;
    }
    if( bDefLen == TRUE )
    {/*旋转刀轴*/
        nc_FloatToDouble( dVec, Start.m_fSurfNor, 3 ) ;
        dAngInc = nc_ACos( nc_OProduct( Start.m_fSurfNor, End.m_fSurfNor, 3 )) ;
        for( int k = 0 ; k < 3 ; k ++ )
        {
            dRotAxis[k] = fAxis[k] / dLen ;
        }
        mathRotVec( dRotAxis, dRotBase, t * dAngInc , dVec, dVec ) ;
        nc_DoubleToFloat( TNew.m_fSurfNor, dVec, 3 ) ;
    }
    else
    {
        nc_GetPointAtLine( Start.m_fSurfNor, End.m_fSurfNor, t, TNew.m_fSurfNor, 3)  ;
        if( nc_Normalize( TNew.m_fSurfNor, 3 ) == 0 )
        {
            nc_VectorCopy( TNew.m_fSurfNor, Start.m_fSurfNor, 3 ) ;
        }
    }
    if( SurfPosType == 1 ) 
    {/*旋转切线*/
        bDefLen = FALSE ;
        if( fabs(t - 0.5f) > 0.002f )
        {
            nc_VProduct( Start.m_fSurfPos, End.m_fSurfPos, fAxis ) ;
            nc_FloatToDouble( dRotAxis, fAxis, 3 ) ; 
            dLen = nc_VectorLen( fAxis,3 ) ;
            if( dLen > 0.0002 ) bDefLen = TRUE ;
        }
        if( bDefLen == TRUE )
        {/*旋转切线*/
            nc_FloatToDouble( dVec, Start.m_fSurfPos, 3 ) ;
            dAngInc = nc_ACos( nc_OProduct( Start.m_fSurfPos, End.m_fSurfPos, 3 )) ;
            for( int k = 0 ; k < 3 ; k ++ )
            {
                dRotAxis[k] = fAxis[k] / dLen ;
            }
            mathRotVec( dRotAxis, dRotBase, t * dAngInc , dVec, dVec ) ;
            nc_DoubleToFloat( TNew.m_fSurfPos, dVec, 3 ) ;
        }
        else
        {
            nc_GetPointAtLine( Start.m_fSurfPos, End.m_fSurfPos, t, TNew.m_fSurfPos, 3)  ;
            if( nc_Normalize( TNew.m_fSurfPos, 3 ) == 0 )
            {
                nc_VectorCopy( TNew.m_fSurfPos, Start.m_fSurfPos, 3 ) ;
            }
        }
    }
}
static void Mill5AX_BuildNcMtx( RFRAME& NcMtx, FPNT3D BasePnt, FPNT3D Normal )
{
    nc_FloatToDouble( NcMtx.O,  BasePnt , 3 ) ;
    nc_FloatToDouble( NcMtx.Z,  Normal , 3  ) ;
    NcMtx.scale = 1.0 ;
    if( fabs(Normal[0]) < fabs(Normal[1]) )
    {
        NcMtx.X[0] = 1.0 ;
        NcMtx.X[1] = NcMtx.X[2] = 0.0 ;
    }
    else
    {
        NcMtx.X[1] = 1.0 ;
        NcMtx.X[0] = NcMtx.X[2] = 0.0 ;
    }
    nc_VProduct( NcMtx.Z, NcMtx.X, NcMtx.Y ) ;
    nc_Normalize( NcMtx.Y , 3 ) ;
    nc_VProduct( NcMtx.Y, NcMtx.Z, NcMtx.X ) ;
}
void Math5Ax_ProjectConncetPath5D( CSmtCheckMdl& DriveMdl,
								   CSmtCutPath& CutPath  ,
								   JDNC_TOL& Tol         ,
								   DOUBLE RapidHeight    ,
								   CNc5DAxisCtrl& AxisCtrl) 
{
    CSmtCutPointEx* pTPnt, *pTNext;
    double dMaxAng = ANGLE_TO_RADIAN(3.0 ) ;
    double dMaxStep= min( 0.2, Tol.m_dMaxStep ) ;
    // STEP 1 : 插入大角度
    for( pTPnt = (CSmtCutPointEx*)CutPath.m_pHead ; pTPnt && pTPnt->next ; pTPnt = pTNext )
    {
        pTNext = (CSmtCutPointEx*)pTPnt->next ;
        if( pTPnt == CutPath.m_pHead || 
            pTNext == CutPath.m_pTail  )
        {
            continue ;
        }
        double dAngInc = nc_ACos( nc_OProduct( pTPnt->m_fSurfNor, pTNext->m_fSurfNor, 3 ) ) ;
        if( dAngInc > MiniPai1_2 )
        {
            CSmtCutPointEx *midTPnt = new CSmtCutPointEx() ;
            pTPnt->CalcMidPoint( pTNext, 0.5f, midTPnt ) ;
            if( nc_VectorLen(midTPnt->m_fSurfNor, 3 ) < 0.1f )
            {
                RFRAME NcMtx ;
                Mill5AX_BuildNcMtx( NcMtx, pTPnt->m_fPoint, pTPnt->m_fSurfNor ) ;
                nc_DoubleToFloat( midTPnt->m_fSurfNor, NcMtx.X, 3 ) ;
            }
            CSmtCutPointEx dirA( *midTPnt), dirB( *midTPnt ) ;
            nc_VectorReverse( dirB.m_fSurfNor , 3 ) ;
            DriveMdl.Define5AXHeight( dirA, TRUE ) ;
            DriveMdl.Define5AXHeight( dirB, TRUE ) ;
            double dDistA = nc_Distance( midTPnt->m_fPoint, dirA.m_fPoint, 3 ) ;
            double dDistB = nc_Distance( midTPnt->m_fPoint, dirB.m_fPoint, 3 ) ;
            if( AxisCtrl.m_cToolAxis.m_nAxisFlag & NCDEF_AXIS_FLAG_LIMITANG )
            {
                double dAng1 = RADIAN_TO_ANGLE( nc_ACos( dirA.m_fSurfNor[2] ) );
                double dAng2 = RADIAN_TO_ANGLE( nc_ACos( dirB.m_fSurfNor[2] ) );
                if( dAng1 < AxisCtrl.m_cToolAxis.m_dElevationAng[0] ||
                    dAng1 > AxisCtrl.m_cToolAxis.m_dElevationAng[1]   )
                {
                    dDistA = dDistB + dDistA + 1.0;
                }
                if( dAng2 < AxisCtrl.m_cToolAxis.m_dElevationAng[0] ||
                    dAng2 > AxisCtrl.m_cToolAxis.m_dElevationAng[1]   )
                {
                    dDistB = dDistB + dDistA + 1.0;
                }
            }
            if( dDistA <= dDistB )
            {
                *midTPnt = dirA ;
            }
            else
            {
                *midTPnt = dirB ;
            }
            CutPath.InsertAfter( midTPnt, pTPnt ) ;
        }
    }
    // STEP 2 : 插入小角度
    for( pTPnt = (CSmtCutPointEx*)CutPath.m_pHead ; pTPnt && pTPnt->next ; pTPnt = pTNext )
    {
        pTNext = (CSmtCutPointEx*)pTPnt->next ;
        if( pTPnt == CutPath.m_pHead || 
            pTNext == CutPath.m_pTail  )
        {
            continue ;
        }
        double dAngInc = nc_ACos( nc_OProduct( pTPnt->m_fSurfNor, pTNext->m_fSurfNor, 3 ) ) ;
        double dDist = nc_Distance( pTPnt->m_fPoint, pTNext->m_fPoint, 3 ) ;
        if( dAngInc < dMaxAng && dDist < dMaxStep )
        {
            continue ;
        }
        int nCnt = int(max( dAngInc / dMaxAng , dDist / dMaxStep)) + 1 ;
        TFLOAT tInc = 1.0f / nCnt ;
        for( int k = 1 ; k < nCnt ; k ++ ) 
        {
            CSmtCutPointEx* pTNew = new CSmtCutPointEx() ;
            MathCam_CalcMidPoint5Ax( *pTPnt, *pTNext, (k * tInc),0, *pTNew ) ;
            DriveMdl.Define5AXHeight( *pTNew ) ;
            CutPath.InsertBefore( pTNew, pTNext ) ;
        }
    }
    TFLOAT fMaxMove = (TFLOAT)max( 0.2, RapidHeight) ;
    for( pTPnt = (CSmtCutPointEx*)CutPath.m_pHead ; pTPnt ; pTPnt = (CSmtCutPointEx*)pTPnt->next )
    {
        if( pTPnt == CutPath.m_pHead || pTPnt == CutPath.m_pTail  )
        {
            continue ;
        }
        pTPnt->m_fPoint[0] += pTPnt->m_fSurfNor[0] * fMaxMove ;
        pTPnt->m_fPoint[1] += pTPnt->m_fSurfNor[1] * fMaxMove ;
        pTPnt->m_fPoint[2] += pTPnt->m_fSurfNor[2] * fMaxMove ;
    }
}

int Math5Ax_QuickConnectPath5Ax( CSmtCheckMdl& DriveMdl,
                                 CSmtCPathLib& AllPath ,
                                 JDNC_TOL& Tol         ,
                                 DOUBLE RapidHeight    ,
                                 CNc5DAxisCtrl& AxisCtrl,
								 DOUBLE SafeHeight      )
{
    POSITION atPos, atCurr ;
    CSmtCutPath* pTPath ;
    double dMinCos = cos( ANGLE_TO_RADIAN( 2.0 ) ) ;
    atPos = AllPath.m_cAllPath.GetHeadPosition() ;
    while( atPos  )
    {/*连接中间路径*/
        atCurr = atPos ;
        pTPath = AllPath.m_cAllPath.GetNext( atPos ) ;
        if( atPos == NULL ) break ;
        CSmtCutPath* pTNext = AllPath.m_cAllPath.GetAt( atPos ) ;
        CSmtCutPointEx* pSt = (CSmtCutPointEx*)pTPath->m_pTail ;
        CSmtCutPointEx* pEd = (CSmtCutPointEx*)pTNext->m_pHead ;
        if( nc_Distance( pSt->m_fPoint, pEd->m_fPoint, 3 ) < 0.01 && 
            nc_OProduct( pSt->m_fSurfNor, pEd->m_fSurfNor, 3) > dMinCos )
        {
            nc_VectorCopy( pSt->m_fPoint, pEd->m_fPoint, 3 ) ;
            nc_VectorCopy( pSt->m_fSurfNor, pEd->m_fSurfNor, 3 ) ;
            continue ;
        }
        pTPath = new CSmtCutPath( MINI_MILL5AX_PATH ) ;
        pTPath->m_bFeedType = JDNC_FEEDTYPE_QUICK ;
        pTPath->AddTail( pSt->CopyMyself() ) ;
        CSmtCutPointEx* pTAt = (CSmtCutPointEx*)pSt->CopyMyself() ;
        DriveMdl.Define5AXHeight( *pTAt , TRUE ) ;
        pTPath->AddTail( pTAt ) ;
        pTAt = (CSmtCutPointEx*)pEd->CopyMyself() ;
        DriveMdl.Define5AXHeight( *pTAt , TRUE ) ;
        pTPath->AddTail( pTAt ) ;
        pTPath->AddTail( pEd->CopyMyself() ) ;
        Math5Ax_ProjectConncetPath5D( DriveMdl, *pTPath, Tol, RapidHeight,AxisCtrl ) ;
        AllPath.m_cAllPath.InsertAfter( atCurr, pTPath ) ;
    }
    if( AllPath.m_cAllPath.GetCount())
    {
		DriveMdl.UpdateBoundSurf() ;
		pTPath = AllPath.m_cAllPath.GetHead() ;
		if( pTPath->m_bFeedType != JDNC_FEEDTYPE_QUICK )
		{
			CSmtCutPointEx* atStart = (CSmtCutPointEx*) pTPath->m_pHead ;
			CSmtCutPointEx * pTNew = new CSmtCutPointEx( *atStart ) ;
			DriveMdl.Define5AXHeightOnBoundBox( *pTNew ) ;
			double dis = mathDistCutPoint((CSmtCutPoint*)atStart, (CSmtCutPoint*)pTNew);
			if (!(dis < 1.0e-4 && SafeHeight < 1.0e-4)) // 当dis和SafeHeight都为0时，起末点不生成快速定位路径 qqs 2014.02.27
			{
				if ( dis < SafeHeight)
				{
					VEC3D nor;
					for (int i = 0; i < 3; i++)
					{
						nor[i] = atStart->m_fSurfNor[i];
					}
					mathUniVec(nor, 1.0e-4);
					for (int i = 0; i < 3; i++)
					{
						pTNew->m_fPoint[i] = float(atStart->m_fPoint[i] + SafeHeight*nor[i]);
					}
				}

				pTPath = new CSmtCutPath(MINI_MILL5AX_PATH)  ;
				pTPath->m_bFeedType = JDNC_FEEDTYPE_QUICK  ;
				pTPath->AddTail( pTNew ) ;
				pTPath->AddTail( atStart->CopyMyself() ) ;
				AllPath.m_cAllPath.AddHead( pTPath ) ;
			}		
			else
			{
				delete pTNew;
				pTNew = NULL;
			}
		}
		pTPath = AllPath.m_cAllPath.GetTail() ;
		if( pTPath->m_bFeedType != JDNC_FEEDTYPE_QUICK )
		{
			CSmtCutPointEx* atEnd = (CSmtCutPointEx*) pTPath->m_pTail ;
			CSmtCutPointEx * pTNew = new CSmtCutPointEx( *atEnd ) ;
			DriveMdl.Define5AXHeightOnBoundBox( *pTNew ) ;
			DOUBLE dis = mathDistCutPoint((CSmtCutPoint*)atEnd, (CSmtCutPoint*)pTNew);
			if (!(dis < 1.0e-4 && SafeHeight < 1.0e-4))// 当dis和SafeHeight都为0时，起末点不生成快速定位路径 qqs 2014.02.27
			{
				if (dis < SafeHeight)
				{
					VEC3D nor;
					for (int i = 0; i < 3; i++)
					{
						nor[i] = atEnd->m_fSurfNor[i];
					}
					mathUniVec(nor, 1.0e-4);
					for (int i = 0; i < 3; i++)
					{
						pTNew->m_fPoint[i] = float(atEnd->m_fPoint[i] + SafeHeight*nor[i]);
					}
				}

				pTPath = new CSmtCutPath(MINI_MILL5AX_PATH)  ;
				pTPath->m_bFeedType = JDNC_FEEDTYPE_QUICK  ;
				pTPath->AddTail( atEnd->CopyMyself() ) ;
				pTPath->AddTail( pTNew ) ;
				AllPath.AddToTail( pTPath ) ;
			}		
			else
			{
				delete pTNew;
				pTNew = NULL;
			}
		}
    }
    return TRUE ;
}

// 进度条前进n次
void MathCAM_MovePrgStep(JDNC_PRGDEF &PrgDef, int n/* = 1*/)
{
	for (int i=0; i<n; i++)
	{
		PrgDef.m_dStepAt += PrgDef.m_dIncStep ;
		while( PrgDef.m_pPrgFunc && PrgDef.m_dStepAt >= PrgDef.m_dLimitAt )
		{
			PrgDef.m_pPrgFunc(1) ;
			PrgDef.m_dStepAt -= PrgDef.m_dLimitAt  ;
		}
	}
}
void MathCAM_TrimBox3DByContours( CSmartLoop* ContHead, float Box3D[2][3] )
{
    if( ! ContHead ) return ;
    PNT2D dBox[2] ;
    nc_VectorCopy( dBox[0], ContHead->m_dBox[0], 2 ) ;
    nc_VectorCopy( dBox[1], ContHead->m_dBox[1], 2 ) ;
    for( CSmartLoop* pCont = ContHead->next ; pCont ; pCont = pCont->next )
    {
        for( int k = 0 ; k< 2 ; k ++) 
        {
            dBox[0][k] = min( pCont->m_dBox[0][k], dBox[0][k] ) ;
            dBox[1][k] = max( pCont->m_dBox[1][k], dBox[1][k] ) ;
        }
    }
    for( int k = 0 ; k < 2 ; k ++) 
    {
        Box3D[0][k] = (float)max( dBox[0][k], Box3D[0][k] ) ;
        Box3D[1][k] = (float)min( dBox[1][k], Box3D[1][k] ) ;
    }
}
void MathCAM_TrimBox3DByContours( CSmartLoop* ContHead, PNT3D Box3D[2] )
{
    if( ! ContHead ) return ;
    PNT2D dBox[2] ;
    nc_VectorCopy( dBox[0], ContHead->m_dBox[0], 2 ) ;
    nc_VectorCopy( dBox[1], ContHead->m_dBox[1], 2 ) ;
    for( CSmartLoop* pCont = ContHead->next ; pCont ; pCont = pCont->next )
    {
        for( int k = 0 ; k< 2 ; k ++) 
        {
            dBox[0][k] = min( pCont->m_dBox[0][k], dBox[0][k] ) ;
            dBox[1][k] = max( pCont->m_dBox[1][k], dBox[1][k] ) ;
        }
    }
    for( int k = 0 ; k < 2 ; k ++) 
    {
        Box3D[0][k] = max( dBox[0][k], Box3D[0][k] ) ;
        Box3D[1][k] = min( dBox[1][k], Box3D[1][k] ) ;
    }
}

//////////////////////////////////////////////////////////////////////////
void MathCAM_ReportErrorGenPath(int nResID, int nType)
{
	LPSTR lpBuffer = glbf_GetPromptString();
	glbf_LoadSysString(nResID, lpBuffer, 100) ;
	MathCAM_ReportErrorGenPath(lpBuffer, nType);
}
void MathCAM_ReportErrorGenPath(CString strErr, int nType)
{
	if (SurfNC_IsBatchGenPathMode())
	{
		SurfNC_SetLastError(strErr, nType);
	}
	else
	{
		AfxMessageBox(strErr, nType);
	}
}

//////////////////////////////////////////////////////////////////////////
// 是否需要为多线程计算拷贝干涉检查模型
BOOL MathCAM_IsNeedMultiCopy(CSmtCheckMdl& DriveMdl, int nThreadNum)
{
	return Nc5D_IsNeedMultiCopy(DriveMdl, nThreadNum);	
}

// 是否支持双核多线程计算
BOOL MathCAM_IsSupportMultiThread()
{
	// 个人定制版，不支持多线程计算
	WORD wProductID = glbf_GetProductID() ;
	if ( wProductID == PRODUCT_ID_SURFMILL_PRO )
	{
		return FALSE ;
	}
	tagNCPathCFG *pNcCFG = glbf_GetNCPathCFG();
	if(pNcCFG && (pNcCFG->m_wFlag & NC_CFG_ENABLE_MULTITHREAD))
	{
		SYSTEM_INFO si;
		GetSystemInfo(&si);
		if(si.dwNumberOfProcessors >= 2)
		{
			return TRUE;
		}
	}
	return FALSE;
}

// 线程函数调用主体
BOOL MathCAM_ThreadMainFunc(AFX_THREADPROC func, LPVOID lpParam[], int nThreadNum)
{	
	return 	Nc5D_ThreadMainFunc(func, lpParam, nThreadNum);
}

// guomin 多线程运算线程函数
static UINT MathCAM_AddRelativeHSubProc(LPVOID lpParam)
{	
	if (lpParam == NULL)
	{
		return 0;
	}
	ADRH_DATA& cData = *(ADRH_DATA *)lpParam;	
	if( cData.pPathGen == NULL || 
		cData.TmpPath == NULL    || 
		cData.AllPath == NULL    || 
		cData.DriveMdl == NULL   ) 
	{
		return 0 ;
	}
	return cData.pPathGen->AddRelativeHSubProc(cData.nAtCore, cData.nCoreNum, *cData.TmpPath, 
		cData.AllPath, *cData.DriveMdl,cData.dTop,cData.bConnect,cData.dZ,cData.nCount);
}

void CSmartPathGen::AddRelativeH( CSmtCheckMdl *DriveMdl, CPathCombine *pComb,CPathEntity *pHead,CPathEntity *pNext,double dTop,BOOL bConnect,double *dZ, INT_PTR nCount)
{
	if( !pHead || !pNext )
		return ;

	CPathLine3D *pLine = NULL ;
	CPathPLine3D *pPLine = NULL ;
	PNT3D start, end, end1 ;
	double dist = -1., h = 0. ;
	BOOL bSuccess = FALSE ;

	pHead->GetEndPoint ( 1, start ) ;
	pNext->GetEndPoint ( 0, end ) ;

	dist = mathDist( start, end ) ;
	if( dist >= 0.0001 ) 
	{
		bSuccess = FALSE ;
		if( bConnect && dist < m_cFeedDef.m_cConnectDef.m_dMaxDist )
		{
			// 然后添加相对高度连刀
			bSuccess = AddConnectLine( DriveMdl, pComb, pHead, start, end ) ;
		}
		if( !bSuccess )
		{
			// 首先添加慢速下刀
			if( m_cSpeedDef.m_dPlungeDist > 0.01 )
			{
				mathCpyPnt( end, end1 ) ;
				end[2] += m_cSpeedDef.m_dPlungeDist ;
				pLine = new CPathLine3D( end, end1 ) ;
				pLine->m_bFeedType = JDNC_FEEDTYPE_PLUNGE ;
				pComb->InsertAfter ( pLine, pHead ) ;
			}
			// 然后添加相对高度连刀
			h = CheckRelativeQuickH( DriveMdl, start, end, dZ, nCount, dTop ) ;
			pPLine = CreateRelativeQuick( start, end, h ) ;
			if( pPLine )
				pComb->InsertAfter ( pPLine, pHead ) ;
		}
	}
}

BOOL CSmartPathGen::AddRelativeHSubProc(int nAtCore, int nCoreNum, CPathGroup &TmpPath, LPVOID* AllPath, 
	CSmtCheckMdl &DriveMdl,double dTop,BOOL bConnect,double *dZ, INT_PTR nCount )
{
	int nIndex = 0;
	CPathEntity *pHead = NULL, *pNext = NULL ;
	CPathCombine *pComb = TmpPath.m_pHead ;
	while( pComb )
	{
		if (nIndex++ % nCoreNum == nAtCore )
		{
			//处理当前的pathcombine
			pHead = pComb->m_pHead ;
			while( pHead )
			{
				pNext = pHead->next ;
				if( !pNext ) break ;
				AddRelativeH(&DriveMdl,pComb,pHead,pNext,dTop,bConnect,dZ,nCount) ;
				pHead = pNext ;
			}
			AllPath[nIndex-1] = pComb;
		}
		pComb = pComb->next ;
	}
	return TRUE;
}

void CSmartPathGen::AddPlungeAndRelativeHbyMultithread( CSmtCheckMdl *DriveMdl, CPathGroup *NewPath, BOOL bConnect, double *dZ /*= NULL*/, INT_PTR nCount /*= 0*/  )
{
	if( !DriveMdl || !NewPath || !NewPath->m_pHead ) return ;
	CPathCombine *PCombHead = NULL, *PCombNext = NULL ;
	int nPathComb = 0 ;
	PCombHead = NewPath->m_pHead ;
	while( PCombHead )
	{
		nPathComb ++ ;
		PCombHead = PCombHead->next ;
	}

	//计算结果
	CVecPointer TempPath;
	TempPath.resize(nPathComb + 5);
	ZeroMemory(&TempPath[0], sizeof(LPVOID) * (nPathComb + 5));

	FPNT3D minpt, maxpt ;
	DriveMdl->CalcFacetBox ( minpt, maxpt ) ;
	double dTop = maxpt[2] ;

	if( nPathComb > 10 )//超过30才使用多线程
	{
		BOOL bLocalCopy = FALSE ;	
		if(IsNeedMultiCopy(*DriveMdl))
		{
			bLocalCopy = TRUE ;
			DriveMdl->CreateMultiCopy(m_nCalcThreadNum - 1) ;
		}
		ADRH_DATA ThreadData[NC_CFG_CPU_MAXNUM];
		LPVOID lpParam[NC_CFG_CPU_MAXNUM] = {NULL};
		int nThreadNum = min(m_nCalcThreadNum, (int)nPathComb);
		int i=0, k=1;
		for (i=0; i<nThreadNum; i++)
		{
			ThreadData[i].DriveMdl = i == 0 ? DriveMdl : DriveMdl->GetMultiCopy(i-k);
			if (ThreadData[i].DriveMdl == NULL)
			{
				i--, k--, nThreadNum--;
				continue;
			}
			ThreadData[i].nAtCore = i;
			ThreadData[i].TmpPath = NewPath;
			ThreadData[i].AllPath = &TempPath[0];
			ThreadData[i].dTop = dTop ;
			ThreadData[i].dZ = dZ ;
			ThreadData[i].nCount = nCount ;
			ThreadData[i].bConnect = bConnect ;
			ThreadData[i].pPathGen = this;	
			lpParam[i] = &ThreadData[i];
		}
		for (i=0; i<nThreadNum; i++)
		{
			ThreadData[i].nCoreNum = nThreadNum;
		}
		// 调用多线程函数
		MathCAM_ThreadMainFunc(MathCAM_AddRelativeHSubProc, lpParam, nThreadNum);
	}
	else
	{
		AddRelativeHSubProc(0,1,*NewPath,&TempPath[0],*DriveMdl,dTop,bConnect,dZ,nCount) ;
	}	

	//NewPath销毁
	while( NewPath->m_pHead )
	{
		PCombHead = NewPath->m_pHead ;
		NewPath->RemoveCombine(PCombHead) ;
	}

	// 合并TempPath中的pathcombine,同时在pathcombine之间加上连刀
	PCombHead = NULL ;
	for( int i=0 ;i<nPathComb ; i++ )
	{
		if( !PCombHead )
			PCombHead = (CPathCombine *)TempPath[i] ;
		else
		{
			//PCombHead与TempPath[i]之间加连刀
			PCombNext = (CPathCombine *)TempPath[i] ;
			if( !PCombNext ) continue ;
			CPathEntity *pHead = PCombHead->m_pTail ;
			CPathEntity *pNext = PCombNext->m_pHead ;			
			AddRelativeH(DriveMdl,PCombHead,pHead,pNext,dTop,bConnect,dZ,nCount) ;
			PCombHead->AppendCombine(PCombNext) ;
		}
	}	
	// 只有一个CPathCombine 添加进刀
	AddHeadPlungePath( PCombHead ) ;
	NewPath->AddData(0.0,PCombHead) ;
}

BOOL CSmartPathGen::RedepthPathByParamT(	CPathGroup& NewPath,	/*雕刻路径*/ 
											CSmartGraphic& Graph,	/*图形函数*/   
											DOUBLE& dTop )			/*最大高度*/
{
	CPtrList    Curve3D ;
	Graph.GetAllEntity( m_cSetupDef, 
		Curve3D, 
		SMARTGRAPH_TYPE_CURVE2D ) ;
	if( Curve3D.GetCount() == 0 ) 
	{
		return TRUE ;
	}
	RFRAME LocFrame ;
	if( !Graph.GetTPlaneFrame( LocFrame ) )
	{
		mathInitRFrame( &LocFrame ) ;
	}

	if (m_c5DCtrlDef.m_cToolAxisApp.m_nFrameFlag != NCDEF_FRAME_DEFAULT)
	{
		mathMultRFrame( &LocFrame, &m_c5DCtrlDef.m_cToolAxisApp.m_dIndexFrame, &LocFrame ) ;
	}

	// 防止保留曲线高度时，进退刀路径或下刀路径高度错误，edit by liuxin, 2013.9.24
	for( CPathCombine* pComb = NewPath.m_pHead ; pComb ; pComb = pComb->next )
	{
		RedepthCPathCombineByT(Curve3D, pComb, LocFrame);
	}

	//	CModel *pModel = glbf_GetModel() ;
	//	pModel->BeginUndo ( "Test_AddCurve") ;
	BOX3D box ;
	POSITION pos, atpos = NULL ;
	pos = Curve3D.GetHeadPosition() ;
	while( pos )
	{
		atpos = pos ;
		CRedepthCurve *pCurve = ( CRedepthCurve *)Curve3D.GetNext ( pos ) ;
		if( pCurve->m_pCurve3D )
		{
			mathTransWorldBox3D( &m_dNcMtx, &pCurve->m_pCurve3D->m_dBox, &box ) ;
			if( box.max[2] > dTop )
				dTop = box.max[2] ;
		}
		//		CEttCurve *pEttCurve = new CEttCurve( pCurve->m_pCurve2D->CopyMyself () ) ;
		//		pModel->UD_AddEntity ( pEttCurve ) ;
		delete pCurve ;
		Curve3D.RemoveAt ( atpos ) ;
	}
	//	pModel->EndUndo () ;
	return TRUE ;
}

BOOL CSmartPathGen::RedepthCPathCombineByT (CPtrList & CurveList, CPathCombine * PComb, RFRAME & LocFrame)
{
	if (!PComb || CurveList.IsEmpty())
	{
		return FALSE;
	}

	double dLength = PComb->GetPathLen(), dDist = 0., t = 0., dTmpDist = 0. ;
	for( CPathEntity* pEnt = PComb->m_pHead ; pEnt ; pEnt = pEnt->next )
	{
		if( !pEnt ) continue ;

		switch( pEnt->GetType() )
		{
		case NC_PATH_LINE3D :
			{
				CPathLine3D *pLine = (CPathLine3D*) pEnt ;
				dTmpDist = pLine->GetLength() ;
				// 更新起点高度
				t = dDist / dLength ;
				pLine->m_fStart[2] += SnapNearestPointByT( CurveList,pLine->m_fStart, t, LocFrame ) ;
				// 更新末点高度
				dDist += dTmpDist ;
				t = dDist / dLength ;
				pLine->m_fEnd[2] += SnapNearestPointByT( CurveList,pLine->m_fEnd, t, LocFrame ) ;
			}
			break ;
		case NC_PATH_PLINE3D :
			{
				CPathPLine3D *pPLine = (CPathPLine3D*) pEnt ;
				for( int i = 0 ;  i < pPLine->m_nCount ; i ++ ) 
				{
					dTmpDist = mathDist( pPLine->m_pTAPos[i], pPLine->m_pTAPos[i+1]) ;
					if( i == 0 )
					{
						// 该段的起点
						t = dDist / dLength ;
						pPLine->m_pTAPos[i][2] += SnapNearestPointByT( CurveList,pPLine->m_pTAPos[i], t, LocFrame ) ;
					}
	
					// 该段的末点
					dDist += dTmpDist ;
					t = dDist / dLength ;
					pPLine->m_pTAPos[i+1][2] += SnapNearestPointByT( CurveList,pPLine->m_pTAPos[i+1], t, LocFrame ) ;
				}
			}
			break ;
		case NC_PATH_ARC3D :
			{
				TPNT3D dStart, dEnd ;
				CPathArc3D *pArc3D = (CPathArc3D*) pEnt ;
				pArc3D->GetEndPoint( 0, dStart ) ;
				pArc3D->GetEndPoint( 1, dEnd   ) ;
				dTmpDist = pArc3D->GetLength() ;
				// 圆弧起点
				t = dDist / dLength ;
				dStart[2] += SnapNearestPointByT( CurveList,dStart, t, LocFrame ) ;
				
				// 圆弧末点
				dDist += dTmpDist ;
				t = dDist / dLength ;
				dEnd[2]   += SnapNearestPointByT( CurveList,dEnd  , t, LocFrame ) ;
				pArc3D->m_fCenter[2] = dStart[2] ;
				pArc3D->m_fDepth = dEnd[2] - dStart[2] ;
			}
			break ;
		default :
			ASSERT( 0 ) ;
			break ;
		}
	}

	return TRUE;
}

// 计算高度
DOUBLE CSmartPathGen::SnapNearestPointByT(	CPtrList &CurveList,	//<I> CRedepthCurve 
											PNT3D point		,		//<I> point
											double	&i_t		,	//<I> 输入初始参数t
											RFRAME &LocFrame )		//<I> 相对坐标系
{
	DOUBLE dDist = 0. ;
	DOUBLE dHeight = 0.0 , dMinDist = 1.0e6, t = i_t ;
	PNT3D o_p, near_p ;
	BOOL bFind = FALSE ;
	

	CRedepthCurve *pDepCurve = NULL ;
	CGeoCurve *pCurve3D = NULL, *pCurve2D = NULL ;
	POSITION pos = CurveList.GetHeadPosition() ;
	while( pos )
	{
		pDepCurve = ( CRedepthCurve *) CurveList.GetNext( pos ) ;
		if( ! pDepCurve ) continue ;
		pCurve3D = pDepCurve->m_pCurve3D ;
		pCurve2D = pDepCurve->m_pCurve2D ;
		
		pCurve3D->GetPoint( &LocFrame, t, near_p ) ;
		dDist = mathDist( near_p, point ) ;
		if( dDist < dMinDist )
		{
			dMinDist = dDist ;
			mathCpyPnt( near_p , o_p ) ;
			bFind = TRUE ;
		}

	}
	if( bFind )
	{
		dHeight = o_p[2] ;
	}
	return dHeight ;
}

void ConvertNurbsArcToPLine3D(CPathCombine* PComb, JDNC_TOL &Tol)
{
	if (!PComb || !PComb->m_pHead)
		return ;

	CPathEntity *pEnt = NULL, *pPrev = NULL, *pNext = NULL ;
	int nCnt = 0, nSize = 2000, i = 0 ;
	TPNT3D Buff[2000] ;
	double dArcTol = Tol.m_dArcTol ;
	double dAngTol = ANGLE_TO_RADIAN(Tol.m_dAngTol);
	for (pEnt = PComb->m_pHead; pEnt;)
	{
		pPrev = pEnt ;
		pNext = pEnt->next ;
		if (pEnt->GetType() == NC_PATH_NURBS3D )
		{
			CPathNurbs3D *pNbsEnt = ( CPathNurbs3D *)pEnt ;			
			if(pNbsEnt->m_nOrder>0 && pNbsEnt->m_nCount>0 && pNbsEnt->m_fPoint && pNbsEnt->m_fKnot)
			{
				CGCodeNurbs *pNurbs = new CGCodeNurbs() ;
				pNurbs->CreateNurbs( pNbsEnt->m_nOrder, pNbsEnt->m_nCount ) ;  
				memcpy(pNurbs->m_fPoint,pNbsEnt->m_fPoint, (pNurbs->m_nCount + 1 ) * sizeof(double[4]) );
				memcpy(pNurbs->m_fKnot, pNbsEnt->m_fKnot, (pNurbs->m_nCount + pNurbs->m_nOrder + 1) * sizeof(double) );
				nCnt = 0;
				pNurbs->ConvertToLine3D(Buff, nSize, nCnt, dArcTol);
				delete pNurbs;
				pNurbs = NULL;
				if( nCnt > 1 ) 
				{
					CPathPLine3D* pNewPLine3D = new CPathPLine3D;
					pNewPLine3D->m_nCount = nCnt;
					pNewPLine3D->m_pTAPos = new TPNT3D[nCnt + 1];
					for( i = 0 ; i <= nCnt ; i++ )
					{
						mathCpyPnt(Buff[i], pNewPLine3D->m_pTAPos[i]);
					}
					pNewPLine3D->m_bFeedType = pEnt->m_bFeedType ;
					pNewPLine3D->m_bMoveFlag = pEnt->m_bMoveFlag ;
					PComb->InsertAfter( pNewPLine3D, pPrev ) ;
					pPrev = pNewPLine3D ;
					PComb->RemoveEntity ( pEnt ) ;
					delete pEnt ;
				}
			}
		}
		else if (pEnt->GetType() == NC_PATH_ARC3D)
		{
			CPathArc3D* pArc3D = (CPathArc3D*)pEnt;
			nCnt = pArc3D->Discrete(dArcTol, dAngTol, Buff, nSize);
			if( nCnt > 1 ) 
			{
				CPathPLine3D* pNewPLine3D = new CPathPLine3D;
				pNewPLine3D->m_nCount = nCnt;
				pNewPLine3D->m_pTAPos = new TPNT3D[nCnt + 1];
				for( i = 0 ; i <= nCnt ; i++ )
				{
					mathCpyPnt(Buff[i], pNewPLine3D->m_pTAPos[i]);
				}
				pNewPLine3D->m_bFeedType = pEnt->m_bFeedType ;
				pNewPLine3D->m_bMoveFlag = pEnt->m_bMoveFlag ;
				PComb->InsertAfter( pNewPLine3D, pPrev ) ;
				pPrev = pNewPLine3D ;
				PComb->RemoveEntity ( pEnt ) ;
				delete pEnt ;
			}
		}

		pEnt = pNext;
	}
}

BOOL GetPathEntEndTan(CPathEntity* Ent, int Type, VEC3D VTan, double & Length)
{
	if (!Ent)
		return FALSE;
	if (Ent->GetType() == NC_PATH_PLINE3D)
	{
		CPathPLine3D* pPLine3D = (CPathPLine3D*)Ent;
		if (Type == 0)
		{
			mathGetVec(pPLine3D->m_pTAPos[0], pPLine3D->m_pTAPos[1], VTan);		
		}
		else
		{
			mathGetVec(pPLine3D->m_pTAPos[pPLine3D->m_nCount - 1], pPLine3D->m_pTAPos[pPLine3D->m_nCount], VTan);
		}
	}
	else if (Ent->GetType() == NC_PATH_LINE3D)
	{
		CPathLine3D* pLine3D = (CPathLine3D*)Ent;
		mathGetVec(pLine3D->m_fStart, pLine3D->m_fEnd, VTan);
	}
	else
	{
		return FALSE;
	}

	Length = mathVecLen(VTan);
	if (Length < MIN_LEN)
	{
		return FALSE;
	}
	for (int i = 0; i < 3; ++i)
	{
		VTan[i] /= Length;
	}

	return TRUE;
}

/*转成刮铣路径*/
BOOL CSmartPathGen::ConvertShavePath(CPathGroup& NewPath, JDNC_TOL & Tol)
{
	if (glbf_GetProductID() == PRODUCT_ID_SURFMILL_PRO)
	{// 个人版软件不支持磨削调整功能，2014.10.15 liuxin
		return FALSE;
	}
	if (!NewPath.m_pHead)
		return FALSE;

	if ((m_cMethodDef.m_nMethodType == surfncMethodCurve && m_cMethodDef.m_cCurveCut.m_bCurveFlag & NCDEF_FCURVE_SHAVE) ||
		(m_cMethodDef.m_nMethodType == surfncMethodLoop && m_cMethodDef.m_cLoopCut.m_bLoopFlag & NCDEF_FLOOP_SHAVE))
	{
		CPathCombine* pComb = NULL;
		for (pComb = NewPath.m_pHead; pComb; pComb = pComb->next)
		{
			ConvertShavePath(pComb, Tol);
		}
		// 路径类型改为多轴模式
		NewPath.m_nType = NC_PATHTYPE_ENGRAVE5AX;

		return TRUE;
	}

	return FALSE;
}

BOOL CSmartPathGen::ConvertShavePath(CPathCombine* PComb, JDNC_TOL & Tol)
{
	if (!PComb || !PComb->m_pHead)
		return FALSE;

	// 预先删除重点，去除长度为0的线段
	PComb->DelOverlapPoint();

	// 圆弧、样条路径转成折线路径
	ConvertNurbsArcToPLine3D(PComb, Tol);

	int i = 0, j = 0;
	double dLen[2], dScale = 1.0, dDist = 0.0;
	PNT3D ptStart, ptEnd;
	VEC3D vTan[2], vMove;
	CPathEntity* pEnt = NULL, *pPrevEnt = NULL;
	for (pEnt = PComb->m_pHead; pEnt; pEnt = pEnt->next)
	{
		if (pEnt->m_bFeedType != JDNC_FEEDTYPE_ROUGH)
			continue;
		pPrevEnt = pEnt->prev;
		if (pPrevEnt)
		{
			pEnt->GetEndPoint(0, ptStart);
			pPrevEnt->GetEndPoint(1, ptEnd);
			dDist = mathDist(ptStart, ptEnd);
		}
		else
		{
			dDist = 1.0;
		}
		if (pEnt->GetType() == NC_PATH_LINE3D ||
			pEnt->GetType() == NC_PATH_PLINE3D)
		{
			if (pPrevEnt && pPrevEnt->m_bFeedType == JDNC_FEEDTYPE_ROUGH && dDist < MIN_DIS)
			{// 计算前一段的切向
				if (!GetPathEntEndTan(pPrevEnt, 1, vTan[1], dLen[1]))
					continue;
				if (!GetPathEntEndTan(pEnt, 0, vTan[0], dLen[0]))
					continue;
				if (dLen[0] < MIN_LEN || dLen[1] < MIN_LEN)
				{
					continue;
				}
				dScale = dLen[0]/dLen[1];
				if (dScale > 0.5 && dScale < 2.0)
				{
					for (i = 0; i < 3; ++i)
					{
						vMove[i] = vTan[0][i] + vTan[1][i];
					}
					mathUniVec(vMove, MIN_LEN);
				}
				else if (dLen[0] > dLen[1])
				{
					mathCpyPnt(vTan[0], vMove);
				}
				else
				{
					mathCpyPnt(vTan[1], vMove);
				}
			}
			else
			{
				if (!GetPathEntEndTan(pEnt, 0, vMove, dLen[0]))
					continue;
			}
			if (pEnt->GetType() == NC_PATH_LINE3D)
			{
				if (!pEnt->m_pTDir)
				{
					pEnt->m_pTDir = new CNcToolDir;					
					if (!pEnt->m_pTDir->m_pTDir)
					{
						pEnt->m_pTDir->m_nNumDir = 2;
						pEnt->m_pTDir->m_pTDir = new TPNT3D[2];
					}					
				}
				mathCpyPnt(vMove, pEnt->m_pTDir->m_pTDir[0]);
				if (pPrevEnt && pPrevEnt->m_bFeedType == JDNC_FEEDTYPE_ROUGH && dDist < MIN_DIS &&
					pPrevEnt->m_pTDir && pPrevEnt->m_pTDir->m_pTDir)
				{
					mathCpyPnt(vMove, pPrevEnt->m_pTDir->m_pTDir[pPrevEnt->m_pTDir->m_nNumDir-1]);
				}
				if (pEnt->next == NULL || pEnt->next->m_bFeedType != JDNC_FEEDTYPE_ROUGH)
				{// 处理最后一段末点
					mathCpyPnt(vMove, pEnt->m_pTDir->m_pTDir[pEnt->m_pTDir->m_nNumDir - 1]);
				}
			}
			else if (pEnt->GetType() == NC_PATH_PLINE3D)
			{
				CPathPLine3D* pPLine3D = (CPathPLine3D*)pEnt;
				if (!pEnt->m_pTDir)
				{
					pEnt->m_pTDir = new CNcToolDir;					
					if (!pEnt->m_pTDir->m_pTDir)
					{
						pEnt->m_pTDir->m_nNumDir = pPLine3D->m_nCount + 1;
						pEnt->m_pTDir->m_pTDir = new TPNT3D[pEnt->m_pTDir->m_nNumDir];
					}					
				}
				mathCpyPnt(vMove, pEnt->m_pTDir->m_pTDir[0]);
				if (pPrevEnt && pPrevEnt->m_bFeedType == JDNC_FEEDTYPE_ROUGH && dDist < MIN_DIS &&
					pPrevEnt->m_pTDir && pPrevEnt->m_pTDir->m_pTDir)
				{
					mathCpyPnt(vMove, pPrevEnt->m_pTDir->m_pTDir[pPrevEnt->m_pTDir->m_nNumDir-1]);
				}
				// 处理折线内部方向
				mathCpyPnt(vMove, vTan[0]);
				for (j = 1; j < pPLine3D->m_nCount; ++j)
				{
					mathGetVec(pPLine3D->m_pTAPos[j], pPLine3D->m_pTAPos[j+1], vTan[1]);
					dLen[1] = mathVecLen(vTan[1]);
					if (dLen[1] < MIN_LEN)
					{
						mathCpyPnt(pPLine3D->m_pTDir->m_pTDir[j-1], pPLine3D->m_pTDir->m_pTDir[j]);
					}
					else
					{
						for (i = 0; i < 3; ++i)
						{
							vTan[1][i] /= dLen[1];
						}
						dScale = dLen[0]/dLen[1];
						if (dScale > 0.5 && dScale < 2.0)
						{
							for (i = 0; i < 3; ++i)
							{
								vMove[i] = vTan[0][i] + vTan[1][i];
							}
							mathUniVec(vMove, MIN_LEN);
						}
						else if (dLen[0] > dLen[1])
						{
							mathCpyPnt(vTan[0], vMove);
						}
						else
						{
							mathCpyPnt(vTan[1], vMove);
						}
						mathCpyPnt(vMove, pPLine3D->m_pTDir->m_pTDir[j]);
						mathCpyPnt(vTan[1], vTan[0]);
						dLen[0] = dLen[1];
					}
				}
				if (pEnt->next == NULL || pEnt->next->m_bFeedType != JDNC_FEEDTYPE_ROUGH)
				{// 处理最后一段末点
					mathCpyPnt(vMove, pEnt->m_pTDir->m_pTDir[pEnt->m_pTDir->m_nNumDir - 1]);
				}
			}
		}
	}

	// 处理非切削路径
	TreatNonCutShavePath(PComb);

	return TRUE;
}

BOOL CSmartPathGen::TreatNonCutShavePath(CPathCombine* PComb)
{
	double dDist = 0.0;
	PNT3D ptEnd[2];
	VEC3D vTan[2];
	CPathEntity* pEnt = NULL, *pPrevEnt = NULL, *pNextEnt = NULL;
	for (pEnt = PComb->m_pHead; pEnt; pEnt = pEnt->next)
	{
		if (!pEnt || pEnt->m_bFeedType != JDNC_FEEDTYPE_ROUGH || !pEnt->m_pTDir
			|| !pEnt->m_pTDir->m_pTDir)
		{
			continue;
		}
		if (pEnt->prev && pEnt->prev->m_bFeedType != JDNC_FEEDTYPE_ROUGH)
		{
			nc_VectorCopy(vTan[0], pEnt->m_pTDir->m_pTDir[0], 3);
			pEnt->GetEndPoint(0, ptEnd[0]);
			pPrevEnt = pEnt->prev;
			pPrevEnt->GetEndPoint(1, ptEnd[1]);
			dDist = mathDist(ptEnd[0], ptEnd[1]);
			while(pPrevEnt &&  dDist < MIN_DIS)
			{
				SetShavePathEntDir(pPrevEnt, vTan[0]);
				pPrevEnt->GetEndPoint(0, ptEnd[0]);
				if (pPrevEnt->prev)
				{
					pPrevEnt->prev->GetEndPoint(1, ptEnd[1]);
				}
				else
				{
					break;
				}
				dDist = mathDist(ptEnd[0], ptEnd[1]);
				if ((pPrevEnt->m_bFeedType == JDNC_FEEDTYPE_QUICK || pPrevEnt->m_bFeedType == JDNC_FEEDTYPE_RAPIDCNT ||
					pPrevEnt->m_bFeedType == JDNC_FEEDTYPE_CONNECT)&&
					(pPrevEnt->prev->m_bFeedType != JDNC_FEEDTYPE_QUICK && pPrevEnt->prev->m_bFeedType != JDNC_FEEDTYPE_RAPIDCNT &&
					pPrevEnt->prev->m_bFeedType != JDNC_FEEDTYPE_CONNECT)
					&& dDist < MIN_DIS &&
					pPrevEnt->prev->m_pTDir && pPrevEnt->prev->m_pTDir->m_pTDir)
				{
					nc_VectorCopy(vTan[1], pPrevEnt->prev->m_pTDir->m_pTDir[pPrevEnt->prev->m_pTDir->m_nNumDir - 1], 3);
					nc_VectorCopy(pPrevEnt->m_pTDir->m_pTDir[0], vTan[1], 3);
					if (pPrevEnt->m_pTDir->m_nNumDir > 2)
					{
						nc_VectorCopy(pPrevEnt->m_pTDir->m_pTDir[1], vTan[1], 3);
					}
					break;
				}
				pPrevEnt = pPrevEnt->prev;
			}
		}
		if (pEnt->next && pEnt->next->m_bFeedType != JDNC_FEEDTYPE_ROUGH)
		{
			nc_VectorCopy(vTan[1], pEnt->m_pTDir->m_pTDir[pEnt->m_pTDir->m_nNumDir - 1], 3);
			pEnt->GetEndPoint(1, ptEnd[1]);
			pNextEnt = pEnt->next;
			pNextEnt->GetEndPoint(0, ptEnd[0]);
			dDist = mathDist(ptEnd[0], ptEnd[1]);
			while (pNextEnt && dDist < MIN_DIS)
			{
				SetShavePathEntDir(pNextEnt, vTan[1]);
				pNextEnt->GetEndPoint(1, ptEnd[1]);
				if (pNextEnt->next)
				{
					pNextEnt->next->GetEndPoint(0, ptEnd[0]);
				}
				else
				{
					break;
				}
				dDist = mathDist(ptEnd[0], ptEnd[1]);
				if (pNextEnt->m_bFeedType == JDNC_FEEDTYPE_QUICK ||
					pNextEnt->m_bFeedType == JDNC_FEEDTYPE_RAPIDCNT || 
					pNextEnt->m_bFeedType == JDNC_FEEDTYPE_CONNECT) 
				{
					break;
				}
				pNextEnt = pNextEnt->next;				
			}
		}
	}
	return TRUE;
}

void CSmartPathGen::SetShavePathEntDir(CPathEntity* Ent, VEC3D Dir)
{
	if (!Ent)
	{
		return;
	}

	switch (Ent->GetType())
	{
	case NC_PATH_LINE3D:
	case NC_PATH_ARC3D:
		{
			if (!Ent->m_pTDir)
			{
				Ent->m_pTDir = new CNcToolDir;
				if (!Ent->m_pTDir->m_pTDir)
				{
					Ent->m_pTDir->m_nNumDir = 2;
					Ent->m_pTDir->m_pTDir = new TPNT3D[2];
				}
			}
			nc_VectorCopy(Ent->m_pTDir->m_pTDir[0], Dir, 3);
			nc_VectorCopy(Ent->m_pTDir->m_pTDir[1], Dir, 3);
		}
		break;
	case NC_PATH_PLINE3D:
		{
			CPathPLine3D* pPLine3D = (CPathPLine3D*)Ent;
			if (!pPLine3D->m_pTDir)
			{
				pPLine3D->m_pTDir = new CNcToolDir;
				if (!pPLine3D->m_pTDir->m_pTDir)
				{
					pPLine3D->m_pTDir->m_nNumDir = pPLine3D->m_nCount + 1;
					pPLine3D->m_pTDir->m_pTDir = new TPNT3D[pPLine3D->m_pTDir->m_nNumDir];
				}
			}
			for (int i = 0; i < pPLine3D->m_pTDir->m_nNumDir; ++i)
			{
				nc_VectorCopy(pPLine3D->m_pTDir->m_pTDir[i], Dir, 3);
			}
		}
		break;
	default:
		break;
	}
}

CPathEntity * MathCam_TransfSmtCurveToPathEntity( CSmartCurve &Curve, double dZ )
{
	if( !Curve.m_pHead ) return NULL ;
	CPathEntity *pHead = NULL ;

	int nCnt = 0, i = 0 ;
	PNT2D start, end ;
	CSmartSect *pSect = Curve.m_pHead, *pNext = NULL, *pEnt = NULL ;
	while( pSect )
	{
		pNext = pSect->next ;
		if( pSect->IsSectArc() )
		{
			// 圆弧逼近方式
			CPathArc3D* pArc3D = new CPathArc3D() ;
			pArc3D->Create( dZ, *(CSmartArc*) pSect ) ;	
			if( pHead )
			{
				pHead->next = pArc3D ;
				pArc3D->prev = pHead ;
			}
			else
			{
				pHead = pArc3D ;
			}
			goto EndLine ;
		}
		
		// 将多条SectLine组成PLINE
		pEnt = pSect ;
		nCnt = 1 ;
		pEnt->GetPoint( 1, start ) ;
		while( pEnt && pEnt->IsSectLine() )
		{
			if( nCnt > 1 )
			{
				pEnt->GetPoint ( 0., end ) ;
				if( mathDist2D( start, end ) > 0.0001 )
				{
					break ;
				}
			}
			pEnt->GetPoint ( 1., start ) ;
			nCnt++ ;
			pEnt = pEnt->next ; 
			pNext = pEnt ;
		}

		// 组成长折线
		CPathPLine3D * pLine = new CPathPLine3D() ;
		pLine->m_pTAPos = new TPNT3D[ nCnt ] ;
		pLine->m_nCount = nCnt - 1 ;
		pEnt = pSect ;
		for( i = 1 ; i < nCnt ; i++ ) 
		{
			if( i == 1 )
			{
				pEnt->GetPoint( 0., start ) ;
				pLine->m_pTAPos[i-1][0] = start[0] ;
				pLine->m_pTAPos[i-1][1] = start[1] ;
				pLine->m_pTAPos[i-1][2] = dZ ;
			}

			pEnt->GetPoint ( 1., end ) ;				
			pLine->m_pTAPos[i][0] = end[0] ;  
			pLine->m_pTAPos[i][1] = end[1] ;  
			pLine->m_pTAPos[i][2] = dZ	  ; 

			pEnt = pEnt->next ;
		}
		if( pHead )
		{
			pHead->next = pLine ;
			pLine->prev = pHead ;
		}
		else
		{
			pHead = pLine ;
		}
EndLine:
		pSect = pNext ;
	}
	
	return pHead ;
}

CSmtCutPath* ReadTimeFromFile(int StartIndex, int EndIndex)
{
	CFileDialog dlgFile(TRUE, NULL, NULL, OFN_HIDEREADONLY| OFN_OVERWRITEPROMPT, NULL, NULL );

	//CString title = "Read parameter from File" ;
	//CString strFilter = "控制参数文件 (*.ctp)" ;
	LPSTR lpBuffer = glbf_GetPromptString() ;
	glbf_LoadSysString ( IDS_MDFYGRND_CTRLPARAMFILE, lpBuffer, MAX_STR_LENG ) ;

	CString strFilter = lpBuffer;
	CString strDefault;

	strFilter += (TCHAR)'\0'; 
	strFilter += "*.ctp"; 
	strFilter += (TCHAR)'\0'; 
	dlgFile.m_ofn.nMaxCustFilter++;

	dlgFile.m_ofn.lpstrFilter = strFilter;
	glbf_LoadSysString ( IDS_MDFYGRND_READPARAMFILE, lpBuffer, MAX_STR_LENG ) ;
	dlgFile.m_ofn.lpstrTitle = lpBuffer;

	if(  dlgFile.DoModal() != IDOK )
	{
		//glbf_LoadSysString ( IDS_MDFYGRND_READFILEFAILURE, lpBuffer, MAX_STR_LENG ) ;
		//AfxMessageBox( lpBuffer );
		return NULL ;
	}
	CString file = dlgFile.GetPathName();
	CFile File;
	if (!File.Open(file,  CFile::modeRead |CFile::typeBinary ))
	{
		//AfxMessageBox( "不能打开给定文件!" );
		//glbf_LoadSysString ( IDS_MDFYGRND_CANNOTOPENFILE, lpBuffer, MAX_STR_LENG ) ;
		//AfxMessageBox( lpBuffer );
		return NULL ;
	}

	// STEP 1 : 读取文件头
	TIME_FileHead fileHead ;
	File.SeekToBegin() ;

	if( File.Read( &fileHead, sizeof( fileHead ) )!= sizeof( fileHead )  ||
		strcmp( fileHead.szIdentify, TIME_IDENTIFY )  )  
	{ //读取文件头错误
		//AfxMessageBox( "读取文件头错误！") ;
		//glbf_LoadSysString ( IDS_MDFYGRND_READFILEERROR, lpBuffer, MAX_STR_LENG ) ;
		//AfxMessageBox( lpBuffer );
		return NULL ;
	}
	// 读取时间文件
	CSmtCutPath *pPath = NULL ;
	if( fileHead.lVersion == TIME_VERSION_1001 )
	{
		pPath = ReadTimeFromFileByVersion1001( File, fileHead ) ;
	}
	else
	{
		ASSERT( 0 ) ;
	}

	if (pPath)
	{
		// 删除后面没用的路径点
		CSmtCutPoint* pHead = NULL, *pCPnt = NULL;
		while (pPath->m_pTail && pPath->m_nNumPnt > EndIndex)
		{
			pCPnt = pPath->m_pTail;
			pPath->RemovePoint(pCPnt);
			delete pCPnt;
		}
		// 删除前面没用的路径点
		int nPos = 0;
		while (pPath->m_pHead && nPos < StartIndex)
		{
			pCPnt = pPath->m_pHead;
			pPath->RemovePoint(pCPnt);
			delete pCPnt;
			++nPos;
		}
		// 修正加工时间
		pHead = pPath->m_pHead;
		if (!pHead)
		{
			delete pPath;
			return NULL;
		}
		pCPnt = pHead->next;
		while (pCPnt)
		{
			pCPnt->m_fPoint[0] -= pHead->m_fPoint[0];
			pCPnt = pCPnt->next;
		}
		pHead->m_fPoint[0] = 0.0;
	}

	if (pPath->m_nNumPnt != EndIndex - StartIndex)
	{
		delete pPath;
		pPath = NULL;
	}

	return pPath ;
}

CSmtCutPath * ReadTimeFromFileByVersion1001( CFile &File, TIME_FileHead &FileHead ) 
{
	// 搜索到数据点的位置，进行时间的读取
	File.Seek( FileHead.lTimehDataOffset, CFile::begin ) ;
	long  npos = -1 ;
	CSmtCutPath *pPath = new CSmtCutPath() ;
	PNT3D point = { 0., 0., 0. } ;

	// 获取点数
	CString strItem, result ; 
	strItem = FileHead.szTimePntNum ;
	npos = strItem.Find ( ' ' ) ;
	result = strItem.Right( strItem.GetLength() - npos ) ;

	TIME_FileBody data ;
	memset( &data, 0, sizeof( TIME_FileBody ) ) ;
	int nCnt = atoi( result ), i = 0 ;
	for( i = 0 ; i < nCnt ; i++ )
	{
		File.Read( &data, sizeof( TIME_FileBody ) ) ;
		point[0]= data.dTimeData ;
		pPath->AddPoint( point ) ;
	}

	File.Close() ; 

	return pPath ;
}

double CSmartPathGen::NormalizeLenByCutTime(CSmtCutPath* CutPath5x, int StartIndex, int EndIndex, BOOL ReadFromFile, BOOL Trochoidal)
{
	if ( !CutPath5x || !CutPath5x->m_pHead || CutPath5x->NumPoint() < 2  ||
		StartIndex >= EndIndex || StartIndex >= CutPath5x->m_nNumPnt || EndIndex - StartIndex != CutPath5x->m_nNumPnt)
		return -1 ;

	double dTotalTime = 0.0;
	// 计算速度和长度，并优化速度，使用m_fTempPos记录长度[0],速度[1],优化后速度[2]
	if (ReadFromFile && !Trochoidal) // 摆线不支持读取时间文件
	{
		CJDJob* pJob = SurfNC_GetCurJobSetup() ;
		if (!pJob)
		{
			return -1;
		}	

		double dTLen = 0., dSLen = 0., dV = 0., dFeed = m_cSpeedDef.m_dFeedRate, dTipFeed = 0.0;
		double dMinFeed[2] = {/*pJob->m_dRateDm[0]*/dFeed, /*pJob->m_dRateDm[0]*/dFeed}, dMaxFeed = min(pJob->m_dRateDm[1], /*m_cParam.m_dMaxFeed*/12.0);
		CSmtCutPoint* pCPnt = NULL, * pPrevCPnt = NULL;
		CSmtCutPointEx *pCPntEx = NULL, *pPrevCPntEx = NULL;
		CSmtCutPath* pTimePath = ReadTimeFromFile(StartIndex, EndIndex);
		if (!pTimePath || !pTimePath->m_pHead)
		{
			LPSTR lpBuffer = glbf_GetPromptString() ;
			glbf_LoadSysString ( IDS_MDFYGRND_READFILEFAILURE, lpBuffer, MAX_STR_LENG ) ;
			AfxMessageBox( lpBuffer );
			return NormalizeLenByDefaultCtrlParam(CutPath5x);
		}
		dTotalTime = pTimePath->m_pTail->m_fPoint[0];
		pPrevCPnt = pTimePath->m_pHead;
		pPrevCPntEx = (CSmtCutPointEx *)CutPath5x->m_pHead ;
		pPrevCPntEx->m_fPoint[3] = 0.0;
		pPrevCPntEx->m_fTempPos[0] = 0.0;
		pPrevCPntEx->m_fTempPos[1] = TFLOAT(dMinFeed[0]);
		pPrevCPntEx->m_fTempPos[2] = TFLOAT(dMinFeed[0]);
		dTLen = 0.0;
		for (pCPntEx = (CSmtCutPointEx*)pPrevCPntEx->next, pCPnt = pPrevCPnt->next; 
			pCPntEx, pCPnt; 
			pCPntEx = (CSmtCutPointEx*)pCPntEx->next, pCPnt = pCPnt->next)
		{
			dTLen = nc_Distance(pCPntEx->m_fPoint, pPrevCPntEx->m_fPoint, 3);
			dSLen = nc_Distance ( pPrevCPntEx->m_fSurfPos, pCPntEx->m_fSurfPos, 3 ) ;
			pCPntEx->m_fPoint[3] = pCPnt->m_fPoint[0];
			if( dSLen < MIN_LEN )
			{
				pCPntEx->m_fTempPos[0] = pPrevCPntEx->m_fTempPos[0] ;
				pCPntEx->m_fTempPos[1] = pPrevCPntEx->m_fTempPos[1];
			}
			else
			{
				// 刀尖点速度
				dTipFeed = dTLen / dSLen * dFeed;
				if( dTipFeed > dMaxFeed ) 
					dTipFeed = dMaxFeed ;
				pCPntEx->m_fTempPos[0] = FLOAT( pPrevCPntEx->m_fTempPos[0] + dTLen ) ; 
				pCPntEx->m_fTempPos[1] = FLOAT( dTipFeed ) ;						
			}

			dV = dTLen/(pCPnt->m_fPoint[0] - pPrevCPnt->m_fPoint[0]) * 0.06;
			if (dV > dMaxFeed)
			{
				dV = dMaxFeed;
			}
			pCPntEx->m_fTempPos[2] = TFLOAT(dV);

			pPrevCPntEx = pCPntEx;
			pPrevCPnt = pCPnt;
		}
		delete pTimePath;
		// 依据加工时间参数化
		pPrevCPntEx = (CSmtCutPointEx*)CutPath5x->m_pHead;
		for ( pCPntEx = (CSmtCutPointEx*)pPrevCPntEx->next ; pCPntEx ; ) 
		{
			pCPntEx->m_fPoint[3] /= TFLOAT(dTotalTime);
			pPrevCPntEx = pCPntEx ;
			pCPntEx = (CSmtCutPointEx*)pCPntEx->next ;
		}
	}
	else
	{
		return NormalizeLenByDefaultCtrlParam(CutPath5x, Trochoidal);
	}

	return dTotalTime ;
}

double CSmartPathGen::NormalizeLenByDefaultCtrlParam(CSmtCutPath* CutPath5x, BOOL Trochoidal)
{
	if ( !CutPath5x || !CutPath5x->m_pHead || CutPath5x->NumPoint() < 2 )
		return -1 ;

	CJDJob* pJob = SurfNC_GetCurJobSetup() ;
	if (!pJob)
	{
		return -1;
	}	

	double dTLen = 0., dSLen = 0.;
	double dFeed = m_cSpeedDef.m_dFeedRate, dTipFeed = 0.0;
	double dMinFeed[2] = {dFeed, dFeed}, dMaxFeed = min(pJob->m_dRateDm[1], /*m_cParam.m_dMaxFeed*/12.0);
	double dTotalTime = 0.0, dTo_m_s = 1000.0/60.0;
	CSmtCutPointEx *pCPntEx = NULL, *pPrevCPntEx = NULL;
	// 计算速度和长度，并优化速度，使用m_fTempPos记录长度[0],速度[1],优化后速度[2]
	double dt[3] = {0.}, dPrevV = 0., dV = 0., dVa = /*m_cParam.m_dKeywayWidth*/0.4 * 1000.0;
	double dS = 0., dLen[2] = {0.0};
	if (Trochoidal)
	{// 摆线时，直接使用加工速度作为最小速度
		dMinFeed[0] = dFeed;
		dMinFeed[1] = dFeed;
	}
	dMinFeed[0] *= dTo_m_s;
	dMinFeed[1] *= dTo_m_s;
	dMaxFeed *= dTo_m_s;
	pPrevCPntEx = (CSmtCutPointEx *)CutPath5x->m_pHead ;
	pPrevCPntEx->m_fPoint[3] = 0.0;
	pPrevCPntEx->m_fTempPos[0] = 0.0;
	pPrevCPntEx->m_fTempPos[1] = TFLOAT(dFeed*dTo_m_s);
	pPrevCPntEx->m_fTempPos[2] = pPrevCPntEx->m_fTempPos[1];
	if (CutPath5x->m_nNumPnt > 2)
	{
		pPrevCPntEx->m_fTempPos[1] = pPrevCPntEx->m_fTempPos[2] = TFLOAT(dMinFeed[0]);
	}
	else
	{
		pPrevCPntEx->m_fTempPos[1] = pPrevCPntEx->m_fTempPos[2] = TFLOAT(dFeed * dTo_m_s);
	}
	for ( pCPntEx = (CSmtCutPointEx*)pPrevCPntEx->next ; pCPntEx ; ) 
	{
		dTLen = nc_Distance ( pPrevCPntEx->m_fPoint, pCPntEx->m_fPoint, 3 ) ;
		dSLen = nc_Distance ( pPrevCPntEx->m_fSurfPos, pCPntEx->m_fSurfPos, 3 ) ;
		pCPntEx->m_fPoint[3] = 0.0;
		if( dSLen < MIN_LEN )
		{
			nc_VectorCopy(pCPntEx->m_fTempPos, pPrevCPntEx->m_fTempPos, 3);
		}
		else
		{
			// 刀尖点速度
			dTipFeed = dTLen / dSLen * dFeed * dTo_m_s;
			if( dTipFeed > dMaxFeed ) 
				dTipFeed = dMaxFeed ;
			pCPntEx->m_fTempPos[0] = FLOAT( pPrevCPntEx->m_fTempPos[0] + dTLen ) ; 
			pCPntEx->m_fTempPos[1] = FLOAT( dTipFeed ) ;
			pCPntEx->m_fTempPos[2] = pCPntEx->m_fTempPos[1];
		}

		pPrevCPntEx = pCPntEx ;
		pCPntEx = (CSmtCutPointEx*)pCPntEx->next ;
	}

	// 末点速度为。
	pCPntEx = ( CSmtCutPointEx *)CutPath5x->m_pTail ;
	if (CutPath5x->m_nNumPnt > 2)
	{
		pCPntEx->m_fTempPos[1] = pCPntEx->m_fTempPos[2] = TFLOAT(dMinFeed[0]);
	}
	else
	{
		pCPntEx->m_fTempPos[1] = pCPntEx->m_fTempPos[2] = TFLOAT(dFeed * dTo_m_s);
	}

	// 正向优化速度
	pPrevCPntEx = (CSmtCutPointEx*)CutPath5x->m_pHead;
	for ( pCPntEx = (CSmtCutPointEx*)pPrevCPntEx->next ; pCPntEx ; ) 
	{
		dS = pCPntEx->m_fTempPos[0] - pPrevCPntEx->m_fTempPos[0];
		dV = pCPntEx->m_fTempPos[2];
		dPrevV = pPrevCPntEx->m_fTempPos[2];
		if (dV > dPrevV)
		{// 加速
			dt[0] = (dV - dPrevV)/dVa;
			dt[1] = (-dPrevV + sqrt(dPrevV * dPrevV + 2 * dS * dVa))/dVa;			
			if (dt[0] > dt[1])
			{// 速度达不到
				dV = dPrevV + dVa * dt[1];
				pCPntEx->m_fTempPos[2] = TFLOAT(dV);
			}
			else
			{// 速度能够达到
				dV = pCPntEx->m_fTempPos[2];
			}
		}

		pPrevCPntEx = pCPntEx ;
		pCPntEx = (CSmtCutPointEx*)pCPntEx->next ;
	}
	// 反向优化速度
	pPrevCPntEx = (CSmtCutPointEx*)CutPath5x->m_pTail;
	for ( pCPntEx = (CSmtCutPointEx*)pPrevCPntEx->prev ; pCPntEx ; ) 
	{
		dS = pPrevCPntEx->m_fTempPos[0] - pCPntEx->m_fTempPos[0];
		dV = pCPntEx->m_fTempPos[2];
		dPrevV = pPrevCPntEx->m_fTempPos[2];
		if (dV > dPrevV)
		{// 加速
			dt[0] = (dV - dPrevV)/dVa;
			dt[1] = (-dPrevV + sqrt(dPrevV * dPrevV + 2 * dS * dVa))/dVa;			
			if (dt[0] > dt[1])
			{// 速度达不到
				dV = dPrevV + dVa * dt[1];
				pCPntEx->m_fTempPos[2] = TFLOAT(dV);
			}
			else
			{// 速度能够达到
				dV = pCPntEx->m_fTempPos[2];
			}
		}

		pPrevCPntEx = pCPntEx ;
		pCPntEx = (CSmtCutPointEx*)pCPntEx->prev ;
	}

	// 通过优化后的速度计算实际的加工时间
	pPrevCPntEx = (CSmtCutPointEx*)CutPath5x->m_pHead;
	for ( pCPntEx = (CSmtCutPointEx*)pPrevCPntEx->next ; pCPntEx ; ) 
	{
		dS = pCPntEx->m_fTempPos[0] - pPrevCPntEx->m_fTempPos[0];
		dV = pCPntEx->m_fTempPos[2];
		dPrevV = pPrevCPntEx->m_fTempPos[2];
		if (dV > dPrevV)
		{// 加速
			dt[0] = (dV - dPrevV)/dVa;
			dt[1] = dS * 2/(dV + dPrevV);
			if (dt[0] > dt[1])
			{// 速度达不到
				pCPntEx->m_fPoint[3] = TFLOAT(dt[1]) + pPrevCPntEx->m_fPoint[3];
			}
			else
			{
				dLen[0] = dPrevV * dt[0] + 0.5 * dVa * dt[0] * dt[0];
				dLen[1] = dS - dLen[0];
				dt[1] = dLen[1]/dV;
				pCPntEx->m_fPoint[3] = TFLOAT(dt[0] + dt[1]) + pPrevCPntEx->m_fPoint[3];
			}
		}
		else if (pCPntEx->m_fTempPos[2] < pPrevCPntEx->m_fTempPos[2])
		{// 减速
			dt[0] = (dPrevV - dV)/dVa;
			dt[1] = dS * 2/(dV + dPrevV);
			if (dt[0] > dt[1])
			{// 速度达不到
				pCPntEx->m_fPoint[3] = TFLOAT(dt[1]) + pPrevCPntEx->m_fPoint[3];
			}
			else
			{
				dLen[0] = dV * dt[0] + 0.5 * dVa * dt[0] * dt[0];
				dLen[1] = dS - dLen[0];
				dt[1] = dLen[1]/dPrevV;
				pCPntEx->m_fPoint[3] = TFLOAT(dt[0] + dt[1]) + pPrevCPntEx->m_fPoint[3];
			}
		}
		else
		{// 匀速
			dt[0] = dS/dV;
			pCPntEx->m_fPoint[3] = TFLOAT(dt[0]) + pPrevCPntEx->m_fPoint[3];
		}

		pPrevCPntEx = pCPntEx ;
		pCPntEx = (CSmtCutPointEx*)pCPntEx->next ;
	}

	// 最后一点速度设置与前一点速度一致
	pPrevCPntEx = (CSmtCutPointEx*)CutPath5x->m_pTail;
	pCPntEx = (CSmtCutPointEx*)pPrevCPntEx->prev;
	pPrevCPntEx->m_fTempPos[2] = pCPntEx->m_fTempPos[2];

	// 依据加工时间参数化
	dTotalTime = CutPath5x->m_pTail->m_fPoint[3];
	pPrevCPntEx = (CSmtCutPointEx*)CutPath5x->m_pHead;
	pPrevCPntEx->m_fTempPos[1] = pPrevCPntEx->m_fTempPos[2] *= TFLOAT(0.06);
	for ( pCPntEx = (CSmtCutPointEx*)pPrevCPntEx->next ; 
		pCPntEx ; 
		pPrevCPntEx = pCPntEx, pCPntEx = (CSmtCutPointEx*)pCPntEx->next) 
	{
		pCPntEx->m_fPoint[3] /= TFLOAT(dTotalTime);
		pCPntEx->m_fTempPos[1] *= TFLOAT(0.06);
		pCPntEx->m_fTempPos[2] *= TFLOAT(0.06);
	}

	return dTotalTime ;
}


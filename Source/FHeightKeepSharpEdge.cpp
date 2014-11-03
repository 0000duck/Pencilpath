#include "StdAfx.H"
#include "SmartNC.H"
#include "Entity.H"
#include "SurfEtt.H"
#include "Model.H"
#include "FHeightKeepSharpEdge.h"
#include "SurfNC.H"
#include "Nc3DStepAndSpiral.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char BASED_CODE THIS_FILE[] = __FILE__;
#endif


double MathCam_CalTriAng(PNT3D Prev,PNT3D Pt,PNT3D Next)
{
	double a,b,c,dCosAngle;
	//计算三角形内角
	b = nc_Distance(Prev,Pt,3);
	c = nc_Distance(Next,Pt,3);
	a = nc_Distance(Prev,Next,3);
	dCosAngle=(b*b+c*c-a*a)/(2.0*b*c);
	if(fabs(dCosAngle)<=1.0 )
	{
		return acos(dCosAngle);
	}
	else
	{
		return -1;
	}
}
static BOOL MathCam_IsLineSteep(PNT3D pt1,PNT3D pt2,double angle)
{
	double dZValue = fabs(pt1[2] - pt2[2]);
	if(dZValue<0.002)
		return FALSE;
	DOUBLE dAngle = atan2(dZValue,double(nc_Distance(pt1,pt2,2))) ;
	if(dAngle>= angle)
		return TRUE;
	else 
		return FALSE;
}
static void MathCam_MovePtAlongVec2D(FPNT3D OrigPt,FPNT3D DestPt,
									   VEC3D Dir,double Dist)
{
	for(int i=0; i<2; i++)
	{
		DestPt[i] = TFLOAT( OrigPt[i] + Dir[i]*Dist );
	}
}
static void MathCam_MovePtAlongVec2D(double OrigPt[],float DestPt[],
									   VEC3D Dir,double Dist)
{
	for(int i=0; i<2; i++)
	{
		DestPt[i] = TFLOAT( OrigPt[i] + Dir[i]*Dist );
	}
}
static void MathCam_CalMidPnt2D(FPNT3D dBegin,FPNT3D dEnd,FPNT3D dMid)
{
	for(int i=0; i<2; i++) 
	{
		dMid[i] = TFLOAT( (dBegin[i] + dEnd[i])*0.5 );
	}
}
static void MathCam_CopyPnt2D(float SourcePt[], PT_2D & DestPt)
{
	DestPt.x = SourcePt[0];
	DestPt.y = SourcePt[1];
}
static void MathCam_GetUnitVec2D(float Start[], float End[],double Vec2D[])
{
	Vec2D[0] = End[0] - Start[0];
	Vec2D[1] = End[1] - Start[1];
	mathUniVec2D(Vec2D, 1.0e-8);
}
CSmartTool*  MathCam_GetInnerOffsetTool(CSmartTool & SrcTool, DOUBLE Offset) 
{
	DOUBLE BottomOff = fabs(Offset);
	if( BottomOff < 0.001 ) 
	{
		return  SrcTool.CopyMyself() ;
	}

	DOUBLE		dAngle = SrcTool.m_fAngle   ;
	DOUBLE		dRadius = SrcTool.m_fRadius ;
	DOUBLE		dBottom = SrcTool.m_fBottom ;
	DOUBLE		dCorner = SrcTool.m_fCorner ;
	CSmartTool*	pMiller = NULL ; 
	switch( SrcTool.GetType() ) 
	{
	case smtToolFlat: //平底刀
		{
			pMiller=new CSmtToolFlat(); 
			pMiller->SetParam( dRadius-BottomOff,	//顶半径
				0.0 , 0.0, 0.0 ); //圆角半径
		}
		break ;
	case smtToolBall: //球头刀
		{
			pMiller=new CSmtToolBall(); 
			pMiller->SetParam( dRadius-BottomOff,	//顶半径
				0.0 ,
				0.0, dRadius-BottomOff ) ;
		}
		break ;
	case smtToolNose: //牛鼻刀
		{
			pMiller = new CSmtToolNose();
			pMiller->SetParam( dRadius-BottomOff,	//顶半径
				dBottom,				//底半径
				0.0, dCorner-BottomOff); //圆角半径
		}
		break ;
	case smtToolAFlat : //锥度平底刀
		{
			pMiller = new CSmtToolAFlat(); 
			pMiller->SetParam( dRadius-BottomOff,	//顶半径
				dBottom-BottomOff,				//底半径
				dAngle, 0.0); //圆角半径
		}
		break ;
	case smtToolABall: //锥度球头刀
		{
			pMiller = new CSmtToolABall(); 
			DOUBLE dNewCorner = (dBottom - BottomOff) /cos(SrcTool.m_fAngle);
			pMiller->SetParam(dRadius-BottomOff,	//顶半径
				dBottom - BottomOff,				//底半径
				dAngle, dNewCorner); //圆角半径
		}
		break;
	case smtToolANose: //锥度牛鼻刀
		{
			pMiller = new CSmtToolANose(); 
			pMiller->SetParam( dRadius-BottomOff,	//顶半径
				dBottom,				//底半径
				dAngle, dCorner-BottomOff); //圆角半径
		}
		break ;
	default:
		break;
	}
	if(!pMiller) 
	{
		return SrcTool.CopyMyself();
	}
	pMiller->m_nToolID = SrcTool.m_nToolID;
	strcpy( pMiller->m_sToolName, SrcTool.m_sToolName ) ;
	pMiller->RefineParam();
	return pMiller;	
}

CFHeightKeepSharpEdge::CFHeightKeepSharpEdge()
{
	m_bKeepHoz = FALSE;
	m_dHozEdgeZShift = 0.05;
}

CFHeightKeepSharpEdge::~CFHeightKeepSharpEdge()
{
	ClearAll();
}

void CFHeightKeepSharpEdge::ClearAll()
{
	for(INT_PTR i=0; i<m_cSharpEdgeChkMdl.GetCount(); i++)
	{
		CEdgeChkMdl * p = m_cSharpEdgeChkMdl.GetAt(i);
		if(p != NULL)
		{
			delete p;
			p = NULL;
		}
	}
	m_cSharpEdgeChkMdl.RemoveAll();
}

CSmartCheck * CFHeightKeepSharpEdge::GetSmtEdge(double Edge0[3],double Edge1[3])
{
	CSmtCutPath cTmpPath;
	cTmpPath.AddPoint(Edge0);
	cTmpPath.AddPoint(Edge1);
	CSmartCheck *pSmtChk = new CSmartCheck() ;
	pSmtChk->m_nNumVert = cTmpPath.m_nNumPnt;
	pSmtChk->m_aVertex = new TFLOAT[pSmtChk->m_nNumVert][3] ;
	pSmtChk->m_nNumEdge = pSmtChk->m_nNumVert - 1 ;
	pSmtChk->m_aAllEdge = new CSmtCheckEdge[pSmtChk->m_nNumEdge] ;
	int nVertCount = 0;
	for(CSmtCutPoint * pPnt=cTmpPath.m_pHead; pPnt; pPnt=pPnt->next)
	{
		nc_VectorCopy(pSmtChk->m_aVertex[nVertCount],pPnt->m_fPoint,3);
		nVertCount++;
	}
	int nEdgeCount = 0;
	for(CSmtCutPoint * pPnt=cTmpPath.m_pHead; pPnt&&pPnt->next ;pPnt=pPnt->next)
	{
		pSmtChk->m_aAllEdge[nEdgeCount].m_pAtCheck = pSmtChk ;
		pSmtChk->m_aAllEdge[nEdgeCount].m_nID[0] = nEdgeCount ;
		pSmtChk->m_aAllEdge[nEdgeCount].m_nID[1] = nEdgeCount + 1 ;
		pSmtChk->m_aAllEdge[nEdgeCount].m_bVtFlag = SMTEDGE_END ;
		if( nEdgeCount == 0 )
			pSmtChk->m_aAllEdge[nEdgeCount].m_bVtFlag |= SMTEDGE_START ;
		nEdgeCount++;
	}
	pSmtChk->DefineBox () ;
	return pSmtChk;
}

void CFHeightKeepSharpEdge::AddOneEdgeMdl(PNT3D* aEdgePts, int EdgePtCnt
										  ,BOOL IsSteep,CSmartTool * Tool)
{
	if(aEdgePts && EdgePtCnt>1 && Tool)
	{
		double dMinZ = aEdgePts[0][2];
		double dMaxZ = aEdgePts[0][2];
		for(int i=1; i<EdgePtCnt; i++)
		{
			dMinZ = min(dMinZ, aEdgePts[i][2]);
			dMaxZ = max(dMaxZ, aEdgePts[i][2]);
		}

		//相同高度的水平边界放到一个检查模型中
		CSmtCheckMdl * pEdgeChkMdl = NULL;
		BOOL bSameZEdge = FALSE;
		
		for(INT_PTR i=0; i<m_cSharpEdgeChkMdl.GetCount(); i++)
		{//搜索边是否是重边
			CEdgeChkMdl * p = m_cSharpEdgeChkMdl.GetAt(i);
			if(p == NULL)	{  continue; }
			if( (nc_Distance(p->m_dBegin, aEdgePts[0], 3 ) <0.005 &&
				nc_Distance(p->m_dEnd, aEdgePts[1], 3 ) <0.005 ) || 
				(nc_Distance(p->m_dBegin, aEdgePts[1], 3 ) <0.005 &&
				nc_Distance(p->m_dEnd, aEdgePts[0], 3 ) <0.005 ) )
			{
				return;
			}
		}

		if(m_bKeepHoz && (dMaxZ-dMinZ) < 0.05 )
		{
			for(INT_PTR i=0; i<m_cSharpEdgeChkMdl.GetCount(); i++)
			{
				CEdgeChkMdl * p = m_cSharpEdgeChkMdl.GetAt(i);
				if(p != NULL && fabs(p->m_dBegin[2] - p->m_dEnd[2]) <0.02 
					&& fabs(dMaxZ - p->m_dBegin[2]) <0.02 )
				{
					bSameZEdge = TRUE;
					pEdgeChkMdl = p->m_pEdgeChk;
					break;
				}
			}
		}
		if(pEdgeChkMdl==NULL)
		{
			pEdgeChkMdl = new CSmtCheckMdl;
		}
		pEdgeChkMdl->m_bCheckMode = SMART_MODEL_PROJECT ;
		pEdgeChkMdl->m_fBottom = -1.0e6 ;
		for(int i=0; i<EdgePtCnt-1; i++)
		{
			CSmartCheck * pSmtChk = GetSmtEdge(aEdgePts[i], aEdgePts[i+1]);
			if(pSmtChk )//&& fabs(aEdgePts[i][2]- aEdgePts[i+1][2])>0.01 )
			{
				pEdgeChkMdl->AddCheck(pSmtChk);
			}
		}
		pEdgeChkMdl->UpdateCurrTool( Tool ) ;
		if(!bSameZEdge)
		{
			CEdgeChkMdl * pNewEdge = new CEdgeChkMdl();
			pNewEdge->m_pEdgeChk = pEdgeChkMdl;
			nc_VectorCopy(pNewEdge->m_dBegin, aEdgePts[0], 3);
			nc_VectorCopy(pNewEdge->m_dEnd, aEdgePts[EdgePtCnt-1], 3);
			pNewEdge->m_bIsSteep = IsSteep;
			m_cSharpEdgeChkMdl.Add(pNewEdge);
		}
		
	}
}

//生成锋利边界检查模型
BOOL CFHeightKeepSharpEdge::GenSharpEdgeChkMdl(CPtrList & SharpEdges,CSmartTool * Tool)
{
	BOX3D box;
	for (POSITION ChildPos=SharpEdges.GetHeadPosition(); ChildPos!=NULL; )
	{
		CStrpt* pStrpt = (CStrpt*)SharpEdges.GetNext(ChildPos) ;
		if(pStrpt == NULL || pStrpt->m_ps==NULL || pStrpt->m_np<2)
		{
			continue;
		}
		pStrpt->CalBox(&box);
		PNT3D* aEdgePts = pStrpt->m_ps;
		BOOL bIsFlat = TRUE;
		for(int i=0; i<pStrpt->m_np-1; i++)
		{
			if( MathCam_IsLineSteep(aEdgePts[i],aEdgePts[i+1],ANGLE_TO_RADIAN(88) ) )
			{
				bIsFlat = FALSE;
				break;
			}
		}
		if(0)//bIsFlat)
		{
			AddOneEdgeMdl(aEdgePts, pStrpt->m_np, FALSE, Tool);
		}
		else
		{
			for(int i=0; i<pStrpt->m_np-1; i++)
			{
				if(m_bKeepHoz && fabs(aEdgePts[i][2] - aEdgePts[i+1][2])<=0.05 )
				{
					AddOneEdgeMdl(&aEdgePts[i], 2, MathCam_IsLineSteep(aEdgePts[i],aEdgePts[i+1],ANGLE_TO_RADIAN(88)),Tool);
				}
				else if(!m_bKeepHoz && fabs(aEdgePts[i][2] - aEdgePts[i+1][2])>0.05)
				{
					AddOneEdgeMdl(&aEdgePts[i], 2, MathCam_IsLineSteep(aEdgePts[i],aEdgePts[i+1],ANGLE_TO_RADIAN(88)),Tool);
				}
			}
		}
	}
	return TRUE;
}

BOOL CFHeightKeepSharpEdge::IsPointOnEdge(CSmtCutPoint & Pnt,
										  CSmtCheckMdl & EdgeChkMdl)
{
	FPNT3D fDefinePt;
	nc_VectorCopy(fDefinePt, Pnt.m_fPoint, 3);
	fDefinePt[2] = (TFLOAT)EdgeChkMdl.m_fBottom;
	EdgeChkMdl.DefineHeight(fDefinePt);
	if(fabs(fDefinePt[2] - Pnt.m_fPoint[2]) < 0.01 )
	{
		return TRUE;
	}
	return FALSE;
}
int CFHeightKeepSharpEdge::GetSmtCutPathCount(CSmtCutPath* Path)
{
	int nCount = 0;
	CSmtCutPath * pTemp = Path;
	while(pTemp)
	{
		nCount++;
		pTemp = pTemp->next;
	}
	return nCount;
}

BOOL CFHeightKeepSharpEdge::VerifyLocalPath(double Begin[], double End[],
											CSmtCheckMdl &DriveMdl)
{
	FPNT3D  dLBox[2] ;
	dLBox[0][2] = (TFLOAT)DriveMdl.m_fBottom ;
	dLBox[1][2] = 1.0e6;//(TFLOAT)DriveMdl.m_fBottom;//
	dLBox[0][0] = TFLOAT( min(Begin[0], End[0]) - 0.1);
	dLBox[0][1] = TFLOAT( min(Begin[1], End[1]) - 0.1);
	dLBox[1][0] = TFLOAT( max(Begin[0], End[0]) + 0.1);
	dLBox[1][1] = TFLOAT( max(Begin[1], End[1]) + 0.1);
	DriveMdl.LabelCheckByBox( dLBox );

	double dLen = nc_Distance(Begin, End, 3);
	int nPoint = int(dLen / 0.015) + 2;
	double n = nPoint - 1 ;
	FPNT3D fPrjPt;
	double vLineDir[2], vHorPrj[3], dCurrPt[3];
	nc_VectorMinus(Begin, End, vLineDir, 2);
	nc_Normalize( vLineDir, 2 ) ;
	mathRotVec2D( PI1_2, vLineDir, vHorPrj ) ;
	for(int i=1; i<nPoint; i++)
	{
		double u = double(i) / n;
		dCurrPt[0] = TFLOAT( (1.0-u)*Begin[0] + u*End[0] );
		dCurrPt[1] = TFLOAT( (1.0-u)*Begin[1] + u*End[1] );
		dCurrPt[2] = TFLOAT( (1.0-u)*Begin[2] + u*End[2] );
		nc_VectorCopy(fPrjPt, dCurrPt, 2);
		fPrjPt[2] = DriveMdl.m_fBottom;
		DriveMdl.DefineHeight(fPrjPt);
		if( dCurrPt[2] >= fPrjPt[2] || 
			fabs(dCurrPt[2]-fPrjPt[2])<0.002 || 
			CanHorPrjPntOnChkMdl(dCurrPt, vHorPrj, DriveMdl,0.01))
		{
			continue;
		}
		else
		{
			return FALSE;
		}
	}
	return TRUE;
}

CSmtCutPath *  CFHeightKeepSharpEdge::BreakPathByPntType(CSmtCutPath & Path, short Type)
{
	CTypedPtrArray<CPtrArray,CSmtCutPath*> BrkPathArr;
	CSmtCutPath * pNewPath = new CSmtCutPath();
	for(CSmtCutPoint *p=Path.m_pHead; p; p=p->next)
	{
		pNewPath->AddPoint(p->m_fPoint);
		if(p==Path.m_pTail)
		{
			BrkPathArr.Add(pNewPath);
			break;
		}
		if(p->m_bType == Type)
		{
			BrkPathArr.Add(pNewPath);
			pNewPath = new CSmtCutPath();
			pNewPath->AddPoint(p->m_fPoint);
		}
	}

	CSmtCutPath * pBrkHead, * pBrkTail;
	pBrkHead = pBrkTail = BrkPathArr[0];
	for(INT_PTR i=1; i<BrkPathArr.GetCount(); i++)
	{
		pBrkTail->next = BrkPathArr[i];
		pBrkTail = BrkPathArr[i];
	}
	return pBrkHead;
}

void CFHeightKeepSharpEdge::MoveUpPathAtHozEdge(CSmtCutPoint * pArcBegin, CSmtCutPoint * pArcEnd)
{
	if(pArcBegin && pArcEnd)
	{
		//以下处理部分路径落在水平锋利边上
		int nPtOnEdge = 0;
		CSmtCutPoint * pPntOnEdge = pArcBegin;
		while(pPntOnEdge)	{
			nPtOnEdge++;
			if(pPntOnEdge==pArcEnd) {break;	}
			pPntOnEdge = pPntOnEdge->next;
		}
		pPntOnEdge = pArcBegin->next;
		while(pPntOnEdge)	{
			if(pPntOnEdge==pArcEnd) {break;	}
			pPntOnEdge->m_fPoint[3] = SMART_CUTPNT_ONHOZEDGE;
			pPntOnEdge = pPntOnEdge->next;
		}
		int nLead = 4;
		if(nPtOnEdge<4)
			return;
		else if( nPtOnEdge <= 5)
			nLead = 1;
		else if( nPtOnEdge > 5 && nPtOnEdge < 8 )
			nLead = 2;
		double nAngStep = 45.0/nLead;
		int nCount = 0;
		pPntOnEdge = pArcBegin;
		//add lead in
		while(pPntOnEdge)
		{
			if(pPntOnEdge==pArcEnd) {break;	}
			if(nCount<nLead) 
			{
				pPntOnEdge->m_fPoint[2] += TFLOAT( m_dHozEdgeZShift*tan(ANGLE_TO_RADIAN(nAngStep*nCount)) );
				pPntOnEdge->m_bType = SMART_CUTPNT_LEADINOUT;
			}
			nCount++;
			pPntOnEdge = pPntOnEdge->next;
		}

		//add lead out
		nCount = 0;
		pPntOnEdge = pArcEnd;
		while(pPntOnEdge)
		{
			if(pPntOnEdge==pArcBegin) {	break;	}
			if(nCount<nLead)	
			{
				pPntOnEdge->m_fPoint[2] += TFLOAT( m_dHozEdgeZShift*tan(ANGLE_TO_RADIAN(nAngStep*nCount)) );
				pPntOnEdge->m_bType = SMART_CUTPNT_LEADINOUT;
			}
			nCount++;
			pPntOnEdge = pPntOnEdge->prev;
		}
		pPntOnEdge = pArcBegin->next;
		while(pPntOnEdge)
		{
			if(pPntOnEdge==pArcEnd) {break;	}
			if(pPntOnEdge->m_bType != SMART_CUTPNT_LEADINOUT) 
			{
				pPntOnEdge->m_fPoint[2] += (TFLOAT)m_dHozEdgeZShift;
			}
			pPntOnEdge = pPntOnEdge->next;
		}
	}
}
void CFHeightKeepSharpEdge::ReconstructPathAtEdgeR(CSmtCPathLib& AllPath,CSmtCutPath *pPath,CEdgeChkMdl & EdgeMdl,
							   CSmtCheckMdl &DriveMdl, CSmtPtArr & cEdegeRPnts)
{//非加工水平边界路径重构原则：尽量不修改路径起末点
	int nPtNum = (int)cEdegeRPnts.GetCount();
	if(pPath==NULL || nPtNum < 3)
	{
		return;
	}

	if(!m_bKeepHoz && pPath->IsClosed() && nPtNum==pPath->m_nNumPnt)
	{//整圈路径都在边界检查模型上(都在加工顶点保护面),这时并没有破坏该锋利边
		return;
	}

	double dToolR = DriveMdl.m_pTool->m_fRadius;
	BOOL bHeadOnR , bTailOnR;
	bHeadOnR = bTailOnR = FALSE;
	for(int k=0; k<nPtNum; k++)
	{
		if(cEdegeRPnts.GetAt(k)==pPath->m_pHead)
			bHeadOnR = TRUE;
		if(cEdegeRPnts.GetAt(k)==pPath->m_pTail)
			bTailOnR = TRUE;
	}

	CSmtCutPoint * pArcBegin = NULL;
	CSmtCutPoint * pArcEnd   = NULL;
	for(CSmtCutPoint * pPt = pPath->m_pHead; pPt; pPt=pPt->next)
	{
		if(pArcBegin==NULL && pPt->prev && pPt->prev->m_bType!=SMART_CUTPNT_ONEDGER
			&& pPt->m_bType==SMART_CUTPNT_ONEDGER)
		{
			pArcBegin = pPt;
		}
		else if( pPt->next && pPt->next->m_bType!=SMART_CUTPNT_ONEDGER
			&& pPt->m_bType==SMART_CUTPNT_ONEDGER)
		{
			pArcEnd = pPt;
		}
	}

	if(!pArcBegin && bHeadOnR)
	{
		pArcBegin = pPath->m_pHead;
	}
	if(!pArcEnd && bTailOnR)
	{
		pArcEnd = pPath->m_pTail;
	}

	if(!pArcBegin && !pArcEnd )
	{
		return;
	}


	pPath->DefineBox();
	if(m_bKeepHoz && fabs(EdgeMdl.m_dBegin[2] - EdgeMdl.m_dEnd[2])<0.02 && nPtNum==pPath->m_nNumPnt 
		&&	(pPath->m_fBox[1][2]-pPath->m_fBox[0][2]<0.002) )
	{//水平锋利边 整段路径都在加工水平锋利边
		for(CSmtCutPoint * pPt = pPath->m_pHead; pPt; pPt=pPt->next) {
			pPt->m_fPoint[2] += (TFLOAT)m_dHozEdgeZShift;
			pPt->m_fPoint[3] = SMART_CUTPNT_ONHOZEDGE;
		}
		pPath->m_bPathFlag |= SMARTNC_CUTPATH_ONHOZEDGE ;
		return;
	}

	double dPathHeight = pArcBegin->m_fPoint[2] ;
	if(pArcBegin->m_fPoint[2] != pArcEnd->m_fPoint[2])
		dPathHeight = (pArcBegin->m_fPoint[2] + pArcEnd->m_fPoint[2])*0.5;
	double dDir0[2], dDir1[2], dInsertPt[3], dBeginPt[3], dEndPt[3];
	dInsertPt[2] = dPathHeight;
	nc_VectorCopy(dBeginPt, pArcBegin->m_fPoint, 3);
	nc_VectorCopy(dEndPt,   pArcEnd->m_fPoint,   3);

	CSectArc cArc2D ;
	PT_2D aEndTan[2], aArcPt[3];
	for(int i=0; i<3; i++)	MathCam_CopyPnt2D(cEdegeRPnts[i]->m_fPoint, aArcPt[i]);
	cArc2D.Create(aArcPt[0], aArcPt[1], aArcPt[2]);
	cArc2D.GetEndTangent(aEndTan[0], aEndTan[1]);
	dDir0[0] = aEndTan[0].x;	dDir0[1] = aEndTan[0].y;
	for(int i=3; i>0; i--)	MathCam_CopyPnt2D(cEdegeRPnts[nPtNum-i]->m_fPoint, aArcPt[3-i]);
	cArc2D.Create(aArcPt[0], aArcPt[1], aArcPt[2]);
	cArc2D.GetEndTangent(aEndTan[0], aEndTan[1]);
	dDir1[0] = aEndTan[1].x;	dDir1[1] = aEndTan[1].y;
	mathUniVec2D(dDir0, 1.0e-8);  mathUniVec2D(dDir1, 1.0e-8);

	if(bHeadOnR && bTailOnR &&	!(pArcBegin == pPath->m_pHead && pArcEnd == pPath->m_pTail) )
	{//起末点落在边界检查模型上
		if( m_bKeepHoz )
		{//水平锋利边
			if(pPath->IsClosed() && fabs(EdgeMdl.m_dBegin[2] - EdgeMdl.m_dEnd[2])<0.02 && (pPath->m_fBox[1][2]-pPath->m_fBox[0][2]<0.002) )
			{
				CSmtCutPoint * pPntOnEdge = pArcBegin;
				//校验段中是否包含不在边界检查模型上的点
				while(pPntOnEdge)	{
					if(pPntOnEdge->m_bType != SMART_CUTPNT_ONEDGER) { return; }
					pPntOnEdge = pPntOnEdge->next;
				}
				pPntOnEdge = pArcEnd;
				while(pPntOnEdge)	{
					if(pPntOnEdge->m_bType != SMART_CUTPNT_ONEDGER)  { return; }
					pPntOnEdge = pPntOnEdge->prev;
				}
				if(pArcEnd->next)
					pPath->SetClosedPathHead(pArcEnd->next);
				else if(pArcBegin->prev)
					pPath->SetClosedPathHead(pArcBegin->prev);
				else
					return;

				MoveUpPathAtHozEdge(pArcBegin, pArcEnd);
				pPath->m_bPathFlag |= SMARTNC_CUTPATH_ONHOZEDGE ;
			}
		}
		return;
	}
	
	
	int nInvalid = 0;
	CSmtCutPoint * pDel = pArcBegin->next;
	while(pDel)	{
		if(pDel->m_bType != SMART_CUTPNT_ONEDGER) nInvalid++;
		if(pDel==pArcEnd) {break;}
		pDel = pDel->next;
	}
	if(nInvalid==0)
	{
		if(m_bKeepHoz )
		{//水平锋利边
			if( fabs(EdgeMdl.m_dBegin[2] - EdgeMdl.m_dEnd[2])<0.02 && (pPath->m_fBox[1][2]-pPath->m_fBox[0][2]<0.02))
			{
				MoveUpPathAtHozEdge(pArcBegin, pArcEnd);
				pPath->m_bPathFlag |= SMARTNC_CUTPATH_ONHOZEDGE ;
			}
			return;
		}
		VEC2D vEdge, vStartEnd, dTan[2], dAngle;
		mathGetVec2D(EdgeMdl.m_dBegin, EdgeMdl.m_dEnd, vEdge);
		mathGetVec2D(dBeginPt, dEndPt, vStartEnd);
		mathUniVec2D(vEdge, 1.0e-8);	mathUniVec2D(vStartEnd, 1.0e-8);
		int nLin2DInt = 0;
		dAngle[0] = dAngle[1] = -1;
		if(pArcBegin->prev )  
		{
			MathCam_GetUnitVec2D(pArcBegin->prev->m_fPoint, pArcBegin->m_fPoint, dTan[0]);
			dAngle[0] =  mathGetAngle2DUnit(dTan[0], dDir0); 
			if( dAngle[0] < ANGLE_TO_RADIAN(5) )  memcpy(dDir0, dTan[0], sizeof(double[2])); //调整切线方向使之整齐
		}
		if(pArcEnd->next) 
		{
			MathCam_GetUnitVec2D(pArcEnd->m_fPoint, pArcEnd->next->m_fPoint , dTan[1]);
			dAngle[1] =  mathGetAngle2DUnit(dTan[1], dDir1);
			if( dAngle[1] < ANGLE_TO_RADIAN(5) )  memcpy(dDir1, dTan[1], sizeof(double[2])); //调整切线方向使之整齐
		}
		
		if( fabs( mathGetAngle2DUnit(vEdge, vStartEnd) - PI1_2 ) < ANGLE_TO_RADIAN(1) )  
		{//对称
			for(int m=0; m<2; m++)
			{
				if( dAngle[m]<0 ) continue;
				if( (dAngle[m] < ANGLE_TO_RADIAN(2)) || (PI1 - dAngle[m] < ANGLE_TO_RADIAN(2))  )
				{
					if(nLin2DInt!=1 && m==0 && pArcBegin->prev) nLin2DInt = mathIntLin2D(dBeginPt, dTan[0], EdgeMdl.m_dBegin, vEdge, 1.0e-4, 1.0e-3, dInsertPt);
					if(nLin2DInt!=1 && m==1 && pArcEnd->next)  nLin2DInt = mathIntLin2D(dEndPt, dTan[1], EdgeMdl.m_dBegin, vEdge, 1.0e-4, 1.0e-3, dInsertPt);
				}
			}
		}
		if(nLin2DInt!=1) nLin2DInt = mathIntLin2D(dBeginPt, dDir0, dEndPt, dDir1, 1.0e-4, 1.0e-3, dInsertPt);

		//以下处理非水平锋利边
		pDel = pArcBegin->next;
		while(pDel)
		{//删掉边界检查模型上的路径点
			if(pDel==pArcEnd) {break;}
			CSmtCutPoint * pNext = pDel->next;
			pPath->DeletePoint(pDel);
			pDel = pNext;
		}
		if( nLin2DInt==1 && VerifyLocalPath(dBeginPt, dInsertPt, DriveMdl)
			&& VerifyLocalPath(dEndPt, dInsertPt, DriveMdl) 
			&& nc_Distance(dInsertPt, dBeginPt,2)<3.0*dToolR 
			&& nc_Distance(dInsertPt,dEndPt,2)<3.0*dToolR )
		{
			CSmtCutPoint * pNewPt = new CSmtCutPoint(dInsertPt[0], dInsertPt[1], dPathHeight);
			pNewPt->m_bOrgType = SMARTNC_CUTPNT_SHARPEDGE;
			pPath->InsertAfter(pNewPt,	pArcBegin);
		}
		else
		{//断开
			pArcBegin->m_bType = pArcEnd->m_bType = 0;
			//CSmtCutPath* pBrkHead = pPath->BreakAtNullPoint(0);
			CSmtCutPath* pBrkHead = BreakPathByPntType(*pPath, 0);
			int nCount = 0;
			while( pBrkHead )
			{
				CSmtCutPath * pNewPath = pBrkHead ;
				pBrkHead = pBrkHead->next ;
				pNewPath->next = pNewPath->prev = NULL ; 
				if(nCount == 0) {
					POSITION pInsertPos = AllPath.m_cAllPath.Find(pPath);
					if(pInsertPos)	AllPath.m_cAllPath.InsertBefore(pInsertPos, pNewPath) ;
				}
				else if(nCount == 2)	{
					pPath->ClearAllPoint();
					for(CSmtCutPoint * p = pNewPath->m_pHead; p; p=p->next)  pPath->AddTail(p->CopyMyself());
					delete pNewPath;  pNewPath = NULL;
				}
				else {
					delete pNewPath;  pNewPath = NULL;
				}
				nCount++;
			}
		}
	}
}

BOOL CFHeightKeepSharpEdge::IsEdgeSteep(CStrpt & Strpt)
{
	PNT3D* aEdgePts = Strpt.m_ps;
	if(aEdgePts)
	{
		for(int i=0; i<Strpt.m_np-1; i++)
		{
			if(!MathCam_IsLineSteep(aEdgePts[i],aEdgePts[i+1],ANGLE_TO_RADIAN(88)))
			{
				return FALSE;
			}
		}
		return TRUE;
	}
	return FALSE;
}

//水平方向投影
BOOL CFHeightKeepSharpEdge::CanHorPrjPntOnChkMdl(PNT3D Pnt,PNT3D Dir,
									 CSmtCheckMdl & CheckMdl, double dDist)
{
	Dir[2] = 0;
	CSmtCutPoint dBegin, dEnd;
	nc_VectorCopy(dBegin.m_fPoint, Pnt, 3);
	nc_VectorCopy(dEnd.m_fPoint, Pnt, 3);
	MathCam_MovePtAlongVec2D(Pnt, dBegin.m_fPoint, Dir, dDist);
	MathCam_MovePtAlongVec2D(Pnt, dEnd.m_fPoint, Dir, -dDist);
	if(CheckMdl.Is3AxPointOvercut(dBegin, 0) != CheckMdl.Is3AxPointOvercut(dEnd, 0))
	{
		return TRUE;
	}
	return FALSE;
}
void CFHeightKeepSharpEdge::ResetPathHeadTail(CSmtCutPath & Path)
{
	CSmtCutPoint *pFindPt = NULL;
	for(CSmtCutPoint *pPnt = Path.m_pHead; pPnt; pPnt = pPnt->next)
	{
		if(pPnt->m_bType != SMART_CUTPNT_ONEDGER )
		{
			pFindPt = pPnt;
			break;
		}
	}
	if(pFindPt && pFindPt->prev  )
	{
		if(nc_Distance(pFindPt->m_fPoint, pFindPt->prev->m_fPoint, 3) < 2)
			Path.SetClosedPathHead(pFindPt);
		else
		{
			CSmtCutPoint * pNewPt = new CSmtCutPoint();
			double dMoveDir[2];
			MathCam_GetUnitVec2D(pFindPt->prev->m_fPoint, pFindPt->m_fPoint, dMoveDir);
			MathCam_MovePtAlongVec2D(pFindPt->prev->m_fPoint, pNewPt->m_fPoint, dMoveDir, 1);
			pNewPt->m_fPoint[2] = pFindPt->m_fPoint[2];
			Path.InsertAfter(pNewPt, pFindPt->prev);
			Path.SetClosedPathHead(pNewPt);
		}
	}
}

BOOL CFHeightKeepSharpEdge::IsSmtCutPntOnSharpEdge(CSmtCutPoint *pPnt,CSmtCheckMdl * pEdgeChkMdl,
	double dEdgeMinZ,double dEdgeMaxZ, CEdgeChkMdl * pAtEdge)
{
	FPNT3D  dLBox[2] ;
	PNT3D dCurrPt, vPrjDir ;
	double dToolCenterPos;
	if(pPnt && pEdgeChkMdl && pAtEdge)
	{
		double dToolRadius = pEdgeChkMdl->m_pTool->m_fRadius;
		nc_InitBox3D( pPnt->m_fPoint, pPnt->m_fPoint, dLBox ) ;
		dLBox[0][2] = (TFLOAT)pEdgeChkMdl->m_fBottom ;
		dLBox[1][2] = 1.0e6;//(TFLOAT)DriveMdl.m_fBottom;//
		nc_ExpandBox3D( dLBox, 0.15f, FALSE ) ;
		pEdgeChkMdl->LabelCheckByBox( dLBox );
		nc_VectorCopy(dCurrPt, pPnt->m_fPoint, 3);
		if(IsPointOnEdge(*pPnt, *pEdgeChkMdl))
		{
			return TRUE;
		}
		dToolCenterPos = dCurrPt[2] + dToolRadius;
		if(pAtEdge->m_bIsSteep &&  dToolCenterPos>= dEdgeMinZ && dToolCenterPos<= dEdgeMaxZ)
		{
			nc_VectorMinus(dCurrPt, pAtEdge->m_dBegin, vPrjDir, 2);
			nc_Normalize(vPrjDir, 2);
			if(CanHorPrjPntOnChkMdl(dCurrPt, vPrjDir, *pEdgeChkMdl,0.002))
			{
				return TRUE;
			}
		}
	}
	return FALSE;
}

BOOL CFHeightKeepSharpEdge::KeepSharpEdge(CSmtCPathLib& AllPath,CSmtCutPath *pPath, 
			   JDNC_SETUP& /*SetupDef*/,CSmtCheckMdl &DriveMdl)
{
	if(pPath==NULL || pPath->m_pHead==NULL || pPath->m_pTail==NULL ||
		DriveMdl.m_pTool==NULL || pPath->m_nNumPnt<3 || pPath->GetCutMode()!=MINI_MILL_PATH)
	{
		return FALSE;
	}
	//pPath->InsertCPoint3D(0.1);
	INT_PTR nSEChkMdl = m_cSharpEdgeChkMdl.GetCount();
	double dEdgeMinZ, dEdgeMaxZ;
	for(INT_PTR i=0; i<nSEChkMdl; i++)
	{
		CEdgeChkMdl * pAtEdge = m_cSharpEdgeChkMdl.GetAt(i);
		if(pAtEdge==NULL)	{	continue; }
		CSmtCheckMdl * pEdgeChkMdl = pAtEdge->m_pEdgeChk;
		if(pEdgeChkMdl==NULL || pEdgeChkMdl->m_pTool==NULL)
		{
			continue; 
		}
		CSmtPtArr cEdegeRPnts;
		double dToolRadius = pEdgeChkMdl->m_pTool->m_fRadius;
		dEdgeMinZ = min(pAtEdge->m_dBegin[2], pAtEdge->m_dEnd[2]) - 0.02; 
		dEdgeMaxZ = max(pAtEdge->m_dBegin[2], pAtEdge->m_dEnd[2]) + dToolRadius + 0.02; 
		for(CSmtCutPoint *pPnt = pPath->m_pHead; pPnt; pPnt = pPnt->next)
		{
			if(pPnt->m_fPoint[3] == SMART_CUTPNT_ONHOZEDGE )continue;
			pPnt->m_bType = 100;
			if(IsSmtCutPntOnSharpEdge(pPnt, pEdgeChkMdl, dEdgeMinZ, dEdgeMaxZ, pAtEdge))
			{
				pPnt->m_bType = SMART_CUTPNT_ONEDGER;
				cEdegeRPnts.Add(pPnt);
			}
		}
		if(pPath->IsClosed() && fabs(pAtEdge->m_dBegin[2]-pAtEdge->m_dEnd[2])>0.02 &&
			cEdegeRPnts.GetCount()>2 && pPath->m_pHead->m_bType == SMART_CUTPNT_ONEDGER)
		{
			ResetPathHeadTail(*pPath);
			cEdegeRPnts.RemoveAll();
			for(CSmtCutPoint *pPnt = pPath->m_pHead; pPnt; pPnt = pPnt->next)	
			{
				pPnt->m_bType = 100;
				if(IsSmtCutPntOnSharpEdge(pPnt, pEdgeChkMdl, dEdgeMinZ, dEdgeMaxZ, pAtEdge)){
					pPnt->m_bType = SMART_CUTPNT_ONEDGER;
					cEdegeRPnts.Add(pPnt);
				}
			}
		}
		ReconstructPathAtEdgeR(AllPath, pPath, *pAtEdge, DriveMdl, cEdegeRPnts);
	}
	return TRUE;
}

void CFHeightKeepSharpEdge::KeepSharpEdges(CSmtCPathLib& AllPath, CPtrList & SharpEdges,
			   JDNC_SETUP& SetupDef,JDNC_STOCKEX & StockDef,CSmtCheckMdl &DriveMdl)
{
	if(SharpEdges.GetCount()<1 || fabs(StockDef.m_dDriveOffset[0]) > 0.001 ||
		DriveMdl.m_pTool==NULL )
	{//加工余量必须为0，否则没有意义
		return;
	}
	ClearAll();
	CSmartTool * pToolCpy = DriveMdl.m_pTool->CopyMyself();
	GenSharpEdgeChkMdl(SharpEdges, pToolCpy);
	if(m_cSharpEdgeChkMdl.GetCount() > 0 )
	{
		POSITION pAtPos = AllPath.m_cAllPath.GetHeadPosition();
		while(pAtPos)
		{
			CSmtCutPath * pCurrPath = AllPath.m_cAllPath.GetNext(pAtPos);
			KeepSharpEdge(AllPath, pCurrPath, SetupDef, DriveMdl );
		}
		AllPath.DefineBox();
	}
	delete pToolCpy;
	pToolCpy = NULL;

}

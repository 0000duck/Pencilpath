#include "StdAfx.H"
#include "SmartNC.H"
#include "SysPrompt.h"
#include "SmartSurf.H"
#include "SurfGeo.h"
#include "CurveFair.H"
#include "geo_curve.h"
#include "FFollowNoMosaic.h"


#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

static void MathCam_DrawOneLine(double Pnt[],double dVecEnd[])
{
	CModel* pModel = glbf_GetModel();
	pModel->BeginUndo("test no mosaic");
	CGeoLine *pGeoLine = new CGeoLine(Pnt,dVecEnd);
	CEttCurve* pEttCur = new CEttCurve ( pGeoLine ) ;
	pModel->UD_AddEntity(pEttCur);
	pModel->EndUndo();
}
static void MathCam_DrawOneLine(double Pnt[],float dVecEnd[])
{
	CModel* pModel = glbf_GetModel();
	pModel->BeginUndo("test no mosaic");
	double dEnd[3];
	nc_VectorCopy(dEnd, dVecEnd, 3);
	CGeoLine *pGeoLine = new CGeoLine(Pnt, dEnd);
	CEttCurve* pEttCur = new CEttCurve ( pGeoLine ) ;
	pModel->UD_AddEntity(pEttCur);
	pModel->EndUndo();
}
static void MathCam_DrawOneVec(PNT3D Pnt,double Dir[])
{
	double dVecEnd[3];
	CModel* pModel = glbf_GetModel();
	pModel->BeginUndo("test no mosaic");
	dVecEnd[0] = Pnt[0] + Dir[0]*0.5;
	dVecEnd[1] = Pnt[1] + Dir[1]*0.5;
	dVecEnd[2] = Pnt[2] + Dir[2]*0.5;
	CGeoLine *pGeoLine = new CGeoLine(Pnt,dVecEnd);
	CEttCurve* pEttCur = new CEttCurve ( pGeoLine ) ;
	pModel->UD_AddEntity(pEttCur);
	pModel->EndUndo();
}
static void MathCam_DrawOneVec(PNT3D Pnt,float Dir[])
{
	double dVecEnd[3];
	CModel* pModel = glbf_GetModel();
	pModel->BeginUndo("test no mosaic");
	dVecEnd[0] = Pnt[0] + Dir[0]*0.5;
	dVecEnd[1] = Pnt[1] + Dir[1]*0.5;
	dVecEnd[2] = Pnt[2] + Dir[2]*0.5;
	CGeoLine *pGeoLine = new CGeoLine(Pnt,dVecEnd);
	CEttCurve* pEttCur = new CEttCurve ( pGeoLine ) ;
	pModel->UD_AddEntity(pEttCur);
	pModel->EndUndo();
}
static void MathCam_DrawPolyLine(PNT3D aPts[], int nPts)
{
	if(aPts==NULL || nPts<2)
		return;

	CGeoPLine3d* pGeoPLine3D = new CGeoPLine3d(nPts-1);
	for(int i = 0 ; i<nPts ; i++)
	{
		nc_VectorCopy(pGeoPLine3D->m_dPoint[i], aPts[i], 3);
	}
	CModel* pModel = glbf_GetModel();
	pModel->BeginUndo("test no mosaic");
	CEttCurve* pEttCur = new CEttCurve ( pGeoPLine3D ) ;
	pModel->UD_AddEntity(pEttCur);
	pModel->EndUndo();
}
static void MathCam_DrawDirPolyLine(CDirPoint aPts[], int nPts)
{
	if(aPts==NULL || nPts<2)
		return;

	CGeoPLine3d* pGeoPLine3D = new CGeoPLine3d(nPts-1);
	for(int i = 0 ; i<nPts ; i++)
	{
		nc_VectorCopy(pGeoPLine3D->m_dPoint[i], aPts[i].m_fPoint, 3);
	}
	CModel* pModel = glbf_GetModel();
	pModel->BeginUndo("test no mosaic");
	CEttCurve* pEttCur = new CEttCurve ( pGeoPLine3D ) ;
	pModel->UD_AddEntity(pEttCur);
	pModel->EndUndo();
	for(int i=0; i<nPts; i++)
	{
		MathCam_DrawOneVec(aPts[i].m_fPoint, aPts[i].m_fDir);
	}
}
static double MathCam_GetPntSegmNearestPnt(double P[], double Begin[], double End[], 
									 double Nearest_p[] ,double & ft)
{
    double v1[3], v2[3] ;
    double fA, fB  ; 

    v1[0] = End[0] - Begin[0] ;
	v1[1] = End[1] - Begin[1] ;
	v1[2] = End[2] - Begin[2] ;
    v2[0] = Begin[0] - P[0] ;
	v2[1] = Begin[1] - P[1] ;
	v2[2] = Begin[2] - P[2] ;
    fA = v1[0] * v1[0] + v1[1] * v1[1] + v1[2] * v1[2] ;
    fB = v1[0] * v2[0] + v1[1] * v2[1] + v1[2] * v2[2] ;

    ft = ( fA > INF_DBL ? mathClamp01 ( -fB / fA ) : 0.0 ) ;
    Nearest_p[0] = Begin[0] + ft * v1[0] ;
    Nearest_p[1] = Begin[1] + ft * v1[1] ;
    Nearest_p[2] = Begin[2] + ft * v1[2] ;

	return nc_Distance( P, Nearest_p,3 ) ;
}
//计算三角形内角
static double MathCam_GetTriVertAng(CSmtCutPoint * pPrev,
							 CSmtCutPoint * pPoint,
							 CSmtCutPoint * pNext)
{
	if(pPrev && pPoint && pNext)
	{
		double a, b, c, dCosAngle;
		b = nc_Distance(pPrev->m_fPoint, pPoint->m_fPoint, 3);
		c = nc_Distance(pNext->m_fPoint, pPoint->m_fPoint, 3);
		if(b>0.0001 && c>0.0001)
		{
			a = nc_Distance(pPrev->m_fPoint,pNext->m_fPoint,3);
			dCosAngle=(b*b + c*c- a*a)/(2.0*b*c);
			return acos(dCosAngle);
		}
	}
	return -1;
}


#define  SMART_CUTPNT_SHARP -10
static void MathCam_BreakPathByAngle(CSmtCutPath & Path, double Angle,
									 CTypedPtrArray<CPtrArray,CSmtCutPath*> & BrkPathArr)
{
	double dCurrAng;
	for(CSmtCutPoint *p=Path.m_pHead; p; p=p->next)
	{
		dCurrAng = MathCam_GetTriVertAng(p->prev,p,p->next);
		if(dCurrAng>0 && dCurrAng<ANGLE_TO_RADIAN(Angle))
		{
			p->m_bType = SMART_CUTPNT_SHARP;
		}
		else if(p->prev && p->next && 
			nc_Distance(p->m_fPoint, p->next->m_fPoint,3)>5)
		{
			p->m_bType = SMART_CUTPNT_SHARP;
		}
		else if(p->prev && p->next && 
			nc_Distance(p->m_fPoint, p->prev->m_fPoint,3)>5)
		{
			p->m_bType = SMART_CUTPNT_SHARP;
		}
	}

	CSmtCutPath * pNewPath = new CSmtCutPath();
	for(CSmtCutPoint *p=Path.m_pHead; p; p=p->next)
	{
		pNewPath->AddPoint(p->m_fPoint);
		if(p==Path.m_pTail)
		{
			BrkPathArr.Add(pNewPath);
			break;
		}
		if(p->m_bType == SMART_CUTPNT_SHARP)
		{
			BrkPathArr.Add(pNewPath);
			pNewPath = new CSmtCutPath();
			pNewPath->AddPoint(p->m_fPoint);
		}
	}	
}

static void MathCam_BreakPathByLen(CSmtCutPath & Path, double Len,
									 CTypedPtrArray<CPtrArray,CSmtCutPath*> & BrkPathArr)
{
	double dDist = 0;
	CSmtCutPath * pNewPath = new CSmtCutPath();
	for(CSmtCutPoint *p=Path.m_pHead; p && p->next; p=p->next)
	{
		pNewPath->AddPoint(p->m_fPoint);
		dDist += nc_Distance(p->m_fPoint, p->next->m_fPoint, 3);
		if(p->next == Path.m_pTail)
		{
			pNewPath->AddPoint(p->next->m_fPoint);
			BrkPathArr.Add(pNewPath);
			break;
		}
		else if(dDist > Len)
		{
			dDist = 0;
			pNewPath->AddPoint(p->next->m_fPoint);
			BrkPathArr.Add(pNewPath);
			pNewPath = new CSmtCutPath();
		}
		
	}
}
static void MathCam_CalMidPnt(FPNT3D dBegin,FPNT3D dEnd,FPNT3D dMid)
{
	for(int i=0; i<3; i++) 
	{
		dMid[i] = TFLOAT( (dBegin[i] + dEnd[i])*0.5 );
	}
}
static void MathCam_CopyBox3D(PNT3D SrcBox[2],FPNT3D DestBox[2])
{
	nc_VectorCopy(DestBox[0], SrcBox[0], 3);
	nc_VectorCopy(DestBox[1], SrcBox[1], 3);
}

void CPNT3DBlock::DefineBox()
{
	for(INT_PTR i=0; i<m_cAllPnt.GetCount(); i++)
	{
		double * p = m_cAllPnt.GetAt(i);
		if(0 == i)
		{
			nc_VectorCopy(m_fBox[0],  p, 3);
			nc_VectorCopy(m_fBox[1],  p, 3);
			continue;
		}
		for(int j=0; j<3; j++)
		{
			m_fBox[0][j] = (TFLOAT)min( m_fBox[0][j] , p[j] );
			m_fBox[1][j] = (TFLOAT)max( m_fBox[1][j] , p[j] );
		}
	}
}

void CDirPointBlock::DefineBox()
{
	for(INT_PTR i=0; i<m_cAllPnt.GetCount(); i++)
	{
		CDirPoint * p = m_cAllPnt.GetAt(i);
		if(0 == i)
		{
			nc_VectorCopy(m_fBox[0],  p->m_fPoint, 3);
			nc_VectorCopy(m_fBox[1],  p->m_fPoint, 3);
			continue;
		}
		for(int j=0; j<3; j++)
		{
			m_fBox[0][j] = (TFLOAT)min( m_fBox[0][j] , p->m_fPoint[j] );
			m_fBox[1][j] = (TFLOAT)max( m_fBox[1][j] , p->m_fPoint[j] );
		}
	}
}
CPathSmooth::CPathSmooth()
{
	m_nMaxCheckNum = 100;
	m_aBuff = new PNT3D [PSMOOTH_BUFFSIZE];
}
CPathSmooth::~CPathSmooth()
{
	if(m_aBuff)
	{
		delete [] m_aBuff;
		m_aBuff = NULL;
	}
}
CGeoSpline * CPathSmooth::Smooth(CSmtCutPath & Path, double SmoothTol)
{
	CGeoSpline * pSpl = CreateFairSpline(Path);
	if(pSpl)
	{
		SmoothSpline(pSpl, SmoothTol);
	}
	return pSpl;
}
CGeoSpline * CPathSmooth::Smooth(PNT3D AllPt[],int PtCnt, double SmoothTol)
{
	CGeoSpline *spline = NULL;
	if(AllPt && PtCnt>2)
	{
		spline = new CGeoSpline() ;
		BOOL bClosed = 0;
		if(nc_Distance(AllPt[0], AllPt[PtCnt-1], 3)<0.002)
		{
			bClosed = 1;
		}
		if( spline->CreateByCubicInterp( PtCnt-1, AllPt, bClosed, -1))
		{
			SmoothSpline(spline, SmoothTol);
		}
		else
		{
			delete spline ;
			spline = NULL ;
		}
	}
	return spline;
}
CGeoSpline * CPathSmooth::Smooth(CSmtCutPath & Path, double SmoothTol, FPNT3D tan0,
		FPNT3D tan1)
{
	if(Path.m_nNumPnt >= PSMOOTH_BUFFSIZE)
	{
		return NULL;
	}

	if(tan0==NULL && tan1==NULL)
	{
		return Smooth(Path, SmoothTol);
	}
	int np = Path.m_nNumPnt ; 
	CSmtCutPoint* pTPnt = NULL  ;
	np = 0 ; 
	for( pTPnt = Path.m_pHead ; pTPnt ; pTPnt = pTPnt->next )  
	{
		nc_FloatToDouble( m_aBuff[np], pTPnt->m_fPoint, 3 ) ;
		np ++ ;
	}
	CGeoSpline *spline = new CGeoSpline() ;
	VEC3D tan[2];
	if(tan0==NULL || tan1==NULL)
	{
		nbCalcOpenTang(np-1, m_aBuff, tan);
	}
	if(tan0)
		nc_VectorCopy(tan[0], tan0, 3);
	if(tan1)
		nc_VectorCopy(tan[1], tan1, 3);

    spline->CreateByInterpPsET(np-1, m_aBuff, tan ) ;
	SmoothSpline(spline, SmoothTol);
	return spline;
}
int CPathSmooth::SmoothSpline( CGeoSpline* &Spline,double tolerance) 
{
    CCurveFair fair ;
	fair.SetCheckPntNum(m_nMaxCheckNum);
    if( Spline->IsClosed() )
    {
        CGeoSpline *pNew = fair.FairPeriodicGeoSpline( Spline, tolerance );
        if( pNew )
        {
            delete Spline ;
            Spline = pNew ;
        }
    }
    else
    {
        fair.FairGeoSpline( Spline, tolerance, CURVEFAIR_FIXENDTAN,CURVEFAIR_FIXENDTAN ) ;
    }
    return TRUE ;
}
CGeoSpline* CPathSmooth::CreateFairSpline( CSmtCutPath& CutPath)
{
    // STEP 1 : 统计点数
	int np = CutPath.m_nNumPnt ; 
    if( np < 3 || np>=PSMOOTH_BUFFSIZE) return NULL ; 
	CSmtCutPoint* pTPnt = NULL  ;
    np = 0 ; 
    for( pTPnt = CutPath.m_pHead ; pTPnt ; pTPnt = pTPnt->next )
    {
        nc_FloatToDouble( m_aBuff[np], pTPnt->m_fPoint, 3 ) ;
        np ++ ;
    }
    // STEP 2 : 插值数据点
    CGeoSpline *spline = new CGeoSpline() ;
    BOOL bClosed = CutPath.IsClosed() ; 
    BOOL bOk = spline->CreateByCubicInterp( np-1, m_aBuff, bClosed, -1) ;
    if( bOk == FALSE ) 
    {
        delete spline ;
        return NULL ;
    }
    return spline ;
}

C3DStepNomosaic::C3DStepNomosaic()
{
	SurfNC_InitPathParam ( m_cCalTol ) ;
	m_cCalTol.m_dArcTol = 0.001 ;
	m_nRecursive = 0;
	m_pCurrPathCpy = NULL;
}
C3DStepNomosaic::~C3DStepNomosaic()
{
}

void C3DStepNomosaic::MovePtAlongVec3D(double OrigPt[],double DestPt[],
									   VEC3D Dir,double Dist)
{
	for(int i=0; i<3; i++)
	{
		DestPt[i] = OrigPt[i] + Dir[i]*Dist;
	}
}

void C3DStepNomosaic::MovePtAlongVec3D(double OrigPt[],float DestPt[],
									   float Dir[],double Dist)
{
	for(int i=0; i<3; i++)
	{
		DestPt[i] = float(OrigPt[i] + Dir[i]*Dist);
	}
}
void C3DStepNomosaic::MovePtAlongVec3D(float OrigPt[],float DestPt[],
									   VEC3D Dir,double Dist)
{
	for(int i=0; i<3; i++)
	{
		DestPt[i] = float(OrigPt[i] + Dir[i]*Dist);
	}
}
void C3DStepNomosaic::MovePtAlongUnitVec3D(PNT3D OrigPt,PNT3D DestPt,
									   VEC3D Dir)
{
	for(int i=0; i<3; i++)
	{
		DestPt[i] = OrigPt[i] + Dir[i];
	}
}

void C3DStepNomosaic::SplitPntToBlocks(PNT3D AllPnt[],int PntCnt,
										double dBlockLen,CPNT3DBlkArr & PtBlock)
{
	double dDist2D = 0;
	CPtrList cOneBlock;
	for(int i=0; i<PntCnt; i++)
	{
		cOneBlock.AddTail(&AllPnt[i]);
		if(i+1 < PntCnt)
		{
			dDist2D += nc_Distance(AllPnt[i], AllPnt[i+1], 2);
		}

		if(dDist2D>dBlockLen || i==PntCnt-1)
		{
			CPNT3DBlock * pNew = new CPNT3DBlock();
			while(cOneBlock.GetCount())
			{
				double * pPt = (double*)cOneBlock.RemoveHead();
				pNew->m_cAllPnt.Add(pPt);
			}
			pNew->DefineBox();
			PtBlock.Add(pNew);
			dDist2D = 0;
		}

	}
}
void C3DStepNomosaic::SplitPntToBlocks(CDirPoint AllPnt[],int PntCnt,
										double dBlockLen,CDirPntBlkArr & PtBlock)
{
	double dDist2D = 0;
	CPtrList cOneBlock;
	for(int i=0; i<PntCnt; i++)
	{
		cOneBlock.AddTail(&AllPnt[i]);
		if(i+1 < PntCnt)
		{
			dDist2D += nc_Distance(AllPnt[i].m_fPoint, AllPnt[i+1].m_fPoint, 2);
		}
		if(dDist2D>dBlockLen || i==PntCnt-1)
		{
			CDirPointBlock * pNew = new CDirPointBlock();
			INT_PTR nBlock = PtBlock.GetCount();
			if( nBlock>0 )
			{//加入上块点的末点，方便以后的插点投影计算
				CDirPointBlock * pLastBlock = PtBlock.GetAt(nBlock-1);
				CTypedPtrArray<CPtrArray,CDirPoint*> &cLastPtArr = pLastBlock->m_cAllPnt;
				INT_PTR nPoint = cLastPtArr.GetCount();
				cOneBlock.AddHead(cLastPtArr.GetAt(nPoint-1));
			}
			while(cOneBlock.GetCount())
			{
				CDirPoint * pPt = (CDirPoint*)cOneBlock.RemoveHead();
				pNew->m_cAllPnt.Add(pPt);
			}
			pNew->DefineBox();
			PtBlock.Add(pNew);
			dDist2D = 0;
		}
	}
}


PNT3D * C3DStepNomosaic::GetSplinePntByStep(CGeoSpline & Spline,double Step,
										  int & nPnt)
{
	double dLen = Spline.GetLength();
	nPnt = int(dLen/Step) + 1;
	if(nPnt < 10 )
	{
		nPnt = 10;
	}
	PNT3D * aSmoothPts = new PNT3D [nPnt];
	double dInc = 1.0 /double(nPnt-1);
	for(int i=0; i<nPnt; i++)
	{
		Spline.GetPoint(i*dInc, aSmoothPts[i]);
	}
	return aSmoothPts;
}
PNT3D * C3DStepNomosaic::GetSplinePntByPtCnt(CGeoSpline & Spline,int nPnt)
{
	if(nPnt < 1 )
	{
		return NULL;
	}
	PNT3D * aSmoothPts = new PNT3D [nPnt];
	double dInc = 1.0 /double(nPnt-1);
	for(int i=0; i<nPnt; i++)
	{
		Spline.GetPoint(i*dInc, aSmoothPts[i]);
	}
	return aSmoothPts;
}

void C3DStepNomosaic::GetNearestPathPt(double pt[3],CSmtCutPath * pPath,
								  FPNT3D & nearest_p/*,PNT3D Tangent[]*/)
{
	if(!pPath)
		return ;

	//Tangent[0][0] = Tangent[0][1] = Tangent[0][2] = 0;
	//Tangent[1][0] = Tangent[1][1] = Tangent[1][2] = 0;
	PNT3D begin,end,curnearestp/*, dTempPt*/;
	double dNearestDis = 99999999.0;
	double u;
	for(CSmtCutPoint * p=pPath->m_pHead;p&&p->next;p=p->next)
	{
		nc_VectorCopy(begin, p->m_fPoint, 3);
		nc_VectorCopy(end  , p->next->m_fPoint, 3);
		double curDis = MathCam_GetPntSegmNearestPnt(pt,begin,end,
			curnearestp, u);
		if(curDis<dNearestDis)
		{
			dNearestDis = curDis;
			nc_VectorCopy(nearest_p, curnearestp, 3);
#if 0
			if(fabs(u) < 1.0e-3)
			{
				if(p->prev)
				{
					nc_VectorCopy(dTempPt, p->prev->m_fPoint, 3);
					nc_VectorMinus(dTempPt, begin, Tangent[0], 3 );
					nc_VectorMinus(begin, end, Tangent[1], 3 );
				}
				else
				{
					Tangent[0][0] = Tangent[0][1] = Tangent[0][2] = 0;
					nc_VectorMinus(begin, end, Tangent[1], 3 );
				}
			}
			else if(fabs(u - 1) < 1.0e-3)
			{
				if(p->next->next)
				{
					nc_VectorCopy(dTempPt, p->next->next->m_fPoint, 3);
					nc_VectorMinus(end, dTempPt, Tangent[1], 3 );
					nc_VectorMinus(begin, end, Tangent[0], 3 );
				}
				else
				{
					Tangent[1][0] = Tangent[1][1] = Tangent[1][2] = 0;
					nc_VectorMinus(begin, end, Tangent[0], 3 );
				}
			}
			else
			{
				Tangent[1][0] = Tangent[1][1] = Tangent[1][2] = 0;
				nc_VectorMinus(begin, end, Tangent[0], 3 );
			}
#endif
		}
	}
}
void C3DStepNomosaic::GetPtNorm(float fCurrPt[3],float fNorm[3],CSmtCheckMdl & CheckMdl)
{
	PNT3D dPnt, dVec;
	nc_VectorCopy(dPnt, fCurrPt, 3);
	GetPtNorm(dPnt, dVec, CheckMdl);
	nc_VectorCopy(fNorm, dVec, 3);
}
void C3DStepNomosaic::GetPtNorm(double fCurrPt[3],double fNorm[3],CSmtCheckMdl & CheckMdl)
{
	//得到对应的原始路径上的点和切矢
	FPNT3D fOrigPathPt, fStart, fEnd ;
	GetNearestPathPt(fCurrPt, m_pCurrPathCpy, fOrigPathPt/*, vPtTan*/);
	FPNT3D aPrjVec[2];
	for(int j=0; j<2; j++)
	{
		nc_VectorCopy(fStart, fOrigPathPt, 3);
		nc_VectorCopy(fEnd,   fOrigPathPt, 3);
		fStart[2] = fEnd[2] = CheckMdl.m_fBottom;
		fStart[j] -= 0.01f;
		fEnd[j]   += 0.01f;
		CheckMdl.DefineHeight(fStart);
		CheckMdl.DefineHeight(fEnd);
		nc_VectorMinus(fStart, fEnd, aPrjVec[j], 3 );
	}

	FPNT3D vProduct;
	nc_VProduct(aPrjVec[0], aPrjVec[1], vProduct);
	nc_Normalize( vProduct, 3 ) ;
	if(vProduct[2]<0)
	{
		nc_VectorReverse(vProduct, 3);
	}
	nc_VectorCopy(fNorm, vProduct, 3);
}
void C3DStepNomosaic::GetPathPtNorm(FPNT3D fCurrPt,FPNT3D fNorm,CSmtCheckMdl & CheckMdl)
{
	FPNT3D fStart, fEnd;
	FPNT3D aPrjVec[2];
	for(int j=0; j<2; j++)
	{
		nc_VectorCopy(fStart, fCurrPt, 3);
		nc_VectorCopy(fEnd,   fCurrPt, 3);
		fStart[2] = fEnd[2] = CheckMdl.m_fBottom;
		fStart[j] -= 0.01f;
		fEnd[j]   += 0.01f;
		CheckMdl.DefineHeight(fStart);
		CheckMdl.DefineHeight(fEnd);
		nc_VectorMinus(fStart, fEnd, aPrjVec[j], 3 );
	}

	nc_VProduct(aPrjVec[0], aPrjVec[1], fNorm);
	nc_Normalize( fNorm, 3 ) ;
	if(fNorm[2]<0)
	{
		nc_VectorReverse(fNorm, 3);
	}
}
PNT3D * C3DStepNomosaic::GetAllPtNorm(PNT3D aSmoothPts[],int nPnt,
									CSmtCheckMdl & CheckMdl)
{
	PNT3D * aPtNorms = new PNT3D [nPnt];
	FPNT3D fBox3D[2];
	FPNT3D fStart, fEnd, fNorm;
	FPNT3D aPrjVec[2];

#if 1
	//将点分块减少LabelCheck次数
	CPNT3DBlkArr cSmoothedPtBlk;
	SplitPntToBlocks(aSmoothPts, nPnt, 8, cSmoothedPtBlk);
	int nCount = 0;
	for(INT_PTR i=0; i<cSmoothedPtBlk.GetCount(); i++)
	{
		CPNT3DBlock * pCurrBlk = cSmoothedPtBlk.GetAt(i);
		if(pCurrBlk==NULL) {continue;}
		memcpy(fBox3D, pCurrBlk->m_fBox, sizeof(FPNT3D[2]));
		nc_ExpandBox3D( fBox3D, 0.05f, FALSE ) ;
		fBox3D[0][2] = CheckMdl.m_fBottom ;
		fBox3D[1][2] = 1.0e6;
		//为了陡峭处法矢量计算的准确性，Labelbox深度要够
		CheckMdl.LabelCheckByGroupBox3D( fBox3D );
		for(INT_PTR j=0; j<pCurrBlk->m_cAllPnt.GetCount(); j++)
		{
			double *pPoint = pCurrBlk->m_cAllPnt.GetAt(j);
			if(pPoint==NULL) continue;
			nc_VectorCopy(fBox3D[0], pPoint, 3);
			nc_VectorCopy(fBox3D[1], pPoint, 3);
			nc_ExpandBox3D( fBox3D, 0.03f, FALSE ) ;
			fBox3D[0][2] = CheckMdl.m_fBottom ;
			fBox3D[1][2] = 1.0e6;
			CheckMdl.LabelCheckByFacetBox3D( fBox3D );

			for(int k=0; k<2; k++)
			{
				nc_VectorCopy(fStart, pPoint, 3);
				nc_VectorCopy(fEnd,   pPoint, 3);
				fStart[2] = fEnd[2] = CheckMdl.m_fBottom;
				fStart[k] -= 0.01f;
				fEnd[k]   += 0.01f;
				CheckMdl.DefineHeight(fStart);
				CheckMdl.DefineHeight(fEnd);
				nc_VectorMinus(fStart, fEnd, aPrjVec[k], 3 );
			}
			nc_VProduct(aPrjVec[0], aPrjVec[1], fNorm);
			nc_Normalize( fNorm, 3 ) ;
			if(fNorm[2]<0)
			{
				nc_VectorReverse(fNorm, 3);
			}
			nc_VectorCopy(aPtNorms[nCount], fNorm, 3);
			nCount++;
		}
		delete pCurrBlk;
		pCurrBlk = NULL;
	}
#else
	for(int i=0; i<nPnt; i++)
	{
		nc_VectorCopy(fCurrPt, aSmoothPts[i], 3);
		nc_InitBox3D( fCurrPt, fCurrPt, fBox3D ) ;
		nc_ExpandBox3D( fBox3D, 0.05f, FALSE ) ;
		fBox3D[0][2] = CheckMdl.m_fBottom ;
		fBox3D[1][2] = 1.0e6;
		CheckMdl.LabelCheckByFacetBox3D( fBox3D );
		
		for(int j=0; j<2; j++)
		{
			nc_VectorCopy(fStart, aSmoothPts[i], 3);
			nc_VectorCopy(fEnd,   aSmoothPts[i], 3);
			fStart[2] = fEnd[2] = CheckMdl.m_fBottom;
			fStart[j] -= 0.01f;
			fEnd[j]   += 0.01f;
			CheckMdl.DefineHeight(fStart);
			CheckMdl.DefineHeight(fEnd);
			nc_VectorMinus(fStart, fEnd, aPrjVec[j], 3 );
		}

		nc_VProduct(aPrjVec[0], aPrjVec[1], fNorm);
		nc_Normalize( fNorm, 3 ) ;
		if(fNorm[2]<0)
		{
			nc_VectorReverse(fNorm, 3);
		}
		nc_VectorCopy(aPtNorms[i], fNorm, 3);
	}
#endif

	return aPtNorms;
}


void C3DStepNomosaic::DeletePntArr(PNT3D * & PtArr)
{
	if(PtArr)
	{
		delete [] PtArr;
		PtArr = NULL;
	}
}
void C3DStepNomosaic::DeleteSpline(CGeoSpline * & Spline)
{
	if(Spline)
	{
		delete Spline;
		Spline = NULL;
	}
}

CDirPoint * C3DStepNomosaic::GetDirPntByStep(CGeoSpline & SmoothedPt,
											 CGeoSpline & SmoothedVec,
											 double Step, int & nPnt)
{
	nPnt = int( SmoothedPt.GetLength()/Step ) + 1;
	if(nPnt < 10 ) nPnt = 10;
	CDirPoint * aDirPt = new CDirPoint [nPnt];
	PNT3D dSmoothedPt, dSmoothedVec, vNormDir ;
	double dInc = 1.0 /double(nPnt-1);
	double u;
	for(int i=0; i<nPnt; i++)
	{
		u = i * dInc;
		aDirPt[i].m_dU = u;
		SmoothedPt.GetPoint(u, dSmoothedPt);
		nc_VectorCopy(aDirPt[i].m_fPoint, dSmoothedPt, 3);

		SmoothedVec.GetPoint(u, dSmoothedVec);
		nc_VectorMinus(dSmoothedPt, dSmoothedVec, vNormDir, 3 );
		nc_Normalize ( vNormDir, 3 ) ;
		nc_VectorCopy(aDirPt[i].m_fDir,  vNormDir, 3);
	}
	return aDirPt;
}
void C3DStepNomosaic::ClearPntBlockArr(CDirPntBlkArr & cPtBlock)
{
	for(INT_PTR i=0; i<cPtBlock.GetCount(); i++)
	{
		CDirPointBlock * pPBlk = cPtBlock.GetAt(i);
		if(pPBlk)
		{
			delete pPBlk;
			pPBlk = NULL;
		}
	}
	cPtBlock.RemoveAll();
}

BOOL C3DStepNomosaic::PrjPntOnChkMdl(PNT3D Pnt,FPNT3D Dir,
									 CSmtCheckMdl & CheckMdl, PNT3D PrjPt)
{
	CSmtCutPoint dBegin, dEnd, dMid;
	nc_VectorCopy(dBegin.m_fPoint, Pnt, 3);
	double PrjDir[3];
	nc_VectorCopy(PrjDir, Dir, 3);
	BOOL bOverCut = CheckMdl.Is3AxPointOvercut(dBegin, 0);
	if(!bOverCut)	nc_VectorReverse(PrjDir, 3);
	//有时路径点距离检查模型较远（尤其光滑尖角半径较大的时候），
	//投影距离适当大点否则可能有的地方投影失败导致产生尖点
	double dDist = 0.09;
	MovePtAlongVec3D(dBegin.m_fPoint, dEnd.m_fPoint, PrjDir, dDist);

	//查找过切状态不同的终点
	if( bOverCut == CheckMdl.Is3AxPointOvercut(dEnd, 0) )
	{
		MovePtAlongVec3D(dBegin.m_fPoint, dEnd.m_fPoint, PrjDir, -dDist);
		if( bOverCut == CheckMdl.Is3AxPointOvercut(dEnd, 0) )
		{//异常情况
			nc_VectorCopy(PrjPt, Pnt, 3);
			return FALSE;
		}
	}

	
	int nTimes = 0;
	while(nTimes<50)
	{
		nTimes++;
		MathCam_CalMidPnt(dBegin.m_fPoint, dEnd.m_fPoint, dMid.m_fPoint);
		if(dDist < 0.0002)  
		{
			break; 
		}
		if( CheckMdl.Is3AxPointOvercut(dMid, 0) != bOverCut )
			nc_VectorCopy(dEnd.m_fPoint, dMid.m_fPoint, 3);
		else
			nc_VectorCopy(dBegin.m_fPoint, dMid.m_fPoint, 3);
		dDist *= 0.5;
	}
	nc_VectorCopy(PrjPt, dMid.m_fPoint, 3);
	return TRUE;
}

void C3DStepNomosaic::OutputDebugInfo(CDirPntBlkArr & cPtBlock)
{
	double dVecEnd[3];
	CModel* pModel = glbf_GetModel();
	pModel->BeginUndo("test no mosaic");
	for(INT_PTR i=0; i<cPtBlock.GetCount(); i++)
	{
		CDirPointBlock * pCurrBlk = cPtBlock.GetAt(i);
		if(pCurrBlk==NULL) {continue;}
		for(INT_PTR j=0; j<pCurrBlk->m_cAllPnt.GetCount(); j++)
		{
			CDirPoint *pPoint = pCurrBlk->m_cAllPnt.GetAt(j);
			dVecEnd[0] = pPoint->m_fPoint[0] + pPoint->m_fDir[0]*0.5;
			dVecEnd[1] = pPoint->m_fPoint[1] + pPoint->m_fDir[1]*0.5;
			dVecEnd[2] = pPoint->m_fPoint[2] + pPoint->m_fDir[2]*0.5;
			CGeoLine *pGeoLine = new CGeoLine(pPoint->m_fPoint,dVecEnd);
			CEttCurve* pEttCur = new CEttCurve ( pGeoLine ) ;
			pModel->UD_AddEntity(pEttCur);
		}
	}
	pModel->EndUndo();
}

void C3DStepNomosaic::ClearPathArr(CTypedPtrArray<CPtrArray,CSmtCutPath*> & cBrkPathArr)
{
	for(INT_PTR nPath=0; nPath<cBrkPathArr.GetCount(); nPath++)
	{
		CSmtCutPath *pPath = cBrkPathArr.GetAt(nPath);
		if(pPath)
		{
			delete pPath;
			pPath = NULL;
		}
	}
	cBrkPathArr.RemoveAll();
}


void C3DStepNomosaic::RecursiveAddMidPnt(CSmtCutPath * pPath,CSmtCutPoint * p,
										 CSmtCutPoint * pNext, CSmtCheckMdl & CheckMdl)
{
	m_nRecursive++;
	if(m_nRecursive>20 || nc_Distance(p->m_fPoint, pNext->m_fPoint, 3)<0.0001) 
	{ 
		return;
	}

	FPNT3D dMid, fPtNorm;
	MathCam_CalMidPnt(p->m_fPoint, pNext->m_fPoint, dMid);
	GetPtNorm(dMid, fPtNorm,  CheckMdl);
	PNT3D dCurrPt, dPrjPos, dBegin , dEnd;
	nc_VectorCopy(dCurrPt,  dMid, 3);
	PrjPntOnChkMdl(dCurrPt, fPtNorm, CheckMdl, dPrjPos);
	nc_VectorCopy(dBegin, p->m_fPoint, 3);
	nc_VectorCopy(dEnd,  pNext->m_fPoint, 3);
	double dArcTol = nc_PointLineDist(dPrjPos, dBegin, dEnd );
	if( dArcTol >= m_cCalTol.m_dArcTol )
	{
		CSmtCutPoint * pNewPt = new CSmtCutPoint(dPrjPos);
		pPath->InsertAfter(pNewPt,p);
		RecursiveAddMidPnt(pPath,  p,   pNewPt, CheckMdl);
		RecursiveAddMidPnt(pPath,pNewPt, pNext, CheckMdl);
	}
}

void C3DStepNomosaic::RecursiveAddMidPnt(CSmtCutPath * pPath,CSmtCutPoint * p,CSmtCutPoint * pNext,
									  CGeoSpline & SmoothedPt, CGeoSpline & SmoothedVec,
									  double U, double nextU, CSmtCheckMdl & CheckMdl)
{
	m_nRecursive++;
	if(m_nRecursive>20 || nc_Distance(p->m_fPoint, pNext->m_fPoint, 3)<0.0001) 
	{ 
		return;
	}

	double dMidU = ( U + nextU ) * 0.5;
	PNT3D dSmoothedPt, dSmoothedVec, vNormDir, dPrjPos ;
	SmoothedPt.GetPoint(dMidU, dSmoothedPt);
	SmoothedVec.GetPoint(dMidU, dSmoothedVec);
	nc_VectorMinus(dSmoothedPt, dSmoothedVec, vNormDir, 3 );
	nc_Normalize ( vNormDir, 3 ) ;
	FPNT3D fPrjDir;
	nc_VectorCopy(fPrjDir,  vNormDir, 3);
	PrjPntOnChkMdl(dSmoothedPt, fPrjDir, CheckMdl, dPrjPos);
	PNT3D dBegin, dEnd;
	nc_VectorCopy(dBegin, p->m_fPoint, 3);
	nc_VectorCopy(dEnd,  pNext->m_fPoint, 3);
	double dArcTol = nc_PointLineDist(dPrjPos, dBegin, dEnd );
	if( dArcTol >= m_cCalTol.m_dArcTol )
	{
		CSmtCutPoint * pNewPt = new CSmtCutPoint(dPrjPos);
		pPath->InsertAfter(pNewPt,p);
		RecursiveAddMidPnt(pPath,  p,   pNewPt,SmoothedPt,SmoothedVec,  U, dMidU,  CheckMdl);
		RecursiveAddMidPnt(pPath,pNewPt,pNext, SmoothedPt,SmoothedVec, dMidU, nextU,CheckMdl);
	}
}


void C3DStepNomosaic::SmoothPrjSharpPath(CSmtCutPath *pPath, CSmtCutPath * pAtPath,
										 CSmtCheckMdl & CheckMdl)
{
	if(pPath==NULL || pAtPath==NULL)
	{
		return;
	}
	PNT3D dPrjPos, dCurrPt ;
	FPNT3D fPtNorm, fBox3D[2];
	CSmtCutPoint * pStartSmtCutPt = NULL;
	CSmtCutPoint * pEndSmtCutPt = NULL;
	//投影并加密点
	for(CSmtCutPoint * pPnt=pPath->m_pHead; pPnt&&pPnt->next; pPnt=pPnt->next)
	{
		CSmtCutPoint *pNext  = pPnt->next;
		nc_InitBox3D( pPnt->m_fPoint, pNext->m_fPoint, fBox3D ) ;
		nc_ExpandBox3D( fBox3D, 0.1f, FALSE ) ;
		fBox3D[0][2] = CheckMdl.m_fBottom ;
		fBox3D[1][2] = 1.0e6;
		CheckMdl.LabelCheckByBox( fBox3D );
		if(pPnt == pPath->m_pHead) 
		{
			GetPathPtNorm(pPnt->m_fPoint, fPtNorm,  CheckMdl);
			nc_VectorCopy(dCurrPt, pPnt->m_fPoint, 3);
			PrjPntOnChkMdl(dCurrPt, fPtNorm, CheckMdl, dPrjPos);
			pStartSmtCutPt = new CSmtCutPoint(dPrjPos);
			pAtPath->AddTail(pStartSmtCutPt);
		}
		GetPathPtNorm(pNext->m_fPoint, fPtNorm,  CheckMdl);
		nc_VectorCopy(dCurrPt, pNext->m_fPoint, 3);
		PrjPntOnChkMdl(dCurrPt, fPtNorm, CheckMdl, dPrjPos);
		pEndSmtCutPt = new CSmtCutPoint(dPrjPos);
		pAtPath->AddTail(pEndSmtCutPt);
		m_nRecursive = 0;
		RecursiveAddMidPnt(pAtPath, pStartSmtCutPt, pEndSmtCutPt,CheckMdl);
		pStartSmtCutPt = pEndSmtCutPt;
	}
}

static void MathCam_RemovePathGap(CSmtCutPath * pPrevPath,CSmtCutPath * pAtPath)
{//防止路径段间出现空隙
	if( pPrevPath && pAtPath && pAtPath->m_pHead && pAtPath->m_pTail &&/*pPrevPath->GetCutMode()==MINI_MILL_PATH &&*/ 
		pPrevPath->m_pTail && !( pPrevPath->IsClosed() && pAtPath->IsClosed() ) && 
		nc_Distance(pPrevPath->m_pTail->m_fPoint, pAtPath->m_pHead->m_fPoint, 3)<0.01 )
	{
		if(pAtPath->IsClosed())
			nc_VectorCopy(pAtPath->m_pTail->m_fPoint, pPrevPath->m_pTail->m_fPoint, 3);
		nc_VectorCopy(pAtPath->m_pHead->m_fPoint, pPrevPath->m_pTail->m_fPoint, 3);
	}
}
BOOL C3DStepNomosaic::RemoveMosaic(CSmtCPathLib& AllPath, CSmtCheckMdl & CheckMdl,
								   JDNC_TOL & CalTol,JDNC_PRGDEF& PrgDef,double SmoothTol)
{
	INT_PTR nTotalPath = AllPath.m_cAllPath.GetCount() ;
	if( nTotalPath < 1 )
	{
		return TRUE;
	}

	//step 0:进度条设置
	SurfNC_SetNewStepEx(IDS_3DSTEP_REMOVEMOSAIC);
	PrgDef.m_dLimitAt = 1.0 ;
	PrgDef.m_dStepAt  = 0.0 ;
	if( PrgDef.m_dTotalMove > 0.0 )
		PrgDef.m_dIncStep = PrgDef.m_dTotalMove / double(nTotalPath); 
	else
		PrgDef.m_dIncStep = 0.0; 

	m_cCalTol = CalTol;
	int nSmoothPts;
	CPathSmooth cPSmooth;
	PNT3D dPrjPos, dCurrPt, vTempTan, dTempBox[2];
	FPNT3D vFirstTan, fBox3D[2];
	double dStep = 0.15;
	double dNoMosaicDist = min(0.15, m_cCalTol.m_dMaxStep) ;
	CSmtCutPoint * pStartSmtCutPt = NULL;
	CSmtCutPoint * pEndSmtCutPt = NULL;
	CSmtCutPath * pPrevPath = NULL;
	
	//step 1:对每条切削路径、光顺点和法矢，再投影到精细检查模型上
	POSITION pAtPos = AllPath.m_cAllPath.GetHeadPosition();
	while(pAtPos)
	{
		if( PrgDef.m_pBrkFunc && PrgDef.m_pBrkFunc())	{return FALSE; }
		PrgDef.m_dStepAt += PrgDef.m_dIncStep ;
		while( PrgDef.m_pPrgFunc && PrgDef.m_dStepAt >= PrgDef.m_dLimitAt )	
		{
			PrgDef.m_pPrgFunc( 1 ) ;
			PrgDef.m_dStepAt -= PrgDef.m_dLimitAt ;
		}
		CSmtCutPath * pAtPath = AllPath.m_cAllPath.GetNext(pAtPos);
		if(!pAtPath || pAtPath->GetCutMode()!=MINI_MILL_PATH || pAtPath->m_nNumPnt<3)
		{
			MathCam_RemovePathGap(pPrevPath, pAtPath);
			pPrevPath = pAtPath;
			continue;
		}
		CTypedPtrArray<CPtrArray,CSmtCutPath*> cBrkPathArr;
		MathCam_BreakPathByAngle(*pAtPath, 160, cBrkPathArr);
		pAtPath->ClearAllPoint();
		INT_PTR nBrkPath = cBrkPathArr.GetCount();
		for(INT_PTR nPath=0; nPath<nBrkPath; nPath++)
		{
			CSmtCutPath *pPath = cBrkPathArr.GetAt(nPath);
			if(pPath==NULL) continue;
			m_pCurrPathCpy = pAtPath->CopyMyself();
			double dLength = pPath->GetLength();
			if(pPath->m_nNumPnt<=3 || dLength<1)
			{
				SmoothPrjSharpPath(pPath, pAtPath, CheckMdl);
				if(m_pCurrPathCpy) {
					delete m_pCurrPathCpy;
					m_pCurrPathCpy = NULL;
				}
				continue;
			}

			cPSmooth.m_nMaxCheckNum = max(100, int(dLength/0.15)); 
			//点密一些 光顺效果好些, 否则变形量会超出光顺误差导致投影失败!!!!!!!
			pPath->InsertCPoint3D(0.25);

			//光顺初始路径得到样条
			CGeoSpline * pSpline = NULL;
			if(nPath==0)
			{
				pSpline = cPSmooth.Smooth(*pPath, SmoothTol);
				pSpline->GetTangent(0, dCurrPt, vTempTan);
				nc_VectorCopy(vFirstTan, vTempTan, 3);
			}
			else if(nPath==nBrkPath-1 && cBrkPathArr[0]->m_nNumPnt>3)
			{
				pSpline = cPSmooth.Smooth(*pPath, SmoothTol, NULL, vFirstTan);
			}
			else
			{
				pSpline = cPSmooth.Smooth(*pPath, SmoothTol);
			}
			
			//按照步长取出一些框架采样点
			double dLen = pSpline->GetLength();
			nSmoothPts = int(dLen/dStep) + 1;
			if(nSmoothPts<pPath->m_nNumPnt)	nSmoothPts = pPath->m_nNumPnt;
			if(nSmoothPts<10)	nSmoothPts = 10;
			PNT3D * aSmoothPts = GetSplinePntByPtCnt(*pSpline, nSmoothPts);
			pPath->DefineBox();
#if 1
			//光顺后的点计算法矢
			PNT3D * aPtNorms = GetAllPtNorm(aSmoothPts, nSmoothPts,	CheckMdl);
#else
			memcpy(fBox3D, pPath->m_fBox, sizeof(FPNT3D[2]));
			nc_ExpandBox3D( fBox3D, 0.1f, FALSE ) ;
			fBox3D[0][2] = CheckMdl.m_fBottom ;
			fBox3D[1][2] = 1.0e6;
			//为了陡峭处法矢量计算的准确性，Labelbox深度要够
			CheckMdl.LabelCheckByGroupBox3D( fBox3D );
			//计算法矢量
			PNT3D * aPtNorms = GetAllPtNorm(aSmoothPts, nSmoothPts,	CheckMdl);
#endif

			double dNormLen = 0;
			for(int i=0; i<nSmoothPts; i++)	{
				MovePtAlongVec3D(aSmoothPts[i],aPtNorms[i], aPtNorms[i], 0.1);
				if(i-1>=0)	dNormLen += nc_Distance(aPtNorms[i-1],aPtNorms[i],3);
			}

			//光顺法矢 法矢的变形量也不宜太大，否则拐弯的地方变化太大导致投影出尖点
			cPSmooth.m_nMaxCheckNum = max(100, int(dNormLen/0.15)); 
			CGeoSpline * pSmoothedNorm = cPSmooth.Smooth(aPtNorms, nSmoothPts, SmoothTol);
			int nPnt = 0;
			//重新取出带方向点
			CDirPoint * aDirPt = GetDirPntByStep(*pSpline, *pSmoothedNorm, dNoMosaicDist, nPnt);
			//将点分块减少LabelCheck次数
			CDirPntBlkArr cPtBlock;
			SplitPntToBlocks(aDirPt, nPnt, 4, cPtBlock);
#if 0//_DEBUG
			OutputDebugInfo(cPtBlock);
#endif
			//按0.15点距投影重新分布路径点
			INT_PTR nBlock = cPtBlock.GetCount();
			for(INT_PTR i=0; i<nBlock; i++)
			{
				CDirPointBlock * pCurrBlk = cPtBlock.GetAt(i);
				if(pCurrBlk==NULL) {continue;}
				memcpy(fBox3D, pCurrBlk->m_fBox, sizeof(FPNT3D[2]));
				nc_ExpandBox3D( fBox3D, 0.2f, TRUE ) ;
				CheckMdl.LabelCheckByGroupBox3D( fBox3D );
				INT_PTR nPoint = pCurrBlk->m_cAllPnt.GetCount();
#if 0
				//先投影0.15点再加密, 不如一边投影一边加密点快
				CSmtCutPoint * pLastTail = pAtPath->m_pTail;
				for(INT_PTR j=0; j<nPoint; j++)
				{
					CDirPoint *pPoint = pCurrBlk->m_cAllPnt.GetAt(j);
					if(pPoint==NULL) continue;
					MovePtAlongVec3D(pPoint->m_fPoint, fBox3D[0], pPoint->m_fDir,  0.1);
					MovePtAlongVec3D(pPoint->m_fPoint, fBox3D[1], pPoint->m_fDir, -0.1);
					nc_InitBox3D( fBox3D[0], fBox3D[1], fBox3D ) ;
					CheckMdl.LabelCheckByFacetBox3D( fBox3D );
					PrjPntOnChkMdl(pPoint->m_fPoint,pPoint->m_fDir,CheckMdl,dPrjPos);
					CSmtCutPoint * pNewPt = new CSmtCutPoint(dPrjPos);
					pNewPt->m_fPoint[3] = (TFLOAT)pPoint->m_dU;
					pAtPath->AddTail(pNewPt);
				}
				if(pLastTail==NULL) pLastTail = pAtPath->m_pHead;
				CSmtCutPoint * p = pLastTail;
				while(p)
				{
					CSmtCutPoint * pNext = p->next;
					if(p && pNext)
					{
						nc_InitBox3D( p->m_fPoint, pNext->m_fPoint, fBox3D ) ;
						nc_ExpandBox3D( fBox3D, 0.1f, TRUE ) ;
						CheckMdl.LabelCheckByFacetBox3D( fBox3D );
						m_nRecursive = 0;
						RecursiveAddMidPnt(pAtPath, p, pNext, *pSpline, 
							*pSmoothedNorm, p->m_fPoint[3], pNext->m_fPoint[3], CheckMdl);
					}
					p = pNext;
				}

#else
				//一边投影点一边加密
				for(INT_PTR j=0; j<nPoint-1; j++)
				{
					CDirPoint *pPoint = pCurrBlk->m_cAllPnt.GetAt(j);
					CDirPoint *pNext  = pCurrBlk->m_cAllPnt.GetAt(j+1);
					if(pPoint==NULL || pNext==NULL) { continue; }
					nc_InitBox3D( pPoint->m_fPoint, pNext->m_fPoint, dTempBox ) ;
					MathCam_CopyBox3D(dTempBox, fBox3D);
					nc_ExpandBox3D( fBox3D, 0.1f, TRUE ) ;
					CheckMdl.LabelCheckByFacetBox3D( fBox3D );
					if(j==0) 
					{
						PrjPntOnChkMdl(pPoint->m_fPoint,pPoint->m_fDir,CheckMdl,dPrjPos);
						pStartSmtCutPt = new CSmtCutPoint(dPrjPos);
						pAtPath->AddTail(pStartSmtCutPt);
					}
					PrjPntOnChkMdl(pNext->m_fPoint,pNext->m_fDir,CheckMdl,dPrjPos);


#if 0
					if(fabs(dPrjPos[0] +39.0671)<0.004 && fabs(dPrjPos[1] -0.0834)<0.004
						&& fabs(dPrjPos[2] +13.7520)<0.004){//test 光顺前后法矢对比
						MathCam_DrawDirPolyLine(aDirPt, nPnt);
						//for(int k=0; k<nSmoothPts; k++)MathCam_DrawOneLine(aSmoothPts[k],aPtNorms[k]);
					}
#endif

					pEndSmtCutPt = new CSmtCutPoint(dPrjPos);
					pAtPath->AddTail(pEndSmtCutPt);
					m_nRecursive = 0;
					RecursiveAddMidPnt(pAtPath, pStartSmtCutPt, pEndSmtCutPt,
						*pSpline, *pSmoothedNorm, pPoint->m_dU, pNext->m_dU, CheckMdl);
					pStartSmtCutPt = pEndSmtCutPt;
				}
#endif
			}
			if(m_pCurrPathCpy) 
			{
				delete m_pCurrPathCpy;
				m_pCurrPathCpy = NULL;
			}
			ClearPntBlockArr(cPtBlock);
			delete [] aDirPt;
			aDirPt = NULL;
			DeletePntArr(aSmoothPts);
			DeletePntArr(aPtNorms);
			DeleteSpline(pSpline);
			DeleteSpline(pSmoothedNorm);
		}
		ClearPathArr(cBrkPathArr);
		pAtPath->DelPointOverlap(1.0e-5);
		pAtPath->DelPointOnLine(1.0e-4);

		//防止路径段间出现空隙
		MathCam_RemovePathGap(pPrevPath, pAtPath);
		pPrevPath = pAtPath;

	}
	AllPath.DefineBox();
	return TRUE;
}

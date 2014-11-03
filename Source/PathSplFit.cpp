#include "StdAfx.H"
#include "SmartNC.H"
#include "SysPrompt.h"
#include "SmartSurf.H"
#include "SurfGeo.h"
#include "CurveFair.H"
#include "geo_curve.h"
#include "PathSplFit.h"
#include "FFollowNoMosaic.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

//////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////CPathBezier3D//////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////
CPathBezier3D::CPathBezier3D() 
{
	m_n = 0;
}
CPathBezier3D::~CPathBezier3D() 
{
}
void CPathBezier3D::RotateEntity( DOUBLE /*Cos*/, DOUBLE /*Sin*/ ) 
{
}
void CPathBezier3D::MoveEntity( DOUBLE Dx, DOUBLE Dy, DOUBLE Dz ) 
{
	for(int i=0; i<m_n+1; i++)
	{
		m_Pw[i][0] += Dx;
		m_Pw[i][1] += Dy;
		m_Pw[i][2] += Dz;
	}
}
void CPathBezier3D::SetZCoord( DOUBLE ZCoord ) 
{
	for(int i=0; i<m_n+1; i++)
	{
		m_Pw[i][2] = ZCoord;
	}
}
DOUBLE CPathBezier3D::GetLength( ) 
{
	double Len = 0;
	if(m_n>1)
	{
		CBZCurve cBz(m_n, m_Pw);
		Len = cBz.GetLength(0, 1);
	}
	return Len;
}

BOOL CPathBezier3D::GetEndPoint(BOOL End, TPNT3D TPoint )   
{
	if(m_n<2) return FALSE;
	CBZCurve cBz(m_n, m_Pw);
	if(End)
		cBz.GetPoint(1, TPoint);
	else
		cBz.GetPoint(0, TPoint);
	return TRUE;
}
BOOL CPathBezier3D::GetEndTangent(BOOL Flag, VEC3D Tangent) 
{
	if(m_n<2) return FALSE;
	CBZCurve cBz(m_n, m_Pw);
	PNT3D TPoint;
	if(Flag)
		cBz.GetTangent(1, TPoint, Tangent);
	else
		cBz.GetTangent(0, TPoint, Tangent);
	return TRUE;
}
BOOL CPathBezier3D::GetEndNormal(BOOL Flag , VEC2D Normal ) 
{
	if(m_n<2) return FALSE;
	VEC3D vTan;
	GetEndTangent(Flag , vTan);
	nc_Normalize( vTan, 2 ) ;
	mathRotVec2D( PI1_2, vTan, Normal ) ;
	return TRUE;
}
CPathEntity* CPathBezier3D::CopyMyself() 
{
	CPathBezier3D * p = new CPathBezier3D();
	p->m_n = m_n;
	memcpy(p->m_Pw, m_Pw, MAX_ORD * sizeof(PNT4D));
	return NULL;
}
BOOL CPathBezier3D::ReverseDirect() 
{
	PNT4D temp ;
	int nCnt = m_n + 1;
	for( int i = 0 ; i < nCnt / 2 ; i++ )
	{
		memcpy(temp, m_Pw[i], sizeof(PNT4D));
		memcpy( m_Pw[i], m_Pw[ nCnt-i-1], sizeof(PNT4D)  ) ;
		memcpy( m_Pw[ nCnt-i-1], temp,  sizeof(PNT4D)) ;
	}
	return TRUE;
}
BOOL CPathBezier3D::GetBoundBox( double min[3], double max[3] ) 
{
	BOX3D cBox;
	mathDefBox3D(  m_Pw[0],  m_Pw[0], &cBox ) ;
	for(int i=1; i<m_n+1; i++)
	{
		mathEnlargeBox3D( m_Pw[i], &cBox ) ;
	}
	memcpy(min, cBox.min, sizeof(double[3]));
	memcpy(max, cBox.max, sizeof(double[3]));
	return TRUE;
}
BOOL CPathBezier3D::IsValid()  
{
	if(m_n<2) return FALSE;
	return TRUE;
}
BOOL CPathBezier3D::CreateBezier( PNT4D aCtrlPt[], int Count ) 
{
	if(aCtrlPt && Count>1 && Count<MAX_ORD)
	{
		m_n = Count - 1;
		memcpy(m_Pw, aCtrlPt, Count*sizeof(PNT4D));
		return TRUE;
	}
	return FALSE;
}
BOOL CPathBezier3D::Serialize(REGEN_PARAM& Param ) 
{
	CPathEntity::Serialize( Param ) ;
	if( !Param.m_bIsStoring )
    { /*读取数据*/
		int n = -1;
		Param.m_cRegenFile.Read(&n, sizeof(int) );
		if( n  == -1 )
		{
			return FALSE ;
		}
		PNT4D aCtrlPt[4];
		Param.m_cRegenFile.Read(aCtrlPt, (n+1) * sizeof(PNT4D) ) ; 
		CreateBezier( aCtrlPt, n+1 ) ;
	}
	else
	{
		if(m_n>0 )
		{
			Param.m_cRegenFile.Write( &m_n , sizeof(int) ) ;
			Param.m_cRegenFile.Write( &m_Pw, (m_n + 1) * sizeof(PNT4D) ) ;
		}
		else
		{
			int n = -1;
			Param.m_cRegenFile.Write( &n , sizeof(int) ) ;
		}
	}
	return TRUE;
}



//////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////CPathSplFit//////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////
CPathSplFit::CPathSplFit()
{
	m_dFitTol = 0.01;
	m_aSplDiscPt = NULL;
	m_aBezierDiscPt = NULL;
	m_nGenType = BSplInterpolateFit;
	m_pPSmooth = NULL;
}
CPathSplFit::~CPathSplFit()
{
	if(m_aSplDiscPt)
	{
		delete [] m_aSplDiscPt;
		m_aSplDiscPt = NULL;
	}
	if(m_aBezierDiscPt)
	{
		delete [] m_aBezierDiscPt;
		m_aBezierDiscPt = NULL;
	}
	if(m_pPSmooth)
	{
		delete m_pPSmooth;
		m_pPSmooth = NULL;
	}
}

double CPathSplFit::GetTriVertAng(CSmtCutPoint * pPrev,
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

void CPathSplFit::BreakPathByAngAndLen(CSmtCutPath & Path, double Angle, double OneSegMaxLen,
							   double OneSplMaxLen, CTypedPtrArray<CPtrArray,CSmtCutPath*> & BrkPathArr)
{
	if(Path.m_nNumPnt<3)
	{
		return ;
	}
	double dCurrAng;
	double dOneSplLen = 0;
	for(CSmtCutPoint *p=Path.m_pHead; p; p=p->next)
	{
		dCurrAng = GetTriVertAng(p->prev,p,p->next);
		p->m_fPoint[3] = (TFLOAT)dCurrAng;
		if(p->prev && nc_Distance(p->m_fPoint, p->prev->m_fPoint, 3)>OneSegMaxLen)
		{
			if(p != Path.m_pTail)
				p->m_bType = PNTTYPE_MAXLEN;
			if(p->prev != Path.m_pHead  &&  p->prev->m_bType != PNTTYPE_SHARP) 
				p->prev->m_bType = PNTTYPE_MAXLEN;
			dOneSplLen = 0;
			continue;
		}
		if(dCurrAng>0 && dCurrAng<ANGLE_TO_RADIAN(Angle))
		{
			p->m_bType = PNTTYPE_SHARP;
			dOneSplLen = 0;
			continue;
		}

		if(p->prev)
		{
			dOneSplLen += nc_Distance(p->m_fPoint, p->prev->m_fPoint, 3);
			if(dOneSplLen > OneSplMaxLen)
			{
				if(p != Path.m_pTail)
					p->m_bType = PNTTYPE_MAXLEN;
				if(p->prev != Path.m_pHead  &&  p->prev->m_bType != PNTTYPE_SHARP) 
					p->prev->m_bType = PNTTYPE_MAXLEN;
				dOneSplLen = 0;
			}
		}
		
	}

	CSmtCutPath * pNewPath = new CSmtCutPath();
	CSmtCutPoint * pNewPt = NULL;
	for(CSmtCutPoint *p=Path.m_pHead; p; p=p->next)
	{
		pNewPt = new CSmtCutPoint(p->m_fPoint, p->m_fPoint[3]);
		pNewPath->AddTail(pNewPt);
		if(p==Path.m_pTail)
		{
			BrkPathArr.Add(pNewPath);
			break;
		}
		if( p->m_bType == PNTTYPE_SHARP ||  p->m_bType == PNTTYPE_MAXLEN )
		{
			BrkPathArr.Add(pNewPath);
			pNewPath = new CSmtCutPath();
			pNewPt = new CSmtCutPoint(p->m_fPoint, p->m_fPoint[3]);
			pNewPt->m_bType = p->m_bType;
			pNewPath->AddTail(pNewPt);
			
		}
	}	
}
void CPathSplFit::BreakPathByLen(CSmtCutPath & Path, double Len,
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

void CPathSplFit::ClearPathArr(CTypedPtrArray<CPtrArray,CSmtCutPath*> & PathArr)
{
	for(INT_PTR nPath=0; nPath<PathArr.GetCount(); nPath++)  
	{
		CSmtCutPath *pPath = PathArr.GetAt(nPath);
		if(pPath) 
		{
			delete pPath;
			pPath = NULL;
		}
	}
	PathArr.RemoveAll();
}

void CPathSplFit::AddLinSegsToPath(CSmtCutPath * pCurrPath, CSmtCutPath * &pPath, int nPath,
								   FPNT3D vFirstTan, FPNT3D vLastTan)
{
	if(pCurrPath && pPath && pPath->m_nNumPnt>1)
	{
		nc_VectorMinus(pPath->m_pTail->prev->m_fPoint, pPath->m_pTail->m_fPoint, vLastTan, 3);
		nc_Normalize(vLastTan, 3);
		if(nPath==0 ) 
		{	
			nc_VectorMinus(pPath->m_pHead->next->m_fPoint, pPath->m_pHead->m_fPoint, vFirstTan, 3);
			nc_Normalize(vFirstTan, 3);
		}
		for(CSmtCutPoint * p=pPath->m_pHead; p; p=p->next)
		{
			pCurrPath->AddPoint(p->m_fPoint);
		}
		delete pPath; 
		pPath = NULL;
	}
}

BOOL CPathSplFit::IsPathSmooth(CSmtCutPath & CurrPath, double Ang)
{
	double dCurrAng;
	for(CSmtCutPoint *p=CurrPath.m_pHead; p; p=p->next)
	{
#if 0
		dCurrAng = GetTriVertAng(p->prev,p,p->next);
#else
		dCurrAng = p->m_fPoint[3];//保存的角度
#endif
		if(dCurrAng>0 && dCurrAng<ANGLE_TO_RADIAN(Ang))
		{
			return FALSE;
		}

	}
	return TRUE;
}

int CPathSplFit::DiscLinSegByStepNoBE(FPNT3D Begin, FPNT3D End,double LinLen, double Step, PNT3D AllPt[])
{
	int nPnt = 0;
	if( (LinLen - Step) > 0.001)
	{
		nPnt = int(LinLen/Step);
	}
	double u, nu, inc;
	u = inc = Step / LinLen ;
	for(int i=0; i<nPnt; i++)
	{
		nu = 1.0 - u;
		AllPt[i][0] = nu * Begin[0] + u*End[0];
		AllPt[i][1] = nu * Begin[1] + u*End[1];
		AllPt[i][2] = nu * Begin[2] + u*End[2];
		u += inc;
	}
	return nPnt;
}

void CPathSplFit::AdaptiveAddInterpolatePt(CSmtCutPath &Path)
{
	double dInsertDist, dCurrSegLen ;
	int i, nPnt;
	CSmtCutPoint * p = Path.m_pHead;
	double dLastSegLen = -1;
	while(p && p->next)
	{
		CSmtCutPoint * pNext = p->next;
		dInsertDist = 0.2;
		if(dLastSegLen > 0 )
		{//上一段存在
			dInsertDist = min(dInsertDist, dLastSegLen); 
		}
		if(pNext->next)
		{//下一段存在
			dInsertDist = min(dInsertDist, nc_Distance(pNext->m_fPoint, pNext->next->m_fPoint, 3)); 
		}
		if(dInsertDist<0.002)
		{
			dInsertDist = 0.002;
		}
		dCurrSegLen = nc_Distance(p->m_fPoint, pNext->m_fPoint, 3);
		nPnt = DiscLinSegByStepNoBE(p->m_fPoint, pNext->m_fPoint, dCurrSegLen, dInsertDist, m_aSplDiscPt);
		for(i=0; i<nPnt; i++)
		{
			Path.InsertBefore(new CSmtCutPoint(m_aSplDiscPt[i]), pNext);
		}
		p = pNext;
		dLastSegLen = dCurrSegLen;
	}
}


void CPathSplFit::AddLinSegsToPath(CSmtCutPath * pCurrPath, CSmtCutPath * &pPath )
{
	if(pCurrPath && pPath )
	{
		for(CSmtCutPoint * p=pPath->m_pHead; p; p=p->next)
		{
			pCurrPath->AddPoint(p->m_fPoint);
		}
		delete pPath; 
		pPath = NULL;
	}
}
void CPathSplFit::CalMidPnt(PNT3D dBegin,PNT3D dEnd,PNT3D dMid)
{
	for(int i=0; i<3; i++) 
	{
		dMid[i] = TFLOAT( (dBegin[i] + dEnd[i])*0.5 );
	}
}
BOOL CPathSplFit::GenBezier(PNT4D dCtrlPnt[], double Tol, int & MaxDepth,
							CTypedPtrArray<CPtrArray,CBZCurve*> & cBezierArr)
{
	MaxDepth++;
	for(int i=0; i<4; i++)
	{
		dCtrlPnt[i][3] = 1;
	}
	CBZCurve * pNewBZ = new CBZCurve(3, dCtrlPnt);
#if 0
	double u, dDist0, dDist1;
	PNT3D dPrjPt;
	//CBZCurve::PntProject有问题?
	pNewBZ->PntProject(dCtrlPnt[1], &u, dPrjPt, &dDist0 );
	pNewBZ->PntProject(dCtrlPnt[2], &u, dPrjPt, &dDist1 );
#else
	PNT3D aAllPt[300];
	int nAllPt = 0;
	for(double u=0; u<=1; u+= 0.005)
	{
		pNewBZ->GetPoint(u, aAllPt[nAllPt]);
		nAllPt++;
	}
	double dDist0, dDist1;
	dDist0 = dDist1 = 9999999999;
	for(int i=0; i<nAllPt; i++)
	{
		dDist0 = min(dDist0, nc_Distance(dCtrlPnt[1], aAllPt[i] , 3) );
		dDist1 = min(dDist1, nc_Distance(dCtrlPnt[2], aAllPt[i] , 3) );
	}
#endif

	if(MaxDepth>50 || (dDist0<Tol && dDist1<Tol) )
	{
		cBezierArr.Add(pNewBZ);
		return TRUE;
	}	
	delete pNewBZ;
	pNewBZ = NULL;

	PNT4D aSubPnt[4];
	nc_VectorCopy(aSubPnt[0], dCtrlPnt[0], 3);
	CalMidPnt(dCtrlPnt[0], dCtrlPnt[1], aSubPnt[1]);
	nc_VectorCopy(aSubPnt[2], dCtrlPnt[1], 3);
	CalMidPnt(dCtrlPnt[1], dCtrlPnt[2], aSubPnt[3]);
	GenBezier(aSubPnt, Tol, MaxDepth, cBezierArr);
	
	CalMidPnt(dCtrlPnt[1], dCtrlPnt[2], aSubPnt[0]);
	nc_VectorCopy(aSubPnt[1], dCtrlPnt[2], 3);
	CalMidPnt(dCtrlPnt[2], dCtrlPnt[3], aSubPnt[2]);
	nc_VectorCopy(aSubPnt[3], dCtrlPnt[3], 3);
	GenBezier(aSubPnt, Tol, MaxDepth, cBezierArr);
	return TRUE;
}

BOOL CPathSplFit::GenSectBezier(CSmtCutPath &Path, CTypedPtrArray<CPtrArray,CBZCurve*> & cBezierArr)
{
	PNT4D	dCtrlPnt[4];
	int n = 0;
	PNT3D dMidPt ;
	CSmtCutPoint * pPnt = Path.m_pHead;
	int nMaxGenTimes = 0;
	while(pPnt)
	{
		CSmtCutPoint * pNext = pPnt->next;
		nc_VectorCopy(dCtrlPnt[n], pPnt->m_fPoint, 3);
		n++;
		if(pNext)
		{
			if(n==3)
			{
				if(pNext->next==NULL)
				{
					nc_VectorCopy(dCtrlPnt[n], pNext->m_fPoint, 3);
					nMaxGenTimes = 0;
					GenBezier(dCtrlPnt, m_dFitTol, nMaxGenTimes, cBezierArr);
					break;
				}
				else
				{
					dMidPt[0] = ( pPnt->m_fPoint[0] + pNext->m_fPoint[0] ) * 0.5;
					dMidPt[1] = ( pPnt->m_fPoint[1] + pNext->m_fPoint[1] ) * 0.5;
					dMidPt[2] = ( pPnt->m_fPoint[2] + pNext->m_fPoint[2] ) * 0.5;
					nc_VectorCopy(dCtrlPnt[n], dMidPt, 3);
					nMaxGenTimes = 0;
					GenBezier(dCtrlPnt, m_dFitTol, nMaxGenTimes, cBezierArr);

					CSmtCutPoint * pNewPt = new CSmtCutPoint(dMidPt);
					Path.InsertAfter(pNewPt, pPnt);
					pPnt = pNewPt;
					n = 0;
					continue;
				}
			}
		}
		else
		{
			for(int i=0; i<4; i++)
			{
				nc_VectorCopy(dCtrlPnt[n], pPnt->m_fPoint, 3);
				n++;
				if(n>=4)	{	break; }
			}
			nMaxGenTimes = 0;
			GenBezier(dCtrlPnt, m_dFitTol, nMaxGenTimes, cBezierArr);
			break;
		}
		pPnt = pNext;
	}
	return TRUE;
}
void CPathSplFit::MovePtAlongVec(double OrigPt[],double DestPt[],
									   VEC3D Dir,double Dist)
{
	for(int i=0; i<3; i++)
	{
		DestPt[i] = OrigPt[i] + Dir[i]*Dist;
	}
}
void CPathSplFit::ReverseVec(double Orig[],double Dest[])
{
	for(int i=0; i<3; i++)
	{
		Dest[i] = -Orig[i];
	}
}
BOOL CPathSplFit::GenSectBezierEx(CSmtCutPath &Path, CTypedPtrArray<CPtrArray,CBZCurve*> & cBezierArr)
{
	PNT4D	dCtrlPnt[4];
	VEC3D  v0, v1, vAxis, vLastTan, vCurrEndTan;
	PNT3D p0, p1, p2, dPrjPt, dPt ;
	double t, dMov, dAng , dLen1_2, dSinA;
	int n;

	for(CSmtCutPoint * p=Path.m_pHead; p&&p->next; p=p->next)
	{

#if 0
		if(fabs(p->next->m_fPoint[0]+31.469)<0.01 && 
			fabs(p->next->m_fPoint[1]-46.07)<0.01 )
		{
			int ntest = 1;
		}
#endif

		CSmtCutPoint * pNext = p->next;
		nc_VectorCopy(p0, p->m_fPoint, 3);
		nc_VectorCopy(p1, pNext->m_fPoint, 3);
		dLen1_2 = nc_Distance(p0, p1, 3) * 0.5;
		dCtrlPnt[0][3] = dCtrlPnt[1][3] = dCtrlPnt[2][3] = dCtrlPnt[3][3] = 1;
		if(m_nGenType == SectBezierEx2)
		{//二次Bezier
			n = 0;
			nc_VectorCopy(dCtrlPnt[n], p0, 3);
			n++;
			if(p->prev == NULL && pNext->next != NULL)
			{
				mathGetVecUnit(p1, p0, v0);
				nc_VectorCopy(p2, pNext->next->m_fPoint, 3);
				mathGetVecUnit(p1, p2, v1);
				mathVProduct(v0, v1, vAxis);
				mathUniVec(vAxis, 1.0e-8);
				dAng = MiniPai1_2 - mathGetAngleUnit(v0, v1) * 0.5;
				mathRotVec(vAxis, p1, -dAng , v0, vCurrEndTan);
				mathUniVec(vCurrEndTan, 1.0e-8);
				//确定切线方向
				MovePtAlongVec(p1, dPt, vCurrEndTan,  dLen1_2);
				mathPrjPntLineEx(dPt, p0, p1, dPrjPt, &t);
				if(t<0 || t>1)	
					mathRevVec(vCurrEndTan);
				dSinA = sin(dAng);
				dMov = min(dLen1_2, m_dFitTol / dSinA );
				MovePtAlongVec(p1, dPt, vCurrEndTan, dMov);
				ReverseVec(vCurrEndTan, vLastTan);
				nc_VectorCopy(dCtrlPnt[n], dPt, 3);
				n++;
			}
			else if(p->prev != NULL)
			{
				dMov = min(dLen1_2, m_dFitTol / dSinA );
				MovePtAlongVec(p0, dCtrlPnt[n], vLastTan,  dMov);
				n++;
				mathGetVecUnit(dCtrlPnt[n-1], p1, vLastTan);
				if(pNext->next != NULL)
				{
					nc_VectorCopy(p2, pNext->next->m_fPoint, 3);
					mathGetVecUnit(p1, p2, v1);
					dAng = mathGetAngleUnit(vLastTan, v1);
				}
			}
			nc_VectorCopy(dCtrlPnt[n], p1, 3);
			n++;
			//MathCam_DrawPolyLine(dCtrlPnt, n);
			cBezierArr.Add(new CBZCurve( n - 1 , dCtrlPnt) );
		}
		else if(m_nGenType == SectBezierEx3)
		{//三次Bezier
			n = 0;
			nc_VectorCopy(dCtrlPnt[n], p0, 3);
			n++;
			if(p->prev != NULL)
			{
				dMov = min(dLen1_2, m_dFitTol / dSinA );
				MovePtAlongVec(p0, dCtrlPnt[n], vLastTan,  dMov);
				n++;
			}
			if(pNext->next != NULL)
			{
				mathGetVecUnit(p1, p0, v0);
				nc_VectorCopy(p2, pNext->next->m_fPoint, 3);
				mathGetVecUnit(p1, p2, v1);

				mathVProduct(v0, v1, vAxis);
				mathUniVec(vAxis, 1.0e-8);
				dAng = MiniPai1_2 - mathGetAngleUnit(v0, v1) * 0.5;
				mathRotVec(vAxis, p1, -dAng , v0, vCurrEndTan);
				mathUniVec(vCurrEndTan, 1.0e-8);
				//确定切线方向
				MovePtAlongVec(p1, dPt, vCurrEndTan,  dLen1_2);
				mathPrjPntLineEx(dPt, p0, p1, dPrjPt, &t);
				if(t<0 || t>1)	
				{
					mathRevVec(vCurrEndTan);
				}
				dSinA = sin(dAng);
				dMov = min(dLen1_2, m_dFitTol / dSinA );
				MovePtAlongVec(p1, dPt, vCurrEndTan, dMov);
				ReverseVec(vCurrEndTan, vLastTan);
				nc_VectorCopy(dCtrlPnt[n], dPt, 3);
				n++;
			}
			nc_VectorCopy(dCtrlPnt[n], p1, 3);
			n++;
			cBezierArr.Add(new CBZCurve( n - 1 , dCtrlPnt));
		}
	}
	return TRUE;
}

void CPathSplFit::SectBezierFitOnePath(CSmtCutPath * pCurrPath)
{
	if(pCurrPath==NULL || pCurrPath->m_nNumPnt<3)
	{
		return;
	}

	if(m_aBezierDiscPt==NULL)
	{
		m_aBezierDiscPt = new PNT5D [SPL_DISC_MAXPT];
	}

	CTypedPtrArray<CPtrArray,CSmtCutPath*> cBrkPathArr;
	//将路径按角度、单段线段最大长度和单段样条最大长度打断分段
	BreakPathByAngAndLen(*pCurrPath, BRKSEG_ANG, LINESEG_MAXLEN, ONESPL_MAXLEN, cBrkPathArr);
	int nBrkPath = (int)cBrkPathArr.GetCount();
	if(nBrkPath<1)	
	{	
		return; 
	}
	pCurrPath->ClearAllPoint();
	for(int nPath=0; nPath<nBrkPath; nPath++)
	{
		CSmtCutPath *pPath = cBrkPathArr.GetAt(nPath);
		if(pPath==NULL )  
		{	
			continue; 
		}
		if(pPath->m_nNumPnt<2)
		{
			ASSERT(0);
			delete pPath; 
			pPath = NULL;
			continue;
		}
		if( pPath->m_nNumPnt<3 ||       //长直线段
			pPath->GetLength() < ONESPL_MINLEN ||     //极短的待拟合线条
			IsPathSmooth(*pPath, 176) ) //当前路径光滑
		{
			AddLinSegsToPath(pCurrPath, pPath);
			continue;
		}

		CTypedPtrArray<CPtrArray,CBZCurve*> cBezierArr;
		if(m_nGenType == SectBezier)
			GenSectBezier(*pPath, cBezierArr);
		else if(m_nGenType == SectBezierEx2 || m_nGenType == SectBezierEx3 )
			GenSectBezierEx(*pPath, cBezierArr);
		delete pPath;
		pPath = NULL;
		int nBZ = (int)cBezierArr.GetCount();
		for(int n=0; n<nBZ; n++)
		{
			CBZCurve * pBZ = cBezierArr.GetAt(n);
			if(pBZ==NULL)		{ continue; }
			int nPtNum = 0;
			pBZ->DiscreteToLine(SPL_DISC_TOL, ANGLE_TO_RADIAN(SPL_DISC_ANGTOL), 
				SPL_DISC_MAXPT, nPtNum, m_aBezierDiscPt);
			delete pBZ;
			pBZ = NULL;
			for(int i=0; i<nPtNum; i++) 
			{
				pCurrPath->AddPoint(m_aBezierDiscPt[i]);
			}
		}
	}
	pCurrPath->DelPointOverlap();
	pCurrPath->DelPointOnLine(1.0e-4);
}
void CPathSplFit::SplFittingOnePath(CSmtCutPath * pCurrPath)
{
	if(pCurrPath==NULL || pCurrPath->m_nNumPnt<3)
	{
		return;
	}

	if(m_aSplDiscPt==NULL)
	{
		m_aSplDiscPt = new PNT3D [SPL_DISC_MAXPT];
	}
	if(m_pPSmooth==NULL)
	{
		m_pPSmooth = new CPathSmooth;
	}

	FPNT3D vFirstTan, vLastTan ;
	PNT3D dCurrPt, vTempTan;
	BOOL bClosed = pCurrPath->IsClosed();
	CTypedPtrArray<CPtrArray,CSmtCutPath*> cBrkPathArr;
	//将路径按角度、单段线段最大长度和单段样条最大长度打断分段
	BreakPathByAngAndLen(*pCurrPath, BRKSEG_ANG, LINESEG_MAXLEN, ONESPL_MAXLEN, cBrkPathArr);
	int nBrkPath = (int)cBrkPathArr.GetCount();
	if(nBrkPath == 1 && bClosed)
	{//由于光顺周期样条曲线没有端点条件，为了使封闭路径首末点固定须打断路径
		ClearPathArr(cBrkPathArr);
		BreakPathByLen(*pCurrPath, pCurrPath->GetLength()*0.6, cBrkPathArr);
		nBrkPath = (int)cBrkPathArr.GetCount();
	}
	if(nBrkPath<1)	
	{	
		return; 
	}
	BOOL bSharpClosed = FALSE;
	if(bClosed )
	{
		double dHTAng = GetTriVertAng(pCurrPath->m_pTail->prev, pCurrPath->m_pHead,pCurrPath->m_pHead->next);
		if(dHTAng>0 && dHTAng<ANGLE_TO_RADIAN(BRKSEG_ANG))
		{//闭合曲线首末点是否尖锐
			bSharpClosed = TRUE;
		}
	}
	pCurrPath->ClearAllPoint();
	for(int nPath=0; nPath<nBrkPath; nPath++)
	{
		CSmtCutPath *pPath = cBrkPathArr.GetAt(nPath);
		if(pPath==NULL )  
		{	
			continue; 
		}
		if(pPath->m_nNumPnt<2)
		{
			ASSERT(0);
			delete pPath; 
			pPath = NULL;
			continue;
		}
		if(pPath->m_nNumPnt<3) 
		{//长直线段
			AddLinSegsToPath(pCurrPath, pPath, nPath, vFirstTan, vLastTan);
			continue;
		}
		double dLen = pPath->GetLength();
		if(dLen < ONESPL_MINLEN || IsPathSmooth(*pPath, 175))
		{//极短的待拟合线条 or 当前路径光滑
			AddLinSegsToPath(pCurrPath, pPath, nPath, vFirstTan, vLastTan);
			continue;
		}
		int nPtNum = max( 70 , int(dLen/SPLINE_VRFYINC) );
		m_pPSmooth->m_nMaxCheckNum = nPtNum; 
#if 1
		AdaptiveAddInterpolatePt(*pPath);
#else
		//均匀化插值拟合点(在很短和很长的段交接的地方、按固定值均匀插值变形鼓起大)
		pPath->InsertCPoint3D(0.2);
#endif
		CGeoSpline * pSpline = NULL;
		if(nPath==0)
		{
			pSpline = m_pPSmooth->Smooth(*pPath, m_dFitTol);
		}
		else if(bClosed && nPath==nBrkPath-1 )  
		{//周期样条曲线最后一段
			if(pPath->m_pHead->m_bType != PNTTYPE_SHARP )//与上一段为非尖点断开、非尖点断开处切矢连续
			{
				if(bSharpClosed)
					pSpline = m_pPSmooth->Smooth(*pPath, m_dFitTol, vLastTan, NULL);
				else
					pSpline = m_pPSmooth->Smooth(*pPath, m_dFitTol, vLastTan, vFirstTan);
			}
			else 
			{
				if(bSharpClosed)
					pSpline = m_pPSmooth->Smooth(*pPath, m_dFitTol );
				else
					pSpline = m_pPSmooth->Smooth(*pPath, m_dFitTol, NULL, vFirstTan);
			}
		}
		else 
		{
			if(pPath->m_pHead->m_bType != PNTTYPE_SHARP )//与上一段为非尖点断开、非尖点断开处切矢连续
				pSpline = m_pPSmooth->Smooth(*pPath, m_dFitTol, vLastTan, NULL);
			else
				pSpline = m_pPSmooth->Smooth(*pPath, m_dFitTol);
		}

		if(pSpline==NULL)
		{//异常
			ASSERT(0);
			AddLinSegsToPath(pCurrPath, pPath, nPath, vFirstTan, vLastTan);
			continue;
		}

		delete pPath;
		pPath = NULL;
		pSpline->GetTangent(1, dCurrPt, vTempTan);
		nc_VectorCopy(vLastTan, vTempTan, 3);
		if(nPath==0)
		{
			pSpline->GetTangent(0, dCurrPt, vTempTan);
			nc_VectorCopy(vFirstTan, vTempTan, 3);
		}
#if 1
		pSpline->DiscreteByChord(SPL_DISC_TOL, ANGLE_TO_RADIAN(SPL_DISC_ANGTOL), 
			SPL_DISC_MAXPT, nPtNum, m_aSplDiscPt);
		delete pSpline;
		pSpline = NULL;
		for(int i=0; i<nPtNum; i++) 
		{
			pCurrPath->AddPoint(m_aSplDiscPt[i]);
		}
#else
		nPtNum = max(4, int( dLen / SPLINE_PNTINC ) );
		double dInc = 1.0 /double(nPtNum - 1 );
		for(int i=0; i<nPtNum; i++) 
		{
			pSpline->GetPoint(i*dInc, dCurrPt);
			pCurrPath->AddPoint(dCurrPt);
		}
		delete pSpline;
		pSpline = NULL;
#endif
		
	}
	pCurrPath->DelPointOverlap();
	pCurrPath->DelPointOnLine(2.0e-4);
}

BOOL CPathSplFit::SplFit(CSmtCPathLib& AllPath,  /*所有路径*/
						 double FitTol ,      /*样条拟合误差*/
						 JDNC_PRGDEF& PrgDef)
{
	if(fabs(FitTol)>0.0001 )
	{
		double dTol = FitTol;
		if(dTol > 0.2)
			dTol = 0.2;
		else if( dTol < 0.002)
			dTol = 0.002;
		m_dFitTol = dTol;
		POSITION pAtPos = AllPath.m_cAllPath.GetHeadPosition();
		while(pAtPos)
		{
			if( PrgDef.m_pBrkFunc && PrgDef.m_pBrkFunc())	
			{
				return FALSE; 
			}
			PrgDef.m_dStepAt += PrgDef.m_dIncStep ;
			while( PrgDef.m_pPrgFunc && PrgDef.m_dStepAt >= PrgDef.m_dLimitAt )	
			{
				PrgDef.m_pPrgFunc( 1 ) ;
				PrgDef.m_dStepAt -= PrgDef.m_dLimitAt ;
			}

			CSmtCutPath * pCurrPath = AllPath.m_cAllPath.GetNext(pAtPos);
			if(pCurrPath && pCurrPath->GetCutMode()==MINI_MILL_PATH && pCurrPath->m_nNumPnt>3 )
			{
				if(m_nGenType == SectBezier || m_nGenType == SectBezierEx2 || m_nGenType == SectBezierEx3)
					SectBezierFitOnePath(pCurrPath);
				else if(m_nGenType == BSplInterpolateFit)
					SplFittingOnePath(pCurrPath);
			}
		}
		AllPath.DefineBox();
	}
	return TRUE;
}

BOOL CPathSplFit::SplFitSubProc(int /*nAtCore*/, int /*nCoreNum*/, VOID* AllPath,
								double FitTol, JDNC_PRGDEF &PrgDef)
{
	BOOL bRet = TRUE;
	if(AllPath)
	{
		CSmtCPathLib * pAllPath = (CSmtCPathLib *) AllPath;
		bRet = SplFit(*pAllPath, FitTol, PrgDef);
	}
	return bRet;
}
BOOL CPathSplFit::SplFitSubProcEx(int /*nAtCore*/, int /*nCoreNum*/, VOID* AllPath,
								double FitTol, VOID* AllComb, JDNC_PRGDEF &PrgDef)
{
	BOOL bRet = TRUE;
	if(AllPath && AllComb)
	{
		CSmtCPathLib * pAllPath = (CSmtCPathLib *) AllPath;
		CPathCombine * pAllComb = (CPathCombine *) AllComb;
		bRet = SplFit(*pAllPath, FitTol, *pAllComb, PrgDef);
	}
	return bRet;
}


////////////////////生成CPathBezier3D路径段/////////////////////////////
void CPathSplFit::SectBezierFitOnePath(CSmtCutPath &CurrPath,
									   CPathCombine & PathComb)
{
	CTypedPtrArray<CPtrArray,CSmtCutPath*> cBrkPathArr;
	//将路径按角度、单段线段最大长度和单段样条最大长度打断分段
	BreakPathByAngAndLen(CurrPath, BRKSEG_ANG, LINESEG_MAXLEN, ONESPL_MAXLEN, cBrkPathArr);
	for(INT_PTR nPath=0; nPath<cBrkPathArr.GetCount(); nPath++)
	{
		CSmtCutPath *pPath = cBrkPathArr.GetAt(nPath);
		if(pPath==NULL )  
		{	
			continue; 
		}
		if(pPath->m_nNumPnt<2)
		{
			ASSERT(0);
			delete pPath; 
			pPath = NULL;
			continue;
		}
		if( pPath->m_nNumPnt<3 ||       //长直线段
			pPath->GetLength() < ONESPL_MINLEN ||     //极短的待拟合线条
			IsPathSmooth(*pPath, 176) ) //当前路径光滑
		{
			CPathPLine3D *pPLin3D = new CPathPLine3D();
			pPLin3D->m_bFeedType = CurrPath.m_bFeedType;
			pPLin3D->m_fFeedRate = CurrPath.m_fFeedRate;
			pPLin3D->m_bMoveFlag = CurrPath.m_bMoveFlag;
			pPLin3D->m_bFeedType = CurrPath.m_bFeedType;
			pPLin3D->m_pTAPos = new TPNT3D [pPath->m_nNumPnt];
			pPLin3D->m_nCount = pPath->m_nNumPnt - 1;
			int n = 0;
			for(CSmtCutPoint * p=pPath->m_pHead; p; p=p->next)
			{
				nc_VectorCopy(pPLin3D->m_pTAPos[n], p->m_fPoint, 3);
				n++;
			}
			delete pPath; 
			pPath = NULL;
			PathComb.AddEntity(pPLin3D);
			continue;
		}

		CTypedPtrArray<CPtrArray,CBZCurve*> cBezierArr;
		if(m_nGenType == SectBezier)
			GenSectBezier(*pPath, cBezierArr);
		else if(m_nGenType == SectBezierEx2 ||m_nGenType == SectBezierEx3)
			GenSectBezierEx(*pPath, cBezierArr);
		delete pPath;
		pPath = NULL;
		int nBZ = (int)cBezierArr.GetCount();
		for(int n=0; n<nBZ; n++)
		{
			CBZCurve * pBZ = cBezierArr.GetAt(n);
			if(pBZ==NULL)		{ continue; }
#if 1
			CPathNurbs3D * pNurbs = new CPathNurbs3D();
			pNurbs->CreateNurbs(pBZ->m_n+1, pBZ->m_n);
			memcpy(pNurbs->m_fPoint, pBZ->m_Pw, (pNurbs->m_nCount + 1 ) * sizeof(double[4]) );
			if(pBZ->m_n==3)
			{
				pNurbs->m_fKnot[0] = pNurbs->m_fKnot[1] = pNurbs->m_fKnot[2] = pNurbs->m_fKnot[3] = 0;
				pNurbs->m_fKnot[4] = pNurbs->m_fKnot[5] = pNurbs->m_fKnot[6] = pNurbs->m_fKnot[7] = 1;
			}
			else if(pBZ->m_n==2)
			{
				pNurbs->m_fKnot[0] = pNurbs->m_fKnot[1] = pNurbs->m_fKnot[2] = 0;
				pNurbs->m_fKnot[3] = pNurbs->m_fKnot[4] = pNurbs->m_fKnot[5] = 1;
			}
			else 
			{
				ASSERT(0);
			}
			pNurbs->m_bFeedType = CurrPath.m_bFeedType;
			pNurbs->m_fFeedRate = CurrPath.m_fFeedRate;
			pNurbs->m_bMoveFlag = CurrPath.m_bMoveFlag;
			pNurbs->m_bFeedType = CurrPath.m_bFeedType;
			delete pBZ;
			pBZ = NULL;
			PathComb.AddEntity(pNurbs);
#else
			CPathBezier3D *pBezier = new CPathBezier3D();
			pBezier->m_bFeedType = CurrPath.m_bFeedType;
			pBezier->m_fFeedRate = CurrPath.m_fFeedRate;
			pBezier->m_bMoveFlag = CurrPath.m_bMoveFlag;
			pBezier->m_bFeedType = CurrPath.m_bFeedType;
			pBezier->m_n = pBZ->m_n;
			memcpy(pBezier->m_Pw, pBZ->m_Pw, MAX_ORD*sizeof(PNT4D));
			delete pBZ;
			pBZ = NULL;
			PathComb.AddEntity(pBezier);
#endif
		}
	}
}

double CPathSplFit::Cal3PtsArcCurvature(FPNT3D p0, FPNT3D p1, FPNT3D p2)
{//计算三点组成圆弧之曲率半径
	PNT3D dPt0, dPt1, dPt2, dMid0, dMid1,v0, v1, vAxis, dInt0, dInt1;
	nc_VectorCopy(dPt0, p0, 3);
	nc_VectorCopy(dPt1, p1, 3);
	nc_VectorCopy(dPt2, p2, 3);
	CalMidPnt(dPt1, dPt0, dMid0);
	CalMidPnt(dPt1, dPt2, dMid1);
	nc_VectorMinus(dPt1, dPt0, v0, 3);
	nc_VectorMinus(dPt1, dPt2, v1, 3);
	nc_VProduct(v0, v1, vAxis);
	nc_Normalize(vAxis, 3);	
	mathRotVec(vAxis, dMid0, MiniPai1_2, v0, v0);
	mathRotVec(vAxis, dMid1, MiniPai1_2, v1, v1);
	nc_Normalize(v0, 3);
	nc_Normalize(v1, 3);
	if(IDINT == mathIntLin(dMid0, v0, dMid1, v1, 0.0001, 0.0001, dInt0, dInt1))	{
		return 1.0/nc_Distance(dPt0, dInt0, 3);
	}
	return -1;
}
BOOL CPathSplFit::GetPathTailTan(CSmtCutPath *pPath,FPNT3D Tan)
{
	if(pPath && pPath->m_pTail && pPath->m_pTail->prev)
	{
		nc_VectorMinus(pPath->m_pTail->prev->m_fPoint, pPath->m_pTail->m_fPoint, Tan, 3);
		nc_Normalize(Tan, 3);
		return TRUE;
	}
	return FALSE;
}
BOOL CPathSplFit::GetPathHeadTan(CSmtCutPath *pPath,FPNT3D Tan)
{
	if(pPath && pPath->m_pHead && pPath->m_pHead->next )
	{
		nc_VectorMinus(pPath->m_pHead->m_fPoint, pPath->m_pHead->next->m_fPoint, Tan, 3);
		nc_Normalize(Tan, 3);
		return TRUE;
	}
	return FALSE;
}
void CPathSplFit::BreakPathByAngAndMaxPt(CSmtCutPath & Path, double Angle, double OneSegMaxLen,
							   int /*OneSplMaxPt*/, CTypedPtrArray<CPtrArray,CSmtCutPath*> & BrkPathArr)
{
	if(Path.m_nNumPnt<3)
	{
		return ;
	}

	double dCurrAng;
	double dOneSplLen = 0;
	int nPnt = 0;
	double dLimitAng = ANGLE_TO_RADIAN(Angle);
	for(CSmtCutPoint *p=Path.m_pHead; p; p=p->next)
	{
		nPnt ++ ;
		dCurrAng = GetTriVertAng(p->prev,p,p->next);
		p->m_fPoint[3] = (TFLOAT)dCurrAng;
		if(dCurrAng>0 && dCurrAng<dLimitAng)
		{
			p->m_bType = PNTTYPE_SHARP;
			dOneSplLen = 0;
			nPnt = 0;
			if(p->next && nc_Distance(p->m_fPoint, p->next->m_fPoint, 3)>OneSegMaxLen)
				p->next->m_bType = PNTTYPE_MAXLEN;
			continue;
		}
		if(p->next && nc_Distance(p->m_fPoint, p->next->m_fPoint, 3)>OneSegMaxLen)
		{
			if(p->next != Path.m_pTail)
				p->next->m_bType = PNTTYPE_MAXLEN;
			if(p != Path.m_pHead  &&  p->m_bType != PNTTYPE_SHARP) 
				p->m_bType = PNTTYPE_MAXLEN;
			dOneSplLen = 0;
			nPnt = 0;
			continue;
		}
#if 0
		if(p->prev && nPnt>OneSplMaxPt)
		{
			p->m_bType = PNTTYPE_MAXPNT;
			dOneSplLen = 0;
			nPnt = 0;
			continue;
		}
		if(p->prev && p->prev->prev && p->next && p->next->next)
		{
			double dCurvature0 = Cal3PtsArcCurvature(p->prev->prev->m_fPoint, p->prev->m_fPoint, p->m_fPoint);
			double dCurvature1 = Cal3PtsArcCurvature(p->m_fPoint, p->next->m_fPoint, p->next->next->m_fPoint);
			if(dCurvature0>-1 && dCurvature1>-1 && fabs(dCurvature0-dCurvature1)>2.5 )
			{//在曲率突变的地方打断
				p->m_bType = PNTTYPE_RCHANGE;
				dOneSplLen = nPnt = 0;
			}
		}
#endif
	}

	CSmtCutPath * pNewPath = new CSmtCutPath();
	CSmtCutPoint * pNewPt = NULL;
	for(CSmtCutPoint *p=Path.m_pHead; p; p=p->next)
	{
		pNewPt = new CSmtCutPoint(p->m_fPoint, p->m_fPoint[3]);
		pNewPath->AddTail(pNewPt);
		if(p==Path.m_pTail)
		{
			BrkPathArr.Add(pNewPath);
			break;
		}
		if( p->m_bType == PNTTYPE_SHARP ||  p->m_bType == PNTTYPE_MAXLEN 
			|| p->m_bType == PNTTYPE_MAXPNT || p->m_bType == PNTTYPE_RCHANGE)
		{
			BrkPathArr.Add(pNewPath);
			pNewPath = new CSmtCutPath();
			pNewPt = new CSmtCutPoint(p->m_fPoint, p->m_fPoint[3]);
			pNewPt->m_bType = p->m_bType;
			pNewPath->AddTail(pNewPt);
			
		}
	}	
}
void CPathSplFit::AddPathPLine3D(CSmtCutPath *pPath, CPathCombine & PathComb)
{
	if(pPath)
	{
		CPathPLine3D *pPLin3D = new CPathPLine3D();
		pPLin3D->m_bFeedType = pPath->m_bFeedType;
		pPLin3D->m_fFeedRate = pPath->m_fFeedRate;
		pPLin3D->m_bMoveFlag = pPath->m_bMoveFlag;
		pPLin3D->m_bFeedType = pPath->m_bFeedType;
		pPLin3D->m_pTAPos = new TPNT3D [pPath->m_nNumPnt];
		pPLin3D->m_nCount = pPath->m_nNumPnt - 1;
		int n = 0;
		for(CSmtCutPoint * p=pPath->m_pHead; p; p=p->next)
		{
			nc_VectorCopy(pPLin3D->m_pTAPos[n], p->m_fPoint, 3);
			n++;
		}
		PathComb.AddEntity(pPLin3D);
	}
}
BOOL CPathSplFit::IsArcSeg(CSmtCutPath & CurrSeg)
{
	if(CurrSeg.m_nNumPnt<4) return FALSE;
	PNT3D dArcPt[3];
	CGeoArc arc;
	int nMid = CurrSeg.m_nNumPnt / 2;
	CSmtCutPoint * p = CurrSeg.m_pHead;
	for(int i=0 ; p ; p=p->next,i++)
	{
		if(i==1) nc_VectorCopy(dArcPt[0], p->m_fPoint, 3);
		if(i==nMid) break;
	}
	nc_VectorCopy(dArcPt[1], p->m_fPoint, 3);
	nc_VectorCopy(dArcPt[2], CurrSeg.m_pTail->m_fPoint, 3);
	
	if(arc.CreateBy3P(dArcPt[0], dArcPt[1], dArcPt[2], 0)==0)	return FALSE;
	float fCenter[3] = { (float)arc.m_lf.O[0], (float)arc.m_lf.O[1], (float)arc.m_lf.O[2]};
	double dLimitAng = ANGLE_TO_RADIAN(170);
	for(p = CurrSeg.m_pHead; p; p=p->next )
	{
		if(p==CurrSeg.m_pTail) break;
		if(p->m_fPoint[3] < dLimitAng || fabs( nc_Distance(p->m_fPoint, fCenter, 3) - arc.m_radius) >0.2 )	
			return FALSE;//记录该点的夹角太尖 或 该点不在假设的圆弧上(粗判)
	}
	return TRUE;
}
CPathArc3D * CPathSplFit::CreateArcPath(CGeoArc & arc)
{
	CPathArc3D *pArcPath = new CPathArc3D();
	pArcPath->m_fRadius = arc.m_radius;
	pArcPath->m_fAngle[0] = 0;
	pArcPath->m_fAngle[1] = arc.m_angle;
	memcpy(pArcPath->m_fCenter, arc.m_lf.O, sizeof(double[3]));
	pArcPath->m_bArcPlane = 3;
	if( ! pArcPath->m_fXYZ )  pArcPath->m_fXYZ = new VEC3D[3] ;
	memcpy(pArcPath->m_fXYZ[0], arc.m_lf.X, sizeof(double[3]));
	memcpy(pArcPath->m_fXYZ[1], arc.m_lf.Y, sizeof(double[3]));
	memcpy(pArcPath->m_fXYZ[2], arc.m_lf.Z, sizeof(double[3]));
	return pArcPath;
}

BOOL CPathSplFit::VerifyArc(CSmtCutPath & OrigCurv,CGeoArc & Arc)
{
	double dCurrDist, dCurrPt[3], prj_pt[3];
	for(CSmtCutPoint *p=OrigCurv.m_pHead; p; p=p->next)
	{
		nc_VectorCopy(dCurrPt, p->m_fPoint, 3);
		Arc.GetPosOnCir(dCurrPt, 0, prj_pt);
		dCurrDist = nc_Distance(dCurrPt, prj_pt, 3 );
		if(dCurrDist  > m_dFitTol ) 
			return FALSE;
	}
	return TRUE;
}
CGeoSpline * CPathSplFit::CreateCircle(CSmtCutPath * pCurrPath)
{
	if(pCurrPath )
	{
		CGeoSpline *pSpline = m_pPSmooth->CreateFairSpline(*pCurrPath);
		if(pSpline)
		{
			CCurveFair fair ;
			fair.SetCheckPntNum( max(100, pCurrPath->m_nNumPnt*2) );
			fair.FairGeoSpline( pSpline, 0.00001, CURVEFAIR_FIXENDTAN,CURVEFAIR_FIXENDTAN ) ;
			return pSpline;
		}
	}
	return NULL;
}

static BOOL GetNearestPtFromPath(PNT3D pt,CSmtCutPath * pPath,
								 PNT3D & nearest_p,double & dNearestDis)
{
	if(!pPath)
		return FALSE;

	PNT3D begin,end,curnearestp;
	dNearestDis = 99999999.0;
	BOOL ret = FALSE;
	for(CSmtCutPoint * p=pPath->m_pHead;p&&p->next;p=p->next)
	{
		nc_VectorCopy(begin, p->m_fPoint, 3);
		nc_VectorCopy(end, p->next->m_fPoint, 3);
		double curDis = mathGetPntSegmNearestPnt(pt,begin,end,curnearestp);
		if(curDis<dNearestDis)
		{
			dNearestDis = curDis;
			nc_VectorCopy(nearest_p, curnearestp, 3);
			ret = TRUE;
		}
	}
	return ret;
}
static BOOL VerifySpline(CSmtCutPath & Path, CGeoSpline & Spln, double Tol)
{
	double dDist, dStep = 0.1;//使用内插点后，变形量会小一些，减少校验点提高计算速度
	PNT3D dCurrPt, dPrjPt;
	for(double u = dStep; u<1; u+=dStep)
	{
		Spln.GetPoint(u, dCurrPt);
		if(GetNearestPtFromPath(dCurrPt, &Path, dPrjPt, dDist) && dDist>Tol)
			return FALSE;
	}
	return TRUE;
}

void CPathSplFit::SplFittingOnePath(CSmtCutPath * pCurrPath, CPathCombine & PathComb)
{
	if(pCurrPath==NULL || pCurrPath->m_nNumPnt<3)
	{
		AddPathPLine3D(pCurrPath, PathComb);
		return;
	}

	if(m_pPSmooth==NULL)
	{
		m_pPSmooth = new CPathSmooth;
	}

	FPNT3D vFirstTan, vLastTan, vHeadTan, vTailTan ;
	BOOL bClosed = pCurrPath->IsClosed();
	CTypedPtrArray<CPtrArray,CSmtCutPath*> cBrkPathArr;
	//将路径按角度、单段线段最大长度和单段样条最大长度打断分段
	BreakPathByAngAndMaxPt(*pCurrPath, BRKSEG_ANG, LINESEG_MAXLEN, ONESPL_MAXPNT, cBrkPathArr);
	int nBrkPath = (int)cBrkPathArr.GetCount();
	if(nBrkPath == 1 && bClosed && !IsArcSeg(*pCurrPath))
	{//由于光顺周期样条曲线没有端点条件，为了使封闭路径首末点固定须打断路径
		ClearPathArr(cBrkPathArr);
		BreakPathByLen(*pCurrPath, pCurrPath->GetLength()*0.6, cBrkPathArr);
		nBrkPath = (int)cBrkPathArr.GetCount();
	}
	if(nBrkPath<1)	
	{	
		return; 
	}
	BOOL bSharpClosed = FALSE;
	if(bClosed )
	{
		double dHTAng = GetTriVertAng(pCurrPath->m_pTail->prev, pCurrPath->m_pHead,pCurrPath->m_pHead->next);
		if(dHTAng>0 && dHTAng<ANGLE_TO_RADIAN(BRKSEG_ANG))
		{//闭合曲线首末点是否尖锐
			bSharpClosed = TRUE;
		}
	}
	CCurveFair cSplBrk;
	CGeoCurveList cSubSplList;
	double dStartAng, dCloseHead[3], dNewHead[3];
	for(int nPath=0; nPath<nBrkPath; nPath++)
	{
		CSmtCutPath *pPath = cBrkPathArr.GetAt(nPath);
		if(pPath==NULL )  continue; 
		GetPathHeadTan(pPath, vHeadTan);
		GetPathTailTan(pPath, vTailTan);
		if(nPath==0) nc_VectorCopy(vFirstTan, vHeadTan, 3);
		double dLen = pPath->GetLength();
		if(pPath->m_nNumPnt<3 || dLen<ONESPL_MINLEN ) 
		{//长直线段
			AddPathPLine3D(pPath, PathComb);
			nc_VectorCopy(vLastTan, vTailTan, 3);
			delete pPath;
			pPath = NULL;
			continue;
		}

		int nPtNum = max( 10 , int(dLen/SPLINE_VRFYINC) );
		m_pPSmooth->m_nMaxCheckNum = nPtNum; 
		BOOL bIsArcSeg = IsArcSeg(*pPath);
		double dOrigTol = m_dFitTol;
		if(bIsArcSeg)	m_dFitTol *= 0.1;
		else			m_dFitTol *= 0.5;
		if(!bIsArcSeg)	
		{
			GetWeakenCornerPath(*pPath);//计算路径内插点
			pPath->InsertCPoint3D(0.15);
		}
		CGeoSpline * pSpline = NULL;
		if(nPath==0)
		{
			if(pPath->IsClosed() && bIsArcSeg)	pSpline = CreateCircle(pPath);
			else	pSpline = m_pPSmooth->Smooth(*pPath, m_dFitTol, vHeadTan, vTailTan);
		}
		else if(bClosed && nPath==nBrkPath-1 )  
		{//周期样条曲线最后一段
			if(pPath->m_pHead->m_bType != PNTTYPE_SHARP )//与上一段为非尖点断开、非尖点断开处切矢连续
			{
				if(bSharpClosed)
					pSpline = m_pPSmooth->Smooth(*pPath, m_dFitTol, vLastTan, vTailTan);
				else
					pSpline = m_pPSmooth->Smooth(*pPath, m_dFitTol, vLastTan, vFirstTan);
			}
			else 
			{
				if(bSharpClosed)
					pSpline = m_pPSmooth->Smooth(*pPath, m_dFitTol, vHeadTan, vTailTan);
				else
					pSpline = m_pPSmooth->Smooth(*pPath, m_dFitTol, vHeadTan, vFirstTan);
			}
		}
		else 
		{
			if(pPath->m_pHead->m_bType != PNTTYPE_SHARP )//与上一段为非尖点断开、非尖点断开处切矢连续
				pSpline = m_pPSmooth->Smooth(*pPath, m_dFitTol, vLastTan, vTailTan);
			else
				pSpline = m_pPSmooth->Smooth(*pPath, m_dFitTol, vHeadTan, vTailTan);
		}
		m_dFitTol = dOrigTol;
		nc_VectorCopy(vLastTan, vTailTan, 3);
		if(pSpline==NULL)
		{//异常
			ASSERT(0);
			AddPathPLine3D(pPath, PathComb);
			delete pPath;
			pPath = NULL;
			continue;
		}
		if(!bIsArcSeg)  pPath->DelPointOnLine(1.0e-5);
		if(!VerifySpline(*pPath, *pSpline, m_dFitTol+0.001))
		{//样条误差不满足
			delete pSpline;
			AddPathPLine3D(pPath, PathComb);
			delete pPath;
			pPath = NULL;
			continue;
		}
		nc_VectorCopy(dCloseHead, pPath->m_pHead->m_fPoint, 3);

		//样条减点 误差累加可能会超出允许的偏差
		//pSpline->RemoveKnots(m_dFitTol*0.1);

		if(bIsArcSeg)
		{
			CGeoConvert convt;
			CGeoArc * pArc = convt.ConvertNurbsToArc(pSpline, m_dFitTol*0.4);
			if(pArc)
			{
				BOOL bValidArc = TRUE;
				if(pSpline->IsClosed())
				{
					pArc->GetPosAngle(dCloseHead, dStartAng);
					if(fabs(dStartAng)>1.0e-6)	pArc->Rotate(pArc->m_lf.O, pArc->m_lf.Z, dStartAng);
					pArc->GetPoint(0, dNewHead);
					if(nc_Distance(dNewHead, dCloseHead, 3)>0.0001) bValidArc = FALSE;//不满足误差，会造成多余抬刀
					pArc->GetPoint(1, dNewHead);
					if(nc_Distance(dNewHead, dCloseHead, 3)>0.0001) bValidArc = FALSE;//不满足误差，会造成多余抬刀
					if(!VerifyArc(*pPath, *pArc)) bValidArc = FALSE;
				}
				if(bValidArc == TRUE)
				{
					CPathArc3D * pArcPath = CreateArcPath(*pArc);
					pArcPath->m_bFeedType = pCurrPath->m_bFeedType;   pArcPath->m_fFeedRate = pCurrPath->m_fFeedRate;
					pArcPath->m_bMoveFlag = pCurrPath->m_bMoveFlag;	  pArcPath->m_bFeedType = pCurrPath->m_bFeedType;
					PathComb.AddEntity(pArcPath);
					delete pPath;       pPath= NULL;
					delete pSpline;		pSpline = NULL;
					delete pArc;		pArc = NULL;
					continue;
				}
				delete pArc;		
				pArc = NULL;
			}
		}
		delete pPath;
		pPath = NULL;
		

		//不再按照曲率打断样条
		CPathNurbs3D *pNbsPath = new CPathNurbs3D();
		pNbsPath->CreateNurbs(pSpline->m_p+1, pSpline->m_n);
		memcpy(pNbsPath->m_fPoint, pSpline->m_Pw, (pNbsPath->m_nCount + 1 ) * sizeof(double[4]) );
		memcpy(pNbsPath->m_fKnot, pSpline->m_U, (pNbsPath->m_nCount + pNbsPath->m_nOrder +1 ) * sizeof(double) );
		pNbsPath->m_bFeedType = pCurrPath->m_bFeedType;   pNbsPath->m_fFeedRate = pCurrPath->m_fFeedRate;
		pNbsPath->m_bMoveFlag = pCurrPath->m_bMoveFlag;	  pNbsPath->m_bFeedType = pCurrPath->m_bFeedType;
		PathComb.AddEntity(pNbsPath);

		delete pSpline;
		pSpline = NULL;
	}
}

BOOL CPathSplFit::SplFit(CSmtCPathLib& AllPath,   /*所有路径*/
						 double FitTol ,          /*样条拟合误差*/
						 CPathCombine & PathComb, /*样条拟合结果*/
						 JDNC_PRGDEF& PrgDef)
{
	if(fabs(FitTol)>0.0001 )
	{
		double dTol = FitTol;
		if(dTol > 0.2)	dTol = 0.2;
		else if( dTol < 0.002)	dTol = 0.002;
		m_dFitTol = dTol;
		POSITION pAtPos = AllPath.m_cAllPath.GetHeadPosition();
		while(pAtPos)
		{
			if( PrgDef.m_pBrkFunc && PrgDef.m_pBrkFunc())	
			{
				return FALSE; 
			}
			PrgDef.m_dStepAt += PrgDef.m_dIncStep ;
			while( PrgDef.m_pPrgFunc && PrgDef.m_dStepAt >= PrgDef.m_dLimitAt )	
			{
				PrgDef.m_pPrgFunc( 1 ) ;
				PrgDef.m_dStepAt -= PrgDef.m_dLimitAt ;
			}
			CSmtCutPath * pCurrPath = AllPath.m_cAllPath.GetNext(pAtPos);
			if(pCurrPath && pCurrPath->GetCutMode()==MINI_MILL_PATH && pCurrPath->m_nNumPnt>3 )
			{
				if(m_nGenType==BSplInterpolateFit)
					SplFittingOnePath(pCurrPath, PathComb);
				else
					SectBezierFitOnePath(*pCurrPath, PathComb);
			}
			else 
			{
				CSmtCPathLib cTmpPathLib;
				cTmpPathLib.AddToTail(pCurrPath->CopyMyself());
				cTmpPathLib.AddToPathCombine( PathComb ) ;
			}
		}
	}
	return TRUE;
}

BOOL CPathSplFit::FitBySpline( CPathCombine& AllPath, JDNC_TOL& Tol, PNT3D Point[], int NumPnt )
{
	if( (Point==NULL) || (NumPnt<2) || (Tol.m_nFitType != NCDEF_FITTYPE_SPLINE) )
	{
		return FALSE;
	}

	m_nGenType = BSplInterpolateFit;
	m_dFitTol = Tol.m_dArcTol;
	CSmtCutPath * pCurrPath = new CSmtCutPath();
	for(int i=0; i<NumPnt; i++)
	{
		pCurrPath->AddPoint(Point[i]);
	}
	SplFittingOnePath(pCurrPath, AllPath);
	delete pCurrPath;
	pCurrPath = NULL;
	return TRUE;
}

static BOOL IsValidParam(double t) {	return (t>=0 && t<=1 ? TRUE : FALSE);  }
//计算路径内插点
BOOL CPathSplFit::GetWeakenCornerPath(CSmtCutPath & CurrPath)
{
	if(CurrPath.m_pHead == NULL || CurrPath.m_pTail == NULL ) return FALSE;

	CSmtCutPoint * pPoint = CurrPath.m_pHead;
	PNT3D dIntPt[2], dArcPt[3];
	double t1[2], t2[2];
	double dTol = m_dFitTol ;
	CGeoInter geoint;
	int pn0, pn1;
	double dLimitAng = ANGLE_TO_RADIAN(179);
	while(pPoint)
	{
		CSmtCutPoint * pPrev = pPoint->prev;
		CSmtCutPoint * pNext = pPoint->next;
		if(pPrev && pNext && GetTriVertAng(pPrev, pPoint, pNext) < dLimitAng )
		{
			CGeoArc arc;
			nc_VectorCopy(dArcPt[0], pPrev->m_fPoint, 3);
			nc_VectorCopy(dArcPt[1], pPoint->m_fPoint, 3);
			nc_VectorCopy(dArcPt[2], pNext->m_fPoint, 3);
			if( arc.CreateBy3P(dArcPt[0], dArcPt[1], dArcPt[2], 0) )
			{
				arc.m_radius -= dTol;
				CGeoLine cLinSeg0(dArcPt[0], dArcPt[1]);
				int nRet = geoint.CalcLineArcInt(&cLinSeg0, &arc, t1, t2, dIntPt, pn0);
				if(pn0 == 1 && IsValidParam(t1[0]) && IsValidParam(t1[1]))//加入内插点
					CurrPath.InsertBefore(new CSmtCutPoint(dIntPt[0]), pPoint);
				else if(pn0 == 2 && IsValidParam(t1[0]) &&  IsValidParam(t2[0]) && IsValidParam(t1[1]) &&  IsValidParam(t2[1]))
				{//加入内插点
					if(t1[0] < t2[0])
					{
						CurrPath.InsertBefore(new CSmtCutPoint(dIntPt[0]), pPoint);
						CurrPath.InsertBefore(new CSmtCutPoint(dIntPt[1]), pPoint);
					}
					else
					{
						CurrPath.InsertBefore(new CSmtCutPoint(dIntPt[1]), pPoint);
						CurrPath.InsertBefore(new CSmtCutPoint(dIntPt[0]), pPoint);
					}
				}

				CGeoLine cLinSeg1(dArcPt[1], dArcPt[2]);
				nRet = geoint.CalcLineArcInt(&cLinSeg1, &arc, t1, t2, dIntPt, pn1);
				if(pn1 == 1 && IsValidParam(t1[0]) && IsValidParam(t1[1]) )//加入内插点
					CurrPath.InsertAfter(new CSmtCutPoint(dIntPt[0]), pPoint);
				else if(pn1 == 2 && IsValidParam(t1[0]) &&  IsValidParam(t2[0]) && IsValidParam(t1[1]) &&  IsValidParam(t2[1]))
				{//加入内插点
					if(t1[0] < t2[0])
					{
						CurrPath.InsertAfter(new CSmtCutPoint(dIntPt[0]), pPoint);
						CurrPath.InsertAfter(new CSmtCutPoint(dIntPt[1]), pPoint);
					}
					else
					{
						CurrPath.InsertAfter(new CSmtCutPoint(dIntPt[1]), pPoint);
						CurrPath.InsertAfter(new CSmtCutPoint(dIntPt[0]), pPoint);
					}
				}

				if(pn0 > 0 &&  pn1 == 0)
				{
					CurrPath.DeletePoint(pPoint);
					pPoint = pNext;
					continue;
				}
				else if(pn1 > 0 )
				{
					CurrPath.DeletePoint(pPoint);
					pPoint = pNext->next;
					continue;
				}
			}
		}
		pPoint = pNext;
	}
	return TRUE;
}
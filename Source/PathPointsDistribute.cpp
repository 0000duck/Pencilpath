#include "StdAfx.H"
#include "PathPointsDistribute.h"
#include "model.h"
#include "global.h"
#include "entcurve3d.h"
#include <vector>

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char BASED_CODE THIS_FILE[] = __FILE__;
#endif

struct Point3D//���ڼ�¼��Ľṹ
{
	PNT3D m_dPt ;
};

//�Ƚ����߲��,�ٽ��о��Ȼ�
CPathEntity* MathCAM_MaxPointDistance_PathEntity( CPathEntity *pPath,
																								DOUBLE ArcTol,
																								DOUBLE AngTol,
																								DOUBLE MaxStep )
{
	if( !pPath )
		return FALSE ;
	// step 1: ȡ��·����
	int nCount = 0;
	PNT3D* ptArr = NULL ;
	if( pPath->GetType() == NC_PATH_LINE3D )
	{
		CPathLine3D *pLine3D = (CPathLine3D *)pPath ;
		nCount = 2 ;
		ptArr = new PNT3D[nCount + 1];
		memcpy(ptArr[0],pLine3D->m_fStart,sizeof(PNT3D)) ;
		memcpy(ptArr[1],pLine3D->m_fEnd,sizeof(PNT3D)) ;
	}
	else if( pPath->GetType() == NC_PATH_PLINE3D )
	{
		CPathPLine3D *pPLine = (CPathPLine3D *)pPath ;
		nCount = pPLine->m_nCount+1 ;
		ptArr = new PNT3D[nCount + 1];
		for( int i=0 ; i<nCount ; i++ )
			memcpy(ptArr[i],pPLine->m_pTAPos[i],sizeof(PNT3D)) ;
	}	
	else
		return FALSE;

	//step2:�ڳ����ϲ��
	PNT3D pt1,pt2,pt3 ;
	double dPointStep = 2.0;//����Ԥ�Ȳ�㲽��
	std::vector<Point3D> vPts ;
	Point3D point ;
	double dLen = 0.0 ;
	double dStep = 0.0 ;
	int num = 0 ;
	mathCpyPnt(ptArr[0],pt1) ;
	mathCpyPnt(pt1,point.m_dPt) ;
	vPts.push_back(point) ;
	for(int i=1 ; i<nCount ;i++)
	{
		mathCpyPnt(ptArr[i],pt2) ;
		//���������������󲽳�,���ӵ�
		dLen = mathDist(pt1,pt2) ;
		num  = (int)(dLen / dPointStep) + 2 ;
		dStep = 1.0 / (double)(num-1) ;
		for( int j=1 ; j<num ; j++ )
		{
			for( int k=0 ; k<3 ; k++)
				pt3[k] = pt1[k] + j*dStep*(pt2[k]-pt1[k]) ;
			mathCpyPnt(pt3,point.m_dPt) ;
			vPts.push_back(point) ;
		}
		mathCpyPnt(pt2,pt1) ;
	}
	delete[]ptArr ;//����

	nCount = (int)vPts.size() ;	
	ptArr = new PNT3D[nCount + 1];
	for( int i=0 ; i<nCount ;i++)
		memcpy(ptArr[i],vPts[i].m_dPt,sizeof(PNT3D)) ;
	vPts.clear() ;

	// step 3: �ֶ�ת��������
	CGeoConvert convert;
	CGeoCurveList listCurve;
	double dAngle = 10 * PAI/180 ;//�ֶνǶȲ���
	convert.ConvertPtsToSmoothSegs(ptArr,nCount, ArcTol,dAngle , &listCurve);
	delete[]ptArr ;//����

	// step 4: ����������ɢ,��ת��·��
	CStrpt* pStrpt = NULL;
	CGeoCurve* pCurve = NULL;
	CTypedPtrList<CPtrList,CStrpt*>	strptList ;

	while (!listCurve.IsEmpty())
	{
		pCurve = listCurve.RemoveHead();
		if (!pCurve)
			continue;

		pStrpt = DiscreteCurveToStrptForPtReDistrbt(pCurve, MaxStep, ArcTol, AngTol);
		delete pCurve;
		if (!pStrpt)
			continue;
		strptList.AddTail(pStrpt) ;
	}

	//ת��һ��·��
	int nPts = 0 ;
	POSITION pos = strptList.GetHeadPosition() ;
	while(pos)//ͳ�Ƶ���
	{
		pStrpt = strptList.GetNext(pos) ;
		if(nPts == 0)
		{
			nPts += pStrpt->m_np ;
		}
		else
		{
			mathCpyPnt(pStrpt->m_ps[0],pt2) ;
			if( mathDist(pt1,pt2)<1e-3 )
				nPts += pStrpt->m_np-1 ;
			else
				nPts += pStrpt->m_np ;
		}
		mathCpyPnt(pStrpt->m_ps[pStrpt->m_np-1],pt1) ;
	}
	//����·��
	CPathPLine3D *pLine = new CPathPLine3D() ;
	pLine->m_nCount = nPts-1  ;
	pLine->m_pTAPos = new PNT3D[nPts] ;
	nPts = 0 ;

	while (!strptList.IsEmpty())
	{
		pStrpt = strptList.RemoveHead();
		if (!pStrpt)
			continue;

		if(nPts == 0)
		{
			for( int i=0 ; i<pStrpt->m_np ; i++ )
			{
				mathCpyPnt( pStrpt->m_ps[i] , pLine->m_pTAPos[nPts] ) ;
				nPts ++ ;
			}
		}
		else
		{
			mathCpyPnt(pStrpt->m_ps[0],pt2) ;
			for( int i=0 ; i<pStrpt->m_np ; i++ )
			{
				if( i==0 )
				{
					if( mathDist(pt1,pt2)>1e-3)
					{
						mathCpyPnt( pStrpt->m_ps[i] , pLine->m_pTAPos[nPts] ) ;
						nPts ++ ;
					}
				}
				else
				{
					mathCpyPnt( pStrpt->m_ps[i] , pLine->m_pTAPos[nPts] ) ;
					nPts ++ ;
				}
			}
		}
		mathCpyPnt(pStrpt->m_ps[pStrpt->m_np-1],pt1) ;
		delete pStrpt ;
	}
	pLine->m_bFeedType = pPath->m_bFeedType ;
	pLine->m_fFeedRate = pPath->m_fFeedRate ;//����ԭ·�����ٶȲ���
	pLine->m_fFeedScale = pPath->m_fFeedScale ;
	pLine->m_bMoveFlag = pPath->m_bMoveFlag ;
	CPathEntity *pNewPath = NULL ;
	if( pLine )
		pNewPath = (CPathEntity *)pLine ;

	return pNewPath ;
}

//���Բ��
void MathCAM_ReplaceArc_PathGroup(CPathGroup	*NewPath,
								  int			ReDistFlag,
								  DOUBLE		ArcTol,
								  DOUBLE		AngTol,
								  BOOL			bMaxDist,
								  double		dMaxDist)
{
	if( !NewPath || !NewPath->m_pHead )
		return ;


	AngTol = ANGLE_TO_RADIAN(AngTol) ;
	CPathCombine *pComb = NewPath->m_pHead ;
	CPathEntity *pPath = NULL ;
	CPathPLine3D *pLine = NULL ;
	CPathEntity *pNewPath = NULL ;
	int nNum = 2000 ;
	PNT3D Buffer[2000] ;

	while( pComb )
	{
		pPath = pComb->m_pHead ;
		while(pPath)
		{
			if( (pPath->m_bFeedType == JDNC_FEEDTYPE_ROUGH && (ReDistFlag & REDISTRIBUTE_ROUTH)) || 
				(pPath->m_bFeedType == JDNC_FEEDTYPE_LEAD && (ReDistFlag & REDISTRIBUTE_LEAD)) ||
				(pPath->m_bFeedType == JDNC_FEEDTYPE_PLUNGE && (ReDistFlag & REDISTRIBUTE_PLUNGE)) ||
				(pPath->m_bFeedType == JDNC_FEEDTYPE_CONNECT && (ReDistFlag & REDISTRIBUTE_CONNECT)) ||
				(pPath->m_bFeedType == JDNC_FEEDTYPE_SLOT && (ReDistFlag & REDISTRIBUTE_SLOT)))
			{
				if( pPath->GetType() == NC_PATH_ARC3D)
				{
					nNum = 2000 ;
					CPathArc3D *pArc = (CPathArc3D*)pPath;
					nNum = pArc->Discrete(ArcTol,AngTol,Buffer,nNum) ;
					//����·��
					pLine = new CPathPLine3D() ;
					pLine->m_nCount = nNum  ;
					pLine->m_pTAPos = new PNT3D[pLine->m_nCount+1] ;
					for( int i=0 ; i<nNum+1 ; i++)
						mathCpyPnt( Buffer[i] , pLine->m_pTAPos[i] ) ;
					pLine->m_bFeedType = pPath->m_bFeedType ;
					pLine->m_fFeedRate = pPath->m_fFeedRate;
					pLine->m_fFeedScale = pPath->m_fFeedScale;
					pLine->m_bMoveFlag = pPath->m_bMoveFlag;
					pNewPath = (CPathEntity*)pLine ;
					// Բ������ҲҪ���·ֲ� 2014.3.11 liuxin
					pComb->InsertAfter(pNewPath,pPath) ;
					pComb->RemoveEntity(pPath) ;
					delete pPath ;
					pPath = pNewPath ;
					pNewPath = NULL ;
				}//���ƽڵ㲽��
				if( pPath->GetType() == NC_PATH_PLINE3D || pPath->GetType() == NC_PATH_LINE3D  )
				{
					if( bMaxDist)
						pNewPath = MathCAM_MaxPointDistance_PathEntity(pPath,ArcTol,AngTol,dMaxDist) ;
				}	
				if( pNewPath )
				{
					pComb->InsertAfter(pNewPath,pPath) ;
					pComb->RemoveEntity(pPath) ;
					delete pPath ;
					pPath = pNewPath ;
					pNewPath = NULL ;
				}
			}
			pPath = pPath->next ;
		}
		pComb = pComb->next ;
	}	
}

void MathCAM_MaxPointDistance_PathGroup(CPathGroup *NewPath,
										int			ReDistFlag,
										DOUBLE		ArcTol,
										DOUBLE		AngTol,
										DOUBLE		MaxStep )
{
	if( !NewPath || !NewPath->m_pHead )
		return ;
	AngTol = ANGLE_TO_RADIAN(AngTol) ;
	CPathCombine *pComb = NewPath->m_pHead ;
	CPathEntity *pPath = NULL ;
	CPathEntity *pNewPath = NULL ;
	while( pComb  )
	{
		pPath = pComb->m_pHead ;
		while(pPath)
		{
			if( (pPath->m_bFeedType == JDNC_FEEDTYPE_ROUGH && (ReDistFlag & REDISTRIBUTE_ROUTH)) || 
				(pPath->m_bFeedType == JDNC_FEEDTYPE_LEAD && (ReDistFlag & REDISTRIBUTE_LEAD)) ||
				(pPath->m_bFeedType == JDNC_FEEDTYPE_PLUNGE && (ReDistFlag & REDISTRIBUTE_PLUNGE)) ||
				(pPath->m_bFeedType == JDNC_FEEDTYPE_CONNECT && (ReDistFlag & REDISTRIBUTE_CONNECT)) ||
				(pPath->m_bFeedType == JDNC_FEEDTYPE_SLOT && (ReDistFlag & REDISTRIBUTE_SLOT)))
			{
				if( pPath->GetType() == NC_PATH_PLINE3D || pPath->GetType() == NC_PATH_LINE3D  )
				{
					pNewPath = MathCAM_MaxPointDistance_PathEntity(pPath,ArcTol,AngTol,MaxStep) ;
					if( pNewPath )
					{
						pComb->InsertAfter(pNewPath,pPath) ;
						pComb->RemoveEntity(pPath) ;
						delete pPath ;
						pPath = pNewPath ;
						pNewPath = NULL ;
					}
				}
			}
			pPath = pPath->next ;
		}
		pComb = pComb->next ;
	}	
}
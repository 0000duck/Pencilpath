#include "StdAfx.H"
#include "SmtNewUWLinePath.H"
#include "mathcam.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

//////////////////////////////////////////////////////////////////////////////////////////
//CSmtNewUWLinePathGen : 曲面流线曲面精雕刻
//
CSmtNewUWLinePathGen::CSmtNewUWLinePathGen()
{
}

CSmtNewUWLinePathGen::~CSmtNewUWLinePathGen()
{
}

int CSmtNewUWLinePathGen::GeneratePathEx( CPathGroup&    NewPath ,  /*雕刻路径*/
	CSmartGraphic& Graph   )  /*雕刻图形*/
{ 
	UpdateTool3D(Graph);
	C3DSurfArray AllSurf;
	Graph.GetAllSurface(m_cSetupDef,AllSurf);
	
	int surfCount = AllSurf.GetSize();
	if (!surfCount) return FALSE;

	CSmartPathGen pPathGen;
	JDNC_TOL cTol = {0.002,10,0,0,0};
	
	BOOL bRet = FALSE;
	for (int i = 0; i < surfCount; i++ )
	{
		CGeoTrmSurf* pSurf = (CGeoTrmSurf*) AllSurf[i];
		if (!pSurf) continue;
		CSmartLoop* AllLoop = pPathGen.ExtractSurfLoop(pSurf,cTol);
		if (CreatOneSurfPath(AllLoop, pSurf, NewPath)) 
			bRet = TRUE;
	}
	
	return bRet;
}

//生成单个面的加工路径
BOOL CSmtNewUWLinePathGen::CreatOneSurfPath(CSmartLoop* AllLoop, CGeoTrmSurf* pSurf,CPathGroup& NewPath)
{
	CSmtCPathLib AllPath ;
	CSmtCutPath *pNewPath = new CSmtCutPath;	

	float fBox2d[2][2];
	for (int i = 0; i <2; i++)
	{
		for (int j = 0; j<2; j++)
		{
			fBox2d[i][j] =(float) AllLoop->m_dBox[i][j];
		}
	}
	int pCutDir = m_cParam.m_nCutDir;

	CreatParamPath(pSurf,*pNewPath, fBox2d, pCutDir);
	if (CreatFinalPath(AllLoop, AllPath, pNewPath, pSurf, fBox2d, pCutDir))
	{
		CPathCombine *pPComb = new CPathCombine(NC_WPROCESS_ROUGH);
		AllPath.AddToPathCombine(*pPComb);
		NewPath.AddData(0,pPComb);
		return TRUE;
	}
	else
		return FALSE;
}

//计算参数域路径
void CSmtNewUWLinePathGen::CreatParamPath(CGeoTrmSurf* pSurf, CSmtCutPath& pNewPath, float fBox2d[2][2], int pCutDir)
{
	CSmtCutPath *pPath[3] = {NULL,NULL,NULL};
	PNT4D ptStart[3] = { NULL, NULL, NULL };
	PNT4D ptEnd[3] = { NULL, NULL, NULL };
	if (pCutDir)
	{
		ptStart[0][1] = ptEnd[0][1] = fBox2d[0][1];
		ptStart[0][0] = ptStart[2][0] = ptStart[1][0] = fBox2d[0][0];
		ptStart[1][1] = ptEnd[1][1] = (fBox2d[0][1] + fBox2d[1][1])*0.5;		
		ptStart[2][1] = ptEnd[2][1] = fBox2d[1][1];	
		ptEnd[0][0] = ptEnd[1][0] = ptEnd[2][0] = fBox2d[1][0];
	}
	else
	{
		ptStart[0][0] = ptEnd[0][0] = fBox2d[0][0];
		ptStart[0][1] = ptStart[1][1] = ptStart[2][1] = fBox2d[0][1];
		ptStart[1][0] = ptEnd[1][0] = (fBox2d[0][0] + fBox2d[1][0])*0.5;
		ptStart[2][0] = ptEnd[2][0] = fBox2d[1][0];
		ptEnd[0][1] = ptEnd[1][1] = ptEnd[2][1] = fBox2d[1][1];
	}

	for (int j = 0 ; j<3 ;j++)
	{
		pPath[j] = new CSmtCutPath;
		CSmtCutPointEx *ptStartEx = new CSmtCutPointEx;
		CSmtCutPointEx *ptEndEx = new CSmtCutPointEx;
		for(int i = 0; i < 4; i++)
		{
			ptStartEx->m_fPoint[i] = (float)ptStart[j][i];
			ptEndEx->m_fPoint[i] = (float)ptEnd[j][i];
		}
		pPath[j]->AddTail(ptStartEx);
		pPath[j]->AddTail(ptEndEx);
		pPath[j]->DefineBox() ;
	}
	double lengthMax = 0.0;
	int nMax = 0;
	for (int i = 0; i<3; i++)
	{
		MappingPath(pPath[i],pSurf);
		double length = pPath[i]->GetLength();
		if (length > lengthMax)
		{
			lengthMax = length;
			nMax = i;
		}
	}
	for (int i = 0; i<3; i++)
	{
		if (i != nMax)
			delete pPath[i];
	}

	CreatStepPath(pPath[nMax], pNewPath);
}

//生成路径节点满足路径间距的参数域路径
void CSmtNewUWLinePathGen::CreatStepPath(CSmtCutPath* ParamPath, CSmtCutPath& pNewPath)
{
	double dDisSum = 0.;
	double length = ParamPath->GetLength();	
	double dStep = m_cParam.m_dOverStep ;

	int nCount = (int)ceil(length/dStep);
	double dStepNew = length / nCount;
	CSmtCutPointEx* ptHead = (CSmtCutPointEx*)ParamPath->m_pHead;
	CSmtCutPointEx* ptFst = new CSmtCutPointEx;
	ptFst = (CSmtCutPointEx*)ParamPath->m_pHead->CopyMyself();
	pNewPath.AddTail(ptFst);
	while(ptHead)
	{
		CSmtCutPointEx* ptNext = (CSmtCutPointEx*) ptHead->next;
		if (!ptNext) 
		{
			pNewPath.AddTail((CSmtCutPoint*)ptHead);
			break;
		}
		double dis = mathFDist(ptHead->m_fPoint,ptNext->m_fPoint);
		dDisSum = dDisSum + dis;
		if(dDisSum > dStepNew)
		{
			double t = (dStepNew - (dDisSum - dis))/dis;
			CSmtCutPointEx* ptIn = new CSmtCutPointEx;
			for (int i = 0; i < 3; i++)
			{
				ptIn->m_fPoint[i] =(float)( ptHead->m_fPoint[i] + t * ( ptNext->m_fPoint[i] - ptHead->m_fPoint[i]));
				ptIn->m_fSurfNor[i] =(float)( ptHead->m_fSurfNor[i] + t * ( ptNext->m_fSurfNor[i] - ptHead->m_fSurfNor[i]));
				ptIn->m_fSurfPos[i] =(float)(ptHead->m_fSurfPos[i] + t * ( ptNext->m_fSurfPos[i] - ptHead->m_fSurfPos[i]));
			}
			dDisSum = 0;
			pNewPath.AddTail((CSmtCutPoint*)ptIn);
			CSmtCutPointEx* ptInNew = new CSmtCutPointEx;
			ptInNew = (CSmtCutPointEx*)ptIn->CopyMyself();
			ParamPath->InsertAfter((CSmtCutPoint*)ptInNew,(CSmtCutPoint*)ptHead);
			ptHead = (CSmtCutPointEx*) ptHead->next;
			continue;
		}
		ptHead = ptNext;
	}
}

//生成最终的曲面加工路径
BOOL CSmtNewUWLinePathGen::CreatFinalPath(CSmartLoop* AllLoop, CSmtCPathLib& AllPath, CSmtCutPath* pNewPath, CGeoTrmSurf* pSurf , float fBox2d[2][2], int pCutDir)
{
	CSmtCutPointEx* ptNewHead = (CSmtCutPointEx*)pNewPath->m_pHead;
	while(ptNewHead)
	{
		CSmtCutPointEx* ptNewNext = (CSmtCutPointEx*) ptNewHead->next;

		CSmtCutPointEx* ptFEnd[2] = {NULL, NULL};
		ptFEnd[0] = new CSmtCutPointEx;
		ptFEnd[1] = new CSmtCutPointEx;

		if (pCutDir)
		{
			if (m_cFeedDef.m_cConnectDef.m_bConnect3DFlag & NCDEF_FCONNECT3D_SPIRAL)
			{
				if (!ptNewNext) break;
				ptFEnd[0]->m_fPoint[0] = ptNewHead->m_fSurfPos[0];
				ptFEnd[0]->m_fPoint[1] = fBox2d[0][1];
				ptFEnd[1]->m_fPoint[0] = ptNewNext->m_fSurfPos[0];
				ptFEnd[1]->m_fPoint[1] = fBox2d[1][1];
			}
			else
			{
				ptFEnd[0]->m_fPoint[0] = ptFEnd[1]->m_fPoint[0] = ptNewHead->m_fSurfPos[0];
				ptFEnd[0]->m_fPoint[1] = fBox2d[0][1];
				ptFEnd[1]->m_fPoint[1] = fBox2d[1][1];
			}
		}
		else
		{
			ptFEnd[0]->m_fPoint[0] = fBox2d[0][0];
			ptFEnd[0]->m_fPoint[1] = ptFEnd[1]->m_fPoint[1] = ptNewHead->m_fSurfPos[1];
			ptFEnd[1]->m_fPoint[0] = fBox2d[1][0];
		}
		ptFEnd[0]->m_fPoint[2] = ptFEnd[1]->m_fPoint[2] = 0;

		//计算符合加工面边界的最终路径
		PathTrim(AllLoop, pSurf, ptFEnd, AllPath);	

		if (!ptNewNext) break;
		ptNewHead = ptNewNext;
		continue;
	}	

	if (!AllPath.m_cAllPath.GetCount())	return FALSE;
	return TRUE;
}

//生成符合加工曲面边界的路径
void CSmtNewUWLinePathGen::PathTrim(CSmartLoop* AllLoop, CGeoTrmSurf* pSurf,CSmtCutPointEx* ptFEnd[2], CSmtCPathLib& AllPath)
{
	PNT2D pFirst,pSecond;
	PNT2D ptList[200];
	for (int i = 0; i < 2; i++)
	{
		pFirst[i] = ptFEnd[0]->m_fPoint[i];
		pSecond[i] = ptFEnd[1]->m_fPoint[i];
	}
	//判断生成的切割路径是否与加工面的边界环有交点，如果有交点，则取作为路径的始末点
	UINT ptIntNum = 200;
	ptIntNum = AllLoop->GetLineIntContour(pFirst, pSecond, ptList, ptIntNum);

	if(ptIntNum < 2|| ptIntNum%2 ) 	
	{
		delete ptFEnd[0];
		delete ptFEnd[1];
		return ;
	}

	for (int i = 0; i < (int)ptIntNum; i += 2)
	{
		CSmtCutPath* pFPath = new CSmtCutPath;
		for (int j = 0 ; j < 2; j++)
		{
			ptFEnd[0]->m_fPoint[j] =(float) ptList[i][j];
			ptFEnd[1]->m_fPoint[j] = (float)ptList[i+1][j];
		}
		pFPath->AddTail((CSmtCutPoint*) ptFEnd[0]);
		pFPath->AddTail((CSmtCutPoint*) ptFEnd[1]);
		MappingPath(pFPath,pSurf);
		pFPath->DefineBox();
		AllPath.AddToTail(pFPath);			
	}
}

//将参数域路径向曲面进行投影
void CSmtNewUWLinePathGen::MappingPath(CSmtCutPath* pCutPath, CGeoTrmSurf *pSurf)
{
	CSmtCutPointEx *pHead = (CSmtCutPointEx *) pCutPath->m_pHead;
	CSmtCutPointEx *pTail = (CSmtCutPointEx *) pCutPath->m_pTail;
	PNT2D ptOr1, ptOr2;
	for (int i = 0; i<2; i++)
	{
		ptOr1[i] = pHead->m_fSurfPos[i] = pHead->m_fPoint[i];//将参数域坐标保存在m_fSurfPos中
		ptOr2[i] = pTail->m_fSurfPos[i] = pTail->m_fPoint[i];
	}
	PNT3D ptCL[2], nor[2];
	MappingPnt(ptOr1,pSurf,ptCL[0],nor[0]);
	MappingPnt(ptOr2,pSurf,ptCL[1],nor[1]);

	for (int j = 0; j < 3; j++)
	{
		pHead->m_fPoint[j] = (float)ptCL[0][j];
		pTail->m_fPoint[j] = (float)ptCL[1][j];
	}

	while(pHead&&pHead->next)
	{
		if (pHead == pHead->next) break;
		
		PNT2D ptInsert2d ;//参数域坐标
		PNT3D ptInsert;//插入点         
		VEC3D ptInsertNor;//插入点在曲面上的法向

		//当pHead和ptNext不满足弦高误差时，向他们的中点插点，知道满足为止
		CSmtCutPointEx* ptNext = (CSmtCutPointEx*) pHead->next;
		for (int i = 0; i < 2; i++)
		{
			ptInsert2d[i] = (pHead->m_fSurfPos[i] + ptNext->m_fSurfPos[i])*0.5;
		}
		MappingPnt(ptInsert2d,pSurf,ptInsert,ptInsertNor);
		PNT3D ptStart, ptEnd;
		for (int i = 0; i < 3; i++)
		{
			ptStart[i] = pHead->m_fPoint[i];
			ptEnd[i] = ptNext->m_fPoint[i];
		}
		//判断是否满足弦高误差
		double dArc = mathDistPntLinEx(ptInsert,ptStart,ptEnd);
		if (dArc>1.0e-4)
		{
			CSmtCutPointEx* ptInCutPath = new CSmtCutPointEx;//插入到路径中的点
			for (int i = 0; i < 3; i++)
			{
				ptInCutPath->m_fPoint[i] = (float)ptInsert[i];
				ptInCutPath->m_fSurfNor[i] = (float)ptInsertNor[i];
			}
			for(int i = 0; i < 2; i++)
			{
				ptInCutPath->m_fSurfPos[i] =(float) ptInsert2d[i];
			}

			pCutPath->InsertAfter(ptInCutPath,pHead);
		}
		else
		{
			pHead = (CSmtCutPointEx*)pHead->next;
		}
	}
}

//将参数域点向曲面投影
void CSmtNewUWLinePathGen::MappingPnt(PNT2D ptOr, CGeoTrmSurf* pSurf, PNT3D ptCL, VEC3D nor)
{
	PNT3D ptSurf;
	pSurf->GetNormal(ptOr[0],ptOr[1],ptSurf,nor);
	for (int i = 0; i < 3; i++)
	{
		ptCL[i] = ptSurf[i];
	}
	CalCLPt(ptSurf,nor,ptCL);
}

//计算刀尖点
void CSmtNewUWLinePathGen::CalCLPt(PNT3D ptSurf, VEC3D nor, PNT3D ptCL)
{
	PNT3D axis = {0.0,0.0,1.0} ;
	int nType = m_pTool->GetType() ;
	if( nType == smtToolBall )
	{
		for( int i=0 ; i<3 ; i++ )
			ptCL[i] = ptSurf[i]+m_pTool->m_fRadius*(nor[i]-axis[i]) ;
	}
	else if ( nType == smtToolFlat )
	{
		double ax = mathOProduct(axis,nor) ;
		if( fabs(ax-1.0) > MIN_DIS )
		{
			for( int i=0 ; i<3 ;i++ )
				ptCL[i] = ptSurf[i] + m_pTool->m_fRadius*(nor[i]-ax*axis[i])/sqrt(1-ax*ax)  ;
		}
		else
		{
			for( int i=0 ; i<3 ;i++ )
				ptCL[i] = ptSurf[i]  ;			
		}
	}
	else if( nType == smtToolNose )
	{
		double ax = mathOProduct(axis,nor) ;
		if( fabs(ax-1.0) > MIN_DIS )
		{
			for( int i=0 ; i<3 ;i++ )
				ptCL[i] = ptSurf[i] + m_pTool->m_fBottom*(nor[i]-ax*axis[i])/sqrt(1-ax*ax) + (m_pTool->m_fRadius-m_pTool->m_fBottom)*(nor[i]-axis[i]) ;
		}
		else
		{
			for( int i=0 ; i<3 ;i++ )
				ptCL[i] = ptSurf[i]  ;			
		}
	}

}
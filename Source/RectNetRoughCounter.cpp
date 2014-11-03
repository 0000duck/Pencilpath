#include "stdafx.h"
#include "SmartNC.H"
#include "SysPrompt.h"
#include "SmartSurf.H"
#include "SurfGeo.h"
#include "CurveFair.H"
#include "geo_curve.h"
#include "RectNetRoughCounter.h"
#include <vector>
#include "omp.h"


#ifdef _DEBUG
#undef THIS_FILE
static char THIS_FILE[]=__FILE__;
#define new DEBUG_NEW
#endif


BOOL EnlargeRectMesh(CGeoNetSurf*& pSurf,int num)
{
	if (	pSurf == NULL || 
		!pSurf->IsRectMesh() || 
		num<=0)
		return FALSE;

	//if (	pSurf == NULL || 
	//		!pSurf->IsRectMesh() || 
	//		num<=0 || 
	//		2*num+pSurf->m_nNumVertX>10000 ||
	//		2*num+pSurf->m_nNumVertY>10000 )
	//	return FALSE;

	CGeoNetSurf* destSurf;
	destSurf = new CGeoNetSurf;
	destSurf->m_nNumVert = (2*num+pSurf->m_nNumVertX)*(2*num+pSurf->m_nNumVertY);
	destSurf->m_nNumVertX = 2*num+pSurf->m_nNumVertX;
	destSurf->m_nNumVertY = 2*num+pSurf->m_nNumVertY;
	destSurf->m_BasePoint.x = pSurf->m_BasePoint.x-num*pSurf->m_fStepX;
	destSurf->m_BasePoint.y = pSurf->m_BasePoint.y-num*pSurf->m_fStepY;
	destSurf->m_fStepX    = pSurf->m_fStepX;
	destSurf->m_fStepY    = pSurf->m_fStepY;

	int colNum = 2*num+pSurf->m_nNumVertX;
	int rowNum = 2*num+pSurf->m_nNumVertY;
	destSurf->m_aCoordX   = new float[colNum];
	destSurf->m_aCoordY   = new float[rowNum];
	destSurf->m_aCoordZ   = new float[rowNum*colNum];
	for (int i=0;i<colNum;i++)
		destSurf->m_aCoordX[i]   = destSurf->m_BasePoint.x+i*destSurf->m_fStepX;
	for (int i=0;i<rowNum;i++)
		destSurf->m_aCoordY[i]   = destSurf->m_BasePoint.y+i*destSurf->m_fStepY;

	float minZ = pSurf->m_aCoordZ[0];
	for (int i=0;i<pSurf->m_nNumVert;i++)
		if (pSurf->m_aCoordZ[i]<minZ)
			minZ = pSurf->m_aCoordZ[i];

	int index = -1;
	int k     = -1;
	for (int i=0;i<destSurf->m_nNumVertY;i++)
		for (int j=0;j<destSurf->m_nNumVertX;j++) 
			if (i<num || j<num || i>=(num+pSurf->m_nNumVertY) || j>=(num+pSurf->m_nNumVertX))
				destSurf->m_aCoordZ[++index] = minZ;
			else
				destSurf->m_aCoordZ[++index] = pSurf->m_aCoordZ[++k];
	// 	CSurfNetEdit edit;
	// 	edit.SurfNetNorm(destSurf);
	delete pSurf;
	pSurf = destSurf;
	return TRUE;
}


void MakeToolHeightMask(	float		diameter,		 //for all tools
	float		diameterBase,    //for v-bit and bull nose only
	float		angleFull,       //for v-bit, coned ball and coned bull nose only, full angle, in degree
	float       roundRadius,     //for coned ball, bull nose and coned bull nose only
	int			k,
	int			toolType,
	float		stepX,
	float       stepY,
	float**&	heightMask)
{
	ASSERT(k>0);
	int num = 2*k+1;
	heightMask = new float*[num];
	for (int i=0;i<num;i++)
		heightMask[i] = new float[num];
	float radius;
	radius = diameter/2.f;

	switch (toolType) {
	case 0:		//ball tool
		{
			float x,y;
			float radiusSqr = radius*radius;
			for (int i=0;i<num;i++)
				for (int j=0;j<num;j++) {
					y = (i-k)*stepY;
					x = (j-k)*stepX;
					float delta = radiusSqr-x*x-y*y;
					if (delta>=0.f)
						heightMask[i][j] = radius- (float)sqrt(radiusSqr-x*x-y*y);
					else
						heightMask[i][j] = -1.f;
				}
				//set center point as zero
				heightMask[k][k] = 0.f;
		}
		break;
	case 1:		// flat tool
		{
			float x,y;
			for (int i=0;i<num;i++)
				for (int j=0;j<num;j++) {
					y = (i-k)*stepY;
					x = (j-k)*stepX;
					float delta = radius-(float) sqrt(x*x+y*y);
					if (delta>=0.f)
						heightMask[i][j] = 0.f;
					else
						heightMask[i][j] = -1.f;
				}
		}
		break;
	case 2:		// V bit tool
		{
			float x,y;
			for (int i=0;i<num;i++)
				for (int j=0;j<num;j++) {
					y = (i-k)*stepY;
					x = (j-k)*stepX;
					float d = (float) sqrt(x*x+y*y);
					if (d <= diameterBase/2.f)
						heightMask[i][j] = 0.f;
					else if (d > radius) 
						heightMask[i][j] = -1.f;
					else {
						d = d - diameterBase/2.f;
						float ang = (90.f-angleFull/2.f)*3.14159f/180.f;
						heightMask[i][j] = (float) tan(ang)*d;
					}
				}
		}
		break;
	case 3:		//coned ball
		{
			float x,y;
			float radiusOfTangPt;
			radiusOfTangPt = roundRadius*(float)cos(angleFull*0.5f*3.14159f/180.f);

			for (int i=0;i<num;i++)
				for (int j=0;j<num;j++) {
					y = (i-k)*stepY;
					x = (j-k)*stepX;
					float d = (float) sqrt(x*x+y*y);
					if (d <= radiusOfTangPt)
						heightMask[i][j] = roundRadius- (float)sqrt(roundRadius*roundRadius-x*x-y*y);
					else if (d > radius) 
						heightMask[i][j] = -1.f;
					else {
						d = d - radiusOfTangPt;
						float ang = (90.f-angleFull/2.f)*3.14159f/180.f;
						heightMask[i][j] = (float) tan(ang)*d;
					}
				}
				//set center point as zero
				heightMask[k][k] = 0.f;
		}
		break;
	case 4:		//coned bull nose
		{
			float x,y;
			float radiusOfTangPt;
			radiusOfTangPt = diameterBase/2.f+roundRadius*(float)cos(angleFull*0.5f*3.14159f/180.f);

			for (int i=0;i<num;i++)
				for (int j=0;j<num;j++) {
					y = (i-k)*stepY;
					x = (j-k)*stepX;
					float d = (float) sqrt(x*x+y*y);
					if (d <= diameterBase*0.5f)
						heightMask[i][j] = 0.f;
					else if (d <= radiusOfTangPt) {
						d = d - radiusOfTangPt;
						heightMask[i][j] = roundRadius- (float)sqrt(roundRadius*roundRadius-d*d);
					}
					else if (d > radius) 
						heightMask[i][j] = -1.f;
					else {
						d = d - radiusOfTangPt;
						float ang = (90.f-angleFull/2.f)*3.14159f/180.f;
						heightMask[i][j] = (float) tan(ang)*d;
					}
				}
		}
		break;
	case 5:		//bull nose
		{
			float x,y;
			float radiusOfTangPt;
			radiusOfTangPt = diameter/2.f-roundRadius;

			for (int i=0;i<num;i++)
				for (int j=0;j<num;j++) {
					y = (i-k)*stepY;
					x = (j-k)*stepX;
					float d = (float) sqrt(x*x+y*y);
					if (d <= radiusOfTangPt)
						heightMask[i][j] = 0.f;
					else if (d > radius) 
						heightMask[i][j] = -1.f;
					else {
						d = d - radiusOfTangPt;
						heightMask[i][j] = roundRadius- (float)sqrt(roundRadius*roundRadius-d*d);
					}
				}
		}
		break;
	default:
		ASSERT(0);
		break;
	}
}

void CheckToolHeightMaskVirtualCarving(		int r,
	int c,
	float** heightMask,
	int k,
	CGeoNetSurf* pSurf,
	unsigned char* pSurfMask,
	float* firmLayer,
	float& tipZCoord)
{
	float delta = 0.f;
	float baseZ = firmLayer[r*pSurf->m_nNumVertX+c];

	//calculate adjustAmount
	for (int i=r-k; i<=r+k;i++)
		for (int j=c-k;j<=c+k;j++) {

			//outside of the operation region
			if ( i < 0 || i>pSurf->m_nNumVertY-1 ||
				j < 0 || j>pSurf->m_nNumVertX-1)
				continue;

			//index of the mask
			int i1 = i-r+k;
			int j1 = j-c+k;
			if (heightMask[i1][j1] < 0.f) continue;


			float realDiff = firmLayer[i*pSurf->m_nNumVertX+j] - baseZ;
			if (realDiff > heightMask[i1][j1]) {
				float adjustAmount = realDiff -  heightMask[i1][j1];
				if ( adjustAmount > delta)
					delta = adjustAmount;
			}
		}
		tipZCoord = baseZ+delta+heightMask[k][k];


		//modify the surf
		for (int i=r-k; i<=r+k;i++)
			for (int j=c-k;j<=c+k;j++) {

				//outside of the operation region
				if ( i < 0 || i>pSurf->m_nNumVertY-1 ||
					j < 0 || j>pSurf->m_nNumVertX-1)
					continue;

				//index of the mask
				int i1 = i-r+k;
				int j1 = j-c+k;
				if (heightMask[i1][j1] < 0.f) continue;

				int index = i*pSurf->m_nNumVertX+j;
				if (pSurfMask[index] == 0) {
					pSurf->m_aCoordZ[index] = baseZ+delta+heightMask[i1][j1];
					pSurfMask[index] = 1;
				}
				else if ( pSurf->m_aCoordZ[index] > baseZ+delta+heightMask[i1][j1])
					pSurf->m_aCoordZ[index] = baseZ+delta+heightMask[i1][j1];
			}
}


void CheckToolHeightMaskTipCenterSurf(		int r,
	int c,
	float** heightMask,
	int k,
	CGeoNetSurf* pSurf,
	float* firmLayer,
	float& tipZCoord)
{
	float delta = 0.f;
	float baseZ = firmLayer[r*pSurf->m_nNumVertX+c];

	//calculate adjustAmount
	for (int i=r-k; i<=r+k;i++)
		for (int j=c-k;j<=c+k;j++) {

			//outside of the operation region
			if ( i < 0 || i>pSurf->m_nNumVertY-1 ||
				j < 0 || j>pSurf->m_nNumVertX-1)
				continue;

			//index of the mask
			int i1 = i-r+k;
			int j1 = j-c+k;
			if (heightMask[i1][j1] < 0.f) continue;


			float realDiff = firmLayer[i*pSurf->m_nNumVertX+j] - baseZ;
			if (realDiff > heightMask[i1][j1]) {
				float adjustAmount = realDiff -  heightMask[i1][j1];
				if ( adjustAmount > delta)
					delta = adjustAmount;
			}
		}
		tipZCoord = baseZ+delta+heightMask[k][k];
		pSurf->m_aCoordZ[r*pSurf->m_nNumVertX+c] = tipZCoord;
}

BOOL VirtualCarving(		int mode,			//1--virtual carving; 0--tip center surf
	CGeoNetSurf* pSurf,
	float diameter,
	float diameterBase,	
	float angleFull,	
	float roundRadius,   
	int   toolType,			//0--ball tool; 1--flat tool; 2--v-bit tool
	JDNC_PRGDEF &PrgDef)		
{
	if (	fabs(pSurf->m_fStepX-pSurf->m_fStepY)>0.001 ||
		diameter<=0.1f ||
		toolType <0  ||
		toolType > 5)
		return FALSE;


	float zMin,zMax;
	zMin = pSurf->m_aCoordZ[0];
	zMax = pSurf->m_aCoordZ[0];
	for (int i=1;i<pSurf->m_nNumVert;i++) {
		if (pSurf->m_aCoordZ[i] < zMin)
			zMin = pSurf->m_aCoordZ[i];
		if (pSurf->m_aCoordZ[i] > zMax)
			zMax = pSurf->m_aCoordZ[i];
	}

	float validDiameter = diameter;
	if (toolType == 2) {   //v-bit
		validDiameter = (zMax-zMin)*(float)tan(angleFull*0.5f*3.14159f/180.f)*2.f+diameterBase;
		if (validDiameter > diameter)
			validDiameter = diameter;
	}
	else if (toolType == 3)  { //coned ball
		float t1 = roundRadius*(float)sin(angleFull*0.5f*3.14159f/180.f);
		float t2 = roundRadius*(float)cos(angleFull*0.5f*3.14159f/180.f);
		float t3 = 	(zMax-zMin-roundRadius+t1);
		if (t3<0.f)	t3 = 0.f;
		validDiameter = t2*2.f+
			t3*(float)tan(angleFull*0.5f*3.14159f/180.f);
		if (validDiameter > diameter)
			validDiameter = diameter;
	}

	//wholly firm the model
	float* firmLayer;
	firmLayer = new float[pSurf->m_nNumVert];
	memcpy(firmLayer,pSurf->m_aCoordZ,sizeof(float)*pSurf->m_nNumVert);

	int numRadius = (int) floor(validDiameter*0.5f/pSurf->m_fStepX)+1;
	float** heightMask;
	MakeToolHeightMask(validDiameter,diameterBase,angleFull,roundRadius,numRadius,toolType,pSurf->m_fStepX,pSurf->m_fStepY,heightMask);
	int num = 2*numRadius+1;

	unsigned char* pSurfMask;
	pSurfMask = new unsigned char[pSurf->m_nNumVertX*pSurf->m_nNumVertY];
	memset(pSurfMask,0,pSurf->m_nNumVertX*pSurf->m_nNumVertY);

	//进度条
//	DOUBLE dTotalMove = PrgDef.m_dTotalMove ;
	PrgDef.m_dIncStep = 0.0; 
	PrgDef.m_dLimitAt = 1.0 ;
	PrgDef.m_dStepAt  = 0.0 ;

	float tipZCoord;
	if (mode == 1 || mode == 2) {	//virtual carving
//		int count = pSurf->m_nNumVertY*pSurf->m_nNumVertX;

		char strTmp[128];
		switch (glbf_GetLocalLangID()) {
			//case LANGID_CHINESE_SIMPLIFIED:
			//	strcpy(str,"请拾取网格曲面jjj");
			//	break;
		case LANGID_CHINESE_TRADITIONAL:
			strcpy(strTmp,"干");
			break;
		case LANGID_ENGLISH_US:
			strcpy(strTmp,"Filling Undercut");
			break;
		default:
			strcpy(strTmp,"补料");
			break;
		}
		//PathEdit_SetNewStep(strTmp) ;
		//PathEdit_SetNewStep("补料") ;

		for (int i=0;i<=pSurf->m_nNumVertY-1;i++) 
			for (int j=0;j<=pSurf->m_nNumVertX-1;j++) {
				//PathEdit_SetCurPos( i*pSurf->m_nNumVertX+j, count) ;
				//if( PathEdit_IsAbort())
				//	return FALSE;		
				CheckToolHeightMaskVirtualCarving( i,j,heightMask,numRadius,pSurf,pSurfMask,firmLayer,tipZCoord);
			}
	}
	else {				//creating tip center surface
//		int count = pSurf->m_nNumVertY*pSurf->m_nNumVertX;

		char strTmp[128];
		switch (glbf_GetLocalLangID()) {
		case LANGID_CHINESE_TRADITIONAL:
			strcpy(strTmp,"熬");
			break;
		case LANGID_ENGLISH_US:
			strcpy(strTmp,"Offseting Tool");
			break;
		default:
			strcpy(strTmp,"偏刀");
			break;
		}
		//PathEdit_SetNewStep(strTmp) ;
		//PathEdit_SetNewStep("偏刀") ;
		//个数为pSurf->m_nNumVertY*pSurf->m_nNumVertX
		int nTotal = (pSurf->m_nNumVertX-1 ) * (pSurf->m_nNumVertY-1)+1;
		ASSERT(nTotal>0) ;
		if( PrgDef.m_dTotalMove > 0.0 )
		{
			PrgDef.m_dIncStep = PrgDef.m_dTotalMove / DOUBLE(nTotal); 
		}


		for (int i=0;i<=pSurf->m_nNumVertY-1;i++) 
			for (int j=0;j<=pSurf->m_nNumVertX-1;j++) {
				//PathEdit_SetCurPos( i*pSurf->m_nNumVertX+j, count) ;
				//if( PathEdit_IsAbort())
				//	return FALSE;	
				if( PrgDef.m_pBrkFunc &&  PrgDef.m_pBrkFunc()  )
				{ /* 用户中断 || 进度提示 */
					break; 
				}
				PrgDef.m_dStepAt += PrgDef.m_dIncStep ;
				while( PrgDef.m_pPrgFunc && PrgDef.m_dStepAt >= PrgDef.m_dLimitAt )
				{
					PrgDef.m_pPrgFunc( 1 ) ;
					PrgDef.m_dStepAt -= PrgDef.m_dLimitAt ;
				}
				CheckToolHeightMaskTipCenterSurf( i,j,heightMask,numRadius,pSurf,firmLayer,tipZCoord);
			}
	}

	if (mode == 2) {	//leftover model
		for (int i=0;i<pSurf->m_nNumVert;i++)
			pSurf->m_aCoordZ[i] -= firmLayer[i];
	}

	for (int i=0;i<num;i++)
		delete[] heightMask[i];
	delete[] heightMask;

	delete[] pSurfMask;
	pSurfMask = NULL;

	delete[] firmLayer;

	return TRUE;
}

BOOL GetAnotherCrossingEdge(	int currRow,
	int currCol,
	int currEdgeIndex,
	int numX,
	unsigned char* tag,
	unsigned char* vEdgeTag,
	unsigned char* hEdgeTag,
	int& edgeIndex)
{
	int k0 = currRow*numX+currCol;
	int k1 = k0+1;
	int k2 = k1+numX;
	int k3 = k0+numX;

	switch (currEdgeIndex) {
	case 0:
		if (vEdgeTag[k1] == 0 && tag[k1] != tag[k2]) {
			edgeIndex = 1;
			return TRUE;
		}
		else if (hEdgeTag[k3] == 0 && tag[k2] != tag[k3]) {
			edgeIndex = 2;
			return TRUE;
		}
		else if (vEdgeTag[k0] == 0 && tag[k0] != tag[k3]) {
			edgeIndex = 3;
			return TRUE;
		}
		else {
			ASSERT(1==1);
			edgeIndex = -1;
			return FALSE;
		}
		break;
	case 1:
		if (hEdgeTag[k3] == 0 && tag[k2] != tag[k3]) {
			edgeIndex = 2;
			return TRUE;
		}
		else if (vEdgeTag[k0] == 0 && tag[k0] != tag[k3]) {
			edgeIndex = 3;
			return TRUE;
		}
		else if (hEdgeTag[k0] == 0 && tag[k0] != tag[k1]) {
			edgeIndex = 0;
			return TRUE;
		}
		else {
			ASSERT(1==1);
			edgeIndex = -1;
			return FALSE;
		}
		break;
	case 2:
		if (vEdgeTag[k0] == 0 && tag[k0] != tag[k3]) {
			edgeIndex = 3;
			return TRUE;
		}
		else if (hEdgeTag[k0] == 0 && tag[k0] != tag[k1]) {
			edgeIndex = 0;
			return TRUE;
		}
		else if (vEdgeTag[k1] == 0 && tag[k1] != tag[k2]) {
			edgeIndex = 1;
			return TRUE;
		}
		else {
			ASSERT(1==1);
			edgeIndex = -1;
			return FALSE;
		}
		break;
	case 3:
		if (hEdgeTag[k0] == 0 && tag[k0] != tag[k1]) {
			edgeIndex = 0;
			return TRUE;
		}
		else if (vEdgeTag[k1] == 0 && tag[k1] != tag[k2]) {
			edgeIndex = 1;
			return TRUE;
		}
		else if (hEdgeTag[k3] == 0 && tag[k2] != tag[k3]) {
			edgeIndex = 2;
			return TRUE;
		}
		else {
			ASSERT(1==1);
			edgeIndex = -1;
			return FALSE;
		}
		break;
	default:
		edgeIndex = -1;
		return FALSE;
		break;
	}
}

void TraceOneLoop(		CGeoNetSurf* surf,
	unsigned char* tag,
	unsigned char* hEdgeTag,
	unsigned char* vEdgeTag,
	float zValue,
	int row,
	int col,
	std::vector<PT_3D>& m_arrPt,
	BOOL bAdjustZ)
{
	int currEdgeIndex = 3;
	int currRow = row;
	int currCol = col;
	float z1,z2;
	PT_3D pt;
	do {
		int k0 = currRow*surf->m_nNumVertX+currCol;
		int k1 = k0+1;
//		int k2 = k1+surf->m_nNumVertX;
		int k3 = k0+surf->m_nNumVertX;

		int edgeIndex;
		BOOL bRet = GetAnotherCrossingEdge(	currRow,
			currCol,
			currEdgeIndex,
			surf->m_nNumVertX,
			tag,
			vEdgeTag,
			hEdgeTag,
			edgeIndex);
		if (!bRet)	//loop closed
			break;

		switch (edgeIndex) {
		case 0: 
			z1 = surf->m_aCoordZ[ currRow*(surf->m_nNumVertX)+currCol] ;
			z2 = surf->m_aCoordZ[ currRow*(surf->m_nNumVertX)+currCol+1] ;
			pt.x = surf->m_aCoordX[currCol] + surf->m_fStepX * (z1-zValue)/(z1-z2) ;
			pt.y = surf->m_aCoordY[currRow] ;
			if (bAdjustZ) {
				if (z1 >= z2)
					pt.z = z1;
				else
					pt.z = z2;
			}
			else 
				pt.z = zValue;
			m_arrPt.push_back( pt ) ;

			currRow--;
			currEdgeIndex = 2;
			hEdgeTag[k0] = 1;
			break;
		case 1: 
			z1 = surf->m_aCoordZ[currRow*(surf->m_nNumVertX)+currCol+1] ;
			z2 = surf->m_aCoordZ[(currRow+1)*(surf->m_nNumVertX)+currCol+1] ;
			pt.x = surf->m_aCoordX[currCol+1] ;
			pt.y = surf->m_aCoordY[currRow] + surf->m_fStepY * (z1-zValue)/(z1-z2) ;
			if (bAdjustZ) {
				if (z1 >= z2)
					pt.z = z1;
				else
					pt.z = z2;
			}
			else 
				pt.z = zValue;
			m_arrPt.push_back( pt ) ;
			currCol++;
			currEdgeIndex = 3;
			vEdgeTag[k1] = 1;
			break;
		case 2: 
			z1 = surf->m_aCoordZ[ (currRow+1)*(surf->m_nNumVertX)+currCol] ;
			z2 = surf->m_aCoordZ[ (currRow+1)*(surf->m_nNumVertX)+currCol+1] ;
			pt.x = surf->m_aCoordX[currCol] + surf->m_fStepX * (z1-zValue)/(z1-z2) ;
			pt.y = surf->m_aCoordY[currRow+1] ;
			if (bAdjustZ) {
				if (z1 >= z2)
					pt.z = z1;
				else
					pt.z = z2;
			}
			else 
				pt.z = zValue;
			m_arrPt.push_back( pt ) ;

			currRow++;
			currEdgeIndex = 0;
			hEdgeTag[k3] = 1;
			break;
		case 3: 
			z1 = surf->m_aCoordZ[currRow*(surf->m_nNumVertX)+currCol] ;
			z2 = surf->m_aCoordZ[(currRow+1)*(surf->m_nNumVertX)+currCol] ;
			pt.x = surf->m_aCoordX[currCol] ;
			pt.y = surf->m_aCoordY[currRow] + surf->m_fStepY * (z1-zValue)/(z1-z2) ;
			if (bAdjustZ) {
				if (z1 >= z2)
					pt.z = z1;
				else
					pt.z = z2;
			}
			else 
				pt.z = zValue;
			m_arrPt.push_back( pt ) ;
			currCol--;
			currEdgeIndex = 1;
			vEdgeTag[k0] = 1;
			break;
		default:
			break;
		}
	} while (1==1);
}

//To find iso-height loops of a rectmesh
//bAdjustZ == TRUE--- adjust interseting point's Z to that of the higher vertex
BOOL RectMeshIsoHeightLoop(	CGeoNetSurf* surf,
	float		zValue,
	BOOL*&		bReverseLoopDir,
	PT_3D**&	intCurve,			//int curves
	int*&		numPtsIntCurve,		//num of points in each int. curve
	int&		numIntCurve,		//num of int. curves
	BOOL        bAdjustZ = FALSE )
{
	intCurve		= NULL;
	numPtsIntCurve	= NULL;
	numIntCurve		= 0;
	bReverseLoopDir = NULL;

	if (!surf || !surf->IsRectMesh())
		return FALSE;

	//set tags
	unsigned char* tag;
	tag = new unsigned char[surf->m_nNumVert];
	memset(tag,0,surf->m_nNumVert);
	for (int i=0;i<surf->m_nNumVertY;i++)
		for (int j=0;j<surf->m_nNumVertX;j++) {
			int k = i*surf->m_nNumVertX+j;
			if (surf->m_aCoordZ[k]-zValue > 5.0e-4)
				tag[k] = 1;
		}
		//edge tags 0--undefine, 1--alread processed 100--is start edge and processed
		unsigned char* vEdgeTag;
		unsigned char* hEdgeTag;
		vEdgeTag = new unsigned char[surf->m_nNumVert];
		hEdgeTag = new unsigned char[surf->m_nNumVert];
		memset(vEdgeTag,0,surf->m_nNumVert);
		memset(hEdgeTag,0,surf->m_nNumVert);

		std::vector<PT_3D> m_arrPt[5000] ;	     
		bReverseLoopDir = new int[5000];
		for (int i=0;i<surf->m_nNumVertY-1;i++)
			for (int j=0;j<surf->m_nNumVertX-1;j++) {
				int k0 = i*surf->m_nNumVertX+j;
				//int k1 = k0+1;
//				int k2 = k1+surf->m_nNumVertX;
				int k3 = k0+surf->m_nNumVertX;

				//to find low left vertical edge
				if (vEdgeTag[k0]!=0 || tag[k0] == tag[k3])
					continue;

				PT_3D pt;
				float z1 = surf->m_aCoordZ[ k0] ;
				float z2 = surf->m_aCoordZ[ k3] ;
				pt.x = surf->m_aCoordX[j] ;
				pt.y = surf->m_aCoordY[i] + surf->m_fStepY * (z1-zValue)/(z1-z2) ;
				if (bAdjustZ) {
					if (z1 >= z2)
						pt.z = z1;
					else
						pt.z = z2;
				}
				else 
					pt.z = zValue;
				m_arrPt[numIntCurve].push_back( pt ) ;
				hEdgeTag[k0] = 100;
				if (surf->m_aCoordZ[k3] > surf->m_aCoordZ[k0])
					bReverseLoopDir[numIntCurve] = FALSE;
				else
					bReverseLoopDir[numIntCurve] = TRUE;

				TraceOneLoop( surf,tag,hEdgeTag,vEdgeTag,zValue,i,j,m_arrPt[numIntCurve],bAdjustZ) ;				
				numIntCurve++ ;
				if (numIntCurve == 5000)
					goto JumpOut;
			}
JumpOut:
			if (tag)
				delete[] tag;
			if (vEdgeTag)
				delete[] vEdgeTag;
			if (hEdgeTag)
				delete[] hEdgeTag;

			if (numIntCurve>0) {
				numPtsIntCurve = new int[numIntCurve];
				intCurve = new PT_3D*[numIntCurve];

				for (int i=0;i<numIntCurve;i++) {
					numPtsIntCurve[i] = (int)m_arrPt[i].size() ;
					intCurve[i] = new PT_3D[numPtsIntCurve[i]] ;
					for( int j = 0 ; j<(int)m_arrPt[i].size();j++) {
						intCurve[i][j].x = m_arrPt[i][j].x ;
						intCurve[i][j].y = m_arrPt[i][j].y ;
						intCurve[i][j].z = m_arrPt[i][j].z;
					}
				}
				return TRUE ;
			}

			delete[] bReverseLoopDir;
			bReverseLoopDir = NULL;
			return FALSE;
}


void CalContoursFromSurf(
	CGeoNetSurf*	surf ,
	float			zValue ,
	PT_3D**&		intCurve ,
	int*&			numPtsIntCurve ,
	int	&			numIntCurve ,
	BOOL *&			bReverseLoop )
{
	if( !RectMeshIsoHeightLoop(	surf,
		zValue,
		bReverseLoop ,
		intCurve,			//int curves
		numPtsIntCurve,		//num of points in each int. curve
		numIntCurve))		// Reverse Loop Curves
	{
		if( intCurve )
		{
			for( int i=0 ; i <numIntCurve ; i++ )
			{
				delete []intCurve[i] ;
			}
			delete []intCurve ;
			intCurve = NULL ;
		}
		if( numPtsIntCurve )
		{
			delete []numPtsIntCurve ;
			numPtsIntCurve = NULL ;
		}
		if( bReverseLoop )
		{
			delete []bReverseLoop ;
			bReverseLoop = NULL ;
		}
	}

}

void CalAllContours(	
	CGeoNetSurf * pCreatedRect ,
	float zValue,
	CToolPathContours & contour )
{
	// 0 , 计算内轮廓
	PT_3D**		intCurve = NULL ;			//int curves
	int*		numPtsIntCurve = NULL ;		//num of points in each int. curve
	int			numIntCurve = 0 ;
	BOOL *		bReverseLoop = NULL ;

	CalContoursFromSurf(
		pCreatedRect ,
		zValue ,
		intCurve ,
		numPtsIntCurve ,
		numIntCurve ,
		bReverseLoop ) ;


	// 1 , 计算外轮廓 ( 矩形网格的外轮廓是一个矩形 )
	contour.nCurve		= numIntCurve ;
	contour.anPtsInCurve= numPtsIntCurve ;
	contour.aPts		= intCurve ;
	contour.aReverse	= bReverseLoop ;
	contour.fZ			= zValue ;

}


// 曲线光顺
void CurveAverage(PT_3D* pts, // 曲线上的点
	int numVet, // 曲线上点的个数
	BOOL closedTag) // 曲线两个端点是否移动,TRUE-不移动,FALSE-移动
{
	PT_3D* edgeCenter;
	edgeCenter=new PT_3D[numVet];
	if (!closedTag) {
		for (int i=0;i<numVet-1;i++)
			edgeCenter[i]=(pts[i]+pts[i+1])/2.f;
		edgeCenter[numVet-1]=(pts[numVet-1]+pts[0])/2.f;

		pts[0]=(edgeCenter[0]+edgeCenter[numVet-1])/2.f;
		for (i=1;i<numVet;i++)
			pts[i]=(edgeCenter[i-1]+edgeCenter[i])/2.f;
	}
	else {
		for (int i=0;i<numVet-1;i++)
			edgeCenter[i]=(pts[i]+pts[i+1])/2.f;

		for (i=1;i<=numVet-2;i++)
			pts[i]=(edgeCenter[i-1]+edgeCenter[i])/2.f;
	}
	delete[] edgeCenter;
}

float AS_PointLine3DDistance(	// 点到线段的距离
	const	PT_3D	&	point ,
	const	PT_3D	&	linepoint1 ,
	const	PT_3D	&	linepoint2 ,
	float	&	t , // 最近点对应的参数 , 
	// 最近点坐标为 (linepoint1 + t*(linepoint2 - linepoint1))
	const	float	&	tol ) 
{
	float distance ;

	PT_3D dir = linepoint2 - linepoint1 ;

	float length2 = dir.Iproduct( dir ) ;

	if( length2<tol )// 如果输入射线方向被认为是零向量 , 则
	{
		distance = point.DistFrom2( ( linepoint1 + linepoint2 ) / 2 ) ;
		t = 0.5 ;
	}
	else
	{
		t = dir.Iproduct( point - linepoint1 ) / length2 ;

		PT_3D tempPT ;
		if( t>=0 && t<=1 )
			tempPT = point - ( linepoint1 + t * dir) ;
		else if( t<0 )
		{
			tempPT = point - linepoint1 ;
			t = 0 ;
		}
		else // if( t>1 )
		{
			tempPT = point - linepoint2 ;
			t = 1 ;
		}

		distance = tempPT.Iproduct( tempPT ) ;
	}

	return distance ;
}

// 封闭点围成的面积
float CalPtsArea( PT_3D * aPts , int nPts )
{
	double area=0 ;
	for (int i=0; i<(nPts-1); i++)
		area += (double)aPts[i].x * (double)aPts[i+1].y - (double)aPts[i].y * (double)aPts[i+1].x;
	area += (double)aPts[nPts-1].x * (double)aPts[0].y - (double)aPts[nPts-1].y * (double)aPts[0].x;
	return (float)area ;
}

float CalPtsArea( PT_3D * aPts ,int ai[], int nPts )
{
	double area=0 ;
	for (int i=0; i<(nPts-1); i++)
		area += (double)aPts[ai[i]].x * (double)aPts[ai[i+1]].y - (double)aPts[ai[i]].y * (double)aPts[ai[i+1]].x;
	area += (double)aPts[ai[nPts-1]].x * (double)aPts[ai[0]].y - (double)aPts[ai[nPts-1]].y * (double)aPts[ai[0]].x;
	return (float)area ;
}

float AS_PointLine3DSegDistance2(	// 点到线段的距离
	const	PT_3D	&	point ,
	const	PT_3D	&	linepoint1 ,
	const	PT_3D	&	linepoint2 ,
	float	&	t , // 最近点对应的参数 , 
	// 最近点坐标为 (linepoint1 + t*(linepoint2 - linepoint1))
	const	float	&	tol ) 
{
	float distance ;

	PT_3D dir = linepoint2 - linepoint1 ;

	float length2 = dir.Iproduct( dir ) ;

	if( length2<tol )// 如果输入射线方向被认为是零向量 , 则
	{
		distance = point.DistFrom2( ( linepoint1 + linepoint2 ) / 2 ) ;
		t = 0.5 ;
	}
	else
	{
		t = dir.Iproduct( point - linepoint1 ) / length2 ;

		PT_3D tempPT ;
		if( t>=0 && t<=1 )
			tempPT = point - ( linepoint1 + t * dir) ;
		else if( t<0 )
		{
			tempPT = point - linepoint1 ;
			t = 0 ;
		}
		else // if( t>1 )
		{
			tempPT = point - linepoint2 ;
			t = 1 ;
		}

		distance = tempPT.Iproduct( tempPT ) ;
	}


	return distance ;
}


CSmartLoop *ConvertToSmartLoop(PT_3D pts[],int nPt,float tol)
{
	CSmartLoop *pLoop = NULL ;
	if( nPt >6 )//光顺
	{
		for( int m=0 ; m<4 ; m++ )
			CurveAverage(pts,nPt,TRUE) ;
	}
	//优化

	int * aI = new int[ nPt ] ;//序号
	int nI = 0 ;
	BOOL bFalse = FALSE ;
	if( nPt > 3 )
	{
		//首点
		aI[nI] = 0 ; nI++ ; 
		aI[nI] = nPt - 1 ; nI++ ;

		if( pts[0].DistFrom( pts[nPt-1] )<1e-3f ) 
		{
			float maxDist = -1 ;
			int iMax = -1 ;
			for( int i=1 ; i<nPt-1 ; i++ )
			{
				float d = pts[0].DistFrom( pts[i] ) ;
				if( maxDist<d )
				{
					maxDist = d ;
					iMax = i ;
				}
			}
			if( maxDist>tol )
			{
				aI[nI] = aI[nI-1] ;
				aI[nI-1] = iMax ;
				nI++ ;
			}
		}
		else
			bFalse = TRUE ;
		if( !bFalse )
		{
			for( int i=1 ; i<nI ; )
			{
				int iS = aI[i-1] ;
				int iE = aI[i] ;
				float maxDist = -1 ;
				int iMax = -1 ;
				for( int j=	iS + 1 ; j<iE ; j++ )
				{
					float t = 0 ;
					float d = AS_PointLine3DSegDistance2(	// 点到线段的最近距离平方
						pts[j] ,		// 点
						pts[iS] ,// 直线端点1
						pts[iE] ,// 直线端点2
						t, // 最近点对应的参数 , 
						tol) ;
					if( maxDist<d )
					{
						maxDist = d ;
						iMax = j ;
					}
				}

				if( maxDist>tol*tol )
				{
					for( int k=nI ; k>i ; k-- )
						aI[k] = aI[k-1] ;
					aI[i] = iMax ;
					nI++ ;
				}
				else
					i++ ;
			}
			nPt = nI ;
		}		
	}
	else
	{
		for( int i=0 ; i<nPt ; i++ )
			aI[i] = i ;
	}

	if( !bFalse )
	{
		float fArea = CalPtsArea(pts,aI,nPt) ;
		if( fArea<2.f)
			bFalse = TRUE ;
	}
	if( nPt > 3 && !bFalse )
	{
		//转为smartloop
		CSmartCurve* pSmtCurve = new CSmartCurve();
		ASSERT(pSmtCurve);
		for( int i=0 ; i<nPt-1 ; i++ )
		{
			PNT2D dStart, dEnd;
			dStart[0] = pts[aI[i]].x ;
			dStart[1] = pts[aI[i]].y ;
			dEnd[0] = pts[aI[i+1]].x ;
			dEnd[1] = pts[aI[i+1]].y ;
			CSmartLine* pLine = new CSmartLine(dStart, dEnd);
			// 		pLine->SetZValue(pts[aI[i]].z,pts[aI[i+1]].z) ;
			pSmtCurve->AddSect(pLine);
		}
		pLoop = new CSmartLoop;
		ASSERT(pLoop);
		pLoop->UpdateSect(pSmtCurve);
		pLoop->DefineBox();
		pLoop->RemoveAutoIntSect();
		if(pLoop->m_dArea < 0.0)
			pLoop->ReverseLoop();
	}	

	delete []aI ;
	return pLoop ;
}

inline float CrossXY(PT_3D v1,PT_3D v2) { return v1.x*v2.y-v1.y*v2.x;};
inline float CrossXY(PT_2D v1,PT_2D v2) { return v1.x*v2.y-v1.y*v2.x;};

//to see if p3d's projection (along z) is inside
//p's projection
BOOL IsInsideTriangle(PT_3D p3d,PT_3D* p)
{
	//assume boxcheck done
	//if (p3d.x<Min3(p[0].x,p[1].x,p[2].x) ||
	//	p3d.x>Max3(p[0].x,p[1].x,p[2].x) ||
	//	p3d.y<Min3(p[0].y,p[1].y,p[2].y) ||
	//	p3d.y>Max3(p[0].y,p[1].y,p[2].y))
	//	return FALSE;

	float minDistToEdge;
	float d1,d2,d3;
	PT_2D p2d, p1,p2,p3;

	p2d.Set(p3d.x,p3d.y);

	p1.Set(p[0].x,p[0].y);
	p2.Set(p[1].x,p[1].y);
	p3.Set(p[2].x,p[2].y);

	/*
	//to chaeck the case that (p1,p2,p3) are the same point;
	d1 = p1.DistFrom(p2);
	d2 = p2.DistFrom(p3);
	d3 = p3.DistFrom(p1);
	if (d1 < 0.0001f && d2 < 0.0001f && d3 < 0.0001f)
		return FALSE;
	*/

	d1 = p2d.GetDistLine(p1,p2);
	d2 = p2d.GetDistLine(p2,p3);
	d3 = p2d.GetDistLine(p3,p1);

	minDistToEdge = d1;
	if (d2<minDistToEdge)
		minDistToEdge = d2;
	if (d3<minDistToEdge)
		minDistToEdge = d3;
	if (minDistToEdge< 0.000001f) //0.001f too large, may cause flying point, Aug.23,04
		return TRUE;

	float ddd1=0.0f;
	float ddd2=(0.0f);
	if ((CrossXY(p[0]-p3d, p[1]-p[0])>ddd1 && 
		 CrossXY(p[1]-p3d, p[2]-p[1])>ddd1 &&
		 CrossXY(p[2]-p3d, p[0]-p[2])>ddd1) ||
		(CrossXY(p[0]-p3d, p[1]-p[0])<ddd2 &&
		 CrossXY(p[1]-p3d, p[2]-p[1])<ddd2 &&
		 CrossXY(p[2]-p3d, p[0]-p[2])<ddd2))
		 return TRUE;
	
	return FALSE;
}

//to see if p3d's projection (along z) is inside
//p's projection
//assume boxcheck done
BOOL IsInsideTriangle(PT_2D p2d,PT_2D p1,PT_2D p2,PT_2D p3)
{
	float minDistToEdge;
	float d1,d2,d3;

	d1 = p2d.GetDistLine(p1,p2);
	d2 = p2d.GetDistLine(p2,p3);
	d3 = p2d.GetDistLine(p3,p1);

	minDistToEdge = d1;
	if (d2<minDistToEdge)
		minDistToEdge = d2;
	if (d3<minDistToEdge)
		minDistToEdge = d3;
	if (minDistToEdge< 0.000001f) //0.001f too large, may cause flying point, Aug.23,04
		return TRUE;

	float ddd1= 0.0f;
	float ddd2= 0.0f;

	if ((CrossXY(p1-p2d, p2-p1)>ddd1 && 
		 CrossXY(p2-p2d, p3-p2)>ddd1 &&
		 CrossXY(p3-p2d, p1-p3)>ddd1) ||
		(CrossXY(p1-p2d, p2-p1)<ddd2 &&
		 CrossXY(p2-p2d, p3-p2)<ddd2 &&
		 CrossXY(p3-p2d, p1-p3)<ddd2))
		 return TRUE;
	
	return FALSE;
}

//to find the int point of triangle p and a line that passes p3d and
//parrele to z axis
BOOL LineTriangleInt(PT_3D& p3d,PT_3D* p)
{
	float a=(p[1].x-p[0].x)*(p[2].y-p[0].y)-(p[1].y-p[0].y)*(p[2].x-p[0].x);
	if (fabs((double)a)<0.00001)
		return FALSE;
	float deltaZ= ((p[1].x-p[0].x)*(p[2].y-p[0].y)*p[0].z-
				   (p[1].y-p[0].y)*(p[2].x-p[0].x)*p[0].z+
			       (p3d.y -p[0].y)*(p[1].x-p[0].x)*(p[2].z-p[0].z)+
			       (p3d.x -p[0].x)*(p[1].z-p[0].z)*(p[2].y-p[0].y)-
			       (p3d.x -p[0].x)*(p[1].y-p[0].y)*(p[2].z-p[0].z)-
			       (p3d.y -p[0].y)*(p[1].z-p[0].z)*(p[2].x-p[0].x)) / a;
	p3d.z = deltaZ;
	return TRUE;
}


//assume surf and destSurf are rectMesh
void AddRectSurfToRectSurfHighLow(CGeoNetSurf* surf,CGeoNetSurf* destSurf,BOOL bKeepHighest)
{
	int xStart,yStart;
	int xEnd,yEnd;
	PT_3D pt1;
	pt1 = surf->GetRectPoint(0);
	xStart =(int) floor((pt1.x - destSurf->GetRectPoint(0).x)/destSurf->m_fStepX);
	yStart =(int) floor((pt1.y - destSurf->GetRectPoint(0).y)/destSurf->m_fStepY);
	xEnd =(int) ceil((surf->GetRectPoint(surf->m_nNumVert-1).x - destSurf->GetRectPoint(0).x)/destSurf->m_fStepX);
	yEnd =(int) ceil((surf->GetRectPoint(surf->m_nNumVert-1).y - destSurf->GetRectPoint(0).y)/destSurf->m_fStepY);

	if (xStart<0)	xStart = 0;
	if (yStart<0)   yStart = 0;
	if (xEnd > destSurf->m_nNumVertX-1 ) xEnd = destSurf->m_nNumVertX-1;
	if (yEnd > destSurf->m_nNumVertY-1 ) yEnd = destSurf->m_nNumVertY-1;

	float f00,f01,f10,f11;
	for (int row = yStart;row<=yEnd;row++)
		for (int col = xStart; col<=xEnd;col++) {
			int index = row*destSurf->m_nNumVertX+col;
			PT_3D pt = destSurf->GetRectPoint(index);
			if (pt.x < surf->GetRectPoint(0).x ||
				pt.x > surf->GetRectPoint(surf->m_nNumVert-1).x ||
				pt.y < surf->GetRectPoint(0).y ||
				pt.y > surf->GetRectPoint(surf->m_nNumVert-1).y)
				continue;

				int c = (int)floor((pt.x - surf->m_BasePoint.x)/surf->m_fStepX);
				int r = (int)floor((pt.y - surf->m_BasePoint.y)/surf->m_fStepY);

				float x = (pt.x - surf->m_BasePoint.x)/surf->m_fStepX - c;
				float y = (pt.y - surf->m_BasePoint.y)/surf->m_fStepY - r;

				//to handle the up/right edge case
				if (r >= surf->m_nNumVertY-1) {
					r--;
					y = 1.f;
				}
				if (c >= surf->m_nNumVertX-1) {
					c--;
					x = 1.f;
				}

				int k = r*surf->m_nNumVertX+c;
				f00 = surf->m_aCoordZ[k];
				f10 = surf->m_aCoordZ[k+1];
				f11 = surf->m_aCoordZ[k+surf->m_nNumVertX+1];
				f01 = surf->m_aCoordZ[k+surf->m_nNumVertX];
				float z = (f10-f00)*x+(f01-f00)*y+
								(f11+f00-f01-f10)*x*y+f00;

				if (bKeepHighest) {
					if (z>pt.z)
						destSurf->m_aCoordZ[index] = z;	
				}
				else {
					if (z<pt.z)
						destSurf->m_aCoordZ[index] = z;	
				}
		}
}

void AddTriangleToRectMeshHighLow(CGeoNetSurf* destSurf,PT_3D* tri,BOOL bKeepHighest)
{
	float xmin,ymin,xmax,ymax;
	float zmin,zmax;
	
	//cal box of triangle
	xmin = tri[0].x;
	ymin = tri[0].y;
	zmin = tri[0].z;
	xmax = xmin;
	ymax = ymin;
	zmax = zmin;

	if (tri[1].x < xmin ) xmin = tri[1].x;
	if (tri[2].x < xmin ) xmin = tri[2].x;
	if (tri[1].y < ymin ) ymin = tri[1].y;
	if (tri[2].y < ymin ) ymin = tri[2].y;
	if (tri[1].z < zmin ) zmin = tri[1].z;
	if (tri[2].z < zmin ) zmin = tri[2].z;

	if (tri[1].x > xmax ) xmax = tri[1].x;
	if (tri[2].x > xmax ) xmax = tri[2].x;
	if (tri[1].y > ymax ) ymax = tri[1].y;
	if (tri[2].y > ymax ) ymax = tri[2].y;
	if (tri[1].z > zmax ) zmax = tri[1].z;
	if (tri[2].z > zmax ) zmax = tri[2].z;

	int xStart,yStart;
	int xEnd,yEnd;
	xStart =(int) floor((xmin - destSurf->m_BasePoint.x)/destSurf->m_fStepX);
	yStart =(int) floor((ymin - destSurf->m_BasePoint.y)/destSurf->m_fStepY);
	xEnd =(int) ceil((xmax - destSurf->m_BasePoint.x)/destSurf->m_fStepX);
	yEnd =(int) ceil((ymax - destSurf->m_BasePoint.y)/destSurf->m_fStepY);
	if (xStart<0) xStart = 0;
	if (yStart<0) yStart = 0;
	if (xEnd > destSurf->m_nNumVertX-1) xEnd = destSurf->m_nNumVertX-1;
	if (yEnd > destSurf->m_nNumVertY-1) yEnd = destSurf->m_nNumVertY-1;

	for (int row = yStart; row<=yEnd;row++)
		for (int col = xStart;col<=xEnd; col++) {
			int index = row*destSurf->m_nNumVertX+col;
			PT_3D pt = destSurf->GetRectPoint(index);
			if (pt.x < xmin || pt.x > xmax || pt.y < ymin || pt.y > ymax)
				continue;
			if (IsInsideTriangle(pt,tri)) {
				LineTriangleInt(pt,tri);
				if (pt.z > zmax)	pt.z = zmax;
				if (pt.z < zmin)    pt.z = zmin;

				if (bKeepHighest) {
					if (pt.z>destSurf->GetRectPoint(index).z)
						destSurf->m_aCoordZ[index] = pt.z;
				}
				else {
					if (pt.z<destSurf->GetRectPoint(index).z)
						destSurf->m_aCoordZ[index] = pt.z;
				}
			}
		}
}


BOOL SurfNetNorm(CGeoNetSurf*& surf)		//
{
	if (surf->IsPointCloud()) {
		return FALSE;

		/*
		float xmin,ymin,zmin,xmax,ymax,zmax;
		float step;

		destSurf = NULL;

		xmin = pts[0].x;
		ymin = pts[0].y;
		zmin = pts[0].z;
		xmax = pts[0].x;
		ymax = pts[0].y;
		zmax = pts[0].z;
		for (int i = 1;i<numPts;i++) {
			if (pts[i].x < xmin)	xmin = pts[i].x;
			if (pts[i].y < ymin)	ymin = pts[i].y;
			if (pts[i].z < zmin)	zmin = pts[i].z;

			if (pts[i].x > xmax)	xmax = pts[i].x;
			if (pts[i].y > ymax)	ymax = pts[i].y;
			if (pts[i].z > zmax)	zmax = pts[i].z;
		}
		xmin -= 0.1f;
		ymin -= 0.1f;
		zmin -= 0.1f;

		xmax += 0.1f;
		ymax += 0.1f;
		zmax += 0.1f;

		//get step
		{
			step = 0.f;
			for (int i=1;i<=100;i++) {
				int k = (int) (numPts*(float) i/ 101.f);
				PT_3D pt=pts[k];
				float minDist = 9000000.f;
				for (int j=0;j<numPts;j++) {
					if (j==k)
						continue;
					float dist = pts[j].DistFrom(pt);
					if (dist < minDist)
						minDist = dist;
				}
				step += minDist;
			}
			step /= 100.f;
		}
		//step = 0.24f;

		CPointCloud3D* ptsCloud;
		ptsCloud = new CPointCloud3D;
		ptsCloud->Create(xmin,ymin,zmin,xmax,ymax,zmax,step*3.f,step*3.f,step*3.f);
		ptsCloud->AddPoints(pts,numPts);

		int num;
		int* nnborsTmp;
		float* distTmp;
		for (int i=0;i<numPts;i++) {
			ptsCloud->GetNearestNeighbors(i,16,nnborsTmp,distTmp,num);
			if (num<16)
				AfxMessageBox("num < 10",0,0);
			else {
				for (int j=0;j<16;j++) {
					nnbors[i][j] = nnborsTmp[j];
					dist[i][j]   = distTmp[j];
				}
			}
			if (nnborsTmp)
				delete[] nnborsTmp;
			if (distTmp)
				delete[] distTmp;
		}
		ptsCloud->Destroy();
		delete ptsCloud ;

		if (norm == NULL) {
			norm = new PT_3D[numPts];
			for (int i=0;i<numPts;i++) {
				BOOL  bRet;
				bRet = GetNormalEx(i,nnbors[i],16,norm[i]);
				if (bRet) {
					if (norm[i].Length()>0.0001f)
						norm[i].Unit();
					else
						norm[i].Set(0.f,0.f,1.f);
				}
				else
					norm[i].Set(0.f,0.f,1.f);
			}
		}

		return TRUE;
		*/
	}
	if (surf->IsRectMesh()) {
		if (surf->m_aNormal==NULL) 
			surf->m_aNormal=new PT_3D[surf->m_nNumVert];

		PT_3D v1,v2,v3,v4;
		PT_3D n1,n2,n3,n4,n;
		int k,k1,k2,k3,k4;

		//middle 
		for (int j=0+1;j<surf->m_nNumVertY-1;j++)
			for (int i=0+1;i<surf->m_nNumVertX-1;i++) {
				k	= j*surf->m_nNumVertX+i;
				k1	= k-1;
				k2  = k-surf->m_nNumVertX;
				k3  = k+1;
				k4  = k+surf->m_nNumVertX;

				PT_3D pt = surf->GetRectPoint(k);
				v1 = surf->GetRectPoint(k1)-pt;
				v2 = surf->GetRectPoint(k2)-pt;
				v3 = surf->GetRectPoint(k3)-pt;
				v4 = surf->GetRectPoint(k4)-pt;
				n1 = v1.Xproduct(v2);
				n2 = v2.Xproduct(v3);
				n3 = v3.Xproduct(v4);
				n4 = v4.Xproduct(v1);
				n  = (n1+n2+n3+n4)/4.f;
				n.Unit();
				surf->m_aNormal[k] = n;
			}

		//corner 
		j		=0;
		int i	=0;
		{
			k	= j*surf->m_nNumVertX+i;
			k3  = k+1;
			k4  = k+surf->m_nNumVertX;
			v3 = surf->GetRectPoint(k3)-surf->GetRectPoint(k);
			v4 = surf->GetRectPoint(k4)-surf->GetRectPoint(k);
			n3 = v3.Xproduct(v4);
			n3.Unit();
			surf->m_aNormal[k] = n3;
		}

		j	=0;
		i	=surf->m_nNumVertX-1;
		{
			k	= j*surf->m_nNumVertX+i;
			k1	= k-1;
			k4  = k+surf->m_nNumVertX;

			v1 = surf->GetRectPoint(k1)-surf->GetRectPoint(k);
			v4 = surf->GetRectPoint(k4)-surf->GetRectPoint(k);
			n4 = v4.Xproduct(v1);
			n4.Unit();
			surf->m_aNormal[k] = n4;
		}

		j	=surf->m_nNumVertY-1;
		i	=surf->m_nNumVertX-1;
		{
			k	= j*surf->m_nNumVertX+i;
			k1	= k-1;
			k2  = k-surf->m_nNumVertX;

			v1 = surf->GetRectPoint(k1)-surf->GetRectPoint(k);
			v2 = surf->GetRectPoint(k2)-surf->GetRectPoint(k);
			n1 = v1.Xproduct(v2);
			n1.Unit();
			surf->m_aNormal[k] = n1;
		}

		j	=surf->m_nNumVertY-1;
		i	=0;
		{
			k	= j*surf->m_nNumVertX+i;
			k2  = k-surf->m_nNumVertX;
			k3  = k+1;

			v2 = surf->GetRectPoint(k2)-surf->GetRectPoint(k);
			v3 = surf->GetRectPoint(k3)-surf->GetRectPoint(k);
			n2 = v2.Xproduct(v3);
			n2.Unit();
			surf->m_aNormal[k] = n2;
		}

		//edges
		j	=0;
		for (i=0+1;i<surf->m_nNumVertX-1;i++) {
			k	= j*surf->m_nNumVertX+i;
			k1	= k-1;
			k3  = k+1;
			k4  = k+surf->m_nNumVertX;

			v1 = surf->GetRectPoint(k1)-surf->GetRectPoint(k);
			v3 = surf->GetRectPoint(k3)-surf->GetRectPoint(k);
			v4 = surf->GetRectPoint(k4)-surf->GetRectPoint(k);
			n3 = v3.Xproduct(v4);
			n4 = v4.Xproduct(v1);
			n  = (n3+n4)/2.f;
			n.Unit();
			surf->m_aNormal[k] = n;
		}

		j = surf->m_nNumVertY-1;
		for (i=0+1;i<surf->m_nNumVertX-1;i++) {
			k	= j*surf->m_nNumVertX+i;
			k1	= k-1;
			k2  = k-surf->m_nNumVertX;
			k3  = k+1;

			v1 = surf->GetRectPoint(k1)-surf->GetRectPoint(k);
			v2 = surf->GetRectPoint(k2)-surf->GetRectPoint(k);
			v3 = surf->GetRectPoint(k3)-surf->GetRectPoint(k);
			n1 = v1.Xproduct(v2);
			n2 = v2.Xproduct(v3);
			n  = (n1+n2)/2.f;
			n.Unit();
			surf->m_aNormal[k] = n;
		}


		i = 0;
		for (j=0+1;j<surf->m_nNumVertY-1;j++) {
			k	= j*surf->m_nNumVertX+i;
			k2  = k-surf->m_nNumVertX;
			k3  = k+1;
			k4  = k+surf->m_nNumVertX;

			v2 = surf->GetRectPoint(k2)-surf->GetRectPoint(k);
			v3 = surf->GetRectPoint(k3)-surf->GetRectPoint(k);
			v4 = surf->GetRectPoint(k4)-surf->GetRectPoint(k);
			n2 = v2.Xproduct(v3);
			n3 = v3.Xproduct(v4);
			n  = (n2+n3)/4.f;
			n.Unit();
			surf->m_aNormal[k] = n;
		}

		i = surf->m_nNumVertX-1;
		for (j=0+1;j<surf->m_nNumVertY-1;j++) {
			k	= j*surf->m_nNumVertX+i;
			k1	= k-1;
			k2  = k-surf->m_nNumVertX;
			k4  = k+surf->m_nNumVertX;

			v1 = surf->GetRectPoint(k1)-surf->GetRectPoint(k);
			v2 = surf->GetRectPoint(k2)-surf->GetRectPoint(k);
			v4 = surf->GetRectPoint(k4)-surf->GetRectPoint(k);
			n1 = v1.Xproduct(v2);
			n4 = v4.Xproduct(v1);
			n  = (n1+n4)/2.f;
			n.Unit();
			surf->m_aNormal[k] = n;
		}
		return TRUE;
	}
	
	if (surf->IsMetaRectMesh()) {
		if (surf->m_aNormal==NULL) 
			surf->m_aNormal=new PT_3D[surf->m_nNumVert];

		PT_3D v1,v2,v3,v4;
		PT_3D n1,n2,n3,n4,n;
		int k,k1,k2,k3,k4;

		//middle 
		for (int j=0+1;j<surf->m_nNumVertY-1;j++)
			for (int i=0+1;i<surf->m_nNumVertX-1;i++) {
				k	= j*surf->m_nNumVertX+i;
				k1	= k-1;
				k2  = k-surf->m_nNumVertX;
				k3  = k+1;
				k4  = k+surf->m_nNumVertX;

				PT_3D pt = surf->GetMetaRectPoint(k);
				v1 = surf->GetMetaRectPoint(k1)-pt;
				v2 = surf->GetMetaRectPoint(k2)-pt;
				v3 = surf->GetMetaRectPoint(k3)-pt;
				v4 = surf->GetMetaRectPoint(k4)-pt;
				n1 = v1.Xproduct(v2);
				n2 = v2.Xproduct(v3);
				n3 = v3.Xproduct(v4);
				n4 = v4.Xproduct(v1);
				n  = (n1+n2+n3+n4)/4.f;
				n.Unit();
				surf->m_aNormal[k] = n;
			}

		//corner 
		j		=0;
		int i	=0;
		{
			k	= j*surf->m_nNumVertX+i;
			k3  = k+1;
			k4  = k+surf->m_nNumVertX;
			v3 = surf->GetMetaRectPoint(k3)-surf->GetMetaRectPoint(k);
			v4 = surf->GetMetaRectPoint(k4)-surf->GetMetaRectPoint(k);
			n3 = v3.Xproduct(v4);
			n3.Unit();
			surf->m_aNormal[k] = n3;
		}

		j	=0;
		i	=surf->m_nNumVertX-1;
		{
			k	= j*surf->m_nNumVertX+i;
			k1	= k-1;
			k4  = k+surf->m_nNumVertX;

			v1 = surf->GetMetaRectPoint(k1)-surf->GetMetaRectPoint(k);
			v4 = surf->GetMetaRectPoint(k4)-surf->GetMetaRectPoint(k);
			n4 = v4.Xproduct(v1);
			n4.Unit();
			surf->m_aNormal[k] = n4;
		}

		j	=surf->m_nNumVertY-1;
		i	=surf->m_nNumVertX-1;
		{
			k	= j*surf->m_nNumVertX+i;
			k1	= k-1;
			k2  = k-surf->m_nNumVertX;

			v1 = surf->GetMetaRectPoint(k1)-surf->GetMetaRectPoint(k);
			v2 = surf->GetMetaRectPoint(k2)-surf->GetMetaRectPoint(k);
			n1 = v1.Xproduct(v2);
			n1.Unit();
			surf->m_aNormal[k] = n1;
		}

		j	=surf->m_nNumVertY-1;
		i	=0;
		{
			k	= j*surf->m_nNumVertX+i;
			k2  = k-surf->m_nNumVertX;
			k3  = k+1;

			v2 = surf->GetMetaRectPoint(k2)-surf->GetMetaRectPoint(k);
			v3 = surf->GetMetaRectPoint(k3)-surf->GetMetaRectPoint(k);
			n2 = v2.Xproduct(v3);
			n2.Unit();
			surf->m_aNormal[k] = n2;
		}

		//edges
		j	=0;
		for (i=0+1;i<surf->m_nNumVertX-1;i++) {
			k	= j*surf->m_nNumVertX+i;
			k1	= k-1;
			k3  = k+1;
			k4  = k+surf->m_nNumVertX;

			v1 = surf->GetMetaRectPoint(k1)-surf->GetMetaRectPoint(k);
			v3 = surf->GetMetaRectPoint(k3)-surf->GetMetaRectPoint(k);
			v4 = surf->GetMetaRectPoint(k4)-surf->GetMetaRectPoint(k);
			n3 = v3.Xproduct(v4);
			n4 = v4.Xproduct(v1);
			n  = (n3+n4)/2.f;
			n.Unit();
			surf->m_aNormal[k] = n;
		}

		j = surf->m_nNumVertY-1;
		for (i=0+1;i<surf->m_nNumVertX-1;i++) {
			k	= j*surf->m_nNumVertX+i;
			k1	= k-1;
			k2  = k-surf->m_nNumVertX;
			k3  = k+1;

			v1 = surf->GetMetaRectPoint(k1)-surf->GetMetaRectPoint(k);
			v2 = surf->GetMetaRectPoint(k2)-surf->GetMetaRectPoint(k);
			v3 = surf->GetMetaRectPoint(k3)-surf->GetMetaRectPoint(k);
			n1 = v1.Xproduct(v2);
			n2 = v2.Xproduct(v3);
			n  = (n1+n2)/2.f;
			n.Unit();
			surf->m_aNormal[k] = n;
		}


		i = 0;
		for (j=0+1;j<surf->m_nNumVertY-1;j++) {
			k	= j*surf->m_nNumVertX+i;
			k2  = k-surf->m_nNumVertX;
			k3  = k+1;
			k4  = k+surf->m_nNumVertX;

			v2 = surf->GetMetaRectPoint(k2)-surf->GetMetaRectPoint(k);
			v3 = surf->GetMetaRectPoint(k3)-surf->GetMetaRectPoint(k);
			v4 = surf->GetMetaRectPoint(k4)-surf->GetMetaRectPoint(k);
			n2 = v2.Xproduct(v3);
			n3 = v3.Xproduct(v4);
			n  = (n2+n3)/4.f;
			n.Unit();
			surf->m_aNormal[k] = n;
		}

		i = surf->m_nNumVertX-1;
		for (j=0+1;j<surf->m_nNumVertY-1;j++) {
			k	= j*surf->m_nNumVertX+i;
			k1	= k-1;
			k2  = k-surf->m_nNumVertX;
			k4  = k+surf->m_nNumVertX;

			v1 = surf->GetMetaRectPoint(k1)-surf->GetMetaRectPoint(k);
			v2 = surf->GetMetaRectPoint(k2)-surf->GetMetaRectPoint(k);
			v4 = surf->GetMetaRectPoint(k4)-surf->GetMetaRectPoint(k);
			n1 = v1.Xproduct(v2);
			n4 = v4.Xproduct(v1);
			n  = (n1+n4)/2.f;
			n.Unit();
			surf->m_aNormal[k] = n;
		}
		return TRUE;
	}

	if (surf->IsTopoRectMesh()) {
		if (surf->m_aNormal==NULL) 
			surf->m_aNormal=new PT_3D[surf->m_nNumVert];

		PT_3D v1,v2,v3,v4;
		PT_3D n1,n2,n3,n4,n;
		int k,k1,k2,k3,k4;

		//middle 
		for (int j=0+1;j<surf->m_nNumVertY-1;j++)
			for (int i=0+1;i<surf->m_nNumVertX-1;i++) {
				k	= j*surf->m_nNumVertX+i;
				k1	= k-1;
				k2  = k-surf->m_nNumVertX;
				k3  = k+1;
				k4  = k+surf->m_nNumVertX;

				v1 = surf->m_aVertex[k1]-surf->m_aVertex[k];
				v2 = surf->m_aVertex[k2]-surf->m_aVertex[k];
				v3 = surf->m_aVertex[k3]-surf->m_aVertex[k];
				v4 = surf->m_aVertex[k4]-surf->m_aVertex[k];
				n1 = v1.Xproduct(v2);
				n2 = v2.Xproduct(v3);
				n3 = v3.Xproduct(v4);
				n4 = v4.Xproduct(v1);
				n  = (n1+n2+n3+n4)/4.f;
				n.Unit();
				surf->m_aNormal[k] = n;
			}

		//corner 
		j		=0;
		int i	=0;
		{
			k	= j*surf->m_nNumVertX+i;
			k3  = k+1;
			k4  = k+surf->m_nNumVertX;
			v3 = surf->m_aVertex[k3]-surf->m_aVertex[k];
			v4 = surf->m_aVertex[k4]-surf->m_aVertex[k];
			n3 = v3.Xproduct(v4);
			n3.Unit();
			surf->m_aNormal[k] = n3;
		}

		j	=0;
		i	=surf->m_nNumVertX-1;
		{
			k	= j*surf->m_nNumVertX+i;
			k1	= k-1;
			k4  = k+surf->m_nNumVertX;

			v1 = surf->m_aVertex[k1]-surf->m_aVertex[k];
			v4 = surf->m_aVertex[k4]-surf->m_aVertex[k];
			n4 = v4.Xproduct(v1);
			n4.Unit();
			surf->m_aNormal[k] = n4;
		}

		j	=surf->m_nNumVertY-1;
		i	=surf->m_nNumVertX-1;
		{
			k	= j*surf->m_nNumVertX+i;
			k1	= k-1;
			k2  = k-surf->m_nNumVertX;

			v1 = surf->m_aVertex[k1]-surf->m_aVertex[k];
			v2 = surf->m_aVertex[k2]-surf->m_aVertex[k];
			n1 = v1.Xproduct(v2);
			n1.Unit();
			surf->m_aNormal[k] = n1;
		}

		j	=surf->m_nNumVertY-1;
		i	=0;
		{
			k	= j*surf->m_nNumVertX+i;
			k2  = k-surf->m_nNumVertX;
			k3  = k+1;

			v2 = surf->m_aVertex[k2]-surf->m_aVertex[k];
			v3 = surf->m_aVertex[k3]-surf->m_aVertex[k];
			n2 = v2.Xproduct(v3);
			n2.Unit();
			surf->m_aNormal[k] = n2;
		}

		//edges
		j	=0;
		for (i=0+1;i<surf->m_nNumVertX-1;i++) {
			k	= j*surf->m_nNumVertX+i;
			k1	= k-1;
			k3  = k+1;
			k4  = k+surf->m_nNumVertX;

			v1 = surf->m_aVertex[k1]-surf->m_aVertex[k];
			v3 = surf->m_aVertex[k3]-surf->m_aVertex[k];
			v4 = surf->m_aVertex[k4]-surf->m_aVertex[k];
			n3 = v3.Xproduct(v4);
			n4 = v4.Xproduct(v1);
			n  = (n3+n4)/2.f;
			n.Unit();
			surf->m_aNormal[k] = n;
		}

		j = surf->m_nNumVertY-1;
		for (i=0+1;i<surf->m_nNumVertX-1;i++) {
			k	= j*surf->m_nNumVertX+i;
			k1	= k-1;
			k2  = k-surf->m_nNumVertX;
			k3  = k+1;

			v1 = surf->m_aVertex[k1]-surf->m_aVertex[k];
			v2 = surf->m_aVertex[k2]-surf->m_aVertex[k];
			v3 = surf->m_aVertex[k3]-surf->m_aVertex[k];
			n1 = v1.Xproduct(v2);
			n2 = v2.Xproduct(v3);
			n  = (n1+n2)/2.f;
			n.Unit();
			surf->m_aNormal[k] = n;
		}


		i = 0;
		for (j=0+1;j<surf->m_nNumVertY-1;j++) {
			k	= j*surf->m_nNumVertX+i;
			k2  = k-surf->m_nNumVertX;
			k3  = k+1;
			k4  = k+surf->m_nNumVertX;

			v2 = surf->m_aVertex[k2]-surf->m_aVertex[k];
			v3 = surf->m_aVertex[k3]-surf->m_aVertex[k];
			v4 = surf->m_aVertex[k4]-surf->m_aVertex[k];
			n2 = v2.Xproduct(v3);
			n3 = v3.Xproduct(v4);
			n  = (n2+n3)/4.f;
			n.Unit();
			surf->m_aNormal[k] = n;
		}

		i = surf->m_nNumVertX-1;
		for (j=0+1;j<surf->m_nNumVertY-1;j++) {
			k	= j*surf->m_nNumVertX+i;
			k1	= k-1;
			k2  = k-surf->m_nNumVertX;
			k4  = k+surf->m_nNumVertX;

			v1 = surf->m_aVertex[k1]-surf->m_aVertex[k];
			v2 = surf->m_aVertex[k2]-surf->m_aVertex[k];
			v4 = surf->m_aVertex[k4]-surf->m_aVertex[k];
			n1 = v1.Xproduct(v2);
			n4 = v4.Xproduct(v1);
			n  = (n1+n4)/2.f;
			n.Unit();
			surf->m_aNormal[k] = n;
		}
		return TRUE;
	}


	//periodic along x 
	if (surf->IsPeriodicAlongXMesh()) {

		if (surf->m_aNormal!=NULL) 
			delete[] surf->m_aNormal;
		surf->m_aNormal=new PT_3D[surf->m_nNumVert];

		PT_3D v1,v2,v3,v4;
		PT_3D n1,n2,n3,n4,n;
		int k,k1,k2,k3,k4;
		int numVetX = surf->m_nNumVertX;
		int numVetY = surf->m_nNumVertY;

		//middle 
		for (int j=0+1;j<numVetY-1;j++)
			for (int i=0;i<numVetX;i++) {
				if (i==0) {
					k	= j*numVetX+i;
					k1	= k+numVetX-1;
					k2	= k-numVetX;
					k3	= k+1;
					k4	= k+numVetX;
				}
				else if (i==(numVetX-1)) {
					k	= j*numVetX+i;
					k1	= k-1;
					k2	= k-numVetX;
					k3	= k-numVetX+1;
					k4	= k+numVetX;
				}
				else {
					k	= j*numVetX+i;
					k1	= k-1;
					k2	= k-numVetX;
					k3	= k+1;
					k4	= k+numVetX;
				}

				v1 = surf->m_aVertex[k1]-surf->m_aVertex[k];
				v2 = surf->m_aVertex[k2]-surf->m_aVertex[k];
				v3 = surf->m_aVertex[k3]-surf->m_aVertex[k];
				v4 = surf->m_aVertex[k4]-surf->m_aVertex[k];
				n1 = v1.Xproduct(v2);
				n2 = v2.Xproduct(v3);
				n3 = v3.Xproduct(v4);
				n4 = v4.Xproduct(v1);
				n  = (n1+n2+n3+n4)/4.f;
				n.Unit();
				surf->m_aNormal[k] = n;
			}

		//lower corner 
		j		=0;
		int i	=0;
		{
			k	= 0;
			k1	= numVetX-1;
			k3	= 1;
			k4	= numVetX;

			v1 = surf->m_aVertex[k1]-surf->m_aVertex[k];
			v3 = surf->m_aVertex[k3]-surf->m_aVertex[k];
			v4 = surf->m_aVertex[k4]-surf->m_aVertex[k];
			n3 = v3.Xproduct(v4);
			n4 = v4.Xproduct(v1);
			n  = (n3+n4)/2.f;
			n.Unit();
			surf->m_aNormal[k] = n;
		}

		//up corner
		j	=numVetY-1;
		i	=0;
		{
			k	= j*numVetX+i;
			k1	= numVetX*numVetY-1;
			k2	= k-numVetX; 
			k3	= k+1;


			v1 = surf->m_aVertex[k1]-surf->m_aVertex[k];
			v2 = surf->m_aVertex[k2]-surf->m_aVertex[k];
			v3 = surf->m_aVertex[k3]-surf->m_aVertex[k];
			n1 = v1.Xproduct(v2);
			n2 = v2.Xproduct(v3);
			n  = (n1+n2)/2.f;
			n.Unit();
			surf->m_aNormal[k] = n;
		}

		//low edge
		j	=0;
		for (i=1;i<numVetX;i++) {
			if (i==(numVetX-1)) {
				k	= j*numVetX+i;
				k1	= k-1;
				k3	= k-numVetX+1;
				k4	= k+numVetX;
			}
			else {
				k	= j*numVetX+i;
				k1	= k-1;
				k3	= k+1;
				k4	= k+numVetX;
			}

			v1 = surf->m_aVertex[k1]-surf->m_aVertex[k];
			v3 = surf->m_aVertex[k3]-surf->m_aVertex[k];
			v4 = surf->m_aVertex[k4]-surf->m_aVertex[k];
			n3 = v3.Xproduct(v4);
			n4 = v4.Xproduct(v1);
			n  = (n3+n4)/2.f;
			n.Unit();
			surf->m_aNormal[k] = n;
		}

		//up edge
		j	=numVetY-1;
		for (i=1;i<numVetX;i++) {
			if (i==(numVetX-1)) {
				k	= j*numVetX+i;
				k1	= k-1;
				k2	= k-numVetX; 
				k3	= k-numVetX+1;
			}
			else {
				k	= j*numVetX+i;
				k1	= k-1;
				k2	= k-numVetX; 
				k3	= k+1;
			}

			v1 = surf->m_aVertex[k1]-surf->m_aVertex[k];
			v2 = surf->m_aVertex[k2]-surf->m_aVertex[k];
			v3 = surf->m_aVertex[k3]-surf->m_aVertex[k];
			n1 = v1.Xproduct(v2);
			n2 = v2.Xproduct(v3);
			n  = (n1+n2)/2.f;
			n.Unit();
			surf->m_aNormal[k] = n;
		}
		return TRUE;
	}


	//now must be common surfnet
	//Jan. 12,08
	//unable to handle surfNet with vertex count exeeding 3000000
	//so to replace the code segment below
	/*
	if (surf->m_aNormal!=NULL) {
		delete[] surf->m_aNormal;
		surf->m_aNormal=NULL;
	}

	CVertexNode*		vertexHead;
	CEdgeNode*			edgeHead;
	CPolyNode*			polyHead;

	SurfNetToVetEdgePoly(surf,vertexHead,edgeHead,polyHead);
	vertexHead->CalcNormList();
	
	surf->m_aNormal=new PT_3D[surf->m_nNumVert];
	CVertexNode* ptr=vertexHead;
	for (int i=0;i<surf->m_nNumVert;i++) {
		surf->m_aNormal[i][0]=ptr->norm.x;
		surf->m_aNormal[i][1]=ptr->norm.y;
		surf->m_aNormal[i][2]=ptr->norm.z;
		ptr=ptr->next;
	}

	if (vertexHead)
		vertexHead->FreeMe();
	if (edgeHead)
		edgeHead->FreeMe();
	if (polyHead)
		polyHead->FreeMe();
	*/

	if (surf->m_aNormal!=NULL) {
		delete[] surf->m_aNormal;
		surf->m_aNormal=NULL;
	}
	surf->m_aNormal=new PT_3D[surf->m_nNumVert];
	int* beRefCount;
	beRefCount = new int[surf->m_nNumVert];
	memset(beRefCount,0,sizeof(int)*surf->m_nNumVert);
	memset(surf->m_aNormal,0,sizeof(PT_3D)*surf->m_nNumVert);

	for (int i=0;i<surf->m_nNumQuad;i++) {
		int i0 = surf->m_aQuad[i][0];
		int i1 = surf->m_aQuad[i][1];
		int i2 = surf->m_aQuad[i][2];
		int i3 = surf->m_aQuad[i][3];

		PT_3D norm;

		norm = (surf->m_aVertex[i1]-surf->m_aVertex[i0]).Xproduct(surf->m_aVertex[i3]-surf->m_aVertex[i0]);
		if (norm.Length()>0.0001f) {
			norm.Unit();
			surf->m_aNormal[i0] += norm;
			beRefCount[i0]++;
		}

		norm = (surf->m_aVertex[i2]-surf->m_aVertex[i1]).Xproduct(surf->m_aVertex[i0]-surf->m_aVertex[i1]);
		if (norm.Length()>0.0001f) {
			norm.Unit();
			surf->m_aNormal[i1] += norm;
			beRefCount[i1]++;
		}

		if (i2 == i3) {
			norm = (surf->m_aVertex[i0]-surf->m_aVertex[i3]).Xproduct(surf->m_aVertex[i1]-surf->m_aVertex[i3]);
			if (norm.Length()>0.0001f) {
				norm.Unit();
				surf->m_aNormal[i2] += norm;
				beRefCount[i2]++;
			}
		}
		else {
			norm = (surf->m_aVertex[i3]-surf->m_aVertex[i2]).Xproduct(surf->m_aVertex[i1]-surf->m_aVertex[i2]);
			if (norm.Length()>0.0001f) {
				norm.Unit();
				surf->m_aNormal[i2] += norm;
				beRefCount[i2]++;
			}

			norm = (surf->m_aVertex[i0]-surf->m_aVertex[i3]).Xproduct(surf->m_aVertex[i2]-surf->m_aVertex[i3]);
			if (norm.Length()>0.0001f) {
				norm.Unit();
				surf->m_aNormal[i3] += norm;
				beRefCount[i3]++;
			}
		}
	}

	for (int i=0;i<surf->m_nNumTri;i++) {
		int i0 = surf->m_aTri[i][0];
		int i1 = surf->m_aTri[i][1];
		int i2 = surf->m_aTri[i][2];

		PT_3D norm;

		norm = (surf->m_aVertex[i1]-surf->m_aVertex[i0]).Xproduct(surf->m_aVertex[i2]-surf->m_aVertex[i0]);
		if (norm.Length()>0.0001f) {
			norm.Unit();

			surf->m_aNormal[i0] += norm;
			beRefCount[i0]++;

			surf->m_aNormal[i1] += norm;
			beRefCount[i1]++;

			surf->m_aNormal[i2] += norm;
			beRefCount[i2]++;
		}
	}

	for (int i=0;i<surf->m_nNumVert;i++) {
		if (beRefCount[i]<=0)
			surf->m_aNormal[i].Set(0.f,0.f,1.f);
		else
			surf->m_aNormal[i] /= (float) beRefCount[i];
	}
	delete[]  beRefCount;
	return TRUE;
}

BOOL SurfNetRecreateMeshRect01(	CGeoNetSurf**	surfs,	
	int				numSurf,
	float			errX,		
	float			errY,	
	float           margin,
	CGeoNetSurf*&	destSurf,
	CGeoNetSurf*&	destSurf01,
	BOOL			bSetOutsiderZ,
	float			outsiderZ,
	BOOL            bDoDoubleSide)
{
	destSurf=NULL;
	destSurf01 = NULL;
	if (numSurf<1 )
		return FALSE;

	//sometimes steps cannot be changed, otherwise it may cause problems.
	//if (errX < 0.05f)
	//	errX = 0.05f;
	//if (errY < 0.05f)
	//	errY = 0.05f;

	if (margin < 0.f)	margin = 0.f;
	if (margin > 6.f)   margin = 6.f;


	// calc box of surfs
	float xmin,ymin,zmin,xmax,ymax,zmax;
	int numX,numY;

	//initialize to avoid warning
	xmin = 1.f;
	ymin = 1.f;
	zmin = 1.f;
	xmax = -1.f;
	ymax = -1.f;
	zmax = -1.f;

	int totalVetNum = 0;
	for (int i=0;i<numSurf;i++) {
		totalVetNum += surfs[i]->m_nNumVert;
		BOX3D box;
		surfs[i]->UpdateBox(&box);
		if (i==0) {
			xmin = (float) surfs[i]->m_dBox.min[0];
			ymin = (float) surfs[i]->m_dBox.min[1];
			zmin = (float) surfs[i]->m_dBox.min[2];

			xmax = (float) surfs[i]->m_dBox.max[0];
			ymax = (float) surfs[i]->m_dBox.max[1];
			zmax = (float) surfs[i]->m_dBox.max[2];
		}
		else {
			if (xmin > (float) surfs[i]->m_dBox.min[0])	
				xmin = (float) surfs[i]->m_dBox.min[0];
			if (ymin > (float) surfs[i]->m_dBox.min[1])	
				ymin = (float) surfs[i]->m_dBox.min[1];
			if (zmin > (float) surfs[i]->m_dBox.min[2])	
				zmin = (float) surfs[i]->m_dBox.min[2];

			if (xmax < (float) surfs[i]->m_dBox.max[0])	
				xmax = (float) surfs[i]->m_dBox.max[0];
			if (ymax < (float) surfs[i]->m_dBox.max[1])	
				ymax = (float) surfs[i]->m_dBox.max[1];
			if (zmax < (float) surfs[i]->m_dBox.max[2])	
				zmax = (float) surfs[i]->m_dBox.max[2];
		}
	}
	xmin -= margin;
	ymin -= margin;
	xmax += margin;
	ymax += margin;

	//number of interval in x and y dir
	numX=(int) ((xmax-xmin)/errX+0.5f);
	numY=(int) ((ymax-ymin)/errY+0.5f);
	if (numX<2 || numY<2) 	return FALSE;


	//if of huge vertex count, delete surf norm array to free some space
	if (totalVetNum > 10000000) {
		for (int i=0;i<numSurf;i++) {
			if (surfs[i]->m_aNormal)
				delete[] surfs[i]->m_aNormal;
			surfs[i]->m_aNormal = NULL;
		}
	}

	//adjust steps  
	//commented out Aug. 29, 2007 , it seems steps should not be modified
	//errX = (float)(xmax-xmin)/(float)numX;
	//errY = (float)(ymax-ymin)/(float)numY;
	if (bDoDoubleSide)
		bSetOutsiderZ = FALSE;
	BOOL bKeepHighest;
	{
		bKeepHighest = TRUE;
		//initiate a rect surf
		destSurf = new CGeoNetSurf;
		destSurf->m_nNumVert = (numX+1)*(numY+1);
		destSurf->m_nNumVertX = numX+1;
		destSurf->m_nNumVertY = numY+1;
		destSurf->m_BasePoint.x = xmin;
		destSurf->m_BasePoint.y = ymin;
		destSurf->m_fStepX    = errX;
		destSurf->m_fStepY    = errY;

		destSurf->m_aCoordX   = new float[numX+1];
		destSurf->m_aCoordY   = new float[numY+1];
		destSurf->m_aCoordZ   = new float[(numX+1)*(numY+1)];
		for (int i=0;i<numX+1;i++)
			destSurf->m_aCoordX[i]   = xmin+i*errX;
		for (int i=0;i<numY+1;i++)
			destSurf->m_aCoordY[i]   = ymin+i*errY;
		if (bSetOutsiderZ) {
			for (int i=0;i<destSurf->m_nNumVert;i++)
				destSurf->m_aCoordZ[i]   = outsiderZ;
		}
		else {
			if (bKeepHighest) {
				for (int i=0;i<destSurf->m_nNumVert;i++)
					destSurf->m_aCoordZ[i]   = zmin-1.f;
			}
			else {
				for (int i=0;i<destSurf->m_nNumVert;i++)
					destSurf->m_aCoordZ[i]   = zmax+1.f;
			}
		}

		for (int i=0;i<numSurf;i++) {
			BOOL bCreateIndex = FALSE;

			if (surfs[i]->IsRectMesh()) 
				AddRectSurfToRectSurfHighLow(surfs[i],destSurf,bKeepHighest);
			else if (surfs[i]->IsMetaRectMesh()) {
				int index = 0;
				for (int row=0;row<surfs[i]->m_nNumVertY-1;row++)
					for (int col=0;col<surfs[i]->m_nNumVertX-1;col++) {
						index = row*surfs[i]->m_nNumVertX+col;
						PT_3D tri[3];
						tri[0].x = surfs[i]->m_aCoordX[index];
						tri[0].y = surfs[i]->m_aCoordY[index];
						tri[0].z = surfs[i]->m_aCoordZ[index];

						tri[1].x = surfs[i]->m_aCoordX[index+1];
						tri[1].y = surfs[i]->m_aCoordY[index+1];
						tri[1].z = surfs[i]->m_aCoordZ[index+1];

						tri[2].x = surfs[i]->m_aCoordX[index+1+surfs[i]->m_nNumVertX];
						tri[2].y = surfs[i]->m_aCoordY[index+1+surfs[i]->m_nNumVertX];
						tri[2].z = surfs[i]->m_aCoordZ[index+1+surfs[i]->m_nNumVertX];
						AddTriangleToRectMeshHighLow(destSurf,tri,bKeepHighest);

						tri[1] = tri[2];
						tri[2].x = surfs[i]->m_aCoordX[index+surfs[i]->m_nNumVertX];
						tri[2].y = surfs[i]->m_aCoordY[index+surfs[i]->m_nNumVertX];
						tri[2].z = surfs[i]->m_aCoordZ[index+surfs[i]->m_nNumVertX];
						AddTriangleToRectMeshHighLow(destSurf,tri,bKeepHighest);
					}
			}
			else {
				if (surfs[i]->IsTopoRectMesh()) {
					surfs[i]->CreateRectMeshOrTopRectMeshIndex();
					bCreateIndex = TRUE;
				}
				else if (surfs[i]->IsPeriodicAlongXMesh()) {
					surfs[i]->CreatePeriodicAlongXMeshIndex();
					bCreateIndex = TRUE;
				}

				for (int j=0;j<surfs[i]->m_nNumTri;j++) {
					PT_3D tri[3];
					tri[0] = surfs[i]->m_aVertex[surfs[i]->m_aTri[j][0]];
					tri[1] = surfs[i]->m_aVertex[surfs[i]->m_aTri[j][1]];
					tri[2] = surfs[i]->m_aVertex[surfs[i]->m_aTri[j][2]];
					AddTriangleToRectMeshHighLow(destSurf,tri,bKeepHighest);
				}
				for (int j=0;j<surfs[i]->m_nNumQuad;j++) {
					PT_3D tri[3];
					tri[0] = surfs[i]->m_aVertex[surfs[i]->m_aQuad[j][0]];
					tri[1] = surfs[i]->m_aVertex[surfs[i]->m_aQuad[j][1]];
					tri[2] = surfs[i]->m_aVertex[surfs[i]->m_aQuad[j][2]];
					AddTriangleToRectMeshHighLow(destSurf,tri,bKeepHighest);

					tri[1] = tri[2];
					tri[2] = surfs[i]->m_aVertex[surfs[i]->m_aQuad[j][3]];
					AddTriangleToRectMeshHighLow(destSurf,tri,bKeepHighest);
				}
			}
			if (bCreateIndex)
				surfs[i]->DeleteQuadIndex();
		}

		//set color info for the surf
		if (!bSetOutsiderZ) {
			destSurf->m_aVetColor = new unsigned char[destSurf->m_nNumVert][3];
			if (bKeepHighest) {
				for (int i=0;i<destSurf->m_nNumVert;i++) {
					if (destSurf->m_aCoordZ[i] < zmin) {
						destSurf->m_aCoordZ[i] = zmin;
						destSurf->m_aVetColor[i][0] = 255;
						destSurf->m_aVetColor[i][1] = 255;
						destSurf->m_aVetColor[i][2] = 0;
					}
					else {
						destSurf->m_aVetColor[i][0] = 255;
						destSurf->m_aVetColor[i][1] = 104;
						destSurf->m_aVetColor[i][2] = 32;
					}
				}
			}
			else {
				for (int i=0;i<destSurf->m_nNumVert;i++) {
					if (destSurf->m_aCoordZ[i] > zmax) {
						destSurf->m_aCoordZ[i] = zmax;
						destSurf->m_aVetColor[i][0] = 255;
						destSurf->m_aVetColor[i][1] = 255;
						destSurf->m_aVetColor[i][2] = 0;
					}
					else {
						destSurf->m_aVetColor[i][0] = 192;
						destSurf->m_aVetColor[i][1] = 192;
						destSurf->m_aVetColor[i][2] = 192;
					}
				}
			}
		}
		SurfNetNorm(destSurf);
	}

	if (bDoDoubleSide) {
		bKeepHighest = FALSE;
		//initiate a rect surf
		destSurf01 = new CGeoNetSurf;
		destSurf01->m_nNumVert = (numX+1)*(numY+1);
		destSurf01->m_nNumVertX = numX+1;
		destSurf01->m_nNumVertY = numY+1;
		destSurf01->m_BasePoint.x = xmin;
		destSurf01->m_BasePoint.y = ymin;
		destSurf01->m_fStepX    = errX;
		destSurf01->m_fStepY    = errY;

		destSurf01->m_aCoordX   = new float[numX+1];
		destSurf01->m_aCoordY   = new float[numY+1];
		destSurf01->m_aCoordZ   = new float[(numX+1)*(numY+1)];
		for (int i=0;i<numX+1;i++)
			destSurf01->m_aCoordX[i]   = xmin+i*errX;
		for (int i=0;i<numY+1;i++)
			destSurf01->m_aCoordY[i]   = ymin+i*errY;
		if (bSetOutsiderZ) {
			for (int i=0;i<destSurf01->m_nNumVert;i++)
				destSurf01->m_aCoordZ[i]   = outsiderZ;
		}
		else {
			if (bKeepHighest) {
				for (int i=0;i<destSurf01->m_nNumVert;i++)
					destSurf01->m_aCoordZ[i]   = zmin-1.f;
			}
			else {
				for (int i=0;i<destSurf01->m_nNumVert;i++)
					destSurf01->m_aCoordZ[i]   = zmax+1.f;
			}
		}

		for (int i=0;i<numSurf;i++) {
			BOOL bCreateIndex = FALSE;

			if (surfs[i]->IsRectMesh()) 
				AddRectSurfToRectSurfHighLow(surfs[i],destSurf01,bKeepHighest);
			else if (surfs[i]->IsMetaRectMesh()) {
				int index = 0;
				for (int row=0;row<surfs[i]->m_nNumVertY-1;row++)
					for (int col=0;col<surfs[i]->m_nNumVertX-1;col++) {
						index = row*surfs[i]->m_nNumVertX+col;
						PT_3D tri[3];
						tri[0].x = surfs[i]->m_aCoordX[index];
						tri[0].y = surfs[i]->m_aCoordY[index];
						tri[0].z = surfs[i]->m_aCoordZ[index];

						tri[1].x = surfs[i]->m_aCoordX[index+1];
						tri[1].y = surfs[i]->m_aCoordY[index+1];
						tri[1].z = surfs[i]->m_aCoordZ[index+1];

						tri[2].x = surfs[i]->m_aCoordX[index+1+surfs[i]->m_nNumVertX];
						tri[2].y = surfs[i]->m_aCoordY[index+1+surfs[i]->m_nNumVertX];
						tri[2].z = surfs[i]->m_aCoordZ[index+1+surfs[i]->m_nNumVertX];
						AddTriangleToRectMeshHighLow(destSurf01,tri,bKeepHighest);

						tri[1] = tri[2];
						tri[2].x = surfs[i]->m_aCoordX[index+surfs[i]->m_nNumVertX];
						tri[2].y = surfs[i]->m_aCoordY[index+surfs[i]->m_nNumVertX];
						tri[2].z = surfs[i]->m_aCoordZ[index+surfs[i]->m_nNumVertX];
						AddTriangleToRectMeshHighLow(destSurf01,tri,bKeepHighest);
					}
			}
			else {
				if (surfs[i]->IsTopoRectMesh()) {
					surfs[i]->CreateRectMeshOrTopRectMeshIndex();
					bCreateIndex = TRUE;
				}
				else if (surfs[i]->IsPeriodicAlongXMesh()) {
					surfs[i]->CreatePeriodicAlongXMeshIndex();
					bCreateIndex = TRUE;
				}

				for (int j=0;j<surfs[i]->m_nNumTri;j++) {
					PT_3D tri[3];
					tri[0] = surfs[i]->m_aVertex[surfs[i]->m_aTri[j][0]];
					tri[1] = surfs[i]->m_aVertex[surfs[i]->m_aTri[j][1]];
					tri[2] = surfs[i]->m_aVertex[surfs[i]->m_aTri[j][2]];
					AddTriangleToRectMeshHighLow(destSurf01,tri,bKeepHighest);
				}
				for (int j=0;j<surfs[i]->m_nNumQuad;j++) {
					PT_3D tri[3];
					tri[0] = surfs[i]->m_aVertex[surfs[i]->m_aQuad[j][0]];
					tri[1] = surfs[i]->m_aVertex[surfs[i]->m_aQuad[j][1]];
					tri[2] = surfs[i]->m_aVertex[surfs[i]->m_aQuad[j][2]];
					AddTriangleToRectMeshHighLow(destSurf01,tri,bKeepHighest);

					tri[1] = tri[2];
					tri[2] = surfs[i]->m_aVertex[surfs[i]->m_aQuad[j][3]];
					AddTriangleToRectMeshHighLow(destSurf01,tri,bKeepHighest);
				}
			}
			if (bCreateIndex)
				surfs[i]->DeleteQuadIndex();
		}

		//set color info for the surf
		if (!bSetOutsiderZ) {
			destSurf01->m_aVetColor = new unsigned char[destSurf01->m_nNumVert][3];
			if (bKeepHighest) {
				for (int i=0;i<destSurf01->m_nNumVert;i++) {
					if (destSurf01->m_aCoordZ[i] < zmin) {
						destSurf01->m_aCoordZ[i] = zmin;
						destSurf01->m_aVetColor[i][0] = 255;
						destSurf01->m_aVetColor[i][1] = 255;
						destSurf01->m_aVetColor[i][2] = 0;
					}
					else {
						destSurf01->m_aVetColor[i][0] = 255;
						destSurf01->m_aVetColor[i][1] = 104;
						destSurf01->m_aVetColor[i][2] = 32;
					}
				}
			}
			else {
				for (int i=0;i<destSurf01->m_nNumVert;i++) {
					if (destSurf01->m_aCoordZ[i] > zmax) {
						destSurf01->m_aCoordZ[i] = zmin;
						destSurf01->m_aVetColor[i][0] = 255;
						destSurf01->m_aVetColor[i][1] = 255;
						destSurf01->m_aVetColor[i][2] = 0;
					}
					else {
						destSurf01->m_aVetColor[i][0] = 192;
						destSurf01->m_aVetColor[i][1] = 192;
						destSurf01->m_aVetColor[i][2] = 192;
					}
				}
			}
		}
		SurfNetNorm(destSurf01);
	}
	return TRUE;
}

BOOL RecreateMesh(CGeoNetSurf *&pSurf,float fToolDia,float fRemainder,float flowz ) 
{
	if( pSurf==NULL )
		return FALSE ;	

		if( fabs(pSurf->m_fStepX - pSurf->m_fStepY)<0.0001f ) //如果 XY步长相等，则向外扩边
		{							
			CGeoNetSurf *pTempSurf = (CGeoNetSurf *)pSurf->CopyMyself() ;			
			float minStep = min( pSurf->m_fStepX , pSurf->m_fStepY ) ;
			int num = int((fToolDia+1.5f*fRemainder) / minStep) + 2 ;
			EnlargeRectMesh(pTempSurf,num); 
			if( pSurf )
				delete pSurf ;
				pSurf = pTempSurf ;
		}
		else
		{													//如果 XY步长不相等，则以最小步长重构，并且扩边
			float step = min(pSurf->m_fStepX , pSurf->m_fStepY)  ;
			CGeoNetSurf ** surfs = new CGeoNetSurf*[1] ;
			surfs[0] = pSurf ;
			CGeoNetSurf* pSurfTmp1,*pSurfTmp2;
			SurfNetRecreateMeshRect01(	surfs,
					1,
					step,
					step,
					fToolDia*0.5f + fRemainder + 2*step ,	//margin
					pSurfTmp1,
					pSurfTmp2,
					TRUE,
					flowz - 0.1f,
					FALSE);  
			if( pSurf )
				delete pSurf ;
			pSurf = pSurfTmp1 ;
		}
	return TRUE ;
}

void MakeHeightMask(float radius,float stepX,int k,float**& heightMask)
{
	int num = 2*k+1;
	heightMask = new float*[num];
	for (int i=0;i<num;i++)
		heightMask[i] = new float[num];

	float x,y;
	float radiusSqr = radius*radius;
	for (int i=0;i<num;i++)
		for (int j=0;j<num;j++) {
			y = (i-k)*stepX;
			x = (j-k)*stepX;
			float delta = radiusSqr-x*x-y*y;
			if (delta>=0.f)
				heightMask[i][j] = radius- (float)sqrt(radiusSqr-x*x-y*y);
			else
				heightMask[i][j] = -1.f;
		}
}

float CheckHeight(CGeoNetSurf* pSurf,float radius,int r,int c,float** heightMask,int k)
{
	float delta = 0.f;
	float baseZ = pSurf->m_aCoordZ[r*pSurf->m_nNumVertX+c];

	for (int i=r-k; i<=r+k;i++)
		for (int j=c-k;j<=c+k;j++) {
			if (	i < 0 || i>pSurf->m_nNumVertY-1 ||
				j < 0 || j>pSurf->m_nNumVertX-1)
				continue;

			int i1 = i-r+k;
			int j1 = j-c+k;
			if (heightMask[i1][j1] < 0.f) continue;

			float realDiff = pSurf->m_aCoordZ[i*pSurf->m_nNumVertX+j] - baseZ;
			if (realDiff > heightMask[i1][j1]) {
				float adjustAmount = realDiff -  heightMask[i1][j1];
				if ( adjustAmount > delta)
					delta = adjustAmount;
			}
		}
		return delta+radius;
}


BOOL OffsetRectMesh_Loc(CGeoNetSurf* &pSurf,	float radius)
{
	//get parameters
	//CGeoNetSurf*	pSurf;
	//float			radius;

	// 	pSurf		= m_pSurfOff;
	// 	radius		= m_radiusOff;

	int numRadius = (int) floor(radius/pSurf->m_fStepX)+1;
	float** heightMask;
	MakeHeightMask(radius,pSurf->m_fStepX,numRadius,heightMask);
	int num = 2*numRadius+1;

	//tmp array to save height of the model
	float* tmpHeights;
	tmpHeights = new float[pSurf->m_nNumVert];
	memset(tmpHeights,0,sizeof(float)*pSurf->m_nNumVert);

	int index = -1;

//	PathEdit_SetCurPos( 0, 100 ) ;
// 
// 	char strTmp[128];
// 	switch (glbf_GetLocalLangID()) {
// 		//case LANGID_CHINESE_SIMPLIFIED:
// 		//	lstrcpy(str,"请拾取网格曲面jjj");
// 		//	break;
// 	case LANGID_CHINESE_TRADITIONAL:
// 		lstrcpy(strTmp,"熬簿");
// 		break;
// 	case LANGID_ENGLISH_US:
// 		lstrcpy(strTmp,"Offseting Surf");
// 		break;
// 	default:
// 		lstrcpy(strTmp,"偏移");
// 		break;
// 	}
// 	PathEdit_SetNewStep(strTmp) ;

#if _MSC_VER < 1600
	int count = pSurf->m_nNumVertY*pSurf->m_nNumVertX;
	for (int i=0;i<pSurf->m_nNumVertY;i++)
		for (int j=0;j<pSurf->m_nNumVertX;j++) {
			PathEdit_SetCurPos( i*pSurf->m_nNumVertX+j, count) ;
			if( PathEdit_IsAbort())
				return FALSE;		
			tmpHeights[++index] = CheckHeight(pSurf,radius,i,j,heightMask,numRadius);
		}
#else
	//使用并行计算		
	int nCore = min( omp_get_num_procs(), 20 );		//可用CPU核数	
	int nTotalLoop = pSurf->m_nNumVertY * pSurf->m_nNumVertX ;
	int nLoad = max( 1, nTotalLoop / nCore );	
	if( nCore<=1 )
	{
//		int count = pSurf->m_nNumVertY*pSurf->m_nNumVertX;
		for (int i=0;i<pSurf->m_nNumVertY;i++)
			for (int j=0;j<pSurf->m_nNumVertX;j++) {
// 				PathEdit_SetCurPos( i*pSurf->m_nNumVertX+j, count) ;
// 				if( PathEdit_IsAbort())
// 					return FALSE;		
				tmpHeights[++index] = CheckHeight(pSurf,radius,i,j,heightMask,numRadius);
			}
	}
	else
	{
//		BOOL bBreak = FALSE ;
		//设置线程个数
#pragma omp parallel num_threads( nCore)
		{	//执行并行计算
#pragma omp for 
			for( int nIndex = 0; nIndex < nCore; nIndex++ )
			{					
				int nLoadIndex0 = nIndex * nLoad ;//每个核上的点序号范围[]				
				if( nLoadIndex0>=nTotalLoop )
					continue ;

				int	nLoadIndex1 = ( nIndex + 1 ) * nLoad ;
				if( nIndex==(nCore-1) )
					nLoadIndex1 = nTotalLoop ;

				for (int nIndex0=nLoadIndex0;nIndex0<nLoadIndex1;nIndex0++) 							
				{	
// 					if( nIndex==0 )
// 						PathEdit_SetCurPos( nIndex0, nLoad) ;
// 					if( PathEdit_IsAbort())
// 						bBreak = TRUE ;		
// 					if( bBreak )
// 						break ;
					int i = nIndex0 / pSurf->m_nNumVertX ;
					int j = nIndex0 - i * pSurf->m_nNumVertX ;
					tmpHeights[nIndex0] = CheckHeight(pSurf,radius,i,j,heightMask,numRadius);
				}
			}
		}

	}
#endif

	for (int i=0;i<pSurf->m_nNumVert;i++)
		pSurf->m_aCoordZ[i] += tmpHeights[i];

	for (int i=0;i<num;i++)
		delete[] heightMask[i];
	delete[] heightMask;

	delete[] tmpHeights;
	return TRUE;
}


// static UINT	 ProgressCallbackOffset( LPVOID lpParam)
// {
// 	int PathEdit_RetFlag =	OffsetRectMesh_Loc() ;
// 	if(PathEdit_RetFlag==1)
// 		return 0 ;
// 	else
// 		return 1 ;
// }


BOOL OffsetRectMesh(CGeoNetSurf* &pSurf,	float radius)
{
	if (	pSurf == NULL ||
		!pSurf->IsRectMesh() ||
		fabs(pSurf->m_fStepX-pSurf->m_fStepY)>0.0001 ||
		radius<=0.f)
		return FALSE;

// 	m_pSurfOff		= pSurf;
// 	m_radiusOff		= radius;
// 
// 	CPogressThread	Thread ;
// 	Thread.EnterEx(	ProgressCallbackOffset , this , PathEdit_GenProgress ) ;
	OffsetRectMesh_Loc( pSurf,radius) ;

// 	if( PathEdit_IsAbort())
// 		return FALSE;		
// 	else
		return TRUE;
}

void RectSurfMirror(CGeoNetSurf *&pSurf)
{
	if (	pSurf == NULL ||	!pSurf->IsRectMesh() )
		return ;

	for (int i=0;i<pSurf->m_nNumVertY;i++)
		for (int j=0;j<pSurf->m_nNumVertX;j++)
			pSurf->m_aCoordZ[ i*pSurf->m_nNumVertX+j] *= -1 ;
}
